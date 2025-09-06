// SPDX-License-Identifier: GPL-2.0
//
// ENV-COMBO Assignment Driver
//
// Channels: Temperature + Humidity
// Attributes per channel: RAW (ro), SCALE (ro), OFFSET (rw), PROCESSED (ro)
//
// Sysfs files:
//   in_temp_raw / in_temp_scale / in_temp_offset / in_temp_input
//   in_humidityrelative_raw / in_humidityrelative_scale
//   in_humidityrelative_offset / in_humidityrelative_input
//
// Triggered buffer:
//   - devm_iio_triggered_buffer_setup()
//   - Periodic sampling implemented with an **internal software trigger**
//     backed by a high-resolution timer (hrtimer).
//   - The hrtimer callback (`envcombo_timer_cb`) fires at a fixed interval
//     (default = 1 second), which calls iio_trigger_poll().
//   - That poll executes the registered pollfunc:
//       1. store timestamp
//       2. call our trigger handler to read temp/hum
//       3. push sample + timestamp into the IIO buffer.
//   - When the buffer is enabled, hrtimer is started via
//     envcombo_trigger_set_state(); when disabled, the hrtimer is canceled.
//   - This makes the device self-triggering without requiring an external
//     GPIO/interrupt trigger.
//   - Triggering is set to 100ms in the Driver.

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/err.h>

#include <linux/iio/iio.h>
#include <linux/iio/driver.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include <linux/hrtimer.h>
#include <linux/ktime.h>

/* Offset clamp limits (datasheet units) */
#define ENV_S8_MIN   (-128)
#define ENV_S8_MAX   127
#define ENV_S16_MIN  (-32768)
#define ENV_S16_MAX  32767

/* Registers */
#define ENV_WHO_AM_I            0x00
#define ENV_WHO_AM_I_VAL        0xEB

#define ENV_TEMP_MSB            0x01
#define ENV_TEMP_LSB            0x02
#define ENV_HUM_OUT             0x03

#define ENV_CALIB_TEMP_OFFMSB   0x0D
#define ENV_CALIB_TEMP_OFFLSB   0x0E
#define ENV_CALIB_HUM_OFF       0x0F

// Driver state
// - Driver state holds everything we need for this sensor instance.
// - The lock ensures safe I2C access from multiple contexts.
// - scan_buf is what gets pushed into the IIO buffer (temp + hum).
// - trig + timer implement the internal self-trigger.

struct envcombo_data {
	struct i2c_client *client;
	struct mutex lock;
	__le16 scan_buf[2];

	/* Internal periodic trigger */
	struct iio_trigger *trig;
	struct hrtimer      timer;
	ktime_t             period;
};

// I2C helpers

// Read two registers (MSB + LSB) and combine into signed 16-bit value
// Used for temperature and temperature offset.
static int envcombo_read_s16(struct envcombo_data *st, u8 reg_msb, u8 reg_lsb)
{
	int msb, lsb;

	mutex_lock(&st->lock);
	msb = i2c_smbus_read_byte_data(st->client, reg_msb);
	lsb = i2c_smbus_read_byte_data(st->client, reg_lsb);
	mutex_unlock(&st->lock);

	if (msb < 0 || lsb < 0) {
		dev_err(&st->client->dev,
			"read s16 0x%02x/0x%02x failed (msb=%d, lsb=%d)\n",
			reg_msb, reg_lsb, msb, lsb);
		return (msb < 0) ? msb : lsb;
	}
	return (s16)((msb << 8) | lsb);
}

// Write a signed 16-bit value into two registers (MSB + LSB).
// Called when user updates the temperature offset via sysfs.
static int envcombo_write_s16(struct envcombo_data *st, u8 reg_msb, u8 reg_lsb, s16 val)
{
	int ret;

	mutex_lock(&st->lock);

	ret = i2c_smbus_write_byte_data(st->client, reg_msb, (val >> 8) & 0xFF);
	if (ret < 0) {
		dev_err(&st->client->dev,
			"write s16 MSB 0x%02x failed (val=%d ret=%d)\n",
			reg_msb, val, ret);
		goto out;
	}

	ret = i2c_smbus_write_byte_data(st->client, reg_lsb, val & 0xFF);
	if (ret < 0)
		dev_err(&st->client->dev,
			"write s16 LSB 0x%02x failed (val=%d ret=%d)\n",
			reg_lsb, val, ret);
out:
	mutex_unlock(&st->lock);
	return ret;
}

static int envcombo_read_s8(struct envcombo_data *st, u8 reg)
{
	int v;

	mutex_lock(&st->lock);
	v = i2c_smbus_read_byte_data(st->client, reg);
	mutex_unlock(&st->lock);

	if (v < 0) {
		dev_err(&st->client->dev, "read s8 0x%02x failed (%d)\n", reg, v);
		return v;
	}
	return (s8)v;
}

static int envcombo_write_s8(struct envcombo_data *st, u8 reg, s8 val)
{
	int ret;

	mutex_lock(&st->lock);
	ret = i2c_smbus_write_byte_data(st->client, reg, val);
	mutex_unlock(&st->lock);

	if (ret < 0)
		dev_err(&st->client->dev,
			"write s8 0x%02x failed (val=%d ret=%d)\n", reg, val, ret);
	return ret;
}

// Simple wrappers around the raw I2C helpers.
static inline int envcombo_read_temp(struct envcombo_data *st)
{
	return envcombo_read_s16(st, ENV_TEMP_MSB, ENV_TEMP_LSB);
}

static inline int envcombo_read_hum(struct envcombo_data *st)
{
	int v = envcombo_read_s8(st, ENV_HUM_OUT);
	return (v < 0) ? v : (v & 0xFF);
}

static inline int envcombo_read_temp_offset(struct envcombo_data *st)
{
	return envcombo_read_s16(st, ENV_CALIB_TEMP_OFFMSB, ENV_CALIB_TEMP_OFFLSB);
}

static inline int envcombo_write_temp_offset(struct envcombo_data *st, int val)
{
	if (val > ENV_S16_MAX) val = ENV_S16_MAX;
	else if (val < ENV_S16_MIN) val = ENV_S16_MIN;
	return envcombo_write_s16(st, ENV_CALIB_TEMP_OFFMSB, ENV_CALIB_TEMP_OFFLSB, (s16)val);
}

static inline int envcombo_read_hum_offset(struct envcombo_data *st)
{
	return envcombo_read_s8(st, ENV_CALIB_HUM_OFF);
}

static inline int envcombo_write_hum_offset(struct envcombo_data *st, int val)
{
	if (val > ENV_S8_MAX) val = ENV_S8_MAX;
	else if (val < ENV_S8_MIN) val = ENV_S8_MIN;
	return envcombo_write_s8(st, ENV_CALIB_HUM_OFF, (s8)val);
}

// IIO read/write
// read_raw callback:
// - read_raw is called whenever user-space reads sysfs attributes
// - like in_temp_raw, in_temp_offset, in_temp_input, etc.
// - Depending on 'mask', we return raw data, scale, offset, or processed value.
static int envcombo_read_raw(struct iio_dev *indio_dev,
			     const struct iio_chan_spec *chan,
			     int *val, int *val2, long mask)
{
	struct envcombo_data *st = iio_priv(indio_dev);
	int ret = iio_device_claim_direct_mode(indio_dev);
	int raw;

	if (ret)
		return ret;

	switch (chan->type) {
	case IIO_TEMP:
		switch (mask) {
		case IIO_CHAN_INFO_RAW:
			ret = envcombo_read_temp(st);
			if (ret >= 0) { *val = ret; ret = IIO_VAL_INT; }
			break;
		case IIO_CHAN_INFO_SCALE:
			*val = 0; *val2 = 10000; /* 0.01 °C */
			ret = IIO_VAL_INT_PLUS_MICRO;
			break;
		case IIO_CHAN_INFO_OFFSET:
			ret = envcombo_read_temp_offset(st);
			if (ret >= 0) { *val = ret; ret = IIO_VAL_INT; }
			break;
        case IIO_CHAN_INFO_PROCESSED:
            raw = envcombo_read_temp(st);
            if (raw < 0) { ret = raw; break; }
            /* old: sim updates raw return values according to offset
                   off = envcombo_read_temp_offset(st);
            *      if (off < 0) off = 0;
            *      *val = (raw + off) * 10;
            */
            *val = raw * 10;   /* raw is already offset-adjusted by hw */
            ret = IIO_VAL_INT;
            break;
		default:
			ret = -EINVAL;
		}
		break;

	case IIO_HUMIDITYRELATIVE:
		switch (mask) {
		case IIO_CHAN_INFO_RAW:
			ret = envcombo_read_hum(st);
			if (ret >= 0) { *val = ret; ret = IIO_VAL_INT; }
			break;
		case IIO_CHAN_INFO_SCALE:
			*val = 0; *val2 = 500000; /* 0.5 %RH */
			ret = IIO_VAL_INT_PLUS_MICRO;
			break;
		case IIO_CHAN_INFO_OFFSET:
			ret = envcombo_read_hum_offset(st);
			if (ret >= 0) { *val = ret; ret = IIO_VAL_INT; }
			break;
		case IIO_CHAN_INFO_PROCESSED:
            raw = envcombo_read_hum(st);
            if (raw < 0) { ret = raw; break; }
            *val = raw * 500;  // raw already includes hw offset
            ret = IIO_VAL_INT;
            break;
		default:
			ret = -EINVAL;
		}
		break;

	default:
		ret = -EINVAL;
	}

	iio_device_release_direct_mode(indio_dev);
	return ret;
}

// write_Raw callback:
//  - write_raw is called whenever user-space writes to sysfs
//  - (e.g., to change offset calibration values).
static int envcombo_write_raw(struct iio_dev *indio_dev,
			      const struct iio_chan_spec *chan,
			      int val, int val2, long mask)
{
	struct envcombo_data *st = iio_priv(indio_dev);
	int ret = iio_device_claim_direct_mode(indio_dev);

	if (ret)
		return ret;

	switch (chan->type) {
	case IIO_TEMP:
		ret = (mask == IIO_CHAN_INFO_OFFSET) ?
			envcombo_write_temp_offset(st, val) : -EINVAL;
		break;
	case IIO_HUMIDITYRELATIVE:
		ret = (mask == IIO_CHAN_INFO_OFFSET) ?
			envcombo_write_hum_offset(st, val) : -EINVAL;
		break;
	default:
		ret = -EINVAL;
	}

	iio_device_release_direct_mode(indio_dev);
	return ret;
}

static const struct iio_info envcombo_iio_info = {
	.read_raw  = envcombo_read_raw,
	.write_raw = envcombo_write_raw,
};

//Triggered buffer handler

static irqreturn_t envcombo_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf   = p;
	struct iio_dev *indio_dev  = pf->indio_dev;
	struct envcombo_data *st   = iio_priv(indio_dev);
	s16 t;
	int h;

	t = envcombo_read_temp(st);
	h = envcombo_read_hum(st);
	if (t < 0 || h < 0)
		goto done;

	st->scan_buf[0] = cpu_to_le16(t);
	st->scan_buf[1] = cpu_to_le16((u16)(h & 0xFF));

	iio_push_to_buffers_with_timestamp(indio_dev,
                                       st->scan_buf,
                                       ktime_get_boottime_ns());

done:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

// Channels
enum { ENV_CH_TEMP = 0, ENV_CH_HUM = 1 };

static const struct iio_chan_spec envcombo_channels[] = {
	{
		.type = IIO_TEMP,
		.indexed = 0,
		.channel = 0,
		.scan_index = ENV_CH_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_OFFSET) |
				      BIT(IIO_CHAN_INFO_PROCESSED),
		.scan_type = {
			.sign = 's', .realbits = 16, .storagebits = 16, .endianness = IIO_LE,
		},
	},
	{
		.type = IIO_HUMIDITYRELATIVE,
		.indexed = 0,
		.channel = 0,
		.scan_index = ENV_CH_HUM,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_OFFSET) |
				      BIT(IIO_CHAN_INFO_PROCESSED),
		.scan_type = {
			.sign = 'u',
			.realbits = 8,
			.storagebits = 16,  /* store u8 in 16 bits for alignment */
			.endianness = IIO_LE,
		},
	},
};

// Internal trigger ops
// hrtimer callback: runs periodically at configured 'period'.
// Each time it fires, we poll the trigger to collect a new sample.
// mechanism which makes the device "self-sampling".
static enum hrtimer_restart envcombo_timer_cb(struct hrtimer *t)
{
	struct envcombo_data *st = container_of(t, struct envcombo_data, timer);

	iio_trigger_poll(st->trig);

	hrtimer_forward_now(&st->timer, st->period);
	
    return HRTIMER_RESTART;
}

// Called by the IIO core when buffer is enabled/disabled.
// If enabled, we start the hrtimer so sampling begins.
// If disabled, we cancel the hrtimer so device goes idle.
static int envcombo_trigger_set_state(struct iio_trigger *trig, bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct envcombo_data *st  = iio_priv(indio_dev);

	if (state) {
		/* Start periodic sampling when buffer is enabled */
		hrtimer_start(&st->timer, st->period, HRTIMER_MODE_REL);
	} else {
		/* Stop when buffer is disabled */
		hrtimer_cancel(&st->timer);
	}
	return 0;
}

static const struct iio_trigger_ops envcombo_trigger_ops = {
	.set_trigger_state = envcombo_trigger_set_state,
	.validate_device   = iio_trigger_validate_own_device,
};

// Probe / Remove

// Probe: runs when the I2C device is created.
// verify WHO_AM_I, allocate the IIO device,
// setup channels, allocate + register the trigger, and
// finally register the IIO device with the core.
static int envcombo_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct envcombo_data *st;
	int whoami;
	int ret;

	dev_info(&client->dev, "=== envcombo_probe starting ===\n");

	whoami = i2c_smbus_read_byte_data(client, ENV_WHO_AM_I);
	dev_info(&client->dev, "WHO_AM_I=0x%02x\n", whoami);
	if (whoami < 0) {
		dev_err(&client->dev, "WHO_AM_I read failed: %d\n", whoami);
		return whoami;
	}
	if (whoami != ENV_WHO_AM_I_VAL) {
		dev_err(&client->dev, "WHO_AM_I mismatch: 0x%02x\n", whoami);
		return -ENODEV;
	}

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->client = client;
	mutex_init(&st->lock);

	indio_dev->dev.parent   = &client->dev;
	indio_dev->info         = &envcombo_iio_info;
	indio_dev->channels     = envcombo_channels;
	indio_dev->num_channels = ARRAY_SIZE(envcombo_channels);
	indio_dev->name         = "envcombo";
	indio_dev->modes        = INDIO_DIRECT_MODE | INDIO_BUFFER_TRIGGERED;

	// Make indio_dev retrievable in .remove()
	i2c_set_clientdata(client, indio_dev);

	// Allocate + register internal trigger
	st->trig = devm_iio_trigger_alloc(&client->dev, "%s-trigger",
					  indio_dev->name);
	if (!st->trig)
		return -ENOMEM;

	iio_trigger_set_drvdata(st->trig, indio_dev);
	st->trig->ops = &envcombo_trigger_ops;

	ret = devm_iio_trigger_register(&client->dev, st->trig);
	if (ret) {
		dev_err(&client->dev, "trigger register failed: %d\n", ret);
		return ret;
	}

	// Set default period and init timer (e.g., 100 ms = 10 Hz)
	st->period = ms_to_ktime(1500);  // 1 second period
	hrtimer_init(&st->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	st->timer.function = envcombo_timer_cb;

	// Triggered buffer
	ret = devm_iio_triggered_buffer_setup(&client->dev, indio_dev,
					      iio_pollfunc_store_time,
					      envcombo_trigger_handler,
					      NULL);
	dev_info(&client->dev, "Triggered buffer setup ret=%d\n", ret);
	if (ret) {
		dev_err(&client->dev, "triggered buffer setup failed: %d\n", ret);
		return ret;
	}

	ret = devm_iio_device_register(&client->dev, indio_dev);
	dev_info(&client->dev, "iio_device_register ret=%d\n", ret);
	dev_info(&client->dev, "=== envcombo_probe done ===\n");
	return ret;
}

// Remove: runs when the driver is unloaded or device is deleted.
// only need to stop the timer. devm_* manages resources ->
// will clean up everything else automatically.
static int envcombo_remove(struct i2c_client *client)
{
    struct iio_dev *indio_dev = i2c_get_clientdata(client);
    struct envcombo_data *st  = iio_priv(indio_dev);

    dev_info(&client->dev, "envcombo removed\n");

    hrtimer_cancel(&st->timer);

    return 0;
}


static const struct i2c_device_id envcombo_id[] = {
	{ "envcombo", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, envcombo_id);

static struct i2c_driver envcombo_driver = {
	.driver = {
		.name = "envcombo",
	},
	.probe    = envcombo_probe,
	.remove   = envcombo_remove,
	.id_table = envcombo_id,
};
module_i2c_driver(envcombo_driver);

MODULE_AUTHOR("Sami Khatib <sami.khalid.khatib@gmail.com>");
MODULE_DESCRIPTION("ENV-COMBO Temp/Humidity IIO driver with periodic internal trigger");
MODULE_LICENSE("GPL");
