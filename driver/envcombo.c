// SPDX-License-Identifier: GPL-2.0
//
// ENV-COMBO Assignment Driver (I2C + IIO)
//
// Channels: Temperature + Humidity
// Attributes: RAW, SCALE, OFFSET (rw), PROCESSED
//
// Sysfs entries (example):
//   in_temp_raw / in_temp_scale / in_temp_offset / in_temp_input
//   in_humidityrelative_raw / in_humidityrelative_scale / in_humidityrelative_offset / in_humidityrelative_input
//
// Datasheet registers:
//   0x00 WHO_AM_I (0xEB expected)
//   0x01–0x02 TEMP_OUT (s16, 0.01 °C/LSB)
//   0x03 HUM_OUT (u8, 0.5 %RH/LSB)
//   0x0D–0x0E CALIB_TEMP_OFF (s16, 0.01 °C/LSB)
//   0x0F CALIB_HUM_OFF (s8, 0.5 %RH/LSB)

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/err.h>

#include <linux/iio/iio.h>
#include <linux/iio/driver.h>

/* --- Device registers --- */
#define ENV_WHO_AM_I            0x00
#define ENV_TEMP_MSB            0x01
#define ENV_TEMP_LSB            0x02
#define ENV_HUM_OUT             0x03
#define ENV_WHO_AM_I_VAL        0xEB

#define ENV_CALIB_TEMP_OFFMSB   0x0D
#define ENV_CALIB_TEMP_OFFLSB   0x0E
#define ENV_CALIB_HUM_OFF       0x0F

/* --- Private driver state --- */
struct envcombo_data {
    struct i2c_client *client;
    struct mutex lock;
};

/* --- Generic helpers --- */

/* Read 16-bit signed (MSB/LSB) */
static int envcombo_read_s16(struct envcombo_data *st, u8 reg_msb, u8 reg_lsb)
{
    int msb, lsb;
    mutex_lock(&st->lock);
    msb = i2c_smbus_read_byte_data(st->client, reg_msb);
    lsb = i2c_smbus_read_byte_data(st->client, reg_lsb);
    mutex_unlock(&st->lock);

    if (msb < 0 || lsb < 0) {
        dev_err(&st->client->dev,
                "Failed to read s16 regs 0x%02x/0x%02x (msb=%d, lsb=%d)\n",
                reg_msb, reg_lsb, msb, lsb);
        return -EIO;
    }
    return (s16)((msb << 8) | lsb);
}

/* Write 16-bit signed (MSB/LSB) */
static int envcombo_write_s16(struct envcombo_data *st, u8 reg_msb, u8 reg_lsb, s16 val)
{
    int ret;
    mutex_lock(&st->lock);
    ret = i2c_smbus_write_byte_data(st->client, reg_msb, (val >> 8) & 0xFF);
    if (ret < 0)
        dev_err(&st->client->dev, "Failed to write MSB reg 0x%02x (val=%d)\n", reg_msb, val);
    ret = i2c_smbus_write_byte_data(st->client, reg_lsb, val & 0xFF);
    if (ret < 0)
        dev_err(&st->client->dev, "Failed to write LSB reg 0x%02x (val=%d)\n", reg_lsb, val);
    mutex_unlock(&st->lock);
    return 0;
}

/* Read 8-bit signed */
static int envcombo_read_s8(struct envcombo_data *st, u8 reg)
{
    int v;
    mutex_lock(&st->lock);
    v = i2c_smbus_read_byte_data(st->client, reg);
    mutex_unlock(&st->lock);
    if (v < 0) {
        dev_err(&st->client->dev, "Failed to read s8 reg 0x%02x (ret=%d)\n", reg, v);
        return -EIO;
    }
    return (s8)v;
}

/* Write 8-bit signed */
static int envcombo_write_s8(struct envcombo_data *st, u8 reg, s8 val)
{
    int ret;
    mutex_lock(&st->lock);
    ret = i2c_smbus_write_byte_data(st->client, reg, val);
    mutex_unlock(&st->lock);
    if (ret < 0) {
        dev_err(&st->client->dev, "Failed to write s8 reg 0x%02x (val=%d)\n", reg, val);
        return -EIO;
    }
    return 0;
}

/* --- Channel-specific wrappers --- */
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
    return envcombo_write_s16(st, ENV_CALIB_TEMP_OFFMSB, ENV_CALIB_TEMP_OFFLSB, val);
}

static inline int envcombo_read_hum_offset(struct envcombo_data *st)
{
    return envcombo_read_s8(st, ENV_CALIB_HUM_OFF);
}

static inline int envcombo_write_hum_offset(struct envcombo_data *st, int val)
{
    return envcombo_write_s8(st, ENV_CALIB_HUM_OFF, (s8)val);
}

/* --- IIO callbacks --- */
static int envcombo_read_raw(struct iio_dev *indio_dev,
                             struct iio_chan_spec const *chan,
                             int *val, int *val2, long mask)
{
    struct envcombo_data *st = iio_priv(indio_dev);
    int raw, off;

    switch (chan->type) {
    case IIO_TEMP:
        switch (mask) {
        case IIO_CHAN_INFO_RAW:
            raw = envcombo_read_temp(st);
            if (raw < 0) return raw;
            *val = raw;
            return IIO_VAL_INT;

        case IIO_CHAN_INFO_SCALE:
            *val = 0; *val2 = 10000; // 0.01 °C
            return IIO_VAL_INT_PLUS_MICRO;

        case IIO_CHAN_INFO_OFFSET:
            *val = envcombo_read_temp_offset(st);
            return IIO_VAL_INT;

        case IIO_CHAN_INFO_PROCESSED:
            raw = envcombo_read_temp(st);
            if (raw < 0) return raw;
            off = envcombo_read_temp_offset(st);
            if (off < 0) off = 0;

            /* (raw + offset) * 0.01°C = milli°C */
            *val = (raw + off) * 10;
            return IIO_VAL_INT;   // in milli°C
        }
        break;

    case IIO_HUMIDITYRELATIVE:
        switch (mask) {
        case IIO_CHAN_INFO_RAW:
            raw = envcombo_read_hum(st);
            if (raw < 0) return raw;
            *val = raw;
            return IIO_VAL_INT;

        case IIO_CHAN_INFO_SCALE:
            *val = 0; *val2 = 500000; // 0.5 %RH
            return IIO_VAL_INT_PLUS_MICRO;

        case IIO_CHAN_INFO_OFFSET:
            *val = envcombo_read_hum_offset(st);
            return IIO_VAL_INT;

        case IIO_CHAN_INFO_PROCESSED:
            raw = envcombo_read_hum(st);
            if (raw < 0) return raw;
            off = envcombo_read_hum_offset(st);
            if (off < 0) off = 0;

            /* (raw + offset) * 0.5 %RH = milli-%RH */
            *val = (raw + off) * 500;
            return IIO_VAL_INT;   // in milli-%RH
        }
        break;

    default:
        return -EINVAL;
    }

    return -EINVAL;
}


static int envcombo_write_raw(struct iio_dev *indio_dev,
                              struct iio_chan_spec const *chan,
                              int val, int val2, long mask)
{
    struct envcombo_data *st = iio_priv(indio_dev);

    switch (chan->type) {
    case IIO_TEMP:
        if (mask == IIO_CHAN_INFO_OFFSET)
            return envcombo_write_temp_offset(st, val);
        break;

    case IIO_HUMIDITYRELATIVE:
        if (mask == IIO_CHAN_INFO_OFFSET)
            return envcombo_write_hum_offset(st, val);
        break;
    default:
        return -EINVAL;   // not supported
    }
    return -EINVAL;
}

static const struct iio_info envcombo_iio_info = {
    .read_raw = envcombo_read_raw,
    .write_raw = envcombo_write_raw,
};

/* --- Channel specs --- */
static const struct iio_chan_spec envcombo_channels[] = {
    {
        .type = IIO_TEMP,
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
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
                              BIT(IIO_CHAN_INFO_SCALE) |
                              BIT(IIO_CHAN_INFO_OFFSET) |
                              BIT(IIO_CHAN_INFO_PROCESSED),
        .scan_type = {
            .sign = 'u', .realbits = 8, .storagebits = 16, .endianness = IIO_LE,
        },
    },
};

/* --- Probe / Remove --- */
static int envcombo_probe(struct i2c_client *client,
                          const struct i2c_device_id *id)
{
    struct iio_dev *indio_dev;
    struct envcombo_data *st;
    int whoami;

    printk(KERN_INFO "PROBING 1....\n");

    whoami = i2c_smbus_read_byte_data(client, ENV_WHO_AM_I);
    if (whoami != ENV_WHO_AM_I_VAL) {
        dev_err(&client->dev, "WHO_AM_I mismatch: 0x%x\n", whoami);
        return -ENODEV;
    }

    indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*st));
    if (!indio_dev)
        return -ENOMEM;

    st = iio_priv(indio_dev);
    st->client = client;
    mutex_init(&st->lock);

    indio_dev->dev.parent = &client->dev;
    indio_dev->info = &envcombo_iio_info;
    indio_dev->channels = envcombo_channels;
    indio_dev->num_channels = ARRAY_SIZE(envcombo_channels);
    indio_dev->name = "envcombo";
    indio_dev->modes = INDIO_DIRECT_MODE;

    return devm_iio_device_register(&client->dev, indio_dev);
}

static int envcombo_remove(struct i2c_client *client)
{
    dev_info(&client->dev, "envcombo removed\n");
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

MODULE_AUTHOR("Sami Khatib");
MODULE_DESCRIPTION("ENV-COMBO Temp/Humidity IIO driver (clean version)");
MODULE_LICENSE("GPL");
