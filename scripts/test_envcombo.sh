#!/bin/sh
set -e

echo "=== ENV-COMBO Full Test Script ==="

# 0. Pre-clean
##if lsmod | grep -q envcombo; then
  ##  echo "[0] Removing existing envcombo module..."
    ##echo 0x39 > /sys/bus/i2c/devices/i2c-0/delete_device || true
    ##rmmod envcombo || true
##fi
##if lsmod | grep -q i2c_envcombo_sim; then
  ##  echo "[0] Removing existing simulator module..."
   ## rmmod i2c_envcombo_sim || true
##fi

# 1. Load driver if not already loaded
if ! lsmod | grep -q envcombo; then
    echo "[1/9] Loading driver..."
    insmod /tmp/envcombo.ko
else
    echo "[1/9] Driver already loaded, skipping..."
fi

# 2. Load simulator if not already loaded
if ! lsmod | grep -q i2c_envcombo_sim; then
    echo "[2/9] Loading simulator..."
    insmod /tmp/i2c-envcombo-sim.ko
else
    echo "[2/9] Simulator already loaded, skipping..."
fi

# 3. Register I2C client if not already registered
if [ ! -d /sys/bus/i2c/devices/0-0039 ]; then
    echo "[3/9] Registering I2C client..."
    echo envcombo 0x39 > /sys/bus/i2c/devices/i2c-0/new_device
else
    echo "[3/9] I2C client already registered, skipping..."
fi

## sleep to make sure all is set 
sleep 2

DEV=/sys/bus/iio/devices/iio:device0
if [ ! -d "$DEV" ]; then
    echo "IIO device not found, something went wrong."
    exit 1
fi

# 4. Baseline sysfs read
# Make sure buffer is disabled before sysfs reads
if [ -e /sys/bus/iio/devices/iio:device0/buffer/enable ]; then
    echo 0 > /sys/bus/iio/devices/iio:device0/buffer/enable 2>/dev/null || true
fi

echo "[4/9] Reading baseline sysfs values..."
sleep 0.3
TEMP_RAW=$(cat $DEV/in_temp_raw)
TEMP_SCALE=$(cat $DEV/in_temp_scale)
TEMP_IN=$(awk -v raw=$TEMP_RAW -v scale=0.01 'BEGIN { printf "%.2f", raw*scale }')

HUM_RAW=$(cat $DEV/in_humidityrelative_raw)
HUM_SCALE=$(cat $DEV/in_humidityrelative_scale)
HUM_IN=$(awk -v raw=$HUM_RAW -v scale=0.5 'BEGIN { printf "%.2f", raw*scale }')

echo "Baseline:"
echo "  Temp raw=$TEMP_RAW -> $TEMP_IN °C"
echo "  Hum  raw=$HUM_RAW -> $HUM_IN %RH"


# 5. Offsets
echo "[5/9] Applying offsets..."
echo 1000 > $DEV/in_temp_offset             # +10 °C
echo 40 > $DEV/in_humidityrelative_offset   # +20 %RH

# 6. Verify offsets effect
echo "[6/9] Checking processed values..."
NEW_TEMP_IN=$(cat $DEV/in_temp_input)
NEW_HUM_IN=$(cat $DEV/in_humidityrelative_input)

NEW_TEMP_REAL=$(awk -v v=$NEW_TEMP_IN 'BEGIN { printf "%.2f", v/1000 }')
NEW_HUM_REAL=$(awk -v v=$NEW_HUM_IN 'BEGIN { printf "%.2f", v/1000 }')

echo "After offsets:"
echo "  Temp=${NEW_TEMP_REAL} °C (was ${TEMP_IN} °C)"
echo "  Hum=${NEW_HUM_REAL} %RH (was ${HUM_IN} %RH)"


# 7. Buffer + trigger setup
echo "[7/9] Setting up buffer and trigger..."
echo 1 > $DEV/scan_elements/in_temp_en
echo 1 > $DEV/scan_elements/in_humidityrelative_en
echo 128 > $DEV/buffer/length
echo envcombo-trigger > $DEV/trigger/current_trigger
echo 1 > $DEV/buffer/enable

# 8. Run binary read test (default 10 samples if no param)
SAMPLES=${1:-10}
echo "[8/9] Running read_iio.sh for $SAMPLES samples..."
/var/volatile/tmp/read_iio.sh $SAMPLES

# 9. Cleanup
echo "[9/9] Cleaning up..."
echo 0 > $DEV/buffer/enable || true
echo 0 > /sys/bus/i2c/devices/i2c-0/delete_device || true
# First delete the client and enable buffer
echo 0 > /sys/bus/iio/devices/iio:device0/buffer/enable 2>/dev/null || true
echo 0x39 > /sys/bus/i2c/devices/i2c-0/delete_device || true

# Then remove driver
rmmod envcombo || true
rmmod i2c-envcombo-sim || true

echo "=== Full Test Completed Successfully ==="
