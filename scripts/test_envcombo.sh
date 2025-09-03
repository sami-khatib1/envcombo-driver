#!/bin/sh

set -e

echo "=== ENV-COMBO Test Script ==="

# 1. Load simulator
echo "[1/7] Loading simulator..."
insmod /tmp/i2c-envcombo-sim.ko || { echo "Failed to load simulator"; exit 1; }

# 2. Load driver
echo "[2/7] Loading driver..."
insmod /tmp/envcombo.ko || { echo "Failed to load driver"; exit 1; }

# 3. Register I2C client at address 0x39
echo "[3/7] Registering I2C client..."
echo envcombo 0x39 > /sys/bus/i2c/devices/i2c-0/new_device

DEV=/sys/bus/iio/devices/iio:device0

if [ ! -d "$DEV" ]; then
    echo "IIO device not found, something went wrong."
    exit 1
fi

# 4. Read baseline values
echo "[4/7] Reading baseline sysfs values..."
TEMP_RAW=$(cat $DEV/in_temp_raw)
TEMP_OFF=$(cat $DEV/in_temp_offset)
TEMP_IN=$(cat $DEV/in_temp_input)
HUM_RAW=$(cat $DEV/in_humidityrelative_raw)
HUM_OFF=$(cat $DEV/in_humidityrelative_offset)
HUM_IN=$(cat $DEV/in_humidityrelative_input)

echo "Baseline:"
echo "  Temp raw=$TEMP_RAW, offset=$TEMP_OFF, input=$TEMP_IN"
echo "  Hum  raw=$HUM_RAW, offset=$HUM_OFF, input=$HUM_IN"

# 5. Modify offsets
echo "[5/7] Modifying offsets..."
echo 100 > $DEV/in_temp_offset         # +1.00 °C
echo 4 > $DEV/in_humidityrelative_offset  # +2.0 %RH

# 6. Verify processed values updated
echo "[6/7] Verifying processed values after offsets..."
NEW_TEMP_IN=$(cat $DEV/in_temp_input)
NEW_HUM_IN=$(cat $DEV/in_humidityrelative_input)

echo "After offsets:"
echo "  Temp input=$NEW_TEMP_IN (was $TEMP_IN)"
echo "  Hum  input=$NEW_HUM_IN (was $HUM_IN)"

if [ "$NEW_TEMP_IN" -eq "$TEMP_IN" ]; then
    echo "❌ Temperature input did not change!"
else
    echo "✅ Temperature input changed as expected."
fi

if [ "$NEW_HUM_IN" -eq "$HUM_IN" ]; then
    echo "❌ Humidity input did not change!"
else
    echo "✅ Humidity input changed as expected."
fi

# 7. Cleanup
echo "[7/7] Cleaning up..."
echo 0 > /sys/bus/i2c/devices/i2c-0/delete_device || true
rmmod envcombo || true
rmmod i2c-envcombo-sim || true

echo "=== Test completed successfully ==="
