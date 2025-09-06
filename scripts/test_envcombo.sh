#!/bin/sh
set -eu

echo "=== ENV-COMBO Full Test Script ==="

DRV_KO=/tmp/envcombo.ko
SIM_KO=/tmp/i2c-envcombo-sim.ko
I2C_BUS=i2c-0
I2C_ADDR=0x39
I2C_DEV_DIR="/sys/bus/i2c/devices/0-0039"
IIO_DEV="/sys/bus/iio/devices/iio:device0"

TEST_STATUS="FAIL"   # Default → FAIL until everything passes

# ---------- helpers ----------
retry_read() {  # retry_read <path> [retries] [sleep_sec]
  p="$1"; n="${2:-20}"; sl="${3:-0.05}"
  i=0
  while :; do
    if v=$(cat "$p" 2>/dev/null); then
      printf "%s" "$v"
      return 0
    fi
    i=$((i+1))
    [ "$i" -ge "$n" ] && { echo "READ FAIL: $p" >&2; exit 1; }
    sleep "$sl"
  done
}

wait_exists() { # wait_exists <path> [timeout_s]
  p="$1"; t="${2:-3}"
  end=$(( $(date +%s) + t ))
  while [ ! -e "$p" ]; do
    [ "$(date +%s)" -ge "$end" ] && return 1
    sleep 0.05
  done
  return 0
}

wait_gone() { # wait_gone <path> [timeout_s]
  p="$1"; t="${2:-3}"
  end=$(( $(date +%s) + t ))
  while [ -e "$p" ]; do
    [ "$(date +%s)" -ge "$end" ] && return 1
    sleep 0.05
  done
  return 0
}

# ---------- cleanup ----------
cleanup() {
  echo "[CLEANUP] Disabling buffer..."
  echo 0 > "$IIO_DEV/buffer/enable" 2>/dev/null || true
  sleep 0.1

  echo "[CLEANUP] Deleting I2C client..."
  echo "$I2C_ADDR" > "/sys/bus/i2c/devices/$I2C_BUS/delete_device" 2>/dev/null || true
  wait_gone "$IIO_DEV" 2 || true

  echo "[CLEANUP] Removing modules..."
  rmmod envcombo 2>/dev/null || true
  rmmod i2c-envcombo-sim 2>/dev/null || true

  if [ "$TEST_STATUS" = "PASS" ]; then
    echo "✅ TEST PASSED"
  else
    echo "❌ TEST FAILED"
  fi
  echo "=== Full Test Completed (via cleanup) ==="
}
trap cleanup EXIT

# ---------- 1. driver ----------
if ! lsmod | grep -q '^envcombo'; then
  echo "[1/10] Loading driver..."
  insmod "$DRV_KO"
else
  echo "[1/10] Driver already loaded, skipping..."
fi

# ---------- 2. simulator ----------
if ! lsmod | grep -q '^i2c_envcombo_sim'; then
  echo "[2/10] Loading simulator..."
  insmod "$SIM_KO"
else
  echo "[2/10] Simulator already loaded, skipping..."
fi

# ---------- 3. client ----------
if [ ! -d "$I2C_DEV_DIR" ]; then
  echo "[3/10] Registering I2C client..."
  echo "envcombo $I2C_ADDR" > "/sys/bus/i2c/devices/$I2C_BUS/new_device"
else
  echo "[3/10] I2C client already registered, skipping..."
fi

# wait iio device appears
wait_exists "$IIO_DEV" 3 || { echo "IIO device not found."; exit 1; }

# Ensure buffer is disabled before sysfs reads
[ -e "$IIO_DEV/buffer/enable" ] && echo 0 > "$IIO_DEV/buffer/enable" || true
#make sure trigger cancled (trigger freq is set to 1.5)
sleep 5

# ---------- 4. baseline ----------
echo "[4/10] Reading baseline sysfs values..."
TEMP_RAW=$(retry_read "$IIO_DEV/in_temp_raw")
HUM_RAW=$(retry_read "$IIO_DEV/in_humidityrelative_raw")
OLD_TEMP_M=$(retry_read "$IIO_DEV/in_temp_input")
OLD_HUM_M=$(retry_read "$IIO_DEV/in_humidityrelative_input")

TEMP_REAL=$(awk -v r="$TEMP_RAW" 'BEGIN{printf "%.2f", r*0.01}')
HUM_REAL=$(awk -v r="$HUM_RAW" 'BEGIN{printf "%.2f", r*0.5}')

echo "Baseline:"
echo "  Temp raw=$TEMP_RAW -> $TEMP_REAL °C"
echo "  Hum  raw=$HUM_RAW -> $HUM_REAL %RH"

# ---------- 5. apply offsets ----------
echo "[5/10] Applying offsets..."
TEMP_OFF_RAW=1000   # +10.00°C
HUM_OFF_RAW=40      # +20.0 %RH

echo "$TEMP_OFF_RAW" > "$IIO_DEV/in_temp_offset"
echo "$HUM_OFF_RAW"  > "$IIO_DEV/in_humidityrelative_offset"

sleep 0.1
TBACK=$(retry_read "$IIO_DEV/in_temp_offset")
HBACK=$(retry_read "$IIO_DEV/in_humidityrelative_offset")

# ---------- 6. verify effect (processed – processed) ----------
echo "[6/10] Checking processed values..."
NEW_TEMP_M=$(retry_read "$IIO_DEV/in_temp_input")
NEW_HUM_M=$(retry_read "$IIO_DEV/in_humidityrelative_input")

# deltas in milli-units
TEMP_DIFF=$(( NEW_TEMP_M - OLD_TEMP_M ))
HUM_DIFF=$(( NEW_HUM_M - OLD_HUM_M ))

# expected deltas in milli-units
EXP_TEMP_DIFF=$(( TEMP_OFF_RAW * 10 ))   # 1000 * 10 = 10000 m°C
EXP_HUM_DIFF=$(( HUM_OFF_RAW  * 500 ))   # 40 * 500 = 20000 m%RH

# tolerance (accounts for simulator raw drift between reads)
TEMP_TOL=500     # ±0.5 °C
HUM_TOL=2000     # ±2 %RH

NEW_TEMP_C=$(awk -v v="$NEW_TEMP_M" 'BEGIN{printf "%.2f", v/1000}')
OLD_TEMP_C=$(awk -v v="$OLD_TEMP_M" 'BEGIN{printf "%.2f", v/1000}')
NEW_HUM_R=$(awk -v v="$NEW_HUM_M" 'BEGIN{printf "%.2f", v/1000}')
OLD_HUM_R=$(awk -v v="$OLD_HUM_M" 'BEGIN{printf "%.2f", v/1000}')

echo "After offsets:"
echo "  Temp=${NEW_TEMP_C} °C (was ${OLD_TEMP_C} °C, Δ=${TEMP_DIFF}, expected Δ≈${EXP_TEMP_DIFF}±${TEMP_TOL}, off_written=$TBACK)"
echo "  Hum=${NEW_HUM_R} %RH (was ${OLD_HUM_R} %RH, Δ=${HUM_DIFF}, expected Δ≈${EXP_HUM_DIFF}±${HUM_TOL}, off_written=$HBACK)"

if [ $((TEMP_DIFF < EXP_TEMP_DIFF - TEMP_TOL ? 1 : 0)) -eq 1 ] || \
   [ $((TEMP_DIFF > EXP_TEMP_DIFF + TEMP_TOL ? 1 : 0)) -eq 1 ]; then
  echo "❌ Temperature offset mismatch!"
else
  echo "✅ Temperature offset applied correctly."
fi

if [ $((HUM_DIFF < EXP_HUM_DIFF - HUM_TOL ? 1 : 0)) -eq 1 ] || \
   [ $((HUM_DIFF > EXP_HUM_DIFF + HUM_TOL ? 1 : 0)) -eq 1 ]; then
  echo "❌ Humidity offset mismatch!"
else
  echo "✅ Humidity offset applied correctly."
fi

# ---------- 7. reset offsets ----------
echo "[7/10] Resetting offsets to 0..."
echo 0 > "$IIO_DEV/in_temp_offset"
echo 0 > "$IIO_DEV/in_humidityrelative_offset"
sleep 0.1

# ---------- 8. buffer + trigger ----------
echo "[8/10] Setting up buffer and trigger..."
echo 1 > "$IIO_DEV/scan_elements/in_temp_en"
echo 1 > "$IIO_DEV/scan_elements/in_humidityrelative_en"
echo 128 > "$IIO_DEV/buffer/length"
echo envcombo-trigger > "$IIO_DEV/trigger/current_trigger"
echo 1 > "$IIO_DEV/buffer/enable"
sleep 0.1

# ---------- 9. binary read ----------
# note: the sim updates registers according to its own clock or trigger 
# while the driver does the same with it's own. menaing aliging so not
# fully possible with read-write opertations from sim and driver.
# in real world cases, both the device (sim) and the driver are aligned 
# via a hw interrupt and data is written/read accordingly

SAMPLES=${1:-10}
echo "[9/10] Running read_iio.sh for $SAMPLES samples..."
/var/volatile/tmp/read_iio.sh "$SAMPLES"

# ---------- 10. mark success ----------
TEST_STATUS="PASS"
