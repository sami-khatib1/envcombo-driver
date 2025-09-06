#!/bin/sh
# Read and decode IIO binary samples from envcombo driver (BusyBox safe)

DEVICE=/dev/iio:device0
BYTES_PER_SAMPLE=12   # 2 bytes temp + 2 bytes hum + 8 bytes timestamp
SAMPLES=${1:-10}      # default 10 if no arg

echo "=== Reading $SAMPLES samples from $DEVICE ==="
echo "Format: Temp[°C]  Hum[%RH]  Timestamp[ns]"

for i in $(seq 1 $SAMPLES); do
    dd if=$DEVICE bs=$BYTES_PER_SAMPLE count=1 2>/dev/null | \
    hexdump -v -e '1/2 "%d " 1/2 "%d " 8/1 "%02x" "\n"' | \
    awk '{
        temp=$1*0.01;
        hum=$2*0.5;
        # rebuild 64-bit timestamp from 8 hex bytes (little endian)
        ts="0x"$3$4$5$6$7$8$9$10;
        cmd="printf \"%llu\" " ts;
        cmd | getline dec; close(cmd);
        printf "Temp=%6.2f °C  Hum=%5.2f %%RH  TS=%s\n", temp, hum, dec;
    }'
    sleep 1.5
done