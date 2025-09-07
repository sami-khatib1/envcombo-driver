# ENV-COMBO Linux Driver

This repository contains the Linux kernel driver for the fictional **ENV-COMBO** temperature and humidity sensor, implemented as part of a firmware technical assignment.  
The driver integrates with the **I²C subsystem** and the **IIO subsystem**, exposing sysfs entries for raw, scale, offset, and processed values.

---

## Repository Layout

```
driver/       # Driver source (envcombo.c, Makefile)
scripts/      # Test script (test_envcombo.sh)
simulator/    # Contains i2c-envcombo-sim.zip (the simulator kernel module)
yocto-setup.md
README.md
```

---

## Prerequisites

Before building and testing the driver, make sure your Yocto/QEMU environment is prepared.  
Follow [yocto-setup.md](./docs/yocto-setup.md) to:

- Build a `qemuarm64` Yocto image with I²C + IIO enabled.  
- Boot the image in QEMU with SSH access.  
- Generate and install the cross-compile SDK.  

> ⚠️ **Important:** Steps for building the driver must be executed in the **SDK shell**  
> (the shell where you sourced `environment-setup-aarch64-poky-linux`).

---

## Step 1 — Build the Driver

From the SDK shell:

```bash
cd driver
make
cd ..
```

This produces `envcombo.ko`.

---

## Step 2 — Extract the Simulator

The simulator module is provided as a `.zip` file in the repo.  
Unzip it to get `i2c-envcombo-sim.ko`:

```bash
unzip simulator/i2c-envcombo-sim.zip -d simulator/
```

---

## Step 3 — Copy Modules and Script into QEMU

From your host machine (not inside QEMU):

```bash
scp driver/envcombo.ko root@192.168.7.2:/tmp/
scp simulator/i2c-envcombo-sim.ko root@192.168.7.2:/tmp/
scp scripts/* root@192.168.7.2:/tmp/
```

---

## Step 4 — Run Automated Test (inside QEMU)

Inside the QEMU console:

```bash
sh /tmp/test_envcombo.sh
```

The script will:

- Load the simulator and driver.  
- Instantiate the I²C client at address `0x39`.  
- Verify sysfs nodes:

  - Temperature:  
    - `in_temp_raw` (ro)  
    - `in_temp_scale` (ro)  
    - `in_temp_offset` (rw)  
    - `in_temp_input` (ro, milli-°C)  

  - Humidity:  
    - `in_humidityrelative_raw` (ro)  
    - `in_humidityrelative_scale` (ro)  
    - `in_humidityrelative_offset` (rw)  
    - `in_humidityrelative_input` (ro, milli-%RH)  

- Modify offsets and re-read values.  
- Setup triggered buffer via hrtimer trigger.  
- Capture 10 samples to `/tmp/capture.bin` and show preview.  
- Clean up: disable buffer, remove trigger, unload modules.

---

## Notes

- Only the `*_offset` attributes are writable; all others are read-only.  
- Use `dmesg | tail` to see driver logs.  
- If you hit issues, confirm required kernel configs are enabled (see `docs/yocto-setup.md`).  
