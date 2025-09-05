# Yocto Setup Guide

This document explains how to prepare a Yocto/QEMU environment for building and testing the ENV-COMBO driver.

---

## Step 0 — Enable sudo

If your Ubuntu user isn’t in sudoers:

open new terminal:

```bash
su -
usermod -aG sudo <your username>
reboot
```

---

## Step 1 — Install prerequisites

After reboot, open terminal again:

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y gawk wget git-core diffstat unzip texinfo build-essential     chrpath socat cpio python3 python3-pip python3-pexpect xz-utils debianutils     iputils-ping python3-gitpython python3-jinja2 libgl1 libglx-mesa0 libsdl1.2-dev     pylint xterm python3-subunit mesa-common-dev lz4 zstd
```

---

## Step 2 — Clone Yocto (poky)

```bash
cd ~
git clone git://git.yoctoproject.org/poky
cd poky
git checkout tags/yocto-4.0.26 -b yocto-4.0.26-local
```

---

## Step 3 — Initialize build directory

```bash
source oe-init-build-env build-qemuarm64
```

Creates `~/poky/build-qemuarm64`.

---

## Step 4 — Configure build

Append these to `conf/local.conf`:

```conf
MACHINE ?= "qemuarm64"

# Add SSH + I2C tools
CORE_IMAGE_EXTRA_INSTALL += "openssh i2c-tools"

# Allow root login
EXTRA_IMAGE_FEATURES += "debug-tweaks"

# Speed up build - adjust threads to number of cores on your machine
BB_NUMBER_THREADS = "12"
PARALLEL_MAKE = "-j12"

# Shared cache
DL_DIR ?= "/home/sami/yocto-downloads"
SSTATE_DIR ?= "/home/sami/yocto-sstate-cache"
```

---

## Step 5 — Configure kernel (I²C + IIO + buffer)

```bash
bitbake -c menuconfig virtual/kernel
```

In menuconfig, enable the following:

- Device Drivers → I2C support → I2C support  
  → `CONFIG_I2C=y`

- Device Drivers → I2C support → I2C device interface  
  → `CONFIG_I2C_CHARDEV=y`

- Device Drivers → Industrial I/O support → Enable Industrial I/O support  
  → `CONFIG_IIO=y`

- Device Drivers → Industrial I/O support → Enable buffer support within IIO  
  → `CONFIG_IIO_BUFFER=y`

- Device Drivers → Industrial I/O support → Enable IIO kfifo buffer support  
  → `CONFIG_IIO_KFIFO_BUF=y`

- Device Drivers → Industrial I/O support → Enable trigger support within IIO  
  → `CONFIG_IIO_TRIGGER=y`

- Device Drivers → Industrial I/O support → Enable triggered buffer support within IIO  
  → `CONFIG_IIO_TRIGGERED_BUFFER=y`

- Device Drivers → Industrial I/O support → Hrtimer based trigger  
  → `CONFIG_IIO_HRTIMER_TRIGGER=y`

- Device Drivers → Industrial I/O support → Triggers - standalone → High resolution timer trigger 
  → `CONFIG_IIO_TRIGGER=y`

- Device Drivers → Industrial I/O support → Triggers - standalone → SYSFS trigger 
  → `CONFIG_IIO_HRTIMER_TRIGGER=y`

Save and exit.

---

## Step 6 — Build the image

```bash
bitbake virtual/kernel -c compile -f
bitbake core-image-minimal -c rootfs -f
bitbake core-image-minimal
```

Image goes to:

```
~/poky/build-qemuarm64/tmp/deploy/images/qemuarm64/
```

---

## Step 7 — Boot image in QEMU

```bash
runqemu qemuarm64 nographic
```

Login:

```
login: root
password: (empty)
```

Check in QEMU:

```bash
uname -a
ls /dev/i2c-*
ls /sys/bus/iio/
```

---

## Step 8 — Generate cross-compile SDK (⚠️ in a separate shell, not QEMU)

```bash
cd ~/poky/build-qemuarm64
bitbake -c populate_sdk core-image-minimal
ls tmp/deploy/sdk/
sh tmp/deploy/sdk/poky-glibc-x86_64-core-image-minimal-aarch64-qemuarm64-toolchain-4.0.26.sh -d ~/yocto-sdk
source ~/yocto-sdk/environment-setup-aarch64-poky-linux
```

---

## Step 9 — Point to kernel headers

```bash
export KDIR=~/poky/build-qemuarm64/tmp/work/qemuarm64-poky-linux/linux-yocto/*/linux-qemuarm64-standard-build
```
