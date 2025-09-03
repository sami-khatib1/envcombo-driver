obj-m += envcombo.o

KDIR := $(HOME)/poky/build-qemuarm64/tmp/work/qemuarm64-poky-linux/linux-yocto/*/linux-qemuarm64-standard-build

all:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=arm64 CROSS_COMPILE=aarch64-poky-linux- modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
