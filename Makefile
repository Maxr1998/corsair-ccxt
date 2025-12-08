obj-m := corsair-ccxt.o


ifndef KERNELRELEASE
KRELEASE := $(shell uname -r)
else
KRELEASE := $(KERNELRELEASE)
endif

KDIR := /usr/lib/modules/$(KRELEASE)/build
PWD := $(shell pwd)

all: default

debug-tool: debug-tool.c
	gcc -o $@ $< -lhidapi-hidraw

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	rm -f corsair-ccxt.ko.zst debug-tool

install: default
	zstd -f corsair-ccxt.ko
	sudo cp -f corsair-ccxt.ko.zst /lib/modules/6.8.0-88-generic/kernel/drivers/hwmon/
	sudo rmmod corsair_ccxt || true
	sudo modprobe corsair_ccxt
