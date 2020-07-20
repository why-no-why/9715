ifneq ($(KERNELRELEASE),)
	obj-m := rtk_btusblinux.o
	rtk_btusblinux-y = rtk_coex.o rtk_bt.o
else
	PWD := $(shell pwd)
	KVER := $(shell uname -r)
	KDIR := /lib/modules/$(KVER)/build

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules -s

clean:
	rm -rf *.o *.mod.c *.mod.o *.ko *.symvers *.order *.a

endif
