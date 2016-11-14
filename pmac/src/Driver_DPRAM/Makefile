#----------------------------------------------------------------
#TARGET := hello-1
#WARN := -W -Wall -Wstrict-prototypes -Wmissing-prototypes
#INCLUDE := -isystem /lib/modules/`uname -r`/build/include
#CFLAGS := -O2 -DMODULE -D__KERNEL__ ${WARN} ${INCLUDE}
#CC := gcc-3.0

#${TARGET}.o: ${TARGET}.c

#.PHONY: clean

#clean:
#rm -rf {TARGET}.o
#----------------------------------------------------------------


# To build modules outside of the kernel tree, we run "make"
# in the kernel source tree; the Makefile these then includes this
# Makefile once again.
# This conditional selects whether we are being included from the
# kernel Makefile or not.
ifeq ($(KERNELRELEASE),)

    # Assume the source tree is where the running kernel was built
    # You should set KERNELDIR in the environment if it's elsewhere
    KERNELDIR ?= /lib/modules/$(shell uname -r)/build
    # The current directory is passed to sub-makes as argument
    PWD := $(shell pwd)

modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions

.PHONY: modules modules_install clean

else    
    # called from kernel build system just declare what our modules are    
obj-m := dpram_driver.o

endif
