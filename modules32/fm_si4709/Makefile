##############################################################################
# COPYRIGHT(C)	: Samsung Electronics Co.Ltd, 2006-2011 ALL RIGHTS RESERVED
# AUTHOR		: Varun Mahajan <m.varun@samsung.com>
##############################################################################
# VERSION&DATE	: Version 0.1
##############################################################################

#MOD_NAME = Si4709_driver
obj-m 		:= Si4709_driver.o

EXTRA_CFLAGS 	:= -I$(PRJROOT)/modules/include

Si4709_driver-y 	:= Si4709_dev.o \
				Si4709_i2c_drv.o \
				Si4709_main.o

all:
	@echo "$(MAKE) --no-print-directory -C $(KDIR) SUBDIRS=$(CURDIR) modules"
	@$(MAKE) --no-print-directory -C $(KDIR) \
		SUBDIRS=$(CURDIR) modules

clean:
	rm -rf *.o *.ko *.mod.* *~ .*.cmd Module.symvers modules.order .tmp_versions .*.o.d 

install:
	@$(MAKE) --no-print-directory -C $(KDIR) \
		SUBDIRS=$(CURDIR) modules_install
