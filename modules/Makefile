subdir-y += fm_si4709
subdir-y += vibetonz
subdir-y += rotator
subdir-y += mfc
subdir-y += jpeg
subdir-y += g3d 
subdir-y += g2d 
subdir-y += cmm 
subdir-y += pp
subdir-y += rotator
#subdir-y += camera
subdir-y += dpram
subdir-y += multipdp
subdir-y += param

SUBDIRS := $(subdir-y)

DO_INSTALL_MODS = y

all: $(SUBDIRS)

$(SUBDIRS)::
	@$(MAKE) -C $@

install: $(SUBDIRS)
ifeq ($(strip $(DO_INSTALL_MODS)),y)
	@for i in `echo $(SUBDIRS)`; do \
		$(MAKE) --no-print-directory -C $$i $@ > /dev/null 2>&1; \
	done
endif
	@$(MAKE) --no-print-directory -C $(KDIR) modules_install

clean:
	@for i in `echo $(SUBDIRS)`; do \
		$(MAKE) -C $$i $@; \
	done
