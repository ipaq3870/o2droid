cmd_../modules/param/crc32.o := /opt/cross/bin/arm-linux-gnueabi-gcc -Wp,-MD,../modules/param/.crc32.o.d  -nostdinc -isystem /opt/cross/lib/gcc/arm-linux-gnueabi/4.4.4/include -Iinclude  -I/dka/bss/Omnia/marc/kern_oII/arch/arm/include -include include/linux/autoconf.h -D__KERNEL__ -mlittle-endian -Iarch/arm/mach-s3c6400/include -Iarch/arm/mach-s3c6410/include -Iarch/arm/plat-s3c64xx/include -Iarch/arm/plat-s3c/include -Wall -Wundef -Wstrict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -Werror-implicit-function-declaration -Wno-format-security -fno-delete-null-pointer-checks -O2 -marm -fno-omit-frame-pointer -mapcs -mno-sched-prolog -mabi=aapcs-linux -mno-thumb-interwork -D__LINUX_ARM_ARCH__=6 -march=armv6k -mtune=arm1136j-s -msoft-float -Uarm -Wframe-larger-than=1024 -fno-stack-protector -fno-omit-frame-pointer -fno-optimize-sibling-calls -g -Wdeclaration-after-statement -Wno-pointer-sign -fno-strict-overflow -fno-dwarf2-cfi-asm -I./modules/xsr/Inc  -DMODULE -D"KBUILD_STR(s)=\#s" -D"KBUILD_BASENAME=KBUILD_STR(crc32)"  -D"KBUILD_MODNAME=KBUILD_STR(crc32)"  -c -o ../modules/param/crc32.o ../modules/param/crc32.c

deps_../modules/param/crc32.o := \
  ../modules/param/crc32.c \

../modules/param/crc32.o: $(deps_../modules/param/crc32.o)

$(deps_../modules/param/crc32.o):
