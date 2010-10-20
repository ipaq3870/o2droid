cmd_../modules/mfc/Prism_S_V13C.o := /opt/cross/bin/arm-linux-gnueabi-gcc -Wp,-MD,../modules/mfc/.Prism_S_V13C.o.d  -nostdinc -isystem /opt/cross/lib/gcc/arm-linux-gnueabi/4.4.4/include -Iinclude  -I/dka/bss/Omnia/marc/kern_oII/arch/arm/include -include include/linux/autoconf.h -D__KERNEL__ -mlittle-endian -Iarch/arm/mach-s3c6400/include -Iarch/arm/mach-s3c6410/include -Iarch/arm/plat-s3c64xx/include -Iarch/arm/plat-s3c/include -Wall -Wundef -Wstrict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -Werror-implicit-function-declaration -Wno-format-security -fno-delete-null-pointer-checks -O2 -marm -fno-omit-frame-pointer -mapcs -mno-sched-prolog -mabi=aapcs-linux -mno-thumb-interwork -D__LINUX_ARM_ARCH__=6 -march=armv6k -mtune=arm1136j-s -msoft-float -Uarm -Wframe-larger-than=1024 -fno-stack-protector -fno-omit-frame-pointer -fno-optimize-sibling-calls -g -Wdeclaration-after-statement -Wno-pointer-sign -fno-strict-overflow -fno-dwarf2-cfi-asm -DLINUX -DDIVX_ENABLE  -DMODULE -D"KBUILD_STR(s)=\#s" -D"KBUILD_BASENAME=KBUILD_STR(Prism_S_V13C)"  -D"KBUILD_MODNAME=KBUILD_STR(s3c_mfc)"  -c -o ../modules/mfc/Prism_S_V13C.o ../modules/mfc/Prism_S_V13C.c

deps_../modules/mfc/Prism_S_V13C.o := \
  ../modules/mfc/Prism_S_V13C.c \

../modules/mfc/Prism_S_V13C.o: $(deps_../modules/mfc/Prism_S_V13C.o)

$(deps_../modules/mfc/Prism_S_V13C.o):
