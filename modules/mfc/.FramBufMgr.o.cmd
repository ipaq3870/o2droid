cmd_../modules/mfc/FramBufMgr.o := /opt/cross/bin/arm-linux-gnueabi-gcc -Wp,-MD,../modules/mfc/.FramBufMgr.o.d  -nostdinc -isystem /opt/cross/lib/gcc/arm-linux-gnueabi/4.4.4/include -Iinclude  -I/dka/bss/Omnia/marc/kern_oII/arch/arm/include -include include/linux/autoconf.h -D__KERNEL__ -mlittle-endian -Iarch/arm/mach-s3c6400/include -Iarch/arm/mach-s3c6410/include -Iarch/arm/plat-s3c64xx/include -Iarch/arm/plat-s3c/include -Wall -Wundef -Wstrict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -Werror-implicit-function-declaration -Wno-format-security -fno-delete-null-pointer-checks -O2 -marm -fno-omit-frame-pointer -mapcs -mno-sched-prolog -mabi=aapcs-linux -mno-thumb-interwork -D__LINUX_ARM_ARCH__=6 -march=armv6k -mtune=arm1136j-s -msoft-float -Uarm -Wframe-larger-than=1024 -fno-stack-protector -fno-omit-frame-pointer -fno-optimize-sibling-calls -g -Wdeclaration-after-statement -Wno-pointer-sign -fno-strict-overflow -fno-dwarf2-cfi-asm -DLINUX -DDIVX_ENABLE  -DMODULE -D"KBUILD_STR(s)=\#s" -D"KBUILD_BASENAME=KBUILD_STR(FramBufMgr)"  -D"KBUILD_MODNAME=KBUILD_STR(s3c_mfc)"  -c -o ../modules/mfc/FramBufMgr.o ../modules/mfc/FramBufMgr.c

deps_../modules/mfc/FramBufMgr.o := \
  ../modules/mfc/FramBufMgr.c \
  ../modules/mfc/MfcMemory.h \
  ../modules/mfc/FramBufMgr.h \
  ../modules/mfc/MfcTypes.h \
  include/linux/types.h \
    $(wildcard include/config/uid16.h) \
    $(wildcard include/config/lbdaf.h) \
    $(wildcard include/config/phys/addr/t/64bit.h) \
    $(wildcard include/config/64bit.h) \
  /dka/bss/Omnia/marc/kern_oII/arch/arm/include/asm/types.h \
  include/asm-generic/int-ll64.h \
  /dka/bss/Omnia/marc/kern_oII/arch/arm/include/asm/bitsperlong.h \
  include/asm-generic/bitsperlong.h \
  include/linux/posix_types.h \
  include/linux/stddef.h \
  include/linux/compiler.h \
    $(wildcard include/config/trace/branch/profiling.h) \
    $(wildcard include/config/profile/all/branches.h) \
    $(wildcard include/config/enable/must/check.h) \
    $(wildcard include/config/enable/warn/deprecated.h) \
  include/linux/compiler-gcc.h \
    $(wildcard include/config/arch/supports/optimized/inlining.h) \
    $(wildcard include/config/optimize/inlining.h) \
  include/linux/compiler-gcc4.h \
  /dka/bss/Omnia/marc/kern_oII/arch/arm/include/asm/posix_types.h \
  ../modules/mfc/LogMsg.h \

../modules/mfc/FramBufMgr.o: $(deps_../modules/mfc/FramBufMgr.o)

$(deps_../modules/mfc/FramBufMgr.o):
