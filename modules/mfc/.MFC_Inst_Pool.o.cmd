cmd_../modules/mfc/MFC_Inst_Pool.o := /opt/cross/bin/arm-linux-gnueabi-gcc -Wp,-MD,../modules/mfc/.MFC_Inst_Pool.o.d  -nostdinc -isystem /opt/cross/lib/gcc/arm-linux-gnueabi/4.4.4/include -Iinclude  -I/dka/bss/Omnia/marc/kern_oII/arch/arm/include -include include/linux/autoconf.h -D__KERNEL__ -mlittle-endian -Iarch/arm/mach-s3c6400/include -Iarch/arm/mach-s3c6410/include -Iarch/arm/plat-s3c64xx/include -Iarch/arm/plat-s3c/include -Wall -Wundef -Wstrict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -Werror-implicit-function-declaration -Wno-format-security -fno-delete-null-pointer-checks -O2 -marm -fno-omit-frame-pointer -mapcs -mno-sched-prolog -mabi=aapcs-linux -mno-thumb-interwork -D__LINUX_ARM_ARCH__=6 -march=armv6k -mtune=arm1136j-s -msoft-float -Uarm -Wframe-larger-than=1024 -fno-stack-protector -fno-omit-frame-pointer -fno-optimize-sibling-calls -g -Wdeclaration-after-statement -Wno-pointer-sign -fno-strict-overflow -fno-dwarf2-cfi-asm -DLINUX -DDIVX_ENABLE  -DMODULE -D"KBUILD_STR(s)=\#s" -D"KBUILD_BASENAME=KBUILD_STR(MFC_Inst_Pool)"  -D"KBUILD_MODNAME=KBUILD_STR(s3c_mfc)"  -c -o ../modules/mfc/MFC_Inst_Pool.o ../modules/mfc/MFC_Inst_Pool.c

deps_../modules/mfc/MFC_Inst_Pool.o := \
  ../modules/mfc/MFC_Inst_Pool.c \
  ../modules/mfc/MfcConfig.h \
    $(wildcard include/config/h//.h) \
  include/linux/version.h \
  /dka/bss/Omnia/marc/kern_oII/arch/arm/include/asm/memory.h \
    $(wildcard include/config/mmu.h) \
    $(wildcard include/config/page/offset.h) \
    $(wildcard include/config/highmem.h) \
    $(wildcard include/config/dram/size.h) \
    $(wildcard include/config/dram/base.h) \
    $(wildcard include/config/zone/dma.h) \
    $(wildcard include/config/discontigmem.h) \
    $(wildcard include/config/sparsemem.h) \
  include/linux/compiler.h \
    $(wildcard include/config/trace/branch/profiling.h) \
    $(wildcard include/config/profile/all/branches.h) \
    $(wildcard include/config/enable/must/check.h) \
    $(wildcard include/config/enable/warn/deprecated.h) \
  include/linux/compiler-gcc.h \
    $(wildcard include/config/arch/supports/optimized/inlining.h) \
    $(wildcard include/config/optimize/inlining.h) \
  include/linux/compiler-gcc4.h \
  include/linux/const.h \
  arch/arm/mach-s3c6400/include/mach/memory.h \
  /dka/bss/Omnia/marc/kern_oII/arch/arm/include/asm/sizes.h \
  include/asm-generic/memory_model.h \
    $(wildcard include/config/flatmem.h) \
    $(wildcard include/config/sparsemem/vmemmap.h) \
  arch/arm/mach-s3c6400/include/mach/hardware.h \
  arch/arm/mach-s3c6400/include/mach/Omnia_II.h \
    $(wildcard include/config/kernel/logging.h) \
    $(wildcard include/config/reserved/mem/cmm/jpeg/mfc/post/camera.h) \
  arch/arm/plat-s3c64xx/include/plat/reserved_mem.h \
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
  /dka/bss/Omnia/marc/kern_oII/arch/arm/include/asm/posix_types.h \
  include/linux/list.h \
    $(wildcard include/config/debug/list.h) \
  include/linux/poison.h \
  include/linux/prefetch.h \
  /dka/bss/Omnia/marc/kern_oII/arch/arm/include/asm/processor.h \
  /dka/bss/Omnia/marc/kern_oII/arch/arm/include/asm/ptrace.h \
    $(wildcard include/config/cpu/endian/be8.h) \
    $(wildcard include/config/arm/thumb.h) \
    $(wildcard include/config/smp.h) \
  /dka/bss/Omnia/marc/kern_oII/arch/arm/include/asm/hwcap.h \
  /dka/bss/Omnia/marc/kern_oII/arch/arm/include/asm/cache.h \
    $(wildcard include/config/aeabi.h) \
  /dka/bss/Omnia/marc/kern_oII/arch/arm/include/asm/system.h \
    $(wildcard include/config/cpu/xsc3.h) \
    $(wildcard include/config/cpu/fa526.h) \
    $(wildcard include/config/cpu/sa1100.h) \
    $(wildcard include/config/cpu/sa110.h) \
    $(wildcard include/config/cpu/32v6k.h) \
  include/linux/linkage.h \
  /dka/bss/Omnia/marc/kern_oII/arch/arm/include/asm/linkage.h \
  include/linux/irqflags.h \
    $(wildcard include/config/trace/irqflags.h) \
    $(wildcard include/config/irqsoff/tracer.h) \
    $(wildcard include/config/preempt/tracer.h) \
    $(wildcard include/config/trace/irqflags/support.h) \
    $(wildcard include/config/x86.h) \
  include/linux/typecheck.h \
  /dka/bss/Omnia/marc/kern_oII/arch/arm/include/asm/irqflags.h \
  include/asm-generic/cmpxchg-local.h \
  /dka/bss/Omnia/marc/kern_oII/arch/arm/include/asm/setup.h \
    $(wildcard include/config/arch/lh7a40x.h) \
  ../modules/mfc/MFC_Inst_Pool.h \

../modules/mfc/MFC_Inst_Pool.o: $(deps_../modules/mfc/MFC_Inst_Pool.o)

$(deps_../modules/mfc/MFC_Inst_Pool.o):
