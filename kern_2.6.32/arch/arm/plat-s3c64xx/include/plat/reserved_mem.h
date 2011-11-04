#ifndef _ASM_ARM_ARCH_RESERVED_MEM_H
#define _ASM_ARM_ARCH_RESERVED_MEM_H

#include <linux/types.h>
#include <linux/list.h>
#include <asm/setup.h>

#define DRAM_END_ADDR 				(UL(0x65000000))
//#define DRAM_END_ADDR 				(UL(0x58000000))
//#define DRAM_END_ADDR 					(UL(0x60000000))

#define RESERVED_PMEM_END_ADDR 		(DRAM_END_ADDR)



#define RESERVED_MEM_CMM		(16 * 1024 * 1024)
#define RESERVED_MEM_MFC		(12 * 1024 * 1024)
#define RESERVED_PMEM_PICTURE		(12 * 1024 * 1024)	/* PMEM_PIC and MFC use share area */
#define RESERVED_PMEM_JPEG		(6 * 1024 * 1024) //
#define RESERVED_PMEM_PREVIEW		(4 * 1024 * 1024) //2 
#define RESERVED_PMEM_RENDER	  	(4 * 1024 * 1024) //4 
#define RESERVED_PMEM_STREAM	  	(8 * 1024 * 1024) //4
#define RESERVED_G3D			(63 * 1024 * 1024) 	/* G3D is shared with uppper memory areas */ //34
#define RESERVED_PMEM			(16 * 1024 * 1024)//12 working well, i9000 16mb --> if we move to 16M then 16mb *FIX*
#define RESERVED_PMEM_SKIA		(0)//0

#define RESERVED_G3D_UI			(8 * 1024 * 1024) //(6mb -->unable 4th block
//3+6+6+3+2+6=28M
#define RESERVED_G3D_SHARED		(RESERVED_MEM_CMM + RESERVED_MEM_MFC + RESERVED_PMEM_STREAM + RESERVED_PMEM_JPEG + RESERVED_PMEM_PREVIEW + RESERVED_PMEM_RENDER) //22
#define RESERVED_G3D_APP		(RESERVED_G3D - /*RESERVED_G3D_UI*/ - RESERVED_G3D_SHARED) //32-26=6m //THIS IS ONLY FOR UNDERSTANDING PURPOSES, DOESNT DO ANYTHING, NOW 2MB (1 block)

//#if defined(CONFIG_RESERVED_MEM_CMM_JPEG_MFC_POST_CAMERA)
#define CMM_RESERVED_MEM_START		(RESERVED_PMEM_END_ADDR - RESERVED_MEM_CMM) 		//3m	-3m
#define MFC_RESERVED_MEM_START		(CMM_RESERVED_MEM_START - RESERVED_MEM_MFC)		//6m	-9m
#define PICTURE_RESERVED_PMEM_START	(MFC_RESERVED_MEM_START)				//0m	-9m
#define JPEG_RESERVED_PMEM_START	(MFC_RESERVED_MEM_START - RESERVED_PMEM_JPEG)		//3m	-12m
#define PREVIEW_RESERVED_PMEM_START	(JPEG_RESERVED_PMEM_START - RESERVED_PMEM_PREVIEW)	//2m	-14m
#define RENDER_RESERVED_PMEM_START	(PREVIEW_RESERVED_PMEM_START - RESERVED_PMEM_RENDER)	//6m	-20m
#define STREAM_RESERVED_PMEM_START	(RENDER_RESERVED_PMEM_START - RESERVED_PMEM_STREAM)	//6m	-26m
#define G3D_RESERVED_START		(RESERVED_PMEM_END_ADDR - RESERVED_G3D)			//32m	-32m G3D shared!!!
#define RESERVED_PMEM_START		(G3D_RESERVED_START - RESERVED_PMEM)		//12m	-44m
#define PHYS_UNRESERVED_SIZE		(RESERVED_PMEM_START - UL(0x60000000))
//#define PHYS_UNRESERVED_SIZE		(RESERVED_PMEM_START - UL(0x50000000))

#define SKIA_RESERVED_PMEM_START	(0)


struct s3c6410_pmem_setting{
        resource_size_t pmem_start;
        resource_size_t pmem_size;
        resource_size_t pmem_render_start;
        resource_size_t pmem_render_size;
        resource_size_t pmem_render_pic_start;
        resource_size_t pmem_render_pic_size;
        resource_size_t pmem_stream_start;
        resource_size_t pmem_stream_size;
        resource_size_t pmem_preview_start;
        resource_size_t pmem_preview_size;
        resource_size_t pmem_picture_start;
        resource_size_t pmem_picture_size;
        resource_size_t pmem_jpeg_start;
        resource_size_t pmem_jpeg_size;
};
 
void s3c6410_add_mem_devices (struct s3c6410_pmem_setting *setting);

#endif /* _ASM_ARM_ARCH_RESERVED_MEM_H */

