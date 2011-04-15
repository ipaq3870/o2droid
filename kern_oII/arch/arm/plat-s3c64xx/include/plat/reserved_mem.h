#ifndef _ASM_ARM_ARCH_RESERVED_MEM_H
#define _ASM_ARM_ARCH_RESERVED_MEM_H

#include <linux/types.h>
#include <linux/list.h>
#include <asm/setup.h>

#define DRAM_END_ADDR 				(UL(0x65000000))//?65

#define RESERVED_PMEM_END_ADDR 		(DRAM_END_ADDR)


//W/copybit ( 1953): stretch_copybit::run_render fail
//W/libs3c2drender( 1953): int S3c2DRender::JustdoG2D(s3c_img*, s3c_rect*, s3c_img*, s3c_rect*)::FrameSize(1463040) is too bigger than //m_pmem_render_partition_size(1048576)


#define RESERVED_MEM_CMM		(3 * 1024 * 1024) //original 3
#define RESERVED_MEM_MFC		(4 * 1024 * 1024) //was 6,maybe decrease to 4 in the future?, latest working 9
#define RESERVED_PMEM_PICTURE		(4 * 1024 * 1024)	/* PMEM_PIC and MFC use share area */
#define RESERVED_PMEM_JPEG		(2 * 1024 * 1024) // could be done -1, maybe merge with preview in the future?
#define RESERVED_PMEM_PREVIEW		(2 * 1024 * 1024) //2, 1 gives errors --> but they dont matter, could be 1 again
#define RESERVED_PMEM_RENDER	  	(3 * 1024 * 1024) //4 
#define RESERVED_PMEM_STREAM	  	(4 * 1024 * 1024) //4
#define RESERVED_G3D			(32 * 1024 * 1024) 	/* G3D is shared with uppper memory areas */ //34
#define RESERVED_PMEM_GPU1		(0) //0
#define RESERVED_PMEM			(16 * 1024 * 1024)//12 working well, i9000 16mb --> if we move to 16M then 16mb *FIX*
#define RESERVED_PMEM_SKIA		(0)//0

#define RESERVED_G3D_UI			(6 * 1024 * 1024)//was 4
//3+6+6+3+2+6=28M
#define RESERVED_G3D_SHARED		(RESERVED_MEM_CMM + RESERVED_MEM_MFC + RESERVED_PMEM_STREAM + RESERVED_PMEM_JPEG + RESERVED_PMEM_PREVIEW + RESERVED_PMEM_RENDER) //22 --> 18?
#define RESERVED_G3D_APP		(RESERVED_G3D - RESERVED_G3D_UI - RESERVED_G3D_SHARED)

//#if defined(CONFIG_RESERVED_MEM_CMM_JPEG_MFC_POST_CAMERA)
#define CMM_RESERVED_MEM_START		(RESERVED_PMEM_END_ADDR - RESERVED_MEM_CMM) 		//3m	-3m
#define MFC_RESERVED_MEM_START		(CMM_RESERVED_MEM_START - RESERVED_MEM_MFC)		//6m	-9m
#define PICTURE_RESERVED_PMEM_START	(MFC_RESERVED_MEM_START)				//0m	-9m
#define JPEG_RESERVED_PMEM_START	(MFC_RESERVED_MEM_START - RESERVED_PMEM_JPEG)		//3m	-12m
#define PREVIEW_RESERVED_PMEM_START	(JPEG_RESERVED_PMEM_START - RESERVED_PMEM_PREVIEW)	//2m	-14m
#define RENDER_RESERVED_PMEM_START	(PREVIEW_RESERVED_PMEM_START - RESERVED_PMEM_RENDER)	//6m	-20m
#define STREAM_RESERVED_PMEM_START	(RENDER_RESERVED_PMEM_START - RESERVED_PMEM_STREAM)	//6m	-26m
#define G3D_RESERVED_START		(RESERVED_PMEM_END_ADDR - RESERVED_G3D)			//32m	-32m G3D shared!!!
#define GPU1_RESERVED_PMEM_START	(G3D_RESERVED_START - RESERVED_PMEM_GPU1)		//0m
#define RESERVED_PMEM_START		(GPU1_RESERVED_PMEM_START - RESERVED_PMEM)		//12m	-44m
#define PHYS_UNRESERVED_SIZE		(RESERVED_PMEM_START - UL(0x60000000))

#define SKIA_RESERVED_PMEM_START	(0)


struct s3c6410_pmem_setting{
        resource_size_t pmem_start;
        resource_size_t pmem_size;
        resource_size_t pmem_gpu1_start;
        resource_size_t pmem_gpu1_size;
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
        resource_size_t pmem_skia_start;
        resource_size_t pmem_skia_size;
};
 
void s3c6410_add_mem_devices (struct s3c6410_pmem_setting *setting);

#endif /* _ASM_ARM_ARCH_RESERVED_MEM_H */

