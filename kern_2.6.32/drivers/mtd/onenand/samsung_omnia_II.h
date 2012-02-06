/* linux/drivers/mtd/onenand/samsung.h
 *
 * Generic Partition Layout for S5PV210
 *
 */


struct mtd_partition s3c_partition_info[] = {
/*	{
		.name		= "boot",
		.offset		= (0),          
		.size		= (768*SZ_1K),
		.mask_flags	= MTD_CAP_NANDFLASH,
	},
	{
		.name		= "misc",
		.offset		= (768*SZ_1K),          
		.size		= (256*SZ_1K),
		.mask_flags	= MTD_CAP_NANDFLASH,
	},
*/	{
		.name		= "nv_data",
		.offset		= ((768+256+509824)*SZ_1K), //must all the previous commented sizes
		.size		= (512*SZ_1K),
	},
/*	{
		.name		= "data",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL
	}, 
*/};

