#ifndef __LIGHT_H__
#define __LIGHT_H__
/********************************************************************************	
 *										*
 *	ILS2903 Light Sensor Driver for Samsung Omnia II GT-I8000		*
 *		NO backlight contol, NO interrup oprations			*
 ********************************************************************************
 *
 * If you need kenel level backlight controll please check isl29023_int.h and isl29023_int.c
 * Froyo framework handles the backlight controll thats why I cleared the code 
 */

/* LUX calculation */
static int adc_resolution[] = { 0xffff, 0x0fff, 0x00ff, 0x000f }; // 16, 12, 8, 4 bit ADC resolution 
static int FSR_LUX_range[] = { 1000, 4000, 16000, 64000 }; 
/*
 * 	LUX = FSR_LUX_range[k] * sensor_data / adc_resolution[m]  
 * 	k = op_mode & 3; 
 * 	m = (op_mode >> 2) & 3;
 */

#define LIGHT_PERIOD	1		// secs in timer operation mode

#define I2C_DF_NOTIFY	0x01		// for i2c 

#define LIGHT_ADDR	0x88		// i2c slave addr

#define REGS_COMMAND_I		0x0 
#define REGS_COMMAND_II		0x1
#define REGS_LBS_SENSOR		0x2
#define REGS_MBS_SENSOR		0x3
#define REGS_INT_LSB_TH_LO	0x4
#define REGS_INT_MSB_TH_LO	0x5
#define REGS_INT_LSB_TH_HI	0x6
#define REGS_INT_MSB_TH_HI	0x7

/* power control */
#define ON              1
#define OFF		0

/* driver data */
struct light_data {
	struct input_dev *input_dev;
	struct work_struct work_light; 
	struct hrtimer timer;
};

struct workqueue_struct *light_wq;

/* IOCTL for light sensor */
#define KIONIX_ISL_IOC_MAGIC		'K'                                 
#define KIONIX_ISL_IOCTL_GET_ENABLED	_IOR(KIONIX_ISL_IOC_MAGIC, 0x1, int)            
#define KIONIX_ISL_IOCTL_ENABLE		_IOW(KIONIX_ISL_IOC_MAGIC, 0x2, int)            
#define KIONIX_ISL_PRINT_ON		_IO(KIONIX_ISL_IOC_MAGIC, 0x3)      
#define KIONIX_ISL_PRINT_OFF		_IO(KIONIX_ISL_IOC_MAGIC, 0x4)      

/* prototype */
int isl_i2c_read(u8 reg);
int isl_i2c_write( u8 reg, int val );
#endif
