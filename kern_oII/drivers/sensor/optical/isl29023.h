#ifndef __LIGHT_H__
#define __LIGHT_H__
/********************************************************************************	
 *										*
 *	ILS2903 Light Sensor Driver for Samsung Omnia II GT-I8000		*
 *			with backlight contol					*
 ********************************************************************************
 *
 *	Setup interface:
 *		/sys/class/lightsensor/switch_cmd/lightsensor_file_cmd
 *
 *	Turn on 
 *	    default mode: (0x1a304) 
 *
 *		echo 1 > /sys/class/lightsensor/switch_cmd/lightsensor_file_cmd 
 *
 *	Turn on 
 *	    timer mode, 1-4000 LUX range, 16 bit ADC, (0.06 lux/count) resolution
 *
 *		echo 2001 > /sys/class/lightsensor/switch_cmd/lightsensor_file_cmd 
 *									 
 *	Turn off
 *		echo 0 > /sys/class/lightsensor/switch_cmd/lightsensor_file_cmd 
 *		
 *	To check current operation mode
 *		cat /sys/class/lightsensor/switch_cmd/lightsensor_file_cmd
 *
 *	Setup parameters: ICP0M	(default: 0x1a304)
 *  	Operation modes: 
 *  		timer 				I = 0
 *  		interrupt			I = 1
 *
 * 	Conversion modes:
 * 	C:	BITS 7 TO 4 OPERATION		
 * 		0000 Power-down the device	C = 0
 * 		0010 ALS once			C = 2
 * 		0100 IR once			C = 4
 * 		1000 Reserved (Do not use)
 * 		1010 ALS continuous		C = A
 * 		1100 IR continuous		C = E
 * 		1110 Reserved (Do not use) 	
 *
 * 	INTERRUPT PERSIST (number of mesurement before set interrupt)
 * 	P:	BIT 1:0 NUMBER OF INTEGRATION CYCLES
 * 		0000 	1			P = 0
 * 		0001 	4			P = 1
 * 		0010 	8			P = 2
 * 		0011 	16			P = 3
 * 
 * 	ADC RESOLUTION DATA WIDTH 
 * 	M:	BITS 3:2 NUMBER OF CLOCK CYCLES n-BIT ADC
 * 		00 	2^16 = 65,536 		16
 * 		01 	2^12 = 4,096 		12
 * 		10 	2^8 = 256 		8
 * 		11 	2^4 = 16 		4
 *
 * 	RANGE/FSR LUX RESOLUTION
 * 	M:	BITS 1:0	RANGE(k)	FSR (MAX LUX) 
 * 		00 		0  		1,000
 * 		01 		1		4,000 
 * 		10 		2		16,000
 * 		11 		3		64,000 
 *
 *  	TO get ouput data (LUX) :
 *  		cat /sys/class/lightsensor/switch_cmd/get_lux
 *  		/dev/input/evenX 
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

#define USE_INPUT_DEVICE 1
#define I2C_DF_NOTIFY	0x01		// for i2c 

#define IRQ_LIGHT_INT IRQ_EINT(27)	// s3c64xx ext int 
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

/* for state transition */
struct _light_state {
	int lux_bottom_limit;
	int lux_top_limit;
	int brightness;
};

/* LCD brightness  */
#define STATE_0_BRIGHTNESS	40
#define STATE_1_BRIGHTNESS	90    
#define STATE_2_BRIGHTNESS	140   
#define STATE_3_BRIGHTNESS	190   
#define STATE_4_BRIGHTNESS	255   

/* State threshold levels in LUX  */
#define STATE_0_1_TH	30
#define STATE_1_2_TH	60
#define STATE_2_3_TH	90
#define STATE_3_4_TH	120

#define CUT_GAP	20  

static struct _light_state light_state[] = {
	[0] = {
		 //0~40
		.lux_bottom_limit = 0,
		.lux_top_limit    = STATE_0_1_TH + CUT_GAP/2,
		.brightness	  = STATE_0_BRIGHTNESS,
		},

	[1] = {
		//20~70
		.lux_bottom_limit = STATE_0_1_TH - CUT_GAP/2, 
		.lux_top_limit    = STATE_1_2_TH + CUT_GAP/2, 
		.brightness	  = STATE_1_BRIGHTNESS,
		},

	[2] = {
		//50~100
		.lux_bottom_limit = STATE_1_2_TH - CUT_GAP/2, 
		.lux_top_limit    = STATE_2_3_TH + CUT_GAP/2, 
		.brightness	  = STATE_2_BRIGHTNESS,
		},

	[3] = {
		//80~130
		.lux_bottom_limit = STATE_2_3_TH - CUT_GAP/2, 
		.lux_top_limit    = STATE_3_4_TH + CUT_GAP/2, 
		.brightness	  = STATE_3_BRIGHTNESS,
		},

	[4] = {
		// 110~unlimited
		.lux_bottom_limit = STATE_3_4_TH - CUT_GAP/2, 
		.lux_top_limit    = 0xffff, //unlimited
		.brightness	  = STATE_4_BRIGHTNESS,
		},
};

/* driver data */
struct light_data {
#if USE_INPUT_DEVICE 
	struct input_dev *input_dev;
#endif	
	struct work_struct work_light; 
	int    irq;
	struct hrtimer timer;
};

struct workqueue_struct *light_wq;

/* prototype */
int isl_i2c_read(u8 reg);
int isl_i2c_write( u8 reg, int val );
extern void backlight_level_ctrl(s32 value);
static int isl_attach_adapter(struct i2c_adapter *adap);
#endif
