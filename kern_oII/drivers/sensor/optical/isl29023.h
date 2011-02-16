#ifndef __LIGHT_H__
#define __LIGHT_H__

#define I2C_DF_NOTIFY	0x01		//for i2c 

#define IRQ_LIGHT_INT IRQ_EINT(27)	//s3c64xx ext int 
#define LIGHT_ADDR	0x88		//i2c slave addr

#define REGS_COMMAND_I		0x0 
#define REGS_COMMAND_II		0x1
#define REGS_LBS_SENSOR		0x2
#define REGS_MBS_SENSOR		0x3
#define REGS_INT_LSB_TH_LO	0x4
#define REGS_INT_MSB_TH_LO	0x5
#define REGS_INT_LSB_TH_HI	0x6
#define REGS_INT_MSB_TH_HI	0x7

// power control 
#define ON              1
#define OFF		0

// WARNING the interrut operation is not implemented fully yet !!!
#define INT_OP_MODE	0	// 0 = normal operation, 1 = INT operation 
#define LIGHT_PERIOD	1	// secs 

/*for light sensor */
#define STATE_NUM		3   /* number of states */
#define STATE_0_BRIGHTNESS	255   /* brightness of lcd */
#define STATE_1_BRIGHTNESS	130    
#define STATE_2_BRIGHTNESS	40

#define ADC_CUT_HIGH		800            /* boundary line between STATE_0 and STATE_1 */
#define ADC_CUT_LOW		300            /* boundary line between STATE_1 and STATE_2 */
#define ADC_CUT_GAP		200            /* in order to prevent chattering condition */

/* state type */
typedef enum t_light_state
{
	STATE_0   = 0,
	STATE_1   = 1,
	STATE_2   = 2,
	

} state_type;

/* for state transition */
struct _light_state {
	state_type type;
	int adc_bottom_limit;
	int adc_top_limit;
	int brightness;

};

static struct _light_state light_state[] = {
	[0] = {
		.type = STATE_0, // 700~20000
		.adc_bottom_limit = ADC_CUT_HIGH - ADC_CUT_GAP/2, 
		.adc_top_limit    = 20000, //unlimited
		.brightness		  = STATE_0_BRIGHTNESS,
		},
	[1] = {
		.type = STATE_1, //200~900
		.adc_bottom_limit = ADC_CUT_LOW  - ADC_CUT_GAP/2, 
		.adc_top_limit    = ADC_CUT_HIGH + ADC_CUT_GAP/2, 
		.brightness		  = STATE_1_BRIGHTNESS,
		},
	
	[2] = {
		.type = STATE_2,   //1~400
		.adc_bottom_limit = 1,
		.adc_top_limit    = ADC_CUT_LOW  + ADC_CUT_GAP/2,
		.brightness		  = STATE_2_BRIGHTNESS,
		},

};

/* driver data */
struct light_data {
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
