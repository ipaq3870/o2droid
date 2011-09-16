#ifndef __GP2A_H__
#define __GP2A_H__

#define IRQ_GP2A_INT	IRQ_EINT(20)  /*s3c64xx int number */
#define I2C_DF_NOTIFY	0x01 /* for i2c */
#define GP2A_ADDR	0x88 /* slave addr for i2c */

#define REGS_PROX	0x0 // Read  Only
#define REGS_GAIN	0x1 // Write Only
#define REGS_HYS	0x2 // Write Only
#define REGS_CYCLE	0x3 // Write Only
#define REGS_OPMOD	0x4 // Write Only
#define REGS_CON	0x6 // Write Only

/* power control */
#define ON	1
#define OFF	0

/* IOCTL for proximity sensor */
#define SHARP_GP2AP_IOC_MAGIC		'C'                                 
#define SHARP_GP2AP_IOCTL_GET_ENABLED	_IOR(SHARP_GP2AP_IOC_MAGIC, 0x1, int)            
#define SHARP_GP2AP_IOCTL_ENABLE	_IOW(SHARP_GP2AP_IOC_MAGIC, 0x2, int)            
#define SHARP_GP2AP_PRINT_PROX_VALUE	_IO(SHARP_GP2AP_IOC_MAGIC, 0x3)      

/* input device for proximity sensor */
#define USE_INPUT_DEVICE 	1

/* initial value for sensor register */
static int gp2a_original_image[8] = {
	0x00,  
	0x08,  
	0x40,  	
	0x04,  
	0x03,   
};

/* driver data */
struct gp2a_data {
	struct input_dev *input_dev;
	struct work_struct work_prox;
	int	irq;
	struct mutex            lock;
};


struct workqueue_struct *gp2a_wq;

/* prototype */
int opt_i2c_read(int reg);
int opt_i2c_write( u8 reg, int val );
static int proximity_open(struct inode *ip, struct file *fp);
static int proximity_release(struct inode *ip, struct file *fp);
static long proximity_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#endif
