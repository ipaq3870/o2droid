/* 
 *  Title : Proximity sensor driver for GP2AP002S00F   
 *  Date  : 2010.02.20
 *  Name  : Sandor Bognar
 *
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <mach/hardware.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-gpio.h>

#include <linux/input.h>
#include <linux/workqueue.h>

#include "gp2aP002S00F.h"

/*********** for debug **********************************************************/
#if 0 
#define gprintk(fmt, x... ) printk( "%s(%d): " fmt, __FUNCTION__ ,__LINE__, ## x)
#else
#define gprintk(x...) do { } while (0)
#endif
/*******************************************************************************/

/* global var */
static struct i2c_client *opt_i2c_client = NULL;

static bool proximity_enable = OFF;

static short proximity_value = 0;

static struct i2c_driver opt_i2c_driver = {
	.driver = {
		.name = "gp2a",
	},
	.attach_adapter = &opt_attach_adapter,
};

static struct file_operations proximity_fops = {
	.owner  = THIS_MODULE,
	.open   = proximity_open,
    .release = proximity_release,
    .unlocked_ioctl = proximity_ioctl,
};
                 
static struct miscdevice proximity_device = {
    .minor  = MISC_DYNAMIC_MINOR,
    .name   = "proximity",
    .fops   = &proximity_fops,
};

static unsigned short opt_normal_i2c[] = {(GP2A_ADDR>>1),I2C_CLIENT_END};
static unsigned short opt_ignore[] = {0, (GP2A_ADDR>>1),I2C_CLIENT_END};
static unsigned short opt_probe[] = {I2C_CLIENT_END};

static struct i2c_client_address_data opt_addr_data = {
	.normal_i2c = opt_normal_i2c,
	.ignore		= opt_ignore,
	.probe		= opt_probe,	
};

short gp2a_get_proximity_value(void) {
	  return proximity_value;
}
EXPORT_SYMBOL(gp2a_get_proximity_value);

/*****************************************************************************************
 *  
 *  function    : gp2a_work_func_prox 
 *  description : This function is for proximity sensor (using B-1 Mode ). 
 *                when INT signal is occured , it gets value from VO register.   
 */
void gp2a_work_func_prox(struct work_struct *work) {
	struct gp2a_data *gp2a = container_of(work, struct gp2a_data, work_prox);
	
	unsigned char value;
	unsigned char vout = 0;

	/* Read VO & INT Clear */
	value = opt_i2c_read(REGS_PROX);
	vout = value & 0x01;
	gprintk("value = %x, vout = %d\n",value, vout);

	/* Report proximity information */
	proximity_value = vout;

#if USE_INPUT_DEVICE 
    		input_report_abs(gp2a->input_dev,ABS_DISTANCE,(int)vout);
		input_sync(gp2a->input_dev);
		mdelay(1);
#endif
	
	/* Write HYS Register */
	if(!vout) 
		value = 0x40;
	else 
		value = 0x20;
	opt_i2c_write((u8)(REGS_HYS), value);

	/* Forcing vout terminal to go high */
	opt_i2c_write((u8)(REGS_CON), 0x18);

	/* enable INT */
	enable_irq(gp2a->irq);

	/* enabling VOUT terminal in nomal operation */
	opt_i2c_write((u8)(REGS_CON), 0x00);
}

irqreturn_t gp2a_irq_handler(int irq, void *dev_id) {
	struct gp2a_data *gp2a = dev_id;

	gprintk("gp2a->irq = %d value = %d\n",gp2a->irq, gpio_get_value(GPIO_PS_VOUT));

	if(gp2a->irq !=-1) {
		mutex_lock(&gp2a->lock);
		disable_irq_nosync(gp2a->irq);
		queue_work(gp2a_wq, &gp2a->work_prox);
		mutex_unlock(&gp2a->lock);
	}

	return IRQ_HANDLED;
}


static int opt_attach(struct i2c_adapter *adap, int addr, int kind) {
	struct i2c_client *c;
	int ret,err=0;
	
	gprintk("\n");
	if ( !i2c_check_functionality(adap,I2C_FUNC_SMBUS_BYTE_DATA) ) {
		printk(KERN_INFO "byte op is not permited.\n");
		return err;
	}
	c = kzalloc(sizeof(struct i2c_client),GFP_KERNEL);
	if (!c) {
		printk("kzalloc error \n");
		return -ENOMEM;
	}

	memset(c, 0, sizeof(struct i2c_client));	
	strncpy(c->name,"gp2a",I2C_NAME_SIZE);
	c->addr = addr;
	c->adapter = adap;
	c->driver = &opt_i2c_driver;
	c->flags = I2C_DF_NOTIFY | I2C_M_IGNORE_NAK;

	if ((ret = i2c_attach_client(c)) < 0) {
		printk("i2c_attach_client error\n");
		goto error;
	}
	opt_i2c_client = c;

	gprintk("\n");
	return ret;

error:
	printk("in %s , ret = %d \n",__func__,ret);
	kfree(c);
	return err;
}

static int opt_attach_adapter(struct i2c_adapter *adap) {
	int ret;
	gprintk("\n");
	ret =  i2c_probe(adap, &opt_addr_data, opt_attach);
	return ret;
}

static int opt_i2c_init(void) {
	if( i2c_add_driver(&opt_i2c_driver)) {
		printk("i2c_add_driver failed \n");
		return -ENODEV;
	}
	return 0;
}

int opt_i2c_read(int reg) {
	int val;
#if 0	
	val = i2c_smbus_read_byte_data(opt_i2c_client, reg);	
	if (val < 0)
		printk("%s %d i2c transfer error\n", __func__, __LINE__);
	return val;
#else	
	int err;
	u8 buf[2];
	struct i2c_msg msg[2];

	buf[0] = reg; 
	msg[0].addr = opt_i2c_client->addr;
	msg[0].flags = 1;
	msg[0].len = 2;
	msg[0].buf = buf;
	err = i2c_transfer(opt_i2c_client->adapter, msg, 1);
	
	val = buf[1];
	
	if (err >= 0) return val;

	printk("%s %d i2c transfer error\n", __func__, __LINE__);
	return err;
#endif
}

int opt_i2c_write(u8 reg, int val) {
	int err;

	if( (opt_i2c_client == NULL) || (!opt_i2c_client->adapter) ){
		return -ENODEV;
	}

	err = i2c_smbus_write_byte_data(opt_i2c_client, reg, val);
	if (err >= 0) return 0;

	printk("%s %d i2c transfer error\n", __func__, __LINE__);
	return err;
}

void gp2a_chip_init(void) {
	gprintk("\n");
	
	/* set INT 	*/
	s3c_gpio_cfgpin(GPIO_PS_VOUT, S3C_GPIO_SFN(GPIO_PS_VOUT_AF));
	s3c_gpio_setpull(GPIO_PS_VOUT, S3C_GPIO_PULL_UP);

	set_irq_type(IRQ_GP2A_INT, IRQ_TYPE_EDGE_FALLING);
}

void gp2a_on(struct gp2a_data *gp2a) {
	int i;
	if(proximity_enable == OFF) {
		gprintk("gp2a power on\n");
		opt_i2c_write((u8)(REGS_CON),  0x18);

		gprintk("enable irq for proximity\n");
		enable_irq(gp2a ->irq);
		msleep(2);		
		/* GP2A Regs INIT SETTINGS */
		for(i = 1;i <= 4;i++) 
			opt_i2c_write((u8)(i), gp2a_original_image[i]);

		opt_i2c_write((u8)(REGS_CON),  0x0);

		opt_i2c_write((u8)(REGS_OPMOD),  0x3);

		proximity_enable = ON;
	}
}

void gp2a_off(struct gp2a_data *gp2a) {
	gprintk("gp2a_off\n");
	if(proximity_enable == ON) {
		gprintk("disable irq for proximity \n");
		disable_irq_nosync(gp2a ->irq);
		opt_i2c_write((u8)(REGS_OPMOD), 0x2);
		proximity_enable = OFF;
	}
}

static int gp2a_opt_probe( struct platform_device* pdev ) {
	struct gp2a_data *gp2a;
	int irq;
	int i;
	int ret;

	/* allocate driver_data */
	gp2a = kzalloc(sizeof(struct gp2a_data),GFP_KERNEL);
	if(!gp2a) {
		pr_err("kzalloc error\n");
		return -ENOMEM;
	}

	gprintk("in %s \n",__func__);
	
	/* init i2c */
	opt_i2c_init();

	if(opt_i2c_client == NULL) {
		pr_err("opt_probe failed : i2c_client is NULL\n"); 
		return -ENODEV;
	}

#if USE_INPUT_DEVICE	
	/* Input device Settings */
	gp2a->input_dev = input_allocate_device();
	if (gp2a->input_dev == NULL) {
		pr_err("Failed to allocate input device\n");
		return -ENOMEM;
	}
	gp2a->input_dev->name = "proximity";

	set_bit(EV_SYN,gp2a->input_dev->evbit);
	set_bit(EV_ABS,gp2a->input_dev->evbit);
	
	input_set_abs_params(gp2a->input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	ret = input_register_device(gp2a->input_dev);
	if (ret) {
		pr_err("Unable to register %s input device\n", gp2a->input_dev->name);
		input_free_device(gp2a->input_dev);
		kfree(gp2a);
		return -1;
	}
#endif

	/* WORK QUEUE Settings */
	gp2a_wq = create_singlethread_workqueue("gp2a_wq");
	if (!gp2a_wq)
		return -ENOMEM;
	INIT_WORK(&gp2a->work_prox, gp2a_work_func_prox);
	mutex_init(&gp2a->lock);
	gprintk("Workqueue Settings complete\n");

	/* misc device Settings */
	ret = misc_register(&proximity_device);
	if(ret) {
		pr_err(KERN_ERR "misc_register failed \n");
	}

	/* INT Settings */	
	irq = IRQ_GP2A_INT;
	gp2a->irq = -1;
	ret = request_irq(irq, gp2a_irq_handler, 0, "gp2a_int", gp2a);
	if (ret) {
		pr_err("unable to request irq %d\n", irq);
		return ret;
	}       
	gp2a->irq = irq;

	gprintk("INT Settings complete\n");

	/* set platdata */
	platform_set_drvdata(pdev, gp2a);
	i2c_set_clientdata(opt_i2c_client, gp2a);	
	/* GP2A Regs INIT SETTINGS */
	for(i = 1;i < 5;i++) {
		opt_i2c_write((u8)(i), gp2a_original_image[i]);
	}

	mdelay(2);

	/* maintain power-down mode before using sensor */
	proximity_enable = ON;
//	gp2a_off(gp2a);
		
	return 0;
}

static int gp2a_opt_suspend( struct platform_device* pdev, pm_message_t state ) {
	struct gp2a_data *gp2a = platform_get_drvdata(pdev);

	if(proximity_enable) {
		disable_irq_nosync(gp2a ->irq);
		opt_i2c_write((u8)(REGS_OPMOD), 0x02);
	}
	return 0;
}

static int gp2a_opt_resume( struct platform_device* pdev ) {
	struct gp2a_data *gp2a = platform_get_drvdata(pdev);
	int i;

	if(proximity_enable) {
		opt_i2c_write((u8)(REGS_CON),  0x18);

		gprintk("enable irq for proximity\n");
		enable_irq(gp2a ->irq);
		msleep(2);		
		/* GP2A Regs INIT SETTINGS */
		for(i = 1;i <= 4;i++) 
			opt_i2c_write((u8)(i), gp2a_original_image[i]);

		opt_i2c_write((u8)(REGS_CON),  0x0);

		opt_i2c_write((u8)(REGS_OPMOD),  0x3);
	}
	return 0;
}

static int proximity_open(struct inode *ip, struct file *fp) {
	return 0;
}

static int proximity_release(struct inode *ip, struct file *fp) {
	return 0;
}

static long proximity_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
	struct gp2a_data *gp2a = i2c_get_clientdata(opt_i2c_client);
	int ret=0;
	switch(cmd) {
		case SHARP_GP2AP_OPEN:
			{
				printk(KERN_INFO "[PROXIMITY] %s : case OPEN\n", __FUNCTION__);
				gp2a_on(gp2a);
				proximity_enable = ON;
			}
			break;

		case SHARP_GP2AP_CLOSE:
			{
				printk(KERN_INFO "[PROXIMITY] %s : case CLOSE\n", __FUNCTION__);
				gp2a_off(gp2a);
				proximity_enable = OFF;
			}
			break;
		case BSS_PRINT_PROX_VALUE:
			{
				printk(KERN_INFO "[PROXIMITY] %s, value: %d \n", __FUNCTION__, proximity_value);
			}
			break;
		default:
			printk(KERN_INFO "[PROXIMITY] unknown ioctl %d\n", cmd);
			ret = -1;
			break;
	}
	return ret;
}

static struct platform_driver gp2a_opt_driver = {
	.probe 	 = gp2a_opt_probe,
	.suspend = gp2a_opt_suspend,
	.resume  = gp2a_opt_resume,
	.driver  = {
		.name = "gp2a-opt",
		.owner = THIS_MODULE,
	},
};

static int __init gp2a_opt_init(void) {
	int ret;
	
	gp2a_chip_init();
	ret = platform_driver_register(&gp2a_opt_driver);
	return ret;
}

static void __exit gp2a_opt_exit(void) {
	struct gp2a_data *gp2a = i2c_get_clientdata(opt_i2c_client);
	if (gp2a_wq)
		destroy_workqueue(gp2a_wq);

	free_irq(IRQ_GP2A_INT,gp2a);
	
#if USE_INPUT_DEVICE
	input_unregister_device(gp2a->input_dev);
#endif
	kfree(gp2a);

	platform_driver_unregister(&gp2a_opt_driver);
}


module_init( gp2a_opt_init );
module_exit( gp2a_opt_exit );

MODULE_AUTHOR("SAMSUNG");
MODULE_DESCRIPTION("Optical Sensor driver for gp2ap002s00f");
MODULE_LICENSE("GPL");
