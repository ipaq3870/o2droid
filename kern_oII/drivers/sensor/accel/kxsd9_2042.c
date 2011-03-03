 /*****************************************************************************
 *
 * COPYRIGHT(C) : Samsung Electronics Co.Ltd, 2006-2015 ALL RIGHTS RESERVED
 *
 *****************************************************************************/

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/earlysuspend.h>

#include <mach/hardware.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-gpio.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>

#include "kxsd9_2042.h"

static struct i2c_client *g_i2c_client;

struct kxsd9_data {
	struct input_dev *input_dev;
	struct work_struct work;
	struct hrtimer timer;
	struct early_suspend	early_suspend;
};

#define KXSD9_SADDR	0x18 	// [0011000]
#define I2C_DF_NOTIFY	0x01
#define IRQ_ACC_INT 	IRQ_EINT(3)

#define TIMER_OFF 	0
#define TIMER_ON	1

/*
   * CLKhld = 1 //held low during A/D conversions
   * ENABLE = 1 //normal operation
   * ST = 0 //selftest disable
   * MOTIen = 0 //normal operation
*/
#define REGB_VALUE_NORMAL	0xC0
#define REGB_VALUE_STANDBY	0x80
                              
static int kxsd9_timer_oper = TIMER_OFF;

static unsigned short ignore[] = { I2C_CLIENT_END };
static unsigned short normal_addr[] = { I2C_CLIENT_END };
static unsigned short probe_addr[] = { 0, KXSD9_SADDR, I2C_CLIENT_END };

static struct i2c_client_address_data addr_data = {
	.normal_i2c     = normal_addr,
	.probe          = probe_addr,
	.ignore         = ignore,
};

static int kxsd9_i2c_attach_adapter(struct i2c_adapter *adapter);
static int kxsd9_i2c_detach_adapter(struct i2c_client *client);
static int kxsd9_i2c_probe_found(struct i2c_adapter *adapter, int, int);

static struct i2c_driver kxsd9_2042_i2c_driver = {
	.attach_adapter = kxsd9_i2c_attach_adapter,
	.detach_client  = kxsd9_i2c_detach_adapter,
	.driver = 
	{
		.name = "kxsd9_2042",
	},
};

#if 0
static irqreturn_t kxsd9_interrupt_handler(int irq, void *dev_id)
{
	gprintk("start\n");

	//TODO: interrupt (XEINT3)
	return IRQ_HANDLED;
}
#endif

static int kxsd9_i2c_write(char *buf_to_write, int length)
{
	struct i2c_msg msg[] = {
		{
			 .addr = g_i2c_client->addr,
			 .flags = 0,
			 .len = length,
			 .buf = buf_to_write,
		 },
	};

	if (i2c_transfer(g_i2c_client->adapter, msg, 1) < 0) {
		printk(KERN_ERR "kxsd9_i2c_write() failed!\n");
		return -EIO;
	} else
		return 0;
}

static int kxsd9_i2c_read(char *buf_to_read, int length)
{
	struct i2c_msg messages[] = {
		{
			 .addr = g_i2c_client->addr,
			 .flags = 0,
			 .len = 1,
			 .buf = buf_to_read,
		},
		{
			 .addr = g_i2c_client->addr,
			 .flags = I2C_M_RD,
			 .len = length,
			 .buf = buf_to_read,
		},
	};

	if (i2c_transfer(g_i2c_client->adapter, messages, 2) < 0) {
		printk(KERN_ERR "kxsd9_i2c_read() failed!\n");
		return -EIO;
	} else
		return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void kxsd9_early_suspend(struct early_suspend *handler)
{
	struct kxsd9_data *data = container_of(handler, struct kxsd9_data, early_suspend);
	char buf_write[2];

	hrtimer_cancel(&data->timer);
	// switch to low-power standby now
	buf_write[0] = KXSD9_REG_CTRL_REGB;
	buf_write[1] = REGB_VALUE_STANDBY;
	
	kxsd9_i2c_write(buf_write, 2);	

	gprintk("kxsd9 suspend\n");
}

static void kxsd9_early_resume(struct early_suspend *handler)
{
	struct kxsd9_data *data = container_of(handler, struct kxsd9_data, early_suspend);
	char buf_write[2];

	// switch to normal operation now
	buf_write[0] = KXSD9_REG_CTRL_REGB;
	buf_write[1] = REGB_VALUE_NORMAL;
	
	kxsd9_i2c_write(buf_write, 2);	

	if(kxsd9_timer_oper == TIMER_ON)
		hrtimer_start(&data->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	gprintk("ksxd9 resume\n");
}
#endif	/* CONFIG_HAS_EARLYSUSPEND */

/* BMA150 IOCTL */
#define BMA150_IOC_MAGIC 				'B'
#define BMA150_CALIBRATE				_IOW(BMA150_IOC_MAGIC,2, unsigned char)
#define BMA150_SET_RANGE            	_IOWR(BMA150_IOC_MAGIC,4, unsigned char)
#define BMA150_SET_MODE             	_IOWR(BMA150_IOC_MAGIC,6, unsigned char)
#define BMA150_SET_BANDWIDTH            _IOWR(BMA150_IOC_MAGIC,8, unsigned char)
#define BMA150_READ_ACCEL_XYZ           _IOWR(BMA150_IOC_MAGIC,46,short)
#define BMA150_IOC_MAXNR            	48

static int bma150_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int bma150_release(struct inode *inode, struct file *file)
{
	return 0;
}
static int bma150_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	unsigned char mode;
	struct kxsd9_data *kxsd9 = i2c_get_clientdata(g_i2c_client);
	
	switch(cmd)
	{
		case BMA150_CALIBRATE:
		case BMA150_SET_RANGE:
		case BMA150_SET_BANDWIDTH:
			break;
		case BMA150_SET_MODE:
			copy_from_user(&mode,(unsigned char*)arg,1);
			if (mode != 2) {	
				gprintk("hrtimer_start!\n");
				kxsd9_timer_oper = TIMER_ON;
				hrtimer_start(&kxsd9->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
			}
			else {
				gprintk("hrtimer_cancel!\n");
				kxsd9_timer_oper = TIMER_OFF;
				hrtimer_cancel(&kxsd9->timer);
			}
			break;
		case BMA150_READ_ACCEL_XYZ:
			copy_to_user((bma020acc_t*)arg, &acc_data, sizeof(acc_data));
			break;
		default:
			break;
	}
	return ret;
}


#ifdef KXSD9_TESTMODE
static int kxsd9_get_valid_value(char* p)
{
	int ret=0;
	char buf[2]={0,0};

	memcpy(buf, p, 2);

	gprintk("[DBG] buf[0]=%d, buf[1]=%d\n", buf[0], buf[1]);


#if 1	//12bit
	ret = ((int)buf[0]<<4) ^ ((int)buf[1]>>4);
#else	//8bit
	ret = (int)buf[0];
#endif
	return ret;
}

static void kxsd9_workqueue_func(struct work_struct *work)
{
	char buf_read[RBUFF_SIZE+1];
	int ret = 0;
	int x=0, y=0, z=0;
	struct kxsd9_data *kx_data = i2c_get_clientdata(g_i2c_client);
	
	gprintk("has been activated by kxsd9_timer_func()\n");

	memset(buf_read, 0, RBUFF_SIZE+1);
	buf_read[0] = KXSD9_REG_XOUT_H;

	ret = kxsd9_i2c_read(buf_read, 6);
	if (ret<0)
		printk(KERN_ERR "kxsd9_workqueue_func: I2C read failed! \n");

	gprintk("\n");
	gprintk("[DBG] buf_read[0]=%d, buf_read[1]=%d\n", buf_read[0], buf_read[1]);
	gprintk("[DBG] buf_read[2]=%d, buf_read[3]=%d\n", buf_read[2], buf_read[3]);
	gprintk("[DBG] buf_read[4]=%d, buf_read[5]=%d\n\n", buf_read[4], buf_read[5]);
	
	x = kxsd9_get_valid_value(&buf_read[0]);
	y = kxsd9_get_valid_value(&buf_read[2]);
	z = kxsd9_get_valid_value(&buf_read[4]);

	acc_data.x = (x - 2080) / -3;
	acc_data.y = (y - 2080) / -3;
	acc_data.z = (z - 2080) / -3;

	gprintk("\ninput_report_abs\n");

	input_report_abs(kx_data->input_dev, ABS_X, x);
	input_report_abs(kx_data->input_dev, ABS_Y, y);
	input_report_abs(kx_data->input_dev, ABS_Z, z);
	input_sync(kx_data->input_dev);

//	printk("Read value [x=%d, y=%d, z=%d]\n", x, y, z);
	gprintk("Read value [x=%d, y=%d, z=%d]\n", acc_data.x, acc_data.y, acc_data.z);

//	printk("============ END ============\n");
}

static enum hrtimer_restart kxsd9_timer_func(struct hrtimer *timer)
{
	struct kxsd9_data* kxsd9 = container_of(timer, struct kxsd9_data, timer);

//	printk("============START============\n");

	// puts a job in the kernel-global workqueue
	schedule_work(&kxsd9->work);
	
	hrtimer_start(&kxsd9->timer, ktime_set(0, KXSD9_TESTMODE_PERIOD), HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}
#endif

static int kxsd9_init_device(void)
{
	int ret = 0;
	char buf_write[2];
	
	gprintk("start!\n");
	
	/*
	 * CLKhld = 1 //held low during A/D conversions
	 * ENABLE = 1 //normal operation
	 * ST = 0 //selftest disable
	 * MOTIen = 0 //normal operation
	 */
	buf_write[0] = KXSD9_REG_CTRL_REGB;
	buf_write[1] = REGB_VALUE_NORMAL;
	
	ret = kxsd9_i2c_write(buf_write, 2);	
	if (ret<0) {
		printk(KERN_ERR "I2C write failed! \n");
		return ret;
	}
	gprintk("kxsd9 REGB Setting!\n");

	/*
	 * Filter Coner Frequency : 50Hz
	 * MOTLev : 0
	 * MOTLat : 0
	 * FS1,FS0 : 1,1
	 */
	buf_write[0] = KXSD9_REG_CTRL_REGC;
	buf_write[1] = 0xE3;	//11100011
	
	ret = kxsd9_i2c_write(buf_write, 2);	
	if (ret<0) {
		printk(KERN_ERR "I2C write failed! \n");
		return ret;
	}
	gprintk("kxsd9 REGC Setting!\n");
	
	buf_write[0] = KXSD9_REG_RESET_WRITE;
	buf_write[1] = KSXD9_VAL_RESET;  //0xCA(11001010)
	
	ret = kxsd9_i2c_write(buf_write, 2);	
	if (ret<0) {
		printk(KERN_ERR "I2C write failed! \n");
		return ret;
	}
	mdelay(16);  // 16msec delay
	
	gprintk("kxsd9 module RESET!\n");

	return ret;
}


static struct file_operations bma150_fops = {
	.owner = THIS_MODULE,
	.open = bma150_open,
	.ioctl = bma150_ioctl,
	.release = bma150_release,
};

static struct miscdevice bma150_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "bma150",
	.fops = &bma150_fops,
};


static int kxsd9_i2c_probe_found(struct i2c_adapter *adapter, int address, int kind)
{
	struct i2c_client *new_client;
	struct kxsd9_data *kxsd9;

	int ret = 0;

	gprintk("start\n");
	
	kxsd9 = kzalloc(sizeof(struct kxsd9_data), GFP_KERNEL);
	if (!kxsd9) {
		ret = -ENOMEM;
		goto exit;
	}

	//TOCHK: I2C_FUNC_SMBUS_I2C_BLOCK OK? (or I2C_FUNC_SMBUS_BYTE_DATA?)
	ret = i2c_check_functionality(adapter, I2C_FUNC_SMBUS_I2C_BLOCK);
	if (!ret)
		goto exit;

	new_client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (!new_client) 
	{
		ret = -ENOMEM;
		goto exit;
	}

	//TOCHK: Is this flag setting Right?
	new_client->flags = I2C_DF_NOTIFY|I2C_M_IGNORE_NAK;
	new_client->addr = address;
	strlcpy(new_client->name, "kxsd9_2042", I2C_NAME_SIZE);
	new_client->adapter = adapter;
	new_client->driver = &kxsd9_2042_i2c_driver;
	#if 0  //TODO: interrupt (XEINT3)
	new_client->irq = IRQ_ACC_INT;  //TODO: interrupt (XEINT3)
	#endif
	
	if ((ret = i2c_attach_client(new_client)))
		goto exit;


#ifdef KXSD9_TESTMODE
	// initialize variables of workqueue structure
	INIT_WORK(&kxsd9->work, kxsd9_workqueue_func);
#endif

	i2c_set_clientdata(new_client, kxsd9);

	g_i2c_client = new_client;
	if (g_i2c_client  == NULL)	{
		printk(KERN_ERR "i2c_client is NULL\n");
		return -ENODEV;
	}

	kxsd9->input_dev = input_allocate_device();
	
	if(!kxsd9->input_dev){
		gprintk("input_allocate_device failed!\n");
		return -ENODEV;
	}

	set_bit(EV_ABS, kxsd9->input_dev->evbit);
	set_bit(EV_SYN, kxsd9->input_dev->evbit);

	input_set_abs_params(kxsd9->input_dev, ABS_X, 0, 4095, 0, 0);
	input_set_abs_params(kxsd9->input_dev, ABS_Y, 0, 4095, 0, 0);
	input_set_abs_params(kxsd9->input_dev, ABS_Z, 0, 4095, 0, 0);

	kxsd9->input_dev->name = "kxsd9";

	ret = input_register_device(kxsd9->input_dev);
	if(ret)
	{
		gprintk("Unable to register input device: %s\n", kxsd9->input_dev->name);
		goto exit;
	}

	ret = misc_register(&bma150_device);
	if(ret)
	{
		gprintk("kxsd9 misc device register failed\n");
		goto exit;
	}

	kxsd9_init_device();

#if 0  //TODO: interrupt (XEINT3)
	ret = request_irq(new_client->irq, kxsd9_interrupt_handler, IRQF_TRIGGER_HIGH, "kxsd9_2042", kxsd9);
	if (ret < 0) {
		gprintk("request() irq failed!\n");
		goto exit;
	}
#endif

#ifdef KXSD9_TESTMODE
	gprintk("timer init\n");

	hrtimer_init(&kxsd9->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	kxsd9->timer.function = kxsd9_timer_func;
	//hrtimer_start(&kxsd9->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	//kxsd9_timer_oper = TIMER_ON;
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	kxsd9->early_suspend.suspend = kxsd9_early_suspend;
	kxsd9->early_suspend.resume = kxsd9_early_resume;

	register_early_suspend(&kxsd9->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

exit:
	return ret;
}

static int kxsd9_i2c_attach_adapter(struct i2c_adapter *adapter)
{	
	return i2c_probe(adapter, &addr_data, &kxsd9_i2c_probe_found);
}

static int kxsd9_i2c_detach_adapter(struct i2c_client *client)
{
	struct kxsd9_data *kxsd9 = i2c_get_clientdata(client);
	int ret;

	//free_irq(client->irq, kxsd9);  //TODO: interrupt
#ifdef KXSD9_TESTMODE
	gprintk("timer_cancel\n");
	hrtimer_cancel(&kxsd9->timer);
#endif
	input_unregister_device(kxsd9->input_dev);

	if ( (ret = i2c_detach_client(client)) )
		return ret;
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&kxsd9->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */
	misc_deregister(&bma150_device);
	kfree(kxsd9);
	kfree(client);  //TOCHK
	g_i2c_client = NULL;
	kxsd9 = NULL;
	
	return 0;
}

static int __init kxsd9_2042_init(void)
{
	gprintk("init\n");

	// for Interrupt GPIO setting
#if 0
	s3c_gpio_cfgpin(GPIO_ACC_INT, S3C_GPIO_SFN(GPIO_ACC_INT_AF));
	s3c_gpio_setpull(GPIO_ACC_INT, S3C_GPIO_PULL_NONE);

	set_irq_type(IRQ_ACC_INT, IRQ_TYPE_EDGE_RISING);
#endif
	return i2c_add_driver(&kxsd9_2042_i2c_driver);
}

static void __exit kxsd9_2042_exit(void)
{
	gprintk("exit\n");
	i2c_del_driver(&kxsd9_2042_i2c_driver);
}

module_init(kxsd9_2042_init);
module_exit(kxsd9_2042_exit);

MODULE_AUTHOR("SAMSUNG");
MODULE_DESCRIPTION("KXSD9-2042 accelerometer driver");
MODULE_LICENSE("GPL");

