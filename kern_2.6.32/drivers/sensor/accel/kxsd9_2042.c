 /*****************************************************************************
 *
 * COPYRIGHT(C) : Samsung Electronics Co.Ltd, 2006-2015 ALL RIGHTS RESERVED
 *
 *****************************************************************************/

#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/earlysuspend.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <asm/uaccess.h>

#include "kxsd9_2042.h"

static struct i2c_client *g_i2c_client;

struct kxsd9_data {
	struct early_suspend	early_suspend;
	struct i2c_client	*client;
};

#define KXSD9_SADDR	0x30 	// [00110000]
#define I2C_DF_NOTIFY	0x01
#define IRQ_ACC_INT 	IRQ_EINT(3)

/*
   * CLKhld = 1 //held low during A/D conversions
   * ENABLE = 1 //normal operation
   * ST = 0 //selftest disable
   * MOTIen = 0 //normal operation
*/
#define REGB_VALUE_NORMAL	0xC0
#define REGB_VALUE_STANDBY	0x80
                              
static int kxsd9_i2c_remove(struct i2c_client *client);
static int kxsd9_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);

static const struct i2c_device_id kxsd9_2042_i2c_id[] = {
	        { "kxsd9_2042", 0}, 
		        { }
};

MODULE_DEVICE_TABLE(i2c, kxsd9_2042_i2c_id);

static struct i2c_driver kxsd9_2042_i2c_driver = {
	.driver = {
		.name = "kxsd9_2042",
		.owner = THIS_MODULE,
	},
	.probe		= kxsd9_i2c_probe,
	.remove		= kxsd9_i2c_remove,
	.id_table	= kxsd9_2042_i2c_id,
};

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
	char buf_write[2];

	// switch to low-power standby now
	buf_write[0] = KXSD9_REG_CTRL_REGB;
	buf_write[1] = REGB_VALUE_STANDBY;
	
	kxsd9_i2c_write(buf_write, 2);	

	gprintk("kxsd9 suspend\n");
}

static void kxsd9_early_resume(struct early_suspend *handler)
{
	char buf_write[2];

	// switch to normal operation now
	buf_write[0] = KXSD9_REG_CTRL_REGB;
	buf_write[1] = REGB_VALUE_NORMAL;
	
	kxsd9_i2c_write(buf_write, 2);	

	gprintk("ksxd9 resume\n");
}
#endif	/* CONFIG_HAS_EARLYSUSPEND */

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

static void kxsd9_read_accel(void)
{
	char buf_read[RBUFF_SIZE+1];
	int ret = 0;
	int x=0, y=0, z=0;
	
	memset(buf_read, 0, RBUFF_SIZE+1);
	buf_read[0] = KXSD9_REG_XOUT_H;

	ret = kxsd9_i2c_read(buf_read, 6);
	if (ret<0)
		printk(KERN_ERR "kxsd9_read_accel: I2C read failed! \n");

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


//	printk("Read value [x=%d, y=%d, z=%d]\n", x, y, z);
	gprintk("Read value [x=%d, y=%d, z=%d]\n", acc_data.x, acc_data.y, acc_data.z);

//	printk("============ END ============\n");
}

/* BMA150 IOCTL */
#define BMA150_IOC_MAGIC 				'B'
#define BMA150_CALIBRATE				_IOW(BMA150_IOC_MAGIC,2, unsigned char)
#define BMA150_SET_RANGE            	_IOWR(BMA150_IOC_MAGIC,4, unsigned char)
#define BMA150_SET_MODE             	_IOWR(BMA150_IOC_MAGIC,6, unsigned char)
#define BMA150_SET_BANDWIDTH            _IOWR(BMA150_IOC_MAGIC,8, unsigned char)
#define BMA150_READ_ACCEL_XYZ           _IOWR(BMA150_IOC_MAGIC,46,short)
#define BMA150_IOC_MAXNR            	48

static int bma150_open(struct inode *inode, struct file *file) {
	return 0;
}

static int bma150_release(struct inode *inode, struct file *file) {
	return 0;
}
static int bma150_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	
	switch(cmd)
	{
		case BMA150_CALIBRATE:
		case BMA150_SET_RANGE:
		case BMA150_SET_BANDWIDTH:
			break;
		case BMA150_SET_MODE:
			break;
		case BMA150_READ_ACCEL_XYZ:
			kxsd9_read_accel();
			copy_to_user((bma020acc_t*)arg, &acc_data, sizeof(acc_data));
			break;
		default:
			break;
	}
	return ret;
}


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


static int kxsd9_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct kxsd9_data *kxsd9;
	int ret = 0;

	gprintk("start\n");
	
	kxsd9 = kzalloc(sizeof(struct kxsd9_data), GFP_KERNEL);
	if (!kxsd9) {
		ret = -ENOMEM;
		goto exit;
	}

	kxsd9->client = client;
	i2c_set_clientdata(client, kxsd9);

	g_i2c_client = client;
	if (g_i2c_client  == NULL)	{
		printk(KERN_ERR "i2c_client is NULL\n");
		return -ENODEV;
	}

	ret = misc_register(&bma150_device);
	if(ret) {
		gprintk("kxsd9 misc device register failed\n");
		goto exit;
	}

	kxsd9_init_device();

#ifdef CONFIG_HAS_EARLYSUSPEND
	kxsd9->early_suspend.suspend = kxsd9_early_suspend;
	kxsd9->early_suspend.resume = kxsd9_early_resume;

	register_early_suspend(&kxsd9->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

exit:
	return ret;
}

static int kxsd9_i2c_remove(struct i2c_client *client)
{
	struct kxsd9_data *kxsd9 = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&kxsd9->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */
	misc_deregister(&bma150_device);
	kfree(kxsd9);
	g_i2c_client = NULL;
	
	return 0;
}

static int __init kxsd9_2042_init(void)
{
	gprintk("init\n");
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

