
/* btgpio/btgpio.c
 *
 * Copyright (c) 2008 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/kernel.h>	/* We're doing kernel work */
#include <linux/module.h>	/* Specifically, a module */
#include <linux/fs.h>
#include <asm/uaccess.h>	/* for get_user and put_user */


#include <linux/version.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/init.h>
#include <mach/gpio.h>	
#include <mach/saturn.h>	/*GPIO Pin mapping is done in this file*/
#include <plat/gpio-cfg.h>	


#include <mach/hardware.h>
#include "btgpio.h"



static int Device_Open = 0;


/* 
 * This is called whenever a process attempts to open the device file 
 */
static int btgpio_open(struct inode *inode, struct file *file)
{
	printk(KERN_DEBUG "device_open(%p)\n", file);

	if (Device_Open)
		return -EBUSY;

	Device_Open++;

	if (try_module_get(THIS_MODULE))
		return SUCCESS;
	else
		return -ENOENT;

}


static int  btgpio_release(struct inode *inode, struct file *file)
{
	
	printk(KERN_DEBUG "device_release(%p,%p)\n", inode, file);
	Device_Open = 0;

	module_put(THIS_MODULE);
	return SUCCESS;
}


int device_poweron ()
{

	if (gpio_is_valid(GPIO_BT_WAKE)) {
		if (gpio_request(GPIO_BT_WAKE, S3C_GPIO_LAVEL(GPIO_BT_WAKE)))
		{
			printk(KERN_ERR "Failed to request GPIO_BT_WAKE!\n");
			goto ERROR_VAL;
		}
		gpio_direction_output(GPIO_BT_WAKE, GPIO_LEVEL_LOW);
	}
	s3c_gpio_setpull(GPIO_BT_WAKE, S3C_GPIO_PULL_NONE);
	

	if (gpio_is_valid(GPIO_BT_RST_N)) {
		if (gpio_request(GPIO_BT_RST_N, S3C_GPIO_LAVEL(GPIO_BT_RST_N)))
		{		
			printk(KERN_ERR "Failed to request GPIO_BT_RST_N!\n");
			goto ERROR_VAL;
		}
		gpio_direction_output(GPIO_BT_RST_N, GPIO_LEVEL_LOW);
	}
	s3c_gpio_setpull(GPIO_BT_RST_N, S3C_GPIO_PULL_NONE);
	
	s3c_gpio_cfgpin(GPIO_BT_HOST_WAKE, S3C_GPIO_SFN(GPIO_BT_HOST_WAKE_AF));
	s3c_gpio_setpull(GPIO_BT_HOST_WAKE, S3C_GPIO_PULL_DOWN);

	//s3c_gpio_cfgpin(GPIO_WLAN_HOST_WAKE, S3C_GPIO_SFN(GPIO_WLAN_HOST_WAKE_AF));
	//s3c_gpio_setpull(GPIO_WLAN_HOST_WAKE, S3C_GPIO_PULL_NONE);

	gpio_set_value(GPIO_BT_RST_N, GPIO_LEVEL_LOW);		

	msleep (100);

	gpio_set_value(GPIO_BT_RST_N, GPIO_LEVEL_HIGH);
	printk (KERN_INFO "BT GPIO Power On Success\n");

	return 0;
	
ERROR_VAL: 
	printk (KERN_ERR "BT GPIO Power On Failure!\n");
	return -1;
	
}

int device_poweroff ()
{

	gpio_set_value(GPIO_BT_RST_N, GPIO_LEVEL_LOW);
		
	gpio_free (GPIO_BT_RST_N);
	gpio_free (GPIO_BT_WAKE);
	return 0;
}

void device_suspendline (int bsuspend)
{

	if(bsuspend)
	{
        // Set BT_WAKE Low
        	gpio_set_value(GPIO_BT_WAKE, GPIO_LEVEL_LOW);
	}
	else
	{
	    // Set BT_WAKE high
        	gpio_set_value(GPIO_BT_WAKE, GPIO_LEVEL_HIGH);
	}
}

int device_readhostline (int *hostval)
{	
	if (gpio_is_valid(GPIO_BT_HOST_WAKE)) {
		if (gpio_request(GPIO_BT_HOST_WAKE, S3C_GPIO_LAVEL(GPIO_BT_HOST_WAKE)))
		{
			printk(KERN_ERR "Failed to request GPIO_BT_HOST_WAKE!\n");
			goto ERROR_VAL;
		}
		gpio_direction_input(GPIO_BT_HOST_WAKE);
	}
	*hostval = gpio_get_value (GPIO_BT_HOST_WAKE);
	
	printk (KERN_DEBUG "The READ_HOST_WAKE value is = %d\n", *hostval);
	gpio_free(GPIO_BT_HOST_WAKE);

	return 0;
ERROR_VAL:
	return -1;

}
int btgpio_ioctl(struct inode *inode,	
		 struct file *file,	
		 unsigned int ioctl_num,	/* number and param for ioctl */
		 unsigned long ioctl_param)
{
	int nretVal;
	int nHostVal;
	/* 
	 * Switch according to the ioctl called 
	 */
	nretVal = SUCCESS;
	switch (ioctl_num) {
	case IOCTL_BTPWRON:
		nretVal = device_poweron ();
		break;

	case IOCTL_BTPWROFF:
		nretVal = device_poweroff ();
		break;
	case IOCTL_BTSUSPENDLINE:
		device_suspendline (1);
		break;
	case IOCTL_BTWAKELINE:
		device_suspendline (0);
		break;
	case IOCTL_BTREADSUSPEND:
	{
		nretVal = device_readhostline (&nHostVal);
		if(copy_to_user (ioctl_param,  &nHostVal, 1))
			printk("%s : IOCTL_BTREADSUSPEND copy to user error\n", __func__);
		break;
	}

	}

	return nretVal;
}



struct file_operations Fops = {
	.read = device_readhostline,
	.ioctl = btgpio_ioctl,
	.open = btgpio_open,
	.release = btgpio_release,	
};

/* 
 * Initialize the module - Register the character device 
 */
static int __init btgpioinit_module()
{
	int ret_val;
	ret_val = register_chrdev(MAJOR_NUM, DEVICE_NAME, &Fops);

	if (ret_val < 0) {
		printk(KERN_ALERT "%s failed with %d\n",
		       "Registering the BT_GPO device ", ret_val);
		return ret_val;
	}
	printk(KERN_DEBUG "%s The major device number is %d.\n", "Registering 'BT GPIO is a success' ", MAJOR_NUM);
	return 0;
}


/* 
 * Cleanup - unregister the appropriate file from /proc 
 */
static void __exit btgpioclean_module()
{
	unregister_chrdev(MAJOR_NUM, DEVICE_NAME);
	printk (KERN_DEBUG "The char device is unregistered\n");
}

module_init(btgpioinit_module);
module_exit(btgpioclean_module);

MODULE_AUTHOR("Sivakumar KK <sivakumar.kk@samsung.com>, Jayachandra M <jaya.chandra@samsung.com>");
MODULE_DESCRIPTION("GPIO enable to poweron and reset the BT chip");
MODULE_LICENSE("GPL");

