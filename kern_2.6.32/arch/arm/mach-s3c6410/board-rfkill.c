/*
 * Copyright (C) 2008 Google, Inc.
 * Author: Nick Pelly <npelly@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/* Control bluetooth power for saturn platform */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/rfkill.h>
#include <linux/delay.h>
#include <asm/gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/egpio.h>
#include <linux/wakelock.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>

#include <mach/hardware.h>
#include <linux/i2c/pmic.h>

#undef BT_SLEEP_ENABLER

#define IRQ_BT_HOST_WAKE      IRQ_EINT(22)

static struct wake_lock rfkill_wake_lock;
#ifdef BT_SLEEP_ENABLER
static struct wake_lock bt_wake_lock;
#endif

void rfkill_switch_all(enum rfkill_type type, enum rfkill_user_states state);

extern void s3c_setup_uart_cfg_gpio(unsigned char port);
extern void s3c_reset_uart_cfg_gpio(unsigned char port);

static struct rfkill *bt_rfk;
static const char bt_name[] = "CSR-bc06";

static int bluetooth_set_power(void *data, enum rfkill_user_states state)
{
	int ret = 0; 
	switch (state) {
		
	case RFKILL_USER_STATE_UNBLOCKED:
		printk("[BT] Device Powering ON \n");
		s3c_setup_uart_cfg_gpio(1);

		if (gpio_is_valid(GPIO_BT_EN))
		{
			ret = gpio_request(GPIO_BT_EN, S3C_GPIO_LAVEL(GPIO_BT_EN));
			if (ret < 0) {
					printk("[BT] Failed to request GPIO_BT_EN!\n");
				return ret;
			}
			gpio_direction_output(GPIO_BT_EN, GPIO_LEVEL_HIGH);
		}
				
		if (gpio_is_valid(GPIO_BT_nRST))
		{
			ret = gpio_request(GPIO_BT_nRST, S3C_GPIO_LAVEL(GPIO_BT_nRST));
			if (ret < 0) {
				gpio_free(GPIO_BT_EN);
					printk("[BT] Failed to request GPIO_BT_nRST!\n");
				return ret;			
			}
			gpio_direction_output(GPIO_BT_nRST, GPIO_LEVEL_LOW);
		}
		
		/* Set GPIO_BT_EN high */ 
		s3c_gpio_setpull(GPIO_BT_EN, S3C_GPIO_PULL_NONE);
		gpio_set_value(GPIO_BT_EN, GPIO_LEVEL_HIGH);
		
		s3c_gpio_slp_cfgpin(GPIO_BT_EN, S3C_GPIO_SLP_OUT1);  
		s3c_gpio_slp_setpull_updown(GPIO_BT_EN, S3C_GPIO_PULL_NONE);
		
		printk("[BT] GPIO_BT_EN = %d\n", gpio_get_value(GPIO_BT_EN));		

		msleep(150);  // 100msec, delay  between reg_on & rst. (bcm4325 powerup sequence)
		
		/* Set GPIO_BT_nRST high */
		s3c_gpio_setpull(GPIO_BT_nRST, S3C_GPIO_PULL_NONE);
		gpio_set_value(GPIO_BT_nRST, GPIO_LEVEL_HIGH);
		
		s3c_gpio_slp_cfgpin(GPIO_BT_nRST, S3C_GPIO_SLP_OUT1);
		s3c_gpio_slp_setpull_updown(GPIO_BT_nRST, S3C_GPIO_PULL_NONE);
		
		printk("[BT] GPIO_BT_nRST = %d\n", gpio_get_value(GPIO_BT_nRST));

		gpio_free(GPIO_BT_nRST);
		gpio_free(GPIO_BT_EN);

		break;
		
	case RFKILL_USER_STATE_SOFT_BLOCKED:
		printk("[BT] Device Powering OFF \n");
		s3c_reset_uart_cfg_gpio(1);

		s3c_gpio_setpull(GPIO_BT_nRST, S3C_GPIO_PULL_NONE);
		gpio_set_value(GPIO_BT_nRST, GPIO_LEVEL_LOW);
		
		s3c_gpio_slp_cfgpin(GPIO_BT_nRST, S3C_GPIO_SLP_OUT0);
		s3c_gpio_slp_setpull_updown(GPIO_BT_nRST, S3C_GPIO_PULL_NONE);
		
		printk("[BT] GPIO_BT_nRST = %d\n",gpio_get_value(GPIO_BT_nRST));
		
		if(gpio_get_value(GPIO_WLAN_RST_N) == 0)
		{		
			s3c_gpio_setpull(GPIO_BT_EN, S3C_GPIO_PULL_NONE);
			gpio_set_value(GPIO_BT_EN, GPIO_LEVEL_LOW);
			
			s3c_gpio_slp_cfgpin(GPIO_BT_EN, S3C_GPIO_SLP_OUT0);
			s3c_gpio_slp_setpull_updown(GPIO_BT_EN, S3C_GPIO_PULL_NONE);
			
			printk("[BT] GPIO_BT_EN = %d\n", gpio_get_value(GPIO_BT_EN));
		}
		
		gpio_free(GPIO_BT_nRST);
		gpio_free(GPIO_BT_EN);
		
		break;
		
	default:
			printk(KERN_ERR "[BT] Bad bluetooth rfkill state %d\n", state);
	}
	
	return 0;
}


static void bt_host_wake_work_func(struct work_struct *ignored)
{
        wake_lock_timeout(&rfkill_wake_lock, 5*HZ);
	enable_irq(IRQ_BT_HOST_WAKE);
}
static DECLARE_WORK(bt_host_wake_work, bt_host_wake_work_func);


irqreturn_t bt_host_wake_irq_handler(int irq, void *dev_id)
{
	disable_irq_nosync(IRQ_BT_HOST_WAKE);
	schedule_work(&bt_host_wake_work);
		
	return IRQ_HANDLED;
}

static int bt_rfkill_set_block(void *data, bool blocked)
{
	unsigned int ret =0;
		
	ret = bluetooth_set_power(data, blocked? RFKILL_USER_STATE_SOFT_BLOCKED : RFKILL_USER_STATE_UNBLOCKED);
					
	return ret;
}

static const struct rfkill_ops bt_rfkill_ops = {
	.set_block = bt_rfkill_set_block,
};


static int __init saturn_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	int irq,ret;

	//Initialize wake locks
	wake_lock_init(&rfkill_wake_lock, WAKE_LOCK_SUSPEND, "board-rfkill");
#ifdef BT_SLEEP_ENABLER
	wake_lock_init(&bt_wake_lock, WAKE_LOCK_SUSPEND, "bt-rfkill");
#endif
	//BT Host Wake IRQ
	irq = IRQ_BT_HOST_WAKE;

	//set_irq_type(irq, IRQ_TYPE_EDGE_BOTH);
	set_irq_type(irq, IRQ_TYPE_EDGE_RISING);	
	ret = request_irq(irq, bt_host_wake_irq_handler, 0, "bt_host_wake_irq_handler", NULL);
	if(ret < 0)
		printk("[BT] Request_irq failed \n");

//	enable_irq(IRQ_BT_HOST_WAKE);

	//RFKILL init - default to bluetooth off
	//rfkill_switch_all(RFKILL_TYPE_BLUETOOTH, RFKILL_USER_STATE_SOFT_BLOCKED);

	bt_rfk = rfkill_alloc(bt_name, &pdev->dev, RFKILL_TYPE_BLUETOOTH,
			                                  &bt_rfkill_ops, NULL);
	if (!bt_rfk)
		return -ENOMEM;

	rfkill_init_sw_state(bt_rfk, 0);

	printk("[BT] rfkill_register(bt_rfk) \n");
	rc = rfkill_register(bt_rfk);
	if (rc)
		rfkill_destroy(bt_rfk);

	rfkill_set_sw_state(bt_rfk, 1);
	bluetooth_set_power(NULL, RFKILL_USER_STATE_SOFT_BLOCKED);

	return rc;
}

static struct platform_driver saturn_device_rfkill = {
	.probe = saturn_rfkill_probe,
	.driver = {
		.name = "bt_rfkill",
		.owner = THIS_MODULE,
	},
};

#ifdef BT_SLEEP_ENABLER
static struct rfkill *bt_sleep;

static int bluetooth_set_sleep(void *data, enum rfkill_state state)
{	int ret =0;
	switch (state) {
		
	case RFKILL_STATE_UNBLOCKED:
		if (gpio_is_valid(GPIO_BT_WAKE))
		{
			ret = gpio_request(GPIO_BT_WAKE, S3C_GPIO_LAVEL(GPIO_BT_WAKE));
			if(ret < 0) {
					printk("[BT] Failed to request GPIO_BT_WAKE!\n");
				return ret;
			}
			gpio_direction_output(GPIO_BT_WAKE, GPIO_LEVEL_LOW);
		}

		s3c_gpio_setpull(GPIO_BT_WAKE, S3C_GPIO_PULL_NONE);
		gpio_set_value(GPIO_BT_WAKE, GPIO_LEVEL_LOW);

		printk("[BT] GPIO_BT_WAKE = %d\n", gpio_get_value(GPIO_BT_WAKE) );
		gpio_free(GPIO_BT_WAKE);
		//billy's changes
			printk("[BT] wake_unlock(bt_wake_lock)\n");
			wake_unlock(&bt_wake_lock);
		break;
		
	case RFKILL_STATE_SOFT_BLOCKED:
		if (gpio_is_valid(GPIO_BT_WAKE))
		{
			ret = gpio_request(GPIO_BT_WAKE, S3C_GPIO_LAVEL(GPIO_BT_WAKE));
			if(ret < 0) {
					printk("[BT] Failed to request GPIO_BT_WAKE!\n");
				return ret;
			}
			gpio_direction_output(GPIO_BT_WAKE, GPIO_LEVEL_HIGH);
		}

		s3c_gpio_setpull(GPIO_BT_WAKE, S3C_GPIO_PULL_NONE);
		gpio_set_value(GPIO_BT_WAKE, GPIO_LEVEL_HIGH);
		
		printk("[BT] GPIO_BT_WAKE = %d\n", gpio_get_value(GPIO_BT_WAKE) );
		gpio_free(GPIO_BT_WAKE);
		//billy's changes
			printk("[BT] wake_lock(bt_wake_lock)\n");
			wake_lock(&bt_wake_lock);
		break;

	default:
			printk(KERN_ERR "[BT] bad bluetooth rfkill state %d\n", state);
	}
	return 0;
}

static int __init saturn_btsleep_probe(struct platform_device *pdev)
{
	int rc = 0;

	bt_sleep = rfkill_allocate(&pdev->dev, RFKILL_TYPE_BLUETOOTH);
	if (!bt_sleep)
		return -ENOMEM;

	bt_sleep->name = bt_name;
	bt_sleep->state = RFKILL_STATE_UNBLOCKED;
	/* userspace cannot take exclusive control */
	bt_sleep->user_claim_unsupported = 1;
	bt_sleep->user_claim = 0;
	bt_sleep->data = NULL;  // user data
	bt_sleep->toggle_radio = bluetooth_set_sleep;

	rc = rfkill_register(bt_sleep);
	if (rc)
		rfkill_free(bt_sleep);
	
	printk("[BT] rfkill_force_state(bt_sleep, RFKILL_STATE_UNBLOCKED) \n");
	rfkill_force_state(bt_sleep, RFKILL_STATE_UNBLOCKED);

	bluetooth_set_sleep(NULL, RFKILL_STATE_UNBLOCKED);

	return rc;
}

static struct platform_driver saturn_device_btsleep = {
	.probe = saturn_btsleep_probe,
	.driver = {
		.name = "bt_sleep",
		.owner = THIS_MODULE,
	},
};
#endif

static int __init saturn_rfkill_init(void)
{
	int rc = 0;
	rc = platform_driver_register(&saturn_device_rfkill);

#ifdef BT_SLEEP_ENABLER
	platform_driver_register(&saturn_device_btsleep);
#endif

	return rc;
}

module_init(saturn_rfkill_init);
MODULE_DESCRIPTION("saturn rfkill");
MODULE_AUTHOR("Nick Pelly <npelly@google.com>");
MODULE_LICENSE("GPL");
