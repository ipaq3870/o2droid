/****************************************************************************
**
** COPYRIGHT(C)	: Samsung Electronics Co.Ltd, 2006-2015 ALL RIGHTS RESERVED
**
*****************************************************************************/
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/err.h>
#include <linux/gpio.h>


#include <asm/io.h>
#include <mach/hardware.h>
#include <plat/gpio-cfg.h>
#include <linux/delay.h>

#include <linux/timed_output.h>

//#include "vibetonz.h"

/*********** for debug **********************************************************/
#if 0 
#define gprintk(fmt, x... ) printk( "%s(%d): " fmt, __FUNCTION__ ,__LINE__, ## x)
#else
#define gprintk(x...) do { } while (0)
#endif
/*******************************************************************************/

#define OFFSET_VIBRATOR_ON      (0x1 << 0)

static struct hrtimer timer;

static int max_timeout = 5000;
static int vibrator_value = 0;
static int vibrator_pwm = 0;

extern int s3c6410_timer_setup (int channel, int usec, unsigned long g_tcnt, unsigned long g_tcmp);

#if defined(CONFIG_MACH_SPICA) || defined(CONFIG_MACH_INSTINCTQ)
extern void s3c_bat_set_compensation_for_drv(int mode,int offset);
#endif


/* pwm change function
 * nforce : 3 ~ 297
 * 0   is min
 * 297 if max
 */
static void set_pwm_freq(int nforce)
{
	int vibrator_on_period;
	int vibrator_off_period;

	//Temporally set vibrator strength to Max.
	nforce = 297;

//	if(vibrator_pwm != nforce) {
		vibrator_on_period = 300;
		vibrator_off_period = 300 - nforce;

		s3c6410_timer_setup(1,10,vibrator_on_period,vibrator_off_period);
		vibrator_pwm = nforce;
		gprintk("%s, pwm set to %d\n",__func__,nforce);
//	}
}

static int set_vibetonz(int timeout)
{

	gprintk("[VIBETONZ] : timeout is %d\n",timeout);	
	if(!timeout) {
//		printk("[VIBETONZ] DISABLE\n");
		gpio_set_value(GPIO_VIB_EN, GPIO_LEVEL_LOW);

#if defined(CONFIG_MACH_SPICA) || defined(CONFIG_MACH_INSTINCTQ)
		s3c_bat_set_compensation_for_drv(0,OFFSET_VIBRATOR_ON);
#endif
	}
	else {
		if(timeout > 0 ) {
			if(timeout == 3241) { 
				/* for test if input is 3241 ms*/
				gprintk("[VIBETONZ] : test mode %d\n",vibrator_pwm);
				/* for alarm or vibe mode */
			} else {
				gprintk("[VIBETONZ] : normal mode %d\n",vibrator_pwm);
				set_pwm_freq(270);
			}
		} else {
			/* for vibrator test app. */
			set_pwm_freq(297);
		}
		gpio_set_value(GPIO_VIB_EN, GPIO_LEVEL_HIGH);

#if defined(CONFIG_MACH_SPICA) || defined(CONFIG_MACH_INSTINCTQ)  
		s3c_bat_set_compensation_for_drv(1,OFFSET_VIBRATOR_ON);
#endif
	}

	vibrator_value = timeout;

	
	return 0;
}

static enum hrtimer_restart vibetonz_timer_func(struct hrtimer *timer)
{

	//gprintk("[VIBETONZ] %s : \n",__func__);
	set_vibetonz(0);
	return HRTIMER_NORESTART;
}

static int get_time_for_vibetonz(struct timed_output_dev *dev)
{
	int remaining;


	if (hrtimer_active(&timer)) {
		ktime_t r = hrtimer_get_remaining(&timer);
		remaining = r.tv.sec * 1000 + r.tv.nsec / 1000000;
	} else
		remaining = 0;

	if (vibrator_value ==-1)
		remaining = -1;

	return remaining;

}
static void enable_vibetonz_from_user(struct timed_output_dev *dev,int value)
{

	//printk("[VIBETONZ] %s : time = %d msec \n",__func__,value);
	hrtimer_cancel(&timer);

	
	set_vibetonz(value);
	vibrator_value = value;

	if(value == 0xFFFF)
	{
		printk("[VIBETONZ] now magic value, unlimited vibration\n");
	}

	else if (value > 0) 
	{
		if (value > max_timeout)
			value = max_timeout;

		hrtimer_start(&timer,
						ktime_set(value / 1000, (value % 1000) * 1000000),
						HRTIMER_MODE_REL);
		vibrator_value = 0;
	}


}

static ssize_t show_pwm(struct device *dev, struct device_attribute *attr, char *buf)
{
	        return sprintf(buf, "PWM ( 3 ~ 297 ) current pwm is [%d]\n", vibrator_pwm);
}

static ssize_t store_pwm(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size) {

	int value;
	sscanf(buf, "%d", &value);
	gprintk("PWM Changed to %d\n",value);

	vibrator_pwm = value;
	if(vibrator_pwm < 3) {
		printk("wrong pwm value\n");
	} else if(vibrator_pwm > 297) {
		printk("wrong pwm value\n");
	}

	set_pwm_freq(vibrator_pwm);

	return vibrator_pwm;
}

static DEVICE_ATTR(pwm, 0777, show_pwm, store_pwm);

static struct timed_output_dev timed_output_vt = {
	.name     = "vibrator",
	.get_time = get_time_for_vibetonz,
	.enable   = enable_vibetonz_from_user,
};


static void vibetonz_start(void)
{
	int ret = 0;

	//printk("[VIBETONZ] %s : \n",__func__);


	/* hrtimer settings */
	hrtimer_init(&timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	timer.function = vibetonz_timer_func;


	if (gpio_is_valid(GPIO_VIB_EN)) {
		if (gpio_request(GPIO_VIB_EN, S3C_GPIO_LAVEL(GPIO_VIB_EN))) 
			printk(KERN_ERR "Failed to request GPIO_VIB_EN!\n");
		gpio_direction_output(GPIO_VIB_EN,0);
		mdelay(10);
		gpio_set_value(GPIO_VIB_EN, GPIO_LEVEL_LOW);
	}
	s3c_gpio_setpull(GPIO_VIB_EN, S3C_GPIO_PULL_NONE);

		
	/* timed_output_device settings */
	ret = timed_output_dev_register(&timed_output_vt);
	if(ret)
		printk(KERN_ERR "[VIBETONZ] timed_output_dev_register is fail \n");
	ret = device_create_file(timed_output_vt.dev, &dev_attr_pwm);
	if(ret) {
		printk(KERN_ERR "[VIBETONZ] pwm device file create failed\n");
        }
}


static void vibetonz_end(void)
{
	gprintk("[VIBETONZ] %s \n",__func__);
}


static void __init vibetonz_init(void)
{
	vibetonz_start();
}


static void __exit vibetonz_exit(void)
{
	vibetonz_end();
}

module_init(vibetonz_init);
module_exit(vibetonz_exit);

MODULE_AUTHOR("SAMSUNG");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("vibetonz control interface");

