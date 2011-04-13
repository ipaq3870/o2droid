/* drivers/input/keyboard/s3c-keypad.c
 *
 * Driver core for Samsung SoC onboard UARTs.
 *
 * Kim Kyoungil, Copyright (c) 2006-2009 Samsung Electronics
 *      http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/clk.h>
#include <linux/irq.h>

#include <linux/io.h>
#include <linux/irq.h>
#include <mach/hardware.h>
#include <asm/delay.h>
#include <asm/irq.h>

#include <plat/regs-gpio.h>
#include <plat/regs-keypad.h>

#include <mach/gpio.h>
#include <plat/gpio-cfg.h>
#ifdef CONFIG_CPU_FREQ
#include <plat/s3c64xx-dvfs.h>
#endif

#include "s3c-keypad.h"	

#include <plat/pm.h>	

#undef S3C_KEYPAD_DEBUG 
//#define S3C_KEYPAD_DEBUG 

#ifdef S3C_KEYPAD_DEBUG
#define DPRINTK(x...) printk(KERN_INFO "S3C-Keypad " x)
#define INPUT_REPORT_KEY(a,b,c) do {				\
		printk(KERN_ERR "%s:%d input_report_key(%x, %d, %d)\n", \
		       __func__, __LINE__, a, b, c);			\
		input_report_key(a,b,c);				\
	} while (0)
#else
#define DPRINTK(x...)		/* !!!! */
#define INPUT_REPORT_KEY	input_report_key
#endif

#define DEVICE_NAME "s3c-keypad"	

#define TRUE 1
#define FALSE 0


static struct timer_list keypad_timer;
//static struct timer_list powerkey_timer;
static int is_timer_on = FALSE;
static struct clk *keypad_clock;
struct resource *keypad_mem = NULL;		
struct resource *keypad_irq = NULL;	

struct resource *res = NULL;
struct input_dev *input_dev;
struct s3c_keypad *s3c_keypad;

static u32 keymask[KEYPAD_COLUMNS];
static u32 prevmask[KEYPAD_COLUMNS];

extern struct task_struct *find_task_by_pid(pid_t nr);
//static int keylock = 0;
//static unsigned int process_id = 0;

static int in_sleep = 0;


void earjack_report_key(unsigned int keycode, int value)
{
    input_report_key(s3c_keypad->dev, KEY_SEND, value);
    //printk(DCM_INP, "[KEY] earjack key %s\n", value ? "pressed" : "released");   
}
EXPORT_SYMBOL(earjack_report_key);

static int keypad_scan(void)
{

	u32 col, cval, rval;

	for (col = 0; col < KEYPAD_COLUMNS; col++) {

		cval = KEYCOL_DMASK & ~((1 << col) | (1 << (col + 8)));	

		writel(cval, key_base + S3C_KEYIFCOL);	
		udelay(KEYPAD_DELAY);

		rval =~(readl(key_base + S3C_KEYIFROW)) & ((1 << KEYPAD_ROWS) -1);
		keymask[col] = rval;
	}

	writel(KEYIFCOL_CLEAR, key_base + S3C_KEYIFCOL);

	return 0;
}

static void keypad_timer_handler(unsigned long data)
{
	u32 press_mask;
	u32 release_mask;
	u32 restart_timer = 0;
	int i, col;
	struct s3c_keypad *pdata = (struct s3c_keypad *)data;
	struct input_dev *dev = pdata->dev;

	keypad_scan();

	for (col = 0; col < KEYPAD_COLUMNS; col++) {
		press_mask = ((keymask[col] ^ prevmask[col]) & keymask[col]);
		release_mask = ((keymask[col] ^ prevmask[col]) & prevmask[col]);
/*
#ifdef CONFIG_CPU_FREQ
#if USE_PERF_LEVEL_KEYPAD
		if (press_mask || release_mask)
			set_dvfs_target_level(LEV_400MHZ);
#endif
#endif
*/
		i = col * KEYPAD_ROWS;

		while (press_mask) {
			if (press_mask & 1) {
				input_report_key(dev, pdata->keycodes[i], 1);
				DPRINTK("\nkey Pressed  : key %d map %d\n", i,
					pdata->keycodes[i]);
			}
			press_mask >>= 1;
			i++;
		}

		i = col * KEYPAD_ROWS;

		while (release_mask) {
			if (release_mask & 1) {
				input_report_key(dev, pdata->keycodes[i], 0);
				DPRINTK("\nkey Released : %d  map %d\n", i,
					pdata->keycodes[i]);

			}
			release_mask >>= 1;
			i++;
		}
		prevmask[col] = keymask[col];

		restart_timer |= keymask[col];
	}

	if (restart_timer) {
		mod_timer(&keypad_timer, jiffies + HZ / 10);
	} else {
		writel(KEYIFCON_INIT, key_base + S3C_KEYIFCON);
		is_timer_on = FALSE;
	}

}



static irqreturn_t s3c_keypad_isr(int irq, void *dev_id)
{

	/* disable keypad interrupt and schedule for keypad timer handler */
	writel(readl(key_base + S3C_KEYIFCON) & ~(INT_F_EN | INT_R_EN), key_base + S3C_KEYIFCON);

	keypad_timer.expires = jiffies;
	if (is_timer_on == FALSE) {
		add_timer(&keypad_timer);
		is_timer_on = TRUE;
	} else {
		mod_timer(&keypad_timer, keypad_timer.expires);
	}
	/*Clear the keypad interrupt status */
	writel(KEYIFSTSCLR_CLEAR, key_base + S3C_KEYIFSTSCLR);

	return IRQ_HANDLED;
}

static irqreturn_t s3c_keygpio_isr(int irq, void *dev_id)
{
	unsigned int key_status;
	static unsigned int prev_key_status= 0;
	struct s3c_keypad *pdata = (struct s3c_keypad *)dev_id;
	struct input_dev *dev = pdata->dev;

	// Beware that we may not obtain exact key up/down status at
	// this point.
	key_status = gpio_get_value(GPIO_nPOWER);

	// If ISR is called and up/down status remains the same, we
	// must have lost one and should report that first with
	// upside/down.
	if (in_sleep) {
		if (key_status == prev_key_status) {
			INPUT_REPORT_KEY(dev, 26, key_status ? 1 : 0); 
		}
		in_sleep = 0;
	}

	INPUT_REPORT_KEY(dev, 26, key_status ? 0 : 1);

	prev_key_status = key_status;
	//printk(KERN_DEBUG "s3c_keygpio_isr pwr key_status =%d\n", key_status);

	return IRQ_HANDLED;
}

static int s3c_keygpio_isr_setup(void *pdev)
{
	int ret;

	//PWR key
	s3c_gpio_cfgpin(GPIO_nPOWER, S3C_GPIO_SFN(GPIO_nPOWER_AF));
	s3c_gpio_setpull(GPIO_nPOWER, S3C_GPIO_PULL_NONE);

	set_irq_type(IRQ_EINT(5), IRQ_TYPE_EDGE_BOTH);

	// stk.lim: Add IRQF_DISABLED to eliminate any possible race
	// regarding key status
	ret = request_irq(IRQ_EINT(5), s3c_keygpio_isr, IRQF_DISABLED
			  | IRQF_SAMPLE_RANDOM, "key gpio", (void *)pdev);

	if (ret) {
		printk("request_irq failed (IRQ_KEYPAD (gpio)) !!!\n");
		ret = -EIO;
	}

	return ret;

}

static ssize_t keyshort_test(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	int count;

	if (!gpio_get_value(GPIO_KBR0) || !gpio_get_value(GPIO_KBR1)
	    || !gpio_get_value(GPIO_KBR2) || !gpio_get_value(GPIO_nPOWER)) {
		count = sprintf(buf, "PRESS\n");
		//printk("keyshort_test: PRESS\n");
	} else {
		count = sprintf(buf, "RELEASE\n");
		//printk("keyshort_test: RELEASE\n");
	}

	return count;
}

static DEVICE_ATTR(key_pressed, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH,
		   keyshort_test, NULL);

static int __init s3c_keypad_probe(struct platform_device *pdev)
{
	struct resource *res, *keypad_mem, *keypad_irq;
	struct input_dev *input_dev;
	struct s3c_keypad *s3c_keypad;
	int ret, size;
	int key, code;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no memory resource specified\n");
		return -ENOENT;
	}

	size = (res->end - res->start) + 1;

	keypad_mem = request_mem_region(res->start, size, pdev->name);
	if (keypad_mem == NULL) {
		dev_err(&pdev->dev, "failed to get memory region\n");
		ret = -ENOENT;
		goto err_req;
	}

	key_base = ioremap(res->start, size);
	if (key_base == NULL) {
		printk(KERN_ERR "Failed to remap register block\n");
		ret = -ENOMEM;
		goto err_map;
	}

	keypad_clock = clk_get(&pdev->dev, "keypad");
	if (IS_ERR(keypad_clock)) {
		dev_err(&pdev->dev, "failed to find keypad clock source\n");
		ret = PTR_ERR(keypad_clock);
		goto err_clk;
	}

	clk_enable(keypad_clock);

	s3c_keypad = kzalloc(sizeof(struct s3c_keypad), GFP_KERNEL);
	input_dev = input_allocate_device();

	if (!s3c_keypad || !input_dev) {
		ret = -ENOMEM;
		goto err_alloc;
	}



	platform_set_drvdata(pdev, s3c_keypad);
	s3c_keypad->dev = input_dev;

	writel(KEYIFCON_INIT, key_base + S3C_KEYIFCON);
	writel(KEYIFFC_DIV, key_base + S3C_KEYIFFC);

	/* Set GPIO Port for keypad mode and pull-up disable */
	s3c_setup_keypad_cfg_gpio(KEYPAD_ROWS, KEYPAD_COLUMNS);




	writel(KEYIFCOL_CLEAR, key_base + S3C_KEYIFCOL);

	/* create and register the input driver */
	set_bit(EV_KEY, input_dev->evbit);
	/*Commenting the generation of repeat events */
	//set_bit(EV_REP, input_dev->evbit);
	s3c_keypad->nr_rows = KEYPAD_ROWS;
	s3c_keypad->no_cols = KEYPAD_COLUMNS;
	s3c_keypad->total_keys = MAX_KEYPAD_NR;

	for (key = 0; key < s3c_keypad->total_keys; key++) {
		code = s3c_keypad->keycodes[key] = keypad_keycode[key];
		if (code <= 0)
			continue;
		set_bit(code & KEY_MAX, input_dev->keybit);
	}

	printk("%s, keypad row number is %d, column is %d",__FUNCTION__, s3c_keypad->nr_rows, s3c_keypad->no_cols);

	set_bit(26 & KEY_MAX, input_dev->keybit);

	input_dev->name = DEVICE_NAME;
	input_dev->phys = "s3c-keypad/input0";

	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0001;

	input_dev->keycode = keypad_keycode;

	keypad_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	ret = input_register_device(input_dev);
	if (ret) {
		printk("Unable to register s3c-keypad input device!!!\n");
		goto out;
	}

	/* Scan timer init */
	init_timer(&keypad_timer);
	keypad_timer.function = keypad_timer_handler;
	keypad_timer.data = (unsigned long)s3c_keypad;

	/* For IRQ_KEYPAD */
	if (keypad_irq == NULL) {
		dev_err(&pdev->dev, "no irq resource specified\n");
		ret = -ENOENT;
		goto err_irq;
	}

	ret = request_irq(IRQ_KEYPAD, s3c_keypad_isr, IRQF_SAMPLE_RANDOM, //PREVIOUS OLD ONE
		DEVICE_NAME, (void *) pdev);
	if (ret) {
		printk("request_irq failed (IRQ_KEYPAD) !!!\n");
		ret = -EIO;
		goto err_irq;
	}
	s3c_keygpio_isr_setup((void *)s3c_keypad);
	printk(DEVICE_NAME " Initialized\n");

	if (device_create_file(&pdev->dev, &dev_attr_key_pressed) < 0) {
		printk("%s s3c_keypad_probe\n", __FUNCTION__);
		pr_err("Failed to create device file(%s)!\n",
		       dev_attr_key_pressed.attr.name);
	}

	return 0;

 out:
	free_irq(keypad_irq->start, input_dev);
	free_irq(keypad_irq->end, input_dev);

 err_irq:
	input_free_device(input_dev);
	kfree(s3c_keypad);

 err_alloc:
	clk_disable(keypad_clock);
	clk_put(keypad_clock);

 err_clk:
	iounmap(key_base);

 err_map:
	release_resource(keypad_mem);
	kfree(keypad_mem);

 err_req:
	return ret;
}


static int s3c_keypad_remove(struct platform_device *pdev)
{
	struct input_dev *input_dev = platform_get_drvdata(pdev);
	writel(KEYIFCON_CLEAR, key_base + S3C_KEYIFCON);

	if (keypad_clock) {
		clk_disable(keypad_clock);
		clk_put(keypad_clock);
		keypad_clock = NULL;
	}

	input_unregister_device(input_dev);
	iounmap(key_base);
	kfree(pdev->dev.platform_data);
	free_irq(IRQ_KEYPAD, (void *)pdev);

	del_timer(&keypad_timer);
	printk(DEVICE_NAME " Removed.\n");
	return 0;
}

#ifdef CONFIG_PM


static struct sleep_save s3c_keypad_save[] = {
	SAVE_ITEM(KEYPAD_ROW_GPIOCON),
	SAVE_ITEM(KEYPAD_COL_GPIOCON),
	SAVE_ITEM(KEYPAD_ROW_GPIOPUD),
	SAVE_ITEM(KEYPAD_COL_GPIOPUD),
};

static unsigned int keyifcon, keyiffc;
static int s3c_keypad_suspend(struct platform_device *dev, pm_message_t state)
{
	keyifcon = readl(key_base + S3C_KEYIFCON);
	keyiffc = readl(key_base + S3C_KEYIFFC);

	s3c6410_pm_do_save(s3c_keypad_save, ARRAY_SIZE(s3c_keypad_save));

	//writel(~(0xfffffff), KEYPAD_ROW_GPIOCON);
	//writel(~(0xfffffff), KEYPAD_COL_GPIOCON);

	disable_irq(IRQ_KEYPAD);

	clk_disable(keypad_clock);

	in_sleep = 1;

	return 0;
}

static int s3c_keypad_resume(struct platform_device *dev)
{
//	struct s3c_keypad *s3c_keypad =
//	    (struct s3c_keypad *)platform_get_drvdata(dev);
//	struct input_dev *iDev = s3c_keypad->dev;
//	unsigned int key_temp_data = 0;

	printk(KERN_DEBUG "++++ %s\n", __FUNCTION__);

	clk_enable(keypad_clock);

	writel(KEYIFCON_INIT, key_base + S3C_KEYIFCON);
	writel(keyiffc, key_base + S3C_KEYIFFC);
	writel(KEYIFCOL_CLEAR, key_base + S3C_KEYIFCOL);

	s3c6410_pm_do_restore(s3c_keypad_save, ARRAY_SIZE(s3c_keypad_save));

	enable_irq(IRQ_KEYPAD);
	printk(KERN_DEBUG "---- %s\n", __FUNCTION__);
	return 0;
}
#else
#define s3c_keypad_suspend NULL
#define s3c_keypad_resume  NULL
#endif				/* CONFIG_PM */

static struct platform_driver s3c_keypad_driver = {
	.probe		= s3c_keypad_probe,
	.remove		= s3c_keypad_remove,
	.suspend	= s3c_keypad_suspend,
	.resume		= s3c_keypad_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= DEVICE_NAME,
	},
};

static int __init s3c_keypad_init(void)
{
	int ret;

	ret = platform_driver_register(&s3c_keypad_driver);
	
	if(!ret)
	   printk(KERN_INFO "S3C Keypad Driver\n");

	return ret;
}

static void __exit s3c_keypad_exit(void)
{
	platform_driver_unregister(&s3c_keypad_driver);
}

module_init(s3c_keypad_init);
module_exit(s3c_keypad_exit);

MODULE_AUTHOR("Samsung");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("KeyPad interface for Samsung S3C");
