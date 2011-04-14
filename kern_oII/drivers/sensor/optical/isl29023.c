/* 
 *  Title : Optical Sensor driver for ISL29023 light sensor 
 *  Date  : 2011.02.14
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

#include "isl29023.h"

/*********** for debug **********************************************************/
#if 0 
#define gprintk(fmt, x... ) printk( "%s(%d): " fmt, __FUNCTION__ ,__LINE__, ## x)
#else
#define gprintk(x...) do { } while (0)
#endif
/*******************************************************************************/

/* global var */
static struct i2c_client *isl_i2c_client = NULL;

struct class *lightsensor_class;
struct device *switch_cmd_dev;

static int lux_value = 0;
static int int_op_mode = 1; // set to 1 to diable irq in probe (light_off) at init time
static u16 op_mode;
static bool light_enable = ON;
static int  cur_state = 0;

static struct i2c_driver isl_i2c_driver = {
	.driver = {
		.name = "ISL29023",
	},
	.attach_adapter = &isl_attach_adapter,
};

static unsigned short isl_normal_i2c[] = {(LIGHT_ADDR >> 1),I2C_CLIENT_END};
static unsigned short isl_ignore[] =  {1 ,(LIGHT_ADDR >> 1),I2C_CLIENT_END};
static unsigned short isl_probe[] = {I2C_CLIENT_END};


static struct i2c_client_address_data isl_addr_data = {
	.normal_i2c = isl_normal_i2c,
	.ignore		= isl_ignore,
	.probe		= isl_probe,	
};

#if 0
void print_sensor_regs(void) {
	gprintk("REGS_COMMAND_I: %x\n", isl_i2c_read((u8)(REGS_COMMAND_I)));
	gprintk("REGS_COMMAND_II: %x\n", isl_i2c_read((u8)(REGS_COMMAND_II)));
	gprintk("REGS_INT_MSB_TH_LO: %x\n", isl_i2c_read((u8)(REGS_INT_MSB_TH_LO)));
	gprintk("REGS_INT_LSB_TH_LO: %x\n", isl_i2c_read((u8)(REGS_INT_LSB_TH_LO)));
	gprintk("REGS_INT_MSB_TH_HI: %x\n", isl_i2c_read((u8)(REGS_INT_MSB_TH_HI)));
	gprintk("REGS_INT_LSB_TH_HI: %x\n", isl_i2c_read((u8)(REGS_INT_LSB_TH_HI)));
	gprintk("REGS_MBS_SENSOR: %x\n", isl_i2c_read((u8)(REGS_MBS_SENSOR)));
	gprintk("REGS_LBS_SENSOR: %x\n", isl_i2c_read((u8)(REGS_LBS_SENSOR)));
}	
#endif

int adc_to_lux(int value) {

	// see isl29023.h for detailes
	return FSR_LUX_range[op_mode & 3] * value / 
			adc_resolution[(op_mode >> 2) & 3];
}

int lux_to_adc(int value) {
	int ret;
	ret = adc_resolution[(op_mode >> 2) & 3] * value /
				FSR_LUX_range[op_mode & 3];
	if (ret > 0xffff) 
		return 0xffff;
	return ret;
}

void set_irq_threshold(int low, int high) {
	/* convert back LUX to ADC */
	isl_i2c_write((u8)(REGS_INT_LSB_TH_LO), lux_to_adc(low) & 0xff);
	isl_i2c_write((u8)(REGS_INT_MSB_TH_LO), (lux_to_adc(low) >> 8) & 0xff);
	isl_i2c_write((u8)(REGS_INT_LSB_TH_HI), lux_to_adc(high) & 0xff);
	isl_i2c_write((u8)(REGS_INT_MSB_TH_HI), (lux_to_adc(high) >> 8) & 0xff);
}

void set_new_state(void) {
	int no_change = 0;

	while (!no_change) {
		if(lux_value < light_state[cur_state].lux_bottom_limit) {
			if (cur_state > 0)  cur_state--;
		} else if(lux_value > light_state[cur_state].lux_top_limit) {
			if (cur_state < (ARRAY_SIZE(light_state) -1))  cur_state++;
		} else no_change = 1;	
	}
	
	backlight_level_ctrl(light_state[cur_state].brightness);

	if (int_op_mode)
		set_irq_threshold(light_state[cur_state].lux_bottom_limit, 
				light_state[cur_state].lux_top_limit);
}

/*****************************************************************************************
 *  
 *  function    : work_func_light 
 */
void work_func_light(struct work_struct *work) {
	struct light_data *light = container_of(work, struct light_data, work_light);

	/* read light data from sensor i2c */
	unsigned char v_msb;
	unsigned char v_lsb;
	unsigned int vout = 0;

	isl_i2c_write((u8)(REGS_COMMAND_I), (op_mode >> 8) & 0xff); 
	isl_i2c_write((u8)(REGS_COMMAND_II), op_mode & 0xff);

	if (!int_op_mode) {
		set_irq_threshold(0x0, 0xffff);
		msleep(150);
	}	
	//print_sensor_regs();

	v_msb = isl_i2c_read((u8)(REGS_MBS_SENSOR));
	v_lsb = isl_i2c_read((u8)(REGS_LBS_SENSOR));
	vout =  (v_msb << 8) | v_lsb;

	lux_value = adc_to_lux(vout);	
	set_new_state();
#if USE_INPUT_DEVICE 
    		input_report_abs(light->input_dev,ABS_DISTANCE, lux_value);
		input_sync(light->input_dev);
		mdelay(1);
#endif
	printk("Ligh Sensor LCD = %d ADC= %d LUX= %d\n", 
					cur_state, vout, lux_value);
	if (int_op_mode) {
		gprintk("enable irq for light sensor\n");
		enable_irq(light->irq);
		// clear interrupt enable bit
		isl_i2c_read((u8)(REGS_COMMAND_I)); 
	}	
}

irqreturn_t light_irq_handler(int irq, void *dev_id) {
	struct light_data *light = dev_id;

	gprintk("light->irq = %d\n",light->irq);

	if(light->irq !=-1) {
		disable_irq_nosync(light->irq);
		queue_work(light_wq, &light->work_light);
	}

	return IRQ_HANDLED;
}

static enum hrtimer_restart light_timer_func(struct hrtimer *timer) {
	struct light_data *light = container_of(timer, struct light_data, timer);
	
	queue_work(light_wq, &light->work_light);
	hrtimer_start(&light->timer,ktime_set(LIGHT_PERIOD,0),HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static int isl_attach(struct i2c_adapter *adap, int addr, int kind) {
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
	strncpy(c->name,"ISL29023",I2C_NAME_SIZE);
	c->addr = addr;
	c->adapter = adap;
	c->driver = &isl_i2c_driver;
	c->flags = I2C_DF_NOTIFY | I2C_M_IGNORE_NAK;

	if ((ret = i2c_attach_client(c)) < 0) {
		printk("i2c_attach_client error\n");
		goto error;
	}
	isl_i2c_client = c;

	gprintk("\n");
	return ret;
error:
	printk("in %s , ret = %d \n",__func__,ret);
	kfree(c);
	return err;
}

static int isl_attach_adapter(struct i2c_adapter *adap) {
	int ret;
	gprintk("\n");
	ret =  i2c_probe(adap, &isl_addr_data, isl_attach);
	return ret;
}

static int isl_i2c_init(void) {
	if( i2c_add_driver(&isl_i2c_driver)) {
		printk("i2c_add_driver failed \n");
		return -ENODEV;
	}
	return 0;
}

int isl_i2c_read(u8 reg) {
	int val;
	val = i2c_smbus_read_byte_data(isl_i2c_client, reg);	
	if (val < 0)
		printk("%s %d i2c transfer error\n", __func__, __LINE__);
	return val;
}

int isl_i2c_write( u8 reg, int val ) {
	int err;

	if( (isl_i2c_client == NULL) || (!isl_i2c_client->adapter) ){
		return -ENODEV;
	}
	err = i2c_smbus_write_byte_data(isl_i2c_client, reg, val);
	if (err >= 0) return 0;

	printk("%s %d i2c transfer error\n", __func__, __LINE__);
	return err;
}

void light_chip_init(void) {
	gprintk("\n");
	
	/* Power On */
	if (gpio_is_valid(GPIO_ALS_EN)) {
		if (gpio_request(GPIO_ALS_EN, S3C_GPIO_LAVEL(GPIO_ALS_EN)))
			printk(KERN_ERR "Filed to request GPIO_ALS_EN!\n");
		gpio_direction_output(GPIO_ALS_EN, GPIO_LEVEL_HIGH);
		s3c_gpio_setpull(GPIO_ALS_EN, S3C_GPIO_PULL_NONE); 
	}

	mdelay(5);
	
	/* set INT 	*/
	s3c_gpio_cfgpin(GPIO_AMBIENT_INT_N, S3C_GPIO_SFN(GPIO_AMBIENT_INT_N_AF));
	s3c_gpio_setpull(GPIO_AMBIENT_INT_N, S3C_GPIO_PULL_UP);
	set_irq_type(IRQ_LIGHT_INT, IRQ_TYPE_EDGE_FALLING);
}

/*****************************************************************************************
 *  
 *  function    : light_on 
 *  description : This function is power-on function for optical sensor.
 */
void light_on(struct light_data *light) {
	if( light_enable == OFF) {
		gprintk("light power on \n");
		gpio_direction_output(GPIO_ALS_EN, GPIO_LEVEL_HIGH);
		mdelay(5);
		if (int_op_mode) {
			queue_work(light_wq, &light->work_light);
		} 
		else {		
			gprintk("timer start for light sensor\n");
			hrtimer_start(&light->timer,ktime_set(LIGHT_PERIOD,0),HRTIMER_MODE_REL);
		}		
	}
}

/*****************************************************************************************
 *  
 *  function    : light_off 
 *  description : This function is power-off function for optical sensor.
 */
void light_off(struct light_data *light) {
	if(light_enable == ON) {
		gprintk("Light sensor power off \n");

		if (int_op_mode) {
			gprintk("disable irq for light sensor\n");
			disable_irq_nosync(light->irq);
		}	
		else {		
			gprintk("timer cancel for light sensor\n");
			hrtimer_cancel(&light->timer);
		}
		
		isl_i2c_write((u8)(REGS_COMMAND_I), 0x0);
		isl_i2c_write((u8)(REGS_COMMAND_II), 0x0);

		gpio_direction_output(GPIO_ALS_EN, GPIO_LEVEL_LOW);
		mdelay(5);
	}
}

static ssize_t get_lux_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%u\n", lux_value);
}

static ssize_t get_lux_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
	int value;
	sscanf(buf, "%d", &value);
	return size;
}

static DEVICE_ATTR(get_lux,0444, get_lux_show, get_lux_store);

static ssize_t lightsensor_file_cmd_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%x\n", (int_op_mode << 16) | op_mode);
}

static ssize_t lightsensor_file_cmd_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
	struct light_data *light = dev_get_drvdata(dev);
	int value;
	sscanf(buf, "%x", &value);

	printk("in lightsensor_file_cmd_store, input value = %x \n",value);


	if(value && light_enable == OFF) {
		if (value == 1) {
			op_mode = 0xa304; // default int operation mode
			int_op_mode = 1;
		} 
		else {
			int_op_mode = ((value >> 16 ) & 0x1);
			op_mode = (value & 0xffff);
		}
		light_on(light);
		light_enable = ON;
	}
	else if(!value && light_enable ==ON) {
		light_off(light);
		light_enable = OFF;
	}
	else if(value && light_enable ==ON) {
		
	}
	return size;
}

static DEVICE_ATTR(lightsensor_file_cmd,0644, lightsensor_file_cmd_show, lightsensor_file_cmd_store);

static int isl_light_probe( struct platform_device* pdev ) {
	struct light_data *light;
	int irq;
	int ret;

	/* allocate driver_data */
	light = kzalloc(sizeof(struct light_data),GFP_KERNEL);
	if(!light) {
		pr_err("kzalloc error\n");
		return -ENOMEM;
	}
	gprintk("in %s \n",__func__);
	
	/* init i2c */
	isl_i2c_init();

	if(isl_i2c_client == NULL) {
		pr_err("isl_probe failed : i2c_client is NULL\n"); 
		return -ENODEV;
	}

	/* hrtimer Settings */
	hrtimer_init(&light->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	light->timer.function = light_timer_func;

	/* WORK QUEUE Settings */
    	light_wq = create_singlethread_workqueue("light_wq");
	if (!light_wq)
	    return -ENOMEM;
        INIT_WORK(&light->work_light, work_func_light);
	gprintk("Workqueue Settings complete\n");

	/* INT Settings */	
	irq = IRQ_LIGHT_INT;
	light->irq = -1;
	ret = request_irq(irq, light_irq_handler, 0, "light_int", light);
	if (ret) {
		pr_err("unable to request irq %d\n", irq);
		return ret;
	}       
	light->irq = irq;
	gprintk("INT Settings complete\n");

	/* set platdata */
	platform_set_drvdata(pdev, light);

	/* set sysfs for light sensor */
	lightsensor_class = class_create(THIS_MODULE, "lightsensor");
	if (IS_ERR(lightsensor_class))
		pr_err("Failed to create class(lightsensor)!\n");

	switch_cmd_dev = device_create(lightsensor_class, NULL, 0, NULL, "switch_cmd");
	if (IS_ERR(switch_cmd_dev))
		pr_err("Failed to create device(switch_cmd_dev)!\n");

	if (device_create_file(switch_cmd_dev, &dev_attr_lightsensor_file_cmd) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_lightsensor_file_cmd.attr.name);

	if (device_create_file(switch_cmd_dev, &dev_attr_get_lux) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_get_lux.attr.name);

	dev_set_drvdata(switch_cmd_dev,light);

#if USE_INPUT_DEVICE	
	/* Input device Settings */
	light->input_dev = input_allocate_device();
	if (light->input_dev == NULL) {
		pr_err("Failed to allocate input device\n");
		return -ENOMEM;
	}
	light->input_dev->name = "LightSensor";

	set_bit(EV_SYN,light->input_dev->evbit);
	set_bit(EV_ABS,light->input_dev->evbit);
	
	input_set_abs_params(light->input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	ret = input_register_device(light->input_dev);
	if (ret) {
		pr_err("Unable to register %s input device\n", light->input_dev->name);
		input_free_device(light->input_dev);
		kfree(light);
		return -1;
	}
#endif


	light_off(light);
	light_enable = OFF;
	printk("ISL29023 Light Sensor initialized\n");
	
	return 0;
}

static int isl_light_suspend( struct platform_device* pdev, pm_message_t state ) {
	struct light_data *light = platform_get_drvdata(pdev);

	if(light_enable) {
		if (int_op_mode)
			disable_irq_nosync(light ->irq);
		isl_i2c_write((u8)(REGS_COMMAND_I), 0x0);
		isl_i2c_write((u8)(REGS_COMMAND_II), 0x0);
	}	
	return 0;
}


static int isl_light_resume( struct platform_device* pdev ) {
	struct light_data *light = platform_get_drvdata(pdev);

	if(light_enable) {
		if (int_op_mode)
			queue_work(light_wq, &light->work_light);
		else
			hrtimer_start(&light->timer,ktime_set(LIGHT_PERIOD,0),HRTIMER_MODE_REL);
	}
	return 0;
}

static struct platform_driver isl_light_driver = {
	.probe 	 = isl_light_probe,
	.suspend = isl_light_suspend,
	.resume  = isl_light_resume,
	.driver  = {
		.name = "isl29023-opt",
		.owner = THIS_MODULE,
	},
};

static int __init isl_light_init(void) {
	int ret;
	
	light_chip_init();
	ret = platform_driver_register(&isl_light_driver);
	return ret;
}

static void __exit isl_light_exit(void) {
	struct light_data *light = dev_get_drvdata(switch_cmd_dev);
	if (light_wq)
		destroy_workqueue(light_wq);

	free_irq(IRQ_LIGHT_INT, light);
	gpio_direction_output(GPIO_AMBIENT_INT_N, GPIO_LEVEL_LOW);
	s3c_gpio_setpull(GPIO_AMBIENT_INT_N, S3C_GPIO_PULL_NONE);
#if USE_INPUT_DEVICE
	input_unregister_device(light->input_dev);
#endif
	kfree(light);

	gpio_free(GPIO_AMBIENT_INT_N);
	platform_driver_unregister(&isl_light_driver);
}

module_init( isl_light_init );
module_exit( isl_light_exit );

MODULE_AUTHOR("BS");
MODULE_DESCRIPTION("Optical Sensor driver for ISL29023");
MODULE_LICENSE("GPL");
