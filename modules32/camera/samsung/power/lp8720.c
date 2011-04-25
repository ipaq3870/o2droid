/*
 *  lp8720.c - driver for LP8720
 *
 *  Copyright (C) 2009 Jeonghwan Min <jh78.min@samsung.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 */

#include <linux/i2c.h>
#include <linux/delay.h>

#include <mach/hardware.h>

#include <plat/gpio-cfg.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/sysrq.h>
#include <linux/console.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/delay.h>

#include <linux/miscdevice.h>
#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/platform_device.h>

#define LP8720_ID	0xFA

#define GENERAL_SETTINGS_REG	0x00
#define LDO1_SETTINGS_REG		0x01
#define LDO2_SETTINGS_REG		0x02
#define LDO3_SETTINGS_REG		0x03
#define LDO4_SETTINGS_REG		0x04
#define LDO5_SETTINGS_REG		0x05
#define BUCK_SETTINGS1_REG		0x06
#define BUCK_SETTINGS2_REG		0x07
#define ENABLE_BITS_REG			0x08
#define PULLDOWN_BITS_REG		0x09
#define STATUS_BITS_REG			0x0A
#define INTERRUPT_BITS_REG		0x0B
#define INTERRUPT_MASK_REG		0x0C



static struct i2c_driver lp8720_driver;

static struct i2c_client *lp8720_i2c_client = NULL;

static int lp8720_read(struct i2c_client *client, u8 reg, u8 *data)
{
	int ret;
	u8 buf[1];
	struct i2c_msg msg[2];

	buf[0] = reg; 

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = buf;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret != 2) 
		return -EIO;

	*data = buf[0];
	
	return 0;
}

static int lp8720_write(struct i2c_client *client, u8 reg, u8 data)
{
	int ret;
	u8 buf[2];
	struct i2c_msg msg[1];

	buf[0] = reg;
	buf[1] = data;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = buf;

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret != 1) 
		return -EIO;

	return 0;
}

static int lp8720_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *c;
	lp8720_i2c_client = client;
	i2c_set_clientdata(client, lp8720_i2c_client);
	return 0;
}

static int lp8720_detach_client(struct i2c_client *client)
{
	i2c_del_driver(&lp8720_driver);
	return 0;
}


static const struct i2c_device_id lp8720_id[] = {
	{ "lp8720", 0 },
	{ },
};

static struct i2c_driver lp8720_driver = {
	.driver = {
		.name = "lp8720",
	},
	.probe = lp8720_probe,
	.remove = lp8720_detach_client,
	.id_table = lp8720_id,
};

static int lp8720_init(void)
{
	printk("lp8720_init \n");
	if (lp8720_i2c_client == NULL)
		return i2c_add_driver(&lp8720_driver);

	return 0;
}

static void lp8720_exit(void)
{
	i2c_del_driver(&lp8720_driver);
}

#define GENERAL_SETTINGS_REG	0x00
#define LDO1_SETTINGS_REG		0x01
#define LDO2_SETTINGS_REG		0x02
#define LDO3_SETTINGS_REG		0x03
#define LDO4_SETTINGS_REG		0x04
#define LDO5_SETTINGS_REG		0x05
#define BUCK_SETTINGS1_REG		0x06
#define BUCK_SETTINGS2_REG		0x07
#define ENABLE_BITS_REG			0x08
#define PULLDOWN_BITS_REG		0x09
#define STATUS_BITS_REG			0x0A
#define INTERRUPT_BITS_REG		0x0B
#define INTERRUPT_MASK_REG		0x0C

/*void s5k4ca_sensor_power_init(void)
{
	u8 data;

	if (!lp8720_init()) {
		lp8720_read(lp8720_i2c_client, LDO1_SETTINGS_REG, &data);	
		data &= ~(0x1F << 0);	
		data |= (0x19 << 0);	
		lp8720_write(lp8720_i2c_client, LDO1_SETTINGS_REG, data);	
		printk("LDO1_SETTINGS_REG 0x%02x, DATA 0x%02x\n", LDO1_SETTINGS_REG, data);

		lp8720_read(lp8720_i2c_client, LDO2_SETTINGS_REG, &data);	
		data &= ~(0x1F << 0);	
		data |= (0x19 << 0);	
		lp8720_write(lp8720_i2c_client, LDO2_SETTINGS_REG, data);	
		printk("LDO2_SETTINGS_REG 0x%02x, DATA 0x%02x\n", LDO2_SETTINGS_REG, data);

		lp8720_read(lp8720_i2c_client, LDO3_SETTINGS_REG, &data);	
		data &= ~(0x1F << 0);	
		data |= (0x0C << 0);	
		lp8720_write(lp8720_i2c_client, LDO3_SETTINGS_REG, data);	
		printk("LDO3_SETTINGS_REG 0x%02x, DATA 0x%02x\n", LDO3_SETTINGS_REG, data);
		
		lp8720_read(lp8720_i2c_client, LDO5_SETTINGS_REG, &data);	
		data &= ~(0x1F << 0);	
		data |= (0x19 << 0);	
		lp8720_write(lp8720_i2c_client, LDO5_SETTINGS_REG, data);	
		printk("LDO5_SETTINGS_REG 0x%02x, DATA 0x%02x\n", LDO5_SETTINGS_REG, data);
		
		lp8720_read(lp8720_i2c_client, ENABLE_BITS_REG, &data);	
		data &= ~(0x1F << 0);	
		data |= (0x17 << 0);	
		lp8720_write(lp8720_i2c_client, ENABLE_BITS_REG, data);	
		printk("ENABLE_BITS_REG 0x%02x, DATA 0x%02x\n", ENABLE_BITS_REG, data);
	}
} */

void ce131_sensor_power_init(void)
{
	u8 data;

	if (!lp8720_init()) {
		data = 0x5;	
		lp8720_write(lp8720_i2c_client, GENERAL_SETTINGS_REG, data);
		data = 0x79;	
		lp8720_write(lp8720_i2c_client, LDO1_SETTINGS_REG, data);		
		data = 0x79;	
		lp8720_write(lp8720_i2c_client, LDO2_SETTINGS_REG, data);	
		data = 0x6c;
		lp8720_write(lp8720_i2c_client, LDO3_SETTINGS_REG, data);	
		data = 0x71;
		lp8720_write(lp8720_i2c_client, LDO4_SETTINGS_REG, data);	
		data = 0x7d;	
		lp8720_write(lp8720_i2c_client, LDO5_SETTINGS_REG, data);	
		data = 0xb;	
		lp8720_write(lp8720_i2c_client, BUCK_SETTINGS1_REG, data);	
		data = 0x9;	
		lp8720_write(lp8720_i2c_client, BUCK_SETTINGS2_REG, data);	
		data = 0xbf;
		lp8720_write(lp8720_i2c_client, ENABLE_BITS_REG, data);	
		data = 0x3f;
		lp8720_write(lp8720_i2c_client, PULLDOWN_BITS_REG, data);	
		data = 0x03;
		lp8720_write(lp8720_i2c_client, INTERRUPT_MASK_REG, data);	
	}
}

