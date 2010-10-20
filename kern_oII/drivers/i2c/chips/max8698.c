/*
 * max8698.c - driver for MAX8698 PMIC.
 *
 * Copyright (C) 2009 Samsung Electronics.
 *
 * Author: Geun-Young, Kim <nenggun.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 */

#include <linux/kernel.h>
#include <linux/i2c/pmic.h>
#include <linux/i2c.h>
#include <linux/spinlock.h>

#define ARM_VOLTAGE_MAX_INDEX	0x10
#define INT_VOLTAGE_MAX_INDEX	0x10	

/* prototypes. */
static int max8698_write(u8 reg, u8 value);
static int max8698_read(u8 reg);

/* global variables. */
static struct i2c_client *max8698_client = NULL;
static spinlock_t pmic_lock = SPIN_LOCK_UNLOCKED;

static const int arm_voltage_table[ARM_VOLTAGE_MAX_INDEX] = {
	750,			/* 0.75V */
	800,			/* 0.80V */
	850,			/* 0.85V */
	900,			/* 0.90V */
	950,			/* 0.95V */
	1000,			/* 1.00V */
	1050,			/* 1.05V */
	1100,			/* 1.10V */
	1150,			/* 1.15V */
	1200,			/* 1.20V */
	1250,			/* 1.25V */
	1300,			/* 1.30V */
	1350,			/* 1.35V */
	1400,			/* 1.40V */
	1450,			/* 1.45V */
	1500			/* 1.50V */
};

static const int int_voltage_table[INT_VOLTAGE_MAX_INDEX] = {
	750,			/* 0.75V */
	800,			/* 0.80V */
	850,			/* 0.85V */
	900,			/* 0.90V */
	950,			/* 0.95V */
	1000,			/* 1.00V */
	1050,			/* 1.05V */
	1100,			/* 1.10V */
	1150,			/* 1.15V */
	1200,			/* 1.20V */
	1250,			/* 1.25V */
	1300,			/* 1.30V */
	1350,			/* 1.35V */
	1400,			/* 1.40V */
	1450,			/* 1.45V */
	1500			/* 1.50V */
};

static int index_by_value(voltage_symbol_t vol_type, const int value)
{
	int i;
	
	switch (vol_type) {
	case VCC_ARM:
		for (i = 0; i < ARM_VOLTAGE_MAX_INDEX; i++) {
			if (arm_voltage_table[i] == value)
				return i;
		}

		break;

	case VCC_INT:
		for (i = 0; i < INT_VOLTAGE_MAX_INDEX; i++) {
			if (int_voltage_table[i] == value)
				return i;
		}

		break;
	}

	return -1;
}

static int change_vcc_arm(const int voltage)
{
	int ret;
	ret = index_by_value(VCC_ARM, voltage);

	if (ret < 0) {
		pr_err("%s: error!(%d)\n", __func__, ret);
		return -1;
	}

	return max8698_write(BUCK1_DVSARM12_REG, (u8)(ret & ~0xf0));
}

static int change_vcc_internal(const int voltage)
{
	int ret;
	ret = index_by_value(VCC_INT, voltage);

	if (ret < 0) {
		pr_err("%s: error!(%d)\n", __func__, ret);
		return -1;
	}

	return max8698_write(BUCK2_DVSINT12_REG, (u8)(ret & ~0xf0));
}

int set_pmic_voltage(voltage_symbol_t vol_type, const int value)
{
	int ret = -1;

	/*
	pr_info("VCC type: %d, requested voltage: %d\n", vol_type, value);
	*/

	switch (vol_type) {
	case VCC_ARM:
		ret = change_vcc_arm(value);
		break;
	case VCC_INT:
		ret = change_vcc_internal(value);
		break;
	}

	return ret;
}

int get_pmic_voltage(voltage_symbol_t vol_type, int *pvalue)
{
	int ret = -1;

	switch (vol_type) {
	case VCC_ARM:
		ret = max8698_read(BUCK1_DVSARM12_REG);

		if (ret > 0) {
			/* high 4bit DVSARM2, low 4bit DVSARM1 */
			ret &= ~0xf0;
			*pvalue = arm_voltage_table[ret];
		}

		break;

	case VCC_INT:
		ret = max8698_read(BUCK2_DVSINT12_REG);

		if (ret > 0) {
			/* high 4bit DVSINT2, low 4bit DVSINT1 */
			ret &= ~0xf0;
			*pvalue = int_voltage_table[ret];
		}

		break;
	}

	return ret;
}

EXPORT_SYMBOL(set_pmic_voltage);
EXPORT_SYMBOL(get_pmic_voltage);

/* .. */
static inline
void pmic_bit_control(u8 *pdata, u8 bit, int ctl)
{
	if (ctl == PMIC_POWER_ON)
		*pdata |= bit;
	else if (ctl == PMIC_POWER_OFF)
		*pdata &= ~bit;
}

int pmic_power_control(const int symbol, const int ctl)
{
	int reg, onoff;

	if (symbol == POWER_SYM_LDO1 || symbol == POWER_SYM_LDO2 ||
		symbol == POWER_SYM_LDO9) {
		pr_warning("you can't set LDO symbol(%d)!\n", symbol);
		return -1;
	}

	spin_lock(&pmic_lock);

	if (symbol >= POWER_SYM_LDO3 && symbol <= POWER_SYM_LDO5)
		reg = ONOFF1_REG;
	else if (symbol >= POWER_SYM_LDO6 && symbol <= POWER_SYM_LDO8)
		reg = ONOFF2_REG;
	else {
		spin_unlock(&pmic_lock);
		return -1;
	}

	onoff = max8698_read((u8)reg);

	if (onoff < 0) {
		pr_err("%s: error!(%d)\n", __func__, onoff);
		spin_unlock(&pmic_lock);

		return -1;
	}

	switch (symbol) {
	case POWER_SYM_LDO3:
		pmic_bit_control((u8 *)&onoff, ELDO3_BIT, ctl);
		break;
	case POWER_SYM_LDO4:
		pmic_bit_control((u8 *)&onoff, ELDO4_BIT, ctl);
		break;
	case POWER_SYM_LDO5:
		pmic_bit_control((u8 *)&onoff, ELDO5_BIT, ctl);
		break;
	case POWER_SYM_LDO6:
		pmic_bit_control((u8 *)&onoff, ELDO6_BIT, ctl);
		break;
	case POWER_SYM_LDO7:
		pmic_bit_control((u8 *)&onoff, ELDO7_BIT, ctl);
		break;
	case POWER_SYM_LDO8:
		pmic_bit_control((u8 *)&onoff, ELDO8_BIT, ctl);
		break;
	}

	spin_unlock(&pmic_lock);

	return max8698_write((u8)reg, (u8)onoff);
}

int pmic_power_read(const int symbol, int *pvalue)
{
	int reg, onoff, value = 0;

	if (symbol >= POWER_SYM_LDO3 && symbol <= POWER_SYM_LDO5)
		reg = ONOFF1_REG;
	else if (symbol >= POWER_SYM_LDO6 && symbol <= POWER_SYM_LDO8)
		reg = ONOFF2_REG;
	else
		return -1;

	onoff = max8698_read((u8)reg);

	if (onoff < 0) {
		pr_err("%s: error!(%d)\n", __func__, onoff);
		return -1;
	}

	switch (symbol) {
	case POWER_SYM_LDO3:
		value = onoff & ELDO3_BIT;
		break;
	case POWER_SYM_LDO4:
		value = onoff & ELDO4_BIT;
		break;
	case POWER_SYM_LDO5:
		value = onoff & ELDO5_BIT;
		break;
	case POWER_SYM_LDO6:
		value = onoff & ELDO6_BIT;
		break;
	case POWER_SYM_LDO7:
		value = onoff & ELDO7_BIT;
		break;
	case POWER_SYM_LDO8:
		value = onoff & ELDO8_BIT;
		break;
	}

	*pvalue = (value != 0) ? PMIC_POWER_ON : PMIC_POWER_OFF;

	return 0;
}

EXPORT_SYMBOL(pmic_power_control);
EXPORT_SYMBOL(pmic_power_read);

static int pmic_initial_regulation(void)
{
	pr_info("PMIC Initial Regulation!\n");

	if (max8698_write(BUCK1_DVSARM12_REG, 0xCC) < 0)  /* VAP_ARM */
		return -1;
	if (max8698_write(BUCK1_DVSARM34_REG, 0xCC) < 0)  /* VAP_ARM */
		return -1;
	if (max8698_write(BUCK2_DVSINT12_REG, 0xAA) < 0)  /* VAP_CORE */
		return -1;
	if (max8698_write(BUCK3_REG, 0x02) < 0)           /* VAP_MEM_1.8V */
		return -1;
	if (max8698_write(LDO2_LDO3_REG, 0x88) < 0)       /* VAP_ALIVE_1.2V + VAP_OTGI_1.2V */
		return -1;
	if (max8698_write(LDO4_REG, 0x0E) < 0)            /* VAP_3.0V_MOVI */
		return -1;
	if (max8698_write(LDO5_REG, 0x0E) < 0)            /* VAP_MMC_3.0V */
		return -1;
	if (max8698_write(LDO6_REG, 0x0E) < 0)            /* BT_3.0V */
		return -1;
	if (max8698_write(LDO7_REG, 0x0E) < 0)            /* VAP_LCD_3.0V */
		return -1;
	if (max8698_write(LDO8_BKCHR_REG, 0x33) < 0)      /* VAP_OTG_3.3V */
		return -1;
	if (max8698_write(LDO9_REG, 0x0E) < 0)            /* VAP_DAC_3.0V */
		return -1;

	return 0;
}

/* ------------------------------------------------------------ */
/*                      i2c interface                           */
/* ------------------------------------------------------------ */
static int max8698_write(u8 reg, u8 value)
{
	int ret;
	ret = i2c_smbus_write_byte_data(max8698_client, reg, value);

	if (ret < 0)
		pr_err("%s: error!(%d)\n", __func__, ret);

	return ret;
}

static int max8698_read(u8 reg)
{
	int ret;
	ret = i2c_smbus_read_byte_data(max8698_client, reg);

	if (ret < 0)
		pr_err("%s: error!(%d)\n", __func__, ret);

	return ret;
}

static int max8698_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	if (!client)
		return -EINVAL;

	max8698_client = client;

//bss	if (pmic_initial_regulation() < 0)
//bss		return -EIO;

	return 0;
}

static int max8698_remove(struct i2c_client *client)
{
	/* nothing.. */

	return 0;
}

static const struct i2c_device_id max8698_id[] = {
	{ "max8698", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max8698_id);

static struct i2c_driver max8698_i2c_driver = {
	.driver = {
		.name = "max8698",
		.owner = THIS_MODULE,
	},
	.probe = max8698_probe,
	.remove = max8698_remove,
	.id_table = max8698_id,
};

static int __init max8698_init(void)
{
	return i2c_add_driver(&max8698_i2c_driver);
}

static void __exit max8698_exit(void)
{
	i2c_del_driver(&max8698_i2c_driver);
}

module_init(max8698_init);
module_exit(max8698_exit);

MODULE_AUTHOR("Geun-Young, Kim <nenggun.kim@samsung.com>");
MODULE_DESCRIPTION("MAX8698 PMIC driver");
MODULE_LICENSE("GPL");

