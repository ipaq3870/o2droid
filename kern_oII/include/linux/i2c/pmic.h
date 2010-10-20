/*
 * pmic.h - interface for PMIC.
 *
 * Copyright (C) 2009 Samsung Electronics.
 *
 * Author: Geun-Young, Kim <nenggun.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 */

#ifndef __PMIC_H__
#define __PMIC_H__

#include <linux/i2c/max8698.h>
#include <mach/hardware.h>

#define POWER_RTC		POWER_SYM_LDO1
#define POWER_ALIVE		POWER_SYM_LDO2
#define POWER_USB_OTGi	POWER_SYM_LDO3
#define POWER_MOVINAND	POWER_SYM_LDO4
#define POWER_MMC		POWER_SYM_LDO5

#define POWER_BT		POWER_SYM_LDO6

#define POWER_LCD		POWER_SYM_LDO7
#define POWER_USB_OTG	POWER_SYM_LDO8
#define POWER_DAC		POWER_SYM_LDO9

#define PMIC_POWER_ON	1
#define PMIC_POWER_OFF	0

typedef enum _voltage_symbol {
	VCC_ARM = 0,
	VCC_INT
} voltage_symbol_t;

extern int pmic_power_control(const int symbol, const int ctl);
extern int pmic_power_read(const int symbol, int *pvalue);

extern int set_pmic_voltage(voltage_symbol_t vol_type, const int value);
extern int get_pmic_voltage(voltage_symbol_t vol_type, int *pvalue);

#endif	/* __PMIC_H__ */


