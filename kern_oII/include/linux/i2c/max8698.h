/*
 * max8698.h - interface for MAX8698 PMIC.
 *
 * Copyright (C) 2009 Samsung Electronics.
 *
 * Author: Geun-Young, Kim <nenggun.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 */

#ifndef __MAX8698_H__
#define __MAX8698_H__

/* register address. */
#define ONOFF1_REG			0x00
#define ONOFF2_REG			0x01
#define ADISCHG_EN1_REG		0x02
#define ADISCHG_EN2_REG		0x03
#define BUCK1_DVSARM12_REG	0x04
#define BUCK1_DVSARM34_REG	0x05
#define BUCK2_DVSINT12_REG	0x06
#define BUCK3_REG			0x07
#define LDO2_LDO3_REG		0x08
#define LDO4_REG			0x09
#define LDO5_REG			0x0A
#define LDO6_REG			0x0B
#define LDO7_REG			0x0C
#define LDO8_BKCHR_REG		0x0D
#define LDO9_REG			0x0E
#define LBCNFG_REG			0x0F

/* ONOFF1 register bit. */
#define EN1_BIT				(1 << 7)
#define EN2_BIT				(1 << 6)
#define EN3_BIT				(1 << 5)
#define ELDO2_BIT			(1 << 4)
#define ELDO3_BIT			(1 << 3)
#define ELDO4_BIT			(1 << 2)
#define ELDO5_BIT			(1 << 1)

/* ONOFF2 register bit. */
#define ELDO6_BIT			(1 << 7)
#define ELDO7_BIT			(1 << 6)
#define ELDO8_BIT			(1 << 5)
#define ELDO9_BIT			(1 << 4)

/* voltage min & max value. */
#define MIN_VOLTAGE			750
#define MAX_VOLTAGE			1500

/* power symbols. */
#define POWER_SYM_LDO1		0
#define POWER_SYM_LDO2		1
#define POWER_SYM_LDO3		2
#define POWER_SYM_LDO4		3
#define POWER_SYM_LDO5		4
#define POWER_SYM_LDO6		5
#define POWER_SYM_LDO7		6
#define POWER_SYM_LDO8		7
#define POWER_SYM_LDO9		8

#endif	/* __MAX8698_H__ */


