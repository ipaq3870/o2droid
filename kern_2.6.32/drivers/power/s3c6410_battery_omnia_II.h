/*
 * linux/drivers/power/s3c6410_battery.h
 *
 * Battery measurement code for S3C6410 platform.
 *
 * Copyright (C) 2009 Samsung Electronics.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#define DRIVER_NAME	"omnia-II-battery"

/*
 * Omnia_II Rev01 board Temperature Table
 */
#define NUM_TEMP_TBL    69
const int temper_table[][2] =  {
	/* ADC, Temperature (C) */
	{	1640 	,	-50	},
	{	1626 	,	-40	},
	{	1612 	,	-30	},
	{	1598 	,	-20	},
	{	1584 	,	-10	},
	{	1570 	,	0	},
	{	1545	,	10	},
	{	1520	,	20	},
	{	1495	,	30	},
	{	1470	,	40	},
	{	1445	,	50	},
	{	1420	,	60	},
	{	1395	,	70	},
	{	1370	,	80	},
	{	1345	,	90	},
	{	1321	,	100 },
	{	1296	,	110 },
	{	1271	,	120 },
	{	1246	,	130 },
	{	1221	,	140 },
	{	1196	,	150 },
	{	1171	,	160 },
	{	1146	,	170 },
	{	1121	,	180 },
	{	1096	,	190 },
	{	1071	,	200 },
	{	1046	,	210 },
	{	1021	,	220 },
	{	996 	,	230 },
	{	971 	,	240 },
	{	946 	,	250 },
	{	921 	,	260 },
	{	896 	,	270 },
	{	871 	,	280 },
	{	846 	,	290 },
	{	822 	,	300 },
	{	797 	,	310 },
	{	772 	,	320 },
	{	747 	,	330 },
	{	722 	,	340 },
	{	697 	,	350 },
	{	672 	,	360 },
	{	647 	,	370 },
	{	622 	,	380 },
	{	597 	,	390 },
	{	572 	,	400	},
	{	561 	,	410	},
	{	549 	,	420 },
	{	538 	,	430	},
	{	526 	,	440 },
	{	515 	,	450 },
	{	503 	,	460 },
	{	492 	,	470 },
	{	480 	,	480 },
	{	469 	,	490 },
	{	457 	,	500 },
	{	446 	,	510 },
	{	434 	,	520 },
	{	423 	,	530 },
	{	411 	,	540 },
	{	400 	,	550 },
	{	388 	,	560 },
	{	377 	,	570 },
	{	365 	,	580	},
	{	357 	,	590	},
	{	349 	,	600	},
	{	341 	,	610	},
	{	333 	,	620	},
	{	325 	,	630	},
};

#define TEMP_HIGH_BLOCK		temper_table[50][0]
#define TEMP_HIGH_RECOVER	temper_table[48][0]
#define TEMP_LOW_BLOCK		temper_table[0][0]
#define TEMP_LOW_RECOVER	temper_table[5][0]
#define TEMP_RCOMP		temper_table[24][1]		// 20.0C

/*
 * Omnia_II Rev00 board ADC channel
 */
typedef enum s3c_adc_channel {
	S3C_ADC_VOLTAGE = 0,
	S3C_ADC_TEMPERATURE,
	S3C_ADC_V_F,
	S3C_ADC_EAR,
	ENDOFADC
} adc_channel_type;

#define IRQ_TA_CONNECTED_N	IRQ_EINT(19)
#define IRQ_TA_CHG_N		IRQ_EINT(25)

/*
 * Omnia_II GPIO for battery driver
 */

const unsigned int gpio_ta_connected	= GPIO_TA_nCONNECTED;
const unsigned int gpio_ta_connected_af	= GPIO_TA_nCONNECTED_AF;
const unsigned int gpio_chg_ing		= GPIO_TA_nCHG;
const unsigned int gpio_chg_ing_af	= GPIO_TA_nCHG_AF;
const unsigned int gpio_chg_en		= GPIO_TA_EN;
const unsigned int gpio_chg_en_af	= GPIO_TA_EN_AF;

/******************************************************************************
 * Battery driver features
 * ***************************************************************************/
/* #define __USE_EGPIO__ */
#define __CHECK_BATTERY_V_F__
#define __BATTERY_COMPENSATION__
#define __TEST_DEVICE_DRIVER__
//#define __ALWAYS_AWAKE_DEVICE__ 
#define __TEST_MODE_INTERFACE__
/*****************************************************************************/

#define TOTAL_CHARGING_TIME	(5*60*60*1000)	/* 5 hours */
#define TOTAL_RECHARGING_TIME	(2*60*60*1000)	/* 2 hours */

#define RECHARGE_COND_VOLTAGE 4050	/* 4.13V */	
#define FULL_CHARGE_COND_VOLTAGE 4190	/* 4.19V */

#ifdef __CHECK_BATTERY_V_F__
#define BATT_VF_MAX		35	
#define BATT_VF_MIN		0
#endif /* __CHECK_BATTERY_V_F__ */

#ifdef __BATTERY_COMPENSATION__
#if defined (__FUEL_GAUGES_IC__)
#define COMPENSATE_VIBRATOR		0
#define COMPENSATE_CAMERA		0//20 /* 20mV */
#define COMPENSATE_MP3			0
#define COMPENSATE_VIDEO		0//20 /* 20mV */
#define COMPENSATE_VOICE_CALL_2G	0//20 /* 20mV */
#define COMPENSATE_VOICE_CALL_3G	0//20 /* 20mV */
#define COMPENSATE_DATA_CALL		0//20 /* 20mV */
#define COMPENSATE_LCD			0
#define COMPENSATE_TA			0//(-70) /* 70mV */
#define COMPENSATE_CAM_FALSH		0
#define COMPENSATE_BOOTING		0
#define COMPENSATE_BROWSER      0
#define COMPENSATE_WIMAX      0
#else
#define COMPENSATE_VIBRATOR		2
#define COMPENSATE_CAMERA		2
#define COMPENSATE_MP3			2
#define COMPENSATE_VIDEO		1
#define COMPENSATE_VOICE_CALL_2G	3
#define COMPENSATE_VOICE_CALL_3G	3
#define COMPENSATE_DATA_CALL		3
#define COMPENSATE_LCD			3
#define COMPENSATE_TA			(-9)
#define COMPENSATE_CAM_FALSH		3
#define COMPENSATE_BOOTING		9
#define COMPENSATE_BROWSER      3
#define COMPENSATE_WIMAX        3
#endif /* __FUEL_GAUGES_IC__ */
#endif /* __BATTERY_COMPENSATION__ */

