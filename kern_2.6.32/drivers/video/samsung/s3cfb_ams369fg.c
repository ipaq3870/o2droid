/*
 * drivers/video/samsung/s3cfb_ams369FG.c
 * Based upon s3cfb_ams320FS01.c
 * Copyright (C) 2008 Jinsung Yang <jsgood.yang@samsung.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	S3C Frame Buffer Driver
 *	based on skeletonfb.c, sa1100fb.h, s3c2410fb.c
 */

#include <linux/wait.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/leds.h>

#include <linux/i2c/maximi2c.h>
#include <linux/i2c/pmic.h>

#include <plat/gpio-cfg.h>
#include <plat/regs-gpio.h>
#include <plat/regs-lcd.h>

#include <mach/hardware.h>

#include "s3cfb.h"

#include <linux/miscdevice.h>
#include "s3cfb_ams320fs01_ioctl.h"
#include "AMS369FG06.h"
#include "omnia2_bootlogo.h"

#include <mach/param.h>
#include <mach/gpio.h>

/*BACKLIGHT*/
#define BACKLIGHT_STATUS_ALC	0x100
#define BACKLIGHT_LEVEL_VALUE	0x0FF	/* 0 ~ 255 */

#define BACKLIGHT_LEVEL_MIN		0
#define BACKLIGHT_LEVEL_MAX	 BACKLIGHT_LEVEL_VALUE //255

#define BACKLIGHT_LEVEL_DEFAULT	100		/* Default Setting */
#define AMS320FS01_DEFAULT_BACKLIGHT_BRIGHTNESS		255

#define OFFSET_LCD_ON           (0x1 << 7)

/*SPI INTERFACE*/
#define LCD_CS_N_HIGH	gpio_set_value(GPIO_LCD_CS_N, GPIO_LEVEL_HIGH); //CSB
#define LCD_CS_N_LOW	gpio_set_value(GPIO_LCD_CS_N, GPIO_LEVEL_LOW);

#define LCD_SCLK_HIGH	gpio_set_value(GPIO_LCD_SCLK, GPIO_LEVEL_HIGH); //SCL
#define LCD_SCLK_LOW	gpio_set_value(GPIO_LCD_SCLK, GPIO_LEVEL_LOW);

#define LCD_SDI_HIGH	gpio_set_value(GPIO_LCD_SDI, GPIO_LEVEL_HIGH); //SDI
#define LCD_SDI_LOW	    gpio_set_value(GPIO_LCD_SDI, GPIO_LEVEL_LOW);

#define DEFAULT_UDELAY	10	//WAS 5

/* LCD SETTINGS */
#define S3C_FB_HFP			8 		/* Front Porch */
#define S3C_FB_HSW			1 		/* Hsync Width */
#define S3C_FB_HBP			7 		/* Back Porch */

#define S3C_FB_VFP			8 		/* Front Porch */
#define S3C_FB_VSW			1 		/* Vsync Width */
#define S3C_FB_VBP			7 		/* Back Porch */

#define S3C_FB_HRES             480     /* Horizon pixel Resolition */
#define S3C_FB_VRES             800     /* Vertical pixel Resolution */
#define S3C_FB_HRES_VIRTUAL     S3C_FB_HRES     /* Horizon pixel Resolition */
#define S3C_FB_VRES_VIRTUAL     S3C_FB_VRES * 2 /* Vertial pixel Resolution */

#define S3C_FB_HRES_OSD         480     /* Horizon pixel Resolition */
#define S3C_FB_VRES_OSD         800     /* Vertial pixel Resolution */
#define S3C_FB_HRES_OSD_VIRTUAL S3C_FB_HRES_OSD     /* Horizon pixel Resolition */
#define S3C_FB_VRES_OSD_VIRTUAL S3C_FB_VRES_OSD * 2 /* Vertial pixel Resolution */

#define S3C_FB_VFRAME_FREQ  	60		/* Frame Rate Frequency */

#define S3C_FB_PIXEL_CLOCK		(S3C_FB_VFRAME_FREQ * \
								(S3C_FB_HFP + S3C_FB_HSW + S3C_FB_HBP + S3C_FB_HRES) * \
								(S3C_FB_VFP + S3C_FB_VSW + S3C_FB_VBP + S3C_FB_VRES))
/* PMIC SETTINGS */
#define MAX8698_ID		0xCC

#define ONOFF2			0x01

#define ONOFF2_ELDO6	(0x01 << 7)
#define ONOFF2_ELDO7	(0x03 << 6)

#define LOGO_MEM_SIZE		(S3C_FB_HRES * S3C_FB_VRES * 2)


typedef enum {
	LCD_IDLE = 0,
	LCD_VIDEO,
	LCD_CAMERA
} lcd_gamma_status;


extern void s3cfb_enable_clock_power(void);
extern int s3cfb_is_clock_on(void);
extern int lcd_late_resume;


void lcd_reset(void)
{
	gpio_set_value(GPIO_LCD_RST_N, GPIO_LEVEL_LOW);
};
EXPORT_SYMBOL(lcd_reset);

int lcd_power = OFF;
EXPORT_SYMBOL(lcd_power);

void lcd_power_ctrl(s32 value);
EXPORT_SYMBOL(lcd_power_ctrl);

int backlight_power = OFF;
EXPORT_SYMBOL(backlight_power);

void backlight_power_ctrl(s32 value);
EXPORT_SYMBOL(backlight_power_ctrl);

int backlight_level = BACKLIGHT_LEVEL_DEFAULT;
EXPORT_SYMBOL(backlight_level);

void backlight_level_ctrl(s32 value);
EXPORT_SYMBOL(backlight_level_ctrl);

static s32 old_level = 0;
int lcd_gamma_present = 0;

static s32 ams320fs01_backlight_brightness = AMS320FS01_DEFAULT_BACKLIGHT_BRIGHTNESS;
static DEFINE_MUTEX(ams320fs01_backlight_lock);


static void s3cfb_set_fimd_info(void)
{
	s3c_fimd.dithmode	= S3C_DITHMODE_RDITHPOS_6BIT |
				  S3C_DITHMODE_GDITHPOS_6BIT |
				  S3C_DITHMODE_BDITHPOS_6BIT |
				  S3C_DITHMODE_DITHERING_DISABLE;

	s3c_fimd.vidcon1	= S3C_VIDCON1_IHSYNC_INVERT |
							S3C_VIDCON1_IVSYNC_INVERT |
							S3C_VIDCON1_IVDEN_INVERT|S3C_VIDCON1_IVCLK_RISE_EDGE;

	s3c_fimd.vidtcon0 	= S3C_VIDTCON0_VBPD(S3C_FB_VBP - 1) |
							S3C_VIDTCON0_VFPD(S3C_FB_VFP - 1) |
							S3C_VIDTCON0_VSPW(S3C_FB_VSW - 1);
	s3c_fimd.vidtcon1	= S3C_VIDTCON1_HBPD(S3C_FB_HBP - 1) |
							S3C_VIDTCON1_HFPD(S3C_FB_HFP - 1) |
							S3C_VIDTCON1_HSPW(S3C_FB_HSW - 1);
	s3c_fimd.vidtcon2	= S3C_VIDTCON2_LINEVAL(S3C_FB_VRES - 1) |
							S3C_VIDTCON2_HOZVAL(S3C_FB_HRES - 1);

	s3c_fimd.vidosd0a 	= S3C_VIDOSDxA_OSD_LTX_F(0) |
							S3C_VIDOSDxA_OSD_LTY_F(0);
	s3c_fimd.vidosd0b 	= S3C_VIDOSDxB_OSD_RBX_F(S3C_FB_HRES - 1) |
							S3C_VIDOSDxB_OSD_RBY_F(S3C_FB_VRES - 1);

	s3c_fimd.vidosd1a 	= S3C_VIDOSDxA_OSD_LTX_F(0) |
							S3C_VIDOSDxA_OSD_LTY_F(0);
	s3c_fimd.vidosd1b 	= S3C_VIDOSDxB_OSD_RBX_F(S3C_FB_HRES_OSD - 1) |
							S3C_VIDOSDxB_OSD_RBY_F(S3C_FB_VRES_OSD - 1);

	s3c_fimd.width		= 50;//OLD: S3C_FB_HRES;
	s3c_fimd.height 	= 80;//OLD: S3C_FB_VRES;

	s3c_fimd.xres 		= S3C_FB_HRES;
	s3c_fimd.yres 		= S3C_FB_VRES;

#if defined(CONFIG_FB_S3C_VIRTUAL_SCREEN)
	s3c_fimd.xres_virtual = S3C_FB_HRES_VIRTUAL;
	s3c_fimd.yres_virtual = S3C_FB_VRES_VIRTUAL;
#elif defined(CONFIG_FB_S3C_DOUBLE_BUFFERING)
	s3c_fimd.xres_virtual = S3C_FB_HRES;
	s3c_fimd.yres_virtual = S3C_FB_VRES * 2;
#else
	s3c_fimd.xres_virtual = S3C_FB_HRES;
	s3c_fimd.yres_virtual = S3C_FB_VRES;
#endif

	s3c_fimd.osd_width 	= S3C_FB_HRES_OSD;
	s3c_fimd.osd_height = S3C_FB_VRES_OSD;
	s3c_fimd.osd_xres 	= S3C_FB_HRES_OSD;
	s3c_fimd.osd_yres 	= S3C_FB_VRES_OSD;

#if defined(CONFIG_FB_S3C_VIRTUAL_SCREEN)
	s3c_fimd.osd_xres_virtual = S3C_FB_HRES_OSD_VIRTUAL;
	s3c_fimd.osd_yres_virtual = S3C_FB_VRES_OSD_VIRTUAL;
#elif defined(CONFIG_FB_S3C_DOUBLE_BUFFERING)
	s3c_fimd.osd_xres_virtual = S3C_FB_HRES_OSD;
	s3c_fimd.osd_yres_virtual = S3C_FB_VRES_OSD * 2;
#else
	s3c_fimd.osd_xres_virtual = S3C_FB_HRES_OSD;
	s3c_fimd.osd_yres_virtual = S3C_FB_VRES_OSD;
#endif

    s3c_fimd.pixclock		= S3C_FB_PIXEL_CLOCK;

	s3c_fimd.hsync_len 		= S3C_FB_HSW;
	s3c_fimd.vsync_len 		= S3C_FB_VSW;
	s3c_fimd.left_margin 	= S3C_FB_HFP;
	s3c_fimd.upper_margin 	= S3C_FB_VFP;
	s3c_fimd.right_margin 	= S3C_FB_HBP;
	s3c_fimd.lower_margin 	= S3C_FB_VBP;

	s3c_fimd.set_lcd_power		 = lcd_power_ctrl;
	s3c_fimd.set_backlight_power = backlight_power_ctrl;
	s3c_fimd.set_brightness 	 = backlight_level_ctrl;

	s3c_fimd.backlight_min = BACKLIGHT_LEVEL_MIN;
	s3c_fimd.backlight_max = BACKLIGHT_LEVEL_MAX;
}

static void lcd_gpio_init(void)
{
	/* B(0:7) */
	s3c_gpio_cfgpin(GPIO_LCD_B_0, S3C_GPIO_SFN(GPIO_LCD_B_0_AF));
	s3c_gpio_cfgpin(GPIO_LCD_B_1, S3C_GPIO_SFN(GPIO_LCD_B_1_AF));
	s3c_gpio_cfgpin(GPIO_LCD_B_2, S3C_GPIO_SFN(GPIO_LCD_B_2_AF));
	s3c_gpio_cfgpin(GPIO_LCD_B_3, S3C_GPIO_SFN(GPIO_LCD_B_3_AF));
	s3c_gpio_cfgpin(GPIO_LCD_B_4, S3C_GPIO_SFN(GPIO_LCD_B_4_AF));
	s3c_gpio_cfgpin(GPIO_LCD_B_5, S3C_GPIO_SFN(GPIO_LCD_B_5_AF));
	s3c_gpio_cfgpin(GPIO_LCD_B_6, S3C_GPIO_SFN(GPIO_LCD_B_6_AF));
	s3c_gpio_cfgpin(GPIO_LCD_B_7, S3C_GPIO_SFN(GPIO_LCD_B_7_AF));
	/* G(0:7) */
	s3c_gpio_cfgpin(GPIO_LCD_G_0, S3C_GPIO_SFN(GPIO_LCD_G_0_AF));
	s3c_gpio_cfgpin(GPIO_LCD_G_1, S3C_GPIO_SFN(GPIO_LCD_G_1_AF));
	s3c_gpio_cfgpin(GPIO_LCD_G_2, S3C_GPIO_SFN(GPIO_LCD_G_2_AF));
	s3c_gpio_cfgpin(GPIO_LCD_G_3, S3C_GPIO_SFN(GPIO_LCD_G_3_AF));
	s3c_gpio_cfgpin(GPIO_LCD_G_4, S3C_GPIO_SFN(GPIO_LCD_G_4_AF));
	s3c_gpio_cfgpin(GPIO_LCD_G_5, S3C_GPIO_SFN(GPIO_LCD_G_5_AF));
	s3c_gpio_cfgpin(GPIO_LCD_G_6, S3C_GPIO_SFN(GPIO_LCD_G_6_AF));
	s3c_gpio_cfgpin(GPIO_LCD_G_7, S3C_GPIO_SFN(GPIO_LCD_G_7_AF));
	/* R(0:7) */
	s3c_gpio_cfgpin(GPIO_LCD_R_0, S3C_GPIO_SFN(GPIO_LCD_R_0_AF));
	s3c_gpio_cfgpin(GPIO_LCD_R_1, S3C_GPIO_SFN(GPIO_LCD_R_1_AF));
	s3c_gpio_cfgpin(GPIO_LCD_R_2, S3C_GPIO_SFN(GPIO_LCD_R_2_AF));
	s3c_gpio_cfgpin(GPIO_LCD_R_3, S3C_GPIO_SFN(GPIO_LCD_R_3_AF));
	s3c_gpio_cfgpin(GPIO_LCD_R_4, S3C_GPIO_SFN(GPIO_LCD_R_4_AF));
	s3c_gpio_cfgpin(GPIO_LCD_R_5, S3C_GPIO_SFN(GPIO_LCD_R_5_AF));
	s3c_gpio_cfgpin(GPIO_LCD_R_6, S3C_GPIO_SFN(GPIO_LCD_R_6_AF));
	s3c_gpio_cfgpin(GPIO_LCD_R_7, S3C_GPIO_SFN(GPIO_LCD_R_7_AF));
	/* HSYNC */
	s3c_gpio_cfgpin(GPIO_LCD_HSYNC, S3C_GPIO_SFN(GPIO_LCD_HSYNC_AF));
	/* VSYNC */
	s3c_gpio_cfgpin(GPIO_LCD_VSYNC, S3C_GPIO_SFN(GPIO_LCD_VSYNC_AF));
	/* DE */
	s3c_gpio_cfgpin(GPIO_LCD_DE, S3C_GPIO_SFN(GPIO_LCD_DE_AF));
	/* CLK */
	s3c_gpio_cfgpin(GPIO_LCD_CLK, S3C_GPIO_SFN(GPIO_LCD_CLK_AF));

	/* LCD_RST_N */
	if (gpio_is_valid(GPIO_LCD_RST_N)) {
		if (gpio_request(GPIO_LCD_RST_N, S3C_GPIO_LAVEL(GPIO_LCD_RST_N))) 
			printk(KERN_ERR "Failed to request GPIO_LCD_RST_N!\n");
		gpio_direction_output(GPIO_LCD_RST_N, GPIO_LEVEL_HIGH);
	}
	s3c_gpio_setpull(GPIO_LCD_RST_N, S3C_GPIO_PULL_NONE);
	/* LCD_ID */
	if (gpio_is_valid(GPIO_LCD_ID)) {
		if (gpio_request(GPIO_LCD_ID, S3C_GPIO_LAVEL(GPIO_LCD_ID))) 
			printk(KERN_ERR "Failed to request GPIO_LCD_ID!\n");
		gpio_direction_input(GPIO_LCD_ID);
	}
	s3c_gpio_setpull(GPIO_LCD_ID, S3C_GPIO_PULL_NONE);
	/* LCD_SCLK */
	if (gpio_is_valid(GPIO_LCD_SCLK)) {
		if (gpio_request(GPIO_LCD_SCLK, S3C_GPIO_LAVEL(GPIO_LCD_SCLK))) 
			printk(KERN_ERR "Failed to request GPIO_LCD_SCLK!\n");
		gpio_direction_output(GPIO_LCD_SCLK, GPIO_LEVEL_HIGH);
	}
	s3c_gpio_setpull(GPIO_LCD_SCLK, S3C_GPIO_PULL_NONE);
	/* LCD_CS_N */
	if (gpio_is_valid(GPIO_LCD_CS_N)) {
		if (gpio_request(GPIO_LCD_CS_N, S3C_GPIO_LAVEL(GPIO_LCD_CS_N))) 
			printk(KERN_ERR "Failed to request GPIO_LCD_CS_N!\n");
		gpio_direction_output(GPIO_LCD_CS_N, GPIO_LEVEL_HIGH);
	}
	s3c_gpio_setpull(GPIO_LCD_CS_N, S3C_GPIO_PULL_NONE);
	/* LCD_SDO */ //????? 
	if (gpio_is_valid(GPIO_LCD_SDO)) {
		if (gpio_request(GPIO_LCD_SDO, S3C_GPIO_LAVEL(GPIO_LCD_SDO))) 
			printk(KERN_ERR "Failed to request GPIO_LCD_SDO!\n");
		gpio_direction_output(GPIO_LCD_SDO, GPIO_LEVEL_HIGH);
	}
	s3c_gpio_setpull(GPIO_LCD_SDO, S3C_GPIO_PULL_NONE);
	// LCD_SDI /
	if (gpio_is_valid(GPIO_LCD_SDI)) {
		if (gpio_request(GPIO_LCD_SDI, S3C_GPIO_LAVEL(GPIO_LCD_SDI))) 
			printk(KERN_ERR "Failed to request GPIO_LCD_SDI!\n");
		gpio_direction_output(GPIO_LCD_SDI, GPIO_LEVEL_HIGH);
	}
	s3c_gpio_setpull(GPIO_LCD_SDI, S3C_GPIO_PULL_NONE);
}

static void spi_write(u16 reg_data)
{	
	s32 i;
	u8 ID, ID2,reg_data1, reg_data2;
	reg_data1 = (reg_data >> 8); // last byte
	reg_data2 = reg_data; //firt byte
	ID=0x70;
	ID2=0x72;

	
	LCD_SCLK_HIGH
	udelay(DEFAULT_UDELAY);
	LCD_CS_N_HIGH
	udelay(DEFAULT_UDELAY);
	LCD_CS_N_LOW
	udelay(DEFAULT_UDELAY);
	 LCD_SCLK_HIGH
	 udelay(DEFAULT_UDELAY);

	for (i = 7; i >= 0; i--) { 
		LCD_SCLK_LOW
		udelay(DEFAULT_UDELAY);

		if ((ID >> i) & 0x1)
			LCD_SDI_HIGH
		else
			LCD_SDI_LOW
		udelay(DEFAULT_UDELAY);	
		LCD_SCLK_HIGH
		udelay(DEFAULT_UDELAY);
	}
	for (i = 7; i >= 0; i--) { 
		LCD_SCLK_LOW
		udelay(DEFAULT_UDELAY);

		if ((reg_data1 >> i) & 0x1) //only the first byte, L->R
			LCD_SDI_HIGH
		else
			LCD_SDI_LOW
		udelay(DEFAULT_UDELAY);
		LCD_SCLK_HIGH
		udelay(DEFAULT_UDELAY);	
	}

	LCD_SCLK_HIGH
	udelay(DEFAULT_UDELAY);
	LCD_SDI_HIGH
	udelay(DEFAULT_UDELAY);
	LCD_CS_N_HIGH
	udelay(DEFAULT_UDELAY);

	LCD_SCLK_HIGH
	udelay(DEFAULT_UDELAY);
	LCD_CS_N_HIGH
	udelay(DEFAULT_UDELAY);
	LCD_CS_N_LOW
	udelay(DEFAULT_UDELAY);


	for (i = 7; i >= 0; i--) { 
		LCD_SCLK_LOW
		udelay(DEFAULT_UDELAY);

		if ((ID2 >> i) & 0x1)
			LCD_SDI_HIGH
		else
			LCD_SDI_LOW
		udelay(DEFAULT_UDELAY);
		LCD_SCLK_HIGH
		udelay(DEFAULT_UDELAY);	
	}
	for (i = 7; i >= 0; i--) { 
		LCD_SCLK_LOW
		udelay(DEFAULT_UDELAY);

		if ((reg_data2 >> i) & 0x1)
			LCD_SDI_HIGH
		else
			LCD_SDI_LOW
		udelay(DEFAULT_UDELAY);

		LCD_SCLK_HIGH
		udelay(DEFAULT_UDELAY);	
	}
	

	LCD_SCLK_HIGH
	udelay(DEFAULT_UDELAY);
	LCD_SDI_HIGH
	udelay(DEFAULT_UDELAY);
	LCD_CS_N_HIGH
	udelay(DEFAULT_UDELAY);

}

static void spi_write2(u8 reg_data)
{	
	s32 i;
	u8 ID;
	ID=0x72;

	
	LCD_SCLK_HIGH
	udelay(DEFAULT_UDELAY);
	LCD_CS_N_HIGH
	udelay(DEFAULT_UDELAY);
	LCD_CS_N_LOW
	udelay(DEFAULT_UDELAY);
	 LCD_SCLK_HIGH
	 udelay(DEFAULT_UDELAY);

	for (i = 7; i >= 0; i--) { 
		LCD_SCLK_LOW
		udelay(DEFAULT_UDELAY);

		if ((ID >> i) & 0x1)
			LCD_SDI_HIGH
		else
			LCD_SDI_LOW
		udelay(DEFAULT_UDELAY);	
		LCD_SCLK_HIGH
		udelay(DEFAULT_UDELAY);
	}
	for (i = 7; i >= 0; i--) { 
		LCD_SCLK_LOW
		udelay(DEFAULT_UDELAY);

		if ((reg_data >> i) & 0x1) //only the first byte, L->R
			LCD_SDI_HIGH
		else
			LCD_SDI_LOW
		udelay(DEFAULT_UDELAY);
		LCD_SCLK_HIGH
		udelay(DEFAULT_UDELAY);	
	}

	LCD_SCLK_HIGH
	udelay(DEFAULT_UDELAY);
	LCD_SDI_HIGH
	udelay(DEFAULT_UDELAY);
	LCD_CS_N_HIGH
	udelay(DEFAULT_UDELAY);

}

static void setting_table_write(struct setting_table *table)
{
//	printk("setting table write! \n");
	spi_write(table->reg_data);
	if(table->wait)
		msleep(table->wait);
}


void lcd_gamma_change(int gamma_status)
{
	int i;

	if(old_level < 1)
		return;
		
	//printk("[S3C LCD] %s : level %d, gamma_status %d\n", __FUNCTION__, (old_level-1), gamma_status);


	if(gamma_status == LCD_IDLE)
	{
		lcd_gamma_present = LCD_IDLE;
		for (i = 0; i < GAMMA_SETTINGS; i++)
			setting_table_write(&gamma_setting_table[(old_level - 1)][i]);
	}
	else if(gamma_status == LCD_VIDEO)
	{
		lcd_gamma_present = LCD_VIDEO;
		for (i = 0; i < GAMMA_SETTINGS; i++)
			setting_table_write(&gamma_setting_table_video[(old_level - 1)][i]);
	}
	else if(gamma_status == LCD_CAMERA)
	{
		lcd_gamma_present = LCD_CAMERA;
		for (i = 0; i < GAMMA_SETTINGS; i++)
			setting_table_write(&gamma_setting_table_cam[(old_level - 1)][i]);
	}
	else
		return;
}

EXPORT_SYMBOL(lcd_gamma_change);

void lcd_power_ctrl(s32 value)
{

	//printk("lcd_power_ctrl value=%x \n", value);
	s32 i;	
	u8 data;
	u32 timeout = 100;
	if (value) {
	/*	while (timeout-- > 0) {
			if (lcd_late_resume == 1)
				break;
			msleep(50);
		} 
		
		if (timeout == -1) {
			printk(KERN_ERR "lcd power control time out\n");
			//return -1;
		}

		if (lcd_late_resume == 0)
			return;
*/
		//printk("Lcd power on sequence start\n");

		/* Power On Sequence */

	
			/* Power Enable */
			if(pmic_read(MAX8698_ID, ONOFF2, &data, 1) != PMIC_PASS) {
				printk(KERN_ERR "LCD POWER CONTROL can't read the status from PMIC\n");
				//return -1;
			}
			data |= (ONOFF2_ELDO6);
			
			if(pmic_write(MAX8698_ID, ONOFF2, &data, 1) != PMIC_PASS) {
				printk(KERN_ERR "LCD POWER CONTROL can't write the command to PMIC\n");
			//return -1;
			}
			msleep(1);

			data |= (ONOFF2_ELDO7);
			
			if(pmic_write(MAX8698_ID, ONOFF2, &data, 1) != PMIC_PASS) {
				printk(KERN_ERR "LCD POWER CONTROL can't write the command to PMIC\n");
			//return -1;
			}
			gpio_set_value(GPIO_LCD_RST_N, GPIO_LEVEL_HIGH);
			udelay(50);
			gpio_set_value(GPIO_LCD_RST_N, GPIO_LEVEL_LOW);
			udelay(50);
			gpio_set_value(GPIO_LCD_RST_N, GPIO_LEVEL_HIGH);

			msleep(20);		
	
			for (i = 0; i < POWER_ON_SETTINGS; i++)
				setting_table_write(&power_on_setting_table[i]);
			spi_write2(0xE8); // 3rd value

	

			switch(lcd_gamma_present)
			{
				printk("[S3C LCD] %s : level dimming, lcd_gamma_present %d\n", __FUNCTION__, lcd_gamma_present);
				spi_write(0x3944); //set gamma have to check!

				case LCD_IDLE:
					for (i = 0; i < GAMMA_SETTINGS; i++)
						setting_table_write(&gamma_setting_table[1][i]);
					break;
				case LCD_VIDEO:
					for (i = 0; i < GAMMA_SETTINGS; i++)
						setting_table_write(&gamma_setting_table_video[1][i]);
					break;
				case LCD_CAMERA:
					for (i = 0; i < GAMMA_SETTINGS; i++)
						setting_table_write(&gamma_setting_table_cam[1][i]);
					break;
				default:
					break;
			}
			
			for (i = 0; i < DISPLAY_ON_SETTINGS; i++)
				setting_table_write(&display_on_setting_table[i]);	
			//printk("Lcd power on sequence end\n");

}
		else {
			//printk("Lcd power off sequence start\n");
			/* Power Off Sequence */

			for (i = 0; i < DISPLAY_OFF_SETTINGS; i++)
				setting_table_write(&display_off_setting_table[i]);	
			
			/* Reset Assert */
			gpio_set_value(GPIO_LCD_RST_N, GPIO_LEVEL_LOW);

			/* Power Disable */
			if(pmic_read(MAX8698_ID, ONOFF2, &data, 1) != PMIC_PASS) {
				printk(KERN_ERR "LCD POWER CONTROL can't read the status from PMIC\n");
				//return -1;
			}
			data &= ~(ONOFF2_ELDO7);
		
			if(pmic_write(MAX8698_ID, ONOFF2, &data, 1) != PMIC_PASS) {
				printk(KERN_ERR "LCD POWER CONTROL can't write the command to PMIC\n");
				//return -1;
			}
			msleep(1);
			data &= ~(ONOFF2_ELDO6);
		
			if(pmic_write(MAX8698_ID, ONOFF2, &data, 1) != PMIC_PASS) {
				printk(KERN_ERR "LCD POWER CONTROL can't write the command to PMIC\n");
				//return -1;
			}
			msleep(1);
			printk("Lcd power off sequence end\n");
		}

		lcd_power = value;
}


void backlight_ctrl(s32 value)
{

	//printk("backlight_ctrl is called VALUE = %x \n", value);
	s32 i, level;

	value &= BACKLIGHT_LEVEL_VALUE;

	if (value == 0)
		level = 0;
	else if ((value > 0) && (value < 15))//15
		level = 1;
	else	
		level = (((value - 15) / 15) + 1); //max = 16 = 255
	if (value > 240)
		level = 16;

	if (level) {	
	//printk(" backlight_ctrl level:%x, old_level:%x &value=%x\n", level, old_level, value);		
		if (level != old_level) {
			old_level = level;

			if (lcd_power == OFF)
			{
				//printk("LCD power == OFF \n");
				if(!s3cfb_is_clock_on())
				{
					//printk("s3cfb_enable_clock_power \n");
					s3cfb_enable_clock_power();
				}
				//printk("lcd_power_ctrl(ON) \n");
				lcd_power_ctrl(ON);
			}

			//printk("LCD Backlight level setting value ==> %d  , level ==> %d \n",value,level);
			
			switch(lcd_gamma_present)
			{
				//printk("[S3C LCD] %s : level %d, lcd_gamma_present %d\n", __FUNCTION__, (level-1), lcd_gamma_present);

				case LCD_IDLE:
					for (i = 0; i < GAMMA_SETTINGS; i++)
						setting_table_write(&gamma_setting_table[(level - 1)][i]);
					break;
				case LCD_VIDEO:
					for (i = 0; i < GAMMA_SETTINGS; i++)
						setting_table_write(&gamma_setting_table_video[(level - 1)][i]);
					break;
				case LCD_CAMERA:
					for (i = 0; i < GAMMA_SETTINGS; i++)
						setting_table_write(&gamma_setting_table_cam[(level - 1)][i]);
					break;
				default:
					break;
			}

		}
	}
	else {
		//printk("lcd_power_ctrl(OFF) \n");
		old_level = level;
		lcd_power_ctrl(OFF);	
		
	}
}

void backlight_level_ctrl(s32 value)
{
	//printk("backlight level ctrl called, Previous backlight_VALUE =%x \n, NEW VALUE = %x", backlight_level, value);
	if ((value < BACKLIGHT_LEVEL_MIN) ||	/* Invalid Value */
		(value > BACKLIGHT_LEVEL_MAX) ||
		(value == backlight_level))	/* Same Value */
		return;

	//printk("%s %d\n", __FUNCTION__, __LINE__);

	if (lcd_late_resume == 0) {
		printk(KERN_ERR "backlight control is not allowed after early suspend\n");
	   	return;
	}


	if (backlight_power)
	backlight_ctrl(value);	
	
	backlight_level = value;
	
}

void backlight_power_ctrl(s32 value)
{

	//printk("backlight_power_ctrl VALUE: %x \n", value);
	if ((value < OFF) ||	/* Invalid Value */
		(value > ON))
		return;

	backlight_ctrl((value ? backlight_level : OFF));	
	
	backlight_power = (value ? ON : OFF);
}

static void ams320fs01_set_backlight_level(u8 level)
{
	//printk("ams320fs01_set_backlight_level VALUE = %x BACLIGHT_LEVEL= %x \n", level, backlight_level);
	if (backlight_level == level)
		return;

	backlight_ctrl(level);

	backlight_level = level;
}

static void ams320fs01_brightness_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	//printk("ams320fs01_brightness_set \n");
	mutex_lock(&ams320fs01_backlight_lock);
	ams320fs01_backlight_brightness = value;
	ams320fs01_set_backlight_level(ams320fs01_backlight_brightness);
	mutex_unlock(&ams320fs01_backlight_lock);
}

static struct led_classdev ams320fs01_backlight_led  = {
	.name		= "lcd-backlight",
	.brightness = AMS320FS01_DEFAULT_BACKLIGHT_BRIGHTNESS,
	.brightness_set = ams320fs01_brightness_set,
};

static int ams320fs01_open (struct inode *inode, struct file *filp)
{
    printk(KERN_ERR "[S3C_LCD] ams320fs01_open called\n");

    return nonseekable_open(inode, filp);
}

static int ams320fs01_release (struct inode *inode, struct file *filp)
{
    printk(KERN_ERR "[S3C_LCD] ams320fs01_release called\n");

    return 0;
}

static int ams320fs01_ioctl(struct inode *inode, struct file *filp, 
	                        unsigned int ioctl_cmd,  unsigned long arg)
{
	int ret = 0;

	if( _IOC_TYPE(ioctl_cmd) != AMS320FS01_IOC_MAGIC )
	{
		printk("[S3C_LCD] Inappropriate ioctl 1 0x%x\n",ioctl_cmd);
		return -ENOTTY;
	}
	if( _IOC_NR(ioctl_cmd) > AMS320FS01_IOC_NR_MAX )
	{
		printk("[S3C_LCD] Inappropriate ioctl 2 0x%x\n",ioctl_cmd);	
		return -ENOTTY;
	}

	switch (ioctl_cmd)
	{
		case AMS320FS01_IOC_GAMMA22:
			printk(KERN_ERR "[S3C_LCD] Changing gamma to 2.2\n");
			lcd_gamma_change(LCD_IDLE);
			ret = 0;
			break;

		case AMS320FS01_IOC_GAMMA19:
			printk(KERN_ERR "[S3C_LCD] Changing gamma to 1.9\n");
			lcd_gamma_change(LCD_VIDEO);
			ret = 0;
			break;

		case AMS320FS01_IOC_GAMMA17:
			printk(KERN_ERR "[S3C_LCD] Changing gamma to 1.7\n");
			lcd_gamma_change(LCD_CAMERA);
			ret = 0;
			break;

        default:
            printk(KERN_ERR "[S3C_LCD] ioctl default\n");
            ret = -ENOTTY;
            break;
    }

    return ret;
}

static struct file_operations ams320fs01_fops = {
	.owner = THIS_MODULE,
	.open = ams320fs01_open,
	.release = ams320fs01_release,
	.ioctl = ams320fs01_ioctl,
};

static struct miscdevice ams320fs01_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ams320fs01",
	.fops = &ams320fs01_fops,
};

static int ams320fs01_backlight_probe(struct platform_device *pdev)
{
	int ret = 0;
	
	ret = misc_register(&ams320fs01_device);
	
	if (ret) {
		printk(KERN_ERR "[S3C_LCD] misc_register err %d\n", ret);
		return ret;
	}
	
	ret = led_classdev_register(&pdev->dev, &ams320fs01_backlight_led);
	if (ret < 0)
		printk("%s fail\n", __func__);
	return ret;
}

static int ams320fs01_backlight_remove(struct platform_device *pdev)
{
	misc_deregister(&ams320fs01_device);

	led_classdev_unregister(&ams320fs01_backlight_led); 

	return 0;
}

static struct platform_driver ams320fs01_backlight_driver = {
	.probe		= ams320fs01_backlight_probe,
	.remove		= ams320fs01_backlight_remove,
	.driver		= {
		.name		= "ams320fs01-backlight",
		.owner		= THIS_MODULE,
	},
};

static int __init ams320fs01_backlight_init(void)
{
	return platform_driver_register(&ams320fs01_backlight_driver);
}

static void __exit ams320fs01_backlight_exit(void)
{
	platform_driver_unregister(&ams320fs01_backlight_driver); 
}
module_init(ams320fs01_backlight_init);
module_exit(ams320fs01_backlight_exit);

void s3cfb_init_hw(void)
{
	s3cfb_set_fimd_info();
	s3cfb_set_gpio();
	lcd_gpio_init();
}

//#define LOGO_MEM_BASE		(0x50000000 + 0x08000000 - 0x100000)	/* SDRAM_BASE + SRAM_SIZE(128MB) - 1MB */

void s3cfb_display_logo(int win_num)
{
	s3c_fb_info_t *fbi = &s3c_fb_info[0];
#if 1
#ifdef CONFIG_FB_S3C_BPP_24
	struct rgb565 {
		u16 red:5;
		u16 green:6;
		u16 blue:5;
	};

	struct rgb888 {
		u8 red;
		u8 green;
		u8 blue;
		u8 alpha;
	};

	int i;
	u16 *src = (u16 *)pixel_data;
	u32 *dst = (u32 *)fbi->map_cpu_f1;

	memset(fbi->map_cpu_f1, 0x00, 800*480*4);
	for (i = 0; i < 480*160*2; i += 2, src++, dst++) {
		((struct rgb888*)dst)->red = ((struct rgb565*)src)->red << 3;
		((struct rgb888*)dst)->green = ((struct rgb565*)src)->green<< 2;
		((struct rgb888*)dst)->blue = ((struct rgb565*)src)->blue << 3;
	}
#else
	memset(fbi->map_cpu_f1, 0x00, 800*480*2);
	memcpy(fbi->map_cpu_f1, pixel_data, 480*160*2);         
#endif
#else
	u16 *logo_virt_buf;
#ifdef CONFIG_FB_S3C_BPP_24
	u32 count;
	u32 *scr_virt_buf = (u32 *)fbi->map_cpu_f1;
#endif

	if(win_num != 0)
		return;
 	
 	logo_virt_buf = ioremap_nocache(LOGO_MEM_BASE, LOGO_MEM_SIZE);
 
#ifdef CONFIG_FB_S3C_BPP_24
	count = LOGO_MEM_SIZE / 2;
	do {
		u16 srcpix = *(logo_virt_buf++);
		u32 dstpix =	((srcpix & 0xF800) << 8) |
				((srcpix & 0x07E0) << 5) |
				((srcpix & 0x001F) << 3);
		*(scr_virt_buf++) = dstpix;
	} while (--count);
#else
 	memcpy(fbi->map_cpu_f1, logo_virt_buf, LOGO_MEM_SIZE);	
#endif
 
 	iounmap(logo_virt_buf);
#endif
}
