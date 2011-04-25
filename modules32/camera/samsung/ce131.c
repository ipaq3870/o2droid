/*
 *  
 *            
 *            - based on s5k4ca.c from Samsung electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Driver for NEC CE131
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <mach/hardware.h>
#include <plat/gpio-cfg.h>
#include <plat/egpio.h>
#include "../s3c_camif.h"
#include "videodev2_samsung.h"
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/platform_device.h>
#include <plat/regs-camif.h>


/// debugging print
//#define __TRACE_CAM_SENSOR__
//#define __TRACE_FULL_CAM_SENSOR__

#if defined(__TRACE_FULL_CAM_SENSOR__)
	#define __TRACE_CAM_SENSOR(s) (s)
	#define __TRACE_FULL_CAM_SENSOR(s) (s) 
#elif defined(__TRACE_CAM_SENSOR__)
	#define __TRACE_CAM_SENSOR(s) (s)
	#define __TRACE_FULL_CAM_SENSOR(s)
#else
	#define __TRACE_CAM_SENSOR(s)
	#define __TRACE_FULL_CAM_SENSOR(s)
#endif

//#ifdef CE131_DEBUG
#define ce131_msg	dev_err
/*#else
#define ce131_msg 	dev_dbg
#endif*/


#define	MCAM_RST_DIS do {	\
	/* MCAM RST Low */	\
	if (gpio_is_valid(GPIO_MCAM_RST_N)) {	\
		if (gpio_request	\
		    (GPIO_MCAM_RST_N, S3C_GPIO_LAVEL(GPIO_MCAM_RST_N_AF)))	\
		gpio_direction_output(GPIO_MCAM_RST_N, GPIO_LEVEL_LOW);	\
	}	\
	s3c_gpio_setpull(GPIO_MCAM_RST_N, S3C_GPIO_PULL_NONE);	\
} while (0)



#define	MCAM_STB_DIS do {	\
	/* CAM_3M STB Low */	\
	if (gpio_is_valid(GPIO_CAM_3M_STBY_N)) {	\
		if (gpio_request	\
		    (GPIO_CAM_3M_STBY_N, S3C_GPIO_LAVEL(GPIO_CAM_3M_STBY_N_AF)))	\
		gpio_direction_output(GPIO_CAM_3M_STBY_N, GPIO_LEVEL_LOW);	\
	}	\
	s3c_gpio_setpull(GPIO_CAM_3M_STBY_N, S3C_GPIO_PULL_NONE);	\
} while (0)



#define	MCAM_STB_EN do {	\
	/* MCAM STB High */	\
	gpio_set_value(GPIO_CAM_3M_STBY_N, GPIO_LEVEL_HIGH);	\
} while (0)

#define	CAM_PWR_EN do {	\
	gpio_direction_output(GPIO_CAM_EN, GPIO_LEVEL_HIGH);	\
} while (0)

#define	CAM_PWR_DIS do {	\
	/* CAM PWR Low */	\
	if (gpio_is_valid(GPIO_CAM_EN)) {	\
		if (gpio_request(GPIO_CAM_EN, S3C_GPIO_LAVEL(GPIO_CAM_EN_AF)))	\
		gpio_direction_output(GPIO_CAM_EN, GPIO_LEVEL_LOW);	\
	}	\
	s3c_gpio_setpull(GPIO_CAM_EN, S3C_GPIO_PULL_NONE);	\
} while (0)

#define	MCAM_RST_EN do {	\
	gpio_direction_output(GPIO_MCAM_RST_N, GPIO_LEVEL_HIGH);	\
} while (0)




#define CMD_INIT			0xF0
#define DATA_VERSION_FW			0x00
#define CMD_VERSION			0x00
#define DATA_VERSION_DATE		0x01
#define CMD_GET_BATCH_REFLECTION_STATUS	0x02
#define DATA_VERSION_SENSOR		0x03
#define DATA_VERSION_AF			0x05
#define DATA_VERSION_SENSOR_MAKER 	0xE0
#define CMD_CAPTURE_SIZE		0x73
#define CMD_PREVIEW_SIZE		0x54 
#define CMD_FPS				0x5A 
#define CMD_PREVIEW			0x6B 
#define	CMD_PREVIEW_STATUS		0x6C
#define CMD_SET_DZOOM			0xB9
#define CMD_GET_DZOOM_LEVEL 		0xBA
#define CMD_SET_ANTI_BANDING		0x14
#define CMD_SET_ANTI_SHAKE		0x5B
#define CMD_AE_WB_LOCK			0x11
#define CMD_GET_PREVIEW_EXPOSURE	0xA4  
#define CMD_SET_AUTO_FOCUS_MODE 	0x20 
#define CMD_START_AUTO_FOCUS_SEARCH 	0x23 
#define CMD_CHECK_AUTO_FOCUS_SEARCH 	0x24
#define CMD_FW_STATUS			0xF5 //CHECKED, not implemented yet --> for firmware update
#define CMD_GET_STROBE_RESULT		0xBD //NOT in CE131 FIXME
#define CMD_JPEG_CONFIG			0x90
#define CMD_JPEG_BUFFERING		0x8F
//#define CMD_SET_FLASH_MANUAL
#define CMD_SET_FLASH 			0xb2
#define CMD_SET_WB 			0x1b
#define CMD_SET_WB_AUTO			0x1A
#define CMD_SET_EFFECT 			0x48
#define CMD_SET_ISO 			0x13
#define CMD_JPEG_SIZE			0x8E
#define CMD_SET_DATA			0x65
#define CMD_DATA_OUT_REQ		0x66
#define CMD_SET_CONTRAST 		0x7f	
#define CMD_SET_SATURATION 		0x49
#define CMD_SET_SHARPNESS 		0x7e
#define CMD_SET_METERING 		0xa6
#define CMD_SET_WDR			0x88
#define CMD_SET_EV			0x1e
#define CMD_BUFFERING_CAPTURE		0x74

#define ce131_ID			0x78
#define POLL_TIME_MS			10

static unsigned char ce131_buf_set_dzoom[31] = {0xff,0xe7,0xd3,0xc2,0xb4,0xa7,0x9c,0x93,0x8b,0x83,0x7c,0x76,0x71,0x6c,0x67,0x63,0x5f,0x5b,0x58,0x55,0x52,0x4f,0x4d,0x4a,0x48,0x46,0x44,0x42,0x41,0x40,0x3f};
static int DZoom_State = 0;

static struct i2c_driver ce131_driver;

static void ce131_sensor_gpio_init(void);
void ce131_sensor_enable(void);
static void ce131_sensor_disable(void);

static int ce131_sensor_init(void);
static void ce131_sensor_exit(void);

static int ce131_set_iso(struct i2c_client *client, struct v4l2_control *ctrl);
static int ce131_set_metering(struct i2c_client *client, struct v4l2_control *ctrl);
static int ce131_set_ae_awb(struct i2c_client *client, struct v4l2_control *ctrl);
static int ce131_set_capture_cmd(struct i2c_client *client);
static int ce131_set_capture_start(struct i2c_client *client);
static int ce131_get_snapshot_data(struct i2c_client *client);

struct ce131_jpeg_param {
	unsigned int enable;
	unsigned int quality;
	unsigned int main_size;  /* Main JPEG file size */
	unsigned int thumb_size; /* Thumbnail file size */
	unsigned int main_offset;
	unsigned int thumb_offset;
	unsigned int postview_offset;
} ; 

enum ce131_oprmode {
	CE131_OPRMODE_VIDEO = 0,
	CE131_OPRMODE_IMAGE = 1,
};

enum ce131_frame_size {
	CE131_PREVIEW_QCIF = 0,
	CE131_PREVIEW_QVGA,
	CE131_PREVIEW_592x480,
	CE131_PREVIEW_VGA,
	CE131_PREVIEW_D1,
	CE131_PREVIEW_WVGA,
	CE131_PREVIEW_720P,
	CE131_PREVIEW_VERTICAL_QCIF,
	//CE131_CAPTURE_VGA, /* 640 x 480 */	
	CE131_CAPTURE_WVGA, /* 800 x 480 */
	CE131_CAPTURE_W1MP, /* 1600 x 960 */
	//CE131_CAPTURE_2MP, /* UXGA  - 1600 x 1200 */
	CE131_CAPTURE_W2MP, /* 35mm Academy Offset Standard 1.66  - 2048 x 1232, 2.4MP */	
	//CE131_CAPTURE_3MP, /* QXGA  - 2048 x 1536 */
	CE131_CAPTURE_W4MP, /* WQXGA - 2560 x 1536 */
	//CE131_CAPTURE_5MP, /* 2560 x 1920 */
};

#define CE131_CAPTURE_5MP SENSOR_QSXGA
#define CE131_CAPTURE_3MP SENSOR_QXGA
#define CE131_CAPTURE_2MP SENSOR_UXGA
#define CE131_CAPTURE_VGA SENSOR_VGA


struct ce131_enum_framesize {
	/* mode is 0 for preview, 1 for capture */
	enum ce131_oprmode mode;
	unsigned int index;
	unsigned int width;
	unsigned int height;	
};

extern void	ce131_sensor_power_init(void);	
extern void lcd_gamma_change(int gamma_status);

typedef enum {
	LCD_IDLE = 0,
	LCD_VIDEO,
	LCD_CAMERA
} lcd_gamma_status;

static struct ce131_enum_framesize ce131_framesize_list[] = {
	{ CE131_OPRMODE_VIDEO, CE131_PREVIEW_QCIF,       176,  144 },
	{ CE131_OPRMODE_VIDEO, CE131_PREVIEW_QVGA,       320,  240 },
	{ CE131_OPRMODE_VIDEO, CE131_PREVIEW_592x480,    592,  480 },
	{ CE131_OPRMODE_VIDEO, CE131_PREVIEW_VGA,		 640,  480 },
	{ CE131_OPRMODE_VIDEO, CE131_PREVIEW_D1,         720,  480 },
	{ CE131_OPRMODE_VIDEO, CE131_PREVIEW_WVGA,       800,  480 },
	{ CE131_OPRMODE_VIDEO, CE131_PREVIEW_720P,      1280,  720 },
	{ CE131_OPRMODE_VIDEO, CE131_PREVIEW_VERTICAL_QCIF,	144,	176},
	{ CE131_OPRMODE_IMAGE, CE131_CAPTURE_VGA,		 640,  480 },
	{ CE131_OPRMODE_IMAGE, CE131_CAPTURE_WVGA,		 800,  480 },
	{ CE131_OPRMODE_IMAGE, CE131_CAPTURE_W1MP,   	1600,  960 },
	{ CE131_OPRMODE_IMAGE, CE131_CAPTURE_2MP,       1600, 1200 },
	{ CE131_OPRMODE_IMAGE, CE131_CAPTURE_W2MP,		2048, 1232 },
	{ CE131_OPRMODE_IMAGE, CE131_CAPTURE_3MP,       2048, 1536 },
	{ CE131_OPRMODE_IMAGE, CE131_CAPTURE_W4MP,      2560, 1536 },
	{ CE131_OPRMODE_IMAGE, CE131_CAPTURE_5MP,       2560, 1920 },
};



enum ce131_runmode {
	CE131_RUNMODE_NOTREADY,
	CE131_RUNMODE_IDLE, 
	CE131_RUNMODE_READY,
	CE131_RUNMODE_RUNNING, 
	CE131_RUNMODE_CAPTURE, 
};

struct ce131_state {
	struct i2c_client *client;
	struct ce131_jpeg_param jpeg;
	enum ce131_runmode runmode;
	int flash;
	int fps;
	int iso;
	int preview_size;
	int framesize_index;
	int saturation;
	int sharpness;
	int af;
	int contrast;
	int metering;
	int brightness;
	int whitebalance;
	int effect;
};


static inline struct ce131_state *to_state(struct i2c_client *client)
{
	return container_of(client, struct ce131_state, client);
}

/* 
 * MCLK: 24MHz, PCLK: 54MHz
 * 
 * In case of PCLK 54MHz
 *
 * Preview Mode (1024 * 768)  
 * 
 * Capture Mode (2048 * 1536)
 * 
 * Camcorder Mode
 */
static camif_cis_t ce131_data = {
	itu_fmt:       	CAMIF_ITU601,
	order422:      	CAMIF_CBYCRY, //WINMO CAMIF_CBYCRY, original CAMIF_CRYCBY
	camclk:        	24000000,		
	source_x:      	640,	//winmo 640, original 1024	
	source_y:      	480, //winmo 480, original 768
	win_hor_ofst:  	0,
	win_ver_ofst:  	0,
	win_hor_ofst2: 	0,
	win_ver_ofst2: 	0,
	polarity_pclk: 	1, //original 0 , winmo 1
	polarity_vsync:	1,
	polarity_href: 	0, 
	href_mask:      1, //fix for HREF_MASK
	reset_type:		CAMIF_RESET,
	reset_udelay: 	5000,
};

/** 
 * ce131_i2c_write_multi: Write (I2C) multiple bytes to the camera sensor 
 * @client: pointer to i2c_client
 * @cmd: command register
 * @w_data: data to be written
 * @w_len: length of data to be written
 *
 * Returns 0 on success, <0 on error
 */
static int ce131_i2c_write_multi(struct i2c_client *client, unsigned char cmd, 
		unsigned char *w_data, unsigned int w_len)
{
	int retry_count = 1;
	unsigned char buf[w_len+1];
	struct i2c_msg msg = {client->addr, 0, w_len+1, buf};

	int ret = -1;

	buf[0] = cmd;
	memcpy(buf+1, w_data, w_len);

#ifdef CE131_DEBUG
	{
		int j;
		printk("ce131_i2c_write_multi W: ");
		for(j = 0; j <= w_len; j++){
			printk("0x%02x ", buf[j]);
		}
		printk("\n");
	}
#endif

	while(retry_count--){
		ret  = i2c_transfer(client->adapter, &msg, 1);
		if(ret == 1)
			break;
		msleep(POLL_TIME_MS);
		}

	return (ret == 1) ? 0 : -EIO;
}

static int ce131_i2c_read_multi(struct i2c_client *client, unsigned char cmd, 
		unsigned char *w_data, unsigned int w_len, 
		unsigned char *r_data, unsigned int r_len)
{
	unsigned char buf[w_len+1];
	struct i2c_msg msg = {client->addr, 0, w_len + 1, buf};
	int ret = -1;
	int retry_count = 1;

	buf[0] = cmd;
	memcpy(buf+1, w_data, w_len);

#ifdef CE131_DEBUG
	{
		int j;
		printk("R: ");
		for(j = 0; j <= w_len; j++){
			printk("0x%02x ", buf[j]);
		}
		printk("\n");
	}
#endif

	while(retry_count--){
		ret = i2c_transfer(client->adapter, &msg, 1);
		if(ret == 1)
			break;
		msleep(POLL_TIME_MS);
	}

	if(ret < 0)
		return -EIO;

	msg.flags = I2C_M_RD;
	msg.len = r_len;
	msg.buf = r_data;

	retry_count = 1;
	while(retry_count--){
		ret  = i2c_transfer(client->adapter, &msg, 1);
		if(ret == 1)
			break;
		msleep(POLL_TIME_MS);
	}

	return (ret == 1) ? 0 : -EIO;
}

static int ce131_load_fw(struct i2c_client *client)
{
	struct ce131_state *state = to_state(client);
	printk("ce131_load_fw \n");
	int ce131_reglen_init = 1;
	unsigned char ce131_regbuf_init[1] = { 0x00 };
	int err;


        err = ce131_i2c_write_multi(client, CMD_INIT, ce131_regbuf_init, ce131_reglen_init);
        if(err < 0)
                return -EIO;

        /* At least 700ms delay required to load the firmware for ce131 camera ISP */
        msleep(700);

	state->runmode = CE131_RUNMODE_IDLE; 

	
	return 0;
}



static void ce131_sensor_gpio_init(void)
{
#ifdef CE131_DEBUG
	printk("functie! ce131_sensor_gpio_init \n");
#endif
	__TRACE_CAM_SENSOR(printk("[CAM-SENSOR] +%s\n",__func__));
	CAM_PWR_DIS;

	MCAM_RST_DIS;

	MCAM_STB_DIS;

	__TRACE_CAM_SENSOR(printk("[CAM-SENSOR] -%s\n",__func__));
}

void ce131_sensor_enable(void)
{
#ifdef CE131_DEBUG
	printk("functie! ce131_sensor_enable \n");
#endif
	ce131_sensor_gpio_init();


	/* > 0 ms */
	msleep(1);	


	ce131_sensor_power_init();	


	/* > 0 ms */
	msleep(1);

	/* MCLK Set */
	clk_set_rate(cam_clock, ce131_data.camclk);

	/* MCLK Enable */
	clk_enable(cam_clock);
	clk_enable(cam_hclk);
	
	msleep(1);

	CAM_PWR_EN;
	MCAM_STB_EN;

	msleep(1);
	MCAM_RST_EN;
	
	mdelay(5);
#ifdef CE131_DEBUG	
	(printk("[CAM-SENSOR] -%s\n",__func__));
#endif
}

static void ce131_sensor_disable(void)
{
#ifdef CE131_DEBUG
	printk("functie! ce131_sensor_disable \n");
#endif
	MCAM_RST_DIS;
	msleep(1);
	MCAM_STB_DIS;
	msleep(1);

	CAM_PWR_DIS;
	// > 20 cycles 

	// > 0 ms 
	msleep(1);
	// MCLK Disable 
	clk_disable(cam_clock);
	clk_disable(cam_hclk);
#ifdef CE131_DEBUG
	(printk("[CAM-SENSOR] -%s\n",__func__)); 
#endif
}

static int ce131_get_version(struct i2c_client *client, int object_id, unsigned char version_info[])
{
	//struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char cmd_buf[1] = {0x00};
	unsigned int cmd_len = 1;
	unsigned int info_len = 4;
	int err;
	
	switch(object_id)
	{
	case DATA_VERSION_FW:
	case DATA_VERSION_DATE:
	case DATA_VERSION_SENSOR:
	case DATA_VERSION_SENSOR_MAKER:
	case DATA_VERSION_AF:			
		cmd_buf[0] = object_id;
		break;
	default:
		return -EINVAL;
	}

        err = ce131_i2c_read_multi(client, CMD_VERSION, cmd_buf, cmd_len, version_info, info_len);
        if(err < 0)
                return -EIO;
	printk("CE131 version: %x \n", version_info);
	return 0;
}


static int ce131_get_fw_version(struct i2c_client *client)
{
	unsigned char version_info[4] = {0x00, 0x00, 0x00, 0x00};
	int err = -1;

	err = ce131_get_version(client, DATA_VERSION_FW, version_info);
	printk(" CE131 DATA VERSION FW: %x \n", version_info);
	if(err < 0) 
		return  err;

	return 0;
}
/*
static int ce131_get_strobe_result(struct i2c_client *client) FIXME
{
	unsigned char strobe_info[1] = {0x01};
	unsigned int cmd_len_strobe = 1;
	int err = -1;


	err = ce131_i2c_read_multi(client, CMD_GET_STROBE_RESULT, strobe_info, cmd_len_strobe, strobe_status, strobe_status_len);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_read for set_data_request\n", __func__);
		return -EIO;
	}
	printk(" CMD_GET_STROBE_RESULT: %x \n", version_info);

	return 0;
}
*/
static int sensor_init(struct i2c_client *client)
{
	struct ce131_state *state = to_state(client);
	state->runmode = CE131_RUNMODE_NOTREADY;
	printk("CE131 sensor_init \n");


	int err = -EINVAL;

	err = ce131_load_fw(client);	
	if(err < 0){
		dev_err(&client->dev, "%s: Failed: Camera Initialization\n", __func__);
		return -EIO;
	}
#ifdef CE131_DEBUG
	printk("ce131_load_fw succesfull \n");
#endif
	msleep(100);
	err = ce131_get_fw_version(client);
	if(err < 0){
		dev_err(&client->dev, "%s: Failed: Reading firmware version\n",__func__);
		return -EIO;
	}

	return 0;
}

static int ce131_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	printk("CE131_probe \n");
	struct i2c_client *c;
	ce131_data.sensor = client;
	//i2c_set_clientdata(client, c);
	i2c_set_clientdata(client, ce131_data.sensor);
	return 0;
}

static int ce131_sensor_detach(struct i2c_client *client)
{
	printk("CE131_sensor_detach\n");
	i2c_del_driver(&ce131_driver);
	return 0;
}

static int ce131_waitfordone_timeout(struct i2c_client *client, unsigned char cmd, unsigned char value, 
					int timeout, int polling_interval)
{
	int err;
	unsigned char cam_status = 0xFF;
	unsigned long jiffies_start = jiffies;
	unsigned long jiffies_timeout = jiffies_start + msecs_to_jiffies(timeout);

	if(polling_interval < 0)
		polling_interval = POLL_TIME_MS;

	while(time_before(jiffies, jiffies_timeout)){
		cam_status = 0xFF;
		err = ce131_i2c_read_multi(client, cmd, NULL, 0, &cam_status, 1);
		if(err < 0)
			return -EIO;
#ifdef CE131_DEBUG
		printk("Status check returns %02x\n", cam_status);
#endif
		if(cam_status == value) 
			break;

		msleep(polling_interval);
	}

	if(cam_status != value)
		return -EBUSY;
	else
		return jiffies_to_msecs(jiffies - jiffies_start);
}

static int ce131_get_batch_reflection_status(struct i2c_client *client)
{
	int err;		
	int end_cnt = 0;


	unsigned char ce131_buf_batch_data[1] = {0x00};
	unsigned char ce131_batch_ref_status = 0x00;

	err = ce131_i2c_write_multi(client, 0x01, ce131_buf_batch_data, 1);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write forget_batch_reflection_status\n", __func__);
		return -EIO;
	}

	//To Do: This code needs timeout API for do-while
	do
	{
		msleep(10); 
        	err = ce131_i2c_read_multi(client, CMD_GET_BATCH_REFLECTION_STATUS, NULL, 0, &ce131_batch_ref_status, 1);
		if(err < 0){
			dev_err(&client->dev, "%s: failed: i2c_read for get_batch_reflection_status\n", __func__);
			return -EIO;
		}
		end_cnt++;
	} while(ce131_batch_ref_status && end_cnt < 200);

	if(end_cnt > 5) 
	{
		ce131_msg(&client->dev, "%s: count(%d) status(%02x) \n", __func__, end_cnt, ce131_batch_ref_status);
	}
		
	if (ce131_batch_ref_status != 0x00)
	{
		dev_err(&client->dev, "%s: failed: to get_batch_reflection_status\n", __func__);
		return -EINVAL;
	}

	ce131_msg(&client->dev, "%s: done\n", __func__);

	return 0;
}


static int ce131_set_preview_size(struct i2c_client *client)
{
	int err;
	unsigned char ce131_regbuf_preview_size[3] = { 0x08, 0x01, 0x00 }; 
	unsigned int ce131_reglen_preview_size = 3; 

	struct ce131_state *state = to_state(client);

	switch(state->preview_size){
		case CE131_PREVIEW_VGA:
			ce131_regbuf_preview_size[0] = 0x08;
			break;
		case CE131_PREVIEW_WVGA:
			ce131_regbuf_preview_size[0] = 0x10;
			break;
		/*case CE131_PREVIEW_UNKNOWN:
			ce131_regbuf_preview_size[0] = 0x0d;
			break;
		case CE131_PREVIEW_UNKNOWN2:
			ce131_regbuf_preview_size[0] = 0x19;
			break;
		case CE131_PREVIEW_UNKNOWN3:
			ce131_regbuf_preview_size[0] = 0x06;
			break; */
		default:
			ce131_regbuf_preview_size[0] = 0x08;
			printk("Setting preview resolution as VGA for image capture mode, send: %x \n", state->preview_size );
			break;
	}

    	err = ce131_i2c_write_multi(client, CMD_PREVIEW_SIZE, ce131_regbuf_preview_size, ce131_reglen_preview_size);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for preview_size\n", __func__);
		return -EIO;
	}
	return 0;
}



static int ce131_set_anti_banding(struct i2c_client *client)
{
	int err;

	unsigned char ce131_regbuf_anti_banding[1] = { 0x02 }; //CHECK REG!!!
	unsigned int ce131_reglen_anti_banding = 1;

	/*switch(state->anti_banding)
	{
		case ANTI_BANDING_OFF:
			ce131_regbuf_anti_banding[0] = 0x00;
		break;

		case ANTI_BANDING_AUTO:
			ce131_regbuf_anti_banding[0] = 0x01;
		break;

		case ANTI_BANDING_50HZ:
			ce131_regbuf_anti_banding[0] = 0x02;
		break;

		case ANTI_BANDING_60HZ:
			ce131_regbuf_anti_banding[0] = 0x03;

		default:*/
			ce131_regbuf_anti_banding[0] = 0x02;
		/*break;
	} */

	err = ce131_i2c_write_multi(client, CMD_SET_ANTI_BANDING, ce131_regbuf_anti_banding, ce131_reglen_anti_banding);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for anti_banding\n", __func__);
		return -EIO;
	}

	return 0;
}

static int ce131_set_preview_stop(struct i2c_client *client)
{

	int ce131_reglen_preview = 1;
	unsigned char ce131_regbuf_preview_stop[1] = { 0x00 };
	int err;
	struct ce131_state *state = to_state(client);
#ifdef CE131_DEBUG
	printk("[5B] ce131_set_preview_stop: (%d)\n", state->runmode);
#endif
	if(CE131_RUNMODE_RUNNING == state->runmode){
		err = ce131_i2c_write_multi(client, CMD_PREVIEW, ce131_regbuf_preview_stop, ce131_reglen_preview);
		if(err < 0){
			dev_err(&client->dev, "%s: failed: i2c_write for preview_stop\n", __func__);
			return -EIO;
		}

		err = ce131_waitfordone_timeout(client, CMD_PREVIEW_STATUS, 0x00, 3000, POLL_TIME_MS);
		if(err < 0){
			dev_err(&client->dev, "%s: Wait for preview_stop failed\n", __func__ );
			return err;
		}
		ce131_msg(&client->dev, "%s: preview_stop - wait time %d ms\n", __func__, err);	

		state->runmode = CE131_RUNMODE_READY;
	}
	return 0;
}

static int ce131_set_dzoom(struct i2c_client *client, struct v4l2_control *ctrl)
{
	struct ce131_state *state = to_state(client);
	unsigned char ce131_buf_get_dzoom_status[2] = { 0x00, 0x00 };
	unsigned int ce131_len_get_dzoom_status = 2;
	int err;
	int count;

	
	if(CE131_RUNMODE_RUNNING == state->runmode){
		err = ce131_i2c_write_multi(client, CMD_SET_DZOOM, &ce131_buf_set_dzoom[ctrl->value], 1);
		if(err < 0){
			dev_err(&client->dev, "%s: failed: i2c_write for set_dzoom\n", __func__);
			return -EIO;
		}

		//To Do: This code needs to use ce131_waitfordone_timeout() API
		for(count = 0; count < 300; count++)
		{
			err = ce131_i2c_read_multi(client, CMD_GET_DZOOM_LEVEL, NULL, 0, ce131_buf_get_dzoom_status, ce131_len_get_dzoom_status);
			if(err < 0){
				dev_err(&client->dev, "%s: failed: i2c_read for set_dzoom\n", __func__);
				return -EIO;
			}
			if(ce131_buf_get_dzoom_status[1] == 0x00) break;
		}
	}

	DZoom_State = ctrl->value;

	ce131_msg(&client->dev, "%s: done\n", __func__);

	return 0;
}

static int ce131_set_capture_size(struct i2c_client *client)
{
	int err;

	unsigned char ce131_regbuf_capture_size[4] = { 0x0D, 0x00, 0x01, 0x00}; // Default 640, have to adjust
	unsigned int ce131_reglen_capture_size = 4;

	struct ce131_state *state = to_state(client);

	int index = state->framesize_index;
	printk("CE131: set capture size: ");
	switch(index){
	case CE131_CAPTURE_VGA: // 640x480 
		ce131_regbuf_capture_size[0] = 0x08;
		printk("VGA \n");
		break;
	//case CE131_CAPTURE_SVGA: // 800x600 
	//	ce131_regbuf_capture_size[0] = 0x09;
	//	break;
	case CE131_CAPTURE_WVGA: // 800x480 
		ce131_regbuf_capture_size[0] = 0x10;
		printk("WVGA \n");
		break;
	case CE131_CAPTURE_W1MP: // 1280x768 
		ce131_regbuf_capture_size[0] = 0x11;
		printk("W1 \n");
		break;	
	//case CE131_CAPTURE_1MP: // 1280x960 
	//	ce131_regbuf_capture_size[0] = 0x12;
	//	break;
	case CE131_CAPTURE_2MP: // 1600x1200 
		ce131_regbuf_capture_size[0] = 0x0C;
		printk("2m \n");
		break;
	case CE131_CAPTURE_W2MP: //2048x960
		ce131_regbuf_capture_size[0] = 0x12;
		printk("w2m\n");
		break;
	case CE131_CAPTURE_3MP: // 2048x1536 
		//ce131_regbuf_capture_size[0] = 0x0F;//0x0D; 
		ce131_regbuf_capture_size[0] = 0x0D; 
		printk("3m \n");
		break;
	//case CE131_CAPTURE_4MP: //2560x1440
	//	("NOT IMPLEMENTED \n");
	//	break;
	case CE131_CAPTURE_W4MP: //2560x1536 
		printk("4m \n");
		ce131_regbuf_capture_size[0] = 0x14;
		break;
	case CE131_CAPTURE_5MP: // 2560x1920 
		printk("5m \n");
		ce131_regbuf_capture_size[0] = 0x0F;
		break;
	default:
		printk("Default 3MP is set \n");
		ce131_regbuf_capture_size[0] = 0x0D; 
	}

	// Set capture image size 
	err = ce131_i2c_write_multi(client, CMD_CAPTURE_SIZE, ce131_regbuf_capture_size, ce131_reglen_capture_size);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for capture_resolution\n", __func__);
		return -EIO; 
	}

#ifdef CE131_DEBUG
	printk("CE131: set_capture_size: 0x%02x\n", index);
#endif
	return 0;	
}

static int ce131_set_frame_rate(struct i2c_client *client)
{
	int err;
	
	unsigned char ce131_regbuf_fps[2] = { 0x1e, 0x00 }; 
	unsigned int ce131_reglen_fps = 2;
	struct ce131_state *state = to_state(client);

	switch(state->fps)
	{
		case FRAME_RATE_7:
			ce131_regbuf_fps[0] = 0x07;
		break;

		case FRAME_RATE_14:
			ce131_regbuf_fps[0] = 0x14;
		break;

		case FRAME_RATE_15:
			ce131_regbuf_fps[0] = 0x15;
		break;

		case FRAME_RATE_60:
			ce131_regbuf_fps[0] = 0x3C; //NOT CONFIRMED BUT LIKELY TO BE THERE
		break;

		case FRAME_RATE_120:
			ce131_regbuf_fps[0] = 0x78;

		case FRAME_RATE_30:
		default:
			ce131_regbuf_fps[0] = 0x1E;

		break;
	}

	err = ce131_i2c_write_multi(client, CMD_FPS, ce131_regbuf_fps, ce131_reglen_fps);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for set_frame_rate\n", __func__);
		return -EIO;
	}

	err = ce131_get_batch_reflection_status(client);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: ce131_get_batch_reflection_status \n", __func__);
		return -EIO; 
	}
#ifdef CE131_DEBUG
	printk("Preview site fps 30 (hardcoded for now) \n");
#endif
	return 0;
}




static int ce131_set_anti_shake(struct i2c_client *client, struct v4l2_control *ctrl)
{
	int err;

	unsigned char ce131_buf_set_anti_shake[1] = { 0x00 };
	unsigned int ce131_len_set_anti_shake = 1;
	
	switch(ctrl->value)
	{
		case ANTI_SHAKE_STILL_ON:
			ce131_buf_set_anti_shake[0] = 0x01;
		break;

		case ANTI_SHAKE_MOVIE_ON:
			ce131_buf_set_anti_shake[0] = 0x10; //NOT CHECKED YET
		break;

		case ANTI_SHAKE_OFF:
		default:
			ce131_buf_set_anti_shake[0] = 0x00;

		break;
	}

	err = ce131_i2c_write_multi(client, CMD_SET_ANTI_SHAKE, ce131_buf_set_anti_shake, ce131_len_set_anti_shake);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for anti_shake\n", __func__);
		return -EIO;
	}

	ce131_msg(&client->dev, "%s: done, AHS: 0x%02x\n", __func__, ce131_buf_set_anti_shake[0]);

	return 0;
}


static int ce131_set_jpeg_quality(struct i2c_client *client) //sub_10019F6C
{


#if 0
	unsigned char ce131_regbuf_jpeg_comp_level[7] = { 0x00, 0xF4, 0x01, 0x90, 0x01, 0x05, 0x01 };
	unsigned int ce131_reglen_jpeg_comp_level = 7;

	unsigned int comp_ratio = 1500;

	int err;

	if(state->jpeg.quality < 0)
		state->jpeg.quality = 0;
	if(state->jpeg.quality > 100)
		state->jpeg.quality = 100;

	comp_ratio -= (100 - state->jpeg.quality) * 10;
	ce131_regbuf_jpeg_comp_level[1] = comp_ratio & 0xFF;
	ce131_regbuf_jpeg_comp_level[2] = (comp_ratio & 0xFF00) >> 8;
	
	ce131_msg(&client->dev, "Quality = %d, Max value = %d\n", state->jpeg.quality, comp_ratio);

	comp_ratio = (comp_ratio * 9) / 10;  /* 10% range for final JPEG image size */
	ce131_regbuf_jpeg_comp_level[3] = comp_ratio & 0xFF;
	ce131_regbuf_jpeg_comp_level[4] = (comp_ratio & 0xFF00) >> 8;

	ce131_msg(&client->dev, "Quality = %d, Min Comp Ratio = %d\n", state->jpeg.quality, comp_ratio);
#endif

	unsigned char ce131_regbuf_jpeg_comp_level[7] = { 0x00, 0xA4, 0x06, 0x78, 0x05, 0x05, 0x01 };
	unsigned int ce131_reglen_jpeg_comp_level = 7;
/*	unsigned int quality = state->jpeg.quality;
	unsigned int compressionRatio = 0;
	unsigned int minimumCompressionRatio = 0;*/
	int err;
/*
//	dev_err(&client->dev, "%s: quality = %d\n", __func__, quality);

	if(quality >= 91 && quality <= 100) { // 91 ~ 100
		compressionRatio = 17; // 17%
	}

	else if(quality >= 81 && quality <= 90) {	// 81 ~ 90
		compressionRatio = 16; // 16%
	}

	else if(quality >= 71 && quality <= 80) { // 71 ~ 80
		compressionRatio = 15; // 15%
	}

	else if(quality >= 61 && quality <= 70) { // 61 ~ 70
		compressionRatio = 14; // 14%
	}

	else if(quality >= 51 && quality <= 60) { // 51 ~ 60
		compressionRatio = 13; // 13%
	}
	
	else if(quality >= 41 && quality <= 50) { // 41 ~ 50
		compressionRatio = 12; // 12%
	}

	else if(quality >= 31 && quality <= 40) { // 31 ~ 40
		compressionRatio = 11; // 11%
	}

	else if(quality >= 21 && quality <= 30) { // 21 ~ 30
		compressionRatio = 10; // 10%
	}

	else if(quality >= 11 && quality <= 20) { // 11 ~ 20
		compressionRatio = 9; // 9%
	}
	
	else if(quality >= 1 && quality <= 10) { // 1 ~ 10
		compressionRatio = 8; // 8%
	}

	else {		
		dev_err(&client->dev, "%s: Invalid Quality(%d)\n", __func__, quality);
		
		return -1;
	}

	minimumCompressionRatio = compressionRatio - 3; // ex) if compression ratio is 17%, minimum compression ratio is 14%
	ce131_regbuf_jpeg_comp_level[1] = (compressionRatio * 100) & 0xFF;
	ce131_regbuf_jpeg_comp_level[2] = ((compressionRatio * 100) & 0xFF00) >> 8;
	ce131_regbuf_jpeg_comp_level[3] = (minimumCompressionRatio * 100) & 0xFF;
	ce131_regbuf_jpeg_comp_level[4] = ((minimumCompressionRatio * 100) & 0xFF00) >> 8;
	
//	dev_err(&client->dev, "%s: compression ratio low byte: 0x%x, high byte: 0x%x\n", __func__, ce131_regbuf_jpeg_comp_level[1], ce131_regbuf_jpeg_comp_level[2]);

//	dev_err(&client->dev, "%s: minimum compression ratio low byte: 0x%x, high byte: 0x%x\n", __func__, ce131_regbuf_jpeg_comp_level[3], ce131_regbuf_jpeg_comp_level[4]); */

	err = ce131_i2c_write_multi(client, CMD_JPEG_CONFIG, ce131_regbuf_jpeg_comp_level, ce131_reglen_jpeg_comp_level);
	
	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for jpeg_comp_level\n", __func__);

		return -EIO;
	}

	return 0;
}

static int ce131_set_jpeg_config(struct i2c_client *client)
{
	int err;

	unsigned char ce131_regbuf_set_lump[2] = { 0x00, 0x08}; //org 4
	unsigned int ce131_reglen_set_lump = 2;

	err = ce131_set_jpeg_quality(client);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: ce131_set_jpeg_quality\n", __func__);
		return -EIO;
	}


	err = ce131_i2c_write_multi(client, CMD_JPEG_BUFFERING, ce131_regbuf_set_lump, ce131_reglen_set_lump);

	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for set_lump\n", __func__);
		return -EIO;
	}

	ce131_msg(&client->dev, "%s: done\n", __func__);

	return 0;
}


static int ce131_set_flash(struct i2c_client *client)
{
	int err;
	struct ce131_state *state = to_state(client);
	unsigned char ce131_buf_set_flash[2] = { 0x03, 0x00 };
	unsigned int ce131_len_set_flash = 2;
	unsigned char ce131_buf_set_flash_manual[2] = { 0x00, 0x00 }; //HAVE TO CHECK
	unsigned int ce131_len_set_flash_manual = 2;
	

	switch(state->flash)
	{
		case FLASH_MODE_OFF:
			ce131_buf_set_flash[1] = 0x00;
		break;

		case FLASH_MODE_AUTO:
			ce131_buf_set_flash[1] = 0x02;
		break;

		case FLASH_MODE_ON:
			ce131_buf_set_flash[1] = 0x01;
		break;

		case FLASH_MODE_TORCH:
			ce131_buf_set_flash_manual[0] = 0x01; //NOT CHECKED
		break;

		default:
			ce131_buf_set_flash[1] = 0x02; //standard AUTO

		break;
	}

	//need to modify flash off for torch mode
/*	if(ctrl->value == FLASH_MODE_OFF)
	{
		err = ce131_i2c_write_multi(client, CMD_SET_FLASH_MANUAL, ce131_buf_set_flash_manual, ce131_len_set_flash_manual);
		if(err < 0){
			dev_err(&client->dev, "%s: failed: i2c_write for set_flash\n", __func__);
			return -EIO;
		}	
	}
*/
	err = ce131_i2c_write_multi(client, CMD_SET_FLASH, ce131_buf_set_flash, ce131_len_set_flash);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for set_flash\n", __func__);
		return -EIO;
	}
	
	ce131_msg(&client->dev, "%s: done, flash: 0x%02x\n", __func__, ce131_buf_set_flash[1]);

	return 0;
}


static int ce131_set_af_softlanding(struct i2c_client *client) //FIXME: IMPLEMENT IN AUTO_FOCUS --> Make one for softlanding
{
	int err;
	int count;
	struct ce131_state *state = to_state(client);
	unsigned char ce131_buf_get_af_status[1] = { 0x00 };
	unsigned char ce131_buf_set_af_land[1] = { 0x08 };
	unsigned int ce131_len_set_af_land = 1;

	if(state->runmode > CE131_RUNMODE_IDLE)
	{
		// make lens landing mode
		err = ce131_i2c_write_multi(client, CMD_SET_AUTO_FOCUS_MODE, ce131_buf_set_af_land, ce131_len_set_af_land);
		if(err < 0){
			dev_err(&client->dev, "%s: failed: i2c_write for auto_focus\n", __func__);
			return -EIO;
		}	
		//check whether af data is valid or not
		for(count = 0; count < 600; count++)
		{
			msleep(10);
			ce131_buf_get_af_status[0] = 0xFF;
			err = ce131_i2c_read_multi(client, CMD_CHECK_AUTO_FOCUS_SEARCH, NULL, 0, ce131_buf_get_af_status, 1);
			if(err < 0){
				dev_err(&client->dev, "%s: failed: i2c_read for get_focus_mode\n", __func__);
				return -EIO;
			}
			if((ce131_buf_get_af_status[0]) == 0x08) break;
		}
	}
	return 0;
}

static int ce131_set_white_balance(struct i2c_client *client, struct v4l2_control *ctrl)
{
	int err;
	unsigned char ce131_buf_set_wb_auto[1] = { 0x01 };
	unsigned char ce131_buf_set_wb[1] = { 0x00 };
	unsigned int ce131_len_set_wb_auto = 1;
	unsigned int ce131_len_set_wb = 1;
#ifdef CE131_DEBUG
	printk("ce131_set_white_balance \n");
#endif
	switch(ctrl->value)
	{
		case WHITE_BALANCE_AUTO:
			printk("WHITE_BALANCE_AUTO \n");
			ce131_buf_set_wb_auto[0] = 0x00;
		break;

		case WHITE_BALANCE_SUNNY:
			printk("WHITE_BALANCE_SUNNY \n");
			ce131_buf_set_wb[0] = 0x00;
		break;

		case WHITE_BALANCE_CLOUDY:
			printk("WHITE_BALANCE_CLOUDY \n");
			ce131_buf_set_wb[0] = 0x01;
		break;

		case WHITE_BALANCE_TUNGSTEN:
			printk("WHITE_BALANCE_TUNGSTEN \n");
			ce131_buf_set_wb[0] = 0x02;
		break;

		case WHITE_BALANCE_FLUORESCENT:
			printk("WHITE_BALANCE_FLUORESCENT \n");
			ce131_buf_set_wb[0] = 0x03;
		break;

		default:
			printk("WHITE_BALANCE_default \n");
			dev_err(&client->dev, "%s: failed: to set_white_balance, enum: %d\n", __func__, ctrl->value);
			ce131_buf_set_wb_auto[0] = 0x00;
		break;
	}

	if(ctrl->value != WHITE_BALANCE_AUTO)
	{
		err = ce131_i2c_write_multi(client, CMD_SET_WB, ce131_buf_set_wb, ce131_len_set_wb);
		if(err < 0){
			dev_err(&client->dev, "%s: failed: i2c_write for white_balance\n", __func__);
			return -EIO;
		}
	}
	err = ce131_i2c_write_multi(client, CMD_SET_WB_AUTO, ce131_buf_set_wb_auto, ce131_len_set_wb_auto);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for white_balance\n", __func__);
		return -EIO;
	}

	err = ce131_get_batch_reflection_status(client);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: ce131_get_batch_reflection_status \n", __func__);
		return -EIO; 
	}

	ce131_msg(&client->dev, "%s: done\n", __func__);

	return 0;
}


static int ce131_set_effect(struct i2c_client *client, struct v4l2_control *ctrl)
{
	int err;



	unsigned char ce131_buf_set_effect[1] = { 0x00 };
	unsigned int ce131_len_set_effect = 1;
	
	switch(ctrl->value)
	{
		case IMAGE_EFFECT_NONE:
			ce131_buf_set_effect[0] = 0x00;
		break;

		case IMAGE_EFFECT_BNW:
			ce131_buf_set_effect[0] = 0x01;
		break;

		case IMAGE_EFFECT_SEPIA:
			ce131_buf_set_effect[0] = 0x03;
		break;

		case IMAGE_EFFECT_AQUA:
			printk("UNCONFIRMED IMAGE EFFECT AQUA \n");
			ce131_buf_set_effect[0] = 0x0D; //NOT CONFIRMED
		break;

		/*case IMAGE_EFFECT_GREEN: *FIXME*
			ce131_buf_set_effect[0] = 0x0A; 
		break;

		case IMAGE_EFFECT_BLUE:
			ce131_buf_set_effect[0] = 0x0B; 
		break;*/

		case IMAGE_EFFECT_ANTIQUE:
			ce131_buf_set_effect[0] = 0x06;
		break;

		case IMAGE_EFFECT_NEGATIVE:
			ce131_buf_set_effect[0] = 0x05;
		break;

		case IMAGE_EFFECT_SHARPEN:
			printk("UNCONFIRMED IMAGE EFFECT SHARPEN \n");
			ce131_buf_set_effect[0] = 0x04; //NOT CONFIRMED
		break;

		default:
			ce131_buf_set_effect[0] = 0x00;

		break;
	}

	err = ce131_i2c_write_multi(client, CMD_SET_EFFECT, ce131_buf_set_effect, ce131_len_set_effect);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for set_effect\n", __func__);
		return -EIO;
	}

	err = ce131_get_batch_reflection_status(client);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: ce131_get_batch_reflection_status for set_effect\n", __func__);
		return -EIO; 
	}

	ce131_msg(&client->dev, "%s: done\n", __func__);

	return 0;
}

static int ce131_set_iso(struct i2c_client *client, struct v4l2_control *ctrl)
{
	int err;


	unsigned char ce131_buf_set_iso[1] = { 0x06 };
	unsigned int ce131_len_set_iso = 1;

	ce131_msg(&client->dev, "%s: Enter : iso %d\n", __func__, ctrl->value);
	
	switch(ctrl->value)
	{
		case ISO_AUTO:
			ce131_buf_set_iso[0] = 0x06;
		break;

		case ISO_50:
			ce131_buf_set_iso[0] = 0x07;
		break;

		case ISO_100:
			ce131_buf_set_iso[0] = 0x08;
		break;

		case ISO_200:
			ce131_buf_set_iso[0] = 0x09;
		break;

		case ISO_400:
			ce131_buf_set_iso[0] = 0x0A; //UNTIL HERE CONFIRMED
		break;

		case ISO_800:
			printk("NOT CONFIRMED ISO LEVEL 800 \n"); 
			ce131_buf_set_iso[0] = 0x0B;
		break;

		case ISO_1600:
			printk("NOT CONFIRMED ISO LEVEL 1600 \n"); 
			ce131_buf_set_iso[0] = 0x0C;
		break;

		/* This is additional setting for Sports' scene mode */
		case ISO_SPORTS:
			printk("NOT CONFIRMED ISO LEVEL SPORTS \n"); 
			ce131_buf_set_iso[0] = 0x12;
		break;
		
		/* This is additional setting for 'Night' scene mode */
		case ISO_NIGHT:
			printk("NOT CONFIRMED ISO LEVEL NIGHT \n"); 
			ce131_buf_set_iso[0] = 0x17;
		break;

		/* This is additional setting for video recording mode */
		case ISO_MOVIE:
			printk("NOT CONFIRMED ISO LEVEL MOVIE \n"); 
			ce131_buf_set_iso[0] = 0x02;
		break;

		default:
			dev_err(&client->dev, "%s: failed: to set_iso, enum: %d\n", __func__, ctrl->value);
			ce131_buf_set_iso[0] = 0x06;
		break;
	}

	err = ce131_i2c_write_multi(client, CMD_SET_ISO, ce131_buf_set_iso, ce131_len_set_iso);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for set_iso\n", __func__);
		return -EIO;
	}

	err = ce131_get_batch_reflection_status(client);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: ce131_get_batch_reflection_status \n", __func__);
		return -EIO; 
	}

	ce131_msg(&client->dev, "%s: done, iso: 0x%02x\n", __func__, ce131_buf_set_iso[0]);

	return 0;
}
	
static int ce131_get_snapshot_data(struct i2c_client *client) //sub_1001A4E4, only needed for transfer of jpeg
{
	int err;
	struct ce131_state *state = to_state(client);

	unsigned char cmd_buf_framesize[1] = { 0x00 };
	unsigned int cmd_len_framesize = 1;

	unsigned char cmd_buf_setdata[2] = { 0x02, 0x00 }; 
	unsigned int cmd_len_setdata = 2;

	unsigned char jpeg_status[3] = { 0x00, 0x00, 0x00 };
	unsigned char jpeg_status_len = 3;

	unsigned char jpeg_framesize[4] = { 0x00, 0x00, 0x00, 0x00 };
	unsigned int jpeg_framesize_len = 4; //r3



	//if(state->jpeg.enable){ *FIXME*
		/* Get main JPEG size */
		cmd_buf_framesize[0] = 0x00; //r1
		err = ce131_i2c_read_multi(client, CMD_JPEG_SIZE, cmd_buf_framesize, cmd_len_framesize, jpeg_framesize, jpeg_framesize_len);
		if(err < 0){
			dev_err(&client->dev, "%s: failed: i2c_read for jpeg_framesize\n", __func__);
			return -EIO;
		}			
		state->jpeg.main_size = jpeg_framesize[1] | (jpeg_framesize[2] << 8) | (jpeg_framesize[3] << 16);

		ce131_msg(&client->dev, "%s: JPEG main filesize = %d bytes\n", __func__, state->jpeg.main_size );

		/* Get Thumbnail size */
		//if(!state->thumb_null)
		//{
		cmd_buf_framesize[0] = 0x02; //0x01
		err = ce131_i2c_read_multi(client, CMD_JPEG_SIZE, cmd_buf_framesize, cmd_len_framesize, jpeg_framesize, jpeg_framesize_len);
		if(err < 0){
			dev_err(&client->dev, "%s: failed: i2c_read for jpeg_framesize\n", __func__);
			return -EIO;
		}			
		state->jpeg.thumb_size = jpeg_framesize[1] | (jpeg_framesize[2] << 8) | (jpeg_framesize[3] << 16);
		ce131_msg(&client->dev, "%s: JPEG thumb filesize = %d bytes\n", __func__, state->jpeg.thumb_size );
		
	/*	else
			state->jpeg.thumb_size = 0;

		ce131_msg(&client->dev, "%s: JPEG thumb filesize = %d bytes\n", __func__, state->jpeg.thumb_size );

		state->jpeg.main_offset = 0;
		state->jpeg.thumb_offset = 0x271000;
		state->jpeg.postview_offset = 0x280A00;*/ //2a3000 Itś there!
	//}

	//if(state->jpeg.enable) //CHECK OUT THE BUFF
		cmd_buf_setdata[0] = 0x02;
	//else
	//	cmd_buf_setdata[0] = 0x01; //IS THERE
	/* Set Data out */
	err = ce131_i2c_read_multi(client, CMD_SET_DATA, cmd_buf_setdata, cmd_len_setdata, jpeg_status, jpeg_status_len);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_read for set_data\n", __func__);
		return -EIO;
	}
	ce131_msg(&client->dev, "%s:JPEG framesize (after set_data_out) = 0x%02x.%02x.%02x\n",__func__,\
			jpeg_status[2], jpeg_status[1],jpeg_status[0]);


	/* 0x66 */ //WINMO sub_1001D994
		cmd_buf_setdata[0] = 0x03; //WINMO
	err = ce131_i2c_read_multi(client, CMD_DATA_OUT_REQ, cmd_buf_setdata, cmd_len_framesize, jpeg_status, jpeg_status_len);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_read for set_data_request\n", __func__);
		return -EIO;
	}

	ce131_msg(&client->dev, "%s:JPEG framesize (after set_data_request) = 0x%02x.%02x.%02x\n",__func__,\
			jpeg_status[2], jpeg_status[1],jpeg_status[0]);


/* Not sure what this routine does yet, at least not very important
	#define CMD_DATA_CHECK 0x61
	// 0x61 WINMO loc_1001A474 NOT IN CE131 
		cmd_buf_setdata[0] = 0x01; //WINMO
	err = ce131_i2c_read_multi(client, CMD_DATA_CHECK, cmd_buf_setdata, cmd_len_framesize, jpeg_status, jpeg_status_len);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_read for set_data_request\n", __func__);
		return -EIO;
	}

	ce131_msg(&client->dev, "%sCMD_DATA_CHECK JPEG  after set_data_request = 0x %x\n",__func__,\
			jpeg_status);

	msleep(32); //WINMO DISSASEMBLY

*/
	ce131_msg(&client->dev, "%s: done\n", __func__);

	return 0;
}


static int ce131_set_contrast(struct i2c_client *client, int type)
{
	int err;
#ifdef CE131_DEBUG
	printk("ce131_set_contrast: %x \n", type);
#endif
	unsigned char ce131_buf_set_contrast[1] = { 0x00 };
	unsigned int ce131_len_set_contrast = 1;



	switch(type)
	{
		case CONTRAST_MINUS_2:
			ce131_buf_set_contrast[0] = 0x01;
		break;

		case CONTRAST_MINUS_1:
			ce131_buf_set_contrast[0] = 0x02;
		break;

		case CONTRAST_DEFAULT:
			ce131_buf_set_contrast[0] = 0x03;
		break;

		case CONTRAST_PLUS_1:
			ce131_buf_set_contrast[0] = 0x04;
		break;

		case CONTRAST_PLUS_2:
			ce131_buf_set_contrast[0] = 0x05;
		break;

		default:
			ce131_buf_set_contrast[0] = 0x03;

		break;
	}

	err = ce131_i2c_write_multi(client, CMD_SET_CONTRAST, ce131_buf_set_contrast, ce131_len_set_contrast);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for set_contrast\n", __func__);
		return -EIO;
	}

	err = ce131_get_batch_reflection_status(client);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: ce131_get_batch_reflection_status \n", __func__);
		return -EIO; 
	}

	ce131_msg(&client->dev, "%s: done, contrast: 0x%02x\n", __func__, ce131_buf_set_contrast[1]);

	return 0;
}

static int ce131_set_saturation(struct i2c_client *client, struct v4l2_control *ctrl)
{
	int err;
	unsigned char ce131_buf_set_saturation[1] = { 0x00 };
	unsigned int ce131_len_set_saturation = 1;
	
	switch(ctrl->value)
	{
		case SATURATION_MINUS_2:
			ce131_buf_set_saturation[0] = 0x01;
		break;

		case SATURATION_MINUS_1:
			ce131_buf_set_saturation[0] = 0x02;
		break;

		case SATURATION_DEFAULT:
			ce131_buf_set_saturation[0] = 0x03;
		break;

		case SATURATION_PLUS_1:
			ce131_buf_set_saturation[0] = 0x04;
		break;

		case SATURATION_PLUS_2:
			ce131_buf_set_saturation[0] = 0x05;
		break;

		default:
			ce131_buf_set_saturation[0] = 0x03;

		break;
	}

	err = ce131_i2c_write_multi(client, CMD_SET_SATURATION, ce131_buf_set_saturation, ce131_len_set_saturation);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for set_saturation\n", __func__);
		return -EIO;
	}

	err = ce131_get_batch_reflection_status(client);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: ce131_get_batch_reflection_status \n", __func__);
		return -EIO; 
	}

	ce131_msg(&client->dev, "%s: done, saturation: 0x%02x\n", __func__, ce131_buf_set_saturation[1]);

	return 0;
}

static int ce131_set_sharpness(struct i2c_client *client, struct v4l2_control *ctrl)
{
	int err;


	unsigned char ce131_buf_set_SHARPNESS[1] = { 0x00 };
	unsigned int ce131_len_set_SHARPNESS = 1;
	
	switch(ctrl->value)
	{
		case SHARPNESS_MINUS_2:
			ce131_buf_set_SHARPNESS[0] = 0x01;
		break;

		case SHARPNESS_MINUS_1:
			ce131_buf_set_SHARPNESS[0] = 0x02;
		break;

		case SHARPNESS_DEFAULT:
			ce131_buf_set_SHARPNESS[0] = 0x03;
		break;

		case SHARPNESS_PLUS_1:
			ce131_buf_set_SHARPNESS[0] = 0x04;
		break;

		case SHARPNESS_PLUS_2:
			ce131_buf_set_SHARPNESS[0] = 0x05;
		break;

		default:
			ce131_buf_set_SHARPNESS[0] = 0x03;

		break;
	}

	err = ce131_i2c_write_multi(client, CMD_SET_SHARPNESS, ce131_buf_set_SHARPNESS, ce131_len_set_SHARPNESS);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for set_SHARPNESS\n", __func__);
		return -EIO;
	}

	err = ce131_get_batch_reflection_status(client);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: ce131_get_batch_reflection_status \n", __func__);
		return -EIO; 
	}

	ce131_msg(&client->dev, "%s: done, SHARPNESS: 0x%02x\n", __func__, ce131_buf_set_SHARPNESS[1]);

	return 0;
}

static int ce131_set_metering(struct i2c_client *client, struct v4l2_control *ctrl)
{
	int err;
	unsigned char ce131_buf_set_metering[1] = { 0x00};
	unsigned int ce131_len_set_metering = 1;
	
	switch(ctrl->value)
	{
		case METERING_MATRIX:
			ce131_buf_set_metering[0] = 0x00;//original 0x02
		break;

		case METERING_CENTER:
			ce131_buf_set_metering[0] = 0x02; //original 0x00
		break;

		case METERING_SPOT:
			ce131_buf_set_metering[0] = 0x01;
		break;

		default:
			printk("set_photometry ->default\n");
			ce131_buf_set_metering[0] = 0x00;
		break;
	}
	
	err = ce131_i2c_write_multi(client, CMD_SET_METERING, ce131_buf_set_metering, ce131_len_set_metering);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for set_photometry\n", __func__);
		return -EIO;
	}

	err = ce131_get_batch_reflection_status(client);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: ce131_get_batch_reflection_status \n", __func__);
		return -EIO; 
	}

	ce131_msg(&client->dev, "%s: done\n", __func__);

	return 0;
}

static int ce131_set_auto_focus(struct i2c_client *client, int type)
{
	int err;
	int size = 0;

/* FIXME: THIS IS NOT A PROPER IMPLEMENTATION */
	unsigned char ce131_buf_set_af[1] = { 0x00 };
	unsigned int ce131_len_set_af = 1;
	unsigned char ce131_buf_get_af_status[1] = { 0x00 };
	int count;
   	int ret = 0;

    switch (type)
	{
		case 0: //RELEASE
			printk("RELEASE AF choosen ? \n");
			ret = 1;
		break;
		case 1: //SINGLE
			printk("SINGLE AF choosen ? \n");
			ret = 1;
		break;
		case 2: //AUTO
			printk("AUTO AF choosen ? \n");
			ce131_buf_set_af[0] =  0x00 ;
			err = ce131_i2c_write_multi(client, CMD_START_AUTO_FOCUS_SEARCH, ce131_buf_set_af, ce131_len_set_af);
			if(err < 0){
				dev_err(&client->dev, "%s: failed: i2c_write for auto_focus\n", __func__);
			return -EIO;

			for(count = 0; count < 600; count++)
			{
				msleep(10);
				ce131_buf_get_af_status[0] = 0xFF;
				err = ce131_i2c_read_multi(client, CMD_CHECK_AUTO_FOCUS_SEARCH, NULL, 0, ce131_buf_get_af_status, 1);
				if(err < 0){
					dev_err(&client->dev, "%s: failed: i2c_read for auto_focus\n", __func__);
					return -EIO;
				}
				if(ce131_buf_get_af_status[0] == 0x05) continue;
				if(ce131_buf_get_af_status[0] == 0x00 || ce131_buf_get_af_status[0] == 0x02) break;
			}	

			/*err = ce131_i2c_write_multi(client, CMD_SET_AUTO_FOCUS_MODE, 0x1, 0x1); //HAVE TO CHECK WHERE TO PUT
			if(err < 0){
				dev_err(&client->dev, "%s: failed: i2c_write for auto_focus\n", __func__);
				return -EIO;
			}*/	
		
			if(ce131_buf_get_af_status[0] == 0x00)
			{
				if(err < 0){
					dev_err(&client->dev, "%s: failed: AF is failed\n", __func__);
					return -EIO;
				}
				ret =0;
			}
			else 
				ret = 1;

			ce131_msg(&client->dev, "%s: done\n", __func__);

			}
			break;
		case 3: //INFINITY
			printk("INFINITY choosen ? \n");
			ret = 1;
			break;
		case 4: //MACRO
		case 5: //FIXED
			printk("MACRO AF choosen ? \n");
			ce131_buf_set_af[0] =  0x01 ;
			err = ce131_i2c_write_multi(client, CMD_START_AUTO_FOCUS_SEARCH, ce131_buf_set_af, ce131_len_set_af);
			if(err < 0){
				dev_err(&client->dev, "%s: failed: i2c_write for auto_focus\n", __func__);
			return -EIO;

			for(count = 0; count < 600; count++)
			{
				msleep(10);
				ce131_buf_get_af_status[0] = 0xFF;
				err = ce131_i2c_read_multi(client, CMD_CHECK_AUTO_FOCUS_SEARCH, NULL, 0, ce131_buf_get_af_status, 1);
				if(err < 0){
					dev_err(&client->dev, "%s: failed: i2c_read for auto_focus\n", __func__);
					return -EIO;
				}
				if(ce131_buf_get_af_status[0] == 0x05) continue;
				if(ce131_buf_get_af_status[0] == 0x00 || ce131_buf_get_af_status[0] == 0x02) break;
			}	

			/*err = ce131_i2c_write_multi(client, CMD_SET_AUTO_FOCUS_MODE, 0x1, 0x1); //NEW!!
			if(err < 0){
				dev_err(&client->dev, "%s: failed: i2c_write for auto_focus\n", __func__);
				return -EIO;
			}*/	
		
			if(ce131_buf_get_af_status[0] == 0x00)
			{
				if(err < 0){
					dev_err(&client->dev, "%s: failed: AF is failed\n", __func__);
					return -EIO;
				}
				ret =0;
			}
			else 
				ret = 1;

			ce131_msg(&client->dev, "%s: done\n", __func__);

			}
			break;
		break;


		default:
			dev_err(&client->dev, "%s: failed: to set autofocus, enum: %d\n", __func__, type);
		break;
	}

	return ret;
}

static int ce131_set_ae_awb(struct i2c_client *client, struct v4l2_control *ctrl)
{
	int err;

	unsigned char ce131_buf_set_ae_awb[1] = { 0x00 };
	
	switch(ctrl->value)
	{
		case AE_LOCK_AWB_UNLOCK: 
			ce131_buf_set_ae_awb[0] = 0x01; //NOT CHECKED
			break;

		case AE_UNLOCK_AWB_LOCK: 
			ce131_buf_set_ae_awb[0] = 0x10; //NOT CHECKED
			break;
			
		case AE_LOCK_AWB_LOCK: 
			ce131_buf_set_ae_awb[0] = 0x11;
			break;

		case AE_UNLOCK_AWB_UNLOCK: 	
		default:
			ce131_buf_set_ae_awb[0] = 0x00;			
			break;
	}
	err = ce131_i2c_write_multi(client, CMD_AE_WB_LOCK, ce131_buf_set_ae_awb, 1);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for set_effect\n", __func__);
		return -EIO;
	}

	ce131_msg(&client->dev, "%s: done\n", __func__);

	return 0;
}

/**
 *  lock:	1 to lock, 0 to unlock
 */
static int ce131_set_awb_lock(struct i2c_client *client, int lock)
{
	int err;

	unsigned char ce131_regbuf_awb_lock[1] = { 0x11 };
	unsigned int ce131_reglen_awb_lock = 1;

	if(lock)
		ce131_regbuf_awb_lock[0] = 0x11;
	else
		ce131_regbuf_awb_lock[0] = 0x00;

	err = ce131_i2c_write_multi(client, CMD_AE_WB_LOCK, ce131_regbuf_awb_lock, ce131_reglen_awb_lock);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for awb_lock\n", __func__);
		return -EIO;
	}

	ce131_msg(&client->dev, "%s: done\n", __func__);

	return 0;
}

static int ce131_set_wdr(struct i2c_client *client, struct v4l2_control *ctrl)
{
	int err;
	unsigned char ce131_buf_set_wdr[1] = { 0x00 };
	unsigned int ce131_len_set_wdr = 1;
	

	switch(ctrl->value)
	{
		case WDR_ON:
			ce131_buf_set_wdr[0] = 0x01;
		break;
		
		case WDR_OFF:			
		default:
			ce131_buf_set_wdr[0] = 0x00;

		break;
	}

	err = ce131_i2c_write_multi(client, CMD_SET_WDR, ce131_buf_set_wdr, ce131_len_set_wdr);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for set_wdr\n", __func__);
		return -EIO;
	}

	ce131_msg(&client->dev, "%s: done, wdr: 0x%02x\n", __func__, ce131_buf_set_wdr[0]);

	return 0;
}

static int ce131_set_capture_start(struct i2c_client *client)
{
	int err;
#ifdef CE131_DEBUG
		printk("ce131_set_capture_start \n"); 
#endif
		err = ce131_waitfordone_timeout(client, 0x6C, 0x00, 3000, POLL_TIME_MS);
		if(err < 0){
			dev_err(&client->dev, "%s: failed: Wait for buffering_capture\n", __func__ );
			return err;
		}
		ce131_msg(&client->dev, "%s: buffering_capture - wait time %d ms\n", __func__, err);
	

		err = ce131_set_jpeg_config(client);
		if(err < 0){
			dev_err(&client->dev, "%s: failed: ce131_set_jpeg_config\n", __func__);
			return -EIO;
		}

		err = ce131_waitfordone_timeout(client, 0x6C, 0x00, 3000, POLL_TIME_MS);
		if(err < 0){
			dev_err(&client->dev, "%s: failed: Wait for buffering_capture\n", __func__ );
			return err;
		}
		ce131_msg(&client->dev, "%s: buffering_capture - wait time %d ms\n", __func__, err);

		err = ce131_get_snapshot_data(client);
		if(err < 0){
			dev_err(&client->dev, "%s: failed: ce131_set_jpeg_config\n", __func__);
			return -EIO;
		}


		err = ce131_waitfordone_timeout(client, 0x6C, 0x00, 3000, POLL_TIME_MS);
		if(err < 0){
			dev_err(&client->dev, "%s: failed: Wait for buffering_capture\n", __func__ );
			return err;
		}
		ce131_msg(&client->dev, "%s: buffering_capture - wait time %d ms\n", __func__, err);


		return 0;
}


static int ce131_set_ev(struct i2c_client *client, struct v4l2_control *ctrl)
{
	int err;


	unsigned char ce131_buf_set_ev[1] = {  0x00 };
	unsigned int ce131_len_set_ev = 1;


	switch(ctrl->value)
	{
		case EV_MINUS_4:
			ce131_buf_set_ev[0] = 0x02;
		break;

		case EV_MINUS_3:
			ce131_buf_set_ev[0] = 0x03;
		break;

		case EV_MINUS_2:
			ce131_buf_set_ev[0] = 0x04;
		break;

		case EV_MINUS_1:
			ce131_buf_set_ev[0] = 0x05;
		break;

		case EV_DEFAULT:
			ce131_buf_set_ev[0] = 0x06;
		break;

		case EV_PLUS_1:
			ce131_buf_set_ev[0] = 0x07;
		break;

		case EV_PLUS_2:
			ce131_buf_set_ev[0] = 0x08;
		break;
		
		case EV_PLUS_3:
			ce131_buf_set_ev[0] = 0x09;
		break;

		case EV_PLUS_4:
			ce131_buf_set_ev[0] = 0x0A;
		break;			

		default:
			dev_err(&client->dev, "%s: failed: to set_ev, enum: %d\n", __func__, ctrl->value);
			ce131_buf_set_ev[0] = 0x06;
		break;
	}

#ifdef CE131_DEBUG
	printk("ce131_set_ev: set_ev:, data: 0x%02x\n", ce131_buf_set_ev[1]);
#endif
	err = ce131_i2c_write_multi(client, CMD_SET_EV, ce131_buf_set_ev, ce131_len_set_ev);
	if(err < 0){
		return -EIO;
	}

	err = ce131_get_batch_reflection_status(client);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: ce131_get_batch_reflection_status \n", __func__);
		return -EIO; 
	}

	ce131_msg(&client->dev, "%s: done\n", __func__);

	return 0;
}

static int ce131_set_capture_cmd(struct i2c_client *client)
{
	int err;
	unsigned char ce131_regbuf_buffering_capture[1] = { 0x00 };
	unsigned int ce131_reglen_buffering_capture = 1;

	err = ce131_i2c_write_multi(client, CMD_BUFFERING_CAPTURE, ce131_regbuf_buffering_capture, ce131_reglen_buffering_capture);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for buffering_capture\n", __func__);
		return -EIO;
	}

	ce131_msg(&client->dev, "%s: done\n", __func__);

	return 0;
}


static int ce131_sensor_mode_set(struct i2c_client *client, int type)
{
	int err;
	int count;
#ifdef CE131_DEBUG
	printk("functie! ce131_sensor_mode_set type = 0x%x\n", type);
#endif
	struct v4l2_control ctrl;
	struct ce131_state *state = to_state(client);



	if (type & SENSOR_PREVIEW)
	{	
		printk("#-> Preview  \n");

		err = ce131_set_preview_stop(client); //make sure it is off beforehand
		if(err < 0){
			dev_err(&client->dev, "%s: failed: Could not stop the running preview.\n", __func__);
			return -EIO;
		}
		
		err = ce131_set_flash(client);
	   	if(err < 0){
			dev_err(&client->dev, "%s: failed: Settings flash\n", __func__);
	        return -EIO;
	   	}

		printk("CMD_SET_FACE_DETECTION NOT HERE \n");


		ctrl.value = state->metering;
		err = ce131_set_metering(client, &ctrl);
	   	if(err < 0){
			dev_err(&client->dev, "%s: failed: Could not set metering\n", __func__);
	        return -EIO;
	   	}

		ctrl.value = state->iso;
		err = ce131_set_iso(client, &ctrl);
	   	if(err < 0){
			dev_err(&client->dev, "%s: failed: Could not set anti banding\n", __func__);
	        return -EIO;
	   	}

		err = ce131_set_anti_banding(client);
	   	if(err < 0){
			dev_err(&client->dev, "%s: failed: Could not set anti banding\n", __func__);
	        return -EIO;
	   	}

		ctrl.value = ANTI_SHAKE_STILL_ON; //STANDARD ON
		err = ce131_set_anti_shake(client, &ctrl);
	   	if(err < 0){
			dev_err(&client->dev, "%s: failed: Could not set anti shake \n", __func__);
	        return -EIO;
	   	}

		err = ce131_set_preview_size(client); 
	    	if(err < 0){
			dev_err(&client->dev, "%s: failed: Could not set preview size\n", __func__);
	        return -EIO;
		} 

		err = ce131_set_frame_rate(client); 
	        if(err < 0){
			dev_err(&client->dev, "%s: failed: Could not set fps\n", __func__);
	                return -EIO;
	        }

		err = ce131_set_capture_size(client); 
	        if(err < 0){
			dev_err(&client->dev, "%s: failed: Could not set capture size\n", __func__);
	                return -EIO;
	        }

		ctrl.value = AE_UNLOCK_AWB_UNLOCK;
		err = ce131_set_ae_awb(client, &ctrl);
		if(err < 0){
			dev_err(&client->dev, "%s: failed: Could not set AWB LOCK \n", __func__);
	        return -EIO;
		}

		int ce131_reglen_preview = 1;
		unsigned char ce131_regbuf_preview[1] = { 0x00 };
		ce131_regbuf_preview[0] =  0x01 ;
	        err = ce131_i2c_write_multi(client, CMD_PREVIEW, ce131_regbuf_preview, ce131_reglen_preview);
	        if(err < 0){
			dev_err(&client->dev, "%s: failed: i2c_write for preview_start\n", __func__);
	                return -EIO;
	        }

		err = ce131_waitfordone_timeout(client, CMD_PREVIEW_STATUS, 0x08, 3000, POLL_TIME_MS);
		if(err < 0){
			dev_err(&client->dev, "%s: Wait for preview_start failed\n", __func__ );
			return err;
		}
		ce131_msg(&client->dev, "%s: preview_start - wait time %d ms\n", __func__, err);

		ctrl.value = 0;
		err = ce131_set_dzoom(client, &ctrl);
		if(err < 0){
			dev_err(&client->dev, "%s: ce131_set_dzoom\n", __func__ );
			return err;
		}

		//ctrl.value = //state->af;
		err = ce131_set_auto_focus(client, 2); //NEW, is this better?
	   	if(err < 0){
			dev_err(&client->dev, "%s: failed: Settings AF\n", __func__);
	        return -EIO;
	   	}

		state->runmode = CE131_RUNMODE_RUNNING;
	}
	else if (type & SENSOR_CAPTURE)
	{	//THIS IS ALL A MESS CAN GO AWAY TO MAKE IT WORK, D is the correct resolution

		printk("#-> Capture \n");
		printk("runmode = %x \n", state->runmode);
		//state->runmode = CE131_RUNMODE_CAPTURE;

		err = ce131_set_capture_size(client); //--> without same idea
	        if(err < 0){
			dev_err(&client->dev, "%s: failed: Could not set fps\n", __func__);
	                return -EIO;
	        }

		err = ce131_set_flash(client);
	   	if(err < 0){
			dev_err(&client->dev, "%s: failed: Settings flash\n", __func__);
	        return -EIO;
	   	}

		//ctrl.value = state->af; --> NOT WORKING, HERE
/*		err = ce131_set_auto_focus(client, 2); //NEW, is this better?, DISABLE HW_AF IN ANDROID!
	   	if(err < 0){
			dev_err(&client->dev, "%s: failed: Settings AF\n", __func__);
	        return -EIO;
	   	}*/

		ctrl.value = 0;
		err = ce131_set_dzoom(client, &ctrl);
		if(err < 0){
			dev_err(&client->dev, "%s: ce131_set_dzoom\n", __func__ );
			return err;
		}
		

		ctrl.value = AE_LOCK_AWB_LOCK;
		err = ce131_set_ae_awb(client, &ctrl);
		if(err < 0){
			dev_err(&client->dev, "%s: failed: Could not set AWB LOCK \n", __func__);
	        return -EIO;
		}

		err = ce131_set_capture_cmd(client);
		if(err < 0){
			dev_err(&client->dev, "%s: failed: ce131_set_capture_cmd\n", __func__);
			return -EIO;
		}


		err = ce131_set_capture_start(client); //WATCH OUT NORMALLY I JPEG TRANSFER
		if(err < 0){
			dev_err(&client->dev, "%s: failed: ce131_set_capture_cmd\n", __func__);
			return -EIO;
		}
		
		//state->runmode = CE131_RUNMODE_RUNNING;
		printk("runmode = %x \n", state->runmode);
	}
	else if (type & SENSOR_CAMCORDER )
	{
		printk("#-> Record\n");	
		//FIXME: HERE PROBABLY FPS FIX
	}

	return 0;
}

static int ce131_sensor_command(struct i2c_client *client, unsigned int cmd, void *arg)
{
#ifdef CE131_DEBUG
	printk("functie! ce131_sensor_command cmd=0x%x  arg=0x%x\n",cmd, arg);
#endif
	struct ce131_state *state = to_state(client);

	__TRACE_CAM_SENSOR(printk("[CAM-SENSOR] +%s cmd=0x%x\n",__func__,cmd));
	struct v4l2_control *ctrl;
	unsigned short *w_data;		/* To support user level i2c */	


	int ret=0;

	switch (cmd)
	{
		case SENSOR_INIT:
			printk("SENSOR_INIT\n");
			ret = sensor_init(client);
			break;

		case USER_ADD:
			printk("USER_ADD\n");
			break;

		case USER_EXIT:
			printk("USER_EXIT\n");
			ce131_sensor_exit();
			break;

		case SENSOR_EFFECT:
			printk("SENSOR_EFFECT\n");
			ctrl = (struct v4l2_control *)arg;
			if(state->runmode != CE131_RUNMODE_RUNNING)
			{
				state->effect = ctrl->value;
				ret = 0;
			}
			else
			{
				ret = ce131_set_effect(client, ctrl);
			}
			break;

		case SENSOR_BRIGHTNESS:
			printk("SENSOR_BRIGHTNESS\n");
			ctrl = (struct v4l2_control *)arg;
			if(state->runmode != CE131_RUNMODE_RUNNING)
			{
				state->brightness = ctrl->value;
				ret = 0;
			}
			else
			{
				ret = ce131_set_ev(client, ctrl);
			}
			break;

		case SENSOR_WB:
			ctrl = (struct v4l2_control *)arg;
			if(state->runmode != CE131_RUNMODE_RUNNING)
			{
				state->whitebalance = ctrl->value;
				ret = 0;
			}
			else
			{
				ret = ce131_set_white_balance(client, ctrl);
			}
			break;

		case SENSOR_SCENE_MODE:
			printk("SENSOR_SCENE_MODE\n");
			break;

		case SENSOR_PHOTOMETRY:
			printk("SENSOR_PHOTOMETRY\n");
			ctrl = (struct v4l2_control *)arg;
			if(state->runmode != CE131_RUNMODE_RUNNING)
			{
				state->metering = ctrl->value;
				ret = 0;
			}
			else
			{
				ret = ce131_set_metering(client, ctrl);
			}
			break;

		case SENSOR_ISO:
			printk("SENSOR_ISO \n");
			ctrl = (struct v4l2_control *)arg;
			//ret = ce131_set_iso(client, ctrl); //HAVE TO BE CHANGED INTO 
			if(state->runmode != CE131_RUNMODE_RUNNING)
			{
				state->iso = ctrl->value;
				ret = 0;
			}
			else
			{
				ret = ce131_set_iso(client, ctrl);
			}
			break;

		case SENSOR_CONTRAST:
			printk("SENSOR_CONTRAST \n");
			ctrl = (struct v4l2_control *)arg;
			if(state->runmode != CE131_RUNMODE_RUNNING)
			{
				state->contrast = ctrl->value;
				ret = 0;
			}
			else
			{
				ret = ce131_set_contrast(client, ctrl);
			}
			break;

		case SENSOR_SATURATION:
			printk("SENSOR_SATURATION \n");
			ctrl = (struct v4l2_control *)arg;
			if(state->runmode != CE131_RUNMODE_RUNNING)
			{
				state->saturation = ctrl->value;
				ret = 0;
			}
			else
			{
				ret = ce131_set_saturation(client, ctrl);
			}
			break;

		case SENSOR_SHARPNESS:
			printk("SENSOR_SHARPNESS \n");
			ctrl = (struct v4l2_control *)arg;
			if(state->runmode != CE131_RUNMODE_RUNNING)
			{
				state->sharpness = ctrl->value;
				ret = 0;
			}
			else
			{
				ret = ce131_set_sharpness(client, ctrl);
			}
			break;

		case SENSOR_AF:
			printk("SENSOR_AF \n");
			ctrl = (struct v4l2_control *)arg;
			if(state->runmode != CE131_RUNMODE_RUNNING)
			{
				state->af = ctrl->value;
				ret = 0;
			}
			else
			{
				ret = ce131_set_auto_focus(client, ctrl->value);
			}
			break;

		case SENSOR_MODE_SET:
			ctrl = (struct v4l2_control *)arg;
			//printk("SENSOR_MODE_SET CTRL: %x \n", ctrl);
			ce131_sensor_mode_set(client, ctrl->value);
			break;
/* REMOVE ASAP
		case SENSOR_XGA:
			printk("SENSOR_xga \n");
			break;

		case SENSOR_QXGA:
			printk("SENSOR_qxGA \n");	
			break;

		case SENSOR_QSVGA:
			printk("SENSOR_qvGA \n");
			break;

		case SENSOR_VGA:
			printk("SENSOR_vGA \n");
			break;

		case SENSOR_SVGA:
			printk("SENSOR_svGA \n");
			break;

		case SENSOR_SXGA:
			printk("SENSOR_sXGA \n");
			break;

		case SENSOR_UXGA:
			printk("SENSOR_UXGA \n");
			break;

		case SENSOR_USER_WRITE:
			printk("SENSOR_USER_WRITE \n");
			break;

		case SENSOR_USER_READ:
			printk("SENSOR_USER_READ \n");
			break;
	*/
		case SENSOR_FLASH_CAMERA:
			printk("SENSOR_FLASH_CAMERA \n");	
			state->flash = ctrl->value;
			break;

		case SENSOR_FLASH_MOVIE:
			printk("SENSOR_FLASH_MOVIE \n");
			state->flash = ctrl->value;
			break;
		case SENSOR_EXIF_DATA:
			printk("SENSOR_EXIF_DATA \n");	
			break;
		case SENSOR_GET_FWVERSION:  //CE131 specific from here on 
			printk("SENSOR_GET_FWVERSION \n");	
			break;
		case SENSOR_JPEG_TRANSFER:
			printk("SENSOR_JPEG_TRANSFER \n");
			//ret = ce131_set_capture_start(client);	
			break;
		case SENSOR_FIX_FRAMERATE:
			printk("SENSOR_FIX_FRAMERATE \n");
			state->fps = ctrl->value; //TODO		
			break;
		case SENSOR_SET_CAPTURE_SIZE:
			ctrl = (struct v4l2_control *)arg;
			printk("SENSOR_SET_CAPTURE_SIZE: %x \n", ctrl);
			state->framesize_index = ctrl; //TODO	FOR NOW LIKE THIS OTHERWISE LOCK UP
			break;
		case SENSOR_GET_JPEG_SIZE:
			printk("SENSOR_GET_JPEG_SIZE bytes = %d \n", state->jpeg.main_size);
			ret = state->jpeg.main_size;	
			break;
		case SENSOR_SET_PREVIEW_SIZE:
			printk("SENSOR_SET_PREVIEW_SIZE \n");	
			state->preview_size = ctrl->value; //TODO NOT IMPLEMENTED IN ANDROID YET	
			break;
		case SENSOR_SET_JPEG_QUAL:
			printk("SENSOR_SET_JPEG_QUAL \n");	
			break;
		case SENSOR_DIG_ZOOM:
			ctrl = (struct v4l2_control *)arg;
			printk("SENSOR_DIG_ZOOM \n");
			ret =ce131_set_dzoom(client, ctrl);	//NOT USED IN ANDROID	
			break;
		case SENSOR_WDR:
			printk("SENSOR_WDR \n");
			ctrl = (struct v4l2_control *)arg;
			ret = ce131_set_wdr(client, ctrl); //NOT USED IN ANDROID
			break;
		case SENSOR_FW_UPDATE:
			printk("CE131 SENSOR_FW_UPDATE -> NOT IMPLEMENTED \n");	
			break;

		default:
			printk("Default \n \n");	
			break;
	}
	//WHERE IS set_anti_shake??, BANDING

	__TRACE_CAM_SENSOR(printk("[CAM-SENSOR] -%s cmd=0x%x\n",__func__,cmd));
	return ret;
}

static const struct i2c_device_id ce131_id[] = {
	{ "ce131", 0 },
	{ },
};


static struct i2c_driver ce131_driver = {
	.driver = {
		.name = "ce131",
	},
	.probe = ce131_probe,
	.remove = ce131_sensor_detach,
	.id_table = ce131_id,
	.command = ce131_sensor_command
};


static int ce131_sensor_init(void)
{	

	int ret;
#ifdef CE131_DEBUG
	printk("ce131_sensor_init \n"); 
#endif	
	s3c_camif_open_sensor(&ce131_data); 
	if (ce131_data.sensor == NULL)
		if ((ret = i2c_add_driver(&ce131_driver)))
			return ret;
	if (ce131_data.sensor == NULL) {
		i2c_del_driver(&ce131_driver);	
		return -ENODEV;
	}
	s3c_camif_register_sensor(&ce131_data); 
	return 0;
}

static void ce131_sensor_exit(void)
{
	ce131_sensor_disable();
#ifdef CE131_DEBUG
	(printk("[CAM-SENSOR] +%s\n",__func__));
#endif
	if (ce131_data.sensor != NULL)
		s3c_camif_unregister_sensor(&ce131_data);
}

static struct v4l2_input ce131_input = {
	.index		= 0,
	.name		= "Camera Input (CE131)",
	.type		= V4L2_INPUT_TYPE_CAMERA,
	.audioset	= 1,
	.tuner		= 0,
	.std		= V4L2_STD_PAL_BG | V4L2_STD_NTSC_M,
	.status		= 0,
};

static struct v4l2_input_handler ce131_input_handler = {
	ce131_sensor_init,
	ce131_sensor_exit	
};

int ce131_sensor_add(void)
{
#ifdef CE131_DEBUG
	printk("ce131_sensor_add non module \n");
#endif
	return s3c_camif_add_sensor(&ce131_input, &ce131_input_handler);

}

void ce131_sensor_remove(void)
{
#ifdef CE131_DEBUG
	(printk("[CAM-SENSOR] +%s\n",__func__));
#endif
	if (ce131_data.sensor != NULL)
		i2c_del_driver(&ce131_driver);

	s3c_camif_remove_sensor(&ce131_input, &ce131_input_handler);
#ifdef CE131_DEBUG
	(printk("[CAM-SENSOR] -%s\n",__func__));
#endif
}

