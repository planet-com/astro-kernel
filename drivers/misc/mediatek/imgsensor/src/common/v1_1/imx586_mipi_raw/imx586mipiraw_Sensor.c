/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 IMX586mipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "imx586mipiraw_Sensor.h"
#include "imx586_eeprom.h"

#define IMX586_PDAF_ON

#undef VENDOR_EDIT

/***************Modify Following Strings for Debug**********************/
#define PFX "IMX586_camera_sensor"
#define LOG_1 LOG_INF("IMX586,MIPI 4LANE\n")
/****************************   Modify end	**************************/

#define LOG_INF(format, args...) pr_debug(PFX "[%s] " format, __func__, ##args)


#define CUS1_12M_60_FPS 1 /* 0:CUS1_12M_100_FPS, 1:CUS1_12M_60_FPS */

#ifdef VENDOR_EDIT
#define MODULE_ID_OFFSET 0x0000
#endif

static kal_uint8 qsc_flag;
extern BYTE IMX586_QSC_data[2304];

BYTE imx586_LRC_data[384] = {0};
#define FAST_READOUT 0
#define VIDEO_60_FPS 0

static DEFINE_SPINLOCK(imgsensor_drv_lock);


static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = IMX586_SENSOR_ID,

	.checksum_value = 0xa4c32546,
#if FAST_READOUT
	.pre = {
		.pclk = 1728000000,
		.linelength = 7872,
		.framelength = 3658,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		/* following for GetDefaultFramerateByScenario() */
		.mipi_pixel_rate = 1629260000,
		.max_framerate = 600, /* 30fps */
	},
#else
	.pre = {
		.pclk = 844800000,
		.linelength = 8976,//8976
		.framelength = 3136,//5310
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		/* following for GetDefaultFramerateByScenario() */
		.mipi_pixel_rate = 523200000,
		.max_framerate = 300, /* 30fps */
	},
#endif
	.cap = { /* reg_E Full size @15fps*/
		.pclk = 1440000000,
		.linelength = 15504,
		.framelength = 6191,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 8000,
		.grabwindow_height = 6000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 924000000,
		.max_framerate = 150,
	},
#if VIDEO_60_FPS
	.normal_video = {
		.pclk = 1051000000,
		.linelength = 7872,
		.framelength = 2234,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3840,
		.grabwindow_height = 2160,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 670630000,
		.max_framerate = 600,
	},
#else
	.normal_video = {
		.pclk = 844800000,
		.linelength = 8976,//8976
		.framelength = 3136,//5310
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		/* following for GetDefaultFramerateByScenario() */
		.mipi_pixel_rate = 523200000,
		.max_framerate = 300, /* 30fps */
	},

#endif
	.hs_video = { /* reg_G 1920x1080 @120fps (binning)*/
		.pclk = 1728000000,
		.linelength = 5376,
		.framelength = 2678,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1920,
		.grabwindow_height = 1280,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 789940000,
		.max_framerate = 1200,
	},

	.slim_video = { /* reg_F 2000x1500 @30fps (binning) */
		.pclk = 1152000000,
		.linelength = 7872,
		.framelength = 2384,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3840,
		.grabwindow_height = 2160,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 867760000,
		.max_framerate = 600,
	},

#if 0
#if CUS1_12M_60_FPS
	.custom1 = { /*reg_B 4000x3000 @60fps (binning)*/
		.pclk = 1728000000,
		.linelength = 8976,
		.framelength = 3208,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 998400000,
		.max_framerate = 600, /*60fps*/
	},
#else /*CUS1_12M_100_FPS*/
	.custom1 = { /*reg_B 4000x3000 @100fps (binning)*/
		.pclk = 1702400000,
		.linelength = 5376,
		.framelength = 3166,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 1492110000,
		.max_framerate = 1000, /*100fps*/
	},
#endif
#if 1
	.custom2 = { /* reg_E 1280x720 @ 240fps (binning)*/
		.pclk = 1440000000,
		.linelength = 15504,
		.framelength = 6191,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 8000,
		.grabwindow_height = 6000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 924000000,
		.max_framerate = 150,
	},
#endif
#if 1
		.custom3 = { /* reg_E 1280x720 @ 240fps (binning)*/
			.pclk = 1440000000,
			.linelength = 15504,
			.framelength = 6191,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 8000,
			.grabwindow_height = 6000,
			.mipi_data_lp2hs_settle_dc = 85,
			.mipi_pixel_rate = 924000000,
			.max_framerate = 150,
		},
#endif
#endif

	.margin = 48,		/* sensor framelength & shutter margin */
	.min_shutter = 6,	/* min shutter */
	.min_gain = 73,
	.max_gain = 4096,
	.min_gain_iso = 100,
	.exp_step = 2,
	.gain_step = 1,
	.gain_type = 0,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,	/* isp gain delay frame for AE cycle */
	.frame_time_delay_frame = 3,
	.ihdr_support = 0,	//0,/* 1, support; 0,not support */
	.ihdr_le_firstline = 0,	/* 1,le first ; 0, se first */
	.temperature_support = 0, /*1, support; 0,not support */
	.sensor_mode_num = 5,	/* support sensor mode num */

	.cap_delay_frame = 2,	/* enter capture delay frame num */
	.pre_delay_frame = 2,	/* enter preview delay frame num */
	.video_delay_frame = 2,	/* enter video delay frame num */
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,	/* enter slim video delay frame num */
	//.custom1_delay_frame = 2,	/* enter custom1 delay frame num */
	//.custom2_delay_frame = 2,	/* enter custom2 delay frame num */
	//.custom3_delay_frame = 2,	/* enter custom3 delay frame num */

	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	/* .mipi_sensor_type = MIPI_OPHY_NCSI2, */
	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_sensor_type = MIPI_OPHY_NCSI2, /* 0,MIPI_OPHY_NCSI2; 1,MIPI_OPHY_CSI2 */
	.mipi_settle_delay_mode = 0,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,//SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_Gb,//SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_Gr,		//SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_R,
	.mclk = 24, /* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
	/*.mipi_lane_num = SENSOR_MIPI_4_LANE,*/
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x34, 0xff},
	/* record sensor support all write id addr,
	 * only supprt 4 must end with 0xff
	 */
	.i2c_speed = 1000, /* i2c read/write speed */
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_HV_MIRROR, //IMAGE_HV_MIRROR,	/* mirrorflip information */
	.sensor_mode = IMGSENSOR_MODE_INIT,
	/* IMGSENSOR_MODE enum value,record current sensor mode,such as:
	 * INIT, Preview, Capture, Video,High Speed Video, Slim Video
	 */
	.shutter = 0x3D0,	/* current shutter */
	.gain = 0x100,		/* current gain */
	.dummy_pixel = 0,	/* current dummypixel */
	.dummy_line = 0,	/* current dummyline */
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_mode = 0, /* sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0x34, /* record current sensor's i2c write id */
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[8] = {
	{8000, 6000, 0000, 0000, 8000, 6000, 4000, 3000, 0000, 0000, 4000, 3000,    0,    0, 4000, 3000}, /* Preview */
	{8000, 6000, 0000, 0000, 8000, 6000, 8000, 6000, 0000, 0000, 8000, 6000,    0,    0, 8000, 6000}, /* capture */
	{8000, 6000, 0000, 0000, 8000, 6000, 4000, 3000, 0000, 0000, 4000, 3000,    0,    0, 4000, 3000}, /* video */
	{8000, 6000, 0000,  440, 8000, 5120, 2000, 1280, 40, 0000, 1920, 1280,    0,    0, 1920, 1280}, /* high speed video */
	//{8000, 6000, 0000, 0000, 8000, 6000, 2000, 1500, 0000, 0000, 2000, 1500,    0,    0, 2000, 1500}, /*  slim */
	{8000, 6000, 0,   0, 8000, 6000, 4000, 3000,	80, 420, 3840, 2160,  0,  0, 3840, 2160}, /*  slim */
	//#if CUS1_12M_60_FPS
//	{8000, 6000, 0000, 0000, 8000, 6000, 4000, 3000, 0000, 0000, 4000, 3000,    0,    0, 4000, 3000}, /*  custom1 */
//#else /*CUS1_12M_100_FPS*/
//	{8000, 6000, 0000, 0000, 8000, 6000, 4000, 3000, 0000, 0000, 4000, 3000,    0,    0, 4000, 3000}, /*  custom1 */
//#endif
//	{8000, 6000, 0000, 0000, 8000, 6000, 8000, 6000, 0000, 0000, 8000, 6000,    0,    0, 8000, 6000}, /* custom2 */
//	{8000, 6000, 0000, 0000, 8000, 6000, 8000, 6000, 0000, 0000, 8000, 6000,    0,    0, 8000, 6000}, /* custom3 */	
};
 /*VC1 for HDR(DT=0X35), VC2 for PDAF(DT=0X36), unit : 10bit */
static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[3] = {
	/* Preview mode setting */
	{0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x0FA0, 0x0BB8, 0x00, 0x00, 0x00, 0x00,
	 0x00, 0x34, 0x04D8, 0x05D0, 0x00, 0x00, 0x0000, 0x0000},
	/* Capture mode setting */
	{0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x0FA0, 0x0BB8, 0x00, 0x00, 0x00, 0x00,
	 0x00, 0x34, 0x04D8, 0x05D0, 0x00, 0x00, 0x0000, 0x0000},
	/* Video mode setting */
	{0x02, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x0FA0, 0x0BB8, 0x00, 0x00, 0x0000, 0x0000,
	 0x00, 0x34, 0x04D8, 0x05D0, 0x00, 0x00, 0x0000, 0x0000}
};

#if 1
/* If mirror flip */
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX = 15,
	.i4OffsetY = 12,
	.i4PitchX = 8,
	.i4PitchY = 16,
	.i4PairNum = 8,
	.i4SubBlkW = 8,
	.i4SubBlkH = 2,
	.i4PosL = { {16, 12}, {18, 14}, {22, 16}, {20, 18},
		   {16, 20}, {18, 22}, {22, 24}, {20, 26} },
	.i4PosR = { {15, 12}, {17, 14}, {21, 16}, {19, 18},
		   {15, 20}, {17, 22}, {21, 24}, {19, 26} },
	.iMirrorFlip = 0,
	.i4BlockNumX = 496,
	.i4BlockNumY = 186,
//	.i4Crop = {{0, 0}, {0, 0}, {0, 372}, {0, 0}, {0, 0},
//		   {0, 0}, {0, 0}, {0,   0}, {0, 0}, {0, 0} }
};
#endif

#if 0
/*No Mirror&Flip*/
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX = 17,
	.i4OffsetY = 12,
	.i4PitchX = 8,
	.i4PitchY = 16,
	.i4PairNum = 8,
	.i4SubBlkW = 8,
	.i4SubBlkH = 2,
	.i4PosL = {{20, 13}, {18, 15}, {22, 17}, {24, 19},
		   {20, 21}, {18, 23}, {22, 25}, {24, 27} },
	.i4PosR = {{19, 13}, {17, 15}, {21, 17}, {23, 19},
		   {19, 21}, {17, 23}, {21, 25}, {23, 27} },
	.iMirrorFlip = 0,
	.i4BlockNumX = 496,
	.i4BlockNumY = 186,
	//.i4Crop = {{0, 0}, {0, 0}, {0, 372}, {0, 0}, {0, 0},
	//	   {0, 0}, {0, 0}, {0,   0}, {0, 0}, {0, 0} }
};
#endif

#define IMX586MIPI_MaxGainIndex (223)
kal_uint16 IMX586MIPI_sensorGainMapping[IMX586MIPI_MaxGainIndex][2] ={
	{72,114},
	{76,162},
	{80,205},
	{84,244},
	{88,279},
	{92,312},
	{96,341},
	{100,369},
	{104,394},
	{108,417},
	{112,439},
	{116,459},
	{120,478},
	{124,495},
	{128,512},
	{132,528},
	{136,542},
	{140,556},
	{144,569},
	{148,581},
	{152,593},
	{156,604},
	{160,614},
	{164,624},
	{168,634},
	{172,643},
	{176,652},
	{180,660},
	{184,668},
	{188,675},
	{192,683},
	{196,690},
	{200,696},
	{204,703},
	{208,709},
	{212,715},
	{216,721},
	{220,726},
	{224,731},
	{228,737},
	{232,742},
	{236,746},
	{240,751},
	{244,755},
	{248,760},
	{252,764},
	{256,768},
	{260,772},
	{264,776},
	{267,779},
	{272,783},
	{277,787},
	{280,790},
	{284,793},
	{287,796},
	{293,800},
	{297,803},
	{301,806},
	{303,808},
	{308,811},
	{312,814},
	{317,817},
	{320,819},
	{324,822},
	{328,824},
	{333,827},
	{336,829},
	{340,831},
	{343,833},
	{349,836},
	{352,838},
	{356,840},
	{360,842},
	{364,844},
	{368,846},
	{372,848},
	{377,850},
	{381,852},
	{383,853},
	{388,855},
	{392,857},
	{397,859},
	{400,860},
	{405,862},
	{407,863},
	{412,865},
	{415,866},
	{420,868},
	{423,869},
	{428,871},
	{431,872},
	{437,874},
	{440,875},
	{443,876},
	{449,878},
	{452,879},
	{455,880},
	{462,882},
	{465,883},
	{468,884},
	{471,885},
	{475,886},
	{478,887},
	{485,889},
	{489,890},
	{493,891},
	{496,892},
	{500,893},
	{504,894},
	{508,895},
	{512,896},
	{516,897},
	{520,898},
	{524,899},
	{529,900},
	{533,901},
	{537,902},
	{542,903},
	{546,904},
	{551,905},
	{555,906},
	{560,907},
	{565,908},
	{570,909},
	{575,910},
	{580,911},
	{585,912},
	{590,913},
	{596,914},
	{601,915},
	{607,916},
	{612,917},
	{618,918},
	{624,919},
	{630,920},
	{636,921},
	{643,922},
	{649,923},
	{655,924},
	{662,925},
	{669,926},
	{676,927},
	{683,928},
	{690,929},
	{697,930},
	{705,931},
	{712,932},
	{720,933},
	{728,934},
	{736,935},
	{745,936},
	{753,937},
	{762,938},
	{771,939},
	{780,940},
	{790,941},
	{799,942},
	{809,943},
	{819,944},
	{830,945},
	{840,946},
	{851,947},
	{862,948},
	{874,949},
	{886,950},
	{898,951},
	{910,952},
	{923,953},
	{936,954},
	{950,955},
	{964,956},
	{978,957},
	{993,958},
	{1008,959},
	{1024,960},
	{1040,961},
	{1057,962},
	{1074,963},
	{1092,964},
	{1111,965},
	{1130,966},
	{1150,967},
	{1170,968},
	{1192,969},
	{1214,970},
	{1237,971},
	{1260,972},
	{1285,973},
	{1311,974},
	{1337,975},
	{1365,976},
	{1394,977},
	{1425,978},
	{1456,979},
	{1489,980},
	{1524,981},
	{1560,982},
	{1598,983},
	{1638,984},
	{1680,985},
	{1725,986},
	{1771,987},
	{1820,988},
	{1872,989},
	{1928,990},
	{1986,991},
	{2048,992},
	{2114,993},
	{2185,994},
	{2260,995},
	{2341,996},
	{2427,997},
	{2521,998},
	{2621,999},
	{2731,1000},
	{2849,1001},
	{2979,1002},
	{3121,1003},
	{3277,1004},
	{3449,1005},
	{3641,1006},
	{3855,1007},
	{4096,1008},
};
static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF)};

	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 2, imgsensor.i2c_write_id);
	return ((get_byte<<8)&0xff00) | ((get_byte>>8)&0x00ff);
}

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
	char pusendcmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF),
			(char)(para & 0xFF)};

	iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}

static void imx586_get_pdaf_reg_setting(MUINT32 regNum, kal_uint16 *regDa)
{
	int i, idx;

	for (i = 0; i < regNum; i++) {
		idx = 2 * i;
		regDa[idx + 1] = read_cmos_sensor_8(regDa[idx]);
		LOG_INF("%x %x", regDa[idx], regDa[idx+1]);
	}
}
static void imx586_set_pdaf_reg_setting(MUINT32 regNum, kal_uint16 *regDa)
{
	int i, idx;

	for (i = 0; i < regNum; i++) {
		idx = 2 * i;
		write_cmos_sensor_8(regDa[idx], regDa[idx + 1]);
		LOG_INF("%x %x", regDa[idx], regDa[idx+1]);
	}
}
#ifdef IMX586_PDAF_ON
static void imx586_apply_LRC(void)
{
	unsigned int Lstart_reg = 0x7510;
	unsigned int Rstart_reg = 0x7600;
	char puSendCmd[194]; /*2byte Laddr + L/R 192 byte */

	LOG_INF("E");

	read_imx586_LRC(imx586_LRC_data);

	/* L LRC*/
	puSendCmd[0] = (char)(Lstart_reg >> 8);
	puSendCmd[1] = (char)(Lstart_reg & 0xFF);
	memcpy((void *)&puSendCmd[2], imx586_LRC_data, 192);
	iBurstWriteReg_multi(puSendCmd, 194, imgsensor.i2c_write_id,
						194, imgsensor_info.i2c_speed);

	/* R LRC*/
	puSendCmd[0] = (char)(Rstart_reg >> 8);
	puSendCmd[1] = (char)(Rstart_reg & 0xFF);
	memcpy((void *)&puSendCmd[2], imx586_LRC_data + 192, 192);
	iBurstWriteReg_multi(puSendCmd, 194, imgsensor.i2c_write_id,
						194, imgsensor_info.i2c_speed);

	LOG_INF("readback LRC, L1(%d) L70(%d) R1(%d) R70(%d)\n",
		read_cmos_sensor_8(0x7520), read_cmos_sensor_8(0x7565),
		read_cmos_sensor_8(0x7568), read_cmos_sensor_8(0x75AD));
}
#endif

static void write_sensor_QSC(void)
{
	kal_uint16 i;	

	for (i = 0; i < 2304; i++) {
		write_cmos_sensor_8(0x7F00 + i, IMX586_QSC_data[i]);
		//printk("IMX586_QSC_data[%d] = 0x%x\n", i, IMX586_QSC_data[i]); 
	}
}

static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d\n",
		imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* return;*/ /* for test */
	write_cmos_sensor_8(0x0104, 0x01);

	write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
	write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
	write_cmos_sensor_8(0x0342, imgsensor.line_length >> 8);
	write_cmos_sensor_8(0x0343, imgsensor.line_length & 0xFF);

	write_cmos_sensor_8(0x0104, 0x00);

}	/*	set_dummy  */

static void set_mirror_flip(kal_uint8 image_mirror)
{
	kal_uint8 itemp;

	LOG_INF("image_mirror = %d\n", image_mirror);
	itemp = read_cmos_sensor_8(0x0101);
	itemp &= ~0x03;

	switch (image_mirror) {

	case IMAGE_NORMAL:
	write_cmos_sensor_8(0x0101, itemp);
	break;

	case IMAGE_V_MIRROR:
	write_cmos_sensor_8(0x0101, itemp | 0x02);
	break;

	case IMAGE_H_MIRROR:
	write_cmos_sensor_8(0x0101, itemp | 0x01);
	break;

	case IMAGE_HV_MIRROR:
	write_cmos_sensor_8(0x0101, itemp | 0x03);
	break;
	}
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	/*kal_int16 dummy_line;*/
	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_INF("framerate = %d, min framelength should enable %d\n", framerate,
		min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	if (frame_length >= imgsensor.min_frame_length)
		imgsensor.frame_length = frame_length;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;

	imgsensor.dummy_line =
			imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line =
			imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */

static void write_shutter(kal_uint32 shutter)
{
	kal_uint16 realtime_fps = 0;
	#ifdef VENDOR_EDIT
	/*Yijun.Tan@camera.driver,20180116,add for slow shutter */
	int longexposure_times = 0;
	static int long_exposure_status;
	#endif

	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	if (shutter < imgsensor_info.min_shutter)
		shutter = imgsensor_info.min_shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10
				/ imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
		/* Extend frame length*/
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_8(0x0104, 0x00);
		}
	} else {
		/* Extend frame length*/
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_8(0x0104, 0x00);

		LOG_INF("(else)imgsensor.frame_length = %d\n",
			imgsensor.frame_length);

	}
	#ifdef VENDOR_EDIT
	/*Yijun.Tan@camera.driver,20180116,add for slow shutter */
	while (shutter >= 65535) {
		shutter = shutter / 2;
		longexposure_times += 1;
	}

	if (longexposure_times > 0) {
		LOG_INF("enter long exposure mode, time is %d",
			longexposure_times);
		long_exposure_status = 1;
		imgsensor.frame_length = shutter + 32;
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor_8(0x3100, longexposure_times & 0x07);
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_8(0x0104, 0x00);
	} else if (long_exposure_status == 1) {
		long_exposure_status = 0;
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor_8(0x3100, 0x00);
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_8(0x0104, 0x00);

		LOG_INF("exit long exposure mode");
	}
	#endif
	/* Update Shutter */
	write_cmos_sensor_8(0x0104, 0x01);
	write_cmos_sensor_8(0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor_8(0x0203, shutter  & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);
	LOG_INF("shutter =%d, framelength =%d\n",
		shutter, imgsensor.frame_length);

}	/*	write_shutter  */

/*************************************************************************
 * FUNCTION
 *	set_shutter
 *
 * DESCRIPTION
 *	This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *	iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void set_shutter(kal_uint32 shutter)
{
	unsigned long flags;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
} /* set_shutter */


/*************************************************************************
 * FUNCTION
 *	set_shutter_frame_length
 *
 * DESCRIPTION
 *	for frame & 3A sync
 *
 *************************************************************************/
static void set_shutter_frame_length(kal_uint16 shutter,
				     kal_uint16 frame_length)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	/*0x3500, 0x3501, 0x3502 will increase VBLANK to
	 *get exposure larger than frame exposure
	 *AE doesn't update sensor gain at capture mode,
	 *thus extra exposure lines must be updated here.
	 */

	/* OV Recommend Solution */
	/*if shutter bigger than frame_length,
	 *should extend frame length first
	 */
	spin_lock(&imgsensor_drv_lock);
	/* Change frame time */
	dummy_line = frame_length - imgsensor.frame_length;
	imgsensor.frame_length = imgsensor.frame_length + dummy_line;
	imgsensor.min_frame_length = imgsensor.frame_length;

	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter)
			? imgsensor_info.min_shutter : shutter;
	shutter =
	(shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		? (imgsensor_info.max_frame_length - imgsensor_info.margin)
		: shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 /
				imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor_8(0x0104, 0x01);
			write_cmos_sensor_8(0x0340,
					imgsensor.frame_length >> 8);
			write_cmos_sensor_8(0x0341,
					imgsensor.frame_length & 0xFF);
			write_cmos_sensor_8(0x0104, 0x00);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_8(0x0104, 0x00);
	}

	/* Update Shutter */
	write_cmos_sensor_8(0x0104, 0x01);
	write_cmos_sensor_8(0x0350, 0x00); /* Disable auto extend */
	write_cmos_sensor_8(0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor_8(0x0203, shutter  & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);
	LOG_INF(
		"Exit! shutter =%d, framelength =%d/%d, dummy_line=%d, auto_extend=%d\n",
		shutter, imgsensor.frame_length, frame_length,
		dummy_line, read_cmos_sensor(0x0350));

}	/* set_shutter_frame_length */

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 i;

	for (i = 0; i < IMX586MIPI_MaxGainIndex; i++) {
		if(gain <= IMX586MIPI_sensorGainMapping[i][0]){
			break;
		}
	}
	if(gain != IMX586MIPI_sensorGainMapping[i][0])
		LOG_INF("Gain mapping don't correctly:%d %d \n", gain, IMX586MIPI_sensorGainMapping[i][0]);
	return IMX586MIPI_sensorGainMapping[i][1];
}

/*************************************************************************
 * FUNCTION
 *	set_gain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *	iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	if (gain < 72 || gain > 64 * BASEGAIN) {
	LOG_INF("Error gain setting");

	if (gain < 72)
		gain = 72;
	else if (gain > 64 * BASEGAIN)
		gain = 64 * BASEGAIN;
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d, reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor_8(0x0104, 0x01);
	write_cmos_sensor_8(0x0204, (reg_gain>>8) & 0xFF);
	write_cmos_sensor_8(0x0205, reg_gain & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);

	return gain;
} /* set_gain */

/*************************************************************************
 * FUNCTION
 *	night_mode
 *
 * DESCRIPTION
 *	This function night mode of sensor.
 *
 * PARAMETERS
 *	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n",
		enable);
	if (enable)
		write_cmos_sensor_8(0x0100, 0X01);
	else
		write_cmos_sensor_8(0x0100, 0x00);
	return ERROR_NONE;
}


#define MULTI_WRITE 1

#if MULTI_WRITE
#define I2C_BUFFER_LEN 765 /* trans# max is 255, each 3 bytes */
#else
#define I2C_BUFFER_LEN 3
#endif
static kal_uint16 imx586_table_write_cmos_sensor(kal_uint16 *para,
						 kal_uint32 len)
{
	char puSendCmd[I2C_BUFFER_LEN];
	kal_uint32 tosend, IDX;
	kal_uint16 addr = 0, addr_last = 0, data;

	tosend = 0;
	IDX = 0;

	while (len > IDX) {
		addr = para[IDX];

		{
			puSendCmd[tosend++] = (char)(addr >> 8);
			puSendCmd[tosend++] = (char)(addr & 0xFF);
			data = para[IDX + 1];
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;

		}
#if MULTI_WRITE
		/* Write when remain buffer size is less than 3 bytes
		 * or reach end of data
		 */
		if ((I2C_BUFFER_LEN - tosend) < 3
			|| IDX == len || addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd,
						tosend,
						imgsensor.i2c_write_id,
						3,
						imgsensor_info.i2c_speed);
			tosend = 0;
		}
#else
		iWriteRegI2C(puSendCmd, 3, imgsensor.i2c_write_id);
		tosend = 0;

#endif
	}
	return 0;
}

static kal_uint16 imx586_init_setting[] = {
	0x0136, 0x18,
	0x0137, 0x00,
	0x3C7E, 0x02,
	0x3C7F, 0x0A,
	0x0111, 0x02,
	0x380C, 0x00,
	0x3C00, 0x10,
	0x3C01, 0x10,
	0x3C02, 0x10,
	0x3C03, 0x10,
	0x3C04, 0x10,
	0x3C05, 0x01,
	0x3C06, 0x00,
	0x3C07, 0x00,
	0x3C08, 0x03,
	0x3C09, 0xFF,
	0x3C0A, 0x01,
	0x3C0B, 0x00,
	0x3C0C, 0x00,
	0x3C0D, 0x03,
	0x3C0E, 0xFF,
	0x3C0F, 0x20,
	0x3F88, 0x00,
	0x3F8E, 0x00,
	0x5282, 0x01,
	0x9004, 0x14,
	0x9200, 0xF4,
	0x9201, 0xA7,
	0x9202, 0xF4,
	0x9203, 0xAA,
	0x9204, 0xF4,
	0x9205, 0xAD,
	0x9206, 0xF4,
	0x9207, 0xB0,
	0x9208, 0xF4,
	0x9209, 0xB3,
	0x920A, 0xB7,
	0x920B, 0x34,
	0x920C, 0xB7,
	0x920D, 0x36,
	0x920E, 0xB7,
	0x920F, 0x37,
	0x9210, 0xB7,
	0x9211, 0x38,
	0x9212, 0xB7,
	0x9213, 0x39,
	0x9214, 0xB7,
	0x9215, 0x3A,
	0x9216, 0xB7,
	0x9217, 0x3C,
	0x9218, 0xB7,
	0x9219, 0x3D,
	0x921A, 0xB7,
	0x921B, 0x3E,
	0x921C, 0xB7,
	0x921D, 0x3F,
	0x921E, 0x77,
	0x921F, 0x77,
	0x9222, 0xC4,
	0x9223, 0x4B,
	0x9224, 0xC4,
	0x9225, 0x4C,
	0x9226, 0xC4,
	0x9227, 0x4D,
	0x9810, 0x14,
	0x9814, 0x14,
	0x99B2, 0x20,
	0x99B3, 0x0F,
	0x99B4, 0x0F,
	0x99B5, 0x0F,
	0x99B6, 0x0F,
	0x99E4, 0x0F,
	0x99E5, 0x0F,
	0x99E6, 0x0F,
	0x99E7, 0x0F,
	0x99E8, 0x0F,
	0x99E9, 0x0F,
	0x99EA, 0x0F,
	0x99EB, 0x0F,
	0x99EC, 0x0F,
	0x99ED, 0x0F,
	0xA569, 0x06,
	0xA56A, 0x13,
	0xA56B, 0x13,
	0xA679, 0x20,
	0xA830, 0x68,
	0xA831, 0x56,
	0xA832, 0x2B,
	0xA833, 0x55,
	0xA834, 0x55,
	0xA835, 0x16,
	0xA837, 0x51,
	0xA838, 0x34,
	0xA854, 0x4F,
	0xA855, 0x48,
	0xA856, 0x45,
	0xA857, 0x02,
	0xA85A, 0x23,
	0xA85B, 0x16,
	0xA85C, 0x12,
	0xA85D, 0x02,
	0xAC72, 0x01,
	0xAC73, 0x26,
	0xAC74, 0x01,
	0xAC75, 0x26,
	0xAC76, 0x00,
	0xAC77, 0xC4,
	0xB051, 0x02,
	0xC020, 0x01,
	0xC61D, 0x00,
	0xC625, 0x00,
	0xC638, 0x03,
	0xC63B, 0x01,
	0xE286, 0x31,
	0xE2A6, 0x32,
	0xE2C6, 0x33,
	0xEA4B, 0x00,
	0xEA4C, 0x00,
	0xEA4D, 0x00,
	0xEA4E, 0x00,
	0xF000, 0x00,
	0xF001, 0x10,
	0xF00C, 0x00,
	0xF00D, 0x40,
	0xF030, 0x00,
	0xF031, 0x10,
	0xF03C, 0x00,
	0xF03D, 0x40,
	0xF44B, 0x80,
	0xF44C, 0x10,
	0xF44D, 0x06,
	0xF44E, 0x80,
	0xF44F, 0x10,
	0xF450, 0x06,
	0xF451, 0x80,
	0xF452, 0x10,
	0xF453, 0x06,
	0xF454, 0x80,
	0xF455, 0x10,
	0xF456, 0x06,
	0xF457, 0x80,
	0xF458, 0x10,
	0xF459, 0x06,
	0xF478, 0x20,
	0xF479, 0x80,
	0xF47A, 0x80,
	0xF47B, 0x20,
	0xF47C, 0x80,
	0xF47D, 0x80,
	0xF47E, 0x20,
	0xF47F, 0x80,
	0xF480, 0x80,
	0xF481, 0x20,
	0xF482, 0x60,
	0xF483, 0x80,
	0xF484, 0x20,
	0xF485, 0x60,
	0xF486, 0x80,
	0x9852, 0x00,
	0x9954, 0x0F,
	0xA7AD, 0x01,
	0xA7CB, 0x01,
	0xAE09, 0xFF,
	0xAE0A, 0xFF,
	0xAE12, 0x58,
	0xAE13, 0x58,
	0xAE15, 0x10,
	0xAE16, 0x10,
	0xAF05, 0x48,
	0xB07C, 0x02,
	0xBCF1, 0x00,
};

#if FAST_READOUT
static kal_uint16 imx586_preview_setting[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x02,
	0x0342, 0x1E,
	0x0343, 0xC0,
	0x0340, 0x0E,
	0x0341, 0x4A,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x17,
	0x034B, 0x6F,
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x08,
	0x3140, 0x00,
	0x3246, 0x81,
	0x3247, 0x81,
	0x3F15, 0x00,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x0F,
	0x040D, 0xA0,
	0x040E, 0x0B,
	0x040F, 0xB8,
	0x034C, 0x0F,
	0x034D, 0xA0,
	0x034E, 0x0B,
	0x034F, 0xB8,
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x68,
	0x030B, 0x01,
	0x030D, 0x04,
	0x030E, 0x01,
	0x030F, 0x8C,
	0x0310, 0x01,
	0x3620, 0x00,
	0x3621, 0x00,
	0x3C11, 0x04,
	0x3C12, 0x03,
	0x3C13, 0x2D,
	0x3F0C, 0x01,
	0x3F14, 0x00,
	0x3F80, 0x01,
	0x3F81, 0x90,
	0x3F8C, 0x00,
	0x3F8D, 0x14,
	0x3FF8, 0x01,
	0x3FF9, 0x2A,
	0x3FFE, 0x00,
	0x3FFF, 0x6C,
	0x0202, 0x0E,
	0x0203, 0x1A,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x3E20, 0x02,
	0x3E37, 0x00,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x01,
	0x4435, 0xF0,
};

#else
static kal_uint16 imx586_preview_setting[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x23,
	0x0343, 0x10,
	0x0340, 0x0C,
	0x0341, 0x40,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x17,
	0x034B, 0x6F,
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x08,
	0x3140, 0x00,
	0x3246, 0x81,
	0x3247, 0x81,
	0x3F15, 0x00,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x0F,
	0x040D, 0xA0,
	0x040E, 0x0B,
	0x040F, 0xB8,
	0x034C, 0x0F,
	0x034D, 0xA0,
	0x034E, 0x0B,
	0x034F, 0xB8,
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x00,
	0x0307, 0xB0,
	0x030B, 0x01,
	0x030D, 0x04,
	0x030E, 0x00,
	0x030F, 0xDA,
	0x0310, 0x01,
	0x3620, 0x00,
	0x3621, 0x00,
	0x3C11, 0x04,
	0x3C12, 0x03,
	0x3C13, 0x2D,
	0x3F0C, 0x01,
	0x3F14, 0x00,
	0x3F80, 0x00,
	0x3F81, 0x14,
	0x3F8C, 0x00,
	0x3F8D, 0x14,
	0x3FF8, 0x00,
	0x3FF9, 0x3C,
	0x3FFE, 0x01,
	0x3FFF, 0x8C,
	0x0202, 0x0C,
	0x0203, 0x10,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x3E20, 0x02,
	0x3E37, 0x00,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x01,
	0x4435, 0xF0,
};
#endif

static kal_uint16 imx586_capture_setting[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x3C,
	0x0343, 0x90,
	0x0340, 0x18,
	0x0341, 0x2F,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x17,
	0x034B, 0x6F,
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x00,
	0x0901, 0x11,
	0x0902, 0x0A,
	0x3140, 0x00,
	0x3246, 0x01,
	0x3247, 0x01,
	0x3F15, 0x00,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x1F,
	0x040D, 0x40,
	0x040E, 0x17,
	0x040F, 0x70,
	0x034C, 0x1F,
	0x034D, 0x40,
	0x034E, 0x17,
	0x034F, 0x70,
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x2C,
	0x030B, 0x01,
	0x030D, 0x04,
	0x030E, 0x01,
	0x030F, 0x81,
	0x0310, 0x01,
	0x3620, 0x01,
	0x3621, 0x01, //0x00
	0x3C11, 0x08,
	0x3C12, 0x08,
	0x3C13, 0x2A,
	0x3F0C, 0x00,
	0x3F14, 0x01,
	0x3F80, 0x00,
	0x3F81, 0x14,
	0x3F8C, 0x00,
	0x3F8D, 0x14,
	0x3FF8, 0x00,
	0x3FF9, 0x00,
	0x3FFE, 0x03,
	0x3FFF, 0x65,
	0x0202, 0x17,
	0x0203, 0xFF,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x3E20, 0x02,
	0x3E37, 0x00,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x01,
	0x4435, 0xF0,
};

#if VIDEO_60_FPS
static kal_uint16 imx586_normal_video_setting_60fps[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x02,
	0x0342, 0x1E,
	0x0343, 0xC0,
	0x0340, 0x08,
	0x0341, 0xB0,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x03,
	0x0347, 0x48,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x14,
	0x034B, 0x27,
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x09,
	0x3140, 0x00,
	0x3246, 0x81,
	0x3247, 0x81,
	0x3F15, 0x00,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x50,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x0F,
	0x040D, 0x00,
	0x040E, 0x08,
	0x040F, 0x70,
	0x034C, 0x0F,
	0x034D, 0x00,
	0x034E, 0x08,
	0x034F, 0x70,
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x00,
	0x0307, 0xDB,
	0x030B, 0x02,
	0x030D, 0x0C,
	0x030E, 0x03,
	0x030F, 0xD2,
	0x0310, 0x01,
	0x3620, 0x00,
	0x3621, 0x00,
	0x3C11, 0x04,
	0x3C12, 0x03,
	0x3C13, 0x2D,
	0x3F0C, 0x01,
	0x3F14, 0x00,
	0x3F80, 0x01,
	0x3F81, 0x90,
	0x3F8C, 0x00,
	0x3F8D, 0x14,
	0x3FF8, 0x01,
	0x3FF9, 0x2A,
	0x3FFE, 0x00,
	0x3FFF, 0x6C,
	0x0202, 0x08,
	0x0203, 0x80,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x3E20, 0x02,
	0x3E37, 0x00,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x01,
	0x4435, 0xE0,
};
#else
#if 1
static kal_uint16 imx586_normal_video_setting[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x23,
	0x0343, 0x10,
	0x0340, 0x0C,
	0x0341, 0x40,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x17,
	0x034B, 0x6F,
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x08,
	0x3140, 0x00,
	0x3246, 0x81,
	0x3247, 0x81,
	0x3F15, 0x00,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x0F,
	0x040D, 0xA0,
	0x040E, 0x0B,
	0x040F, 0xB8,
	0x034C, 0x0F,
	0x034D, 0xA0,
	0x034E, 0x0B,
	0x034F, 0xB8,
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x00,
	0x0307, 0xB0,
	0x030B, 0x01,
	0x030D, 0x04,
	0x030E, 0x00,
	0x030F, 0xDA,
	0x0310, 0x01,
	0x3620, 0x00,
	0x3621, 0x00,
	0x3C11, 0x04,
	0x3C12, 0x03,
	0x3C13, 0x2D,
	0x3F0C, 0x01,
	0x3F14, 0x00,
	0x3F80, 0x00,
	0x3F81, 0x14,
	0x3F8C, 0x00,
	0x3F8D, 0x14,
	0x3FF8, 0x00,
	0x3FF9, 0x3C,
	0x3FFE, 0x01,
	0x3FFF, 0x8C,
	0x0202, 0x0C,
	0x0203, 0x10,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x3E20, 0x02,
	0x3E37, 0x00,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x01,
	0x4435, 0xF0,
};
#endif
#endif
static kal_uint16 imx586_hs_video_setting[] = {
	/*MIPI output setting*/
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x02,
	/*Line Length PCK Setting*/
	0x0342, 0x15,
	0x0343, 0x00,
	/*Frame Length Lines Setting*/
	0x0340, 0x0A,
	0x0341, 0x76,
	/*ROI Setting*/
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x01,
	0x0347, 0xB8,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x15,
	0x034B, 0xB7,
	/*Mode Setting*/
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x44,
	0x0902, 0x09,
	0x3140, 0x00,
	0x3246, 0x89,
	0x3247, 0x89,
	0x3F15, 0x00,
	/*Digital Crop & Scaling*/
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x28,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x07,
	0x040D, 0x80,
	0x040E, 0x05,
	0x040F, 0x00,
	/*Output Size Setting*/
	0x034C, 0x07,
	0x034D, 0x80,
	0x034E, 0x05,
	0x034F, 0x00,
	/*Clock Setting*/
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x68,
	0x030B, 0x01,
	0x030D, 0x04,
	0x030E, 0x00,
	0x030F, 0xC0,
	0x0310, 0x01,
	/*Other Setting*/
	0x3620, 0x00,
	0x3621, 0x00,
	0x3F0C, 0x00,
	0x3F14, 0x00,
	0x3F80, 0x00,
	0x3F81, 0x00,
	0x3FFC, 0x00,
	0x3FFD, 0x00,
	/*Integration Setting*/
	0x0202, 0x0A,
	0x0203, 0x46,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	/*Gain Setting*/
	0x0204, 0x00,
	0x0205, 0x00,
	0x0216, 0x00,
	0x0217, 0x00,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x00,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
};

static kal_uint16 imx586_slim_video_setting[] = {
	/*MIPI output setting*/
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x02,
	/*Line Length PCK Setting*/
	0x0342, 0x1E,
	0x0343, 0xC0,
	/*Frame Length Lines Setting*/
	0x0340, 0x09,
	0x0341, 0x50,
	/*ROI Setting*/
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x02,
	0x0347, 0xE8,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x14,
	0x034B, 0x87,
	/*Mode Setting*/
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x08,
	0x3140, 0x00,
	0x3246, 0x81,
	0x3247, 0x81,
	0x3F15, 0x00,
	/*Digital Crop & Scaling*/
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x50,
	0x040A, 0x00,
	0x040B, 0x30,
	0x040C, 0x0F,
	0x040D, 0x00,
	0x040E, 0x08,
	0x040F, 0x70,
	/*Output Size Setting*/
	0x034C, 0x0F,
	0x034D, 0x00,
	0x034E, 0x08,
	0x034F, 0x70,
	/*Clock Setting*/
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x00,
	0x0307, 0xF0,
	0x030B, 0x01,
	0x030D, 0x04,
	0x030E, 0x00,
	0x030F, 0xD8,
	0x0310, 0x01,
	/*Other Setting*/
	0x3620, 0x00,
	0x3621, 0x00,
	0x3C11, 0x04,
	0x3C12, 0x03,
	0x3C13, 0x2D,
	0x3F0C, 0x01,
	0x3F14, 0x00,
	0x3F80, 0x01,
	0x3F81, 0x90,
	0x3F8C, 0x00,
	0x3F8D, 0x14,
	0x3FF8, 0x01,
	0x3FF9, 0x2A,
	0x3FFE, 0x00,
	0x3FFF, 0x6C,
	/*Integration Setting*/
	0x0202, 0x09,
	0x0203, 0x20,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	/*Gain Setting*/
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	/*PDAF TYPE2 Setting*/
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x01,
	0x4435, 0xE0,
};

#if 0
#if CUS1_12M_60_FPS
static kal_uint16 imx586_custom1_setting[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x23,
	0x0343, 0x10,
	0x0340, 0x0C,
	0x0341, 0x88,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x17,
	0x034B, 0x6F,
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x08,
	0x3140, 0x00,
	0x3246, 0x81,
	0x3247, 0x81,
	0x3F15, 0x00,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x0F,
	0x040D, 0xA0,
	0x040E, 0x0B,
	0x040F, 0xB8,
	0x034C, 0x0F,
	0x034D, 0xA0,
	0x034E, 0x0B,
	0x034F, 0xB8,
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x68,
	0x030B, 0x01,
	0x030D, 0x04,
	0x030E, 0x01,
	0x030F, 0xA0,
	0x0310, 0x01,
	0x3620, 0x00,
	0x3621, 0x00,
	0x3C11, 0x04,
	0x3C12, 0x03,
	0x3C13, 0x2D,
	0x3F0C, 0x01,
	0x3F14, 0x00,
	0x3F80, 0x00,
	0x3F81, 0x14,
	0x3F8C, 0x00,
	0x3F8D, 0x14,
	0x3FF8, 0x00,
	0x3FF9, 0x3C,
	0x3FFE, 0x01,
	0x3FFF, 0x8C,
	0x0202, 0x0C,
	0x0203, 0x58,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x3E20, 0x02,
	0x3E37, 0x00,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x01,
	0x4435, 0xF0,
};

#else /* CUS1_12M_100_FPS */

static kal_uint16 imx586_custom1_setting[] = {
	/*MIPI output setting*/
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x02,
	/*Line Length PCK Setting*/
	0x0342, 0x15,
	0x0343, 0x00,
	/*Frame Length Lines Setting*/
	0x0340, 0x0C,
	0x0341, 0x5E,
	/*ROI Setting*/
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x17,
	0x034B, 0x6F,
	/*Mode Setting*/
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x09,
	0x3140, 0x00,
	0x3246, 0x81,
	0x3247, 0x81,
	0x3F15, 0x00,
	/*Digital Crop & Scaling*/
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x0F,
	0x040D, 0xA0,
	0x040E, 0x0B,
	0x040F, 0xB8,
	/*Output Size Setting*/
	0x034C, 0x0F,
	0x034D, 0xA0,
	0x034E, 0x0B,
	0x034F, 0xB8,
	/*Clock Setting*/
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x03,
	0x0306, 0x01,
	0x0307, 0x0A,
	0x030B, 0x01,
	0x030D, 0x03,
	0x030E, 0x01,
	0x030F, 0x10,
	0x0310, 0x01,
	/*Other Setting*/
	0x3620, 0x00,
	0x3621, 0x00,
	0x3F0C, 0x00,
	0x3F14, 0x00,
	0x3F80, 0x00,
	0x3F81, 0x00,
	0x3FFC, 0x00,
	0x3FFD, 0x00,
	/*Integration Setting*/
	0x0202, 0x0C,
	0x0203, 0x2E,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	/*Gain Setting*/
	0x0204, 0x00,
	0x0205, 0x00,
	0x0216, 0x00,
	0x0217, 0x00,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x00,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
};
#endif

#if 1
static kal_uint16 imx586_custom2_setting[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x3C,
	0x0343, 0x90,
	0x0340, 0x18,
	0x0341, 0x2F,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x17,
	0x034B, 0x6F,
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x00,
	0x0901, 0x11,
	0x0902, 0x0A,
	0x3140, 0x00,
	0x3246, 0x01,
	0x3247, 0x01,
	0x3F15, 0x00,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x1F,
	0x040D, 0x40,
	0x040E, 0x17,
	0x040F, 0x70,
	0x034C, 0x1F,
	0x034D, 0x40,
	0x034E, 0x17,
	0x034F, 0x70,
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x2C,
	0x030B, 0x01,
	0x030D, 0x04,
	0x030E, 0x01,
	0x030F, 0x81,
	0x0310, 0x01,
	0x3620, 0x01,
	0x3621, 0x00,
	0x3C11, 0x08,
	0x3C12, 0x08,
	0x3C13, 0x2A,
	0x3F0C, 0x00,
	0x3F14, 0x01,
	0x3F80, 0x00,
	0x3F81, 0x14,
	0x3F8C, 0x00,
	0x3F8D, 0x14,
	0x3FF8, 0x00,
	0x3FF9, 0x00,
	0x3FFE, 0x03,
	0x3FFF, 0x65,
	0x0202, 0x17,
	0x0203, 0xFF,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x3E20, 0x02,
	0x3E37, 0x00,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x01,
	0x4435, 0xF0,
};
#endif
#if 1
static kal_uint16 imx586_custom3_setting[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x3C,
	0x0343, 0x90,
	0x0340, 0x18,
	0x0341, 0x2F,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x17,
	0x034B, 0x6F,
	0x0220, 0x62,
	0x0222, 0x01,
	0x0900, 0x00,
	0x0901, 0x11,
	0x0902, 0x0A,
	0x3140, 0x00,
	0x3246, 0x01,
	0x3247, 0x01,
	0x3F15, 0x00,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x1F,
	0x040D, 0x40,
	0x040E, 0x17,
	0x040F, 0x70,
	0x034C, 0x1F,
	0x034D, 0x40,
	0x034E, 0x17,
	0x034F, 0x70,
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x2C,
	0x030B, 0x01,
	0x030D, 0x04,
	0x030E, 0x01,
	0x030F, 0x81,
	0x0310, 0x01,
	0x3620, 0x01,
	0x3621, 0x01, //0x00
	0x3C11, 0x08,
	0x3C12, 0x08,
	0x3C13, 0x2A,
	0x3F0C, 0x00,
	0x3F14, 0x01,
	0x3F80, 0x00,
	0x3F81, 0x14,
	0x3F8C, 0x00,
	0x3F8D, 0x14,
	0x3FF8, 0x00,
	0x3FF9, 0x00,
	0x3FFE, 0x03,
	0x3FFF, 0x65,
	0x0202, 0x17,
	0x0203, 0xFF,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x3E20, 0x02,
	0x3E37, 0x00,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x01,
	0x4435, 0xF0,
};
#endif
#endif

static void sensor_init(void)
{
	LOG_INF("E\n");

	imx586_table_write_cmos_sensor(imx586_init_setting,
		sizeof(imx586_init_setting)/sizeof(kal_uint16));


	set_mirror_flip(imgsensor.mirror);

	LOG_INF("X");
}	/*	  sensor_init  */

static void preview_setting(void)
{
	LOG_INF("E\n");

	imx586_table_write_cmos_sensor(imx586_preview_setting,
		sizeof(imx586_preview_setting)/sizeof(kal_uint16));
	LOG_INF("X");
} /* preview_setting */


/*full size 30fps*/
static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("%s 30 fps E! currefps:%d\n", __func__, currefps);
#if 1
	imx586_table_write_cmos_sensor(imx586_capture_setting,
		sizeof(imx586_capture_setting)/sizeof(kal_uint16));
#else
	/*************MIPI output setting************/
	/*MIPI output setting*/
	write_cmos_sensor_8(0x0112, 0x0A);
	write_cmos_sensor_8(0x0113, 0x0A);
	write_cmos_sensor_8(0x0114, 0x02);
	/*Line Length PCK Setting*/
	write_cmos_sensor_8(0x0342, 0x24);
	write_cmos_sensor_8(0x0343, 0xE0);
	/*Frame Length Lines Setting*/
	write_cmos_sensor_8(0x0340, 0x17);
	write_cmos_sensor_8(0x0341, 0xB3);
	/*ROI Setting*/
	write_cmos_sensor_8(0x0344, 0x00);
	write_cmos_sensor_8(0x0345, 0x00);
	write_cmos_sensor_8(0x0346, 0x00);
	write_cmos_sensor_8(0x0347, 0x00);
	write_cmos_sensor_8(0x0348, 0x1F);
	write_cmos_sensor_8(0x0349, 0x3F);
	write_cmos_sensor_8(0x034A, 0x17);
	write_cmos_sensor_8(0x034B, 0x6F);
	/*Mode Setting*/
	write_cmos_sensor_8(0x0220, 0x62);
	write_cmos_sensor_8(0x0222, 0x01);
	write_cmos_sensor_8(0x0900, 0x00);
	write_cmos_sensor_8(0x0901, 0x11);
	write_cmos_sensor_8(0x0902, 0x08);
	write_cmos_sensor_8(0x3140, 0x00);
	write_cmos_sensor_8(0x3246, 0x01);
	write_cmos_sensor_8(0x3247, 0x01);
	write_cmos_sensor_8(0x3F15, 0x00);
	/*Digital Crop & Scaling*/
	write_cmos_sensor_8(0x0401, 0x00);
	write_cmos_sensor_8(0x0404, 0x00);
	write_cmos_sensor_8(0x0405, 0x10);
	write_cmos_sensor_8(0x0408, 0x00);
	write_cmos_sensor_8(0x0409, 0x00);
	write_cmos_sensor_8(0x040A, 0x00);
	write_cmos_sensor_8(0x040B, 0x00);
	write_cmos_sensor_8(0x040C, 0x1F);
	write_cmos_sensor_8(0x040D, 0x40);
	write_cmos_sensor_8(0x040E, 0x17);
	write_cmos_sensor_8(0x040F, 0x70);
	/*Output Size Setting*/
	write_cmos_sensor_8(0x034C, 0x1F);
	write_cmos_sensor_8(0x034D, 0x40);
	write_cmos_sensor_8(0x034E, 0x17);
	write_cmos_sensor_8(0x034F, 0x70);
	/*Clock Setting*/
	write_cmos_sensor_8(0x0301, 0x05);
	write_cmos_sensor_8(0x0303, 0x02);
	write_cmos_sensor_8(0x0305, 0x04);
	write_cmos_sensor_8(0x0306, 0x01);
	write_cmos_sensor_8(0x0307, 0x66);
	write_cmos_sensor_8(0x030B, 0x01);
	write_cmos_sensor_8(0x030D, 0x0C);
	write_cmos_sensor_8(0x030E, 0x04);
	write_cmos_sensor_8(0x030F, 0xD2);
	write_cmos_sensor_8(0x0310, 0x01);
	/*Other Setting*/
	write_cmos_sensor_8(0x3620, 0x01);
	write_cmos_sensor_8(0x3621, 0x01);
	write_cmos_sensor_8(0x3C11, 0x08);
	write_cmos_sensor_8(0x3C12, 0x08);
	write_cmos_sensor_8(0x3C13, 0x2A);
	write_cmos_sensor_8(0x3F0C, 0x00);
	write_cmos_sensor_8(0x3F14, 0x01);
	write_cmos_sensor_8(0x3F80, 0x02);
	write_cmos_sensor_8(0x3F81, 0x20);
	write_cmos_sensor_8(0x3F8C, 0x01);
	write_cmos_sensor_8(0x3F8D, 0x9A);
	write_cmos_sensor_8(0x3FF8, 0x00);
	write_cmos_sensor_8(0x3FF9, 0x00);
	write_cmos_sensor_8(0x3FFE, 0x02);
	write_cmos_sensor_8(0x3FFF, 0x0E);
	/*Integration Setting*/
	write_cmos_sensor_8(0x0202, 0x17);
	write_cmos_sensor_8(0x0203, 0x83);
	write_cmos_sensor_8(0x0224, 0x01);
	write_cmos_sensor_8(0x0225, 0xF4);
	write_cmos_sensor_8(0x3FE0, 0x01);
	write_cmos_sensor_8(0x3FE1, 0xF4);
	/*Gain Setting*/
	write_cmos_sensor_8(0x0204, 0x00);
	write_cmos_sensor_8(0x0205, 0x70);
	write_cmos_sensor_8(0x0216, 0x00);
	write_cmos_sensor_8(0x0217, 0x70);
	write_cmos_sensor_8(0x0218, 0x01);
	write_cmos_sensor_8(0x0219, 0x00);
	write_cmos_sensor_8(0x020E, 0x01);
	write_cmos_sensor_8(0x020F, 0x00);
	write_cmos_sensor_8(0x0210, 0x01);
	write_cmos_sensor_8(0x0211, 0x00);
	write_cmos_sensor_8(0x0212, 0x01);
	write_cmos_sensor_8(0x0213, 0x00);
	write_cmos_sensor_8(0x0214, 0x01);
	write_cmos_sensor_8(0x0215, 0x00);
	write_cmos_sensor_8(0x3FE2, 0x00);
	write_cmos_sensor_8(0x3FE3, 0x70);
	write_cmos_sensor_8(0x3FE4, 0x01);
	write_cmos_sensor_8(0x3FE5, 0x00);
	/*PDAF TYPE1 Setting*/
	write_cmos_sensor_8(0x3E20, 0x02);
	write_cmos_sensor_8(0x3E37, 0x00);
	/*PDAF TYPE2 Setting*/
	write_cmos_sensor_8(0x3E20, 0x02);
	write_cmos_sensor_8(0x3E3B, 0x01);
	write_cmos_sensor_8(0x4434, 0x00);
	write_cmos_sensor_8(0x4435, 0xF8);

#endif
	LOG_INF("%s 30 fpsX\n", __func__);
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("%s E! currefps:%d\n", __func__, currefps);

#if VIDEO_60_FPS
	LOG_INF("imx586 VIDEO_60_FPS\n");
	imx586_table_write_cmos_sensor(imx586_normal_video_setting_60fps,
		sizeof(imx586_normal_video_setting_60fps)/sizeof(kal_uint16));
#else
	LOG_INF("imx586 VIDEO_30_FPS\n");
	imx586_table_write_cmos_sensor(imx586_normal_video_setting,
		sizeof(imx586_normal_video_setting)/sizeof(kal_uint16));
#endif
	LOG_INF("X\n");
}

static void hs_video_setting(void)
{
	LOG_INF("%s E! currefps 120\n", __func__);


	imx586_table_write_cmos_sensor(imx586_hs_video_setting,
		sizeof(imx586_hs_video_setting)/sizeof(kal_uint16));

	LOG_INF("X\n");
}

static void slim_video_setting(void)
{
	LOG_INF("%s E\n", __func__);

	LOG_INF("%s 30 fps E! currefps\n", __func__);


	imx586_table_write_cmos_sensor(imx586_slim_video_setting,
		sizeof(imx586_slim_video_setting)/sizeof(kal_uint16));

	LOG_INF("X\n");
}

#if 0
static void custom1_setting(void)
{
	LOG_INF("%s custom1_setting E! currefps %d\n",
				__func__, imgsensor.current_fps);
	/*************MIPI output setting************/

	imx586_table_write_cmos_sensor(imx586_custom1_setting,
		sizeof(imx586_custom1_setting)/sizeof(kal_uint16));

	LOG_INF("X");
}


static void custom2_setting(void)
{
#if 0
	LOG_INF("%s 8000*6000@300fps E! currefps\n", __func__);
	/*************MIPI output setting************/
	/*MIPI output setting*/
	write_cmos_sensor_8(0x0112, 0x0A);
	write_cmos_sensor_8(0x0113, 0x0A);
	write_cmos_sensor_8(0x0114, 0x02);
	/*Line Length PCK Setting*/
	write_cmos_sensor_8(0x0342, 0x24);
	write_cmos_sensor_8(0x0343, 0xE0);
	/*Frame Length Lines Setting*/
	write_cmos_sensor_8(0x0340, 0x17);
	write_cmos_sensor_8(0x0341, 0xB3);
	/*ROI Setting*/
	write_cmos_sensor_8(0x0344, 0x00);
	write_cmos_sensor_8(0x0345, 0x00);
	write_cmos_sensor_8(0x0346, 0x00);
	write_cmos_sensor_8(0x0347, 0x00);
	write_cmos_sensor_8(0x0348, 0x1F);
	write_cmos_sensor_8(0x0349, 0x3F);
	write_cmos_sensor_8(0x034A, 0x17);
	write_cmos_sensor_8(0x034B, 0x6F);
	/*Mode Setting*/
	write_cmos_sensor_8(0x0220, 0x62);
	write_cmos_sensor_8(0x0222, 0x01);
	write_cmos_sensor_8(0x0900, 0x00);
	write_cmos_sensor_8(0x0901, 0x11);
	write_cmos_sensor_8(0x0902, 0x08);
	write_cmos_sensor_8(0x3140, 0x00);
	write_cmos_sensor_8(0x3246, 0x01);
	write_cmos_sensor_8(0x3247, 0x01);
	write_cmos_sensor_8(0x3F15, 0x00);
	/*Digital Crop & Scaling*/
	write_cmos_sensor_8(0x0401, 0x00);
	write_cmos_sensor_8(0x0404, 0x00);
	write_cmos_sensor_8(0x0405, 0x10);
	write_cmos_sensor_8(0x0408, 0x00);
	write_cmos_sensor_8(0x0409, 0x00);
	write_cmos_sensor_8(0x040A, 0x00);
	write_cmos_sensor_8(0x040B, 0x00);
	write_cmos_sensor_8(0x040C, 0x1F);
	write_cmos_sensor_8(0x040D, 0x40);
	write_cmos_sensor_8(0x040E, 0x17);
	write_cmos_sensor_8(0x040F, 0x70);
	/*Output Size Setting*/
	write_cmos_sensor_8(0x034C, 0x1F);
	write_cmos_sensor_8(0x034D, 0x40);
	write_cmos_sensor_8(0x034E, 0x17);
	write_cmos_sensor_8(0x034F, 0x70);
	/*Clock Setting*/
	write_cmos_sensor_8(0x0301, 0x05);
	write_cmos_sensor_8(0x0303, 0x02);
	write_cmos_sensor_8(0x0305, 0x04);
	write_cmos_sensor_8(0x0306, 0x01);
	write_cmos_sensor_8(0x0307, 0x66);
	write_cmos_sensor_8(0x030B, 0x01);
	write_cmos_sensor_8(0x030D, 0x0C);
	write_cmos_sensor_8(0x030E, 0x04);
	write_cmos_sensor_8(0x030F, 0xD2);
	write_cmos_sensor_8(0x0310, 0x01);
	/*Other Setting*/
	write_cmos_sensor_8(0x3620, 0x01);
	write_cmos_sensor_8(0x3621, 0x01);
	write_cmos_sensor_8(0x3C11, 0x08);
	write_cmos_sensor_8(0x3C12, 0x08);
	write_cmos_sensor_8(0x3C13, 0x2A);
	write_cmos_sensor_8(0x3F0C, 0x00);
	write_cmos_sensor_8(0x3F14, 0x01);
	write_cmos_sensor_8(0x3F80, 0x02);
	write_cmos_sensor_8(0x3F81, 0x20);
	write_cmos_sensor_8(0x3F8C, 0x01);
	write_cmos_sensor_8(0x3F8D, 0x9A);
	write_cmos_sensor_8(0x3FF8, 0x00);
	write_cmos_sensor_8(0x3FF9, 0x00);
	write_cmos_sensor_8(0x3FFE, 0x02);
	write_cmos_sensor_8(0x3FFF, 0x0E);
	/*Integration Setting*/
	write_cmos_sensor_8(0x0202, 0x17);
	write_cmos_sensor_8(0x0203, 0x83);
	write_cmos_sensor_8(0x0224, 0x01);
	write_cmos_sensor_8(0x0225, 0xF4);
	write_cmos_sensor_8(0x3FE0, 0x01);
	write_cmos_sensor_8(0x3FE1, 0xF4);
	/*Gain Setting*/
	write_cmos_sensor_8(0x0204, 0x00);
	write_cmos_sensor_8(0x0205, 0x70);
	write_cmos_sensor_8(0x0216, 0x00);
	write_cmos_sensor_8(0x0217, 0x70);
	write_cmos_sensor_8(0x0218, 0x01);
	write_cmos_sensor_8(0x0219, 0x00);
	write_cmos_sensor_8(0x020E, 0x01);
	write_cmos_sensor_8(0x020F, 0x00);
	write_cmos_sensor_8(0x0210, 0x01);
	write_cmos_sensor_8(0x0211, 0x00);
	write_cmos_sensor_8(0x0212, 0x01);
	write_cmos_sensor_8(0x0213, 0x00);
	write_cmos_sensor_8(0x0214, 0x01);
	write_cmos_sensor_8(0x0215, 0x00);
	write_cmos_sensor_8(0x3FE2, 0x00);
	write_cmos_sensor_8(0x3FE3, 0x70);
	write_cmos_sensor_8(0x3FE4, 0x01);
	write_cmos_sensor_8(0x3FE5, 0x00);
	/*PDAF TYPE1 Setting*/
	write_cmos_sensor_8(0x3E20, 0x02);
	write_cmos_sensor_8(0x3E37, 0x00);
	/*PDAF TYPE2 Setting*/
	write_cmos_sensor_8(0x3E20, 0x02);
	write_cmos_sensor_8(0x3E3B, 0x01);
	write_cmos_sensor_8(0x4434, 0x00);
	write_cmos_sensor_8(0x4435, 0xF8);

	LOG_INF("%s 30 fpsX\n", __func__);
#else
	imx586_table_write_cmos_sensor(imx586_custom2_setting, sizeof(imx586_custom2_setting)/sizeof(kal_uint16));
	LOG_INF("X");
#endif
}
static void custom3_setting(void)
{
#if 0
	LOG_INF("%s 8000*6000@300fps E! currefps\n", __func__);
	/*************MIPI output setting************/
	/*MIPI output setting*/
	write_cmos_sensor_8(0x0112, 0x0A);
	write_cmos_sensor_8(0x0113, 0x0A);
	write_cmos_sensor_8(0x0114, 0x02);
	/*Line Length PCK Setting*/
	write_cmos_sensor_8(0x0342, 0x24);
	write_cmos_sensor_8(0x0343, 0xE0);
	/*Frame Length Lines Setting*/
	write_cmos_sensor_8(0x0340, 0x17);
	write_cmos_sensor_8(0x0341, 0xB3);
	/*ROI Setting*/
	write_cmos_sensor_8(0x0344, 0x00);
	write_cmos_sensor_8(0x0345, 0x00);
	write_cmos_sensor_8(0x0346, 0x00);
	write_cmos_sensor_8(0x0347, 0x00);
	write_cmos_sensor_8(0x0348, 0x1F);
	write_cmos_sensor_8(0x0349, 0x3F);
	write_cmos_sensor_8(0x034A, 0x17);
	write_cmos_sensor_8(0x034B, 0x6F);
	/*Mode Setting*/
	write_cmos_sensor_8(0x0220, 0x62);
	write_cmos_sensor_8(0x0222, 0x01);
	write_cmos_sensor_8(0x0900, 0x00);
	write_cmos_sensor_8(0x0901, 0x11);
	write_cmos_sensor_8(0x0902, 0x08);
	write_cmos_sensor_8(0x3140, 0x00);
	write_cmos_sensor_8(0x3246, 0x01);
	write_cmos_sensor_8(0x3247, 0x01);
	write_cmos_sensor_8(0x3F15, 0x00);
	/*Digital Crop & Scaling*/
	write_cmos_sensor_8(0x0401, 0x00);
	write_cmos_sensor_8(0x0404, 0x00);
	write_cmos_sensor_8(0x0405, 0x10);
	write_cmos_sensor_8(0x0408, 0x00);
	write_cmos_sensor_8(0x0409, 0x00);
	write_cmos_sensor_8(0x040A, 0x00);
	write_cmos_sensor_8(0x040B, 0x00);
	write_cmos_sensor_8(0x040C, 0x1F);
	write_cmos_sensor_8(0x040D, 0x40);
	write_cmos_sensor_8(0x040E, 0x17);
	write_cmos_sensor_8(0x040F, 0x70);
	/*Output Size Setting*/
	write_cmos_sensor_8(0x034C, 0x1F);
	write_cmos_sensor_8(0x034D, 0x40);
	write_cmos_sensor_8(0x034E, 0x17);
	write_cmos_sensor_8(0x034F, 0x70);
	/*Clock Setting*/
	write_cmos_sensor_8(0x0301, 0x05);
	write_cmos_sensor_8(0x0303, 0x02);
	write_cmos_sensor_8(0x0305, 0x04);
	write_cmos_sensor_8(0x0306, 0x01);
	write_cmos_sensor_8(0x0307, 0x66);
	write_cmos_sensor_8(0x030B, 0x01);
	write_cmos_sensor_8(0x030D, 0x0C);
	write_cmos_sensor_8(0x030E, 0x04);
	write_cmos_sensor_8(0x030F, 0xD2);
	write_cmos_sensor_8(0x0310, 0x01);
	/*Other Setting*/
	write_cmos_sensor_8(0x3620, 0x01);
	write_cmos_sensor_8(0x3621, 0x01);
	write_cmos_sensor_8(0x3C11, 0x08);
	write_cmos_sensor_8(0x3C12, 0x08);
	write_cmos_sensor_8(0x3C13, 0x2A);
	write_cmos_sensor_8(0x3F0C, 0x00);
	write_cmos_sensor_8(0x3F14, 0x01);
	write_cmos_sensor_8(0x3F80, 0x02);
	write_cmos_sensor_8(0x3F81, 0x20);
	write_cmos_sensor_8(0x3F8C, 0x01);
	write_cmos_sensor_8(0x3F8D, 0x9A);
	write_cmos_sensor_8(0x3FF8, 0x00);
	write_cmos_sensor_8(0x3FF9, 0x00);
	write_cmos_sensor_8(0x3FFE, 0x02);
	write_cmos_sensor_8(0x3FFF, 0x0E);
	/*Integration Setting*/
	write_cmos_sensor_8(0x0202, 0x17);
	write_cmos_sensor_8(0x0203, 0x83);
	write_cmos_sensor_8(0x0224, 0x01);
	write_cmos_sensor_8(0x0225, 0xF4);
	write_cmos_sensor_8(0x3FE0, 0x01);
	write_cmos_sensor_8(0x3FE1, 0xF4);
	/*Gain Setting*/
	write_cmos_sensor_8(0x0204, 0x00);
	write_cmos_sensor_8(0x0205, 0x70);
	write_cmos_sensor_8(0x0216, 0x00);
	write_cmos_sensor_8(0x0217, 0x70);
	write_cmos_sensor_8(0x0218, 0x01);
	write_cmos_sensor_8(0x0219, 0x00);
	write_cmos_sensor_8(0x020E, 0x01);
	write_cmos_sensor_8(0x020F, 0x00);
	write_cmos_sensor_8(0x0210, 0x01);
	write_cmos_sensor_8(0x0211, 0x00);
	write_cmos_sensor_8(0x0212, 0x01);
	write_cmos_sensor_8(0x0213, 0x00);
	write_cmos_sensor_8(0x0214, 0x01);
	write_cmos_sensor_8(0x0215, 0x00);
	write_cmos_sensor_8(0x3FE2, 0x00);
	write_cmos_sensor_8(0x3FE3, 0x70);
	write_cmos_sensor_8(0x3FE4, 0x01);
	write_cmos_sensor_8(0x3FE5, 0x00);
	/*PDAF TYPE1 Setting*/
	write_cmos_sensor_8(0x3E20, 0x02);
	write_cmos_sensor_8(0x3E37, 0x00);
	/*PDAF TYPE2 Setting*/
	write_cmos_sensor_8(0x3E20, 0x02);
	write_cmos_sensor_8(0x3E3B, 0x01);
	write_cmos_sensor_8(0x4434, 0x00);
	write_cmos_sensor_8(0x4435, 0xF8);

	LOG_INF("%s 30 fpsX\n", __func__);
#else
	imx586_table_write_cmos_sensor(imx586_custom3_setting, sizeof(imx586_custom3_setting)/sizeof(kal_uint16));
	LOG_INF("X");
#endif
}
#endif

/*************************************************************************
 * FUNCTION
 *	get_imgsensor_id
 *
 * DESCRIPTION
 *	This function get the sensor ID
 *
 * PARAMETERS
 *	*sensorID : return the sensor ID
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	/*sensor have two i2c address 0x34 & 0x20,
	 *we should detect the module used i2c address
	 */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = ((read_cmos_sensor_8(0x0016) << 8)
					| read_cmos_sensor_8(0x0017));
			printk(
				"read_0x0000=0x%x, 0x0001=0x%x,0x0000_0001=0x%x\n",
				read_cmos_sensor_8(0x0016),
				read_cmos_sensor_8(0x0017),
				read_cmos_sensor(0x0000));
			if (*sensor_id == imgsensor_info.sensor_id) {
				printk("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, *sensor_id);	
				read_imx586_QSC();
				return ERROR_NONE;
			}

			printk("Read sensor id fail, id: 0x%x\n",
				imgsensor.i2c_write_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		/*if Sensor ID is not correct,
		 *Must set *sensor_id to 0xFFFFFFFF
		 */
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
 * FUNCTION
 *	open
 *
 * DESCRIPTION
 *	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0;


	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
	 *we should detect the module used i2c address
	 */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = ((read_cmos_sensor_8(0x0016) << 8)
					| read_cmos_sensor_8(0x0017));
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail, id: 0x%x\n",
				imgsensor.i2c_write_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
	sensor_init();

#ifdef IMX586_PDAF_ON
	imx586_apply_LRC();
#endif
	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x3D0;
	imgsensor.gain = 0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_mode = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
} /* open */

/*************************************************************************
 * FUNCTION
 *	close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/

	streaming_control(KAL_FALSE);
	qsc_flag = 0;
	
	return ERROR_NONE;
} /* close */


/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *	This function start the sensor preview.
 *
 * PARAMETERS
 *	*image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	preview_setting();

	return ERROR_NONE;
} /* preview */

/*************************************************************************
 * FUNCTION
 *	capture
 *
 * DESCRIPTION
 *	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
#if 0
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
	/* PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M */
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else if (imgsensor.current_fps == imgsensor_info.cap2.max_framerate) {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF(
			"Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",
		imgsensor.current_fps, imgsensor_info.cap1.max_framerate/10);
		imgsensor.pclk = imgsensor_info.cap2.pclk;
		imgsensor.line_length = imgsensor_info.cap2.linelength;
		imgsensor.frame_length = imgsensor_info.cap2.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap2.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF(
			"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.cap.max_framerate / 10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
#else
	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		LOG_INF(
			"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.cap.max_framerate / 10);
	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
#endif

	spin_unlock(&imgsensor_drv_lock);

	streaming_control(KAL_FALSE);
	capture_setting(imgsensor.current_fps);
	if (!qsc_flag) {
		LOG_INF("write_sensor_QSC Start\n");
		write_sensor_QSC();
		LOG_INF("write_sensor_QSC End\n");
		qsc_flag = 1;
	}
	streaming_control(KAL_TRUE);
	
	//capture_setting(imgsensor.current_fps);


	/* set_mirror_flip(imgsensor.mirror); */

	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	/*set_mirror_flip(imgsensor.mirror);*/

	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/*imgsensor.video_mode = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	/*set_mirror_flip(imgsensor.mirror);*/

	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	/*imgsensor.video_mode = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	/*set_mirror_flip(imgsensor.mirror);*/

	return ERROR_NONE;
}	/* slim_video */

#if 0
static kal_uint32 custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s.\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	imgsensor.pclk = imgsensor_info.custom1.pclk;
	imgsensor.line_length = imgsensor_info.custom1.linelength;
	imgsensor.frame_length = imgsensor_info.custom1.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom1_setting();


	return ERROR_NONE;
}	/* custom1 */

static kal_uint32 custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s.\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	imgsensor.pclk = imgsensor_info.custom2.pclk;
	imgsensor.line_length = imgsensor_info.custom2.linelength;
	imgsensor.frame_length = imgsensor_info.custom2.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom2_setting();


	return ERROR_NONE;
}	/* custom2 */
static kal_uint32 custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s.\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
	imgsensor.pclk = imgsensor_info.custom3.pclk;
	imgsensor.line_length = imgsensor_info.custom3.linelength;
	imgsensor.frame_length = imgsensor_info.custom3.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	
	if (!qsc_flag) {
		LOG_INF("write_sensor_QSC Start\n");
		write_sensor_QSC();
		LOG_INF("write_sensor_QSC End\n");
		qsc_flag = 1;
	}
	
	custom3_setting();

	return ERROR_NONE;
}	/* custom3 */
#endif

static kal_uint32
get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth =
		imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight =
		imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth =
		imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight =
		imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth =
		imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight =
		imgsensor_info.normal_video.grabwindow_height;

	sensor_resolution->SensorHighSpeedVideoWidth =
		imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight =
		imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth =
		imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight =
		imgsensor_info.slim_video.grabwindow_height;
#if 0
	sensor_resolution->SensorCustom1Width =
		imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height =
		imgsensor_info.custom1.grabwindow_height;

	sensor_resolution->SensorCustom2Width =
		imgsensor_info.custom2.grabwindow_width;
	sensor_resolution->SensorCustom2Height =
		imgsensor_info.custom2.grabwindow_height;

	sensor_resolution->SensorCustom3Width =
		imgsensor_info.custom3.grabwindow_width;
	sensor_resolution->SensorCustom3Height =
		imgsensor_info.custom3.grabwindow_height;
#endif
	return ERROR_NONE;
} /* get_resolution */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id1 = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat =
		imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame =
		imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame =
		imgsensor_info.slim_video_delay_frame;
#if 0		
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
	sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame;
#endif
	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->FrameTimeDelayFrame = imgsensor_info.frame_time_delay_frame; /* The delay frame of setting frame length  */
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->TEMPERATURE_SUPPORT = imgsensor_info.temperature_support;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	#ifdef IMX586_PDAF_ON
	sensor_info->PDAF_Support = 2;
	#else
	sensor_info->PDAF_Support = 0;
	#endif
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0; /* 0 is default 1x */
	sensor_info->SensorHightSampling = 0; /* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		LOG_INF("E55\n");

		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.cap.mipi_data_lp2hs_settle_dc;
		LOG_INF("E56\n");

		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

		sensor_info->SensorGrabStartX =
			imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.normal_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX =
			imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.slim_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

		break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
#if 0
	case MSDK_SCENARIO_ID_CUSTOM1:
		sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
		break;

	case MSDK_SCENARIO_ID_CUSTOM2:
		sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom2.mipi_data_lp2hs_settle_dc;
		LOG_INF("E57\n");
		break;

		case MSDK_SCENARIO_ID_CUSTOM3:
		sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom3.mipi_data_lp2hs_settle_dc;
		LOG_INF("E577\n");
		break;
#endif
	}

	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		LOG_INF("E5\n");
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		LOG_INF("E6\n");
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		LOG_INF("E7\n");
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		LOG_INF("E8\n");
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		LOG_INF("E9\n");
		slim_video(image_window, sensor_config_data);
		break;
#if 0
	case MSDK_SCENARIO_ID_CUSTOM1:
		LOG_INF("E10\n");
		custom1(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		LOG_INF("E11\n");
		custom2(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		LOG_INF("E111\n");
		custom3(image_window, sensor_config_data);
		break;
#endif
	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}

	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
		/* Dynamic frame rate */
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) /*enable auto flicker*/
		imgsensor.autoflicker_en = KAL_TRUE;
	else /*Cancel Auto flick*/
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(
		enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id3 = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk / framerate * 10
				/ imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.pre.framelength)
		? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.pre.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk /
				framerate * 10 /
				imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.normal_video.framelength)
		? (frame_length - imgsensor_info.normal_video.framelength)
		: 0;
		imgsensor.frame_length =
			imgsensor_info.normal_video.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
#if 0
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
		frame_length = imgsensor_info.cap1.pclk / framerate * 10
				/ imgsensor_info.cap1.linelength;
		spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line =
			(frame_length > imgsensor_info.cap1.framelength)
			? (frame_length - imgsensor_info.cap1.framelength) : 0;
			imgsensor.frame_length =
				imgsensor_info.cap1.framelength
				+ imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
	} else if (imgsensor.current_fps == imgsensor_info.cap2.max_framerate) {
		frame_length = imgsensor_info.cap2.pclk / framerate * 10
					/ imgsensor_info.cap2.linelength;
		spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line =
			  (frame_length > imgsensor_info.cap2.framelength)
			  ? (frame_length - imgsensor_info.cap2.framelength)
			  : 0;
			imgsensor.frame_length =
				imgsensor_info.cap2.framelength
				+ imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
	} else {
			if (imgsensor.current_fps
				!= imgsensor_info.cap.max_framerate)
			LOG_INF(
			"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n"
			, framerate, imgsensor_info.cap.max_framerate/10);
		frame_length = imgsensor_info.cap.pclk / framerate * 10
				/ imgsensor_info.cap.linelength;
		spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line =
			(frame_length > imgsensor_info.cap.framelength)
			  ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length =
				imgsensor_info.cap.framelength
				+ imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
	}
#else
	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		LOG_INF(
			"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n"
			, framerate, imgsensor_info.cap.max_framerate/10);
		frame_length = imgsensor_info.cap.pclk / framerate * 10
				/ imgsensor_info.cap.linelength;
		spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line =
			(frame_length > imgsensor_info.cap.framelength)
			  ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length =
				imgsensor_info.cap.framelength
				+ imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
#endif
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10
				/ imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.hs_video.framelength)
			  ? (frame_length - imgsensor_info.hs_video.framelength)
			  : 0;
		imgsensor.frame_length =
			imgsensor_info.hs_video.framelength
				+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10
			/ imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.slim_video.framelength)
			? (frame_length - imgsensor_info.slim_video.framelength)
			: 0;
		imgsensor.frame_length =
			imgsensor_info.slim_video.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
#if 0
	case MSDK_SCENARIO_ID_CUSTOM1:
		frame_length = imgsensor_info.custom1.pclk / framerate * 10
				/ imgsensor_info.custom1.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.custom1.framelength)
			? (frame_length - imgsensor_info.custom1.framelength)
			: 0;
		imgsensor.frame_length =
			imgsensor_info.custom1.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom2.pclk / framerate * 10
				/ imgsensor_info.custom2.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.custom2.framelength)
			? (frame_length - imgsensor_info.custom2.framelength)
			: 0;
		imgsensor.frame_length =
			imgsensor_info.custom2.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		frame_length = imgsensor_info.custom3.pclk / framerate * 10
				/ imgsensor_info.custom3.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.custom3.framelength)
			? (frame_length - imgsensor_info.custom3.framelength)
			: 0;
		imgsensor.frame_length =
			imgsensor_info.custom3.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
#endif

	default:  /*coding with  preview scenario by default*/
		frame_length = imgsensor_info.pre.pclk / framerate * 10
			/ imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		LOG_INF("error scenario_id = %d, we use preview scenario\n",
			scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(
		enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	LOG_INF("scenario_id2 = %d\n", scenario_id);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
#if 0
	case MSDK_SCENARIO_ID_CUSTOM1:
		*framerate = imgsensor_info.custom1.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		*framerate = imgsensor_info.custom2.max_framerate;
		break;

	case MSDK_SCENARIO_ID_CUSTOM3:
		*framerate = imgsensor_info.custom3.max_framerate;
		break;
#endif
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable)
		write_cmos_sensor_8(0x0601, 0x0002); /*100% Color bar*/
	else
		write_cmos_sensor_8(0x0601, 0x0000); /*No pattern*/

	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}
static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				 UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *) feature_para;
	/* unsigned long long *feature_return_para
	 *  = (unsigned long long *) feature_para;
	 */
	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	struct SENSOR_VC_INFO_STRUCT *pvcinfo;
	/* SET_SENSOR_AWB_GAIN *pSetSensorAWB
	 *  = (SET_SENSOR_AWB_GAIN *)feature_para;
	 */
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data
		= (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	/*LOG_INF("feature_id = %d\n", feature_id);*/
	switch (feature_id) {
	case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_gain;
		*(feature_data + 2) = imgsensor_info.max_gain;
		break;
	case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:
		*(feature_data + 0) = imgsensor_info.min_gain_iso;
		*(feature_data + 1) = imgsensor_info.gain_step;
		*(feature_data + 2) = imgsensor_info.gain_type;
		break;
	case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_shutter;
		*(feature_data + 2) = imgsensor_info.exp_step;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.pclk;
		LOG_INF("E78\n");
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.pclk;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.pclk;
			break;
	#if 0
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom1.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom2.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom3.pclk;
			break;
	#endif

		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.slim_video.pclk;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.pclk;
		LOG_INF("E77\n");
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.cap.framelength << 16)
				+ imgsensor_info.cap.linelength;
		LOG_INF("E76\n");
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.normal_video.framelength << 16)
				+ imgsensor_info.normal_video.linelength;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.hs_video.framelength << 16)
				+ imgsensor_info.hs_video.linelength;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.slim_video.framelength << 16)
				+ imgsensor_info.slim_video.linelength;
			break;
	#if 0
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom1.framelength << 16)
				+ imgsensor_info.custom1.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom2.framelength << 16)
				+ imgsensor_info.custom2.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom3.framelength << 16)
				+ imgsensor_info.custom3.linelength;
			break;
	#endif


		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
		LOG_INF("E75\n");
			break;
		}
		break;

	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		 set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		 /* night_mode((BOOL) *feature_data); */
		break;
	#ifdef VENDOR_EDIT
	case SENSOR_FEATURE_CHECK_MODULE_ID:
		*feature_return_para_32 = imgsensor_info.module_id;
		break;
	#endif
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor_8(sensor_reg_data->RegAddr,
				    sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData =
			read_cmos_sensor_8(sensor_reg_data->RegAddr);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/*get the lens driver ID from EEPROM
		 * or just return LENS_DRIVER_ID_DO_NOT_CARE
		 * if EEPROM does not exist in camera module.
		 */
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL)*feature_data_16,
				      *(feature_data_16+1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		 set_max_framerate_by_scenario(
				(enum MSDK_SCENARIO_ID_ENUM)*feature_data,
				*(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		 get_default_framerate_by_scenario(
				(enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
				(MUINT32 *)(uintptr_t)(*(feature_data+1)));
		break;
	case SENSOR_FEATURE_GET_PDAF_DATA:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
		#if 0
		read_3P8_eeprom((kal_uint16)(*feature_data),
				(char *)(uintptr_t)(*(feature_data+1)),
				(kal_uint32)(*(feature_data+2)));
		#endif
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		/* for factory mode auto testing */
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", (UINT32)*feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_mode = *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
			(UINT32)*feature_data);
		wininfo =
	(struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[1],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		LOG_INF("E72\n");
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[2],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[3],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[4],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
	#if 0
		case MSDK_SCENARIO_ID_CUSTOM1:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[5],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[6],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;

		case MSDK_SCENARIO_ID_CUSTOM3:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[7],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
	#endif
			
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[0],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		LOG_INF("E71\n");
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PDAF_INFO:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n",
			(UINT16) *feature_data);
		PDAFinfo =
		  (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			imgsensor_pd_info.i4BlockNumX = 496;
			imgsensor_pd_info.i4BlockNumY = 186;
			memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info,
				 sizeof(struct SET_PD_BLOCK_INFO_T));
			LOG_INF("E74\n");
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			imgsensor_pd_info.i4BlockNumX = 496;
			imgsensor_pd_info.i4BlockNumY = 186;
			memcpy((void *)PDAFinfo,
				(void *)&imgsensor_pd_info,
				sizeof(struct SET_PD_BLOCK_INFO_T));
			LOG_INF("E73\n");
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)PDAFinfo,
				(void *)&imgsensor_pd_info,
				sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:

		default:
			break;
		}
		break;
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		LOG_INF(
		"SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n",
			(UINT16) *feature_data);
		/*PDAF capacity*/
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
		LOG_INF("E69\n");
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			/* video & capture use same setting */
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			LOG_INF("E70\n");
			break;
	#if 0
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
	#endif
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PDAF_REG_SETTING:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_REG_SETTING %d",
			(*feature_para_len));
		imx586_get_pdaf_reg_setting((*feature_para_len) / sizeof(UINT32)
					   , feature_data_16);
		break;
	case SENSOR_FEATURE_SET_PDAF_REG_SETTING:
		LOG_INF("SENSOR_FEATURE_SET_PDAF_REG_SETTING %d",
			(*feature_para_len));
		imx586_set_pdaf_reg_setting((*feature_para_len) / sizeof(UINT32)
					   , feature_data_16);
		break;
	case SENSOR_FEATURE_SET_PDAF:
		LOG_INF("PDAF mode :%d\n", *feature_data_16);
		imgsensor.pdaf_mode = *feature_data_16;
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16)*feature_data,
			(UINT16)*(feature_data+1),
			(UINT16)*(feature_data+2));
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16) (*feature_data),
					(UINT16) (*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
		break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data+1));
		#if 0
		ihdr_write_shutter((UINT16)*feature_data,
				   (UINT16)*(feature_data+1));
		#endif
		break;

	case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
		/*
		 * 1, if driver support new sw frame sync
		 * set_shutter_frame_length() support third para auto_extend_en
		 */
		*(feature_data + 1) = 1;
		/* margin info by scenario */
		*(feature_data + 2) = imgsensor_info.margin;
		break;

	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
			*feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_BINNING_TYPE:
		switch (*(feature_data + 1)) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*feature_return_para_32 = 1; /*BINNING_NONE*/
		LOG_INF("E68\n");
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*feature_return_para_32 = 4; /*BINNING_SUMMED*/
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*feature_return_para_32 = 2; /*BINNING_AVERAGED*/
		LOG_INF("E67\n");
			break;
		}
		pr_debug("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
			*feature_return_para_32);
		*feature_para_len = 4;

		break;
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
	{
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.slim_video.mipi_pixel_rate;
			break;
	#if 0
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom1.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom2.mipi_pixel_rate;
			break;

		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom3.mipi_pixel_rate;
			break;
	#endif

		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
	}
break;

	case SENSOR_FEATURE_GET_VC_INFO:
		LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n",
			(UINT16)*feature_data);
		pvcinfo =
		 (struct SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1],
				sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[2],
				sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0],
				sizeof(struct SENSOR_VC_INFO_STRUCT));
			LOG_INF("E65\n");
			break;
	#if 0
		case MSDK_SCENARIO_ID_CUSTOM1:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1],
				sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1],
				sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1],
				sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
	#endif

		default:
			#if 0
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0],
				sizeof(struct SENSOR_VC_INFO_STRUCT));
			#endif
			break;
		}
	default:
		break;
	}

	return ERROR_NONE;
} /* feature_control() */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 IMX586_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
} /* IMX586_MIPI_RAW_SensorInit */
