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
 *     gc5035mipi_Sensor.c
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
 */
#define PFX "gc5035_camera_sensor"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__

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

#include "gc5035mipiraw_Sensor.h"

#define MULTI_WRITE 1
#define LONG_EXP               1




static DEFINE_SPINLOCK(imgsensor_drv_lock);
static kal_uint32 Dgain_ratio = GC5035_SENSOR_DGAIN_BASE;

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = GC5035_SENSOR_ID,

	.checksum_value = 0x3f04a99,

	.pre = {
		.pclk = 87600000,
		.linelength = 1460,
		.framelength = 2008,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1296,
		.grabwindow_height = 972,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 87600000,
		.max_framerate = 299,
	},
	.cap = {
		.pclk = 175200000,
		.linelength = 2920,
		.framelength = 2008,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 175200000,
		.max_framerate = 299,
	},
	.normal_video = {
		.pclk = 175200000,
		.linelength = 2920,
		.framelength = 2008,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 175200000,
		.max_framerate = 299,
	},
	.hs_video = {
		.pclk = 87600000,
		.linelength = 1460,
		.framelength = 2008,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1296,
		.grabwindow_height = 972,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 87600000,
		.max_framerate = 299,
	},
	.slim_video = {
		.pclk = 87600000,
		.linelength = 1460,
		.framelength = 2008,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 87600000,
		.max_framerate = 299,
	},

	.margin = 16,
	.min_shutter = 4,
	.min_gain = 64,/*1x*/
	.max_gain = 1024,/*16x*/
	.min_gain_iso = 100,
	.exp_step = 1,
	.gain_step = 1,
	.gain_type = 4,
	.max_frame_length = 0x3fff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,	/* isp gain delay frame for AE cycle */
	.ihdr_support = 0,	/* 1, support; 0,not support */
	.ihdr_le_firstline = 0,	/* 1,le first ; 0, se first */
	.temperature_support = 0,/* 1, support; 0,not support */
	.sensor_mode_num = 5,	/* support sensor mode num */
	.cap_delay_frame = 2,	/* enter capture delay frame num */
	.pre_delay_frame = 2,	/* enter preview delay frame num */
	.video_delay_frame = 2,	/* enter video delay frame num */
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,	/* enter slim video delay frame num */
	.frame_time_delay_frame = 2,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,
#if GC5035_MIRROR_FLIP_ENABLE
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,
#else
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
#endif
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_2_LANE,
	.i2c_addr_table = {0x6e, 0x7e, 0xff},
	.i2c_speed = 400,
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,	/* mirrorflip information */
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x258,
	.gain = 0x40,
	.dummy_pixel = 0,	/* current dummypixel */
	.dummy_line = 0,	/* current dummyline */
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_mode = 0,
	.i2c_write_id = 0x6e,
};

static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
	{ 2592, 1944,   0,   0, 2592, 1944, 1296,  972, 0, 0, 1296,  972, 0, 0, 1296,  972 }, /* preview */
	{ 2592, 1944,   0,   0, 2592, 1944, 2592, 1944, 0, 0, 2592, 1944, 0, 0, 2592, 1944 }, /* cpature */
	{ 2592, 1944,   0,   0, 2592, 1944, 2592, 1944, 0, 0, 2592, 1944, 0, 0, 2592, 1944 }, /* normal video */
	{ 2592, 1944,   0,   0, 2592, 1944, 1296,  972, 0, 0, 1296,  972, 0, 0, 1296,  972 }, /* hs video */
	{ 2592, 1944,   0,   0, 2592, 1944, 1296,  972, 8, 126, 1280,  720, 0, 0, 1280,  720 },  /* slim video */
};

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[1] = {(char)(addr & 0xff)};

	iReadRegI2C(pu_send_cmd, 1, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[2] = {(char)(addr & 0xff), (char)(para & 0xff)};

	iWriteRegI2C(pu_send_cmd, 2, imgsensor.i2c_write_id);
}


#if MULTI_WRITE
#define I2C_BUFFER_LEN  510 /* Max is 255, each 2 bytes */
#else
#define I2C_BUFFER_LEN  2
#endif
static void gc5035_table_write_cmos_sensor(kal_uint16 *para, kal_uint32 len)
{
	char puSendCmd[I2C_BUFFER_LEN];
	kal_uint32 tosend = 0, idx = 0;
	kal_uint16 addr = 0, data = 0;

	while (len > idx) {
		addr = para[idx];
		puSendCmd[tosend++] = (char)(addr & 0xff);
		data = para[idx + 1];
		puSendCmd[tosend++] = (char)(data & 0xff);
		idx += 2;
#if MULTI_WRITE
		if (tosend >= I2C_BUFFER_LEN || idx == len) {
			iBurstWriteReg_multi(puSendCmd, tosend, imgsensor.i2c_write_id,
					2, imgsensor_info.i2c_speed);
			tosend = 0;
		}
#else
		iWriteRegI2CTiming(puSendCmd, 2, imgsensor.i2c_write_id, imgsensor_info.i2c_speed);
		tosend = 0;
#endif
	}
}

static void set_dummy(void)
{
	kal_uint32 frame_length = imgsensor.frame_length >> 2;

	frame_length = frame_length << 2;
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x41, (frame_length >> 8) & 0x3f);
	write_cmos_sensor(0x42, frame_length & 0xff);
	pr_debug("Exit! framelength = %d\n", imgsensor.frame_length);
}	/*	set_dummy  */

static void set_max_framerate(kal_uint16 framerate, kal_bool min_framelength_en)
{
	/*kal_int16 dummy_line;*/
	kal_uint32 frame_length = imgsensor.frame_length;
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


static void set_mirror_flip(kal_uint8 image_mirror)
{
}

static void write_shutter(kal_uint32 shutter)
{
	kal_uint16 realtime_fps = 0, cal_shutter = 0;
	#ifdef LONG_EXP
	/* add for long shutter */
	static bool bNeedSetNormalMode = KAL_FALSE;
	kal_uint32 long_shutter = 0;
	kal_uint16 long_exp_h = 0;
	kal_uint16 long_exp_m = 0;
	kal_uint16 long_exp_l = 0;
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
	#ifndef LONG_EXP
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ?
	(imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
	if (imgsensor.autoflicker_en) {
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else
			set_max_framerate(realtime_fps, 0);
	} else
		set_max_framerate(realtime_fps, 0);
	cal_shutter = shutter >> 2;
	cal_shutter = cal_shutter << 2;
	Dgain_ratio = 256 * shutter / cal_shutter;
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x03, (cal_shutter >> 8) & 0x3F);
	write_cmos_sensor(0x04, cal_shutter & 0xFF);
	#else
	if (shutter >= 0x3fee) {
		pr_debug("enter long exposure mode, time is %d", shutter);
		bNeedSetNormalMode = KAL_TRUE;
		long_shutter = (shutter - 0x3f00) / 4 - 1;
		long_exp_h = (long_shutter >> 16) & 0xF;
		long_exp_m = (long_shutter >> 8) & 0xFF;
		long_exp_l = long_shutter & 0xFF;
		write_cmos_sensor(0xfd, 0x2c);
		write_cmos_sensor(0xef, 0x00);
		write_cmos_sensor(0x3e, 0x00);
		write_cmos_sensor(0xfd, 0x2c);
		write_cmos_sensor(0xef, 0x02);
		write_cmos_sensor(0x81, long_exp_h);
		write_cmos_sensor(0x82, long_exp_m);
		write_cmos_sensor(0x83, long_exp_l);
		write_cmos_sensor(0xef, 0x00);
		write_cmos_sensor(0x17, (GC5035_MIRROR | 0x04));
		write_cmos_sensor(0xfd, 0x2d);
		write_cmos_sensor(0xef, 0x00);
		write_cmos_sensor(0x03, 0x3f);
		write_cmos_sensor(0x04, 0x00);
		write_cmos_sensor(0x41, 0x3f);
		write_cmos_sensor(0x42, 0x10);
		write_cmos_sensor(0xfd, 0x2c);

	} else {
		if (bNeedSetNormalMode) {
			pr_debug("exit long exposure mode");
			bNeedSetNormalMode = KAL_FALSE;
			write_cmos_sensor(0xfd, 0x2d);
			write_cmos_sensor(0xef, 0x00);
			write_cmos_sensor(0x36, 0x01);
			write_cmos_sensor(0x17, GC5035_MIRROR);
			write_cmos_sensor(0x36, 0x00);
			write_cmos_sensor(0x3e, 0x00);
			write_cmos_sensor(0xef, 0x02);
			write_cmos_sensor(0x81, 0x00);
			write_cmos_sensor(0x82, 0x00);
			write_cmos_sensor(0x83, 0x00);
			write_cmos_sensor(0xfd, 0x2c);
			write_cmos_sensor(0xef, 0x00);
	}

	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ?
		(imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
	if (imgsensor.autoflicker_en) {
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else
			set_max_framerate(realtime_fps, 0);
	} else
		set_max_framerate(realtime_fps, 0);

	/* Update Shutter */
	cal_shutter = shutter >> 2;
	cal_shutter = cal_shutter << 2;
	Dgain_ratio = 256 * shutter / cal_shutter;

	pr_debug("normal mode");
	write_cmos_sensor(0xfd, 0x2c);
	write_cmos_sensor(0xef, 0x00);
	write_cmos_sensor(0x03, (cal_shutter >> 8) & 0x3F);
	write_cmos_sensor(0x04, cal_shutter & 0xFF);

	}
	#endif
	pr_debug("shutter =%d, framelength =%d\n", shutter, imgsensor.frame_length);
	pr_debug("Exit! cal_shutter = %d, ", cal_shutter);

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

static void set_shutter_frame_length(kal_uint32 shutter,
				     kal_uint16 frame_length)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0, cal_shutter = 0;
	kal_int32 dummy_line = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	/*AE doesn't update sensor gain at capture mode,
	 *thus extra exposure lines must be updated here.
	 */
	spin_lock(&imgsensor_drv_lock);
	/* Change frame time */
	if (frame_length > 1)
		dummy_line = frame_length - imgsensor.frame_length;

	imgsensor.frame_length = imgsensor.frame_length + dummy_line;

	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter)
			? imgsensor_info.min_shutter : shutter;
	shutter =
	(shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		? (imgsensor_info.max_frame_length - imgsensor_info.margin)
		: shutter;

	realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 /
	imgsensor.frame_length;

	if (imgsensor.autoflicker_en) {
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			set_max_framerate(realtime_fps, 0);
		}
	} else {
		/* Extend frame length */
		set_max_framerate(realtime_fps, 0);
	}

	/* Update Shutter */
	cal_shutter = shutter >> 2;
	cal_shutter = cal_shutter << 2;
	Dgain_ratio = 256 * shutter / cal_shutter;

	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x03, (cal_shutter >> 8) & 0x3F);
	write_cmos_sensor(0x04, cal_shutter & 0xFF);

	pr_debug(
		"Exit! shutter =%d, framelength =%d/%d, dummy_line=%d,\n",
		shutter, imgsensor.frame_length, frame_length,
		dummy_line);
	pr_debug("Exit! cal_shutter = %d, ", cal_shutter);

}	/* set_shutter_frame_length */

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = gain << 2;

	if (reg_gain < GC5035_SENSOR_GAIN_BASE)
		reg_gain = GC5035_SENSOR_GAIN_BASE;
	else if (reg_gain > GC5035_SENSOR_GAIN_MAX)
		reg_gain = GC5035_SENSOR_GAIN_MAX;

	return (kal_uint16)reg_gain;

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
	kal_uint32 temp_gain;
	kal_int16 gain_index;
	#ifdef LONG_EXP
	/* add for long shutter */
	static bool bNeedSetNormalMode = KAL_FALSE;
	#endif
	kal_uint16 GC5035_AGC_Param[GC5035_SENSOR_GAIN_MAP_SIZE][2] = {
		{  256,  0 },
		{  302,  1 },
		{  358,  2 },
		{  425,  3 },
		{  502,  8 },
		{  599,  9 },
		{  717, 10 },
		{  845, 11 },
		{  998, 12 },
		{ 1203, 13 },
		{ 1434, 14 },
		{ 1710, 15 },
		{ 1997, 16 },
		{ 2355, 17 },
		{ 2816, 18 },
		{ 3318, 19 },
		{ 3994, 20 },
	};

	reg_gain = gain2reg(gain);
	for (gain_index = GC5035_SENSOR_GAIN_MAX_VALID_INDEX - 1; gain_index >= 0; gain_index--)
		if (reg_gain >= GC5035_AGC_Param[gain_index][0])
			break;
	if (imgsensor.shutter >= 0x3fee) {
		pr_debug("enter long exposure!\n");
		bNeedSetNormalMode = KAL_TRUE;
		write_cmos_sensor(0xfd, 0x2d);
		write_cmos_sensor(0xef, 0x00);
		write_cmos_sensor(0xb6, GC5035_AGC_Param[gain_index][1]);
		temp_gain = reg_gain * Dgain_ratio / GC5035_AGC_Param[gain_index][0];
		write_cmos_sensor(0xb1, (temp_gain >> 8) & 0x0f);
		write_cmos_sensor(0xb2, temp_gain & 0xfc);
		write_cmos_sensor(0x3e, 0x91);
		write_cmos_sensor(0xfd, 0x2c);
		} else {
		if (bNeedSetNormalMode) {
			pr_debug("exit long exposure mode");
			bNeedSetNormalMode = KAL_FALSE;
			write_cmos_sensor(0xfd, 0x2d);
			write_cmos_sensor(0xef, 0x00);
			write_cmos_sensor(0xb6, GC5035_AGC_Param[gain_index][1]);
			temp_gain = reg_gain * Dgain_ratio / GC5035_AGC_Param[gain_index][0];
			write_cmos_sensor(0xb1, (temp_gain >> 8) & 0x0f);
			write_cmos_sensor(0xb2, temp_gain & 0xfc);
			write_cmos_sensor(0xfd, 0x2e);
			write_cmos_sensor(0x3e, 0x91);
			write_cmos_sensor(0xfd, 0x2c);
	}
			pr_debug("normal mode");
			write_cmos_sensor(0xfd, 0x2c);
			write_cmos_sensor(0xef, 0x00);
			write_cmos_sensor(0xb6, GC5035_AGC_Param[gain_index][1]);
			temp_gain = reg_gain * Dgain_ratio / GC5035_AGC_Param[gain_index][0];
			write_cmos_sensor(0xb1, (temp_gain >> 8) & 0x0f);
			write_cmos_sensor(0xb2, temp_gain & 0xfc);
	}
	pr_debug("Exit! GC5035_AGC_Param[gain_index][1] = 0x%x, temp_gain = 0x%x, reg_gain = %d\n",
	GC5035_AGC_Param[gain_index][1], temp_gain, reg_gain);
	return reg_gain;
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
	pr_debug("streaming_enable(0=Sw Standby,1=streaming): %d\n",
		enable);
	if (enable) {
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0x3e, 0x91);
	} else {
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0x3e, 0x01);
	}
	return ERROR_NONE;
}

static kal_uint16 gc5035_init_setting[] = {
	/* SYSTEM */
	0xfd, 0x00,
	0xfc, 0x01,
	0xf4, 0x40,
	0xf5, 0xe9,
	0xf6, 0x14,
	0xf8, 0x49,
	0xf9, 0x82,
	0xfa, 0x00,
	0xfc, 0x81,
	0xfe, 0x00,
	0x36, 0x01,
	0xd3, 0x87,
	0x36, 0x00,
	0x33, 0x00,
	0xfe, 0x03,
	0x01, 0xe7,
	0xf7, 0x01,
	0xfc, 0x8f,
	0xfc, 0x8f,
	0xfc, 0x8e,
	0xfe, 0x00,
	0xee, 0x30,
	0x87, 0x18,
	0xfe, 0x01,
	0x8c, 0x90,
	0xfe, 0x00,
	/* Analog & CISCTL */
	0xfe, 0x00,
	0x05, 0x02,
	0x06, 0xda,
	0x9d, 0x0c,
	0x09, 0x00,
	0x0a, 0x04,
	0x0b, 0x00,
	0x0c, 0x03,
	0x0d, 0x07,
	0x0e, 0xa8,
	0x0f, 0x0a,
	0x10, 0x30,
	0x11, 0x02,
	0x17, GC5035_MIRROR,
	0x19, 0x05,
	0xfe, 0x02,
	0x30, 0x03,
	0x31, 0x03,
	0xfe, 0x00,
	0xd9, 0xc0,
	0x1b, 0x20,
	0x21, 0x48,
	0x28, 0x22,
	0x29, 0x58,
	0x44, 0x20,
	0x4b, 0x10,
	0x4e, 0x1a,
	0x50, 0x11,
	0x52, 0x33,
	0x53, 0x44,
	0x55, 0x10,
	0x5b, 0x11,
	0xc5, 0x02,
	0x8c, 0x1a,
	0xfe, 0x02,
	0x33, 0x05,
	0x32, 0x38,
	0xfe, 0x00,
	0x91, 0x80,
	0x92, 0x28,
	0x93, 0x20,
	0x95, 0xa0,
	0x96, 0xe0,
	0xd5, 0xfc,
	0x97, 0x28,
	0x16, 0x0c,
	0x1a, 0x1a,
	0x1f, 0x11,
	0x20, 0x10,
	0x46, 0x83,
	0x4a, 0x04,
	0x54, GC5035_RSTDUMMY1,
	0x62, 0x00,
	0x72, 0x8f,
	0x73, 0x89,
	0x7a, 0x05,
	0x7d, 0xcc,
	0x90, 0x00,
	0xce, 0x18,
	0xd0, 0xb2,
	0xd2, 0x40,
	0xe6, 0xe0,
	0xfe, 0x02,
	0x12, 0x01,
	0x13, 0x01,
	0x14, 0x01,
	0x15, 0x02,
	0x22, GC5035_RSTDUMMY2,
	0x91, 0x00,
	0x92, 0x00,
	0x93, 0x00,
	0x94, 0x00,
	0xfe, 0x00,
	0xfc, 0x88,
	0xfe, 0x10,
	0xfe, 0x00,
	0xfc, 0x8e,
	0xfe, 0x00,
	0xfe, 0x00,
	0xfe, 0x00,
	0xfc, 0x88,
	0xfe, 0x10,
	0xfe, 0x00,
	0xfc, 0x8e,
	/* Gain */
	0xfe, 0x00,
	0xb0, 0x6e,
	0xb1, 0x01,
	0xb2, 0x00,
	0xb3, 0x00,
	0xb4, 0x00,
	0xb6, 0x00,
	/* ISP */
	0xfe, 0x01,
	0x53, 0x00,
	0x89, 0x03,
	0x60, 0x40,
	/* BLK */
	0xfe, 0x01,
	0x42, 0x21,
	0x49, 0x03,
	0x4a, 0xff,
	0x4b, 0xc0,
	0x55, 0x00,
	/* Anti_blooming */
	0xfe, 0x01,
	0x41, 0x28,
	0x4c, 0x00,
	0x4d, 0x00,
	0x4e, 0x3c,
	0x44, 0x08,
	0x48, 0x02,
	/* Crop */
	0xfe, 0x01,
	0x91, 0x00,
	0x92, 0x08,
	0x93, 0x00,
	0x94, 0x07,
	0x95, 0x07,
	0x96, 0x98,
	0x97, 0x0a,
	0x98, 0x20,
	0x99, 0x00,
	/* MIPI */
	0xfe, 0x03,
	0x02, 0x57,
	0x03, 0xb7,
	0x15, 0x14,
	0x18, 0x0f,
	0x21, 0x22,
	0x22, 0x06,
	0x23, 0x48,
	0x24, 0x12,
	0x25, 0x28,
	0x26, 0x08,
	0x29, 0x06,
	0x2a, 0x58,
	0x2b, 0x08,
	0xfe, 0x01,
	0x8c, 0x10,
};


static kal_uint16 gc5035_preview_setting[] = {
	/* SYSTEM */
	0xfd, 0x00,
	0xfe, 0x00,
	0x3e, 0x01,
	0xfc, 0x01,
	0xf4, 0x40,
	0xf5, 0xe4,
	0xf6, 0x14,
	0xf8, 0x49,
	0xf9, 0x12,
	0xfa, 0x01,
	0xfc, 0x81,
	0xfe, 0x00,
	0x36, 0x01,
	0xd3, 0x87,
	0x36, 0x00,
	0x33, 0x20,
	0xfe, 0x03,
	0x01, 0x87,
	0xf7, 0x11,
	0xfc, 0x8f,
	0xfc, 0x8f,
	0xfc, 0x8e,
	0xfe, 0x00,
	0xee, 0x30,
	0x87, 0x18,
	0xfe, 0x01,
	0x8c, 0x90,
	0xfe, 0x00,

	/* Analog & CISCTL */
	0xfe, 0x00,
	0x05, 0x02,
	0x06, 0xda,
	0x9d, 0x0c,
	0x09, 0x00,
	0x0a, 0x04,
	0x0b, 0x00,
	0x0c, 0x03,
	0x0d, 0x07,
	0x0e, 0xa8,
	0x0f, 0x0a,
	0x10, 0x30,
	0x21, 0x60,
	0x29, 0x30,
	0x44, 0x18,
	0x4e, 0x20,
	0x8c, 0x20,
	0x91, 0x15,
	0x92, 0x3a,
	0x93, 0x20,
	0x95, 0x45,
	0x96, 0x35,
	0xd5, 0xf0,
	0x97, 0x20,
	0x1f, 0x19,
	0xce, 0x18,
	0xd0, 0xb3,
	0xfe, 0x02,
	0x14, 0x02,
	0x15, 0x00,
	0xfe, 0x00,
	0xfc, 0x88,
	0xfe, 0x10,
	0xfe, 0x00,
	0xfc, 0x8e,
	0xfe, 0x00,
	0xfe, 0x00,
	0xfe, 0x00,
	0xfc, 0x88,
	0xfe, 0x10,
	0xfe, 0x00,
	0xfc, 0x8e,

	/* BLK */
	0xfe, 0x01,
	0x49, 0x00,
	0x4a, 0x01,
	0x4b, 0xf8,

	/* Anti_blooming */
	0xfe, 0x01,
	0x4e, 0x06,
	0x44, 0x02,

	/* Crop */
	0xfe, 0x01,
	0x91, 0x00,
	0x92, 0x04,
	0x93, 0x00,
	0x94, 0x03,
	0x95, 0x03,
	0x96, 0xcc,
	0x97, 0x05,
	0x98, 0x10,
	0x99, 0x00,

	/* MIPI */
	0xfe, 0x03,
	0x02, 0x58,
	0x22, 0x03,
	0x26, 0x06,
	0x29, 0x03,
	0x2b, 0x06,
	0xfe, 0x01,
	0x8c, 0x10,
};

static kal_uint16 gc5035_capture_setting[] = {
	/*SYSTEM*/
	0xfd, 0x00,
	0xfe, 0x00,
	0x3e, 0x01,
	0xfc, 0x01,
	0xf4, 0x40,
	0xf5, 0xe9,
	0xf6, 0x14,
	0xf8, 0x49,
	0xf9, 0x82,
	0xfa, 0x00,
	0xfc, 0x81,
	0xfe, 0x00,
	0x36, 0x01,
	0xd3, 0x87,
	0x36, 0x00,
	0x33, 0x00,
	0xfe, 0x03,
	0x01, 0xe7,
	0xf7, 0x01,
	0xfc, 0x8f,
	0xfc, 0x8f,
	0xfc, 0x8e,
	0xfe, 0x00,
	0xee, 0x30,
	0x87, 0x18,
	0xfe, 0x01,
	0x8c, 0x90,
	0xfe, 0x00,

	/* Analog & CISCTL */
	0xfe, 0x00,
	0x05, 0x02,
	0x06, 0xda,
	0x9d, 0x0c,
	0x09, 0x00,
	0x0a, 0x04,
	0x0b, 0x00,
	0x0c, 0x03,
	0x0d, 0x07,
	0x0e, 0xa8,
	0x0f, 0x0a,
	0x10, 0x30,
	0x21, 0x48,
	0x29, 0x58,
	0x44, 0x20,
	0x4e, 0x1a,
	0x8c, 0x1a,
	0x91, 0x80,
	0x92, 0x28,
	0x93, 0x20,
	0x95, 0xa0,
	0x96, 0xe0,
	0xd5, 0xfc,
	0x97, 0x28,
	0x1f, 0x11,
	0xce, 0x18,
	0xd0, 0xb2,
	0xfe, 0x02,
	0x14, 0x01,
	0x15, 0x02,
	0xfe, 0x00,
	0xfc, 0x88,
	0xfe, 0x10,
	0xfe, 0x00,
	0xfc, 0x8e,
	0xfe, 0x00,
	0xfe, 0x00,
	0xfe, 0x00,
	0xfc, 0x88,
	0xfe, 0x10,
	0xfe, 0x00,
	0xfc, 0x8e,

	/*BLK*/
	0xfe, 0x01,
	0x49, 0x03,
	0x4a, 0xff,
	0x4b, 0xc0,

	/* Anti_blooming */
	0xfe, 0x01,
	0x4e, 0x3c,
	0x44, 0x08,

	/* Crop */
	0xfe, 0x01,
	0x91, 0x00,
	0x92, 0x08,
	0x93, 0x00,
	0x94, 0x07,
	0x95, 0x07,
	0x96, 0x98,
	0x97, 0x0a,
	0x98, 0x20,
	0x99, 0x00,

	/*MIPI*/
	0xfe, 0x03,
	0x02, 0x57,
	0x22, 0x06,
	0x26, 0x08,
	0x29, 0x06,
	0x2b, 0x08,
	0xfe, 0x01,
	0x8c, 0x10,
};


static kal_uint16 gc5035_slim_video_setting[] = {
	/* SYSTEM */
	0xfd, 0x00,
	0xfe, 0x00,
	0x3e, 0x01,
	0xfc, 0x01,
	0xf4, 0x40,
	0xf5, 0xe4,
	0xf6, 0x14,
	0xf8, 0x49,
	0xf9, 0x12,
	0xfa, 0x01,
	0xfc, 0x81,
	0xfe, 0x00,
	0x36, 0x01,
	0xd3, 0x87,
	0x36, 0x00,
	0x33, 0x20,
	0xfe, 0x03,
	0x01, 0x87,
	0xf7, 0x11,
	0xfc, 0x8f,
	0xfc, 0x8f,
	0xfc, 0x8e,
	0xfe, 0x00,
	0xee, 0x30,
	0x87, 0x18,
	0xfe, 0x01,
	0x8c, 0x90,
	0xfe, 0x00,

	/* Analog & CISCTL */
	0xfe, 0x00,
	0x05, 0x02,
	0x06, 0xda,
	0x9d, 0x0c,
	0x09, 0x00,
	0x0a, 0x04,
	0x0b, 0x00,
	0x0c, 0x03,
	0x0d, 0x07,
	0x0e, 0xa8,
	0x0f, 0x0a,
	0x10, 0x30,
	0x21, 0x60,
	0x29, 0x30,
	0x44, 0x18,
	0x4e, 0x20,
	0x8c, 0x20,
	0x91, 0x15,
	0x92, 0x3a,
	0x93, 0x20,
	0x95, 0x45,
	0x96, 0x35,
	0xd5, 0xf0,
	0x97, 0x20,
	0x1f, 0x19,
	0xce, 0x18,
	0xd0, 0xb3,
	0xfe, 0x02,
	0x14, 0x02,
	0x15, 0x00,
	0xfe, 0x00,
	0xfc, 0x88,
	0xfe, 0x10,
	0xfe, 0x00,
	0xfc, 0x8e,
	0xfe, 0x00,
	0xfe, 0x00,
	0xfe, 0x00,
	0xfc, 0x88,
	0xfe, 0x10,
	0xfe, 0x00,
	0xfc, 0x8e,

	/* BLK */
	0xfe, 0x01,
	0x49, 0x00,
	0x4a, 0x01,
	0x4b, 0xf8,

	/* Anti_blooming */
	0xfe, 0x01,
	0x4e, 0x06,
	0x44, 0x02,

	/* Crop */
	0xfe, 0x01,
	0x91, 0x00,
	0x92, 0x82,
	0x93, 0x00,
	0x94, 0x0b,
	0x95, 0x02,
	0x96, 0xd0,
	0x97, 0x05,
	0x98, 0x00,
	0x99, 0x00,

	/* MIPI */
	0xfe, 0x03,
	0x02, 0x58,
	0x22, 0x03,
	0x26, 0x06,
	0x29, 0x03,
	0x2b, 0x06,
	0xfe, 0x01,
	0x8c, 0x10,
};


static void sensor_init(void)
{
	pr_debug("[%s] init_start\n", __func__);
	gc5035_table_write_cmos_sensor(gc5035_init_setting,
		sizeof(gc5035_init_setting)/sizeof(kal_uint16));
	pr_debug("[%s] End\n", __func__);
}	/*	  sensor_init  */
/*preview size 30fps*/
static void preview_setting(void)
{
	pr_debug("%s +\n", __func__);

	gc5035_table_write_cmos_sensor(gc5035_preview_setting,
		sizeof(gc5035_preview_setting)/sizeof(kal_uint16));

	pr_debug("%s preview_30 fpsX\n", __func__);
}

/*full size 30fps*/
static void capture_setting(kal_uint16 currefps)
{
	pr_debug("%s 30 fps E! currefps:%d\n", __func__, currefps);
	/*************MIPI output setting************/
	gc5035_table_write_cmos_sensor(gc5035_capture_setting,
		sizeof(gc5035_capture_setting)/sizeof(kal_uint16));

	pr_debug("%s capture_30 fpsX\n", __func__);
}

static void normal_video_setting(kal_uint16 currefps)
{
	pr_debug("%s E! currefps:%d\n", __func__, currefps);
	gc5035_table_write_cmos_sensor(gc5035_capture_setting,
		sizeof(gc5035_capture_setting)/sizeof(kal_uint16));
	pr_debug("X\n");
}


/*hs_video */
static void hs_video_setting(void)
{
	pr_debug("%s E! 1296x972@30fps\n", __func__);

	gc5035_table_write_cmos_sensor(gc5035_preview_setting,
		sizeof(gc5035_preview_setting)/sizeof(kal_uint16));

	pr_debug("%s 30 fpsX\n", __func__);
}

static void slim_video_setting(void)
{
	pr_debug("%s E! 1280x720@30fps\n", __func__);
	gc5035_table_write_cmos_sensor(gc5035_slim_video_setting,
		sizeof(gc5035_slim_video_setting)/sizeof(kal_uint16));
	pr_debug("X\n");
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	pr_debug("enable: %d\n", enable);

	if (enable) {
		write_cmos_sensor(0xfe, 0x01);/*100% Color bar*/
		write_cmos_sensor(0x8c, 0x11);
		write_cmos_sensor(0xfe, 0x00);
	} else {
		write_cmos_sensor(0xfe, 0x01);/*No pattern*/
		write_cmos_sensor(0x8c, 0x10);
		write_cmos_sensor(0xfe, 0x00);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

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
	/*sensor have two i2c address 0x6e & 0x20,
	 *we should detect the module used i2c address
	 */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = (read_cmos_sensor(0xf0) << 8) | read_cmos_sensor(0xf1);
			if (*sensor_id == imgsensor_info.sensor_id) {
				gc5035_otp_identify();
				printk("[gc5035_camera_sensor]get_imgsensor_id:i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, *sensor_id);
				return ERROR_NONE;
			}
			printk("[gc5035_camera_sensor]get_imgsensor_id:Read sensor id fail, write id: 0x%x, id: 0x%x\n",
				imgsensor.i2c_write_id, *sensor_id);
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

	pr_debug("%s +\n", __func__);
	/*sensor have two i2c address 0x62 0x63 & 0x20 0x21,
	 *we should detect the module used i2c address
	 */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = (read_cmos_sensor(0xf0) << 8) | read_cmos_sensor(0xf1);
			if (sensor_id == imgsensor_info.sensor_id) {
				pr_debug("[gc5035_camera_sensor]open:i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			pr_debug("[gc5035_camera_sensor]open:Read sensor id fail, write id: 0x%x, id: 0x%x\n",
				imgsensor.i2c_write_id, sensor_id);
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
	gc5035_otp_function();
	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x258;
	imgsensor.gain = 0x40;
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
	pr_debug("%s -\n", __func__);

	return ERROR_NONE;
}

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
	pr_debug("E\n");
	/* No Need to implement this function */
	streaming_control(KAL_FALSE);
	return ERROR_NONE;
}


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
	pr_debug("%s E\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	preview_setting();
	set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}

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
	pr_debug("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		pr_debug(
			"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.cap.max_framerate / 10);
	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;

	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/*imgsensor.video_mode = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("%s. 720P@30FPS\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}

static kal_uint32
get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	pr_debug("E\n");
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


	return ERROR_NONE;
}

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("scenario_id = %d\n", scenario_id);

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


	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->TEMPERATURE_SUPPORT = imgsensor_info.temperature_support;
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

	sensor_info->FrameTimeDelayFrame =
		imgsensor_info.frame_time_delay_frame;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

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
	}

	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		slim_video(image_window, sensor_config_data);
		break;
	default:
		pr_debug("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}

	return ERROR_NONE;
}



static kal_uint32 set_video_mode(UINT16 framerate)
{
	pr_debug("framerate = %d\n ", framerate);
	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	spin_lock(&imgsensor_drv_lock);
	if (enable) /*enable auto flicker*/ {
		imgsensor.autoflicker_en = KAL_TRUE;
		pr_debug("enable! fps = %d", framerate);
	} else {
		 /*Cancel Auto flick*/
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(
		enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	pr_debug("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

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
	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		pr_debug(
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
		pr_debug("error scenario_id = %d, we use preview scenario\n",
			scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(
		enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	pr_debug("scenario_id = %d\n", scenario_id);

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
	default:
		break;
	}

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

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;

	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data
		= (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	/* pr_debug("feature_id = %d\n", feature_id); */
	switch (feature_id) {
	case SENSOR_FEATURE_GET_AWB_REQ_BY_SCENARIO:
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
		break;
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
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.pclk;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.pclk;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.slim_video.pclk;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.pclk;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.cap.framelength << 16)
				+ imgsensor_info.cap.linelength;
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
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
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
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor(sensor_reg_data->RegAddr,
				    sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData =
			read_cmos_sensor(sensor_reg_data->RegAddr);
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
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		/* for factory mode auto testing */
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		pr_debug("current fps :%d\n", (UINT32)*feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		pr_debug("ihdr enable :%d\n", (BOOL)*feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_mode = *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
	#if 0
		pr_debug("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
			(UINT32)*feature_data);
	#endif
		wininfo =
	(struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[1],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
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

		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[0],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		pr_debug("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16)*feature_data,
			(UINT16)*(feature_data+1),
			(UINT16)*(feature_data+2));
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16) (*feature_data),
					(UINT16) (*(feature_data + 1)));
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
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		pr_debug("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data+1));
		#if 0
		ihdr_write_shutter((UINT16)*feature_data,
				   (UINT16)*(feature_data+1));
		#endif
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		pr_debug("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		pr_debug("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
			*feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_BINNING_TYPE:
		*feature_return_para_32 = 2; /* BINNING_AVERAGED */
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
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
	}
		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
	case SENSOR_FEATURE_SET_LSC_TBL:
	default:
		break;
	}

	return ERROR_NONE;
}

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};
UINT32 GC5035_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}
