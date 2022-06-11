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
 *     gc5035mipi_otp.c
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
#define PFX "gc5035_otp"
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

#include "gc5035mipiraw_otp.h"


static struct gc5035_otp_t gc5035_otp_data;
kal_uint8 otp_i2c_write_id = 0x6e;

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[1] = {(char)(addr & 0xff)};

	iReadRegI2C(pu_send_cmd, 1, (u8 *)&get_byte, 1, otp_i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[2] = {(char)(addr & 0xff), (char)(para & 0xff)};

	iWriteRegI2C(pu_send_cmd, 2, otp_i2c_write_id);
}



static kal_uint8 gc5035_otp_read_byte(kal_uint16 addr)
{
	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x69, (addr >> 8) & 0x1f);
	write_cmos_sensor(0x6a, addr & 0xff);
	write_cmos_sensor(0xf3, 0x20);

	return read_cmos_sensor(0x6c);
}


static void gc5035_otp_read_group(kal_uint16 addr,
	kal_uint8 *data, kal_uint16 length)
{
	kal_uint16 i = 0;

	if ((((addr & 0x1fff) >> 3) + length) > GC5035_OTP_DATA_LENGTH) {
		pr_debug("out of range, start addr: 0x%.4x, length = %d\n", addr & 0x1fff, length);
		return;
	}

	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x69, (addr >> 8) & 0x1f);
	write_cmos_sensor(0x6a, addr & 0xff);
	write_cmos_sensor(0xf3, 0x20);
	write_cmos_sensor(0xf3, 0x12);

	for (i = 0; i < length; i++)
		data[i] = read_cmos_sensor(0x6c);

	write_cmos_sensor(0xf3, 0x00);
}


static void gc5035_gcore_read_dpc(void)
{
	kal_uint8 dpcFlag = 0;
	struct gc5035_dpc_t *pDPC = &gc5035_otp_data.dpc;

	dpcFlag = gc5035_otp_read_byte(GC5035_OTP_DPC_FLAG_OFFSET);
	pr_debug("dpc flag = 0x%x\n", dpcFlag);
	switch (GC5035_OTP_GET_2BIT_FLAG(dpcFlag, 0)) {
	case GC5035_OTP_FLAG_EMPTY: {
		pr_debug("dpc info is empty!!\n");
		pDPC->flag = GC5035_OTP_FLAG_EMPTY;
		break;
	}
	case GC5035_OTP_FLAG_VALID: {
		pr_debug("dpc info is valid!\n");
		pDPC->total_num = gc5035_otp_read_byte(GC5035_OTP_DPC_TOTAL_NUMBER_OFFSET)
			+ gc5035_otp_read_byte(GC5035_OTP_DPC_ERROR_NUMBER_OFFSET);
		pDPC->flag = GC5035_OTP_FLAG_VALID;
		pr_debug("total_num = %d\n", pDPC->total_num);
		break;
	}
	default:
		pDPC->flag = GC5035_OTP_FLAG_INVALID;
		break;
	}
}

static void gc5035_gcore_read_reg(void)
{
	kal_uint8 i = 0;
	kal_uint8 j = 0;
	kal_uint16 base_group = 0;
	kal_uint8 reg[GC5035_OTP_REG_DATA_SIZE];
	struct gc5035_reg_update_t *pRegs = &gc5035_otp_data.regs;

	memset(&reg, 0, GC5035_OTP_REG_DATA_SIZE);
	pRegs->flag = gc5035_otp_read_byte(GC5035_OTP_REG_FLAG_OFFSET);
	pr_debug("register update flag = 0x%x\n", pRegs->flag);
	if (pRegs->flag == GC5035_OTP_FLAG_VALID) {
		gc5035_otp_read_group(GC5035_OTP_REG_DATA_OFFSET, &reg[0], GC5035_OTP_REG_DATA_SIZE);

		for (i = 0; i < GC5035_OTP_REG_MAX_GROUP; i++) {
			base_group = i * GC5035_OTP_REG_BYTE_PER_GROUP;
			for (j = 0; j < GC5035_OTP_REG_REG_PER_GROUP; j++)
				if (GC5035_OTP_CHECK_1BIT_FLAG(reg[base_group], (4 * j + 3))) {
					pRegs->reg[pRegs->cnt].page =
						(reg[base_group] >> (4 * j)) & 0x07;
					pRegs->reg[pRegs->cnt].addr =
						reg[base_group + j * GC5035_OTP_REG_BYTE_PER_REG + 1];
					pRegs->reg[pRegs->cnt].value =
						reg[base_group + j * GC5035_OTP_REG_BYTE_PER_REG + 2];
					pr_debug("register[%d] P%d:0x%x->0x%x\n",
						pRegs->cnt, pRegs->reg[pRegs->cnt].page,
						pRegs->reg[pRegs->cnt].addr, pRegs->reg[pRegs->cnt].value);
					pRegs->cnt++;
				}
		}

	}
}

#if GC5035_OTP_FOR_CUSTOMER
static kal_uint8 gc5035_otp_read_module_info(void)
{
	kal_uint8 i = 0;
	kal_uint8 idx = 0;
	kal_uint8 flag = 0;
	kal_uint16 check = 0;
	kal_uint16 module_start_offset = GC5035_OTP_MODULE_DATA_OFFSET;
	kal_uint8 info[GC5035_OTP_MODULE_DATA_SIZE];
	struct gc5035_module_info_t module_info = { 0 };

	memset(&info, 0, GC5035_OTP_MODULE_DATA_SIZE);
	memset(&module_info, 0, sizeof(struct gc5035_module_info_t));

	flag = gc5035_otp_read_byte(GC5035_OTP_MODULE_FLAG_OFFSET);
	pr_debug("flag = 0x%x\n", flag);

	for (idx = 0; idx < GC5035_OTP_GROUP_CNT; idx++) {
		switch (GC5035_OTP_GET_2BIT_FLAG(flag, 2 * (1 - idx))) {
		case GC5035_OTP_FLAG_EMPTY: {
			pr_debug("group %d is empty!\n", idx + 1);
			break;
		}
		case GC5035_OTP_FLAG_VALID: {
			pr_debug("group %d is valid!\n", idx + 1);
			module_start_offset = GC5035_OTP_MODULE_DATA_OFFSET
				+ GC5035_OTP_GET_OFFSET(idx * GC5035_OTP_MODULE_DATA_SIZE);
			gc5035_otp_read_group(module_start_offset, &info[0], GC5035_OTP_MODULE_DATA_SIZE);
			for (i = 0; i < GC5035_OTP_MODULE_DATA_SIZE - 1; i++)
				check += info[i];

			if ((check % 255 + 1) == info[GC5035_OTP_MODULE_DATA_SIZE - 1]) {
				module_info.module_id = info[0];
				module_info.lens_id = info[1];
				module_info.year = info[2];
				module_info.month = info[3];
				module_info.day = info[4];

				pr_debug("module_id = 0x%x\n", module_info.module_id);
				pr_debug("lens_id = 0x%x\n", module_info.lens_id);
				pr_debug("data = %d-%d-%d\n", module_info.year, module_info.month, module_info.day);
			} else
				pr_debug("check sum %d error! check sum = 0x%x, calculate result = 0x%x\n",
					idx + 1, info[GC5035_OTP_MODULE_DATA_SIZE - 1], (check % 255 + 1));
			break;
		}
		case GC5035_OTP_FLAG_INVALID:
		case GC5035_OTP_FLAG_INVALID2: {
			pr_debug("group %d is invalid!\n", idx + 1);
			break;
		}
		default:
			break;
		}
	}

	return module_info.module_id;
}

static void gc5035_otp_read_wb_info(void)
{
	kal_uint8 i = 0;
	kal_uint8 idx = 0;
	kal_uint8 flag = 0;
	kal_uint16 wb_check = 0;
	kal_uint16 golden_check = 0;
	kal_uint16 wb_start_offset = GC5035_OTP_WB_DATA_OFFSET;
	kal_uint16 golden_start_offset = GC5035_OTP_GOLDEN_DATA_OFFSET;
	kal_uint8 wb[GC5035_OTP_WB_DATA_SIZE];
	kal_uint8 golden[GC5035_OTP_GOLDEN_DATA_SIZE];
	struct gc5035_wb_t *pWB = &gc5035_otp_data.wb;
	struct gc5035_wb_t *pGolden = &gc5035_otp_data.golden;

	memset(&wb, 0, GC5035_OTP_WB_DATA_SIZE);
	memset(&golden, 0, GC5035_OTP_GOLDEN_DATA_SIZE);
	flag = gc5035_otp_read_byte(GC5035_OTP_WB_FLAG_OFFSET);
	pr_debug("flag = 0x%x\n", flag);

	for (idx = 0; idx < GC5035_OTP_GROUP_CNT; idx++) {
		switch (GC5035_OTP_GET_2BIT_FLAG(flag, 2 * (1 - idx))) {
		case GC5035_OTP_FLAG_EMPTY: {
			pr_debug("wb group %d is empty!\n", idx + 1);
			pWB->flag = pWB->flag | GC5035_OTP_FLAG_EMPTY;
			break;
		}
		case GC5035_OTP_FLAG_VALID: {
			pr_debug("wb group %d is valid!\n", idx + 1);
			wb_start_offset = GC5035_OTP_WB_DATA_OFFSET
				+ GC5035_OTP_GET_OFFSET(idx * GC5035_OTP_WB_DATA_SIZE);
			gc5035_otp_read_group(wb_start_offset, &wb[0], GC5035_OTP_WB_DATA_SIZE);

			for (i = 0; i < GC5035_OTP_WB_DATA_SIZE - 1; i++)
				wb_check += wb[i];

			if ((wb_check % 255 + 1) == wb[GC5035_OTP_WB_DATA_SIZE - 1]) {
				pWB->rg = (wb[0] | ((wb[1] & 0xf0) << 4));
				pWB->bg = (((wb[1] & 0x0f) << 8) | wb[2]);
				pWB->rg = pWB->rg == 0 ? GC5035_OTP_WB_RG_TYPICAL : pWB->rg;
				pWB->bg = pWB->bg == 0 ? GC5035_OTP_WB_BG_TYPICAL : pWB->bg;
				pWB->flag = pWB->flag | GC5035_OTP_FLAG_VALID;
				pr_debug("wb r/g = 0x%x\n", pWB->rg);
				pr_debug("wb b/g = 0x%x\n", pWB->bg);
			} else {
				pWB->flag = pWB->flag | GC5035_OTP_FLAG_INVALID;
				pr_debug("wb check sum %d error! check sum = 0x%x, calculate result = 0x%x\n",
					idx + 1, wb[GC5035_OTP_WB_DATA_SIZE - 1], (wb_check % 255 + 1));
			}
			break;
		}
		case GC5035_OTP_FLAG_INVALID:
		case GC5035_OTP_FLAG_INVALID2: {
			pr_debug("wb group %d is invalid!\n", idx + 1);
			pWB->flag = pWB->flag | GC5035_OTP_FLAG_INVALID;
			break;
		}
		default:
			break;
		}

		switch (GC5035_OTP_GET_2BIT_FLAG(flag, 2 * (3 - idx))) {
		case GC5035_OTP_FLAG_EMPTY: {
			pr_debug("golden group %d is empty!\n", idx + 1);
			pGolden->flag = pGolden->flag | GC5035_OTP_FLAG_EMPTY;
			break;
		}
		case GC5035_OTP_FLAG_VALID: {
			pr_debug("golden group %d is valid!\n", idx + 1);
			golden_start_offset = GC5035_OTP_GOLDEN_DATA_OFFSET
				+ GC5035_OTP_GET_OFFSET(idx * GC5035_OTP_GOLDEN_DATA_SIZE);
			gc5035_otp_read_group(golden_start_offset, &golden[0], GC5035_OTP_GOLDEN_DATA_SIZE);
			for (i = 0; i < GC5035_OTP_GOLDEN_DATA_SIZE - 1; i++)
				golden_check += golden[i];

			if ((golden_check % 255 + 1) == golden[GC5035_OTP_GOLDEN_DATA_SIZE - 1]) {
				pGolden->rg = (golden[0] | ((golden[1] & 0xf0) << 4));
				pGolden->bg = (((golden[1] & 0x0f) << 8) | golden[2]);
				pGolden->rg = pGolden->rg == 0 ? GC5035_OTP_WB_RG_TYPICAL : pGolden->rg;
				pGolden->bg = pGolden->bg == 0 ? GC5035_OTP_WB_BG_TYPICAL : pGolden->bg;
				pGolden->flag = pGolden->flag | GC5035_OTP_FLAG_VALID;
				pr_debug("golden r/g = 0x%x\n", pGolden->rg);
				pr_debug("golden b/g = 0x%x\n", pGolden->bg);
			} else {
				pGolden->flag = pGolden->flag | GC5035_OTP_FLAG_INVALID;
				pr_debug("golden check sum %d error! check sum = 0x%x, calculate result = 0x%x\n",
					idx + 1, golden[GC5035_OTP_WB_DATA_SIZE - 1], (golden_check % 255 + 1));
			}
			break;
		}
		case GC5035_OTP_FLAG_INVALID:
		case GC5035_OTP_FLAG_INVALID2: {
			pr_debug("golden group %d is invalid!\n", idx + 1);
			pGolden->flag = pGolden->flag | GC5035_OTP_FLAG_INVALID;
			break;
		}
		default:
			break;
		}
	}
}
#endif

static kal_uint8 gc5035_otp_read_sensor_info(void)
{
	kal_uint8 moduleID = 0;
#if GC5035_OTP_DEBUG
	kal_uint16 i = 0;
	kal_uint8 debug[GC5035_OTP_DATA_LENGTH];
#endif

	gc5035_gcore_read_dpc();
	gc5035_gcore_read_reg();
#if GC5035_OTP_FOR_CUSTOMER
	moduleID = gc5035_otp_read_module_info();
	gc5035_otp_read_wb_info();

#endif

#if GC5035_OTP_DEBUG
	memset(&debug[0], 0, GC5035_OTP_DATA_LENGTH);
	gc5035_otp_read_group(GC5035_OTP_START_ADDR, &debug[0], GC5035_OTP_DATA_LENGTH);
	for (i = 0; i < GC5035_OTP_DATA_LENGTH; i++)
		pr_debug("addr = 0x%x, data = 0x%x\n", GC5035_OTP_START_ADDR + i * 8, debug[i]);
#endif

	return moduleID;
}

static void gc5035_otp_update_dd(void)
{
	kal_uint8 state = 0;
	kal_uint8 n = 0;
	struct gc5035_dpc_t *pDPC = &gc5035_otp_data.dpc;

	if (GC5035_OTP_FLAG_VALID == pDPC->flag) {
		pr_debug("DD auto load start!\n");
		write_cmos_sensor(0xfe, 0x02);
		write_cmos_sensor(0xbe, 0x00);
		write_cmos_sensor(0xa9, 0x01);
		write_cmos_sensor(0x09, 0x33);
		write_cmos_sensor(0x01, (pDPC->total_num >> 8) & 0x07);
		write_cmos_sensor(0x02, pDPC->total_num & 0xff);
		write_cmos_sensor(0x03, 0x00);
		write_cmos_sensor(0x04, 0x80);
		write_cmos_sensor(0x95, 0x0a);
		write_cmos_sensor(0x96, 0x30);
		write_cmos_sensor(0x97, 0x0a);
		write_cmos_sensor(0x98, 0x32);
		write_cmos_sensor(0x99, 0x07);
		write_cmos_sensor(0x9a, 0xa9);
		write_cmos_sensor(0xf3, 0x80);
		while (n < 3) {
			state = read_cmos_sensor(0x06);
			if ((state | 0xfe) == 0xff)
				mdelay(10);
			else
				n = 3;
			n++;
		}
		write_cmos_sensor(0xbe, 0x01);
		write_cmos_sensor(0x09, 0x00);
		write_cmos_sensor(0xfe, 0x01);
		write_cmos_sensor(0x80, 0x02);
		write_cmos_sensor(0xfe, 0x00);
	}
}

#if GC5035_OTP_FOR_CUSTOMER
static void gc5035_otp_update_wb(void)
{
	kal_uint16 r_gain = GC5035_OTP_WB_GAIN_BASE;
	kal_uint16 g_gain = GC5035_OTP_WB_GAIN_BASE;
	kal_uint16 b_gain = GC5035_OTP_WB_GAIN_BASE;
	kal_uint16 base_gain = GC5035_OTP_WB_CAL_BASE;
	kal_uint16 r_gain_curr = GC5035_OTP_WB_CAL_BASE;
	kal_uint16 g_gain_curr = GC5035_OTP_WB_CAL_BASE;
	kal_uint16 b_gain_curr = GC5035_OTP_WB_CAL_BASE;
	kal_uint16 rg_typical = GC5035_OTP_WB_RG_TYPICAL;
	kal_uint16 bg_typical = GC5035_OTP_WB_BG_TYPICAL;
	struct gc5035_wb_t *pWB = &gc5035_otp_data.wb;
	struct gc5035_wb_t *pGolden = &gc5035_otp_data.golden;

	if (GC5035_OTP_CHECK_1BIT_FLAG(pGolden->flag, 0)) {
		rg_typical = pGolden->rg;
		bg_typical = pGolden->bg;
	} else {
		rg_typical = GC5035_OTP_WB_RG_TYPICAL;
		bg_typical = GC5035_OTP_WB_BG_TYPICAL;
	}
	pr_debug("typical rg = 0x%x, bg = 0x%x\n", rg_typical, bg_typical);

	if (GC5035_OTP_CHECK_1BIT_FLAG(pWB->flag, 0)) {
		r_gain_curr = GC5035_OTP_WB_CAL_BASE * rg_typical / pWB->rg;
		b_gain_curr = GC5035_OTP_WB_CAL_BASE * bg_typical / pWB->bg;
		g_gain_curr = GC5035_OTP_WB_CAL_BASE;

		base_gain = (r_gain_curr < b_gain_curr) ? r_gain_curr : b_gain_curr;
		base_gain = (base_gain < g_gain_curr) ? base_gain : g_gain_curr;

		r_gain = GC5035_OTP_WB_GAIN_BASE * r_gain_curr / base_gain;
		g_gain = GC5035_OTP_WB_GAIN_BASE * g_gain_curr / base_gain;
		b_gain = GC5035_OTP_WB_GAIN_BASE * b_gain_curr / base_gain;
		pr_debug("channel gain r = 0x%x, g = 0x%x, b = 0x%x\n", r_gain, g_gain, b_gain);

		write_cmos_sensor(0xfe, 0x04);
		write_cmos_sensor(0x40, g_gain & 0xff);
		write_cmos_sensor(0x41, r_gain & 0xff);
		write_cmos_sensor(0x42, b_gain & 0xff);
		write_cmos_sensor(0x43, g_gain & 0xff);
		write_cmos_sensor(0x44, g_gain & 0xff);
		write_cmos_sensor(0x45, r_gain & 0xff);
		write_cmos_sensor(0x46, b_gain & 0xff);
		write_cmos_sensor(0x47, g_gain & 0xff);
		write_cmos_sensor(0x48, (g_gain >> 8) & 0x07);
		write_cmos_sensor(0x49, (r_gain >> 8) & 0x07);
		write_cmos_sensor(0x4a, (b_gain >> 8) & 0x07);
		write_cmos_sensor(0x4b, (g_gain >> 8) & 0x07);
		write_cmos_sensor(0x4c, (g_gain >> 8) & 0x07);
		write_cmos_sensor(0x4d, (r_gain >> 8) & 0x07);
		write_cmos_sensor(0x4e, (b_gain >> 8) & 0x07);
		write_cmos_sensor(0x4f, (g_gain >> 8) & 0x07);
		write_cmos_sensor(0xfe, 0x00);
	}
}
#endif

static void gc5035_otp_update_reg(void)
{
	kal_uint8 i = 0;

	pr_debug("reg count = %d\n", gc5035_otp_data.regs.cnt);

	if (GC5035_OTP_CHECK_1BIT_FLAG(gc5035_otp_data.regs.flag, 0))
		for (i = 0; i < gc5035_otp_data.regs.cnt; i++) {
			write_cmos_sensor(0xfe, gc5035_otp_data.regs.reg[i].page);
			write_cmos_sensor(gc5035_otp_data.regs.reg[i].addr, gc5035_otp_data.regs.reg[i].value);
			pr_debug("reg[%d] P%d:0x%x -> 0x%x\n", i, gc5035_otp_data.regs.reg[i].page,
				gc5035_otp_data.regs.reg[i].addr, gc5035_otp_data.regs.reg[i].value);
		}
}

static void gc5035_otp_update(void)
{
	gc5035_otp_update_dd();
#if GC5035_OTP_FOR_CUSTOMER
	gc5035_otp_update_wb();
#endif
	gc5035_otp_update_reg();
}

kal_uint8 gc5035_otp_identify(void)
{
	kal_uint8 moduleID = 0;

	memset(&gc5035_otp_data, 0, sizeof(gc5035_otp_data));

	write_cmos_sensor(0xfc, 0x01);
	write_cmos_sensor(0xf4, 0x40);
	write_cmos_sensor(0xf5, 0xe9);
	write_cmos_sensor(0xf6, 0x14);
	write_cmos_sensor(0xf8, 0x49);
	write_cmos_sensor(0xf9, 0x82);
	write_cmos_sensor(0xfa, 0x00);
	write_cmos_sensor(0xfc, 0x81);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x36, 0x01);
	write_cmos_sensor(0xd3, 0x87);
	write_cmos_sensor(0x36, 0x00);
	write_cmos_sensor(0x33, 0x00);
	write_cmos_sensor(0xf7, 0x01);
	write_cmos_sensor(0xfc, 0x8e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xee, 0x30);
	write_cmos_sensor(0xfa, 0x10);
	write_cmos_sensor(0xf5, 0xe9);
	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x67, 0xc0);
	write_cmos_sensor(0x59, 0x3f);
	write_cmos_sensor(0x55, 0x84);
	write_cmos_sensor(0x65, 0x80);
	write_cmos_sensor(0x66, 0x03);
	write_cmos_sensor(0xfe, 0x00);

	gc5035_otp_read_group(GC5035_OTP_ID_DATA_OFFSET, &gc5035_otp_data.otp_id[0], GC5035_OTP_ID_SIZE);
	moduleID = gc5035_otp_read_sensor_info();

	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x67, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfa, 0x00);
	return moduleID;
}

void gc5035_otp_function(void)
{
	kal_uint8 i = 0, flag = 0;
	kal_uint8 otp_id[GC5035_OTP_ID_SIZE];

	memset(&otp_id, 0, GC5035_OTP_ID_SIZE);

	write_cmos_sensor(0xfa, 0x10);
	write_cmos_sensor(0xf5, 0xe9);
	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x67, 0xc0);
	write_cmos_sensor(0x59, 0x3f);
	write_cmos_sensor(0x55, 0x84);
	write_cmos_sensor(0x65, 0x80);
	write_cmos_sensor(0x66, 0x03);
	write_cmos_sensor(0xfe, 0x00);

	gc5035_otp_read_group(GC5035_OTP_ID_DATA_OFFSET, &otp_id[0], GC5035_OTP_ID_SIZE);
	for (i = 0; i < GC5035_OTP_ID_SIZE; i++)
		if (otp_id[i] != gc5035_otp_data.otp_id[i]) {
			flag = 1;
			break;
		}

	if (flag == 1) {
		pr_debug("otp id mismatch, read again");
		memset(&gc5035_otp_data, 0, sizeof(gc5035_otp_data));
		for (i = 0; i < GC5035_OTP_ID_SIZE; i++)
			gc5035_otp_data.otp_id[i] = otp_id[i];
		gc5035_otp_read_sensor_info();
	}
	gc5035_otp_update();

	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x67, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfa, 0x00);
}
