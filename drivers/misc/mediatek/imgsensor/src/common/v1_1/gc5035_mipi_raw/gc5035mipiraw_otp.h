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
 *     GC5035mipi_otp.h
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     CMOS sensor header file
 *
 ****************************************************************************/
#ifndef _GC5035MIPI_OTP_H
#define _GC5035MIPI_OTP_H

/* SENSOR PRIVATE INFO FOR OTP SETTINGS */
#define GC5035_OTP_FOR_CUSTOMER            0
#define GC5035_OTP_DEBUG                   0

/* DEBUG */
#if GC5035_OTP_DEBUG
#define GC5035_OTP_START_ADDR              0x0000
#endif

#define GC5035_OTP_DATA_LENGTH             1024

/* OTP FLAG TYPE */
#define GC5035_OTP_FLAG_EMPTY              0x00
#define GC5035_OTP_FLAG_VALID              0x01
#define GC5035_OTP_FLAG_INVALID            0x02
#define GC5035_OTP_FLAG_INVALID2           0x03
#define GC5035_OTP_GET_OFFSET(size)           (size << 3)
#define GC5035_OTP_GET_2BIT_FLAG(flag, bit)   ((flag >> bit) & 0x03)
#define GC5035_OTP_CHECK_1BIT_FLAG(flag, bit) (((flag >> bit) & 0x01) == GC5035_OTP_FLAG_VALID)

#define GC5035_OTP_ID_SIZE                 9
#define GC5035_OTP_ID_DATA_OFFSET          0x0020

/* OTP DPC PARAMETERS */
#define GC5035_OTP_DPC_FLAG_OFFSET         0x0068
#define GC5035_OTP_DPC_TOTAL_NUMBER_OFFSET 0x0070
#define GC5035_OTP_DPC_ERROR_NUMBER_OFFSET 0x0078

/* OTP REGISTER UPDATE PARAMETERS */
#define GC5035_OTP_REG_FLAG_OFFSET         0x0880
#define GC5035_OTP_REG_DATA_OFFSET         0x0888
#define GC5035_OTP_REG_MAX_GROUP           5
#define GC5035_OTP_REG_BYTE_PER_GROUP      5
#define GC5035_OTP_REG_REG_PER_GROUP       2
#define GC5035_OTP_REG_BYTE_PER_REG        2
#define GC5035_OTP_REG_DATA_SIZE           (GC5035_OTP_REG_MAX_GROUP * GC5035_OTP_REG_BYTE_PER_GROUP)
#define GC5035_OTP_REG_REG_SIZE            (GC5035_OTP_REG_MAX_GROUP * GC5035_OTP_REG_REG_PER_GROUP)

#if GC5035_OTP_FOR_CUSTOMER
#define GC5035_OTP_CHECK_SUM_BYTE          1
#define GC5035_OTP_GROUP_CNT               2

/* OTP MODULE INFO PARAMETERS */
#define GC5035_OTP_MODULE_FLAG_OFFSET      0x1f10
#define GC5035_OTP_MODULE_DATA_OFFSET      0x1f18
#define GC5035_OTP_MODULE_DATA_SIZE        6

/* OTP WB PARAMETERS */
#define GC5035_OTP_WB_FLAG_OFFSET          0x1f78
#define GC5035_OTP_WB_DATA_OFFSET          0x1f80
#define GC5035_OTP_WB_DATA_SIZE            4
#define GC5035_OTP_GOLDEN_DATA_OFFSET      0x1fc0
#define GC5035_OTP_GOLDEN_DATA_SIZE        4
#define GC5035_OTP_WB_CAL_BASE             0x0800
#define GC5035_OTP_WB_GAIN_BASE            0x0400

/* WB TYPICAL VALUE 0.5 */
#define GC5035_OTP_WB_RG_TYPICAL           0x0400
#define GC5035_OTP_WB_BG_TYPICAL           0x0400
#endif

enum {
	otp_close = 0,
	otp_open,
};

/* DPC STRUCTURE */
struct gc5035_dpc_t {
	kal_uint8 flag;
	kal_uint16 total_num;
};

struct gc5035_reg_t {
	kal_uint8 page;
	kal_uint8 addr;
	kal_uint8 value;
};

/* REGISTER UPDATE STRUCTURE */
struct gc5035_reg_update_t {
	kal_uint8 flag;
	kal_uint8 cnt;
	struct gc5035_reg_t reg[GC5035_OTP_REG_REG_SIZE];
};

#if GC5035_OTP_FOR_CUSTOMER
/* MODULE INFO STRUCTURE */
struct gc5035_module_info_t {
	kal_uint8 module_id;
	kal_uint8 lens_id;
	kal_uint8 year;
	kal_uint8 month;
	kal_uint8 day;
};

/* WB STRUCTURE */
struct gc5035_wb_t {
	kal_uint8  flag;
	kal_uint16 rg;
	kal_uint16 bg;
};
#endif

/* OTP STRUCTURE */
struct gc5035_otp_t {
	kal_uint8 otp_id[GC5035_OTP_ID_SIZE];
	struct gc5035_dpc_t dpc;
	struct gc5035_reg_update_t regs;
#if GC5035_OTP_FOR_CUSTOMER
	struct gc5035_wb_t wb;
	struct gc5035_wb_t golden;
#endif
};

extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData,
	u8 *a_pRecvData, u16 a_sizeRecvData,
		       u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);
#endif
