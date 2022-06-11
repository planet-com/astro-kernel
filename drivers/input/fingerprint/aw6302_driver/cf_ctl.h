/**
 * User space driver API for chipsailing's fingerprint device.
 * ATTENTION: Do NOT edit this file unless the corresponding driver changed.
 *
 * Copyright (C) 2016 chipsailing Corporation. <http://www.chipsailingcorp.com>
 * Copyright (C) 2016 XXX <mailto:xxx@chipsailingcorp.com>
 *
 * This program is free software; you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by the Free 
 * Software Foundation; either version 2 of the License, or (at your option) 
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General 
 * Public License for more details.
**/

#ifndef __CF_CTRL_API_H__
#define __CF_CTRL_API_H__

/* Device node. */
#define FP_DEV_NAME      "aw6302_spi"
#define FP_CLASS_NAME    "aw6302fpsensor"

/* Max driver version buffer length. */
#define CF_DRV_VERSION_LEN 32

/***********************************************/
//chose  REE OR TEE
/***********************************************/
#define REE
//#define TEE
#define KERNEL49
//#define FP_POWER
/***********************************************/

/***********************************************/
//chose platform
/***********************************************/
#define MTK_PLATFORM
//if is MTK,chose MTK platform,if is MTK6739 open 
#if defined(MTK_PLATFORM)
#define MTK6739
//#define MTK6580
#endif
//#define QCOM_PLATFORM
//#define SPEAD_PLATFORM
//#define RK_PLATFORM
/***********************************************/


/***********************************************/
//if is tee ,chose tee platform
/***********************************************/
#if defined(TEE)
#define IS_ISEE
//#define IS_TRUSTKERNEL 
//#define IS_RSEE 
//#define IS_QSEE 
//#define IS_SPEAD 
//#define IS_TRUSTONIC 
//#define IS_TRUSTY 
//#define IS_WATCHTRUST
#endif
/***********************************************/

/***********************************************/
//if platform is there ,to  pen HAL_COMPATIBLE to be compatible
/***********************************************/
#if (defined(IS_RSEE) || defined(IS_QSEE) || defined(IS_TRUSTY) || defined(IS_WATCHTRUST) || defined(IS_SPEAD))
//#define HAL_COMPATIBLE
#endif
/***********************************************/

#if defined(MTK_PLATFORM)
#if 0//defined(MTK6739)
	#include <mtk_spi.h>
#else
	#include <mt_spi.h>
#endif
#endif

typedef enum {
    CF_KEY_NONE = 0,
    CF_KEY_HOME,
    CF_KEY_MENU,
    CF_KEY_BACK,
    CF_KEY_DOWNUP,//press_down or lift_up
    CF_KEY_ONETAP,//single tap
    CF_KEY_DOUBLETAP,//double tap
    CF_KEY_LONGTOUCH,//long press
    CF_KEY_ENTER,
    CF_KEY_UP,
    CF_KEY_LEFT,
    CF_KEY_RIGHT,
    CF_KEY_DOWN,
    CF_KEY_WAKEUP,
} aw_key_type_t;

typedef struct {
    aw_key_type_t key;
    int value; /* for key type, 0 means up, 1 means down. */
} aw_key_event_t;

struct param {
	unsigned char cmd;
	unsigned short addr;
	unsigned short data;	
};

struct config {
	struct param *p_param;
	int num;
};

struct aw_ioc_transfer {
	unsigned char cmd;                      
	unsigned short addr;             
	unsigned char *buf;
	unsigned short actual_len;
};

/*SPI protocol*/
#define CHIPS_W_SRAM 0xAA
#define CHIPS_R_SRAM 0xBB
#define CHIPS_W_SFR  0xCC
#define CHIPS_R_SFR  0xDD


/* Magic code for IOCTL-subsystem, 'k' means 'chipsailing'. */
#define CF_IOC_MAGIC 'k'

/* Androind system-wide key event, for navigation purpose. */
#define CF_IOC_REPORT_KEY_EVENT _IOW(CF_IOC_MAGIC, 7, aw_key_event_t *)

/* HW reset the fingerprint module. */
#define CF_IOC_RESET_DEVICE     _IOW(CF_IOC_MAGIC, 8, unsigned char)

/*HW sensor spi read/write*/
#define CF_IOC_SPI_MESSAGE	         _IOWR(CF_IOC_MAGIC, 9, struct aw_ioc_transfer)
#define CF_IOC_SPI_CMD               _IOW(CF_IOC_MAGIC,10,unsigned char)

/* Low-level IRQ control. */
#define CF_IOC_ENABLE_IRQ       _IO(CF_IOC_MAGIC, 11)
#define CF_IOC_DISABLE_IRQ      _IO(CF_IOC_MAGIC, 12)
#define CF_IOC_SENSOR_CONFIG    _IOW(CF_IOC_MAGIC,13,void*)

/* Allocate/Release driver resource (GPIO/SPI etc.). */
#define CF_IOC_INIT_GPIO        _IO(CF_IOC_MAGIC, 0x13)
#define CF_IOC_REQ_IRQ          _IO(CF_IOC_MAGIC, 0x14)
#define CF_IOC_DEINIT_GPIO      _IO(CF_IOC_MAGIC, 0x15)

/* Set spi rate. */
#define CF_IOC_SPI_SETUP     _IOW(CF_IOC_MAGIC, 0x22, unsigned int)
 

/* Sync 'aw_driver_config_t', the driver configuration. */
#define CF_IOC_SYNC_CONFIG      _IOWR(CF_IOC_MAGIC, 0x0a, void *)

/* Query the driver version string. */
#define CF_IOC_GET_VERSION      _IOR(CF_IOC_MAGIC, 0x20, const char *)

/* SPI bus clock control, for power-saving purpose. */
#define CF_IOC_ENABLE_SPI_CLK   _IO(CF_IOC_MAGIC, 0x30)
#define CF_IOC_DISABLE_SPI_CLK  _IO(CF_IOC_MAGIC, 0x40)

/* Fingerprint module power control. */
#define CF_IOC_ENABLE_POWER     _IO(CF_IOC_MAGIC, 0x50)
#define CF_IOC_DISABLE_POWER    _IO(CF_IOC_MAGIC, 0x60)

#define CF_IOC_FP_HAL_COMPAT    _IOR(CF_IOC_MAGIC, 0x70, int *)

#if defined(MTK_PLATFORM)
#if 0
extern void aw_spi_setup(struct spi_device *spi, u32 speed, enum spi_transfer_mode mode);
#endif
extern int aw_sfr_read(struct spi_device *spi, unsigned short addr, unsigned char *recv_buf, unsigned short buflen);

extern int aw_sfr_write(struct spi_device *spi, unsigned short addr, unsigned char *send_buf, unsigned short buflen);

extern int aw_sram_read(struct spi_device *spi, unsigned short addr, unsigned char *recv_buf, unsigned short buflen);

extern int aw_sram_write(struct spi_device *spi, unsigned short addr, unsigned char *send_buf, unsigned short buflen);

extern int aw_spi_cmd(struct spi_device *spi, unsigned char *cmd, unsigned short cmdlen);

#endif

#endif /* __CF_CTRL_API_H__ */
