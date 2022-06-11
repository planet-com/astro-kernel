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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <typec.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>

#include <linux/gpio.h>


#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/gameport.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
//#include <linux/wakelock.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>

#include <linux/proc_fs.h>
#include<linux/slab.h>
#include <linux/uaccess.h>

#include "fusb302.h"

#ifndef USB_TYPE_C
#define USB_TYPE_C

#define K_EMERG	(1<<7)
#define K_QMU	(1<<7)
#define K_ALET		(1<<6)
#define K_CRIT		(1<<5)
#define K_ERR		(1<<4)
#define K_WARNIN	(1<<3)
#define K_NOTICE	(1<<2)
#define K_INFO		(1<<1)
#define K_DEBUG	(1<<0)

#define fusb_printk(level, fmt, args...) do { \
			pr_err("[FUSB302]" fmt, ## args); \
	} while (0)

#define SKIP_TIMER

//static u32 debug_level = (255 - K_DEBUG);
static struct usbtypc *g_exttypec;
static struct i2c_client *typec_client;

/* /////////////////////////////////////////////////////////////////////////// */
/* Variables accessible outside of the FUSB300 state machine */
/* /////////////////////////////////////////////////////////////////////////// */
static FUSB300reg_t Registers;	/* Variable holding the current status of the FUSB300 registers */
void InitializeFUSB300(void)
{
	fusb_printk(K_DEBUG, "%s\n", __func__);
	FUSB300Read(regDeviceID, 1, &Registers.DeviceID.byte);	/* Read the device ID */
	fusb_printk(K_DEBUG, "%s DeviceID: 0x%x\n", __func__, Registers.DeviceID.byte);
	
	Registers.Mode.byte = 0x10;
	FUSB300Write(regMode, 1, &Registers.Mode.byte);
	
//	FUSB300Read(regSlice, 1, &Registers.Slice.byte);	/* Read the slice */
//	Registers.Mask.byte = 0x00;	/* Do not mask any interrupts */

	/* Clear all interrupt masks (we want to do something with them) */
//	FUSB300Write(regMask, 1, &Registers.Mask.byte);
}

/* /////////////////////////////////////////////////////////////////////////// */
/* FUSB300 I2C Routines */
/* /////////////////////////////////////////////////////////////////////////// */
/* BOOL FUSB300Write(struct usbtypc *typec, unsigned char regAddr, unsigned char length, unsigned char* data) */
int FUSB300Write(unsigned char regAddr, unsigned char length, unsigned char *data)
{
	int i;

	for (i = 0; i < length; i++)
		fusb300_i2c_w_reg8(typec_client, regAddr + i, data[i]);

	return 1;
}

/* BOOL FUSB300Read(struct usbtypc *typec, unsigned char regAddr, unsigned char length, unsigned char* data) */
int FUSB300Read(unsigned char regAddr, unsigned char length, unsigned char *data)
{
	int i;

	for (i = 0; i < length; i++)
		data[i] = fusb300_i2c_r_reg(typec_client, regAddr + i);

	return 1;
}

void fusb300_i2c_w_reg8(struct i2c_client *client, u8 addr, u8 var)
{
	char buffer[2];

	buffer[0] = addr;
	buffer[1] = var;
	i2c_master_send(client, buffer, 2);
}

u8 fusb300_i2c_r_reg(struct i2c_client *client, u8 addr)
{
	u8 var;

	i2c_master_send(client, &addr, 1);
	i2c_master_recv(client, &var, 1);
	return var;
}

int check_usbl_type(void)
{
	unsigned char CCXstate = 0;
	int ret = 0;
	
	FUSB300Read(regType, 1, &CCXstate);
	fusb_printk(K_INFO, "%s:CCXstate = 0x%x\n", __func__, CCXstate);
	CCXstate &= 0x18;
	if (CCXstate == 0x8){ //SRC
		ret = 1;
	} else if (CCXstate == 0x10){ //SNK
		ret = 2;
	} else{
		fusb_printk(K_INFO, "%s:no usb plugin, maybe wireless charger is working!\n", __func__);
		ret = 0;
	}
	return ret;
}

static int fusb300_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct usbtypc *typec;
//	unsigned char port_type;
	unsigned char CCXstate = 0;

	fusb_printk(K_INFO, "%s 0x%x\n", __func__, client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		fusb_printk(K_ERR, "fusb300 i2c functionality check fail.\n");
		return -ENODEV;
	}

	fusb_printk(K_DEBUG, "%s %s\n", __func__, client->dev.driver->name);

	if (!g_exttypec)
		g_exttypec = kzalloc(sizeof(struct usbtypc), GFP_KERNEL);

	typec = g_exttypec;

	typec_client = client;
	
//	fusb300_init_debugfs(typec);
	
	InitializeFUSB300();

//	usb_redriver_init(typec);

	fusb_printk(K_INFO, "%s %x\n", __func__, fusb300_i2c_r_reg(client, 0x1));
	
	mdelay(600);
	
	FUSB300Read(regStatus, 1, &CCXstate);
	fusb_printk(K_INFO, "%s:CCXstate = 0x%x\n", __func__, CCXstate);
	
	FUSB300Read(regType, 1, &CCXstate);
	fusb_printk(K_INFO, "%s:CCXstate = 0x%x\n", __func__, CCXstate);
	
	/*precheck status */
	/* StateMachineFUSB300(typec); */
	return 0;
}

#define FUSB302_NAME "FUSB301_1"

static const struct i2c_device_id usb_i2c_id[] = {
		{FUSB302_NAME, 0},
		{}
	};

#ifdef CONFIG_OF
static const struct of_device_id fusb301_of_match[] = {
		{.compatible = "mediatek,fusb301a"},
		{},
	};
#endif

struct i2c_driver usb_i2c_driver = {
	.probe = fusb300_i2c_probe,
	.driver = {
		.owner = THIS_MODULE,
		.name = FUSB302_NAME,
#ifdef CONFIG_OF
		.of_match_table = fusb301_of_match,
#endif
	},
	.id_table = usb_i2c_id,
};

static int __init fusb300_init(void)
{
	int ret = 0;
	
	if (i2c_add_driver(&usb_i2c_driver) != 0) {
		fusb_printk(K_ERR, "fusb300_init initialization failed!!\n");
		ret = -1;
	} else {
		fusb_printk(K_DEBUG, "fusb300_init initialization succeed!!\n");
	}
	return ret;
}

static void __exit fusb300_exit(void)
{

}
fs_initcall(fusb300_init);
/* module_exit(fusb300_exit); */

#endif				/*USB_TYPE_C */
