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
#ifndef __MT_SPI_HAL_H__
#define __MT_SPI_HAL_H__

#if !defined(CONFIG_MTK_CLKMGR)
#include <linux/clk.h>
#endif				/* !defined(CONFIG_MTK_CLKMGR) */
//#include <linux/wakelock.h>
#include "cf_ctl.h"
#include <linux/pm_wakeup.h>


struct mt_spi_t {
	struct platform_device *pdev;
	void __iomem *regs;
	int irq;
	int running;
#ifdef CONFIG_PM_WAKELOCKS
	struct wakeup_source wk_lock;
#else
	struct wake_lock wk_lock;
#endif
	//#if !defined(KERNEL49)
	//struct wake_lock wk_lock;
	//#endif
	struct mt_chip_conf *config;
	struct spi_master *master;

	struct spi_transfer *cur_transfer;
	struct spi_transfer *next_transfer;

	spinlock_t lock;
	struct list_head queue;
#if !defined(CONFIG_MTK_CLKMGR)
	struct clk *clk_main;	/* main clock for spi bus */
#endif				/* !defined(CONFIG_MTK_CLKMGR) */
};

extern void mt_spi_enable_clk(struct mt_spi_t *ms);
extern void mt_spi_disable_clk(struct mt_spi_t *ms);



#endif

