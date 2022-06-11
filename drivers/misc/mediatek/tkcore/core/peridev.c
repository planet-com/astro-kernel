/*
 * Copyright (c) 2015-2018 TrustKernel Incorporated
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/compiler.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mutex.h>

#include <linux/clk.h>
#include <linux/spi/spi.h>

#include <linux/tee_clkmgr.h>
#include <linux/tee_client_api.h>
#include <linux/tee_kernel_api.h>

#include "tee_core_priv.h"

static struct TEEC_UUID SENSOR_DETECTOR_TA_UUID = { 0x966d3f7c, 0x04ef, 0x1beb,
	{ 0x08, 0xb7, 0x57, 0xf3, 0x7a, 0x6d, 0x87, 0xf9 } };

#define CMD_READ_CHIPID		0x0
#define CMD_DISABLE			0x1
#define CMD_CONFIG_PADSEL	0x2


struct clkmgr_handle {
	uint32_t token;
	void *e, *d;
	const void *p0, *p1, *p2;
	size_t argnum;
	struct list_head le;
};

/* sync with tee-os */
enum tee_clkmgr_type {
	TEE_CLKMGR_TYPE_SPI = 0,
	TEE_CLKMGR_TYPE_I2C,
	TEE_CLKMGR_TYPE_I2C_DMA
};

static const char * const clkid[] = {
	[TEE_CLKMGR_TYPE_SPI] = "spi",
	[TEE_CLKMGR_TYPE_I2C] = "i2c",
	[TEE_CLKMGR_TYPE_I2C_DMA] = "i2c-dma",
};

static LIST_HEAD(clk_list);
static DEFINE_MUTEX(clk_list_lock);

static struct clkmgr_handle *try_spi_clk(uint32_t token)
{
	struct clk *clk;
	struct spi_controller *master;
	struct clkmgr_handle *h;

	uint32_t busnum = TEE_CLKMGR_TOKEN_ID(token);

	h = kmalloc(sizeof(struct clkmgr_handle), GFP_KERNEL);
	if (h == NULL) {
		return NULL;
	}

	memset(h, 0, sizeof(*h));
	h->token = token;

	master = spi_busnum_to_master(busnum);
	if (master == NULL) {
		pr_warn("tkcoredrv: spi%u not found\n",
			busnum);
		return h;
	}

	clk = devm_clk_get(master->dev.parent, "spi-clk");
	put_device(&master->dev);

	if (IS_ERR(clk)) {
		pr_warn("tkcoredrv: failed to get spi-clk: %d\n",
			PTR_ERR(clk));
		return h;
	}

	h->token = token;
	h->e = (void *) &clk_prepare_enable;
	h->d = (void *) &clk_disable_unprepare;
	h->p0 = (const void *) clk;
	h->argnum = 1;

	return h;
}

/* called inside list_lock */
static struct clkmgr_handle *get_clkmgr_handle(uint32_t token)
{
	struct clkmgr_handle *h;

	list_for_each_entry(h, &clk_list, le) {
		if (h->token == token)
			return h;
	}

	h = NULL;

	if (TEE_CLKMGR_TOKEN_TYPE(token) == TEE_CLKMGR_TYPE_SPI) {
		h = try_spi_clk(token);
		if (h == NULL)
			return NULL;

		list_add((&h->le), &clk_list);
	}

	return h;
}

int tee_clkmgr_handle(uint32_t token, uint32_t op)
{
	struct clkmgr_handle *ph, h;
	void *fn;

	mutex_lock(&clk_list_lock);

	ph = get_clkmgr_handle(token | TEE_CLKMGR_TOKEN_NOT_LEGACY);
	if (ph == NULL) {
		mutex_unlock(&clk_list_lock);
		return TEEC_ERROR_ITEM_NOT_FOUND;
	}

	memcpy(&h, ph, sizeof(h));

	mutex_unlock(&clk_list_lock);

	fn = (op & TEE_CLKMGR_OP_ENABLE) ? h.e : h.d;

	if (fn == NULL)
		TEEC_ERROR_NOT_SUPPORTED;

	if (h.argnum == 0) {
		((void (*)(void)) fn) ();
	} else if (h.argnum == 1) {
		((void (*)(const void *)) fn) (h.p0);
	} else if (h.argnum == 2) {
		((void (*)(const void *, const void *)) fn) (h.p0, h.p1);
	} else if (h.argnum == 3) {
		((void (*) (const void *, const void *, const void *)) fn)
			(h.p0, h.p1, h.p2);
	} else {
		pr_err("unsupported token %u argnum %zu\n",
			h.token, h.argnum);
		return TEEC_ERROR_NOT_SUPPORTED;
	}

	return 0;
}
EXPORT_SYMBOL(tee_clkmgr_handle);

int tee_clkmgr_register(const char *clkname, int id, void *e, void *d,
	void *p0, void *p1, void *p2, size_t argnum)
{
	size_t n;

	struct clkmgr_handle *h, *w;

	pr_info("tkcoredrv: clkname=%s id=%d\n",
		clkname, id);

	if (argnum > 3) {
		pr_err("does not support argnum %zu\n", argnum);
		return -EINVAL;
	}

	for (n = 0; n < ARRAY_SIZE(clkid); n++) {
		if (clkid[n] && strcmp(clkname, clkid[n]) == 0)
			break;
	}

	if (n == ARRAY_SIZE(clkid)) {
		pr_err("invalid clkname %s\n", clkname);
		return -EINVAL;
	}

	if ((id << TEE_CLKMGR_TOKEN_ID_SHIFT) &
		(TEE_CLKMGR_TOKEN_TYPE_MASK << TEE_CLKMGR_TOKEN_TYPE_SHIFT)) {
		pr_err("%s-%d: invalid id\n", clkname, id);
		return -EINVAL;
	}

	h = kmalloc(sizeof(struct clkmgr_handle), GFP_KERNEL);
	if (h == NULL)
		return -ENOMEM;

	h->token = TEE_CLKMGR_TOKEN((uint32_t) n, (uint32_t) id);
	h->e = e;
	h->d = d;
	h->p0 = p0;
	h->p1 = p1;
	h->p2 = p2;
	h->argnum = argnum;

	mutex_lock(&clk_list_lock);

	/* check for duplication */
	list_for_each_entry(w, &clk_list, le) {
		if (w->token == h->token) {
			pr_err("clk 0x%x already registered\n",
				h->token);
			mutex_unlock(&clk_list_lock);
			return -EINVAL;
		}
	}

	list_add(&(h->le), &clk_list);
	mutex_unlock(&clk_list_lock);

	return 0;
}
EXPORT_SYMBOL(tee_clkmgr_register);

int tee_spi_cfg_padsel(uint32_t padsel)
{
	struct TEEC_Context context;
	struct TEEC_Session session;
	struct TEEC_Operation op;

	TEEC_Result r;

	uint32_t returnOrigin;

	pr_info("padsel=0x%x\n", padsel);

	memset(&context, 0, sizeof(context));
	memset(&session, 0, sizeof(session));
	memset(&op, 0, sizeof(op));

	r = TEEC_InitializeContext(NULL, &context);
	if (r != TEEC_SUCCESS) {
		pr_err(
			"TEEC_InitializeContext() failed with 0x%08x\n", r);
		return r;
	}

	r = TEEC_OpenSession(
		&context, &session, &SENSOR_DETECTOR_TA_UUID,
		TEEC_LOGIN_PUBLIC,
		NULL, NULL, &returnOrigin);

	if (r != TEEC_SUCCESS) {
		pr_err(
			"TEEC_OpenSession failed with 0x%x returnOrigun: %u\n",
			r, returnOrigin);
		TEEC_FinalizeContext(&context);
		return r;
	}

	op.paramTypes = TEEC_PARAM_TYPES(
		TEEC_VALUE_INPUT,
		TEEC_NONE,
		TEEC_NONE,
		TEEC_NONE);

	op.params[0].value.a = padsel;

	r = TEEC_InvokeCommand(&session, CMD_CONFIG_PADSEL, &op, &returnOrigin);
	if (r != TEEC_SUCCESS) {
		pr_err(
			"TEEC_InvokeCommand() failed with 0x%08x returnOrigin: %u\n",
			r, returnOrigin);
	}

	TEEC_CloseSession(&session);
	TEEC_FinalizeContext(&context);

	return r;
}
EXPORT_SYMBOL(tee_spi_cfg_padsel);

int tee_spi_transfer(void *conf, uint32_t conf_size,
	void *inbuf, void *outbuf, uint32_t size)
{
	struct TEEC_Context context;
	struct TEEC_Session session;
	struct TEEC_Operation op;

	TEEC_Result r;

	char *buf;
	uint32_t returnOrigin;

	pr_info("conf=%p conf_size=%u inbuf=%p outbuf=%p size=%u\n",
		conf, conf_size, inbuf, outbuf, size);

	if (!conf || !inbuf || !outbuf) {
		pr_err("Bad parameters NULL buf\n");
		return -EINVAL;
	}

	if (size == 0) {
		pr_err("zero buf size\n");
		return -EINVAL;
	}

	memset(&context, 0, sizeof(context));
	memset(&session, 0, sizeof(session));
	memset(&op, 0, sizeof(op));

	memcpy(outbuf, inbuf, size);

	r = TEEC_InitializeContext(NULL, &context);
	if (r != TEEC_SUCCESS) {
		pr_err(
			"TEEC_InitializeContext() failed with 0x%08x\n", r);
		return r;
	}

	r = TEEC_OpenSession(
		&context, &session, &SENSOR_DETECTOR_TA_UUID,
		TEEC_LOGIN_PUBLIC,
		NULL, NULL, &returnOrigin);
	if (r != TEEC_SUCCESS) {
		pr_err(
			"TEEC_OpenSession failed with 0x%x returnOrigun: %u\n",
			r, returnOrigin);
		TEEC_FinalizeContext(&context);
		return r;
	}

	op.paramTypes = TEEC_PARAM_TYPES(
		TEEC_MEMREF_TEMP_INPUT,
		TEEC_MEMREF_TEMP_INOUT,
		TEEC_NONE,
		TEEC_NONE);

	op.params[0].tmpref.buffer = conf;
	op.params[0].tmpref.size = conf_size;

	op.params[1].tmpref.buffer = outbuf;
	op.params[1].tmpref.size = size;

	buf = outbuf;

	r = TEEC_InvokeCommand(&session, CMD_READ_CHIPID, &op, &returnOrigin);
	if (r != TEEC_SUCCESS) {
		pr_err(
			"TEEC_InvokeCommand() failed with 0x%08x returnOrigin: %u\n",
			r, returnOrigin);
	}

	TEEC_CloseSession(&session);
	TEEC_FinalizeContext(&context);

	return r;
}
EXPORT_SYMBOL(tee_spi_transfer);

int tee_spi_transfer_disable(void)
{
	struct TEEC_Context context;
	struct TEEC_Session session;
	struct TEEC_Operation op;

	TEEC_Result r;

	uint32_t returnOrigin;

	memset(&context, 0, sizeof(context));
	memset(&session, 0, sizeof(session));
	memset(&op, 0, sizeof(op));

	r = TEEC_InitializeContext(NULL, &context);
	if (r != TEEC_SUCCESS) {
		pr_err(
			"TEEC_InitializeContext() failed with 0x%08x\n", r);
		return r;
	}

	r = TEEC_OpenSession(
		&context, &session, &SENSOR_DETECTOR_TA_UUID,
		TEEC_LOGIN_PUBLIC,
		NULL, NULL, &returnOrigin);
	if (r != TEEC_SUCCESS) {
		pr_err(
			"TEEC_OpenSession failed with 0x%x returnOrigun: %u\n",
			r, returnOrigin);
		TEEC_FinalizeContext(&context);
		return r;
	}

	op.paramTypes = TEEC_PARAM_TYPES(
		TEEC_NONE,
		TEEC_NONE,
		TEEC_NONE,
		TEEC_NONE);

	r = TEEC_InvokeCommand(&session, CMD_DISABLE, &op, &returnOrigin);
	if (r != TEEC_SUCCESS) {
		pr_err(
			"TEEC_InvokeCommand() failed with 0x%08x returnOrigin: %u\n",
			r, returnOrigin);
	}

	TEEC_CloseSession(&session);
	TEEC_FinalizeContext(&context);

	return r;
}
EXPORT_SYMBOL(tee_spi_transfer_disable);

void tee_clkmgr_exit(void)
{
	struct clkmgr_handle *h, *n;

	mutex_lock(&clk_list_lock);

	list_for_each_entry_safe(h, n, &clk_list, le) {
		list_del(&(h->le));
		kfree(h);
	}

	mutex_unlock(&clk_list_lock);
}
