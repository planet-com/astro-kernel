# Copyright (c) 2015-2018 TrustKernel Incorporated
# All Rights Reserved.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# version 2 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.

obj-$(CONFIG_TRUSTKERNEL_TEE_SUPPORT) += tkcore.o

M ?=$(srctree)/drivers/misc/mediatek/tkcore

ccflags-y += \
	-I$(M)/include/linux \
	-I$(M)/include \
	-I$(M)

tkcore-y := \
	tee_core.o \
	tee_context.o \
	tee_session.o \
	tee_shm.o \
	tee_supp_com.o \
	tee_sysfs.o \
	tee_kernel_api.o \
	tee_wait_queue.o \
	tee_procfs.o \
	tee_ta_mgmt.o \
	pm.o

tkcore-$(CONFIG_TRUSTKERNEL_TEE_FP_SUPPORT) += \
	peridev.o
