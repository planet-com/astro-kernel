/*
 * Copyright (C) 2017 MediaTek Inc.
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

#include "kd_imgsensor.h"
#include "imgsensor_sensor_list.h"

/* Add Sensor Init function here
 * Note:
 * 1. Add by the resolution from ""large to small"", due to large sensor
 *    will be possible to be main sensor.
 *    This can avoid I2C error during searching sensor.
 * 2. This file should be the same as
 *    mediatek\custom\common\hal\imgsensor\src\sensorlist.cpp
 */
struct IMGSENSOR_SENSOR_LIST
	gimgsensor_sensor_list[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(IMX586_MIPI_RAW)
{IMX586_SENSOR_ID, SENSOR_DRVNAME_IMX586_MIPI_RAW, IMX586_MIPI_RAW_SensorInit},
#endif
#if defined(S5K3L8_MIPI_RAW)
{S5K3L8_SENSOR_ID, SENSOR_DRVNAME_S5K3L8_MIPI_RAW, S5K3L8_MIPI_RAW_SensorInit},
#endif

#if defined(IMX214_MIPI_RAW)
{IMX214_SENSOR_ID, SENSOR_DRVNAME_IMX214_MIPI_RAW, IMX214_MIPI_RAW_SensorInit},
#endif
/*GC*/
#if defined(GC5035_MIPI_RAW)
{GC5035_SENSOR_ID, SENSOR_DRVNAME_GC5035_MIPI_RAW, GC5035_MIPI_RAW_SensorInit},
#endif
	/*  ADD sensor driver before this line */
	{0, {0}, NULL}, /* end of list */
};

/* e_add new sensor driver here */

