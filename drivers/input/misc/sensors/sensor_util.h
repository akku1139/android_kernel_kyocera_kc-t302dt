/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 */
/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#ifndef __SENSOR_UTIL_H_INCLUDED
#define __SENSOR_UTIL_H_INCLUDED

#include <linux/kernel.h>
#include <linux/device.h>

//#define SENSOR_DRIVER_DEBUG

#ifdef SENSOR_DRIVER_DEBUG
#define SENSOR_DEVD_LOG(dev, msg, ...) \
	dev_notice(dev, "[SENSOR][%s][D](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#define SENSOR_D_LOG(msg, ...) \
	pr_notice("[SENSOR][%s][D](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#define SENSOR_V_LOG(msg, ...) \
	pr_notice("[SENSOR][%s][V](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#else
#define SENSOR_DEVD_LOG(dev, msg, ...) \
	dev_dbg(dev, "[SENSOR][%s][D](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#define SENSOR_D_LOG(msg, ...) \
	pr_debug("[SENSOR][%s][D](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#define SENSOR_V_LOG(msg, ...)
#endif

#define SENSOR_DEVN_LOG(dev, msg, ...) \
	dev_notice(dev, "[SENSOR][%s][N](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#define SENSOR_N_LOG(msg, ...) \
	pr_notice("[SENSOR][%s][N](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)

#define SENSOR_A_LOG(msg, ...) \
	pr_notice("[SENSOR][%s][A](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)

#define SENSOR_DEVE_LOG(msg, ...) \
	dev_err(dev, "[SENSOR][%s][E](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#define SENSOR_ERR_LOG(msg, ...) \
	pr_err("[SENSOR][%s][E](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)

#endif /* __SENSOR_UTIL_H_INCLUDED */
