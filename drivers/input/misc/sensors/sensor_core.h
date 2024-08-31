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

#ifndef __SENSOR_CORE_H_INCLUDED
#define __SENSOR_CORE_H_INCLUDED

#include <linux/device.h>
#include <linux/input.h>
#include "sensor_api.h"

int sensor_core_initialize_common(struct device *dev, const char *name,
	void (*setup_func)(struct input_dev *),
	struct sensor_api_info *sai, struct device **event_dev);
int sensor_core_terminate_common(struct sensor_api_info *sai,
	struct device *event_dev);

#endif		/* __SENSOR_CORE_H_INCLUDED */
