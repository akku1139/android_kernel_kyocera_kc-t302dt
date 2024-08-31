/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 * (C) 2019 KYOCERA Corporation
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

#ifndef __SENSOR_API_H_INCLUDED
#define __SENSOR_API_H_INCLUDED

#include <linux/device.h>

struct sensor_api_info {
	struct device	*api_dev;

	int (*enable_store)(struct sensor_api_info *sai, unsigned int enable);
	int (*enable_show)(struct sensor_api_info *sai, char *buf);
	int (*delay_store)(struct sensor_api_info *sai, unsigned int delay);
	int (*delay_show)(struct sensor_api_info *sai, char *buf);
	int (*value_show)(struct sensor_api_info *sai, char *buf);
	int (*status_show)(struct sensor_api_info *sai, char *buf);
	int (*imit_store)(struct sensor_api_info *sai, const char *buf);
	int (*imit_show)(struct sensor_api_info *sai, char *buf);
	int (*prop_store)(struct sensor_api_info *sai, const char *buf);
	int (*prop_show)(struct sensor_api_info *sai, char *buf);
	int (*valid_store)(struct sensor_api_info *sai, unsigned int valid);
	int (*valid_show)(struct sensor_api_info *sai, char *buf);
	int (*flush_store)(struct sensor_api_info *sai, unsigned int flush);
	int (*flush_show)(struct sensor_api_info *sai, char *buf);
	int (*thresh_show)(struct sensor_api_info *sai, char *buf);
	int (*calib_store)(struct sensor_api_info *sai, const char *buf);
	int (*calib_show)(struct sensor_api_info *sai, char *buf);
};

extern int sensor_api_register(struct device *api_dev, struct sensor_api_info *sai);
extern int sensor_api_unregister(struct sensor_api_info *sai);

#endif		/* __SENSOR_API_H_INCLUDED */
