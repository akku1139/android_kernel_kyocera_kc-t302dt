/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include "sensor_util.h"
#include "sensor_core.h"
#include "als_sensor.h"

#define	SENSOR_NAME	"als_sensor"
#define	ALS_DUMMY_VALUE	-1

int als_sensor_report_event(struct als_sensor_info *info, uint32_t lux, bool als_en_first)
{
	struct input_dev *idev;

	SENSOR_V_LOG("start lux[%d]", lux);

	if (unlikely(!info)) {
		SENSOR_ERR_LOG("fail bad parm --> info");
		return (-EINVAL);
	}

	idev = container_of(info->event_dev, struct input_dev, dev);

	if (unlikely(als_en_first))
		input_report_abs(idev, ABS_MISC, ALS_DUMMY_VALUE);

	input_report_abs(idev, ABS_MISC, lux);
	input_sync(idev);
	SENSOR_D_LOG("lux=%d", lux);

	SENSOR_V_LOG("end");
	return 0;
}

int als_sensor_report_flush_event(struct als_sensor_info *info)
{
	struct input_dev *idev;

	SENSOR_V_LOG("start");

	if (unlikely(!info)) {
		SENSOR_ERR_LOG("fail bad parm --> info");
		return (-EINVAL);
	}

	idev = container_of(info->event_dev, struct input_dev, dev);

	input_report_abs(idev, ABS_MISC+1, ~ALS_DUMMY_VALUE);
	input_report_abs(idev, ABS_MISC+1, ALS_DUMMY_VALUE);
	input_sync(idev);

	SENSOR_V_LOG("end");
	return 0;
}

static void als_sensor_setup_inputdev(struct input_dev *dev)
{
#define ALSMAX (65535)
	SENSOR_V_LOG("start");

	/* set event bit */
	set_bit(EV_ABS, dev->evbit);
	set_bit(EV_SYN, dev->evbit);

	/* initialization of abs event */
	input_set_abs_params(dev, ABS_MISC, 0, ALSMAX, 0, 0);
	input_set_abs_params(dev, ABS_MISC+1, INT_MIN, INT_MAX, 0, 0);

	SENSOR_V_LOG("end");
	return;
#undef ALSMAX
}

int als_sensor_register(struct als_sensor_info *info)
{
	int ret;

	SENSOR_D_LOG("start");

	if (!info) {
		SENSOR_ERR_LOG("fail bad parm --> info");
		return (-EINVAL);
	}

	ret = sensor_core_initialize_common(info->dev, SENSOR_NAME,
		als_sensor_setup_inputdev, &info->sai, &info->event_dev);
	if (ret) {
		SENSOR_ERR_LOG("fail sensor_core_initialize_common --> ret[%d]", ret);
		return ret;
	}

	SENSOR_D_LOG("end");
	return 0;
}

int als_sensor_unregister(struct als_sensor_info *info)
{
	SENSOR_D_LOG("start");

	if (info) {
		sensor_core_terminate_common(&info->sai, info->event_dev);
	}

	SENSOR_D_LOG("end");
	return 0;
}

MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("Light Sensor Driver");
MODULE_LICENSE("GPL");
