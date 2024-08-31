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

#include <linux/module.h>
#include <linux/kernel.h>
#include "sensor_core.h"
#include "sensor_api.h"
#include "sensor_util.h"

static struct input_dev *sensor_core_inputdev_create(
	struct device *parent, const char *name,
	void (*setup_func)(struct input_dev *))
{
	struct input_dev *idev;
	int err;

	SENSOR_D_LOG("start");

	if (!parent) {
		SENSOR_ERR_LOG("fail bad parm --> parent");
		goto fail;
	}

	if (!name) {
		SENSOR_ERR_LOG("fail bad parm --> name");
		goto fail;
	}

	if (!setup_func) {
		SENSOR_ERR_LOG("fail bad parm --> setup_func");
		goto fail;
	}

	idev = input_allocate_device();
	if (!idev) {
		SENSOR_ERR_LOG("fail input_allocate_device");
		goto fail;
	}
	idev->name = name;
	idev->dev.parent = parent;
	setup_func(idev);

	err = input_register_device(idev);
	if (err < 0) {
		SENSOR_ERR_LOG("fail input_register_device --> err[%d]", err);
		goto fail_input_register_device;
	}

	SENSOR_D_LOG("end -return[%p]", idev);
	return idev;

fail_input_register_device:
	input_free_device(idev);
fail:
	SENSOR_ERR_LOG("end -return[NULL]");
	return NULL;
}

static int sensor_core_inputdev_destroy(struct input_dev *idev)
{
	SENSOR_D_LOG("start idev[%p]", idev);

	if (idev) {
		input_unregister_device(idev);
		input_free_device(idev);
	}

	SENSOR_D_LOG("end");
	return 0;
}

int sensor_core_initialize_common(struct device *dev, const char *name,
	void (*setup_func)(struct input_dev *),
	struct sensor_api_info *sai, struct device **event_dev)
{
	struct input_dev *idev;
	int ret;

	SENSOR_D_LOG("start");

	if (!dev) {
		SENSOR_ERR_LOG("fail bad parm --> dev");
		return (-EINVAL);
	}
	if (!name) {
		SENSOR_ERR_LOG("fail bad parm --> name");
		return (-EINVAL);
	}
	if (!setup_func) {
		SENSOR_ERR_LOG("fail bad parm --> setup_func");
		return (-EINVAL);
	}
	if (!sai) {
		SENSOR_ERR_LOG("fail bad parm --> sai");
		return (-EINVAL);
	}
	if (!event_dev) {
		SENSOR_ERR_LOG("fail bad parm --> event_dev");
		return (-EINVAL);
	}

	idev = sensor_core_inputdev_create(dev, name, setup_func);

	if (!idev) {
		SENSOR_ERR_LOG("fail sensor_core_inputdev_create");
		return (-ENOMEM);
	}

	ret = sensor_api_register(&idev->dev, sai);
	if (ret) {
		SENSOR_ERR_LOG("fail sensor_api_register --> ret[%d]", ret);
		goto err_sensor_api_register;
	}

	*event_dev = &idev->dev;

	SENSOR_D_LOG("end");
	return 0;

err_sensor_api_register:
	sensor_core_inputdev_destroy(idev);

	SENSOR_ERR_LOG("end -return[%d]", ret);
	return ret;
}
EXPORT_SYMBOL(sensor_core_initialize_common);

int sensor_core_terminate_common(struct sensor_api_info *sai, struct device *event_dev)
{
	struct input_dev *idev;

	SENSOR_D_LOG("start");

	if (sai && event_dev) {
		sensor_api_unregister(sai);
		idev = container_of(event_dev, struct input_dev, dev);
		sensor_core_inputdev_destroy(idev);
	}

	SENSOR_D_LOG("end");
	return 0;
}
EXPORT_SYMBOL(sensor_core_terminate_common);

MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("Sensor Driver");
MODULE_LICENSE("GPL");
