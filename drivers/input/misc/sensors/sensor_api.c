/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 * (C) 2017 KYOCERA Corporation
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

#include <linux/module.h>
#include <linux/kernel.h>
#include "sensor_api.h"
#include "sensor_util.h"

#define DEFINE_SENSOR_API_SHOW(_name)	\
static ssize_t sensor_api_ ## _name ## _show(		\
	struct device *dev,				\
	struct device_attribute *attr,			\
	char *buf )			\
{		\
	struct sensor_api_info *sai = dev_get_drvdata(dev);	\
	int ret;	\
	\
	SENSOR_DEVD_LOG(dev, "start");	\
	\
	if (sai-> _name ## _show) {	\
		ret = sai-> _name ## _show(sai, buf);	\
		if (ret < 0)	\
			return scnprintf(buf, PAGE_SIZE, "Failed to read!\n");	\
	} else {	\
		return scnprintf(buf, PAGE_SIZE, "not supported.\n");	\
	}	\
	\
	SENSOR_DEVD_LOG(dev, "end");	\
	return ret;	\
}

#define DEFINE_SENSOR_API_STORE_STRING(_name)	\
static ssize_t sensor_api_ ## _name ## _store(	\
	struct device *dev,	\
	struct device_attribute *attr,	\
	const char *buf,	\
	size_t count)	\
{	\
	struct sensor_api_info *sai = dev_get_drvdata(dev);	\
	\
	SENSOR_DEVD_LOG(dev, "start buf=%s", buf);	\
	\
	if (sai-> _name ## _store)	\
		sai-> _name ## _store(sai, buf);	\
	\
	SENSOR_DEVD_LOG(dev, "end - return[%d]", (int)count);	\
	\
    return count;	\
}

#define DEFINE_SENSOR_API_STORE_UL(_name)	\
static ssize_t sensor_api_ ## _name ## _store(	\
	struct device *dev,	\
	struct device_attribute *attr,	\
	const char *buf,	\
	size_t count )	\
{	\
	struct sensor_api_info *sai = dev_get_drvdata(dev);	\
	unsigned long value = 0;	\
	int ret = 0;	\
	\
	SENSOR_DEVD_LOG(dev, "start buf=%s", buf);	\
	\
	ret = kstrtoul(buf, 10, &value);	\
	\
	if (sai-> _name ## _store)	\
		sai-> _name ## _store(sai, value);	\
	\
	SENSOR_DEVD_LOG(dev, "end - return[%d]", (int)count);	\
	return count;	\
}

#define DEFINE_SENSOR_API_ATTR_RO(_name) \
	static DEVICE_ATTR(_name, 0440, sensor_api_##_name##_show, NULL)
#define DEFINE_SENSOR_API_ATTR_RW(_name) \
	static DEVICE_ATTR(_name, 0660, sensor_api_##_name##_show, sensor_api_##_name##_store)

DEFINE_SENSOR_API_SHOW(enable);
DEFINE_SENSOR_API_STORE_UL(enable);
DEFINE_SENSOR_API_ATTR_RW(enable);

DEFINE_SENSOR_API_SHOW(delay);
DEFINE_SENSOR_API_STORE_UL(delay);
DEFINE_SENSOR_API_ATTR_RW(delay);

DEFINE_SENSOR_API_SHOW(value);
DEFINE_SENSOR_API_ATTR_RO(value);

DEFINE_SENSOR_API_SHOW(status);
DEFINE_SENSOR_API_ATTR_RO(status);

DEFINE_SENSOR_API_SHOW(imit);
DEFINE_SENSOR_API_STORE_STRING(imit);
DEFINE_SENSOR_API_ATTR_RW(imit);

DEFINE_SENSOR_API_SHOW(prop);
DEFINE_SENSOR_API_STORE_STRING(prop);
DEFINE_SENSOR_API_ATTR_RW(prop);

DEFINE_SENSOR_API_SHOW(valid);
DEFINE_SENSOR_API_STORE_UL(valid);
DEFINE_SENSOR_API_ATTR_RW(valid);

DEFINE_SENSOR_API_SHOW(flush);
DEFINE_SENSOR_API_STORE_UL(flush);
DEFINE_SENSOR_API_ATTR_RW(flush);

DEFINE_SENSOR_API_SHOW(thresh);
DEFINE_SENSOR_API_ATTR_RO(thresh);

DEFINE_SENSOR_API_SHOW(calib);
DEFINE_SENSOR_API_STORE_STRING(calib);
DEFINE_SENSOR_API_ATTR_RW(calib);


static struct attribute *sensor_api_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_value.attr,
	&dev_attr_status.attr,
	&dev_attr_imit.attr,
	&dev_attr_prop.attr,
	&dev_attr_valid.attr,
	&dev_attr_flush.attr,
	&dev_attr_thresh.attr,
	&dev_attr_calib.attr,
	NULL,
};

static struct attribute_group sensor_api_attr_grp = {
    .attrs = sensor_api_attrs
};

int sensor_api_register(struct device *api_dev, struct sensor_api_info *sai)
{
	int err;

	SENSOR_D_LOG("start");

	if (!api_dev) {
		SENSOR_ERR_LOG("fail bad parm --> api_dev");
		return (-EINVAL);
	}
	if (!sai) {
		SENSOR_ERR_LOG("fail bad parm --> sai");
		return (-EINVAL);
	}

	dev_set_drvdata(api_dev, sai);

	err = sysfs_create_group(&api_dev->kobj, &sensor_api_attr_grp);
	if (err < 0) {
		SENSOR_ERR_LOG("fail sysfs_create_group() --> err[%d]", err);
		goto err_sysfs_create_group;
	}
	sai->api_dev = api_dev;

	SENSOR_D_LOG("end -return[0]");
	return 0;

err_sysfs_create_group:
	dev_set_drvdata(api_dev, NULL);

	SENSOR_ERR_LOG("end -return[%d]", err);
	return err;
}

int sensor_api_unregister(struct sensor_api_info *sai)
{
	struct device *api_dev;

	SENSOR_D_LOG("start");

	if (sai && sai->api_dev) {
		api_dev = sai->api_dev;

		sai->api_dev = NULL;
		sysfs_remove_group(&api_dev->kobj, &sensor_api_attr_grp);
		dev_set_drvdata(api_dev, NULL);
	}

	SENSOR_D_LOG("end");

	return 0;
}

MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("Sensor API");
MODULE_LICENSE("GPL");
