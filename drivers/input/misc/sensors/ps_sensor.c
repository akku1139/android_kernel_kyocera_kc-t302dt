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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include "sensor_util.h"
#include "sensor_core.h"
#include "ps_sensor.h"

#define	SENSOR_NAME	"ps_sensor"
#define	PROXIMITY_DUMMY_VALUE	(-1)
#define	WAKE_LOCK_TIME_DETECT	(200)
#define	WAKE_LOCK_TIME_NODETECT	(1000)

static atomic_t g_prox_cal_result;
static DEFINE_MUTEX(prox_event_mutex);

int ps_sensor_report_event(struct ps_sensor_info *info, uint32_t detect)
{
	struct input_dev *idev;
	long	wake_lock_time;

	SENSOR_V_LOG("start detect[%d]", detect);

	if (unlikely(!info)) {
		SENSOR_ERR_LOG("fail bad parm --> info");
		return (-EINVAL);
	}

	if (!atomic_read(&info->valid)) {
		SENSOR_ERR_LOG("ps sensor invalid. skip.");
		return 0;
	}

	idev = container_of(info->event_dev, struct input_dev, dev);

    mutex_lock(&prox_event_mutex);
	input_report_abs(idev, ABS_DISTANCE, PROXIMITY_DUMMY_VALUE);
	input_report_abs(idev, ABS_DISTANCE, detect);
	input_sync(idev);
    mutex_unlock(&prox_event_mutex);
	SENSOR_N_LOG("detect=%d", detect);

	wake_lock_time = detect ? WAKE_LOCK_TIME_DETECT : WAKE_LOCK_TIME_NODETECT;
	pm_wakeup_event(info->dev, wake_lock_time);

	SENSOR_V_LOG("end");
	return 0;
}

static void ps_sensoor_cal_send_uevent(struct input_dev *dev)
{
    char *envp[] = {"PS_CALIB_UEVENT", NULL };

    SENSOR_D_LOG("start");
    if (!dev) {
        SENSOR_ERR_LOG("dev NULL");
        return;
    }
    if (!&dev->dev.kobj) {
        SENSOR_ERR_LOG("&dev->dev.kobj NULL");
        return;
    }

    kobject_uevent_env(&dev->dev.kobj, KOBJ_CHANGE, envp);
    SENSOR_D_LOG("dev = 0x%lx, &dev->dev.kobj = 0x%lx"
                    ,(unsigned long)dev, (unsigned long)&dev->dev.kobj);

    SENSOR_D_LOG("end");
}

void ps_sensor_cal_report_event(struct ps_sensor_info *info, int8_t cal_result)
{
	struct input_dev *idev;
    SENSOR_D_LOG("start");

	idev = container_of(info->event_dev, struct input_dev, dev);
    mutex_lock(&prox_event_mutex);
    if(cal_result == PS_CAL_OK){
        ps_sensoor_cal_send_uevent(idev);
    }
    mutex_unlock(&prox_event_mutex);

    SENSOR_D_LOG("end");
}


int ps_sensor_report_flush_event(struct ps_sensor_info *info)
{
	struct input_dev *idev;

	SENSOR_V_LOG("start");

	if (unlikely(!info)) {
		SENSOR_ERR_LOG("fail bad parm --> info");
		return (-EINVAL);
	}

	idev = container_of(info->event_dev, struct input_dev, dev);

	input_report_abs(idev, ABS_MISC, ~PROXIMITY_DUMMY_VALUE);
	input_report_abs(idev, ABS_MISC, PROXIMITY_DUMMY_VALUE);
	input_sync(idev);

	SENSOR_V_LOG("end");
	return 0;
}

static void ps_sensor_setup_inputdev(struct input_dev *dev)
{
	SENSOR_V_LOG("start");

	/* set event bit */
	set_bit(EV_ABS, dev->evbit);
	set_bit(EV_SYN, dev->evbit);

	/* initialization of abs event */
	input_set_abs_params(dev, ABS_DISTANCE, 0, 1, 0, 0);
	input_set_abs_params(dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);

	SENSOR_V_LOG("end");
	return;
}

int ps_sensor_register(struct ps_sensor_info *info)
{
	int ret;

	SENSOR_D_LOG("start");

	if (!info) {
		SENSOR_ERR_LOG("fail bad parm --> info");
		return (-EINVAL);
	}

	ret = sensor_core_initialize_common(info->dev, SENSOR_NAME,
		ps_sensor_setup_inputdev, &info->sai, &info->event_dev);
	if (ret) {
		SENSOR_ERR_LOG("fail sensor_core_initialize_common --> ret[%d]", ret);
		return ret;
	}

	device_init_wakeup(info->dev, 1);

	atomic_set(&info->valid, 1);
    atomic_set(&g_prox_cal_result, PS_CAL_OK);

	SENSOR_D_LOG("end");
	return 0;
}

int ps_sensor_unregister(struct ps_sensor_info *info)
{
	SENSOR_D_LOG("start");

	if (info) {
		device_init_wakeup(info->dev, 0);
		sensor_core_terminate_common(&info->sai, info->event_dev);
	}

	atomic_set(&info->valid, 0);

	SENSOR_D_LOG("end");
	return 0;
}

MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("Proximity Sensor Driver");
MODULE_LICENSE("GPL");
