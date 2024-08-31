/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 *
 * drivers/input/touchscreen/kc_dt.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef __LINUX_KC_DT_H
#define __LINUX_KC_DT_H

#include <linux/types.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/cdev.h>

#define KC_DT_DEV_DBG(fmt, arg...)		pr_debug(fmt, ## arg)
#define KC_DT_DEV_INFO(fmt, arg...)		pr_info(fmt, ## arg)
#define KC_DT_DEV_TOUCH(fmt, arg...)	pr_debug(fmt, ## arg)
#define KC_DT_DEV_I2C(fmt, arg...)		pr_debug(fmt, ## arg)
#define KC_DT_DEV_ERR(fmt, arg...)		pr_err(fmt, ## arg)

#define DT_XY_TEST	34
#define START_X		269
#define X_LINE		640
#define START_Y		13168
#define Y_LINE		388
#define MAX_FLUCTUATION		368
#define DT_READ_DATA	12

enum dt_ps_status {
	DT_PS_STATUS_CHARGE = 0,
	DT_PS_STATUS_DISCHARGE,
	DT_PS_STATUS_MAX,
};

struct dt_coordinate_test {
	int result_coordinate[DT_XY_TEST * 2];
	uint8_t success_coordinate[DT_XY_TEST];
};

struct dt_coordinate {
	uint8_t mode;
	uint16_t fluctuation;
	struct dt_coordinate_test coordinate_data;
};

struct dt_diag_type {
	int pressure;
	int x;
	int y;
	int hover;
	int rdy;
	int invert;
	int eraser;
	int tipswitch;
};

enum dt_sig_request {
	DT_SIG_UPDATE_FW = 0,
	DT_SIG_SET_CAL_NOTIFY,
};

struct kc_dt_data {
	const struct kc_dt_operations		*tops;
	struct device						*dev;
	void								*vdata;
	struct mutex						lock;
	struct cdev							device_cdev;
	int									device_major;
	struct class						*device_class;
	struct dt_coordinate				*dt_coordinate_data;
	struct dt_diag_type					*diag_data;
	bool								coordinate_test_enable;
	pid_t 								pid;
	int									android_notify;
	int									power_lock;
	int									soft_fw_ver;
	int									fw_ver_check_flg;
};

struct kc_dt_operations {
	int (*get_fw_version)(struct kc_dt_data *dt, int *fw_version);
	int (*hw_reset)(struct kc_dt_data *dt);
	int (*power_off)(struct kc_dt_data *dt);
	int (*power_on)(struct kc_dt_data *dt);
	int (*irq)(struct kc_dt_data *dt, unsigned int flg);
	long (*ioctl)(struct kc_dt_data *dt, unsigned int cmd, unsigned long arg);
};

extern unsigned int dt_notify_flag;

int kc_dt_probe(struct kc_dt_data *dt);
void kc_dt_remove(struct kc_dt_data *dt);
int dt_ctrl_init(struct kc_dt_data *dt, const struct file_operations *fops);
int dt_ctrl_exit(struct kc_dt_data *dt);
void kc_dt_diag_store(struct kc_dt_data *dt, struct dt_diag_type *coordinate);
int kc_dt_send_signal(struct kc_dt_data *dt, enum dt_sig_request req);

#define IOC_MAGIC 'd'
#define IOCTL_DIGI_COORDINATE_START _IO(IOC_MAGIC, 0xE1)
#define IOCTL_DIGI_COORDINATE_GET _IOR(IOC_MAGIC, 0xE2, struct dt_diag_type)
#define IOCTL_DIGI_COORDINATE_END _IO(IOC_MAGIC, 0xE3)
#define IOCTL_OPEN_SHORT _IOR(IOC_MAGIC, 0xE4, struct dt_coordinate)
#define IOCTL_DIGI_FW_VER_GET _IOWR(IOC_MAGIC, 0xF0, int)
#define IOCTL_DIGI_SET_PS_STAT _IOW(IOC_MAGIC, 0xF3, enum dt_ps_status)
#define IOCTL_DIGI_SET_PID _IOW(IOC_MAGIC, 0xF4, pid_t)

#endif /* __LINUX_KC_TS_H */
