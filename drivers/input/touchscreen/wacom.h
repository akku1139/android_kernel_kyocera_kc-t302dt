/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 */
/*
 * Wacom Penabled Driver for I2C
 *
 * Copyright (c) 2011-2015 Tatsunosuke Tobita, Wacom.
 * <tobita.tatsunosuke@wacom.co.jp>
 *
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software
 * Foundation; either version of 2 of the License,
 * or (at your option) any later version.
 */

#ifndef WACOM_I2C_H
#define WACOM_I2C_H

#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/input/mt.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/version.h>
#include <asm/unaligned.h>
#include <asm/uaccess.h>
#include "kc_dt.h"

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif //CONFIG_HAS_EARLYSUSPEND

/*Defining flags to ebable/disable a functionality*/
#define CALIBRATION
#define ROTATION_CALIB    1
#define OFFSET_SETTING
#define CHECK_MODE
/*---------------------*/

#define WACOM_QUERY_SIZE	19
#define WACOM_INPUT_SIZE        10
#define AA_OFFSET               100

#define WACOM_MAX_SIZE          128
#define WACOM_EMR_INPUTSIZE     10

#define CMD_SET_FEATURE         0x03
#define CMD_GET_FEATURE         0x02

/*Added for using this prog. in Linux user-space program*/
#define RTYPE_FEATURE           0x03 /*: Report type -> feature(11b)*/
#define GET_FEATURE             0x02
#define SET_FEATURE             0x03
#define GFEATURE_SIZE           6
#define SFEATURE_SIZE           8

#ifdef OFFSET_SETTING
/*ReportID for GET/SET features*/
/*PEN*/
#define REPORT_CMD_SETOFFSET           22
#define RCSOFFSET_SIZE                 7
#define REPORT_CMD_GETOFFSET           REPORT_CMD_SETOFFSET           
#define RCGOFFSET_SIZE                 RCSOFFSET_SIZE
#endif

/*HID over I2C spec*/
#define HID_DESC_REGISTER       0x01
#define USAGE_PAGE              0x05
#define USAGE_PAGE_EXTENDED     0x06
#define USAGE_PAGE_DIGITIZERS   0x0d
#define USAGE_PAGE_DESKTOP      0x01
#define USAGE_PAGE_VENDOR       0x00ff
#define USAGE                   0x09
#define USAGE_PEN               0x02
#define USAGE_MOUSE             0x02
#define USAGE_FINGER            0x22
#define USAGE_STYLUS            0x20
#define USAGE_TOUCHSCREEN       0x04
#define USAGE_X                 0x30
#define USAGE_TIPPRESSURE       0x30
#define USAGE_Y                 0x31
#define USAGE_VENDOR            0x04
#define ENDCOLLECTION           0xc0

#ifdef CHECK_MODE
#define WACOM_CMD_QUERY0	0x04
#define WACOM_CMD_QUERY1	0x00
#define WACOM_CMD_QUERY2	0x33
#define WACOM_CMD_QUERY3	0x02
#define WACOM_CMD_THROW0	0x05
#define WACOM_CMD_THROW1	0x00
#define WACOM_QUERY_SIZE	19

#define IN_USER_MODE            0
#define IN_BOOT_MODE            1
#define NOT_WORKING             -1

#define REPORT_CMD_SETBOOT      0x07
#define QUERY_BOOT              0x07
#define RCSBOOT_SIZE            3
#define REPORT_CMD_GETBOOT      0x08
#define RCGBOOT_SIZE            6
#endif

#define WACOM_DEVICE_ERROR_RETRY_CNT	3
#define WACOM_I2C_RETRY_WAIT			10000	/* usec */

#define DIGI_MAX_X				13536
#define DIGI_MAX_Y				21658
#define HALF_X					6768
#define HALF_Y					10829

typedef struct hid_descriptor {
	u16 wHIDDescLength;
	u16 bcdVersion;
	u16 wReportDescLength;
	u16 wReportDescRegister;
	u16 wInputRegister;
	u16 wMaxInputLength;
	u16 wOutputRegister;
	u16 wMaxOutputLength;
	u16 wCommandRegister;
	u16 wDataRegister;
	u16 wVendorID;
	u16 wProductID;
	u16 wVersion;
	u16 RESERVED_HIGH;
	u16 RESERVED_LOW;
} HID_DESC;

#ifdef CALIBRATION
/*Added for Calibration purpose*/
typedef enum {
	STATE_NORMAL,
	STATE_QUERY,
	STATE_GETCAL,
	STATE_POINTS,
} NODE_STATE;

struct calibrationData {
  int   originX;
  int   originY;
  int   extentX;
  int   extentY; 
};

struct rotationCalData {
  int   cor_x;
  int   cor_y;
  int   cor_theta;
};
#endif

struct wacom_features {
	HID_DESC hid_desc;
	unsigned int input_size;
	int x_max;
	int y_max;
	int pressure_max;
	int height_max;
	int fw_version;
	int vendorId;
	int productId;

#ifdef CALIBRATION
	struct calibrationData calib_data;
	struct rotationCalData rotation_calib_data;
	NODE_STATE node_state;
	bool bCalibrationSet;
#endif
};

struct wacom_i2c {
	struct i2c_client *client;
	struct input_dev *input;
	struct wacom_features *features;
	struct class *class;
	struct device *dev;

#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif

	int tool;
	u8 data[WACOM_MAX_SIZE];
	u8 cmd;
	bool prox;
	struct pinctrl *dt_pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_suspend;
	u32 gpio_fwe;
	u32 gpio_reset;
	u32 gpio_pdctb;
	u32 gpio_irq;
	u32 gpio_ldo;
	struct kc_dt_data kc_dt;

	struct work_struct hw_reset_work;
	int hw_error_count;

	struct workqueue_struct *resume_wq;
	struct work_struct resumework;

	int is_enable;
	struct mutex seq_lock;
	int fw_updating;
	bool fwdl_mode;
	bool is_suspend;
	struct completion fwdl_completion;
};

struct gpio_request_tbl_type {
	int     gpio;
	char    *request_name;
	};

struct gpio_set_tbl_type {
	int     gpio;
	int     direction;
	int     val;
};

struct wacom_trigonometric_func {
	int     drv_sin;
	int     drv_cos;
};

int wacom_query_device(struct i2c_client *client, struct wacom_features *features);

#ifdef OFFSET_SETTING
bool wacom_i2c_set_feature(struct i2c_client *client, u8 report_id, unsigned int buf_size, u8 *data, 
			   u16 cmdreg, u16 datareg);
bool wacom_i2c_get_feature(struct i2c_client *client, u8 report_id, unsigned int buf_size, u8 *data, 
			   u16 cmdreg, u16 datareg);
#endif

#ifdef CALIBRATION
void set_calib(int *x, int *y, int x_max, int y_max, 
	       int originX, int originY, int extentX, int extentY);
void set_rotation_calib(int *x, int *y, int cor_x, int cor_y, int cor_theta);
#endif

int register_sysfs(struct wacom_i2c *wac_i2c);
void remove_sysfs(struct wacom_i2c *wac_i2c);

int wacom_i2c_send( struct i2c_client *client, u8 *buf, u16 len );
int wacom_i2c_transfer( struct i2c_client *client, struct i2c_msg *msgs, int size );
void wacom_hw_reset(struct wacom_i2c *wac_i2c);
void wacom_i2c_enable_irq(struct wacom_i2c *wac_i2c);
void wacom_i2c_disable_irq(struct wacom_i2c *wac_i2c);
void wacom_i2c_disable_irq_nosync(struct wacom_i2c *wac_i2c);
void wacom_i2c_pre_fw_update(struct wacom_i2c *wac_i2c);
int wacom_i2c_post_fw_update(struct wacom_i2c *wac_i2c);
irqreturn_t wacom_i2c_irq(int irq, void *dev_id);
#endif
