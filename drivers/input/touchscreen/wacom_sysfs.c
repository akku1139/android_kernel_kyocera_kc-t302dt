/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 */
/*
 * Wacom Penabled Driver for I2C
 *
 * Copyright (c) 2011 - 2015 Tatsunosuke Tobita, Wacom.
 * <tobita.tatsunosuke@wacom.co.jp>
 *
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software
 * Foundation; either version of 2 of the License,
 * or (at your option) any later version.
 */

#include "wacom.h"

#define CLASS_NAME "wacom"
#define DEVICE_NAME "wacom_emr"
#define ERR_REGISTER_SYSFS 56

#define WACOM_OFFSET_X_MIN (-200)
#define WACOM_OFFSET_X_MAX (200)
#define WACOM_OFFSET_Y_MIN (-200)
#define WACOM_OFFSET_Y_MAX (200)

#ifdef CALIBRATION
static const struct wacom_trigonometric_func sWacomTrigonometricFuncTable[] = {
	{  0, 16384},
	{ 14, 16384},
	{ 29, 16384},
	{ 43, 16384},
	{ 57, 16384},
	{ 71, 16384},
	{ 86, 16384},
	{100, 16384},
	{114, 16384},
	{129, 16383},
	{143, 16383},
	{157, 16383},
	{172, 16383},
	{186, 16383},
	{200, 16383},
	{214, 16383},
	{229, 16382},
	{243, 16382},
	{257, 16382},
	{272, 16382},
	{286, 16382},
	{300, 16381},
	{315, 16381},
	{329, 16381},
	{343, 16380},
	{357, 16380},
	{372, 16380},
	{386, 16379},
	{400, 16379},
	{415, 16379},
	{429, 16378},
	{443, 16378},
	{457, 16378},
	{472, 16377},
	{486, 16377},
	{500, 16376},
	{515, 16376},
	{529, 16375},
	{543, 16375},
	{558, 16375},
	{572, 16374},
	{586, 16374},
	{600, 16373},
	{615, 16372},
	{629, 16372},
	{643, 16371},
	{658, 16371},
	{672, 16370},
	{686, 16370},
	{700, 16369},
	{715, 16368},
	{729, 16368},
	{743, 16367},
	{758, 16366},
	{772, 16366},
	{786, 16365},
	{800, 16364},
	{815, 16364},
	{829, 16363},
	{843, 16362},
	{857, 16362}
};
#endif	/* CALIBRATION */

#ifdef OFFSET_SETTING
void convert_offsets(u8 flag_x, u8 flag_y, u16 temp_x, u16 temp_y,
		     int *offset_x, int *offset_y)
{
	if (flag_x & 0x80) {
		temp_x -= 1;
		temp_x = temp_x ^ 0xffff;
		*offset_x = -(int)temp_x;
	} else
		*offset_x = (int)temp_x;

	if (flag_y & 0x80) {
		temp_y -= 1;
		temp_y = temp_y ^ 0xffff;
		*offset_y = -(int)temp_y;
	} else
		*offset_y = (int)temp_y;
}

/*Below wacom_read, wacom_write, and following attributes added for sysfs*/
static ssize_t wacom_get_offset(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct wacom_i2c *wac_i2c = (struct wacom_i2c *)dev_get_drvdata(dev);
	u16 cmdreg;
	u16 datareg;
	u8 temp[RCGOFFSET_SIZE] = {0};
	u16 temp_x = 0, temp_y = 0;
	bool bRet = false;
	int offset_x = 0, offset_y = 0;

	if(wac_i2c->is_suspend){
		offset_x = offset_y = 0;
		pr_err("%s: IC is not active \n", __func__);
		goto out;
	}

	mutex_lock(&wac_i2c->seq_lock);
	cmdreg = wac_i2c->features->hid_desc.wCommandRegister;
	datareg = wac_i2c->features->hid_desc.wDataRegister;
	bRet = wacom_i2c_get_feature(wac_i2c->client, REPORT_CMD_GETOFFSET, RCGOFFSET_SIZE, temp,
				     cmdreg, datareg);
	mutex_unlock(&wac_i2c->seq_lock);
	if (!bRet) {
		printk("%s get feature failed \n", __func__);
		offset_x = offset_y = 10000;
		goto out;
	}

	temp_x = (temp[3] << 8 | temp[2]);
	temp_y = (temp[5] << 8 | temp[4]);
	convert_offsets(temp[3], temp[5], temp_x, temp_y, &offset_x, &offset_y);

 out:
	printk("%s: Offsets: x: %d y: %d \n", __func__, offset_x, offset_y);
	return sprintf(buf, "%d|%d", offset_x, offset_y);
}

static ssize_t wacom_set_offset(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct wacom_i2c *wac_i2c = (struct wacom_i2c *)dev_get_drvdata(dev);
	u16 cmdreg;
	u16 datareg;
	u8 temp[RCGOFFSET_SIZE] = {0};
	bool bRet = false;
	int offset_x = 0, offset_y = 0;
	u16 temp_x = 0, temp_y = 0;

	if(wac_i2c->is_suspend){
		pr_err("%s: IC is not active \n", __func__);
		goto no_active;
	}

	mutex_lock(&wac_i2c->seq_lock);
	cmdreg = wac_i2c->features->hid_desc.wCommandRegister;
	datareg = wac_i2c->features->hid_desc.wDataRegister;
	if (count == 8) {
		temp_x = (buf[1] << 8 | buf[0]);
		temp_y = (buf[5] << 8 | buf[4]);

		convert_offsets(buf[1], buf[5], temp_x, temp_y, &offset_x, &offset_y);
		if (( offset_x < WACOM_OFFSET_X_MIN ) ||
			( offset_x > WACOM_OFFSET_X_MAX ) ||
			( offset_y < WACOM_OFFSET_Y_MIN ) ||
			( offset_y > WACOM_OFFSET_Y_MAX )) {
			pr_err("%s: Offsets value error x: %d y: %d \n", __func__, offset_x, offset_y);
			goto out;
		}

		temp[0] = 22;
		temp[2] = buf[0];
		temp[3] = buf[1];
		temp[4] = buf[4];
		temp[5] = buf[5];

		bRet = wacom_i2c_set_feature(wac_i2c->client, REPORT_CMD_SETOFFSET, RCSOFFSET_SIZE, temp,
					     cmdreg, datareg);
		if (!bRet) {
			printk("%s setting offsets failed \n", __func__);
			goto out;
		}

		/*Just to show the obtained offsets*/
		printk("%s: Offsets x: %d y: %d \n", __func__, offset_x, offset_y);
	} else if (buf[0] == '$') {
		temp[0] = 22;
		bRet = wacom_i2c_set_feature(wac_i2c->client, REPORT_CMD_SETOFFSET, RCSOFFSET_SIZE, temp,
					     cmdreg, datareg);
		if (!bRet) {
			printk("%s: setting offsets failed \n", __func__);
			goto out;
		}
		
		printk("%s: offsets have been zeroed \n", __func__);
	} else {

		printk("%s: data is invalid\n", __func__);
	}

 out:
	mutex_unlock(&wac_i2c->seq_lock);
 no_active:
	return count;
}
#endif

#ifdef CALIBRATION
/*below for calibration operation*/
static ssize_t wacom_read_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct wacom_i2c *wac_i2c = (struct wacom_i2c *)dev_get_drvdata(dev);

	int x = wac_i2c->features->x_max;
	int y = wac_i2c->features->y_max;

	switch(wac_i2c->features->node_state){
	case STATE_QUERY:
		printk("%s STATE_QUERY: x_max:%d y_max:%d \n", __func__, x, y);
		return sprintf(buf, "%d|%d", x, y);

	case STATE_NORMAL:
		printk("%s STATE_NORMAL \n", __func__);
		break;

	default:
		printk("No mode is set\n");
		break;
	}

	return sprintf(buf, "%d", 0);
}

static ssize_t wacom_write_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct wacom_i2c *wac_i2c = (struct wacom_i2c *)dev_get_drvdata(dev);
	int i, err;
	u8 tmp;

	if (count == 1) {
	  switch(buf[0]){
	  case '*':
	    wac_i2c->features->node_state = STATE_QUERY;
	    printk("%s set STATE_QUERY \n", __func__);
	    break;

	  case '@':
		  wac_i2c->features->bCalibrationSet = false;
		  wac_i2c->features->calib_data.originX = 0;
		  wac_i2c->features->calib_data.originY = 0;
		  wac_i2c->features->calib_data.extentX = wac_i2c->features->x_max;
		  wac_i2c->features->calib_data.extentY = wac_i2c->features->y_max;
		  printk("-------------------------------------------------------\n");
		  printk("%s Calibration set to default: %d, %d, %d, %d\n", __func__,
			 wac_i2c->features->calib_data.originX,  wac_i2c->features->calib_data.originY,
			 wac_i2c->features->calib_data.extentX,  wac_i2c->features->calib_data.extentY);
		  printk("-------------------------------------------------------\n");
		  break;

	  case 0:
	  default:
	    printk("%s set STATE_NORMAL \n", __func__);
	    wac_i2c->features->node_state = STATE_NORMAL;
	    break;
	  }

	} else if (count == 16) {
	  struct calibrationData *tmp = (struct calibrationData*)buf;

	  wac_i2c->features->calib_data.originX = tmp->originX;
	  wac_i2c->features->calib_data.originY = tmp->originY;
	  wac_i2c->features->calib_data.extentX = tmp->extentX;
	  wac_i2c->features->calib_data.extentY = tmp->extentY;
	  wac_i2c->features->bCalibrationSet = true;

	  printk("-------------------------------------------------------\n");
	  printk("%s Calibration Data Set: %d, %d, %d, %d\n", __func__,
		  wac_i2c->features->calib_data.originX,  wac_i2c->features->calib_data.originY,
		  wac_i2c->features->calib_data.extentX,  wac_i2c->features->calib_data.extentY);
	  printk("-------------------------------------------------------\n");
	  err = kc_dt_send_signal(&wac_i2c->kc_dt, DT_SIG_SET_CAL_NOTIFY);
	  if (err < 0)
			pr_err("%s: failed kc_dt_send_signal()\n", __func__);

	} else if (count == 32) {
		wac_i2c->features->calib_data.originX = 0;
		wac_i2c->features->calib_data.originY = 0;
		wac_i2c->features->calib_data.extentX = 0;
		wac_i2c->features->calib_data.extentY = 0;

		for(i=0; i<4; i++){
			if(hex2bin(&tmp, buf, 1))
				pr_err("%s: x hex2bin is error. i=%d\n", __func__, i);
			buf = buf + 2;
			wac_i2c->features->calib_data.originX |= (tmp << (i*8));
		}

		for(i=0; i<4; i++){
			if(hex2bin(&tmp, buf, 1))
				pr_err("%s: y hex2bin is error. i=%d\n", __func__, i);
			buf = buf + 2;
			wac_i2c->features->calib_data.originY |= (tmp << (i*8));
		}

		for(i=0; i<4; i++){
			if(hex2bin(&tmp, buf, 1))
				pr_err("%s: ex hex2bin is error. i=%d\n", __func__, i);
			buf = buf + 2;
			wac_i2c->features->calib_data.extentX |= (tmp << (i*8));
		}

		for(i=0; i<4; i++){
			if(hex2bin(&tmp, buf, 1))
				pr_err("%s: ey hex2bin is error. i=%d\n", __func__, i);
			buf = buf + 2;
			wac_i2c->features->calib_data.extentY |= (tmp << (i*8));
		}

		printk("-------------------------------------------------------\n");
		printk("%s Calibration Data Set: %d, %d, %d, %d\n", __func__,
		  wac_i2c->features->calib_data.originX,  wac_i2c->features->calib_data.originY,
		  wac_i2c->features->calib_data.extentX,  wac_i2c->features->calib_data.extentY);
		printk("-------------------------------------------------------\n");
		err = kc_dt_send_signal(&wac_i2c->kc_dt, DT_SIG_SET_CAL_NOTIFY);
		if (err < 0)
			pr_err("%s: failed kc_dt_send_signal()\n", __func__);
	}
	return count;
}

/*Below for exposing information */
/*sensorData contains digitizer's */
/*x and y maximum size and its firmware verision*/
/*Format: items separated by '|'*/
static ssize_t wacom_get_sensor_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct wacom_i2c *wac_i2c = (struct wacom_i2c *)dev_get_drvdata(dev);

	return sprintf(buf, "%d|%d|%x", wac_i2c->features->x_max,
		       wac_i2c->features->y_max, wac_i2c->features->fw_version);
}

/*calibData contains digitizer's */
/*software calibration data     */
/*Format: items separated by '|'*/
static ssize_t wacom_get_calib_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct wacom_i2c *wac_i2c = (struct wacom_i2c *)dev_get_drvdata(dev);

	return sprintf(buf, "%d|%d|%d|%d",  wac_i2c->features->calib_data.originX,  wac_i2c->features->calib_data.originY,
		        wac_i2c->features->calib_data.extentX,  wac_i2c->features->calib_data.extentY);
}

static ssize_t wacom_read_rotation_cal(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct wacom_i2c *wac_i2c = (struct wacom_i2c *)dev_get_drvdata(dev);

	pr_info("%s cor_x:%d cor_y:%d  cor_theta:%d\n", __func__,
				wac_i2c->features->rotation_calib_data.cor_x,
				wac_i2c->features->rotation_calib_data.cor_y,
				wac_i2c->features->rotation_calib_data.cor_theta);
	return sprintf(buf, "%d %d %d",
				wac_i2c->features->rotation_calib_data.cor_x,
				wac_i2c->features->rotation_calib_data.cor_y,
				wac_i2c->features->rotation_calib_data.cor_theta);
}

static ssize_t wacom_write_rotation_cal(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct wacom_i2c *wac_i2c = (struct wacom_i2c *)dev_get_drvdata(dev);
	int i;
	u8 tmp;

	if(count == 12){
		struct rotationCalData *tmp = (struct rotationCalData*)buf;

		if( tmp->cor_x == 0 && tmp->cor_y == 0 && tmp->cor_theta == 0 ){
			wac_i2c->features->rotation_calib_data.cor_x = 0;
			wac_i2c->features->rotation_calib_data.cor_y = 0;
			wac_i2c->features->rotation_calib_data.cor_theta = 0;
		} else{
			wac_i2c->features->rotation_calib_data.cor_x = (tmp->cor_x * 1692) / 100;
			wac_i2c->features->rotation_calib_data.cor_y = (tmp->cor_y * 1692) / 100;
			wac_i2c->features->rotation_calib_data.cor_theta = tmp->cor_theta;
		}
	} else if(count == 24){
		wac_i2c->features->rotation_calib_data.cor_x = 0;
		wac_i2c->features->rotation_calib_data.cor_y = 0;
		wac_i2c->features->rotation_calib_data.cor_theta = 0;

		for(i=0; i<4; i++){
			if(hex2bin(&tmp, buf, 1))
				pr_err("%s: x hex2bin is error. i=%d\n", __func__, i);
			buf = buf + 2;
			wac_i2c->features->rotation_calib_data.cor_x |= (tmp << (i*8));
		}
		wac_i2c->features->rotation_calib_data.cor_x = (wac_i2c->features->rotation_calib_data.cor_x * 1692) / 100;

		for(i=0; i<4; i++){
			if(hex2bin(&tmp, buf, 1))
				pr_err("%s: y hex2bin is error. i=%d\n", __func__, i);
			buf = buf + 2;
			wac_i2c->features->rotation_calib_data.cor_y |= (tmp << (i*8));
		}
		wac_i2c->features->rotation_calib_data.cor_y = (wac_i2c->features->rotation_calib_data.cor_y * 1692) / 100;

		for(i=0; i<4; i++){
			if(hex2bin(&tmp, buf, 1))
				pr_err("%s: theta hex2bin is error. i=%d\n", __func__, i);
			buf = buf + 2;
			wac_i2c->features->rotation_calib_data.cor_theta |= (tmp << (i*8));
		}
	}

	pr_info("-------------------------------------------------------\n");
	pr_info("%s Rotation Calibration Data Set: %d, %d, %d\n", __func__,
			wac_i2c->features->rotation_calib_data.cor_x,
			wac_i2c->features->rotation_calib_data.cor_y,
			wac_i2c->features->rotation_calib_data.cor_theta);
	pr_info("-------------------------------------------------------\n");

	return count;
}
#endif	/* CALIBRATION */

#ifdef CHECK_MODE
bool check_if_user(struct wacom_i2c *wac_i2c)
{
	bool bRet = false;
	int ret;
	u8 cmd1[] = { WACOM_CMD_QUERY0, WACOM_CMD_QUERY1,
			WACOM_CMD_QUERY2, WACOM_CMD_QUERY3 };
	u8 cmd2[] = { WACOM_CMD_THROW0, WACOM_CMD_THROW1 };
	u8 data[WACOM_QUERY_SIZE];
	struct i2c_msg msgs[] = {
		{
			.addr = wac_i2c->client->addr,
			.flags = 0,
			.len = sizeof(cmd1),
			.buf = cmd1,
		},
		{
			.addr = wac_i2c->client->addr,
			.flags = 0,
			.len = sizeof(cmd2),
			.buf = cmd2,
		},
		{
			.addr = wac_i2c->client->addr,
			.flags = I2C_M_RD,
			.len = sizeof(data),
			.buf = data,
		},
	};

	mutex_lock(&wac_i2c->seq_lock);
	ret = wacom_i2c_transfer(wac_i2c->client, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		goto fail;
	if (ret != ARRAY_SIZE(msgs))
		goto fail;
	if (( le16_to_cpup( (__le16 *)data ) != WACOM_QUERY_SIZE ) || data[2] != 0x03) {
		dev_err( &wac_i2c->client->dev,
				 "%s: format is not valid;  length: %d; report id: %d\n",
			 __func__, le16_to_cpup( (__le16 *)data ), data[2] );
		goto fail;
	}

	bRet = true;
 fail:
	mutex_unlock(&wac_i2c->seq_lock);
	return bRet;
}

bool check_if_boot(struct wacom_i2c *wac_i2c)
{
	int ech = 0;
	u16 cmdreg;
	u16 datareg;
	unsigned char cmd[RCSBOOT_SIZE];
	unsigned char rsp[RCGBOOT_SIZE];
	bool bRet = false;

	cmd[0] = REPORT_CMD_SETBOOT;
	cmd[1] = QUERY_BOOT;
	cmd[2] = ech = 7;

	mutex_lock(&wac_i2c->seq_lock);
	cmdreg = wac_i2c->features->hid_desc.wCommandRegister;
	datareg = wac_i2c->features->hid_desc.wDataRegister;
	bRet = wacom_i2c_set_feature(wac_i2c->client, REPORT_CMD_SETBOOT, RCSBOOT_SIZE, cmd,
				     cmdreg, datareg);
	if (!bRet) {
		printk("%s setting offsets failed \n", __func__);
		goto fail;
	}

	bRet = wacom_i2c_get_feature(wac_i2c->client, REPORT_CMD_GETBOOT, RCGBOOT_SIZE, rsp,
				     cmdreg, datareg);
	if (!bRet) {
		printk("%s get feature failed \n", __func__);
		goto fail;
	}
	if ( (rsp[3] != REPORT_CMD_SETBOOT) ||
	     (rsp[4] != ech) ) {
		printk("%s returned values are invalid; rsp[3]: %x rsp[4]: %x \n", __func__, rsp[3], rsp[4]);
		goto fail;
	}
	if (rsp[5] != RCGBOOT_SIZE) {
		printk("%s returned value is invalid; rsp[5]: %x \n", __func__, rsp[5]);
		goto fail;
	}
	
	bRet = true;
 fail:
	mutex_unlock(&wac_i2c->seq_lock);
	return bRet;
}

/*0: user-mode; 1: boot-mode; -1: not working*/
static ssize_t wacom_check_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct wacom_i2c *wac_i2c = (struct wacom_i2c *)dev_get_drvdata(dev);
	int ret = NOT_WORKING;

	if(wac_i2c->is_suspend){
		ret = NOT_WORKING;
		pr_err("%s: IC is not active \n", __func__);
		goto out;
	}

	if (!check_if_user(wac_i2c)) {
		printk("%s: The digitizer is not in user-mode\n", __func__);
	} else {
		printk("Currently in user mode\n");
		ret = IN_USER_MODE;
		goto out;
	}

	if (!check_if_boot(wac_i2c)) {
		printk("%s: The digitizer is not in boot-mode\n", __func__);
	} else {
		printk("Currently in boot mode\n");
		ret = IN_BOOT_MODE;
		goto out;
	}

	printk("the digitizer is not currently working \n");
	ret = NOT_WORKING;
 out:
	return sprintf(buf, "%d", ret);
}
#endif

static ssize_t wacom_write_fw_update(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct wacom_i2c *wac_i2c = (struct wacom_i2c *)dev_get_drvdata(dev);
	struct kc_dt_data *dt = (struct kc_dt_data *)&wac_i2c->kc_dt;
	struct i2c_client *client = wac_i2c->client;
	int error;

	if (sscanf(buf, "%d", &wac_i2c->fw_updating) != 1)
		printk("%s: Arg Error!\n", __func__);

	if (wac_i2c->fw_updating) {
		wacom_i2c_disable_irq(wac_i2c);
		mutex_lock(&wac_i2c->seq_lock);
		wacom_i2c_pre_fw_update(wac_i2c);
		wac_i2c->fwdl_mode = true;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0))
		reinit_completion(&wac_i2c->fwdl_completion);
#else
		INIT_COMPLETION(wac_i2c->fwdl_completion);
#endif

		mutex_unlock(&wac_i2c->seq_lock);
	} else {
		msleep(130);
		wacom_hw_reset(wac_i2c);
		error = wacom_i2c_post_fw_update(wac_i2c);
		if (error){
			if(wac_i2c->is_enable != -1){
				free_irq(client->irq, wac_i2c);
				wac_i2c->is_enable = -1;
			}
			pr_err("%s: wacom_i2c_post_fw_update fail error=%d \n", __func__, error);
			goto done;
		}

		if(dt->soft_fw_ver != wac_i2c->features->fw_version){
			if(wac_i2c->is_enable != -1){
				free_irq(client->irq, wac_i2c);
				wac_i2c->is_enable = -1;
			}
			pr_err("%s: fail fw version is 0x%04x  soft_fw_ver  is 0x%04x error=%d \n"
				, __func__, wac_i2c->features->fw_version, dt->soft_fw_ver, error);
			goto done;
		}
		/* IRQ is not register ? */
		if(wac_i2c->is_enable == -1){
			error = request_threaded_irq(client->irq, NULL, wacom_i2c_irq,
						     IRQF_TRIGGER_LOW | IRQF_ONESHOT,
						     "wacom_i2c", wac_i2c);
			if (error){
				pr_err("Failed to enable IRQ, error: %d\n", error);
				goto  done;
			}
			wac_i2c->is_enable = 1;
		}
		else{
			wacom_i2c_enable_irq(wac_i2c);
		}
		wac_i2c->fwdl_mode = false;
		complete_all(&wac_i2c->fwdl_completion);

		mutex_lock(&wac_i2c->seq_lock);
		if (unlikely(wac_i2c->is_suspend)){
			wacom_i2c_disable_irq_nosync(wac_i2c);
		}
		mutex_unlock(&wac_i2c->seq_lock);

		pr_info("%s: SUCCESS \n", __func__);
	}
done:
	pr_info("%s: fw_updating set to %d\n", __func__, wac_i2c->fw_updating);

	return count;
}

static ssize_t wacom_read_fw_update(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct wacom_i2c *wac_i2c = (struct wacom_i2c *)dev_get_drvdata(dev);

	return sprintf(buf, "%d", wac_i2c->fw_updating);
}

#ifdef OFFSET_SETTING
static DEVICE_ATTR(get_offset, S_IRUSR, wacom_get_offset, NULL);
static DEVICE_ATTR(set_offset, S_IWUSR, NULL, wacom_set_offset);
#endif

#ifdef CALIBRATION
static DEVICE_ATTR(calibration, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP, wacom_read_status, wacom_write_data);
static DEVICE_ATTR(information, S_IRUSR, wacom_get_sensor_data, NULL);
static DEVICE_ATTR(calData, S_IRUSR, wacom_get_calib_data, NULL);
static DEVICE_ATTR(rotation_cal, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP, wacom_read_rotation_cal, wacom_write_rotation_cal);
#endif

#ifdef CHECK_MODE
static DEVICE_ATTR(check_mode, S_IRUSR, wacom_check_mode, NULL);
#endif

static DEVICE_ATTR(fw_update, S_IRUSR|S_IWUSR, wacom_read_fw_update, wacom_write_fw_update);

static struct attribute *wacom_attributes[] = {
#ifdef OFFSET_SETTING
	&dev_attr_get_offset.attr,
	&dev_attr_set_offset.attr,
#endif

#ifdef CALIBRATION
	&dev_attr_calibration.attr,
	&dev_attr_information.attr,
	&dev_attr_calData.attr,
	&dev_attr_rotation_cal.attr,
#endif

#ifdef CHECK_MODE
	&dev_attr_check_mode.attr,
#endif
	&dev_attr_fw_update.attr,
	NULL,
};

static struct attribute_group wacom_attr_group = {
	.attrs = wacom_attributes,
};

#define STRING_MAX 64
int register_sysfs(struct wacom_i2c *wac_i2c)
{
	int error;

	/*Below added for sysfs*/
	/*If everything done without any failure, register sysfs*/
	wac_i2c->class = class_create(THIS_MODULE, CLASS_NAME);
	wac_i2c->dev = device_create(wac_i2c->class, NULL, 0, NULL, DEVICE_NAME);
	if (IS_ERR(&wac_i2c->dev)) {
		dev_err(wac_i2c->dev,
			"Failed to create device, \"wac_i2c\"\n");
		goto err_create_class;
	}

	dev_set_drvdata(wac_i2c->dev, wac_i2c);
	error = sysfs_create_group(&wac_i2c->dev->kobj, &wacom_attr_group);
	if (error) {
		dev_err(wac_i2c->dev,
			"Failed to create sysfs group, \"wacom_attr_group\": (%d) \n", error);
		goto err_create_sysfs;
	}
	
	return 0;
	
 err_create_sysfs:
	sysfs_remove_group(&wac_i2c->dev->kobj, &wacom_attr_group);
 err_create_class:
	device_destroy(wac_i2c->class, 0);
	class_destroy(wac_i2c->class);
	
	return -ERR_REGISTER_SYSFS;
}

void remove_sysfs(struct wacom_i2c *wac_i2c)
{
	/*Clear sysfs resources*/
	sysfs_remove_group(&wac_i2c->dev->kobj, &wacom_attr_group);
	device_destroy(wac_i2c->class, 0);
	class_destroy(wac_i2c->class);
}

#ifdef CALIBRATION
void set_calib(int *x, int *y, int x_max, int y_max,
	       int originX, int originY, int extentX, int extentY)
{
	int temp_coord;
	/*X-Coordination with calibration value*/
	temp_coord = *x * extentX / x_max + originX;

	/*Check if obtained coordinations don't exceeds thier maximum or get negative numbers*/
	if (temp_coord < 0)
		*x = 0;
	else if (temp_coord > x_max)
		*x = x_max;
	else
		*x = temp_coord;
	
	/*Y-Coordination with calibration value*/
	temp_coord = *y * extentY / y_max + originY;
	
	/*Check if obtained coordinations don't exceeds thier maximum or get negative numbers*/
	if (temp_coord < 0)
		*y = 0;
	else if (temp_coord > y_max)
		*y = y_max;
	else
		*y = temp_coord;
}

void set_rotation_calib(int *x, int *y, int cor_x, int cor_y, int cor_theta)
{
	int temp_x, temp_y, index, work_sin, work_cos;
	int org_x = *x;
	int org_y = *y;

	if( (cor_theta < -300) || (cor_theta > 300)){
		pr_err("%s: cor_theta value error cor_theta: %d \n", __func__, cor_theta);
		return;
	}

	index = abs(cor_theta) / 5;
	if(cor_theta < 0){
		work_sin = -sWacomTrigonometricFuncTable[index].drv_sin;
		work_cos = sWacomTrigonometricFuncTable[index].drv_cos;
	} else{
		work_sin = sWacomTrigonometricFuncTable[index].drv_sin;
		work_cos = sWacomTrigonometricFuncTable[index].drv_cos;
	}

	temp_x  = ((org_x - HALF_X) * work_cos) >>14;
	temp_x -= ((org_y - HALF_Y) * work_sin) >>14;
	temp_x += HALF_X -cor_x;
	*x = temp_x;

	temp_y  = ((org_x - HALF_X) * work_sin) >>14;
	temp_y += ((org_y - HALF_Y) * work_cos) >>14;
	temp_y += HALF_Y -cor_y;
	*y = temp_y;

	pr_debug("%s: org_x:%d org_y:%d cor_x:%d cor_y:%d theta:%d x:%d y:%d sin:%d cos:%d\n",
			__func__, org_x, org_y, cor_x, cor_y, cor_theta, *x, *y, work_sin, work_cos );

	return;
}
#endif
