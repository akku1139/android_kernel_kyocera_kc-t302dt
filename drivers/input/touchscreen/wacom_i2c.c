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
#include "kc_ts_in.h"

#define WACOM_HID_REG0		0x01
#define WACOM_HID_REG1		0x00
#define WACOM_HID_DATA_SIZE		30
#define WACOM_HID_FW_VER_OFFSET	24

#define WACOM_FWDL_TIMEOUT		20000

#define WACOM_SYSFS
#define ERR_REGISTER_SYSFS 0x56a

static int  wacom_get_fw_version(struct i2c_client *client, int *fw_version);
static int  wacom_i2c_input_register(struct wacom_i2c *wac_i2c);


ktime_t wac_report_time;
struct wacom_i2c *digitizer_data = NULL;

/*Eliminating the digitizer AA offset; this makes the coordination exactly fit the LCD size*/
/*However, on the other hand, the difference in rotation of the screens against digitizers */
/*must be manually considered*/
static void set_offset(int *x, int *y, int x_max, int y_max)
{
	int temp_coord = *y;
	*y = *x;
	*x = x_max - temp_coord;
}

static void wacom_open_short_test(struct kc_dt_data *dt, int fluctuation, int x, int y)
{
	struct dt_coordinate_test *xy_res =
						&dt->dt_coordinate_data->coordinate_data;
	int i, j = 0;
	uint16_t fluctuation_min;

	for(i=0; i<DT_XY_TEST*2; i+=2){
		if(i==0){
			if(fluctuation > START_X)
				fluctuation_min = START_X;
			else
				fluctuation_min = fluctuation;
		} else {
			fluctuation_min = fluctuation;
		}

		if(xy_res->result_coordinate[i] - fluctuation_min <= x &&
		   x <= xy_res->result_coordinate[i] + fluctuation &&
		   xy_res->result_coordinate[i+1] - fluctuation <= y &&
		   y <= xy_res->result_coordinate[i+1] + fluctuation){
			xy_res->success_coordinate[j] = 0;
			pr_info("%s: success %d: (x, y) = (%5d, %5d)\n",
						 __func__, j, x, y);
			break;
		}
		j++;
	}
}

void wacom_i2c_pre_fw_update(struct wacom_i2c *wac_i2c)
{
	pr_debug("%s is called\n", __func__);
	gpio_set_value_cansleep(wac_i2c->gpio_fwe, 0);
	usleep_range(2000, 2000);
	gpio_set_value_cansleep(wac_i2c->gpio_reset, 1);
	usleep_range(6000, 6000);
	gpio_set_value_cansleep(wac_i2c->gpio_reset, 0);
	msleep(35);
	pr_debug("%s is ended\n", __func__);
}

int wacom_i2c_post_fw_update(struct wacom_i2c *wac_i2c)
{
	int error = -1;
	pr_debug("%s is called\n", __func__);

	error = wacom_i2c_input_register(wac_i2c);
	if(error){
		pr_err("%s: wacom_i2c_input_register fail error=%d\n", __func__, error);
	}
	pr_debug("%s is ended ret = %d \n", __func__, error);
	return error;
}

static void wacom_i2c_power_off(struct wacom_i2c *wac_i2c){
	pr_debug("%s is called\n", __func__);
	gpio_set_value_cansleep(wac_i2c->gpio_ldo, 0);
	usleep_range(2000,2000);
	gpio_set_value_cansleep(wac_i2c->gpio_fwe, 0);
	usleep_range(10000,10000);
	wac_i2c->is_suspend = true;
	pr_debug("%s is ended\n", __func__);
}

static void wacom_i2c_power_on(struct wacom_i2c *wac_i2c){
	pr_debug("%s is called\n", __func__);
	gpio_set_value_cansleep(wac_i2c->gpio_reset, 1);
	gpio_set_value_cansleep(wac_i2c->gpio_fwe, 1);
	usleep_range(1000,1000);
	gpio_set_value_cansleep(wac_i2c->gpio_ldo, 1);
	usleep_range(2000,2000);
	gpio_set_value_cansleep(wac_i2c->gpio_reset, 0);
	msleep(35);

	wac_i2c->is_suspend = false;
	pr_debug("%s is ended\n", __func__);
}

static void wacom_i2c_release_notification(struct wacom_i2c *wac_i2c)
{
	struct input_dev *input = wac_i2c->input;
	pr_debug("%s is called\n", __func__);

	if(input != NULL){
		input_report_key(input, BTN_TOUCH, 0);
		input_report_key(input, BTN_TOOL_RUBBER, 0);
		input_report_key(input, BTN_TOOL_PEN, 0);
		input_sync(input);
	}else{
		pr_err("%s wac_i2c->input is not init\n", __func__);
	}

	pr_debug("%s is ended\n", __func__);
}

static void wacom_i2c_hw_err_reset_work(struct work_struct *work)
{
	struct wacom_i2c *wac_i2c = container_of(work, struct wacom_i2c, hw_reset_work);

	pr_err("%s is called\n", __func__);

	if(wac_i2c->fwdl_mode){
		pr_info("%s fwdl execution.\n",__func__);
		return;
	}

	mutex_lock(&wac_i2c->seq_lock);

	wacom_i2c_power_off(wac_i2c);
	wacom_i2c_release_notification(wac_i2c);

	wacom_i2c_power_on(wac_i2c);
	/* hardware reset */
	wacom_hw_reset(wac_i2c);

	wacom_i2c_enable_irq(wac_i2c);

	mutex_unlock(&wac_i2c->seq_lock);

	pr_err("%s is ended\n", __func__);
}

static void wacom_i2c_hw_err_reset( struct i2c_client *client )
{
	int ret = 0;
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);

	pr_err("%s is called\n", __func__);

	wacom_i2c_disable_irq_nosync(wac_i2c);

	if (wac_i2c->hw_error_count < WACOM_DEVICE_ERROR_RETRY_CNT) {
		wac_i2c->hw_error_count++;
		pr_err("%s hw_error_count:%d\n", __func__, wac_i2c->hw_error_count);
		ret = schedule_work( &wac_i2c->hw_reset_work );
	} else {
		pr_err("%s recovery skip hw_error_count:%d\n", __func__, wac_i2c->hw_error_count);
	}
	pr_err("%s is ended[%d]\n", __func__, ret);
}

int wacom_i2c_send( struct i2c_client *client, u8 *buf, u16 len )
{
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);
	u8 retry = WACOM_DEVICE_ERROR_RETRY_CNT;
	int err = 0;

	do{
		err = i2c_master_send(client, buf, len);
		if (unlikely(err < 0)) {
			pr_err("%s: i2c_master_recv fail err=%d\n", __func__, err);
			usleep_range(WACOM_I2C_RETRY_WAIT,WACOM_I2C_RETRY_WAIT);
			continue;
		}
		usleep_range(55, 55);
		break;
	} while (retry-- > 0);

	if (unlikely(err < 0)) {
		wacom_i2c_hw_err_reset(client);
	} else {
		wac_i2c->hw_error_count = 0;
	}

	return err;
}

static int wacom_i2c_recv( struct i2c_client *client, u8 *buf, u16 len )
{
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);
	u8 retry = WACOM_DEVICE_ERROR_RETRY_CNT;
	int err = 0;

	do{
		err = i2c_master_recv(client, buf, len);
		if (unlikely(err < 0)) {
			pr_err("%s: i2c_master_recv fail err=%d\n", __func__, err);
			usleep_range(WACOM_I2C_RETRY_WAIT,WACOM_I2C_RETRY_WAIT);
			continue;
		}
		usleep_range(55, 55);
		break;
	} while (retry-- > 0);

	if (unlikely(err < 0)) {
		wacom_i2c_hw_err_reset(client);
	} else {
		wac_i2c->hw_error_count = 0;
	}

	return err;
}

int wacom_i2c_transfer( struct i2c_client *client, struct i2c_msg *msgs, int size )
{
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);
	u8 retry = WACOM_DEVICE_ERROR_RETRY_CNT;
	int err = 0;

	do{
		err = i2c_transfer(client->adapter, msgs, size);
		if (unlikely(err < 0)) {
			pr_err("%s: i2c_transfer fail err=%d\n", __func__, err);
			usleep_range(WACOM_I2C_RETRY_WAIT,WACOM_I2C_RETRY_WAIT);
			continue;
		}
		usleep_range(55, 55);
		break;
	} while (retry-- > 0);

	if (unlikely(err < 0)) {
		wacom_i2c_hw_err_reset(client);
	} else {
		wac_i2c->hw_error_count = 0;
	}

	return err;
}

irqreturn_t wacom_i2c_irq(int irq, void *dev_id)
{
	struct wacom_i2c *wac_i2c = dev_id;
	struct input_dev *input = wac_i2c->input;
	struct kc_dt_data *dt = (struct kc_dt_data *)&wac_i2c->kc_dt;
	u8 *data = wac_i2c->data;
	int x, y, pressure, height;
	char tsw, ers;
	int error;

	mutex_lock(&wac_i2c->seq_lock);

	if (unlikely(wac_i2c->is_enable != 1))
		goto out;

	if (unlikely(wac_i2c->is_suspend)){
		goto out;
	}

	error = wacom_i2c_recv(wac_i2c->client, data, wac_i2c->features->input_size);
	if (unlikely(error < 0))
		goto out;

	tsw = data[3] & 0x01;
	ers = data[3] & 0x04;
	x = le16_to_cpup((__le16 *)&data[4]);
	y = le16_to_cpup((__le16 *)&data[6]);
	pressure = le16_to_cpup((__le16 *)&data[8]);
	height = pressure ? 0 : data[10];

	if(unlikely(wac_i2c->kc_dt.diag_data)) {
		struct dt_diag_type coordinate;

		coordinate.pressure = pressure;
		coordinate.x = x;
		coordinate.y = y;
		coordinate.hover = height;
		coordinate.rdy = (data[3] & 0x20)>>5;
		coordinate.invert = (data[3] & 0x08)>>3;
		coordinate.eraser = ers>>2;
		coordinate.tipswitch = tsw;
		pr_debug("%s(%d) X=%d, Y=%d, Height=%xd, RDY=%d\n",
				 __func__, __LINE__, x, y, height, coordinate.rdy);
		kc_dt_diag_store(&wac_i2c->kc_dt, &coordinate);
	}

	if(unlikely(wac_i2c->kc_dt.coordinate_test_enable))
		wacom_open_short_test(&wac_i2c->kc_dt,
					wac_i2c->kc_dt.dt_coordinate_data->fluctuation, x, y);

	set_offset(&x, &y, wac_i2c->features->x_max, wac_i2c->features->y_max);
	if(unlikely( x > DIGI_MAX_X || y > DIGI_MAX_Y)){
		pr_err("%s: Abnormal coordinates = %d , %d\n", __func__, x, y);
		goto out;
	}

#ifdef CALIBRATION
	/*Set calibration values*/
	if(likely(ROTATION_CALIB)){
		if (( wac_i2c->features->node_state == STATE_NORMAL ) &&
			( wac_i2c->features->rotation_calib_data.cor_x != 0 || wac_i2c->features->rotation_calib_data.cor_y != 0 ))
			set_rotation_calib(	&x, &y,
					wac_i2c->features->rotation_calib_data.cor_x,
					wac_i2c->features->rotation_calib_data.cor_y,
					wac_i2c->features->rotation_calib_data.cor_theta);
	} else{
		if (wac_i2c->features->bCalibrationSet && wac_i2c->features->node_state == STATE_NORMAL)
			set_calib(&x, &y, wac_i2c->features->x_max, wac_i2c->features->y_max,
				  wac_i2c->features->calib_data.originX, wac_i2c->features->calib_data.originY,
				  wac_i2c->features->calib_data.extentX, wac_i2c->features->calib_data.extentY);
	}
#endif


	if (!wac_i2c->prox)
		wac_i2c->tool = (data[3] & 0x0c) ?
			BTN_TOOL_RUBBER : BTN_TOOL_PEN;

	wac_i2c->prox = data[3] & 0x20;

	if (unlikely(!dt->android_notify)){
		pr_info("%s android_notify lock\n",__func__);
		goto out;
	}

	wac_report_time = ktime_get();

	input_report_key(input, wac_i2c->tool, wac_i2c->prox);
	input_report_key(input, BTN_TOUCH, tsw || ers);
	input_report_abs(input, ABS_X, x);
	input_report_abs(input, ABS_Y, y);
	input_report_abs(input, ABS_PRESSURE, pressure);
	input_report_abs(input, ABS_DISTANCE, height);
	input_sync(input);

out:
	mutex_unlock(&wac_i2c->seq_lock);
	return IRQ_HANDLED;
}

void wacom_hw_reset(struct wacom_i2c *wac_i2c)
{
	gpio_set_value(wac_i2c->gpio_fwe, 1);
//	usleep_range(2000,2000);
	gpio_set_value(wac_i2c->gpio_reset,1);
	usleep_range(6000,6000);
	gpio_set_value(wac_i2c->gpio_reset,0);
	wac_i2c->is_suspend = false;
	msleep(35);
}

static int wacom_i2c_wait_for_completion(struct wacom_i2c *wac_i2c,
				   struct completion *comp,
				   unsigned int timeout_ms)
{
	struct device *dev = &wac_i2c->client->dev;
	unsigned long timeout = msecs_to_jiffies(timeout_ms);
	long ret;

	ret = wait_for_completion_interruptible_timeout(comp, timeout);
	if (ret < 0) {
		dev_err(dev, "Wait for completion error : %ld.\n", ret);
		return ret;
	} else if (ret == 0) {
		dev_err(dev, "Wait for completion timed out.\n");
		return -ETIMEDOUT;
	}
	return 0;
}

static int wacom_i2c_open(struct input_dev *dev)
{
	pr_debug("%s(%d)\n", __func__, __LINE__);
	return 0;
}

static void wacom_i2c_close(struct input_dev *dev)
{
	pr_debug("%s(%d)\n", __func__, __LINE__);
}

#ifdef CONFIG_PM_SLEEP
int wacom_i2c_suspend(struct device *dev)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);
	struct kc_dt_data *dt = (struct kc_dt_data *)&wac_i2c->kc_dt;
	int ret = 0;

	pr_debug("%s(%d)\n", __func__, __LINE__);

	if(wac_i2c->is_enable == -1){
		pr_info("%s wac_i2c->enable is not init \n",__func__);
		return -1;
	}

	if(dt->power_lock){
		pr_info("%s power_lock\n",__func__);
		return 0;
	}

	cancel_work_sync(&wac_i2c->hw_reset_work);
	wacom_i2c_disable_irq(wac_i2c);

	mutex_lock(&wac_i2c->seq_lock);

	if (wac_i2c->fwdl_mode) {
		pr_info("%s fwdl wait\n",__func__);
		ret = wacom_i2c_wait_for_completion( wac_i2c, &wac_i2c->fwdl_completion, WACOM_FWDL_TIMEOUT );
		if (ret) {
			pr_err("%s wait_for_completion error [%d]\n",__func__, ret);
			mutex_unlock(&wac_i2c->seq_lock);
			return ret;
		}
	}

	wacom_i2c_power_off(wac_i2c);
	wacom_i2c_release_notification(wac_i2c);
	mutex_unlock(&wac_i2c->seq_lock);

	return 0;
}

int wacom_i2c_resume(struct device *dev)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);
	struct kc_dt_data *dt = (struct kc_dt_data *)&wac_i2c->kc_dt;
	int ret;

	pr_debug("%s(%d)\n", __func__, __LINE__);

	if(wac_i2c->is_enable == -1){
		pr_info("%s wac_i2c->enable is not init \n",__func__);
		return -1;
	}

	if(dt->power_lock){
		pr_info("%s power_lock\n",__func__);
		return 0;
	}

	mutex_lock(&wac_i2c->seq_lock);

	if (wac_i2c->fwdl_mode) {
		pr_info("%s fwdl wait\n",__func__);
		ret = wacom_i2c_wait_for_completion( wac_i2c, &wac_i2c->fwdl_completion, WACOM_FWDL_TIMEOUT );
		if (ret) {
			pr_err("%s wait_for_completion error [%d]\n",__func__, ret);
			mutex_unlock(&wac_i2c->seq_lock);
			return -1;
		}
	}

	wacom_i2c_power_on(wac_i2c);
	/* hardware reset */
	wacom_hw_reset(wac_i2c);

	wacom_i2c_enable_irq(wac_i2c);
	mutex_unlock(&wac_i2c->seq_lock);

	return 0;
}

static void wacom_resume_work(struct work_struct *work)
{
	struct wacom_i2c *wac_i2c = container_of(work, struct wacom_i2c, resumework);
	struct kc_dt_data *dt = (struct kc_dt_data *)&wac_i2c->kc_dt;
	int ret = 0;

	pr_debug("%s(%d)\n", __func__, __LINE__);

	if(wac_i2c->is_enable == -1){
		pr_info("%s wac_i2c->enable is not init \n",__func__);
		return ;
	}

	if(dt->power_lock){
		pr_info("%s power_lock\n",__func__);
		return ;
	}

	mutex_lock(&wac_i2c->seq_lock);

	if (wac_i2c->fwdl_mode) {
		pr_info("%s fwdl wait\n",__func__);
		ret = wacom_i2c_wait_for_completion( wac_i2c, &wac_i2c->fwdl_completion, WACOM_FWDL_TIMEOUT );
		if (ret) {
			pr_err("%s wait_for_completion error [%d]\n",__func__, ret);
			mutex_unlock(&wac_i2c->seq_lock);
			return ;
		}
	}

	wacom_i2c_power_on(wac_i2c);
	/* hardware reset */
	wacom_hw_reset(wac_i2c);

	wacom_i2c_enable_irq(wac_i2c);
	mutex_unlock(&wac_i2c->seq_lock);
	return ;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct wacom_i2c *wac_i2c =
		container_of(self, struct wacom_i2c, fb_notif);

	printk("%s(%d)\n", __func__, __LINE__);

	if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			wac_i2c->hw_error_count = 0;
			if(likely(wac_i2c->resume_wq))
				queue_work(wac_i2c->resume_wq, &wac_i2c->resumework);
		} else if (*blank == FB_BLANK_POWERDOWN){
			if(likely(wac_i2c->resume_wq))
				flush_workqueue(wac_i2c->resume_wq);
			wacom_i2c_suspend(&wac_i2c->client->dev);
		}
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void wacom_i2c_set_sleep(struct device *dev, bool into_sleep)
{
	int ret = -1;
	char cmd[4] = {0x04, 0x00, 0x01, 0x08};
	struct i2c_client *client = to_i2c_client(dev);

	if (!into_sleep)
		cmd[2] = 0x00;

	ret = i2c_master_send(client, cmd, 4);
	if (ret != 4) {
		printk("Failed: %s \n", __func__);
	} else {
		printk("Succeeded: %s\n", __func__);
	}
//	usleep(55);
	usleep_range(55, 55);

	return;
}

static void wacom_i2c_early_suspend(struct early_suspend *handler)
{
	struct wacom_i2c *wac_i2c = container_of(handler, struct wacom_i2c, early_suspend);
	struct device *dev = &wac_i2c->client->dev;

	printk("%s()\r\n", __func__);
	wacom_i2c_suspend(dev);
	wacom_i2c_set_sleep(dev, true);

	return;
}

static void wacom_i2c_late_resume(struct early_suspend *handler)
{
	struct wacom_i2c *wac_i2c = container_of(handler, struct wacom_i2c, early_suspend);
	struct device *dev = &wac_i2c->client->dev;

	printk("%s()\r\n", __func__);
	wacom_i2c_set_sleep(dev, false);
	wacom_i2c_resume(dev);

	return;
}
#endif //CONFIG_FB CONFIG_HAS_EARLYSUSPEND
#endif //CONFIG_PM_SLEEP

static int wacom_pinctrl_init(struct wacom_i2c *data)
{
	int retval;
	pr_debug("%s is called\n", __func__);

	/* Get pinctrl if target uses pinctrl */
	data->dt_pinctrl = devm_pinctrl_get(&(data->client->dev));
	if (IS_ERR_OR_NULL(data->dt_pinctrl)) {
		retval = PTR_ERR(data->dt_pinctrl);
		dev_err(&data->client->dev,
			"Target does not use pinctrl %d\n", retval);
		return retval;
	}

	data->pinctrl_state_active
		= pinctrl_lookup_state(data->dt_pinctrl, "pmx_digi_active");
	if (IS_ERR_OR_NULL(data->pinctrl_state_active)) {
		retval = PTR_ERR(data->pinctrl_state_active);
		dev_err(&data->client->dev,
			"Can not lookup pmx_digi_active pinstate %d\n", retval);
		return retval;
	}

	data->pinctrl_state_suspend
		= pinctrl_lookup_state(data->dt_pinctrl, "pmx_digi_suspend");
	if (IS_ERR_OR_NULL(data->pinctrl_state_suspend)) {
		retval = PTR_ERR(data->pinctrl_state_suspend);
		dev_err(&data->client->dev,
			"Can not lookup pmx_digi_suspend pinstate %d\n", retval);
		return retval;
	}

	pr_debug("%s is ended\n", __func__);
	return 0;
}

static int wacom_pinctrl_select(struct wacom_i2c *data, bool on)
{
	struct pinctrl_state *pins_state;
	int error;

	pr_debug("%s is called. bool is %d\n",__func__,on);

	pins_state = on ? data->pinctrl_state_active :
					  data->pinctrl_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		error = pinctrl_select_state(data->dt_pinctrl, pins_state);
		if (error) {
			dev_err(&data->client->dev, "can not set %s pins\n",
				on ? "pmx_digi_active" : "pmx_digi_suspend");
			return error;
		}
	} else {
		dev_err(&data->client->dev,
			"not a valid '%s' pinstate\n",
			on ? "pmx_digi_active" : "pmx_digi_suspend");
	}

	pr_debug("%s is ended\n", __func__);
	return 0;

}

static int wacom_gpio_init(struct wacom_i2c *data, int on)
{
	struct device *dev = &data->client->dev;

	int rc = 0;
	int i,j;
	struct gpio_request_tbl_type gpio_req[]={
		{ data->gpio_fwe    ,"wacom_fwe"    },
		{ data->gpio_reset  ,"wacom_reset"  },
		{ data->gpio_pdctb  ,"wacom_pdctb"  },
		{ data->gpio_irq    ,"wacom_irq"    },
		{ data->gpio_ldo    ,"wacom_ldo"    }
	};
	struct gpio_set_tbl_type gpio_set[]={
		{ data->gpio_fwe    , 1, 0},
		{ data->gpio_reset  , 1, 0},
		{ data->gpio_pdctb  , 0, 0},
		{ data->gpio_irq    , 0, 0},
		{ data->gpio_ldo    , 1, 0},
//		{ data->gpio_ldo    , 1, 1},
	};

	if(on){
		for(i=0 ; i<(sizeof(gpio_req)/sizeof(struct gpio_request_tbl_type)) ; i++){
			rc = gpio_request(gpio_req[i].gpio, gpio_req[i].request_name);
			if (rc < 0) {
				pr_err("%s: Fail request gpio=%d\n", __func__,gpio_req[i].gpio);
				break;
			}
		}
		/* err */
		if(rc < 0){
			for(j=0 ; j<i; j++){
				gpio_free(gpio_req[j].gpio);
				pr_debug("%s: gpio_free gpio=%d\n",__func__, gpio_req[j].gpio);
			}
			return rc;
		}
		for(i=0 ; i<(sizeof(gpio_set)/sizeof(struct gpio_set_tbl_type)) ; i++){
			if(gpio_set[i].direction == 1){
				rc = gpio_direction_output(gpio_set[i].gpio, gpio_set[i].val);
				pr_debug("%s: set output gpio=%d val=%d \n",__func__, gpio_set[i].gpio,gpio_set[i].val);
			}else{
				rc = gpio_direction_input(gpio_set[i].gpio);
				pr_debug("%s: set input gpio=%d\n",__func__, gpio_set[i].gpio);
			}
			if (rc < 0) {
				pr_err("%s: Fail set output/input gpio=%d\n",__func__, gpio_set[i].gpio);
				break;
			}
		}
		if(rc < 0){
			for(i=0 ; i<(sizeof(gpio_req)/sizeof(struct gpio_request_tbl_type)) ; i++)
				gpio_free(gpio_req[i].gpio);
		}
	}else{
		for(i=0 ; i<(sizeof(gpio_req)/sizeof(struct gpio_request_tbl_type)) ; i++)
			gpio_free(gpio_req[i].gpio);
	}

	dev_info(dev, "%s: INIT WACOM GPIO rc=%d on=%d\n", __func__, rc, on);
	return rc;
}

static void wacom_parse_dt(struct wacom_i2c *wac_i2c)
{
	u32 value;
	pr_debug("%s is called. \n",__func__);

	if (!wac_i2c->client->dev.of_node)
		return;

	value = of_get_named_gpio(wac_i2c->client->dev.of_node, "wacom,fwe-gpio", 0);
	wac_i2c->gpio_fwe = value;

	value = of_get_named_gpio(wac_i2c->client->dev.of_node, "wacom,rst-gpio", 0);
	wac_i2c->gpio_reset = value;

	value = of_get_named_gpio(wac_i2c->client->dev.of_node, "wacom,pdctb-gpio", 0);
	wac_i2c->gpio_pdctb = value;

	value = of_get_named_gpio(wac_i2c->client->dev.of_node, "wacom,irq-gpio", 0);
	wac_i2c->gpio_irq = value;

	value = of_get_named_gpio(wac_i2c->client->dev.of_node, "wacom,ldo-gpio", 0);
	wac_i2c->gpio_ldo = value;

    pr_info("%s: fwe:%d rst:%d pdctb:%d irq:%d ldo:%d", __func__, wac_i2c->gpio_fwe,
             wac_i2c->gpio_reset, wac_i2c->gpio_pdctb, wac_i2c->gpio_irq, wac_i2c->gpio_ldo);
	return;
}

void wacom_i2c_enable_irq(struct wacom_i2c *wac_i2c)
{
	struct i2c_client *client = wac_i2c->client;
	pr_debug("%s is called.\n", __func__);

	if (wac_i2c->is_enable != 0) {
		pr_err("%s not disable\n",__func__);
		WARN_ON(1);
		goto done;
	}

	enable_irq(client->irq);
	wac_i2c->is_enable = 1;
done:
	pr_debug("%s is completed.\n", __func__);
}

void wacom_i2c_disable_irq(struct wacom_i2c *wac_i2c)
{
	struct i2c_client *client = wac_i2c->client;
	pr_debug("%s is called.\n", __func__);

	if (wac_i2c->is_enable != 1) {
		pr_err("%s not enable\n",__func__);
		WARN_ON(1);
		goto done;
	}

	disable_irq(client->irq);
	wac_i2c->is_enable = 0;
done:
	pr_debug("%s is completed.\n", __func__);
}

void wacom_i2c_disable_irq_nosync(struct wacom_i2c *wac_i2c)
{
	struct i2c_client *client = wac_i2c->client;
	pr_debug("%s is called.\n", __func__);

	if (wac_i2c->is_enable != 1) {
		pr_err("%s not enable\n",__func__);
		WARN_ON(1);
		goto done;
	}

	disable_irq_nosync(client->irq);
	wac_i2c->is_enable = 0;
done:
	pr_debug("%s is completed.\n", __func__);
}

void wacom_i2c_down_check_and_release(void)
{
	ktime_t now_time;
	s64 elapsed_time_ms;

	pr_debug("%s is called. \n",__func__);

	if(!digitizer_data){
		pr_err("%s: wac_i2c is null\n", __func__);
		return;
	}

	if( digitizer_data->prox ){
		now_time = ktime_get();
		elapsed_time_ms = ktime_to_ms(ktime_sub(now_time, wac_report_time));
		if ( elapsed_time_ms > 1000 ){
			pr_info("%s Time over. [%lld - %lld] Recovery start \n",
				__func__, ktime_to_ms(wac_report_time), ktime_to_ms(now_time));
			wacom_i2c_release_notification(digitizer_data);
		}
	}
	return;
}

static int wacom_dt_irq(struct kc_dt_data *dt, unsigned int flg)
{
	struct wacom_i2c *wac_i2c = dt->vdata;

	pr_info("%s is called. flg =%d \n",__func__,flg);

    if (flg == 1) {
        wacom_i2c_enable_irq(wac_i2c);
    } else {
        wacom_i2c_disable_irq(wac_i2c);
    }
    return 0;
}

static int wacom_dt_suspend(struct kc_dt_data *dt)
{
#ifdef CONFIG_PM_SLEEP
	struct wacom_i2c *wac_i2c = dt->vdata;
	int err = 0;

	pr_info("%s is called.\n",__func__);

	err = wacom_i2c_suspend(&wac_i2c->client->dev);

	return err;
#else
	return 0;
#endif
}

static int wacom_dt_resume(struct kc_dt_data *dt)
{
#ifdef CONFIG_PM_SLEEP
	struct wacom_i2c *wac_i2c = dt->vdata;
	int err = 0;

	pr_info("%s is called.\n",__func__);

	err = wacom_i2c_resume(&wac_i2c->client->dev);

	return err;
#else
	return 0;
#endif
}

static int wacom_dt_hw_reset(struct kc_dt_data *dt)
{
	struct wacom_i2c *wac_i2c = dt->vdata;

	pr_info("%s is called.\n",__func__);

	wacom_i2c_hw_err_reset(wac_i2c->client);

	return 0;
}

static int wacom_get_current_fw_version(struct kc_dt_data *dt, int *fw_version)
{
	struct wacom_i2c *wac_i2c = dt->vdata;
	int ret = 0;

	pr_info("%s is called.\n",__func__);

	mutex_lock(&wac_i2c->seq_lock);
	ret = wacom_get_fw_version(wac_i2c->client, fw_version);
	mutex_unlock(&wac_i2c->seq_lock);

	return ret;
}

static int wacom_get_fw_version(struct i2c_client *client, int *fw_version)
{
	int ret = 0;
	u8 cmd1[] = { WACOM_HID_REG0, WACOM_HID_REG1};
	u8 data[WACOM_HID_DATA_SIZE];
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(cmd1),
			.buf = cmd1,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = sizeof(data),
			.buf = data,
		},
	};

	ret = wacom_i2c_transfer(client, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*fw_version = get_unaligned_le16(&data[WACOM_HID_FW_VER_OFFSET]);

	pr_info("%s: fw version is 0x%04x\n",__func__,  *fw_version);

	return 0;
}

static long wacom_dt_ioctl(struct kc_dt_data *dt, unsigned int cmd,
						unsigned long arg)
{
	int fw_version;
	long err = 0;

	pr_debug("%s is called.\n",__func__);

	switch (cmd) {
	case IOCTL_DIGI_FW_VER_GET:
		err = copy_from_user(&fw_version, (void __user *)arg, sizeof(fw_version));
		if (err) {
			pr_err("%s: copy_from_user error\n", __func__);
			goto done;
		}
		dt->soft_fw_ver = fw_version;
		break;

	case IOCTL_DIGI_SET_PS_STAT:
		pr_debug("%s: IOCTL_DIGI_SET_PS_STAT\n", __func__);
		/* no process */
		break;

	default:
		pr_err("%s() (L:%d) default \n", __FUNCTION__ , __LINE__);
		break;
	}
done:
	return err;
}

static const struct kc_dt_operations wacom_operations = {
	.get_fw_version		= wacom_get_current_fw_version,
	.hw_reset			= wacom_dt_hw_reset,
	.power_off			= wacom_dt_suspend,
	.power_on			= wacom_dt_resume,
	.irq				= wacom_dt_irq,
	.ioctl				= wacom_dt_ioctl,
};

static int wacom_i2c_input_register(struct wacom_i2c *wac_i2c)
{
	int temp_coord;
	int error = -1;
	struct input_dev *input;
	u8 retry = WACOM_DEVICE_ERROR_RETRY_CNT;
	struct i2c_client *client = wac_i2c->client;

	pr_debug("%s is called\n", __func__);

	if(wac_i2c->input != NULL){
		input_unregister_device(wac_i2c->input);
		wac_i2c->input = NULL;
	}

	input = input_allocate_device();
	if (!input) {
		error = -ENOMEM;
		goto done;
	}

	memset(wac_i2c->features, 0, sizeof(struct wacom_features));

	do {
		error = wacom_query_device(client, wac_i2c->features);
		if (error >= 0)
			break;
		pr_err("%s: wacom_query_device fail error=%d\n", __func__, error);
		if (retry == 0){
			input_free_device(input);
			goto done;
		}
		wacom_i2c_power_off(wac_i2c);
		wacom_i2c_power_on(wac_i2c);
		wacom_hw_reset(wac_i2c);
		usleep_range(WACOM_I2C_RETRY_WAIT,WACOM_I2C_RETRY_WAIT);
	} while (retry-- > 0);

	input->name = "Wacom I2C Digitizer";
	input->id.bustype = BUS_I2C;
	input->id.vendor = 0x56a;
	input->id.version = wac_i2c->features->fw_version;
	input->dev.parent = &client->dev;
	input->open = wacom_i2c_open;
	input->close = wacom_i2c_close;

	input->evbit[0] |= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

	__set_bit(BTN_TOOL_PEN, input->keybit);
	__set_bit(BTN_TOOL_RUBBER, input->keybit);
	__set_bit(BTN_TOUCH, input->keybit);
	__set_bit(INPUT_PROP_DIRECT, input->propbit);

	/*Setting maximum coordinate values  */
	/*eliminating 1mm offset on each side*/

	temp_coord = wac_i2c->features->x_max;
	wac_i2c->features->x_max = wac_i2c->features->y_max;
	wac_i2c->features->y_max = temp_coord;
	pr_info("%s: feature_xmax: %d feature_ymax: %d \n", __func__, wac_i2c->features->x_max, wac_i2c->features->y_max);
	pr_info("%s: feature_pressure: %d \n", __func__, wac_i2c->features->pressure_max);
	pr_info("%s: feature_height: %d \n", __func__, wac_i2c->features->height_max);

	input_set_abs_params(input, ABS_X, 0, wac_i2c->features->x_max, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, wac_i2c->features->y_max, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE,
			     0, wac_i2c->features->pressure_max, 0, 0);
	input_set_abs_params(input, ABS_DISTANCE,
			     0, wac_i2c->features->height_max, 0, 0);

#ifdef CALIBRATION
	wac_i2c->features->calib_data.originX = wac_i2c->features->calib_data.originY = 0;
	wac_i2c->features->calib_data.extentX = wac_i2c->features->x_max;
	wac_i2c->features->calib_data.extentY = wac_i2c->features->y_max;
	wac_i2c->features->node_state = STATE_NORMAL;
	wac_i2c->features->bCalibrationSet = false;
#endif

	input_set_drvdata(input, wac_i2c);
	error = input_register_device(input);
	if (!error) {
		wac_i2c->input = input;
	} else {
		input_free_device(input);
		dev_err(&client->dev,
			"Failed to register input device, error: %d\n", error);
	}
done:
	pr_debug("%s is ended ret = %d \n", __func__, error);
	return error;
}

static int wacom_i2c_probe(struct i2c_client *client,
				     const struct i2c_device_id *id)
{
	struct wacom_i2c *wac_i2c;
	int error = -1;
	int irq_ret = -1;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		return -EIO;
	}

	wac_i2c = kzalloc(sizeof(*wac_i2c), GFP_KERNEL);
	if (!wac_i2c) {
		error = -ENOMEM;
		goto err_no_mem;
	}

	digitizer_data = wac_i2c;
	wac_report_time = ktime_get();

	wac_i2c->input = NULL;

	wac_i2c->client = client;

	wac_i2c->is_enable = -1;
	wac_i2c->is_suspend = true;

	wac_i2c->features = kzalloc(sizeof(struct wacom_features), GFP_KERNEL);
	if (!wac_i2c->features) {
		printk("failed to preserve memory \n");
		goto err_free_mem;
	}

	wacom_parse_dt(wac_i2c);

	mutex_init(&wac_i2c->seq_lock);
	init_completion(&wac_i2c->fwdl_completion);

	/* Get pinctrl */
	error = wacom_pinctrl_init(wac_i2c);
	if (error){
		pr_err("%s pinctrl Fail.\n", __func__);
		goto err_mutex_free;
	}

	error = wacom_pinctrl_select(wac_i2c, true);
	if (error){
		pr_err("%s pin select Fail.\n", __func__);
		goto err_mutex_free;
	}

	error = wacom_gpio_init(wac_i2c, 1);
	if (error < 0){
		dev_err(&client->dev, "%s: HW Init fail r=%d\n", __func__, error);
		goto err_mutex_free;
	}

	/* hardware reset */
    wacom_hw_reset(wac_i2c);

	i2c_set_clientdata(client, wac_i2c);

	INIT_WORK( &wac_i2c->hw_reset_work, wacom_i2c_hw_err_reset_work );

	mutex_lock(&wac_i2c->seq_lock);

	error = wacom_i2c_input_register(wac_i2c);
	if(!error){
		irq_ret = request_threaded_irq(client->irq, NULL, wacom_i2c_irq,
					     IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					     "wacom_i2c", wac_i2c);
		if (irq_ret) {
			error = irq_ret;
			dev_err(&client->dev,
				"Failed to enable IRQ, error: %d\n", irq_ret);
			goto  err_mutex_lock;
		}
		wac_i2c->is_enable = 1;

		/* Disable the IRQ, we'll enable it in wac_i2c_open() */
		wacom_i2c_disable_irq_nosync(wac_i2c);
	}
#ifdef CONFIG_PM_SLEEP
#if defined(CONFIG_FB)
	wac_i2c->fb_notif.notifier_call = fb_notifier_callback;

	error = fb_register_client(&wac_i2c->fb_notif);

	if (error) {
		dev_err(&client->dev, "Unable to register fb_notifier: %d\n",
			error);
		goto err_free_input_dev;
	}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	wac_i2c->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	wac_i2c->early_suspend.suspend = wacom_i2c_early_suspend;
	wac_i2c->early_suspend.resume  = wacom_i2c_late_resume;
	register_early_suspend(&wac_i2c->early_suspend);
	printk("%s: early_suspend registered \n", __func__);
#endif //CONFIG_HAS_EARLYSUSPEND
#endif //CONFIG_PM_SLEEP

	wac_i2c->resume_wq = alloc_workqueue("wac_resume_wq", WQ_MEM_RECLAIM, 1);
	if (!wac_i2c->resume_wq){
		dev_err(&client->dev, "%s: Fail to allocate workqueue!\n", __func__);
		error = -ENOMEM;
		goto err_unreg_suspend;
	}
	INIT_WORK(&wac_i2c->resumework, wacom_resume_work);

#ifdef WACOM_SYSFS
	error = register_sysfs(wac_i2c);
	if (error == -ERR_REGISTER_SYSFS) {
		dev_err(&client->dev, "%s: register_sysfs fail err=%d\n", __func__, error);
		goto err_free_wq;
	}
#endif

	wac_i2c->kc_dt.android_notify = 1;
	wac_i2c->kc_dt.power_lock = 0;
	wac_i2c->kc_dt.dev = &client->dev;
	wac_i2c->kc_dt.tops = &wacom_operations;
	wac_i2c->kc_dt.vdata = wac_i2c;
	error = kc_dt_probe(&wac_i2c->kc_dt);
	if (error < 0) {
		pr_err("%s: Failed to create kc cdev/sysfs\n", __func__);
		goto err_probe;
	}

	mutex_unlock(&wac_i2c->seq_lock);
	return 0;

 err_probe:

#ifdef WACOM_SYSFS
	remove_sysfs(wac_i2c);
#endif

err_free_wq:
	destroy_workqueue(wac_i2c->resume_wq);

 err_unreg_suspend:
#ifdef CONFIG_PM_SLEEP
#if defined(CONFIG_FB)
	fb_unregister_client(&wac_i2c->fb_notif);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&wac_i2c->early_suspend);
	printk("%s: Unregister early_suspend done\n", __func__);
#endif //CONFIG_HAS_EARLYSUSPEND
err_free_input_dev:
	input_unregister_device(wac_i2c->input);
	wac_i2c->input = NULL;
#endif //CONFIG_PM_SLEEP
	wac_i2c->is_enable = -1;
	wac_i2c->is_suspend = true;

 err_mutex_lock:
	mutex_unlock(&wac_i2c->seq_lock);
	if (irq_ret == 0)
		free_irq(client->irq, wac_i2c);

	wacom_gpio_init(wac_i2c, 0);

 err_mutex_free:
	mutex_destroy(&wac_i2c->seq_lock);

	kfree(wac_i2c->features);
	wac_i2c->features = NULL;
err_free_mem:
	kfree(wac_i2c);

 err_no_mem:
	return error;
}

int wacom_i2c_remove(struct i2c_client *client)
{
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);

	cancel_work_sync(&wac_i2c->hw_reset_work);
	cancel_work_sync(&wac_i2c->resumework);
#ifdef WACOM_SYSFS
	remove_sysfs(wac_i2c);
#endif

	kc_dt_remove(&wac_i2c->kc_dt);
#ifdef CONFIG_PM_SLEEP
#if defined(CONFIG_FB)
	fb_unregister_client(&wac_i2c->fb_notif);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&wac_i2c->early_suspend);
	printk("%s: Unregister early_suspend done\n", __func__);
#endif //CONFIG_HAS_EARLYSUSPEND
#endif //CONFIG_PM_SLEEP
	if(wac_i2c->is_enable != -1)
		free_irq(client->irq, wac_i2c);

	if(wac_i2c->input != NULL){
		input_unregister_device(wac_i2c->input);
		wac_i2c->input = NULL;
	}
	wacom_gpio_init(wac_i2c, 0);
	kfree(wac_i2c->features);
	kfree(wac_i2c);
	digitizer_data = NULL;
	return 0;
}

static void wacom_i2c_shutdown(struct i2c_client *client)
{
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);

	wacom_i2c_power_off(wac_i2c);
}

static const struct i2c_device_id wacom_i2c_id[] = {
	{ "WAC_I2C_EMR", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, wacom_i2c_id);

static struct of_device_id wac_match_table[] = {
	{ .compatible = "wacom,emr",},
	{ },
};

static struct i2c_driver wacom_i2c_driver = {
	.driver	= {
		.name	= "wacom_i2c",
		.owner	= THIS_MODULE,
		.of_match_table = wac_match_table,
	},

	.probe		= wacom_i2c_probe,
	.remove		= wacom_i2c_remove,
	.shutdown	= wacom_i2c_shutdown,
	.id_table	= wacom_i2c_id,
};
static int __init wacom_i2c_init(void)
{
	return i2c_add_driver(&wacom_i2c_driver);
}

static void __exit wacom_i2c_exit(void)
{
	i2c_del_driver(&wacom_i2c_driver);
}

module_init(wacom_i2c_init);
module_exit(wacom_i2c_exit);

MODULE_AUTHOR("Tatsunosuke Tobita <tobita.tatsunosuke@wacom.co.jp>");
MODULE_DESCRIPTION("WACOM EMR I2C Driver");
MODULE_LICENSE("GPL");
