/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 */
/*
 * Wacom Penabled Driver for I2C
 *
 * Copyright (c) 2011-2014 Tatsunosuke Tobita, Wacom.
 * <tobita.tatsunosuke@wacom.co.jp>
 *
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software
 * Foundation; either version of 2 of the License,
 * or (at your option) any later version.
 */
#include "wacom.h"

static void parse_report_desc(struct wacom_features *features, u8 *report_desc, int report_desc_size)
{
	int i, len;
	int usage = 0;
	bool vendor_defined = false;

	for (i = 0; i < report_desc_size; i++) {
		switch (report_desc[i]) {
		case USAGE_PAGE:
			switch (report_desc[i + 1]) {
			case USAGE_PAGE_DIGITIZERS:
				usage = USAGE_PAGE_DIGITIZERS;
				if (report_desc[i + 3] == USAGE_PEN) {
					i += 4;
				}
				else
					i++;
				break;

			case USAGE_PAGE_DESKTOP:
				usage = USAGE_PAGE_DESKTOP;
				if (report_desc[i + 3] == USAGE_MOUSE) {
					len = i + 3;

					while(report_desc[len] != ENDCOLLECTION)
						len++;
					i = len;
				}
				else
					i++;
				break;
			}
			break;

		case USAGE_PAGE_EXTENDED:
			if (report_desc[i + 1] == 0x00 && 
			    report_desc[i + 2] == USAGE_PAGE_VENDOR) {
				vendor_defined = true;
				i += 2;
			}
			break;

		case USAGE:
			switch (report_desc[i + 1]) {
			case USAGE_X:
				if (usage == USAGE_PAGE_DESKTOP) {		
					features->x_max = get_unaligned_le16(&report_desc[i + 3]);
					printk("USAGE_X : %d \n", features->x_max);
					i += 4;
				} else if (usage == USAGE_PAGE_DIGITIZERS) {
					features->pressure_max = get_unaligned_le16(&report_desc[i + 3]);
					printk("PRESSURE : %d \n", features->pressure_max);	
					i += 4;
				} else
					i++;
				break;
				
			case USAGE_Y:
				if (usage == USAGE_PAGE_DESKTOP) {
					features->y_max = get_unaligned_le16(&report_desc[i + 3]);
					printk("USAGE_Y : %d \n", features->y_max);
					i += 4;
				} else
					i++;
				break;

			case USAGE_VENDOR:
				if (vendor_defined) {
					features->height_max = get_unaligned_le16(&report_desc[i + 5]);
					printk("USAGE_VENDOR : %d \n", features->height_max);
					i += 6;
				}
				vendor_defined = false;
				break;
			}
			break;
		}		
	}
}

static int retrieve_report_desc(struct i2c_client *client, struct wacom_features *features,
			 HID_DESC hid_desc)
{
	struct i2c_msg msgs_touch[2];
	int ret = -1;
	int report_desc_size = hid_desc.wReportDescLength;
	u8 cmd[] = {hid_desc.wReportDescRegister, 0x00};
	u8 *report_desc = NULL;

	report_desc = kzalloc(sizeof(u8) * report_desc_size, GFP_KERNEL);
	if (!report_desc) {
		dev_err(&client->dev, "No memory left for this device \n");
		return -ENOMEM;
	}

	msgs_touch[0].addr = client->addr;
	msgs_touch[0].flags = 0;
	msgs_touch[0].len = sizeof(cmd);
	msgs_touch[0].buf = cmd;

	msgs_touch[1].addr = client->addr;
	msgs_touch[1].flags = I2C_M_RD;
	msgs_touch[1].len = report_desc_size;
	msgs_touch[1].buf = report_desc;
		
	ret = wacom_i2c_transfer(client, msgs_touch, ARRAY_SIZE(msgs_touch));
	if (ret < 0) {
		dev_err(&client->dev, "%s obtaining report descriptor failed \n", __func__);
		goto err;
	}
	if (ret != ARRAY_SIZE(msgs_touch)) {
		ret = -EIO;
		goto err;
	}

	parse_report_desc(features, report_desc, report_desc_size);
	ret = 0;

 err:
	kfree(report_desc);
	report_desc = NULL;
	return ret;
}

bool wacom_i2c_set_report(u8 feature_type, u8 *feature, size_t size, 
			  u8 report_id, u16 cmdreg, u16 datareg)
{
	if (size < GFEATURE_SIZE) {
		printk("%s: the size must be larger than 5 \n", __func__);
		return false;
	}

	feature[0] = (u8)(cmdreg & 0x00ff);
	feature[1] = (u8)((cmdreg & 0xff00) >> 8);
	feature[2] = (RTYPE_FEATURE << 4) | report_id;
	feature[3] = (feature_type == GET_FEATURE) ? GET_FEATURE : SET_FEATURE;
	feature[4] = (u8)(datareg & 0x00ff);
	feature[5] = (u8)((datareg & 0xff00) >> 8);

	if (report_id > 15) {
		feature[2] = (RTYPE_FEATURE << 4) | 0x0f;
		feature[3] = (feature_type == GET_FEATURE) ? GET_FEATURE : SET_FEATURE;
		feature[4] = report_id;
		feature[5] = (u8)(datareg & 0x00ff);
		feature[6] = (u8)((datareg & 0xff00) >> 8);
	}

	return true;
}

bool wacom_i2c_set_feature(struct i2c_client *client, u8 report_id, unsigned int buf_size, u8 *data, 
			   u16 cmdreg, u16 datareg)
{
	int i, ret = -1;
	int total = SFEATURE_SIZE + buf_size;
	int array_pos1 = 6;
	int array_pos2 = 7;
	int sfeature_size = SFEATURE_SIZE;
	u8 *sFeature = NULL;
	bool bRet = false;

	if (report_id > 15) {
		total++;
		array_pos1++;
		array_pos2++;
		sfeature_size++;
	}

	sFeature = kzalloc(total, GFP_KERNEL);
	if (!sFeature) {
		dev_err(&client->dev, "%s cannot preserve memory \n", __func__);
		goto out;
	}
	memset(sFeature, 0, sizeof(u8) * total);

	bRet = wacom_i2c_set_report(SET_FEATURE, sFeature, 
				    total, report_id, cmdreg, datareg);
	if (!bRet) {
		dev_err(&client->dev, "%s: Setting features failed \n", __func__);
			goto err;
	}

	/*Add extra 2 bytes for length fields*/
	if ( (buf_size +2) > 255) {
		sFeature[array_pos1] = (u8)((buf_size + 2) & 0x00ff);
		sFeature[array_pos2] = (u8)(( (buf_size + 2) & 0xff00) >> 8);
	} else {
		sFeature[array_pos1] = (u8)(buf_size + 2);
		sFeature[array_pos2] = (u8)(0x00);
	}

	for (i = 0; i < buf_size; i++)
		sFeature[i + sfeature_size] = *(data + i);


	ret = wacom_i2c_send(client, sFeature, total);
	if (ret != total) {
		dev_err(&client->dev, "Sending Set_Feature failed sent bytes: %d \n", ret);
		goto err;
	}

	bRet = true;
 err:
	kfree(sFeature);
	sFeature = NULL;

 out:
	return bRet;
}

bool wacom_i2c_get_feature(struct i2c_client *client, u8 report_id, unsigned int buf_size, u8 *data, 
			   u16 cmdreg, u16 datareg)
{
	int ret = -1;
	char gFeature_size = 6;
	u8 *recv = NULL;
	bool bRet = false;
	u8 gFeature[7] = {0};
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = gFeature_size,
			.buf = NULL,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = (buf_size + 2),
			.buf = NULL,
		},
	};

	if (report_id > 15) {
		gFeature_size++;
		msgs[0].len++;
	}

	/*"+ 2", adding 2 more spaces for organizeing again later in the passed data, "data"*/
	recv = kzalloc((buf_size + 2), GFP_KERNEL);
	if (!recv) {
		dev_err(&client->dev, "%s cannot preserve memory \n", __func__);
		goto out;
	}
	memset(recv, 0, (buf_size + 2)); /*Append 2 bytes for length low and high of the byte*/

	bRet = wacom_i2c_set_report(GET_FEATURE, gFeature, 
				    gFeature_size, report_id, cmdreg, datareg);
	if (!bRet) {
		dev_err(&client->dev, "%s: Setting features failed \n", __func__);
			goto err;
	}

	msgs[0].buf = gFeature;
	msgs[1].buf = recv;

	ret = wacom_i2c_transfer(client, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	/*Coppy data pointer, subtracting the first two bytes of the length*/
	memcpy(data, (recv + 2), buf_size);

	bRet = true;
 err:
	kfree(recv);
	recv = NULL;

 out:
	return bRet;
}

int wacom_query_device(struct i2c_client *client, struct wacom_features *features)
{
	HID_DESC hid_descriptor;
	int ret = -1;
	u16 cmd_reg;
	u16 data_reg;
	u8 retry = WACOM_DEVICE_ERROR_RETRY_CNT;
	u8 cmd[] = {HID_DESC_REGISTER, 0x00};
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(cmd),
			.buf = cmd,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = sizeof(HID_DESC),
			.buf = (u8 *)(&hid_descriptor),
		},
	};
	
	do{
		ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
		if (ret >= 0){
//			usleep(55);
			usleep_range(55, 55);
			break;
		}
		pr_err("%s: i2c_transfer fail ret=%d\n", __func__, ret);
		if (retry == 0)
			break;
		usleep_range(WACOM_I2C_RETRY_WAIT,WACOM_I2C_RETRY_WAIT);
	} while (retry-- > 0);
	if (unlikely(ret < 0)) {
		dev_err(&client->dev, "%s obtaining query failed: %d \n", __func__, ret);
		goto err;
	}
	if (ret != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s input/output error occured; \n returned: %dbyte(s) \n", __func__, ret);
		ret = -EIO;
		goto err;
	}

	cmd_reg = hid_descriptor.wCommandRegister;
	data_reg = hid_descriptor.wDataRegister;
	features->input_size = hid_descriptor.wMaxInputLength;
	features->vendorId = hid_descriptor.wVendorID;
	features->productId = hid_descriptor.wProductID;
	features->fw_version = hid_descriptor.wVersion;
	memcpy(&features->hid_desc, &hid_descriptor, sizeof(HID_DESC));

	pr_info("%s: cmd_reg=%04x, data_reg=%04x, input_size=%04x\n", __func__,
				cmd_reg, data_reg, features->input_size);
	pr_info("%s: vendorId=%04x, productId=%04x, fw_version=%04x\n", __func__,
				features->vendorId, features->productId, features->fw_version);

	dev_dbg(&client->dev, "Retrieving report descriptor \n");
	ret = retrieve_report_desc(client, features, hid_descriptor);
	if (ret < 0)
		goto err;

	dev_dbg(&client->dev, "addr: %x x_max:%d, y_max:%d\n", client->addr,
	       features->x_max, features->y_max);
	dev_dbg(&client->dev, "pressure_max:%d, fw_version:%x \n",
	       features->pressure_max, features->fw_version);
	
	ret = 0;
 err:
	return ret;
}
