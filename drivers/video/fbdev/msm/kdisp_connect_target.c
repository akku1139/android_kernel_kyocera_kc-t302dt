/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
*/
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2019 KYOCERA Corporation
*/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <soc/qcom/socinfo.h>

#include "mdss.h"
#include "mdss_dsi.h"
#include "kdisp_com.h"

#define PANEL_FOUND_GPIO_VALUE		0
#define PANEL_CHECK_RETRY_NUM		5
extern char *saved_command_line;

enum {
	PF_SSTYPE_HINOKI00 = 0x0,
	PF_SSTYPE_HINOKI01 = 0x1,
	PF_SSTYPE_HINOKI02 = 0x2,
	PF_SSTYPE_INVALID
};

static int panel_detect_status			= PANEL_NOT_TEST;
static int panel_detect_active 			= PANEL_NOT_TEST;
static int panel_detect_status_update	= 0; /* 1: update */
static int panel_detect = PANEL_NOT_FOUND;

module_param(panel_detect, int, S_IRUGO );
MODULE_PARM_DESC(panel_detect, "Panel Detect");

static bool get_panel_online_status(void)
{
	char *t = NULL;
	t = strstr(saved_command_line, "online");
	if(!t)
	{
		pr_err("%s, panel is offline\n", __func__);
		return false;
	}
	else
		return true;
}

static int __init set_panel_detect1(char *buf)
{
	panel_detect = PANEL_FOUND1;
	pr_info("%s: early_param_set! panel_detect=%d\n", __func__, panel_detect);
	return 0;
}
early_param("panel_detect1", set_panel_detect1);

static int __init set_panel_detect2(char *buf)
{
	panel_detect = PANEL_FOUND2;
	pr_info("%s: early_param_set! panel_detect=%d\n", __func__, panel_detect);
	return 0;
}
early_param("panel_detect2", set_panel_detect2);

int get_panel_detect(void)
{
	return panel_detect;
}

static int kdisp_connect_pinctrl_init(struct platform_device *pdev)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
	struct kc_ctrl_info *kc_ctrl_data;

	ctrl_pdata = platform_get_drvdata(pdev);
	kc_ctrl_data = &ctrl_pdata->kc_ctrl_data;

	kc_ctrl_data->pin_res.pinctrl = ctrl_pdata->pin_res.pinctrl;

	kc_ctrl_data->pin_res.gpio_state_det_active
		= pinctrl_lookup_state(kc_ctrl_data->pin_res.pinctrl,
				KDISP_PINCTRL_STATE_DET_ACTIVE);
	if (IS_ERR_OR_NULL(kc_ctrl_data->pin_res.gpio_state_det_active))
		pr_warn("%s: can not get det_active pinstate\n", __func__);

	kc_ctrl_data->pin_res.gpio_state_det_suspend
		= pinctrl_lookup_state(kc_ctrl_data->pin_res.pinctrl,
				KDISP_PINCTRL_STATE_DET_SUSPEND);
	if (IS_ERR_OR_NULL(kc_ctrl_data->pin_res.gpio_state_det_suspend))
		pr_warn("%s: can not get det_suspend pinstate\n", __func__);

	return 0;

}

static int kdisp_connect_pinctrl_set_state(
	struct kc_ctrl_info *kctrl_data,
	bool active)
{
	struct pinctrl_state *pin_state;
	int rc = -EFAULT;

	if (IS_ERR_OR_NULL(kctrl_data->pin_res.pinctrl))
		return PTR_ERR(kctrl_data->pin_res.pinctrl);

	pin_state = active ? kctrl_data->pin_res.gpio_state_det_active
				: kctrl_data->pin_res.gpio_state_det_suspend;

	if (!IS_ERR_OR_NULL(pin_state)) {
		rc = pinctrl_select_state(kctrl_data->pin_res.pinctrl,
				pin_state);
		if (rc)
			pr_err("%s: can not set %s pins\n", __func__,
					active ? KDISP_PINCTRL_STATE_DET_ACTIVE
					: KDISP_PINCTRL_STATE_DET_SUSPEND);
	} else {
		pr_err("%s: invalid '%s' pinstate\n", __func__,
					active ? KDISP_PINCTRL_STATE_DET_ACTIVE
					: KDISP_PINCTRL_STATE_DET_SUSPEND);
	}
	msleep(1);
	return rc;
}

static int kdisp_connect_parse_gpio_params(struct platform_device *pdev,
	struct kc_ctrl_info *kc_ctrl_data)
{

	kc_ctrl_data->lcd_id1_gpio = of_get_named_gpio(pdev->dev.of_node,
			"kc,lcd-id1-gpio", 0);

	kc_ctrl_data->lcd_id2_gpio = of_get_named_gpio(pdev->dev.of_node,
			"kc,lcd-id2-gpio", 0);

	if (!gpio_is_valid(kc_ctrl_data->lcd_id1_gpio))
		pr_err("%s:%d, lcd_id1 gpio not specified\n",
			 							__func__, __LINE__);

	if (!gpio_is_valid(kc_ctrl_data->lcd_id2_gpio))
		pr_err("%s:%d, lcd_id2 gpio not specified\n",
			 							__func__, __LINE__);

	return 0;
}

static int kdisp_connect_check_panel_detect(struct kc_ctrl_info *kctrl_data)
{
	int id1[1], id2[1];
	int i;
	int pf_sstype;
	pf_sstype = socinfo_get_platform_subtype();

	id1[0] = PANEL_FOUND_GPIO_VALUE;
	id1[1] = PANEL_FOUND_GPIO_VALUE;
	id2[0] = PANEL_FOUND_GPIO_VALUE;
	id2[1] = PANEL_FOUND_GPIO_VALUE;

	if (kctrl_data == NULL) {
		pr_err("%s: param error\n", __func__);
		panel_detect_status = PANEL_FOUND2;
		return panel_detect_status;
	}

	if (!kctrl_data->panel_det_en) {
		pr_err("%s: panel detect disable\n", __func__);
		panel_detect_status = PANEL_FOUND2;
	} else {
		if (panel_detect_status == PANEL_NOT_TEST) {
			if (panel_detect == PANEL_FOUND1) {
				panel_detect_status = PANEL_FOUND1;
				pr_notice("%s: already panel1 found\n", __func__);
			} else if (panel_detect == PANEL_FOUND2) {
				panel_detect_status = PANEL_FOUND2;
				pr_notice("%s: already panel2 found\n", __func__);
			} else {
				if(PF_SSTYPE_HINOKI01 == pf_sstype)
				{
					if(get_panel_online_status())
						panel_detect_status = PANEL_FOUND1;
					else
						panel_detect_status = PANEL_NOT_FOUND;
				}
				else
				{
					for (i=0; i<PANEL_CHECK_RETRY_NUM; i++) {
						kdisp_connect_pinctrl_set_state(kctrl_data, 1);
						msleep(1);
						id1[0] = gpio_get_value(kctrl_data->lcd_id1_gpio);
						id2[0] = gpio_get_value(kctrl_data->lcd_id2_gpio);

						kdisp_connect_pinctrl_set_state(kctrl_data, 0);
						msleep(1);
						id1[1] = gpio_get_value(kctrl_data->lcd_id1_gpio);
						id2[1] = gpio_get_value(kctrl_data->lcd_id2_gpio);

						if (id1[0] == PANEL_FOUND_GPIO_VALUE && id2[0] == PANEL_FOUND_GPIO_VALUE && id1[1] == PANEL_FOUND_GPIO_VALUE && id2[1] == PANEL_FOUND_GPIO_VALUE) {
							panel_detect_status = PANEL_FOUND1;
							pr_notice("%s: panel1 found\n", __func__);
							break;
						} else if (id1[0] != PANEL_FOUND_GPIO_VALUE && id2[0] != PANEL_FOUND_GPIO_VALUE && id1[1] != PANEL_FOUND_GPIO_VALUE && id2[1] != PANEL_FOUND_GPIO_VALUE) {
							panel_detect_status = PANEL_FOUND2;
							pr_notice("%s: panel2 found\n", __func__);
							kdisp_connect_pinctrl_set_state(kctrl_data, 1);
							break;
						}
						pr_notice("%s: panel detect error. retry=%d\n", __func__, i);
					}

					if (panel_detect_status == PANEL_NOT_TEST) {
						panel_detect_status = PANEL_NOT_FOUND;
						pr_notice("%s: panel not found\n", __func__);
					}
				}
			}
		}
	}

	return panel_detect_status;
}

static void kdisp_connect_notify_to_light(void)
{
#ifdef ENABLE_LCD_DETECTION
	if ((panel_detect_status == PANEL_FOUND1) || (panel_detect_status == PANEL_FOUND2)) {
		light_led_disp_set_panel(LIGHT_MAIN_WLED_LCD_EN, 0);
		pr_notice("%s: notify found to led\n", __func__);
	} else {
		light_led_disp_set_panel(LIGHT_MAIN_WLED_LCD_DIS, 0);
		pr_err("%s: notify not found to led\n", __func__);
	}
#endif /* ENABLE_LCD_DETECTION */
}

int kdisp_connect_get_panel_detect(void)
{
	return panel_detect_status;
}

void kdisp_connect_update_panel_detect_status(void)
{
	if (panel_detect_status_update == 1) {
		if (panel_detect_active == PANEL_NOT_FOUND) {
			panel_detect_status = PANEL_NOT_FOUND;
		} else if (panel_detect_active == PANEL_FOUND1) {
			panel_detect_status = PANEL_FOUND1;
		} else if (panel_detect_active == PANEL_FOUND2) {
			panel_detect_status = PANEL_FOUND2;
		}
		panel_detect_status_update = 0;
		pr_notice("%s: status update=%d panel_detect_active=%d\n",
				__func__, panel_detect_status, panel_detect_active);
	}
}

static char reg_read_cmd[1] = {0xF4};
static struct dsi_cmd_desc dsi_cmds = {
	{DTYPE_GEN_READ1, 1, 0, 1, 0, sizeof(reg_read_cmd)},
	reg_read_cmd
};
static char panel_id[] = {0xF6};

void kdisp_connect_check_panel_active(struct mdss_dsi_ctrl_pdata *ctrl)
{
	int i;
	struct dcs_cmd_req cmdreq;
	char rbuf[5];
	struct kc_ctrl_info *kctrl_data;

	kctrl_data = &ctrl->kc_ctrl_data;

	if (!kctrl_data->panel_det_en) {
		pr_err("%s: panel detect disable\n", __func__);
		return;
	}

	if ((panel_detect_active != PANEL_NOT_TEST) || (panel_detect_status != PANEL_FOUND1) || (panel_detect_status != PANEL_FOUND2)) {
		pr_debug("%s: skip active_check=%d panel_detect=%d\n",
			__func__, panel_detect_active, panel_detect_status);
		return;
	}

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &dsi_cmds;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT | CMD_REQ_HS_MODE;
	cmdreq.rlen = 1;
	cmdreq.cb = NULL;
	cmdreq.rbuf = rbuf;

	for (i = 0; i < PANEL_CHECK_RETRY_NUM; i++) {
		mdss_dsi_cmdlist_put(ctrl, &cmdreq);
		pr_notice("%s - panel ID: %02x\n", __func__,
			ctrl->rx_buf.data[0]);
		if (ctrl->rx_buf.data[0] == panel_id[0]) {
			break;
		}
	}

	if (i >= PANEL_CHECK_RETRY_NUM) {
		panel_detect_active = PANEL_NOT_FOUND;
		pr_notice("%s: panel not active\n", __func__);
	} else {
		if (panel_detect == PANEL_FOUND1) {
			panel_detect_active = PANEL_FOUND1;
			pr_notice("%s: panel active\n", __func__);
		} else if (panel_detect == PANEL_FOUND2) {
			panel_detect_active = PANEL_FOUND2;
			pr_notice("%s: panel active\n", __func__);
		} else {
			panel_detect_active = PANEL_NOT_FOUND;
			pr_notice("%s: panel not active\n", __func__);			
		}
	}

	panel_detect_status_update = 1;
}

struct device_node *kdisp_connect_find_panel_of_node(struct platform_device *pdev)
{
	struct device_node *mdss_node = NULL, *kpanel_node = NULL;
	int pf_sstype;

	mdss_node = of_parse_phandle(pdev->dev.of_node,
		"qcom,mdss-mdp", 0);
	
	if (!mdss_node) {
		pr_err("%s: %d: mdss_node null\n", __func__, __LINE__);
		return NULL;
	}

	pf_sstype = socinfo_get_platform_subtype();
	
	switch(pf_sstype) {
		case PF_SSTYPE_HINOKI00 :
			if (panel_detect == PANEL_FOUND1) {
				kpanel_node = of_find_node_by_name(mdss_node,
						"qcom,mdss_dsi_kc_boe_wxga_video");
				pr_notice("%s: PF_SSTYPE_HINOKI00\n", __func__);
				break;
			} else if (panel_detect == PANEL_FOUND2) {
				kpanel_node = of_find_node_by_name(mdss_node,
						"qcom,mdss_dsi_kc_boe_ilitek_wxga_video");
				pr_notice("%s: PF_SSTYPE_HINOKI00\n", __func__);
				break;
			} else {
				kpanel_node = of_find_node_by_name(mdss_node,
						"qcom,mdss_dsi_kc_boe_ilitek_wxga_video");
				pr_notice("%s: PF_SSTYPE_HINOKI00\n", __func__);
				break;
			}
		case PF_SSTYPE_HINOKI01 :
			kpanel_node = of_find_node_by_name(mdss_node,
					"qcom,mdss_dsi_kc_lce_hx83102b_video"); /* Provisional Setting */
			pr_notice("%s: PF_SSTYPE_HINOKI01\n", __func__);
			break;
		case PF_SSTYPE_HINOKI02 :
			kpanel_node = of_find_node_by_name(mdss_node,
					"qcom,mdss_dsi_kc_boe_wxga_video"); /* Provisional Setting */
			pr_notice("%s: PF_SSTYPE_HINOKI02\n", __func__);
			break;
		default:
			kpanel_node = of_find_node_by_name(mdss_node,
					"qcom,mdss_dsi_kc_boe_wxga_video");
			pr_err("%s: Invalid HW platform sub type\n", __func__);
			break;
		};

	return kpanel_node;
}

void kdisp_connect_init(struct platform_device *pdev,
	struct device_node *np,
	struct kc_ctrl_info *kctrl_data)
{

	if (!np || !kctrl_data) {
		pr_err("%s: Invalid arguments\n", __func__);
		return;
	}

	kctrl_data->panel_det_en = of_property_read_bool(np,
		"kc,panel-detect-enabled");

	pr_notice("%s:panel_det_en=%d\n",
			__func__, kctrl_data->panel_det_en);

	kdisp_connect_pinctrl_init(pdev);
	kdisp_connect_parse_gpio_params(pdev, kctrl_data);
	kdisp_connect_check_panel_detect(kctrl_data);
	kdisp_connect_notify_to_light();
}
