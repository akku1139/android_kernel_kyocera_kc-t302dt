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
#include "mdss.h"
#include "mdss_dsi.h"
#include "kdisp_com.h"

#define KFBM_DISABLE 0
#define KFBM_ENABLE  1
static int kfbm_stat = KFBM_DISABLE;

extern void mdss_dsi_ctl_phy_reset(struct mdss_dsi_ctrl_pdata *ctrl, u32 event);
extern void mdss_dsi_err_intr_ctrl(struct mdss_dsi_ctrl_pdata *ctrl, u32 mask,
					int enable);
extern struct mutex dsi_mtx;

int kdisp_com_check_not_supported_ioctl(void)
{
	if (!strcmp(current->group_leader->comm, "cam_drv_diag")) {
		pr_err("%s: only support cam_drv_diag\n", __func__);
		return 0;
	}

	return 1;
}

static int kdisp_com_get_kfbm_status(char *buf)
{
	if (strcmp(buf, "kcfactory") == 0)
	{
		kfbm_stat = KFBM_ENABLE;
	}
	return 0;
}
early_param("androidboot.mode", kdisp_com_get_kfbm_status);

int kdisp_com_is_invalid_all(void)
{
	if (kfbm_stat == KFBM_ENABLE) {
		pr_notice("%s: KFBM_ENABLE\n", __func__);
		if (kdisp_connect_get_panel_detect() == PANEL_NOT_FOUND) {
			return 1;
		}
	}

	return 0;
}

int kdisp_com_is_invalid(void)
{
	if (kdisp_diag_get_dmflag() != 0 && strcmp(current->comm, "kdispdiag") != 0) {
		pr_notice("%s: dispdiag invalid!\n", __func__);
		return 1;
	} else {
		return 0;
	}
}

int kdisp_com_dsihost_force_recovery(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_err("%s: force recovery!!!!\n", __func__);

	mutex_lock(&dsi_mtx);

	mdss_dsi_clk_ctrl(ctrl, ctrl->dsi_clk_handle,
			  MDSS_DSI_ALL_CLKS,
			  MDSS_DSI_CLK_ON);
	mdss_dsi_ctl_phy_reset(ctrl,
			DSI_EV_DLNx_FIFO_OVERFLOW);
	mdss_dsi_err_intr_ctrl(ctrl,
			DSI_INTR_ERROR_MASK, 1);
	mdss_dsi_clk_ctrl(ctrl, ctrl->dsi_clk_handle,
			  MDSS_DSI_ALL_CLKS,
			  MDSS_DSI_CLK_OFF);

	mutex_unlock(&dsi_mtx);

	return 0;
}

static int kdisp_com_pinctrl_init(struct platform_device *pdev)
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

static int kdisp_com_parse_gpio_params(struct platform_device *pdev,
	struct kc_ctrl_info *kc_ctrl_data)
{
	int rc = 0;
	u32 tmp = 0;

    kc_ctrl_data->vlcdio_18_en_gpio = of_get_named_gpio(pdev->dev.of_node,
                     "kc,vlcdio_18-en-gpio", 0);
    if (!gpio_is_valid(kc_ctrl_data->vlcdio_18_en_gpio))
            pr_err("%s:%d, vlcdio_18 en gpio not specified\n",
                                            __func__, __LINE__);

    kc_ctrl_data->vddlcd_32_en_gpio = of_get_named_gpio(pdev->dev.of_node,
                     "kc,vddlcd-32-en-gpio", 0);
    if (!gpio_is_valid(kc_ctrl_data->vddlcd_32_en_gpio))
            pr_err("%s:%d, vddlcd_32 en gpio not specified\n",
                                            __func__, __LINE__);

	rc = of_property_read_u32(pdev->dev.of_node, "kc,lp00-delay-us", &tmp);
	kc_ctrl_data->lp00_delay = (!rc ? tmp : 0);

	rc = of_property_read_u32(pdev->dev.of_node, "kc,dsi-off-rst-delay-us", &tmp);
	kc_ctrl_data->dsi_off_rst_delay = (!rc ? tmp : 0);

	return 0;
}

void kdisp_com_init(struct platform_device *pdev,
	struct device_node *np, struct kc_ctrl_info *kc_ctrl_data)
{
	kdisp_com_pinctrl_init(pdev);
	kdisp_com_parse_gpio_params(pdev, kc_ctrl_data);

	kdisp_diag_init();
	kdisp_connect_init(pdev, np, kc_ctrl_data);
}

void kdisp_com_panel_power_onoff(struct kc_ctrl_info *kctrl_data, int onoff)
{
	int rc = 0;

	pr_debug("%s+: onoff=%d\n", __func__, onoff);

    if (!gpio_is_valid(kctrl_data->vlcdio_18_en_gpio)) {
		pr_err("%s:%d, vlcdio_18_en_gpio line not configured\n",
			   __func__, __LINE__);
		return;
	}

    if (!gpio_is_valid(kctrl_data->vddlcd_32_en_gpio)) {
		pr_err("%s:%d, vddlcd_32_en_gpio line not configured\n",
			   __func__, __LINE__);
		return;
	}

	if(onoff){
		rc = gpio_request(kctrl_data->vlcdio_18_en_gpio, "vlcdio_18_en");
		if (rc) {
		        pr_err("request vlcdio_18 en gpio failed,rc=%d\n",
		                                                rc);
		        return;
		}

		rc = gpio_request(kctrl_data->vddlcd_32_en_gpio, "vddlcd_32_en");
		if (rc) {
		        pr_err("request vddlcd_32 en gpio failed,rc=%d\n",
		                                                rc);
		        return;
		}
		gpio_set_value((kctrl_data->vlcdio_18_en_gpio), 1);
		usleep_range(8 * 1000, 8 * 1000);
		gpio_set_value((kctrl_data->vddlcd_32_en_gpio), 1);
		usleep_range(3 * 1000, 3 * 1000);
	}else{
		gpio_set_value((kctrl_data->vddlcd_32_en_gpio), 0);
		gpio_free(kctrl_data->vddlcd_32_en_gpio);
		usleep_range(3 * 1000, 3 * 1000);
		gpio_set_value((kctrl_data->vlcdio_18_en_gpio), 0);
		gpio_free(kctrl_data->vlcdio_18_en_gpio);
		usleep_range(3 * 1000, 3 * 1000);
	}

	pr_debug("%s-:\n", __func__);
}
