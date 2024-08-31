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
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/moduleparam.h>
#include "mdss.h"
#include "mdss_dsi.h"
#include "mdss_panel.h"
#include "mdss_debug.h"
#include "mdss_smmu.h"
#include "mdss_dsi_phy.h"
#include "fb_slr_mode.h"

#ifdef FB_SLR_MODE
enum {
	DISP_EXT_SLR_OFF,
	DISP_EXT_SLR_WEAK,
	DISP_EXT_SLR_MEDIUM,
	DISP_EXT_SLR_STRONG,
};

int fb_slr_set(struct mdss_dsi_ctrl_pdata *ctrl_pdata, unsigned int mode)
{
	struct dsi_panel_cmds *slr_cmds;
	struct slr_ctrl_info *slr_data;

	pr_debug("%s+: mode=%d\n", __func__, mode);

	slr_data = ctrl_pdata->slr_ctrl_data;

	if(!slr_data->slr_mode_en){
		pr_debug("%s: Mode does not support SRE\n", __func__);
		return 0;
	}
	if (mode > slr_data->slr_mode_max) {
		pr_err("%s: invalid mode=%d\n", __func__, mode);
		return -EINVAL;
	}

	switch(mode){
		case DISP_EXT_SLR_STRONG:
			slr_cmds = (struct dsi_panel_cmds *)&slr_data->slr_str_cmds;
			break;
		case DISP_EXT_SLR_MEDIUM:
			slr_cmds = (struct dsi_panel_cmds *)&slr_data->slr_med_cmds;
			break;
		case DISP_EXT_SLR_WEAK:
			slr_cmds = (struct dsi_panel_cmds *)&slr_data->slr_weak_cmds;
			break;
		case DISP_EXT_SLR_OFF:
			slr_cmds = (struct dsi_panel_cmds *)&slr_data->slr_off_cmds;
			break;
		default:
			pr_err("%s: invalid mode=%d\n", __func__, mode);
			return -EINVAL;
	}

	if (!slr_cmds->cmd_cnt){
		pr_err("%s: No setting SRE params\n", __func__);
		return 0;
	}

	mdss_dsi_panel_cmds_send(ctrl_pdata, slr_cmds, CMD_REQ_COMMIT);
	slr_data->current_slr_mode = mode;

	pr_debug("%s-: mode=%d\n", __func__, mode);
	return 0;
}

int fb_slr_init(struct device_node *np,
	struct slr_ctrl_info *slr_data)
{
	int rc;

	slr_data->current_slr_mode = 0;

	if (!np || !slr_data) {
		pr_err("%s: Invalid arguments\n", __func__);
		return -ENODEV;
	}

	slr_data->slr_mode_en = of_property_read_bool(np,
		"slr-mode-enabled");
	if (!slr_data->slr_mode_en) {
		pr_err("%s: Mode does not support SRE\n", __func__);
		return 0;
	}

	rc = mdss_dsi_parse_dcs_cmds(np,
		(struct dsi_panel_cmds *)&slr_data->slr_str_cmds,
		"slr-mode-strong-command", "slr-mode-command-state");
	if (rc) {
		pr_err("%s: command parse failed\n", __func__);
		goto read_error;
	}
	rc = mdss_dsi_parse_dcs_cmds(np,
		(struct dsi_panel_cmds *)&slr_data->slr_med_cmds,
		"slr-mode-medium-command", "slr-mode-command-state");
	if (rc) {
		pr_err("%s: command parse failed\n", __func__);
		goto read_error;
	}
	rc = mdss_dsi_parse_dcs_cmds(np,
		(struct dsi_panel_cmds *)&slr_data->slr_weak_cmds,
		"slr-mode-weak-command", "slr-mode-command-state");
	if (rc) {
		pr_err("%s: command parse failed\n", __func__);
		goto read_error;
	}
	rc = mdss_dsi_parse_dcs_cmds(np,
		(struct dsi_panel_cmds *)&slr_data->slr_off_cmds,
		"slr-mode-off-command", "slr-mode-command-state");
	if (rc) {
		pr_err("%s: command parse failed\n", __func__);
		goto read_error;
	}
	rc = of_property_read_u32(np,
		"slr-mode-max", &slr_data->slr_mode_max);
	if (rc) {
		pr_err("%s: slr mode max parse failed\n", __func__);
		goto read_error;
	}

	pr_debug("%s: max=%u\n", __func__, slr_data->slr_mode_max);

	return 0;
read_error:
	slr_data->slr_mode_en = 0;
	return -EINVAL;
}

ssize_t fb_slr_set_mode(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mdss_mdp_ctl *mdp_ctl = NULL;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
	struct slr_ctrl_info *slr_data;
	int rc = 0;
	int slr_mode = 0;

	mdp_ctl = mfd_to_ctl(mfd);
	if (!mdp_ctl) {
		pr_err("%s: mdp_ctl is null\n", __func__);
		return -EFAULT;
	}

	pdata = dev_get_platdata(&mfd->pdev->dev);
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	if (!ctrl_pdata) {
		pr_err("%s: ctrl_pdata is null\n", __func__);
		return -EFAULT;
	}

	slr_data = ctrl_pdata->slr_ctrl_data;
	if (!slr_data) {
		pr_err("%s: slr_data is null, slr set mode failed\n", __func__);
		return -EFAULT;
	}

	rc = kstrtoint(buf, 10, &slr_mode);
	if (rc) {
		pr_err("kstrtoint failed. rc=%d\n", rc);
		return rc;
	}
	pr_debug("slr_mode = %d\n", slr_mode);
	if ( slr_mode < 0 )
		slr_mode = 0;
	if ( slr_mode > slr_data->slr_mode_max)
		slr_mode = slr_data->slr_mode_max;

	lock_fb_info(mfd->fbi);
	mutex_lock(&mdp_ctl->offlock);

	if (mdss_panel_is_power_off(mfd->panel_power_state) || mfd->shutdown_pending) {
		pr_info("%s: DSI turning off, avoiding send slr command\n", __func__);
	}
	else{
		fb_slr_set(ctrl_pdata, slr_mode);
	}

	mutex_unlock(&mdp_ctl->offlock);
	unlock_fb_info(mfd->fbi);

	return count;
}

ssize_t fb_slr_get_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
	struct slr_ctrl_info *slr_data;

	pdata = dev_get_platdata(&mfd->pdev->dev);
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	if (!ctrl_pdata) {
		pr_err("%s: ctrl_pdata is null\n", __func__);
		return -EFAULT;
	}

	slr_data = ctrl_pdata->slr_ctrl_data;
	if (!slr_data) {
		pr_err("%s: slr_data is null, slr get mode failed\n", __func__);
		return -EFAULT;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", slr_data->current_slr_mode);
}
#endif
