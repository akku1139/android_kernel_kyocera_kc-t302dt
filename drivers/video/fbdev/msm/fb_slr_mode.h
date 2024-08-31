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
#ifndef FB_SLR_MODE_H
#define FB_SLR_MODE_H

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/workqueue.h>
#include <linux/pinctrl/consumer.h>
#include <linux/fb.h>
#include "mdss_panel.h"
#include "mdss_dsi_cmd.h"

/*==================================================================*/
/*         Define                                                   */
/*==================================================================*/
struct slr_dsi_panel_cmds {
	char *buf;
	int blen;
	struct dsi_cmd_desc *cmds;
	int cmd_cnt;
	int link_state;
};

struct slr_ctrl_info {
	bool						slr_mode_en;
	u32						current_slr_mode;
	struct slr_dsi_panel_cmds	slr_str_cmds;
	struct slr_dsi_panel_cmds	slr_med_cmds;
	struct slr_dsi_panel_cmds	slr_weak_cmds;
	struct slr_dsi_panel_cmds	slr_off_cmds;
	u32						slr_mode_max;
};

ssize_t fb_slr_set_mode(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
ssize_t fb_slr_get_mode(struct device *dev,
		struct device_attribute *attr, char *buf);
int fb_slr_init(struct device_node *np,
	struct slr_ctrl_info *slr_data);
int fb_slr_set(struct mdss_dsi_ctrl_pdata *ctrl_pdata, unsigned int mode);

#endif /* FB_SLR_MODE_H */
