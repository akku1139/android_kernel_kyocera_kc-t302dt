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
#ifndef KDISP_COM_H
#define KDISP_COM_H

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
#define KDISP_PINCTRL_STATE_DET_ACTIVE  "lcd_id_active"
#define KDISP_PINCTRL_STATE_DET_SUSPEND  "lcd_id_suspend"

struct kc_pinctrl_res {
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_det_active;
	struct pinctrl_state *gpio_state_det_suspend;
};

struct kc_dsi_freq_info {
	u32		hfp;
	u32		hbp;
	u32		hpw;
	u32		vfp;
	u32		vbp;
	u32		vpw;
	u32		phy_timing_8996[40];
	char	t_clk_post;
	char	t_clk_pre;
};

struct kc_dsi_panel_cmds {
	char *buf;
	int blen;
	struct dsi_cmd_desc *cmds;
	int cmd_cnt;
	int link_state;
};

struct kc_ctrl_info {
	int							vlcdio_18_en_gpio;
	int							vddlcd_32_en_gpio;
	int							lcd_id1_gpio;
	int							lcd_id2_gpio;
	struct delayed_work			on_comp_work;
	struct kc_pinctrl_res		pin_res;
	bool						panel_det_en;
	/* touch */
	bool						tp_incell_en;
	int							(*tp_display_resume) (void *data, int order);
	int							(*tp_display_syspend) (void *data, int order);
	int							(*tp_display_watchdog) (void *data);
	void						(*tp_display_set_easywake_mode) (void *data, int easywake_mode);
	/* freq mode */
	bool						dsi_freq_en;
	u32							dsi_freq_mode;
	struct kc_dsi_freq_info		dsi_freq_j;
	struct kc_dsi_freq_info		dsi_freq_u;
	struct kc_dsi_freq_info		dsi_freq_e;
	/* sre mode */
	bool						sre_mode_en;
	u32							current_sre_mode;
	u32							user_sre_mode;
	struct kc_dsi_panel_cmds	sre_str_cmds;
	struct kc_dsi_panel_cmds	sre_med_cmds;
	struct kc_dsi_panel_cmds	sre_weak_cmds;
	struct kc_dsi_panel_cmds	sre_off_cmds;
	u32							sre_mode_max;
	bool						easywake_en;
	u32							easywake_req;
	u32							touch_state;
	u32							lp00_delay;
	u32							dsi_off_rst_delay;
};

/* kdisp_diag */
#define REG_ERR_CHECK 0x01
#define IMG_ERR_CHECK 0x02
#define MIPI_ERR_CHECK 0x04

/* kdisp_touch_ctrl */
enum kdisp_touch_ctrl_cmd {
	KDISP_TOUCH_CTRL_PRE_PANEL_POWERON = 0,
	KDISP_TOUCH_CTRL_POST_PANEL_POWERON,
	KDISP_TOUCH_CTRL_PRE_PANEL_POWEROFF,
	KDISP_TOUCH_CTRL_POST_PANEL_POWEROFF,
	KDISP_TOUCH_CTRL_POST_PANEL_ON
};

enum kdisp_touch_state {
	KDISP_TOUCH_STATE_PWROFF,
	KDISP_TOUCH_STATE_PWRON,
	KDISP_TOUCH_STATE_EASYWAKE,
};

/* kdisp_connect_target */
#define PANEL_NOT_TEST				0
#define PANEL_FOUND					1
#define PANEL_NOT_FOUND				-1

/*==================================================================*/
/*         Function                                                 */
/*==================================================================*/
/* kdisp_com */
int kdisp_com_check_not_supported_ioctl(void);
int kdisp_com_is_invalid(void);
int kdisp_com_is_invalid_all(void);
int kdisp_com_dsihost_force_recovery(struct mdss_panel_data *pdata);
void kdisp_com_init(struct platform_device *pdev,
	struct device_node *np, struct kc_ctrl_info *kc_ctrl_data);
void kdisp_com_panel_power_onoff(struct kc_ctrl_info *kctrl_data, int onoff);

/* kdisp_diag */
void kdisp_diag_init(void);
int kdisp_diag_ioctl_handler(struct fb_info *info, unsigned int cmd, void __user *argp);
int kdisp_diag_get_dmflag(void);

/* kdisp_touch_ctrl */
int kdisp_touch_ctrl_power(enum kdisp_touch_ctrl_cmd cmd,
	struct kc_ctrl_info *kctrl_data);
int kdisp_touch_ctrl_check_status(struct kc_ctrl_info *kctrl_data);
ssize_t kdisp_touch_easywake_set_mode(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
ssize_t kdisp_touch_easywake_get_mode(struct device *dev,
		struct device_attribute *attr, char *buf);
int kdisp_touch_ctrl_init(struct device_node *np,
	struct kc_ctrl_info *kctrl_data);
int kdisp_touch_get_easywake_onoff(struct kc_ctrl_info *kctrl_data, int blank);

/* kdisp_bl_ctrl */
void kdisp_bl_ctrl_init(struct kc_ctrl_info *kc_ctrl_data);
void kdisp_bl_ctrl_onoff(struct kc_ctrl_info *kc_ctrl_data, bool onoff);

/* kdisp_connect_target */
void kdisp_connect_init(struct platform_device *pdev,
	struct device_node *np,
	struct kc_ctrl_info *kctrl_data);
int kdisp_connect_get_panel_detect(void);
void kdisp_connect_update_panel_detect_status(void);
struct device_node *kdisp_connect_find_panel_of_node(struct platform_device *pdev);
void kdisp_connect_check_panel_active(struct mdss_dsi_ctrl_pdata *ctrl);

/* kdisp_debug */
void kdisp_debug_log_output_dsi_cmd_tx(char *bp, int len);
void kdisp_debug_dump_mdp_timing_reg(char __iomem *base);
void kdisp_debug_dump_dsi_timing_reg(void __iomem *phy_io_base, unsigned char *ctrl_base);
void kdisp_debug_dump_dsi_video_reg(char __iomem *base);
ssize_t kdisp_panel_set_test_mode(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
ssize_t kdisp_panel_get_test_mode(struct device *dev,
		struct device_attribute *attr, char *buf);
ssize_t kdisp_panel_get_kctrl_info(struct device *dev,
		struct device_attribute *attr, char *buf);

/* kdisp_panel */
int kdisp_panel_init(struct device_node *np,
	struct kc_ctrl_info *kc_ctrl_data);
void kdisp_panel_set_dsi_freq(struct mdss_panel_info *pinfo, struct kc_ctrl_info *kctrl_data, int mode);
ssize_t kdisp_panel_set_dsi_freq_mode(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
ssize_t kdisp_panel_get_dsi_freq_mode(struct device *dev,
		struct device_attribute *attr, char *buf);

/* kdisp_sre */
ssize_t kdisp_sre_set_mode(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
ssize_t kdisp_sre_get_mode(struct device *dev,
		struct device_attribute *attr, char *buf);
int kdisp_sre_init(struct device_node *np,
	struct kc_ctrl_info *kctrl_data);
int kdisp_sre_set(struct mdss_dsi_ctrl_pdata *ctrl_pdata, unsigned int mode);

#endif /* KDISP_COM_H */
