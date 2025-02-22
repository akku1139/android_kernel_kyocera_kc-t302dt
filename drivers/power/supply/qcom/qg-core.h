/* Copyright (c) 2018 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2019 KYOCERA Corporation
 */

#ifndef __QG_CORE_H__
#define __QG_CORE_H__

#include <linux/kernel.h>
#include "fg-alg.h"
#include "qg-defs.h"

/*-------*/
struct oem_cycle_counter {
	struct mutex	lock;
	int				count;
	int				start_soc;
	int				last_soc;
	int				increase_soc;
	bool			dnand_loaded;
};

struct oem_online_time {
	struct mutex	lock;
	int				total_time;
	bool			dnand_loaded;
};

enum {
	CYCLE_COUNT = 0,
	CONT_CHG,
};

enum {
	BATT_CARE_MODE_INIT = -1,
	BATT_CARE_MODE_OFF = 0,
	BATT_CARE_MODE_ON = 1,
};
/*-------*/

struct qg_batt_props {
	const char		*batt_type_str;
	int			float_volt_uv;
	int			vbatt_full_mv;
	int			fastchg_curr_ma;
	int			qg_profile_version;

	/* oem add */
	int			oem_batt_capacity_mah;
	int			oem_deterioration_thresh_good;
	int			oem_deterioration_thresh_norm;
	int			oem_deterioration_thresh_normtogood;
	int			oem_deterioration_thresh_deadtonorm;
	int			oem_float_volt_uv_design;
	int			*oem_cycle_count_thresh;
	int			*oem_cycle_count_fv_comp_mv;
	int			oem_cycle_count_levels;
	int			*oem_cont_chg_thresh;
	int			*oem_cont_chg_fv_comp_mv;
	int			oem_cont_chg_levels;
	int			*oem_batt_temp_thresh;
	int			*oem_cont_chg_factor;
	int			*oem_cont_chg_factor_care_mode;
	int			oem_cont_chg_factor_levels;
};

struct qg_irq_info {
	const char		*name;
	const irq_handler_t	handler;
	const bool		wake;
	int			irq;
};

struct qg_dt {
	int			vbatt_empty_mv;
	int			vbatt_empty_cold_mv;
	int			vbatt_low_mv;
	int			vbatt_low_cold_mv;
	int			vbatt_cutoff_mv;
	int			iterm_ma;
	int			s2_fifo_length;
	int			s2_vbat_low_fifo_length;
	int			s2_acc_length;
	int			s2_acc_intvl_ms;
	int			ocv_timer_expiry_min;
	int			ocv_tol_threshold_uv;
	int			s3_entry_fifo_length;
	int			s3_entry_ibat_ua;
	int			s3_exit_ibat_ua;
	int			delta_soc;
	int			rbat_conn_mohm;
	int			ignore_shutdown_soc_secs;
	int			shutdown_temp_diff;
	int			cold_temp_threshold;
	int			esr_qual_i_ua;
	int			esr_qual_v_uv;
	int			esr_disable_soc;
	int			esr_min_ibat_ua;
	int			shutdown_soc_threshold;
	bool			hold_soc_while_full;
	bool			linearize_soc;
	bool			cl_disable;
	bool			cl_feedback_on;
	bool			esr_disable;
	bool			esr_discharge_enable;
	bool			qg_ext_sense;
	bool			qg_vbms_mode;
};

struct qg_esr_data {
	u32			pre_esr_v;
	u32			pre_esr_i;
	u32			post_esr_v;
	u32			post_esr_i;
	u32			esr;
	bool			valid;
};

struct qpnp_qg {
	struct device		*dev;
	struct pmic_revid_data	*pmic_rev_id;
	struct regmap		*regmap;
	struct qpnp_vadc_chip	*vadc_dev;
	struct power_supply	*qg_psy;
	struct class		*qg_class;
	struct device		*qg_device;
	struct cdev		qg_cdev;
	dev_t			dev_no;
	struct work_struct	udata_work;
	struct work_struct	scale_soc_work;
	struct work_struct	qg_status_change_work;
	struct notifier_block	nb;
	struct mutex		bus_lock;
	struct mutex		data_lock;
	struct mutex		soc_lock;
	wait_queue_head_t	qg_wait_q;
	struct votable		*awake_votable;
	struct votable		*vbatt_irq_disable_votable;
	struct votable		*fifo_irq_disable_votable;
	struct votable		*good_ocv_irq_disable_votable;
	u32			qg_base;

	/* local data variables */
	u32			batt_id_ohm;
	struct qg_kernel_data	kdata;
	struct qg_user_data	udata;
	struct power_supply	*batt_psy;
	struct power_supply	*usb_psy;
	struct power_supply	*parallel_psy;
	struct qg_esr_data	esr_data[QG_MAX_ESR_COUNT];

	/* status variable */
	u32			*debug_mask;
	bool			qg_device_open;
	bool			profile_loaded;
	bool			battery_missing;
	bool			data_ready;
	bool			suspend_data;
	bool			vbat_low;
	bool			charge_done;
	bool			parallel_enabled;
	bool			usb_present;
	bool			charge_full;
	int			charge_status;
	int			charge_type;
	int			chg_iterm_ma;
	int			next_wakeup_ms;
	int			esr_actual;
	int			esr_nominal;
	int			soh;
	int			soc_reporting_ready;
	int			vbms_ibat_ua;
	u32			fifo_done_count;
	u32			wa_flags;
	u32			seq_no;
	u32			charge_counter_uah;
	u32			esr_avg;
	u32			esr_last;
	ktime_t			last_user_update_time;
	ktime_t			last_fifo_update_time;
	unsigned long		last_maint_soc_update_time;

	/* soc params */
	int			catch_up_soc;
	int			maint_soc;
	int			msoc;
	int			pon_soc;
	int			batt_soc;
	int			cc_soc;
	int			full_soc;
	int			sys_soc;
	int			last_adj_ssoc;
	int			recharge_soc;
	struct alarm		alarm_timer;
	u32			sdam_data[SDAM_MAX];

	/* DT */
	struct qg_dt		dt;
	struct qg_batt_props	bp;
	/* capacity learning */
	struct cap_learning	*cl;
	/* charge counter */
	struct cycle_counter	*counter;
	/* ttf */
	struct ttf		*ttf;

	/* oem add */
	struct votable		*fv_votable;
	struct votable		*usb_icl_votable;
	struct oem_cycle_counter	oem_cycle;
	struct oem_online_time		oem_online;
	struct delayed_work			oem_cycle_dnand_work;
	struct delayed_work			oem_online_dnand_work;
	struct delayed_work			oem_online_time_work;
	struct delayed_work			oem_batt_care_work;
	struct delayed_work			oem_ui_soc_check_work;
	struct mutex				oem_batt_care_lock;
	int					oem_batt_care_mode;
	int					oem_batt_care_float_volt_uv;
	int					oem_real_soc;
	int					oem_ui_soc;
	int					oem_batt_care_notification;
	unsigned long		start_time, insertion_time, removal_time;
	unsigned long		connect_time, total_connect_time, lapsed_time;
	bool				oem_read_complete_flag;
	bool				oem_mode_change_flag;
	bool				oem_batt_care_boot_flag;
	bool				oem_ui_soc_increasing_flag;
	bool				oem_notification_flag;
	bool				oem_capacity_flag;
};

struct ocv_all {
	u32 ocv_uv;
	u32 ocv_raw;
	char ocv_type[20];
};

enum ocv_type {
	S7_PON_OCV,
	S3_GOOD_OCV,
	S3_LAST_OCV,
	SDAM_PON_OCV,
	PON_OCV_MAX,
};

enum debug_mask {
	QG_DEBUG_PON		= BIT(0),
	QG_DEBUG_PROFILE	= BIT(1),
	QG_DEBUG_DEVICE		= BIT(2),
	QG_DEBUG_STATUS		= BIT(3),
	QG_DEBUG_FIFO		= BIT(4),
	QG_DEBUG_IRQ		= BIT(5),
	QG_DEBUG_SOC		= BIT(6),
	QG_DEBUG_PM		= BIT(7),
	QG_DEBUG_BUS_READ	= BIT(8),
	QG_DEBUG_BUS_WRITE	= BIT(9),
	QG_DEBUG_ALG_CL		= BIT(10),
	QG_DEBUG_ESR		= BIT(11),
};

enum qg_irq {
	QG_BATT_MISSING_IRQ,
	QG_VBATT_LOW_IRQ,
	QG_VBATT_EMPTY_IRQ,
	QG_FIFO_UPDATE_DONE_IRQ,
	QG_GOOD_OCV_IRQ,
	QG_FSM_STAT_CHG_IRQ,
	QG_EVENT_IRQ,
	QG_MAX_IRQ,
};

enum qg_wa_flags {
	QG_VBAT_LOW_WA = BIT(0),
	QG_RECHARGE_SOC_WA = BIT(1),
};


#endif /* __QG_CORE_H__ */
