/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 * (C) 2017 KYOCERA Corporation
 * (C) 2018 KYOCERA Corporation
 * (C) 2019 KYOCERA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt) "HKADC: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/power_supply.h>
#include <linux/spmi.h>

#include <oem-hkadc_usb_tm_det.h>

#define DEFAULT_TEMP			(25 * 1000)
#define DEFAULT_TIME_MS			5000

#define USB_THERM_OVERHEAT	(90 * 1000)

static struct power_supply *batt_psy = NULL;

struct monitor_setting {
	bool		camera_disable;
	bool		out_camera_disable;
	bool		in_camera_disable;
	bool		pa_disable;
	bool		lcd_disable;
	bool		xo_disable;
	bool		usb_disable;
	bool		usb_tm_det_enable;
};

struct oem_hkadc_chip {
	struct device			*dev;
	struct power_supply		*hkadc_psy;
	struct qpnp_vadc_chip	*vadc_dev;
	struct qpnp_vadc_chip	*vadc_dev_pmi;
	unsigned int			hkadc_monitor_ms;
	unsigned int			hkadc_monitor_resume_ms;
	struct delayed_work		hkadc_monitor_work;
	struct monitor_setting	therm_setting;
};

static enum power_supply_property oem_hkadc_power_props[] = {
	POWER_SUPPLY_PROP_OEM_CAMERA_THERM,
	POWER_SUPPLY_PROP_OEM_OUT_CAMERA_THERM,
	POWER_SUPPLY_PROP_OEM_IN_CAMERA_THERM,
	POWER_SUPPLY_PROP_OEM_PA_THERM,
	POWER_SUPPLY_PROP_OEM_LCD_THERM,
	POWER_SUPPLY_PROP_OEM_USB_THERM,
	POWER_SUPPLY_PROP_OEM_XO_THERM,
};

enum usb_alert_info {
    USB_ALERT_BOOT,
    USB_ALERT_1,
    USB_ALERT_2,
};

static int get_prop_camera_therm(struct oem_hkadc_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	if (chip->therm_setting.camera_disable) {
		return DEFAULT_TEMP;
	}

	rc = qpnp_vadc_read(chip->vadc_dev_pmi, P_MUX1_1_1, &results);
	if (rc) {
		pr_err("Unable to read camera temperature rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	pr_debug("camera therm %d, %lld\n", results.adc_code, results.physical);

	return (int)results.physical;
}

static int get_prop_out_camera_therm(struct oem_hkadc_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	if (chip->therm_setting.out_camera_disable) {
		return DEFAULT_TEMP;
	}

	rc = qpnp_vadc_read(chip->vadc_dev_pmi, VADC_AMUX3_GPIO_PU2, &results);
	if (rc) {
		pr_err("Unable to read out camera temperature rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	pr_debug("out camera therm %d, %lld\n", results.adc_code, results.physical);

	return (int)results.physical;
}

static int get_prop_in_camera_therm(struct oem_hkadc_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	if (chip->therm_setting.in_camera_disable) {
		return DEFAULT_TEMP;
	}

	rc = qpnp_vadc_read(chip->vadc_dev, VADC_AMUX_THM5, &results);
	if (rc) {
		pr_err("Unable to read in camera temperature rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	pr_debug("in camera therm %d, %lld\n", results.adc_code, results.physical);

	return (int)results.physical;
}

static int get_prop_pa_therm(struct oem_hkadc_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	if (chip->therm_setting.pa_disable) {
		return DEFAULT_TEMP;
	}

	rc = qpnp_vadc_read(chip->vadc_dev, VADC_AMUX5_GPIO_PU1, &results);
	if (rc) {
		pr_err("Unable to read pa temperature rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	pr_debug("pa therm %d, %lld\n", results.adc_code, results.physical);

	return (int)results.physical;
}

static int get_prop_lcd_therm(struct oem_hkadc_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	if (chip->therm_setting.lcd_disable) {
		return DEFAULT_TEMP;
	}

	rc = qpnp_vadc_read(chip->vadc_dev, VADC_AMUX2_GPIO, &results);
	if (rc) {
		pr_err("Unable to read lcd temperature rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	pr_debug("lcd therm %d, %lld\n", results.adc_code, results.physical);

	return (int)results.physical;
}

static int get_prop_usb_therm(struct oem_hkadc_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;
	union power_supply_propval ret = {0,};
	static int usb_temp_level = USB_ALERT_BOOT;

	if (chip->therm_setting.usb_disable) {
		return DEFAULT_TEMP;
	}

	if (chip->therm_setting.usb_tm_det_enable) {
		return oem_hkadc_usb_therm_value();
	}

	rc = qpnp_vadc_read(chip->vadc_dev, P_MUX4_1_1, &results);
	if (rc) {
		pr_err("Unable to read usb temperature rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	pr_debug("usb therm %d, %lld\n", results.adc_code, results.physical);

	if ((int)results.physical >= USB_THERM_OVERHEAT) {
		pr_debug("usb therm overheat\n");
		ret.intval = USB_ALERT_1;
	} else {
		ret.intval = USB_ALERT_BOOT;
	}
	if (usb_temp_level != ret.intval) {
		usb_temp_level = ret.intval;
		pr_debug("set property usb_temp_level:%d\n", usb_temp_level);
		rc = power_supply_set_property(batt_psy, POWER_SUPPLY_PROP_USB_TEMP_LEVEL, &ret);
	}

	return (int)results.physical;
}

static int get_prop_xo_therm(struct oem_hkadc_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	if (chip->therm_setting.xo_disable) {
		return DEFAULT_TEMP;
	}

	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX3_XO_THERM, &results);
	if (rc) {
		pr_err("Unable to read xo temperature rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	pr_debug("xo therm %d, %lld\n", results.adc_code, results.physical);

	return (int)results.physical;
}

static int oem_hkadc_power_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *pval)
{
	struct oem_hkadc_chip *chip = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_OEM_CAMERA_THERM:
		pval->intval = get_prop_camera_therm(chip);
		break;
	case POWER_SUPPLY_PROP_OEM_OUT_CAMERA_THERM:
		pval->intval = get_prop_out_camera_therm(chip);
		break;
	case POWER_SUPPLY_PROP_OEM_IN_CAMERA_THERM:
		pval->intval = get_prop_in_camera_therm(chip);
		break;
	case POWER_SUPPLY_PROP_OEM_PA_THERM:
		pval->intval = get_prop_pa_therm(chip);
		break;
	case POWER_SUPPLY_PROP_OEM_LCD_THERM:
		pval->intval = get_prop_lcd_therm(chip);
		break;
	case POWER_SUPPLY_PROP_OEM_USB_THERM:
		pval->intval = get_prop_usb_therm(chip);
		break;
	case POWER_SUPPLY_PROP_OEM_XO_THERM:
		pval->intval = get_prop_xo_therm(chip);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void oem_hkadc_monitor(struct work_struct *work)
{
	struct oem_hkadc_chip *chip = container_of(work,
		struct oem_hkadc_chip, hkadc_monitor_work.work);

	power_supply_changed(chip->hkadc_psy);

	schedule_delayed_work(&chip->hkadc_monitor_work,
		msecs_to_jiffies(chip->hkadc_monitor_ms));
}

static int oem_hkadc_parse_dt(struct oem_hkadc_chip *chip)
{
	int rc;
	struct device_node *node = chip->dev->of_node;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}

	/* Normal monitor time ms */
	rc = of_property_read_u32(node, "oem,monitor-time-ms",
		&chip->hkadc_monitor_ms);
	if (rc < 0) {
		chip->hkadc_monitor_ms = DEFAULT_TIME_MS;
		pr_err("Missing required properties rc=%d\n", rc);
	}
	pr_info("chip->hkadc_monitor_ms=%d\n", chip->hkadc_monitor_ms);

	/* Resume monitor time ms */
	rc = of_property_read_u32(node, "oem,resume-mon-time-ms",
		&chip->hkadc_monitor_resume_ms);
	if (rc < 0) {
		chip->hkadc_monitor_resume_ms = DEFAULT_TIME_MS;
		pr_err("Missing required properties rc=%d\n", rc);
	}
	pr_info("chip->hkadc_monitor_resume_ms=%d\n", chip->hkadc_monitor_resume_ms);

	chip->therm_setting.camera_disable = of_property_read_bool(node,
				"oem,camera-therm-diasble");
	chip->therm_setting.out_camera_disable = of_property_read_bool(node,
				"oem,out-camera-therm-diasble");
	chip->therm_setting.in_camera_disable = of_property_read_bool(node,
				"oem,in-camera-therm-diasble");
	chip->therm_setting.pa_disable = of_property_read_bool(node,
				"oem,pa-therm-diasble");
	chip->therm_setting.lcd_disable = of_property_read_bool(node,
				"oem,lcd-therm-diasble");
	chip->therm_setting.xo_disable = of_property_read_bool(node,
				"oem,xo-therm-diasble");
	chip->therm_setting.usb_disable = of_property_read_bool(node,
				"oem,usb-therm-diasble");
	chip->therm_setting.usb_tm_det_enable = of_property_read_bool(node,
				"oem,usb-therm-det-enable");
	pr_info("camera_disable=%d, out_camera_disable=%d, in_camera_disable=%d, pa_disable=%d, lcd_disable=%d, xo_disable=%d, usb_disable=%d\n",
			chip->therm_setting.camera_disable,
			chip->therm_setting.out_camera_disable,
			chip->therm_setting.in_camera_disable,
			chip->therm_setting.pa_disable,
			chip->therm_setting.lcd_disable,
			chip->therm_setting.xo_disable,
			chip->therm_setting.usb_disable);
	pr_info("usb_tm_det_enable=%d\n", chip->therm_setting.usb_tm_det_enable);

	return 0;
}

static const struct power_supply_desc oem_hkadc_psy_desc = {
	.name = "hkadc",
	.type = POWER_SUPPLY_TYPE_HKADC,
	.properties = oem_hkadc_power_props,
	.num_properties = ARRAY_SIZE(oem_hkadc_power_props),
	.get_property = oem_hkadc_power_get_property,
};

static int oem_hkadc_init_psy(struct oem_hkadc_chip *chip)
{
	struct power_supply_config oem_hkadc_cfg = {};
	int rc = 0;

	oem_hkadc_cfg.drv_data = chip;
	oem_hkadc_cfg.of_node = chip->dev->of_node;
	chip->hkadc_psy = power_supply_register(chip->dev, &oem_hkadc_psy_desc, &oem_hkadc_cfg);
	if (IS_ERR(chip->hkadc_psy)) {
		pr_err("Couldn't register oem hkadc power supply\n");
		return PTR_ERR(chip->hkadc_psy);
	}

	return rc;
}

static int oem_hkadc_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct oem_hkadc_chip *chip;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&pdev->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);

	chip->vadc_dev = qpnp_get_vadc(chip->dev, "oem-hkadc");
	if (IS_ERR(chip->vadc_dev)) {
		rc = PTR_ERR(chip->vadc_dev);
		if (rc == -EPROBE_DEFER) {
			pr_err("vadc not found - defer probe rc=%d\n", rc);
		} else {
			pr_err("vadc property missing, rc=%d\n", rc);
		}

		goto fail_hkadc_enable;
	}

	chip->vadc_dev_pmi = qpnp_get_vadc(chip->dev, "oem-pmi-hkadc");
	if (IS_ERR(chip->vadc_dev_pmi)) {
		rc = PTR_ERR(chip->vadc_dev_pmi);
		if (rc == -EPROBE_DEFER) {
			pr_err("vadc pmi not found - defer probe rc=%d\n", rc);
		} else {
			pr_err("vadc pmi property missing, rc=%d\n", rc);
		}

		goto fail_hkadc_enable;
	}

	rc = oem_hkadc_parse_dt(chip);
	if (rc < 0) {
		dev_err(&pdev->dev, "Unable to parse DT nodes\n");
		return rc;
	}

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		pr_err("battery power supply not found deferring probe\n");
		goto fail_hkadc_enable;
	}

	rc = oem_hkadc_init_psy(chip);
	if (rc < 0) {
		pr_err("Couldn't initialize oem hkadc psy rc=%d\n", rc);
		return rc;
	}

	INIT_DELAYED_WORK(&chip->hkadc_monitor_work, oem_hkadc_monitor);

	schedule_delayed_work(&chip->hkadc_monitor_work, 0);

	pr_info("probe success camera_therm:%dC, out_camera_therm:%dC, in_camera_therm:%dC, pa_therm:%dC, lcd_therm:%dC, xo_therm:%dC, usb_therm:%dC\n",
		get_prop_camera_therm(chip), get_prop_out_camera_therm(chip), get_prop_in_camera_therm(chip), get_prop_pa_therm(chip),
		get_prop_lcd_therm(chip), get_prop_xo_therm(chip), get_prop_usb_therm(chip));
	return 0;

fail_hkadc_enable:
	pr_err("%s: failed.\n", __func__);

	return rc;
}

static int oem_hkadc_remove(struct platform_device *pdev)
{
	struct oem_hkadc_chip *chip = dev_get_drvdata(&pdev->dev);

	cancel_delayed_work_sync(&chip->hkadc_monitor_work);
	power_supply_unregister(chip->hkadc_psy);
	dev_set_drvdata(&pdev->dev, NULL);
	return 0;
}

static int oem_hkadc_suspend(struct device *dev)
{
	struct oem_hkadc_chip *chip = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&chip->hkadc_monitor_work);
	return 0;
}

static int oem_hkadc_resume(struct device *dev)
{
	struct oem_hkadc_chip *chip = dev_get_drvdata(dev);

	schedule_delayed_work(&chip->hkadc_monitor_work,
		msecs_to_jiffies(chip->hkadc_monitor_resume_ms));
	return 0;
}

static const struct of_device_id oem_hkadc_of_match[] = {
	{ .compatible = "oem_hkadc-driver", },
	{},
};

static const struct dev_pm_ops oem_hkadc_pm_ops = {
	.resume = oem_hkadc_resume,
	.suspend = oem_hkadc_suspend,
};

static struct platform_driver oem_hkadc_driver = {
	.driver = {
		.name = "oem_hkadc-driver",
		.owner = THIS_MODULE,
		.of_match_table = oem_hkadc_of_match,
		.pm = &oem_hkadc_pm_ops,
	},
	.probe = oem_hkadc_probe,
	.remove = oem_hkadc_remove,
};

static int __init oem_hkadc_init(void)
{
	return platform_driver_register(&oem_hkadc_driver);
}
module_init(oem_hkadc_init);

static void __exit oem_hkadc_exit(void)
{
	return platform_driver_unregister(&oem_hkadc_driver);
}
module_exit(oem_hkadc_exit);

MODULE_DESCRIPTION("oem_hkadc driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("oem_hkadc");
