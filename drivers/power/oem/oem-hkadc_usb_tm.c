/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
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

#define pr_fmt(fmt)	"OEM_HKADC_USB_TM %s: " fmt, __func__

#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/power_supply.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/pm_wakeup.h>

#define HKADC_USB_TM_ERR		pr_err
#define HKADC_USB_TM_INFO		pr_info
//#define FEATURE_HKADC_USB_TM_DEBUG
#ifdef FEATURE_HKADC_USB_TM_DEBUG
#define HKADC_USB_TM_DEBUG	pr_err
#else
#define HKADC_USB_TM_DEBUG	pr_debug
#endif

#define DRV_NAME "kc,oem_hkadc_usb_tm-driver"
#define DEVICE_NAME "oem_usb_tm_wake-gpio"

struct oem_hkadc_usb_tm_device {
	struct	device			*dev;
	int						gpio;
	int						irq;
};

static struct oem_hkadc_usb_tm_device *chip;

static void set_usb_tm_wakelock(int gpio_high)
{
	if(gpio_high) {
		pm_stay_awake(chip->dev);
		HKADC_USB_TM_INFO("SET wakelock, GPIO:%d\n", gpio_high);
	}else {
		pm_relax(chip->dev);
		HKADC_USB_TM_INFO("CLEAR wakelock, GPIO:%d\n", gpio_high);
	}
}

static irqreturn_t usb_tm_wake_isr(int irq, void *dev)
{
	int gpio = gpio_get_value(chip->gpio);

	HKADC_USB_TM_INFO("HKADC_KAR_WAKE irq detected!! GPIO=%d\n", gpio);
	set_usb_tm_wakelock(gpio);

	return IRQ_HANDLED;
}

static int oem_hkadc_usb_tm_probe(struct platform_device *pdev)
{
	int rc = 0;

	if (!pdev->dev.of_node) {
		HKADC_USB_TM_ERR("No platform supplied from device tree.\n");
		rc = -EINVAL;
		goto err_arg;
	}

	chip = devm_kzalloc(&pdev->dev, sizeof(struct oem_hkadc_usb_tm_device), GFP_KERNEL);
	if (!chip) {
		HKADC_USB_TM_ERR("devm_kzalloc failed.\n");
		rc = -ENOMEM;
		goto err_alloc;
	}

	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);

	device_init_wakeup(chip->dev, true);

	chip->gpio = of_get_named_gpio(pdev->dev.of_node, DEVICE_NAME, 0);
	if (chip->gpio < 0) {
		HKADC_USB_TM_ERR("of_get_named_gpio failed.\n");
		rc = -EINVAL;
		goto err_gpio;
	}

	rc = gpio_request(chip->gpio, DEVICE_NAME);
	if (rc) {
		HKADC_USB_TM_ERR("gpio_request failed.\n");
		goto err_gpio;
	}

	chip->irq = gpio_to_irq(chip->gpio);
	rc = devm_request_irq(chip->dev, chip->irq, usb_tm_wake_isr,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "oem_hkadc_usb_tm-irq", chip);
	if (rc < 0) {
		HKADC_USB_TM_ERR("failed request_irq.\n");
		goto err_irq;
	}
	enable_irq_wake(chip->irq);

	HKADC_USB_TM_INFO("successful. GPIO(%d)=%d\n", chip->gpio, gpio_get_value(chip->gpio));

	return 0;


err_irq:
	gpio_free(chip->gpio);

err_gpio:
err_alloc:
err_arg:
	HKADC_USB_TM_DEBUG("failed.\n");

	return rc;
}

static int oem_hkadc_usb_tm_remove(struct platform_device *pdev)
{
	gpio_free(chip->gpio);

	return 0;
}

static const struct of_device_id oem_hkadc_usb_tm_of_match[] = {
	{ .compatible = DRV_NAME, },
};

static struct platform_driver oem_hkadc_usb_tm_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = oem_hkadc_usb_tm_of_match,
	},
	.probe = oem_hkadc_usb_tm_probe,
	.remove = oem_hkadc_usb_tm_remove,
};

int __init oem_hkadc_usb_tm_init(void)
{
	return platform_driver_register(&oem_hkadc_usb_tm_driver);
}
module_init(oem_hkadc_usb_tm_init);

static void __exit oem_hkadc_usb_tm_exit(void)
{
	platform_driver_unregister(&oem_hkadc_usb_tm_driver);
}
module_exit(oem_hkadc_usb_tm_exit);

MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("OEM HKADC USB TM Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("oem_hkadc_usb_tm");