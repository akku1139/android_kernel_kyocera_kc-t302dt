/*****************************************************************************
*
* Filename:      irda_sd.c
* Version:       0.1
* Description:   IrDA driver
* Status:        Experimental
* Author:        KYOCERA Corporation
*
* This software is contributed or developed by KYOCERA Corporation.
* (C) 2015 KYOCERA Corporation
*
*	This program is free software; you can redistribute it and/or
*   modify it under the terms of the GNU General Public License
*   as published by the Free Software Foundation; only version 2.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program; if not, write to the Free Software
*   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*
*****************************************************************************/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>

/*#include <asm/gpio.h>*/
#include <linux/gpio.h>

#include <linux/major.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include "irda_sd.h"
#include <linux/of_gpio.h>

static struct irda_sd_info *irdasd_info;
static struct platform_device *pDev;
static 	int gpio_shutdown_port = -1;

static int irdasd_fop_open(struct inode *inode, struct file *file) ;
static int irdasd_fop_release(struct inode *inode, struct file *file) ;


/* ------------------------------------------------------------------------------------
 *		irdasd file operation
 * ------------------------------------------------------------------------------------ */
static int irdasd_fop_open(struct inode *inode, struct file *file)
{
	file->private_data = irdasd_info ;

	gpio_set_value_cansleep(gpio_shutdown_port, 0);
	return 0;
}

static int irdasd_fop_release(struct inode *inode, struct file *file)
{
	if(gpio_shutdown_port>0){
		gpio_set_value_cansleep((unsigned int)gpio_shutdown_port, 1);
	}
	else{
	}
	return 0;
}


static struct class *irdasd_class;

struct file_operations irdasd_fops =
{
	.owner		= THIS_MODULE,
	.open		= irdasd_fop_open,
	.release	= irdasd_fop_release,
};

static char *irdasd_devnode(struct device *dev, umode_t *mode)
{
	if (mode)
		*mode = 0666;
	return kasprintf(GFP_KERNEL,"%s", dev_name(dev));
}


static int irdasd_suspend (struct platform_device *pdev, pm_message_t state)
{
	if(gpio_shutdown_port>0){
		gpio_set_value_cansleep((unsigned int)gpio_shutdown_port, 1);
	}
	return 0;
}

static int irdasd_resume (struct platform_device *pdev)
{
	if(gpio_shutdown_port>0){
		gpio_set_value_cansleep((unsigned int)gpio_shutdown_port, 1);
	}
	return 0;
}


static int irdasd_probe(struct platform_device *pdev)
{
	int rc;

	/* PM_GPIO */
	if (!pdev->dev.of_node) {
		pr_err("No platform supplied from device tree.\n");
		return -EINVAL;
	}

	pDev = pdev;

/* Set IrDA SD port */
	gpio_shutdown_port = of_get_named_gpio(pDev->dev.of_node, "kc,irda-sd", 0);
	if (gpio_shutdown_port < 0) {
		pr_err("%s of_get_named_gpio failed.\n", __func__);
		return -EINVAL;
	}

	rc = gpio_request(gpio_shutdown_port, "irda-sd");
	if (rc) {
		pr_err("%s gpio_request failed.\n", __func__);
		return -EINVAL;
	}
	gpio_direction_output(gpio_shutdown_port, 1);

	return 0;
}

//static int __devexit irdasd_remove(struct platform_device *pdev)
static int irdasd_remove(struct platform_device *pdev)
{

	gpio_free(gpio_shutdown_port);
	gpio_shutdown_port = -1;

	return 0;
}

static const struct of_device_id irdasd_of_match[] = {
	{ .compatible = "kc,irdasd", },
	{},
};

static struct platform_driver irdasd_pd = {
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = irdasd_of_match,
	},
	.probe = irdasd_probe,
//	.remove = __devexit_p(irdasd_remove),
	.remove = irdasd_remove,
	.suspend	= irdasd_suspend,
	.resume		= irdasd_resume,
};


static int init_irdasd( void )
{
	int ret = 0;

	printk( KERN_NOTICE"IRDASD module is beeing initialized.\n" ) ;
	platform_driver_register(&irdasd_pd);

	irdasd_info = kzalloc(sizeof(*irdasd_info), GFP_KERNEL);
	if (irdasd_info == NULL) {
		pr_err(MODULE_NAME ":kzalloc err.\n");
		return -ENOMEM;
	}
	irdasd_class = class_create(THIS_MODULE, MODULE_NAME);

	ret = alloc_chrdev_region(&irdasd_info->dev_num, 0, 1, MODULE_NAME);
	if (ret) {
		printk(MODULE_NAME "alloc_chrdev_region err.\n");
		return -ENODEV;
	}

	irdasd_class->devnode = irdasd_devnode;

	irdasd_info->dev = device_create(irdasd_class, NULL, irdasd_info->dev_num,
				      irdasd_info, MODULE_NAME);
	if (IS_ERR(irdasd_info->dev)) {
		printk(MODULE_NAME ":device_create err.\n");
		return -ENODEV;
	}
	irdasd_info->cdev = cdev_alloc();
	if (irdasd_info->cdev == NULL) {
		printk(MODULE_NAME ":cdev_alloc err.\n");
		return -ENODEV;
	}
	cdev_init(irdasd_info->cdev, &irdasd_fops);
	irdasd_info->cdev->owner = THIS_MODULE;

	ret = cdev_add(irdasd_info->cdev, irdasd_info->dev_num, 1);
	if (ret)
		printk(MODULE_NAME ":cdev_add err=%d\n", -ret);
	else
		printk(MODULE_NAME ":irdasd init OK..\n");

	printk( " %s driver installed.\n", MODULE_NAME );

	return ret;

}

static void exit_irdasd( void )
{
	cdev_del(irdasd_info->cdev);
	device_destroy(irdasd_class, irdasd_info->dev_num);
	unregister_chrdev_region(irdasd_info->dev_num, 1);

	kfree(irdasd_info);
	printk( "IRDASD module is removed.\n" ) ;
}

module_init( init_irdasd ) ;
module_exit( exit_irdasd ) ;

