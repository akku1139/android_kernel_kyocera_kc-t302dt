/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 *
 * drivers/input/touchscreen/kc_dt.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/namei.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/i2c.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <asm/siginfo.h>

#include "wacom.h"
#include "kc_dt.h"

struct dt_sysfs_data_{
	char command;
	u16 start_addr;
	u16 size;
} dt_sdata;

/* sysfs ctrl command list */
#define KC_DT_SYSFS_POWER_CTRL		'p'
#define KC_DT_SYSFS_IRQ_CTRL		'i'
#define KC_DT_SYSFS_NOTIFY_CTRL		'n'
#define KC_DT_SYSFS_POWER_LOCK_CTRL	'l'
#define KC_DT_SYSFS_HW_RESET_CTRL	'h'
#define KC_DT_SYSFS_FW_CTRL			'f'

static int kc_dt_open(struct inode *inode, struct file *file)
{
	struct kc_dt_data *dt =
		container_of(inode->i_cdev, struct kc_dt_data, device_cdev);
	KC_DT_DEV_DBG("%s() is called.\n", __func__);

	file->private_data = dt;
	return 0;
};

static int kc_dt_release(struct inode *inode, struct file *file)
{
	KC_DT_DEV_DBG("%s() is called.\n", __func__);
	file->private_data = NULL;
	return 0;
};

static long kc_dt_diag_data_start(struct kc_dt_data *dt)
{
	KC_DT_DEV_DBG("%s is called.\n", __func__);
	if (dt->diag_data == NULL) {
		dt->diag_data = kzalloc(sizeof(struct dt_diag_type), GFP_KERNEL);
		if (!dt->diag_data) {
			pr_err("Failed to allocate memory!\n");
			return -ENOMEM;
		}
	}

	return 0;
}

void kc_dt_diag_store(struct kc_dt_data *dt, struct dt_diag_type *coordinate)
{
	KC_DT_DEV_DBG("%s is called.\n", __func__);
	mutex_lock(&dt->lock);
	dt->diag_data->pressure  = coordinate->pressure;
	dt->diag_data->x         = coordinate->x;
	dt->diag_data->y         = coordinate->y;
	dt->diag_data->hover     = coordinate->hover;
	dt->diag_data->rdy       = coordinate->rdy;
	dt->diag_data->invert    = coordinate->invert;
	dt->diag_data->eraser    = coordinate->eraser;
	dt->diag_data->tipswitch = coordinate->tipswitch;
	mutex_unlock(&dt->lock);
}

static long kc_dt_diag_data_end(struct kc_dt_data *dt)
{
	KC_DT_DEV_DBG("%s is called.\n", __func__);

	if (dt->diag_data != NULL) {
		kfree(dt->diag_data);
		dt->diag_data = NULL;
	}
	return 0;
}

int kc_dt_send_signal(struct kc_dt_data *dt, enum dt_sig_request req)
{
	struct siginfo info;
	struct task_struct *task;
	int ret = 0;

	KC_DT_DEV_DBG("%s is called.\n", __func__);

	if(dt->pid == 0){
		pr_err("%s: pid is not set.\n", __func__);
		ret = -1;
		goto done;
	}

	memset(&info, 0x00, sizeof(struct siginfo));

	info.si_signo = SIGUSR1;
	info.si_code = SI_QUEUE;
	info.si_int = req;

	rcu_read_lock();
	task = find_task_by_vpid(dt->pid);
	rcu_read_unlock();
	if (task == NULL){
		pr_err("%s: failed find_task_by_vpid() pid = %d \n", __func__, dt->pid);
		ret = -ENODEV;
		goto done;
	}
	ret = send_sig_info(SIGUSR1, &info, task);
	if (ret < 0)
		pr_err("%s: failed send_sig_info() ret = %d \n", __func__, ret);
done:
	return ret;
}

static long kc_dt_ioctl(struct file *file, unsigned int cmd,
						unsigned long arg)
{
	struct kc_dt_data *dt = (struct kc_dt_data *)file->private_data;
	struct device *dev = dt->dev;
	long err = 0;
	struct dt_coordinate dt_data;
	int i;

	KC_DT_DEV_DBG("%s() is called.\n", __func__);

	if (!dt) {
		dev_err(dev, "%s: kc_dt data is not set.\n", __func__);
		return -EINVAL;
	}

	switch (cmd) {

	case IOCTL_DIGI_COORDINATE_START:
		pr_info("%s: IOCTL_DIGI_COORDINATE_START\n", __func__);
		mutex_lock(&dt->lock);
		err = kc_dt_diag_data_start(dt);
		mutex_unlock(&dt->lock);
		break;

	case IOCTL_DIGI_COORDINATE_GET:
		pr_info("%s: IOCTL_DIGI_COORDINATE_GET\n", __func__);
		if (!access_ok(VERIFY_WRITE, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			err = -EFAULT;
			pr_err("%s: invalid access\n", __func__);
			return  err;
		}
		mutex_lock(&dt->lock);
		if (dt->diag_data != NULL) {
			err = copy_to_user((void __user *)arg, dt->diag_data,
						sizeof(struct dt_diag_type));
		} else
			pr_info("Digitizer Diag not active!\n");
		mutex_unlock(&dt->lock);

		if (err) {
			pr_err("%s: copy_to_user error\n", __func__);
			return -EFAULT;
		}
		break;

	case IOCTL_DIGI_COORDINATE_END:
		pr_info("%s: IOCTL_DIGI_COORDINATE_END\n", __func__);
		mutex_lock(&dt->lock);
		err = kc_dt_diag_data_end(dt);
		mutex_unlock(&dt->lock);
		break;

	case IOCTL_DIGI_FW_VER_GET:
		if (!access_ok(VERIFY_WRITE, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			pr_err("%s: invalid access\n", __func__);
			return  -EFAULT;
		}
		err = dt->tops->ioctl(dt, cmd, arg);
		break;
	case IOCTL_OPEN_SHORT:
		pr_info("%s: IOCTL_OPEN_SHORT\n", __func__);
		if (!access_ok(VERIFY_READ, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			pr_err("%s: invalid access\n", __func__);
			return -EFAULT;
		}

		mutex_lock(&dt->lock);
		err = copy_from_user(&dt_data, (void __user *)arg, sizeof(dt_data));
		if (err) {
			pr_err("%s: copy_from_user error\n", __func__);
			return -EFAULT;
		}

		if(dt_data.mode == 1){
			pr_info("%s: Coordinate Check\n", __func__);
			if (dt->dt_coordinate_data == NULL) {
				dt->dt_coordinate_data = kzalloc(sizeof(*dt->dt_coordinate_data), GFP_KERNEL);
				if (!dt->dt_coordinate_data) {
					pr_err("%s: Failed to allocate memory!\n", __func__);
					return -ENOMEM;
				}

				for(i=0; i<DT_XY_TEST*2; i+=2){
					dt->dt_coordinate_data->coordinate_data.result_coordinate[i] = START_X + X_LINE*(i >> 1);
					dt->dt_coordinate_data->coordinate_data.result_coordinate[i+1] = START_Y - Y_LINE*(i >> 1);
					dt->dt_coordinate_data->coordinate_data.success_coordinate[i >> 1] = 1;
					pr_debug("%s: coordinate (x,y) = (%5d, %5d)\n", __func__,
							dt->dt_coordinate_data->coordinate_data.result_coordinate[i],
							dt->dt_coordinate_data->coordinate_data.result_coordinate[i+1]);
				}
			}

			dt->dt_coordinate_data->fluctuation = dt_data.fluctuation;
			if(dt->dt_coordinate_data->fluctuation > MAX_FLUCTUATION)
				dt->dt_coordinate_data->fluctuation = MAX_FLUCTUATION;
			dt->coordinate_test_enable = 1;
			pr_info("%s: fluctuation %d\n", __func__, dt->dt_coordinate_data->fluctuation);

			err = copy_to_user((void __user *)arg, dt->dt_coordinate_data,
											sizeof(*dt->dt_coordinate_data));

		} else if(dt_data.mode == 2){
			pr_info("%s: Memory free\n", __func__);
			dt->coordinate_test_enable = 0;
			if (dt->dt_coordinate_data != NULL) {
				kfree(dt->dt_coordinate_data);
				dt->dt_coordinate_data = NULL;
			}

		} else{
			pr_err("%s: Input mode error\n", __func__);
			err = -1;
		}
		mutex_unlock(&dt->lock);
		break;

	case IOCTL_DIGI_SET_PS_STAT:
		pr_debug("%s: IOCTL_DIGI_SET_PS_STAT\n", __func__);
		if (!access_ok(VERIFY_READ, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			err = -EFAULT;
			pr_err("%s: invalid access\n", __func__);
			return err;
		}
		err = dt->tops->ioctl(dt, cmd, arg);
		break;

	case IOCTL_DIGI_SET_PID:
		pr_debug("%s: IOCTL_DIGI_SET_PID\n", __func__);
		if (!access_ok(VERIFY_READ, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			err = -EFAULT;
			pr_err("%s: invalid access\n", __func__);
			return err;
		}
		err = copy_from_user(&dt->pid, (void __user *)arg, sizeof(pid_t));
		if (err){
			err = -EFAULT;
			pr_err("%s: copy_from_user error\n", __func__);
			return err;;
		}
		pr_debug("%s: IOCTL_DIGI_SET_PID pid =0x%x \n", __func__, dt->pid);
		break;

	default:
		dev_err(dev, "%s: cmd error[%X]\n", __func__, cmd);
		err = -EINVAL;
		break;
	}
	return err;
}

int dt_ctrl_init(struct kc_dt_data *dt, const struct file_operations *fops)
{
	struct cdev *device_cdev = &dt->device_cdev;
	int device_major = dt->device_major;

	dev_t device_t = MKDEV(0, 0);
	struct device *class_dev_t = NULL;
	int ret;

	ret = alloc_chrdev_region(&device_t, 0, 1, "dt_ctrl");
	if (ret)
		goto error;

	device_major = MAJOR(device_t);

	cdev_init(device_cdev, fops);
	device_cdev->owner = THIS_MODULE;
	device_cdev->ops = fops;
	ret = cdev_add(device_cdev, MKDEV(device_major, 0), 1);
	if (ret)
		goto err_unregister_chrdev;

	dt->device_class = class_create(THIS_MODULE, "dt_ctrl");
	if (IS_ERR(dt->device_class)) {
		ret = -1;
		goto err_cleanup_cdev;
	};

	class_dev_t = device_create(dt->device_class, NULL,
		MKDEV(device_major, 0), NULL, "dt_ctrl");
	if (IS_ERR(class_dev_t)) {
		ret = -1;
		goto err_destroy_class;
	}

	return 0;

err_destroy_class:
	class_destroy(dt->device_class);
err_cleanup_cdev:
	cdev_del(device_cdev);
err_unregister_chrdev:
	unregister_chrdev_region(device_t, 1);
error:
	return ret;
}
EXPORT_SYMBOL_GPL(dt_ctrl_init);

int dt_ctrl_exit(struct kc_dt_data *dt)
{
	struct cdev *device_cdev = &dt->device_cdev;
	int device_major = dt->device_major;
	struct class *device_class = dt->device_class;
	dev_t device_t = MKDEV(device_major, 0);

	if (device_class) {
		device_destroy(device_class, MKDEV(device_major, 0));
		class_destroy(device_class);
	}
	if (device_cdev) {
		cdev_del(device_cdev);
		unregister_chrdev_region(device_t, 1);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(dt_ctrl_exit);

const struct file_operations kc_dt_fops = {
	.owner = THIS_MODULE,
	.open = kc_dt_open,
	.unlocked_ioctl = kc_dt_ioctl,
	.release = kc_dt_release,
};

static ssize_t kc_dt_ctrl_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);
	struct kc_dt_data *dt = (struct kc_dt_data *)&wac_i2c->kc_dt;
	unsigned int val;
	int err = 0;
	int lock;

	switch (buf[0]) {
	case KC_DT_SYSFS_POWER_CTRL:
		KC_DT_DEV_INFO("%s: KC_DT_SYSFS_POWER_CTRL\n", __func__);
		sscanf(buf, "%c %x", &dt_sdata.command,
					(unsigned int *) &val);
		KC_DT_DEV_INFO("%s: set_power val = %d \n", __func__, val);

		lock = dt->power_lock;
		dt->power_lock = 0;

		if (val == 1)
			err = dt->tops->power_on(dt);
		else
			err = dt->tops->power_off(dt);
		if (err)
			KC_DT_DEV_ERR("%s: failed irq ctrl\n", __func__);

		dt->power_lock = lock;

		break;
	case KC_DT_SYSFS_IRQ_CTRL:
		KC_DT_DEV_INFO("%s: KC_DT_SYSFS_IRQ_CTRL\n", __func__);
		sscanf(buf, "%c %x", &dt_sdata.command,
					(unsigned int *) &val);
		KC_DT_DEV_INFO("%s: set_irq val = %d \n", __func__, val);
		err = dt->tops->irq(dt, val);
		if (err)
			KC_DT_DEV_ERR("%s: failed irq ctrl\n", __func__);
		break;
	case KC_DT_SYSFS_NOTIFY_CTRL:
		KC_DT_DEV_INFO("%s: KC_DT_SYSFS_NOTIFY_CTRL\n", __func__);
		sscanf(buf, "%c %x", &dt_sdata.command,
					(unsigned int *) &val);
		dt->android_notify = val;
		KC_DT_DEV_INFO("%s: android_notify val = %d \n", __func__, dt->android_notify);
		break;
	case KC_DT_SYSFS_POWER_LOCK_CTRL:
		KC_DT_DEV_INFO("%s: KC_DT_SYSFS_POWER_LOCK_CTRL\n", __func__);
		sscanf(buf, "%c %x", &dt_sdata.command,
					(unsigned int *) &val);
		dt->power_lock = val;
		KC_DT_DEV_INFO("%s: power_lock = %x\n", __func__, dt->power_lock);
		break;
	case KC_DT_SYSFS_HW_RESET_CTRL:
		KC_DT_DEV_INFO("%s: KC_DT_SYSFS_HW_RESET_CTRL\n", __func__);
		sscanf(buf, "%c %x", &dt_sdata.command, &val);
		KC_DT_DEV_INFO("%s: set_reset val = %d \n", __func__, val);

		if(val==1){
			err = dt->tops->hw_reset(dt);
			if (err)
				KC_DT_DEV_ERR("%s: failed hw_reset err = %d\n", __func__, err);
		}
		break;

	case KC_DT_SYSFS_FW_CTRL:
		KC_DT_DEV_INFO("%s: KC_DT_SYSFS_FW_CTRL\n", __func__);
		sscanf(buf, "%c %x", &dt_sdata.command, &val);
		KC_DT_DEV_INFO("%s: set_fw_ctrl val = %d \n", __func__, val);
		if(val == 0){
			err = kc_dt_send_signal(dt, DT_SIG_UPDATE_FW);
			if (err < 0)
				KC_DT_DEV_ERR("%s: failed kc_dt_send_signal()\n", __func__);
		}else if (val == 1){
			dt->fw_ver_check_flg = 0;
		}else if (val == 2){
			dt->fw_ver_check_flg = 1;
		}

		break;
	default:
		break;
	}
	return count;
}

static ssize_t kc_dt_ctrl_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct wacom_i2c *wac_i2c = dev_get_drvdata(dev);
	struct kc_dt_data *dt = (struct kc_dt_data *)&wac_i2c->kc_dt;
	int count = 0;
	int err;
	int fw_version;

	switch (dt_sdata.command) {
	case KC_DT_SYSFS_IRQ_CTRL:
		KC_DT_DEV_INFO("%s: KC_DT_SYSFS_IRQ_CTRL\n", __func__);
		count += scnprintf(buf, PAGE_SIZE - count,"irq_status is "
							"[%d]\n",wac_i2c->is_enable);
		break;
	case KC_DT_SYSFS_NOTIFY_CTRL:
		KC_DT_DEV_INFO("%s: KC_DT_SYSFS_NOTIFY_CTRL\n", __func__);
		count += scnprintf(buf, PAGE_SIZE - count,"android_notify is "
							"[%d]\n",dt->android_notify);
		break;
	case KC_DT_SYSFS_POWER_LOCK_CTRL:
		KC_DT_DEV_INFO("%s: KC_DT_SYSFS_POWER_LOCK_CTRL\n", __func__);
		count += scnprintf(buf, PAGE_SIZE - count,"power_lock is "
							"[%d]\n", dt->power_lock);
		break;
	case KC_DT_SYSFS_FW_CTRL:
		KC_DT_DEV_INFO("%s: KC_DT_SYSFS_FW_CTRL\n", __func__);

		err = dt->tops->get_fw_version(dt, &fw_version);
		if (err)
			KC_DT_DEV_ERR("%s: failed hw_reset err = %d\n", __func__, err);

		KC_DT_DEV_INFO("%s: current_fw_ver %04x, soft_fw_ver %04x fw_ver_check_flg = %d \n", __func__
							,fw_version, dt->soft_fw_ver,dt->fw_ver_check_flg);
		if(dt->fw_ver_check_flg == 0){
			count += scnprintf(buf, PAGE_SIZE - count,"current_fw_ver is "
								"[%04x]\n", fw_version);
		}else if(dt->fw_ver_check_flg == 1){
			count += scnprintf(buf, PAGE_SIZE - count,"device fw_version is latest"
								"[%02x]\n", (fw_version == dt->soft_fw_ver)?  0x00 : 0xFF);
		}
		break;
	default:
		break;
	}
	return count;
}

static DEVICE_ATTR(ctrl, S_IRUSR|S_IWUSR, kc_dt_ctrl_show, kc_dt_ctrl_store);

static struct attribute *kc_attrs[] = {
	&dev_attr_ctrl.attr,
	NULL
};

static const struct attribute_group kc_attr_group = {
	.attrs = kc_attrs,
};

int kc_dt_probe(struct kc_dt_data *dt)
{
	int ret = 0;

	KC_DT_DEV_DBG("%s is called.\n", __func__);
	if (!dt) {
		pr_err("%s: kc_dt data is not set.\n", __func__);
		return -EINVAL;
	}

	if (!dt->dev) {
		pr_err("%s: dev is not set.\n", __func__);
		return -EINVAL;
	}

	if (!dt->tops) {
		pr_err("%s: tops is not set.\n", __func__);
		return -EINVAL;
	}

	mutex_init(&dt->lock);

	/* Create cdev file dt_ctrl */
	ret = dt_ctrl_init(dt, &kc_dt_fops);
	if (ret) {
		pr_err("%s: Fail to create cdev.\n", __func__);
		goto err_cdev;
	}

	/* Create sysfs */
	ret = sysfs_create_group(&dt->dev->kobj, &kc_attr_group);
	if (ret) {
		pr_err("%s: Fail to create sysfs.\n", __func__);
		goto err_sysfs;
	}

	KC_DT_DEV_DBG("%s is completed.\n", __func__);

	return ret;

err_sysfs:
	dt_ctrl_exit(dt);
err_cdev:
	mutex_destroy(&dt->lock);

	return ret;
}
EXPORT_SYMBOL(kc_dt_probe);

void kc_dt_remove(struct kc_dt_data *dt)
{
	KC_DT_DEV_DBG("%s is called.\n", __func__);

	if (!dt || !dt->dev) {
		pr_err("%s: kc_dt data is not set.\n", __func__);
		return;
	}

	mutex_destroy(&dt->lock);
	sysfs_remove_group(&dt->dev->kobj, &kc_attr_group);
	dt_ctrl_exit(dt);

	KC_DT_DEV_DBG("%s is completed.\n", __func__);

	return;

}
EXPORT_SYMBOL(kc_dt_remove);

MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("kc digitizer driver");
MODULE_LICENSE("GPL");
