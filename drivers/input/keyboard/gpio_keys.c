/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 * Copyright 2010, 2011 David Jander <david@protonic.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 * (C) 2017 KYOCERA Corporation
 * (C) 2018 KYOCERA Corporation
 * (C) 2019 KYOCERA Corporation
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/spinlock.h>

#include <linux/key_dm_driver.h>

#ifdef DEBUG_KEYLOG_ENABLE
#define KEY_LOG_PRINT(fmt, ...) printk(KERN_ERR fmt, ##__VA_ARGS__)
#else
#define KEY_LOG_PRINT(fmt, ...) pr_debug(fmt, ##__VA_ARGS__)
#endif
#define KEY_LOG_PRINT_SEND(fmt, ...) printk(KERN_NOTICE fmt, ##__VA_ARGS__)

struct gpio_button_data {
#ifdef QUALCOMM_ORIGINAL_FEATURE
	const struct gpio_keys_button *button;
#else
	struct gpio_keys_button *button;
#endif
	struct input_dev *input;
	struct gpio_desc *gpiod;

	struct timer_list release_timer;
	unsigned int release_delay;	/* in msecs, for IRQ-only buttons */

	struct delayed_work work;
	unsigned int software_debounce;	/* in msecs, for GPIO-driven buttons */

	unsigned int irq;
	spinlock_t lock;
	bool disabled;
	bool key_pressed;
};

struct gpio_keys_drvdata {
	const struct gpio_keys_platform_data *pdata;
	struct input_dev *input;
	struct mutex disable_lock;
	struct gpio_button_data data[0];
};

struct gpio_keys_chattering_state {
	unsigned int code;	/* input event code (KEY_*, SW_*) */
	bool is_on;			/* weather key is on. */
};

#define MAX_CHATTERING_KEYS 32
static int num_chattering_keys = 0;
static struct gpio_keys_chattering_state gpio_keys_chattering_data[MAX_CHATTERING_KEYS];

#define KDM_INIT_KEYCODE 0xFFFFFFFF
static int g_kdm_check = 0;
static int g_kdm_keycode = KDM_INIT_KEYCODE;

/*
 * SYSFS interface for enabling/disabling keys and switches:
 *
 * There are 4 attributes under /sys/devices/platform/gpio-keys/
 *	keys [ro]              - bitmap of keys (EV_KEY) which can be
 *	                         disabled
 *	switches [ro]          - bitmap of switches (EV_SW) which can be
 *	                         disabled
 *	disabled_keys [rw]     - bitmap of keys currently disabled
 *	disabled_switches [rw] - bitmap of switches currently disabled
 *
 * Userland can change these values and hence disable event generation
 * for each key (or switch). Disabling a key means its interrupt line
 * is disabled.
 *
 * For example, if we have following switches set up as gpio-keys:
 *	SW_DOCK = 5
 *	SW_CAMERA_LENS_COVER = 9
 *	SW_KEYPAD_SLIDE = 10
 *	SW_FRONT_PROXIMITY = 11
 * This is read from switches:
 *	11-9,5
 * Next we want to disable proximity (11) and dock (5), we write:
 *	11,5
 * to file disabled_switches. Now proximity and dock IRQs are disabled.
 * This can be verified by reading the file disabled_switches:
 *	11,5
 * If we now want to enable proximity (11) switch we write:
 *	5
 * to disabled_switches.
 *
 * We can disable only those keys which don't allow sharing the irq.
 */

/**
 * get_n_events_by_type() - returns maximum number of events per @type
 * @type: type of button (%EV_KEY, %EV_SW)
 *
 * Return value of this function can be used to allocate bitmap
 * large enough to hold all bits for given type.
 */
static int get_n_events_by_type(int type)
{
	BUG_ON(type != EV_SW && type != EV_KEY);

	return (type == EV_KEY) ? KEY_CNT : SW_CNT;
}

/**
 * get_bm_events_by_type() - returns bitmap of supported events per @type
 * @input: input device from which bitmap is retrieved
 * @type: type of button (%EV_KEY, %EV_SW)
 *
 * Return value of this function can be used to allocate bitmap
 * large enough to hold all bits for given type.
 */
static const unsigned long *get_bm_events_by_type(struct input_dev *dev,
						  int type)
{
	BUG_ON(type != EV_SW && type != EV_KEY);

	return (type == EV_KEY) ? dev->keybit : dev->swbit;
}

/**
 * gpio_keys_disable_button() - disables given GPIO button
 * @bdata: button data for button to be disabled
 *
 * Disables button pointed by @bdata. This is done by masking
 * IRQ line. After this function is called, button won't generate
 * input events anymore. Note that one can only disable buttons
 * that don't share IRQs.
 *
 * Make sure that @bdata->disable_lock is locked when entering
 * this function to avoid races when concurrent threads are
 * disabling buttons at the same time.
 */
static void gpio_keys_disable_button(struct gpio_button_data *bdata)
{
	if (!bdata->disabled) {
		/*
		 * Disable IRQ and associated timer/work structure.
		 */
		disable_irq(bdata->irq);

		if (bdata->gpiod)
			cancel_delayed_work_sync(&bdata->work);
		else
			del_timer_sync(&bdata->release_timer);

		bdata->disabled = true;
	}
}

/**
 * gpio_keys_enable_button() - enables given GPIO button
 * @bdata: button data for button to be disabled
 *
 * Enables given button pointed by @bdata.
 *
 * Make sure that @bdata->disable_lock is locked when entering
 * this function to avoid races with concurrent threads trying
 * to enable the same button at the same time.
 */
static void gpio_keys_enable_button(struct gpio_button_data *bdata)
{
	if (bdata->disabled) {
		enable_irq(bdata->irq);
		bdata->disabled = false;
	}
}

/**
 * gpio_keys_attr_show_helper() - fill in stringified bitmap of buttons
 * @ddata: pointer to drvdata
 * @buf: buffer where stringified bitmap is written
 * @type: button type (%EV_KEY, %EV_SW)
 * @only_disabled: does caller want only those buttons that are
 *                 currently disabled or all buttons that can be
 *                 disabled
 *
 * This function writes buttons that can be disabled to @buf. If
 * @only_disabled is true, then @buf contains only those buttons
 * that are currently disabled. Returns 0 on success or negative
 * errno on failure.
 */
static ssize_t gpio_keys_attr_show_helper(struct gpio_keys_drvdata *ddata,
					  char *buf, unsigned int type,
					  bool only_disabled)
{
	int n_events = get_n_events_by_type(type);
	unsigned long *bits;
	ssize_t ret;
	int i;

	bits = kcalloc(BITS_TO_LONGS(n_events), sizeof(*bits), GFP_KERNEL);
	if (!bits)
		return -ENOMEM;

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (only_disabled && !bdata->disabled)
			continue;

		__set_bit(bdata->button->code, bits);
	}

	ret = scnprintf(buf, PAGE_SIZE - 1, "%*pbl", n_events, bits);
	buf[ret++] = '\n';
	buf[ret] = '\0';

	kfree(bits);

	return ret;
}

/**
 * gpio_keys_attr_store_helper() - enable/disable buttons based on given bitmap
 * @ddata: pointer to drvdata
 * @buf: buffer from userspace that contains stringified bitmap
 * @type: button type (%EV_KEY, %EV_SW)
 *
 * This function parses stringified bitmap from @buf and disables/enables
 * GPIO buttons accordingly. Returns 0 on success and negative error
 * on failure.
 */
static ssize_t gpio_keys_attr_store_helper(struct gpio_keys_drvdata *ddata,
					   const char *buf, unsigned int type)
{
	int n_events = get_n_events_by_type(type);
	const unsigned long *bitmap = get_bm_events_by_type(ddata->input, type);
	unsigned long *bits;
	ssize_t error;
	int i;

	bits = kcalloc(BITS_TO_LONGS(n_events), sizeof(*bits), GFP_KERNEL);
	if (!bits)
		return -ENOMEM;

	error = bitmap_parselist(buf, bits, n_events);
	if (error)
		goto out;

	/* First validate */
	if (!bitmap_subset(bits, bitmap, n_events)) {
		error = -EINVAL;
		goto out;
	}

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (test_bit(bdata->button->code, bits) &&
		    !bdata->button->can_disable) {
			error = -EINVAL;
			goto out;
		}
	}

	mutex_lock(&ddata->disable_lock);

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (test_bit(bdata->button->code, bits))
			gpio_keys_disable_button(bdata);
		else
			gpio_keys_enable_button(bdata);
	}

	mutex_unlock(&ddata->disable_lock);

out:
	kfree(bits);
	return error;
}

#define ATTR_SHOW_FN(name, type, only_disabled)				\
static ssize_t gpio_keys_show_##name(struct device *dev,		\
				     struct device_attribute *attr,	\
				     char *buf)				\
{									\
	struct platform_device *pdev = to_platform_device(dev);		\
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);	\
									\
	return gpio_keys_attr_show_helper(ddata, buf,			\
					  type, only_disabled);		\
}

ATTR_SHOW_FN(keys, EV_KEY, false);
ATTR_SHOW_FN(switches, EV_SW, false);
ATTR_SHOW_FN(disabled_keys, EV_KEY, true);
ATTR_SHOW_FN(disabled_switches, EV_SW, true);

/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/gpio-keys/keys [ro]
 * /sys/devices/platform/gpio-keys/switches [ro]
 */
static DEVICE_ATTR(keys, S_IRUGO, gpio_keys_show_keys, NULL);
static DEVICE_ATTR(switches, S_IRUGO, gpio_keys_show_switches, NULL);

#define ATTR_STORE_FN(name, type)					\
static ssize_t gpio_keys_store_##name(struct device *dev,		\
				      struct device_attribute *attr,	\
				      const char *buf,			\
				      size_t count)			\
{									\
	struct platform_device *pdev = to_platform_device(dev);		\
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);	\
	ssize_t error;							\
									\
	error = gpio_keys_attr_store_helper(ddata, buf, type);		\
	if (error)							\
		return error;						\
									\
	return count;							\
}

ATTR_STORE_FN(disabled_keys, EV_KEY);
ATTR_STORE_FN(disabled_switches, EV_SW);

/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/gpio-keys/disabled_keys [rw]
 * /sys/devices/platform/gpio-keys/disables_switches [rw]
 */
static DEVICE_ATTR(disabled_keys, S_IWUSR | S_IRUGO,
		   gpio_keys_show_disabled_keys,
		   gpio_keys_store_disabled_keys);
static DEVICE_ATTR(disabled_switches, S_IWUSR | S_IRUGO,
		   gpio_keys_show_disabled_switches,
		   gpio_keys_store_disabled_switches);

static struct attribute *gpio_keys_attrs[] = {
	&dev_attr_keys.attr,
	&dev_attr_switches.attr,
	&dev_attr_disabled_keys.attr,
	&dev_attr_disabled_switches.attr,
	NULL,
};

static struct attribute_group gpio_keys_attr_group = {
	.attrs = gpio_keys_attrs,
};

unsigned char key_cmd(unsigned char cmd, int *val)
{
	unsigned char ret = 1;
	KEY_LOG_PRINT("%s:  %d\n", __func__, cmd);
	switch (cmd) {
	case KEY_DM_CHECK_COMMAND:
		KEY_LOG_PRINT("key_cmd check:%x val0:%x val1:%x \n", g_kdm_check, val[0], val[1]);
		if (val[0]) {
			g_kdm_check = 1;
		} else {
			g_kdm_check = 0;
		}
		ret = 0;
		break;

	case KEY_DM_KEY_GET_EVENT_COOMAND:
		KEY_LOG_PRINT("key_cmd code:%x\n", g_kdm_keycode );
		*val = g_kdm_keycode;
		g_kdm_keycode = KDM_INIT_KEYCODE;
		ret = 0;
		break;

	default:
		printk(KERN_ERR "%s:  %d\n", __func__, cmd);
		break;

	}
	return ret;

}
EXPORT_SYMBOL(key_cmd);

void key_set_code(unsigned int code )
{
	KEY_LOG_PRINT("key_set_code code:%d \n", code);
	g_kdm_keycode = code;
}
EXPORT_SYMBOL(key_set_code);

static unsigned int gpio_keys_get_chattering_pos(unsigned int code)
{
	unsigned int i;
	unsigned int size;

	size = num_chattering_keys;

	KEY_LOG_PRINT("key off chattering table size=%d \n",size);

	for (i = 0 ; i < size; i++) {
		if (gpio_keys_chattering_data[i].code == code) {
			KEY_LOG_PRINT("find keycode=0x%x,table pos=%d\n", code, i);
			return i;
		}
	}
	KEY_LOG_PRINT("Can't find keycode=0x%x\n", code);
	return KDM_INIT_KEYCODE;
}

static void gpio_keys_set_stateon(unsigned int code)
{
	unsigned int pos;
	pos = gpio_keys_get_chattering_pos(code);

	if (pos != KDM_INIT_KEYCODE) {
		KEY_LOG_PRINT("gpio key 0x%x on\n", code);
		gpio_keys_chattering_data[pos].is_on = true;
	} else {
		KEY_LOG_PRINT("gpio keyon pos invalid\n");
	}
}

static void gpio_keys_set_stateoff(unsigned int code)
{
	unsigned int pos;
	pos = gpio_keys_get_chattering_pos(code);

	if (pos != KDM_INIT_KEYCODE) {
		KEY_LOG_PRINT("gpio key 0x%x off\n", code);
		gpio_keys_chattering_data[pos].is_on = false;
	} else {
		KEY_LOG_PRINT("gpio keyoff pos invalid\n");
	}
}

static bool gpio_keys_is_stateon(unsigned int code)
{
	unsigned int pos;
	pos = gpio_keys_get_chattering_pos(code);

	if (pos != KDM_INIT_KEYCODE) {
		KEY_LOG_PRINT("gpio key0x%x state ", code);
		if (gpio_keys_chattering_data[pos].is_on == true) {
			KEY_LOG_PRINT("on\n");
			return true;
		} else {
			KEY_LOG_PRINT("off\n");
			return false;
		}
	}

	KEY_LOG_PRINT("gpio keystate pos invalid\n");
	return false;
}

static struct _volkey_stat {
	bool ignore;
	bool state;
} volkey_stat[2];

static struct timer_list volupkey_timer;
static struct timer_list voldownkey_timer;
static bool smlpress_timer_init = false;
static spinlock_t volkey_stat_lock;

bool gpio_keys_smlpress_check(unsigned int code, bool state)
{
	int rp_code;
	int vs_code;
	unsigned long flags;
	bool rp_state;
	bool vs_state;

	if(smlpress_timer_init != true) {
		KEY_LOG_PRINT("Not init smlpress timer.\n");
		return false;
	}

	if(KEY_VOLUMEUP != code
	&& KEY_VOLUMEDOWN != code) {
		KEY_LOG_PRINT("non vol key.\n");
		return false;
	}

	if(KEY_VOLUMEUP == code) {
		rp_code = 0;
		vs_code = 1;
	} else {
		rp_code = 1;
		vs_code = 0;
	}

	spin_lock_irqsave(&volkey_stat_lock, flags);

	if(true == volkey_stat[rp_code].ignore) {
		KEY_LOG_PRINT("[%d]key ignored.\n", rp_code);
		spin_unlock_irqrestore(&volkey_stat_lock, flags);
		return true;
	}

	volkey_stat[rp_code].state = state;

	rp_state = volkey_stat[rp_code].state;
	vs_state = volkey_stat[vs_code].state;

	if(0 == volkey_stat[rp_code].state
	&& 0 == volkey_stat[vs_code].state) {
		volkey_stat[rp_code].ignore = false;
		volkey_stat[vs_code].ignore = false;
	}
	
	spin_unlock_irqrestore(&volkey_stat_lock, flags);

	if(rp_state != vs_state) {
		if(KEY_VOLUMEUP == code)
			mod_timer(&volupkey_timer, jiffies + msecs_to_jiffies(70));
		else
			mod_timer(&voldownkey_timer, jiffies + msecs_to_jiffies(70));
	} else {
		if(KEY_VOLUMEUP == code)
			del_timer_sync(&volupkey_timer);
		else
			del_timer_sync(&voldownkey_timer);
	}

	return false;
}
static void volkey_smlpress_timer(unsigned long _data)
{
	struct _volkey_stat *data = (struct _volkey_stat *)_data;
	unsigned long flags;

	spin_lock_irqsave(&volkey_stat_lock, flags);
	if(data[0].state != data[1].state) {
		if(false == data[0].state) {
			KEY_LOG_PRINT("ignored vol up key.\n");
			data[0].ignore = true;
		} else {
			KEY_LOG_PRINT("ignored vol dn key.\n");
			data[1].ignore = true;
		}
	} else {
		data[0].ignore =false;
		data[1].ignore =false;
	}
	spin_unlock_irqrestore(&volkey_stat_lock, flags);
}

struct wakeup_source gpio_wake_src;
struct wakeup_source chattering_wake_src;

static void gpio_keys_gpio_report_event(struct gpio_button_data *bdata)
{
#ifdef QUALCOMM_ORIGINAL_FEATURE
	const struct gpio_keys_button *button = bdata->button;
#else
	struct gpio_keys_button *button = bdata->button;
	bool is_on;
#endif
	struct input_dev *input = bdata->input;
	unsigned int type = button->type ?: EV_KEY;
	int state;

	state = gpiod_get_value_cansleep(bdata->gpiod);
	if (state < 0) {
		dev_err(input->dev.parent,
			"failed to get gpio state: %d\n", state);
		return;
	}

#ifdef QUALCOMM_ORIGINAL_FEATURE
	if (g_kdm_check)
	{
		key_set_code(button->code);
	}
	else
	{
		if (type == EV_ABS) {
			if (state)
				input_event(input, type, button->code, button->value);
		} else {
			input_event(input, type, button->code, !!state);
		}
		input_sync(input);
	}
#else
		if(gpio_keys_smlpress_check(button->code, !!state)) return;

		KEY_LOG_PRINT("gpio_keys type:%x code:%x state:%x \n", type, button->code, state);
		if (state) {
			KEY_LOG_PRINT("gpio_keys on_cnt:%d off_cnt:%d \n", button->on_cnt, button->off_cnt);
			button->off_cnt = button->off_chattering_num;
			if (button->on_cnt) {
				KEY_LOG_PRINT("gpio_keys on_cnt-- \n");
				button->on_cnt--;
				mod_timer(&bdata->release_timer,
						jiffies + msecs_to_jiffies(bdata->software_debounce));
			}
			else {
				KEY_LOG_PRINT("gpio_keys set on \n");
				button->on_cnt = button->on_chattering_num;
				is_on = gpio_keys_is_stateon(button->code);
				gpio_keys_set_stateon(button->code);
				if (g_kdm_check) {
					key_set_code(button->code);
				}
				else if (!is_on) {
					KEY_LOG_PRINT_SEND("code:%x send on event \n", button->code);
					__pm_wakeup_event(&gpio_wake_src, MSEC_PER_SEC);
					input_event(input, type, button->code, !!state);
					input_sync(input);
				}
			}
		} else {
			KEY_LOG_PRINT("gpio_keys on_cnt:%d off_cnt:%d \n", button->on_cnt, button->off_cnt);
			button->on_cnt = button->on_chattering_num;
			if (button->off_cnt){
				KEY_LOG_PRINT("gpio_keys off_cnt-- \n");
				button->off_cnt--;
				mod_timer(&bdata->release_timer,
						jiffies + msecs_to_jiffies(bdata->software_debounce));
			}
			else{
				KEY_LOG_PRINT("gpio_keys set off \n");
				button->off_cnt = button->off_chattering_num;
				is_on = gpio_keys_is_stateon(button->code);
				gpio_keys_set_stateoff(button->code);
				if (g_kdm_check) {
					KEY_LOG_PRINT("release not key_set_code \n");
				}
				else if (is_on) {
					KEY_LOG_PRINT_SEND("code:%x send off event \n", button->code);
					__pm_wakeup_event(&gpio_wake_src, MSEC_PER_SEC);
					input_event(input, type, button->code, !!state);
					input_sync(input);
				}
			}
	}
#endif
}

static void gpio_keys_gpio_work_func(struct work_struct *work)
{
	struct gpio_button_data *bdata =
		container_of(work, struct gpio_button_data, work.work);

	gpio_keys_gpio_report_event(bdata);

	if (bdata->button->wakeup)
		pm_relax(bdata->input->dev.parent);
}

static void gpio_keys_gpio_timer(unsigned long _data)
{
	struct gpio_button_data *bdata = (struct gpio_button_data *)_data;

	schedule_delayed_work(&bdata->work, msecs_to_jiffies(0));
}

static irqreturn_t gpio_keys_gpio_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;

	BUG_ON(irq != bdata->irq);

	if (bdata->button->wakeup) {
		pm_stay_awake(bdata->input->dev.parent);
		__pm_wakeup_event(&chattering_wake_src, MSEC_PER_SEC);
	}

	mod_delayed_work(system_wq,
			 &bdata->work,
			 msecs_to_jiffies(bdata->software_debounce));

	return IRQ_HANDLED;
}

static void gpio_keys_irq_timer(unsigned long _data)
{
	struct gpio_button_data *bdata = (struct gpio_button_data *)_data;
	struct input_dev *input = bdata->input;
	unsigned long flags;

	spin_lock_irqsave(&bdata->lock, flags);
	if (bdata->key_pressed) {
		input_event(input, EV_KEY, bdata->button->code, 0);
		input_sync(input);
		bdata->key_pressed = false;
	}
	spin_unlock_irqrestore(&bdata->lock, flags);
}

static irqreturn_t gpio_keys_irq_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;
	const struct gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned long flags;

	BUG_ON(irq != bdata->irq);

	spin_lock_irqsave(&bdata->lock, flags);

	if (!bdata->key_pressed) {
		if (bdata->button->wakeup)
			pm_wakeup_event(bdata->input->dev.parent, 0);

		input_event(input, EV_KEY, button->code, 1);
		input_sync(input);

		if (!bdata->release_delay) {
			input_event(input, EV_KEY, button->code, 0);
			input_sync(input);
			goto out;
		}

		bdata->key_pressed = true;
	}

	if (bdata->release_delay)
		mod_timer(&bdata->release_timer,
			jiffies + msecs_to_jiffies(bdata->release_delay));
out:
	spin_unlock_irqrestore(&bdata->lock, flags);
	return IRQ_HANDLED;
}

static void gpio_keys_quiesce_key(void *data)
{
	struct gpio_button_data *bdata = data;

	if (bdata->gpiod)
		cancel_delayed_work_sync(&bdata->work);
	else
		del_timer_sync(&bdata->release_timer);
}

static int gpio_keys_setup_key(struct platform_device *pdev,
				struct input_dev *input,
				struct gpio_button_data *bdata,
#ifdef QUALCOMM_ORIGINAL_FEATURE
				const struct gpio_keys_button *button)
#else
				struct gpio_keys_button *button)
#endif
{
	const char *desc = button->desc ? button->desc : "gpio_keys";
	struct device *dev = &pdev->dev;
	irq_handler_t isr;
	unsigned long irqflags;
	int irq;
	int error;

	bdata->input = input;
	bdata->button = button;
	spin_lock_init(&bdata->lock);

	/*
	 * Legacy GPIO number, so request the GPIO here and
	 * convert it to descriptor.
	 */
	if (gpio_is_valid(button->gpio)) {
		unsigned flags = GPIOF_IN;

		if (button->active_low)
			flags |= GPIOF_ACTIVE_LOW;

		error = devm_gpio_request_one(&pdev->dev, button->gpio, flags,
					      desc);
		if (error < 0) {
			dev_err(dev, "Failed to request GPIO %d, error %d\n",
				button->gpio, error);
			return error;
		}

		bdata->gpiod = gpio_to_desc(button->gpio);
		if (!bdata->gpiod)
			return -EINVAL;

		if (button->debounce_interval) {
			error = gpiod_set_debounce(bdata->gpiod,
					button->debounce_interval * 1000);
			/* use timer if gpiolib doesn't provide debounce */
			if (error < 0)
				bdata->software_debounce =
						button->debounce_interval;
		}

		if (button->irq) {
			bdata->irq = button->irq;
		} else {
			irq = gpiod_to_irq(bdata->gpiod);
			if (irq < 0) {
				error = irq;
				dev_err(dev,
					"Unable to get irq number for GPIO %d, error %d\n",
					button->gpio, error);
				return error;
			}
			bdata->irq = irq;
		}

		INIT_DELAYED_WORK(&bdata->work, gpio_keys_gpio_work_func);
		setup_timer(&bdata->release_timer,
			    gpio_keys_gpio_timer, (unsigned long)bdata);

		isr = gpio_keys_gpio_isr;
		irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;

	} else {
		if (!button->irq) {
			dev_err(dev, "No IRQ specified\n");
			return -EINVAL;
		}
		bdata->irq = button->irq;

		if (button->type && button->type != EV_KEY) {
			dev_err(dev, "Only EV_KEY allowed for IRQ buttons.\n");
			return -EINVAL;
		}

		bdata->release_delay = button->debounce_interval;
		setup_timer(&bdata->release_timer,
			    gpio_keys_irq_timer, (unsigned long)bdata);

		isr = gpio_keys_irq_isr;
		irqflags = 0;
	}

	input_set_capability(input, button->type ?: EV_KEY, button->code);

	/*
	 * Install custom action to cancel release timer and
	 * workqueue item.
	 */
	error = devm_add_action(&pdev->dev, gpio_keys_quiesce_key, bdata);
	if (error) {
		dev_err(&pdev->dev,
			"failed to register quiesce action, error: %d\n",
			error);
		return error;
	}

	/*
	 * If platform has specified that the button can be disabled,
	 * we don't want it to share the interrupt line.
	 */
	if (!button->can_disable)
		irqflags |= IRQF_SHARED;

	error = devm_request_any_context_irq(&pdev->dev, bdata->irq,
					     isr, irqflags, desc, bdata);
	if (error < 0) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
			bdata->irq, error);
		return error;
	}

	return 0;
}

static void gpio_keys_report_state(struct gpio_keys_drvdata *ddata)
{
	struct input_dev *input = ddata->input;
	int i;

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];
		if (bdata->gpiod)
			gpio_keys_gpio_report_event(bdata);
	}
	input_sync(input);
}

static int gpio_keys_open(struct input_dev *input)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(input);
	const struct gpio_keys_platform_data *pdata = ddata->pdata;
	int error;

	if (pdata->enable) {
		error = pdata->enable(input->dev.parent);
		if (error)
			return error;
	}

	/* Report current state of buttons that are connected to GPIOs */
	gpio_keys_report_state(ddata);

	return 0;
}

static void gpio_keys_close(struct input_dev *input)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(input);
	const struct gpio_keys_platform_data *pdata = ddata->pdata;

	if (pdata->disable)
		pdata->disable(input->dev.parent);
}

/*
 * Handlers for alternative sources of platform_data
 */

#ifdef CONFIG_OF
/*
 * Translate OpenFirmware node properties into platform_data
 */
static struct gpio_keys_platform_data *
gpio_keys_get_devtree_pdata(struct device *dev)
{
	struct device_node *node, *pp;
	struct gpio_keys_platform_data *pdata;
	struct gpio_keys_button *button;
	int error;
	int nbuttons;
	int i;

	node = dev->of_node;
	if (!node)
		return ERR_PTR(-ENODEV);

	nbuttons = of_get_available_child_count(node);
	if (nbuttons == 0)
		return ERR_PTR(-ENODEV);

	pdata = devm_kzalloc(dev,
			     sizeof(*pdata) + nbuttons * sizeof(*button),
			     GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->buttons = (struct gpio_keys_button *)(pdata + 1);
	pdata->nbuttons = nbuttons;

	pdata->rep = !!of_get_property(node, "autorepeat", NULL);

	of_property_read_string(node, "label", &pdata->name);

	i = 0;
	for_each_available_child_of_node(node, pp) {
		enum of_gpio_flags flags;

		button = &pdata->buttons[i++];

		button->gpio = of_get_gpio_flags(pp, 0, &flags);
		if (button->gpio < 0) {
			error = button->gpio;
			if (error != -ENOENT) {
				if (error != -EPROBE_DEFER)
					dev_err(dev,
						"Failed to get gpio flags, error: %d\n",
						error);
				return ERR_PTR(error);
			}
		} else {
			button->active_low = flags & OF_GPIO_ACTIVE_LOW;
		}

		button->irq = irq_of_parse_and_map(pp, 0);

		if (!gpio_is_valid(button->gpio) && !button->irq) {
			dev_err(dev, "Found button without gpios or irqs\n");
			return ERR_PTR(-EINVAL);
		}

		if (of_property_read_u32(pp, "linux,code", &button->code)) {
			dev_err(dev, "Button without keycode: 0x%x\n",
				button->gpio);
			return ERR_PTR(-EINVAL);
		}else{
			gpio_keys_chattering_data[num_chattering_keys].code = button->code;
			gpio_keys_chattering_data[num_chattering_keys].is_on = false;
			num_chattering_keys++;
		}

		button->desc = of_get_property(pp, "label", NULL);

		if (of_property_read_u32(pp, "linux,input-type", &button->type))
			button->type = EV_KEY;

		button->wakeup = of_property_read_bool(pp, "wakeup-source") ||
				 /* legacy name */
				 of_property_read_bool(pp, "gpio-key,wakeup");

		button->can_disable = !!of_get_property(pp, "linux,can-disable", NULL);

		if (of_property_read_u32(pp, "debounce-interval",
					 &button->debounce_interval))
			button->debounce_interval = 5;

		if (of_property_read_u32(pp, "on_chattering_num",
					&button->on_chattering_num)){
			button->on_chattering_num = 1;
		}
		button->on_cnt = button->on_chattering_num;

		KEY_LOG_PRINT("code:%d reg on_chattering_num:%d on_cnt:%d \n", button->code, button->on_chattering_num, button->on_cnt);

		if (of_property_read_u32(pp, "off_chattering_num",
					&button->off_chattering_num)){
			button->off_chattering_num = 0;
		}
		button->off_cnt = button->off_chattering_num;

		KEY_LOG_PRINT("code:%d reg off_chattering_num:%d off_cnt:%d \n", button->code, button->off_chattering_num, button->off_cnt);
	}

	if (pdata->nbuttons == 0)
		return ERR_PTR(-EINVAL);

	return pdata;
}

static const struct of_device_id gpio_keys_of_match[] = {
	{ .compatible = "gpio-keys", },
	{ },
};
MODULE_DEVICE_TABLE(of, gpio_keys_of_match);

#else

static inline struct gpio_keys_platform_data *
gpio_keys_get_devtree_pdata(struct device *dev)
{
	return ERR_PTR(-ENODEV);
}

#endif

static int gpio_keys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
#ifdef QUALCOMM_ORIGINAL_FEATURE
	const struct gpio_keys_platform_data *pdata = dev_get_platdata(dev);
#else
	struct gpio_keys_platform_data *pdata = dev_get_platdata(dev);
#endif
	struct gpio_keys_drvdata *ddata;
	struct input_dev *input;
	size_t size;
	int i, error;
	int wakeup = 0;

	wakeup_source_init(&gpio_wake_src, "gpio_keys_wake_src");
	wakeup_source_init(&chattering_wake_src, "chattering_keys_wake_src");

	if (!pdata) {
		pdata = gpio_keys_get_devtree_pdata(dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	size = sizeof(struct gpio_keys_drvdata) +
			pdata->nbuttons * sizeof(struct gpio_button_data);
	ddata = devm_kzalloc(dev, size, GFP_KERNEL);
	if (!ddata) {
		dev_err(dev, "failed to allocate state\n");
		return -ENOMEM;
	}

	input = devm_input_allocate_device(dev);
	if (!input) {
		dev_err(dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	ddata->pdata = pdata;
	ddata->input = input;
	mutex_init(&ddata->disable_lock);

	platform_set_drvdata(pdev, ddata);
	input_set_drvdata(input, ddata);

	input->name = pdata->name ? : pdev->name;
	input->phys = "gpio-keys/input0";
	input->dev.parent = &pdev->dev;
	input->open = gpio_keys_open;
	input->close = gpio_keys_close;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);

	for (i = 0; i < pdata->nbuttons; i++) {
#ifdef QUALCOMM_ORIGINAL_FEATURE
		const struct gpio_keys_button *button = &pdata->buttons[i];
#else
		struct gpio_keys_button *button = &pdata->buttons[i];
#endif
		struct gpio_button_data *bdata = &ddata->data[i];

		error = gpio_keys_setup_key(pdev, input, bdata, button);
		if (error)
			return error;

		if (button->wakeup)
			wakeup = 1;
	}

	if(of_property_read_bool(dev->of_node, "enable,smlpress")) {
		spin_lock_init(&volkey_stat_lock);
		setup_timer(&volupkey_timer,
			volkey_smlpress_timer, (unsigned long)&volkey_stat);
		setup_timer(&voldownkey_timer,
			volkey_smlpress_timer, (unsigned long)&volkey_stat);
		smlpress_timer_init = true;
	}

	error = sysfs_create_group(&pdev->dev.kobj, &gpio_keys_attr_group);
	if (error) {
		dev_err(dev, "Unable to export keys/switches, error: %d\n",
			error);
		return error;
	}

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n",
			error);
		goto err_remove_group;
	}

	device_init_wakeup(&pdev->dev, wakeup);

	return 0;

err_remove_group:
	sysfs_remove_group(&pdev->dev.kobj, &gpio_keys_attr_group);
	return error;
}

static int gpio_keys_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &gpio_keys_attr_group);

	device_init_wakeup(&pdev->dev, 0);

	wakeup_source_trash(&gpio_wake_src);
	wakeup_source_trash(&chattering_wake_src);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int gpio_keys_suspend(struct device *dev)
{
	struct gpio_keys_drvdata *ddata = dev_get_drvdata(dev);
	struct input_dev *input = ddata->input;
	int i;

	if (device_may_wakeup(dev)) {
		for (i = 0; i < ddata->pdata->nbuttons; i++) {
			struct gpio_button_data *bdata = &ddata->data[i];
			if (bdata->button->wakeup)
				enable_irq_wake(bdata->irq);
		}
	} else {
		mutex_lock(&input->mutex);
		if (input->users)
			gpio_keys_close(input);
		mutex_unlock(&input->mutex);
	}

	return 0;
}

static int gpio_keys_resume(struct device *dev)
{
	struct gpio_keys_drvdata *ddata = dev_get_drvdata(dev);
	struct input_dev *input = ddata->input;
	int error = 0;
	int i;

	if (device_may_wakeup(dev)) {
		for (i = 0; i < ddata->pdata->nbuttons; i++) {
			struct gpio_button_data *bdata = &ddata->data[i];
			if (bdata->button->wakeup)
				disable_irq_wake(bdata->irq);
		}
	} else {
		mutex_lock(&input->mutex);
		if (input->users)
			error = gpio_keys_open(input);
		mutex_unlock(&input->mutex);
	}

	if (error)
		return error;

	gpio_keys_report_state(ddata);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(gpio_keys_pm_ops, gpio_keys_suspend, gpio_keys_resume);

static struct platform_driver gpio_keys_device_driver = {
	.probe		= gpio_keys_probe,
	.remove		= gpio_keys_remove,
	.driver		= {
		.name	= "gpio-keys",
		.pm	= &gpio_keys_pm_ops,
		.of_match_table = of_match_ptr(gpio_keys_of_match),
	}
};

static int __init gpio_keys_init(void)
{
	return platform_driver_register(&gpio_keys_device_driver);
}

static void __exit gpio_keys_exit(void)
{
	platform_driver_unregister(&gpio_keys_device_driver);
}

late_initcall(gpio_keys_init);
module_exit(gpio_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Phil Blundell <pb@handhelds.org>");
MODULE_DESCRIPTION("Keyboard driver for GPIOs");
MODULE_ALIAS("platform:gpio-keys");
