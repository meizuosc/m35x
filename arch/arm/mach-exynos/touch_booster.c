#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/cpufreq.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/pm_qos.h>
#include <linux/fb.h>

#include <asm/mach-types.h>

#include <plat/cpu.h>

#include <mach/touch_booster.h>

#define DEFUALT_LAUNCH_BOOST_CPUFREQ   1200000	/* 1.2GHz */
#define DEFUALT_FIRST_BOOST_CPUFREQ	600000	/* 600Mhz */
#define DEFUALT_SECOND_BOOST_CPUFREQ	500000	/* 500Mhz */
#define DEFUALT_BOOST_TIME	(100 * 1000)	/* 100ms */
#define MAX_BOOST_DEVICE		2
#define TABLE_SIZE			3

static unsigned int boost_device_count = 0;
static struct pm_qos_request boost_cpu_qos;
static int touch_pre_value = -1, keys_pre_value = 0;
static unsigned int touch_request = 1, keys_request = 1;
static unsigned int multi_timeout = 1;  /* for screen off entend delay timeout*/
static int first_touch = 0;
static int boost_freq_table[TABLE_SIZE] = {DEFUALT_LAUNCH_BOOST_CPUFREQ, DEFUALT_FIRST_BOOST_CPUFREQ, DEFUALT_SECOND_BOOST_CPUFREQ};
static int boost_time_multi[TABLE_SIZE] = {2, 1, 1};
static int req_level = 1, prev_req_level = 1;

/* Boost for app launching */
#define APP_LAUNCH_BOOST_TIME_SECS	2
int app_is_launching = 0;
struct delayed_work cancel_boost_work;

static void cancel_boost_fn(struct work_struct *work)
{
	app_is_launching = 0;
	smp_mb();
	pr_debug("%s: @@@@@@@@@@@ app is not launching...\n", __func__);
}

static void start_boost(struct tb_private_info *info)
{
	struct cpufreq_policy *policy;
	unsigned int cur, target, target_level;
	unsigned int boost_time;

	policy = cpufreq_cpu_get(0);
	if (policy) {
		cur = policy->cur;
		cpufreq_cpu_put(policy);

		if (info->boost_debug)
			pr_info("%s ... current freq = %d\n", __func__, policy->cur);

		/* For touch events, boost to a higher freq at the first time,
		 * otherwise, keep at least 400M */
		target_level = first_touch ? req_level : 2;
		target = info->boost_cpufreq[target_level];

		if (cur < target) {
			boost_time = info->boost_time * boost_time_multi[target_level];
			pm_qos_update_request_timeout(&boost_cpu_qos, target, boost_time * multi_timeout);
			if (info->boost_debug)
				pr_info("%s: request %d cpu freq for %d msecs\n", __func__, target, boost_time * multi_timeout);
		}
	}
}

static void boost_fn(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct tb_private_info *info =
	    list_entry(dwork, struct tb_private_info, boost_work);
	start_boost(info);
}

static void keys_event(struct input_handle *handle,
		     unsigned int type, unsigned int code, int value)
{
	struct tb_private_info *info =
	    list_entry(handle, struct tb_private_info, handle);
	unsigned long delay;

	if (info->boost_debug) {
		pr_info("%s: type = %d, code = %d, value = %d\n", __func__, type, code, value);
		pr_info("%s: request = %d, pre_value = %d\n", __func__, keys_request, keys_pre_value);
	}

	/* Only speed up for KEY_HOME and KEY_POWER */
	if ((code != KEY_HOME) && (code != KEY_POWER) && (code != KEY_VOLUMEUP) && (code != KEY_VOLUMEDOWN) && (code != 0))
		return;

	first_touch = 0;

	/* When touch down, change request */
	if (keys_request == 1 && type != 0) {
		keys_request = 0;
		delay = (keys_pre_value != 0) ? 20 : 0;

		if (info->boost_debug)
			pr_info("%s: Schedule with delay = %ld", __func__, delay);

		queue_delayed_work(info->wq, &info->boost_work, msecs_to_jiffies(delay));
	}

	if (type == 1)
		keys_pre_value = value;
	else
		keys_request = 1;
}

static void touch_event(struct input_handle *handle,
		     unsigned int type, unsigned int code, int value)
{
	struct tb_private_info *info =
	    list_entry(handle, struct tb_private_info, handle);
	unsigned long delay;

	if (info->boost_debug) {
		pr_info("%s: type = %d, code = %d, value = %d\n", __func__, type, code, value);
		pr_info("%s: request = %d, pre_value = %d\n", __func__, touch_request, touch_pre_value);
	}

	/* When touch down, change request */
	if (touch_request == 1 && type != 0) {
		touch_request = 0;
		if (touch_pre_value != -1) {
			first_touch = 0;
			delay = 20;
		} else {
			delay = 0;
			first_touch = 1;
		}

		if (info->boost_debug)
			pr_info("%s: Schedule with delay = %ld", __func__, delay);

		queue_delayed_work(info->wq, &info->boost_work, msecs_to_jiffies(delay));
	}

	if (type == 3)
		touch_pre_value = value;
	else
		touch_request = 1;

}

static bool touch_match(struct input_handler *handler, struct input_dev *dev)
{
	if (test_bit(EV_ABS, dev->evbit) && test_bit(BTN_TOUCH, dev->keybit))
		return true;

	return false;
}

static bool keys_match(struct input_handler *handler, struct input_dev *dev)
{
	if (test_bit(KEY_POWER, dev->keybit))
		return true;

	return false;
}

static ssize_t set_boost_pulse(struct class *class,
			       struct class_attribute *attr,
			       const char *buf, size_t count)
{
	struct tb_private_info *info =
	    list_entry(class, struct tb_private_info, tb_class);
	unsigned int pulse = 0;

	sscanf(buf, "%u", &pulse);
	if (pulse)
		start_boost(info);
	return count;
}

static ssize_t get_boost_level(struct class *class,
				 struct class_attribute *attr, char *buf)
{
	struct tb_private_info *info =
	    list_entry(class, struct tb_private_info, tb_class);

	if (info->boost_debug)
		pr_info("get_boost_level: level=%d , boost_freq_table: [%d,%d,%d]\n", req_level, boost_freq_table[0], boost_freq_table[1], boost_freq_table[2]);

	return sprintf(buf, "%d\n", req_level);
}

static ssize_t set_boost_level(struct class *class,
				 struct class_attribute *attr,
				 const char *buf, size_t count)
{
	struct tb_private_info *info =
	    list_entry(class, struct tb_private_info, tb_class);
	unsigned int level = 0;

	sscanf(buf, "%u", &level);
	if (level < TABLE_SIZE)
		req_level = level;

	if (info->boost_debug)
		pr_info("set_boost_level: level=%d , boost_freq_table [%d,%d,%d]\n", req_level, boost_freq_table[0], boost_freq_table[1], boost_freq_table[2]);

	/* If switch from application to desktop, app is not launching */
	if (req_level == 0 && app_is_launching) {
		app_is_launching = 0;
		smp_mb();
		if (info->boost_debug)
			pr_info("%s: @@@@@@@@@@@ app is not launching...\n", __func__);

		if (delayed_work_pending(&cancel_boost_work))
			cancel_delayed_work_sync(&cancel_boost_work);
	}
	if (req_level == 1 && !app_is_launching) {
		app_is_launching = 1;
		smp_mb();
		if (info->boost_debug)
			pr_info("%s: @@@@@@@@@@@ app is launching, start the auto cancel workqueue...\n", __func__);
		schedule_delayed_work(&cancel_boost_work, HZ * APP_LAUNCH_BOOST_TIME_SECS);
	}
	prev_req_level = req_level;

	return count;
}

static ssize_t get_boost_debug(struct class *class,
			       struct class_attribute *attr, char *buf)
{
	struct tb_private_info *info =
	    list_entry(class, struct tb_private_info, tb_class);
	return sprintf(buf, "%d\n", info->boost_debug);
}

static ssize_t set_boost_debug(struct class *class,
			       struct class_attribute *attr,
			       const char *buf, size_t count)
{
	struct tb_private_info *info =
	    list_entry(class, struct tb_private_info, tb_class);
	unsigned int boost_debug = 0;

	sscanf(buf, "%u", &boost_debug);
	info->boost_debug = !!boost_debug;

	return count;
}

static ssize_t get_boost_time(struct class *class,
			     struct class_attribute *attr, char *buf)
{
	struct tb_private_info *info =
	    list_entry(class, struct tb_private_info, tb_class);
	return sprintf(buf, "%d ms\n", info->boost_time);
}

static ssize_t set_boost_time(struct class *class,
			     struct class_attribute *attr,
			     const char *buf, size_t count)
{
	struct tb_private_info *info =
	    list_entry(class, struct tb_private_info, tb_class);
	unsigned int time = 0;

	sscanf(buf, "%u", &time);

	if (time < 100)
		time = 100;

	info->boost_time = time;

	return count;
}

static struct class_attribute boost_class_attrs[] = {
	__ATTR(boost_debug, 0660, get_boost_debug, set_boost_debug),
	__ATTR(boost_level, 0660, get_boost_level, set_boost_level),
	__ATTR(boost_time, 0660, get_boost_time, set_boost_time),
	__ATTR(touch_pulse, 0660, NULL, set_boost_pulse),
	__ATTR_NULL
};

static int fb_state_change(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct fb_event *evdata = data;
	unsigned int blank;

	if (val != FB_EVENT_BLANK)
		return 0;
	blank = *(int *)evdata->data;

	switch (blank) {
	case FB_BLANK_POWERDOWN:
		multi_timeout = 3;
		break;
	case FB_BLANK_UNBLANK:
		multi_timeout = 1;
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block fb_block = {
	.notifier_call = fb_state_change,
};
static bool boost_fb_notifier = false;

static int boost_connect(struct input_handler *handler,
		      struct input_dev *dev, const struct input_device_id *id)
{
	int ret = 0;
	struct tb_private_info *info;

	if (boost_device_count >= MAX_BOOST_DEVICE)
		return -ENODEV;

	info = kzalloc(sizeof(struct tb_private_info), GFP_KERNEL);
	if (IS_ERR_OR_NULL(info))
		return PTR_ERR(info);

	info->wq = create_singlethread_workqueue(handler->name);
	if (!info->wq) {
		pr_err("%s: Failed to create_singlethread_workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_creat;
	}

	INIT_DELAYED_WORK(&info->boost_work, boost_fn);
	info->boost_time = DEFUALT_BOOST_TIME;
	info->boost_cpufreq = boost_freq_table;
	info->boost_debug = 0;
	info->tb_class.name = handler->name;
	info->tb_class.class_attrs = boost_class_attrs;
	info->handle.dev = dev;
	info->handle.handler = handler;
	info->handle.open = 0;

	if (boost_device_count == 0) {
		pm_qos_add_request(&boost_cpu_qos, PM_QOS_CPU_FREQ_MIN, 0);

		INIT_DELAYED_WORK(&cancel_boost_work, cancel_boost_fn);
	}

	ret = class_register(&info->tb_class);
	if (ret) {
		pr_err("%s: register sysdev performance error!\n", __func__);
		kfree(info);
		goto err_sys;
	}

	ret = input_register_handle(&info->handle);
	if (ret) {
		pr_err("%s: register input handler error!\n", __func__);
		goto err_reg;
	}
	ret = input_open_device(&info->handle);
	if (ret) {
		pr_err("%s: Failed to open input device, error %d\n",
		       __func__, ret);
		goto err_open;
	}

#ifdef CONFIG_EXYNOS5_DYNAMIC_CPU_HOTPLUG
	if (!boost_fb_notifier) {
		fb_register_client(&fb_block);
		multi_timeout = 1;
		boost_fb_notifier = true;
	}
#endif

	boost_device_count++;
	return ret;

 err_open:
	input_unregister_handle(&info->handle);
 err_reg:
	class_unregister(&info->tb_class);
 err_sys:
	destroy_workqueue(info->wq);
 err_creat:
	kfree(info);
	return ret;

}

static void boost_disconnect(struct input_handle *handle)
{
	struct tb_private_info *info =
	    list_entry(handle, struct tb_private_info, handle);

	destroy_workqueue(info->wq);
	input_close_device(handle);
	input_unregister_handle(handle);
}

static const struct input_device_id touch_boost_ids[] = {
	{
	 .flags = INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_KEYBIT,
	 .evbit = {BIT_MASK(EV_ABS)},
	 .keybit = {[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH)},
	 },
	{}
};

static const struct input_device_id keys_boost_ids[] = {
	{
	 .flags = INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_KEYBIT,
	 .evbit = {BIT_MASK(EV_SYN)},
	 .keybit = {[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER)},
	 },
	{}
};

static struct input_handler keys_boost_handler = {
	.name = "keys_booster",
	.event = keys_event,
	.match = keys_match,
	.connect = boost_connect,
	.disconnect = boost_disconnect,
	.id_table = keys_boost_ids,
};

static struct input_handler touch_boost_handler = {
	.name = "touch_booster",
	.event = touch_event,
	.match = touch_match,
	.connect = boost_connect,
	.disconnect = boost_disconnect,
	.id_table = touch_boost_ids,
};

static int __init boost_init(void)
{
	int err;

	err = input_register_handler(&keys_boost_handler);
	err |= input_register_handler(&touch_boost_handler);

	return err;
}

static void __exit boost_exit(void)
{
	input_unregister_handler(&keys_boost_handler);
	input_unregister_handler(&touch_boost_handler);
}

subsys_initcall(boost_init);
module_exit(boost_exit);

/* Module information */
MODULE_AUTHOR("Wenbin Wu <wenbinwu@meizu.com>");
MODULE_DESCRIPTION("Touch Boost driver");
MODULE_LICENSE("GPL");
