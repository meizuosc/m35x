/* drivers/modem/mx2_spt_modemctl.c
 *
 * Copyright (C) 20130 Meizu, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/gpio.h>
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif

#include <asm/mach-types.h>

#include <mach/gpio.h>
#include <mach/modem_sc8803g.h>

static char spt_modem_event_str[8][25] = {
	"SPT_MODEM_EVENT_POWEROFF",
	"SPT_MODEM_EVENT_RESET",
	"SPT_MODEM_EVENT_CRASH",
	"SPT_MODEM_EVENT_DUMP",
	"SPT_MODEM_EVENT_CONN",
	"SPT_MODEM_EVENT_DISCONN",
	"SPT_MODEM_EVENT_SIM",
	"SPT_MODEM_EVENT_BOOT_INIT",
};

struct spt_modem_ctl{
	unsigned short power_on;		/* modem-on gpio */
	unsigned short modem_alive;	/* modem-alive gpio */

	unsigned short power_status;		/* modem-off gpio */
	unsigned short usb_download;		/* usb download enable gpio */
	unsigned short mrts;
	unsigned short mrsd;
	unsigned short mrdy;
	
	int modem_crash_irq;
	int cp_flag;
	int cp_user_reset;
	int cp_usb_download;
	
	wait_queue_head_t  read_wq;
	struct completion done;
#ifdef CONFIG_HAS_WAKELOCK
	struct wake_lock   spt_modem_wakelock;
#endif
};

static bool spt_modem_debug = 0;
static struct spt_modem_ctl *spt_modem_global_mc = NULL;
static struct blocking_notifier_head mc_notifier_list = BLOCKING_NOTIFIER_INIT(mc_notifier_list);

DEFINE_SEMAPHORE(spt_modem_downlock);

static int __init spt_modem_debug_setup(char *args)
{
	int error;
	unsigned long val;

	error = strict_strtoul(args, 0, &val);
	if (!error)
		spt_modem_debug = val;

	return error;
}
__setup("spt_modem_debug=", spt_modem_debug_setup);

static ssize_t show_spt_modem_debug(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *p = buf;

	p += sprintf(buf, "spt_modem_debug=%d\n", spt_modem_debug);

	return p - buf;
}
static ssize_t store_spt_modem_debug(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int error;
	unsigned long val;

	error = strict_strtoul(buf, 10, &val);
	if (!error)
		spt_modem_debug = val;

	return count;
}
static struct device_attribute attr_spt_modem_debug = __ATTR(spt_modem_debug,
		S_IRUGO | S_IWUSR, show_spt_modem_debug, store_spt_modem_debug);

static ssize_t show_spt_usb_download(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *p = buf;

	p += sprintf(buf, "usb_download=%d\n", spt_modem_global_mc->cp_usb_download);

	return p - buf;
}
static ssize_t store_spt_usb_download(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int error;
	unsigned long val;

	error = strict_strtoul(buf, 10, &val);
	if (!error && spt_modem_global_mc){
		gpio_set_value(spt_modem_global_mc->usb_download, !val);
		spt_modem_global_mc->cp_usb_download = !!val;
		pr_info("%s: %s usb download\n", __func__, !!val?"enable":"disable");
	}
	return count;
}

static struct device_attribute attr_spt_usb_download = __ATTR(spt_usb_download,
		S_IRUGO | S_IWUSR, show_spt_usb_download, store_spt_usb_download);

#ifdef CONFIG_HAS_WAKELOCK
static void spt_modem_wake_lock_initial(struct spt_modem_ctl *mc)
{
	wake_lock_init(&mc->spt_modem_wakelock, WAKE_LOCK_SUSPEND, "modemctl");
}

static void spt_modem_wake_lock_destroy(struct spt_modem_ctl *mc)
{
	wake_lock_destroy(&mc->spt_modem_wakelock);
}

#if 0
static void spt_modem_wake_lock(struct spt_modem_ctl *mc)
{
	wake_lock(&mc->spt_modem_wakelock);
}
#endif

static void spt_modem_wake_lock_timeout(struct spt_modem_ctl *mc,  int timeout)
{
	wake_lock_timeout(&mc->spt_modem_wakelock, timeout);
}
#else
static void spt_modem_wake_lock_initial(struct spt_modem_ctl *mc){}

static void spt_modem_wake_lock_destroy(struct spt_modem_ctl *mc){}

//static void spt_modem_wake_lock(struct spt_modem_ctl *mc){}

static void spt_modem_wake_lock_timeout(struct spt_modem_ctl *mc,  int timeout){}
#endif
void spt_modem_notify_event(int type)
{
	if (!spt_modem_global_mc)
		return;

	switch (type) {
		case SPT_MODEM_EVENT_POWEROFF:
			spt_modem_global_mc->cp_flag &= SPT_MODEM_SIM_DETECT_FLAG;
			wake_up_interruptible(&spt_modem_global_mc->read_wq);
			break;

		case SPT_MODEM_EVENT_RESET:
			spt_modem_global_mc->cp_flag |= SPT_MODEM_RESET_FLAG;
			wake_up_interruptible(&spt_modem_global_mc->read_wq);
			break;

		case SPT_MODEM_EVENT_CRASH:
			spt_modem_global_mc->cp_flag |= SPT_MODEM_CRASH_FLAG;
			wake_up_interruptible(&spt_modem_global_mc->read_wq);
			break;

		case SPT_MODEM_EVENT_CONN:
			spt_modem_global_mc->cp_flag &= ~(SPT_MODEM_DISCONNECT_FLAG);
			spt_modem_global_mc->cp_flag |= SPT_MODEM_CONNECT_FLAG;
			wake_up_interruptible(&spt_modem_global_mc->read_wq);
			break;

		case SPT_MODEM_EVENT_DISCONN:
			spt_modem_global_mc->cp_flag &= ~(SPT_MODEM_CONNECT_FLAG);
			spt_modem_global_mc->cp_flag |= SPT_MODEM_DISCONNECT_FLAG;
			wake_up_interruptible(&spt_modem_global_mc->read_wq);
			break;

		case SPT_MODEM_EVENT_SIM:
			spt_modem_global_mc->cp_flag |= SPT_MODEM_SIM_DETECT_FLAG;
			spt_modem_global_mc->cp_flag |= SPT_MODEM_CRASH_FLAG;
			wake_up_interruptible(&spt_modem_global_mc->read_wq);
			break;

		case SPT_MODEM_EVENT_BOOT_INIT:
			spt_modem_global_mc->cp_flag =
				SPT_MODEM_RESET_FLAG | SPT_MODEM_INIT_ON_FLAG;
			wake_up_interruptible(&spt_modem_global_mc->read_wq);
			break;
	}

	pr_info("%s:%s, cp_flag:0x%x\n", __func__, spt_modem_event_str[type],
							spt_modem_global_mc->cp_flag);
}

static int spt_sc8803g_on(struct spt_modem_ctl *mc)
{
	pr_info("spt_sc8803g_on()\n");

	gpio_set_value(mc->power_on, 1);
	spt_modem_wake_lock_timeout(mc, 10 * HZ);
	return 0;
}

static int spt_sc8803g_off(struct spt_modem_ctl *mc)
{
	pr_info("spt_sc8803g_off()\n");
	gpio_set_value(mc->power_on, 0);
	spt_modem_wake_lock_timeout(mc, 10 * HZ);

	return 0;
}

static int __spt_sc8803g_reset(struct spt_modem_ctl *mc)
{
	int count = 50;
	pr_info("spt_sc8803g_reset(%d)\n", gpio_get_value(mc->power_status));

	gpio_set_value(mc->power_on, 0);
	while(gpio_get_value(mc->power_status) && count){
		msleep(10);
		count --;
	}
	msleep(10);
	gpio_set_value(mc->power_on, 1);
	spt_modem_wake_lock_timeout(mc, 10*HZ);
	return 0;
}

static void __spt_sc8803g_reset_io(struct spt_modem_ctl *mc)
{
	/* Restore the I/O pin state when resetting the modem */
	gpio_set_value(mc->mrts, 0);
	gpio_set_value(mc->mrsd, 0);
	gpio_set_value(mc->mrdy, 1);
	gpio_set_value(mc->usb_download, 1);
}

void spt_sc8803g_reset(struct spt_modem_ctl *mc)
{
	init_completion(&mc->done);
	mc->cp_flag = 0;
	mc->cp_user_reset = 1;
	__spt_sc8803g_reset(mc);
	__spt_sc8803g_reset_io(mc);
	wait_for_completion_timeout(&mc->done, 20*HZ);
	mc->cp_user_reset = 0;
	spt_modem_notify_event(SPT_MODEM_EVENT_RESET);
}

static irqreturn_t spt_modem_cpcrash_irq(int irq, void *dev_id)
{
	struct spt_modem_ctl *mc = (struct spt_modem_ctl *)dev_id;
	int val;

	val = gpio_get_value(mc->modem_alive);
	if(!val){
		spt_modem_wake_lock_timeout(mc, HZ * 30);
		if(!mc->cp_user_reset)
			spt_modem_notify_event(SPT_MODEM_EVENT_CRASH);
	}else{
		complete(&mc->done);
	}
	
	blocking_notifier_call_chain(&mc_notifier_list, val, NULL);
	pr_info("%s CP_CRASH_INT:%d\n",  __func__, val);

	return IRQ_HANDLED;
}

int spt_modem_open(struct inode *inode, struct file *file)
{
	file->private_data = spt_modem_global_mc;
	return 0;
}

int spt_modem_close (struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static ssize_t
spt_modem_write(struct file *filp, const char __user *buffer, size_t count,
							loff_t *offset)
{
	struct spt_modem_ctl *mc = (struct spt_modem_ctl *)filp->private_data;

	pr_info("%s:%s\n", __func__, buffer);

	/*power off modem*/
	if(count >= 3 && !strncmp(buffer, "off", 3))
	{
		if (down_interruptible(&spt_modem_downlock) == 0) {
			spt_sc8803g_off(mc);
			up(&spt_modem_downlock);
		}
	}
	/*power on modem*/
	if(count >= 2 && !strncmp(buffer, "on", 2)) {
		if (down_interruptible(&spt_modem_downlock) == 0) {
			spt_sc8803g_on(mc);
			up(&spt_modem_downlock);
		}
	}

	/*reset modem*/
	if(count >= 5 && !strncmp(buffer, "reset", 5)) {
		if (down_interruptible(&spt_modem_downlock) == 0) {
			if(!mc->cp_usb_download)
				spt_sc8803g_reset(mc);
			up(&spt_modem_downlock);
		}
	}

	/*clean modem flags*/
	if(count >= 5 && !strncmp(buffer, "clean", 5)) {
		if (down_interruptible(&spt_modem_downlock) == 0) {
			mc->cp_flag &= SPT_MODEM_CONNECT_FLAG;
			up(&spt_modem_downlock);
		}
	}

	if(count >= 7 && !strncmp(buffer, "debug=", 6)) {
		int error;
		unsigned long val;

		error = strict_strtoul(buffer + 6, 10, &val);
		if (!error)
			spt_modem_debug = val;
	}

	return count;
}

static ssize_t
spt_modem_read(struct file *filp, char __user * buffer, size_t count,
							loff_t * offset)
{
	struct spt_modem_ctl *mc = (struct spt_modem_ctl *)filp->private_data;
	int flag = 0;

	wait_event_interruptible(mc->read_wq,
			(mc->cp_flag & SPT_MODEM_EVENT_MASK));

	flag = mc->cp_flag & SPT_MODEM_EVENT_MASK;
	if(copy_to_user(buffer, &flag, sizeof(flag)))
		return -EFAULT;
	pr_info("%s: modem event = 0x%x\n", __func__, flag);

	mc->cp_flag &= SPT_MODEM_CONNECT_FLAG;

	return 1;
}

long spt_modem_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
	pr_info("%s cmd:0x%x, arg:0x%lx\n", __func__, cmd, arg);

	return 0;
}

unsigned int spt_modem_poll (struct file *filp, struct poll_table_struct *wait)
{
	u32 mask = 0;
	struct spt_modem_ctl *mc = (struct spt_modem_ctl *)filp->private_data;

	if (mc->cp_flag & SPT_MODEM_CONNECT_FLAG)
		mask = POLLIN | POLLOUT | POLLRDNORM;
	else
		poll_wait(filp, &mc->read_wq, wait);

	return mask;
}

static struct file_operations spt_modem_file_ops = {
	.owner          = THIS_MODULE,
	.poll           = spt_modem_poll,
	.open           = spt_modem_open,
	.read           = spt_modem_read,
	.write          = spt_modem_write,
	.release        = spt_modem_close,
	.unlocked_ioctl = spt_modem_ioctl,
};

static struct miscdevice spt_modem_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "modemctl",
	.fops  = &spt_modem_file_ops
};

int register_spt_modem_crash_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&mc_notifier_list, nb);

}
int unregister_spt_modem_crash_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&mc_notifier_list, nb);
}
static int spt_modem_probe(struct platform_device *pdev)
{
	int ret;
	struct spt_modem_ctl *mc;
	struct spt_modem_platform_data *pdata;

	pdata = pdev->dev.platform_data;
	if(pdata == NULL){
		dev_err(&pdev->dev, "no platform data\n");
		return -ENOMEM;
	}
	mc = kzalloc(sizeof(struct spt_modem_ctl), GFP_KERNEL);
	if(mc == NULL){
		dev_err(&pdev->dev, "kmalloc fail\n");
		return -ENOMEM;
	}

	init_completion(&mc->done);
	init_waitqueue_head(&mc->read_wq);
	spt_modem_wake_lock_initial(mc);

	mc->power_on = pdata->pwr_on;
	mc->modem_alive = pdata->salive;
	mc->power_status = pdata->s2m1;
	mc->usb_download = pdata->m2s1;
	mc->mrts = pdata->mrts;
	mc->mrsd = pdata->mrsd;
	mc->mrdy = pdata->mrdy;
	mc->cp_user_reset = 0;
	mc->cp_usb_download = 0;
	/*reset irq*/
	mc->modem_crash_irq = gpio_to_irq(mc->modem_alive);
	ret = request_threaded_irq(mc->modem_crash_irq, NULL,
		spt_modem_cpcrash_irq, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
		"CP_CRASH_INT", mc);
	if (ret) {
		pr_err("Failed register gpio_cp_reset_int irq(%d)!\n",
				mc->modem_crash_irq);
		goto err1;
	}
	ret = enable_irq_wake(mc->modem_crash_irq);
	if (ret) {
		pr_err("failed to enable_irq_wake of modem reset:%d\n",
				ret);
		goto err2;
	}
	ret = misc_register(&spt_modem_miscdev);
	if(ret) {
		pr_err("Failed to register modem control device\n");
		goto err3;
	}
	ret = device_create_file(spt_modem_miscdev.this_device, &attr_spt_modem_debug);
	if (ret) {
		pr_err("failed to create sysfs file:attr_modem_debug!\n");
		goto err4;
	}

	ret = device_create_file(spt_modem_miscdev.this_device, &attr_spt_usb_download);
	if (ret) {
		pr_err("failed to create sysfs file:attr_spt_usb_download!\n");
		goto err4;
	}
	platform_set_drvdata(pdev, mc);

	spt_modem_global_mc = mc;

	/*power on modem*/
	spt_sc8803g_on(mc);
	return 0;
err4:
	misc_deregister(&spt_modem_miscdev);
err3:
	disable_irq_wake(mc->modem_crash_irq);
err2:
	free_irq(mc->modem_crash_irq, NULL);
err1:
	kfree(mc);
	return ret;
}

static int spt_modem_remove(struct platform_device *pdev)
{
	struct spt_modem_ctl *mc = platform_get_drvdata(pdev);

	device_remove_file(spt_modem_miscdev.this_device, &attr_spt_modem_debug);
	disable_irq_wake(mc->modem_crash_irq);
	free_irq(mc->modem_crash_irq, NULL);
	spt_modem_wake_lock_destroy(mc);
	kfree(mc);
	return 0;
}

static int spt_modem_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int spt_modem_resume(struct platform_device *pdev)
{
	return 0;
}

static void spt_modem_shutdown(struct platform_device *pdev)
{
	struct spt_modem_ctl *mc = platform_get_drvdata(pdev);

	pr_info("%s: modem power off!\r\n", __func__);
	gpio_set_value(mc->power_on, 0);
}

static struct platform_driver spt_modem_driver = {
	.driver = {
		.name	= "sc880xg-modem",
		.owner = THIS_MODULE,
	},
	.probe    = spt_modem_probe,
	.remove = spt_modem_remove,
	.suspend = spt_modem_suspend,
	.resume = spt_modem_resume,
	.shutdown = spt_modem_shutdown,
};

static int __init spt_modem_init(void)
{
	if (machine_is_m65())
		return 0;

	return platform_driver_register(&spt_modem_driver);
}
subsys_initcall(spt_modem_init);

static void __exit spt_modem_exit(void)
{
	platform_driver_unregister(&spt_modem_driver);
}
module_exit(spt_modem_exit);

MODULE_AUTHOR("Wenbin Wu <wenbinwu@meizu.com>");
MODULE_DESCRIPTION("SC8803G Modem Control Driver");
MODULE_LICENSE("GPL");

