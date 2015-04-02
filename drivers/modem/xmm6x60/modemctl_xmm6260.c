/* drivers/modem/modemctl_device_xmm6260.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2010 Samsung Electronics.
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

#include <linux/delay.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/platform_data/modem.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>

#include <mach/gpio.h>
#include "modem_prj.h"

static DEFINE_SEMAPHORE(modem_downlock);

static ssize_t show_modem_debug(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *p = buf;

	p += sprintf(buf, "modem_debug=%d\n", modem_debug);

	return p - buf;
}
static ssize_t store_modem_debug(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int error;
	unsigned long val;

	error = strict_strtoul(buf, 10, &val);
	if (!error)
		modem_debug = val;

	return count;
}
static struct device_attribute attr_modem_debug = __ATTR(modem_debug,
		S_IRUGO | S_IWUSR, show_modem_debug, store_modem_debug);

static void xmm6260_state_changed(struct modem_ctl *mc, int type)
{
	struct link_device *ld = mc->ld;

	switch (type) {
		case MODEM_EVENT_POWEROFF:
			mc->phone_state &= MODEM_SIM_DETECT_FLAG;
			goto wakeup_read;

		case MODEM_EVENT_RESET:
			mc->phone_state |= MODEM_RESET_FLAG;
			goto wakeup_read;

		case MODEM_EVENT_CRASH:
			if(ld->enum_done) {
				mc->phone_state |= MODEM_CRASH_FLAG;
				goto wakeup_read;
			}
			return;

		case MODEM_EVENT_CONN:
			if(ld->enum_done) {
				mc->phone_state &= ~(MODEM_DISCONNECT_FLAG);
				mc->phone_state |= MODEM_CONNECT_FLAG;
				goto wakeup_poll;
			}
			return;

		case MODEM_EVENT_DISCONN:
			if(ld->enum_done) {
				mc->phone_state &= ~(MODEM_CONNECT_FLAG);
				mc->phone_state |= MODEM_DISCONNECT_FLAG;
				goto wakeup_read;
			}
			return;

		case MODEM_EVENT_SIM:
			mc->phone_state |= MODEM_SIM_DETECT_FLAG;
			mc->phone_state |= MODEM_CRASH_FLAG;
			goto wakeup_read;

		case MODEM_EVENT_BOOT_INIT:
			mc->phone_state =
				MODEM_RESET_FLAG | MODEM_INIT_ON_FLAG;
			goto wakeup_read;
		default:
			return;
	}

wakeup_read:
	wake_up_interruptible(&mc->read_wq);
	goto done;

wakeup_poll:
	wake_up_interruptible(&mc->conn_wq);
done:
	if (mc->phone_state & MODEM_EVENT_MASK)
		pr_info("mif: report modem event 0x%x called by %pF\n",
				mc->phone_state, __builtin_return_address(0));
}

static int xmm6260_on(struct modem_ctl *mc)
{
	MIF_INFO("xmm6260_on()\n");

	if (!mc->gpio_cp_reset || !mc->gpio_cp_on || !mc->gpio_reset_req_n) {
		MIF_ERR("no gpio data\n");
		return -ENXIO;
	}

	gpio_set_value(mc->gpio_cp_on, 0);
	gpio_set_value(mc->gpio_reset_req_n, 0);
	gpio_set_value(mc->gpio_cp_reset, 0);
	/* must be >500ms for CP can boot up under -20 degrees */
	msleep(500);
	gpio_set_value(mc->gpio_cp_reset, 1);
	mdelay(1);
	gpio_set_value(mc->gpio_reset_req_n, 1);
	mdelay(2);
	gpio_set_value(mc->gpio_cp_on, 1);
	udelay(80);
	gpio_set_value(mc->gpio_cp_on, 0);

	return 0;
}

static int xmm6260_off(struct modem_ctl *mc)
{
	MIF_INFO("xmm6260_off()\n");

	mc->phone_state &= MODEM_SIM_DETECT_FLAG;
	gpio_set_value(mc->gpio_cp_on, 0);
	gpio_set_value(mc->gpio_cp_reset, 0);
	gpio_set_value(mc->gpio_reset_req_n, 0);
	wake_lock_timeout(&mc->wakelock, 10 * HZ);

	return 0;
}

static int xmm6260_reset(struct modem_ctl *mc)
{
	MIF_INFO("xmm6260_reset()\n");
	if (!mc->gpio_cp_reset || !mc->gpio_reset_req_n)
		return -ENXIO;

	gpio_set_value(mc->gpio_reset_req_n, 0);
	gpio_set_value(mc->gpio_cp_reset, 0);
	msleep(20);

	gpio_set_value(mc->gpio_cp_reset, 1);
	udelay(160);

	gpio_set_value(mc->gpio_reset_req_n, 1);
	udelay(100);

	gpio_set_value(mc->gpio_cp_on, 1);
	udelay(80);
	gpio_set_value(mc->gpio_cp_on, 0);

	return 0;
}

static int xmm626x_main_modem(struct modem_ctl *mc, struct link_device *ld)
{
	struct completion done;

	wake_up_interruptible(&mc->read_wq);
	wake_lock(&mc->wakelock);
	ld->enum_done = false;
	mc->set_ehci_power(0);
	xmm6260_off(mc);
	msleep(100);
	mc->phone_state &= MODEM_SIM_DETECT_FLAG;
	xmm6260_on(mc);
	init_completion(&done);
	ld->l2_done = &done;
	wait_for_completion_timeout(&done, 20*HZ);
	ld->l2_done = NULL;
	mc->set_ehci_power(1);
	ld->enum_done = true;
	wake_lock_timeout(&mc->wakelock, 5 * HZ);

	return 0;
}

static int xmm626x_renum_modem(struct modem_ctl *mc, struct link_device *ld)
{
	wake_up_interruptible(&mc->read_wq);
	wake_lock(&mc->wakelock);
	ld->enum_done = false;
	mc->phone_state &= MODEM_SIM_DETECT_FLAG;
	mc->set_ehci_power(0);
	modem_set_slave_wakeup(0);
	msleep(100);
	modem_set_slave_wakeup(1);
	mc->set_ehci_power(1);
	ld->enum_done = true;
	wake_lock_timeout(&mc->wakelock, 5 * HZ);

	return 0;
}

static irqreturn_t modem_cpreset_irq(int irq, void *dev_id)
{
	struct modem_ctl *mc = (struct modem_ctl *)dev_id;
	int val;

	if(mc->ld && mc->ld->enum_done) {
		val = gpio_get_value(mc->gpio_cp_reset);
		wake_lock_timeout(&mc->wakelock, 30 * HZ);
		mc->modem_state_changed(mc, MODEM_EVENT_RESET);
		pr_info("%s CP_RESET_INT:%d\n",  __func__, val);
	}
	return IRQ_HANDLED;
}

static void xmm6260_get_ops(struct modem_ctl *mc)
{
	mc->ops.modem_on         = xmm6260_on;
	mc->ops.modem_off        = xmm6260_off;
	mc->ops.modem_reset      = xmm6260_reset;
}

static long modem_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0, value;
	struct modem_ctl *mc = filp->private_data;
	struct link_device *ld = mc->ld;

	const char *cmd_s[] = {"main", "reset", "renum", "flash", "on", "off"};

	if (cmd <= IOCTL_LINK_OFF_MODEM && cmd >= IOCTL_LINK_MAIN_MODEM)
		pr_info("%s cmd: %s\n", __func__, cmd_s[cmd & 0x07]);

	switch(cmd) {
	case IOCTL_LINK_MAIN_MODEM:
		xmm626x_main_modem(mc, ld);
		break;
	case IOCTL_LINK_RENUM_MODEM:
		xmm626x_renum_modem(mc, ld);
		break;
	case IOCTL_LINK_RESET_MODEM:
		modem_set_active_state(0);
		mc->ops.modem_on(mc);
		break;
	case IOCTL_LINK_ON_MODEM:
		wake_up_interruptible(&mc->read_wq);
		if (ld) ld->enum_done = false;
		mc->set_ehci_power(0);
		mc->ops.modem_on(mc);
		wake_lock_timeout(&mc->wakelock, 10 * HZ);
		mc->modem_state_changed(mc, MODEM_EVENT_RESET);
		break;
	case IOCTL_LINK_OFF_MODEM:
		wake_up_interruptible(&mc->read_wq);
		if (ld) ld->enum_done = false;
		mc->set_ehci_power(0);
		mc->ops.modem_off(mc);
		break;
	case IOCTL_LINK_MODEM_POWER:
		value = gpio_get_value(mc->gpio_cp_reset);
		ret = copy_to_user((void __user *)arg, &value, sizeof(int));
		if (ret < 0)
			return -EFAULT;
		break;
	case IOCTL_LINK_CONNECTED:
	case IOCTL_LINK_HOSTACTIVE:
	case IOCTL_LINK_RPM_STATUS:
	case IOCTL_LINK_HOSTWAKEUP:
		if (ld->ioctl)
			return ld->ioctl(ld, cmd, arg);
	default:
		break;
	}

	return ret;
}

/* ugly modem control command completement, convert to ioctl */
static ssize_t modem_write(struct file *filp, const char __user *buffer,
						size_t count, loff_t *offset)
{
	if (down_interruptible(&modem_downlock) != 0)
		return count;

	if(count >= 5 && !strncmp(buffer, "renum", 5))
		modem_ioctl(filp, IOCTL_LINK_RENUM_MODEM, 0);
	if(count >= 4 && !strncmp(buffer, "main", 4))
		modem_ioctl(filp, IOCTL_LINK_MAIN_MODEM, 0);
	if(count >= 5 && !strncmp(buffer, "reset", 5))
		modem_ioctl(filp, IOCTL_LINK_RESET_MODEM, 0);
	if(count >= 2 && !strncmp(buffer, "on", 2))
		modem_ioctl(filp, IOCTL_LINK_ON_MODEM, 0);
	if(count >= 3 && !strncmp(buffer, "off", 3))
		modem_ioctl(filp, IOCTL_LINK_OFF_MODEM, 0);

	up(&modem_downlock);
	return count;
}

static ssize_t modem_read(struct file *filp, char __user *buffer,
						size_t count, loff_t *offset)
{
	int flag = 0;
	struct modem_ctl *mc = filp->private_data;

	wait_event_interruptible(mc->read_wq,
			(mc->phone_state & MODEM_EVENT_MASK));

	flag = mc->phone_state & MODEM_EVENT_MASK;
	if(copy_to_user(buffer, &flag, sizeof(flag)))
		return -EFAULT;
	pr_info("%s: modem event = 0x%x\n", __func__, flag);

	mc->phone_state &= MODEM_CONNECT_FLAG;

	return 1;
}

static unsigned int modem_poll(struct file *filp, struct poll_table_struct *wait)
{
	u32 mask = 0;
	struct modem_ctl *mc = filp->private_data;

	if (mc->ld->enum_done && (mc->phone_state & MODEM_CONNECT_FLAG))
		mask = POLLIN | POLLOUT | POLLRDNORM;
	else
		poll_wait(filp, &mc->conn_wq, wait);

	return mask;
}

static int modem_open(struct inode *inode, struct file *file)
{
	struct modem_ctl *mc = (struct modem_ctl *)file->private_data;

	file->private_data = (void *)mc;
	return 0;
}

static int modem_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static struct file_operations modem_pm_fops = {
	.owner          = THIS_MODULE,
	.poll           = modem_poll,
	.open           = modem_open,
	.read           = modem_read,
	.write          = modem_write,
	.release        = modem_release,
	.unlocked_ioctl = modem_ioctl,
};

int xmm6260_init_modemctl_device(struct modem_ctl *mc,
			struct modem_data *pdata)
{
	int ret = 0;

	mc->gpio_cp_on        = pdata->gpio_cp_on;
	mc->gpio_cp_reset     = pdata->gpio_cp_reset;
	mc->gpio_reset_req_n  = pdata->gpio_reset_req_n;
	mc->gpio_cp_reset_int = pdata->gpio_cp_reset_int;

	xmm6260_get_ops(mc);
	mc->set_ehci_power = pdata->ehci_power;
	mc->modem_state_changed = xmm6260_state_changed;

	if (mc->gpio_cp_reset_int) {
		mc->irq_modem_reset = gpio_to_irq(mc->gpio_cp_reset_int);
		ret = request_threaded_irq(mc->irq_modem_reset, NULL,
			modem_cpreset_irq, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"CP_RESET_INT", mc);
		if (ret) {
			pr_err("Failed register gpio_cp_reset_int irq(%d)!\n",
					mc->irq_modem_reset);
			goto err_cp_reset_irq;
		}
		ret = enable_irq_wake(mc->irq_modem_reset);
		if (ret) {
			MIF_ERR("failed to enable_irq_wake of modem reset:%d\n",
					ret);
			goto err_reset_irq_enable_wake;
		}

	}

	init_waitqueue_head(&mc->read_wq);
	init_waitqueue_head(&mc->conn_wq);
	mc->miscdev.minor = MISC_DYNAMIC_MINOR;
	mc->miscdev.name = "modemctl";
	mc->miscdev.fops = &modem_pm_fops;

	ret = misc_register(&mc->miscdev);
	if(ret) {
		pr_err("Failed to register modem control device\n");
		goto err_misc_register;
	}

	ret = device_create_file(mc->miscdev.this_device, &attr_modem_debug);
	if (ret) {
		pr_err("failed to create sysfs file:attr_modem_debug!\n");
		goto err_device_create_file;
	}

	return ret;

err_device_create_file:
	misc_deregister(&mc->miscdev);
err_misc_register:
err_reset_irq_enable_wake:
	free_irq(mc->irq_modem_reset, mc);
err_cp_reset_irq:
	return ret;
}
