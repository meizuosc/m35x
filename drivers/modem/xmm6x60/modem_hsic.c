/* drivers/modem/modem_link_device_hsic.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2010 Samsung Electronics.
 * Copyright (C) 2012 Zhuhai Meizu Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/if_arp.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/platform_data/modem.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/suspend.h>
#include <linux/usb/cdc.h>
#include <linux/usb.h>
#include <linux/version.h>
#include <linux/wakelock.h>
#include "modem_prj.h"
#include "modem_utils.h"
#include "modem_hsic.h"

#define RX_POOL_SIZE	5

#define ACM_CTRL_DTR	0x01
#define ACM_CTRL_RTS	0x02

extern void pm_runtime_init(struct device *dev);
extern int usb_resume(struct device *dev, pm_message_t msg);
#define HSIC_MAX_PIPE_ORDER_NR 3

static int hsic_pm_runtime_get_active_async(struct link_pm_data *pm_data);
static void hsic_rx_complete(struct urb *urb);

#define get_hostwake(p) \
	(gpio_get_value((p)->gpio_hostwake) == HOSTWAKE_TRIGLEVEL)

#define get_hostactive(p)	(gpio_get_value((p)->gpio_active))

#define report_modem_state(ld, state) \
	do { \
		if ((ld)->mc && (ld)->mc->modem_state_changed) \
			(ld)->mc->modem_state_changed((ld)->mc, (state)); \
	} while(0)

static void set_slavewake(struct link_pm_data *pm_data, int val)
{
	if (!val)
		gpio_set_value(pm_data->gpio_slavewake, 0);
	else {
		if (gpio_get_value(pm_data->gpio_slavewake)) {
			MIF_INFO("warn.. slavewake toggle\n");
			gpio_set_value(pm_data->gpio_slavewake, 0);
			msleep(20);
		}
		gpio_set_value(pm_data->gpio_slavewake, 1);
	}
	MIF_DEBUG("mif: slave wake(%d)\n",
			gpio_get_value(pm_data->gpio_slavewake));
}

static int hsic_rx_submit(struct usb_link_device *usb_ld,
					struct if_usb_devdata *pipe_data,
					gfp_t gfp_flags)
{
	int ret = 0;
	struct urb *urb = pipe_data->urb;

	urb->transfer_flags = 0;
	usb_fill_bulk_urb(urb, pipe_data->usbdev, pipe_data->rx_pipe,
		pipe_data->rx_buf, pipe_data->rx_buf_size, hsic_rx_complete,
		(void *)pipe_data);

	if (pipe_data->disconnected || !usb_ld->usbdev)
		return -ENOENT;

	usb_mark_last_busy(usb_ld->usbdev);
	ret = usb_submit_urb(urb, gfp_flags);
	if (ret)
		MIF_ERR("submit urb fail with ret (%d)\n", ret);

	/* Hold L0 until rx sumit complete */
	usb_mark_last_busy(usb_ld->usbdev);
	return ret;
}

static void hsic_rx_complete(struct urb *urb)
{
	struct if_usb_devdata *pipe_data = urb->context;
	struct usb_link_device *usb_ld = pipe_data->usb_ld;
	struct io_device *iod = pipe_data->iod;
	int ret;

	if (usb_ld->usbdev)
		usb_mark_last_busy(usb_ld->usbdev);

	switch (urb->status) {
	case -ENOENT:
		/* case for 'link pm suspended but rx data had remained' */
	case 0:
		usb_ld->rx_cnt++;
		usb_ld->rx_err = 0;

		if (!urb->actual_length)
			goto rx_submit;

		if (iod->atdebug && iod->atdebugfunc)
			iod->atdebugfunc(iod, urb->transfer_buffer,
					urb->actual_length);

		ret = iod->recv(iod, &usb_ld->ld, (char *)urb->transfer_buffer,
			urb->actual_length);
		if (ret < 0)
			MIF_ERR("io device recv err:%d\n", ret);
rx_submit:
		if (urb->status == 0) {
			if (usb_ld->usbdev)
				usb_mark_last_busy(usb_ld->usbdev);
			hsic_rx_submit(usb_ld, pipe_data, GFP_ATOMIC);
		}
		break;
	default:
		MIF_ERR("urb err status = %d\n", urb->status);
		/* just warning while more rx errors for debug */
		if (usb_ld->rx_err++ > 10) {
			usb_ld->rx_err = 0;
			MIF_ERR("%s RX error 10 times\n", usb_ld->ld.name);
		}
		break;
	}
}

static void hsic_tx_complete(struct urb *urb)
{
	struct sk_buff *skb = urb->context;
	struct io_device *iod = skbpriv(skb)->iod;
	struct link_device *linkdev = get_current_link(iod);
	struct usb_link_device *usb_ld = to_usb_link_device(linkdev);

	switch (urb->status) {
	case 0:
		if (urb->actual_length != urb->transfer_buffer_length)
			MIF_ERR("TX len=%d, Complete len=%d\n",
				urb->transfer_buffer_length, urb->actual_length);
		break;
	case -ECONNRESET:
		if (urb->actual_length)
			MIF_ERR("ECONNRESET: TX len=%d, Complete len=%d\n",
				urb->transfer_buffer_length, urb->actual_length);
	case -ENOENT:
	case -ESHUTDOWN:
	default:
		MIF_ERR("iod %d TX error (%d)\n", iod->id, urb->status);
	}

	if (iod->atdebug && iod->atdebugfunc)
		iod->atdebugfunc(iod, skb->data, skb->len);

	dev_kfree_skb_any(skb);
	usb_free_urb(urb);

	if (urb->dev && usb_ld->if_usb_connected)
		usb_mark_last_busy(urb->dev);
}

static int hsic_tx_skb(struct if_usb_devdata *pipe_data, struct sk_buff *skb)
{
	int ret = 0;
	struct urb *urb;
	struct usb_link_device *usb_ld = pipe_data->usb_ld;
	gfp_t mem_flags = in_interrupt() ? GFP_ATOMIC : GFP_KERNEL;

	usb_ld->tx_cnt++;

	urb = usb_alloc_urb(0, mem_flags);
	if (!urb) {
		MIF_ERR("alloc urb error\n");
		return -ENOMEM;
	}

	urb->transfer_flags = URB_ZERO_PACKET;
	usb_fill_bulk_urb(urb, pipe_data->usbdev, pipe_data->tx_pipe,
			skb->data, skb->len, hsic_tx_complete, (void *)skb);

	/* Check usb link_pm status and if suspend,  transmission will be done
	* in resume.
	* If TX packet was sent from rild/network while interface suspending,
	* It will fail because link pm will be changed to L2 as soon as tx
	* submit. It should keep the TX packet and send next time when
	* RPM_SUSPENDING status.
	* In RPM_RESUMMING status, pipe_data->state flag can help the submit
	* timming after sending previous kept anchor urb submit.
	*/
	if (pipe_data->state == STATE_SUSPENDED ||
		usb_ld->usbdev->dev.power.runtime_status == RPM_SUSPENDING) {
		usb_anchor_urb(urb, &pipe_data->tx_defered_urbs);
		return 0;
	}

	ret =usb_submit_urb(urb, mem_flags);
	if ( ret < 0) {
		MIF_ERR("usb_submit_urb failed with (%d)\n", ret);
		usb_anchor_urb(urb, &pipe_data->tx_defered_urbs);
	}
	return 0;
}

static int hsic_send(struct link_device *ld, struct io_device *iod,
							struct sk_buff *skb)
{
	int ret, rpm_state;
	struct usb_link_device *usb_ld = to_usb_link_device(ld);
	struct link_pm_data *pm_data = usb_ld->link_pm_data;

	if (!usb_ld->if_usb_connected)
		goto link_disconnect;

	/* delay for net channel, limited by xmm6260 capacity */
	if (iod->send_delay && (iod->io_typ == IODEV_NET) \
			&& (1400 == skb->len))
		udelay(iod->send_delay);

	rpm_state = hsic_pm_runtime_get_active_async(pm_data);
	if (rpm_state == -ENODEV)
		goto link_disconnect;

	pm_runtime_get_noresume(&usb_ld->usbdev->dev);

	ret = hsic_tx_skb(&usb_ld->devdata[iod->id], skb);

	usb_mark_last_busy(usb_ld->usbdev);
	pm_runtime_put(&usb_ld->usbdev->dev);

	return ret;

link_disconnect:
	if (iod->io_typ != IODEV_NET)
		report_modem_state(ld, MODEM_EVENT_DISCONN);
	return -EINVAL;
}

/*
#ifdef CONFIG_LINK_PM
*/
static int hsic_pm_runtime_get_active_async(struct link_pm_data *pm_data)
{
	struct usb_link_device *usb_ld = pm_data->usb_ld;
	struct device *dev = &usb_ld->usbdev->dev;

	if (!usb_ld->if_usb_connected) {
		pr_err("%s if_usb_connected failed\n", __func__);
		return -ENODEV;
	}

	if (dev->power.runtime_status == RPM_ACTIVE) {
		pm_data->resume_retry_cnt = 0;
		return 0;
	}

	if (!pm_data->resume_requested) {
		MIF_DEBUG("QW PM\n");
		queue_delayed_work(pm_data->wq, &pm_data->hsic_pm_work, 0);
	}

	return 0;
}

static void hsic_pm_runtime_start(struct work_struct *work)
{
	struct link_pm_data *pm_data =
		container_of(work, struct link_pm_data, hsic_pm_start.work);
	struct usb_device *usbdev = pm_data->usb_ld->usbdev;
	struct device *dev, *ppdev;

	if (!pm_data->usb_ld->if_usb_connected) {
		MIF_DEBUG("disconnect status, ignore\n");
		return;
	}

	dev = &usbdev->dev;

	/* wait interface driver resumming */
	if (dev->power.runtime_status == RPM_SUSPENDED) {
		MIF_ERR("suspended yet, delayed work\n");
		queue_delayed_work(pm_data->wq, &pm_data->hsic_pm_start,
			msecs_to_jiffies(10));
		return;
	}

	if (usbdev && dev->parent) {
		MIF_DEBUG("rpm_status: %d\n", dev->power.runtime_status);
		ppdev = dev->parent->parent;
		pm_runtime_set_autosuspend_delay(&usbdev->dev, 200);
		pm_runtime_allow(dev);
		pm_runtime_allow(ppdev);
		pm_data->resume_requested = false;
		pm_data->resume_retry_cnt = 0;
		pm_data->rpm_suspending_cnt = 0;
	}
}

static void hsic_pm_runtime_work(struct work_struct *work)
{
	int ret;
	struct link_pm_data *pm_data =
		container_of(work, struct link_pm_data, hsic_pm_work.work);
	struct usb_device *usbdev = pm_data->usb_ld->usbdev;
	struct device *dev = &usbdev->dev;

	if (pm_data->dpm_suspending || !pm_data->usb_ld->if_usb_connected)
		return;

	MIF_DEBUG("for dev 0x%p : current %d\n", dev,
				dev->power.runtime_status);

	switch (dev->power.runtime_status) {
	case RPM_ACTIVE:
		pm_data->resume_retry_cnt = 0;
		pm_data->resume_requested = false;
		pm_data->rpm_suspending_cnt = 0;
		return;
	case RPM_SUSPENDED:
		if (pm_data->resume_requested)
			break;
		pm_data->resume_requested = true;
		wake_lock(&pm_data->rpm_wake);
		if (!pm_data->usb_ld->if_usb_connected) {
			report_modem_state(&pm_data->usb_ld->ld,
							MODEM_EVENT_DISCONN);
			wake_unlock(&pm_data->rpm_wake);
			return;
		}
		ret = pm_runtime_resume(dev);
		if (ret < 0) {
			MIF_ERR("resume error(%d)\n", ret);
			if (!pm_data->usb_ld->if_usb_connected) {
				wake_unlock(&pm_data->rpm_wake);
				return;
			}
			/* force to go runtime idle before retry resume */
			if (dev->power.timer_expires == 0 &&
						!dev->power.request_pending) {
				MIF_ERR("run time idle\n");
				pm_runtime_idle(dev);
			}
		}
		wake_unlock(&pm_data->rpm_wake);
		pm_data->rpm_suspending_cnt = 0;
		break;
	case RPM_SUSPENDING:
		/* checking the usb_runtime_suspend running times */
		wake_lock(&pm_data->rpm_wake);
		pm_data->rpm_suspending_cnt++;
		if (pm_data->rpm_suspending_cnt < 10)
			msleep(20);
		else if (pm_data->rpm_suspending_cnt < 30)
			msleep(50);
		else
			msleep(100);
		wake_unlock(&pm_data->rpm_wake);
		break;
	case RPM_RESUMING:
	default:
		MIF_DEBUG("RPM Resuming, ssuspending_cnt :%d\n",
						pm_data->rpm_suspending_cnt);
		pm_data->rpm_suspending_cnt = 0;
		break;
	}
	pm_data->resume_requested = false;
	/* check until runtime_status goes to active */
	if (dev->power.runtime_status == RPM_ACTIVE) {
		pm_data->resume_retry_cnt = 0;
		pm_data->rpm_suspending_cnt = 0;
	} else if (pm_data->resume_retry_cnt++ > 80) {
		MIF_ERR("runtime_status:%d, retry_cnt:%d, notify MODEM_EVENT_CRASH\n",
			dev->power.runtime_status, pm_data->resume_retry_cnt);
		report_modem_state(&pm_data->usb_ld->ld, MODEM_EVENT_CRASH);
	} else {
		MIF_DEBUG("runtime_status:%d, retry_cnt:%d, redo hsic_pm_work\n",
			dev->power.runtime_status, pm_data->resume_retry_cnt);
		queue_delayed_work(pm_data->wq, &pm_data->hsic_pm_work,
							msecs_to_jiffies(20));
	}
}

static irqreturn_t host_wakeup_irq_handler(int irq, void *data)
{
	int value;
	struct link_pm_data *pm_data = data;
	struct link_device *ld = &pm_data->usb_ld->ld;

	value = gpio_get_value(pm_data->gpio_hostwake);
	MIF_ERR("[HWK]<=[%d]\n", value);

	if (pm_data->dpm_suspending) {
		MIF_ERR("wakeup when suspending\n");
		/* Ignore HWK but AP got to L2 by suspending fail */
		wake_lock(&pm_data->l2_wake);
		return IRQ_HANDLED;
	}

	/* wake up modem control ops */
	if (ld->l2_done && (ld->enum_done || value == HOSTWAKE_TRIGLEVEL)) {
		MIF_INFO("modem complete wakeup from L3\n");
		complete(ld->l2_done);
		ld->l2_done = NULL;
	}

	/* CP inititated L3/L2 -> L0 */
	if (ld->enum_done && value == HOSTWAKE_TRIGLEVEL)
		queue_delayed_work(pm_data->wq, &pm_data->hsic_pm_work, 0);
	else
		/* debounce the slave wakeup at AP inititated after usb_resume */
		set_slavewake(pm_data, 0);

	return IRQ_HANDLED;
}

/* hooking from generic_suspend and generic_resume */
static int (*_usb_suspend) (struct usb_device *, pm_message_t);
static int (*_usb_resume) (struct usb_device *, pm_message_t);

static struct linkpm_devices {
	struct list_head pm_data;
	spinlock_t lock;
} xmm626x_devices;

static struct link_pm_data *linkdata_from_udev(struct usb_device *udev)
{
	struct link_pm_data *pm_data = NULL;

	spin_lock_bh(&xmm626x_devices.lock);
	list_for_each_entry(pm_data, &xmm626x_devices.pm_data, link) {
		if (pm_data
			&& (udev == pm_data->udev || udev == pm_data->hdev)) {
			spin_unlock_bh(&xmm626x_devices.lock);
			return pm_data;
		}
		MIF_DEBUG("udev=%p, %s\n", udev, dev_name(&udev->dev));
	}
	spin_unlock_bh(&xmm626x_devices.lock);
	return NULL;
}


/* XMM626x GPIO L2->L0 sequence */
static int xmm626x_gpio_usb_resume(struct link_pm_data *pm_data)
{
	int spin = 20;

	if (get_hostwake(pm_data)) /* CP inititated L2->L0 */
		goto exit;

	/* AP initiated L2->L0 */
	MIF_ERR("AP wakeup modem from L2\n");
	set_slavewake(pm_data, 1);

	while (spin-- && !get_hostwake(pm_data))
		usleep_range(5000, 5500);

	if (!get_hostwake(pm_data)) /* Hostwakeup timeout */
		return -ETIMEDOUT;
exit:
	return 0;
}

#define CP_PORT 2
/* XMM626x GPIO L3->L0 sequence */
static int xmm626x_gpio_l3_resume(struct link_pm_data *pm_data)
{
	int ret = 0;

	pm_data->roothub_resume_req = false;
	/* this point Host Active should low in L3 */
	if (get_hostactive(pm_data))
		return 0;

	if (!get_hostwake(pm_data)) {
		/* AP initiated L3 -> L0 */
		MIF_ERR("AP wakeup modem from L3\n");
		set_slavewake(pm_data, 1);
		msleep(20);
	}

	/* CP initiated L3 -> L0 */
	gpio_set_value(pm_data->gpio_active, 1);

	if (pm_data->wait_cp_connect)
		ret = pm_data->wait_cp_connect(CP_PORT);

	/* TODO: hsic connect timeout */
	if (ret)
		pr_info("%s: resume connect timeout\n", __func__);

	return 0;
}

static int xmm626x_linkpm_usb_resume(struct usb_device *udev, pm_message_t msg)
{
	struct link_pm_data *pm_data = linkdata_from_udev(udev);
	int ret = 0;
	int cnt = 10;

	if (!pm_data) /* unknown devices */
		goto generic_resume;

	if (udev == pm_data->hdev) {
		pm_runtime_mark_last_busy(&pm_data->hdev->dev);
		xmm626x_gpio_l3_resume(pm_data);
		goto generic_resume;
	}

	/* Because HSIC modem skip the hub dpm_resume by quirk, if root hub
	  dpm_suspend was called at runtmie active status, hub resume was not
	  call by port runtime resume. So, it check the L3 status and root hub
	  resume before port resume */
	if (!get_hostactive(pm_data) || pm_data->roothub_resume_req) {
		MIF_DEBUG("ehci root hub resume first\n");
		pm_runtime_mark_last_busy(&pm_data->hdev->dev);
		ret = usb_resume(&pm_data->hdev->dev, PMSG_RESUME);
		if (ret)
			MIF_ERR("hub resume fail\n");
	}
 retry:
	/* Sometimes IMC modem send remote wakeup with gpio, we should check
	  the runtime status and if aleady resumed, */
	if (udev->dev.power.runtime_status == RPM_ACTIVE) {
		MIF_INFO("aleady resume, skip gpio resume\n");
		goto generic_resume;
	}
	ret = xmm626x_gpio_usb_resume(pm_data);
	if (ret < 0) {
		if (cnt--) {
			MIF_ERR("xmm626x_gpio_resume fail(%d)\n", ret);
			goto retry;
		} else {
			MIF_ERR("hostwakeup fail\n");
			/* TODO: exception handing if hoswakeup timeout */
			report_modem_state(&pm_data->usb_ld->ld,
							MODEM_EVENT_DISCONN);
		}
	}

generic_resume:
	return _usb_resume(udev, msg);
}

static int xmm626x_linkpm_usb_suspend(struct usb_device *udev, pm_message_t msg)
{
	struct link_pm_data *pm_data = linkdata_from_udev(udev);

	if (!pm_data) /* unknown devices */
		goto generic_suspend;

	if (!pm_data->usb_ld->if_usb_connected)
		goto generic_suspend;

	/* dpm suspend to Sleep mode */
	if (msg.event == PM_EVENT_SUSPEND) {
		MIF_INFO("hub suspend with rpm active\n");
		pm_data->roothub_resume_req = true;
	}
	/* DO nothing yet */
generic_suspend:
	return _usb_suspend(udev, msg);
}

static int hsic_usb_notifier_event(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	struct link_pm_data *pm_data =
			container_of(this, struct link_pm_data,	usb_notifier);
	struct usb_device *udev = ptr;
	const struct usb_device_descriptor *desc = &udev->descriptor;
	struct usb_device_driver *udriver =
					to_usb_device_driver(udev->dev.driver);
	unsigned long flags;

	switch (event) {
	case USB_DEVICE_ADD:
		/* xmm6260 usb device */
		if (desc->idVendor == 0x1519 && desc->idProduct == 0x0020) {
			if (pm_data->udev) {
				MIF_ERR("pmdata was assigned for udev=%p\n",
								pm_data->udev);
				return NOTIFY_DONE;
			}
			pm_data->udev = udev;
			pm_data->hdev = udev->bus->root_hub;
			MIF_INFO("udev=%p, hdev=%p\n", udev, pm_data->hdev);

			spin_lock_irqsave(&pm_data->lock, flags);
			if (!_usb_resume && udriver->resume) {
				_usb_resume = udriver->resume;
				udriver->resume = xmm626x_linkpm_usb_resume;
			}
			if (!_usb_suspend && udriver->suspend) {
				_usb_suspend = udriver->suspend;
				udriver->suspend = xmm626x_linkpm_usb_suspend;
			}
			spin_unlock_irqrestore(&pm_data->lock, flags);
			MIF_INFO("hook: (%pf, %pf), (%pf, %pf)\n",
					_usb_resume, udriver->resume,
					_usb_suspend,	udriver->suspend);
		}
		break;
	case USB_DEVICE_REMOVE:
		/* xmm6260 usb device */
		if (desc->idVendor == 0x1519 && desc->idProduct == 0x0020) {
			pm_data->hdev = NULL;
			pm_data->udev = NULL;

			MIF_INFO("unhook: (%pf, %pf), (%pf, %pf)\n",
					_usb_resume, udriver->resume,
					_usb_suspend,	udriver->suspend);
			spin_lock_irqsave(&pm_data->lock, flags);
			if (_usb_resume) {
				udriver->resume = _usb_resume;
				_usb_resume = NULL;
			}
			if (_usb_suspend) {
				udriver->suspend = _usb_suspend;
				_usb_suspend = NULL;
			}
			spin_unlock_irqrestore(&pm_data->lock, flags);
		}
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}

static bool hsic_anchor_empty( struct usb_link_device *usb_ld)
{
	int i;
	struct if_usb_devdata *pipe_data;

	/* clear anchor_list at resume */
	if (!usb_ld->if_usb_connected)
		return true;

	for (i = 0; i < IF_USB_DEVNUM_MAX; i++) {
		pipe_data = &usb_ld->devdata[i];
		if (!usb_anchor_empty(&pipe_data->tx_defered_urbs))
			return false;
	}

	return true;

}

static int hsic_pm_notifier_event(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	struct link_pm_data *pm_data =
			container_of(this, struct link_pm_data,	pm_notifier);

	switch (event) {
	case PM_SUSPEND_PREPARE:
		pm_data->dpm_suspending = true;
		MIF_INFO("dpm suspending set to true\n");
		return NOTIFY_OK;
	case PM_POST_SUSPEND:
		pm_data->dpm_suspending = false;
		if (get_hostwake(pm_data) ||
				!hsic_anchor_empty(pm_data->usb_ld)) {
			wake_lock(&pm_data->l2_wake);
			queue_delayed_work(pm_data->wq, &pm_data->hsic_pm_work,
				0);
			MIF_INFO("post resume\n");
		}
		MIF_INFO("dpm suspending set to false\n");
		return NOTIFY_OK;
	}

	return NOTIFY_DONE;
}

static int hsic_ioctl(struct link_device *ld, unsigned int cmd,
							unsigned long arg)
{
	int ret = 0, value;
	struct usb_link_device *usb_ld = to_usb_link_device(ld);
	struct link_pm_data *pm_data = usb_ld->link_pm_data;

	switch (cmd) {
	case IOCTL_LINK_CONNECTED:
		value = usb_ld->if_usb_connected;
		ret = copy_to_user((void __user *)arg, &value, sizeof(int));
		if (ret < 0)
			return -EFAULT;
		break;
	case IOCTL_LINK_HOSTACTIVE:
		value = get_hostactive(pm_data);
		ret = copy_to_user((void __user *)arg, &value, sizeof(int));
		if (ret < 0)
			return -EFAULT;
		break;
	case IOCTL_LINK_RPM_STATUS:
		value = usb_ld->suspended;
		ret = copy_to_user((void __user *)arg, &value, sizeof(int));
		if (ret < 0)
			return -EFAULT;
		break;
	case IOCTL_LINK_HOSTWAKEUP:
		value = gpio_get_value(pm_data->gpio_hostwake);
		ret = copy_to_user((void __user *)arg, &value, sizeof(int));
		if (ret < 0)
			return -EFAULT;
		break;
	default:
		break;
	}

	return ret;
}

static int modem_hsic_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct if_usb_devdata *devdata = usb_get_intfdata(intf);
	struct usb_link_device *usb_ld = devdata->usb_ld;
	struct link_pm_data *pm_data = devdata->usb_ld->link_pm_data;

	if (!devdata->disconnected && devdata->state == STATE_RESUMED) {
		usb_kill_urb(devdata->urb);
		devdata->state = STATE_SUSPENDED;
	}

	devdata->usb_ld->suspended++;

	if (devdata->usb_ld->suspended == IF_USB_DEVNUM_MAX*2) {
		MIF_ERR("[modem_hsic_suspended]\n");
		wake_lock_timeout(&pm_data->l2_wake, msecs_to_jiffies(500));
		if (!usb_ld->rx_cnt && !usb_ld->tx_cnt) {
			if (usb_ld->ipc_debug_cnt++ > 10) {
				usb_ld->ipc_debug_cnt = 0;
				MIF_ERR("No TX/RX after resume 10times\n");
				report_modem_state(&pm_data->usb_ld->ld,
							MODEM_EVENT_DISCONN);
			}
		} else {
			usb_ld->ipc_debug_cnt = 0;
			usb_ld->rx_cnt = 0;
			usb_ld->tx_cnt = 0;
		}
	}

	return 0;
}

static void usb_anchor_urb_head(struct urb *urb, struct usb_anchor *anchor)
{
	unsigned long flags;

	spin_lock_irqsave(&anchor->lock, flags);
	usb_get_urb(urb);
	list_add(&urb->anchor_list, &anchor->urb_list);
	urb->anchor = anchor;

	if (unlikely(anchor->poisoned))
		atomic_inc(&urb->reject);
	spin_unlock_irqrestore(&anchor->lock, flags);
}

static int hsic_defered_tx_from_anchor(struct if_usb_devdata *pipe_data)
{
	struct urb *urb;
	struct sk_buff *skb;
	int cnt = 0;
	int ret = 0;

	while ((urb = usb_get_from_anchor(&pipe_data->tx_defered_urbs))) {
		usb_put_urb(urb);
		skb = (struct sk_buff *)urb->context;

		/* drop package when disconnect */
		if (!pipe_data->usb_ld->if_usb_connected) {
			usb_free_urb(urb);
			dev_kfree_skb_any(skb);
			continue;
		}

		/* refrash the urb */
		usb_fill_bulk_urb(urb, pipe_data->usbdev, pipe_data->tx_pipe,
			skb->data, skb->len, hsic_tx_complete, (void *)skb);

		ret = usb_submit_urb(urb, GFP_ATOMIC);
		if (ret < 0) {
			/* TODO: deferd TX again */
			MIF_ERR("resume deferd TX fail(%d)\n", ret);
			usb_anchor_urb_head(urb, &pipe_data->tx_defered_urbs);
			goto exit;
		}
		cnt++;
	}
exit:
	if (cnt)
		MIF_INFO("deferd tx urb=%d(CH%d)\n", cnt, pipe_data->channel_id);
	return ret;
}

static int modem_hsic_resume(struct usb_interface *intf)
{
	int ret = 0;
	struct if_usb_devdata *devdata = usb_get_intfdata(intf);
	struct link_pm_data *pm_data = devdata->usb_ld->link_pm_data;

	if (devdata->state != STATE_SUSPENDED) {
		MIF_DEBUG("aleady resume!\n");
		goto done;
	}
	devdata->state = STATE_RESUMED;

	ret = hsic_defered_tx_from_anchor(devdata);
	if (ret < 0)
		goto resume_exit;

	ret = hsic_rx_submit(devdata->usb_ld, devdata, GFP_ATOMIC);
	if (ret < 0) {
		MIF_ERR("hsic_rx_submit error with (%d)\n", ret);
		goto resume_exit;
	}
done:
	pm_data->resume_retry_cnt = 0;
	devdata->usb_ld->suspended--;

	if (!devdata->usb_ld->suspended) {
		MIF_ERR("[modem_hsic_resumed]\n");
		wake_lock(&pm_data->l2_wake);
	}

resume_exit:
	return ret;
}

static int modem_hsic_reset_resume(struct usb_interface *intf)
{
	int ret;
	struct if_usb_devdata *devdata = usb_get_intfdata(intf);
	struct link_pm_data *pm_data = devdata->usb_ld->link_pm_data;

	ret = modem_hsic_resume(intf);
	devdata->usb_ld->ipc_debug_cnt = 0;
	/*
	 * for runtime suspend, kick runtime pm at L3 -> L0 reset resume
	*/
	if (!devdata->usb_ld->suspended)
		queue_delayed_work(pm_data->wq, &pm_data->hsic_pm_start, 0);

	return ret;
}

static void modem_hsic_disconnect(struct usb_interface *intf)
{
	struct if_usb_devdata *devdata = usb_get_intfdata(intf);
	struct link_pm_data *pm_data = devdata->usb_ld->link_pm_data;
	struct device *dev, *ppdev;

	if (devdata->disconnected)
		return;

	wake_lock_timeout(&pm_data->l2_wake, msecs_to_jiffies(1000));

	usb_driver_release_interface(to_usb_driver(intf->dev.driver), intf);

	usb_kill_urb(devdata->urb);

	dev = &devdata->usb_ld->usbdev->dev;
	ppdev = dev->parent->parent;
	pm_runtime_forbid(ppdev);

	MIF_DEBUG("dev 0x%p\n", devdata->usbdev);
	usb_put_dev(devdata->usbdev);

	devdata->data_intf = NULL;
	devdata->usbdev = NULL;
	devdata->disconnected = 1;
	devdata->state = STATE_SUSPENDED;

	devdata->usb_ld->if_usb_connected = 0;
	devdata->usb_ld->suspended = 0;

	usb_set_intfdata(intf, NULL);

	/* cancel runtime start delayed works */
	cancel_delayed_work(&pm_data->hsic_pm_start);

	report_modem_state(&pm_data->usb_ld->ld, MODEM_EVENT_DISCONN);

	return;
}

static int modem_hsic_set_pipe(struct usb_link_device *usb_ld,
			const struct usb_host_interface *desc, int pipe)
{
	if (pipe < 0 || pipe >= IF_USB_DEVNUM_MAX) {
		MIF_ERR("undefined endpoint, exceed max\n");
		return -EINVAL;
	}

	MIF_DEBUG("set %d\n", pipe);

	if ((usb_pipein(desc->endpoint[0].desc.bEndpointAddress)) &&
	    (usb_pipeout(desc->endpoint[1].desc.bEndpointAddress))) {
		usb_ld->devdata[pipe].rx_pipe = usb_rcvbulkpipe(usb_ld->usbdev,
				desc->endpoint[0].desc.bEndpointAddress);
		usb_ld->devdata[pipe].tx_pipe = usb_sndbulkpipe(usb_ld->usbdev,
				desc->endpoint[1].desc.bEndpointAddress);
	} else if ((usb_pipeout(desc->endpoint[0].desc.bEndpointAddress)) &&
		   (usb_pipein(desc->endpoint[1].desc.bEndpointAddress))) {
		usb_ld->devdata[pipe].rx_pipe = usb_rcvbulkpipe(usb_ld->usbdev,
				desc->endpoint[1].desc.bEndpointAddress);
		usb_ld->devdata[pipe].tx_pipe = usb_sndbulkpipe(usb_ld->usbdev,
				desc->endpoint[0].desc.bEndpointAddress);
	} else {
		MIF_ERR("undefined endpoint\n");
		return -EINVAL;
	}

	return 0;
}


static struct usb_id_info hsic_channel_info;

static int __devinit modem_hsic_probe(struct usb_interface *intf,
					const struct usb_device_id *id)
{
	int err;
	int pipe;
	const struct usb_cdc_union_desc *union_hdr;
	const struct usb_host_interface *data_desc;
	unsigned char *buf = intf->altsetting->extra;
	int buflen = intf->altsetting->extralen;
	struct usb_interface *data_intf;
	struct usb_device *usbdev = interface_to_usbdev(intf);
	struct usb_driver *usbdrv = to_usb_driver(intf->dev.driver);
	struct usb_id_info *info = (struct usb_id_info *)id->driver_info;
	struct usb_link_device *usb_ld = info->usb_ld;
	struct usb_interface *control_interface;
	struct usb_device *root_usbdev= to_usb_device(intf->dev.parent->parent);

	pr_info("hsic usbdev=0x%p, intf->dev=0x%p\n", usbdev, &intf->dev);
	pm_runtime_init(&intf->dev);

	usb_ld->usbdev = usbdev;
	pm_runtime_forbid(&usbdev->dev);
	usb_ld->ipc_debug_cnt = 0;
	usb_ld->link_pm_data->dpm_suspending = false;

	union_hdr = NULL;
	/* for WMC-ACM compatibility, WMC-ACM use an end-point for control msg*/
	if (intf->altsetting->desc.bInterfaceSubClass != USB_CDC_SUBCLASS_ACM) {
		MIF_ERR("ignore Non ACM end-point\n");
		return -EINVAL;
	}
	if (!buflen) {
		if (intf->cur_altsetting->endpoint->extralen &&
				    intf->cur_altsetting->endpoint->extra) {
			buflen = intf->cur_altsetting->endpoint->extralen;
			buf = intf->cur_altsetting->endpoint->extra;
		} else {
			MIF_ERR("Zero len descriptor reference\n");
			return -EINVAL;
		}
	}
	while (buflen > 0) {
		if (buf[1] == USB_DT_CS_INTERFACE) {
			switch (buf[2]) {
			case USB_CDC_UNION_TYPE:
				if (union_hdr)
					break;
				union_hdr = (struct usb_cdc_union_desc *)buf;
				break;
			default:
				break;
			}
		}
		buf += buf[0];
		buflen -= buf[0];
	}
	if (!union_hdr) {
		MIF_ERR("USB CDC is not union type\n");
		return -EINVAL;
	}
	control_interface = usb_ifnum_to_if(usbdev, union_hdr->bMasterInterface0);
	control_interface->needs_remote_wakeup = 0;
	pm_runtime_set_autosuspend_delay(&root_usbdev->dev, 200); /*200ms*/

	err = usb_control_msg(usbdev, usb_sndctrlpipe(usbdev, 0),
			USB_CDC_REQ_SET_CONTROL_LINE_STATE,
			USB_TYPE_CLASS | USB_RECIP_INTERFACE,
			ACM_CTRL_DTR | ACM_CTRL_RTS,
			control_interface->altsetting[0].desc.bInterfaceNumber,
			NULL, 0, 5000);
	if (err < 0)
		MIF_ERR("set RTS/CTS failed\n");

	data_intf = usb_ifnum_to_if(usbdev, union_hdr->bSlaveInterface0);
	if (!data_intf) {
		MIF_ERR("data_inferface is NULL\n");
		return -ENODEV;
	}

	data_desc = data_intf->altsetting;
	if (!data_desc) {
		MIF_ERR("data_desc is NULL\n");
		return -ENODEV;
	}

	pipe = intf->altsetting->desc.bInterfaceNumber / 2;
	if (modem_hsic_set_pipe(usb_ld, data_desc, pipe) < 0)
		return -EINVAL;

	usb_ld->devdata[pipe].usbdev = usb_get_dev(usbdev);
	usb_ld->devdata[pipe].state = STATE_RESUMED;
	usb_ld->devdata[pipe].usb_ld = usb_ld;
	usb_ld->devdata[pipe].data_intf = data_intf;
	usb_ld->devdata[pipe].channel_id = pipe;
	usb_ld->devdata[pipe].disconnected = 0;
	usb_ld->devdata[pipe].iod = get_iod_with_channel(&usb_ld->ld, pipe);;

	MIF_DEBUG("devdata usbdev = 0x%p\n", usb_ld->devdata[pipe].usbdev);

	err = usb_driver_claim_interface(usbdrv, data_intf,
				(void *)&usb_ld->devdata[pipe]);
	if (err < 0) {
		MIF_ERR("usb_driver_claim() failed\n");
		return err;
	}

	usb_ld->suspended = 0;
	pm_suspend_ignore_children(&usbdev->dev, true);

	usb_set_intfdata(intf, (void *)&usb_ld->devdata[pipe]);

	/* rx start for this endpoint */
	hsic_rx_submit(usb_ld, &usb_ld->devdata[pipe], GFP_KERNEL);

	if (pipe == HSIC_MAX_PIPE_ORDER_NR) {
		usb_ld->if_usb_connected = 1;
		report_modem_state(&usb_ld->ld, MODEM_EVENT_CONN);
		pr_info("%s pipe:%d\n", __func__, HSIC_MAX_PIPE_ORDER_NR);
		if (!work_pending(&usb_ld->link_pm_data->hsic_pm_start.work)) {
			queue_delayed_work(usb_ld->link_pm_data->wq,
				&usb_ld->link_pm_data->hsic_pm_start,
				msecs_to_jiffies(500));
			wake_lock(&usb_ld->link_pm_data->l2_wake);
		}
	}

	MIF_DEBUG("successfully done\n");

	return 0;
}

static struct usb_id_info hsic_channel_info = {
	.intf_id = IPC_CHANNEL,
};

static struct usb_device_id modem_hsic_usb_ids[] = {
	{
          USB_DEVICE(IMC_MAIN_VID, IMC_MAIN_PID),
	  .driver_info = (unsigned long)&hsic_channel_info,
	},
	{}
};
MODULE_DEVICE_TABLE(usb, modem_hsic_usb_ids);

static struct usb_driver modem_hsic_driver = {
	.name                 = "cdc_modem",
	.probe                = modem_hsic_probe,
	.disconnect           = modem_hsic_disconnect,
	.id_table             = modem_hsic_usb_ids,
	.suspend              = modem_hsic_suspend,
	.resume               = modem_hsic_resume,
	.reset_resume         = modem_hsic_reset_resume,
	.supports_autosuspend = 1,
};

static int modem_hsic_init(struct link_device *ld)
{
	int ret;
	int i;
	struct usb_link_device *usb_ld = to_usb_link_device(ld);
	struct if_usb_devdata *pipe_data;
	struct usb_id_info *id_info;

	/* to connect usb link device with usb interface driver */
	for (i = 0; i < ARRAY_SIZE(modem_hsic_usb_ids); i++) {
		id_info = (struct usb_id_info *)modem_hsic_usb_ids[i].driver_info;
		if (id_info)
			id_info->usb_ld = usb_ld;
	}

	/* allocate rx buffer for usb receive */
	for (i = 0; i < IF_USB_DEVNUM_MAX; i++) {
		pipe_data = &usb_ld->devdata[i];
		pipe_data->channel_id = i;
		pipe_data->rx_buf_size = 16 * 1024;

		pipe_data->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!pipe_data->urb) {
			MIF_ERR("alloc urb fail\n");
			return -ENOMEM;
		}
		pipe_data->rx_buf = kzalloc(pipe_data->rx_buf_size,
						GFP_DMA | GFP_KERNEL);
		if (!pipe_data->rx_buf) {
			usb_kill_urb(pipe_data->urb);
			return -ENOMEM;
		}

		init_usb_anchor(&pipe_data->tx_defered_urbs);
	}

	ret = usb_register(&modem_hsic_driver);
	if (ret) {
		MIF_ERR("usb_register_driver() fail : %d\n", ret);
		return ret;
	}

	MIF_INFO("modem_hsic_init() done : %d, usb_ld (0x%p)\n", ret, usb_ld);

	return ret;
}

#define to_modemctl(pdev) ((struct modem_ctl *)(platform_get_drvdata(pdev)))
static int hsic_pm_init(struct usb_link_device *usb_ld, void *data)
{
	int r = 0;
	struct platform_device *pdev = (struct platform_device *)data;
	struct modem_data *pdata =
			(struct modem_data *)pdev->dev.platform_data;
	struct modemlink_pm_data *pm_pdata = pdata->link_pm_data;
	struct link_pm_data *pm_data =
			kzalloc(sizeof(struct link_pm_data), GFP_KERNEL);
	if (!pm_data) {
		MIF_ERR("link_pm_data is NULL\n");
		return -ENOMEM;
	}

	pm_data->gpio_active = pm_pdata->gpio_host_active;
	pm_data->gpio_hostwake    = pm_pdata->gpio_hostwake;
	pm_data->gpio_slavewake   = pm_pdata->gpio_slavewake;
	pm_data->wait_cp_connect  = pm_pdata->wait_cp_resume;

	pm_data->irq_hostwake = gpio_to_irq(pm_data->gpio_hostwake);

	to_modemctl(pdev)->irq_hostwake = pm_data->irq_hostwake;

	pm_data->usb_ld = usb_ld;
	usb_ld->link_pm_data = pm_data;

	wake_lock_init(&pm_data->l2_wake, WAKE_LOCK_SUSPEND, "l2_hsic");
	wake_lock_init(&pm_data->rpm_wake, WAKE_LOCK_SUSPEND, "rpm_hsic");

	r = request_threaded_irq(pm_data->irq_hostwake,
		NULL, host_wakeup_irq_handler,
		IRQF_NO_SUSPEND | IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
		"modem_hostwake", (void *)pm_data);
	if (r) {
		MIF_ERR("fail to request irq(%d)\n", r);
		goto err_request_irq;
	}

	r = enable_irq_wake(pm_data->irq_hostwake);
	if (r) {
		MIF_ERR("failed to enable_irq_wake:%d\n", r);
		goto err_set_wake_irq;
	}

	/* create work queue & init work for runtime pm */
	pm_data->wq = create_singlethread_workqueue("linkpmd");
	if (!pm_data->wq) {
		MIF_ERR("fail to create wq\n");
		goto err_create_wq;
	}

	pm_data->usb_notifier.notifier_call = hsic_usb_notifier_event;
	usb_register_notify(&pm_data->usb_notifier);

	pm_data->pm_notifier.notifier_call = hsic_pm_notifier_event;
	register_pm_notifier(&pm_data->pm_notifier);

	INIT_DELAYED_WORK(&pm_data->hsic_pm_work, hsic_pm_runtime_work);
	INIT_DELAYED_WORK(&pm_data->hsic_pm_start, hsic_pm_runtime_start);

	spin_lock_init(&pm_data->lock);
	INIT_LIST_HEAD(&xmm626x_devices.pm_data);
	spin_lock_init(&xmm626x_devices.lock);
	spin_lock_bh(&xmm626x_devices.lock);
	list_add(&pm_data->link, &xmm626x_devices.pm_data);
	spin_unlock_bh(&xmm626x_devices.lock);

	return 0;

err_create_wq:
	disable_irq_wake(pm_data->irq_hostwake);
err_set_wake_irq:
	free_irq(pm_data->irq_hostwake, (void *)pm_data);
err_request_irq:
	kfree(pm_data);
	return r;
}

struct link_device *hsic_create_link_device(void *data, enum modem_link link_type)
{
	int ret;
	struct usb_link_device *usb_ld;
	struct link_device *ld;

	if (link_type != LINKDEV_HSIC)
		return NULL;

	usb_ld = kzalloc(sizeof(struct usb_link_device), GFP_KERNEL);
	if (!usb_ld)
		return NULL;

	usb_ld->tx_cnt = 0;
	usb_ld->rx_cnt = 0;
	usb_ld->rx_err = 0;
	usb_ld->ipc_debug_cnt = 0;
	INIT_LIST_HEAD(&usb_ld->ld.link_dev_list);

	ld = &usb_ld->ld;

	ld->name = "hsic";
	ld->send = hsic_send;
	ld->ioctl = hsic_ioctl;
	ld->link_type = link_type;
	ld->l2_done = NULL;

	/* create link pm device */
	ret = hsic_pm_init(usb_ld, data);
	if (ret)
		goto err;

	ret = modem_hsic_init(ld);
	if (ret)
		goto err;

	MIF_INFO("%s : create_link_device DONE\n", usb_ld->ld.name);


	return (void *)ld;
err:
	kfree(usb_ld);
	return NULL;
}

static void __exit modem_hsic_exit(void)
{
	usb_deregister(&modem_hsic_driver);
}

