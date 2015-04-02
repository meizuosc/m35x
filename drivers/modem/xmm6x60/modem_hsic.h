/*
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2010 Samsung Electronics.
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

#ifndef __MODEM_LINK_DEVICE_USB_H__
#define __MODEM_LINK_DEVICE_USB_H__


enum {
	IF_USB_TTY0_EP = 0,
	IF_USB_TTY1_EP,
	IF_USB_TTY2_EP,
	IF_USB_TTY3_EP,
	IF_USB_DEVNUM_MAX,
};

/* each pipe has 2 ep for in/out */
#define LINKPM_DEV_NUM	(IF_USB_DEVNUM_MAX * 2)
/******************/
/* xmm6260 specific */

/* VID,PID for IMC - XMM6260, XMM6262*/
#define IMC_BOOT_VID		0x058b
#define IMC_BOOT_PID		0x0041
#define IMC_MAIN_VID		0x1519
#define IMC_MAIN_PID		0x0020

enum {
	BOOT_DOWN = 0,
	IPC_CHANNEL
};

enum ch_state {
	STATE_SUSPENDED,
	STATE_RESUMED,
};

/******************/

struct link_pm_info {
	struct usb_link_device *usb_ld;
};

struct usb_id_info {
	int intf_id;
	struct usb_link_device *usb_ld;
};

struct link_pm_data {
	struct usb_link_device *usb_ld;
	unsigned irq_hostwake;
	unsigned gpio_active;
	unsigned gpio_hostwake;
	unsigned gpio_slavewake;
	int (*wait_cp_connect)(int);

	struct workqueue_struct *wq;
	struct delayed_work hsic_pm_work;
	struct delayed_work hsic_pm_start;
	bool resume_requested;
	int resume_retry_cnt;
	int rpm_suspending_cnt;

	struct wake_lock l2_wake;
	struct wake_lock rpm_wake;
	struct notifier_block pm_notifier;
	bool dpm_suspending;

	bool roothub_resume_req;
	struct list_head link;
	spinlock_t lock;
	struct usb_device *udev;
	struct usb_device *hdev;
	struct notifier_block usb_notifier;
};

struct if_usb_devdata {
	struct usb_interface *data_intf;
	struct usb_link_device *usb_ld;
	struct usb_device *usbdev;
	unsigned int tx_pipe;
	unsigned int rx_pipe;
	u8 disconnected;

	int channel_id;
	struct urb *urb;
	void *rx_buf;
	unsigned int rx_buf_size;
	enum ch_state state;

	struct io_device *iod;
	struct usb_anchor tx_defered_urbs;
};

struct usb_link_device {
	/*COMMON LINK DEVICE*/
	struct link_device ld;

	/*USB SPECIFIC LINK DEVICE*/
	struct usb_device	*usbdev;
	struct if_usb_devdata	devdata[IF_USB_DEVNUM_MAX];
	unsigned int		suspended;
	int if_usb_connected;

	/* LINK PM DEVICE DATA */
	struct link_pm_data *link_pm_data;

	/* Host wakeup toggle debugging */
	unsigned ipc_debug_cnt;
	unsigned long tx_cnt;
	unsigned long rx_cnt;
	unsigned long tx_err;
	unsigned long rx_err;
};
/* converts from struct link_device* to struct xxx_link_device* */
#define to_usb_link_device(linkdev) \
			container_of(linkdev, struct usb_link_device, ld)

#endif
