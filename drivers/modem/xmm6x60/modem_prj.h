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

#ifndef __MODEM_PRJ_H__
#define __MODEM_PRJ_H__

#include <linux/completion.h>
#include <linux/miscdevice.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/tty.h>
#include <linux/wait.h>
#include <linux/wakelock.h>

/* modem events to rild */
#define  MODEM_CONNECT_FLAG     0x0001
#define  MODEM_RESET_FLAG       0x0002
#define  MODEM_CRASH_FLAG       0x0004
#define  MODEM_DUMP_FLAG        0x0008
#define  MODEM_DISCONNECT_FLAG  0x0010
#define  MODEM_SIM_DETECT_FLAG  0x0020
#define  MODEM_INIT_ON_FLAG     0x0040

#define  MODEM_EVENT_MASK       0x007E

enum MODEM_EVENT_TYPE {
	MODEM_EVENT_POWEROFF,
	MODEM_EVENT_RESET,
	MODEM_EVENT_CRASH,
	MODEM_EVENT_DUMP,
	MODEM_EVENT_CONN,
	MODEM_EVENT_DISCONN,
	MODEM_EVENT_SIM,
	MODEM_EVENT_BOOT_INIT,
};

/* ioctl command definitions */
#define IOCTL_LINK_MAIN_MODEM		_IO('o', 0x20)
#define IOCTL_LINK_RESET_MODEM		_IO('o', 0x21)
#define IOCTL_LINK_RENUM_MODEM		_IO('o', 0x22)
#define IOCTL_LINK_FLASH_MODEM		_IO('o', 0x23)
#define IOCTL_LINK_ON_MODEM		_IO('o', 0x24)
#define IOCTL_LINK_OFF_MODEM		_IO('o', 0x25)

#define IOCTL_LINK_CONNECTED		_IOR('o', 0x30, int)
#define IOCTL_LINK_HOSTACTIVE		_IOR('o', 0x31, int)
#define IOCTL_LINK_MODEM_POWER		_IOR('o', 0x32, int)
#define IOCTL_LINK_RPM_STATUS		_IOR('o', 0x33, int)
#define IOCTL_LINK_HOSTWAKEUP		_IOR('o', 0x34, int)

enum link_mode {
	LINK_MODE_INVALID = 0,
	LINK_MODE_IPC,
	LINK_MODE_BOOT,
	LINK_MODE_DLOAD,
	LINK_MODE_ULOAD,
};

struct vnet {
	int pkt_sz;
	struct io_device *iod;
	struct net_device_stats stats;
};

/** struct skbuff_priv - private data of struct sk_buff
 * this is matched to char cb[48] of struct sk_buff
 */
struct skbuff_private {
	struct io_device *iod;
	struct link_device *ld;
};

static inline struct skbuff_private *skbpriv(struct sk_buff *skb)
{
	BUILD_BUG_ON(sizeof(struct skbuff_private) > sizeof(skb->cb));
	return (struct skbuff_private *)&skb->cb;
}

/** rx_alloc_skb - allocate an skbuff and set skb's iod, ld
 * @length:	length to allocate
 * @gfp_mask:	get_free_pages mask, passed to alloc_skb
 * @iod:	struct io_device *
 * @ld:		struct link_device *
 *
 * %NULL is returned if there is no free memory.
 */
static inline struct sk_buff *rx_alloc_skb(unsigned int length,
		gfp_t gfp_mask, struct io_device *iod, struct link_device *ld)
{
	struct sk_buff *skb = alloc_skb(length, gfp_mask);
	if (likely(skb)) {
		skbpriv(skb)->iod = iod;
		skbpriv(skb)->ld = ld;
	}
	return skb;
}

struct io_device {
	struct list_head  list;

	/* Name of the IO device */
	char *name;

	atomic_t opened;

	struct net_device *ndev;
	struct tty_port port;

	/* ID for channel on the link */
	unsigned id;
	enum modem_link link_type;
	enum modem_io io_typ;
	enum modem_network phone_net_type;

	/* Rx queue of sk_buff and delay work */
	struct sk_buff_head rx_q;
	struct delayed_work rx_work;

	/* called from linkdevice when a packet arrives for this iodevice */
	int (*recv)(struct io_device *iod, struct link_device *ld,
					const char *data, unsigned int len);

	struct modem_ctl *mc;

	struct wake_lock wakelock;
	int atdebug;
	int (*atdebugfunc)(struct io_device *iod, const char* buf, int len);
	int send_delay;

	/* DO NOT use __current_link directly
	 * you MUST use skbpriv(skb)->ld in mc, link, etc..
	 */
	struct link_device *__current_link;
};

/* get_current_link, set_current_link don't need to use locks.
 * In ARM, set_current_link and get_current_link are compiled to
 * each one instruction (str, ldr) as atomic_set, atomic_read.
 * And, the order of set_current_link and get_current_link is not important.
 */
#define get_current_link(iod) ((iod)->__current_link)
#define set_current_link(iod, ld) ((iod)->__current_link = (ld))

struct link_device {
	struct list_head link_dev_list;
	char *name;

	enum modem_link link_type;

	/* Modem control */
	struct modem_ctl *mc;
	bool enum_done;
	struct completion *l2_done;

	/* Operation mode of the link device */
	enum link_mode mode;

	/* called by an io_device when it has a packet to send over link
	 * - the io device is passed so the link device can look at id and
	 *   format fields to determine how to route/format the packet
	 */
	int (*send)(struct link_device *ld, struct io_device *iod,
						struct sk_buff *skb);
	int (*ioctl)(struct link_device *ld, unsigned int cmd,
						unsigned long _arg);

};

struct modemctl_ops {
	int (*modem_on) (struct modem_ctl *);
	int (*modem_off) (struct modem_ctl *);
	int (*modem_reset) (struct modem_ctl *);
};

struct modem_ctl {
	struct miscdevice miscdev;
	wait_queue_head_t  read_wq;
	wait_queue_head_t  conn_wq;
	unsigned int phone_state;

	char *name;
	struct device *dev;
	enum modem_t modem_type;
	struct modem_data *mdm_data;

	/* link device for modem */
	struct link_device *ld;

	unsigned gpio_cp_on;
	unsigned gpio_reset_req_n;
	unsigned gpio_cp_reset;
	unsigned gpio_cp_reset_int;

	int irq_modem_reset;
	int irq_hostwake;

	struct workqueue_struct *rx_wq;

	struct modemctl_ops ops;
	void (*set_ehci_power)(int on);

	void (*modem_state_changed)(struct modem_ctl *mc, int state);
	struct wake_lock wakelock;
};

int meizu_ipc_init_io_device(struct io_device *iod);
int modem_tty_driver_init(struct modem_ctl *modemctl);

#endif
