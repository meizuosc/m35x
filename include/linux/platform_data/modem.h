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

#ifndef __MODEM_IF_H__
#define __MODEM_IF_H__

enum modem_t {
	IMC_XMM6260,
	IMC_XMM6262,
	IMC_SC8803G,
	DUMMY,
};

enum modem_io {
	IODEV_TTY,
	IODEV_NET,
	IODEV_DUMMY,
};

enum modem_link {
	LINKDEV_UNDEFINED,
	LINKDEV_SPI,
	LINKDEV_HSIC,
	LINKDEV_MAX,
};

enum modem_network {
	UMTS_NETWORK,
	CDMA_NETWORK,
	LTE_NETWORK,
};

/**
 * struct modem_io_t - declaration for io_device
 * @name:	device name
 * @id:		contain format & channel information
 *		(id & 11100000b)>>5 = format  (eg, 0=FMT, 1=RAW, 2=RFS)
 *		(id & 00011111b)    = channel (valid only if format is RAW)
 * @format:	device format
 * @io_type:	type of this io_device
 * @link:	link_device to use this io_device
 *
 * This structure is used in board-*-modem.c
 */
struct modem_io_t {
	char *name;
	int   id;
	enum modem_io io_type;
	enum modem_link link;
};

struct modemlink_pm_data {
	char *name;
	unsigned gpio_hostwake;
	unsigned gpio_slavewake;
	unsigned gpio_host_active;

	int (*wait_cp_resume)(int port);
};

/* platform data */
struct modem_data {
	char *name;

	unsigned gpio_cp_on;
	unsigned gpio_reset_req_n;
	unsigned gpio_cp_reset;
	unsigned gpio_cp_reset_int;

	/* Modem component */
	enum modem_network  modem_net;
	enum modem_t        modem_type;
	enum modem_link     link_type;
	char               *link_name;

	/* Information of IO devices */
	unsigned  num_iodevs;
	struct    modem_io_t   *iodevs;

	/* ehci power control */
	void (*ehci_power)(int on);

	/* Modem link PM support */
	struct modemlink_pm_data *link_pm_data;
};

#define  MC_HOST_SUCCESS        0
#define  MC_HOST_HIGH           1
#define  MC_HOST_TIMEOUT        2
#define  MC_HOST_HALT           3

#define  HOSTWAKE_TRIGLEVEL	0

void modem_notify_event(int type);

extern int  modem_debug;
extern void modem_set_active_state(int state);
extern void modem_set_slave_wakeup(int state);

#define LOG_TAG "MODEMIF:"

#define MIF_ERR(fmt, ...) if (modem_debug > 0) \
	pr_err(LOG_TAG "%s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)

#define MIF_INFO(fmt, ...) if (modem_debug > 1) \
	pr_info(LOG_TAG "%s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)

#define MIF_TRACE(fmt, ...) if (modem_debug > 2) \
	pr_info("mif: %s: %d: called(%pF): " fmt, __func__,\
			__LINE__, __builtin_return_address(0), ##__VA_ARGS__)

#define MIF_DEBUG(fmt, ...) if (modem_debug > 3) \
	pr_info(LOG_TAG "%s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)

#endif
