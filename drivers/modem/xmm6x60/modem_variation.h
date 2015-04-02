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

#ifndef __MODEM_VARIATION_H__
#define __MODEM_VARIATION_H__

#include <linux/platform_data/modem.h>

#define DECLARE_MODEM_INIT(type)	\
	int type ## _init_modemctl_device(struct modem_ctl *mc,	\
				struct modem_data *pdata)
#define DECLARE_MODEM_INIT_DUMMY(type) \
	static DECLARE_MODEM_INIT(type) { return -ENOTSUPP; }

#define DECLARE_LINK_INIT(type)	\
		struct link_device *type ## _create_link_device(	\
		struct platform_device *pdev, enum modem_link link_type)
#define DECLARE_LINK_INIT_DUMMY(type) \
	static DECLARE_LINK_INIT(type) { return NULL; }

/* add declaration of modem & link type */
/* modem device support */
#ifdef CONFIG_UMTS_MODEM_XMM6260
DECLARE_MODEM_INIT(xmm6260);
#else
DECLARE_MODEM_INIT_DUMMY(xmm6260);
#endif

#ifdef CONFIG_UMTS_MODEM_XMM6262
DECLARE_MODEM_INIT(xmm6262);
#else
DECLARE_MODEM_INIT_DUMMY(xmm6262);
#endif

#ifdef CONFIG_UMTS_MODEM_SC8803G
DECLARE_MODEM_INIT(sc8803g);
#else
DECLARE_MODEM_INIT_DUMMY(sc8803g);
#endif

/* link device support */
#ifdef CONFIG_LINK_DEVICE_SPI
DECLARE_LINK_INIT(spi);
#else
DECLARE_LINK_INIT_DUMMY(spi);
#endif

#ifdef CONFIG_LINK_DEVICE_HSIC
DECLARE_LINK_INIT(hsic);
#else
DECLARE_LINK_INIT_DUMMY(hsic);
#endif

static int modem_init_device(struct modem_ctl *mc, struct modem_data *pdata)
{
	if (mc->modem_type == IMC_XMM6260)
		return xmm6260_init_modemctl_device(mc, pdata);
	else if(mc->modem_type == IMC_XMM6262)
		return xmm6262_init_modemctl_device(mc, pdata);
	else if(mc->modem_type == IMC_SC8803G)
		return sc8803g_init_modemctl_device(mc, pdata);
	return -ENOTSUPP;
}

static struct link_device *init_link_device(struct platform_device *pdev,
			enum modem_link link_type)
{
	if (link_type == LINKDEV_HSIC)
		return hsic_create_link_device(pdev, link_type);
	else if (link_type == LINKDEV_SPI)
		return spi_create_link_device(pdev, link_type);
	return NULL;
}

#endif
