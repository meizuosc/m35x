/*
 * Copyright (C) 2011 Samsung Electronics.
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

#include <linux/ctype.h>
#include <linux/netdevice.h>
#include <linux/platform_data/modem.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <net/ip.h>

#include "modem_prj.h"
#include "modem_utils.h"

char* format_hex_string(const unsigned char *buf, int count) 
{
	/*define max count of chars to be print*/
#define	MAXCHARS 1024
	/* CHARS_PER_LINE */
#define CPL 16
	const static char hexchar_table[] = {'0', '1', '2', '3', '4', '5', '6',
				'7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
	/*a char=2hex+a space+a printable char+(MAXCHARS+CPL-1)/CPL '\n'+'\0'*/
	static char line[4 * MAXCHARS + (MAXCHARS + CPL - 1)/CPL + 1];
	int actcount = (count < MAXCHARS) ? count : MAXCHARS;
	int index = 0;
	int i, r;

	r = actcount % CPL;
	for (i = 0; i < actcount; i++) {
		index = i/CPL*CPL*4 + i/CPL + i%CPL*3;
		line[index + 0] = hexchar_table[buf[i] >> 4]; 
		line[index + 1] = hexchar_table[buf[i] & 0x0f]; 
		line[index + 2] = ' ';

		if (r && (i >= actcount-r))
			index = i/CPL*CPL*4 + i/CPL + r*3 + i%CPL;
		else
			index = i/CPL*CPL*4 + i/CPL + CPL*3 + i%CPL;

		line[index] = isprint(buf[i]) ?  buf[i]: '.' ;
		
		if (i % CPL == CPL - 1) 
			line[++index] = '\n';
	}
	
	line[++index] = 0;

	return line;
}

struct io_device *get_iod_with_channel(struct link_device *ld,
					unsigned channel)
{
	struct io_device *iodev;

	list_for_each_entry(iodev, &ld->link_dev_list, list) {
		if (iodev->id == channel)
			return iodev;
	}

	return NULL;
}

void iodev_netif_wake(struct io_device *iod, void *args)
{
	if (iod->io_typ == IODEV_NET && iod->ndev) {
		netif_wake_queue(iod->ndev);
		MIF_INFO("%s\n", iod->name);
	}
}

void iodev_netif_stop(struct io_device *iod, void *args)
{
	if (iod->io_typ == IODEV_NET && iod->ndev) {
		netif_stop_queue(iod->ndev);
		MIF_INFO("%s\n", iod->name);
	}
}

