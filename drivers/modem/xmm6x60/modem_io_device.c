/* /drivers/modem/meizu_ipc_io_device.c
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/if_arp.h>
#include <linux/ip.h>
#include <linux/if_ether.h>
#include <linux/etherdevice.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>

#include <linux/platform_data/modem.h>
#include "modem_prj.h"
#include "modem_utils.h"

#include <linux/pm_qos.h>

static struct pm_qos_request gprs_mif_qos;

static int atdebugchannel = 0;
static int atdebuglen = 0;

static const struct tty_port_operations iod_port_ops = {
};

static int __init atdebugchannel_setup(char *args)
{
	int error;
	unsigned long val;

	error = strict_strtoul(args, 16, &val);
	if (!error)
		atdebugchannel = val;

	return error;
}
__setup("atdebugchannel=", atdebugchannel_setup);

static int __init atdebuglen_setup(char *args)
{
	int error;
	unsigned long val;

	error = strict_strtoul(args, 10, &val);
	if (!error)
		atdebuglen = val;

	return error;
}
__setup("atdebuglen=", atdebuglen_setup);

static int atdebugfunc(struct io_device *iod, const char* buf, int len)
{
	int atdebuglen = 0;

	if (iod->atdebug) {
		char *atdebugbuf;

		atdebuglen = iod->atdebug > len ? len : iod->atdebug;
		atdebugbuf = format_hex_string(buf, atdebuglen);
		pr_info("\n%pF, iod id:%d, data len:%d, data:\n%s\n",
			__builtin_return_address(0), iod->id, len, atdebugbuf);
	}

	return atdebuglen;
}

static ssize_t show_atdebug(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct io_device *iod = dev_get_drvdata(dev);
	char *p = buf;
	int atdebug;

	atdebug = iod->atdebug;
	p += sprintf(buf, "iod id:%d, atdebug:%d\n", iod->id, atdebug);

	return p - buf;
}

static ssize_t store_atdebug(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long atdebug;
	struct io_device *iod = dev_get_drvdata(dev);

	ret = strict_strtoul(buf, 10, &atdebug);
	if (ret)
		return count;

	iod->atdebug = atdebug;

	return count;
}

static struct device_attribute attr_atdebug =
	__ATTR(atdebug, S_IRUGO | S_IWUSR, show_atdebug, store_atdebug);

static ssize_t show_send_delay(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct io_device *iod = dev_get_drvdata(dev);
	char *p = buf;
	int send_delay;

	send_delay = iod->send_delay;
	p += sprintf(buf, "iod id:%d, send_delay:%d\n", iod->id, send_delay);

	return p - buf;
}

static ssize_t store_send_delay(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long send_delay;
	struct io_device *iod = dev_get_drvdata(dev);

	ret = strict_strtoul(buf, 10, &send_delay);
	if (ret)
		return count;

	iod->send_delay = send_delay;

	return count;
}

static struct device_attribute attr_send_delay =
	__ATTR(send_delay, S_IRUGO | S_IWUSR, show_send_delay, store_send_delay);

static int rx_tty_data(struct io_device *iod, const char *data,
							unsigned int len)
{
	int count;
	struct tty_struct *tty = tty_port_tty_get(&iod->port);

	if (!tty)
		return -EINVAL;

	if (test_bit(TTY_THROTTLED, &tty->flags)) {
		pr_info("%s: drop packet as throttled\n", iod->name);
		goto out;
	}

	count = tty_insert_flip_string(tty, data, len);
	if (iod->atdebug && iod->atdebugfunc)
		iod->atdebugfunc(iod, data, len);

	tty_flip_buffer_push(tty);

out:
	tty_kref_put(tty);

	return 0;
}

static void rx_net_data_work(struct work_struct *work)
{
	int ret;
	struct iphdr *ip_hdr;
	struct sk_buff *skb = NULL;
	struct io_device *iod = container_of(work, struct io_device,
							rx_work.work);
	struct net_device *ndev = iod->ndev;
	struct vnet *vnet = netdev_priv(ndev);

	if (atomic_read(&iod->opened) == 0 || iod->io_typ != IODEV_NET)
		skb_queue_purge(&iod->rx_q);

	while ((skb = skb_dequeue(&iod->rx_q))) {
		wake_lock_timeout(&iod->wakelock, HZ * 0.5);

		ip_hdr = (struct iphdr *)skb->data;
		vnet->pkt_sz = ntohs(ip_hdr->tot_len);
		if (vnet->pkt_sz <= 0) {
			dev_kfree_skb_any(skb);
			continue;
		}
		if (ip_hdr->ihl !=5 && ip_hdr->version != 4)
			pr_err("rild_net packet ihl %x ver %x\n",
					ip_hdr->ihl, ip_hdr->version);

		skb->dev = ndev;
		if (ip_hdr->version == 6)
			skb->protocol = htons(ETH_P_IPV6);
		else
			skb->protocol = htons(ETH_P_IP);

		skb_reset_mac_header(skb);
		vnet->stats.rx_packets ++;
		vnet->stats.rx_bytes += vnet->pkt_sz;

		ret = netif_rx(skb);
		if (ret != NET_RX_SUCCESS) {
			MIF_ERR("%s: net rx fail(err %d)\n", iod->name, ret);
			dev_kfree_skb_any(skb);
		}
	}
}

/* called from link device when a packet arrives for this io device */
static int recv_data_from_link_dev(struct io_device *iod,
	struct link_device *ld, const char *data, unsigned int len)
{
	struct sk_buff *skb;

	if (len <= 0)
		return -EINVAL;

	if (atomic_read(&iod->opened) == 0)
		return 0;

	wake_lock_timeout(&iod->wakelock, HZ * 0.5);
	if (iod->io_typ == IODEV_TTY)
		return rx_tty_data(iod, data, len);

	skb = rx_alloc_skb(len, GFP_ATOMIC, iod, ld);
	if (!skb)
		return -ENOMEM;

	memcpy(skb_put(skb, len), data, len);

	skb_queue_tail(&iod->rx_q, skb);
	queue_delayed_work(iod->mc->rx_wq, &iod->rx_work, 0);

	return 0;
}

static int vnet_open(struct net_device *ndev)
{
	struct vnet *vnet = netdev_priv(ndev);
	struct io_device *iod = vnet->iod;

	netif_start_queue(ndev);
	atomic_inc(&iod->opened);

	MIF_INFO("%s (opened %d)\n", iod->name, atomic_read(&iod->opened));

	pm_qos_update_request(&gprs_mif_qos, 200000);

	return 0;
}

static int vnet_stop(struct net_device *ndev)
{
	struct vnet *vnet = netdev_priv(ndev);
	struct io_device *iod = vnet->iod;

	if (atomic_dec_return(&iod->opened) == 0) {
		MIF_INFO("close iod = %s\n", iod->name);
		pm_qos_update_request(&gprs_mif_qos, 0);
	}
	netif_stop_queue(ndev);

	MIF_INFO("%s (closed %d)\n", iod->name, atomic_read(&iod->opened));

	return 0;
}

static int vnet_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct vnet *vnet = netdev_priv(ndev);
	struct io_device *iod = vnet->iod;
	struct link_device *ld = get_current_link(iod);
	int is_ip_packet = 0;
	int data_len  = 0;
	int ret = 0;

	wake_lock_timeout(&iod->wakelock, HZ * 0.5);
	is_ip_packet = (skb->protocol == htons(ETH_P_IP)) ? 1 : 0;
	skbpriv(skb)->iod = iod;
	skbpriv(skb)->ld = ld;
	data_len = skb->len;

	ret = ld->send(ld, iod, skb);
	if (ret < 0) {
		MIF_ERR("%s: ld->send fail (err %d)\n", iod->name, ret);
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	if(is_ip_packet) {
		vnet->stats.tx_packets ++;
		vnet->stats.tx_bytes += data_len;
	}

	return NETDEV_TX_OK;
}

static struct net_device_stats *vnet_get_stats(struct net_device *dev)
{
	struct vnet *vnet = netdev_priv(dev);

	return &vnet->stats;
}

static struct net_device_ops vnet_ops = {
	.ndo_open       = vnet_open,
	.ndo_stop       = vnet_stop,
	.ndo_get_stats  = vnet_get_stats,
	.ndo_start_xmit = vnet_xmit,
};

static void vnet_setup(struct net_device *ndev)
{
	ndev->mtu             = ETH_DATA_LEN;
	ndev->type            = ARPHRD_NONE;
	ndev->flags           = IFF_POINTOPOINT | IFF_NOARP;
	ndev->features        = 0;
	ndev->addr_len        = 0;
	ndev->destructor      = free_netdev;
	ndev->netdev_ops      = &vnet_ops;
	ndev->tx_queue_len    = 1000;
	ndev->watchdog_timeo  = 20 * HZ;
	ndev->hard_header_len = 0;
}

static int modem_tty_open(struct tty_struct *tty, struct file *filp)
{
	struct io_device *iod = dev_get_drvdata(tty->dev);

	tty_port_open(&iod->port, tty, filp);
	atomic_inc(&iod->opened);

	MIF_INFO("%s (opened %d)\n", iod->name, atomic_read(&iod->opened));

	return 0;
}

static void modem_tty_close(struct tty_struct *tty, struct file *filp)
{
	struct io_device *iod = dev_get_drvdata(tty->dev);

	atomic_dec(&iod->opened);
	tty_port_close(&iod->port, tty, filp);

	MIF_INFO("%s (closed %d)\n", iod->name, atomic_read(&iod->opened));
}

static int modem_tty_write_room(struct tty_struct *tty)
{
	int ret = 2 * 1024;

	return ret;
}

static int modem_tty_write(struct tty_struct *tty,
					const unsigned char *buf, int count)
{

	struct io_device *iod = dev_get_drvdata(tty->dev);
	struct link_device *ld = get_current_link(iod);
	struct sk_buff *skb;
	size_t tx_size;
	int err;

	skb = alloc_skb(count, GFP_ATOMIC);
	if (!skb) {
		MIF_ERR("fail alloc skb (%d)\n", __LINE__);
		return -ENOMEM;
	}
	wake_lock_timeout(&iod->wakelock, HZ * 0.5);

	memcpy(skb_put(skb, count), buf, count);

	if (iod->atdebug && iod->atdebugfunc)
		iod->atdebugfunc(iod, skb->data, count);

	tx_size = skb->len;

	skbpriv(skb)->iod = iod;
	skbpriv(skb)->ld = ld;

	err = ld->send(ld, iod, skb);
	if (err < 0)
		dev_kfree_skb_any(skb);

	return tx_size;
}

static struct tty_operations tty_ops = {
	.open       = modem_tty_open,
	.close      = modem_tty_close,
	.write      = modem_tty_write,
	.write_room = modem_tty_write_room,
};

/* tty driver for tty io_device */
static struct tty_driver *xmm_tty_driver;

int modem_tty_driver_init(struct modem_ctl *modemctl)
{
	int ret;

	xmm_tty_driver = alloc_tty_driver(4);
	if (xmm_tty_driver == 0) {
		pr_err("%s alloc_tty_driver -ENOMEM!!\n", __func__);
		return -ENOMEM;
	}
	xmm_tty_driver->name         = "ttyACM";
	xmm_tty_driver->type         = TTY_DRIVER_TYPE_SERIAL;
	xmm_tty_driver->major        = 0;
	xmm_tty_driver->owner        = THIS_MODULE;
	xmm_tty_driver->subtype      = SERIAL_TYPE_NORMAL;
	xmm_tty_driver->minor_start  = 0;
	xmm_tty_driver->driver_name  = "tty_driver";
	xmm_tty_driver->init_termios = tty_std_termios;
	xmm_tty_driver->init_termios.c_iflag = 0;
	xmm_tty_driver->init_termios.c_oflag = 0;
	xmm_tty_driver->init_termios.c_cflag = B4000000 | CS8 | CREAD;
	xmm_tty_driver->init_termios.c_lflag = 0;
	xmm_tty_driver->flags = TTY_DRIVER_RESET_TERMIOS |
				TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	tty_set_operations(xmm_tty_driver, &tty_ops);
	ret = tty_register_driver(xmm_tty_driver);
	if (ret) {
		tty_driver_kref_put(xmm_tty_driver);
		xmm_tty_driver = NULL;
		pr_err("%s error!!\n", __func__);
		return ret;
	}
	xmm_tty_driver->driver_state = modemctl;
	pm_qos_add_request(&gprs_mif_qos, PM_QOS_BUS_THROUGHPUT, 0);

	return 0;
}

int meizu_ipc_init_io_device(struct io_device *iod)
{
	int ret = 0;
	struct vnet *vnet;
	struct device *ttydev;

	iod->recv = recv_data_from_link_dev;
	switch (iod->io_typ) {
	case IODEV_TTY:
		iod->atdebugfunc = atdebugfunc;
		if (atdebugchannel & (0x1 << iod->id))
			if (atdebuglen)
				iod->atdebug = atdebuglen;
			else
				iod->atdebug = 255;
		else
			iod->atdebug = 0;
		iod->send_delay = 0;

		if (xmm_tty_driver == NULL)
			break;
		tty_port_init(&iod->port);
		iod->port.ops = &iod_port_ops;
		ttydev = tty_register_device(xmm_tty_driver, iod->id, NULL);
		if (ttydev < 0) {
			MIF_ERR("%s: ERR! tty_register fail\n", iod->name);
			break;
		}
		dev_set_drvdata(ttydev, iod);
		ret = device_create_file(ttydev, &attr_atdebug);
		if (ret)
			MIF_ERR("failed to create sysfs file : %s\n",
					iod->name);
		break;
	case IODEV_NET:
		iod->atdebug = 0;
		iod->atdebugfunc = NULL;
		iod->send_delay = 5000;
		skb_queue_head_init(&iod->rx_q);
		INIT_DELAYED_WORK(&iod->rx_work, rx_net_data_work);
		iod->ndev = alloc_netdev(sizeof(struct vnet), iod->name,
			vnet_setup);
		if (!iod->ndev) {
			MIF_ERR("failed to alloc netdev\n");
			return -ENOMEM;
		}
		ret = register_netdev(iod->ndev);
		if (ret) {
			free_netdev(iod->ndev);
			return ret;
		}

		dev_set_drvdata(&iod->ndev->dev, iod);
		ret = device_create_file(&iod->ndev->dev, &attr_send_delay);
		if (ret)
			MIF_ERR("failed to create send_delay sysfs file : %s\n",
					iod->name);
		vnet = netdev_priv(iod->ndev);
		MIF_DEBUG("(vnet:0x%p)\n", vnet);
		vnet->pkt_sz = 0;
		vnet->iod = iod;
		break;
	default:
		MIF_ERR("wrong io_type : %d\n", iod->io_typ);
		return -EINVAL;
	}

	MIF_DEBUG("%s(%d) : init_io_device() done : %d\n",
				iod->name, iod->io_typ, ret);
	return ret;
}
