/*
 * rmnet_sprd.c
 *
 * Author: 
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/ipv6.h>
#include <linux/ip.h>
#include <asm/byteorder.h>


#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/tty.h>

#define POLL_DELAY 1000000 /* 1 second delay interval */

 //#define RMNET_DEBUG
 #define RMNET_CRITICAL_DEBUG
 #define RMNET_ERR


#define RMNET_WD_TMO		 20
#define RMNET_ETH_HDR_SIZE   14
#define RMNET_MTU_SIZE		(1500 + RMNET_ETH_HDR_SIZE)
#define MAX_PART_PKT_SIZE    2500

typedef enum {
	RMNET_FULL_PACKET,
	RMNET_PARTIAL_PACKET,
	RMNET_PARTIAL_HEADER,
} RMNET_PAST_STATE;

static struct {
	RMNET_PAST_STATE state;
	char buf[MAX_PART_PKT_SIZE];
	int size;
	int type;
} past_packet;

#define RMNET_TTY_IDX_BASE 13
#define RMNET_DEV_NUM 2
static struct  net_device *rmnet_dev[RMNET_DEV_NUM] = {NULL, NULL};

 extern int mux_write(struct tty_struct *tty, const unsigned char *buf, int count);

struct rmnet_private {
	struct net_device_stats stats;
	struct wake_lock wake_lock;
  int tty_index;
#ifdef CONFIG_RMNET_DEBUG
	ktime_t last_packet;
	short active_countdown; /* Number of times left to check */
	short restart_count; /* Number of polls seems so far */
	unsigned long wakeups_xmit;
	unsigned long wakeups_rcv;
	unsigned long timeout_us;
	unsigned long awake_time_ms;
	struct delayed_work work;
#endif
};

static struct {
	int blocked;
	struct net_device *dev;
} rmnet_ch_block_info[16];

//#define DEBUG_RMNET_LOG
#ifdef DEBUG_RMNET_LOG // need debug it later
#include <linux/uaccess.h>
#include <linux/fs.h>
#define MAX_BUFF 8192
#define NAME_LEN 32 
typedef struct Log_Struct
{
  struct file *filp;
  mm_segment_t oldfs;
  char buffer[MAX_BUFF];
  char name[NAME_LEN];
  int len;
} my_log_st;
static my_log_st mylog[RMNET_DEV_NUM];
static void log_write(int dev, char *p, int len)
{
  int i = 0;
  int datalen = len;
  int ileft = 0;
  char *p1 = p;
  my_log_st *plog;
  switch(dev)
  {
    case 13:
    case 14:
      i = dev - RMNET_TTY_IDX_BASE;
      plog = mylog + i;
      plog->oldfs = get_fs();
      break;
    default:
	    printk(KERN_ALERT "invalid dev %d in log_close\n", dev);
      return;
  }
  if (IS_ERR(plog->filp)) 
  {
    printk(KERN_ALERT "invalid file %p\n", plog->filp);
    return;
	}
  for(;;)
  {
    if(plog->len + datalen >= MAX_BUFF)
    {
      ileft = MAX_BUFF - plog->len;
      memcpy(plog->buffer+plog->len, p1, ileft);
      plog->oldfs = get_fs();
      set_fs(KERNEL_DS);
      vfs_write(plog->filp, plog->buffer, MAX_BUFF, &(plog->filp->f_pos));
      set_fs(plog->oldfs);
      datalen -= ileft;
      plog->len = 0;
      p1 += ileft;
      if(datalen == 0)
      {
        break;
      }
    }
    else
    {
      memcpy(plog->buffer+plog->len, p1, datalen);
      plog->len += datalen;
      break;
    }
  }
}

static void log_close(int dev)
{
  int i = 0;
  my_log_st *plog;
  switch(dev)
  {
    case 13:
    case 14:
      i = dev - RMNET_TTY_IDX_BASE;
      plog = mylog + i;
      break;
    default:
	    printk(KERN_ALERT "invalid dev %d in log_close\n", dev);
      return;
  }
  if (IS_ERR(plog->filp)) 
  {
	  printk(KERN_ALERT "invalid file pointer\n");
    return;
  }
  plog->oldfs = get_fs();
  set_fs(KERNEL_DS);
  if(plog->len > 0)
  {
    vfs_write(plog->filp, plog->buffer, plog->len, &(plog->filp->f_pos));
  }
  filp_close(plog->filp, NULL);
  set_fs(plog->oldfs);

  plog->len = 0;
  memset(plog->name, 0x0, NAME_LEN);
}

static void log_open(int dev)
{
  int i = dev - RMNET_TTY_IDX_BASE;
  my_log_st *plog;

  switch(dev)
  {
    case 13:
    case 14:
      i = dev - RMNET_TTY_IDX_BASE;
      plog = mylog + i;
      plog->len = 0;
      memset(plog->name, 0x0, NAME_LEN);
      sprintf(plog->name, "/data/local/rmnet0.log%d",i);
      plog->oldfs = get_fs();
      set_fs(KERNEL_DS);
      plog->filp = filp_open(plog->name, O_RDWR|O_CREAT, 0);
      set_fs(plog->oldfs);
      break;
    default:
	    printk(KERN_ALERT "invalid dev %d\n", dev);
      return;
  }
  if (IS_ERR(plog->filp)) 
  {
    printk(KERN_ALERT "open file rmnet%d failed\n", i);
	}
  else
  {
    printk(KERN_ALERT "open file rmnet%d successfully\n",i);
  }
  return;
}
#endif // DEBUG_RMNET_LOG

static int count_this_packet(void *_hdr, int len)
{
	struct ethhdr *hdr = _hdr;

	if (len >= ETH_HLEN && hdr->h_proto == htons(ETH_P_ARP)) {
		return 0;
	}

	return 1;
}

#ifdef CONFIG_RMNET_DEBUG
static int in_suspend;
static unsigned long timeout_us;
static struct workqueue_struct *rmnet_wq;

static void do_check_active(struct work_struct *work)
{
	struct rmnet_private *p =
		container_of(work, struct rmnet_private, work.work);

	/*
	 * Soft timers do not wake the cpu from suspend.
	 * If we are in suspend, do_check_active is only called once at the
	 * timeout time instead of polling at POLL_DELAY interval. Otherwise the
	 * cpu will sleeps and the timer can fire much much later than POLL_DELAY
	 * casuing a skew in time calculations.
	 */
	if (in_suspend) {
		/*
		 * Assume for N packets sent durring this session, they are
		 * uniformly distributed durring the timeout window.
		 */
		int tmp = p->timeout_us * 2 -
			(p->timeout_us / (p->active_countdown + 1));
		tmp /= 1000;
		p->awake_time_ms += tmp;

		p->active_countdown = p->restart_count = 0;
		return;
	}

	/*
	 * Poll if not in suspend, since this gives more accurate tracking of
	 * rmnet sessions.
	 */
	p->restart_count++;
	if (--p->active_countdown == 0) {
		p->awake_time_ms += p->restart_count * POLL_DELAY / 1000;
		p->restart_count = 0;
	} else {
		queue_delayed_work(rmnet_wq, &p->work,
				usecs_to_jiffies(POLL_DELAY));
	}
}

#ifdef CONFIG_HAS_EARLYSUSPEND
/*
 * If early suspend is enabled then we specify two timeout values,
 * screen on (default), and screen is off.
 */
static unsigned long timeout_suspend_us;
static struct device *rmnet0;

/* Set timeout in us when the screen is off. */
static ssize_t timeout_suspend_store(
	struct device *d,
	struct device_attribute *attr,
	const char *buf, size_t n)
{
	timeout_suspend_us = simple_strtoul(buf, NULL, 10);
	return n;
}

static ssize_t timeout_suspend_show(
	struct device *d,
	struct device_attribute *attr,
	char *buf)
{
	return sprintf(buf, "%lu\n", (unsigned long) timeout_suspend_us);
}

static DEVICE_ATTR(timeout_suspend, 0664, timeout_suspend_show, timeout_suspend_store);

static void rmnet_early_suspend(struct early_suspend *handler)
{
	if (rmnet0) {
		struct rmnet_private *p = netdev_priv(to_net_dev(rmnet0));
		p->timeout_us = timeout_suspend_us;
	}
	in_suspend = 1;
}

static void rmnet_late_resume(struct early_suspend *handler)
{
	if (rmnet0) {
		struct rmnet_private *p = netdev_priv(to_net_dev(rmnet0));
		p->timeout_us = timeout_us;
	}
	in_suspend = 0;
}

static struct early_suspend rmnet_power_suspend = {
	.suspend = rmnet_early_suspend,
	.resume = rmnet_late_resume,
};

static int __init rmnet_late_init(void)
{
	register_early_suspend(&rmnet_power_suspend);
	return 0;
}

late_initcall(rmnet_late_init);
#endif

/* Returns 1 if packet caused rmnet to wakeup, 0 otherwise. */
static int rmnet_cause_wakeup(struct rmnet_private *p)
{
	int ret = 0;
	ktime_t now;
	if (p->timeout_us == 0) /* Check if disabled */
		return 0;

	/* Start timer on a wakeup packet */
	if (p->active_countdown == 0) {
		ret = 1;
		now = ktime_get_real();
		p->last_packet = now;

		if (in_suspend) {
			queue_delayed_work(rmnet_wq, &p->work,
					usecs_to_jiffies(p->timeout_us));
		} else {
			queue_delayed_work(rmnet_wq, &p->work,
					usecs_to_jiffies(POLL_DELAY));
		}
	}

	if (in_suspend) {
		p->active_countdown++;
	} else {
		p->active_countdown = p->timeout_us / POLL_DELAY;
	}

	return ret;
}

static ssize_t wakeups_xmit_show(struct device *d,
	struct device_attribute *attr,
	char *buf)
{
	struct rmnet_private *p = netdev_priv(to_net_dev(d));
	return sprintf(buf, "%lu\n", p->wakeups_xmit);
}

DEVICE_ATTR(wakeups_xmit, 0444, wakeups_xmit_show, NULL);

static ssize_t wakeups_rcv_show(
	struct device *d,
	struct device_attribute *attr,
	char *buf)
{
	struct rmnet_private *p = netdev_priv(to_net_dev(d));
	return sprintf(buf, "%lu\n", p->wakeups_rcv);
}

DEVICE_ATTR(wakeups_rcv, 0444, wakeups_rcv_show, NULL);

/* Set timeout in us. */
static ssize_t timeout_store(
	struct device *d,
	struct device_attribute *attr,
	const char *buf, size_t n)
{
#ifndef CONFIG_HAS_EARLYSUSPEND
	struct rmnet_private *p = netdev_priv(to_net_dev(d));
	p->timeout_us = timeout_us = simple_strtoul(buf, NULL, 10);
#else
	/* If using early suspend/resume hooks do not write the value on store. */
	timeout_us = simple_strtoul(buf, NULL, 10);
#endif
	return n;
}

static ssize_t timeout_show(
	struct device *d,
	struct device_attribute *attr,
	char *buf)
{
	struct rmnet_private *p = netdev_priv(to_net_dev(d));
	p = netdev_priv(to_net_dev(d));
	return sprintf(buf, "%lu\n", timeout_us);
}

DEVICE_ATTR(timeout, 0664, timeout_show, timeout_store);

/* Show total radio awake time in ms */
static ssize_t awake_time_show(
	struct device *d,
	struct device_attribute *attr,
	char *buf)
{
	struct rmnet_private *p = netdev_priv(to_net_dev(d));
	return sprintf(buf, "%lu\n", p->awake_time_ms);
}
DEVICE_ATTR(awake_time_ms, 0444, awake_time_show, NULL);

#endif

#define RMNET_IPV6_VER 0x6
#define RMNET_IPV4_VER 0x4

/*give the packet to TCP/IP*/
static void rmnet_trans_packet(
	struct net_device *dev,
	int type,
	void *buf,
	int sz)
{
	struct rmnet_private *p = netdev_priv(dev);
	struct sk_buff *skb;
	void *ptr = NULL;

	sz += RMNET_ETH_HDR_SIZE;

#if defined (RMNET_CRITICAL_DEBUG)
	printk("\nRMNET: %d<\n",sz);
#endif

	if (sz > RMNET_MTU_SIZE) {
#if defined (RMNET_ERR)
		printk("rmnet_trans_packet() error  discarding %d pkt len\n", sz);
#endif
		ptr = 0;
		return;
	}
	else {
		skb = dev_alloc_skb(sz + NET_IP_ALIGN);
		if (skb == NULL) {
#if defined (RMNET_ERR)
			printk("rmnet_trans_packet() error cannot allocate skb\n");
#endif
			return;
		}
		else {
			skb->dev = dev;
			skb_reserve(skb, NET_IP_ALIGN);
			ptr = skb_put(skb, sz);
			wake_lock_timeout(&p->wake_lock, HZ / 2);

			/* adding ethernet header */
			{
				/* struct ethhdr eth_hdr = {0xB6,0x91,0x24,0xa8,0x14,0x72,0xb6,
										0x91,0x24,0xa8,0x14,0x72,0x08,0x0};*/
				char temp[] = {0xB6,0x91,0x24,0xa8,0x14,0x72,0xb6,0x91,0x24,
							   0xa8,0x14,0x72,0x08,0x0};
				struct ethhdr *eth_hdr = (struct ethhdr *) temp;

				if (type == RMNET_IPV6_VER) {
					eth_hdr->h_proto = 0x08DD;
					eth_hdr->h_proto = htons(eth_hdr->h_proto);
				}

				memcpy((void *)eth_hdr->h_dest,
					   (void*)dev->dev_addr,
					   sizeof(eth_hdr->h_dest));
				memcpy((void *)ptr,
					   (void *)eth_hdr,
					   sizeof(struct ethhdr));
				
			}
			memcpy(ptr + RMNET_ETH_HDR_SIZE, buf, sz - RMNET_ETH_HDR_SIZE);
			skb->protocol = eth_type_trans(skb, dev);
			if (count_this_packet(ptr, skb->len)) {
#ifdef CONFIG_RMNET_DEBUG
				p->wakeups_rcv += rmnet_cause_wakeup(p);
#endif
				p->stats.rx_packets++;
				p->stats.rx_bytes += skb->len;
			}
			printk("rmnet_trans_packet() netif_rx skb\n");
			netif_rx(skb);
			wake_unlock(&p->wake_lock);
		}
	}
}


void rmnet_rx_data(int tty_index, __u8 *uih_data_start, __u32 uih_len)
{
	struct net_device *dev = NULL;
	void *buf = NULL;
	int tot_sz = 0;
  int i = 0;
	struct rmnet_private *p = NULL;
	
#if defined (RMNET_CRITICAL_DEBUG)
  printk("rmnet_rx_data: rmnet_rx_data uihlen = %d on tty %d\n",uih_len, tty_index);
#endif

  for(i = 0; i < RMNET_DEV_NUM; i ++)
  {
    dev = rmnet_dev[i];
    if (!dev) {
#if defined (RMNET_ERR)
      printk("rmnet_rx_data: No device \n");
#endif
      return;
    }

    p = netdev_priv(dev);
    if (!p)
    {
#if defined (RMNET_ERR)
        printk("rmnet_rx_data: no priv data \n");
#endif  return;
    }
    if(p->tty_index == tty_index)
    {
      break;
    }
  }

  if(i >= RMNET_DEV_NUM)
  {
#if defined (RMNET_ERR)
    printk("rmnet_rx_data: cant find rmnet dev\n");
#endif
    return;
  }

	buf = uih_data_start;
	tot_sz = uih_len;
#ifdef DEBUG_RMNET_LOG
  log_write(p->tty_index, (char *) buf , tot_sz);
#endif

	if (!buf) {
#if defined (RMNET_ERR)
		printk("rmnet_rx_data: No buf recvd \n");
#endif
		return;
	}
#if defined (RMNET_DEBUG)
	printk("rmnet_rx_data: total size read = %d  \n",tot_sz);
#endif

	switch (past_packet.state)
	{
	case RMNET_FULL_PACKET:
		/* no need to do anything */
	break;

	case RMNET_PARTIAL_PACKET:
	{
		void *ip_hdr = (void *)past_packet.buf;
		int sz;
		int copy_size;

#if defined (RMNET_DEBUG)
		printk("rmnet_rx_data: past partial packet\n");
#endif
		if (past_packet.type == RMNET_IPV4_VER) {
			sz = ntohs(((struct iphdr*) ip_hdr)->tot_len);
		} else if (past_packet.type == RMNET_IPV6_VER) {
			sz = ntohs(((struct ipv6hdr*) ip_hdr)->payload_len) + sizeof(struct ipv6hdr);
		} else {
#if defined (RMNET_ERR)
		      printk("rmnet_rx_data: Invalid past version (data), %d\n",past_packet.type);
#endif
			past_packet.state = RMNET_FULL_PACKET;
			return;
		}

		copy_size = sz - past_packet.size;

		 /* if read size if > then copy size, copy full packet.*/
		if (tot_sz >= copy_size) {
			memcpy(past_packet.buf + past_packet.size,buf,copy_size);
		} else {
			/* copy whatever read if read size < packet size.*/
			memcpy(past_packet.buf + past_packet.size,buf,tot_sz);
#if defined (RMNET_DEBUG)
		      printk("rmnet_rx_data: RMNET_PARTIAL_PACKET. past size = %d,"
				" total size = %d\n", past_packet.size, tot_sz);
#endif
			past_packet.size += tot_sz;
			return;
		}

		rmnet_trans_packet(dev,past_packet.type,(void*)past_packet.buf,sz);
#if defined (RMNET_DEBUG)
		printk("rmnet_rx_data: pushed reassembled data packet to tcpip,"
				" sz = %d\n", sz);
#endif
		buf = buf + copy_size;
		tot_sz = tot_sz - copy_size;
		past_packet.state = RMNET_FULL_PACKET;
	}
	break;

	case RMNET_PARTIAL_HEADER:
	{
		void *ip_hdr = (void *)past_packet.buf;
		int sz;
		int copy_size;
		int hdr_size = 0;

#if defined (RMNET_DEBUG)
		printk("rmnet_rx_data: past partial header packet\n");
#endif
		if (past_packet.type == RMNET_IPV4_VER)
			hdr_size = sizeof(struct iphdr);
		else if (past_packet.type  == RMNET_IPV6_VER)
			hdr_size = sizeof(struct ipv6hdr);
		else
		{
#if defined (RMNET_ERR)
                printk("rmnet_rx_data: Invalid past version (hdr), %d\n",
					past_packet.type);
#endif
			past_packet.state = RMNET_FULL_PACKET;
			return;
		}

		copy_size = hdr_size - past_packet.size;

		if(tot_sz >= copy_size) {
			memcpy(past_packet.buf + past_packet.size,buf,copy_size);
		} else {
			/* copy whatever read if read size < packet size. */
			memcpy(past_packet.buf + past_packet.size,buf,tot_sz);
#if defined (RMNET_DEBUG)
			printk("rmnet_rx_data: Still partial header \n");
#endif
			past_packet.size += tot_sz;
			return;
		}

		buf = buf + copy_size;
		tot_sz = tot_sz - copy_size;
		past_packet.size = past_packet.size + copy_size;

		if (past_packet.type == RMNET_IPV4_VER) {
			sz = ntohs(((struct iphdr*) ip_hdr)->tot_len);
		} else if (past_packet.type == RMNET_IPV6_VER) {
			sz = ntohs(((struct ipv6hdr*) ip_hdr)->payload_len) + sizeof(struct ipv6hdr);
		} else {
#if defined (RMNET_ERR)
			printk("rmnet_rx_data: Invalid past version, %d\n",
					past_packet.type);
#endif
			past_packet.state = RMNET_FULL_PACKET;
			return;
		}

		copy_size = sz - past_packet.size;

		 /* if read size if > then copy size, copy full packet. */
		if (tot_sz >= copy_size) {
			memcpy(past_packet.buf + past_packet.size,buf,copy_size);
		} else {
			/* copy whatever read if read size < packet size.*/
			memcpy(past_packet.buf + past_packet.size,buf,tot_sz);
#if defined (RMNET_DEBUG)
                printk("rmnet_rx_data: RMNET_PARTIAL_HEADER. past size = %d,"
					" total size = %d\n", past_packet.size, tot_sz);
#endif
			past_packet.size += tot_sz;
			past_packet.state = RMNET_PARTIAL_PACKET;
			return;
		}

		rmnet_trans_packet(dev,past_packet.type,(void *)past_packet.buf,sz);

		buf = buf + copy_size;
		tot_sz = tot_sz - copy_size;

	}
	break;

	default:
#if defined (RMNET_ERR)
         printk("rmnet_rx_data: Invalid past state %d\n", (int)past_packet.state);
#endif
		past_packet.state = RMNET_FULL_PACKET;
		break;
	}

	while (tot_sz) {
		int hdr_size = 0;
		int ver = 0;
		void *ip_hdr = (void *)buf;
		int data_sz = 0;

		ver = (((char *)buf)[0] & 0xF0) >> 4;

#if defined (RMNET_DEBUG)
		printk("rmnet_rx_data: ver = 0x%x, total size : %d \n", ver, tot_sz);
#endif

		if (ver == RMNET_IPV4_VER) {
			hdr_size = sizeof(struct iphdr);
		} else if (ver == RMNET_IPV6_VER) {
			hdr_size = sizeof(struct ipv6hdr);
		} else {
                       printk("rmnet_rx_data: error \n");
		
#if defined (RMNET_ERR)
/*
         if( 1)
         {
			void *ip_hdr = (char*)(buf+4);
			int tmp_sz = ntohs(((struct iphdr*) ip_hdr)->tot_len);
            
			printk("rmnet_rx_data: Invalid version, 0x%x\n", ver);
			printk("rmnet_rx_data: Few bytes of pkt : 0x%x 0x%x 0x%x 0x%x "
					"0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",	((char *)buf)[0],
					((char *)buf)[1], ((char *)buf)[2], ((char *)buf)[3],
					((char *)buf)[4], ((char *)buf)[5], ((char *)buf)[6],
					((char *)buf)[7], ((char *)buf)[8], ((char *)buf)[9],
					((char *)buf)[10]);
			printk("rmnet_rx_data: Current packet size = %d,"
					"ip packet size = %d\n",tot_sz,tmp_sz);
            }*/
#endif
			past_packet.state = RMNET_FULL_PACKET;
			break;
		}
 printk("rmnet_rx_data: hdr_size = %d \n", hdr_size);
		if (tot_sz < 6) {
#if defined (RMNET_ERR)
			printk("rmnet_rx_data:buf size insufficient to decode pkt length\n");
#endif
			past_packet.state = RMNET_FULL_PACKET;
			return;
		}

		if (tot_sz < hdr_size) {
			past_packet.state = RMNET_PARTIAL_HEADER;
			past_packet.size = tot_sz;
			memcpy(past_packet.buf, buf, tot_sz);
			past_packet.type = ver;
#if defined (RMNET_DEBUG)
			printk("rmnet_rx_data: partial header packet copied locally,"
				" sz = %d\n", tot_sz);
#endif
			return;
		}

		if (ver == RMNET_IPV4_VER) {
			data_sz = ntohs(((struct iphdr*) ip_hdr)->tot_len);
		} else if (ver == RMNET_IPV6_VER) {
			data_sz = ntohs(((struct ipv6hdr*) ip_hdr)->payload_len) + sizeof(struct ipv6hdr);
		} else {
#if defined (RMNET_ERR)
		printk("rmnet_rx_data: data sz check -- "
					"Invalid version, %d\n",ver);
#endif
			past_packet.state = RMNET_FULL_PACKET;
			break;
		}
 printk("rmnet_rx_data: data_sz = %d \n", data_sz);
 
#if defined (RMNET_DEBUG)
		printk("rmnet_rx_data: data size = %d\n", data_sz);
#endif

		if (tot_sz < data_sz) {
			past_packet.state = RMNET_PARTIAL_PACKET;
			past_packet.size = tot_sz;
			memcpy(past_packet.buf, buf, tot_sz);
			past_packet.type = ver;
#if defined (RMNET_DEBUG)
			printk("rmnet_rx_data: partial data packet copied locally,"
					" sz = %d\n", tot_sz);
#endif
			return;
		}

		rmnet_trans_packet(dev, ver, buf, data_sz);
#if defined (RMNET_DEBUG)
		printk("rmnet_rx_data: pushed full data packet to tcpip, "
			"sz = %d\n", data_sz);
#endif
		tot_sz = tot_sz - data_sz;
		buf = buf + data_sz;
#if defined (RMNET_DEBUG)
		printk("rmnet_rx_data: looping for another packet"
				" tot_sz = %d\n", tot_sz);
#endif
	}

	past_packet.state = RMNET_FULL_PACKET;
}

static int rmnet_open(struct net_device *dev)
{
	struct rmnet_private *p = netdev_priv(dev);
  int rmnet_dev_idx = p->tty_index - RMNET_TTY_IDX_BASE;

	dev->flags&=~IFF_MULTICAST;
	dev->flags&=~IFF_BROADCAST;

	pr_info("rmnet_open() tty_index %d\n", rmnet_dev_idx);

#ifdef DEBUG_RMNET_LOG
  log_open(p->tty_index);
#endif
	if(!rmnet_dev[rmnet_dev_idx])
  {
		rmnet_dev[rmnet_dev_idx] = dev;
	}

	netif_start_queue(dev);
	return 0;
}

static int rmnet_stop(struct net_device *dev)
{
	struct rmnet_private *p = netdev_priv(dev);

#ifdef DEBUG_RMNET_LOG
  log_close(p->tty_index);
#endif

#if 1 //def RMNET_DEBUG
	printk("rmnet_stop() tty_index %d\n", p->tty_index);
#endif
	netif_stop_queue(dev);

	return 0;
}

void rmnet_restart_queue(int chno)
{
	int index;
#if 0
	if(rmnet_ch_block_info[chno].blocked) {
		rmnet_ch_block_info[chno].blocked = 0;
		netif_wake_queue(rmnet_ch_block_info[chno].dev);
#ifdef CONFIG_RMNET_DEBUG
	//	printk("rmnet: FIFO free so unblocking rmnet %d queue\n", chno);
#endif
	}
#else
	if(chno < RMNET_TTY_IDX_BASE || chno >= (RMNET_TTY_IDX_BASE+RMNET_DEV_NUM)) {
		printk("rmnet: rmnet_restart_queue chno = %d is invalid\n", chno);
		return; 
	}
	index = chno - 	RMNET_TTY_IDX_BASE;
	if(rmnet_dev[index])
		netif_wake_queue(rmnet_dev[index]);

#if 1 //def RMNET_DEBUG
	printk("rmnet_restart_queue chnox %d\n", chno);
#endif
		
#endif

}
 

static int rmnet_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct rmnet_private *p = netdev_priv(dev);
	int ret;
	struct tty_struct tty; 


#if defined (RMNET_CRITICAL_DEBUG)
   printk("\nRMNET skb_len= %d>\n", skb->len);
#endif
	if((skb->len - RMNET_ETH_HDR_SIZE) <= 0) {
#ifdef CONFIG_RMNET_DEBUG
	printk("\nrmnet: Got only header for , return\n");
#endif
		ret = NETDEV_TX_OK;
		dev_kfree_skb_irq(skb);
		goto quit_xmit;
	}

	rmnet_stop(dev);
	
	tty.index = p->tty_index;
#ifdef RMNET_DEBUG	
  printk("\nrmnet: rmnet_xmit len = %d on tty %d\n", skb->len,tty.index);
#endif  
#ifdef DEBUG_RMNET_LOG
  log_write(p->tty_index, ((char *) skb->data + RMNET_ETH_HDR_SIZE), skb->len-RMNET_ETH_HDR_SIZE);
#endif
	mux_write(&tty, ((char *) skb->data + RMNET_ETH_HDR_SIZE), skb->len-RMNET_ETH_HDR_SIZE);
	{
		if (count_this_packet(skb->data, skb->len)) {
			p->stats.tx_packets++;
			p->stats.tx_bytes += skb->len;
#ifdef CONFIG_RMNET_DEBUG
			p->wakeups_xmit += rmnet_cause_wakeup(p);
#endif
		}
	}
	ret = NETDEV_TX_OK;
	dev_kfree_skb_irq(skb);

quit_xmit:
	return ret;
}

static struct net_device_stats *rmnet_get_stats(struct net_device *dev)
{
	struct rmnet_private *p = netdev_priv(dev);
	return &p->stats;
}

static void rmnet_set_multicast_list(struct net_device *dev)
{
}

static void rmnet_tx_timeout(struct net_device *dev)
{

//	int chno;
	pr_info("rmnet_tx_timeout()\n");
#if 0
	for(chno=0; chno < 16; chno++) {
		if(rmnet_ch_block_info[chno].dev == dev) {
			rmnet_restart_queue(chno);
			break;
		}
	}
#endif
}

static struct net_device_ops rmnet_ops = {
	.ndo_open = rmnet_open,
	.ndo_stop = rmnet_stop,
	.ndo_start_xmit = rmnet_xmit,
	.ndo_get_stats = rmnet_get_stats,
	.ndo_set_multicast_list = rmnet_set_multicast_list,
	.ndo_tx_timeout = rmnet_tx_timeout,
};

static void __init rmnet_setup(struct net_device *dev)
{
	dev->netdev_ops = &rmnet_ops;

	dev->watchdog_timeo = RMNET_WD_TMO;
	ether_setup(dev);
	dev->mtu = RMNET_MTU_SIZE;
	dev->flags &= ~IFF_MULTICAST;
	dev->flags &= ~IFF_BROADCAST;
	dev->flags |= IFF_NOARP;

	random_ether_addr(dev->dev_addr);
}

static int __init rmnet_init(void)
{
	int ret;
	struct device *d;
	struct net_device *dev;
	struct rmnet_private *p;
	unsigned n;

#ifdef CONFIG_RMNET_DEBUG
	timeout_us = 0;
#ifdef CONFIG_HAS_EARLYSUSPEND
	timeout_suspend_us = 0;
#endif
#endif

#ifdef CONFIG_RMNET_DEBUG
	rmnet_wq = create_workqueue("rmnet");
#endif

	for (n = 0; n < RMNET_DEV_NUM; n++) {
		dev = alloc_netdev(sizeof(struct rmnet_private),
				   "rmnet%d", rmnet_setup);

		if (!dev)
			return -ENOMEM;
			
        /* For timeout check */			
        dev->watchdog_timeo = (6 * HZ);
        
		d = &(dev->dev);
		p = netdev_priv(dev);
    p->tty_index = RMNET_TTY_IDX_BASE + n;

		wake_lock_init(&p->wake_lock,
						WAKE_LOCK_SUSPEND,
						"channel1");
#ifdef CONFIG_RMNET_DEBUG
		p->timeout_us = timeout_us;
		p->awake_time_ms = p->wakeups_xmit = p->wakeups_rcv = 0;
		p->active_countdown = p->restart_count = 0;
		INIT_DELAYED_WORK_DEFERRABLE(&p->work, do_check_active);
#endif

		ret = register_netdev(dev);
		if (ret) {
			free_netdev(dev);
			return ret;
		}

#ifdef CONFIG_RMNET_DEBUG
		if (device_create_file(d, &dev_attr_timeout))
			continue;
		if (device_create_file(d, &dev_attr_wakeups_xmit))
			continue;
		if (device_create_file(d, &dev_attr_wakeups_rcv))
			continue;
		if (device_create_file(d, &dev_attr_awake_time_ms))
			continue;
#ifdef CONFIG_HAS_EARLYSUSPEND
		if (device_create_file(d, &dev_attr_timeout_suspend))
			continue;

		/* Only care about rmnet0 for suspend/resume tiemout hooks. */
		if (n == 0)
			rmnet0 = d;
#endif
#endif
	}
	past_packet.size = 0;
	past_packet.state = RMNET_FULL_PACKET;
	return 0;
}

module_init(rmnet_init);
