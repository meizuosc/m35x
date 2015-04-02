/****************************************************************************
 *
 * Driver for the SC880xG spi modem.
 *
 * Copyright (C) 2013, Meizu Corp
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA
 *
 * Driver modified by Meizu from Option ifx6x60.c
*
 *
 *****************************************************************************/
#include <linux/module.h>
#include <linux/termios.h>
#include <linux/tty.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/kfifo.h>
#include <linux/tty_flip.h>
#include <linux/timer.h>
#include <linux/serial.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/rfkill.h>
#include <linux/fs.h>
#include <linux/ip.h>
#include <linux/dmapool.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif

#include <asm/mach-types.h>

#include <mach/modem_sc8803g.h>

#include "sc8803g_spt_spi.h"


#define SPT_SPI_CHECKSUM_EN	1
#define SPT_SPI_TTY_ID			0
#define SPT_SPI_TIMEOUT_SEC		2

#define SPT_GPIO_MRTS_ACTIVE_LEVEL		1
#define SPT_GPIO_MRDY_ACTIVE_LEVEL		0
#define SPT_GPIO_MRSD_ACTIVE_LEVEL		1
#define SPT_GPIO_SRTS_ACTIVE_LEVEL		0
#define SPT_GPIO_SRDY_ACTIVE_LEVEL		0
#define SPT_GPIO_SRSD_ACTIVE_LEVEL		1

#define SPT_SPI_TRANSFER_ALIGN_SIZE		64u
#define SPT_SPI_TRANSFER_ALIGN_MASK		63u

static u32	 frame_num = 0;

/* forward reference */
static int spt_spi_reset(struct spt_spi_device *spt_dev);

/* local variables */
static struct tty_driver *spt_tty_drv;
static struct spt_spi_device *saved_spt_dev;
static struct lock_class_key spt_spi_key;

static int sc8803x_modem_debug = 0;

#define LOG_TAG "[MODEM] "
#define MDM_ERR(fmt, ...) pr_err(LOG_TAG "%s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)

#define MDM_INFO(fmt, ...) if (sc8803x_modem_debug > 0) \
	pr_info(LOG_TAG "%s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)

#define MDM_TRACE(fmt, ...) if (sc8803x_modem_debug > 1) \
	pr_info("mif: %s: %d: called(%pF): " fmt, __func__,\
			__LINE__, __builtin_return_address(0), ##__VA_ARGS__)

#define MDM_DEBUG(fmt, ...) if (sc8803x_modem_debug > 2) \
	pr_info(LOG_TAG "%s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)


static int spt_show_modem_gpio(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct spt_spi_device *spt_dev = saved_spt_dev;

	char temp[256];

	memset(temp, 0, 256);

	sprintf(temp, "Modem RTS=%d\n", gpio_get_value(spt_dev->gpio.srts));
	strcat(buf, temp);

	sprintf(temp, "AP RTS=%d\n", gpio_get_value(spt_dev->gpio.mrts));
	strcat(buf, temp);
	
	sprintf(temp, "Modem  RDY=%d\n", gpio_get_value(spt_dev->gpio.srdy));
	strcat(buf, temp);

	sprintf(temp, "AP RDY=%d\n", gpio_get_value(spt_dev->gpio.mrdy));
	strcat(buf, temp);

	sprintf(temp, "Modem RESEND=%d\n", gpio_get_value(spt_dev->gpio.srsd));
	strcat(buf, temp);

	sprintf(temp, "AP RESEND=%d\n", gpio_get_value(spt_dev->gpio.mrsd));
	strcat(buf, temp);

	sprintf(temp, "Modem POWER=%d\n", gpio_get_value(spt_dev->gpio.po));
	strcat(buf, temp);

	sprintf(temp, "RESET_INT(Modem ALIVE)=%d\n", gpio_get_value(spt_dev->gpio.reset_out));
	strcat(buf, temp);
	
	return strlen(buf);
}

static DEVICE_ATTR(modem_gpio, 0644, \
			spt_show_modem_gpio, NULL);

static int spt_store_modem_reset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	struct spt_spi_device *spt_dev = saved_spt_dev;

	dev_info(dev, "modem reset\n");
	spt_spi_reset(spt_dev);
	
	return len;
}

static DEVICE_ATTR(modem_reset, 0644, \
			NULL, spt_store_modem_reset);

static int spt_show_modem_debug(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *p = buf;

	p += scnprintf(p, PAGE_SIZE, "%d\n", sc8803x_modem_debug);

	return p - buf;
}

static int spt_store_modem_debug(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	int error;
	unsigned long val;

	error = strict_strtoul(buf, 10, &val);
	if (!error){
		dev_info(dev, "%s(%d)\n", buf, (int)val);
		sc8803x_modem_debug = val;
	}
	return len;
}

static DEVICE_ATTR(modem_debug, 0644, \
			spt_show_modem_debug,
			spt_store_modem_debug);


static struct attribute *spt_tty_attributes[] = {
	&dev_attr_modem_gpio.attr,
	&dev_attr_modem_reset.attr,
	&dev_attr_modem_debug.attr,
	NULL
};


static const struct attribute_group spt_tty_group = {
	.attrs = spt_tty_attributes,
};
#ifdef CONFIG_HAS_WAKELOCK
static void spt_spi_wake_lock_initial(struct spt_spi_device *spt_dev)
{
	wake_lock_init(&spt_dev->spt_spi_wakelock, WAKE_LOCK_SUSPEND, "modemspi");
}

static void spt_spi_wake_lock_destroy(struct spt_spi_device *spt_dev)
{
	wake_lock_destroy(&spt_dev->spt_spi_wakelock);
}

static void spt_spi_wake_lock(struct spt_spi_device *spt_dev)
{
	wake_lock(&spt_dev->spt_spi_wakelock);
}

static void spt_spi_wake_lock_timeout(struct spt_spi_device *spt_dev,  int timeout)
{
	wake_lock_timeout(&spt_dev->spt_spi_wakelock, timeout);
}
#else
static void spt_spi_wake_lock_initial(struct spt_spi_device *spt_dev){}

static void spt_spi_wake_lock_destroy(struct spt_spi_device *spt_dev){}

static void spt_spi_wake_lock(struct spt_spi_device *spt_dev){}

static void spt_spi_wake_lock_timeout(struct spt_spi_device *spt_dev,  int timeout){}
#endif
/* GPIO settings */
/**
 *	spt_spi_mrdy_set		-	set MRDY GPIO
 *	@spt_dev: device we are controlling
 *
 */
static inline void spt_spi_mrdy_set(struct spt_spi_device *spt_dev, bool active)
{
	gpio_set_value(spt_dev->gpio.mrdy, active==SPT_GPIO_MRDY_ACTIVE_LEVEL?1:0);
	MDM_DEBUG("Current operation is %s [AP_RDY is %s]\n", active ? "active" : "inactive", 
		(gpio_get_value(spt_dev->gpio.mrdy) == SPT_GPIO_MRDY_ACTIVE_LEVEL) ? "active" : "not active");
}
/**
 *	spt_spi_mrdy_get		-	set MRDY GPIO
 *	@spt_dev: device we are controlling
 *
 */
static inline int spt_spi_mrdy_get(struct spt_spi_device *spt_dev)
{
	return (gpio_get_value(spt_dev->gpio.mrdy)==SPT_GPIO_MRDY_ACTIVE_LEVEL)?1:0;
}
/**
 *	spt_spi_mrts_set		-	set MRST GPIO
 *	@spt_dev: device we are controlling
 *
 */
static inline void spt_spi_mrts_set(struct spt_spi_device *spt_dev, bool active)
{
	gpio_set_value(spt_dev->gpio.mrts, active==SPT_GPIO_MRTS_ACTIVE_LEVEL?1:0);
	MDM_DEBUG("Current operation is %s [AP_RTS is %s]\n", active ? "active" : "inactive", 
		(gpio_get_value(spt_dev->gpio.mrts) == SPT_GPIO_MRTS_ACTIVE_LEVEL) ? "active" : "not active");
}
/**
 *	spt_spi_mrts_get		-	set MRST GPIO
 *	@spt_dev: device we are controlling
 *
 */
static inline int spt_spi_mrts_get(struct spt_spi_device *spt_dev)
{
	return (gpio_get_value(spt_dev->gpio.mrts)==SPT_GPIO_MRTS_ACTIVE_LEVEL)?1:0;
}
/**
 *	spt_spi_mrsd_set		-	set MRSD GPIO
 *	@spt_dev: device we are controlling
 *
 */
static inline void spt_spi_mrsd_set(struct spt_spi_device *spt_dev, bool active)
{
	gpio_set_value(spt_dev->gpio.mrsd, active==SPT_GPIO_MRSD_ACTIVE_LEVEL?1:0);
	MDM_DEBUG("Current operation is %s [AP_RESEND is %s]\n", active ? "active" : "inactive", 
		(gpio_get_value(spt_dev->gpio.mrsd) == SPT_GPIO_MRSD_ACTIVE_LEVEL) ? "active" : "not active");
}
/**
 *	spt_spi_mrsd_get		-	set MRSD GPIO
 *	@spt_dev: device we are controlling
 *
 */
static inline int spt_spi_mrsd_get(struct spt_spi_device *spt_dev)
{
	return (gpio_get_value(spt_dev->gpio.mrsd)==SPT_GPIO_MRSD_ACTIVE_LEVEL)?1:0;
}


/**
 *	spt_spi_srdy_get		-	set SRDY GPIO
 *	@spt_dev: device we are controlling
 *
 */
static inline int spt_spi_srdy_get(struct spt_spi_device *spt_dev)
{
	return (gpio_get_value(spt_dev->gpio.srdy)==SPT_GPIO_SRDY_ACTIVE_LEVEL)?1:0;
}

/**
 *	spt_spi_srts_get		-	set SRST GPIO
 *	@spt_dev: device we are controlling
 *
 */
static inline int spt_spi_srts_get(struct spt_spi_device *spt_dev)
{
	return (gpio_get_value(spt_dev->gpio.srts)==SPT_GPIO_SRTS_ACTIVE_LEVEL)?1:0;
}

/**
 *	spt_spi_srsd_get		-	set SRSD GPIO
 *	@spt_dev: device we are controlling
 *
 */
static inline int spt_spi_srsd_get(struct spt_spi_device *spt_dev)
{
	return (gpio_get_value(spt_dev->gpio.srsd)==SPT_GPIO_SRSD_ACTIVE_LEVEL)?1:0;
}

/**
 *	spt_spi_swap_buf_htn
 *	@buf: our buffer
 *	@len : number of bytes (not words) in the buffer
 *
 *	Swap the contents of a buffer into big endian format
 */
static inline void spt_spi_swap_buf_htn(u8 *paket, int len)
{
	int n;
	u32 *buf = (u32*)paket;

	len = ((len + 3) >> 2);
	for (n = 0; n < len; n++) {
		*buf = htonl(*buf);
		buf++;
	}
}
/**
 *	spt_spi_swap_buf_nth
 *	@buf: our buffer
 *	@len : number of bytes (not words) in the buffer
 *
 *	Swap the contents of a buffer into big endian format
 */
static inline void spt_spi_swap_buf_nth(u8 *paket, int len)
{
	int n;
	u32 *buf = (u32*)paket;

	len = ((len + 3) >> 2);
	for (n = 0; n < len; n++) {
		*buf = ntohl(*buf);
		buf++;
	}
}

/**
 *	spt_spi_mrts_assert		-	assert MRTS line
 *	@spt_dev: our SPI device
 *
 *	Assert mrts and set timer to wait for SRTS interrupt, if SRTS is low
 *	now.
 *
 *	FIXME: Can SRTS even go high as we are running this code ?
 */
static void spt_spi_mrts_assert(struct spt_spi_device *spt_dev)
{
	int val1, val2;
	bool isStartTimer = false;

	val1 = spt_spi_srdy_get(spt_dev);
	val2 = spt_spi_srts_get(spt_dev);

	if (!(val1 || val2)) {
		if (!test_and_set_bit(SPT_SPI_STATE_TIMER_PENDING,
				      &spt_dev->flags)) {
			spt_dev->spi_timer.expires =
				jiffies + SPT_SPI_TIMEOUT_SEC*HZ;
			add_timer(&spt_dev->spi_timer);
			isStartTimer = true;
		}
	}
	MDM_DEBUG("%s SPI Timer [MDM_RDY:%s, MDM_RTS:%s].\n", isStartTimer ? "start" : "not start",
		val1 ? "ready" : "not ready",
		val2 ? "active" : "not active");
	spt_spi_mrts_set(spt_dev, true);
}

/**
 *	spt_spi_mrts_assert		-	set MRDY line
 *	@spt_dev: our SPI device
 *
 *	Assert mrdy to inform modem, ap is ready to receive data.
 *
 */
static void spt_spi_receive_response(struct spt_spi_device *spt_dev)
{
	spt_spi_mrdy_set(spt_dev, true);
}

/**
 *	spt_spi_receive_ack		-	unset MRDY line
 *	@spt_dev: our SPI device
 *
 *	Unset mrdy to inform modem, ap receive data is finished.
 *
 */
static void spt_spi_receive_ack(struct spt_spi_device *spt_dev)
{
	int try_count = 100;

	spt_spi_mrdy_set(spt_dev, false);
	while(1){
		if((try_count--) <= 0 || !spt_spi_srts_get(spt_dev)){
			break;
		}
		udelay(100);
	}
	if(try_count == 0)
		pr_info("%s: time out=%d:%d\n", __func__, try_count, spt_spi_srts_get(spt_dev));
}

/**
 *	spt_spi_send_end		-	unset MRST line
 *	@spt_dev: our SPI device
 *
 *	Unset mrts to inform modem, ap send data is finished.
 *
 */
static void spt_spi_send_end(struct spt_spi_device *spt_dev)
{
	int try_count = 100;

	spt_spi_mrts_set(spt_dev, false);
	while(1){
		if((try_count--) <= 0 || !spt_spi_srdy_get(spt_dev)){
			break;
		}
		udelay(100);
	}
	if(try_count == 0)
		pr_info("%s: time out=%d:%d\n", __func__, try_count, spt_spi_srdy_get(spt_dev));
}

/**
 *	spt_spi_request_resend		-	set MRSD line
 *	@spt_dev: our SPI device
 *
 *	Assert mrts to inform modem, ap reciver data fail! modem must resand last packet.
 *
 */
static void spt_spi_request_resend(struct spt_spi_device *spt_dev)
{
	spt_spi_mrsd_set(spt_dev, true);
}

/**
 *	spt_spi_end_resend		-	unset MRSD line
 *	@spt_dev: our SPI device
 *
 *	Unset mrsd to inform modem, resend process is finished.
 *
 */
static void spt_spi_end_resend(struct spt_spi_device *spt_dev)
{
	spt_spi_mrsd_set(spt_dev, false);
}

/**
 *	spt_spi_timeout		-	SPI timeout
 *	@arg: our SPI device
 *
 *	The SPI has timed out: hang up the tty. Users will then see a hangup
 *	and error events.
 */
static void spt_spi_timeout(unsigned long arg)
{
	struct spt_spi_device *spt_dev = (struct spt_spi_device *)arg;

	dev_warn(&spt_dev->spi_dev->dev, "*** 2S SPI Timeout [io_process:%s, io_ready:%s, unack:%d] ***\n",
		test_bit(SPT_SPI_STATE_IO_IN_PROGRESS, &spt_dev->flags) ? "set" : "not",
		test_bit(SPT_SPI_STATE_IO_READY, &spt_dev->flags) ? "set" : "not",
		spt_dev->gpio.unack_srdy_int_nb);
	spt_spi_mrts_set(spt_dev, false);
	spt_spi_mrdy_set(spt_dev, false);
	kfifo_reset(&spt_dev->tx_fifo);
	clear_bit(SPT_SPI_STATE_TIMER_PENDING, &spt_dev->flags);
}

/* char/tty operations */
/**
 *	spt_spi_open	-	called on tty open
 *	@tty: our tty device
 *	@filp: file handle being associated with the tty
 *
 *	Open the tty interface. We let the tty_port layer do all the work
 *	for us.
 *
 *	FIXME: Remove single device assumption and saved_spt_dev
 */
static int spt_spi_open(struct tty_struct *tty, struct file *filp)
{
	MDM_DEBUG("modem tty open\n");
	return tty_port_open(&saved_spt_dev->tty_port, tty, filp);
}

/**
 *	spt_spi_close	-	called when our tty closes
 *	@tty: the tty being closed
 *	@filp: the file handle being closed
 *
 *	Perform the close of the tty. We use the tty_port layer to do all
 *	our hard work.
 */
static void spt_spi_close(struct tty_struct *tty, struct file *filp)
{
	struct spt_spi_device *spt_dev = tty->driver_data;
	tty_port_close(&spt_dev->tty_port, tty, filp);
	/* FIXME: should we do an spt_spi_reset here ? */
	MDM_DEBUG("modem tty close\n");
}


static bool spt_spi_get_packet_header(const u8 * packet, struct spt_spi_packet_header *header)
{
	u32 len;
	u32 i;
	u8 *data = (u8 *)header;

	len = sizeof (struct spt_spi_packet_header);
	/*check header*/
	for (i = 0; i < len; i++) 
	{
		data[i] = packet[i];
	}

	if ( (header->tag != SPT_SPI_HEADER_TAG) || (header->type != SPT_SPI_HEADER_TYPE)
	|| (header->length > SPT_SPI_PAYLOAD_SIZE))
	{
		MDM_ERR("[SPI_ERROR]Receive packet tag(0x%02x), head.type(0x%02x), head.lenght(%d)\n",\
				header->tag, header->type, header->length);
		return false;
	}
	MDM_INFO("SPI Receive packet size(%d)\n", header->length);
	return true;
}

static bool spt_spi_packet_verify(const u8 * packet, struct spt_spi_packet_header *header)
{
#ifdef SPT_SPI_CHECKSUM_EN
	u32 i;
	u32  sum_packet  = 0;
#endif

#ifdef SPT_SPI_CHECKSUM_EN /*<CR:NEWMS00178051 begin add check sum function*/
	if ( (header->tag != SPT_SPI_HEADER_TAG) || (header->type != SPT_SPI_HEADER_TYPE)
	|| (header->length > SPT_SPI_PAYLOAD_SIZE))
	{
		MDM_ERR("SPI Receive packet tag(0x%02x), head.type(0x%02x), head.lenght(%d)\n",\
				header->tag, header->type, header->length);
		return false;
	}

	/*check sum*/
	for (i = sizeof (struct spt_spi_packet_header) ;  i < (header->length + sizeof (struct spt_spi_packet_header)) ; i++) 
	{
		sum_packet += packet[i];
	}

	if(header->reserved2 == sum_packet)      
	{
		MDM_INFO("SPI Receive packet size(%d)\n", header->length);
		return true;
	}

	MDM_ERR("SPI Receive packet sum(%d) != head.reserved2(%d)",\
			sum_packet,header->reserved2);
	return false;

#else
	if ( (header->tag != SPT_SPI_HEADER_TAG) || (header->type != SPT_SPI_HEADER_TYPE)
	|| (header->length > SPT_SPI_PAYLOAD_SIZE))
	{
		MDM_ERR("[SPI_ERROR]Receive packet tag(0x%02x), head.type(0x%02x), head.lenght(%d)\n",\
				header->tag, header->type, header->length);
		return false;
	}
	MDM_INFO("SPI Receive packet size(%d)\n", header->length);
	return true;
#endif /*CR:NEWMS00178051 end add check sum function>*/ 
}

/**
 *	spt_spi_wakeup_serial	-	SPI space made
 *	@port_data: our SPI device
 *
 *	We have emptied the FIFO enough that we want to get more data
 *	queued into it. Poke the line discipline via tty_wakeup so that
 *	it will feed us more bits
 */
static void spt_spi_wakeup_serial(struct spt_spi_device *spt_dev)
{
	struct tty_struct *tty;

	tty = tty_port_tty_get(&spt_dev->tty_port);
	if (!tty)
		return;
	tty_wakeup(tty);
	tty_kref_put(tty);
}

/*****************************************************************************/
//  Description: Setup Packet function
//  Author:      yanxinsheng    
//  Note:        The data should be aligned 64 bytes when use SPI DMA. Other
//               should be aligned 4 bytes
//  Date:        2011-02-19
/*****************************************************************************/
static void spt_spi_setup_packet(u8 * packet, u32 len_ret)
{
	struct spt_spi_packet_header header;

#ifdef SPT_SPI_CHECKSUM_EN /*<CR:NEWMS00178051 begin add check sum function*/
	u8 *data = packet;
	u32 i;
	u32 sum = 0;

	header.tag = SPT_SPI_HEADER_TAG;
	header.type = SPT_SPI_HEADER_TYPE;
	header.length = len_ret;
	header.frame_num = frame_num++;
	/*calculate SPI data sum*/
	for(i = sizeof(struct spt_spi_packet_header); i < (len_ret + sizeof(struct spt_spi_packet_header)); i++)
	{
		sum += data[i];
	}
	header.reserved2 =  sum;
#else
	header.tag = SPT_SPI_HEADER_TAG;
	header.type = SPT_SPI_HEADER_TYPE;
	header.length = len_ret;
	header.frame_num = frame_num; //sending_cnt;
	header.reserved2 = 0xabcdef00;
#endif /*CR:NEWMS00178051 end add check sum function>*/
	memcpy(packet, &header, sizeof(struct spt_spi_packet_header));
	MDM_INFO("SPI send packet size(%d)\n", header.length);
    return;
}

/**
 *	spt_spi_prepare_tx_buffer	-	prepare transmit frame
 *	@spt_dev: our SPI device
 *
 *	The transmit buffr needs a header and various other bits of
 *	information followed by as much data as we can pull from the FIFO
 *	and transfer. This function formats up a suitable buffer in the
 *	spt_dev->tx_buffer
 *
 *	FIXME: performance - should we wake the tty when the queue is half
 *			     empty ?
 */
static int spt_spi_prepare_tx_buffer(struct spt_spi_device *spt_dev)
{
	int temp_count;
	int queue_length;
	int tx_count;
	unsigned char *tx_buffer;

	tx_buffer = spt_dev->tx_buffer;
	memset(tx_buffer, 0, SPT_SPI_TRANSFER_SIZE);

	/* make room for required SPI header */
	tx_buffer += SPT_SPI_HEADER_OVERHEAD;
	tx_count = SPT_SPI_HEADER_OVERHEAD;

	/* clear to signal no more data if this turns out to be the
	 * last buffer sent in a sequence */
	spt_dev->spi_more = 0;

	/* if modem cts is set, just send empty buffer */
	if (!spt_dev->spi_slave_cts) {
		/* see if there's tx data */
		queue_length = kfifo_len(&spt_dev->tx_fifo);
		if (queue_length != 0) {
			/* data to mux -- see if there's room for it */
			temp_count = min(queue_length, SPT_SPI_TX_PAYLOAD_SIZE);
			temp_count = kfifo_out_peek(&spt_dev->tx_fifo,
					tx_buffer, temp_count);

			/* update buffer pointer and data count in message */
			tx_buffer += temp_count;
			tx_count += temp_count;
			if (temp_count != queue_length)
				/* more data in port, use next SPI message */
				spt_dev->spi_more = 1;
		}
	}
	/* have data and info for header -- set up SPI header in buffer */
	/* spi header needs payload size, not entire buffer size */
	spt_spi_setup_packet(spt_dev->tx_buffer, tx_count-SPT_SPI_HEADER_OVERHEAD);
//	spt_spi_swap_buf_htn(spt_dev->tx_buffer, tx_count);
	return tx_count;
}

/**
 *	spt_spi_write		-	line discipline write
 *	@tty: our tty device
 *	@buf: pointer to buffer to write (kernel space)
 *	@count: size of buffer
 *
 *	Write the characters we have been given into the FIFO. If the device
 *	is not active then activate it, when the SRDY line is asserted back
 *	this will commence I/O
 */
static int spt_spi_write(struct tty_struct *tty, const unsigned char *buf,
			 int count)
{
	struct spt_spi_device *spt_dev = tty->driver_data;
	unsigned char *tmp_buf = (unsigned char *)buf;
	int tx_count = kfifo_in_locked(&spt_dev->tx_fifo, tmp_buf, count,
				   &spt_dev->fifo_lock);
	spt_spi_wake_lock(spt_dev);

	spt_dev->gpio.unack_srdy_int_nb = 1;
	if (!test_bit(SPT_SPI_STATE_IO_IN_PROGRESS, &spt_dev->flags))
		wake_up_interruptible(&spt_dev->mdm_event_wait);
	else
		set_bit(SPT_SPI_STATE_IO_READY, &spt_dev->flags);
	
	MDM_DEBUG("count=%d, unack:%d [%s]\n", tx_count, spt_dev->gpio.unack_srdy_int_nb,
		(!test_bit(SPT_SPI_STATE_IO_IN_PROGRESS, &spt_dev->flags)) ? "not in io process" : "in io process");
	return tx_count;
}

/**
 *	spt_spi_write_room	-	line discipline helper
 *	@tty: our tty device
 *
 *	Report how much data we can accept before we drop bytes. As we use
 *	a simple FIFO this is nice and easy.
 */
static int spt_spi_write_room(struct tty_struct *tty)
{
	struct spt_spi_device *spt_dev = tty->driver_data;
	MDM_DEBUG("count=%d\n", SPT_SPI_TXFIFO_SIZE - kfifo_len(&spt_dev->tx_fifo));
	return SPT_SPI_TXFIFO_SIZE - kfifo_len(&spt_dev->tx_fifo);
}

/**
 *	spt_port_activate
 *	@port: our tty port
 *
 *	tty port activate method - called for first open. Serialized
 *	with hangup and shutdown by the tty layer.
 */
static int spt_port_activate(struct tty_port *port, struct tty_struct *tty)
{
	struct spt_spi_device *spt_dev =
		container_of(port, struct spt_spi_device, tty_port);

	/* clear any old data; can't do this in 'close' */
	kfifo_reset(&spt_dev->tx_fifo);
	/* put port data into this tty */
	tty->driver_data = spt_dev;

	/* allows flip string push from int context */
	tty->low_latency = 0; //=1;
	MDM_ERR("modem tty active\r\n");
	return 0;
}

/**
 *	spt_port_shutdown
 *	@port: our tty port
 *
 *	tty port shutdown method - called for last port close. Serialized
 *	with hangup and activate by the tty layer.
 */
static void spt_port_shutdown(struct tty_port *port)
{
	struct spt_spi_device *spt_dev =
		container_of(port, struct spt_spi_device, tty_port);

	spt_spi_mrts_set(spt_dev, false);
	spt_spi_mrdy_set(spt_dev, false);
	del_timer_sync(&spt_dev->spi_timer);
	clear_bit(SPT_SPI_STATE_TIMER_PENDING, &spt_dev->flags);
	kfifo_reset(&spt_dev->tx_fifo);
	MDM_ERR("modem tty shutdown\r\n");
}

static const struct tty_port_operations spt_tty_port_ops = {
	.activate = spt_port_activate,
	.shutdown = spt_port_shutdown,
};

static const struct tty_operations spt_spi_serial_ops = {
	.open = spt_spi_open,
	.close = spt_spi_close,
	.write = spt_spi_write,
	.write_room = spt_spi_write_room,
};

/**
 *	spt_spi_insert_fip_string	-	queue received data
 *	@spt_ser: our SPI device
 *	@chars: buffer we have received
 *	@size: number of chars reeived
 *
 *	Queue bytes to the tty assuming the tty side is currently open. If
 *	not the discard the data.
 */
static void spt_spi_insert_flip_string(struct spt_spi_device *spt_dev,
				    unsigned char *chars, size_t size)
{
	struct tty_struct *tty = tty_port_tty_get(&spt_dev->tty_port);

	if (!tty)
		return;

	tty_insert_flip_string(tty, chars, size);
	tty_flip_buffer_push(tty);

	tty_kref_put(tty);
}

static int spt_spi_get_transfer_type(struct spt_spi_device *spt_dev)
{
	int srts_status = spt_spi_srts_get(spt_dev);//modem send request signal
	int srdy_status = spt_spi_srdy_get(spt_dev);//modem ready signal
	int srsd_status = spt_spi_srsd_get(spt_dev);//modem resend request signal
	int mrsd_status = spt_spi_mrsd_get(spt_dev);//ap resend request signal

	if(srts_status && mrsd_status)
		return SPT_SPI_TYPE_READ_RESEND;
	else if(srts_status)
		return SPT_SPI_TYPE_READ;
	else if(srsd_status && srdy_status)
		return SPT_SPI_TYPE_WRITE_RESEND;
	else if(srdy_status)
		return SPT_SPI_TYPE_WRITE;
	else
		return SPT_SPI_TYPE_NULL;
}
static unsigned int spt_spi_align_transfer_size(unsigned int i)
{
	return (i + SPT_SPI_TRANSFER_ALIGN_MASK) & ~SPT_SPI_TRANSFER_ALIGN_MASK;
}

static void spt_spi_setup_massege(struct spt_spi_device *spt_dev, unsigned int length, unsigned int offset)
{
	spi_message_init(&spt_dev->spi_msg);
	INIT_LIST_HEAD(&spt_dev->spi_msg.queue);

	/* set up our spi transfer */
	spt_dev->spi_xfer.len = spt_spi_align_transfer_size(length);
	spt_dev->spi_xfer.cs_change = 0;
	spt_dev->spi_xfer.speed_hz = spt_dev->spi_dev->max_speed_hz;
	spt_dev->spi_xfer.bits_per_word = spt_dev->spi_dev->bits_per_word;

	spt_dev->spi_xfer.tx_buf = spt_dev->tx_buffer + offset;
	spt_dev->spi_xfer.rx_buf = spt_dev->rx_buffer + offset;

	/*
	 * setup dma pointers
	 */
	if (spt_dev->use_dma) {
		spt_dev->spi_msg.is_dma_mapped = 1;
		spt_dev->tx_dma = spt_dev->tx_bus + offset;
		spt_dev->rx_dma = spt_dev->rx_bus + offset;
		spt_dev->spi_xfer.tx_dma = spt_dev->tx_dma;
		spt_dev->spi_xfer.rx_dma = spt_dev->rx_dma;
	} else {
		spt_dev->spi_msg.is_dma_mapped = 0;
		spt_dev->tx_dma = (dma_addr_t)0;
		spt_dev->rx_dma = (dma_addr_t)0;
		spt_dev->spi_xfer.tx_dma = (dma_addr_t)0;
		spt_dev->spi_xfer.rx_dma = (dma_addr_t)0;
	}

	spi_message_add_tail(&spt_dev->spi_xfer, &spt_dev->spi_msg);
	MDM_DEBUG("count=%d\n", spt_dev->spi_xfer.len);

}
static void spt_spi_request_tty_data(struct spt_spi_device *spt_dev)
{
	spt_spi_wakeup_serial(spt_dev);
}

/**
 *	spt_spi_handle_receive_data	-	SPI transfer completed
 *	@ctx: our SPI device
 *
 *	An SPI transfer has completed. Process any received data and kick off
 *	any further transmits we can commence.
 */
static int spt_spi_handle_receive_data(struct spt_spi_device *spt_dev)
{
	int actual_length;
	int decode_result;
	struct spt_spi_packet_header header;

	if (spt_dev->spi_msg.status) {
		dev_info(&spt_dev->spi_dev->dev, "SPI transfer error %d",
		       spt_dev->spi_msg.status);
		return -EIO;
	}
	MDM_DEBUG("SPI transfer receive data length = %d\n",
	       spt_dev->spi_msg.actual_length);

	/* check header validity, get comm flags */
	decode_result = spt_spi_get_packet_header(spt_dev->rx_buffer, &header);
	if(decode_result == false){
		MDM_ERR("ignore input: verify header fail\n");
		return -EINVAL;
	}

	/* check header validity, get comm flags */
	decode_result = spt_spi_packet_verify(spt_dev->rx_buffer, &header);
	if(decode_result == false){
		MDM_ERR("ignore input: verify packet fail\n");
		spt_dev->spi_slave_cts = 0;
		return -EINVAL;
	}

	actual_length = header.length;

	spt_spi_insert_flip_string(
		spt_dev,
		spt_dev->rx_buffer + SPT_SPI_HEADER_OVERHEAD,
		(size_t)actual_length);

	return 0;
}
static void spt_spi_receive_process(struct spt_spi_device *spt_dev)
{
	int retval;
	
	/*set mrdy to start receive data*/
	spt_spi_receive_response(spt_dev);
	spt_spi_setup_massege(spt_dev, SPT_SPI_TRANSFER_SIZE, 0);
	retval = spi_sync(spt_dev->spi_dev, &spt_dev->spi_msg);
	if (retval) {
		spt_spi_request_resend(spt_dev);
		spt_spi_receive_ack(spt_dev);
		pr_err("%s: *** spi_sync fail! ***\n", __func__);
		return;
	}
	if(spt_spi_handle_receive_data(spt_dev)){
		/*need request resend*/
		spt_spi_request_resend(spt_dev);
		spt_spi_receive_ack(spt_dev);
		pr_err("%s: *** request modem resend! ***\n", __func__);
		return;
	};
	spt_spi_end_resend(spt_dev);
	spt_spi_receive_ack(spt_dev);
}
static void spt_spi_send_process(struct spt_spi_device *spt_dev)
{
	int retval;
	int tx_length = 0;

	tx_length = spt_spi_prepare_tx_buffer(spt_dev);
	spt_spi_setup_massege(spt_dev, tx_length+SPT_SPI_TRANSFER_ALIGN_SIZE, 0);
	retval = spi_sync(spt_dev->spi_dev, &spt_dev->spi_msg);
	if (retval) {
		spt_spi_send_end(spt_dev);
		msleep(5);
		pr_err("%s: *** spi_sync fail! ***\n", __func__);
		return;
	} 
	spt_spi_send_end(spt_dev);
	/*check modem resend signal, to confirm send success*/
	if(spt_spi_srsd_get(spt_dev)){
		/*send fail, request resend*/
		pr_err("%s: *** modem request resend! ***\n", __func__);
		//spt_spi_mrts_set(spt_dev, true);
	}else{
		/*send success, remove tx massege from tx fifo*/
		tx_length = kfifo_out_locked(&spt_dev->tx_fifo,
				spt_dev->tx_buffer, tx_length-SPT_SPI_HEADER_OVERHEAD,
				&spt_dev->fifo_lock);
	}
	return;
}
static int spt_spi_thread(void *data)
{
	struct spt_spi_device *spt_dev = (struct spt_spi_device *) data;
	int status;
	
	dev_info(&spt_dev->spi_dev->dev, "%s: enter\n", __func__);	
	daemonize("spt_spi_thread");
	while (1) {
		status = wait_event_interruptible_timeout(spt_dev->mdm_event_wait, \
						spt_dev->gpio.unack_srdy_int_nb , msecs_to_jiffies(2000));
		if (test_bit(SPT_SPI_STATE_SUSPEND, &spt_dev->flags)) {
			msleep(100);
			continue;
		}
		if (test_bit(SPT_SPI_STATE_RESET, &spt_dev->flags)) {
			MDM_DEBUG("modem is in reset!\r\n");
			kfifo_reset(&spt_dev->tx_fifo);
			msleep(100);
			continue;
		}
		if (!test_and_set_bit(SPT_SPI_STATE_IO_IN_PROGRESS, &spt_dev->flags)) {
			if(spt_dev->gpio.unack_srdy_int_nb){
				spt_dev->gpio.unack_srdy_int_nb = 0;
				if (test_bit(SPT_SPI_STATE_TIMER_PENDING, &spt_dev->flags)) {
					del_timer_sync(&spt_dev->spi_timer);
					clear_bit(SPT_SPI_STATE_TIMER_PENDING, &spt_dev->flags);
					MDM_DEBUG("del SPI timer!\r\n");
				}
			}
			spt_dev->spi_work_type = spt_spi_get_transfer_type(spt_dev);
			switch(spt_dev->spi_work_type){
			case SPT_SPI_TYPE_READ:
			case SPT_SPI_TYPE_READ_RESEND:
				spt_spi_receive_process(spt_dev);
				spt_spi_wake_lock_timeout(spt_dev, 1*HZ);
				break;
			case SPT_SPI_TYPE_WRITE:
			case SPT_SPI_TYPE_WRITE_RESEND:
				spt_spi_send_process(spt_dev);
				spt_spi_wake_lock_timeout(spt_dev, 1*HZ);
				break;
			case SPT_SPI_TYPE_NULL:
			default:
				if(kfifo_len(&spt_dev->tx_fifo)>0)//trigger mrts if tx buffer no empty
					spt_spi_mrts_assert(spt_dev);
				else
					spt_spi_request_tty_data(spt_dev);
				break;
			}
			clear_bit(SPT_SPI_STATE_IO_IN_PROGRESS, &(spt_dev->flags));
			/* schedule output if there is more to do */
			if (test_and_clear_bit(SPT_SPI_STATE_IO_READY, &spt_dev->flags) || \
				(kfifo_len(&spt_dev->tx_fifo)>0 && !spt_spi_mrts_get(spt_dev)) ) {
				spt_dev->gpio.unack_srdy_int_nb = 1;
				MDM_DEBUG("handle in io ready!\r\n");
			}
		}
	}
	dev_info(&spt_dev->spi_dev->dev, "%s: exit\n", __func__);	
	return 0;
}

/**
 *	spt_spi_free_port	-	free up the tty side
 *	@spt_dev: SPT device going away
 *
 *	Unregister and free up a port when the device goes away
 */
static void spt_spi_free_port(struct spt_spi_device *spt_dev)
{
	if (spt_dev->tty_dev)
		tty_unregister_device(spt_tty_drv, spt_dev->minor);
	kfifo_free(&spt_dev->tx_fifo);
}

/**
 *	spt_spi_create_port	-	create a new port
 *	@spt_dev: our spi device
 *
 *	Allocate and initialise the tty port that goes with this interface
 *	and add it to the tty layer so that it can be opened.
 */
static int spt_spi_create_port(struct spt_spi_device *spt_dev)
{
	int ret = 0;
	struct tty_port *pport = &spt_dev->tty_port;

	spin_lock_init(&spt_dev->fifo_lock);
	lockdep_set_class_and_subclass(&spt_dev->fifo_lock,
		&spt_spi_key, 0);

	if (kfifo_alloc(&spt_dev->tx_fifo, SPT_SPI_TXFIFO_SIZE, GFP_KERNEL)) {
		ret = -ENOMEM;
		goto error_ret;
	}

	tty_port_init(pport);
	pport->ops = &spt_tty_port_ops;
	spt_dev->minor = SPT_SPI_TTY_ID;
	spt_dev->tty_dev = tty_register_device(spt_tty_drv, spt_dev->minor,
					       &spt_dev->spi_dev->dev);
	if (IS_ERR(spt_dev->tty_dev)) {
		dev_dbg(&spt_dev->spi_dev->dev,
			"%s: registering tty device failed", __func__);
		ret = PTR_ERR(spt_dev->tty_dev);
		goto error_ret;
	}
	ret = sysfs_create_group(&spt_dev->tty_dev->kobj, &spt_tty_group);
	if (ret) {
		dev_err(spt_dev->tty_dev, "(%d) Failed to create sysfs files\n", __LINE__);
		tty_unregister_device(spt_tty_drv, spt_dev->minor);
		goto error_ret;
	}
	return 0;

error_ret:
	spt_spi_free_port(spt_dev);
	return ret;
}

/**
 *	spt_spi_handle_srdy		-	handle SRDY
 *	@spt_dev: device asserting SRDY
 *
 *	Check our device state and see what we need to kick off when SRDY
 *	is asserted. This usually means killing the timer and firing off the
 *	I/O processing.
 */
static void spt_spi_handle_srdy(struct spt_spi_device *spt_dev)
{
	MDM_DEBUG("%s\n", (!test_bit(SPT_SPI_STATE_IO_IN_PROGRESS, &spt_dev->flags)) ? "not in io process" : "in io process");

	spt_dev->gpio.unack_srdy_int_nb = 1;
	if (!test_bit(SPT_SPI_STATE_IO_IN_PROGRESS, &spt_dev->flags))
		wake_up_interruptible(&spt_dev->mdm_event_wait);
	else
		set_bit(SPT_SPI_STATE_IO_READY, &spt_dev->flags);
	spt_spi_wake_lock_timeout(spt_dev, 2*HZ);
}

/**
 *	spt_spi_srdy_interrupt	-	SRDY asserted
 *	@irq: our IRQ number
 *	@dev: our spt_dev device
 *
 *	The modem asserted SRDY. Handle the srdy event
 */
static irqreturn_t spt_spi_srdy_interrupt(int irq, void *dev)
{
	struct spt_spi_device *spt_dev = dev;
	
	MDM_DEBUG("[MDM RDY/RTS IRQ] modem is %s, modem rts is %s\n", spt_spi_srdy_get(spt_dev) ? "ready" : "not ready",
		spt_spi_srts_get(spt_dev) ? "active" : "not active");

	spt_spi_handle_srdy(spt_dev);

	return IRQ_HANDLED;
}

/**
 *	spt_spi_free_device - free device
 *	@spt_dev: device to free
 *
 *	Free the SPT device
 */
static void spt_spi_free_device(struct spt_spi_device *spt_dev)
{
	spt_spi_free_port(spt_dev);
	dma_free_coherent(&spt_dev->spi_dev->dev,
				SPT_SPI_TRANSFER_SIZE,
				spt_dev->tx_buffer,
				spt_dev->tx_bus);
	dma_free_coherent(&spt_dev->spi_dev->dev,
				SPT_SPI_TRANSFER_SIZE,
				spt_dev->rx_buffer,
				spt_dev->rx_bus);
}

/**
 *	spt_spi_reset	-	reset modem
 *	@spt_dev: modem to reset
 *
 *	Perform a reset on the modem
 */
static int spt_spi_reset(struct spt_spi_device *spt_dev)
{
	/*
	 * set up modem power, reset
	 *
	 * delays are required on some platforms for the modem
	 * to reset properly
	 */
	gpio_set_value(spt_dev->gpio.po, 0);
	msleep(50);
	gpio_set_value(spt_dev->gpio.po, 1);

	return 0;
}

static int spt_spi_notifier_event(struct notifier_block *this,
		unsigned long event, void *ptr)
{
	struct spt_spi_device *spt_dev =
		container_of(this, struct spt_spi_device, crash_notifier);
	int rtn = NOTIFY_DONE;
	
	MDM_DEBUG("modem notify event is : %s\n", 
		event ? "out reset, not clear SPI timer" : "in reset, clear SPI timer");

	if(event)
	{
		/* exited reset */
		clear_bit(SPT_SPI_STATE_RESET, &spt_dev->flags);
		wake_up(&spt_dev->mdm_reset_wait);

	}else{
		/* entered reset */
		set_bit(SPT_SPI_STATE_RESET, &spt_dev->flags);
		del_timer_sync(&spt_dev->spi_timer);
		clear_bit(SPT_SPI_STATE_TIMER_PENDING, &spt_dev->flags);
	}
	return rtn;
}

/**
 *	spt_spi_probe	-	probe callback
 *	@spi: our possible matching SPI device
 *
 *	Probe for a sc880xg modem on SPI bus. Perform any needed device and
 *	GPIO setup.
 *
 *	FIXME:
 *	-	Support for multiple devices
 *	-	Split out MID specific GPIO handling eventually
 */

static int spt_spi_probe(struct spi_device *spi)
{
	int ret;
	int srdy;
	struct spt_modem_platform_data *pl_data;
	struct spt_spi_device *spt_dev;

	if (saved_spt_dev) {
		dev_dbg(&spi->dev, "ignoring subsequent detection");
		return -ENODEV;
	}

	pl_data = (struct spt_modem_platform_data *)spi->dev.platform_data;
	if (!pl_data) {
		dev_err(&spi->dev, "missing platform data!");
		return -ENODEV;
	}

	/* initialize structure to hold our device variables */
	spt_dev = kzalloc(sizeof(struct spt_spi_device), GFP_KERNEL);
	if (!spt_dev) {
		dev_err(&spi->dev, "spi device allocation failed");
		return -ENOMEM;
	}
	saved_spt_dev = spt_dev;
	spt_dev->spi_dev = spi;
	clear_bit(SPT_SPI_STATE_IO_IN_PROGRESS, &spt_dev->flags);
	spin_lock_init(&spt_dev->write_lock);
	spin_lock_init(&spt_dev->power_lock);
	spt_dev->power_status = 0;
	init_timer(&spt_dev->spi_timer);
	spt_dev->spi_timer.function = spt_spi_timeout;
	spt_dev->spi_timer.data = (unsigned long)spt_dev;
	spt_dev->modem = pl_data->modem_type;
	spt_dev->use_dma = pl_data->use_dma;
	spt_dev->max_hz = pl_data->max_hz;
	/* initialize spi mode, etc */
	spi->max_speed_hz = spt_dev->max_hz;
	spi->mode = pl_data->mode;
	spi->bits_per_word = pl_data->bit_per_word;
	ret = spi_setup(spi);
	if (ret) {
		dev_err(&spi->dev, "SPI setup wasn't successful %d", ret);
		return -ENODEV;
	}

	/* ensure SPI protocol flags are initialized to enable transfer */
	spt_dev->spi_more = 0;
	spt_dev->spi_slave_cts = 0;

	/*initialize transfer and dma buffers */
	spt_dev->tx_buffer = dma_alloc_coherent(spt_dev->spi_dev->dev.parent->parent,
				SPT_SPI_TRANSFER_SIZE,
				&spt_dev->tx_bus,
				GFP_KERNEL);
	if (!spt_dev->tx_buffer) {
		dev_err(&spi->dev, "DMA-TX buffer allocation failed");
		ret = -ENOMEM;
		goto error_ret;
	}
	spt_dev->rx_buffer = dma_alloc_coherent(spt_dev->spi_dev->dev.parent->parent,
				SPT_SPI_TRANSFER_SIZE,
				&spt_dev->rx_bus,
				GFP_KERNEL);
	if (!spt_dev->rx_buffer) {
		dev_err(&spi->dev, "DMA-RX buffer allocation failed");
		ret = -ENOMEM;
		goto error_ret;
	}

	/* initialize waitq for modem reset */
	init_waitqueue_head(&spt_dev->mdm_reset_wait);
	init_waitqueue_head(&spt_dev->mdm_event_wait);

	spt_spi_wake_lock_initial(spt_dev);
		
	spi_set_drvdata(spi, spt_dev);

	set_bit(SPT_SPI_STATE_PRESENT, &spt_dev->flags);

	/* create our tty port */
	ret = spt_spi_create_port(spt_dev);
	if (ret != 0) {
		dev_err(&spi->dev, "create default tty port failed");
		goto error_ret;
	}

	spt_dev->gpio.po = pl_data->pwr_on;
	spt_dev->gpio.mrdy = pl_data->mrdy;
	spt_dev->gpio.srdy = pl_data->srdy;
	spt_dev->gpio.mrts = pl_data->mrts;
	spt_dev->gpio.srts = pl_data->srts;
	spt_dev->gpio.mrsd = pl_data->mrsd;
	spt_dev->gpio.srsd = pl_data->srsd;	
	spt_dev->gpio.reset_out = pl_data->salive;

	dev_info(&spi->dev, "gpios %d, %d, %d, %d, %d, %d, %d, %d",
		 spt_dev->gpio.po,  spt_dev->gpio.mrts, spt_dev->gpio.mrdy, spt_dev->gpio.mrsd,
		 spt_dev->gpio.srts, spt_dev->gpio.srdy, spt_dev->gpio.srsd, spt_dev->gpio.reset_out);

	spt_dev->th = kthread_create(spt_spi_thread, (void *)spt_dev, "spt_spi_thread");
	if (IS_ERR(spt_dev->th)) {
		dev_err(&spi->dev, "kernel_thread() failed \n");

		goto error_ret;
	}
	wake_up_process(spt_dev->th);
#ifdef CONFIG_MODEM_SC8803G_CONTROL
	spt_dev->crash_notifier.notifier_call = spt_spi_notifier_event;
	register_spt_modem_crash_notifier(&spt_dev->crash_notifier);
#endif
	ret = request_irq(gpio_to_irq(spt_dev->gpio.srdy),
			  spt_spi_srdy_interrupt,
			  IRQF_TRIGGER_FALLING, DRVNAME,
			  (void *)spt_dev);
	if (ret) {
		dev_err(&spi->dev, "Unable to get irq %x",
			gpio_to_irq(spt_dev->gpio.srdy));
		goto error_ret;
	}

	ret = request_irq(gpio_to_irq(spt_dev->gpio.srts),
			  spt_spi_srdy_interrupt,
			  IRQF_TRIGGER_FALLING, DRVNAME,
			  (void *)spt_dev);
	if (ret) {
		dev_err(&spi->dev, "Unable to get irq %x",
			gpio_to_irq(spt_dev->gpio.srts));
		goto error_ret2;
	}
	enable_irq_wake(gpio_to_irq(spt_dev->gpio.srts));

	/* set pm runtime power state and register with power system */
	pm_runtime_set_active(&spi->dev);
	pm_runtime_enable(&spi->dev);

	/* handle case that modem is already signaling SRDY */
	/* no outgoing tty open at this point, this just satisfies the
	 * modem's read and should reset communication properly
	 */
	srdy = spt_spi_srts_get(spt_dev);

	if (!srdy) {
		spt_spi_handle_srdy(spt_dev);
	} else
		spt_spi_mrts_set(spt_dev, false);

	return 0;

error_ret2:
	free_irq(gpio_to_irq(spt_dev->gpio.srdy), (void *)spt_dev);
error_ret:
	spt_spi_free_device(spt_dev);
	saved_spt_dev = NULL;
	return ret;
}

/**
 *	spt_spi_remove	-	SPI device was removed
 *	@spi: SPI device
 *
 *	FIXME: We should be shutting the device down here not in
 *	the module unload path.
 */

static int spt_spi_remove(struct spi_device *spi)
{
	struct spt_spi_device *spt_dev = spi_get_drvdata(spi);
	/* stop activity */

	/* free irq */
	free_irq(gpio_to_irq(spt_dev->gpio.srts), (void *)spt_dev);
	free_irq(gpio_to_irq(spt_dev->gpio.srdy), (void *)spt_dev);

	/* free allocations */
	spt_spi_free_device(spt_dev);
	spt_spi_wake_lock_destroy(spt_dev);
	saved_spt_dev = NULL;
	return 0;
}

/**
 *	spt_spi_shutdown	-	called on SPI shutdown
 *	@spi: SPI device
 *
 *	No action needs to be taken here
 */

static void spt_spi_shutdown(struct spi_device *spi)
{
}

/*
 * various suspends and resumes have nothing to do
 * no hardware to save state for
 */

/**
 *	spt_spi_suspend	-	suspend SPI on system suspend
 *	@dev: device being suspended
 *
 *	Suspend the SPI side. No action needed on Intel MID platforms, may
 *	need extending for other systems.
 */
static int spt_spi_suspend(struct spi_device *spi, pm_message_t msg)
{
	struct spt_spi_device *spt_dev = spi_get_drvdata(spi);
	
	set_bit(SPT_SPI_STATE_SUSPEND, &spt_dev->flags); 
	return 0;
}

/**
 *	spt_spi_resume	-	resume SPI side on system resume
 *	@dev: device being suspended
 *
 *	Suspend the SPI side. No action needed on Intel MID platforms, may
 *	need extending for other systems.
 */
static int spt_spi_resume(struct spi_device *spi)
{
	struct spt_spi_device *spt_dev = spi_get_drvdata(spi);
	
	clear_bit(SPT_SPI_STATE_SUSPEND, &spt_dev->flags); 
	
	return 0;
}

static const struct spi_device_id spt_id_table[] = {
	{"sc880xg-spi", 0},
	{ }
};
MODULE_DEVICE_TABLE(spi, spt_id_table);

/* spi operations */
static const struct spi_driver spt_spi_driver = {
	.driver = {
		.name = DRVNAME,
		.bus = &spi_bus_type,
		.owner = THIS_MODULE},
	.probe = spt_spi_probe,
	.shutdown = spt_spi_shutdown,
	.remove = __devexit_p(spt_spi_remove),
	.suspend = spt_spi_suspend,
	.resume = spt_spi_resume,
	.id_table = spt_id_table
};

/**
 *	spt_spi_exit	-	module exit
 *
 *	Unload the module.
 */

static void __exit spt_spi_exit(void)
{
	/* unregister */
	tty_unregister_driver(spt_tty_drv);
	spi_unregister_driver((void *)&spt_spi_driver);
}

/**
 *	spt_spi_init		-	module entry point
 *
 *	Initialise the SPI and tty interfaces for the SPT SPI driver
 *	We need to initialize upper-edge spi driver after the tty
 *	driver because otherwise the spi probe will race
 */

static int __init spt_spi_init(void)
{
	int result = 0;

	if (machine_is_m65())
		return result;

	spt_tty_drv = alloc_tty_driver(1);
	if (!spt_tty_drv) {
		pr_err("%s: alloc_tty_driver failed", DRVNAME);
		return -ENOMEM;
	}

	spt_tty_drv->magic = TTY_DRIVER_MAGIC;
	spt_tty_drv->owner = THIS_MODULE;
	spt_tty_drv->driver_name = DRVNAME;
	spt_tty_drv->name = TTYNAME;
	spt_tty_drv->minor_start = SPT_SPI_TTY_ID;
	spt_tty_drv->num = 1;
	spt_tty_drv->type = TTY_DRIVER_TYPE_SERIAL;
	spt_tty_drv->subtype = SERIAL_TYPE_NORMAL;
	spt_tty_drv->flags = TTY_DRIVER_RESET_TERMIOS | TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	spt_tty_drv->init_termios = tty_std_termios;
	spt_tty_drv->init_termios.c_iflag = 0;
	spt_tty_drv->init_termios.c_oflag = 0;
	spt_tty_drv->init_termios.c_cflag = B4000000 | CS8 | CREAD;
	spt_tty_drv->init_termios.c_lflag = 0;

	tty_set_operations(spt_tty_drv, &spt_spi_serial_ops);

	result = tty_register_driver(spt_tty_drv);
	if (result) {
		pr_err("%s: tty_register_driver failed(%d)",
			DRVNAME, result);
		put_tty_driver(spt_tty_drv);
		return result;
	}

	result = spi_register_driver((void *)&spt_spi_driver);
	if (result) {
		pr_err("%s: spi_register_driver failed(%d)",
			DRVNAME, result);
		tty_unregister_driver(spt_tty_drv);
	}
	
	return result;
}

module_init(spt_spi_init);
module_exit(spt_spi_exit);

MODULE_AUTHOR("Wenbin Wu <wenbinwu@meizu.com>");
MODULE_DESCRIPTION("SC880xG spi driver");
MODULE_LICENSE("GPL");
MODULE_INFO(Version, "0.1-SC880xG");

