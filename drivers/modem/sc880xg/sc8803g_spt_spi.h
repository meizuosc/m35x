/****************************************************************************
 *
 * Driver for the SC880xG spi modem.
 *
 * Copyright (C) 2013, Meizu Corp
 *
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
 *
 *
 *****************************************************************************/
#ifndef _SPT_SPI_H
#define _SPT_SPI_H

#define DRVNAME				"sc880xg-spi"
#define TTYNAME				"ttyspi"

#define SPT_SPI_MAX_MINORS		1
#define SPT_SPI_TRANSFER_SIZE		(8192)
#define SPT_SPI_PAYLOAD_SIZE		(8192 - 128)
#define SPT_SPI_TX_PAYLOAD_SIZE	(2048)
#define SPT_SPI_TXFIFO_SIZE		(8192*32)
#define SPT_SPI_RXFIFO_SIZE		(8192*32)

#define SPT_SPI_HEADER_OVERHEAD		16
#define SPT_RESET_TIMEOUT		msecs_to_jiffies(1000)

/* device flags bitfield definitions */
#define SPT_SPI_STATE_PRESENT		0
#define SPT_SPI_STATE_IO_IN_PROGRESS	1
#define SPT_SPI_STATE_IO_READY		2
#define SPT_SPI_STATE_TIMER_PENDING	3
#define SPT_SPI_STATE_RESET			4
#define SPT_SPI_STATE_SUSPEND			5

/* flow control bitfields */
#define SPT_SPI_DCD			0
#define SPT_SPI_CTS			1
#define SPT_SPI_DSR			2
#define SPT_SPI_RI			3
#define SPT_SPI_DTR			4
#define SPT_SPI_RTS			5
#define SPT_SPI_TX_FC			6
#define SPT_SPI_RX_FC			7
#define SPT_SPI_UPDATE			8

#define SPT_SPI_STATUS_TIMEOUT		(2000*HZ)

/* values for bits in power status byte */
#define SPT_SPI_POWER_DATA_PENDING	1
#define SPT_SPI_POWER_SRDY		2

#define SPT_SPI_HEADER_TAG                   	0x7e7f
#define SPT_SPI_HEADER_TYPE	              0xaa55

enum{
	SPT_SPI_TYPE_NULL=0,
	SPT_SPI_TYPE_READ,
	SPT_SPI_TYPE_READ_RESEND,
	SPT_SPI_TYPE_WRITE,
	SPT_SPI_TYPE_WRITE_RESEND,
};

// AT packet header info structure
struct spt_spi_packet_header{
	u16 tag;
	u16 type;	//data type
	u32 length;
	u32 frame_num;	
	u32 reserved2;
}__packed;


struct spt_spi_device {
	/* Our SPI device */
	struct spi_device *spi_dev;

	struct task_struct *th;
	
	/* Port specific data */
	struct kfifo tx_fifo;
	spinlock_t fifo_lock;
	unsigned long signal_state;

	/* TTY Layer logic */
	struct tty_port tty_port;
	struct device *tty_dev;
	int minor;

	/* Low level I/O work */
	struct tasklet_struct io_work_tasklet;
	unsigned long flags;
	dma_addr_t rx_dma;
	dma_addr_t tx_dma;

	int modem;		/* Modem type */
	int use_dma;		/* provide dma-able addrs in SPI msg */
	long max_hz;		/* max SPI frequency */

	spinlock_t write_lock;
	int write_pending;
	spinlock_t power_lock;
	unsigned char power_status;

	unsigned char *rx_buffer;
	unsigned char *tx_buffer;
	dma_addr_t rx_bus;
	dma_addr_t tx_bus;
	unsigned char spi_more;
	unsigned char spi_slave_cts;

	struct timer_list spi_timer;

	struct spi_message spi_msg;
	struct spi_transfer spi_xfer;
	unsigned int 		spi_work_type;
	struct {
		/* gpio lines */
		unsigned short srts;		/* slave request to send gpio */
		unsigned short mrts;		/* master request to send gpio */
		
		unsigned short srdy;		/* slave reday to receive gpio */
		unsigned short mrdy;		/* master ready to receive gpio */

		unsigned short srsd;		/* slave resend request gpio */
		unsigned short mrsd;		/* master resend request gpio */
		
		unsigned short reset;		/* modem-reset gpio */
		unsigned short po;		/* modem-on gpio */
		unsigned short reset_out;	/* modem-in-reset gpio */
		/* state/stats */
		int unack_srdy_int_nb;
		int unack_mrts_int_nb;
	} gpio;

	/* modem reset */
	unsigned long mdm_reset_state;
#define MR_START	0
#define MR_INPROGRESS	1
#define MR_COMPLETE	2
	wait_queue_head_t mdm_reset_wait;
	wait_queue_head_t mdm_event_wait;
	struct notifier_block crash_notifier;
#ifdef CONFIG_HAS_WAKELOCK
	struct wake_lock   spt_spi_wakelock;
#endif
};

#endif /* _SPT_SPI_H */
