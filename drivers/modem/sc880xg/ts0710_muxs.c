/* File: mux_driver.c
 *
 * Portions derived from rfcomm.c, original header as follows:
 *
 * Copyright (C) 2000, 2001  Axis Communications AB
 *
 * Author: Mats Friden <mats.friden@axis.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * Exceptionally, Axis Communications AB grants discretionary and
 * conditional permissions for additional use of the text contained
 * in the company's release of the AXIS OpenBT Stack under the
 * provisions set forth hereunder.
 *
 * Provided that, if you use the AXIS OpenBT Stack with other files,
 * that do not implement functionality as specified in the Bluetooth
 * System specification, to produce an executable, this does not by
 * itself cause the resulting executable to be covered by the GNU
 * General Public License. Your use of that executable is in no way
 * restricted on account of using the AXIS OpenBT Stack code with it.
 *
 * This exception does not however invalidate any other reasons why
 * the executable file might be covered by the provisions of the GNU
 * General Public License.
 *
 */

/*
 * Copyright (C) 2002-2004  Motorola
 * Copyright (C) 2006 Harald Welte <laforge@openezx.org>
 *  07/28/2002  Initial version
 *  11/18/2002  Second version
 *  04/21/2004  Add GPRS PROC
 *  09/28/2008  Porting to kernel 2.6.21 by Spreadtrum 
*/
#include <linux/module.h>
#include <linux/types.h>

#include <linux/kernel.h>

#include <linux/errno.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/fcntl.h>
#include <linux/string.h>
#include <linux/major.h>
#include <linux/init.h>

#include <linux/proc_fs.h>

#ifndef USB_FOR_MUX
#include <linux/serial.h>

#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/slab.h>
//#include <linux/devfs_fs_kernel.h>
#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/bitops.h>
#include <linux/workqueue.h>
#endif
#include <linux/delay.h>
#include <linux/jiffies.h>
#ifdef USB_FOR_MUX
//#include <linux/usb.h>
//#include "ts0710_mux_usb.h"
#endif

#include <mach/ts0710.h>
#include <mach/ts0710_mux.h>
#include <mach/mux_buffer.h>
#include <linux/gpio.h>
//#define CONFIG_TS0710_MUX_UART

#define TS0710MUX_GPRS_SESSION_MAX 3
#define TS0710MUX_MAJOR 251
#define TS0710MUX_MINOR_START 0
#define NR_MUXS 16
#define SPRD_DRV_WAKEUP_BP 
												      /*#define TS0710MUX_TIME_OUT 30 *//* 300ms  */
#define TS0710MUX_TIME_OUT 500//250	/* 2500ms, for BP UART hardware flow control AP UART  */

#define TS0710MUX_IO_DLCI_FC_ON 0x54F2
#define TS0710MUX_IO_DLCI_FC_OFF 0x54F3
#define TS0710MUX_IO_FC_ON 0x54F4
#define TS0710MUX_IO_FC_OFF 0x54F5

#define TS0710MUX_MAX_BUF_SIZE 2048

#define TS0710MUX_SEND_BUF_OFFSET 10
#define TS0710MUX_SEND_BUF_SIZE (DEF_TS0710_MTU + TS0710MUX_SEND_BUF_OFFSET + 34)
#define TS0710MUX_RECV_BUF_SIZE TS0710MUX_SEND_BUF_SIZE

/*For BP UART problem Begin*/
#ifdef TS0710SEQ2
#define ACK_SPACE 0		/* 6 * 11(ACK frame size)  */
#else
#define ACK_SPACE 0		/* 6 * 7(ACK frame size)  */
#endif
/*For BP UART problem End*/

/*#define TS0710MUX_SERIAL_BUF_SIZE (DEF_TS0710_MTU + TS0710_MAX_HDR_SIZE) *//* For BP UART problem  */
#define TS0710MUX_SERIAL_BUF_SIZE (DEF_TS0710_MTU + TS0710_MAX_HDR_SIZE + ACK_SPACE)	/* For BP UART problem: ACK_SPACE  */

#define TS0710MUX_MAX_TOTAL_FRAME_SIZE (DEF_TS0710_MTU + TS0710_MAX_HDR_SIZE + FLAG_SIZE)
#define TS0710MUX_MAX_CHARS_IN_BUF 65535
#define TS0710MUX_THROTTLE_THRESHOLD DEF_TS0710_MTU

#define TEST_PATTERN_SIZE 250

#define CMDTAG 0x55
#define DATATAG 0xAA

#define ACK 0x4F		/*For BP UART problem */	//jim


/*For BP UART problem Begin*/
#ifdef TS0710SEQ2
#define FIRST_BP_SEQ_OFFSET 0	/*offset from start flag */
#define SECOND_BP_SEQ_OFFSET 0	/*offset from start flag */
#define FIRST_AP_SEQ_OFFSET 0	/*offset from start flag */
#define SECOND_AP_SEQ_OFFSET 0	/*offset from start flag */
#define SLIDE_BP_SEQ_OFFSET 0	/*offset from start flag */
#define SEQ_FIELD_SIZE 0
#else
#define SLIDE_BP_SEQ_OFFSET  1	/*offset from start flag */
#define SEQ_FIELD_SIZE    0	//1
#endif

#define ADDRESS_FIELD_OFFSET (1 + SEQ_FIELD_SIZE)	/*offset from start flag */
/*For BP UART problem End*/

#ifndef UNUSED_PARAM
#define UNUSED_PARAM(v) (void)(v)
#endif

#define TS0710MUX_GPRS1_DLCI  3
#define TS0710MUX_GPRS2_DLCI  4
#define TS0710MUX_VT_DLCI         2

#define TS0710MUX_GPRS1_RECV_COUNT_IDX 0
#define TS0710MUX_GPRS1_SEND_COUNT_IDX 1
#define TS0710MUX_GPRS2_RECV_COUNT_IDX 2
#define TS0710MUX_GPRS2_SEND_COUNT_IDX 3
#define TS0710MUX_VT_RECV_COUNT_IDX 4
#define TS0710MUX_VT_SEND_COUNT_IDX 5

#define TS0710MUX_COUNT_MAX_IDX        5
#define TS0710MUX_COUNT_IDX_NUM (TS0710MUX_COUNT_MAX_IDX + 1)

//#define UART_DUMP_DATA_TO_FILE
#define UART_SW_FLOW_CONTROL

#ifdef UART_SW_FLOW_CONTROL
typedef struct
{
 	int  		mux_loop_cnt;
    int         mux_rx_pin;
	int         mux_tx_pin; 
	int         mux_tx_timeout;
}mux_sw_flow_ctrl;


#define MUX_UART_FIFO_SIZE 128
#define HDLC_FLAG 0x7e
#define HDLC_ESCAPE 0x7d
#define HDLC_ESCAPE_MASK 0x20

static mux_sw_flow_ctrl  s_mux_flow_ctrl_info;
static int mux_tx_flow_ctrl_status = 0;
static char g_at[6]={0x7e,0x3,'a','t','\r',0x7e};
static char g_cmux[13]={0x7e,0xa,'a','t','+','c','m','u','x','=','0','\r',0x7e};
static DEFINE_MUTEX(mux_tx_lock);

extern int sprd_3rdparty_gpio_mux_rx_ctrl;
extern int sprd_3rdparty_gpio_mux_tx_ctrl;

#endif

static char mux_data[TS0710MUX_MAX_BUF_SIZE * 16];
struct mux_ringbuffer muxs_rbuf;


static __u8 tty2dlci[NR_MUXS] =
    { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13 ,14,15};
typedef struct {
	__u8 cmdtty;
	__u8 datatty;
} dlci_tty;

static dlci_tty dlci2tty[] = {
	{0, 0},			/* DLCI 0 */
	{0, 0},			/* DLCI 1 */
	{1, 1},			/* DLCI 2 *///vt
	{2, 2},			/* DLCI 3 *///ppp0
	{3, 3},			/* DLCI 4 *///ppp1
	{4, 4},			/* DLCI 5 */
	{5, 5},			/* DLCI 6 */
	{6, 6},			/* DLCI 7 */
	{7, 7},			/* DLCI 8 */
	{8, 8},			/* DLCI 9 */
	{9, 9},			/* DLCI 10 */
	{10, 10},		/* DLCI 11 */
	{11, 11},		/* DLCI 12 */
	{12, 12},		/* DLCI 13 */
	{13, 13},		/* DLCI 14 */
	{14, 14},		/* DLCI 15 */
};
typedef struct {
	volatile __u8 buf[TS0710MUX_SEND_BUF_SIZE];
	volatile __u8 *frame;
	unsigned long flags;
	volatile __u16 length;
	volatile __u8 filled;
	volatile __u8 dummy;	/* Allignment to 4*n bytes */
} mux_send_struct;

/* Bit number in flags of mux_send_struct */
#define BUF_BUSY 0

struct mux_recv_packet_tag {
	__u8 *data;
	__u32 length;
	struct mux_recv_packet_tag *next;
};
typedef struct mux_recv_packet_tag mux_recv_packet;

struct mux_recv_struct_tag {
	__u8 data[TS0710MUX_RECV_BUF_SIZE];
	__u32 length;
	__u32 total;
	mux_recv_packet *mux_packet;
	struct mux_recv_struct_tag *next;
	int no_tty;
	volatile __u8 post_unthrottle;
};
typedef struct mux_recv_struct_tag mux_recv_struct;

#define RECV_RUNNING 0
static unsigned long mux_recv_flags = 0;
int muxs_exiting =0;
static mux_send_struct *mux_send_info[NR_MUXS];
static volatile __u8 mux_send_info_flags[NR_MUXS];
static volatile __u8 mux_send_info_idx = NR_MUXS;

static mux_recv_struct *mux_recv_info[NR_MUXS];
static volatile __u8 mux_recv_info_flags[NR_MUXS];
static mux_recv_struct *mux_recv_queue = NULL;

static struct tty_driver mux_driver;

#define CONFIG_TS0710_MUX_SERIAL_UART
#ifdef USB_FOR_MUX
extern struct tty_driver *usb_for_mux_driver;
extern struct tty_struct *usb_for_mux_tty;
#define COMM_FOR_MUX_DRIVER usb_for_mux_driver
#define COMM_FOR_MUX_TTY usb_for_mux_tty
#define COMM_MUX_DISPATCHER usb_mux_dispatcher
#define COMM_MUX_SENDER usb_mux_sender
#else
#ifdef CONFIG_TS0710_MUX_SERIAL_UART
extern struct tty_driver *serial_for_muxs_driver;
extern struct tty_struct *serial_for_muxs_tty;
extern void (*serial_muxs_dispatcher) (struct tty_struct * tty);
extern void (*serial_muxs_sender) (void);
#define COMM_FOR_MUX_DRIVER serial_for_muxs_driver
#define COMM_FOR_MUX_TTY serial_for_muxs_tty
#define COMM_MUXS_DISPATCHER serial_muxs_dispatcher
#define COMM_MUXS_SENDER serial_muxs_sender
#ifdef CONFIG_NIC_ON_UART_MUX
extern void rmnet_rx_data(int tty_index, __u8 *uih_data_start, __u32 uih_len);
extern void rmnet_restart_queue(int chno);
#endif

#else
#ifdef CONFIG_TS0710_MUX_SPI
extern struct tty_driver *spi_for_mux_driver;
extern struct tty_struct *spi_for_mux_tty;
extern void (*spi_mux_dispatcher) (struct tty_struct * tty);
extern void (*spi_mux_sender) (void);
#define COMM_FOR_MUX_DRIVER spi_for_mux_driver
#define COMM_FOR_MUX_TTY spi_for_mux_tty
#define COMM_MUX_DISPATCHER spi_mux_dispatcher
#define COMM_MUX_SENDER spi_mux_sender
#endif
#endif
#endif

static struct workqueue_struct *muxsend_work_queue;
static struct workqueue_struct *muxpost_receive_work_queue;

static void receive_worker(struct work_struct *private_);
static void post_recv_worker(struct work_struct *private_);
static void send_worker(struct work_struct *private_);

static struct semaphore receive_sem;
static DECLARE_DELAYED_WORK(mux_send_work, send_worker);
static DECLARE_DELAYED_WORK(mux_post_receive_work, post_recv_worker);

static struct tty_struct **mux_table;
static volatile short int mux_tty[NR_MUXS];

static int mux_mode = 0;

#ifdef min
#undef min
#define min(a,b)    ( (a)<(b) ? (a):(b) )
#endif


static int send_ua(ts0710_con * ts0710, __u8 dlci);
static int send_dm(ts0710_con * ts0710, __u8 dlci);
static int send_sabm(ts0710_con * ts0710, __u8 dlci);
#if 0  /*Close here just for rmove building warnings*/
static int send_disc(ts0710_con * ts0710, __u8 dlci);
#endif
static void queue_uih(mux_send_struct * send_info, __u16 len,
		      ts0710_con * ts0710, __u8 dlci);
static int send_pn_msg(ts0710_con * ts0710, __u8 prior, __u32 frame_size,
		       __u8 credit_flow, __u8 credits, __u8 dlci, __u8 cr);
static int send_nsc_msg(ts0710_con * ts0710, mcc_type cmd, __u8 cr);
static void set_uih_hdr(short_frame * uih_pkt, __u8 dlci, __u32 len, __u8 cr);

static __u32 crc_check(__u8 * data, __u32 length, __u8 check_sum);
static __u8 crc_calc(__u8 * data, __u32 length);
static void create_crctable(__u8 table[]);

static void mux_sched_send(void);

static __u8 crctable[256];

static ts0710_con ts0710_connection;
/*
static rpn_values rpn_val;
*/

static int valid_dlci(__u8 dlci)
{
	if ((dlci < TS0710_MAX_CHN) && (dlci > 0))
		return 1;
	else
		return 0;
}


//#define PRINT_OUTPUT_PRINTK
//#define TS0710LOG
#ifdef TS0710DEBUG

#ifdef PRINT_OUTPUT_PRINTK
#define TS0710_DEBUG(fmt, arg...) printk(KERN_INFO "MUXS:%s "fmt, __FUNCTION__ , ## arg)
#else
#include "ezxlog.h"
static __u8 strDebug[256];
#define TS0710_DEBUG(fmt, arg...) ({ snprintf(strDebug, sizeof(strDebug), "MUXS " __FUNCTION__ ": " fmt "\n" , ## arg); \
                                     /*printk("%s", strDebug)*/ezxlogk("MX", strDebug, strlen(strDebug)); })
#endif /* End #ifdef PRINT_OUTPUT_PRINTK */

#else
#define TS0710_DEBUG(fmt...)
#endif /* End #ifdef TS0710DEBUG */

#ifdef TS0710LOG
static unsigned char g_tbuf[TS0710MUX_MAX_BUF_SIZE];
#ifdef PRINT_OUTPUT_PRINTK
#define TS0710_LOG(fmt, arg...) printk(fmt, ## arg)
#define TS0710_PRINTK(fmt, arg...) printk(fmt, ## arg)
#else
#include "ezxlog.h"
static __u8 strLog[256];
#define TS0710_LOG(fmt, arg...) ({ snprintf(strLog, sizeof(strLog), fmt, ## arg); \
                                     /*printk("%s", strLog)*/ezxlogk("MX", strLog, strlen(strLog)); })
#define TS0710_PRINTK(fmt, arg...) ({ printk(fmt, ## arg); \
                                      TS0710_LOG(fmt, ## arg); })
#endif /* End #ifdef PRINT_OUTPUT_PRINTK */

#else
#define TS0710_LOG(fmt...)
#define TS0710_PRINTK(fmt, arg...) printk(fmt, ## arg)
#endif /* End #ifdef TS0710LOG */

#ifdef TS0710DEBUG
static void TS0710_DEBUGHEX(__u8 * buf, int len)
{
	static unsigned char tbuf[TS0710MUX_MAX_BUF_SIZE];

	int i;
	int c;

	if (len <= 0) {
		return;
	}

	c = 0;
	for (i = 0; (i < len) && (c < (TS0710MUX_MAX_BUF_SIZE - 3)); i++) {
		sprintf(&tbuf[c], "%02x ", buf[i]);
		c += 3;
	}
	tbuf[c] = 0;

#ifdef PRINT_OUTPUT_PRINTK
	TS0710_DEBUG("%s", tbuf);
#else
	/*printk("%s\n", tbuf) */ ezxlogk("MX", tbuf, c);
#endif
}
static void TS0710_DEBUGSTR(__u8 * buf, int len)
{
	static unsigned char tbuf[TS0710MUX_MAX_BUF_SIZE];

	if (len <= 0) {
		return;
	}

	if (len > (TS0710MUX_MAX_BUF_SIZE - 1)) {
		len = (TS0710MUX_MAX_BUF_SIZE - 1);
	}

	memcpy(tbuf, buf, len);
	tbuf[len] = 0;

#ifdef PRINT_OUTPUT_PRINTK
	/* 0x00 byte in the string pointed by tbuf may truncate the print result */
	TS0710_DEBUG("%s", tbuf);
#else
	/*printk("%s\n", tbuf) */ ezxlogk("MX", tbuf, len);
#endif
}
#else
#define TS0710_DEBUGHEX(buf, len)
#define TS0710_DEBUGSTR(buf, len)
#endif /* End #ifdef TS0710DEBUG */

#ifdef TS0710LOG
static void TS0710_LOGSTR_FRAME(__u8 send, __u8 * data, int len)
{
	short_frame *short_pkt;
	long_frame *long_pkt;
	__u8 *uih_data_start;
	__u32 uih_len;
	__u8 dlci;
	int pos;

	if (len <= 0) {
		return;
	}

	pos = 0;
	if (send) {
		pos += sprintf(&g_tbuf[pos], "<");
		short_pkt = (short_frame *) (data + 1);	/*For BP UART problem */
	} else {
		/*For BP UART problem */
		/*pos += sprintf(&g_tbuf[pos], ">"); */
		pos += sprintf(&g_tbuf[pos], ">%d ", *(data + SLIDE_BP_SEQ_OFFSET));	/*For BP UART problem */

#ifdef TS0710SEQ2
		pos += sprintf(&g_tbuf[pos], "%02x %02x %02x %02x ", *(data + FIRST_BP_SEQ_OFFSET), *(data + SECOND_BP_SEQ_OFFSET), *(data + FIRST_AP_SEQ_OFFSET), *(data + SECOND_AP_SEQ_OFFSET));	/*For BP UART problem */
#endif

		short_pkt = (short_frame *) (data + ADDRESS_FIELD_OFFSET);	/*For BP UART problem */
	}

	/*For BP UART problem */
	/*short_pkt = (short_frame *)(data + 1); */

	dlci = short_pkt->h.addr.server_chn << 1 | short_pkt->h.addr.d;
	switch (CLR_PF(short_pkt->h.control)) {
	case SABM:
		pos += sprintf(&g_tbuf[pos], "C SABM %d ::", dlci);
		break;
	case UA:
		pos += sprintf(&g_tbuf[pos], "C UA %d ::", dlci);
		break;
	case DM:
		pos += sprintf(&g_tbuf[pos], "C DM %d ::", dlci);
		break;
	case DISC:
		pos += sprintf(&g_tbuf[pos], "C DISC %d ::", dlci);
		break;

		/*For BP UART problem Begin */
	case ACK:
		pos += sprintf(&g_tbuf[pos], "C ACK %d ", short_pkt->data[0]);

#ifdef TS0710SEQ2
		pos += sprintf(&g_tbuf[pos], "%02x %02x %02x %02x ", short_pkt->data[1], short_pkt->data[2], short_pkt->data[3], short_pkt->data[4]);	/*For BP UART problem */
#endif

		pos += sprintf(&g_tbuf[pos], "::");
		break;
		/*For BP UART problem End */

	case UIH:
		if (!dlci) {
			pos += sprintf(&g_tbuf[pos], "C MCC %d ::", dlci);
		} else {

			if ((short_pkt->h.length.ea) == 0) {
				long_pkt = (long_frame *) short_pkt;
				uih_len = GET_LONG_LENGTH(long_pkt->h.length);
				uih_data_start = long_pkt->h.data;
			} else {
				uih_len = short_pkt->h.length.len;
				uih_data_start = short_pkt->data;
			}
			//switch (*uih_data_start) {//jim:  not support
			switch ( /**uih_data_start*/ 0) {
			case CMDTAG:
				pos +=
				    sprintf(&g_tbuf[pos], "I %d A %d ::", dlci,
					    uih_len);
				break;
			case DATATAG:
			default:
				pos +=
				    sprintf(&g_tbuf[pos], "I %d D %d ::", dlci,
					    uih_len);
				break;
			}

		}
		break;
	default:
		pos += sprintf(&g_tbuf[pos], "N!!! %d ::", dlci);
		break;
	}

	if (len > (sizeof(g_tbuf) - pos - 1)) {
		len = (sizeof(g_tbuf) - pos - 1);
	}

	memcpy(&g_tbuf[pos], data, len);
	pos += len;
	g_tbuf[pos] = 0;

#ifdef PRINT_OUTPUT_PRINTK
	/* 0x00 byte in the string pointed by g_tbuf may truncate the print result */
	TS0710_LOG("%s\n", g_tbuf);
#else
	/*printk("%s\n", g_tbuf) */ ezxlogk("MX", g_tbuf, pos);
#endif
}
#else
#define TS0710_LOGSTR_FRAME(send, data, len)
#endif

#ifdef TS0710SIG
#define my_for_each_task(p) \
        for ((p) = current; ((p) = (p)->next_task) != current; )

static void TS0710_SIG2APLOGD(void)
{
	struct task_struct *p;
	static __u8 sig = 0;

	if (sig) {
		return;
	}

	read_lock(&tasklist_lock);
	my_for_each_task(p) {
		if (strncmp(p->comm, "aplogd", 6) == 0) {
			sig = 1;
			if (send_sig(SIGUSR2, p, 1) == 0) {
				TS0710_PRINTK
				    ("MUXS: success to send SIGUSR2 to aplogd!\n");
			} else {
				TS0710_PRINTK
				    ("MUXS: failure to send SIGUSR2 to aplogd!\n");
			}
			break;
		}
	}
	read_unlock(&tasklist_lock);

	if (!sig) {
		TS0710_PRINTK("MUXS: not found aplogd!\n");
	}
}
#else
#define TS0710_SIG2APLOGD()
#endif

#ifdef SPRD_DRV_WAKEUP_BP
#define GPIO_AP_W_BP	66
#define GPIO_BP_STATUS 70
#define  MAX_WAKE_UP_TIME 1000
static unsigned long wake_time_offset=MAX_WAKE_UP_TIME;
static unsigned long wake_time=0;
static firstime_enter=1;
static DEFINE_MUTEX(wake_bp_lock);


static void WakeupBp(void)
{
    int counter=0;
    int bp_state=0;
    mutex_lock(&wake_bp_lock);
    
    if(firstime_enter||(time_after(jiffies,wake_time + msecs_to_jiffies(MAX_WAKE_UP_TIME))))
    {
        firstime_enter=0;
        gpio_direction_output(GPIO_AP_W_BP,0);
        wake_time_offset=jiffies;
        while((bp_state==0)&&(counter<60)){
            bp_state=!!gpio_get_value(GPIO_BP_STATUS);
            mdelay(3);
            counter++;
        }
        wake_time=jiffies;
        gpio_direction_output(GPIO_AP_W_BP,1);
    }
    mutex_unlock(&wake_bp_lock);
//    printk(KERN_ERR "at %s line %d wakeup bp end,counter=%d !!!!\n",__func__,__LINE__,counter);	
}

static int IsBpaWake(void)
{
	gpio_direction_input(GPIO_BP_STATUS);
	/*Maybe some gpio high value is more than 1; so add !!*/
	return !!gpio_get_value(GPIO_BP_STATUS);
}

static void ApWakeBpGpioInit(void)
{
	int retval = 0;
	retval = gpio_request(GPIO_AP_W_BP, "host_wakeup_modem");
	if(retval){
		printk(KERN_ERR "at %s line %d request ap_wakeup_bp fail\n",__func__,__LINE__);
	}
	retval = gpio_request(GPIO_BP_STATUS, "modem_sleep_state");
	if(retval){
		printk(KERN_ERR "at %s line %d request bp_status fail\n",__func__,__LINE__);
	}
}
#endif


#ifdef UART_SW_FLOW_CONTROL

static int Mux_Tx_Data_Process (__u8 *src_ptr, __u8* des_ptr, int num)
{
    int i=0x0, send_len=0x0;
	int  curval;
	
	/*start flag*/
	*(des_ptr++) = HDLC_FLAG;
	 send_len++;
	 *(des_ptr++) = num;//
	 send_len++;
	 
	 /*content */
	 for (i = 0; i < num; i++)
	 {
			curval = *(src_ptr + i);
			if ((HDLC_FLAG == curval) || (HDLC_ESCAPE == curval)) {
				*(des_ptr++) = HDLC_ESCAPE;
				*(des_ptr++) = ~HDLC_ESCAPE_MASK & curval;
				send_len++;
			} else {
				*(des_ptr++) = curval;
			}
			send_len++;
	 }
	 
	/*end flag*/
	  *(des_ptr++) = HDLC_FLAG;
	  send_len++;
	  if (send_len > MUX_UART_FIFO_SIZE)
	  {
		 printk("send_len > MUX_UART_FIFO_SIZE\n");
	  }

	  return send_len;
}


static mux_sw_flow_ctrl  *GPIO_MuxFlowCtrlPinInfo(void)
{
	s_mux_flow_ctrl_info.mux_loop_cnt  = 0xffffffff;
	s_mux_flow_ctrl_info.mux_rx_pin    = sprd_3rdparty_gpio_mux_rx_ctrl;
	s_mux_flow_ctrl_info.mux_tx_pin    = sprd_3rdparty_gpio_mux_tx_ctrl;
	s_mux_flow_ctrl_info.mux_tx_timeout = 200;//1000ms
	
    return (mux_sw_flow_ctrl*)&s_mux_flow_ctrl_info;
}

static bool check_handshake(void)
{
	static bool mux_tx_start = false;
	bool  tx_low_or_high, rx_low_or_high;
	bool handshake= false;
	int loop_value = 0, tick_init = 0, tick_sleep = 0;
	mux_sw_flow_ctrl  *mux_pin_info= GPIO_MuxFlowCtrlPinInfo();

    loop_value = mux_pin_info->mux_loop_cnt;
	tick_init = jiffies;
	
	//printk("check_handshake \n");
	
	while(jiffies_to_msecs(jiffies-tick_init)<loop_value)
	{
		tx_low_or_high = gpio_get_value(mux_pin_info->mux_tx_pin);
		if(!mux_tx_start)
		{
			mux_tx_start = true;
			mux_tx_flow_ctrl_status = tx_low_or_high;
			printk("cj check_handshake : init_enter\n");
		}
	
		if(tx_low_or_high == mux_tx_flow_ctrl_status)
		{
			mux_tx_flow_ctrl_status = !mux_tx_flow_ctrl_status;
			handshake = true;
			//printk("cj check_handshake : handshake\n");
			break;
		}
#if 0 //removed by tao
		if(jiffies_to_msecs(jiffies-tick_sleep)> 10)
		{
			printk("check_handshake : sleep enter\n");
			tick_sleep = jiffies_to_msecs(jiffies);
			msleep(5);
		}
#endif 

		if(jiffies_to_msecs(jiffies-tick_init)> mux_pin_info->mux_tx_timeout)
		{
			mux_tx_start = false;
			printk("cj check_handshake : timeout\n");
		}
	}

	return handshake;

}

#endif

#ifdef UART_DUMP_DATA_TO_FILE

#include <linux/fs.h>
#include <linux/uaccess.h>

#define TEST_BUF_SIZE		(1024*128)

unsigned char trace_buffer2[TEST_BUF_SIZE] = {0};
unsigned long totalcount2 = 0;
/*
static void savelog()
{
    struct file     *filp = NULL;
    int i =0;
	mm_segment_t oldfs;
	printk(KERN_ALERT "========================================\r\n");
	printk(KERN_ALERT "tx total count is: %d  \r\n",totalcount);	
	oldfs = get_fs();
	set_fs(KERNEL_DS);		
    filp = filp_open("/data/local/logger/uart_tx.log", O_RDWR|O_TRUNC|O_CREAT, 0);
	if (IS_ERR(filp)) {
	    printk(KERN_ALERT "can not save as a file  \r\n");	
        return;
	}    
    //write file to fs
    for(i=0;i<128;i++)
    {
        msleep(1);
      	vfs_write(filp, &(trace_buffer2[1024*i]), 1024, &(filp->f_pos) );
      	msleep(1);
    }    
    printk(KERN_ALERT "1  \r\n");
	filp_close(filp, NULL);
	printk(KERN_ALERT "2  \r\n");
	set_fs(oldfs);
	printk(KERN_ALERT "Write log successfull  \r\n");
}*/
#endif


static int basic_write(ts0710_con * ts0710, __u8 * buf, int len)
{
	int res, send;
#ifdef UART_DUMP_DATA_TO_FILE
	int i=0;
#endif

	UNUSED_PARAM(ts0710);

	buf[0] = TS0710_BASIC_FLAG;
	buf[len + 1] = TS0710_BASIC_FLAG;

	if ((COMM_FOR_MUX_DRIVER == 0) || (COMM_FOR_MUX_TTY == 0)) {
		TS0710_PRINTK
			("MUX basic_write: (COMM_FOR_MUX_DRIVER == 0) || (COMM_FOR_MUX_TTY == 0)\n");

#ifndef USB_FOR_MUX
		TS0710_PRINTK
			("MUX basic_write: tapisrv might be down!!! (serial_for_mux_driver == 0) || (serial_for_muxs_tty == 0)\n");
		TS0710_SIG2APLOGD();
#endif

		return -1;
	}

	TS0710_LOGSTR_FRAME(1, buf, len + 2);
	TS0710_DEBUGHEX(buf, len + 2);
#ifdef UART_DUMP_DATA_TO_FILE
    //dump data to file
if(totalcount2+len+2 < TEST_BUF_SIZE-1)
{
    for(i=0;i<(len+2);i++)
    {
        //printk("buf%d, value=%x \n",totalcount2, trace_buffer2[totalcount2]);
        trace_buffer2[totalcount2++] = buf[i];
    }
}
#endif
	//	printk("\nMUX W\n");

#ifdef SPRD_DRV_WAKEUP_BP
	/*wake bp, set a signal: low(10ms)--high*/
	WakeupBp();//temp remove
#endif
	send =0;
#ifdef UART_SW_FLOW_CONTROL
{
	__u8  upacket[MUX_UART_FIFO_SIZE];
      int send_len, real_len;
      
    //printk("basic write len=%d\n",len);

    
    while(send < len+2)
    {
	    res = len+2-send;
		if(res > (MUX_UART_FIFO_SIZE/2-2))
		{
			res = MUX_UART_FIFO_SIZE/2-2;
		}
		send_len = Mux_Tx_Data_Process(buf+send, upacket, res);
		if(send_len > MUX_UART_FIFO_SIZE)
		{
			printk("cj basic_write : send_len=0x%x\n", send_len);
		}
		mutex_lock(&mux_tx_lock);
	    if(check_handshake())
		{
			   real_len = COMM_FOR_MUX_DRIVER->ops->write(COMM_FOR_MUX_TTY, upacket, send_len);
			   if (real_len != send_len)
			   {
			        mutex_unlock(&mux_tx_lock);
			        printk("cj basic_write : len not equal!!!real_len =0x%x, send_len=0x%x\n", real_len, send_len);
			   }
		}
		mutex_unlock(&mux_tx_lock);
		send = send +res;
	}
}
#else

	while (send < len+2) {
		res = COMM_FOR_MUX_DRIVER->ops->write(COMM_FOR_MUX_TTY, buf+send, len + 2-send);
		send=send+res;
	}
#endif	
	/*	if (res != len + 2) {

		TS0710_PRINTK("MUX basic_write: Write Error!\n");
		return -1;
		}
		*/
	return len + 2;
}

/* Functions for the crc-check and calculation */

#define CRC_VALID 0xcf

static __u32 crc_check(__u8 * data, __u32 length, __u8 check_sum)
{
	__u8 fcs = 0xff;
#ifdef TS0710DEBUG
	__u32 len=length;
#endif 
	length=length-1;
	while (length--) {
		fcs = crctable[fcs ^ *data++];
	}
	fcs = crctable[fcs ^ check_sum];
	TS0710_DEBUG("fcs : %d\n", fcs);
	TS0710_DEBUG("crc_check :len:%d check_sum:%d fcs : %d\n",len,check_sum, fcs);
	if (fcs == (uint) 0xcf) {	/*CRC_VALID) */
		TS0710_DEBUG("muxs crc_check: CRC check OK\n");
		return 0;
	} else {
		TS0710_PRINTK("MUXS crc_check: CRC check failed\n");
		return 1;
	}
}

/* Calculates the checksum according to the ts0710 specification */

static __u8 crc_calc(__u8 * data, __u32 length)
{
	__u8 fcs = 0xff;

	while (length--) {
		fcs = crctable[fcs ^ *data++];
	}

	return 0xff - fcs;
}

/* Calulates a reversed CRC table for the FCS check */

static void create_crctable(__u8 table[])
{
	int i, j;

	__u8 data;
	__u8 code_word = (__u8) 0xe0;
	__u8 sr = (__u8) 0;

	for (j = 0; j < 256; j++) {
		data = (__u8) j;

		for (i = 0; i < 8; i++) {
			if ((data & 0x1) ^ (sr & 0x1)) {
				sr >>= 1;
				sr ^= code_word;
			} else {
				sr >>= 1;
			}

			data >>= 1;
			sr &= 0xff;
		}

		table[j] = sr;
		sr = 0;
	}
}

static void ts0710_reset_dlci(__u8 j)
{
	if (j >= TS0710_MAX_CHN)
		return;

	ts0710_connection.dlci[j].state = DISCONNECTED;
	ts0710_connection.dlci[j].flow_control = 0;
	ts0710_connection.dlci[j].mtu = DEF_TS0710_MTU;
	ts0710_connection.dlci[j].initiated = 0;
	ts0710_connection.dlci[j].initiator = 0;
	init_waitqueue_head(&ts0710_connection.dlci[j].open_wait);
	init_waitqueue_head(&ts0710_connection.dlci[j].close_wait);
}

static void ts0710_reset_con(void)
{
	__u8 j;

	ts0710_connection.initiator = 0;
	ts0710_connection.mtu = DEF_TS0710_MTU + TS0710_MAX_HDR_SIZE;
	ts0710_connection.be_testing = 0;
	ts0710_connection.test_errs = 0;
	init_waitqueue_head(&ts0710_connection.test_wait);

	for (j = 0; j < TS0710_MAX_CHN; j++) {
		ts0710_reset_dlci(j);
	}
}

static void ts0710_init(void)
{
	create_crctable(crctable);

	ts0710_reset_con();
}

static void ts0710_upon_disconnect(void)
{
	ts0710_con *ts0710 = &ts0710_connection;
	__u8 j;

	for (j = 0; j < TS0710_MAX_CHN; j++) {
		ts0710->dlci[j].state = DISCONNECTED;
		wake_up_interruptible(&ts0710->dlci[j].open_wait);
		wake_up_interruptible(&ts0710->dlci[j].close_wait);
	}
	ts0710->be_testing = 0;
	wake_up_interruptible(&ts0710->test_wait);
	ts0710_reset_con();
}

/* Sending packet functions */

/* Creates a UA packet and puts it at the beginning of the pkt pointer */

static int send_ua(ts0710_con * ts0710, __u8 dlci)
{
	__u8 buf[sizeof(short_frame) + FCS_SIZE + FLAG_SIZE];
	short_frame *ua;

	TS0710_DEBUG("send_ua: Creating UA packet to DLCI %d\n", dlci);

	ua = (short_frame *) (buf + 1);
	ua->h.addr.ea = 1;
	ua->h.addr.cr = ((~(ts0710->initiator)) & 0x1);
	ua->h.addr.d = (dlci) & 0x1;
	ua->h.addr.server_chn = (dlci) >> 0x1;
	ua->h.control = SET_PF(UA);
	ua->h.length.ea = 1;
	ua->h.length.len = 0;
	ua->data[0] = crc_calc((__u8 *) ua, SHORT_CRC_CHECK);

	return basic_write(ts0710, buf, sizeof(short_frame) + FCS_SIZE);
}

/* Creates a DM packet and puts it at the beginning of the pkt pointer */

static int send_dm(ts0710_con * ts0710, __u8 dlci)
{
	__u8 buf[sizeof(short_frame) + FCS_SIZE + FLAG_SIZE];
	short_frame *dm;

	TS0710_DEBUG("send_dm: Creating DM packet to DLCI %d\n", dlci);

	dm = (short_frame *) (buf + 1);
	dm->h.addr.ea = 1;
	dm->h.addr.cr = ((~(ts0710->initiator)) & 0x1);
	dm->h.addr.d = dlci & 0x1;
	dm->h.addr.server_chn = dlci >> 0x1;
	dm->h.control = SET_PF(DM);
	dm->h.length.ea = 1;
	dm->h.length.len = 0;
	dm->data[0] = crc_calc((__u8 *) dm, SHORT_CRC_CHECK);

	return basic_write(ts0710, buf, sizeof(short_frame) + FCS_SIZE);
}

static int send_sabm(ts0710_con * ts0710, __u8 dlci)
{
	__u8 buf[sizeof(short_frame) + FCS_SIZE + FLAG_SIZE];
	short_frame *sabm;

	TS0710_DEBUG("send_sabm: Creating SABM packet to DLCI %d\n", dlci);

	sabm = (short_frame *) (buf + 1);
	sabm->h.addr.ea = 1;
	sabm->h.addr.cr = ((ts0710->initiator) & 0x1);
	sabm->h.addr.d = dlci & 0x1;
	sabm->h.addr.server_chn = dlci >> 0x1;
	sabm->h.control = SET_PF(SABM);
	sabm->h.length.ea = 1;
	sabm->h.length.len = 0;
	sabm->data[0] = crc_calc((__u8 *) sabm, SHORT_CRC_CHECK);

	return basic_write(ts0710, buf, sizeof(short_frame) + FCS_SIZE);
}
#if 0  /*close here just for remove building warnings*/
static int send_disc(ts0710_con * ts0710, __u8 dlci)
{
	__u8 buf[sizeof(short_frame) + FCS_SIZE + FLAG_SIZE];
	short_frame *disc;

	TS0710_DEBUG("send_disc: Creating DISC packet to DLCI %d\n", dlci);

	disc = (short_frame *) (buf + 1);
	disc->h.addr.ea = 1;
	disc->h.addr.cr = ((ts0710->initiator) & 0x1);
	disc->h.addr.d = dlci & 0x1;
	disc->h.addr.server_chn = dlci >> 0x1;
	disc->h.control = SET_PF(DISC);
	disc->h.length.ea = 1;
	disc->h.length.len = 0;
	disc->data[0] = crc_calc((__u8 *) disc, SHORT_CRC_CHECK);

	return basic_write(ts0710, buf, sizeof(short_frame) + FCS_SIZE);
}
#endif
static void queue_uih(mux_send_struct * send_info, __u16 len,
		      ts0710_con * ts0710, __u8 dlci)
{
	__u32 size;

	TS0710_DEBUG
	    ("MUXS queue_uih: Creating UIH packet with %d bytes data to DLCI %d\n",
	     len, dlci);

	if (len > SHORT_PAYLOAD_SIZE) {
		long_frame *l_pkt;

		size = sizeof(long_frame) + len + FCS_SIZE;
		l_pkt = (long_frame *) (send_info->frame - sizeof(long_frame));
		set_uih_hdr((void *)l_pkt, dlci, len, ts0710->initiator);
		l_pkt->data[len] = crc_calc((__u8 *) l_pkt, LONG_CRC_CHECK);
		send_info->frame = ((__u8 *) l_pkt) - 1;
	} else {
		short_frame *s_pkt;

		size = sizeof(short_frame) + len + FCS_SIZE;
		s_pkt =
		    (short_frame *) (send_info->frame - sizeof(short_frame));
		set_uih_hdr((void *)s_pkt, dlci, len, ts0710->initiator);
		s_pkt->data[len] = crc_calc((__u8 *) s_pkt, SHORT_CRC_CHECK);
		send_info->frame = ((__u8 *) s_pkt) - 1;
	}
	send_info->length = size;
}

/* Multiplexer command packets functions */

/* Turns on the ts0710 flow control */

static int ts0710_fcon_msg(ts0710_con * ts0710, __u8 cr)
{
	__u8 buf[30];
	mcc_short_frame *mcc_pkt;
	short_frame *uih_pkt;
	__u32 size;

	size = sizeof(short_frame) + sizeof(mcc_short_frame) + FCS_SIZE;
	uih_pkt = (short_frame *) (buf + 1);
	set_uih_hdr(uih_pkt, CTRL_CHAN, sizeof(mcc_short_frame),
		    ts0710->initiator);
	uih_pkt->data[sizeof(mcc_short_frame)] =
	    crc_calc((__u8 *) uih_pkt, SHORT_CRC_CHECK);
	mcc_pkt = (mcc_short_frame *) (uih_pkt->data);

	mcc_pkt->h.type.ea = EA;
	mcc_pkt->h.type.cr = cr;
	mcc_pkt->h.type.type = FCON;
	mcc_pkt->h.length.ea = EA;
	mcc_pkt->h.length.len = 0;

	return basic_write(ts0710, buf, size);
}

/* Turns off the ts0710 flow control */

static int ts0710_fcoff_msg(ts0710_con * ts0710, __u8 cr)
{
	__u8 buf[30];
	mcc_short_frame *mcc_pkt;
	short_frame *uih_pkt;
	__u32 size;

	size = (sizeof(short_frame) + sizeof(mcc_short_frame) + FCS_SIZE);
	uih_pkt = (short_frame *) (buf + 1);
	set_uih_hdr(uih_pkt, CTRL_CHAN, sizeof(mcc_short_frame),
		    ts0710->initiator);
	uih_pkt->data[sizeof(mcc_short_frame)] =
	    crc_calc((__u8 *) uih_pkt, SHORT_CRC_CHECK);
	mcc_pkt = (mcc_short_frame *) (uih_pkt->data);

	mcc_pkt->h.type.ea = 1;
	mcc_pkt->h.type.cr = cr;
	mcc_pkt->h.type.type = FCOFF;
	mcc_pkt->h.length.ea = 1;
	mcc_pkt->h.length.len = 0;

	return basic_write(ts0710, buf, size);
}


/* Sends an PN-messages and sets the not negotiable parameters to their
   default values in ts0710 */

static int send_pn_msg(ts0710_con * ts0710, __u8 prior, __u32 frame_size,
		       __u8 credit_flow, __u8 credits, __u8 dlci, __u8 cr)
{
	__u8 buf[30];
	pn_msg *pn_pkt;
	__u32 size;
	TS0710_DEBUG
	    ("send_pn_msg: DLCI 0x%02x, prior:0x%02x, frame_size:%d, credit_flow:%x, credits:%d, cr:%x\n",
	     dlci, prior, frame_size, credit_flow, credits, cr);

	size = sizeof(pn_msg);
	pn_pkt = (pn_msg *) (buf + 1);

	set_uih_hdr((void *)pn_pkt, CTRL_CHAN,
		    size - (sizeof(short_frame) + FCS_SIZE), ts0710->initiator);
	pn_pkt->fcs = crc_calc((__u8 *) pn_pkt, SHORT_CRC_CHECK);

	pn_pkt->mcc_s_head.type.ea = 1;
	pn_pkt->mcc_s_head.type.cr = cr;
	pn_pkt->mcc_s_head.type.type = PN;
	pn_pkt->mcc_s_head.length.ea = 1;
	pn_pkt->mcc_s_head.length.len = 8;

	pn_pkt->res1 = 0;
	pn_pkt->res2 = 0;
	pn_pkt->dlci = dlci;
	pn_pkt->frame_type = 0;
	pn_pkt->credit_flow = credit_flow;
	pn_pkt->prior = prior;
	pn_pkt->ack_timer = 0;
	SET_PN_MSG_FRAME_SIZE(pn_pkt, frame_size);
	pn_pkt->credits = credits;
	pn_pkt->max_nbrof_retrans = 0;

	return basic_write(ts0710, buf, size);
}

/* Send a Not supported command - command, which needs 3 bytes */

static int send_nsc_msg(ts0710_con * ts0710, mcc_type cmd, __u8 cr)
{
	__u8 buf[30];
	nsc_msg *nsc_pkt;
	__u32 size;

	size = sizeof(nsc_msg);
	nsc_pkt = (nsc_msg *) (buf + 1);

	set_uih_hdr((void *)nsc_pkt, CTRL_CHAN,
		    sizeof(nsc_msg) - sizeof(short_frame) - FCS_SIZE,
		    ts0710->initiator);

	nsc_pkt->fcs = crc_calc((__u8 *) nsc_pkt, SHORT_CRC_CHECK);

	nsc_pkt->mcc_s_head.type.ea = 1;
	nsc_pkt->mcc_s_head.type.cr = cr;
	nsc_pkt->mcc_s_head.type.type = NSC;
	nsc_pkt->mcc_s_head.length.ea = 1;
	nsc_pkt->mcc_s_head.length.len = 1;

	nsc_pkt->command_type.ea = 1;
	nsc_pkt->command_type.cr = cmd.cr;
	nsc_pkt->command_type.type = cmd.type;

	return basic_write(ts0710, buf, size);
}

static int ts0710_msc_msg(ts0710_con * ts0710, __u8 value, __u8 cr, __u8 dlci)
{
	__u8 buf[30];
	msc_msg *msc_pkt;
	__u32 size;

	size = sizeof(msc_msg);
	msc_pkt = (msc_msg *) (buf + 1);

	set_uih_hdr((void *)msc_pkt, CTRL_CHAN,
		    sizeof(msc_msg) - sizeof(short_frame) - FCS_SIZE,
		    ts0710->initiator);

	msc_pkt->fcs = crc_calc((__u8 *) msc_pkt, SHORT_CRC_CHECK);

	msc_pkt->mcc_s_head.type.ea = 1;
	msc_pkt->mcc_s_head.type.cr = cr;
	msc_pkt->mcc_s_head.type.type = MSC;
	msc_pkt->mcc_s_head.length.ea = 1;
	msc_pkt->mcc_s_head.length.len = 2;

	msc_pkt->dlci.ea = 1;
	msc_pkt->dlci.cr = 1;
	msc_pkt->dlci.d = dlci & 1;
	msc_pkt->dlci.server_chn = (dlci >> 1) & 0x1f;

	msc_pkt->v24_sigs = value;

	return basic_write(ts0710, buf, size);
}

static int ts0710_test_msg(ts0710_con * ts0710, __u8 * test_pattern, __u32 len,
			   __u8 cr, __u8 * f_buf /*Frame buf */ )
{
	__u32 size;

	if (len > SHORT_PAYLOAD_SIZE) {
		long_frame *uih_pkt;
		mcc_long_frame *mcc_pkt;

		size =
		    (sizeof(long_frame) + sizeof(mcc_long_frame) + len +
		     FCS_SIZE);
		uih_pkt = (long_frame *) (f_buf + 1);

		set_uih_hdr((short_frame *) uih_pkt, CTRL_CHAN, len +
			    sizeof(mcc_long_frame), ts0710->initiator);
		uih_pkt->data[GET_LONG_LENGTH(uih_pkt->h.length)] =
		    crc_calc((__u8 *) uih_pkt, LONG_CRC_CHECK);
		mcc_pkt = (mcc_long_frame *) uih_pkt->data;

		mcc_pkt->h.type.ea = EA;
		/* cr tells whether it is a commmand (1) or a response (0) */
		mcc_pkt->h.type.cr = cr;
		mcc_pkt->h.type.type = TEST;
		SET_LONG_LENGTH(mcc_pkt->h.length, len);
		memcpy(mcc_pkt->value, test_pattern, len);
	} else if (len > (SHORT_PAYLOAD_SIZE - sizeof(mcc_short_frame))) {
		long_frame *uih_pkt;
		mcc_short_frame *mcc_pkt;

		/* Create long uih packet and short mcc packet */
		size =
		    (sizeof(long_frame) + sizeof(mcc_short_frame) + len +
		     FCS_SIZE);
		uih_pkt = (long_frame *) (f_buf + 1);

		set_uih_hdr((short_frame *) uih_pkt, CTRL_CHAN,
			    len + sizeof(mcc_short_frame), ts0710->initiator);
		uih_pkt->data[GET_LONG_LENGTH(uih_pkt->h.length)] =
		    crc_calc((__u8 *) uih_pkt, LONG_CRC_CHECK);
		mcc_pkt = (mcc_short_frame *) uih_pkt->data;

		mcc_pkt->h.type.ea = EA;
		mcc_pkt->h.type.cr = cr;
		mcc_pkt->h.type.type = TEST;
		mcc_pkt->h.length.ea = EA;
		mcc_pkt->h.length.len = len;
		memcpy(mcc_pkt->value, test_pattern, len);
	} else {
		short_frame *uih_pkt;
		mcc_short_frame *mcc_pkt;

		size =
		    (sizeof(short_frame) + sizeof(mcc_short_frame) + len +
		     FCS_SIZE);
		uih_pkt = (short_frame *) (f_buf + 1);

		set_uih_hdr((void *)uih_pkt, CTRL_CHAN, len
			    + sizeof(mcc_short_frame), ts0710->initiator);
		uih_pkt->data[uih_pkt->h.length.len] =
		    crc_calc((__u8 *) uih_pkt, SHORT_CRC_CHECK);
		mcc_pkt = (mcc_short_frame *) uih_pkt->data;

		mcc_pkt->h.type.ea = EA;
		mcc_pkt->h.type.cr = cr;
		mcc_pkt->h.type.type = TEST;
		mcc_pkt->h.length.ea = EA;
		mcc_pkt->h.length.len = len;
		memcpy(mcc_pkt->value, test_pattern, len);

	}
	return basic_write(ts0710, f_buf, size);
}

static void set_uih_hdr(short_frame * uih_pkt, __u8 dlci, __u32 len, __u8 cr)
{
	uih_pkt->h.addr.ea = 1;
	uih_pkt->h.addr.cr = cr;
	uih_pkt->h.addr.d = dlci & 0x1;
	uih_pkt->h.addr.server_chn = dlci >> 1;
	uih_pkt->h.control = CLR_PF(UIH);

	if (len > SHORT_PAYLOAD_SIZE) {
		SET_LONG_LENGTH(((long_frame *) uih_pkt)->h.length, len);
	} else {
		uih_pkt->h.length.ea = 1;
		uih_pkt->h.length.len = len;
	}
}

/* Parses a multiplexer control channel packet */

static void process_mcc(__u8 * data, __u32 len, ts0710_con * ts0710, int longpkt)
{
	__u8 *tbuf = NULL;
	mcc_short_frame *mcc_short_pkt;
	int j;

	if (longpkt) {
		mcc_short_pkt =
		    (mcc_short_frame *) (((long_frame *) data)->data);
	} else {
		mcc_short_pkt =
		    (mcc_short_frame *) (((short_frame *) data)->data);
	}

	switch (mcc_short_pkt->h.type.type) {
	case TEST:
		if (mcc_short_pkt->h.type.cr == MCC_RSP) {
			TS0710_DEBUG("MUXS Received test command response\n");

			if (ts0710->be_testing) {
				if ((mcc_short_pkt->h.length.ea) == 0) {
					mcc_long_frame *mcc_long_pkt;
					mcc_long_pkt =
					    (mcc_long_frame *) mcc_short_pkt;
					if (GET_LONG_LENGTH
					    (mcc_long_pkt->h.length) !=
					    TEST_PATTERN_SIZE) {
						ts0710->test_errs =
						    TEST_PATTERN_SIZE;
						TS0710_DEBUG
						    ("Err: received test pattern is %d bytes long, not expected %d\n",
						     GET_LONG_LENGTH
						     (mcc_long_pkt->h.length),
						     TEST_PATTERN_SIZE);
					} else {
						ts0710->test_errs = 0;
						for (j = 0;
						     j < TEST_PATTERN_SIZE;
						     j++) {
							if (mcc_long_pkt->
							    value[j] !=
							    (j & 0xFF)) {
								(ts0710->
								 test_errs)++;
							}
						}
					}

				} else {

#if TEST_PATTERN_SIZE < 128
					if (mcc_short_pkt->h.length.len !=
					    TEST_PATTERN_SIZE) {
#endif

						ts0710->test_errs =
						    TEST_PATTERN_SIZE;
						TS0710_DEBUG
						    ("Err: received test pattern is %d bytes long, not expected %d\n",
						     mcc_short_pkt->h.length.
						     len, TEST_PATTERN_SIZE);

#if TEST_PATTERN_SIZE < 128
					} else {
						ts0710->test_errs = 0;
						for (j = 0;
						     j < TEST_PATTERN_SIZE;
						     j++) {
							if (mcc_short_pkt->
							    value[j] !=
							    (j & 0xFF)) {
								(ts0710->
								 test_errs)++;
							}
						}
					}
#endif

				}

				ts0710->be_testing = 0;	/* Clear the flag */
				wake_up_interruptible(&ts0710->test_wait);
			} else {
				TS0710_DEBUG
				    ("Err: shouldn't or late to get test cmd response\n");
			}
		} else {
			tbuf = (__u8 *) kmalloc(len + 32, GFP_ATOMIC);
			if (!tbuf) {
				break;
			}

			if ((mcc_short_pkt->h.length.ea) == 0) {
				mcc_long_frame *mcc_long_pkt;
				mcc_long_pkt = (mcc_long_frame *) mcc_short_pkt;
				ts0710_test_msg(ts0710, mcc_long_pkt->value,
						GET_LONG_LENGTH(mcc_long_pkt->h.
								length),
						MCC_RSP, tbuf);
			} else {
				ts0710_test_msg(ts0710, mcc_short_pkt->value,
						mcc_short_pkt->h.length.len,
						MCC_RSP, tbuf);
			}

			kfree(tbuf);
		}
		break;

	case FCON:		/*Flow control on command */
		TS0710_PRINTK
		    ("MUXS Received Flow control(all channels) on command\n");
		if (mcc_short_pkt->h.type.cr == MCC_CMD) {
			ts0710->dlci[0].state = CONNECTED;
			ts0710_fcon_msg(ts0710, MCC_RSP);
			mux_sched_send();
		}
		break;

	case FCOFF:		/*Flow control off command */
		TS0710_PRINTK
		    ("MUXS Received Flow control(all channels) off command\n");
		if (mcc_short_pkt->h.type.cr == MCC_CMD) {
			for (j = 0; j < TS0710_MAX_CHN; j++) {
				ts0710->dlci[j].state = FLOW_STOPPED;
			}
			ts0710_fcoff_msg(ts0710, MCC_RSP);
		}
		break;

	case MSC:		/*Modem status command */
		{
			__u8 dlci;
			__u8 v24_sigs;

			dlci = (mcc_short_pkt->value[0]) >> 2;
			v24_sigs = mcc_short_pkt->value[1];

			if ((ts0710->dlci[dlci].state != CONNECTED)
			    && (ts0710->dlci[dlci].state != FLOW_STOPPED)) {
			        TS0710_DEBUG("MUXS Received Modem status dlci = %d\n",dlci);
				send_dm(ts0710, dlci);
				break;
			}
			if (mcc_short_pkt->h.type.cr == MCC_CMD) {
				TS0710_DEBUG("MUXS Received Modem status command\n");
				if (v24_sigs & 2) {
					if (ts0710->dlci[dlci].state ==
					    CONNECTED) {
						TS0710_LOG
						    ("MUXS Received Flow off on dlci %d\n",
						     dlci);
						ts0710->dlci[dlci].state =
						    FLOW_STOPPED;
					}
				} else {
					if (ts0710->dlci[dlci].state ==
					    FLOW_STOPPED) {
						ts0710->dlci[dlci].state =
						    CONNECTED;
						TS0710_LOG
						    ("MUX Received Flow on on dlci %d\n",
						     dlci);
						mux_sched_send();
					}
				}

				ts0710_msc_msg(ts0710, v24_sigs, MCC_RSP, dlci);

			} else {
				TS0710_DEBUG
				    ("MUXS Received Modem status response\n");

				if (v24_sigs & 2) {
					TS0710_DEBUG("MUXS Flow stop accepted\n");
				}
			}
			break;
		}

		
	case PN:		/*DLC parameter negotiation */
		{
			__u8 dlci;
			__u16 frame_size;
			pn_msg *pn_pkt;

			pn_pkt = (pn_msg *) data;
			dlci = pn_pkt->dlci;
			frame_size = GET_PN_MSG_FRAME_SIZE(pn_pkt);
			TS0710_DEBUG
			    ("MUXS Received DLC parameter negotiation, PN\n");
			if (pn_pkt->mcc_s_head.type.cr == MCC_CMD) {
				TS0710_DEBUG("MUXS received PN command with:\n");
				TS0710_DEBUG("MUXS Frame size:%d\n", frame_size);

				frame_size =
				    min(frame_size, ts0710->dlci[dlci].mtu);
				send_pn_msg(ts0710, pn_pkt->prior, frame_size,
					    0, 0, dlci, MCC_RSP);
				ts0710->dlci[dlci].mtu = frame_size;
				TS0710_DEBUG("MUXS process_mcc : mtu set to %d\n",
					     ts0710->dlci[dlci].mtu);
			} else {
				TS0710_DEBUG("MUXS received PN response with:\n");
				TS0710_DEBUG("Frame size:%d\n", frame_size);

				frame_size =
				    min(frame_size, ts0710->dlci[dlci].mtu);
				ts0710->dlci[dlci].mtu = frame_size;

				TS0710_DEBUG
				    ("MUXS process_mcc : mtu set on dlci:%d to %d\n",
				     dlci, ts0710->dlci[dlci].mtu);

				if (ts0710->dlci[dlci].state == NEGOTIATING) {
					ts0710->dlci[dlci].state = CONNECTING;
					wake_up_interruptible(&ts0710->
							      dlci[dlci].
							      open_wait);
				}
			}
			break;
		}

	case NSC:		/*Non supported command resonse */
		TS0710_LOG("MUXS Received Non supported command response\n");
		break;

	default:		/*Non supported command received */
		TS0710_LOG("MUXS Received a non supported command\n");
		send_nsc_msg(ts0710, mcc_short_pkt->h.type, MCC_RSP);
		break;
	}
}

static mux_recv_packet *get_mux_recv_packet(__u32 size)
{
	mux_recv_packet *recv_packet;

	TS0710_DEBUG("MUXS Enter into get_mux_recv_packet");

	recv_packet =
	    (mux_recv_packet *) kmalloc(sizeof(mux_recv_packet), GFP_ATOMIC);
	if (!recv_packet) {
		return 0;
	}

	recv_packet->data = (__u8 *) kmalloc(size, GFP_ATOMIC);
	if (!(recv_packet->data)) {
		kfree(recv_packet);
		return 0;
	}
	recv_packet->length = 0;
	recv_packet->next = 0;
	return recv_packet;
}

static void free_mux_recv_packet(mux_recv_packet * recv_packet)
{
	TS0710_DEBUG("MUXS Enter into free_mux_recv_packet");

	if (!recv_packet) {
		return;
	}

	if (recv_packet->data) {
		kfree(recv_packet->data);
	}
	kfree(recv_packet);
}

static void free_mux_recv_struct(mux_recv_struct * recv_info)
{
	mux_recv_packet *recv_packet1, *recv_packet2;

	if (!recv_info) {
		return;
	}

	recv_packet1 = recv_info->mux_packet;
	while (recv_packet1) {
		recv_packet2 = recv_packet1->next;
		free_mux_recv_packet(recv_packet1);
		recv_packet1 = recv_packet2;
	}

	kfree(recv_info);
}

static inline void add_post_recv_queue(mux_recv_struct ** head,
				       mux_recv_struct * new_item)
{
	new_item->next = *head;
	*head = new_item;
}

static void ts0710_flow_on(__u8 dlci, ts0710_con * ts0710)
{
	int i;
	__u8 cmdtty;
	__u8 datatty;
	struct tty_struct *tty;
	mux_recv_struct *recv_info;

	if ((ts0710->dlci[0].state != CONNECTED)
	    && (ts0710->dlci[0].state != FLOW_STOPPED)) {
		return;
	} else if ((ts0710->dlci[dlci].state != CONNECTED)
		   && (ts0710->dlci[dlci].state != FLOW_STOPPED)) {
		return;
	}

	if (!(ts0710->dlci[dlci].flow_control)) {
		return;
	}

	cmdtty = dlci2tty[dlci].cmdtty;
	datatty = dlci2tty[dlci].datatty;

	if (cmdtty != datatty) {
		/* Check AT cmd tty */
		tty = mux_table[cmdtty];
		if (mux_tty[cmdtty] && tty) {
			if (test_bit(TTY_THROTTLED, &tty->flags)) {
				return;
			}
		}
		recv_info = mux_recv_info[cmdtty];
		if (mux_recv_info_flags[cmdtty] && recv_info) {
			if (recv_info->total) {
				return;
			}
		}

		/* Check data tty */
		tty = mux_table[datatty];
		if (mux_tty[datatty] && tty) {
			if (test_bit(TTY_THROTTLED, &tty->flags)) {
				return;
			}
		}
		recv_info = mux_recv_info[datatty];
		if (mux_recv_info_flags[datatty] && recv_info) {
			if (recv_info->total) {
				return;
			}
		}
	}

	for (i = 0; i < 3; i++) {
		if (ts0710_msc_msg(ts0710, EA | RTC | RTR | DV, MCC_CMD, dlci) <
		    0) {
			continue;
		} else {
			TS0710_LOG("MUX send Flow on on dlci %d\n", dlci);
			ts0710->dlci[dlci].flow_control = 0;
			break;
		}
	}
}

static void ts0710_flow_off(struct tty_struct *tty, __u8 dlci,
			    ts0710_con * ts0710)
{
	int i;

	if (test_and_set_bit(TTY_THROTTLED, &tty->flags)) {
		return;
	}

	if ((ts0710->dlci[0].state != CONNECTED)
	    && (ts0710->dlci[0].state != FLOW_STOPPED)) {
		return;
	} else if ((ts0710->dlci[dlci].state != CONNECTED)
		   && (ts0710->dlci[dlci].state != FLOW_STOPPED)) {
		return;
	}

	if (ts0710->dlci[dlci].flow_control) {
		return;
	}

	for (i = 0; i < 3; i++) {
		if (ts0710_msc_msg
		    (ts0710, EA | FC | RTC | RTR | DV, MCC_CMD, dlci) < 0) {
			continue;
		} else {
			TS0710_LOG("MUX send Flow off on dlci %d\n", dlci);
			ts0710->dlci[dlci].flow_control = 1;
			break;
		}
	}
}

static int ts0710_is_rmnet_ttyidx(int tty_idx)
{
  TS0710_DEBUG("ts0710_is_rmnet_ttyidx: tty_idx: %d\n", tty_idx);
  switch(tty_idx)
  {
    case 13:
    case 14:
      return 1;
    default :
      return 0;
  }
  return 0;
}
static int ts0710_recv_data(ts0710_con * ts0710, char *data, int len)
{
	short_frame *short_pkt;
	long_frame *long_pkt;
	__u8 *uih_data_start;
	__u32 uih_len;
	__u8 dlci;
	__u8 be_connecting;
#ifdef TS0710DEBUG
	unsigned long t;
#endif

	short_pkt = (short_frame *) data;

	dlci = short_pkt->h.addr.server_chn << 1 | short_pkt->h.addr.d;

	TS0710_DEBUG("MUXS ts0710_recv_data dlci = %d\n",dlci);
	switch (CLR_PF(short_pkt->h.control)) {
	case SABM:
		TS0710_DEBUG("MUXS SABM-packet received\n");


		if (!dlci) {
			TS0710_DEBUG("MUXS server channel == 0\n");
			ts0710->dlci[0].state = CONNECTED;

			TS0710_DEBUG("MUXS sending back UA - control channel\n");
			send_ua(ts0710, dlci);
			wake_up_interruptible(&ts0710->dlci[0].open_wait);

		} else if (valid_dlci(dlci)) {

			TS0710_DEBUG("MUXS Incomming connect on channel %d\n", dlci);

			TS0710_DEBUG("MUXS sending UA, dlci %d\n", dlci);
			send_ua(ts0710, dlci);

			ts0710->dlci[dlci].state = CONNECTED;
			wake_up_interruptible(&ts0710->dlci[dlci].open_wait);

		} else {
			TS0710_DEBUG("MUXS invalid dlci %d, sending DM\n", dlci);
			send_dm(ts0710, dlci);
		}

		break;

	case UA:
		TS0710_DEBUG("MUXS UA packet received\n");


		if (!dlci) {
			TS0710_DEBUG("MUXS server channel == 0|dlci[0].state=%d\n",
				     ts0710->dlci[0].state);

			if (ts0710->dlci[0].state == CONNECTING) {
				ts0710->dlci[0].state = CONNECTED;
				wake_up_interruptible(&ts0710->dlci[0].
						      open_wait);
			} else if (ts0710->dlci[0].state == DISCONNECTING) {
				ts0710_upon_disconnect();
			} else {
				TS0710_DEBUG
				    (" MUXS Something wrong receiving UA packet\n");
			}
		} else if (valid_dlci(dlci)) {
			TS0710_DEBUG
			    ("MUXS Incomming UA on channel %d|dlci[%d].state=%d\n",
			     dlci, dlci, ts0710->dlci[dlci].state);

			if (ts0710->dlci[dlci].state == CONNECTING) {
				ts0710->dlci[dlci].state = CONNECTED;
				wake_up_interruptible(&ts0710->dlci[dlci].
						      open_wait);
			} else if (ts0710->dlci[dlci].state == DISCONNECTING) {
				ts0710->dlci[dlci].state = DISCONNECTED;
				wake_up_interruptible(&ts0710->dlci[dlci].
						      open_wait);
				wake_up_interruptible(&ts0710->dlci[dlci].
						      close_wait);
				ts0710_reset_dlci(dlci);
			} else {
				TS0710_DEBUG
				    (" MUXS Something wrong receiving UA packet\n");
			}
		} else {
			TS0710_DEBUG(" MUXS invalid dlci %d\n", dlci);
		}

		break;

	case DM:
		TS0710_DEBUG("MUXS DM packet received\n");


		if (!dlci) {
			TS0710_DEBUG("MUXS server channel == 0\n");

			if (ts0710->dlci[0].state == CONNECTING) {
				be_connecting = 1;
			} else {
				be_connecting = 0;
			}
			ts0710_upon_disconnect();
			if (be_connecting) {
				ts0710->dlci[0].state = REJECTED;
			}
		} else if (valid_dlci(dlci)) {
			TS0710_DEBUG("MUXS Incomming DM on channel %d\n", dlci);

			if (ts0710->dlci[dlci].state == CONNECTING) {
				ts0710->dlci[dlci].state = REJECTED;
			} else {
				ts0710->dlci[dlci].state = DISCONNECTED;
			}
			wake_up_interruptible(&ts0710->dlci[dlci].open_wait);
			wake_up_interruptible(&ts0710->dlci[dlci].close_wait);
			ts0710_reset_dlci(dlci);
		} else {
			TS0710_DEBUG("MUXS invalid dlci %d\n", dlci);
		}

		break;

	case DISC:
		TS0710_DEBUG("MUXS DISC packet received\n");

		if (!dlci) {
			TS0710_DEBUG("MUXS server channel == 0\n");

			send_ua(ts0710, dlci);
			TS0710_DEBUG("MUXS DISC, sending back UA\n");

			ts0710_upon_disconnect();
		} else if (valid_dlci(dlci)) {
			TS0710_DEBUG("MUXS Incomming DISC on channel %d\n", dlci);

			send_ua(ts0710, dlci);
			TS0710_DEBUG("MUXS DISC, sending back UA\n");

			ts0710->dlci[dlci].state = DISCONNECTED;
			wake_up_interruptible(&ts0710->dlci[dlci].open_wait);
			wake_up_interruptible(&ts0710->dlci[dlci].close_wait);
			ts0710_reset_dlci(dlci);
		} else {
			TS0710_DEBUG("MUXS invalid dlci %d\n", dlci);
		}

		break;

	case UIH:
		TS0710_DEBUG("MUXS UIH packet received\n");

		if ((dlci >= TS0710_MAX_CHN)) {
			TS0710_DEBUG("MUXS invalid dlci %d\n", dlci);
			send_dm(ts0710, dlci);
			break;
		}

		if (GET_PF(short_pkt->h.control)) {
			TS0710_LOG
			    ("MUXS Error %s: UIH packet with P/F set, discard it!\n",
			     __FUNCTION__);
			break;
		}

		if ((ts0710->dlci[dlci].state != CONNECTED)
		    && (ts0710->dlci[dlci].state != FLOW_STOPPED)) {
			TS0710_LOG
			    ("MUXS Error %s: DLCI %d not connected, discard it!\n",
			     __FUNCTION__, dlci);
			send_dm(ts0710, dlci);
			break;
		}

		if ((short_pkt->h.length.ea) == 0) {
			TS0710_DEBUG("MUXS Long UIH packet received\n");
			long_pkt = (long_frame *) data;
			uih_len = GET_LONG_LENGTH(long_pkt->h.length);
			uih_data_start = long_pkt->h.data;
			TS0710_DEBUG("MUXS long packet length %d\n", uih_len);

		} else {
			TS0710_DEBUG("MUXS Short UIH pkt received\n");
			uih_len = short_pkt->h.length.len;
			uih_data_start = short_pkt->data;

		}

		if (dlci == 0) {
			TS0710_DEBUG("MUXS UIH on serv_channel 0\n");
			process_mcc(data, len, ts0710,
				    !(short_pkt->h.length.ea));
		} else if (valid_dlci(dlci)) {
			/* do tty dispatch */
			__u8 tty_idx;
			struct tty_struct *tty;
			__u8 queue_data;
			__u8 post_recv;
			__u8 flow_control;
			mux_recv_struct *recv_info;
			int recv_room;
			mux_recv_packet *recv_packet, *recv_packet2;
			TS0710_DEBUG("MUXS: UIH on channel %d\n", dlci);

			if (uih_len > ts0710->dlci[dlci].mtu) {
				TS0710_PRINTK
				    ("MUXS Error:  DLCI:%d, uih_len:%d is bigger than mtu:%d, discard data!\n",
				     dlci, uih_len, ts0710->dlci[dlci].mtu);
				break;
			}

			
			tty_idx = dlci2tty[dlci].datatty;	//jim : we see data and commmand as same

#ifdef CONFIG_NIC_ON_UART_MUX
      //cheney modify for 6610 
      if(ts0710_is_rmnet_ttyidx(tty_idx))
		  {
			  //send data to vnet
			  TS0710_DEBUG("MUXS: rmnet_rx_data %d\n", tty_idx);
        rmnet_rx_data(tty_idx, uih_data_start, uih_len);
			  break;
			}
#endif			
			tty = mux_table[tty_idx];
			if ((!mux_tty[tty_idx]) || (!tty)) {
				TS0710_PRINTK
				    ("MUX: No application waiting for, discard it! tty=%p,/dev/muxs%d\n",
				     tty, tty_idx);
			} else {	/* Begin processing received data */
				if ((!mux_recv_info_flags[tty_idx])
				    || (!mux_recv_info[tty_idx])) {
					TS0710_PRINTK
					    ("MUX Error: No mux_recv_info, discard it! /dev/muxs%d\n",
					     tty_idx);
					break;
				}

				recv_info = mux_recv_info[tty_idx];
				if (recv_info->total > 8192) {
					TS0710_PRINTK
					    ("MUX : discard data for tty_idx:%d, recv_info->total > 8192 \n",
					     tty_idx);
					break;
				}

				queue_data = 0;
				post_recv = 0;
				flow_control = 0;
				recv_room = 65535;
				if (tty->receive_room)
					recv_room = tty->receive_room;

				if (test_bit(TTY_THROTTLED, &tty->flags)) {
					queue_data = 1;
				} else {
						 if (recv_info->total) {
						queue_data = 1;
						post_recv = 1;
					} else if (recv_room < uih_len) {
						queue_data = 1;
						flow_control = 1;
					}

				}

				if (!queue_data) {
					/* Put received data into read buffer of tty */
					TS0710_DEBUG
					    ("Put received data into read buffer of /dev/muxs%d",
					     tty_idx);

#ifdef TS0710DEBUG
					t = jiffies;
#endif

					(tty->ldisc->ops->receive_buf) (tty,
								  uih_data_start,
								  NULL,
								  uih_len);

#ifdef TS0710DEBUG
					TS0710_DEBUG
					    ("tty->ldisc->ops->receive_buf:%p,  take ticks: %lu\n",
					     tty->ldisc->ops->receive_buf,
					     (jiffies - t));
#endif

				} else {	/* Queue data */

					TS0710_DEBUG
					    ("Put received data into recv queue of /dev/muxs%d",
					     tty_idx);
					if (recv_info->total) {
						/* recv_info is already linked into mux_recv_queue */

						recv_packet =
						    get_mux_recv_packet
						    (uih_len);
						if (!recv_packet) {
							TS0710_PRINTK
							    ("MUX %s: no memory\n",
							     __FUNCTION__);
							break;
						}

						memcpy(recv_packet->data,
						       uih_data_start, uih_len);
						recv_packet->length = uih_len;
						recv_info->total += uih_len;
						recv_packet->next = NULL;

						if (!(recv_info->mux_packet)) {
							recv_info->mux_packet =
							    recv_packet;
						} else {
							recv_packet2 =
							    recv_info->
							    mux_packet;
							while (recv_packet2->
							       next) {
								recv_packet2 =
								    recv_packet2->
								    next;
							}
							recv_packet2->next =
							    recv_packet;
						}	/* End if( !(recv_info->mux_packet) ) */
					} else {	/* recv_info->total == 0 */
						if (uih_len >
						    TS0710MUX_RECV_BUF_SIZE) {
							TS0710_PRINTK
							    ("MUX Error:  tty_idx:%d, uih_len == %d is too big\n",
							     tty_idx, uih_len);
							uih_len =
							    TS0710MUX_RECV_BUF_SIZE;
						}
						memcpy(recv_info->data,
						       uih_data_start, uih_len);
						recv_info->length = uih_len;
						recv_info->total = uih_len;

						add_post_recv_queue
						    (&mux_recv_queue,
						     recv_info);
					}	/* End recv_info->total == 0 */
				}	/* End Queue data */

				if (flow_control) {
					/* Do something for flow control */
					ts0710_flow_off(tty, dlci, ts0710);
				}


			}	/* End processing received data */
		} else {
			TS0710_DEBUG("invalid dlci %d\n", dlci);
		}

		break;

	default:
		TS0710_DEBUG("illegal packet\n");
		break;
	}
	return 0;
}



/* Close ts0710 channel */
static void ts0710_close_channel(__u8 dlci)
{
	ts0710_con *ts0710 = &ts0710_connection;
#if 0
	int try;
	unsigned long t;
#endif
	TS0710_DEBUG("ts0710_disc_command on channel %d\n", dlci);

	if ((ts0710->dlci[dlci].state == DISCONNECTED)
	    || (ts0710->dlci[dlci].state == REJECTED)) {
		return;
	} else {
		if (ts0710->dlci[dlci].state != DISCONNECTED) {
			if (dlci == 0) {	/* Control Channel */
				__u8 j;

				for (j = 0; j < TS0710_MAX_CHN; j++) 
					ts0710->dlci[j].state = DISCONNECTED;
				ts0710->be_testing = 0;
				ts0710_reset_con();

			} else {	/* Other Channel */
				ts0710->dlci[dlci].state = DISCONNECTED;
				ts0710_reset_dlci(dlci);
			}
		}
	}
#if 0   /* If Open here, Plsease open send_disc() function first!*/
		
	if ((ts0710->dlci[dlci].state == DISCONNECTED)
	    || (ts0710->dlci[dlci].state == REJECTED)) {
		return;
	} else if (ts0710->dlci[dlci].state == DISCONNECTING) {
		/* Reentry */
		return;
	} else {
		ts0710->dlci[dlci].state = DISCONNECTING;
		try = 3;
		while (try--) {
			t = jiffies;
			send_disc(ts0710, dlci);
			interruptible_sleep_on_timeout(&ts0710->dlci[dlci].
						       close_wait,
						       TS0710MUX_TIME_OUT);
			if (ts0710->dlci[dlci].state == DISCONNECTED) {
				break;
			} else if (signal_pending(current)) {
				TS0710_PRINTK
				    ("MUXS DLCI %d Send DISC got signal!\n",
				     dlci);
				break;
			} else if ((jiffies - t) >= TS0710MUX_TIME_OUT) {
				TS0710_PRINTK
				    ("MUXS DLCI %d Send DISC timeout!\n", dlci);
				continue;
			}
		}
		if (ts0710->dlci[dlci].state != DISCONNECTED) {
			if (dlci == 0) {	/* Control Channel */
				ts0710_upon_disconnect();
			} else {	/* Other Channel */
				ts0710->dlci[dlci].state = DISCONNECTED;
				wake_up_interruptible(&ts0710->dlci[dlci].
						      close_wait);
				ts0710_reset_dlci(dlci);
			}
		}
	}
#endif 
}

static int ts0710_open_channel(__u8 dlci)
{
	ts0710_con *ts0710 = &ts0710_connection;
	int try;
	int retval;
	unsigned long t;
	
	retval = -ENODEV;
	if (dlci == 0) {	// control channel
		if ((ts0710->dlci[0].state == CONNECTED)
		    || (ts0710->dlci[0].state == FLOW_STOPPED)) {
			return 0;
		} else if (ts0710->dlci[0].state == CONNECTING) {
			/* Reentry */
			TS0710_PRINTK
			    ("MUXS DLCI: 0, reentry to open DLCI 0, pid: %d, %s !\n",
			     current->pid, current->comm);
			try = 11;
			while (try--) {
				t = jiffies;
				interruptible_sleep_on_timeout(&ts0710->dlci[0].
							       open_wait,
							       TS0710MUX_TIME_OUT);
				if ((ts0710->dlci[0].state == CONNECTED)
				    || (ts0710->dlci[0].state == FLOW_STOPPED)) {
					retval = 0;
					break;
				} else if (ts0710->dlci[0].state == REJECTED) {
					retval = -EREJECTED;
					break;
				} else if (ts0710->dlci[0].state ==
					   DISCONNECTED) {
					break;
				} else if (signal_pending(current)) {
					TS0710_PRINTK
					    ("MUXS DLCI:%d Wait for connecting got signal!\n",
					     dlci);
					retval = -EAGAIN;
					break;
				} else if ((jiffies - t) >= TS0710MUX_TIME_OUT) {
					TS0710_PRINTK
					    ("MUXS DLCI:%d Wait for connecting timeout!\n",
					     dlci);
					continue;
				} else if (ts0710->dlci[0].state == CONNECTING) {
					continue;
				}
			}

			if (ts0710->dlci[0].state == CONNECTING) {
				ts0710->dlci[0].state = DISCONNECTED;
			}
		} else if ((ts0710->dlci[0].state != DISCONNECTED)
			   && (ts0710->dlci[0].state != REJECTED)) {
			TS0710_PRINTK("MUXS DLCI:%d state is invalid!\n", dlci);
			return retval;
		} else {
			ts0710->initiator = 1;
			ts0710->dlci[0].state = CONNECTING;
			ts0710->dlci[0].initiator = 1;
			try = 10;
			while (try--) {
				t = jiffies;
				send_sabm(ts0710, 0);
				interruptible_sleep_on_timeout(&ts0710->dlci[0].
							       open_wait,
							       TS0710MUX_TIME_OUT);
				if ((ts0710->dlci[0].state == CONNECTED)
				    || (ts0710->dlci[0].state == FLOW_STOPPED)) {
					retval = 0;
					break;
				} else if (ts0710->dlci[0].state == REJECTED) {
					TS0710_PRINTK
					    ("MUXS DLCI:%d Send SABM got rejected!\n",
					     dlci);
					retval = -EREJECTED;
					break;
				} else if (signal_pending(current)) {
					TS0710_PRINTK
					    ("MUXS DLCI:%d Send SABM got signal!\n",
					     dlci);
					retval = -EAGAIN;
					break;
				} else if ((jiffies - t) >= TS0710MUX_TIME_OUT) {
					TS0710_PRINTK
					    ("MUXS DLCI:%d Send SABM timeout!\n",
					     dlci);
					continue;
				}
			}

			if (ts0710->dlci[0].state == CONNECTING) {
				ts0710->dlci[0].state = DISCONNECTED;
			}
			wake_up_interruptible(&ts0710->dlci[0].open_wait);
		}
	} else {		// other channel
		if ((ts0710->dlci[0].state != CONNECTED)
		    && (ts0710->dlci[0].state != FLOW_STOPPED)) {
			return retval;
		} else if ((ts0710->dlci[dlci].state == CONNECTED)
			   || (ts0710->dlci[dlci].state == FLOW_STOPPED)) {
			return 0;
		} else if ((ts0710->dlci[dlci].state == NEGOTIATING)
			   || (ts0710->dlci[dlci].state == CONNECTING)) {
			/* Reentry */
			try = 8;
			while (try--) {
				t = jiffies;
				interruptible_sleep_on_timeout(&ts0710->
							       dlci[dlci].
							       open_wait,
							       TS0710MUX_TIME_OUT);
				if ((ts0710->dlci[dlci].state == CONNECTED)
				    || (ts0710->dlci[dlci].state ==
					FLOW_STOPPED)) {
					retval = 0;
					break;
				} else if (ts0710->dlci[dlci].state == REJECTED) {
					retval = -EREJECTED;
					break;
				} else if (ts0710->dlci[dlci].state ==
					   DISCONNECTED) {
					break;
				} else if (signal_pending(current)) {
					TS0710_PRINTK
					    ("MUXS DLCI:%d Wait for connecting got signal!\n",
					     dlci);
					retval = -EAGAIN;
					break;
				} else if ((jiffies - t) >= TS0710MUX_TIME_OUT) {
					TS0710_PRINTK
					    ("MUXS DLCI:%d Wait for connecting timeout!\n",
					     dlci);
					continue;
				} else
				    if ((ts0710->dlci[dlci].state ==
					 NEGOTIATING)
					|| (ts0710->dlci[dlci].state ==
					    CONNECTING)) {
					continue;
				}
			}

			if ((ts0710->dlci[dlci].state == NEGOTIATING)
			    || (ts0710->dlci[dlci].state == CONNECTING)) {
				ts0710->dlci[dlci].state = DISCONNECTED;
			}
		} else if ((ts0710->dlci[dlci].state != DISCONNECTED)
			   && (ts0710->dlci[dlci].state != REJECTED)) {
			TS0710_PRINTK("MUXS DLCI:%d state is invalid!\n", dlci);
			return retval;
		} else {
			ts0710->dlci[dlci].state = CONNECTING;
			ts0710->dlci[dlci].initiator = 1;
			if (ts0710->dlci[dlci].state == CONNECTING) {
				try = 3;
				while (try--) {
					t = jiffies;
					send_sabm(ts0710, dlci);
					interruptible_sleep_on_timeout(&ts0710->
								       dlci
								       [dlci].
								       open_wait,
								       TS0710MUX_TIME_OUT);
					if ((ts0710->dlci[dlci].state ==
					     CONNECTED)
					    || (ts0710->dlci[dlci].state ==
						FLOW_STOPPED)) {
						retval = 0;
						break;
					} else if (ts0710->dlci[dlci].state ==
						   REJECTED) {
						TS0710_PRINTK
						    ("MUXS DLCI:%d Send SABM got rejected!\n",
						     dlci);
						retval = -EREJECTED;
						break;
					} else if (signal_pending(current)) {
						TS0710_PRINTK
						    ("MUXS DLCI:%d Send SABM got signal!\n",
						     dlci);
						retval = -EAGAIN;
						break;
					} else if ((jiffies - t) >=
						   TS0710MUX_TIME_OUT) {
						TS0710_PRINTK
						    ("MUXS DLCI:%d Send SABM timeout!\n",
						     dlci);
						continue;
					}
				}
			}

			if ((ts0710->dlci[dlci].state == NEGOTIATING)
			    || (ts0710->dlci[dlci].state == CONNECTING)) {
				ts0710->dlci[dlci].state = DISCONNECTED;
			}
			wake_up_interruptible(&ts0710->dlci[dlci].open_wait);
		}
	}
	return retval;
}

static int ts0710_exec_test_cmd(void)
{
	ts0710_con *ts0710 = &ts0710_connection;
	__u8 *f_buf;		/* Frame buffer */
	__u8 *d_buf;		/* Data buffer */
	int retval = -EFAULT;
	int j;
	unsigned long t;

	if (ts0710->be_testing) {
		/* Reentry */
		t = jiffies;
		interruptible_sleep_on_timeout(&ts0710->test_wait,
					       3 * TS0710MUX_TIME_OUT);
		if (ts0710->be_testing == 0) {
			if (ts0710->test_errs == 0) {
				retval = 0;
			} else {
				retval = -EFAULT;
			}
		} else if (signal_pending(current)) {
			TS0710_DEBUG
			    ("Wait for Test_cmd response got signal!\n");
			retval = -EAGAIN;
		} else if ((jiffies - t) >= 3 * TS0710MUX_TIME_OUT) {
			TS0710_DEBUG("Wait for Test_cmd response timeout!\n");
			retval = -EFAULT;
		}
	} else {
		ts0710->be_testing = 1;	/* Set the flag */

		f_buf = (__u8 *) kmalloc(TEST_PATTERN_SIZE + 32, GFP_KERNEL);
		d_buf = (__u8 *) kmalloc(TEST_PATTERN_SIZE + 32, GFP_KERNEL);
		if ((!f_buf) || (!d_buf)) {
			if (f_buf) {
				kfree(f_buf);
			}
			if (d_buf) {
				kfree(d_buf);
			}

			ts0710->be_testing = 0;	/* Clear the flag */
			ts0710->test_errs = TEST_PATTERN_SIZE;
			wake_up_interruptible(&ts0710->test_wait);
			return -ENOMEM;
		}

		for (j = 0; j < TEST_PATTERN_SIZE; j++) {
			d_buf[j] = j & 0xFF;
		}

		t = jiffies;
		ts0710_test_msg(ts0710, d_buf, TEST_PATTERN_SIZE, MCC_CMD,
				f_buf);
		interruptible_sleep_on_timeout(&ts0710->test_wait,
					       2 * TS0710MUX_TIME_OUT);
		if (ts0710->be_testing == 0) {
			if (ts0710->test_errs == 0) {
				retval = 0;
			} else {
				retval = -EFAULT;
			}
		} else if (signal_pending(current)) {
			TS0710_DEBUG("Send Test_cmd got signal!\n");
			retval = -EAGAIN;
		} else if ((jiffies - t) >= 2 * TS0710MUX_TIME_OUT) {
			TS0710_DEBUG("Send Test_cmd timeout!\n");
			ts0710->test_errs = TEST_PATTERN_SIZE;
			retval = -EFAULT;
		}

		ts0710->be_testing = 0;	/* Clear the flag */
		wake_up_interruptible(&ts0710->test_wait);

		/* Release buffer */
		if (f_buf) {
			kfree(f_buf);
		}
		if (d_buf) {
			kfree(d_buf);
		}
	}

	return retval;
}

static void mux_sched_send(void)
{

	if (queue_delayed_work(muxsend_work_queue, &mux_send_work,0)==0){
		queue_delayed_work(muxsend_work_queue, &mux_send_work,10);	
	}

}

#if 1 //#ifdef CONFIG_TS0710_MUX_SPI

// Returns 1 if found, 0 otherwise. needle must be null-terminated.
// strstr might not work because WebBox sends garbage before the first OKread
static int findInBuf(unsigned char *buf, int len, char *needle)
{
	int i;
	int needleMatchedPos = 0;

	if (needle[0] == '\0') {
		return 1;
	}

	for (i = 0; i < len; i++) {
		if (needle[needleMatchedPos] == buf[i]) {
			needleMatchedPos++;
			if (needle[needleMatchedPos] == '\0') {
				// Entire needle was found
				return 1;
			}
		} else {
			needleMatchedPos = 0;
		}
	}
	return 0;
}

static int cmux_mode = 0;
int is_cmuxs_mode(void)
{
	return cmux_mode;
}

#endif

/****************************
 * TTY driver routines
*****************************/
static int mux_opened = 0;
int cmuxs_opened(void)
{
	int opened = 0;
	if (mux_opened) {
		opened = 1;
	}
	return opened;
}
static void mux_close(struct tty_struct *tty, struct file *filp)
{
	ts0710_con *ts0710 = &ts0710_connection;
	int line;
	__u8 dlci;
	__u8 cmdtty;
	__u8 datatty;

	UNUSED_PARAM(filp);

	if (!tty) {
		return;
	}
	line = tty->index;
	if ((line < 0) || (line >= NR_MUXS)) {
		return;
	}
	if (mux_tty[line] > 0)
		mux_tty[line]--;

	if(mux_opened > 0)
    {
      mux_opened--;
    }

	dlci = tty2dlci[line];
	cmdtty = dlci2tty[dlci].cmdtty;
	datatty = dlci2tty[dlci].datatty;
	if ((mux_tty[cmdtty] == 0) && (mux_tty[datatty] == 0)) {
		if (dlci == 1) {
			ts0710_close_channel(0);
			TS0710_PRINTK("MUXS mux_close: tapisrv might be down!!! Close DLCI 1\n");
			TS0710_SIG2APLOGD();
			cmux_mode = 0;
			printk("cmuxs mode is 0 !!\n");
		}
		ts0710_close_channel(dlci);
	}

	if (mux_tty[line] == 0) {
		if ((mux_send_info_flags[line])
		    && (mux_send_info[line])
		    /*&& (mux_send_info[line]->filled == 0) */
		    ) {
			mux_send_info_flags[line] = 0;
			kfree(mux_send_info[line]);
			mux_send_info[line] = 0;
			TS0710_DEBUG("Free mux_send_info for /dev/muxs%d\n", line);
		}

		if ((mux_recv_info_flags[line])
		    && (mux_recv_info[line])
		    && (mux_recv_info[line]->total == 0)) {
			mux_recv_info_flags[line] = 0;
			free_mux_recv_struct(mux_recv_info[line]);
			mux_recv_info[line] = 0;
			TS0710_DEBUG("Free mux_recv_info for /dev/muxs%d\n", line);
		}

		ts0710_flow_on(dlci, ts0710);
              if (queue_delayed_work(muxpost_receive_work_queue, &mux_post_receive_work,0)==0) {
			queue_delayed_work(muxpost_receive_work_queue, &mux_post_receive_work,10);
              }
		wake_up_interruptible(&tty->read_wait);
		wake_up_interruptible(&tty->write_wait);
		tty->packet = 0;

	}
}

static void mux_throttle(struct tty_struct *tty)
{
	ts0710_con *ts0710 = &ts0710_connection;
	int line;
	int i;
	__u8 dlci;

	if (!tty) {
		return;
	}

	line = tty->index;
	if ((line < 0) || (line >= NR_MUXS)) {
		return;
	}

	TS0710_DEBUG("Enter into %s, minor number is: %d\n", __FUNCTION__,
		     line);

	dlci = tty2dlci[line];
	if ((ts0710->dlci[0].state != CONNECTED)
	    && (ts0710->dlci[0].state != FLOW_STOPPED)) {
		return;
	} else if ((ts0710->dlci[dlci].state != CONNECTED)
		   && (ts0710->dlci[dlci].state != FLOW_STOPPED)) {
		return;
	}

	if (ts0710->dlci[dlci].flow_control) {
		return;
	}

	for (i = 0; i < 3; i++) {
		if (ts0710_msc_msg
		    (ts0710, EA | FC | RTC | RTR | DV, MCC_CMD, dlci) < 0) {
			continue;
		} else {
			TS0710_LOG("MUXS Send Flow off on dlci %d\n", dlci);
			ts0710->dlci[dlci].flow_control = 1;
			break;
		}
	}
}

static void mux_unthrottle(struct tty_struct *tty)
{
	ts0710_con *ts0710 = &ts0710_connection;
	int line;
	__u8 dlci;
	mux_recv_struct *recv_info;

	if (!tty) {
		return;
	}
	line = tty->index;
	if ((line < 0) || (line >= NR_MUXS)) {
		return;
	}

	if ((!mux_recv_info_flags[line]) || (!mux_recv_info[line])) {
		return;
	}

	TS0710_DEBUG("Enter into %s, minor number is: %d\n", __FUNCTION__,
		     line);

	recv_info = mux_recv_info[line];
	dlci = tty2dlci[line];

	if (recv_info->total) {
		recv_info->post_unthrottle = 1;
              if (queue_delayed_work(muxpost_receive_work_queue, &mux_post_receive_work,0)==0) {
			queue_delayed_work(muxpost_receive_work_queue, &mux_post_receive_work,10);
              }
	} else {
		ts0710_flow_on(dlci, ts0710);
	}
}

static int mux_chars_in_buffer(struct tty_struct *tty)
{
	ts0710_con *ts0710 = &ts0710_connection;
	int retval;
	int line;
	__u8 dlci;
	mux_send_struct *send_info;

	retval = TS0710MUX_MAX_CHARS_IN_BUF;
	if (!tty) {
		goto out;
	}
	line = tty->index;
	if ((line < 0) || (line >= NR_MUXS)) {
		goto out;
	}

	dlci = tty2dlci[line];
	if (ts0710->dlci[0].state == FLOW_STOPPED) {
		TS0710_DEBUG
		    ("Flow stopped on all channels, returning MAX chars in buffer\n");
		goto out;
	} else if (ts0710->dlci[dlci].state == FLOW_STOPPED) {
		TS0710_DEBUG("Flow stopped, returning MAX chars in buffer\n");
		goto out;
	} else if (ts0710->dlci[dlci].state != CONNECTED) {
		TS0710_DEBUG("DLCI %d not connected\n", dlci);
		goto out;
	}

	if (!(mux_send_info_flags[line])) {
		goto out;
	}
	send_info = mux_send_info[line];
	if (!send_info) {
		goto out;
	}
	if (send_info->filled) {
		goto out;
	}

	retval = 0;

      out:
	return retval;
}

static int mux_chars_in_serial_buffer(struct tty_struct *tty)
{
	UNUSED_PARAM(tty);

	if ((COMM_FOR_MUX_DRIVER == 0) || (COMM_FOR_MUX_TTY == 0)) {
		TS0710_PRINTK
		    ("MUXS %s: (COMM_FOR_MUX_DRIVER == 0) || (COMM_FOR_MUX_TTY == 0)\n",
		     __FUNCTION__);

#ifndef USB_FOR_MUX
		TS0710_PRINTK
		    ("MUXS %s: tapisrv might be down!!! (serial_for_mux_driver == 0) || (serial_for_muxs_tty == 0)\n",
		     __FUNCTION__);
		TS0710_SIG2APLOGD();
#endif

		return 0;
	}
	return COMM_FOR_MUX_DRIVER->ops->chars_in_buffer(COMM_FOR_MUX_TTY);
}

int mux_write(struct tty_struct *tty,
		     const unsigned char *buf, int count)
{
	ts0710_con *ts0710 = &ts0710_connection;
	int line;
	__u8 dlci;
	mux_send_struct *send_info;
	__u8 *d_buf;
	__u16 c;

	if (count <= 0) {
		return 0;
	}

	if (!tty) {
		return 0;
	}

	line = tty->index;
	if ((line < 0) || (line >= NR_MUXS))
		return -ENODEV;

	dlci = tty2dlci[line];
	if (ts0710->dlci[0].state == FLOW_STOPPED) {
		TS0710_DEBUG
		    ("MUXS Flow stopped on all channels, returning zero /dev/muxs%d\n",
		     line);
		return 0;
	} else if (ts0710->dlci[dlci].state == FLOW_STOPPED) {
		TS0710_DEBUG("MUXS Flow stopped, returning zero /dev/muxs%d\n", line);
		return 0;
	} else if (ts0710->dlci[dlci].state == CONNECTED) {

		if (!(mux_send_info_flags[line])) {
			TS0710_PRINTK
			    ("MUXS Error: mux_write: mux_send_info_flags[%d] == 0\n",
			     line);
			return -ENODEV;
		}
		send_info = mux_send_info[line];
		if (!send_info) {
			TS0710_PRINTK
			    ("MUXS Error: mux_write: mux_send_info[%d] == 0\n",
			     line);
			return -ENODEV;
		}

		c = min(count, (ts0710->dlci[dlci].mtu - 1));
		if (c <= 0) {
			return 0;
		}

		if (test_and_set_bit(BUF_BUSY, &send_info->flags))
			return 0;

		if (send_info->filled) {
			printk("MUXS: write but filled == 1\r\n");
			clear_bit(BUF_BUSY, &send_info->flags);
			return 0;
		}

		d_buf = ((__u8 *) send_info->buf) + TS0710MUX_SEND_BUF_OFFSET;
		memcpy(&d_buf[0], buf, c);	//jim : no tag byte
		//      memcpy(&d_buf[1], buf, c);

		TS0710_DEBUG("MUXS Prepare to send %d bytes from /dev/muxs%d", c,
			     line);
		TS0710_DEBUGHEX(d_buf, c);

		send_info->frame = d_buf;
		queue_uih(send_info, c, ts0710, dlci);
		send_info->filled = 1;
		clear_bit(BUF_BUSY, &send_info->flags);



		if (mux_chars_in_serial_buffer(COMM_FOR_MUX_TTY) == 0) {
			/* Sending bottom half should be
			   run after return from this function */
			mux_sched_send();
		}
		return c;
	} else {
		TS0710_PRINTK("MUXS mux_write: DLCI %d not connected\n", dlci);
		return -EDISCONNECTED;
	}
}

static int mux_write_room(struct tty_struct *tty)
{
	ts0710_con *ts0710 = &ts0710_connection;
	int retval;
	int line;
	__u8 dlci;
	mux_send_struct *send_info;

	retval = 0;
	if (!tty) {
		goto out;
	}
	line = tty->index;
	if ((line < 0) || (line >= NR_MUXS)) {
		goto out;
	}

	dlci = tty2dlci[line];
	if (ts0710->dlci[0].state == FLOW_STOPPED) {
		TS0710_DEBUG("Flow stopped on all channels, returning ZERO\n");
		goto out;
	} else if (ts0710->dlci[dlci].state == FLOW_STOPPED) {
		TS0710_DEBUG("Flow stopped, returning ZERO\n");
		goto out;
	} else if (ts0710->dlci[dlci].state != CONNECTED) {
		TS0710_DEBUG("DLCI %d not connected\n", dlci);
		goto out;
	}

	if (!(mux_send_info_flags[line])) {
		goto out;
	}
	send_info = mux_send_info[line];
	if (!send_info) {
		goto out;
	}
	if (send_info->filled) {
		goto out;
	}

	retval = ts0710->dlci[dlci].mtu - 1;

      out:
	return retval;
}

static int mux_ioctl(struct tty_struct *tty, struct file *file,
		     unsigned int cmd, unsigned long arg)
{
	ts0710_con *ts0710 = &ts0710_connection;
	int line;
	__u8 dlci;

	UNUSED_PARAM(file);
	UNUSED_PARAM(arg);

	if (!tty) {
		return -EIO;
	}
	line = tty->index;
	if ((line < 0) || (line >= NR_MUXS)) {
		return -ENODEV;
	}

	dlci = tty2dlci[line];
	switch (cmd) {
	case TS0710MUX_IO_MSC_HANGUP:
		if (ts0710_msc_msg(ts0710, EA | RTR | DV, MCC_CMD, dlci) < 0) {
			return -EAGAIN;
		} else {
			return 0;
		}

	case TS0710MUX_IO_TEST_CMD:
		return ts0710_exec_test_cmd();

	default:
		break;
	}
	return -ENOIOCTLCMD;
}

static void mux_flush_buffer(struct tty_struct *tty)
{
	int line;

	if (!tty) {
		return;
	}

	line = tty->index;
	if ((line < 0) || (line >= NR_MUXS)) {
		return;
	}

	TS0710_PRINTK("MUXS %s: line is:%d\n", __FUNCTION__, line);

	if ((mux_send_info_flags[line])
	    && (mux_send_info[line])
	    && (mux_send_info[line]->filled)) {

		mux_send_info[line]->filled = 0;
	}

	wake_up_interruptible(&tty->write_wait);
#ifdef SERIAL_HAVE_POLL_WAIT
	wake_up_interruptible(&tty->poll_wait);
#endif
	if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
	    tty->ldisc->ops->write_wakeup) {
		(tty->ldisc->ops->write_wakeup) (tty);
	}

}
static int muxs_open(struct tty_struct *tty, struct file *filp)
{
	int retval;
	int line;
	__u8 dlci;
	__u8 cmdtty;
	__u8 datatty;
	mux_send_struct *send_info;
	mux_recv_struct *recv_info;
	
	printk("MUXS: muxs[%d] opened, COMM_FOR_MUX_DRIVER=%p, COMM_FOR_MUX_TTY=%p\n",tty->index, COMM_FOR_MUX_DRIVER, 
      COMM_FOR_MUX_TTY);
    
    
    WakeupBp();
	
	UNUSED_PARAM(filp);
	
	retval = -ENODEV;
	if ((COMM_FOR_MUX_DRIVER == NULL) || (COMM_FOR_MUX_TTY == NULL)) {

#ifdef USB_FOR_MUX
		TS0710_PRINTK("MUXS: please install and open IPC-USB first\n");
#else
		TS0710_PRINTK("MUXS: please install and open ttyX first\n");
#endif

		goto out;
	}

	if (!tty) {
		goto out;
	}
	line = tty->index;
	if ((line < 0) || (line >= NR_MUXS)) {
		goto out;
	}
#ifdef TS0710SERVER
	/* do nothing as a server */
	mux_tty[line]++;
	retval = 0;
#else
	mux_tty[line]++;
	dlci = tty2dlci[line];
	mux_opened++;
	if (dlci == 1) {

//#ifdef CONFIG_TS0710_MUX_SPI
#if 1
		if (cmux_mode == 0) {
			char buffer[256];
			char *buff = buffer;
			int i = 0;
			memset(buffer, 0, 256);
			mux_ringbuffer_flush(&muxs_rbuf);   //kewang
			if(mux_mode == 1)
				COMM_FOR_MUX_DRIVER->ops->write(COMM_FOR_MUX_TTY, "AT+SMMSWAP=0\r",
						strlen("AT+SMMSWAP=0\r"));
			else
			{
#ifdef UART_SW_FLOW_CONTROL
				COMM_FOR_MUX_DRIVER->ops->write(COMM_FOR_MUX_TTY, g_at,6);
#else
				COMM_FOR_MUX_DRIVER->ops->write(COMM_FOR_MUX_TTY, "at\r",
						   strlen("at\r"));
#endif
            		}
			//wait for response "OK \r"
			printk(" cmuxs send at and then wait...:<\n");
			msleep(1000);
			while (1) {

				int c = mux_ringbuffer_avail(&muxs_rbuf);
				printk("ts muxs receive %d chars\n",c);
				if (c > 0) {
					if(c>256)
						c=256;
					mux_ringbuffer_read(&muxs_rbuf, buff++, 1,
							    0);
					printk("muxs_ringbuffer_read %c", *(buff - 1));
					if (findInBuf
					    (buffer, 256, "OK")) {
					    	printk("muxs normal modem state !!!\r\n");
						break;
					}
					if (findInBuf
					    (buffer, 256, "ERROR")) {
						printk("muxs wait a ERROR,wrong modem state !!!!\n");
						break;
					}

				} else {
				    msleep(1000);// anjun modify from 2s to 1s
					if(mux_mode == 1)
						COMM_FOR_MUX_DRIVER->ops->write(COMM_FOR_MUX_TTY, "AT+SMMSWAP=0\r",
								strlen("AT+SMMSWAP=0\r"));
					else
					{
#ifdef UART_SW_FLOW_CONTROL
						COMM_FOR_MUX_DRIVER->ops->write(COMM_FOR_MUX_TTY, g_at,6);
#else
						COMM_FOR_MUX_DRIVER->ops->write(COMM_FOR_MUX_TTY, "at\r",
								strlen("at\r"));
#endif
					}
					i++;
					msleep(1000);// anjun add 1s 
				}
				if (i > 5) {
					printk("muxs No OK or ERROR wrong modem state !!!!\n");
					retval = -ENODEV;
					goto out;
				}
			}
#ifdef UART_SW_FLOW_CONTROL
            COMM_FOR_MUX_DRIVER->ops->write(COMM_FOR_MUX_TTY, g_cmux,13);
#else
			COMM_FOR_MUX_DRIVER->ops->write(COMM_FOR_MUX_TTY,
						   "at+cmux=0\r",
						   strlen("at+cmux=0\r"));
#endif
			msleep(2000);
			//empty ringbuffer
			mux_ringbuffer_flush(&muxs_rbuf);
			cmux_mode = 1;
			printk("muxs cmuxs mode is 1 !!\n");
		}
#endif

		/* Open server channel 0 first */
		if ((retval = ts0710_open_channel(0)) != 0) {
			TS0710_PRINTK("MUXS: Can't connect server channel 0!\n");
			ts0710_init();

			mux_tty[line]--;
			goto out;
		}
	}

	/* Allocate memory first. As soon as connection has been established, MUX may receive */
	if (mux_send_info_flags[line] == 0) {
		send_info =
		    (mux_send_struct *) kmalloc(sizeof(mux_send_struct),
						GFP_KERNEL);
		if (!send_info) {
			retval = -ENOMEM;

			mux_tty[line]--;
			goto out;
		}
		send_info->length = 0;
		send_info->flags = 0;
		send_info->filled = 0;
		mux_send_info[line] = send_info;
		mux_send_info_flags[line] = 1;
		TS0710_DEBUG("Allocate mux_send_info for /dev/muxs%d", line);
	}
	
	if (mux_recv_info_flags[line] == 0) {
		recv_info =
		    (mux_recv_struct *) kmalloc(sizeof(mux_recv_struct),
						GFP_KERNEL);
		if (!recv_info) {
			mux_send_info_flags[line] = 0;
			kfree(mux_send_info[line]);
			mux_send_info[line] = 0;
			TS0710_DEBUG("Free mux_send_info for /dev/muxs%d", line);
			retval = -ENOMEM;

			mux_tty[line]--;
			goto out;
		}
		recv_info->length = 0;
		recv_info->total = 0;
		recv_info->mux_packet = 0;
		recv_info->next = 0;
		recv_info->no_tty = line;
		recv_info->post_unthrottle = 0;
		mux_recv_info[line] = recv_info;
		mux_recv_info_flags[line] = 1;
		TS0710_DEBUG("Allocate mux_recv_info for /dev/muxs%d", line);
	}

	/* Now establish DLCI connection */
	cmdtty = dlci2tty[dlci].cmdtty;
	datatty = dlci2tty[dlci].datatty;
	if ((mux_tty[cmdtty] > 0) || (mux_tty[datatty] > 0)) {
		if ((retval = ts0710_open_channel(dlci)) != 0) {
			TS0710_PRINTK("MUXS: Can't connected channel %d!\n",
				      dlci);
			ts0710_reset_dlci(dlci);

			mux_send_info_flags[line] = 0;
			kfree(mux_send_info[line]);
			mux_send_info[line] = 0;
			TS0710_DEBUG("Free mux_send_info for /dev/muxs%d", line);

			mux_recv_info_flags[line] = 0;
			free_mux_recv_struct(mux_recv_info[line]);
			mux_recv_info[line] = 0;
			TS0710_DEBUG("Free mux_recv_info for /dev/muxs%d", line);

			mux_tty[line]--;
			goto out;
		}
	}
	retval = 0;
#endif
      out:
	return retval;
}

static inline int rt_policy(int policy)
{
	if (unlikely(policy == SCHED_FIFO) || unlikely(policy == SCHED_RR))
		return 1;
	return 0;
}

static inline int task_has_rt_policy(struct task_struct *p)
{
	return rt_policy(p->policy);
}


/* muxs dispatcher, call from serial.c receiver_chars() */
static void mux_dispatcher(struct tty_struct *tty)
{
	UNUSED_PARAM(tty);
  up(&receive_sem);
}

/*For BP UART problem Begin*/
#ifdef TS0710SEQ2
static int send_ack(ts0710_con * ts0710, __u8 seq_num, __u8 bp_seq1,
		    __u8 bp_seq2)
#else
static int send_ack(ts0710_con * ts0710, __u8 seq_num)
#endif
{
	__u8 buf[20];
	short_frame *ack;

#ifdef TS0710SEQ2
	static __u16 ack_seq = 0;
#endif
	return 0;		//jim
	ack = (short_frame *) (buf + 1);
	ack->h.addr.ea = 1;
	ack->h.addr.cr = ((ts0710->initiator) & 0x1);
	ack->h.addr.d = 0;
	ack->h.addr.server_chn = 0;
	ack->h.control = ACK;
	ack->h.length.ea = 1;

#ifdef TS0710SEQ2
	ack->h.length.len = 5;
	ack->data[0] = seq_num;
	ack->data[1] = bp_seq1;
	ack->data[2] = bp_seq2;
	ack->data[3] = (ack_seq & 0xFF);
	ack->data[4] = (ack_seq >> 8) & 0xFF;
	ack_seq++;
	ack->data[5] = crc_calc((__u8 *) ack, SHORT_CRC_CHECK);
#else
	ack->h.length.len = 1;
	ack->data[0] = seq_num;
	ack->data[1] = crc_calc((__u8 *) ack, SHORT_CRC_CHECK);
#endif

	return basic_write(ts0710, buf,
			   (sizeof(short_frame) + FCS_SIZE +
			    ack->h.length.len));
}


static int mux_set_thread_pro(int pro)
{
	int ret;
	struct sched_param s;

	/* just for this write, set us real-time */
	if (!task_has_rt_policy(current)) {
		struct cred *new = prepare_creds();
		cap_raise(new->cap_effective, CAP_SYS_NICE);
		commit_creds(new);
		s.sched_priority = MAX_RT_PRIO - pro;
		ret = sched_setscheduler(current, SCHED_RR, &s);
		if(ret!=0)
			printk("MUXS: set priority failed!\n");
	}
	return 0;
}

/*For BP UART problem End*/

static void receive_worker(struct work_struct *private_)
{
	//struct tty_struct *tty = COMM_FOR_MUX_TTY;
	int count;
	static unsigned char tbuf[TS0710MUX_MAX_BUF_SIZE];
	static unsigned char *tbuf_ptr = &tbuf[0];
	static unsigned char *start_flag = 0;
	unsigned char *search, *to, *from;
	short_frame *short_pkt;
	long_frame *long_pkt;
	static int framelen = -1;
	/*For BP UART problem Begin */
	static __u8 expect_seq = 0;
	__u32 crc_error;
	__u8 *uih_data_start;
	__u32 uih_len;
	/*For BP UART problem End */
	UNUSED_PARAM(private_);


	count = mux_ringbuffer_avail(&muxs_rbuf);

	if (count == 0) {

		return;
	}

	if (count > (TS0710MUX_MAX_BUF_SIZE - (tbuf_ptr - tbuf))) {

		count = (TS0710MUX_MAX_BUF_SIZE - (tbuf_ptr - tbuf));
		up(&receive_sem);

	}

	mux_ringbuffer_read(&muxs_rbuf, tbuf_ptr, count, 0);
	tbuf_ptr += count;


	if ((start_flag != 0) && (framelen != -1)) {
		if ((tbuf_ptr - start_flag) < framelen) {
			clear_bit(RECV_RUNNING, &mux_recv_flags);
			return;
		}
	}

	search = &tbuf[0];
	while (1) {
		if (start_flag == 0) {	/* Frame Start Flag not found */
			framelen = -1;
			while (search < tbuf_ptr) {
				if (*search == TS0710_BASIC_FLAG) {

					start_flag = search;
					break;
				}
#ifdef TS0710LOG
				else {
					TS0710_LOG(">S %02x %c\n", *search,
						   *search);
				}
#endif

				search++;
			}

			if (start_flag == 0) {
				tbuf_ptr = &tbuf[0];
				break;
			}
		} else {	/* Frame Start Flag found */
			/* 1 start flag + 1 address + 1 control + 1 or 2 length + lengths data + 1 FCS + 1 end flag */
			/* For BP UART problem 1 start flag + 1 seq_num + 1 address + ...... */
			/*if( (framelen == -1) && ((tbuf_ptr - start_flag) > TS0710_MAX_HDR_SIZE) ) */
			if ((framelen == -1) && ((tbuf_ptr - start_flag) > (TS0710_MAX_HDR_SIZE + SEQ_FIELD_SIZE))) {	/*For BP UART problem */
				/*short_pkt = (short_frame *) (start_flag + 1); */
				short_pkt = (short_frame *) (start_flag + ADDRESS_FIELD_OFFSET);	/*For BP UART problem */
				if (short_pkt->h.length.ea == 1) {	/* short frame */
					/*framelen = TS0710_MAX_HDR_SIZE + short_pkt->h.length.len + 1; */
					framelen = TS0710_MAX_HDR_SIZE + short_pkt->h.length.len + 1 + SEQ_FIELD_SIZE;	/*For BP UART problem */
				} else {	/* long frame */
					/*long_pkt = (long_frame *) (start_flag + 1); */
					long_pkt = (long_frame *) (start_flag + ADDRESS_FIELD_OFFSET);	/*For BP UART problem */
					/*framelen = TS0710_MAX_HDR_SIZE + GET_LONG_LENGTH( long_pkt->h.length ) + 2; */
					framelen = TS0710_MAX_HDR_SIZE + GET_LONG_LENGTH(long_pkt->h.length) + 2 + SEQ_FIELD_SIZE;	/*For BP UART problem */
				}

				/*if( framelen > TS0710MUX_MAX_TOTAL_FRAME_SIZE ) { */
				if (framelen > (TS0710MUX_MAX_TOTAL_FRAME_SIZE + SEQ_FIELD_SIZE)) {	/*For BP UART problem */
					TS0710_LOGSTR_FRAME(0, start_flag,
							    (tbuf_ptr -
							     start_flag));
					TS0710_PRINTK
					    ("MUXS Error: %s: frame length:%d is bigger than Max total frame size:%d\n",
		 /*__FUNCTION__, framelen, TS0710MUX_MAX_TOTAL_FRAME_SIZE);*/
					     __FUNCTION__, framelen, (TS0710MUX_MAX_TOTAL_FRAME_SIZE + SEQ_FIELD_SIZE));	/*For BP UART problem */
					search = start_flag + 1;
					start_flag = 0;
					framelen = -1;
					continue;
				}
			}

			if ((framelen != -1)
			    && ((tbuf_ptr - start_flag) >= framelen)) {
				if (*(start_flag + framelen - 1) == TS0710_BASIC_FLAG) {	/* OK, We got one frame */

					/*For BP UART problem Begin */
					TS0710_LOGSTR_FRAME(0, start_flag,
							    framelen);
					TS0710_DEBUGHEX(start_flag, framelen);

					short_pkt =
					    (short_frame *) (start_flag +
							     ADDRESS_FIELD_OFFSET);
					if ((short_pkt->h.length.ea) == 0) {
						long_pkt =
						    (long_frame *) (start_flag +
								    ADDRESS_FIELD_OFFSET);
						uih_len =
						    GET_LONG_LENGTH(long_pkt->h.
								    length);
						uih_data_start =
						    long_pkt->h.data;

						crc_error =
						    crc_check((__u8
							       *) (start_flag +
								   SLIDE_BP_SEQ_OFFSET),
							      LONG_CRC_CHECK +
							      1,
							      *(uih_data_start +
								uih_len));
					} else {
						uih_len =
						    short_pkt->h.length.len;
						uih_data_start =
						    short_pkt->data;

						crc_error =
						    crc_check((__u8
							       *) (start_flag +
								   SLIDE_BP_SEQ_OFFSET),
							      SHORT_CRC_CHECK +
							      1,
							      *(uih_data_start +
								uih_len));
					}

					if (!crc_error) {
						if (	/*expect_seq ==
							 *(start_flag +
							 SLIDE_BP_SEQ_OFFSET)*/
							   1) {
							expect_seq =
							    *(start_flag +
							      SLIDE_BP_SEQ_OFFSET);
							expect_seq++;
							if (expect_seq >= 4) {
								expect_seq = 0;
							}
#ifdef TS0710SEQ2
							send_ack
							    (&ts0710_connection,
							     expect_seq,
							     *(start_flag +
							       FIRST_BP_SEQ_OFFSET),
							     *(start_flag +
							       SECOND_BP_SEQ_OFFSET));
#else
							send_ack
							    (&ts0710_connection,
							     expect_seq);
#endif

							ts0710_recv_data
							    (&ts0710_connection,
							     start_flag +
							     ADDRESS_FIELD_OFFSET,
							     framelen - 2 -
							     SEQ_FIELD_SIZE);
						} else {

#ifdef TS0710DEBUG
							if (*
							    (start_flag +
							     SLIDE_BP_SEQ_OFFSET)
!= 0x9F) {
#endif

								TS0710_LOG
								    ("MUXS sequence number %d is not expected %d, discard data!\n",
								     *
								     (start_flag
								      +
								      SLIDE_BP_SEQ_OFFSET),
								     expect_seq);

#ifdef TS0710SEQ2
								send_ack
								    (&ts0710_connection,
								     expect_seq,
								     *
								     (start_flag
								      +
								      FIRST_BP_SEQ_OFFSET),
								     *
								     (start_flag
								      +
								      SECOND_BP_SEQ_OFFSET));
#else
								send_ack
								    (&ts0710_connection,
								     expect_seq);
#endif

#ifdef TS0710DEBUG
							} else {
								*(uih_data_start
								  + uih_len) =
						     0;
								TS0710_PRINTK
								    ("MUXS bp log: %s\n",
								     uih_data_start);
							}
#endif

						}
					} else {	/* crc_error */
						search = start_flag + 1;
						start_flag = 0;
						framelen = -1;
						continue;
					}	/*End if(!crc_error) */

					
					search = start_flag + framelen;
				} else {
					TS0710_LOGSTR_FRAME(0, start_flag,
							    framelen);
					TS0710_DEBUGHEX(start_flag, framelen);
					TS0710_PRINTK
					    ("MUXS: Lost synchronization!\n");
					search = start_flag + 1;
				}

				start_flag = 0;
				framelen = -1;
				continue;
			}

			if (start_flag != &tbuf[0]) {

				to = tbuf;
				from = start_flag;
				count = tbuf_ptr - start_flag;
				while (count--) {
					*to++ = *from++;
				}

				tbuf_ptr -= (start_flag - tbuf);
				start_flag = tbuf;

			}
			break;
		}		/* End Frame Start Flag found */
	}			/* End while(1) */

	clear_bit(RECV_RUNNING, &mux_recv_flags);
}


static int mux_receive_thread(void *data)
{

	mux_set_thread_pro(95);

	while (1) {
		
		down(&receive_sem);
		
		if (muxs_exiting==1) {
			muxs_exiting = 2;
			return 0;
		}
		receive_worker(0);

	}
}

static void post_recv_worker(struct work_struct *private_)
{
	ts0710_con *ts0710 = &ts0710_connection;
	int tty_idx;
	struct tty_struct *tty;
	__u8 post_recv;
	__u8 flow_control;
	__u8 dlci;
	mux_recv_struct *recv_info, *recv_info2, *post_recv_q;
	int recv_room;
	mux_recv_packet *recv_packet, *recv_packet2;

	UNUSED_PARAM(private_);
	mux_set_thread_pro(90);

	if (test_and_set_bit(RECV_RUNNING, &mux_recv_flags)) {
		if (queue_delayed_work(muxpost_receive_work_queue, &mux_post_receive_work,0)==0) {
			queue_delayed_work(muxpost_receive_work_queue, &mux_post_receive_work,10);
              }
		return;
	}

	TS0710_DEBUG("MUXS Enter into post_recv_worker \n");

	post_recv = 0;
	if (!mux_recv_queue) {
		goto out;
	}

	post_recv_q = NULL;
	recv_info2 = mux_recv_queue;
	while ((recv_info = recv_info2)) {
		recv_info2 = recv_info->next;

		if (!(recv_info->total)) {
			TS0710_PRINTK
			    ("MUXS Error: %s: Should not get here, recv_info->total == 0 \n",
			     __FUNCTION__);
			continue;
		}

		tty_idx = recv_info->no_tty;
		dlci = tty2dlci[tty_idx];
		tty = mux_table[tty_idx];
		if ((!mux_tty[tty_idx]) || (!tty)) {
			TS0710_PRINTK
			    ("MUXS: No application waiting for, free recv_info! tty_idx:%d\n",
			     tty_idx);
			mux_recv_info_flags[tty_idx] = 0;
			free_mux_recv_struct(mux_recv_info[tty_idx]);
			mux_recv_info[tty_idx] = 0;
			ts0710_flow_on(dlci, ts0710);
			continue;
		}

		TS0710_DEBUG("/dev/muxs%d recv_info->total is: %d", tty_idx,
			     recv_info->total);

		if (test_bit(TTY_THROTTLED, &tty->flags)) {
			add_post_recv_queue(&post_recv_q, recv_info);
			continue;
		}
		flow_control = 0;
		recv_packet2 = recv_info->mux_packet;
		while (recv_info->total) {
			recv_room = 65535;
			if (tty->receive_room)
				recv_room = tty->receive_room;

			if (recv_info->length) {
				if (recv_room < recv_info->length) {
					flow_control = 1;
					break;
				}

				/* Put queued data into read buffer of tty */
				TS0710_DEBUG
				    ("MUXS Put queued recv data into read buffer of /dev/muxs%d\n",
				     tty_idx);
				TS0710_DEBUGHEX(recv_info->data,
						recv_info->length);
				(tty->ldisc->ops->receive_buf) (tty, recv_info->data,
							  NULL,
							  recv_info->length);
				recv_info->total -= recv_info->length;
				recv_info->length = 0;
			} else {	/* recv_info->length == 0 */
				if ((recv_packet = recv_packet2)) {
					recv_packet2 = recv_packet->next;

					if (recv_room < recv_packet->length) {
						flow_control = 1;
						recv_info->mux_packet =
						    recv_packet;
						break;
					}

					/* Put queued data into read buffer of tty */
					TS0710_DEBUG
					    ("MUXS Put queued recv data into read buffer2 of /dev/muxs%d \n",
					     tty_idx);
					TS0710_DEBUGHEX(recv_packet->data,
							recv_packet->length);
					(tty->ldisc->ops->receive_buf) (tty,
								  recv_packet->
								  data, NULL,
								  recv_packet->
								  length);
					recv_info->total -= recv_packet->length;
					free_mux_recv_packet(recv_packet);
				} else {
					TS0710_PRINTK
					    ("MUXS Error: %s: Should not get here, recv_info->total is:%u \n",
					     __FUNCTION__, recv_info->total);
				}
			}	/* End recv_info->length == 0 */
		}		/* End while( recv_info->total ) */

		if (!(recv_info->total)) {
			/* Important clear */
			recv_info->mux_packet = 0;

			if (recv_info->post_unthrottle) {
				/* Do something for post_unthrottle */
				ts0710_flow_on(dlci, ts0710);
				recv_info->post_unthrottle = 0;
			}
		} else {
			add_post_recv_queue(&post_recv_q, recv_info);

			if (flow_control) {
				/* Do something for flow control */
				if (recv_info->post_unthrottle) {
					set_bit(TTY_THROTTLED, &tty->flags);
					recv_info->post_unthrottle = 0;
				} else {
					ts0710_flow_off(tty, dlci, ts0710);
				}
			}	/* End if( flow_control ) */
		}
	}			/* End while( (recv_info = recv_info2) ) */

	mux_recv_queue = post_recv_q;

      out:
	if (post_recv) {
              if (queue_delayed_work(muxpost_receive_work_queue, &mux_post_receive_work,0)==0) {
			queue_delayed_work(muxpost_receive_work_queue, &mux_post_receive_work,10);
              }
	}
	clear_bit(RECV_RUNNING, &mux_recv_flags);
}

/* mux sender, call from serial.c transmit_chars() */
static void mux_sender(void)
{
	mux_send_struct *send_info;
	int chars;
	__u8 idx;

	chars = mux_chars_in_serial_buffer(COMM_FOR_MUX_TTY);
	if (!chars) {
		/* chars == 0 */
		TS0710_LOG("<[]\n");
		mux_sched_send();
		return;
	}

	idx = mux_send_info_idx;
	if ((idx < NR_MUXS) && (mux_send_info_flags[idx])) {
		send_info = mux_send_info[idx];
		if ((send_info)
		    && (send_info->filled)
		    && (send_info->length <=
			(TS0710MUX_SERIAL_BUF_SIZE - chars)) && (cmux_mode==1) ) {

			mux_sched_send();
		}
	}
}

static void send_worker(struct work_struct *private_)
{
	ts0710_con *ts0710 = &ts0710_connection;
	__u8 j;
	mux_send_struct *send_info;
	int chars;
	struct tty_struct *tty;
	__u8 dlci;

	UNUSED_PARAM(private_);

	TS0710_DEBUG("MUXS Enter into send_worker");

	mux_send_info_idx = NR_MUXS;

	if (ts0710->dlci[0].state == FLOW_STOPPED) {
		TS0710_DEBUG("MUXS Flow stopped on all channels\n");
		return;
	}

	mux_set_thread_pro(80);

	for (j = 0; j < NR_MUXS; j++) {

		if (!(mux_send_info_flags[j])) {
			continue;
		}

		send_info = mux_send_info[j];
		if (!send_info) {
			continue;
		}

		if (!(send_info->filled)) {
			continue;
		}

		dlci = tty2dlci[j];
		if (ts0710->dlci[dlci].state == FLOW_STOPPED) {
			TS0710_DEBUG("MUXS Flow stopped on channel DLCI: %d\n",
				     dlci);
			continue;
		} else if (ts0710->dlci[dlci].state != CONNECTED) {
			TS0710_DEBUG("MUXS DLCI %d not connected\n", dlci);
			send_info->filled = 0;
			continue;
		}
		chars = mux_chars_in_serial_buffer(COMM_FOR_MUX_TTY);
		if (send_info->length <= (TS0710MUX_SERIAL_BUF_SIZE - chars)) {
			TS0710_DEBUG("MUXS Send queued UIH for /dev/muxs%d\n", j);
			basic_write(ts0710, (__u8 *) send_info->frame,
				    send_info->length);
			send_info->length = 0;
			send_info->filled = 0;
      			if(ts0710_is_rmnet_ttyidx(j)) {
		            TS0710_DEBUG("rmnet_restart on %d\n", j);	
                            rmnet_restart_queue(j);
			}
		} else {
			mux_send_info_idx = j;
			break;
		}
	}			/* End for() loop */

	/* Queue UIH data to be transmitted */
	for (j = 0; j < NR_MUXS; j++) {

		if (!(mux_send_info_flags[j])) {
			continue;
		}

		send_info = mux_send_info[j];
		if (!send_info) {
			continue;
		}

		if (send_info->filled) {
			continue;
		}

		/* Now queue UIH data to send_info->buf */

		if (!mux_tty[j]) {
			continue;
		}

		tty = mux_table[j];
		if (!tty) {
			continue;
		}

		dlci = tty2dlci[j];
		if (ts0710->dlci[dlci].state == FLOW_STOPPED) {
			TS0710_DEBUG("MUXS Flow stopped on channel DLCI: %d\n",
				     dlci);
			continue;
		} else if (ts0710->dlci[dlci].state != CONNECTED) {
			TS0710_DEBUG("MUXS DLCI %d not connected\n", dlci);
			continue;
		}

		if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP))
		    && tty->ldisc->ops->write_wakeup) {
			(tty->ldisc->ops->write_wakeup) (tty);
		}
		wake_up_interruptible(&tty->write_wait);

#ifdef SERIAL_HAVE_POLL_WAIT
		wake_up_interruptible(&tty->poll_wait);
#endif

		if (send_info->filled) {
			if (j < mux_send_info_idx) {
				mux_send_info_idx = j;
			}
		}
	}			/* End for() loop */
}
static int mux_proc_read(char *page, char **start, off_t off, int count, 
	int *eof, void *data)
{
	int len;
	if(off >0){
		*eof=1;
		return 0;
	}
	len = sprintf(page, "%d\n", mux_mode);
	return len;
}

static int mux_proc_write(struct file *filp, const char __user *buf, 
	unsigned long len, void *data)
{
	char mux_buf[len+1];
	int val;

	memset(mux_buf, 0, len+1);
	if(len > 0) {
		if (copy_from_user(mux_buf, buf, len))
			return -EFAULT;
		val = simple_strtoul(mux_buf, NULL, 10);
		mux_mode = val;	
	}
	return len;
}

static int mux_create_proc(void)
{
	struct proc_dir_entry *mux_entry;

	mux_entry = create_proc_entry("muxs_mode", 0666, NULL);  //creat /proc/muxs_mode
	if (!mux_entry) {
		printk(KERN_INFO"Can not create mux proc entry\n");
		return -ENOMEM;
	}
	mux_entry->read_proc = mux_proc_read;
	mux_entry->write_proc = mux_proc_write;
	return 0;
}

static void mux_remove_proc(void)
{
	remove_proc_entry("muxs_mode", NULL);  //remove /proc/muxs_mode
}

static const struct tty_operations tty_ops = {
	.open = muxs_open,
	.close = mux_close,
	.write = mux_write,
	.write_room = mux_write_room,
	.flush_buffer = mux_flush_buffer,
	.chars_in_buffer = mux_chars_in_buffer,
	.throttle = mux_throttle,
	.unthrottle = mux_unthrottle,
	.ioctl = mux_ioctl,
};
static int __init muxs_init(void)
{
	unsigned int j,retval;
	
	//tty_ops = kmalloc(sizeof(struct tty_operations),GFP_KERNEL);

#ifdef  SPRD_DRV_WAKEUP_BP
	ApWakeBpGpioInit();
#endif
	ts0710_init();

	mux_ringbuffer_init(&muxs_rbuf, mux_data, TS0710MUX_MAX_BUF_SIZE * 10);

	for (j = 0; j < NR_MUXS; j++) {
		mux_send_info_flags[j] = 0;
		mux_send_info[j] = 0;
		mux_recv_info_flags[j] = 0;
		mux_recv_info[j] = 0;
	}
	mux_send_info_idx = NR_MUXS;
	mux_recv_queue = NULL;
	mux_recv_flags = 0;

	//mux_send_work.work.prio = 80;
	//mux_post_receive_work.work.prio = 90;

	sema_init(&receive_sem,0);
	retval = kernel_thread(mux_receive_thread, NULL, 0);
	BUG_ON(0 == retval);

	muxsend_work_queue = create_workqueue("muxssend");
	muxpost_receive_work_queue = create_workqueue("muxspostreceive");

	memset(&mux_driver, 0, sizeof(struct tty_driver));
	memset(&mux_tty, 0, sizeof(mux_tty));
/* kewang, start */
	//mux_driver.kref.count = 1;
	kref_init(&mux_driver.kref);
/* kewang, end */
	mux_driver.magic = TTY_DRIVER_MAGIC;
	mux_driver.driver_name = "ts0710muxs";
	mux_driver.name = "ts0710muxs";
	mux_driver.major =TS0710MUX_MAJOR ;
	mux_driver.minor_start = TS0710MUX_MINOR_START;
	mux_driver.num = NR_MUXS;
	mux_driver.name_base = 0;
	mux_driver.type = TTY_DRIVER_TYPE_SERIAL;
	mux_driver.subtype = SERIAL_TYPE_NORMAL;
	mux_driver.init_termios = tty_std_termios;
	mux_driver.init_termios.c_iflag = 0;
	mux_driver.init_termios.c_oflag = 0;
	mux_driver.init_termios.c_cflag = B38400 | CS8 | CREAD;
	mux_driver.init_termios.c_lflag = 0;
	mux_driver.flags = TTY_DRIVER_RESET_TERMIOS | TTY_DRIVER_REAL_RAW;

	mux_driver.other = NULL;
	
	/*tty_ops->open = muxs_open;
	tty_ops->close = mux_close;
	tty_ops->write = mux_write;
	tty_ops->write_room = mux_write_room;
	tty_ops->flush_buffer = mux_flush_buffer;
	tty_ops->chars_in_buffer = mux_chars_in_buffer;
	tty_ops->throttle = mux_throttle;
	tty_ops->unthrottle = mux_unthrottle;
	tty_ops->ioctl = mux_ioctl;*/

	tty_set_operations(&mux_driver,&tty_ops);

	mux_driver.owner = THIS_MODULE;

	if (tty_register_driver(&mux_driver))
		panic("Couldn't register muxs driver");
	mux_table = mux_driver.ttys;
	COMM_MUXS_DISPATCHER = mux_dispatcher;
	COMM_MUXS_SENDER = mux_sender;

	if(mux_create_proc())
		printk("create muxs proc interface failed!\n");

	return 0;
}

static void __exit muxs_exit(void)
{
	int j;

	COMM_MUXS_DISPATCHER = NULL;
	COMM_MUXS_SENDER = NULL;


	mux_send_info_idx = NR_MUXS;
	mux_recv_queue = NULL;
	for (j = 0; j < NR_MUXS; j++) {
		if ((mux_send_info_flags[j]) && (mux_send_info[j])) {
			kfree(mux_send_info[j]);
		}
		mux_send_info_flags[j] = 0;
		mux_send_info[j] = 0;

		if ((mux_recv_info_flags[j]) && (mux_recv_info[j])) {
			free_mux_recv_struct(mux_recv_info[j]);
		}
		mux_recv_info_flags[j] = 0;
		mux_recv_info[j] = 0;
	}
	muxs_exiting =1;
	destroy_workqueue(muxsend_work_queue);
	up(&receive_sem);
	while(muxs_exiting ==1) {
		msleep(10);
	}
	destroy_workqueue(muxpost_receive_work_queue);

	if (tty_unregister_driver(&mux_driver))
		panic("Couldn't unregister muxs driver");

	mux_remove_proc();
}

module_init(muxs_init);
module_exit(muxs_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Harald Welte <laforge@openezx.org>");
MODULE_DESCRIPTION("GSM TS 07.10 Multiplexer");

