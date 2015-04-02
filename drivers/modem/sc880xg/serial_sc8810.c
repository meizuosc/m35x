/* linux/drivers/serial/serial_sp.c
 *
 *
 * Copyright (C) 2010 Spreadtrum
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
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/tty.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/device.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/termios.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <linux/clk.h>
#include <asm/clkdev.h>
#include <linux/err.h>
#include <mach/hardware.h>
#include <mach/mfp.h>
#include <mach/io.h>
#include <mach/clock_common.h>
#include <mach/gpio.h>
/* wakelock UART0 for BT */
#include <linux/wakelock.h>  
#include <mach/regs_int.h>    

//#define CONFIG_TS0710_MUX_UART

#ifdef SERIAL_DEBUG_8810
#define PRINT_DEBUG_SER printk
#else
#define PRINT_DEBUG_SER 
#endif 

#define UART_OVERRUN_DEBUG
#define UART_SW_FLOW_CONTROL
//#define UART_DUMP_DATA_TO_FILE

#ifdef UART_SW_FLOW_CONTROL_DEBUG
#define PRINT_ONTIM  printk
#else
#define PRINT_ONTIM 
#endif


#ifdef CONFIG_ARCH_SC8810
#define UART_NR 3
#else //CONFIG_ARCH_SC8810 
#ifdef CONFIG_ARCH_SC8800S
#define UART_NR 4
#else //CONFIG_ARCH_SC8800S
#error "not define CONFIG_ARCH_SC8800S or CONFIG_ARCH_SC8810"
#endif //CONFIG_ARCH_SC8800S
#endif //CONFIG_ARCH_SC8810

#ifdef CONFIG_MODEM_SC6620
/* If use serial uart as mux, define this MACRO*/
#define CONFIG_TS0710_MUX_SERIAL_UART
#endif

/*
 * port type,it ought to be defined in serial_core.h,
 * we just put here temporary 
*/
#define PORT_SPX 1234

#define SP_TTY_NAME	"ttyS"
#define SP_TTY_MINOR_START	64
#define SP_TTY_MAJOR	TTY_MAJOR

#define UART_CLK	26000000

/*offset*/
#define ARM_UART_TXD	0x0000
#define ARM_UART_RXD	0x0004
#define ARM_UART_STS0	0x0008
#define ARM_UART_STS1	0x000C
#define ARM_UART_IEN	0x0010
#define ARM_UART_ICLR	0x0014
#define ARM_UART_CTL0	0x0018
#define ARM_UART_CTL1	0x001C
#define ARM_UART_CTL2	0x0020
#define ARM_UART_CLKD0	0x0024
#define ARM_UART_CLKD1	0x0028
#define ARM_UART_STS2	0x002C
/*UART IRQ num*/
#ifdef CONFIG_ARCH_SC8810

#define IRQ_WAKEUP 	0    // Wakeup IRQ for BT
#define IRQ_UART0	2
#define IRQ_UART1	3
#define IRQ_UART2	4

#endif


/*UART FIFO watermark*/
#define SP_TX_FIFO	0x40//8 //0x20//added by tao for mux uart flow ctrl

#ifdef UART_SW_FLOW_CONTROL
#define SP_RX_FIFO_UART2 0x7F//1	//added by tao for mux uart flow ctrl
#endif

#define SP_RX_FIFO	0x40//1	

/*UART IEN*/
#define UART_IEN_RX_FIFO_FULL	(0x1<<0)
#define UART_IEN_TX_FIFO_EMPTY	(0x1<<1)
#define UART_IEN_BREAK_DETECT	(0x1<<7)
#define UART_IEN_TIMEOUT     	(0x1<<13)

/*data length*/
#define UART_DATA_BIT	(0x3<<2)
#define UART_DATA_5BIT	(0x0<<2)
#define UART_DATA_6BIT	(0x1<<2)
#define UART_DATA_7BIT	(0x2<<2)
#define UART_DATA_8BIT	(0x3<<2)
/*stop bit*/
#define UART_STOP_1BIT	(0x1<<4)
#define UART_STOP_2BIT	(0x3<<4)
/*parity*/
#define UART_PARITY	0x3
#define UART_PARITY_EN	0x2
#define UART_EVEN_PAR	0x0
#define UART_ODD_PAR	0x1
/*line status */
#define UART_LSR_OE	(0x1<<4)
#define UART_LSR_FE	(0x1<<3)
#define UART_LSR_PE	(0x1<<2)
#define UART_LSR_BI	(0x1<<7)
#define UART_LSR_DR	(0x1<<8)
/*flow control */
#define RX_HW_FLOW_CTL_THRESHOLD	0x40
#define RX_HW_FLOW_CTL_EN		(0x1<<7)
#define TX_HW_FLOW_CTL_EN		(0x1<<8)
/*status indicator*/
#define UART_STS_RX_FIFO_FULL	(0x1<<0)
#define UART_STS_TX_FIFO_EMPTY	(0x1<<1)
#define UART_STS_BREAK_DETECT	(0x1<<7)
#define UART_STS_TIMEOUT     	(0x1<<13)

#ifdef UART_OVERRUN_DEBUG
#define UART_STS_RX_FIFO_OVERFLOW   (0x1<<4)  /*anjun added this for test*/
#define UART_STS_PARITY_ERR   (0x1<<2)  /*anjun added this for test*/
#define UART_STS_FRAME_ERR   (0x1<<3)  /*anjun added this for test*/
#endif

#ifdef UART_SW_FLOW_CONTROL
#define HDLC_FLAG 0x7e
#define HDLC_ESCAPE 0x7d
#define HDLC_ESCAPE_MASK 0x20

static int g_rec_frame_num = 0;
static int g_rec_frame_num_modem = 0;
static bool g_last_rec_escape = false;
static unsigned int s_step = 0;
static bool g_revert = false;
static bool mux_flow_ctrl_status =0;

extern int sprd_3rdparty_gpio_mux_rx_ctrl;
#endif
/*baud rate*/
#define BAUD_1200_26M	0x54A0
#define BAUD_2400_26M	0x2A50
#define BAUD_4800_26M	0x1528
#define BAUD_9600_26M	0x0A94
#define BAUD_19200_26M	0x054A
#define BAUD_38400_26M	0x02A5
#define BAUD_57600_26M	0x0152
#define BAUD_115200_26M	0x00E2
#define BAUD_230400_26M	0x0071
#define BAUD_460800_26M	0x0038
#define BAUD_921600_26M	0x001C
#define BAUD_1000000_26M 0x001A
#define BAUD_1152000_26M 0x0016
#define BAUD_1500000_26M 0x0011
#define BAUD_2000000_26M 0x000D
#define BAUD_2500000_26M 0x000B
#define BAUD_3000000_26M 0x0009

#define SERIALSC8800_IS_INVALID_LINE(line) ((line >=0 && line < UART_NR)?0:1)
/*SPRD FAE anjun added for Uart0 clk source change to 96M --start */
#define BAUD_3000000_96M 0x1001F  
/*SPRD FAE anjun added for Uart0 clk source change to 96M --end */

extern void printascii(const char *);

#ifdef CONFIG_TS0710_MUX_SERIAL_UART
//static struct tty_struct serial_tty;
extern struct mux_ringbuffer muxs_rbuf;
struct tty_driver *serial_for_muxs_driver = NULL;
static int serial_mux_guard = 0;
struct tty_struct *serial_for_muxs_tty = NULL;
extern ssize_t mux_ringbuffer_write(struct mux_ringbuffer *rbuf, const u8 *buf, size_t len);
extern ssize_t mux_ringbuffer_free(struct mux_ringbuffer *rbuf);
extern int cmuxs_opened(void);
extern int is_cmuxs_mode(void);
void (*serial_muxs_dispatcher)(struct tty_struct *tty) = NULL;
void (*serial_muxs_sender)(void) = NULL;
static int g_support_mux_uart_device = 2; /*Only uart 2 support cmux*/
#else

#endif

static struct wake_lock uart_rx_lock;  // UART0  RX  IRQ 
static bool is_uart_rx_wakeup;

int Bt_Uart0_Opened = 0 ;
static inline unsigned int serial_in(struct uart_port *port,int offset)
{
	return __raw_readl(port->membase+offset);
}
static inline void  serial_out(struct uart_port *port,int offset,int value)
{
	//printascii("enter serial_out!!!\r\n");
	__raw_writel(value,port->membase+offset);
	//printascii("leave serial_out!!!\r\n");
}

unsigned int serialsc8800_tx_empty(struct uart_port *port)
{
	if(serial_in(port,ARM_UART_STS1)& 0xff00)
		return 0;
	else
		return 1;
}

EXPORT_SYMBOL(serialsc8800_tx_empty);
static unsigned int serialsc8800_get_mctrl(struct uart_port *port)
{
	return TIOCM_DSR | TIOCM_CTS;
}
static void serialsc8800_set_mctrl(struct uart_port *port,unsigned int mctrl)
{
}
static void serialsc8800_stop_tx(struct uart_port *port)
{
	unsigned long flags;
	unsigned int ien,iclr;
	spin_lock_irqsave(&port->lock,flags);
	iclr=serial_in(port,ARM_UART_ICLR);
	ien=serial_in(port,ARM_UART_IEN);

	iclr |=UART_IEN_TX_FIFO_EMPTY;
	ien &=~ UART_IEN_TX_FIFO_EMPTY;

	serial_out(port,ARM_UART_ICLR,iclr);
	serial_out(port,ARM_UART_IEN,ien);
	spin_unlock_irqrestore(&port->lock,flags);
}
static void serialsc8800_start_tx(struct uart_port *port)
{
	unsigned long flags;
	unsigned int ien;
	spin_lock_irqsave(&port->lock,flags);
	ien=serial_in(port,ARM_UART_IEN);
	if(!(ien & UART_IEN_TX_FIFO_EMPTY)){
		ien |= UART_IEN_TX_FIFO_EMPTY;
		serial_out(port,ARM_UART_IEN,ien);
	}
	spin_unlock_irqrestore(&port->lock,flags);
}	
static void serialsc8800_stop_rx(struct uart_port *port)
{
	unsigned long flags;
	unsigned int ien,iclr;	
	spin_lock_irqsave(&port->lock,flags);
	iclr=serial_in(port,ARM_UART_ICLR);
	ien=serial_in(port,ARM_UART_IEN);

	ien &=~(UART_IEN_RX_FIFO_FULL|UART_IEN_BREAK_DETECT);
	iclr |=UART_IEN_RX_FIFO_FULL|UART_IEN_BREAK_DETECT;

	serial_out(port,ARM_UART_IEN,ien);
	serial_out(port,ARM_UART_ICLR,iclr);
	spin_unlock_irqrestore(&port->lock,flags);
}
static void serialsc8800_enable_ms(struct uart_port *port)
{
}
static void serialsc8800_break_ctl(struct uart_port *port,int break_state)
{
}

#ifdef CONFIG_TS0710_MUX_SERIAL_UART
static inline int serialsc8800_cmuxs_opened_on_device(int device)
{
  int is_cmuxs_opened = 0;
  if(SERIALSC8800_IS_INVALID_LINE(device))
  {
    PRINT_DEBUG_SER("check_cmux_opened_on_line: invalid line %d\n",device);
	  return is_cmuxs_opened;
  }
  switch(device)
  {
    case 2:
      if(cmuxs_opened())
      {
        is_cmuxs_opened = 1;
      }
      break;
    default:
      /*uart 0,1 not support cmux*/
      break;
  }
  PRINT_DEBUG_SER("serialsc8800_cmuxs_opened_on_device: line %d enabled: %d\n", device, is_cmuxs_opened);
  return is_cmuxs_opened;
}
#endif

static char rev[2048];

#ifdef UART_DUMP_DATA_TO_FILE

#include <linux/fs.h>
#include <linux/uaccess.h>

#define TEST_BUF_SIZE		(1024*128)

static unsigned char trace_buffer[TEST_BUF_SIZE] = {0};
static unsigned long totalcount = 0;

extern unsigned char trace_buffer2[];
extern unsigned long totalcount2;

void savelog(void)
{
    struct file     *filp = NULL;
    struct file     *filp2 = NULL;
    
    int i =0;
	mm_segment_t oldfs;
	printk(KERN_ALERT "========================================\r\n");
	printk(KERN_ALERT "Received rx total count is: %d  \r\n",totalcount);	
	printk(KERN_ALERT "Received tx total count is: %d  \r\n",totalcount2);
	oldfs = get_fs();
	set_fs(KERNEL_DS);		
    filp = filp_open("/data/local/logger/uart_rx.log", O_RDWR|O_TRUNC|O_CREAT, 0);
    
    filp2 = filp_open("/data/local/logger/uart_tx.log", O_RDWR|O_TRUNC|O_CREAT, 0);
    
	if (IS_ERR(filp)) {
	    printk(KERN_ALERT "can not save as a file rx \r\n");	
        return;
	}  
	
	if (IS_ERR(filp2)) {
	    printk(KERN_ALERT "can not save as a file tx \r\n");	
        return;
	}  
	  
    //write file to fs rx
    for(i=0;i<128;i++)
    {
        msleep(1);
      	vfs_write(filp, &(trace_buffer[1024*i]), 1024, &(filp->f_pos) );
      	msleep(1);
    }    
    
    //write file to fs tx
    
    for(i=0;i<128;i++)
    {
        msleep(1);
      	vfs_write(filp2, &(trace_buffer2[1024*i]), 1024, &(filp2->f_pos) );
      	msleep(1);
    }  
    
   printk(KERN_ALERT "1  \r\n");
	filp_close(filp, NULL);
	filp_close(filp2, NULL);
	printk(KERN_ALERT "2  \r\n");
	set_fs(oldfs);
	printk(KERN_ALERT "Write log successfull  \r\n");
}
#endif

static inline void serialsc8800_rx_chars(int irq,void *dev_id)
{
	struct uart_port *port=(struct uart_port*)dev_id;
	struct tty_struct *tty=port->state->port.tty;
	unsigned int status,ch,flag,lsr,max_count=2048;
	unsigned int count,st=0;

	
#ifdef CONFIG_TS0710_MUX_SERIAL_UART
	if (serialsc8800_cmuxs_opened_on_device(port->line)){
	#ifdef UART_SW_FLOW_CONTROL
		/* If Mux mode is not enable, Do not parse data !*/
		if(0)//(!is_cmuxs_mode())
		{
			count =0; 	
            PRINT_ONTIM("cj not cmuxs \n");
            st = serial_in(port,ARM_UART_STS1);
			while ((st & 0xff) && (max_count-- > 0))
			{
				rev[count] = serial_in(port, ARM_UART_RXD);
				st = serial_in(port,ARM_UART_STS1);
				PRINT_ONTIM("cj rev%d=%c \n",count, rev[count]);
				count++;
			} 
		}
		else
		{				
			count =0; 	
			st = serial_in(port,ARM_UART_STS1);
			
			while ((st & 0xff) && (max_count-- > 0))
			{
                ch =  serial_in(port, ARM_UART_RXD);
                PRINT_ONTIM("cj read a char=%x s_step=%d STS1=%x \n",ch,s_step, st);
                switch(s_step)
				{
					case 0:
						if (HDLC_FLAG == ch)
							s_step = 1;
						else
						{
							printk("cj HDLC_FLAG: begin flag error=%x \n",ch);
						}
						break;
					
					case 1:						
						g_rec_frame_num_modem = ch;
						PRINT_ONTIM("cj frame_len = %d \n", g_rec_frame_num_modem);
						s_step=2;
						break;
					
					case 2:
						if (HDLC_FLAG == ch)
						{	
							if (g_rec_frame_num != g_rec_frame_num_modem)
							{
								printk("cj HDLC_FLAG: end flag error \n");
							}
							else
							{					
								st = serial_in(port,ARM_UART_STS1);
								if((st & 0xff))
								{
									PRINT_ONTIM("cj STS1=0x%04x,port=%d error: FIFO NOT EMPTY!\n",st,port);
								}
							
								{
								    s_step =0;
								    g_rec_frame_num	= 0;
								    g_rec_frame_num_modem = 0;
								    g_last_rec_escape = false;
								    
								    g_revert = true;
								}
							}
						}
						else
						{						    
							if(HDLC_ESCAPE == ch)  //0x7d
				              {
				              	PRINT_ONTIM("cj recv 0x7d \n");
				              	g_last_rec_escape = true;
				              }
				       		else
				       		{
					       	  if (g_last_rec_escape)
					          {
					             rev[count++] = ch^HDLC_ESCAPE_MASK;
					             #ifdef UART_DUMP_DATA_TO_FILE
					             if(totalcount < TEST_BUF_SIZE-1)
					                 trace_buffer[totalcount++] = ch^HDLC_ESCAPE_MASK;
					             #endif
					             g_last_rec_escape = false;
					             PRINT_ONTIM("cj recv 0x7e = %x \n", rev[(count-1)]);
					          }
					          else
					          {
					             rev[count++] = ch;
					             #ifdef UART_DUMP_DATA_TO_FILE
					             if(totalcount < TEST_BUF_SIZE-1)
					                trace_buffer[totalcount++] = ch;
					             #endif
					             PRINT_ONTIM("cj ch=%x \n",ch);
					          }
					          g_rec_frame_num++;
				       		}
						}
						break;
				}
				st = serial_in(port,ARM_UART_STS1);
	    }
		}
#endif
		if(count>0)
		{
		    mux_ringbuffer_write(&muxs_rbuf, rev, count);
		    if (serial_muxs_dispatcher && is_cmuxs_mode())
			    serial_muxs_dispatcher(tty);
#ifdef UART_SW_FLOW_CONTROL
			  if(g_revert)
			  {
			  	     mux_flow_ctrl_status = !mux_flow_ctrl_status;
					 gpio_set_value(sprd_3rdparty_gpio_mux_rx_ctrl, mux_flow_ctrl_status);//added by tao for mux uart flow ctrl
					 g_revert = false;
			  }
#endif
		}
		return;
	}
	else 
		{
#endif	
	status=serial_in(port,ARM_UART_STS1);
	lsr=serial_in(port,ARM_UART_STS0);
	while((status & 0x00ff) && max_count--){
		ch = serial_in(port,ARM_UART_RXD);
		flag = TTY_NORMAL;
		port->icount.rx++;

		if(unlikely(lsr&(UART_LSR_BI|UART_LSR_PE|UART_LSR_FE|UART_LSR_OE))){
			/*
 			*for statistics only
 			*/ 
			if(lsr & UART_LSR_BI){
				lsr &=~(UART_LSR_FE|UART_LSR_PE);
				port->icount.brk++;
				/*
 				*we do the SysRQ and SAK checking here because otherwise the
				*break may get masked by ignore_status_mask or read_status_mask
 				*/ 
				if(uart_handle_break(port))
					goto ignore_char;
			}else if(lsr & UART_LSR_PE)
				port->icount.parity++;
			else if(lsr & UART_LSR_FE)
				port->icount.frame++;
			if(lsr & UART_LSR_OE)
				port->icount.overrun++;
			/*
 			*mask off conditions which should be ignored
 			*/ 
			lsr &= port->read_status_mask;
			if(lsr & UART_LSR_BI)
				flag= TTY_BREAK;
			else if(lsr & UART_LSR_PE)
				flag= TTY_PARITY;
			else if(lsr & UART_LSR_FE)
				flag= TTY_FRAME;
		}
		if(uart_handle_sysrq_char(port,ch))
			goto ignore_char;

		uart_insert_char(port,lsr,UART_LSR_OE,ch,flag);
	ignore_char:
		status=serial_in(port,ARM_UART_STS1);
		lsr=serial_in(port,ARM_UART_STS0);
	}
	//tty->low_latency = 1;
	tty_flip_buffer_push(tty);
#ifdef CONFIG_TS0710_MUX_SERIAL_UART
	}
#endif
}
static inline void  serialsc8800_tx_chars(int irq,void *dev_id)
{
	struct uart_port *port=dev_id;
	struct circ_buf *xmit=&port->state->xmit;
	int count;

	if(port->x_char){
		serial_out(port,ARM_UART_TXD,port->x_char);
		port->icount.tx++;
		port->x_char=0;
		return;
	}
	if(uart_circ_empty(xmit) || uart_tx_stopped(port)){
		serialsc8800_stop_tx(port);		
		return;
	}

	count=SP_TX_FIFO;
	do{

		serial_out(port,ARM_UART_TXD,xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail+1) & (UART_XMIT_SIZE-1);
		port->icount.tx++;
		if(uart_circ_empty(xmit))
			break;
	}while(--count>0);
	
	if(uart_circ_chars_pending(xmit) < WAKEUP_CHARS){
		uart_write_wakeup(port);
	}
	if(uart_circ_empty(xmit)){
		serialsc8800_stop_tx(port);	
	}

#ifdef CONFIG_TS0710_MUX_SERIAL_UART
	if (serialsc8800_cmuxs_opened_on_device(port->line)){
		if(serial_muxs_sender){
			serial_muxs_sender();	
		}	
	}
#endif

}
/*
 *this handles the interrupt from one port
 */
static irqreturn_t serialsc8800_interrupt_chars(int irq,void *dev_id)
{
	struct uart_port *port=(struct uart_port *)dev_id;
	unsigned long flags;
	int pass_counter=0;
#ifdef UART_OVERRUN_DEBUG
    unsigned long uartsts0 = 0,uartsts1 = 0,uartsts2 = 0,uartctl1=0,uartctl2=0,uartctl0=0,uartclkd0=0,uartclkd1=0;
    unsigned long uartien = 0,uarticlr = 0,uartckd0=0,uartdspwait=0;
#endif

	spin_lock_irqsave(&port->lock,flags);
	do{
		if(!(serial_in(port,ARM_UART_STS0) & serial_in(port,ARM_UART_IEN))){
			break;
		}
#ifdef UART_OVERRUN_DEBUG
			if(port->line == 2)
            {
                uartsts0 = serial_in(port,ARM_UART_STS0);

                if (uartsts0 & (UART_STS_RX_FIFO_OVERFLOW | UART_STS_PARITY_ERR |UART_STS_FRAME_ERR))//
                {
                    uartsts1 = serial_in(port,ARM_UART_STS1);
                    uartsts2 = serial_in(port,ARM_UART_STS2);
                    uartctl0 = serial_in(port,ARM_UART_CTL0); 
                    uartctl1 = serial_in(port,ARM_UART_CTL1); 
                    uartctl2 = serial_in(port,ARM_UART_CTL2);
		      uartclkd0 = serial_in(port,ARM_UART_CLKD0);
		      uartclkd1 = serial_in(port,ARM_UART_CLKD1);
                    printk("UARTREG:uartsts0=0x%08x,uartsts1=0x%08x,uartsts2=0x%08x,uartctl0=0x%08x,uartctl1=0x%08x,uartctl2=0x%08x \r\n",
                            uartsts0,uartsts1,uartsts2,uartctl0,uartctl1,uartctl2);
		      printk("UARTREG:uartclkd0=0x%08x,uartclkd1=0x%08x \r\n", uartclkd0, uartclkd1);
                }  

            }
#endif

		if(serial_in(port,ARM_UART_STS0) & (UART_STS_RX_FIFO_FULL | UART_STS_BREAK_DETECT| UART_STS_TIMEOUT )){
		
			serialsc8800_rx_chars(irq,port);
		
		}
		if(serial_in(port,ARM_UART_STS0) & UART_STS_TX_FIFO_EMPTY){
			serialsc8800_tx_chars(irq,port);
		}
		serial_out(port,ARM_UART_ICLR,0xffffffff);
	}while(pass_counter++<50);
	spin_unlock_irqrestore(&port->lock,flags);
	return IRQ_HANDLED;
}

/*
 *this handles the interrupt from rx0 wakeup
 */
static irqreturn_t wakeup_rx_interrupt(int irq,void *dev_id)
{
//TODO 
#if 0
	// set and then clear BIT_15
	u32 val;
	val = __raw_readl(INT_UINT_CTL);	
	val |= BIT_15;
	__raw_writel(val, INT_UINT_CTL);
	val &= ~ BIT_15;
	__raw_writel(val, INT_UINT_CTL);

	// set wakeup symbol
	is_uart_rx_wakeup = true;
#endif
	return IRQ_HANDLED;
}

static void serialsc8800_pin_config(void)
{
#if 0
	unsigned long serial_func_cfg[] = {
	MFP_CFG_X(U0CTS, AF1, DS1, F_PULL_UP, S_PULL_UP, IO_IE),
		    MFP_CFG_X(U0RTS, AF1, DS1, F_PULL_NONE, S_PULL_NONE,
				      IO_Z),};

	sprd_mfp_config(serial_func_cfg, ARRAY_SIZE(serial_func_cfg));

	__raw_bits_or((1 << 22), SPRD_GREG_BASE + 0x8);
	__raw_bits_or((1 << 6), SPRD_GREG_BASE + 0x28);
	/* 
	   printk("UOTXD=0x%x\n",__raw_readl(SPRD_CPC_BASE+0x128));
	   printk("UORXD=0x%x\n",__raw_readl(SPRD_CPC_BASE+0x12c));
	   printk("UOCTS=0x%x\n",__raw_readl(SPRD_CPC_BASE+0x130));
	   printk("UORTS=0x%x\n",__raw_readl(SPRD_CPC_BASE+0x134));
	   printk("GEN0=0x%x\n",__raw_readl(SPRD_GREG_BASE+0x8));
	   printk("PIN_CTL=0x%x\n",__raw_readl(SPRD_GREG_BASE+0x28));
	 */
#else

	   printk("wjz_s UOTXD=0x%x\n",__raw_readl(SPRD_CPC_BASE+0x128));
	   printk("UORXD=0x%x\n",__raw_readl(SPRD_CPC_BASE+0x12c));
	   printk("UOCTS=0x%x\n",__raw_readl(SPRD_CPC_BASE+0x130));
	   printk("UORTS=0x%x\n",__raw_readl(SPRD_CPC_BASE+0x134));
	   printk("GEN0=0x%x\n",__raw_readl(SPRD_GREG_BASE+0x8));
	   printk("PIN_CTL=0x%x\n",__raw_readl(SPRD_GREG_BASE+0x28));

/*
	unsigned long serial_func_cfg[] = {
	MFP_CFG_X(U0CTS, AF1, DS1, F_PULL_UP, S_PULL_UP, IO_IE),
		    MFP_CFG_X(U0RTS, AF1, DS1, F_PULL_NONE, S_PULL_NONE,
				      IO_Z),};*/

//ADD BEGIN 
 unsigned int sel = 0 ; 
 sel = __raw_readl(SPRD_CPC_BASE+0x134); 
 sel =0x18A;
  __raw_writel(sel,SPRD_CPC_BASE+0x134); 
//ADD END 


	//sprd_mfp_config(serial_func_cfg, ARRAY_SIZE(serial_func_cfg));

	//__raw_bits_or((1 << 22), SPRD_GREG_BASE + 0x8);
	//__raw_bits_or((1 << 6), SPRD_GREG_BASE + 0x28);
	   printk("UOTXD=0x%x\n",__raw_readl(SPRD_CPC_BASE+0x128));
	   printk("UORXD=0x%x\n",__raw_readl(SPRD_CPC_BASE+0x12c));
	   printk("UOCTS=0x%x\n",__raw_readl(SPRD_CPC_BASE+0x130));
	   printk("UORTS=0x%x\n",__raw_readl(SPRD_CPC_BASE+0x134));
	   printk("GEN0=0x%x\n",__raw_readl(SPRD_GREG_BASE+0x8));
	   printk("PIN_CTL=0x%x\n",__raw_readl(SPRD_GREG_BASE+0x28));
	 





	 //__raw_bits_or((1 << 22 | 1 << 21 | 1 << 20), SPRD_GREG_BASE + 0x8);

#endif				//steve.zhan
} 
static int serialsc8800_startup(struct uart_port *port) 
{
	int ret=0;
	unsigned int ien,ctrl1;
	
  if(port == NULL)
  {
    printk("=======serialsc8800_startup null device port");
  	return -EBADF;
  }
	//port->uartclk=26000000;
#ifdef CONFIG_TS0710_MUX_SERIAL_UART
    if(SERIALSC8800_IS_INVALID_LINE(port->line))
    {
      printk("serialsc8800_startup: invalid line %d",port->line);
	  return -EBADF;
    }

  	if(port->line == g_support_mux_uart_device)
    {
        printk("=======serialsc8800_startup line:%d, mux_guard:%d\n",port->line,serial_mux_guard);
        if( serial_mux_guard > 0) {
            serial_mux_guard ++;
			printk("Fail to open ttyS%d, it's busy!\n",port->line);
			return -EBUSY;
		} else {
			serial_mux_guard ++;
			serial_for_muxs_tty = port->state->port.tty;
			printk("=========serial_for_muxs_tty=%p========\r\n",serial_for_muxs_tty);
		}
	}
#endif
  //if(port->line == 2)
	{
        serialsc8800_pin_config(); //fixme don't know who change u0cts pin in 88
	}
	/*
 	*set fifo water mark,tx_int_mark=8,rx_int_mark=1
 	*/
	//serial_out(port,ARM_UART_CTL2,0x801);
#ifdef UART_SW_FLOW_CONTROL
	if(port->line == 2)
	{
	    serial_out(port,ARM_UART_CTL2,((SP_TX_FIFO<<8)|SP_RX_FIFO_UART2));
	}
	else
#endif
	{	
	    serial_out(port,ARM_UART_CTL2,((SP_TX_FIFO<<8)|SP_RX_FIFO));
	}
	/*
 	*clear rx fifo
 	*/ 
	while(serial_in(port,ARM_UART_STS1) & 0x00ff){
		serial_in(port,ARM_UART_RXD);
	}
	/*
 	*clear tx fifo
 	*/ 
	while(serial_in(port,ARM_UART_STS1) & 0xff00);
	/*
 	*clear interrupt
 	*/
	serial_out(port,ARM_UART_IEN,0x00); 
	serial_out(port,ARM_UART_ICLR,0xffffffff); 
	/*
 	*allocate irq
 	*/ 
#ifdef CONFIG_ARCH_SC8810
	ret = request_irq(port->irq,serialsc8800_interrupt_chars,IRQF_DISABLED,"serial",port);
	{
		int ret2 = 0;

		if (!port->line) {
			ret2 = request_irq(IRQ_WAKEUP,wakeup_rx_interrupt,IRQF_SHARED,"wakeup_rx",port);
			if(ret2)
			{
		 		printk("fail to request wakeup irq\n");
				free_irq(IRQ_WAKEUP,NULL);
			}
		}
	}
#endif
#ifdef CONFIG_ARCH_SC8800S
	ret = request_irq(port->irq,serialsc8800_interrupt_chars,IRQF_SHARED,"serial",port);
#endif	
	if(ret)
	{
 		printk("fail to request serial irq\n");
		free_irq(port->irq,port);
	}

    ctrl1=serial_in(port,ARM_UART_CTL1);
#ifdef  UART_SW_FLOW_CONTROL
    if(port->line == 2)
    {
       ctrl1 |= 0x3e00|SP_RX_FIFO_UART2;
    }
    else
#endif
    {
       ctrl1 |= 0x3e00|SP_RX_FIFO;
    }
	serial_out(port,ARM_UART_CTL1,ctrl1);	

	/*
 	*enable interrupt
 	*/ 
	ien=serial_in(port,ARM_UART_IEN);
	ien |= UART_IEN_RX_FIFO_FULL | UART_IEN_TX_FIFO_EMPTY | UART_IEN_BREAK_DETECT| UART_IEN_TIMEOUT;
	serial_out(port,ARM_UART_IEN,ien);	
	if(0 == port->line)
	{
	    printk("===Bt_Uart0_Opened 1 \n");
		Bt_Uart0_Opened = 1;
	}
	return 0;
}
static void serialsc8800_shutdown(struct uart_port *port)
{
  if(port == NULL)
  {
    printk("=======serialsc8800_shutdown null device port\n");
  	return;
  }
  if(SERIALSC8800_IS_INVALID_LINE(port->line))
  {
  	printk("=======serialsc8800_shutdown invalid line:%d\n",port->line);
	return;
  }
#ifdef CONFIG_TS0710_MUX_SERIAL_UART
  if (port->line == g_support_mux_uart_device)
  {
  	printk("=======serialsc8800_shutdown line:%d, mux_guard:%d\n",port->line,serial_mux_guard);
	if(serial_mux_guard > 0)
    {
      serial_mux_guard --;
      /* SPRD anjun added to pretect rild control uart write panic*/
      if(serial_mux_guard == 0)
      {
        printk("%s:serial_for_muxs_tty = NULL !!!\n",__func__);
        serial_for_muxs_tty = NULL;
      }
      /* SPRD anjun added to pretect rild control uart write panic*/
    }
  }
#endif

	serial_out(port,ARM_UART_IEN,0x0);
	serial_out(port,ARM_UART_ICLR,0xffffffff);
	free_irq(port->irq,port);

	if (!port->line) {
		free_irq(IRQ_WAKEUP,port);
	}
	if(0 == port->line)
	{
	    printk("===Bt_Uart0_Opened 0 \n");
		Bt_Uart0_Opened = 0;
	}
	
	
}
static void serialsc8800_set_termios(struct uart_port *port,struct ktermios *termios,struct ktermios *old)
{	

	unsigned int baud,quot;
	unsigned int lcr,fc;
	unsigned long flags;
	//printascii("enter set_termios\r\n");
	/*
 	*ask the core to calculate the divisor for us
 	*/
 	//cj test code
 	static  int reference = 0;
	
    if(port == NULL)
    {
      printk("set_termios invalid port(null) \r\n");
      return ;
    }
	 if(port->line == 2)
	 {
		if(!reference)
			reference++;
		else
			printk("error: who do it!");
	 }
	baud = uart_get_baud_rate(port,termios,old,1200,3000000);
	printk("baud=%d\r\n",baud);
	//quot = uart_get_divisor(port,baud);
	switch (baud){
	case 1200:
		quot=BAUD_1200_26M;
		break;
	case 2400:
		quot=BAUD_2400_26M;
		break;
	case 4800:
		quot=BAUD_4800_26M;
		break;
	case 9600:
		quot=BAUD_9600_26M;
		break;
	case 19200:
		quot=BAUD_19200_26M;
		break;
	case 38400:
		quot=BAUD_38400_26M;
		break;
	case 57600:
		quot=BAUD_57600_26M;
		break;
	case 230400:
		quot=BAUD_230400_26M;
		break;
	case 460800:
		quot=BAUD_460800_26M;
		break;
	case 921600:
		quot=BAUD_921600_26M;
		break;
    case 1000000:
        quot=BAUD_1000000_26M;
        break;
    case 1152000:
        quot=BAUD_1152000_26M;
        break;
    case 1500000:
        quot=BAUD_1500000_26M;
        break;
    case 2000000:
        quot=BAUD_2000000_26M;
        break;
    case 2500000:
        quot=BAUD_2500000_26M;
        break;
    case 3000000:
/*SPRD FAE anjun added for Uart0 clk source change to 96M --start */    	
#if 1
        quot=BAUD_3000000_26M;
#else
        quot=BAUD_3000000_96M;
#endif        
        break;
	default:
	case 115200:
		quot=BAUD_115200_26M;
		break;
	}
	//printk("quot=0x%x\r\n",quot);
	if(quot==BAUD_3000000_96M)
	{
		__raw_bits_and(~(0X3<<((10+port->line)*2)),SPRD_GREG_BASE+0x005c);
		quot= BAUD_3000000_96M&0xffff;
	}	
	/*
 	*set data length
 	*/
	lcr = serial_in(port,ARM_UART_CTL0);
	lcr &=~UART_DATA_BIT;
	switch (termios->c_cflag & CSIZE){
	case CS5:
		lcr |=UART_DATA_5BIT;
		break;
	case CS6:
		lcr |=UART_DATA_6BIT;
		break;
	case CS7:
		lcr |=UART_DATA_7BIT;
		break;
	default:
	case CS8:
		lcr |=UART_DATA_8BIT;
		break;
	}
	/*
 	*calculate stop bits
 	*/ 
	lcr &=~(UART_STOP_1BIT | UART_STOP_2BIT);
	if(termios->c_cflag & CSTOPB)
		lcr |= UART_STOP_2BIT;
	else 
		lcr |= UART_STOP_1BIT;

	//cj test code begin
	if(port->line == 2)
	{
	   lcr &=~(UART_STOP_1BIT | UART_STOP_2BIT);
	   lcr |= UART_STOP_2BIT;
          printk("2 stop bit lcr = 0x%x \r\n", lcr);
	}
	//cj test code end
	/*
 	*calculate parity
 	*/
	lcr &=~UART_PARITY;
	if(termios->c_cflag & PARENB){
		lcr |= UART_PARITY_EN;
		if(termios->c_cflag & PARODD)
			lcr |= UART_ODD_PAR;
		else 
			lcr |= UART_EVEN_PAR;
	}
	//printk("lcr=0x%x\r\n",lcr);
	/*
 	*change the port state.
 	*/ 
	spin_lock_irqsave(&port->lock,flags);
	/*
 	*update the per-port timeout
 	*/ 
	uart_update_timeout(port,termios->c_cflag,baud);
	
	port->read_status_mask = UART_LSR_OE;// | UART_LSR_DR;
	if(termios->c_iflag & INPCK)
		port->read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if(termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= UART_LSR_BI;
	//printk("read_status_mask=0x%x\r\n",port->read_status_mask);
	/*
 	*characters to ignore
 	*/ 
	port->ignore_status_mask=0;
	if(termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if(termios->c_iflag & IGNBRK){
		port->ignore_status_mask |= UART_LSR_BI;
		/*
 		*if we ignore parity and break indicators,ignore overruns too
 		*/ 
		if(termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= UART_LSR_OE;
	}
	/*
 	*ignore all characters if CREAD is not set
 	*/ 	
	//if((termios->c_cflag & CREAD)== 0)
		//port->ignore_status_mask |= UART_LSR_DR;
	/*
 	*flow control
 	*/
	fc=serial_in(port,ARM_UART_CTL1);
	fc &=~(RX_HW_FLOW_CTL_THRESHOLD|RX_HW_FLOW_CTL_EN|TX_HW_FLOW_CTL_EN);
	if(termios->c_cflag & CRTSCTS){
		fc |= RX_HW_FLOW_CTL_THRESHOLD;
		fc |= RX_HW_FLOW_CTL_EN;
		fc |= TX_HW_FLOW_CTL_EN;
	}
	serial_out(port,ARM_UART_CLKD0,quot&0xffff);//clock divider bit0~bit15	 
	serial_out(port,ARM_UART_CLKD1,(quot&0x1f0000)>>16);	 //clock divider bit16~bit20
	//cj test code begin
	if(port->line == 2)
	{
		printk("CLKD0 = 0x%x\r\n", (quot&0xffff) );
		printk("CLKD1 = 0x%x\r\n", ((quot&0x1f0000)>>16));
	}
	//test end
	
	serial_out(port,ARM_UART_CTL0,lcr);
#ifdef UART_SW_FLOW_CONTROL
	if(port->line == 2)
	{
	   fc |= 0x3e00|SP_RX_FIFO_UART2;
	}
	else
#endif
	{
       fc |= 0x3e00|SP_RX_FIFO;
    }
	serial_out(port,ARM_UART_CTL1,fc);
		 
	spin_unlock_irqrestore(&port->lock,flags);
	printk("quot=0x%x\r\n",quot);

}
static const char *serialsc8800_type(struct uart_port *port)
{
	return "SPX";
}     
static void serialsc8800_release_port(struct uart_port *port)
{
}
static int serialsc8800_request_port(struct uart_port *port)
{
	return 0;
}
static void serialsc8800_config_port(struct uart_port *port,int flags)
{
	if(flags & UART_CONFIG_TYPE && serialsc8800_request_port(port)==0)
		port->type =PORT_SPX;
}
static int serialsc8800_verify_port(struct uart_port *port,struct serial_struct *ser)
{
	if(unlikely(ser->type != PORT_SPX))
		return -EINVAL;
	if(unlikely(port->irq !=ser->irq))
		return -EINVAL;
	return 0;
}
static struct uart_ops serialsc8800_ops = {
	.tx_empty =serialsc8800_tx_empty,
	.get_mctrl =serialsc8800_get_mctrl,
	.set_mctrl =serialsc8800_set_mctrl,
	.stop_tx =serialsc8800_stop_tx,
	.start_tx =serialsc8800_start_tx,
	.stop_rx =serialsc8800_stop_rx,
	.enable_ms =serialsc8800_enable_ms,
	.break_ctl =serialsc8800_break_ctl,
	.startup =serialsc8800_startup,
	.shutdown =serialsc8800_shutdown,
	.set_termios =serialsc8800_set_termios,
	.type =serialsc8800_type,
	.release_port =serialsc8800_release_port,
	.request_port =serialsc8800_request_port,
	.config_port=serialsc8800_config_port,
	.verify_port =serialsc8800_verify_port,
};
#ifdef CONFIG_ARCH_SC8810
static struct uart_port serialsc8800_ports[UART_NR] = {
	[0]={
		.iotype =SERIAL_IO_PORT,
		.membase =(void *)SPRD_SERIAL0_BASE,
		.mapbase = SPRD_SERIAL0_BASE,
		.uartclk =UART_CLK,
		.irq =IRQ_UART0,
		.fifosize =128,
		.ops =&serialsc8800_ops,
		.flags =ASYNC_BOOT_AUTOCONF,
		.line =0,
	},
	[1]={
		.iotype =SERIAL_IO_PORT,
		.membase =(void *)SPRD_SERIAL1_BASE,
		.mapbase = SPRD_SERIAL1_BASE,
		.uartclk =UART_CLK,
		.irq =IRQ_UART1,
		.fifosize =128,
		.ops =&serialsc8800_ops,
		.flags =ASYNC_BOOT_AUTOCONF,
		.line =1,
	},
	[2]={
		.iotype =SERIAL_IO_PORT,
		.membase =(void *)SPRD_SERIAL2_BASE,
		.mapbase = SPRD_SERIAL2_BASE,
		.uartclk =UART_CLK,
		.irq =IRQ_UART2,
		.fifosize =128,
		.ops =&serialsc8800_ops,
		.flags =ASYNC_BOOT_AUTOCONF,
		.line =2,
	}

};
#else //CONFIG_ARCH_SC8810 
#ifdef CONFIG_ARCH_SC8800S
static struct uart_port serialsc8800_ports[UART_NR] = {
	[0]={
		.iotype =SERIAL_IO_PORT,
		.membase =(void *)SPRD_SERIAL0_BASE,
		.mapbase = SPRD_SERIAL0_BASE,
		.uartclk =UART_CLK,
		.irq =IRQ_UART0_1,
		.fifosize =128,
		.ops =&serialsc8800_ops,
		.flags =ASYNC_BOOT_AUTOCONF,
		.line =0,
	},
	[1]={
		.iotype =SERIAL_IO_PORT,
		.membase =(void *)SPRD_SERIAL1_BASE,
		.mapbase = SPRD_SERIAL1_BASE,
		.uartclk =UART_CLK,
		.irq =IRQ_UART0_1,
		.fifosize =128,
		.ops =&serialsc8800_ops,
		.flags =ASYNC_BOOT_AUTOCONF,
		.line =1,
	},
	[2]={
		.iotype =SERIAL_IO_PORT,
		.membase =(void *)SPRD_SERIAL2_BASE,
		.mapbase = SPRD_SERIAL2_BASE,
		.uartclk =UART_CLK,
		.irq =IRQ_UART2_3,
		.fifosize =128,
		.ops =&serialsc8800_ops,
		.flags =ASYNC_BOOT_AUTOCONF,
		.line =2,
	},
	[3]={
		.iotype =SERIAL_IO_PORT,
		.membase =(void *)SPRD_SERIAL3_BASE,
		.mapbase = SPRD_SERIAL3_BASE,
		.uartclk =UART_CLK,
		.irq =IRQ_UART2_3,
		.fifosize =128,
		.ops =&serialsc8800_ops,
		.flags =ASYNC_BOOT_AUTOCONF,
		.line =3,
	}


};
#else //CONFIG_ARCH_SC8800S
#error "not define CONFIG_ARCH_SC8800S or CONFIG_ARCH_SC8810"
#endif //CONFIG_ARCH_SC8800S
#endif //CONFIG_ARCH_SC8810

static void serialsc8800_setup_ports(void)
{
	unsigned int i;
	
	for(i=0;i<UART_NR;i++)		
		serialsc8800_ports[i].uartclk=UART_CLK;
}
struct clk *serial_clk[UART_NR];
static int clk_startup(void)
{
	unsigned int i;
	struct clk *clk;
	struct clk *clk_parent;
	char clk_name[10];
	int ret;
    unsigned int div;
	
	for(i=0;i<UART_NR;i++){
		sprintf(clk_name,"clk_uart%d",i);
		clk = clk_get(NULL, clk_name);
		if (IS_ERR(clk)) {
			printk("clock[%s]: failed to get clock by clk_get()!\n",
					clk_name);
			continue;
		}
		clk_parent = clk_get(NULL, "ext_26m");		
		if (IS_ERR(clk_parent)) {
			printk("clock[%s]: failed to get parent [%s] by clk_get()!\n",
					clk_name, "clk_26m");
			continue;
		}		
		ret= clk_set_parent(clk, clk_parent);
		if (ret) {
			printk("clock[%s]: clk_set_parent() failed!\n", clk_name);
			continue;
		}
		div=1;
		ret = clk_set_divisor(clk, div);
		if (ret) {
			printk("clock[%s]: clk_set_divisor() failed!\n", clk_name);
			continue;
		}
		ret = clk_enable(clk);
		if (ret) {
			printk("clock[%s]: clk_enable() failed!\n", clk_name);
			continue;
		}
        serial_clk[i]= clk;
	}
	return 0;
}
static int clk_shutdown(void)
{
	unsigned int i;
	
    for(i=0;i<UART_NR;i++){
		clk_disable(serial_clk[i]);
		clk_put(serial_clk[i]);
	}
	return 0;
}
#ifdef CONFIG_SERIAL_SPRD_CONSOLE
static inline void wait_for_xmitr(struct uart_port *port)
{
	unsigned int status,tmout=10000;
	//printascii("enter wait_for_xmitr!!\r\n");
	/*
 	* wait up to 10ms for the character(s) to be sent
 	*/ 
	do{
		status=serial_in(port,ARM_UART_STS1);
		if(--tmout == 0)
			break;
		udelay(1);
	}while(status & 0xff00);
	//}while((status & UART_STS_TX_FIFO_EMPTY)!=UART_STS_TX_FIFO_EMPTY);
	//printascii("leave wait_for_xmitr!!\r\n");
}
static void serialsc8800_console_putchar(struct uart_port *port,int ch)
{
	//printascii("enter serialsc8800_console_putchar!\r\n");
	wait_for_xmitr(port);
	serial_out(port,ARM_UART_TXD,ch);
	//printascii("leave serialsc8800_console_putchar!\r\n");
}
static void serialsc8800_console_write(struct console *co,const char *s,unsigned int count)
{
	struct uart_port *port=&serialsc8800_ports[co->index];
	int ien;
	printk("%s ,port line=%d \r\n",__func__,port->line);
	/*firstly,save the IEN register and disable the interrupts*/
	ien=serial_in(port,ARM_UART_IEN);
	serial_out(port,ARM_UART_IEN,0x0);
	
	uart_console_write(port,s,count,serialsc8800_console_putchar);
	/*finally,wait for  TXD FIFO to become empty and restore the IEN register*/
	wait_for_xmitr(port);
	serial_out(port,ARM_UART_IEN,ien);
	//printascii("leave serialsc8800_console_write\r\n");
}
static int __init serialsc8800_console_setup(struct console *co,char *options)
{
	struct uart_port *port;
	int baud =115200;
	int bits =8;
	int parity ='n';
	int flow ='n';
	//int i;
	//printascii("enter console_setup!\r\n");
	if(unlikely(co->index >= UART_NR || co->index < 0))
		co->index = 0;	
	//for(i=0;i<4;i++){
		port= &serialsc8800_ports[co->index];	
		if(options)
			uart_parse_options(options,&baud,&parity,&bits,&flow);
		//uart_set_options(port,co,baud,parity,bits,flow);
	//}
	return uart_set_options(port,co,baud,parity,bits,flow);
	//return uart_set_options(port,co,baud,parity,bits,flow);
}
static struct uart_driver serialsc8800_reg;
static struct console serialsc8800_console = {
	.name ="ttyS",
	.write =serialsc8800_console_write,
	.device =uart_console_device,
	.setup =serialsc8800_console_setup,
	.flags =CON_PRINTBUFFER,
	.index =-1,
	.data =&serialsc8800_reg,
};
static int __init serialsc8800_console_init(void)
{
	serialsc8800_setup_ports();
	register_console(&serialsc8800_console);
	return 0;
}
console_initcall(serialsc8800_console_init);
#define SC8800_CONSOLE		&serialsc8800_console
#else
#define SC8800_CONSOLE		NULL
#endif

static struct uart_driver serialsc8800_reg = {
        .owner = THIS_MODULE,
        .driver_name = "serial_sc8800",
        .dev_name = SP_TTY_NAME,
        .major = SP_TTY_MAJOR,
        .minor = SP_TTY_MINOR_START,
        .nr = UART_NR,
        .cons = SC8800_CONSOLE,
};

static int serialsc8800_probe(struct platform_device *dev)
{
	int ret,i;
	
    ret = clk_startup();
	if (ret) {
		printk("func[%s]: serial set clock failed!\n", __FUNCTION__);
		return ret;
	}	
	serialsc8800_setup_ports();	
	ret = uart_register_driver(&serialsc8800_reg);
	if(ret ==0)
		printk("serialsc8800_init:enter uart_add_one_port\n");
#ifdef CONFIG_TS0710_MUX_SERIAL_UART
	serial_for_muxs_driver = serialsc8800_reg.tty_driver;
	printk("=========serial_for_mux_driver=%p========\r\n",serial_for_muxs_driver);
	//serial_for_muxs_tty = &serial_tty;
    serial_mux_guard = 0;
#endif

	for(i=0;i<UART_NR;i++)
		uart_add_one_port(&serialsc8800_reg,&serialsc8800_ports[i]);
	return 0;
}
static int serialsc8800_remove(struct platform_device *dev)
{
	int i,ret;
	
    for(i=0;i<UART_NR;i++)
		uart_remove_one_port(&serialsc8800_reg,&serialsc8800_ports[i]);
	uart_unregister_driver(&serialsc8800_reg);
    ret = clk_shutdown();
	if (ret) {
		printk("func[%s]: serial shutdown clock failed!\n", __FUNCTION__);
		return ret;
	}
    return 0;
}
static int serialsc8800_suspend(struct platform_device *dev, pm_message_t state)
{
	is_uart_rx_wakeup = false;		
	unsigned long fc = 0;
	struct uart_port *port = &(serialsc8800_ports[0]);
	unsigned long u0rts_cfg[]= {
	//MFP_CFG_X(U0RTS, AF2, DS1, F_PULL_UP, S_PULL_UP, IO_OE),
		MFP_CFG_X(U0RTS,	  AF3,	DS3,	F_PULL_UP,	S_PULL_UP,  IO_OE),	//   NC
		
	};
#ifdef CONFIG_BT_AUTO_SLEEP

		//clk_disable(serial_clk[0]);
		//clk_put(serial_clk[0]);

//	if(serial_in(port,ARM_UART_IEN)& 0xff00)
//	serial_in(port,ARM_UART_IEN,0x0);
	if(1 == Bt_Uart0_Opened )
	{	
	    printk("if(1 == Bt_Uart0_Opened ) s \n");
		fc=serial_in(port,ARM_UART_CTL1);
		fc &=~(RX_HW_FLOW_CTL_EN|TX_HW_FLOW_CTL_EN);
		serial_out(port,ARM_UART_CTL1,fc);
		sprd_mfp_config(u0rts_cfg, ARRAY_SIZE(u0rts_cfg));
	}
#endif 
	return 0;
}

static int serialsc8800_resume(struct platform_device *dev)
{
	unsigned long fc = 0;
	struct uart_port *port = &(serialsc8800_ports[0]);
	unsigned long u0rts_cfg[]= {
	//MFP_CFG_X(U0RTS, AF0, DS1, F_PULL_DOWN, S_PULL_UP, IO_OE),
		MFP_CFG_X(U0RTS,	  AF0,	DS1,	F_PULL_NONE,	S_PULL_NONE,	IO_Z),	//   NC 
	};

#ifdef CONFIG_BT_AUTO_SLEEP
	if(1 == Bt_Uart0_Opened )
	{
	    printk("if(1 == Bt_Uart0_Opened ) res \n");
		sprd_mfp_config(u0rts_cfg, ARRAY_SIZE(u0rts_cfg));
		fc=serial_in(port,ARM_UART_CTL1);
		fc |=(RX_HW_FLOW_CTL_EN|TX_HW_FLOW_CTL_EN);
		serial_out(port,ARM_UART_CTL1,fc);
		
		wake_lock_timeout(&uart_rx_lock, HZ / 5); // 0.2s
	}	
#else

    if(is_uart_rx_wakeup)
    {
        is_uart_rx_wakeup = false;
        wake_lock_timeout(&uart_rx_lock, HZ / 5);	// 0.2s
    }
#endif 	
    return 0;
}

static struct platform_driver serialsc8800_driver = {
	.probe = serialsc8800_probe,
	.remove = serialsc8800_remove,
	.suspend = serialsc8800_suspend,
	.resume = serialsc8800_resume,
	.driver = {
		.name = "serial_sp",
		.owner = THIS_MODULE,
	},
};
static int __init serialsc8800_init(void)
{
	printk(KERN_INFO"Serial:sc8800s driver $Revision:1.0 $\n");
	wake_lock_init(&uart_rx_lock, WAKE_LOCK_SUSPEND, "uart_rx_lock"); 	
	return platform_driver_register(&serialsc8800_driver);
}
static void __exit serialsc8800_exit(void)
{
	platform_driver_unregister(&serialsc8800_driver);
}
module_init(serialsc8800_init);
module_exit(serialsc8800_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("sp serial driver $Revision:1.0$");
