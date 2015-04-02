/*
 * mx-sensor-hub.c - mfd core driver for the Maxim 8998
 *
 * Copyright (C) 2013 Meizu Technology Co.Ltd, Zhuhai, China
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
	 
	 
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/i2c-algo-bit.h>
#include <linux/i2c-gpio.h>
#include <plat/gpio-cfg.h>
#include <mach/gpio-meizu.h>
#include <asm/mach-types.h>
#include <linux/firmware.h>
#include <linux/mfd/mx-hub.h>

	 
#define MX_HUB_FW 		"mx/mx3_sensor_hub.bin"
	 
#define 	RESET_BOOTL	5
#define 	RESET_COLD	1
#define	RESET_SOFT	0

//#define	INIT_RESET

//#define	TEST_REST_MODE

#ifdef	TEST_REST_MODE
struct mx_hub_test_data {
	struct delayed_work test_reset_work;
	bool test_reset_enable;
	int test_cycles ;
	int test_failed;
};
struct mx_hub_test_data test;
#endif


static struct mfd_cell mx_hub_devs[] = {
#ifdef	CONFIG_LEDS_MXHUB
	{ .name = "mx-hub-led", },
#endif	
#ifdef	CONFIG_MX_TPI
	{ .name = "mx_tpi", },
#endif	
#ifdef	CONFIG_MXHUB_ACC	
	{ .name = "mx-hub-acc", },
#endif	
#ifdef	CONFIG_MXHUB_GYR
	{ .name = "mx-hub-gyr", },
#endif	
#ifdef	CONFIG_MXHUB_COMPASS
	{ .name = "mx-hub-compass", },
#endif	
};

struct mx_hub_reg_data {
	unsigned char addr;
	unsigned char value;
};

//initial registers
const struct mx_hub_reg_data init_regs[] = {
	{MX_HUB_REG_LED_FUNC_EN, 0xC3},// LED_FUNC_FADE_EN LED_FUNC_SLOPE_EN LED_FUNC_BRN_FADE_OUT LED_FUNC_BRN_FADE_IN
//	{MX_HUB_REG_LED_SLPTOP, 150},// 100
//	{MX_HUB_REG0, 0x00},
};

#define	MAX_IIC_ERROR	5
static volatile int force_update = false;
static volatile int iic_error = 0;

static bool mx_hub_identify(struct mx_hub_dev *hub);
static void mx_hub_reinit(struct mx_hub_dev *hub);
static void mx_hub_init_registers(struct mx_hub_dev *hub);
static int mx_hub_get_imgfw_info(struct mx_hub_dev *hub);
static int mx_hub_update(struct mx_hub_dev *hub);
static void mx_hub_reset(struct mx_hub_dev *hub,int m);
static void mx_hub_wakeup(struct mx_hub_dev *hub,int bEn);
static int mx_hub_is_busy(struct mx_hub_dev *hub);


static int mx_hub_read(struct i2c_client *client,
				  int bytes, void *dest)
{
	struct i2c_msg xfer;
	int ret;

	xfer.addr =  client->addr;
	xfer.flags = I2C_M_RD;
	xfer.len = bytes;
	xfer.buf = (char *)dest;

	ret = i2c_transfer(client->adapter, &xfer, 1);

	if (ret < 0)
		goto err_read;
	if (ret != 1) {
		ret = -EIO;
		goto err_read;
	}

	iic_error = 0;
	return 0;

err_read:
	iic_error ++;
	return ret;
}

int mx_hub_read_addr(struct i2c_client *client,unsigned char addr,
				  int bytes, void *dest)
{
	struct i2c_msg xfer[2];
	int ret;

	xfer[0].addr =  client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 1;
	xfer[0].buf = &addr;

	xfer[1].addr =  client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = bytes;
	xfer[1].buf = (char *)dest;

	ret = i2c_transfer(client->adapter, xfer, 2);

	if (ret < 0)
		goto err_read;

	iic_error = 0;
	return 0;

err_read:
	iic_error ++;
	return ret;
}

static int mx_hub_write(struct i2c_client *client,
				int bytes, const void *src)
{
	struct i2c_msg xfer;
	int ret;

	xfer.addr = client->addr;
	xfer.flags = 0;
	xfer.len = bytes;
	xfer.buf = (char *)src;

	ret = i2c_transfer(client->adapter, &xfer, 1);
	if (ret < 0)
		goto err_write;
	if (ret != 1) {
		ret = -EIO;
		goto err_write;
	}

	iic_error = 0;
	return 0;

err_write:
	iic_error ++;
	return ret;
}

int mx_hub_write_addr(struct i2c_client *client, u8 reg, int bytes, const void *src)
{
	int ret;	
	unsigned char buf[256];
	int size;

	if(bytes > 255 )
		bytes = 255;

	buf[0] = reg;
	memcpy(&buf[1],src,bytes);

	size = bytes + 1;
	ret = mx_hub_write(client,size,buf);
	if (ret < 0)
		dev_err(&client->dev,
			"can not write register, returned %d\n", ret);

	return ret;
}

////////////////////////////////////////////////////////////////////////

int mx_hub_readbyte(struct i2c_client *client, u8 reg)
{
	struct mx_hub_dev *hub = i2c_get_clientdata(client);
	int ret = 0;
	unsigned char data;
	
	if( hub->is_update )
		return -EBUSY;
	
	mutex_lock(&hub->iolock);
#if 0
	ret = mx_hub_write(client,1,&reg);
	if (ret < 0)
		dev_err(&client->dev,
			"can not write register, returned %d at line %d\n", ret,__LINE__);
	
	ret = mx_hub_read(client, 1,&reg);
	if (ret < 0)
		dev_err(&client->dev,
			"can not read register, returned %d at line %d\n", ret,__LINE__);
	else
		ret = reg;
#else
	
	ret = mx_hub_read_addr(client, reg,1,&data);
	if (ret < 0)
		dev_err(&client->dev,
			"can not read register, returned %d at line %d\n", ret,__LINE__);
	else
		ret = data;
#endif
	

	mutex_unlock(&hub->iolock);

	return ret;
}

int mx_hub_writebyte(struct i2c_client *client, u8 reg, u8 data)
{
	struct mx_hub_dev *hub = i2c_get_clientdata(client);
	int ret = 0;	
	unsigned char buf[2];

	if( hub->is_update )
		return -EBUSY;

	mutex_lock(&hub->iolock);

	buf[0] = reg;
	buf[1] = data;
	ret = mx_hub_write(client,2,buf);
	if (ret < 0)
		dev_err(&client->dev,
			"can not write register, returned %d at line %d\n", ret,__LINE__);

	mutex_unlock(&hub->iolock);

	return ret;
}

int mx_hub_readdata_rdy(struct i2c_client *client, u8 reg,int bytes,void *dest)
{
	struct mx_hub_dev *hub = i2c_get_clientdata(client);
	int ret;	
	u8 data[2];
	u8 * buf;

	//if( hub->hub_sleep)
	//	return 0;

	if( hub->is_update )
		return -EBUSY;

	mutex_lock(&hub->iolock);

	buf = (u8 *)dest;
	if( reg == 0xff )
		reg = buf[0];	
	
	data[0] = reg;
	data[1] = bytes;
		
	ret = mx_hub_write_addr(client,MX_HUB_REG_READ_DATA_RDY,2,data);

	if( mx_hub_waitforidle(hub,10) < 0)
		dev_err(&client->dev,
			"wait idle timeout.\n");		
	
	ret = mx_hub_read_addr(client, reg,bytes,dest);
	if (ret < 0)
		dev_err(&client->dev,
			"can not read register, returned %d\n", ret);
	
	mutex_unlock(&hub->iolock);
	
	if (ret < 0)
		mx_hub_reinit(hub);

	return ret;
}


int mx_hub_readdata(struct i2c_client *client, u8 reg,int bytes,void *dest)
{
	struct mx_hub_dev *hub = i2c_get_clientdata(client);
	int ret;	

	if( hub->is_update )
		return -EBUSY;

	mutex_lock(&hub->iolock);
#if 0
	ret = mx_hub_write(client,1,&reg);
	if (ret < 0)
		dev_err(&client->dev,
			"can not read register, returned %d at line %d\n", ret,__LINE__);
	msleep(10);
	ret = mx_hub_read(client, bytes,dest);
	if (ret < 0)
		dev_err(&client->dev,
			"can not read register, returned %d\n", ret);
#else
	ret = mx_hub_read_addr(client, reg,bytes,dest);
	if (ret < 0)
		dev_err(&client->dev,
			"can not read register, returned %d\n", ret);
#endif
	mutex_unlock(&hub->iolock);

	if (ret < 0)
		mx_hub_reinit(hub);

	return ret;
}

int mx_hub_writedata_rdy(struct i2c_client *client, u8 reg, int bytes, const void *src)
{
	struct mx_hub_dev *hub = i2c_get_clientdata(client);
	int ret;	
	unsigned char buf[256];
	int size;

	//if( hub->hub_sleep)
	//	return 0;

	if( hub->is_update )
		return -EBUSY;

	mutex_lock(&hub->iolock);
	if(bytes > 255 )
		bytes = 255;

	if( reg != 0xFF )
	{
		buf[0] = reg;
		memcpy(&buf[1],src,bytes);

		size = bytes + 1;
		ret = mx_hub_write(client,size,buf);
		if (ret < 0)
		dev_err(&client->dev,
			"can not write register, returned %d\n", ret);
	}
	else
	{
		ret = mx_hub_write(client,bytes,src);
		if (ret < 0)
		dev_err(&client->dev,
			"can not write register, returned %d\n", ret);
	}

	if( mx_hub_waitforidle(hub,10) < 0)
		dev_err(&client->dev,
			"wait idle timeout.\n");		

	mutex_unlock(&hub->iolock);
	
	if (ret < 0)
		mx_hub_reinit(hub);

	return ret;
}

int mx_hub_writedata(struct i2c_client *client, u8 reg, int bytes, const void *src)
{
	struct mx_hub_dev *hub = i2c_get_clientdata(client);
	int ret;	
	unsigned char buf[256];
	int size;

	if( hub->is_update )
		return -EBUSY;

	mutex_lock(&hub->iolock);
	if(bytes > 255 )
		bytes = 255;

	buf[0] = reg;
	memcpy(&buf[1],src,bytes);

	size = bytes + 1;
	ret = mx_hub_write(client,size,buf);
	if (ret < 0)
		dev_err(&client->dev,
			"can not write register, returned %d\n", ret);


	mutex_unlock(&hub->iolock);

	return ret;
}


struct mx_hub_dev * gmx_hub = NULL;
int mx_hub_waitforidle(struct mx_hub_dev *hub,int mtimeout)
{
	static struct mx_hub_dev * hmx = NULL;
	int ms = mtimeout *1;

	if(hub)
		hmx = hub;
	if( !hmx )
	{
		pr_err("%s, handle is NULL\n", __func__);
		return -EXDEV;
	}
	
	do
	{
		msleep(1);
		//usleep_range(100,500);
		//udelay(100);
	}while(mx_hub_is_busy(hmx) && ms--);

	return ms? 0:-EBUSY;	
}
EXPORT_SYMBOL(mx_hub_waitforidle);

int mx_hub_setslopeperiod(u16 period)
{
	int ret = 0;
	
	if( !gmx_hub )
	{
		pr_err("%s, handle is NULL\n", __func__);
		return -EXDEV;
	}

	ret = mx_hub_writedata(gmx_hub->client,MX_HUB_REG_LED_SLPPRD,2,(u8 *)&period);

	return ret;
}
EXPORT_SYMBOL(mx_hub_setslopeperiod);


int mx_hub_setbrightness(u8 data)
{
	int ret = 0;
	
	if( !gmx_hub )
	{
		pr_err("%s, handle is NULL\n", __func__);
		return -EXDEV;
	}

	ret = mx_hub_writebyte(gmx_hub->client,MX_HUB_REG_LED_BRN,data);

	if(ret < 0)
		mx_hub_reinit(gmx_hub);
	
	return ret;
}
EXPORT_SYMBOL(mx_hub_setbrightness);

int mx_hub_setslope(u8 data)
{
	int ret = 0;
	
	if( !gmx_hub )
	{
		pr_err("%s, handle is NULL\n", __func__);
		return -EXDEV;
	}

	ret = mx_hub_writebyte(gmx_hub->client,MX_HUB_REG_LED_SLP,data);

	return ret;
}
EXPORT_SYMBOL(mx_hub_setslope);

int mx_hub_trigger_fadeonoff(void )
{
	int ret = 0;
	
	if( !gmx_hub )
	{
		pr_err("%s, handle is NULL\n", __func__);
		return -EXDEV;
	}

	ret = mx_hub_writebyte(gmx_hub->client,MX_HUB_REG_LED_FADE,0x01);

	return ret;
}
EXPORT_SYMBOL(mx_hub_setslope);

int mx_hub_wake_up(void )
{
	if( !gmx_hub )
	{
		pr_err("%s, handle is NULL\n", __func__);
		return -EXDEV;
	}

	mx_hub_wakeup(gmx_hub,true);

	return 0;
}
EXPORT_SYMBOL(mx_hub_wake_up);

int mx_hub_setkeytype(struct mx_hub_dev *hub,u8 type)
{
	int ret = 0;

	if(type >= MX_KEY_TYPE_MAX)
		return -1;
	
	ret = mx_hub_writebyte(hub->client,MX_HUB_REG_WAKEUP_TYPE,type);
	if (ret < 0)
		pr_err("mx_hub_writebyte error at %d line\n", __LINE__);
	else
		hub->key_wakeup_type = type;
	
	pr_info("%s: %d \n", __func__,type);
	return ret;
}
EXPORT_SYMBOL(mx_hub_setkeytype);

static void mx_hub_reinit(struct mx_hub_dev *hub)
{
#ifndef	TEST_REST_MODE
	if( iic_error > MAX_IIC_ERROR )
	{
		iic_error = 0;
		mx_hub_reset(hub,RESET_COLD);
		mx_hub_identify(hub);
		mx_hub_init_registers(hub);
		pr_info("%s\n", __func__);
	}
#endif	

}

static void mx_hub_init_registers(struct mx_hub_dev *hub)
{
	int i, ret;
	unsigned char buf[2];

	//if( hub->binfo.run_in_B_A != 'A' )
	{
		buf[0] = TWI_CMD_EXTBOOT;
		ret = mx_hub_write(hub->client,1,buf);
		if (ret) {
			dev_err(hub->dev, "failed to bootting...\n");
			return;
		}
		
		dev_dbg(hub->dev, "exit bootloader ...\n");
		
		// wait for MCU Boot from bootloader
		msleep(100);
		mx_hub_waitforidle(hub,100);

		if(1)
		{
			i = 0;
			memset(buf,0,2);
			ret = mx_hub_read_addr(hub->client, MX_HUB_REG_VERSION_A,1,buf);
			while( ret < 0 )
			{
				dev_dbg(hub->dev, "ID :0x%.2X \n", buf[0]);
				msleep(100);
				if(i ++> 100)
				{
					dev_err(hub->dev, "failed to init..\n");	
					break;
				}
				ret = mx_hub_read_addr(hub->client, MX_HUB_REG_VERSION_A,1,buf);
			};
			dev_dbg(hub->dev, "exit bootloader %d ...\n",i);
		}
	}

	for (i=0; i<ARRAY_SIZE(init_regs); i++) {		
		buf[0] = init_regs[i].addr;
		buf[1] = init_regs[i].value;
		ret = mx_hub_write(hub->client,2,buf);
		if (ret) {
			dev_err(hub->dev, "failed to init reg[%d], ret = %d\n", i, ret);
		}
	}
	
	//mx_hub_setbrightness(hub,100);
	//mx_hub_setslope(hub,0x00);
}


static int mx_hub_get_bootinfo(struct mx_hub_dev *hub,bool bforcechkcrc)
{
	int ret,retry_cnt,retry_rest;
	unsigned char data;
	unsigned short crc;
	struct i2c_client *client = hub->client;


	retry_rest = 10;

	dev_dbg(&client->dev, "read boot information\n");
retry_reset:	
	mutex_lock(&hub->iolock);
	if(bforcechkcrc)
	{		
		dev_dbg(&client->dev, "Check crc starting ... \n");
		data = TWI_CMD_CRCCAL;
		ret = mx_hub_write(client, 1,&data);
		
		mx_hub_waitforidle(hub,1000);
		
		ret = mx_hub_read_addr(client, TWI_CMD_CRCCHECK,2,&crc);
		dev_dbg(&client->dev, "crc 0x%.4X  %d\n",crc,ret);
		
	}
	else
	{
		msleep(100);
	}
		

	retry_cnt = 0;
retry:	
	memset(&hub->binfo.byte[0],0x00,sizeof(union mx_hub_boot_intfo));
	ret = mx_hub_read_addr(client, TWI_CMD_BINFO,sizeof(union mx_hub_boot_intfo),&hub->binfo.byte[0]);
	if( ret < 0)
	{
		dev_err(&client->dev, "read boot info err %d,try %d \n",ret,retry_cnt);
		dev_dbg(&client->dev, "Boot info :0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X \n", 
			hub->binfo.byte[0],hub->binfo.byte[1],hub->binfo.byte[2],hub->binfo.byte[3],
			hub->binfo.byte[4],hub->binfo.byte[5],hub->binfo.byte[6],hub->binfo.byte[7]);
		if( (++retry_cnt) < 3)
		{
			msleep(100);
			goto retry;
		}
	}
	
	mutex_unlock(&hub->iolock);
	
	if(ret == 0)
	dev_info(&client->dev, "Boot info :0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X \n", 
		hub->binfo.byte[0],hub->binfo.byte[1],hub->binfo.byte[2],hub->binfo.byte[3],
		hub->binfo.byte[4],hub->binfo.byte[5],hub->binfo.byte[6],hub->binfo.byte[7]);

	if(ret < 0 && retry_rest > 0 )
	{
		retry_rest --;

		if (hub->regulator) {
			s3c_gpio_cfgpin(hub->gpio_wake, S3C_GPIO_OUTPUT);
			gpio_set_value(hub->gpio_wake,  0);
			s3c_gpio_cfgpin(hub->gpio_reset, S3C_GPIO_OUTPUT);
			gpio_set_value(hub->gpio_reset,  0);		
			msleep(150);		
			
			ret = regulator_disable(hub->regulator);
			msleep(500);
			ret = regulator_enable(hub->regulator);
			msleep(50);
			
			s3c_gpio_setpull(hub->gpio_reset, S3C_GPIO_PULL_UP);
			s3c_gpio_cfgpin(hub->gpio_reset, S3C_GPIO_INPUT);
			
			msleep(500);
		}
		
		goto retry_reset; 
	}
		

	return ret;
}

static bool mx_hub_identify(struct mx_hub_dev *hub)
{
	int ret,val=0;
	struct i2c_client *client = hub->client;
	unsigned char img_ver=0;
	unsigned short img_crc=0;

	msleep(50);
	/* Read boot information */
	ret = mx_hub_get_bootinfo(hub,false);
	if( ret < 0)
	{
		dev_err(&client->dev, "could not get boot information\n");
		return false;
	}
	
	if (hub->binfo.run_in_B_A == 0x95) {
		dev_err(&client->dev, "Failed to initial Tiny44A\n");
		return false;
	}
	
	if (hub->binfo.ID!= MX_HUB_DEVICE_ID) {
		dev_err(&client->dev, "Failed to get ID 0x%X\n", hub->binfo.ID);
		goto upd_ext;
	}
	
	val = mx_hub_get_imgfw_info(hub);

	img_crc = hub->img_info.crc;
	img_ver = hub->img_info.ver;
	
	if( (hub->img_info.ver &0xFF)  == 0xFF )
		img_ver = FW_VERSION;

	if( (hub->binfo.CRC != img_crc) || (hub->binfo.AVer< hub->img_info.ver )
		||( hub->binfo.bForceUpdate && hub->binfo.run_in_B_A ))
	{
		dev_info(&client->dev, "Old firmware version %d.%d , new image ver %d.%d, crc 0x%.4X,needs updating\n", 
			((hub->binfo.AVer>>4)&0x0F),(hub->binfo.AVer&0x0F), ((hub->img_info.ver>>4)&0x0F),(hub->img_info.ver&0x0F),hub->img_info.crc);
upd_ext:	
		mx_hub_update(hub);
		
		if (hub->binfo.ID != MX_HUB_DEVICE_ID) {
			dev_err(&client->dev, "ID %d not supported\n", hub->binfo.ID);
			return false;
		}
	}
	
	dev_info(&client->dev, "mx_hub id 0x%.2X firmware version %d.%d\n", hub->binfo.ID,((hub->binfo.AVer>>4)&0x0F),(hub->binfo.AVer&0x0F));
	
	return true;
}

static void mx_hub_reset(struct mx_hub_dev *hub,int m)
{
	if(m == 1)
	{	
		s3c_gpio_setpull(hub->gpio_reset, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(hub->gpio_reset, S3C_GPIO_OUTPUT);
		gpio_set_value(hub->gpio_reset,  0);
		msleep(500);
		gpio_set_value(hub->gpio_reset,  1);
		msleep(500);
		s3c_gpio_setpull(hub->gpio_reset, S3C_GPIO_PULL_UP);
		s3c_gpio_cfgpin(hub->gpio_reset, S3C_GPIO_INPUT);		
		dev_info(&hub->client->dev, "%s reset. \n", m?"cold":"soft");
	}
	else if(m == 5)
	{
		unsigned char msg[2];
		//msg[0] = MX_HUB_REG_BOOTL;
		//msg[1] = 1;			
		msg[0] = MX_HUB_REG_SOFTRESET;
		msg[1] = RESET_BOOTL;			
		mx_hub_write(hub->client,2,msg);
		dev_info(&hub->client->dev, "reset into bootloader. \n");
	}
	else 
	{
		unsigned char msg[2];
		msg[0] = MX_HUB_REG_SOFTRESET;
		msg[1] = 1;			
		mx_hub_write(hub->client,2,msg);
		dev_info(&hub->client->dev, "%s reset. \n", m?"cold":"soft");
	}
}


static void mx_hub_wakeup(struct mx_hub_dev *hub,int bEn)
{
	if( bEn )
	{
		hub->hub_sleep = false;
		s3c_gpio_setpull(hub->gpio_wake, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(hub->gpio_wake, S3C_GPIO_OUTPUT);
		gpio_set_value(hub->gpio_wake,  0);
	}
	else
	{	
		s3c_gpio_setpull(hub->gpio_wake, S3C_GPIO_PULL_UP);
		s3c_gpio_cfgpin(hub->gpio_wake, S3C_GPIO_INPUT);
		hub->hub_sleep = true;
	}	

	msleep(10);

	dev_dbg(&hub->client->dev, "mx_hub_wakeup %s\n",bEn?"enable":"disable");
}


static int mx_hub_is_busy(struct mx_hub_dev *hub)
{
	int bbusy;

	bbusy = !(gpio_get_value(hub->gpio_busy));

	dev_dbg(&hub->client->dev, "mx hub is %s.\n",bbusy?"busy":"not busy");

	return bbusy;
}

static int mx_hub_get_imgfw_info(struct mx_hub_dev *hub)
{
	int err = 0;
	const struct firmware *fw;
	const char *fw_name;
	s_device_info  * pdev_info;
	
	fw_name = MX_HUB_FW;

	err = request_firmware(&fw, fw_name,  &hub->client->dev);
	if (err) {
		printk(KERN_ERR "Failed to load firmware \"%s\"\n", fw_name);
		return err;
	}

	pdev_info  = (s_device_info *)(fw->data+fw->size-sizeof(s_device_info));	
	
	memcpy(&hub->img_info, pdev_info ,sizeof(s_device_info));
	
	release_firmware(fw);
	
	return err;
}

#define	DEBUG_TM
#ifdef DEBUG_TM
static inline unsigned long timeval_diff(struct timeval *to,
					struct timeval *from)
{
	return (from->tv_sec * USEC_PER_SEC + from->tv_usec)
		- (to->tv_sec * USEC_PER_SEC + to->tv_usec);
}
#endif

#define	PAGESIZE		(256)
#define	PAGE_START	(16)
#define	PAGE_END	(511)
static int mx_hub_update(struct mx_hub_dev *hub)
{
	const struct firmware *fw;
	const char *fw_name;

	int ret = 0;

	int i,page_num,cnt,size,try_cnt;
	unsigned char buf[PAGESIZE+3];
	const unsigned char * ptr;
	unsigned short crc;
	
#ifdef DEBUG_TM
	struct timeval start;
	struct timeval end;
	unsigned long  duration_us;
#endif

	wake_lock(&hub->wake_lock);
		
	hub->is_update = true;
	
	disable_irq(hub->irq);	
	mx_hub_wakeup(hub,true);
	mx_hub_reset(hub,RESET_BOOTL);
	mx_hub_reset(hub,RESET_COLD);
	
	msleep(1000);

	try_cnt = 1;
_start:		
	ret = mx_hub_get_bootinfo(hub,false);
	if( ret < 0)
	{
		dev_err(&hub->client->dev, "could not read boot information, returned %d at line %d\n", ret,__LINE__);
		goto err_exit10;
	}
	
	if (hub->binfo.ID!= MX_HUB_DEVICE_ID) {
		dev_err(&hub->client->dev, "Failed to get ID 0x%X\n", hub->binfo.ID);
		goto err_exit10;
	}

	if(hub->binfo.run_in_B_A == MCU_RUN_APP )
	{
		if((!IS_ERR( hub->regulator)) && try_cnt > 0)
		{
			mx_hub_reset(hub,RESET_BOOTL);
			mx_hub_reset(hub,RESET_COLD);
			if(1)
			{
				ret = regulator_disable(hub->regulator);
				msleep(500);
				regulator_set_voltage(hub->regulator,1800000,1800000);
				ret = regulator_enable(hub->regulator);
			}
			msleep(500);
			try_cnt--;
			goto _start;
		}
			
		goto err_exit10;
	}
	
	dev_info(&hub->client->dev,"bootloader revision %d.%d\n", ((hub->binfo.BVer>>4)&0x0F),(hub->binfo.BVer&0x0F));
	
	fw_name = MX_HUB_FW;		
	ret = request_firmware(&fw, fw_name,  &hub->client->dev);
	if (ret) {
		dev_err(&hub->client->dev,"Failed to load firmware \"%s\"\n", fw_name);
		goto err_exit;
	}

	try_cnt = 3;
	size = fw->size;	// 1024*124
	ptr = fw->data;
	dev_info(&hub->client->dev,"load firmware %s,size = 0x%X\n",fw_name,size);
	dev_info(&hub->client->dev,"Updating ... \n");
	
#ifdef	DEBUG_TM
	do_gettimeofday(&start);
#endif

	do
	{
		buf[0] = TWI_CMD_UPD;
		for(i=0;i<size;i+=PAGESIZE)
		{
			page_num = PAGE_START*256 + i;
			page_num = page_num >> 8;
			
			buf[2] = page_num & 0xFF;
			buf[1] = (page_num>>8) & 0xFF;

			memcpy(&buf[3],ptr+i,PAGESIZE);

			cnt = 3;
			do
			{
				ret = mx_hub_write(hub->client,sizeof(buf),buf);
				udelay(100);//msleep(1);//usleep_range(500,1000);				
			}while(ret < 0 && cnt--);
			
			if(ret < 0 )
			{
				dev_err(&hub->client->dev,"can not write register, returned %d at addres 0x%.4X(page %d)\n", ret,i,page_num);
				break;
			}
			else
			{
				dev_dbg(&hub->client->dev,"update addres 0x%.4X(page %d)\n", i,page_num);
			}
			
		}	
		
		if(ret < 0 )
			goto err_exit;		
		
	
#ifdef	DEBUG_TM
	do_gettimeofday(&end);
	duration_us = timeval_diff(&start,&end);
	dev_info(&hub->client->dev, "Time to reflash: %ld.%ld s.\n", duration_us/1000000,duration_us%1000000);
#endif
		ret = mx_hub_get_bootinfo(hub,true);
		if( ret < 0)
			dev_err(&hub->client->dev, "could not read boot information, returned %d at line %d\n", ret,__LINE__);
		
		crc  = *(unsigned short *)(fw->data+fw->size-2);		
		if( crc != hub->binfo.CRC)
		{
			dev_err(&hub->client->dev,"crc check 0x%.4X (0x%.4X) failed !!!\n", crc,hub->binfo.CRC);
		}
		else
		{
			dev_info(&hub->client->dev,"Verifying flash OK!\n");
			break;
		}		
	}while(try_cnt--);
	
	dev_info(&hub->client->dev, "Update completed,. \n");
	release_firmware(fw);
	goto exit;
	
err_exit:	
	release_firmware(fw);
err_exit10:	
	dev_info(&hub->client->dev, "Update failed !! \n");
	
exit:
	mx_hub_wakeup(hub,true);

	hub->is_update = false;
	
	enable_irq(hub->irq);
	
	wake_unlock(&hub->wake_lock);
	return ret;
}

static ssize_t mx_hub_show_property(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf);
static ssize_t mx_hub_store(struct device *dev, 
			     struct device_attribute *attr,
			     const char *buf, size_t count);

#define MX_MX_HUB_ATTR(_name)\
{\
    .attr = { .name = #_name, .mode = S_IRUGO | S_IWUSR},\
    .show = mx_hub_show_property,\
    .store = mx_hub_store,\
}

static struct device_attribute mx_hub_attrs[] = {
    MX_MX_HUB_ATTR(status),
    MX_MX_HUB_ATTR(cmd),
    MX_MX_HUB_ATTR(reset),
    MX_MX_HUB_ATTR(version),
    MX_MX_HUB_ATTR(update),
    MX_MX_HUB_ATTR(key_wakeup_count),
    MX_MX_HUB_ATTR(key_wakeup_type),
    MX_MX_HUB_ATTR(irq),
    MX_MX_HUB_ATTR(debug),
    MX_MX_HUB_ATTR(iic),
    MX_MX_HUB_ATTR(test),
    MX_MX_HUB_ATTR(reg),
    MX_MX_HUB_ATTR(sreg),
    MX_MX_HUB_ATTR(sleep),
};
enum {
	MX_MX_HUB_STATUS,
	MX_MX_HUB_CMD,
	MX_MX_HUB_RESET,
	MX_MX_HUB_FWR_VER,
	MX_MX_HUB_UPD,
	MX_MX_HUB_CNT,
	MX_MX_HUB_WAKEUP_TYPE,
	MX_MX_HUB_IRQ,
	MX_MX_HUB_DEBUG,
	MX_MX_HUB_IIC,
	MX_MX_HUB_TEST,
	MX_MX_HUB_REG,
	MX_MX_HUB_SREG,
	MX_MX_HUB_SLEEP,
};
static ssize_t mx_hub_show_property(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf)
{
	int i = 0;
	ptrdiff_t off;
	struct mx_hub_dev *hub = (struct mx_hub_dev*)dev_get_drvdata(dev);
	
	if(!hub)
	{
		pr_err("%s(): failed!!!\n", __func__);
		return -ENODEV;
	}

	off = attr - mx_hub_attrs;

	switch(off){
	case MX_MX_HUB_STATUS:
		i += scnprintf(buf+i, PAGE_SIZE-i, "%d\n",(mx_hub_readbyte(hub->client,MX_HUB_REG_STATUS)&0xFF));
		break;
	case MX_MX_HUB_CMD:
		{
			int ret,val;

			val = 0x00;
			ret = mx_hub_read(hub->client,1,&val);
			if (ret < 0)
				pr_err("mx_hub_readbyte error at %d line\n", __LINE__);
			i += scnprintf(buf+i, PAGE_SIZE-i, "0x%.2X\n",val);
		}
		break;
	case MX_MX_HUB_SREG:
	case MX_MX_HUB_REG:
		{
			int i,ret;
			unsigned char buffer[256];
			
			ret = mx_hub_readdata(hub->client, 0x00,256,buffer);
			if (ret < 0)
			{
				pr_err("mx_hub_readbyte error at %d line\n", __LINE__);
			}
			else
			{
				for(i = 0;i< 256;i++)
					pr_info("R:0x%.2X V:0x%.2X\n",i, buffer[i]);
			}
			
			i += scnprintf(buf+i, PAGE_SIZE-i, "0x%.2X\n",buffer[0]);
		}
		break;
	case MX_MX_HUB_FWR_VER:
		i += scnprintf(buf+i, PAGE_SIZE-i, "ID = %c ,Ver.0x%X\n",(mx_hub_readbyte(hub->client,MX_HUB_REG_DEVICE_ID)&0xFF),
			(mx_hub_readbyte(hub->client,MX_HUB_REG_VERSION_A)&0xFF));
		if(1)
		{
			union mx_hub_boot_intfo binfo;
			memset(&binfo.byte[0],0x00,sizeof(union mx_hub_boot_intfo));
			mx_hub_readdata(hub->client, TWI_CMD_BINFO,sizeof(union mx_hub_boot_intfo),&binfo.byte[0]);
			i += scnprintf(buf+i, PAGE_SIZE-i, "Boot info :0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X 0x%.2X \n", 
				binfo.byte[0],binfo.byte[1],binfo.byte[2],binfo.byte[3],
				binfo.byte[4],binfo.byte[5],binfo.byte[6],binfo.byte[7]);
		}		
		break;
	case MX_MX_HUB_RESET:		
		i += scnprintf(buf+i, PAGE_SIZE-i, "%d\n",(mx_hub_readbyte(hub->client,MX_HUB_REG_SOFTRESET)&0xFF));
		break;
	case MX_MX_HUB_UPD:
		mx_hub_update(hub);
		mx_hub_init_registers(hub);
		i += scnprintf(buf+i, PAGE_SIZE-i, "\n");
		break;
	case MX_MX_HUB_CNT:
		i += scnprintf(buf+i, PAGE_SIZE-i, "%d\n",(mx_hub_readbyte(hub->client,MX_HUB_REG_WAKEUP_CNT)&0xFF));
		break;
	case MX_MX_HUB_WAKEUP_TYPE:
		i += scnprintf(buf+i, PAGE_SIZE-i, "%d\n",hub->key_wakeup_type);
		break;
	case MX_MX_HUB_IRQ:		
		i += scnprintf(buf+i, PAGE_SIZE-i, "0x%X\n",(mx_hub_readbyte(hub->client,MX_HUB_REG_IRQ)&0xFF));
		break;
	case MX_MX_HUB_DEBUG:		
		i += scnprintf(buf+i, PAGE_SIZE-i, "0x%X\n",hub->debug);
		break;
#ifdef	TEST_REST_MODE
	case MX_MX_HUB_TEST:
		i += scnprintf(buf+i, PAGE_SIZE-i, "%d(%d)\n",test.test_failed,test.test_cycles);
		break;
#endif		
	default:
		i += scnprintf(buf+i, PAGE_SIZE-i, "Error\n");
		break;	
	}
	return i;
}

static ssize_t mx_hub_store(struct device *dev, 
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	unsigned int reg,value;
	int ret = 0;
	ptrdiff_t off;
	struct mx_hub_dev *hub = (struct mx_hub_dev*)dev_get_drvdata(dev);
	
	if(!hub)
	{
		pr_err("%s(): failed!!!\n", __func__);
		return -ENODEV;
	}

	off = attr - mx_hub_attrs;

	switch(off){
	case MX_MX_HUB_STATUS:
		if (sscanf(buf, "%d\n", &value) == 1) {	
			ret = mx_hub_writebyte(hub->client,MX_HUB_REG_STATUS,value);
			if (ret < 0)
				pr_err("mx_hub_writebyte error at %d line\n", __LINE__);			
		}
		ret = count;
		break;
	case MX_MX_HUB_REG:
	case MX_MX_HUB_CMD:
		if (sscanf(buf, "%x %x  %x\n", &reg, &value,&ret) == 3) {
			int res;
			unsigned char msg[3];
			msg[0] = reg;
			msg[1] = value;	
			msg[2] = ret;			
			dev_info(dev, "R:0x%.2X V:0x%.2X 0x%.2X \n", reg,value,ret);
			res = mx_hub_write(hub->client,3,msg);
			if (res < 0)
				pr_err("mx_hub_writebyte error %d at %d line\n", res,__LINE__);
		}
		else if (sscanf(buf, "%x %x\n",  &reg, &value) == 2) {
			int ret;
			unsigned char msg[2];
			msg[0] = reg;
			msg[1] = value;			
			dev_info(dev, "R:0x%.2X V:0x%.2X \n", reg,value);
			ret = mx_hub_write(hub->client,2,msg);
			if (ret < 0)
				pr_err("mx_hub_writebyte error %d at %d line\n", ret,__LINE__);
		}
		else if (sscanf(buf, "%x\n", &value) == 1) {	
			int ret;
			unsigned char msg[2];
			msg[0] = value;			
			dev_info(dev, "V:0x%.2X \n", value);
			ret = mx_hub_write(hub->client,1,msg);
			if (ret < 0)
				pr_err("mx_hub_writebyte error %d at %d line\n",ret, __LINE__);
			
		}
		else {			
			pr_err("%s(): failed !!!\n", __func__);
		}
		ret = count;
		break;
	case MX_MX_HUB_RESET:
		if (sscanf(buf, "%x\n", &value) == 1) {	
			mx_hub_reset(hub,value);
		}
		ret = count;
		break;
	case MX_MX_HUB_FWR_VER:
		ret = count;
		break;
	case MX_MX_HUB_UPD:
		mx_hub_update(hub);
		mx_hub_init_registers(hub);
		ret = count;
		break;
	case MX_MX_HUB_CNT:
		ret = count;
		break;
	case MX_MX_HUB_WAKEUP_TYPE:
		if (sscanf(buf, "%d\n", &value) == 1) {	
			int ret;
			dev_info(dev, "V:0x%.2X \n", value);

			ret = mx_hub_setkeytype(hub,value);
			if (ret < 0)
			{
				pr_err("Invalid argument\n");
			}
		}
		else {			
			pr_err("%s(): failed !!!\n", __func__);
		}
		ret = count;
		break;
	case MX_MX_HUB_IRQ:
		if (sscanf(buf, "%x\n", &value) == 1) {	
			if( value )
				enable_irq(hub->irq);
			else
				disable_irq(hub->irq);
			pr_info("%s irq. \n", value?"enable":"disable");
		}
		ret = count;
		break;
	case MX_MX_HUB_DEBUG:
		if (sscanf(buf, "%d\n", &value) == 1) {
			hub->debug = !!value;
			pr_info("%s debug infomation. \n", value?"enable":"disable");
		}
		ret = count;
		break;
	case MX_MX_HUB_IIC:
		if (sscanf(buf, "%d\n", &value) == 1) {
			
			struct i2c_algo_bit_data *bit_adap;
			struct i2c_gpio_platform_data *pI2Cdata;
			
			bit_adap = hub->client->adapter->algo_data;
			pI2Cdata = bit_adap->data;
			
			mutex_lock(&hub->iolock);

			gpio_free(pI2Cdata->sda_pin);
			gpio_free(pI2Cdata->scl_pin);
			
			if(value)
			{
				pI2Cdata->scl_pin = MEIZU_MCU_SCL;
				pI2Cdata->sda_pin = MEIZU_MCU_SDA;
			}
			else
			{
				pI2Cdata->scl_pin = MEIZU_MCU_SCL_O;
				pI2Cdata->sda_pin = MEIZU_MCU_SDA_O;
			}
		
			gpio_request(pI2Cdata->sda_pin, "sda");
			gpio_request(pI2Cdata->scl_pin, "scl");	

			mutex_unlock(&hub->iolock);
				
			pr_info(" iic %d. \n", value);
		}
		ret = count;
		break;
#ifdef	TEST_REST_MODE
	case MX_MX_HUB_TEST:
		if (sscanf(buf, "%d\n", &value) == 1) {
			
			test.test_cycles = 0;
			test.test_failed = 0;
			test.test_reset_enable = !!value;
			pr_info("%s test mode. \n", value?"enable":"disable");
			if(test.test_reset_enable)
			{				
				disable_irq(hub->irq);
				schedule_delayed_work( &test.test_reset_work, msecs_to_jiffies(500));
			}
			else
			{
				cancel_delayed_work_sync(&test.test_reset_work);
				enable_irq(hub->irq);
			}
		}
		ret = count;
		break;
#endif
	case MX_MX_HUB_SLEEP:		
		if (sscanf(buf, "%d\n", &value) == 1) {
			
			mx_hub_wakeup(hub,!value);
			pr_info(" sleep %d. \n", !value);
		}
		ret = count;
		break;
	case MX_MX_HUB_SREG:
		if (sscanf(buf, "%x %x\n", &reg, &value) == 2) {
			int res;
			unsigned char msg[3];
			msg[0] = reg;
			msg[1] = value;
			dev_info(dev, "R:0x%.2X V:0x%.2X 0x%.2X \n", reg,value,ret);
			res = mx_hub_write(hub->client,2,msg);
			if (res < 0)
				pr_err("mx_hub_writebyte error %d at %d line\n", res,__LINE__);
		}
		else if (sscanf(buf, "%x\n",  &reg) == 1) {
			
			ret = mx_hub_readdata_rdy(hub->client,reg,1,&value);
			dev_info(dev, "R:0x%.2X V:0x%.2X ,ret = %d \n", reg,value&0xFF,ret);
		}
		ret = count;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;	
}

#define	LINK_KOBJ_NAME	"mx_hub"
static struct kobject *mx_hub_devices_kobj = NULL;
/**
 * mx_create_link - create a sysfs link to an exported virtual node
 *	@target:	object we're pointing to.
 *	@name:		name of the symlink.
 *
 * Set up a symlink from /sys/class/input/inputX to 
 * /sys/devices/mx_tsp node. 
 *
 * Returns zero on success, else an error.
 */
static int mx_hub_create_link(struct kobject *target, const char *name)
{
	int rc = 0;
	
	struct device *device_handle = NULL;
	struct kset *pdevices_kset;
	
	device_handle = kzalloc(sizeof(*device_handle), GFP_KERNEL);
	if (!device_handle){
		rc = -ENOMEM;
		return rc;
	}
	
	device_initialize(device_handle);
	pdevices_kset = device_handle->kobj.kset;
	mx_hub_devices_kobj = &pdevices_kset->kobj;
	kfree(device_handle);	
	
	if( !mx_hub_devices_kobj )
	{
		rc = -EINVAL;
		goto err_exit;
	}
	
	rc = sysfs_create_link(mx_hub_devices_kobj,target, name);
	if(rc < 0)
	{
		pr_err("sysfs_create_link failed.\n");
		goto err_exit;
	}

	return rc;
	
err_exit:
	mx_hub_devices_kobj = NULL;
	pr_err("mx_create_link failed %d \n",rc);
	return rc;
}
	 
static void mx_hub_remove_link(const char *name)
{
 	if( mx_hub_devices_kobj )
	{
		sysfs_remove_link(mx_hub_devices_kobj, name);
		mx_hub_devices_kobj = NULL;
	}
}

static int mx_hub_create_attrs(struct device * dev)
{
	int i, rc;

	rc = mx_hub_create_link(&dev->kobj,LINK_KOBJ_NAME);
	if (rc  < 0 )
		goto exit_sysfs_create_link_failed;

	for (i = 0; i < ARRAY_SIZE(mx_hub_attrs); i++) {
		rc = device_create_file(dev, &mx_hub_attrs[i]);
		if (rc)
			goto mx_hub_attrs_failed;
	}
	return rc;

mx_hub_attrs_failed:
	printk(KERN_INFO "%s(): failed!!!\n", __func__);	
	while (i--)
		device_remove_file(dev, &mx_hub_attrs[i]);

exit_sysfs_create_link_failed:
	mx_hub_remove_link(LINK_KOBJ_NAME);
	return rc;

}

static void mx_hub_destroy_atts(struct device * dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(mx_hub_attrs); i++)
		device_remove_file(dev, &mx_hub_attrs[i]);

	mx_hub_remove_link(LINK_KOBJ_NAME);	
}

#ifdef CONFIG_HAS_EARLYSUSPEND
 static void mx_hub_early_suspend(struct early_suspend *h)
 {
	 struct mx_hub_dev *mx_hub =
			 container_of(h, struct mx_hub_dev, early_suspend);
	 	  
	 mx_hub_wakeup(mx_hub,false);	 
 }
 
 static void mx_hub_late_resume(struct early_suspend *h)
 {
	 struct mx_hub_dev *mx_hub =
			 container_of(h, struct mx_hub_dev, early_suspend);
	 
	mx_hub_wakeup(mx_hub,true);	
 }
#endif

#ifdef	TEST_REST_MODE	
static void mx_hub_test_rest_work_func(struct work_struct *work)
{
	 struct mx_hub_dev *mx_hub = gmx_hub;
	 int ret;

	test.test_cycles++;
	if (mx_hub->regulator) {		
		s3c_gpio_cfgpin(gmx_hub->gpio_reset, S3C_GPIO_OUTPUT);
		gpio_set_value(gmx_hub->gpio_reset,  0);		
		msleep(10);		
		ret = regulator_disable(mx_hub->regulator);
		
		msleep(5000);
		
		regulator_set_voltage(mx_hub->regulator,1900000,1900000);
		ret = regulator_enable(mx_hub->regulator);
		msleep(10);
		
		s3c_gpio_setpull(gmx_hub->gpio_reset, S3C_GPIO_PULL_UP);
		s3c_gpio_cfgpin(gmx_hub->gpio_reset, S3C_GPIO_INPUT);
		
		msleep(300);	

		if (!mx_hub_identify(gmx_hub))
		{
			test.test_failed ++;
		}
		else
		{
		
			/*initial registers*/
			mx_hub_init_registers(mx_hub);

		}
		
	}
	
	if( test.test_reset_enable)
		schedule_delayed_work( &test.test_reset_work, msecs_to_jiffies(500));
	
	printk(KERN_INFO "%s():cycle:%d  failed %d .\n", __func__,test.test_cycles,test.test_failed);				
}
#endif

static int __devinit mx_hub_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{	
	struct mx_sensor_hub_platform_data *pdata = client->dev.platform_data;
	struct mx_hub_dev *mx_hub;
	struct regulator *regulator = NULL;
	struct regulator *regulator_sen = NULL;
	int ret = 0;
	
	dev_dbg(&client->dev,"%s:++\n",__func__);
			
#ifdef	INIT_RESET
	ret = gpio_request_one(pdata->gpio_reset,
				   GPIOF_OUT_INIT_LOW, "HUB RST");
#else
	ret = gpio_request_one(pdata->gpio_reset,
				   GPIOF_DIR_IN, "HUB RST");
#endif
	if (ret < 0) {
		dev_err(&client->dev, "Failed to request GPIO %d: %d\n",
			pdata->gpio_reset, ret);
		goto exit;
	}
	
	ret = gpio_request_one(pdata->gpio_wake,
				   GPIOF_DIR_IN, "HUB WAK");
	if (ret < 0) {
		dev_err(&client->dev, "Failed to request GPIO %d: %d\n",
			pdata->gpio_wake, ret);
		goto exit_gpio_rst;
	}
	
	regulator = regulator_get(&client->dev, "vdd18_mcu");
	if (!IS_ERR(regulator)) {
		
	#ifdef	INIT_RESET
		regulator_set_voltage(regulator,1850000,1850000);
		ret = regulator_enable(regulator);
		msleep(10);
		
		s3c_gpio_setpull(pdata->gpio_reset, S3C_GPIO_PULL_UP);
		s3c_gpio_cfgpin(pdata->gpio_reset, S3C_GPIO_INPUT);
		dev_info(&client->dev, "regulator_enable: %d\n", ret);
		msleep(300);
	#endif
	
	}
	else
	{
		dev_err(&client->dev, "failed to get regulator (0x%p).\n",regulator);
	}

	mx_hub = kzalloc(sizeof(struct mx_hub_dev), GFP_KERNEL);
	if (mx_hub == NULL)
	{
		ret =  -ENOMEM;
		goto exit_regulator;
	}

	gmx_hub = mx_hub;

	i2c_set_clientdata(client, mx_hub);
	mx_hub->dev = &client->dev;
	mx_hub->client = client;
	mx_hub->regulator = regulator;
	mx_hub->irq = __gpio_to_irq(pdata->gpio_irq);
	mx_hub->type = id->driver_data;
	mx_hub->gpio_wake = pdata->gpio_wake;
	mx_hub->gpio_reset = pdata->gpio_reset;
	mx_hub->gpio_irq = pdata->gpio_irq;
	mx_hub->gpio_busy= pdata->gpio_busy;
	
	mx_hub->is_update = false;
	mx_hub->key_wakeup_type = 0;
	mx_hub->hub_sleep = false;
	mx_hub->debug = false;
		
	if (pdata) {
		mx_hub->irq_base = pdata->irq_base;
		mx_hub->wakeup = pdata->wakeup;
	}
	mutex_init(&mx_hub->iolock);
	wake_lock_init(&mx_hub->wake_lock, WAKE_LOCK_SUSPEND, "mx_hub");
	
	if( force_update )
	{
		dev_info(&client->dev,"mx_hub:force update...\n");
		mx_hub_update(mx_hub);
	}
	
	mx_hub_wakeup(mx_hub,true);
#if 1	
	/* Identify the mx_hub chip */
	if (!mx_hub_identify(mx_hub))
	{
		if (machine_is_m65()) 
		{
		#ifndef CONFIG_MX_HUB_IIC18
			ret = -ENODEV;	
			goto exit_dbg;	

		#else
			struct i2c_algo_bit_data *bit_adap;
			struct i2c_gpio_platform_data *pI2Cdata;
			
			bit_adap = client->adapter->algo_data;
			pI2Cdata = bit_adap->data;

			gpio_free(pI2Cdata->sda_pin);
			gpio_free(pI2Cdata->scl_pin);
			
			pI2Cdata->scl_pin = MEIZU_MCU_SCL_O;
			pI2Cdata->sda_pin = MEIZU_MCU_SDA_O;
			
			gpio_request(pI2Cdata->sda_pin, "sda");
			gpio_request(pI2Cdata->scl_pin, "scl");		
			msleep(5);	
			
			if (!mx_hub_identify(mx_hub))
			{
				ret = -ENODEV;
				goto exit_dbg;
			}
			else
			{
				dev_info(&client->dev,"mx_hub:old hardware version!!!\n");
			}
		#endif
		}
		else
		{
			ret = -ENODEV;
			goto exit_dbg;
		}
	}
#endif	
	if (machine_is_m69()) 
	{
		// "VDD18_SEN"
		regulator_sen = regulator_get(&client->dev, "vdd18_sen");
		if (!IS_ERR(regulator_sen)) {
			ret = regulator_enable(regulator_sen);
			msleep(300);
		}
		else
		{
			dev_err(&client->dev, "failed to get regulator sensor(0x%p).\n",regulator_sen);
		}
	}
	
	mx_hub_create_attrs(&client->dev);
		
	/*initial registers*/
	mx_hub_init_registers(mx_hub);

	mx_hub_irq_init(mx_hub);

	pm_runtime_set_active(mx_hub->dev);
#ifndef	TEST_REST_MODE	
	ret = mfd_add_devices(mx_hub->dev, -1,
			mx_hub_devs, ARRAY_SIZE(mx_hub_devs),
			NULL, mx_hub->irq_base);
	if (ret < 0)
	{
		dev_err(&client->dev,"MX Sensor Hub add devices failed\n");
		//goto exit_err_irq;
	}
#else

	test.test_reset_enable = 0;
	test.test_cycles = 0;
	test.test_failed = 0;

	INIT_DELAYED_WORK(&test.test_reset_work, mx_hub_test_rest_work_func);
#endif	
	
	device_init_wakeup(mx_hub->dev, mx_hub->wakeup);
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	 mx_hub->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN  + 5;
	 mx_hub->early_suspend.suspend = mx_hub_early_suspend;
	 mx_hub->early_suspend.resume = mx_hub_late_resume;
	 register_early_suspend(&mx_hub->early_suspend);
#endif
	dev_dbg(&client->dev,"%s:--\n",__func__);

	return 0;
	
exit_dbg:
	
	mx_hub_create_attrs(&client->dev);	
	
	return 0;
	
//exit_err_irq:
	mx_hub_irq_exit(mx_hub);
	mx_hub_destroy_atts(&client->dev);
//exit_err:
	mfd_remove_devices(mx_hub->dev);
	mutex_destroy(&mx_hub->iolock);
	wake_lock_destroy(&mx_hub->wake_lock);
	i2c_set_clientdata(client, NULL);
	kfree(mx_hub);
	
exit_regulator:
	if ( (!IS_ERR(regulator)) )
	{	
		// set the power & reset pin for program by JTAG
		regulator_set_voltage(regulator,1850000,1850000);
		regulator_put(regulator);
		
		s3c_gpio_setpull(pdata->gpio_reset, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(pdata->gpio_reset, S3C_GPIO_INPUT);
	}
	
	if ( (!IS_ERR(regulator_sen)) )
	{	
		regulator_disable(regulator_sen);	
		regulator_put(regulator_sen);		
	}	
	
	if (pdata->gpio_wake)
		gpio_free(pdata->gpio_wake);
exit_gpio_rst:		
	if (pdata->gpio_reset)
		gpio_free(pdata->gpio_reset);
exit:	
	dev_info(&client->dev,"%s: err  %d --\n",__func__,ret);
	return ret;
}

#ifdef CONFIG_PM
static int mx_hub_suspend(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct mx_hub_dev *hub = i2c_get_clientdata(i2c);

	disable_irq(hub->irq);
	if (device_may_wakeup(dev))
		enable_irq_wake(hub->irq);

	//mx_hub_wakeup(hub,false);
	
#ifdef	TEST_REST_MODE
	if( test.test_reset_enable)
		cancel_delayed_work_sync(&test.test_reset_work);
#endif

	return 0;
}

static int mx_hub_resume(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct mx_hub_dev *hub = i2c_get_clientdata(i2c);
	
	
#ifdef	TEST_REST_MODE
	if( test.test_reset_enable)
		schedule_delayed_work( &test.test_reset_work, msecs_to_jiffies(500));
#endif

	//mx_hub_wakeup(hub,true);
	
	if (device_may_wakeup(dev))
		disable_irq_wake(hub->irq);

	enable_irq(hub->irq);

	return mx_hub_irq_resume(hub);
}

#else
#define mx_hub_suspend	NULL
#define mx_hub_resume	NULL
#endif /* CONFIG_PM */

const struct dev_pm_ops mx_hub_pm = {
	.suspend = mx_hub_suspend,
	.resume = mx_hub_resume,
};

static int __devexit mx_hub_remove(struct i2c_client *client)
{
	struct mx_hub_dev *hub = i2c_get_clientdata(client);
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&hub->early_suspend);
#endif

	mutex_destroy(&hub->iolock);
	wake_lock_destroy(&hub->wake_lock);
	mx_hub_destroy_atts(&client->dev);
	
	mfd_remove_devices(hub->dev);
	mx_hub_irq_exit(hub);

	kfree(hub);
	
	if (hub->gpio_wake)
		gpio_free(hub->gpio_wake);
	
	if (hub->gpio_reset)
		gpio_free(hub->gpio_reset);

	i2c_set_clientdata(client, NULL);

	return 0;
}

void mx_hub_shutdown(struct i2c_client *client)
{
	int ret = 0;
	struct mx_hub_dev *hub = i2c_get_clientdata(client);
	
	disable_irq(hub->irq);

	if (!IS_ERR( hub->regulator))
	{
		ret = regulator_disable(hub->regulator);
		regulator_put(hub->regulator);
	}
}

static const struct i2c_device_id mx_hub_id[] = {
	{ "mx_hub", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, mx_hub_id);

static struct i2c_driver mx_hub_driver = {
	.driver	= {
		.name	= "mx_hub",
		.owner	= THIS_MODULE,
		.pm = &mx_hub_pm,
	},
	.id_table	= mx_hub_id,
	.probe		= mx_hub_probe,
	.remove		= __devexit_p(mx_hub_remove),
	.shutdown	= mx_hub_shutdown,
};

static int __init mx_hub_init(void)
{
	return i2c_add_driver(&mx_hub_driver);
}
module_init(mx_hub_init);

static void __exit mx_hub_exit(void)
{
	i2c_del_driver(&mx_hub_driver);
}
module_exit(mx_hub_exit);


MODULE_DESCRIPTION("MEIZU mx sensor hub driver");
MODULE_AUTHOR("Chwei <Chwei@meizu.com>");
MODULE_LICENSE("GPL");
