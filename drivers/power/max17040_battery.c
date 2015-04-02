/*
 *  max17040_battery.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 *
 *  Copyright (C) 2009 Samsung Electronics
 *  Minkyu Kang <mk7.kang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/max17040_battery.h>
#include <linux/slab.h>
#include <plat/gpio-cfg.h>
#include <mach/regs-gpio.h>
#include <linux/regulator/consumer.h>

#define MAX17040_DBG_MSG	1
#define MAX17040_LOWVOL_TEST	0

#define MAX17040_VCELL_MSB	0x02
#define MAX17040_VCELL_LSB	0x03
#define MAX17040_SOC_MSB	0x04
#define MAX17040_SOC_LSB	0x05
#define MAX17040_MODE_MSB	0x06
#define MAX17040_MODE_LSB	0x07
#define MAX17040_VER_MSB	0x08
#define MAX17040_VER_LSB	0x09
#define MAX17040_RCOMP_MSB	0x0C
#define MAX17040_RCOMP_LSB	0x0D
#define MAX17040_CMD_MSB	0xFE
#define MAX17040_CMD_LSB	0xFF

#define MAX17040_RETRY_COUNT	3
#define MAX17040_DELAY		1000
#define MAX17040_BATTERY_FULL	98


static int g_batt_voltage = 3850;

struct max17040_chip {
	struct i2c_client		*client;
};
static struct max17040_chip *this_chip = NULL;
static int s5p_battery_initial = 0;

static int max17040_i2c_rx_data(char *buf, int len)
{
	uint8_t i;
	struct i2c_client *max17040_client = this_chip->client;
	struct i2c_msg msgs[] = {
		{
			.addr	= max17040_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
		},
		{
			.addr	= max17040_client->addr,
			.flags	= I2C_M_RD,
			.len	= len,
			.buf	= buf,
		}
	};

	if(NULL == this_chip || NULL == max17040_client || 0 == max17040_client->addr)
		return -EIO;

	for (i = 0; i < MAX17040_RETRY_COUNT; i++) {
		if (i2c_transfer(max17040_client->adapter, msgs, 2) > 0) {
			break;
		}
		mdelay(10);
	}

	if (i >= MAX17040_RETRY_COUNT) {
		pr_err("%s: retry over %d\n", __FUNCTION__, MAX17040_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int max17040_i2c_tx_data(char *buf, int len)
{
	uint8_t i;
	struct i2c_client *max17040_client = this_chip->client;
	struct i2c_msg msg[] = {
		{
			.addr	= max17040_client->addr,
			.flags	= 0,
			.len	= len,
			.buf	= buf,
		}
	};
	if(NULL == this_chip || NULL == max17040_client || 0 == max17040_client->addr)
		return -EIO;

	for (i = 0; i < 1/*MAX17040_RETRY_COUNT*/; i++) {
		if (i2c_transfer(max17040_client->adapter, msg, 1) > 0) {
			break;
		}
		mdelay(10);
	}

	if (i >= MAX17040_RETRY_COUNT) {
		pr_err("%s: retry over %d\n", __FUNCTION__, MAX17040_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

#if 0
static int max17040_update_rcomp(int temp)
{
	unsigned char data[4] = {0,0,0,0};
	const int StartingRCOMP = 92;// 20 C = 92, liang, 2011-3-29
	int NewRCOMP;
	int ret;

	if(temp > 20)
		NewRCOMP = StartingRCOMP - (temp-20)*7/8;// -0.875
	else if(temp < 20)
		NewRCOMP = StartingRCOMP + (20-temp)*267/40;// -6.675
	else
		NewRCOMP = StartingRCOMP;

	if(NewRCOMP > 255)
		NewRCOMP = 255;
	else if(NewRCOMP < 0)
		NewRCOMP = 0;

	data[0] = MAX17040_RCOMP_MSB;
	data[1] = NewRCOMP;
	data[2] = 0x00;
	ret = max17040_i2c_tx_data(data,3);
	if(ret)
	{
		printk("%s() error\n",__FUNCTION__);
	}
	return ret;
}

#endif
struct module_config {
	unsigned char addr;
	unsigned char content[16];
};

struct module_config configs[] = { // for v8
#if 1// 2011.3.24 robin
	{0x40,{0xA8,0x00,0xB7,0xF0,0xB9,0x70,0xBB,0x60,0xBC,0x20,0xBD,0x30,0xBD,0xE0,0xBE,0x90}},
	{0x50,{0xC1,0x70,0xC3,0x50,0xC5,0xB0,0xC7,0xE0,0xCA,0x20,0xCD,0x70,0xCF,0x80,0xD2,0xA0}},
	{0x60,{0x02,0xF0,0x2D,0x20,0x25,0x30,0x46,0x00,0x40,0x50,0x44,0xF0,0x44,0xF0,0x20,0xF0}},
	{0x70,{0x17,0xB0,0x17,0xA0,0x1B,0xF0,0x0D,0xC0,0x0E,0x50,0x1D,0x60,0x00,0xF0,0x00,0xF0}},
#endif
};

static int max17040_load_model(void)
{
	unsigned char data[20];
	u8 OriginalRCOMP1,OriginalRCOMP2,OriginalOCV1,OriginalOCV2;
	int ret;
	unsigned char *config;

	memset(data,0,sizeof(data));

	// Unlock Model Access
	data[0] = 0x3E;// OCV
	data[1] = 0x4A;
	data[2] = 0x57;
	ret = max17040_i2c_tx_data(data,3);
	if(ret)
	{
		printk("%s() error1\n",__FUNCTION__);
		return ret;
	}

	// Read RCOMP and OCV
	data[0] = MAX17040_RCOMP_MSB;
	ret = max17040_i2c_rx_data(data,4);
	if(ret)
	{
		printk("%s() error2\n",__FUNCTION__);
		return ret;
	}
	OriginalRCOMP1 = data[0];
	OriginalRCOMP2 = data[1];
	OriginalOCV1 = data[2];
	OriginalOCV2 = data[3];

	// Write OCV
	data[0] = 0x0E;
	data[1] = 0xDC;
	data[2] = 0xA0;
	ret = max17040_i2c_tx_data(data,3);
	if(ret)
	{
		printk("%s() error3\n",__FUNCTION__);
		return ret;
	}

	// Write RCOMP to a Maximum value of 0xFF00h
	data[0] = MAX17040_RCOMP_MSB;
	data[1] = 0xFF;
	data[2] = 0x00;
	ret = max17040_i2c_tx_data(data,3);
	if(ret)
	{
		printk("%s() error4\n",__FUNCTION__);
		return ret;
	}

	// Write the Model
	config = &(configs[0].addr);// 0x40
	ret = max17040_i2c_tx_data(config,17);
	if(ret)
	{
		printk("%s() error5\n",__FUNCTION__);
		return ret;
	}
	config = &(configs[1].addr);// 0x50
	ret = max17040_i2c_tx_data(config,17);
	if(ret)
	{
		printk("%s() error6\n",__FUNCTION__);
		return ret;
	}
	config = &(configs[2].addr);// 0x60
	ret = max17040_i2c_tx_data(config,17);
	if(ret)
	{
		printk("%s() error7\n",__FUNCTION__);
		return ret;
	}
	config = &(configs[3].addr);// 0x70
	ret = max17040_i2c_tx_data(config,17);
	if(ret)
	{
		printk("%s() error8\n",__FUNCTION__);
		return ret;
	}

	msleep(150);

	// Write OCV
	data[0] = 0x0E;
	data[1] = 0xDC;
	data[2] = 0xA0;
	ret = max17040_i2c_tx_data(data,3);
	if(ret)
	{
		printk("%s() error9\n",__FUNCTION__);
		return ret;
	}

	msleep(200);

	// Read SOC and Compare to expected result
	data[0] = MAX17040_SOC_MSB;
	ret = max17040_i2c_rx_data(data,2);
	if(ret)
	{
		printk("%s() error10\n",__FUNCTION__);
		return ret;
	}
	if(data[0] >= 0xC9 && data[0] <=0xCB)
	{
		//printk("%s success, 0x%x\n",__FUNCTION__,data[0]);
		NULL;
	}
	else
	{
		printk("%s fail, 0x%X\n",__FUNCTION__,data[0]);
		return -1;
	}

	// Restore RCOMP and OCV
	data[0] = MAX17040_RCOMP_MSB;
	data[1] = OriginalRCOMP1;
	data[2] = OriginalRCOMP2;
	data[3] = OriginalOCV1;
	data[4] = OriginalOCV2;
	ret = max17040_i2c_tx_data(data,5);
	if(ret)
	{
		printk("%s() error11\n",__FUNCTION__);
		return ret;
	}

	// Lock Model Access
	data[0] = 0x3E;
	data[1] = 0x00;
	data[2] = 0x00;
	ret = max17040_i2c_tx_data(data,3);
	if(ret)
	{
		printk("%s() error12\n",__FUNCTION__);
		return ret;
	}

	return ret;
}

static int max17040_reset(void)
{
	unsigned char data[4] = {0,0,0,0};
	int ret;

	data[0] = MAX17040_CMD_MSB;
	data[1] = 0x54;
	data[2] = 0x00;
	ret = max17040_i2c_tx_data(data,3);
	if(ret)
		printk("%s() error\n",__FUNCTION__);
	return ret;
}

static int max17040_quick_start(void)
{
	unsigned char data[4] = {0,0,0,0};
	int ret;

	data[0] = MAX17040_MODE_MSB;
	data[1] = 0x40;
	data[2] = 0x00;
	ret = max17040_i2c_tx_data(data,3);
	if(ret)
		printk("%s() error\n",__FUNCTION__);
	return ret;
}

static int max17040_get_vcell(void)
{
	unsigned char data[4] = {0,0,0,0};
	u8 msb;
	u8 lsb;
	int unit;
	int ret;

	data[0] = MAX17040_VCELL_MSB;
	ret = max17040_i2c_rx_data(data,2);
	if(ret)
	{
		printk("%s() error\n",__FUNCTION__);
		return ret;
	}
	msb = data[0];
	lsb = data[1];
	//--------------------------------------------------
	//     MSB--ADDRESS 02h      |   LSB--ADDRESS 03H  |
	// ^11 ^10 ^9 ^8 ^7 ^6 ^5 ^4 | ^3 ^2 ^1 ^0 0 0 0 0 |
	//--------------------------------------------------
	unit = (msb << 4) + (lsb >> 4);
#if 1// 1.25mV for MAX17040
	ret = (5*unit)>>2;
#else// 2.5mV for MAX17041
	ret = (5*unit)>>1;
#endif
	//this_chip->bat_info.vcell = (ret+0)*1000;// bias
#if MAX17040_DBG_MSG
//	printk("%s(): vcell: %dmV\n",__FUNCTION__,ret);
#endif
	return ret;

}

int max17040_get_soc(void)
{
	unsigned char data[4] = {0,0,0,0};
	u8 msb;
	u8 lsb;
	int ret;
	int timeout=0;

	data[0] = MAX17040_SOC_MSB;
	ret = max17040_i2c_rx_data(data,2);
	if(ret)
	{
		printk("%s() error\n",__FUNCTION__);
		return ret;
	}
	msb = data[0];
	lsb = data[1];

	//------------------------------------------------------------
	//                  DATESHEET of MAX17040                    |
	//------------------------------------------------------------
	//     MSB--ADDRESS 04h    |        LSB--ADDRESS 05H         |
	// ^7 ^6 ^5 ^4 ^3 ^2 ^1 ^0 | ^-1 ^-2 ^-3 ^-4 ^-5 ^-6 ^-7 ^-8 |
	//------------------------------------------------------------
#if 1 // 2011 -03-24 
	/* The custom model for this cell requires a change in the LSB value
	 * from the datasheet specification. Nominally, the LSB has a value
	 * of 2^-8, but for this custom model, the LSB has been shifted by one
	 * bit so that it now has a value of 2^-9.
	 * SOCValue = ((SOC1 * 256) + SOC2) * 0.001953125
	 */
	ret = msb>>1;
#else// for ZTE V8
	ret = msb;
#endif
	//this_chip->bat_info.soc = ret;
//#if MAX17040_DBG_MSG
//	printk("%s(): soc: %d.%02d%%\n",__FUNCTION__,ret,((((msb & 0x1)<<8)+lsb)*100)>>9);
//#endif
	if (ret < 0)
	{
#if MAX17040_DBG_MSG
		printk("percent:%d%%\n",ret);
#endif
		ret = 5;
	}
	else if (ret > 100)
	{
		while(ret >100 && timeout <15)
		{

			data[0] = MAX17040_SOC_MSB;
			ret = max17040_i2c_rx_data(data,2);
			if(ret)
			{
				printk("%s() error\n",__FUNCTION__);
				return ret;
			}
			msb = data[0];
			lsb = data[1];
			ret = msb>>1;
			timeout ++;
		}

		if(timeout >= 15)
			ret = 99;
	}
	return ret;
}

/* exteral inferface */

#define BATTERY_STORE_CNT 5
int battery_curr_index = 0;
int battery_store[BATTERY_STORE_CNT] = {3900,3900,3900,3900,3900};

static int battery_init = 0;
int smb349_bat_get_vol(void)
{	
	int i, sum_voltage=0;
	int bat_vol = max17040_get_vcell();
	int bat_offset = (bat_vol/100)*2;
	if(bat_offset >= 20) 
		bat_offset = 20;
	bat_vol = bat_vol + bat_offset;

	if(bat_vol >= 4200)
		bat_vol = 4200;
	else if(bat_vol <= 3500) {
		bat_vol = 3650;
	}

  if(battery_init == 0) {
  	battery_init = 1;
  	for(i=0;i<BATTERY_STORE_CNT;i++) {
  		battery_store[i] = bat_vol;
  	}
  }
  else 
		battery_store[battery_curr_index] = bat_vol;

	battery_curr_index++;
	if(battery_curr_index==BATTERY_STORE_CNT)
		battery_curr_index=0;
				
	for(i=0;i<BATTERY_STORE_CNT;i++)	{
	  sum_voltage += battery_store[i];
	}
	bat_vol = sum_voltage/BATTERY_STORE_CNT;

	g_batt_voltage = bat_vol;
	
#if MAX17040_DBG_MSG
	printk("%s(): vcell: %dmV, bat_offset: %dmV\n",__FUNCTION__,bat_vol,bat_offset);
#endif

	return bat_vol;
}

extern int read_charger_status(void);
int smb349_bat_get_soc(void)
{	
	unsigned int bat_soc = max17040_get_soc();
  unsigned int org_soc = bat_soc;
  
	if(g_batt_voltage >= 4200)
		g_batt_voltage = 4200;
	else if(g_batt_voltage <= 3500) {
		g_batt_voltage = 3650;
	}
  
	if(bat_soc >= 99 && g_batt_voltage >= 4180){
		bat_soc = 100;
	}
	else {
		if(read_charger_status()==1){
			if(g_batt_voltage<=4050 && g_batt_voltage>=3790)
				g_batt_voltage = g_batt_voltage - 100;
		}

		if(g_batt_voltage >= 3680) 
			bat_soc = (g_batt_voltage)*16/100-572;
		else
			bat_soc = (g_batt_voltage)*3/10-1089;
	}

#if MAX17040_DBG_MSG
	printk("%s(): org_soc: %d, cal_soc: %d \n",__FUNCTION__,org_soc,bat_soc);
#endif
	return bat_soc;
}


int pm2301_bat_get_vol(void)
{	
	return max17040_get_vcell( );
}

int pm2301_bat_get_soc(void)
{	
	return max17040_get_soc();
}

static int max17040_get_version(void)
{
	unsigned char data[4] = {0,0,0,0};
	u8 msb;
	u8 lsb;
	int ret;

	data[0] = MAX17040_VER_MSB;
	ret = max17040_i2c_rx_data(data,2);
	if(ret)
	{
		printk("%s() error\n",__FUNCTION__);
		return ret;
	}
	msb = data[0];
	lsb = data[1];

	printk("MAX17040 Fuel-Gauge Ver %d%d\n", msb, lsb);
	return ret;
}


static int __devinit max17040_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct max17040_chip *chip;
	int ret;

	printk("[max17040] %s: addr=0x%x @ IIC=%d, irq=%d\n",client->name,client->addr,client->adapter->nr,client->irq);
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))//I2C_FUNC_SMBUS_BYTE
		return -EIO;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	this_chip = chip;
	chip->client = client;

	i2c_set_clientdata(client, chip);

	ret = max17040_reset();
	if (ret)
		return ret;
	ret = max17040_load_model();
	if (ret)
		return ret;
	ret = max17040_quick_start();
	if (ret)
		return ret;
	ret = max17040_get_version();
	if (ret)
		return ret;
	s5p_battery_initial = 1;

	return 0;
}

static int __devexit max17040_remove(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

	i2c_set_clientdata(client, NULL);
	kfree(chip);
	return 0;
}

static const struct i2c_device_id max17040_id[] = {
	{ "max17040", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17040_id);

static struct i2c_driver max17040_i2c_driver = {
	.driver	= {
		.name	= "max17040",
	},
	.probe		= max17040_probe,
	.remove		= __devexit_p(max17040_remove),
	.id_table	= max17040_id,
};

static int __init max17040_init(void)
{
	printk("MAX17040 Fuel Gauge driver: initialize\n");
	return i2c_add_driver(&max17040_i2c_driver);
}
module_init(max17040_init);

static void __exit max17040_exit(void)
{
	i2c_del_driver(&max17040_i2c_driver);
}
module_exit(max17040_exit);

MODULE_AUTHOR("Minkyu Kang <mk7.kang@samsung.com>");
MODULE_DESCRIPTION("MAX17040 Fuel Gauge");
MODULE_LICENSE("GPL");



