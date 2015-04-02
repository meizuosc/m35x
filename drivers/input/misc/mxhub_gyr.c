/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
 *
 * File Name		: lsm330dlc_gyr_sysfs.c
 * Authors		: MEMS Motion Sensors Products Div- Application Team
 *			: Matteo Dameno (matteo.dameno@st.com)
 *			: Carmine Iascone (carmine.iascone@st.com)
 *			: Both authors are willing to be considered the contact
 *			: and update points for the driver.
 * Version		: V 1.1.5.5 sysfs
 * Date			: 2012/Mar/30
 * Description		: LSM330DLC digital output gyroscope sensor API
 *
 ********************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 *******************************************************************************/
	 
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/input-polldev.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/delay.h>
#include <linux/major.h>
//#include <linux/earlysuspend.h>

#include	<linux/mfd/mx-hub.h>
#include	"mxhub_gyr.h"


/** Maximum polled-device-reported rot speed value value in dps*/
#define FS_MAX		32768

/* lsm330dlc gyroscope registers */
#define WHO_AM_I	(0x0F)

#define CTRL_REG1	(0x20)    /* CTRL REG1 */
#define CTRL_REG2	(0x21)    /* CTRL REG2 */
#define CTRL_REG3	(0x22)    /* CTRL_REG3 */
#define CTRL_REG4	(0x23)    /* CTRL_REG4 */
#define CTRL_REG5	(0x24)    /* CTRL_REG5 */
#define	REFERENCE	(0x25)    /* REFERENCE REG */
#define STATUS_REG      0x27
#define	FIFO_CTRL_REG	(0x2E)    /* FIFO CONTROL REGISTER */
#define FIFO_SRC_REG	(0x2F)    /* FIFO SOURCE REGISTER */
#define	OUT_X_L		(0x28)    /* 1st AXIS OUT REG of 6 */

#define AXISDATA_REG	OUT_X_L

/* CTRL_REG1 */
#define ALL_ZEROES	(0x00)
#define PM_OFF		(0x00)
#define PM_NORMAL	(0x08)
#define ENABLE_ALL_AXES	(0x07)
#define ENABLE_NO_AXES	(0x00)
#define BW00		(0x00)
#define BW01		(0x10)
#define BW10		(0x20)
#define BW11		(0x30)
#define ODR095		(0x00)  /* ODR =  95Hz */
#define ODR190		(0x40)  /* ODR = 190Hz */
#define ODR380		(0x80)  /* ODR = 380Hz */
#define ODR760		(0xC0)  /* ODR = 760Hz */

/* CTRL_REG3 bits */
#define	I2_DRDY		(0x08)
#define	I2_WTM		(0x04)
#define	I2_OVRUN	(0x02)
#define	I2_EMPTY	(0x01)
#define	I2_NONE		(0x00)
#define	I2_MASK		(0x0F)

/* CTRL_REG4 bits */
#define	FS_MASK		(0x30)
#define	BDU_ENABLE	(0x80)

/* CTRL_REG5 bits */
#define	FIFO_ENABLE	(0x40)
#define HPF_ENALBE	(0x11)

/* FIFO_CTRL_REG bits */
#define	FIFO_MODE_MASK		(0xE0)
#define	FIFO_MODE_BYPASS	(0x00)
#define	FIFO_MODE_FIFO		(0x20)
#define	FIFO_MODE_STREAM	(0x40)
#define	FIFO_MODE_STR2FIFO	(0x60)
#define	FIFO_MODE_BYPASS2STR	(0x80)
#define	FIFO_WATERMARK_MASK	(0x1F)

#define FIFO_STORED_DATA_MASK	(0x1F)


#define FUZZ			0
#define FLAT			0
#define I2C_AUTO_INCREMENT	(0x80)

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1		0
#define	RES_CTRL_REG2		1
#define	RES_CTRL_REG3		2
#define	RES_CTRL_REG4		3
#define	RES_CTRL_REG5		4
#define	RES_FIFO_CTRL_REG	5
#define	RESUME_ENTRIES		6


/* #define DEBUG 0 */

/** Registers Contents */
#define WHOAMI_LSM330DLC_GYR	(0x00D4)  /* Expected content for WAI register*/

/*
 * LSM330DLC gyroscope data
 * brief structure containing gyroscope values for yaw, pitch and roll in
 * signed short
 */

struct lsm330dlc_gyr_triple {
	short	x,	/* x-axis angular rate data. */
			y,	/* y-axis angluar rate data. */
			z;	/* z-axis angular rate data. */
};

struct output_rate {
	int poll_rate_ms;
	u8 mask;
};

static const struct output_rate odr_table[] = {

	{	2,	ODR760|BW10},
	{	3,	ODR380|BW01},
	{	6,	ODR190|BW00},
	{	11,	ODR095|BW00},
};


struct lsm330dlc_gyr_status {
	struct mx_hub_dev *data;
	int irq;
	
	struct i2c_client *client;
	struct gyr_platform_data *pdata;

	struct mutex lock;

	struct class		*gyroscope;
	struct device		*class_dev;

	struct input_dev		*input_dev;
	struct delayed_work      input_work;

	int hw_initialized;
	atomic_t enabled;

	u8 reg_addr;
	u8 resume_state[RESUME_ENTRIES];

	/* interrupt related */
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;

	/* fifo related */
	u8 watermark;
	u8 fifomode;
/*
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
*/
};


static irqreturn_t lsm330dlc_gyr_isr(int irq, void *dev_id)
{
	struct lsm330dlc_gyr_status *gyr = dev_id;
	//struct regmap *regmap = gyr->iodev->regmap;
	int ret;

	pr_info("ENTER %s:************\n",__func__);

	//wake_lock(&gyr->wake_lock);
	
	ret = gyr->data->irq;

	//wake_unlock(&gyr->wake_lock);

	return IRQ_HANDLED;
}

static int lsm330dlc_gyr_i2c_read(struct lsm330dlc_gyr_status *stat, u8 *buf,
		int len)
{
	int ret;
	u8 reg = buf[0] & (~I2C_AUTO_INCREMENT);
	//u8 cmd = reg;

	//if (len > 1)
	//	cmd = (I2C_AUTO_INCREMENT | reg);

	reg +=MX_HUB_REG_GYR_BASE;	

	ret = mx_hub_readdata_rdy(stat->client,reg,len,buf);
	
	return ret;
}


static int lsm330dlc_gyr_i2c_write(struct lsm330dlc_gyr_status *stat, u8 *buf,
		int len)
{
	int ret;
	u8 reg;


	reg = buf[0] & (~I2C_AUTO_INCREMENT);
	reg += MX_HUB_REG_GYR_BASE;
	buf[0] = reg;
#if 1	
	ret = mx_hub_writedata_rdy(stat->client,reg ,len,&buf[1]);
	return ret;
#else
	
	ret = i2c_master_send(stat->client, buf, len+1);
	msleep(10);
	return (ret == len+1) ? 0 : ret;
#endif
}


static int lsm330dlc_gyr_register_write(struct lsm330dlc_gyr_status *stat,
		u8 *buf, u8 reg_address, u8 new_value)
{
	int err;

	/* Sets configuration register at reg_address
	 *  NOTE: this is a straight overwrite  */
	buf[0] = reg_address;
	buf[1] = new_value;
	err = lsm330dlc_gyr_i2c_write(stat, buf, 1);
	if (err < 0)
		return err;

	return err;
}

static int lsm330dlc_gyr_register_read(struct lsm330dlc_gyr_status *stat,
		u8 *buf, u8 reg_address)
{

	int err = -1;
	buf[0] = (reg_address);
	err = lsm330dlc_gyr_i2c_read(stat, buf, 1);
	return err;
}

static int lsm330dlc_gyr_register_update(struct lsm330dlc_gyr_status *stat,
		u8 *buf, u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 init_val;
	u8 updated_val;
	err = lsm330dlc_gyr_register_read(stat, buf, reg_address);
	if (!(err < 0)) {
		init_val = buf[0];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		err = lsm330dlc_gyr_register_write(stat, buf, reg_address,
				updated_val);
	}
	return err;
}

static int lsm330dlc_gyr_update_fs_range(struct lsm330dlc_gyr_status *stat,
		u8 new_fs)
{
	int res ;
	u8 buf[2];

	dev_dbg(&stat->client->dev, "%s called\n", __func__);
	buf[0] = CTRL_REG4;

	res = lsm330dlc_gyr_register_update(stat, buf, CTRL_REG4,
			FS_MASK, new_fs);

	if (res < 0) {
		dev_err(&stat->client->dev, "%s : failed to update fs:0x%02x\n",
				__func__, new_fs);
		return res;
	}
	stat->resume_state[RES_CTRL_REG4] =
		((FS_MASK & new_fs) |
		 (~FS_MASK & stat->resume_state[RES_CTRL_REG4]));

	return res;
}


static int lsm330dlc_gyr_update_odr(struct lsm330dlc_gyr_status *stat,
		unsigned int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config[2];

	dev_dbg(&stat->client->dev, "%s called\n", __func__);
	for (i = ARRAY_SIZE(odr_table) - 1; i >= 0; i--) {
		if ((odr_table[i].poll_rate_ms <= poll_interval_ms) || (i == 0))
			break;
	}

	config[1] = odr_table[i].mask;
	config[1] |= (ENABLE_ALL_AXES + PM_NORMAL);

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&stat->enabled)) {
		config[0] = CTRL_REG1;
		err = lsm330dlc_gyr_i2c_write(stat, config, 1);
		if (err < 0)
			return err;
		stat->resume_state[RES_CTRL_REG1] = config[1];
	}


	return err;
}

/* gyroscope data readout */
static int lsm330dlc_gyr_get_data(struct lsm330dlc_gyr_status *stat,
		struct lsm330dlc_gyr_triple *data)
{
	int err;
	unsigned char gyro_out[6];
	/* y,p,r hardware data */
	s16 hw_d[3] = { 0 };

	gyro_out[0] = (AXISDATA_REG);

	err = lsm330dlc_gyr_i2c_read(stat, gyro_out, 6);

	if (err < 0)
		return err;

	hw_d[0] = (s16) (((gyro_out[1]) << 8) | gyro_out[0]);
	hw_d[1] = (s16) (((gyro_out[3]) << 8) | gyro_out[2]);
	hw_d[2] = (s16) (((gyro_out[5]) << 8) | gyro_out[4]);

	data->x = ((stat->pdata->negate_x) ? (-hw_d[stat->pdata->axis_map_x])
			: (hw_d[stat->pdata->axis_map_x]));
	data->y = ((stat->pdata->negate_y) ? (-hw_d[stat->pdata->axis_map_y])
			: (hw_d[stat->pdata->axis_map_y]));
	data->z = ((stat->pdata->negate_z) ? (-hw_d[stat->pdata->axis_map_z])
			: (hw_d[stat->pdata->axis_map_z]));
	return err;
}

static void lsm330dlc_gyr_report_values(struct lsm330dlc_gyr_status *stat,
		struct lsm330dlc_gyr_triple *data)
{
	struct input_dev *input = stat->input_dev;
	input_report_abs(input, ABS_X, data->x);
	input_report_abs(input, ABS_Y, data->y);
	input_report_abs(input, ABS_Z, data->z);
	//	pr_info("gyr_triple: x= %d, y = %d, z = %d", data->x, data->y, data->z);
	input_sync(input);
}

static int lsm330dlc_gyr_hw_init(struct lsm330dlc_gyr_status *stat)
{
	int err;
	u8 buf[6];

	dev_dbg(&stat->client->dev, "hw init\n");

	buf[0] = (CTRL_REG1);
	buf[1] = stat->resume_state[RES_CTRL_REG1];
	buf[2] = stat->resume_state[RES_CTRL_REG2];
	buf[3] = stat->resume_state[RES_CTRL_REG3];
	buf[4] = stat->resume_state[RES_CTRL_REG4];
	buf[5] = stat->resume_state[RES_CTRL_REG5];

	err = lsm330dlc_gyr_i2c_write(stat, buf, 5);
	if (err < 0)
		return err;

	msleep(10);
	buf[0] = (FIFO_CTRL_REG);
	buf[1] = stat->resume_state[RES_FIFO_CTRL_REG];
	err = lsm330dlc_gyr_i2c_write(stat, buf, 1);
	if (err < 0){
		pr_info("func :%s failed\n", __func__);
		return err;
	}

	stat->hw_initialized = 1;

	msleep(50);
	dev_dbg(&stat->client->dev, " poll_interval %d ms\n",stat->pdata->poll_interval);
	lsm330dlc_gyr_update_odr(stat,stat->pdata->poll_interval);

	return err;
}

static void lsm330dlc_gyr_device_power_off(struct lsm330dlc_gyr_status *stat)
{
	int err;
	u8 buf[2];

	dev_dbg(&stat->client->dev, "power off\n");

	buf[0] = (CTRL_REG1);
	buf[1] = (PM_OFF);
	err = lsm330dlc_gyr_i2c_write(stat, buf, 1);
	if (err < 0)
		dev_err(&stat->client->dev, "soft power off failed\n");

	if (stat->pdata->power_off) {
		stat->pdata->power_off();
		stat->hw_initialized = 0;
	}

	if (stat->hw_initialized) {
		stat->hw_initialized = 0;
	}
}

static int lsm330dlc_gyr_device_power_on(struct lsm330dlc_gyr_status *stat)
{
	int err;

	dev_dbg(&stat->client->dev, "power on\n");
	if (stat->pdata->power_on) {
		err = stat->pdata->power_on();
		if (err < 0)
			return err;
	}

	if (!stat->hw_initialized) {
		int i;
		for (i = 0; i < 5; i++)
		{
			err = lsm330dlc_gyr_hw_init(stat);
			if (err < 0)
				msleep(100);
			else
				break;
		}
		//err = lsm330dlc_gyr_hw_init(stat);
		if (err < 0) {
			lsm330dlc_gyr_device_power_off(stat);
			return err;
		}
	}
	return 0;
}

static int lsm330dlc_gyr_enable(struct lsm330dlc_gyr_status *stat)
{
	int err = -1;

	pr_info("%s called\n", __func__);
	if (!atomic_cmpxchg(&stat->enabled, 0, 1)) {
		err = lsm330dlc_gyr_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled, 0);
			return err;
		}
		schedule_delayed_work(&stat->input_work, msecs_to_jiffies(stat->pdata->poll_interval));
	}

	return 0;
}

static int lsm330dlc_gyr_disable(struct lsm330dlc_gyr_status *stat)
{
	pr_info("%s called\n", __func__);
	if (atomic_cmpxchg(&stat->enabled, 1, 0)) {		
		cancel_delayed_work_sync(&stat->input_work);
		lsm330dlc_gyr_device_power_off(stat);
	}

	return 0;
}

static ssize_t attr_polling_rate_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int val;
	struct lsm330dlc_gyr_status *stat = dev_get_drvdata(dev);
	mutex_lock(&stat->lock);
	val = stat->pdata->poll_interval;
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_polling_rate_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct lsm330dlc_gyr_status *stat = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	//	pr_info("poll rate set from sysfs interval_ms = %ld \n", interval_ms);
	interval_ms = max((unsigned int)interval_ms, stat->pdata->min_interval);
	mutex_lock(&stat->lock);
	if (atomic_read(&stat->enabled))
		stat->pdata->poll_interval = interval_ms;
	stat->pdata->poll_interval = interval_ms;
	lsm330dlc_gyr_update_odr(stat, interval_ms);
	mutex_unlock(&stat->lock);
	return size;
}

static ssize_t attr_range_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct lsm330dlc_gyr_status *stat = dev_get_drvdata(dev);
	int range = 0;
	u8 val;
	mutex_lock(&stat->lock);
	val = stat->pdata->fs_range;

	switch (val) {
		case LSM330DLC_GYR_FS_250DPS:
			range = 250;
			break;
		case LSM330DLC_GYR_FS_500DPS:
			range = 500;
			break;
		case LSM330DLC_GYR_FS_2000DPS:
			range = 2000;
			break;
	}
	mutex_unlock(&stat->lock);
	/* return sprintf(buf, "0x%02x\n", val); */
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_range_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct lsm330dlc_gyr_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	u8 range;
	int err;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	switch (val) {
		case 250:
			range = LSM330DLC_GYR_FS_250DPS;
			break;
		case 500:
			range = LSM330DLC_GYR_FS_500DPS;
			break;
		case 2000:
			range = LSM330DLC_GYR_FS_2000DPS;
			break;
		default:
			dev_err(&stat->client->dev, "invalid range request: %lu,"
					" discarded\n", val);
			return -EINVAL;
	}
	mutex_lock(&stat->lock);
	err = lsm330dlc_gyr_update_fs_range(stat, range);
	if (err >= 0)
		stat->pdata->fs_range = range;
	mutex_unlock(&stat->lock);
	dev_dbg(&stat->client->dev, "range set to: %lu dps\n", val);
	return size;
}

static ssize_t attr_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct lsm330dlc_gyr_status *stat = dev_get_drvdata(dev);
	int val = atomic_read(&stat->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct lsm330dlc_gyr_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	//	pr_info("gyroscope enable = %ld from sysfs \n",val);
	if (val)
		lsm330dlc_gyr_enable(stat);
	else
		lsm330dlc_gyr_disable(stat);

	return size;
}

static ssize_t attr_polling_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int val = 0;
	struct lsm330dlc_gyr_status *stat = dev_get_drvdata(dev);

	mutex_lock(&stat->lock);
	if (atomic_read(&stat->enabled))
		val = 1;
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_watermark_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct lsm330dlc_gyr_status *stat = dev_get_drvdata(dev);
	int val = stat->watermark;
	return sprintf(buf, "0x%02x\n", val);
}

static ssize_t attr_fifomode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct lsm330dlc_gyr_status *stat = dev_get_drvdata(dev);
	u8 val = stat->fifomode;
	return sprintf(buf, "0x%02x\n", val);
}

#ifdef DEBUG
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int rc;
	struct lsm330dlc_gyr_status *stat = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&stat->lock);
	x[0] = stat->reg_addr;
	mutex_unlock(&stat->lock);
	x[1] = val;
	rc = lsm330dlc_gyr_i2c_write(stat, x, 1);
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	ssize_t ret;
	struct lsm330dlc_gyr_status *stat = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&stat->lock);
	data = stat->reg_addr;
	mutex_unlock(&stat->lock);
	rc = lsm330dlc_gyr_i2c_read(stat, &data, 1);
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct lsm330dlc_gyr_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&stat->lock);

	stat->reg_addr = val;

	mutex_unlock(&stat->lock);

	return size;
}
#endif /* DEBUG */
/* selftest start */
#define SELFTEST_MEASURE_TIMEOUT 100
#define I2C_RETRY_DELAY 10
#define SELFTEST_ZYXDA (0x1 << 3)
#define SELFTEST_SAMPLES 5

static int selftest_init(struct lsm330dlc_gyr_status *gyr)
{
	unsigned char buf[6];
	//	pr_info("%s\n", __func__);

	/* BDU=1, ODR=200Hz, Cut-Off Freq=50Hz, FS=2000 DPS */
	/* Noted by qudao, for l3gd20:
	 * ODR = 190Hz, Cut-Off Freq = 50Hz, x/y/z anxis enabled, Normal mode, FS = 2000DPS
	 */
	buf[0] = CTRL_REG1;
	buf[1] = 0x6F;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = 0xA0;
	buf[5] = 0x02;

	return lsm330dlc_gyr_i2c_write(gyr, &buf[0], 5);
}

static int selftest_enable(struct lsm330dlc_gyr_status *gyr)
{
	/*
	 * For l3g4200d, Self test 0 (+)
	 *
	 */
	u8 buf[2];
	buf[0] = CTRL_REG4;
	buf[1] = 0xA2;
	//	pr_info("%s\n", __func__);

	return lsm330dlc_gyr_i2c_write(gyr, buf, 1);
}


static void selftest_disable(struct lsm330dlc_gyr_status *gyr)
{
	u8 buf[2];
	buf[1]= 0x00;
	//	pr_info("%s\n", __func__);

	/* Disable sensor */
	buf[0] = CTRL_REG1;
	lsm330dlc_gyr_i2c_write(gyr, buf, 1);
	msleep(10);
	/* Disable selftest */
	buf[0] = CTRL_REG4;
	lsm330dlc_gyr_i2c_write(gyr, buf, 1);
}

static int selftest_wait_ZYXDA(struct lsm330dlc_gyr_status *gyr)
{
	int i, ret;
	unsigned char data_ready;

	pr_info("%s\n", __func__);

	for (i = SELFTEST_MEASURE_TIMEOUT/10; i != 0; i--) {
		data_ready = STATUS_REG;
		ret = lsm330dlc_gyr_i2c_read(gyr, &data_ready, 1);
		if (ret < 0) {
			pr_err("%s: lsm330dlc_gyr_i2c_read fail, retry %d\n", __func__, i);
			msleep(I2C_RETRY_DELAY);
			continue;
		} else if (data_ready & SELFTEST_ZYXDA) {
			pr_info("%s: data ready\n", __func__);
			break;
		}else
		{
			pr_info("%s: 0x%X \n", __func__,data_ready);			
		}
	}
	if (i == 0) {
		pr_err("%s: failed\n", __func__);
		return ret;
	}

	return 0;
}

static int selftest_read(struct lsm330dlc_gyr_status *stat,
		struct lsm330dlc_gyr_triple *data)
{
	int total[3];
	int i, ret;

	//	pr_info("%s\n", __func__);

	total[0] = 0;
	total[1] = 0;
	total[2] = 0;
	for (i = 0; i < SELFTEST_SAMPLES; i++) {
		msleep(10);
		ret = selftest_wait_ZYXDA(stat);
		if (ret) {
			pr_err("%s: selftest_check_ZYXDA fail\n", __func__);
			return ret;
		}
		msleep(10);
		ret = lsm330dlc_gyr_get_data(stat,data);
		if (ret < 0) {
			pr_err("%s: l3gd20_read_gyro_values fail\n", __func__);
			return ret;
		}
		pr_debug("%s: data: x = %d, y = %d, z = %d\n", __func__, data->x, data->y, data->z);
		total[0] += data->x;
		total[1] += data->y;
		total[2] += data->z;
		pr_debug("%s: total: x = %d, y = %d, z = %d\n", __func__, total[0], total[1], total[2]);
	}
	data->x = total[0] / SELFTEST_SAMPLES;
	data->y = total[1] / SELFTEST_SAMPLES;
	data->z = total[2] / SELFTEST_SAMPLES;
	pr_debug("%s: average: x = %d, y = %d, z = %d\n", __func__, data->x, data->y, data->z);

	return 0;
}

/*
 * Part Number Min_X Max_X Min_Y Max_Y Min_Z Max_Z Unit
 * L3G4200D 175  875 175 875 175 875 DPS (@ FS = +/-2000dps)
 */

#define SELFTEST_MIN (175UL * 1000)	/* mdps */
#define SELFTEST_MAX (875UL * 1000)	/* mdps */
#define CONVERT_TO_MDPS	70UL		/* for range = 2000 DPS */

#define SELFTEST_NORMAL(st, nost, axis)			\
	({							\
	 unsigned long __abs_data = abs(st->axis - nost->axis) * CONVERT_TO_MDPS;	\
	 int __ret;					\
	 __ret = (__abs_data <= SELFTEST_MAX) && (__abs_data >= SELFTEST_MIN);	\
	 __ret;									\
	 })

static inline int selftest_check(struct lsm330dlc_gyr_triple *data_nost, struct lsm330dlc_gyr_triple *data_st)
{
	pr_info("%s\n", __func__);
	pr_info("%s:  MAX: %lu, MIN: %lu\n", __func__, SELFTEST_MAX, SELFTEST_MIN);
	pr_info("%s:X: %lu\t", __func__, abs(data_st->x - data_nost->x) * CONVERT_TO_MDPS);
	pr_info("%s:Y: %lu\t", __func__, abs(data_st->y - data_nost->y) * CONVERT_TO_MDPS);
	pr_info("%s:Z: %lu\t", __func__, abs(data_st->z - data_nost->z) * CONVERT_TO_MDPS);

	/* Pass return 0, fail return -1 */
	if (SELFTEST_NORMAL(data_st, data_nost, x) \
			&& SELFTEST_NORMAL(data_st, data_nost, y) \
			&& SELFTEST_NORMAL(data_st, data_nost, z)) {
		return 0;
	}

	return -1;
}

static int lsm330dlc_gyr_selftest_result(struct lsm330dlc_gyr_status *stat,int *test_result)
{
	int ret;
	struct lsm330dlc_gyr_triple data_nost, data_st;

	/* Initialize Sensor, turn on sensor, enable P/R/Y */
	ret = selftest_init(stat);
	if (ret < 0) {
		pr_err("%s: selftest_init fail\n", __func__);
		return ret;
	}
	/* Wait for stable output */
	pr_info("%s: wait for stable output\n", __func__);
	msleep(800);
	/* Read out normal output */
	ret = selftest_read(stat,&data_nost);
	if (ret < 0) {
		pr_err("%s: selftest_read fail\n", __func__);
		return ret;
	}
	pr_info("%s: normal output: x = %d, y = %d, z = %d\n",
			__func__, data_nost.x, data_nost.y, data_nost.z);

	/* Enable self test */
	ret = selftest_enable(stat);
	if (ret < 0) {
		pr_err("%s: selftest_enable failed\n", __func__);
		return ret;
	}
	/* ODR=200HZ, wait for 3 * ODR */
	mdelay(3 * (1000 / 200));
	/* Read out selftest output */
	ret = selftest_read(stat,&data_st);
	if (ret < 0) {
		pr_err("%s: selftest_read fail\n", __func__);
		return ret;
	}
	/* Check output */
	ret = selftest_check(&data_nost, &data_st);
	if (ret < 0) {
		pr_err("%s: ***fail***\n", __func__);
		*test_result = 0;
	} else {
		pr_info("%s: ***success***\n", __func__);
		*test_result = 1;
	}
	/* selftest disable */
	selftest_disable(stat);

	return ret;
}

static ssize_t attr_get_selftest_result(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int test_result;
	struct lsm330dlc_gyr_status *stat = dev_get_drvdata(dev);
	lsm330dlc_gyr_selftest_result(stat, &test_result);	
	return sprintf(buf, "%d\n", test_result);
}

/* selftest end */


static struct device_attribute attributes[] = {
	__ATTR(pollrate_ms, 0644, attr_polling_rate_show,
			attr_polling_rate_store),
	__ATTR(range, 0644, attr_range_show, attr_range_store),
	__ATTR(enable_device, 0644, attr_enable_show, attr_enable_store),
	__ATTR(enable_polling, 0644, attr_polling_mode_show,NULL
			/*	attr_polling_mode_store*/),
	__ATTR(fifo_samples, 0644, attr_watermark_show, NULL/*attr_watermark_store*/),
	__ATTR(fifo_mode, 0644, attr_fifomode_show, NULL/*attr_fifomode_store*/),
	__ATTR(selftest, 0644, attr_get_selftest_result,	NULL),
#ifdef DEBUG
	__ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0200, NULL, attr_addr_set),
#endif
};
static char const *const device_link_name = "i2c";
static dev_t const st_gyr_device_dev_t = MKDEV(MISC_MAJOR, 242);

static int create_sysfs_files(struct device *dev)
{
	int i;

	dev_dbg(dev, "%s\n", __func__);
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for (; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int create_sysfs_interfaces(struct lsm330dlc_gyr_status *stat)
{
	int err;

	if (NULL == stat)
		return -EINVAL;

	err = 0;

	stat->gyroscope= class_create(THIS_MODULE, GYR_SYSCLS_NAME);
	if (IS_ERR(stat->gyroscope)) {
		err = PTR_ERR(stat->gyroscope);
		goto exit_class_create_failed;
	}

	stat->class_dev = device_create(
			stat->gyroscope,
			NULL,
			st_gyr_device_dev_t,
			stat,
			GYR_SYSDEV_NAME);
	if (IS_ERR(stat->class_dev)) {
		err = PTR_ERR(stat->class_dev );
		goto exit_class_device_create_failed;
	}

	err = sysfs_create_link(
			&stat->class_dev->kobj,
			&stat->client->dev.kobj,
			device_link_name);
	if (0 > err)
		goto exit_sysfs_create_link_failed;

	err = create_sysfs_files(stat->class_dev);
	if (0 > err)
		goto exit_device_attributes_create_failed;

	return err;

exit_device_attributes_create_failed:
	sysfs_remove_link(&stat->class_dev->kobj, device_link_name);
exit_sysfs_create_link_failed:
	device_destroy(stat->gyroscope, st_gyr_device_dev_t);
exit_class_device_create_failed:
	stat->class_dev = NULL;
	class_destroy(stat->gyroscope);
exit_class_create_failed:
	stat->gyroscope
		= NULL;
	return err;
}



static int remove_sysfs_files(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}

static void remove_sysfs_interfaces(struct lsm330dlc_gyr_status *stat)
{
	if (NULL == stat)
		return;

	if (NULL != stat->class_dev) {
		remove_sysfs_files(stat->class_dev);
		sysfs_remove_link(
				&stat->class_dev->kobj,
				device_link_name);
		stat->class_dev = NULL;
	}
	if (NULL != stat->gyroscope) {
		device_destroy(
				stat->gyroscope,
				st_gyr_device_dev_t);
		class_destroy(stat->gyroscope);
		stat->gyroscope= NULL;
	}
}

int lsm330dlc_gyr_input_open(struct input_dev *input)
{
	struct lsm330dlc_gyr_status *stat = input_get_drvdata(input);

	dev_info(&stat->client->dev, "%s\n", __func__);
	return lsm330dlc_gyr_enable(stat);
}

void lsm330dlc_gyr_input_close(struct input_dev *dev)
{
	struct lsm330dlc_gyr_status *stat = input_get_drvdata(dev);

	dev_info(&stat->client->dev, "%s\n", __func__);
	lsm330dlc_gyr_disable(stat);
}

static int lsm330dlc_gyr_validate_pdata(struct lsm330dlc_gyr_status *stat)
{
	/* checks for correctness of minimal polling period */
	stat->pdata->min_interval =
		max((unsigned int) LSM330DLC_GYR_MIN_POLL_PERIOD_MS,
				stat->pdata->min_interval);

	stat->pdata->poll_interval = max(stat->pdata->poll_interval,
			stat->pdata->min_interval);

	if (stat->pdata->axis_map_x > 2 ||
			stat->pdata->axis_map_y > 2 ||
			stat->pdata->axis_map_z > 2) {
		dev_err(&stat->client->dev,
				"invalid axis_map value x:%u y:%u z%u\n",
				stat->pdata->axis_map_x,
				stat->pdata->axis_map_y,
				stat->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (stat->pdata->negate_x > 1 ||
			stat->pdata->negate_y > 1 ||
			stat->pdata->negate_z > 1) {
		dev_err(&stat->client->dev,
				"invalid negate value x:%u y:%u z:%u\n",
				stat->pdata->negate_x,
				stat->pdata->negate_y,
				stat->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (stat->pdata->poll_interval < stat->pdata->min_interval) {
		dev_err(&stat->client->dev,
				"minimum poll interval violated\n");
		return -EINVAL;
	}
	return 0;
}

static void lsm330dlc_gyr_input_work_func(struct work_struct *work)
{
	int err;
	struct lsm330dlc_gyr_status *stat;
	struct lsm330dlc_gyr_triple data_out;

	//	pr_info("%s called\n", __func__);
	stat = container_of(work, struct lsm330dlc_gyr_status, input_work.work);

	if(atomic_read(&stat->enabled) == 0){
		pr_info("%s disabled\n", __func__);
		return;
	}

	mutex_lock(&stat->lock);

	err = lsm330dlc_gyr_get_data(stat, &data_out);
	if (err < 0)
		dev_err(&stat->client->dev, "get_gyroscope_data failed\n");
	else
		lsm330dlc_gyr_report_values(stat, &data_out);
	schedule_delayed_work(&stat->input_work, msecs_to_jiffies(
				stat->pdata->poll_interval));

	mutex_unlock(&stat->lock);

}

static int lsm330dlc_gyr_input_init(struct lsm330dlc_gyr_status *stat)
{
	int err;

	dev_dbg(&stat->client->dev,"%s called\n", __func__);
	INIT_DELAYED_WORK(&stat->input_work, lsm330dlc_gyr_input_work_func);
	stat->input_dev = input_allocate_device();
	if (!stat->input_dev) {
		err = -ENOMEM;
		dev_err(&stat->client->dev, "input device allocation failed\n");
		goto err0;
	}

//	stat->input_dev->open = lsm330dlc_gyr_input_open;
	stat->input_dev->close = lsm330dlc_gyr_input_close;
	stat->input_dev->name = LSM330DLC_GYR_DEV_NAME;
	/* stat->input_dev->name = "accelerometer"; */
	stat->input_dev->id.bustype = BUS_I2C;
	stat->input_dev->dev.parent = &stat->client->dev;

	input_set_drvdata(stat->input_dev, stat);

	set_bit(EV_ABS, stat->input_dev->evbit);
	/*	next is used for interruptA sources data if the case */
	set_bit(ABS_MISC, stat->input_dev->absbit);
	/*	next is used for interruptB sources data if the case */
	set_bit(ABS_WHEEL, stat->input_dev->absbit);

	input_set_abs_params(stat->input_dev, ABS_X, -FS_MAX, FS_MAX, FUZZ, FLAT);
	input_set_abs_params(stat->input_dev, ABS_Y, -FS_MAX, FS_MAX, FUZZ, FLAT);
	input_set_abs_params(stat->input_dev, ABS_Z, -FS_MAX, FS_MAX, FUZZ, FLAT);
	/*	next is used for interruptA sources data if the case */
	input_set_abs_params(stat->input_dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
	/*	next is used for interruptB sources data if the case */
	input_set_abs_params(stat->input_dev, ABS_WHEEL, INT_MIN, INT_MAX, 0, 0);

	err = input_register_device(stat->input_dev);
	if (err) {
		dev_err(&stat->client->dev,
				"unable to register input device %s\n",
				stat->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(stat->input_dev);
err0:
	return err;
}

static void lsm330dlc_gyr_input_cleanup(struct lsm330dlc_gyr_status *stat)
{
	input_unregister_device(stat->input_dev);
	input_free_device(stat->input_dev);
}

#define SLEEP
static int lsm330dlc_gyr_suspend(struct device *dev)
{
	int err = 0;
#ifdef CONFIG_PM
	struct lsm330dlc_gyr_status *stat = dev_get_drvdata(dev);
	struct i2c_client *client = stat->client;
	u8 buf[2];

	dev_info(&client->dev, "suspend\n");

	dev_dbg(&client->dev, "%s\n", __func__);
	if (atomic_read(&stat->enabled)) {
		mutex_lock(&stat->lock);
#ifdef SLEEP
		err = lsm330dlc_gyr_register_update(stat, buf, CTRL_REG1,
				0x0F, (ENABLE_NO_AXES | PM_NORMAL));
#else
		err = lsm330dlc_gyr_register_update(stat, buf, CTRL_REG1,
				0x08, PM_OFF);
#endif /*SLEEP*/
		mutex_unlock(&stat->lock);
	}
#endif /*CONFIG_PM*/
	return err;
}

static int lsm330dlc_gyr_resume(struct device *dev)
{
	int err = 0;
#ifdef CONFIG_PM
	struct lsm330dlc_gyr_status *stat = dev_get_drvdata(dev);
	struct i2c_client *client = stat->client;
	u8 buf[2];

	dev_info(&client->dev, "resume\n");

	dev_dbg(&client->dev, "%s\n", __func__);
	if (atomic_read(&stat->enabled)) {
		mutex_lock(&stat->lock);
		dev_info(&stat->client->dev, "polling mode enabled\n");
		schedule_delayed_work(&stat->input_work, msecs_to_jiffies(stat->pdata->poll_interval));
#ifdef SLEEP
		err = lsm330dlc_gyr_register_update(stat, buf, CTRL_REG1,
				0x0F, (ENABLE_ALL_AXES | PM_NORMAL));
#else
		err = lsm330dlc_gyr_register_update(stat, buf, CTRL_REG1,
				0x08, PM_NORMAL);
#endif
		mutex_unlock(&stat->lock);

	}
#endif /*CONFIG_PM*/
	return err;
}
/*
#ifdef CONFIG_HAS_EARLYSUSPEND
static void lsm330dlc_gyr_early_suspend(struct early_suspend *handler)
{
	struct lsm330dlc_gyr_status *stat;
	stat = container_of(handler, struct lsm330dlc_gyr_status, early_suspend);

#if DEBUG
	pr_info("%s\n", __func__);
#endif
	lsm330dlc_gyr_suspend(&stat->client->dev);
}

static void lsm330dlc_gyr_early_resume(struct early_suspend *handler)
{
	struct lsm330dlc_gyr_status *stat;
	stat = container_of(handler, struct lsm330dlc_gyr_status, early_suspend);

#if DEBUG
	pr_info("%s\n", __func__);
#endif
	lsm330dlc_gyr_resume(&stat->client->dev);
}
#endif
*/
static int __devinit lsm330dlc_gyr_probe(struct platform_device *pdev)
{
	struct mx_hub_dev *data = dev_get_drvdata(pdev->dev.parent);
	struct mx_sensor_hub_platform_data *pdata = dev_get_platdata(data->dev);
	struct i2c_client *client;
	struct lsm330dlc_gyr_status *stat;
	int err = -1;

	dev_dbg(&pdev->dev, "probe start.\n");
	
	client = data->client;	

	stat = kzalloc(sizeof(*stat), GFP_KERNEL);
	if (stat == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto exit_check_functionality_failed;
	}

	mutex_init(&stat->lock);
	mutex_lock(&stat->lock);

	stat->irq = pdata->irq_base + MX_HUB_IRQ_GYR;
	err = request_threaded_irq(stat->irq, 0, lsm330dlc_gyr_isr,
		0, pdev->name, stat);
	if (unlikely(err < 0)) {
		dev_err(&pdev->dev, "mx hub: failed to request GYR IRQ %d\n",
			stat->irq);
		goto exit_mutex_unlock;
	}
	

	stat->client = client;

	stat->pdata = kmalloc(sizeof(*stat->pdata), GFP_KERNEL);
	if (stat->pdata == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory for pdata: %d\n", err);
		goto exit_mutex_unlock;
	}

	if (pdata == NULL) {
		dev_dbg(&pdev->dev, "platform_data not provided,no default plaform_data\n");
		goto exit_mutex_unlock;
	} else {
		memcpy(stat->pdata, pdata->pgyr, sizeof(*stat->pdata));
	}

	err = lsm330dlc_gyr_validate_pdata(stat);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}

	memset(stat->resume_state, 0, ARRAY_SIZE(stat->resume_state));
	stat->resume_state[RES_CTRL_REG1] = ENABLE_ALL_AXES | PM_NORMAL;
	stat->resume_state[RES_CTRL_REG4] = BDU_ENABLE;

	err = lsm330dlc_gyr_device_power_on(stat);
	if (err < 0) {
		dev_err(&pdev->dev, "power on failed: %d\n", err);
		goto exit_kfree_pdata;
	}
	atomic_set(&stat->enabled, 1);

	err = lsm330dlc_gyr_update_fs_range(stat, stat->pdata->fs_range);
	if (err < 0) {
		dev_err(&pdev->dev, "update_fs_range failed\n");
		goto exit_power_off;
	}

	err = lsm330dlc_gyr_update_odr(stat, stat->pdata->poll_interval);
	if (err < 0) {
		dev_err(&pdev->dev, "update_odr failed\n");
		goto exit_power_off;
	}

	err = lsm330dlc_gyr_input_init(stat);
	if (err < 0)
		goto exit_power_off;

	err = create_sysfs_interfaces(stat);
	if (err < 0) {
		dev_err(&pdev->dev,
				"%s device register failed\n", LSM330DLC_GYR_DEV_NAME);
		goto exit_input_cleanup;
	}

	lsm330dlc_gyr_device_power_off(stat);

	/* As default, do not report information */
	atomic_set(&stat->enabled, 0);
/*
#ifdef CONFIG_HAS_EARLYSUSPEND
	stat->early_suspend.suspend = lsm330dlc_gyr_early_suspend;
	stat->early_suspend.resume = lsm330dlc_gyr_early_resume;
	stat->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 20;
	register_early_suspend(&stat->early_suspend);
#endif
*/
	platform_set_drvdata(pdev,stat);//	dev_set_drvdata(&pdev->dev,stat);
	
	mutex_unlock(&stat->lock);

	dev_dbg(&pdev->dev, "%s probed: device created successfully\n", LSM330DLC_GYR_DEV_NAME);
	return 0;

exit_input_cleanup:
	lsm330dlc_gyr_input_cleanup(stat);
exit_power_off:
	lsm330dlc_gyr_device_power_off(stat);
exit_kfree_pdata:
	kfree(stat->pdata);
exit_mutex_unlock:
	mutex_unlock(&stat->lock);
	kfree(stat);
exit_check_functionality_failed:
	pr_err("%s: Driver Initialization failed\n", LSM330DLC_GYR_DEV_NAME);
	return err;
}

static int __devexit lsm330dlc_gyr_remove(struct platform_device *pdev)
{
	struct lsm330dlc_gyr_status *stat = platform_get_drvdata(pdev);

	dev_dbg(&stat->client->dev, "driver removing\n");

	if (stat->pdata->gpio_int2 >= 0) {
		free_irq(stat->irq2, stat);
		gpio_free(stat->pdata->gpio_int2);
		destroy_workqueue(stat->irq2_work_queue);
	}

	lsm330dlc_gyr_disable(stat);
	lsm330dlc_gyr_input_cleanup(stat);

	remove_sysfs_interfaces(stat);

	kfree(stat->pdata);
	kfree(stat);
	return 0;
}
#if 0
static int lsm330dlc_gyr_suspend(struct device *dev)
{
	int err = 0;
#define SLEEP
#ifdef CONFIG_PM
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm330dlc_gyr_status *stat = i2c_get_clientdata(client);
	u8 buf[2];

	dev_dbg(&client->dev, "suspend\n");

	dev_dbg(&client->dev, "%s\n", __func__);
	if (atomic_read(&stat->enabled)) {
		mutex_lock(&stat->lock);
#ifdef SLEEP
		err = lsm330dlc_gyr_register_update(stat, buf, CTRL_REG1,
				0x0F, (ENABLE_NO_AXES | PM_NORMAL));
#else
		err = lsm330dlc_gyr_register_update(stat, buf, CTRL_REG1,
				0x08, PM_OFF);
#endif /*SLEEP*/
		mutex_unlock(&stat->lock);
	}
#endif /*CONFIG_PM*/
	return err;
}

static int lsm330dlc_gyr_resume(struct device *dev)
{
	int err = 0;
#ifdef CONFIG_PM
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm330dlc_gyr_status *stat = i2c_get_clientdata(client);
	u8 buf[2];


	dev_dbg(&client->dev, "resume\n");

	dev_dbg(&client->dev, "%s\n", __func__);
	if (atomic_read(&stat->enabled)) {
		mutex_lock(&stat->lock);
		dev_dbg(&stat->client->dev, "polling mode enabled\n");
		schedule_delayed_work(&stat->input_work, msecs_to_jiffies(stat->pdata->poll_interval));
#ifdef SLEEP
		err = lsm330dlc_gyr_register_update(stat, buf, CTRL_REG1,
				0x0F, (ENABLE_ALL_AXES | PM_NORMAL));
#else
		err = lsm330dlc_gyr_register_update(stat, buf, CTRL_REG1,
				0x08, PM_NORMAL);
#endif
		mutex_unlock(&stat->lock);

	}
#endif /*CONFIG_PM*/
	return err;
}
#endif

static const struct dev_pm_ops lsm330dlc_gyr_pm = {
	.suspend = lsm330dlc_gyr_suspend,
	.resume = lsm330dlc_gyr_resume,
};


const struct platform_device_id lsm330dlc_gyr_id[] = {
	{ "mx-hub-gyr", 0 },
	{ },
};

static struct platform_driver lsm330dlc_gyr_driver = {
	.driver = {
		.name  = "mx-hub-gyr",
		.owner = THIS_MODULE,
		.pm = &lsm330dlc_gyr_pm,
	},
	.probe = lsm330dlc_gyr_probe,
	.remove = __devexit_p(lsm330dlc_gyr_remove),
	.id_table = lsm330dlc_gyr_id,
};

static int __init lsm330dlc_gyr_init(void)
{
	return platform_driver_register(&lsm330dlc_gyr_driver);
}
module_init(lsm330dlc_gyr_init);

static void __exit lsm330dlc_gyr_exit(void)
{
	platform_driver_unregister(&lsm330dlc_gyr_driver);
}
module_exit(lsm330dlc_gyr_exit); 

MODULE_DESCRIPTION("lsm330dlc digital gyroscope section sysfs driver");
MODULE_AUTHOR("Matteo Dameno, Carmine Iascone, STMicroelectronics");
MODULE_LICENSE("GPL");

