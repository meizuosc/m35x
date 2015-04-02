/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
 *
 * File Name          : lsm330dlc_acc.c
 * Authors            : MSH - Motion Mems BU - Application Team
 *		      : Matteo Dameno (matteo.dameno@st.com)
 *		      : Carmine Iascone (carmine.iascone@st.com)
 *		      : Both authors are willing to be considered the contact
 *		      : and update points for the driver.
 * Version            : V.1.0.12
 * Date               : 2012/Feb/29
 * Description        : LSM330DLC accelerometer sensor API
 *
 *******************************************************************************
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
 ******************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include   <linux/major.h>
#include <linux/earlysuspend.h>
#include	<linux/mfd/mx-hub.h>
#include <asm/mach-types.h>

#include	"mxhub_acc.h"


#define	DEBUG 1

#define	G_MAX		16000

#define SENSITIVITY_2G		1	/**	mg/LSB	*/
#define SENSITIVITY_4G		2	/**	mg/LSB	*/
#define SENSITIVITY_8G		4	/**	mg/LSB	*/
#define SENSITIVITY_16G		12	/**	mg/LSB	*/




/* Accelerometer Sensor Operating Mode */
#define LSM330DLC_ACC_ENABLE	(0x01)
#define LSM330DLC_ACC_DISABLE	(0x00)

#define	HIGH_RESOLUTION		(0x08)

#define	AXISDATA_REG		(0x28)
#define WHOAMI_LSM330DLC_ACC	(0x33)	/*	Expctd content for WAI	*/

/*	CONTROL REGISTERS	*/
#define WHO_AM_I		(0x0F)	/*	WhoAmI register		*/
#define	TEMP_CFG_REG		(0x1F)	/*	temper sens control reg	*/
/* ctrl 1: ODR3 ODR2 ODR ODR0 LPen Zenable Yenable Zenable */
#define	CTRL_REG1		(0x20)	/*	control reg 1		*/
#define	CTRL_REG2		(0x21)	/*	control reg 2		*/
#define	CTRL_REG3		(0x22)	/*	control reg 3		*/
#define	CTRL_REG4		(0x23)	/*	control reg 4		*/
#define	CTRL_REG5		(0x24)	/*	control reg 5		*/
#define	CTRL_REG6		(0x25)	/*	control reg 6		*/
#define	STATUS_REG		0x27


#define	FIFO_CTRL_REG		(0x2E)	/*	FiFo control reg	*/

#define	INT_CFG1		(0x30)	/*	interrupt 1 config	*/
#define	INT_SRC1		(0x31)	/*	interrupt 1 source	*/
#define	INT_THS1		(0x32)	/*	interrupt 1 threshold	*/
#define	INT_DUR1		(0x33)	/*	interrupt 1 duration	*/


#define	TT_CFG			(0x38)	/*	tap config		*/
#define	TT_SRC			(0x39)	/*	tap source		*/
#define	TT_THS			(0x3A)	/*	tap threshold		*/
#define	TT_LIM			(0x3B)	/*	tap time limit		*/
#define	TT_TLAT			(0x3C)	/*	tap time latency	*/
#define	TT_TW			(0x3D)	/*	tap time window		*/
/*	end CONTROL REGISTRES	*/


#define ENABLE_HIGH_RESOLUTION	1
#define ALL_ZEROES		(0x00)

#define LSM330DLC_ACC_PM_OFF		(0x00)
#define LSM330DLC_ACC_ENABLE_ALL_AXES	(0x07)


#define PMODE_MASK		(0x08)
#define ODR_MASK		(0XF0)

#define LSM330DLC_ACC_ODR1	(0x10)  /* 1Hz output data rate */
#define LSM330DLC_ACC_ODR10	(0x20)  /* 10Hz output data rate */
#define LSM330DLC_ACC_ODR25	(0x30)  /* 25Hz output data rate */
#define LSM330DLC_ACC_ODR50	(0x40)  /* 50Hz output data rate */
#define LSM330DLC_ACC_ODR100	(0x50)  /* 100Hz output data rate */
#define LSM330DLC_ACC_ODR200	(0x60)  /* 200Hz output data rate */
#define LSM330DLC_ACC_ODR400	(0x70)  /* 400Hz output data rate */
#define LSM330DLC_ACC_ODR1250	(0x90)  /* 1250Hz output data rate */



#define	IA			(0x40)
#define	ZH			(0x20)
#define	ZL			(0x10)
#define	YH			(0x08)
#define	YL			(0x04)
#define	XH			(0x02)
#define	XL			(0x01)
/* */
/* CTRL REG BITS*/
#define	CTRL_REG3_I1_AOI1	(0x40)
#define	CTRL_REG4_BDU_ENABLE	(0x80)
#define	CTRL_REG4_BDU_MASK	(0x80)
#define	CTRL_REG6_I2_TAPEN	(0x80)
#define	CTRL_REG6_HLACTIVE	(0x02)
/* */
#define NO_MASK			(0xFF)
#define INT1_DURATION_MASK	(0x7F)
#define	INT1_THRESHOLD_MASK	(0x7F)
#define TAP_CFG_MASK		(0x3F)
#define	TAP_THS_MASK		(0x7F)
#define	TAP_TLIM_MASK		(0x7F)
#define	TAP_TLAT_MASK		NO_MASK
#define	TAP_TW_MASK		NO_MASK


/* TAP_SOURCE_REG BIT */
#define	DTAP			(0x20)
#define	STAP			(0x10)
#define	SIGNTAP			(0x08)
#define	ZTAP			(0x04)
#define	YTAP			(0x02)
#define	XTAZ			(0x01)


#define	FUZZ			0
#define	FLAT			0
#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES		5
#define	I2C_AUTO_INCREMENT	(0x80)

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1		0
#define	RES_CTRL_REG2		1
#define	RES_CTRL_REG3		2
#define	RES_CTRL_REG4		3
#define	RES_CTRL_REG5		4
#define	RES_CTRL_REG6		5

#define	RES_INT_CFG1		6
#define	RES_INT_THS1		7
#define	RES_INT_DUR1		8

#define	RES_TT_CFG		9
#define	RES_TT_THS		10
#define	RES_TT_LIM		11
#define	RES_TT_TLAT		12
#define	RES_TT_TW		13

#define	RES_TEMP_CFG_REG	14
#define	RES_REFERENCE_REG	15
#define	RES_FIFO_CTRL_REG	16

#define	RESUME_ENTRIES		17
/* end RESUME STATE INDICES */


struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} lsm330dlc_acc_odr_table[] = {
	{    1, LSM330DLC_ACC_ODR1250 },
	{    3, LSM330DLC_ACC_ODR400  },
	{    5, LSM330DLC_ACC_ODR200  },
	{   10, LSM330DLC_ACC_ODR100  },
	{   20, LSM330DLC_ACC_ODR50   },
	{   40, LSM330DLC_ACC_ODR25   },
	{  100, LSM330DLC_ACC_ODR10   },
	{ 1000, LSM330DLC_ACC_ODR1    },
};

struct lsm330dlc_acc_status {
	struct mx_hub_dev *data;
	struct i2c_client *client;
	int irq;
	
	struct acc_platform_data *pdata;

	struct class		*accelerometer;
	struct device		*class_dev;
	struct mutex lock;
	struct delayed_work input_work;

	struct input_dev *input_dev;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;

	u8 sensitivity;

	u8 resume_state[RESUME_ENTRIES];

	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;
	int calib_data[3];
	int debug;
/*
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
*/
#ifdef DEBUG
	u8 reg_addr;
#endif
};
static int lsm330dlc_acc_update_odr(struct lsm330dlc_acc_status *stat,int poll_interval_ms);

static int lsm330dlc_acc_i2c_read(struct lsm330dlc_acc_status *stat, u8 *buf, int len)
{
	int ret;
	u8 reg = buf[0] & (~I2C_AUTO_INCREMENT);
	

	ret = mx_hub_readdata_rdy(stat->client,reg+MX_HUB_REG_ACC_BASE,len,buf);
	
	return ret;
}

static int lsm330dlc_acc_i2c_write(struct lsm330dlc_acc_status *stat, u8 *buf, int len)
{
	int ret;
	u8 reg, value;

	reg = buf[0] & (~I2C_AUTO_INCREMENT);
	reg += MX_HUB_REG_ACC_BASE;
	 buf[0] = reg;
	value = buf[1];
	
	ret = mx_hub_writedata_rdy(stat->client,reg ,len,&buf[1]);
	
	return ret;
}

static int lsm330dlc_acc_hw_init(struct lsm330dlc_acc_status *stat)
{
	int err = -1;
	u8 buf[7];

	pr_info("%s: hw init start\n", LSM330DLC_ACC_DEV_NAME);

	/*test device id*/
	buf[0] = WHO_AM_I;
	err = lsm330dlc_acc_i2c_read(stat, buf, 1);
	pr_info("%s called, read device id 0x%x\n", __func__, buf[0]);
	if (err < 0) {
		dev_err(&stat->client->dev, "Error reading WHO_AM_I:"
				" is device available/working?\n");
		goto err_firstread;
	} else
		stat->hw_working = 1;		/*read ok, so device is working now*/

	if (buf[0] != WHOAMI_LSM330DLC_ACC) {
		dev_err(&stat->client->dev,
				"device unknown. Expected: 0x%02x, Replies: 0x%02x\n",
				WHOAMI_LSM330DLC_ACC, buf[0]);
		err = -1; /* choose the right coded error */
		goto err_unknown_device;
	}

	/*resume all registers*/
	buf[0] = CTRL_REG1;
	buf[1] = stat->resume_state[RES_CTRL_REG1];
	err = lsm330dlc_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = TEMP_CFG_REG;
	buf[1] = stat->resume_state[RES_TEMP_CFG_REG];
	err = lsm330dlc_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = FIFO_CTRL_REG;
	buf[1] = stat->resume_state[RES_FIFO_CTRL_REG];
	err = lsm330dlc_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = TT_THS;
	buf[1] = stat->resume_state[RES_TT_THS];
	buf[2] = stat->resume_state[RES_TT_LIM];
	buf[3] = stat->resume_state[RES_TT_TLAT];
	buf[4] = stat->resume_state[RES_TT_TW];
	err = lsm330dlc_acc_i2c_write(stat, buf, 4);
	if (err < 0)
		goto err_resume_state;
	
	buf[0] = TT_CFG;
	buf[1] = stat->resume_state[RES_TT_CFG];
	err = lsm330dlc_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = INT_THS1;
	buf[1] = stat->resume_state[RES_INT_THS1];
	buf[2] = stat->resume_state[RES_INT_DUR1];
	err = lsm330dlc_acc_i2c_write(stat, buf, 2);
	if (err < 0)
		goto err_resume_state;
	
	buf[0] = INT_CFG1;
	buf[1] = stat->resume_state[RES_INT_CFG1];
	err = lsm330dlc_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = CTRL_REG2;
	buf[1] = stat->resume_state[RES_CTRL_REG2];
	buf[2] = stat->resume_state[RES_CTRL_REG3];
	buf[3] = stat->resume_state[RES_CTRL_REG4];
	buf[4] = stat->resume_state[RES_CTRL_REG5];
	buf[5] = stat->resume_state[RES_CTRL_REG6];
	err = lsm330dlc_acc_i2c_write(stat, buf, 5);
	if (err < 0)
		goto err_resume_state;
	
	/*resume all registers done. hw_initialized*/
	stat->hw_initialized = 1;
	
	err = lsm330dlc_acc_update_odr(stat, stat->pdata->poll_interval);
	if (err < 0) {
		dev_err(&stat->client->dev, "update_odr failed\n");
	}
	
	pr_info("%s: hw init done\n", LSM330DLC_ACC_DEV_NAME);
	return 0;

err_firstread:
	stat->hw_working = 0;
err_unknown_device:
err_resume_state:
	stat->hw_initialized = 0;
	dev_err(&stat->client->dev, "hw init error 0x%02x,0x%02x: %d\n", buf[0],
			buf[1], err);
	return err;
}

static void lsm330dlc_acc_device_power_off(struct lsm330dlc_acc_status *stat)
{
	int err;
	u8 buf[2] = { CTRL_REG1, LSM330DLC_ACC_PM_OFF };

	dev_info(&stat->client->dev,"%s called\n", __func__);
	err = lsm330dlc_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		dev_err(&stat->client->dev, "soft power off failed: %d\n", err);

	if (stat->pdata->power_off) {
		err = stat->pdata->power_off();
		if(err < 0) {
			dev_err(&stat->client->dev, "hard power off(vdd28_acc) failed: %d\n", err);
		}
		stat->hw_initialized = 0;
	}

	if (stat->hw_initialized)
		stat->hw_initialized = 0;
}

static int lsm330dlc_acc_device_power_on(struct lsm330dlc_acc_status *stat)
{
	int err = -1;

	dev_info(&stat->client->dev,"%s called\n", __func__);
	if (stat->pdata->power_on) {
		err = stat->pdata->power_on();
		if (err < 0) {
			dev_err(&stat->client->dev, "power_on failed: %d\n", err);
			return err;
		}
	}

	if (!stat->hw_initialized) {
		int i;
		for (i = 0; i < 5; i++)
		{
			err = lsm330dlc_acc_hw_init(stat);
			if (err < 0)
				msleep(100);
			else
				break;
		}
		//err = lsm330dlc_acc_hw_init(stat);
		if (stat->hw_working == 1 && err < 0) {
			lsm330dlc_acc_device_power_off(stat);
			return err;
		}
	}

	return 0;
}
static int lsm330dlc_acc_update_fs_range(struct lsm330dlc_acc_status *stat,
		u8 new_fs_range)
{
	int err = -1;

	u8 sensitivity;
	u8 buf[2];
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = LSM330DLC_ACC_FS_MASK | HIGH_RESOLUTION;

	dev_dbg(&stat->client->dev,"%s called\n", __func__);
	switch (new_fs_range) {
		case LSM330DLC_ACC_G_2G:

			sensitivity = SENSITIVITY_2G;
			break;
		case LSM330DLC_ACC_G_4G:

			sensitivity = SENSITIVITY_4G;
			break;
		case LSM330DLC_ACC_G_8G:

			sensitivity = SENSITIVITY_8G;
			break;
		case LSM330DLC_ACC_G_16G:

			sensitivity = SENSITIVITY_16G;
			break;
		default:
			dev_err(&stat->client->dev, "invalid fs range requested: %u\n",
					new_fs_range);
			return -EINVAL;
	}


	/* Updates configuration register 4,
	 * which contains fs range setting */
	buf[0] = CTRL_REG4;
	err = lsm330dlc_acc_i2c_read(stat, buf, 1);
	if (err < 0)
		goto error;
	init_val = buf[0];
	stat->resume_state[RES_CTRL_REG4] = init_val;
	new_val = new_fs_range | HIGH_RESOLUTION;
	updated_val = ((mask & new_val) | ((~mask) & init_val));
	buf[1] = updated_val;
	buf[0] = CTRL_REG4;
	err = lsm330dlc_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto error;
	stat->resume_state[RES_CTRL_REG4] = updated_val;
	stat->sensitivity = sensitivity;

	return err;
error:
	dev_err(&stat->client->dev,
			"update fs range failed 0x%02x,0x%02x: %d\n",
			buf[0], buf[1], err);

	return err;
}

static int lsm330dlc_acc_update_odr(struct lsm330dlc_acc_status *stat,
		int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config[2];

	dev_dbg(&stat->client->dev,"%s called\n", __func__);
	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(lsm330dlc_acc_odr_table) - 1; i >= 0; i--) {
		if ((lsm330dlc_acc_odr_table[i].cutoff_ms <= poll_interval_ms)
				|| (i == 0))
			break;
	}
	config[1] = lsm330dlc_acc_odr_table[i].mask;

	config[1] |= LSM330DLC_ACC_ENABLE_ALL_AXES;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&stat->enabled)) {
		config[0] = CTRL_REG1;
		err = lsm330dlc_acc_i2c_write(stat, config, 1);
		if (err < 0)
			goto error;
		stat->resume_state[RES_CTRL_REG1] = config[1];
	}

	return err;

error:
	dev_err(&stat->client->dev, "update odr failed 0x%02x,0x%02x: %d\n",
			config[0], config[1], err);

	return err;
}



static int lsm330dlc_acc_register_write(struct lsm330dlc_acc_status *stat,
		u8 *buf, u8 reg_address, u8 new_value)
{
	int err = -1;

	/* Sets configuration register at reg_address
	 *  NOTE: this is a straight overwrite  */
	buf[0] = reg_address;
	buf[1] = new_value;
	err = lsm330dlc_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		return err;
	return err;
}

/*
   static int lsm330dlc_acc_register_read(struct lsm330dlc_acc_status *stat,
   u8 *buf, u8 reg_address)
   {

   int err = -1;
   buf[0] = (reg_address);
   err = lsm330dlc_acc_i2c_read(stat, buf, 1);
   return err;
   }
 */

/*
   static int lsm330dlc_acc_register_update(struct lsm330dlc_acc_status *stat,
   u8 *buf, u8 reg_address, u8 mask, u8 new_bit_values)
   {
   int err = -1;
   u8 init_val;
   u8 updated_val;
   err = lsm330dlc_acc_register_read(stat, buf, reg_address);
   if (!(err < 0)) {
   init_val = buf[1];
   updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
   err = lsm330dlc_acc_register_write(stat, buf, reg_address,
   updated_val);
   }
   return err;
   }
 */

static int lsm330dlc_acc_get_acceleration_data(
		struct lsm330dlc_acc_status *stat, int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s16 hw_d[3] = { 0 };

	acc_data[0] = (AXISDATA_REG);
	err = lsm330dlc_acc_i2c_read(stat, acc_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = (((s16) ((acc_data[1] << 8) | acc_data[0])) >> 4);
	hw_d[1] = (((s16) ((acc_data[3] << 8) | acc_data[2])) >> 4);
	hw_d[2] = (((s16) ((acc_data[5] << 8) | acc_data[4])) >> 4);



	hw_d[0] = hw_d[0] * stat->sensitivity;
	hw_d[1] = hw_d[1] * stat->sensitivity;
	hw_d[2] = hw_d[2] * stat->sensitivity;


	xyz[0] = ((stat->pdata->negate_x) ? (-hw_d[stat->pdata->axis_map_x])
			: (hw_d[stat->pdata->axis_map_x]));
	xyz[1] = ((stat->pdata->negate_y) ? (-hw_d[stat->pdata->axis_map_y])
			: (hw_d[stat->pdata->axis_map_y]));
	xyz[2] = ((stat->pdata->negate_z) ? (-hw_d[stat->pdata->axis_map_z])
			: (hw_d[stat->pdata->axis_map_z]));

#ifdef DEBUG
	/*
	   pr_info("%s read x=%d, y=%d, z=%d\n",
	   LSM330DLC_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
	 */
#endif
	return err;
}

static void lsm330dlc_acc_report_values(struct lsm330dlc_acc_status *stat,
		int *xyz)
{
	input_report_abs(stat->input_dev, ABS_X, xyz[0]-stat->calib_data[0]);
	input_report_abs(stat->input_dev, ABS_Y, xyz[1]-stat->calib_data[1]);
	input_report_abs(stat->input_dev, ABS_Z, xyz[2]-stat->calib_data[2]);
	input_sync(stat->input_dev);
	if(stat->debug)
		pr_info("x = %d, y = %d z = %d\n", xyz[0], xyz[1], xyz[2]);
}
/*lis3dh calibration*/
#define CALIB_DATA_AMOUNT 100
static int lsm330dlc_acc_calibration(struct lsm330dlc_acc_status *stat, bool do_calib)
{
	int xyz[3] = {0,};
	int sum[3] = {0,};
	int err = 0;
	int i;

	if (do_calib) {
		for (i =0; i < CALIB_DATA_AMOUNT; i++) {
			mutex_lock(&stat->lock);
			err = lsm330dlc_acc_get_acceleration_data(stat, xyz);
			msleep(stat->pdata->poll_interval);
			if (err < 0) {
				dev_err(&stat->client->dev, "get_acceleration_data failed\n");
				mutex_unlock(&stat->lock);
				return err;
			}
			mutex_unlock(&stat->lock);

			sum[0] += xyz[0];
			sum[1] += xyz[1];
			sum[2] += xyz[2];
		}

		stat->calib_data[0]= sum[0] / CALIB_DATA_AMOUNT;
		stat->calib_data[1] = sum[1] / CALIB_DATA_AMOUNT;
		stat->calib_data[2] = sum[2] / CALIB_DATA_AMOUNT - 1000;
	} else {
		stat->calib_data[0] = 0;
		stat->calib_data[1] = 0;
		stat->calib_data[2] = 0;
	}

	pr_info("%s: calibration data (%d,%d,%d)\n",
			__func__, stat->calib_data[0], stat->calib_data[1],
			stat->calib_data[2]);
	return err;
}

static int lsm330dlc_acc_enable(struct lsm330dlc_acc_status *stat)
{
	int err;

	pr_info("%s called stat->enabled = %d\n", __func__, atomic_read(&stat->enabled));
	if (!atomic_cmpxchg(&stat->enabled, 0, 1)) {
		err = lsm330dlc_acc_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled, 0);
			return err;
		}
		schedule_delayed_work(&stat->input_work, msecs_to_jiffies(stat->pdata->poll_interval));
	}

	return 0;
}

static int lsm330dlc_acc_disable(struct lsm330dlc_acc_status *stat)
{
	pr_info("%s called\n", __func__);
	if (atomic_cmpxchg(&stat->enabled, 1, 0)) {
		cancel_delayed_work_sync(&stat->input_work);
		lsm330dlc_acc_device_power_off(stat);
	}
	

	return 0;
}


static ssize_t read_single_reg(struct device *dev, char *buf, u8 reg)
{
	ssize_t ret;
	struct lsm330dlc_acc_status *stat = dev_get_drvdata(dev);
	int err;

	u8 data = reg;
	err = lsm330dlc_acc_i2c_read(stat, &data, 1);
	if (err < 0)
		return err;
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;

}

static int write_reg(struct device *dev, const char *buf, u8 reg,
		u8 mask, int resumeIndex)
{
	int err = -1;
	struct lsm330dlc_acc_status *stat = dev_get_drvdata(dev);
	u8 x[2];
	u8 new_val;
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	new_val = ((u8) val & mask);
	x[0] = reg;
	x[1] = new_val;
	err = lsm330dlc_acc_register_write(stat, x, reg, new_val);
	if (err < 0)
		return err;
	stat->resume_state[resumeIndex] = new_val;
	return err;
}

static ssize_t attr_get_polling_rate(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int val;
	struct lsm330dlc_acc_status *stat = dev_get_drvdata(dev);
	mutex_lock(&stat->lock);
	val = stat->pdata->poll_interval;
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct lsm330dlc_acc_status *stat = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	//	pr_info("accelerometer set poll_rate = %ld from sysfs\n", interval_ms);
	interval_ms = max((unsigned int)interval_ms, stat->pdata->min_interval);
	mutex_lock(&stat->lock);
	stat->pdata->poll_interval = interval_ms;
	lsm330dlc_acc_update_odr(stat, interval_ms);
	mutex_unlock(&stat->lock);
	return size;
}

static ssize_t attr_get_range(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char val;
	struct lsm330dlc_acc_status *stat = dev_get_drvdata(dev);
	char range = 2;
	mutex_lock(&stat->lock);
	val = stat->pdata->fs_range ;
	switch (val) {
		case LSM330DLC_ACC_G_2G:
			range = 2;
			break;
		case LSM330DLC_ACC_G_4G:
			range = 4;
			break;
		case LSM330DLC_ACC_G_8G:
			range = 8;
			break;
		case LSM330DLC_ACC_G_16G:
			range = 16;
			break;
	}
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct lsm330dlc_acc_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	u8 range;
	int err;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	switch (val) {
		case 2:
			range = LSM330DLC_ACC_G_2G;
			break;
		case 4:
			range = LSM330DLC_ACC_G_4G;
			break;
		case 8:
			range = LSM330DLC_ACC_G_8G;
			break;
		case 16:
			range = LSM330DLC_ACC_G_16G;
			break;
		default:
			dev_err(&stat->client->dev, "invalid range request: %lu,"
					" discarded\n", val);
			return -EINVAL;
	}
	mutex_lock(&stat->lock);
	err = lsm330dlc_acc_update_fs_range(stat, range);
	if (err < 0) {
		mutex_unlock(&stat->lock);
		return err;
	}
	stat->pdata->fs_range = range;
	mutex_unlock(&stat->lock);
	dev_dbg(&stat->client->dev, "range set to: %lu g\n", val);

	return size;
}

static ssize_t attr_get_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct lsm330dlc_acc_status *stat = dev_get_drvdata(dev);
	int val = atomic_read(&stat->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct lsm330dlc_acc_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	//	pr_info("accelerometer enable = %ld from sysfs\n", val);
	if (val)
		lsm330dlc_acc_enable(stat);
	else
		lsm330dlc_acc_disable(stat);

	return size;
}
/*
   static ssize_t attr_get_selftest_result(struct device *dev,
   struct device_attribute *attr, char *buf)
   {
   struct lsm330dlc_acc_status *stat = dev_get_drvdata(dev);
   int ret = lsm330dlc_acc_selftest(stat);
   return sprintf(buf,"%d\n", ret);
   }

   int lsm330dlc_acc_selftest(lsm330dlc_acc_status *stat)
   {
   return 0;
   }
 */

static ssize_t attr_set_intconfig1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_CFG1, NO_MASK, RES_INT_CFG1);
}

static ssize_t attr_get_intconfig1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_CFG1);
}

static ssize_t attr_set_duration1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_DUR1, INT1_DURATION_MASK, RES_INT_DUR1);
}

static ssize_t attr_get_duration1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_DUR1);
}

static ssize_t attr_set_thresh1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_THS1, INT1_THRESHOLD_MASK, RES_INT_THS1);
}

static ssize_t attr_get_thresh1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_THS1);
}

static ssize_t attr_get_source1(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, INT_SRC1);
}

static ssize_t attr_set_click_cfg(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_CFG, TAP_CFG_MASK, RES_TT_CFG);
}

static ssize_t attr_get_click_cfg(struct device *dev,
		struct device_attribute *attr,	char *buf)
{

	return read_single_reg(dev, buf, TT_CFG);
}

static ssize_t attr_get_click_source(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_SRC);
}

static ssize_t attr_set_click_ths(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_THS, TAP_THS_MASK, RES_TT_THS);
}

static ssize_t attr_get_click_ths(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_THS);
}

static ssize_t attr_set_click_tlim(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_LIM, TAP_TLIM_MASK, RES_TT_LIM);
}

static ssize_t attr_get_click_tlim(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_LIM);
}

static ssize_t attr_set_click_tlat(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_TLAT, TAP_TLAT_MASK, RES_TT_TLAT);
}

static ssize_t attr_get_click_tlat(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_TLAT);
}

static ssize_t attr_set_click_tw(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_TLAT, TAP_TW_MASK, RES_TT_TLAT);
}

static ssize_t attr_get_click_tw(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_TLAT);
}


#ifdef DEBUG
/* PAY ATTENTION: These DEBUG functions don't manage resume_state */
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int rc;
	struct lsm330dlc_acc_status *stat = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&stat->lock);
	x[0] = stat->reg_addr;
	mutex_unlock(&stat->lock);
	x[1] = val;
	rc = lsm330dlc_acc_i2c_write(stat, x, 1);
	/*TODO: error need to be managed */
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	ssize_t ret;
	struct lsm330dlc_acc_status *stat = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&stat->lock);
	data = stat->reg_addr;
	mutex_unlock(&stat->lock);
	rc = lsm330dlc_acc_i2c_read(stat, &data, 1);
	/*TODO: error need to be managed */
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct lsm330dlc_acc_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&stat->lock);
	stat->reg_addr = val;
	mutex_unlock(&stat->lock);
	return size;
}
#endif
/* selftest start */
#define SELFTEST_MEASURE_TIMEOUT 100
#define SELFTEST_ZYXDA (0x1 << 3)
#define SELFTEST_SAMPLES 100

static int selftest_init(struct lsm330dlc_acc_status *stat)
{
	unsigned char buf[5];
	//	pr_info("%s\n", __func__);

	/* BDU=1, ODR=200Hz, FS=+/-2G */
	buf[0] = I2C_AUTO_INCREMENT | CTRL_REG1;
	buf[1] = LSM330DLC_ACC_ODR200 | 0xFF;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = 0x80;	/* BDU=1 */

	return lsm330dlc_acc_i2c_write(stat, buf, 4);
}

static int selftest_enable(struct lsm330dlc_acc_status *stat)
{
	unsigned char buf[2];

	buf[0] = CTRL_REG4;
	buf[1] = 0x82;	/* BDU=1, ST1 = 1, ST0 = 0 */

	pr_debug("%s\n", __func__);

	return lsm330dlc_acc_i2c_write(stat, buf, 1);
}

static void selftest_disable(struct lsm330dlc_acc_status *stat)
{
	unsigned char buf[2];

	buf[1] = 0x00;
	pr_debug("%s\n", __func__);

	/* Disable sensor */
	buf[0] = CTRL_REG1;
	lsm330dlc_acc_i2c_write(stat, buf, 1);
	/* Disable selftest */
	buf[0] = CTRL_REG4;
	lsm330dlc_acc_i2c_write(stat, buf, 1);
}

static int selftest_wait_ZYXDA(struct lsm330dlc_acc_status *stat)
{
	int i, ret;
	unsigned char data_ready;

	pr_debug("%s\n", __func__);

	for (i = SELFTEST_MEASURE_TIMEOUT; i != 0; i--) {
		data_ready = STATUS_REG;
		ret = lsm330dlc_acc_i2c_read(stat, &data_ready, 1);
		if (ret < 0) {
			pr_err("%s: lsm330dlc_acc_i2c_read fail, retry %d\n",
					__func__, i);
			msleep(I2C_RETRY_DELAY);
			continue;
		} else if (data_ready & SELFTEST_ZYXDA) {
			pr_debug("%s: data ready\n", __func__);
			break;
		}
	}
	if (i == 0) {
		pr_err("%s: failed\n", __func__);
		return ret;
	}

	return 0;
}

static int selftest_read(struct lsm330dlc_acc_status *stat, int data[3])
{
	int total[3];
	int i, ret;

	pr_debug("%s\n", __func__);

	total[0] = 0;
	total[1] = 0;
	total[2] = 0;
	for (i = 0; i < SELFTEST_SAMPLES; i++) {
		ret = selftest_wait_ZYXDA(stat);
		if (ret) {
			pr_err("%s: selftest_check_ZYXDA fail\n", __func__);
			return ret;
		}
		mutex_lock(&stat->lock);
		ret = lsm330dlc_acc_get_acceleration_data(stat,data);
		mutex_unlock(&stat->lock);
		if (ret < 0) {
			pr_err("%s: lis3dh_read_gyro_values fail\n", __func__);
			return ret;
		}
		pr_debug("%s: data: x = %d, y = %d, z = %d\n", __func__,
				data[0], data[1], data[2]);
		total[0] += data[0];
		total[1] += data[1];
		total[2] += data[2];
		pr_debug("%s: total: x = %d, y = %d, z = %d\n", __func__,
				total[0], total[1], total[2]);
	}
	data[0] = total[0] / SELFTEST_SAMPLES;
	data[1] = total[1] / SELFTEST_SAMPLES;
	data[2] = total[2] / SELFTEST_SAMPLES;
	pr_debug("%s: average: x = %d, y = %d, z = %d\n", __func__,
			data[0], data[1], data[2]);

	return 0;
}

/*
 * Part Number Min_X Max_X Min_Y Max_Y Min_Z Max_Z Unit
 * LIS3DH 60 1700 60 1700 60 1400 LSB (@ FS = +/-2g,2.8v)
 */

#define CONVERT_TO_MG   1      /* for range = +/-2g */

static int check_selftest_result(int data_nost[3],
		int data_st[3])
{
	data_st[0] = abs(data_st[0]- data_nost[0]) * CONVERT_TO_MG;
	data_st[1] = abs(data_st[1]- data_nost[1]) * CONVERT_TO_MG;
	data_st[2] = abs(data_st[2]- data_nost[2]) * CONVERT_TO_MG;

	if(data_st[0] >= 60 && data_st[0] <= 1700){
		pr_debug("expect 60 =< x <= 1700, x = %d selftest pass\n", data_st[0]);
	}

	if(data_st[1] >= 60 && data_st[1] <= 1700){
		pr_debug("expect 60 =< y <= 1700, y = %d selftest pass\n", data_st[1]);
	}

	if(data_st[2] >= 60 && data_st[2]<= 1400){
		pr_debug("expect 60 =< z <= 1400, z = %d selftest pass\n",data_st[2]);
	}

	if(data_st[0] >= 60 && data_st[0] <= 1700 && data_st[1] >= 60
			&& data_st[1] <= 1700 && data_st[2] >= 60 &&
			data_st[2] <= 1400){
		return 1;
	}
	return -1;
}

static int lsm330dlc_acc_selftest(struct lsm330dlc_acc_status *stat, int *test_result,
		int self_data[3])
{
	int ret;
	int  data_nost[3];

	/* Initialize Sensor, turn on sensor, enable P/R/Y */
	ret = selftest_init(stat);
	if (ret < 0) {
		pr_err("%s: selftest_init fail\n", __func__);
		return ret;
	}

	/* Wait for stable output */
	pr_debug("%s: wait for stable output\n", __func__);
	msleep(800);

	/* Read out normal output */
	ret = selftest_read(stat, data_nost);
	if (ret < 0) {
		pr_err("%s: selftest_read fail\n", __func__);
		return ret;
	}
	pr_debug("%s: normal output: x = %d, y = %d, z = %d\n",
			__func__, data_nost[0], data_nost[1], data_nost[2]);

	/* Enable self test */
	ret = selftest_enable(stat);
	if (ret < 0) {
		pr_err("%s: selftest_enable failed\n", __func__);
		return ret;
	}

	/* ODR=200HZ, wait for 3 * ODR */
	mdelay(3 * (1000 / 200));

	/* Read out selftest output */
	ret = selftest_read(stat, self_data);
	if (ret < 0) {
		pr_err("%s: selftest_read fail\n", __func__);
		return ret;
	}

	/* Check output */
	ret = check_selftest_result(data_nost, self_data);
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
	int self_data[3];
	struct lsm330dlc_acc_status *stat = dev_get_drvdata(dev);
	lsm330dlc_acc_selftest(stat, &test_result, self_data);
	return sprintf(buf, "%d\n", test_result);
}

/* selftest end */
/*calibrate*/
static ssize_t attr_set_Calibration(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct lsm330dlc_acc_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	bool do_calib;
	int err = 0;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	if (val)
		do_calib = true;
	else
		do_calib = false;

	/*if off , before the calibration, we should turn on it*/
	if (!atomic_read(&stat->enabled)) {
		err = lsm330dlc_acc_device_power_on(stat);
		if (err < 0) {
			pr_err("%s: lis3dh_acc_device_power_on fail\n",
					__func__);
			return err;
		}
		//		atomic_set(&stat->enabled, 1);
	}

	lsm330dlc_acc_calibration(stat, do_calib);
	if (err < 0) {
		dev_err(&stat->client->dev, "lis3dh_acc_calibration failed\n");
		return err;
	}
	lsm330dlc_acc_device_power_off(stat);
	return size;
}

static ssize_t attr_get_Calibration(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct lsm330dlc_acc_status *stat = dev_get_drvdata(dev);

	return sprintf(buf, "%d %d %d\n", stat->calib_data[0],
			stat->calib_data[1], stat->calib_data[2]);
}

static ssize_t attr_set_calibvalue(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct lsm330dlc_acc_status *stat = dev_get_drvdata(dev);
	char calib_buf[64] = {0};
	char *calib_value1 = NULL;
	char *calib_value2 = NULL;

	if (buf == NULL)
		return -EINVAL;

	strcpy(calib_buf, buf);
	calib_value1 = calib_value2 = calib_buf;

	while (*calib_value1 && *calib_value1 != ' ')
		++calib_value1;
	*calib_value1 = '\0';
	stat->calib_data[0] = simple_strtol(calib_buf, NULL, 10);

	calib_value1++;
	calib_value2 = calib_value1;
	while (*calib_value1 && *calib_value1 != ' ')
		++calib_value1;
	*calib_value1 = '\0';
	stat->calib_data[1] = simple_strtol(calib_value2, NULL, 10);

	stat->calib_data[2] = simple_strtol(calib_value1+1, NULL, 10);

	pr_info("x = %d, y = %d, z= %d\n", stat->calib_data[1],
			stat->calib_data[1], stat->calib_data[2]);

	return size;
}

static ssize_t attr_get_calibvalue(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct lsm330dlc_acc_status *stat = dev_get_drvdata(dev);

	return sprintf(buf, "%d %d %d\n", stat->calib_data[0],
			stat->calib_data[1], stat->calib_data[2]);
}

static ssize_t attr_set_debug(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct lsm330dlc_acc_status *stat = dev_get_drvdata(dev);
	long val = 0;

	if (buf == NULL)
		return -EINVAL;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	stat->debug = val;

	pr_info("debug = %d\n", stat->debug);

	return size;
}

static ssize_t attr_get_debug(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct lsm330dlc_acc_status *stat = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", stat->debug);
}


static struct device_attribute attributes[] = {

	__ATTR(pollrate_ms, 0664, attr_get_polling_rate, attr_set_polling_rate),
	__ATTR(range, 0664, attr_get_range, attr_set_range),
	__ATTR(enable_device, 0664, attr_get_enable, attr_set_enable),
	__ATTR(int1_config, 0664, attr_get_intconfig1, attr_set_intconfig1),
	__ATTR(int1_duration, 0664, attr_get_duration1, attr_set_duration1),
	__ATTR(int1_threshold, 0664, attr_get_thresh1, attr_set_thresh1),
	__ATTR(int1_source, 0444, attr_get_source1, NULL),
	__ATTR(click_config, 0664, attr_get_click_cfg, attr_set_click_cfg),
	__ATTR(click_source, 0444, attr_get_click_source, NULL),
	__ATTR(click_threshold, 0664, attr_get_click_ths, attr_set_click_ths),
	__ATTR(click_timelimit, 0664, attr_get_click_tlim, attr_set_click_tlim),
	__ATTR(click_timelatency, 0664, attr_get_click_tlat,
			attr_set_click_tlat),
	__ATTR(click_timewindow, 0664, attr_get_click_tw, attr_set_click_tw),
	__ATTR(selftest, 0664, attr_get_selftest_result, NULL),
	__ATTR(Calibration, 0644, attr_get_Calibration, attr_set_Calibration),
	__ATTR(calib_value, 0644, attr_get_calibvalue, attr_set_calibvalue),
	__ATTR(debug, 0644, attr_get_debug, attr_set_debug),
#ifdef DEBUG
	__ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0200, NULL, attr_addr_set),
#endif
};
static char const *const device_link_name = "i2c";
static dev_t const st_acc_device_dev_t = MKDEV(MISC_MAJOR, 241);

static int create_sysfs_files(struct device *dev)
{
	int i;

	dev_dbg(dev,"%s called\n", __func__);
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int create_sysfs_interfaces(struct lsm330dlc_acc_status *stat)
{
	int err;

	if (NULL == stat)
		return -EINVAL;

	err = 0;

	stat->accelerometer= class_create(THIS_MODULE, ACC_SYSCLS_NAME);
	if (IS_ERR(stat->accelerometer)) {
		err = PTR_ERR(stat->accelerometer);
		goto exit_class_create_failed;
	}

	stat->class_dev = device_create(
			stat->accelerometer,
			NULL,
			st_acc_device_dev_t,
			stat,
			ACC_SYSDEV_NAME);
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
	device_destroy(stat->accelerometer, st_acc_device_dev_t);
exit_class_device_create_failed:
	stat->class_dev = NULL;
	class_destroy(stat->accelerometer);
exit_class_create_failed:
	stat->accelerometer
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

static void remove_sysfs_interfaces(struct lsm330dlc_acc_status *stat)
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
	if (NULL != stat->accelerometer) {
		device_destroy(
				stat->accelerometer,
				st_acc_device_dev_t);
		class_destroy(stat->accelerometer);
		stat->accelerometer= NULL;
	}
}

static void lsm330dlc_acc_input_work_func(struct work_struct *work)
{
	struct lsm330dlc_acc_status *stat;

	int xyz[3] = { 0 };
	int err;

	stat = container_of((struct delayed_work *)work,
			struct lsm330dlc_acc_status, input_work);

	if(!atomic_read(&stat->enabled)){
		pr_info("%s disabled\n", __func__);
		return;
	}
	mutex_lock(&stat->lock);
	if(stat->debug)
		pr_debug("%s called %d \n", __func__,stat->pdata->poll_interval);
	err = lsm330dlc_acc_get_acceleration_data(stat, xyz);
	if (err < 0)
		dev_err(&stat->client->dev, "get_acceleration_data failed\n");
	else
		lsm330dlc_acc_report_values(stat, xyz);
	schedule_delayed_work( &stat->input_work, msecs_to_jiffies(
				stat->pdata->poll_interval));
	mutex_unlock(&stat->lock);

}

int lsm330dlc_acc_input_open(struct input_dev *input)
{
	struct lsm330dlc_acc_status *stat = input_get_drvdata(input);

	dev_dbg(&stat->client->dev,"%s called\n", __func__);
	return lsm330dlc_acc_enable(stat);
}

void lsm330dlc_acc_input_close(struct input_dev *dev)
{
	struct lsm330dlc_acc_status *stat = input_get_drvdata(dev);

	dev_dbg(&stat->client->dev,"%s called\n", __func__);
	lsm330dlc_acc_disable(stat);
}

static int lsm330dlc_acc_validate_pdata(struct lsm330dlc_acc_status *stat)
{
	/* checks for correctness of minimal polling period */
	stat->pdata->min_interval =
		max((unsigned int)LSM330DLC_ACC_MIN_POLL_PERIOD_MS,
				stat->pdata->min_interval);

	stat->pdata->poll_interval = max(stat->pdata->poll_interval,
			stat->pdata->min_interval);

	if (stat->pdata->axis_map_x > 2 ||
			stat->pdata->axis_map_y > 2 ||
			stat->pdata->axis_map_z > 2) {
		dev_err(&stat->client->dev, "invalid axis_map value "
				"x:%u y:%u z%u\n", stat->pdata->axis_map_x,
				stat->pdata->axis_map_y,
				stat->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (stat->pdata->negate_x > 1 || stat->pdata->negate_y > 1
			|| stat->pdata->negate_z > 1) {
		dev_err(&stat->client->dev, "invalid negate value "
				"x:%u y:%u z:%u\n", stat->pdata->negate_x,
				stat->pdata->negate_y, stat->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (stat->pdata->poll_interval < stat->pdata->min_interval) {
		dev_err(&stat->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lsm330dlc_acc_input_init(struct lsm330dlc_acc_status *stat)
{
	int err;

	dev_dbg(&stat->client->dev,"%s called\n", __func__);
	INIT_DELAYED_WORK(&stat->input_work, lsm330dlc_acc_input_work_func);
	stat->input_dev = input_allocate_device();
	if (!stat->input_dev) {
		err = -ENOMEM;
		dev_err(&stat->client->dev, "input device allocation failed\n");
		goto err0;
	}

//	stat->input_dev->open = lsm330dlc_acc_input_open;
	stat->input_dev->close = lsm330dlc_acc_input_close;
	stat->input_dev->name = LSM330DLC_ACC_DEV_NAME;
	/* stat->input_dev->name = "accelerometer"; */
	stat->input_dev->id.bustype = BUS_I2C;
	stat->input_dev->dev.parent = &stat->client->dev;

	input_set_drvdata(stat->input_dev, stat);

	set_bit(EV_ABS, stat->input_dev->evbit);
	/*	next is used for interruptA sources data if the case */
	set_bit(ABS_MISC, stat->input_dev->absbit);
	/*	next is used for interruptB sources data if the case */
	set_bit(ABS_WHEEL, stat->input_dev->absbit);

	input_set_abs_params(stat->input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(stat->input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(stat->input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);
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

#ifdef CONFIG_PM
static int lsm330dlc_acc_resume(struct device *dev)
{
	int err = 0;
	struct lsm330dlc_acc_status * stat = dev_get_drvdata(dev);

	dev_dbg(&stat->client->dev,"%s called\n", __func__);
	if (atomic_read(&stat->enabled)) {
		err = lsm330dlc_acc_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled, 0);
			return err;
		}
		schedule_delayed_work(&stat->input_work, msecs_to_jiffies(stat->pdata->poll_interval));
	}

	return 0;
}

static int lsm330dlc_acc_suspend(struct device *dev)
{
	struct lsm330dlc_acc_status * stat = dev_get_drvdata(dev);

	dev_dbg(&stat->client->dev,"%s called\n", __func__);

	if (atomic_read(&stat->enabled)) {
		lsm330dlc_acc_device_power_off(stat);
	}

	return 0;
}
#else
#define lsm330dlc_acc_suspend	NULL
#define lsm330dlc_acc_resume	NULL
#endif /* CONFIG_PM */

/*
#ifdef CONFIG_HAS_EARLYSUSPEND
static void lsm330dlc_acc_early_suspend(struct early_suspend *handler)
{
	struct lsm330dlc_acc_status *stat;
	stat = container_of(handler, struct lsm330dlc_acc_status, early_suspend);

#if DEBUG
	pr_info("%s\n", __func__);
#endif
	lsm330dlc_acc_suspend(stat->client, PMSG_SUSPEND);
}

static void lsm330dlc_acc_early_resume(struct early_suspend *handler)
{
	struct lsm330dlc_acc_status *stat;
	stat = container_of(handler, struct lsm330dlc_acc_status, early_suspend);

#if DEBUG
	pr_info("%s\n", __func__);
#endif
	lsm330dlc_acc_resume(stat->client);
}
#endif
*/
static void lsm330dlc_acc_input_cleanup(struct lsm330dlc_acc_status *stat)
{
	input_unregister_device(stat->input_dev);
	input_free_device(stat->input_dev);
}



static irqreturn_t lsm330dlc_acc_isr(int irq, void *dev_id)
{
	struct lsm330dlc_acc_status *stat = dev_id;
	//struct regmap *regmap = stat->iodev->regmap;
	int ret;

	pr_info("ENTER %s:************\n",__func__);

	//wake_lock(&stat->wake_lock);

	ret = stat->data->irq;

	//wake_unlock(&stat->wake_lock);

	return IRQ_HANDLED;
}

static int __devinit lsm330dlc_acc_probe(struct platform_device *pdev)
{
	struct mx_hub_dev *data = dev_get_drvdata(pdev->dev.parent);
	struct mx_sensor_hub_platform_data *pdata = dev_get_platdata(data->dev);
	struct i2c_client *client;
	struct lsm330dlc_acc_status *stat;
	int err = -1;
	
	dev_dbg(&pdev->dev, "start probing.");

	client = data->client;
	stat = kzalloc(sizeof(struct lsm330dlc_acc_status), GFP_KERNEL);
	if (!stat) {
		dev_err(&pdev->dev, "insufficient memory\n");
		err = -ENOMEM;
		goto err_free;
	}

	mutex_init(&stat->lock);
	mutex_lock(&stat->lock);

	stat->data = data;
	stat->client = client;

	stat->irq = pdata->irq_base + MX_HUB_IRQ_ACC;
	err = request_threaded_irq(stat->irq, 0, lsm330dlc_acc_isr,
		0, pdev->name, stat);
	if (unlikely(err < 0)) {
		dev_err(&pdev->dev, "mx hub: failed to request stat IRQ %d\n",
			stat->irq);
		goto err_mutexunlock;
	}
	
	//enable_irq(stat->irq);		

	stat->pdata = kmalloc(sizeof(*stat->pdata), GFP_KERNEL);
	if (stat->pdata == NULL) {
		err = -ENOMEM;
		dev_err(&pdev->dev, "failed to allocate memory for pdata\n");
		goto err_mutexunlock;
	}
	memcpy(stat->pdata, pdata->pacc, sizeof(*stat->pdata));

	err = lsm330dlc_acc_validate_pdata(stat);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}

	
	memset(stat->resume_state, 0, ARRAY_SIZE(stat->resume_state));
	stat->resume_state[RES_CTRL_REG1] = LSM330DLC_ACC_ENABLE_ALL_AXES;
	stat->resume_state[RES_CTRL_REG4] = CTRL_REG4_BDU_ENABLE;

	err = lsm330dlc_acc_device_power_on(stat);
	if (err < 0) {
		dev_err(&pdev->dev, "power on failed: %d\n", err);
		goto exit_kfree_pdata;
	}

	atomic_set(&stat->enabled, 1);
	dev_dbg(&pdev->dev, "device enabled\n");

	err = lsm330dlc_acc_update_fs_range(stat, stat->pdata->fs_range);
	if (err < 0) {
		dev_err(&pdev->dev, "update_fs_range failed\n");
		goto  err_power_off;
	}

	err = lsm330dlc_acc_update_odr(stat, stat->pdata->poll_interval);
	if (err < 0) {
		dev_err(&pdev->dev, "update_odr failed\n");
		goto  err_power_off;
	}

	err = lsm330dlc_acc_input_init(stat);
	if (err < 0) {
		dev_err(&pdev->dev, "input init failed\n");
		goto err_power_off;
	}

	err = create_sysfs_interfaces(stat);
	if (err < 0) {
		dev_err(&pdev->dev,
				"device LSM330DLC_ACC_DEV_NAME sysfs register failed\n");
		goto err_input_cleanup;
	}

	lsm330dlc_acc_device_power_off(stat);

	/* As default, do not report information */
	atomic_set(&stat->enabled, 0);
/*
#ifdef CONFIG_HAS_EARLYSUSPEND
	stat->early_suspend.suspend = lsm330dlc_acc_early_suspend;
	stat->early_suspend.resume = lsm330dlc_acc_early_resume;
	stat->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 20;
	register_early_suspend(&stat->early_suspend);
#endif
*/
	dev_set_drvdata(&pdev->dev,stat);

	mutex_unlock(&stat->lock);

	dev_dbg(&pdev->dev, "%s: probed\n", LSM330DLC_ACC_DEV_NAME);

	return 0;

err_input_cleanup:
	lsm330dlc_acc_input_cleanup(stat);
err_power_off:
	lsm330dlc_acc_device_power_off(stat);
exit_kfree_pdata:
	kfree(stat->pdata);
err_mutexunlock:
	mutex_unlock(&stat->lock);
	kfree(stat);
err_free:	
	pr_err("%s: Driver Init failed\n", LSM330DLC_ACC_DEV_NAME);
	return err;
}


static int __devexit lsm330dlc_acc_remove(struct platform_device *pdev)
{
	struct lsm330dlc_acc_status * stat = platform_get_drvdata(pdev);

	lsm330dlc_acc_device_power_off(stat);

	lsm330dlc_acc_input_cleanup(stat);
	remove_sysfs_interfaces(stat);

	kfree(stat->pdata);
	kfree(stat);
	return 0;
}

const struct platform_device_id lsm330dlc_acc_id[] = {
	{ "mx-hub-acc", 0 },
	{ },
};

static const struct dev_pm_ops lsm330dlc_acc_pm_ops = {
	.suspend	= lsm330dlc_acc_suspend,
	.resume	= lsm330dlc_acc_resume,
};

static struct platform_driver lsm330dlc_acc_driver = {
	.driver = {
		.name  = "mx-hub-acc",
		.owner = THIS_MODULE,
		.pm	= &lsm330dlc_acc_pm_ops,
	},
	.probe = lsm330dlc_acc_probe,
	.remove = __devexit_p(lsm330dlc_acc_remove),
	.id_table = lsm330dlc_acc_id,
};

static int __init lsm330dlc_acc_init(void)
{
	return platform_driver_register(&lsm330dlc_acc_driver);
}
module_init(lsm330dlc_acc_init);

static void __exit lsm330dlc_acc_exit(void)
{
	platform_driver_unregister(&lsm330dlc_acc_driver);
}
module_exit(lsm330dlc_acc_exit); 

MODULE_AUTHOR("Chwei <Chwei@meizu.com>");
MODULE_DESCRIPTION("MX Hub Sensor acc");
MODULE_LICENSE("GPL");
