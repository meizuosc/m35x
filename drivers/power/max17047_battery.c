/*
 *  max17047.c
 *
 *  Copyright (C) 2012 Maxim Integrated Product
 *  Gyungoh Yoo <jack.yoo@maxim-ic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/power/max17047_battery.h>
#include <asm/gpio.h>
#include <asm/mach-types.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/android_alarm.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <plat/adc.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/earlysuspend.h>
#include <mach/hardware.h>

#define MAX17047_DELAYED_TIME (60   * HZ)
#define MAX17047_SAVED_TIME   (3600 * 5)
#define MAX17047_RESTORE_TIME (3600 * 24)
#define SHUTDONW_VOLTAGE      3400
#define UV_TO_MU              1000
#define LEARNED_PARAM_LEN     16
#define BATID_ADC_CHANNEL     0
#define BATID_ADC_COUNT       20
#define CHECK_COUNT           2
#define CUSTOME_MODEL_SIZE    48
#define HIGH_TEMP_ILIM	450
#define LOW_TEMP_ILIM	0
#define MAX17047_MAGIC_NUM   17047

struct m3_learned_paras {
	bool saved_done;
    int magic_num;
	u16 parameters[LEARNED_PARAM_LEN];
};

struct max17047 {
	struct i2c_client *client;
	struct power_supply psy;
	int r_sns;
	bool current_sensing;
	int irq;
	struct workqueue_struct *battery_wq;
	struct delayed_work battery_work;
	struct delayed_work saved_work;
	struct alarm saved_alarm;
    struct alarm restored_alarm;
	struct wake_lock wake_lock;
	struct m3_learned_paras learned_parameters;

	/* battery info */
	int current_now;
	int current_avg;
	int voltage_now;
	int voltage_avg;
	int capacity;
    int prev_capacity;
	int temp;
	int present;
	int health;
	int cycles;
	bool done;
	unsigned long delay;
	int voltage_count;
#ifdef CONFIG_S3C_ADC
	struct s3c_adc_client *adc;
#endif
	char *manufacturer;
	bool temp_debug;
	bool voltage_debug;
	bool suspend_flag;
	int abnormal_temp_cnt;
	int recover_temp_cnt;
	bool vol_exceeded;
	bool soc_exceeded;
	bool fullcap_exceeded;
	struct early_suspend early_suspend;
	struct device *dev;
	bool clear_parameters;
    int first_flag;
    u32 version_id;
};

extern struct class *charging;

static enum power_supply_property max17047_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
#if 0
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_FULL,
#endif
};

static inline int max17047_write_reg(struct max17047 *max17047, u8 reg, u16 val)
{
	val = __cpu_to_le16(val);

	return i2c_smbus_write_i2c_block_data(max17047->client, reg, sizeof(u16), (u8 *)&val);
}

static inline int max17047_read_reg(struct max17047 *max17047, u8 reg, u16 *val)
{
	int ret;
	u16 tmp;

	if (!max17047->client || !max17047->client->adapter) {
		pr_info("%s:1the client is NULL\n", __func__);
		return -EINVAL;
	}
	ret = i2c_smbus_read_i2c_block_data(max17047->client, reg, sizeof(u16), (u8 *)&tmp);
	*val = __le16_to_cpu(tmp);

	return ret;
}

static inline int max17047_regs_show(struct max17047 *max17047)
{
	int ret, err;
	static int i;
	struct file *fp = NULL;
	mm_segment_t pos;
	char buf[10] = {0};
	char time_buf[32] = {0};
	u16 buf1 = 0;
	u16 buf2 = 0;
	struct timespec ts;
	struct rtc_time tm;

	/* get the system current time */
	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
	sprintf(time_buf, "%d-%02d-%02d %02d:%02d:%02d %s",
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			tm.tm_hour + 8, tm.tm_min, tm.tm_sec, " ");

	pos = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open("/sdcard/android/max17047_datalog.txt", O_RDWR | O_APPEND | O_CREAT, 0644);
	if (IS_ERR(fp)) {
		pr_info("create file failed******\n");
		err = PTR_ERR(fp);
		if (err != -ENOENT )
			pr_err("%s:open the file failed\n", __func__);
		set_fs(pos);
		max17047->delay = msecs_to_jiffies(20000);
		return err;
	}

	if (max17047->done) {
		/* record the current system time */
		err = fp->f_op->write(fp, time_buf, 21, &fp->f_pos);

		for (i = 0; i <= 0x4f; i++) {
			sprintf(buf, "0x%02x %s", i," ");
			err = fp->f_op->write(fp, buf, 6, &fp->f_pos);
		}

		for (i = 0xE0; i <= 0xff; i++) {
			if (i == 0xff) {
				sprintf(buf, "0x%02x %s", i, "\n");
			} else {
				sprintf(buf, "0x%02x %s", i, " ");
			}
			err = fp->f_op->write(fp, buf, 6, &fp->f_pos);
		}
		max17047->done = false;
	}

	err = fp->f_op->write(fp, time_buf, 21, &fp->f_pos);

	for (i = 0; i <= 0x4f; i++) {
		ret = max17047_read_reg(max17047, (u8)i, &buf1);
		sprintf(buf, "%04x %s", buf1," ");
		err = fp->f_op->write(fp, buf, 6, &fp->f_pos);
	}

	for (i = 0xE0; i <= 0xff; i++) {
		ret = max17047_read_reg(max17047, (u8)i, &buf2);
		if (i == 0xff) {
			sprintf(buf, "%04x %s", buf2, "\n");
		} else {
			sprintf(buf, "%04x %s", buf2, " ");
		}
		err = fp->f_op->write(fp, buf, 6, &fp->f_pos);
	}

	set_fs(pos);
	filp_close(fp, NULL);

	return ret;
}

static int max17047_verify_reg(struct max17047 *max17047, u16 reg,
		u16 buf)
{
	s32 ret = 0;
	u16 val;
	int i = 0;

	for (i = 0; i < RETRY_CNT; ++i) {
		ret = max17047_write_reg(max17047, reg, buf);
		if (unlikely(ret < 0))
			return ret;

		ret = max17047_read_reg(max17047, reg, &val);
		if (unlikely(ret < 0))
			return ret;

		if ((likely(buf == val)))
			break;
	}
	return ret;
}

static void set_alarm(struct alarm *alarm, long seconds)
{
	ktime_t interval = ktime_set(seconds, 0);
	ktime_t now = alarm_get_elapsed_realtime();
	ktime_t next = ktime_add(now, interval);

	pr_info("set saved alarm after %ld seconds\n", seconds);

	alarm_start_range(alarm, next, next);
}

static int max17047_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct max17047 *max17047 = container_of(psy,
			struct max17047, psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (val->intval) {
			max17047->voltage_now = val->intval;
			max17047->voltage_debug = true;
		} else {
			max17047->voltage_debug = false;
		}
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		if (val->intval) {
			max17047->voltage_avg = val->intval;
			max17047->voltage_debug = true;
		} else {
			max17047->voltage_debug = false;
		}
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (val->intval) {
			max17047->temp = val->intval;
			max17047->temp_debug = true;
		} else {
			max17047->temp_debug = false;
		}
		break;
	default:
		break;
	}

	return 0;
}

static int max17047_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct max17047 *max17047 = container_of(psy, struct max17047, psy);
	u16 value = 0;
	int ret;
	
	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = max17047->present;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = max17047_read_reg(max17047, MAX17047_REG_CURRENT, &value);
			if (likely(ret >= 0)) {
				/* unit = 1.5625uV / RSENSE */
				max17047->current_now =
					(int)(s16)value / (max17047->r_sns * 10) *15625;
			}
		val->intval = max17047->current_now;
		break;

	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = max17047->current_avg;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = max17047->voltage_now;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		val->intval = max17047->voltage_avg;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = max17047->capacity;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		val->intval = max17047->temp;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = max17047->health;
		break;
    case POWER_SUPPLY_PROP_MANUFACTURER:
        val->strval = max17047->manufacturer; 
        break;

    case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	    ret = max17047_read_reg(max17047, MAX17047_REG_MAXMINVCELL, &value);
	    if (likely(ret >= 0)) {
		    /* unit = 20mV */
		    val->intval = (value >> 8) * 20000;
	    }
	    break;

    case POWER_SUPPLY_PROP_VOLTAGE_MIN:
	    ret = max17047_read_reg(max17047, MAX17047_REG_MAXMINVCELL, &value);
	    if (likely(ret >= 0)) {
		    /* unit = 20mV */
		    val->intval = (value & 0xFF) * 20000;
	    }
	    break;

#if 0
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		ret = max17047_read_reg(max17047, MAX17047_REG_FULLCAP, &value);
		if (value > 0x1296) {
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
		} else if (value < 0x0FA0) {
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		} else {
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		}
		break;

	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = max17047->cycles;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		if (max17047->r_sns >= 0) {
			ret = max17047_read_reg(max17047, MAX17047_REG_FULLCAPNOM, &value);
			if (likely(ret >= 0)) {
				/* unit = 5.0uVh / RSENSE */
				val->intval = (int)value * 5 * 1000 / max17047->r_sns;
			}
		} else {
			return -EINVAL;
		}
		break;
#endif
	default:
		return -EINVAL;
	}
	return 0;
}

static int get_current_info(struct max17047 *max17047)
{
	int ret;
	u16 value = 0;

	if (max17047->r_sns >= 0) {
		ret = max17047_read_reg(max17047, MAX17047_REG_CURRENT, &value);
		if (likely(ret >= 0)) {
			/* unit = 1.5625uV / RSENSE */
			max17047->current_now = 
				(int)(s16)value / (max17047->r_sns * 10) *15625;
		}

		ret = max17047_read_reg(max17047, MAX17047_REG_AVERAGECURRENT, &value);
		if (likely(ret >= 0)) {
			/* unit = 1.5625uV / RSENSE */
			max17047->current_avg = 
				(int)(s16)value  * 15625 / max17047->r_sns / 10;
		}
	} else {
		return -EINVAL;
	}

	return ret;
}

static int get_voltage_info(struct max17047 *max17047) {
	int ret;
	u16 value, miscfg = 0;

	ret = max17047_read_reg(max17047, MAX17047_REG_MISCCFG, &miscfg);
	if (miscfg & 0x04) {
		ret = max17047_verify_reg(max17047, MAX17047_REG_MISCCFG, miscfg & 0xfffb);
	}

	ret = max17047_read_reg(max17047, MAX17047_REG_VCELL, &value);
	if (likely(ret >= 0)) {
		/* unit = 0.625mV */
		if (!max17047->voltage_debug)
			max17047->voltage_now = (value >> 3) * 625;
	}

	ret = max17047_read_reg(max17047, MAX17047_REG_AVERAGEVCELL, &value);
	if (likely(ret >= 0)) {
		/* unit = 0.625mV */
		if (!max17047->voltage_debug)
			max17047->voltage_avg = (value >> 3) * 625;
	}
	if (max17047->voltage_avg <= SHUTDONW_VOLTAGE * UV_TO_MU)
		max17047->voltage_count++;

	return ret;
}

static int get_capacity_info(struct max17047 *max17047)
{
	int ret;
	u16 value = 0, capacity = 0;

	if (max17047->voltage_debug) {
		if (max17047->voltage_count >= CHECK_COUNT) {
			max17047->capacity = 0;
			max17047->voltage_count = 0;
			pr_info("the voltage is low, shutdown\n");
		}
		return max17047->capacity;
    	}

	ret = max17047_read_reg(max17047,
			max17047->current_sensing ? MAX17047_REG_SOCREP : MAX17047_REG_SOCVF,
			&value);
	if (likely(ret >= 0)) {
		if (max17047->voltage_count >= CHECK_COUNT) {
			max17047->capacity = 0;
			max17047->voltage_count = 0;
			pr_info("the voltage is low, shutdown\n");
		} else if (max17047->vol_exceeded) {
            max17047_regs_show(max17047);
			capacity = min((value + 0x0080) >> 8, 100);
            if (capacity >= 3) {
                max17047->capacity = capacity;
            } else {
                max17047->capacity = 0;
            }
			max17047->vol_exceeded = false;
        } else if (!max17047->present) {
			max17047->capacity = 21;
		} else {
			/* unit = (1 / 256) % */
			capacity = min((value + 0x0080) >> 8, 100);

            if ((max17047->first_flag != 1) && (capacity - max17047->prev_capacity) > 1) {
                pr_info("%s: the SOCREP occurred jump\n", __func__);
                max17047->first_flag = 3;
                ret = max17047_read_reg(max17047, MAX17047_REG_CURRENT, &value);
                if (value & 0x8000) {
                    pr_info("%s: now it is discharging\n", __func__);
                    max17047->capacity = max17047->prev_capacity;
                } else {
                    pr_info("%s: now it is charging\n", __func__);
                    max17047->capacity = max17047->prev_capacity + 1;
                }
            } else {
                max17047->capacity = capacity;
            }
		}
        max17047->prev_capacity = max17047->capacity;
        max17047->first_flag = 2;
	}

	if (max17047->capacity == 100 && !max17047->fullcap_exceeded) {
		ret = max17047_read_reg(max17047, MAX17047_REG_FULLCAP, &value);
		/* if the fullcap is more than 2500mAh, that mean the fuelgauge is unnormal
		 we should load the custome again, and erase the saved parameters */
		if (value > 0x12E8) {
			pr_info("%s:***0x%02x****\n", __func__, value);
			max17047->fullcap_exceeded = true;
		}
	}
	return ret;
}

static int get_temperature_info(struct max17047 *max17047) 
{
	int ret = 0;
	u16 value = 0;

#ifdef CONFIG_MEIZU_M6X_BOARD
	max17047->temp = 250;
#else
	if (max17047->present) {
		ret = max17047_read_reg(max17047, MAX17047_REG_TEMPERATURE, &value);
		if (likely(ret >= 0)) {
			/* unit = tenths of degree Celsius */
			if (!max17047->temp_debug)
				max17047->temp = ((int)(short)value * 10) / 256;
		}
	} else {
		max17047->temp = 250;
	}
#endif
	if (max17047->temp >= 600 || max17047->temp <= -200) {
		pr_info("the temp is more than 60oC or less than -20oC, shutdown\n");
	}

	return max17047->temp;
}

static int get_battery_online(struct max17047 *max17047)
{
	int ret;
	u16 value;
	int present = 0;

	ret = max17047_read_reg(max17047, MAX17047_REG_STATUS, &value);
	if (likely(ret >= 0)) {
		/* Bst is 0 when a battery is present */
		present = value & MAX17047_R_BST ? 0 : 1;
	}

	max17047->present = present;

	return ret;
}

static int get_battery_health(struct max17047 *max17047)
{
	int temp = max17047->temp;

	if (temp > HIGH_TEMP_ILIM || temp < LOW_TEMP_ILIM) {
		if (max17047->abnormal_temp_cnt < CHECK_COUNT) {
			max17047->abnormal_temp_cnt++;
		}
	} else {
		if (max17047->health != POWER_SUPPLY_HEALTH_GOOD) {
			if (max17047->recover_temp_cnt < CHECK_COUNT)
				max17047->recover_temp_cnt++;
		}
	}

	if (max17047->abnormal_temp_cnt >= CHECK_COUNT) {
		max17047->health = (temp > HIGH_TEMP_ILIM) ? POWER_SUPPLY_HEALTH_OVERHEAT : POWER_SUPPLY_HEALTH_COLD;
		max17047->recover_temp_cnt = 0;
		max17047->abnormal_temp_cnt = 0;
	} else if (max17047->recover_temp_cnt >= CHECK_COUNT) {
		max17047->health = POWER_SUPPLY_HEALTH_GOOD;
		max17047->recover_temp_cnt = 0;
		max17047->abnormal_temp_cnt = 0;
	}

	return max17047->health;
}

static void get_battery_info(struct max17047 *max17047)
{
	get_battery_online(max17047);
	get_current_info(max17047);
	get_voltage_info(max17047);
	get_capacity_info(max17047);
	get_temperature_info(max17047);
	get_battery_health(max17047);
}

static int batid_adc_read(struct max17047 *max17047)
{
	int value, sum = 0;
	int i;

	for (i = 0; i < BATID_ADC_COUNT; ++i) {
		value = s3c_adc_read(max17047->adc, BATID_ADC_CHANNEL);
		sum += value;
	}
	value = sum / BATID_ADC_COUNT;
	if ((value >= 1820) && (value <= 2275)) {
		max17047->manufacturer = "SONY";
	} else {
		max17047->manufacturer = "NULL";
	}

	pr_info("%s: sum value is %d\n", __func__, value);

#ifdef CONFIG_MEIZU_M65_V3
	max17047->manufacturer = "ATL";
#endif

	return value;
}

static int max17047_restore_learned_parameters(struct max17047 *max17047)
{
	int ret;
	u16 fullcap0 = 0, dq_acc = 0, remcap = 0, socmix = 0, vf_fullcap = 0;

	pr_info("entern %s:***************\n", __func__);

	/*read the saved parameters from the blocks */
	ret = read_battery_model(sizeof(max17047->learned_parameters.parameters),
			max17047->learned_parameters.parameters);
	if (ret != 0) {
		pr_err("read the block failed\n");
	}

#ifdef CONFIG_BATTERY_MAX17047_DEBUG
	pr_info("qr0(0x%02x), qr1(0x%02x), qr2x%02x, qr3(0x%02x),(rcomp0)0x%02x,"
			"(tempco)0x%02x, (fullcap)0x%02x, (fullcapnom)0x%02x,"
			"(iavg_empty)0x%02x, (cycles)0x%02x, (flag)%d, magic_num %d\n",
			max17047->learned_parameters.parameters[0],
			max17047->learned_parameters.parameters[1],
			max17047->learned_parameters.parameters[2],
			max17047->learned_parameters.parameters[3],
			max17047->learned_parameters.parameters[4],
			max17047->learned_parameters.parameters[5],
			max17047->learned_parameters.parameters[6],
			max17047->learned_parameters.parameters[7],
			max17047->learned_parameters.parameters[8],
			max17047->learned_parameters.parameters[9],
			max17047->learned_parameters.parameters[10],
			max17047->learned_parameters.parameters[11]);
#endif

    /* check the magic_num */
    max17047->learned_parameters.magic_num = max17047->learned_parameters.parameters[11];
    pr_info("%s: magic_num is %d\n", __func__, max17047->learned_parameters.magic_num);
    if (max17047->learned_parameters.magic_num == MAX17047_MAGIC_NUM) {
        /* when replacing the battery,the asved_done flag will be set 0,
           the lalearned parameters will be invalid before.
         */
        max17047->learned_parameters.saved_done =
            max17047->learned_parameters.parameters[10];
        pr_debug("max17047->learned_parameters.saved_done is %d\n",
                max17047->learned_parameters.saved_done);

        if (max17047->learned_parameters.saved_done) {
            /* restore the learned parameters */
            ret = max17047_write_reg(max17047, MAX17047_REG_VEMPTY,
                    max17047->learned_parameters.parameters[0]);
            ret = max17047_write_reg(max17047, MAX17047_REG_QRESIDUAL10,
                    max17047->learned_parameters.parameters[1]);
            ret = max17047_write_reg(max17047, MAX17047_REG_QRESIDUAL20,
                    max17047->learned_parameters.parameters[2]);
            ret = max17047_write_reg(max17047, MAX17047_REG_QRESIDUAL30,
                    max17047->learned_parameters.parameters[3]);

            /* T22. wait 350ms */
            msleep(350);

            /* T23. restore fullcap */
            vf_fullcap = max17047->learned_parameters.parameters[6];

            ret = max17047_read_reg(max17047, 0x35, &fullcap0);
            ret = max17047_read_reg(max17047, MAX17047_REG_SOCMIX, &socmix);
            remcap = socmix *fullcap0 / 25600;
            ret = max17047_verify_reg(max17047, MAX17047_REG_REMCAPMIX, remcap);
            ret = max17047_verify_reg(max17047, MAX17047_REG_FULLCAP, vf_fullcap);

            if (machine_is_m69() || (max17047->version_id >= 2)) {
                /* qq_acc = (saved_fullcapnom / 4) */
                dq_acc = (max17047->learned_parameters.parameters[7] / 4);
                ret = max17047_verify_reg(max17047, MAX17047_REG_DQACC, dq_acc);
                ret = max17047_verify_reg(max17047, MAX17047_REG_DPACC, 0X3200);
            }

            ret = max17047_write_reg(max17047, MAX17047_REG_RCOMP0,
                    max17047->learned_parameters.parameters[4]);
            ret = max17047_write_reg(max17047, MAX17047_REG_TEMPCO,
                    max17047->learned_parameters.parameters[5]);
            ret = max17047_write_reg(max17047, MAX17047_REG_IAVG_EMPTY,
                    max17047->learned_parameters.parameters[8]);

            /*T25. save the cycles */
            ret = max17047_write_reg(max17047, MAX17047_REG_CYCLES,
                    max17047->learned_parameters.parameters[9]);
            max17047->cycles = max17047->learned_parameters.parameters[9];
            if (max17047->cycles > 0xFF)
                ret = max17047_verify_reg(max17047, MAX17047_REG_LEARNCFG, 0X2676);
        }
    }

	return ret;
}

static int max17047_write_model(struct max17047 *max17047)
{
	int i;
	int ret;
	char *buf;
	u16 tmp[48] = {0};
	u32 sum = 0;
	int retry = RETRY_CNT;
	static u16 model_reg[] = {0x80, 0x90, 0xA0};
	static u16 model_data[CUSTOME_MODEL_SIZE] = {0};
	static const u16 atl_model_data[CUSTOME_MODEL_SIZE] = {
		0xAAA0, 0xB730, 0xB8F0, 0xBAC0, 0xBBE0, 0xBD10, 0xBE30, 0xBF50,
		0xC0A0, 0xC1F0, 0xC430, 0xC670, 0xC940, 0xCC10, 0xD1A0, 0xD760,
		0x0200, 0x0F70, 0x0EE0, 0x18C0, 0x1770, 0x1820, 0x1820, 0x0EC0,
		0x0EC0, 0x09B0, 0x09B0, 0x08F0, 0x08F0, 0x06F0, 0x06C0, 0x06C0,
		0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100,
		0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100,
	};
#if 0
	/* SONY BATTERY SHUTDOWN VOLTAGE IS 3.3V */
	static const u16 sony_model_data[CUSTOME_MODEL_SIZE] = {
		0X9880, 0xB3B0, 0xB8C0, 0xBA20, 0xBC00, 0xBC70, 0xBD20, 0xBE20,
		0xBF20, 0xC100, 0xC3E0, 0xC6D0, 0xCBD0, 0xCEF0, 0xD1D0, 0xD910,
		0x0040, 0x0820, 0x1160, 0x1030, 0x2010, 0x1F20, 0x1810, 0x1930,
		0x0FF0, 0x0A90, 0x0A10, 0x08C0, 0x06D0, 0x08F0, 0x0570, 0x0570,
		0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100,
		0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100,
	};
#endif
	/* SONY BATTERY SHUTDOWN VOLTAGE IS 3.4V */
	static const u16 sony_model_data[CUSTOME_MODEL_SIZE] = {
		0xAA00, 0xB590, 0xB740, 0xB9B0, 0xBB20, 0xBC90, 0xBD70, 0xBE50,
		0xBFA0, 0xC100, 0xC2B0, 0xC5E0, 0xC980, 0xCD20, 0xD1E0, 0xD770,
		0x0130, 0x0E30, 0x0A00, 0x1400, 0x1430, 0x1D00, 0x1D00, 0x1210,
		0x1140, 0x0A50, 0x0A50, 0x08B0, 0x08F0, 0x06F0, 0x05E0, 0x05E0,
		0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100,
		0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100,
	};

	if (!strcmp(max17047->manufacturer, "SONY"))
		memcpy(model_data, sony_model_data, sizeof(sony_model_data));
	else
		memcpy(model_data, atl_model_data, sizeof(atl_model_data));

	/* T4. Unlock Model Access */
	ret = max17047_verify_reg(max17047, 0x62, 0x0059);
	ret = max17047_verify_reg(max17047, 0x63, 0x00c4);

	/* T5. write/read/verify the custom model */
	for (i = 0; i < 3; ++i) {
		do {
			buf = (char *)(model_data + i * 16);
			i2c_smbus_write_i2c_block_data(max17047->client, model_reg[i],
					32, buf);
			i2c_smbus_read_i2c_block_data(max17047->client, model_reg[i],
					32, (char *)tmp);
			if (!memcmp(buf, tmp, 32))
				break;
			else {
				pr_info("%s:write model error, write again\n",
						__func__);
			}
		} while (retry--);
	}

	/* T8. Lock Model Access; T9. Verify Model Access
	   If the model remains unlocked, rhe max17047 won't be able
	   to monitor the capacity of the battery
	 */
	for (i = 0; i < RETRY_CNT; ++i) {
		max17047_verify_reg(max17047, 0x62, 0x0000);
		max17047_verify_reg(max17047, 0x63, 0x0000);

		/* verify the model is locked */
		i2c_smbus_read_i2c_block_data(max17047->client, 0x80, 32, (char *)&tmp[0]);
		i2c_smbus_read_i2c_block_data(max17047->client, 0x90, 32, (char *)&tmp[16]);
		i2c_smbus_read_i2c_block_data(max17047->client, 0xA0, 32, (char *)&tmp[32]);

		/*if the model is locked, the values is zero*/
		for (i = 0; i < 48; ++i) {
			sum += tmp[i];
		}
		if (sum == 0) 
			break;
		else {
			pr_info("verify error, lock model access failed %d\n",
					sum);
		}
	}
	return ret;
}

static void max17047_write_parameters(struct max17047 *max17047)
{
	u16 rcomp0, tempco, qrtable00, qrtable10, qrtable20, qrtable30, v_empty, cgain;

	if (!strcmp(max17047->manufacturer, "SONY")) {
		rcomp0 = 0x004F;
		tempco = 0x2B2F;
		v_empty = 0xACDA;
		qrtable00 = 0x3400;
		qrtable10 = 0x1B00;
		qrtable20 = 0x1200;
		qrtable30 = 0x0D01;
	} else {
		rcomp0 = 0x004B;
		tempco = 0x1C2E;
		v_empty = 0xACDA;
		qrtable00 = 0x4180;
		qrtable10 = 0x1C80;
		qrtable20 = 0x0A02;
		qrtable30 = 0x0802;
	}

	if (machine_is_m69() || (max17047->version_id >= 2)) {
		cgain = 0x4000;
	} else if (machine_is_m65()) {
		cgain = 0x33E1;
	}

	max17047_write_reg(max17047, MAX17047_REG_RCOMP0, rcomp0);
	max17047_write_reg(max17047, MAX17047_REG_TEMPCO, tempco);/*hot temp, cold temp*/
	max17047_write_reg(max17047, MAX17047_REG_ICHGTERM, 0x0280); /*100MA*/
	max17047_write_reg(max17047, MAX17047_REG_AIN, 0x88D0);
	max17047_write_reg(max17047, MAX17047_REG_TGAIN, 0xE3E1);
	max17047_write_reg(max17047, MAX17047_REG_TOFF, 0x290E);
	max17047_write_reg(max17047, MAX17047_REG_CGAIN, cgain);
	max17047_write_reg(max17047, MAX17047_REG_COFF, 0X0000);
	max17047_write_reg(max17047, MAX17047_REG_V_EMPTY, v_empty);/*Empty vol, recvoery vol*/
	max17047_write_reg(max17047, MAX17047_REG_VEMPTY, qrtable00);
	max17047_write_reg(max17047, MAX17047_REG_QRESIDUAL10, qrtable10);
	max17047_write_reg(max17047, MAX17047_REG_QRESIDUAL20, qrtable20);
	max17047_write_reg(max17047, MAX17047_REG_QRESIDUAL30, qrtable30);
}

static int max17047_import_battery_model(struct max17047 *max17047)
{
	int ret;
	u16 val = 0;
	u16 vfsoc = 0, qh = 0, remcap = 0, repcap = 0, filtercfg = 0;
	u16 vf_fullcap, dq_acc;

	if (!strcmp(max17047->manufacturer, "SONY")) {
		vf_fullcap = 0x1296;
	} else {
		vf_fullcap = 0x118A;
	}

	/* T1. delay 500mS */
	msleep(500);

	/* T2. initialize configuration */
	ret = max17047_write_reg(max17047, MAX17047_REG_CONFIG, 0x2210);
	if (unlikely(ret < 0)) {
		dev_err(&max17047->client->dev, "set CONFIG failed\n");
		return ret;
	}

    if (machine_is_m69() || (max17047->version_id >= 2)) {
        max17047_write_reg(max17047, MAX17047_REG_FILTERCFG, 0x87A4);
        max17047_write_reg(max17047, MAX17047_REG_MISCCFG, 0x0870);
    } else if (max17047->version_id <= 1) {
        /* change the mixing time 3.2hrs by changing (MIX3:MIX0) in FilterCFG from Dh to Bh */
        ret = max17047_read_reg(max17047, MAX17047_REG_FILTERCFG, &filtercfg);
        max17047_write_reg(max17047, MAX17047_REG_FILTERCFG, filtercfg & 0xfdff);
        /* change mixing rate (MR4:MR0) in MiscCFG to 0 */
        ret = max17047_read_reg(max17047, MAX17047_REG_MISCCFG, &val);
        ret = max17047_write_reg(max17047, MAX17047_REG_MISCCFG, val & 0xfc1b);
    }

    max17047_write_reg(max17047, MAX17047_REG_RELAXCFG, 0X506b);
	max17047_write_reg(max17047, MAX17047_REG_LEARNCFG, 0x2606);
	max17047_write_reg(max17047, MAX17047_REG_FULLSOCTHR, 0x5A00);

	/* Load Custom Model and Parameters */
	max17047_write_model(max17047);

	/* T10. Write Custom Parameters */
	max17047_write_parameters(max17047);

	/* T11. Update Full Capacity Parameters */
	max17047_verify_reg(max17047, MAX17047_REG_FULLCAP, vf_fullcap);
	max17047_write_reg(max17047, MAX17047_REG_DESIGNCAP, vf_fullcap);
	max17047_verify_reg(max17047, MAX17047_REG_FULLCAPNOM, vf_fullcap);

	/* T13. delay 350mS*/
	msleep(350);

	/* T14. Write VFSOC and OH */
	ret = max17047_read_reg(max17047, MAX17047_REG_SOCVF, &vfsoc);
	max17047_write_reg(max17047, 0x60, 0x0080); /* enable write access to VFSOCO */
	max17047_verify_reg(max17047, 0x48, vfsoc); /* write and verify VFSC0*/
	max17047_write_reg(max17047, 0x60, 0x0000); /* disable write access to VFSOCO */
	max17047_read_reg(max17047, MAX17047_REG_QH, &qh);
	max17047_write_reg(max17047, 0x4C, qh); /* write QH to QH0 */

	/* T15.5. Advance To Coulomb-Counter Mode */
	max17047_verify_reg(max17047, MAX17047_REG_CYCLES, 0x0060);

	/* T16. Load New Capacity Parameters */
	remcap = (vfsoc * vf_fullcap) / 25600;
	repcap = remcap * (vf_fullcap / vf_fullcap) / 1;
	max17047_verify_reg(max17047, MAX17047_REG_REMCAPMIX, remcap);
	max17047_verify_reg(max17047, MAX17047_REG_REMCAPREP, repcap);

    if (machine_is_m69() || (max17047->version_id >= 2)) {
        dq_acc = vf_fullcap / 4;
        max17047_verify_reg(max17047, MAX17047_REG_DQACC, dq_acc);
        max17047_verify_reg(max17047, MAX17047_REG_DPACC, 0x3200);
    } else if (max17047->version_id <= 1) {
        dq_acc = 0x1296;
        max17047_verify_reg(max17047, MAX17047_REG_DQACC, dq_acc);
        max17047_verify_reg(max17047, MAX17047_REG_DPACC, 0xC800);
    }
    
    max17047_verify_reg(max17047, MAX17047_REG_FULLCAP, vf_fullcap);
	max17047_write_reg(max17047, MAX17047_REG_DESIGNCAP, vf_fullcap);
	max17047_verify_reg(max17047, MAX17047_REG_FULLCAPNOM, vf_fullcap);

	/* Update SOC register with the new SOC */
	max17047_write_reg(max17047, MAX17047_REG_SOCREP, vfsoc);

	/*T17. Initialization Comlete , Clear the POR */
	max17047_read_reg(max17047, MAX17047_REG_STATUS, &val);
	max17047_write_reg(max17047, MAX17047_REG_STATUS, val & 0xFFFD);

	/* if the battery pack has saved parameters, the goto Step 21 */
	/* T21. restoring capacity parameters */
	ret = max17047_restore_learned_parameters(max17047);

	return ret;
}

static void max17047_saved_learned_parameters(struct work_struct *work)
{
	struct max17047 *max17047 = container_of(work, struct max17047,
			saved_work.work);
	int ret;
	static u16 saved_parameters[LEARNED_PARAM_LEN] = {0};
    int saved_count = 0;

	pr_info("entern %s:***************\n", __func__);

	/* T20.
	   save the qrtable00, qrtable01, qrtable02, qrtable03 parameters */
	ret = max17047_read_reg(max17047, MAX17047_REG_VEMPTY,
			max17047->learned_parameters.parameters);
	ret = max17047_read_reg(max17047, MAX17047_REG_QRESIDUAL10,
			max17047->learned_parameters.parameters+1);
	ret = max17047_read_reg(max17047, MAX17047_REG_QRESIDUAL20,
			max17047->learned_parameters.parameters+2);
	ret = max17047_read_reg(max17047, MAX17047_REG_QRESIDUAL30,
			max17047->learned_parameters.parameters+3);

	/* save the capacity information */
	ret = max17047_read_reg(max17047, MAX17047_REG_RCOMP0,
			max17047->learned_parameters.parameters+4);
	ret = max17047_read_reg(max17047, MAX17047_REG_TEMPCO,
			max17047->learned_parameters.parameters+5);
	ret = max17047_read_reg(max17047, MAX17047_REG_FULLCAP,
			max17047->learned_parameters.parameters+6);
	ret = max17047_read_reg(max17047, MAX17047_REG_FULLCAPNOM,
			max17047->learned_parameters.parameters+7);
	ret = max17047_read_reg(max17047, MAX17047_REG_IAVG_EMPTY,
			max17047->learned_parameters.parameters+8);

	/* save the cycles */
	ret = max17047_read_reg(max17047, MAX17047_REG_CYCLES,
			max17047->learned_parameters.parameters+9);

	/* save the write flag */
	max17047->learned_parameters.parameters[10] = (1 & 0x00ff);

    max17047->learned_parameters.parameters[11] = MAX17047_MAGIC_NUM;

#ifdef CONFIG_BATTERY_MAX17047_DEBUG
	pr_info("qr0(0x%02x), (qr1)0x%02x, (qr2)0x%02x, (qr3)0x%02x,"
			"(rcomp0)0x%02x, (tempco)0x%02x, (fullcap)0x%02x,"
			"(fullcapnom)0x%02x,(iavg_empty)0x%02x, (cycles)0x%02x, (flag)%d, magic_num %d\n",
			max17047->learned_parameters.parameters[0],
			max17047->learned_parameters.parameters[1],
			max17047->learned_parameters.parameters[2],
			max17047->learned_parameters.parameters[3],
			max17047->learned_parameters.parameters[4],
			max17047->learned_parameters.parameters[5],
			max17047->learned_parameters.parameters[6],
			max17047->learned_parameters.parameters[7],
			max17047->learned_parameters.parameters[8],
			max17047->learned_parameters.parameters[9],
			max17047->learned_parameters.parameters[10],
			max17047->learned_parameters.parameters[11]);
#endif

	memcpy(saved_parameters, max17047->learned_parameters.parameters,
			sizeof(max17047->learned_parameters.parameters));

	/* write the learned parameters to the blocks */
	ret = write_battery_model(sizeof(max17047->learned_parameters.parameters),
			max17047->learned_parameters.parameters);
	if (ret != 0) {
		pr_err("write the block failed\n");
	}

	/* read the saved learned parameters form the block to verify the parameters */
	ret = read_battery_model(sizeof(max17047->learned_parameters.parameters),
			max17047->learned_parameters.parameters);
	if (ret != 0) {
		pr_err("read the block failed\n");
	}

	/* if write saved parameters to the block isn't equal to the write block before,
	 clear the saved data and write again */
	while (memcmp(saved_parameters, max17047->learned_parameters.parameters, 24) != 0) {
		pr_info("%s:the saved_parameters error\n", __func__);
		/* first, clear the saved data */
		memset(max17047->learned_parameters.parameters, '\0', sizeof(max17047->learned_parameters.parameters));
		ret = write_battery_model(sizeof(max17047->learned_parameters.parameters),
                    max17047->learned_parameters.parameters);
		if (ret != 0) {
			pr_err("write the block failed\n");
		}
		/* should write again */
		ret = write_battery_model(sizeof(max17047->learned_parameters.parameters),
				saved_parameters);

		ret = read_battery_model(sizeof(max17047->learned_parameters.parameters),
				max17047->learned_parameters.parameters);
		if (ret != 0) {
			pr_err("read the block failed\n");
		}
        if (saved_count > 5)
            break;
        saved_count++;
	}
}

static int max17047_hw_init(struct max17047 *max17047)
{
	int ret;
	u16 val, remcaprep = 0, socrep = 0, tte = 0;
    u16 misccfg = 0, filtercfg = 0, dq_acc = 0x1296;

	/* T0. check for the POR */
	ret = max17047_read_reg(max17047, MAX17047_REG_STATUS, &val);
	if (unlikely(ret < 0)) {
		dev_err(&max17047->client->dev, "POR read failed\n");
		return ret;
	}

	if (!(val & 0x02)) {
		pr_info("POR is not set\n");
		/* GO T18. read the reported capacity and SOC*/
		ret = max17047_read_reg(max17047, MAX17047_REG_REMCAPREP, &remcaprep);
		ret = max17047_read_reg(max17047, MAX17047_REG_SOCREP, &socrep);
		/* T19: read time to empty*/
		ret = max17047_read_reg(max17047, MAX17047_REG_TTE, &tte);

    if (machine_is_m69() || (max17047->version_id >= 2)) {
        max17047_write_reg(max17047, MAX17047_REG_FILTERCFG, 0x87A4);
        max17047_write_reg(max17047, MAX17047_REG_MISCCFG, 0x0870);
    } else if (max17047->version_id <= 1) {
        /* change mixing rate (MR4:MR0) in MiscCFG to 0 */
        ret = max17047_read_reg(max17047, MAX17047_REG_MISCCFG, &misccfg);
        max17047_write_reg(max17047, MAX17047_REG_MISCCFG, misccfg & 0xfc1b);

        /* change the mixing time 3.2hrs by changing (MIX3:MIX0) in FilterCFG from Dh to Bh */
        ret = max17047_read_reg(max17047, MAX17047_REG_FILTERCFG, &filtercfg);
        max17047_write_reg(max17047, MAX17047_REG_FILTERCFG, filtercfg & 0xfdff);

        max17047_verify_reg(max17047, MAX17047_REG_DQACC, dq_acc);
        max17047_verify_reg(max17047, MAX17047_REG_DPACC, 0xC800);
    }

		/* T20: Saved Learned Parameters */
		queue_delayed_work(max17047->battery_wq, &max17047->saved_work, 0);
	} else {
		pr_info("%s: POR is set\n", __func__);
		/*POR is set, we must load the custome model and restore the saved parameters*/
		max17047_import_battery_model(max17047);
	}

	set_alarm(&max17047->saved_alarm, MAX17047_SAVED_TIME);

    if (machine_is_m65() && (max17047->version_id <= 1))
        set_alarm(&max17047->restored_alarm, MAX17047_RESTORE_TIME);

	return ret;
}

static int max17047_init_threshold(struct max17047 *max17047)
{
	int ret;
	u16 value = 0;

	/* when soc is 10%, wakeup and redmine the system*/
	ret = max17047_write_reg(max17047, MAX17047_REG_SOCALRT_TH, 0xC80A);

	/* when voltage is 3.35V(with 20mV resolution), wakeup and redmine the system */
	ret = max17047_write_reg(max17047, MAX17047_REG_VALRT_TH, 0xffa7);

	/* SOC ALERT CONFIG SOCREP:00 */
	ret = max17047_read_reg(max17047, MAX17047_REG_MISCCFG, &value);
	ret = max17047_write_reg(max17047, MAX17047_REG_MISCCFG, value & 0xfffc);

	/* Enable alert on fuelgauge output */
	ret = max17047_read_reg(max17047, MAX17047_REG_CONFIG, &value);
	max17047_write_reg(max17047, MAX17047_REG_CONFIG, value | 0x4);

	return ret;
}

static irqreturn_t max17047_isr(int irq, void *irq_data)
{
	struct max17047 *max17047 = irq_data;
	int ret;
	u16 value, temp, soc = 0;
	int capacity;

	wake_lock_timeout(&max17047->wake_lock, 3 * HZ);

	pr_info("%s:**************\n", __func__);

	ret = max17047_read_reg(max17047, MAX17047_REG_STATUS, &value);
	if (unlikely(ret < 0)) {
		dev_err(&max17047->client->dev, "read status failed\n");
		return IRQ_HANDLED;
	}

	/* clear the status */
	ret = max17047_write_reg(max17047, MAX17047_REG_STATUS, 0x00);
	if (unlikely(ret < 0)) {
		dev_err(&max17047->client->dev, "clear POR falied\n");
		return IRQ_HANDLED;
	}

	/* check whether the min SOCALRT threshold is exceeded */
	if (((value >> 10) & 0x1) == 1) {
		max17047_read_reg(max17047, MAX17047_REG_SOCREP, &soc);
		capacity = (soc + 0x0080) >> 8;

		pr_info("%s: the min SOCALRT(%d) threshold Exceeded\n",
				__func__, capacity);
		max17047->soc_exceeded = true;
	}

	/* check whether the min VALRT threshold is exceeded */
	if (((value >> 8) & 0x01) == 1) {
		pr_info("%s:the min VALRT(3.35V) threshold Exceeded\n", __func__);
		max17047->vol_exceeded = true;
	}

	get_battery_info(max17047);
	power_supply_changed(&max17047->psy);
	queue_delayed_work_on(0, max17047->battery_wq, &max17047->battery_work, 0);

	if (value & 0x2) {
		/* POR is set */
		dev_info(&max17047->client->dev,"%s:detect POR\n", __func__);
		max17047_hw_init(max17047);
	}

	max17047_read_reg(max17047, MAX17047_REG_CONFIG, &value);
	if (max17047->soc_exceeded || max17047->vol_exceeded) {
		/* clear the interrupt(ALRTP = 1) & set the Aen disable(Aen = 0) */
		temp = value;
		max17047_write_reg(max17047, MAX17047_REG_CONFIG,
				(((value >> 8) | (0x1 << 3)) << 8) |
				((temp & 0x00ff) & (~(0x1 << 2))));
	} else {
		/* only clear the interrupt */
		max17047_write_reg(max17047, MAX17047_REG_CONFIG, value & 0xf7ff);
	}

	return IRQ_HANDLED;
}

static void max17047_battery_work(struct work_struct *work)
{
	struct max17047 *max17047 = container_of(work, struct max17047,
			battery_work.work);
	u16 value = 0;
	int ret;

	get_battery_info(max17047);
	power_supply_changed(&max17047->psy);

	if (max17047->soc_exceeded) {
		/* when soc is 0%, wakeup and redmine the system*/
		ret = max17047_write_reg(max17047, MAX17047_REG_SOCALRT_TH, 0xC800);
		/* set the Aen enable */
		ret = max17047_read_reg(max17047, MAX17047_REG_CONFIG, &value);
		max17047_write_reg(max17047, MAX17047_REG_CONFIG, value | 0x4);
		max17047->soc_exceeded = false;
	}

#ifdef CONFIG_BATTERY_MAX17047_DEBUG
	max17047->delay = msecs_to_jiffies(20000);
	if (!max17047->suspend_flag) {
		max17047_regs_show(max17047);
	}

	if (!delayed_work_pending(&max17047->battery_work))
		queue_delayed_work_on(0, max17047->battery_wq,
				&max17047->battery_work, max17047->delay);
#else
	if (!delayed_work_pending(&max17047->battery_work))
		queue_delayed_work_on(0, max17047->battery_wq,
				&max17047->battery_work, MAX17047_DELAYED_TIME);
#endif

}

static void save_parameters_work(struct alarm *alarm)
{
	struct max17047 *max17047 = container_of(alarm, struct max17047, saved_alarm);	

	if(!max17047->clear_parameters) {
		wake_lock_timeout(&max17047->wake_lock, 3 * HZ);

		schedule_delayed_work_on(0, &max17047->saved_work, 0);

		set_alarm(&max17047->saved_alarm, MAX17047_SAVED_TIME);
	}
}

static void restore_parameters_work(struct alarm *alarm)
{
	struct max17047 *max17047 = container_of(alarm, struct max17047, restored_alarm);
    u16 dq_acc = 0x1296;

    wake_lock_timeout(&max17047->wake_lock, 3 * HZ);

    max17047_verify_reg(max17047, MAX17047_REG_DQACC, dq_acc);
    max17047_verify_reg(max17047, MAX17047_REG_DPACC, 0xC800);

    set_alarm(&max17047->restored_alarm, MAX17047_RESTORE_TIME);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void max17047_late_resume(struct early_suspend *h)
{
	struct max17047 *max17047 = container_of(h, struct max17047, early_suspend);
	int ret;

	if (max17047->fullcap_exceeded) {
		memset(max17047->learned_parameters.parameters, '\0',
				sizeof(max17047->learned_parameters.parameters));
		ret = write_battery_model(sizeof(max17047->learned_parameters.parameters),
				max17047->learned_parameters.parameters);
		if (ret != 0) {
			pr_err("write the block failed\n");
		}
		max17047_import_battery_model(max17047);
		max17047->fullcap_exceeded = false;
	}
}
#endif

static ssize_t board_version_id_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	u32 version_id;

	version_id = m6x_get_board_version();

	return sprintf(buf, "%d\n", version_id);
}
static CLASS_ATTR(version_id, 0644, board_version_id_show, NULL);

static ssize_t clear_learned_parameters_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct max17047 *max17047 = dev_get_drvdata(dev);
	int ret;
    static u16 temp_parameters[LEARNED_PARAM_LEN] = {0};

	max17047->clear_parameters = simple_strtol(buf, NULL, 10);
	if (max17047->clear_parameters) {
		pr_info("%s:need clear the learned parameters\n", __func__);
		memset(max17047->learned_parameters.parameters, '\0',
				sizeof(max17047->learned_parameters.parameters));
		ret = write_battery_model(sizeof(max17047->learned_parameters.parameters),
				max17047->learned_parameters.parameters);
		if (ret != 0) {
			pr_err("write the block failed\n");
		}
        ret = read_battery_model(sizeof(max17047->learned_parameters.parameters), max17047->learned_parameters.parameters);
		if (ret != 0) {
			pr_err("read the block failed\n");
		}
        while (memcmp(temp_parameters, max17047->learned_parameters.parameters, 24) != 0) {
            /* if write error, should write again */
            pr_info("%s:the clear_parameters error\n", __func__);
            memset(max17047->learned_parameters.parameters, '\0', sizeof(max17047->learned_parameters.parameters));
            ret = write_battery_model(sizeof(max17047->learned_parameters.parameters), max17047->learned_parameters.parameters);
            if (ret != 0) {
                pr_err("write the block failed\n");
            }

            ret = read_battery_model(sizeof(max17047->learned_parameters.parameters),
                    max17047->learned_parameters.parameters);
            if (ret != 0) {
                pr_err("read the block failed\n");
            }
        }
        /* if the learned parameters clear successed, load the custome again */
		max17047_import_battery_model(max17047);
    }
	return count;
}

static ssize_t clear_learned_parameters_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct max17047 *max17047 = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", max17047->clear_parameters);
}
static DEVICE_ATTR(clear_parameters, 0644, clear_learned_parameters_show,
		clear_learned_parameters_store);

static int __devinit max17047_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct max17047_platform_data *pdata = client->dev.platform_data;
	struct max17047 *max17047;
	struct platform_device *pdev;
	int ret;
	int bat_id;

	max17047 = kzalloc(sizeof(struct max17047), GFP_KERNEL);
	if (!max17047) {
		return -ENOMEM;
	}

	i2c_set_clientdata(client, max17047);
	pdev = container_of(&client->dev, struct platform_device, dev);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_I2C_BLOCK)) {
		pr_err("%s: i2c adapter dont't support i2c operation\n", __func__);
		ret = -ENODEV;
		goto fail0;
	}

	max17047->client = client;
	max17047->r_sns = pdata->r_sns;
	max17047->current_sensing = pdata->current_sensing;
	max17047->done = true;
	max17047->temp_debug = false;
	max17047->voltage_debug = false;
	max17047->suspend_flag = false;
	max17047->health = POWER_SUPPLY_HEALTH_GOOD;
	max17047->abnormal_temp_cnt = 0;
	max17047->recover_temp_cnt = 0;
	max17047->vol_exceeded = false;
	max17047->soc_exceeded = false;
	max17047->fullcap_exceeded = false;
	max17047->clear_parameters = false;
    max17047->first_flag = 1;

	max17047->psy.name		= MAX17047_NAME;
	max17047->psy.type		= POWER_SUPPLY_TYPE_BATTERY;
	max17047->psy.get_property	= max17047_get_property;
	max17047->psy.set_property	= max17047_set_property;
	max17047->psy.properties	= max17047_props;
	max17047->psy.num_properties	= ARRAY_SIZE(max17047_props);

	ret = power_supply_register(&client->dev, &max17047->psy);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		goto fail1;
	}

	max17047->adc = s3c_adc_register(pdev, NULL, NULL, 0);
	if (max17047->adc < 0) {
		pr_err("%s: s3c_adc_register failed\n", __func__);
		goto fail1;
	}

	bat_id = batid_adc_read(max17047);
	max17047->version_id = m6x_get_board_version();
        pr_info("pcb version_id %d\n", max17047->version_id);

	wake_lock_init(&max17047->wake_lock, WAKE_LOCK_SUSPEND, "battery");
	alarm_init(&max17047->saved_alarm, ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
			save_parameters_work);
    if (machine_is_m65() && (max17047->version_id <= 1))
        alarm_init(&max17047->restored_alarm, ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
                restore_parameters_work);

#ifdef CONFIG_HAS_EARLYSUSPEND
	max17047->early_suspend.resume = max17047_late_resume;
	register_early_suspend(&max17047->early_suspend);
#endif

	max17047->battery_wq = create_singlethread_workqueue("battery_work");
	INIT_DELAYED_WORK_DEFERRABLE(&max17047->battery_work, max17047_battery_work);
	INIT_DELAYED_WORK_DEFERRABLE(&max17047->saved_work, max17047_saved_learned_parameters);

	ret = max17047_hw_init(max17047);
	if (unlikely(ret < 0)) {
		goto fail2;
	}

	/* init the fuelgauge threshold */
	max17047_init_threshold(max17047);

	ret = gpio_request(client->irq, MAX17047_NAME);
	max17047->irq = gpio_to_irq(client->irq);
	if (max17047->irq) {
		ret = request_threaded_irq(max17047->irq, NULL,
				max17047_isr, IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				MAX17047_NAME, max17047);
		if (unlikely(ret < 0)) {
			dev_err(&client->dev, "failed to reqeust IRQ\n");
			goto fail3;
		}
	}

	ret = class_create_file(charging, &class_attr_version_id);
	max17047->dev = device_create(charging, NULL, 0, max17047, "max17047");
	ret = device_create_file(max17047->dev, &dev_attr_clear_parameters);

	queue_delayed_work_on(0, max17047->battery_wq, &max17047->battery_work, 0);

	return 0;

fail3:
	gpio_free(client->irq);
fail2:
	s3c_adc_release(max17047->adc);
fail1:
	power_supply_unregister(&max17047->psy);
fail0:
	kfree(max17047);
	return ret;
}

static int __devexit max17047_remove(struct i2c_client *client)
{
	struct max17047 *max17047 = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&max17047->early_suspend);
#endif

	power_supply_unregister(&max17047->psy);
	s3c_adc_release(max17047->adc);
	free_irq(max17047->irq, max17047);
	kfree(max17047);
	return 0;
}

static void max17047_shutdown(struct i2c_client *client)
{
	struct max17047 *max17047 = i2c_get_clientdata(client);

	pr_info("%s: capacity %d, voltage_now %d, temp %d\n",
			__func__, max17047->capacity, max17047->voltage_now, max17047->temp);
#if 0
	if (!max17047->clear_parameters)
		queue_delayed_work_on(0, max17047->battery_wq,
				&max17047->saved_work, 0);
#endif
}

static const struct i2c_device_id max17047_id[] = {
	{MAX17047_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, max17047_id);

#ifdef CONFIG_PM
static int max17047_suspend(struct device *dev)
{
	struct max17047 *max17047 = dev_get_drvdata(dev);

	max17047->suspend_flag = true;
//	cancel_delayed_work_sync(&max17047->battery_work);
	enable_irq_wake(max17047->irq);

	return 0;
}

static int max17047_resume(struct device *dev)
{
	struct max17047 *max17047 = dev_get_drvdata(dev);

	max17047->suspend_flag = false;
    if (max17047->first_flag != 3) {
        /* if the first_flag is 3, that means the capacity i alreay unnormal */
        max17047->first_flag = 1;
    }
	get_capacity_info(max17047);
	power_supply_changed(&max17047->psy);
	disable_irq_wake(max17047->irq);
	queue_delayed_work_on(0, max17047->battery_wq, &max17047->battery_work, 0);

	return 0;
}

struct dev_pm_ops max17047_pm_ops = {
	.suspend = max17047_suspend,
	.resume = max17047_resume,
};
#else
struct dev_pm_ops max17047_pm_ops = NULL;
#endif

static struct i2c_driver max17047_i2c_driver = {
	.driver	= {
		.name	= MAX17047_NAME,
		.pm 	= &max17047_pm_ops,
	},
	.probe		= max17047_probe,
	.shutdown	= max17047_shutdown,
	.remove		= __devexit_p(max17047_remove),
	.id_table	= max17047_id,
};

static int __init max17047_init(void)
{
	return i2c_add_driver(&max17047_i2c_driver);
}
//module_init(max17047_init);
late_initcall(max17047_init);

static void __exit max17047_exit(void)
{
	i2c_del_driver(&max17047_i2c_driver);
}
module_exit(max17047_exit);

MODULE_AUTHOR("Gyungoh Yoo<jack.yoo@maxim-ic.com>");
MODULE_DESCRIPTION("MAX17047 Fuel Gauge");
MODULE_LICENSE("GPL");
