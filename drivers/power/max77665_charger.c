/*

 * Battery driver for Maxim MAX77665
 *
 * Copyright (C) 2012 Meizu Technology Co.Ltd, Zhuhai, China
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/mfd/max77665.h>
#include <linux/mfd/max77665-private.h>
#include <linux/regulator/machine.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/power/max17047_battery.h>
#include <linux/android_alarm.h>

#define MAX_AC_CURRENT         1500
#define MIN_USB_CURRENT        200
#define TEMP_CHECK_DELAY       (60*HZ)
#define WAKE_ALARM_INT         (60)
#define CURRENT_INCREMENT_STEP 100     /*mA*/
#define MA_TO_UA               1000
#define BATTERY_TEMP_0         0       /*0oC*/
#define BATTERY_TEMP_10	       100	/* 10oC*/
#define BATTERY_TEMP_45        450     /*45oC*/
#define BATTERY_CURRENT_03C    700
#define BATTERY_CURRENT_05C    1166

#define MAX77665_CHGIN_DTLS       0x60 
#define MAX77665_CHGIN_DTLS_SHIFT 5    
#define MAX77665_CHG_DTLS         0x0F 
#define MAX77665_CHG_DTLS_SHIFT   0    
#define MAX77665_BAT_DTLS         0x70 
#define MAX77665_BAT_DTLS_SHIFT   4    
#define MAX77665_BYP_DTLS         0x0F 
#define MAX77665_BYP_DTLS_SHIFT   0    
#define MAX77665_BYP_OK           0x01 
#define MAX77665_BYP_OK_SHIFT     0    
#define MAX77665_CHGIN_OK         0x40 
#define MAX77665_CHGIN_OK_SHIFT   6    

#define ADB_CLOSE	0
#define ADB_OPEN	1
#define RNDIS_OPEN	2
#define RNDIS_CLOSE	3
#define STORAGE_OPEN	4
#define STORAGE_CLOSE	5

struct max77665_charger
{
	struct device *dev;
	struct power_supply psy_usb;
	struct power_supply psy_ac;
	struct power_supply psy_charger;
	struct max77665_dev *iodev;
	struct wake_lock wake_lock;
	struct delayed_work dwork;
	struct delayed_work poll_dwork;
	struct workqueue_struct *irq_wq;
	struct delayed_work irq_dwork;
	struct work_struct chgin_work;
	struct regulator *ps;
	struct regulator *reverse;
	struct regulator *battery;
	struct mutex mutex_t;
		
	enum cable_status_t {
		CABLE_TYPE_NONE = 0,
		CABLE_TYPE_USB,
		CABLE_TYPE_AC,
		CABLE_TYPE_UNKNOW,
	} cable_status;

	enum chg_status_t {
		CHG_STATUS_FAST,
		CHG_STATUS_DONE,
		CHG_STATUS_RECHG,
	} chg_status;

	bool chgin;
	int chgin_irq;
	int chgin_ilim_usb;	/* 60mA ~ 500mA */
	int chgin_ilim_ac;	/* 60mA ~ 2.58A */
	int fast_charge_current;
	int irq_reg;
	int vbusdetect;
	int vbus_irq;

	struct alarm alarm;
	struct alarm delayed_alarm;

	int battery_health;
	struct notifier_block usb_notifer;
	bool adb_open;
	bool storage_open;
	bool rndis_open;
	int fastcharging;
	int fastcharging_ilim_usb;
};

enum {
 	BATTERY_HEALTH_UNKNOW = 0,
 	BATTERY_HEALTH_GOOD,
	BATTERY_HEALTH_OVERHEAT,
	BATTERY_HEALTH_COLD,
};
static int delayedtime = 30;
extern struct class *charging;

#ifdef CONFIG_USB_G_ANDROID
extern int register_usb_gadget_notifier(struct notifier_block *nb);
#else
static int register_usb_gadget_notifier(struct notifier_block *nb) {return 0;}
#endif

static char *supply_list[] = {
	"battery",
};

static enum power_supply_property max77665_power_props[] =
{
	POWER_SUPPLY_PROP_ONLINE,
};

static int max77665_usb_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct max77665_charger *charger =
		container_of(psy, struct max77665_charger, psy_usb);

	if (psp != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;

	/* Set enable=1 only if the AC charger is connected */
	val->intval = (charger->cable_status == CABLE_TYPE_USB);
	
	return 0;
}

static int max77665_ac_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct max77665_charger *charger =
		container_of(psy, struct max77665_charger, psy_ac);
	
	if (psp != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;

	/* Set enable=1 only if the AC charger is connected */
	val->intval = (charger->cable_status == CABLE_TYPE_AC);

	return 0;
}

static int max77665_usb_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct max77665_charger *charger = container_of(psy,
			struct max77665_charger, psy_usb);

	if (psp != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;

	if (val->intval)
		charger->cable_status = CABLE_TYPE_USB; 

	return 0;
}

static int max77665_ac_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct max77665_charger *charger = container_of(psy,
			struct max77665_charger, psy_ac);

	if (psp != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;

	if (val->intval)
		charger->cable_status = CABLE_TYPE_AC; 

	return 0;

}

static enum power_supply_property max77665_charger_props[] = {
	POWER_SUPPLY_PROP_STATUS,
};

static int max77665_charger_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct max77665_charger *charger =
		container_of(psy, struct max77665_charger, psy_charger);
	
	if (psp != POWER_SUPPLY_PROP_STATUS)
		return -EINVAL;

	if (charger->cable_status == CABLE_TYPE_USB || 
		charger->cable_status == CABLE_TYPE_AC) {
		if (charger->chg_status == CHG_STATUS_FAST
				|| charger->chg_status == CHG_STATUS_RECHG)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_FULL;
	} else {
		val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
	}
	
	return 0;
}

static void set_alarm(struct alarm *alarm, int seconds)
{

	ktime_t interval = ktime_set(seconds, 0);
	ktime_t now = alarm_get_elapsed_realtime();
	ktime_t next = ktime_add(now, interval);

	pr_info("set alarm after %d seconds\n", seconds);
	alarm_start_range(alarm, next, next);
}

static void charger_bat_alarm(struct alarm *alarm)
{
	struct max77665_charger *chg = container_of(alarm, 
			struct max77665_charger, alarm);

	wake_lock_timeout(&chg->wake_lock, 3 * HZ);
	set_alarm(alarm, WAKE_ALARM_INT);
}

static void delayed_adjust_work(struct alarm *alarm)
{
	struct max77665_charger *charger = container_of(alarm,
			struct max77665_charger, delayed_alarm);
	
	wake_lock_timeout(&charger->wake_lock, 3*HZ);
	queue_delayed_work_on(0, charger->irq_wq, &charger->dwork, 0);
}

static int get_battery_status(struct max77665_charger *max77665)
{
	struct power_supply *fuelgauge_ps = 
		power_supply_get_by_name("fuelgauge");
	union power_supply_propval val;
	int temp = 0;
	int health = BATTERY_HEALTH_GOOD;
	int battery_online = true;
	int battery_current = min(max77665->fast_charge_current, MAX_AC_CURRENT);
	int now_current;
	int ret;

	if (fuelgauge_ps) {
		if (fuelgauge_ps->get_property(fuelgauge_ps, POWER_SUPPLY_PROP_TEMP, &val) == 0)
			temp = val.intval;
		if (fuelgauge_ps->get_property(fuelgauge_ps, POWER_SUPPLY_PROP_ONLINE, &val) == 0)
			battery_online = val.intval;

		if (battery_online) {
			if (temp < BATTERY_TEMP_0) {
				battery_current = 0;
				health = BATTERY_HEALTH_COLD;
			} else if (temp > BATTERY_TEMP_45) {
				battery_current = 0;
				health = BATTERY_HEALTH_OVERHEAT;
			} else if (temp < BATTERY_TEMP_10) {
				battery_current = min(BATTERY_CURRENT_03C, battery_current);
			} else {
				battery_current = min(BATTERY_CURRENT_05C, battery_current);
			}
		}
	}

	do {
		now_current = regulator_get_current_limit(max77665->battery) / MA_TO_UA;
		if (!(now_current <= battery_current && battery_current <= now_current + CHG_CC_STEP)) {
			pr_info("now_current is %d, current is %d\n", now_current, battery_current);	
			ret = regulator_set_current_limit(max77665->battery, battery_current * MA_TO_UA,
					(battery_current + CHG_CC_STEP) * MA_TO_UA);
			if (ret) {
				pr_err("failed to set the battery current\n");
			}
		}
	} while (0);

	max77665->battery_health = health;

	return max77665->battery_health;
}

#define DECREASE_CURRENT 1
#define REMOVED_CHARGING 2

static int max77665_remove_judge(struct max77665_charger *charger)
{
	struct regmap *regmap = charger->iodev->regmap;
	unsigned int int_ok;
	int ret = 0;
	int vbusdetect;

	ret = max77665_read_reg(regmap, MAX77665_CHG_REG_CHG_INT_OK, &int_ok);
	vbusdetect = gpio_get_value(charger->vbusdetect);

	pr_debug("%s: vbusdetect = %d, int_ok = 0x%02x\n",
			__func__, vbusdetect, int_ok);

	if ((int_ok != 0x5d) && vbusdetect) 
		return DECREASE_CURRENT;
	else if (int_ok != 0x5d) 
		return REMOVED_CHARGING;

	return ret;
}
#if 0
static int max77665_adjust_current(struct max77665_charger *charger,
		int chgin_ilim)
{
	struct regmap *regmap = charger->iodev->regmap;
	unsigned int int_ok = 0;
	int ret = 0, ad_current;
	int max_input_current = charger->fastcharging_ilim_usb;

	for (ad_current = chgin_ilim; ad_current <= max_input_current;
			ad_current += CURRENT_INCREMENT_STEP) {
		
		ret = regulator_set_current_limit(charger->ps,
				ad_current*MA_TO_UA,
				ad_current*MA_TO_UA
				 +CURRENT_INCREMENT_STEP*MA_TO_UA);
		if (ret) {
			pr_err("failed to set increase current limit\n");
			return ret;
		}
		pr_info("%d,waiting..............\n", ad_current);
		msleep(100);

		ret = max77665_remove_judge(charger);
		if (ret == DECREASE_CURRENT) {
			do {
				pr_info("please decrese current %d\n", ad_current);
				ad_current -= CURRENT_INCREMENT_STEP;
				if (ad_current < MIN_USB_CURRENT) {
					break;
				}
				ret = regulator_set_current_limit(charger->ps,
						ad_current*MA_TO_UA,
						CURRENT_INCREMENT_STEP*MA_TO_UA
						+ ad_current*MA_TO_UA);
				if (ret) {
					pr_err("failed to set decrease current limit\n");
					return ret;
				}
				msleep(50);
				max77665_read_reg(regmap, MAX77665_CHG_REG_CHG_INT_OK, 
						&int_ok);
			} while (int_ok != 0x5d);
			break;
		} else if (ret == REMOVED_CHARGING) {
			pr_info("remove charging\n");
			charger->cable_status = CABLE_TYPE_NONE;
			power_supply_changed(&charger->psy_usb);
			power_supply_changed(&charger->psy_ac);
			break;
		}
	}
	return ret;
}
#endif
static void max77665_start_adjust_current(struct work_struct *work)
{
	struct max77665_charger *charger = container_of(work, 
			struct max77665_charger, dwork.work);
	enum cable_status_t cable_status = charger->cable_status;
	int chgin_ilim = 0;
	int ret;
/*	int adjust_done = false;*/
	
	wake_lock(&charger->wake_lock);
	mutex_lock(&charger->mutex_t);

	/*disable the chgin irq during the adjust current*/
	disable_irq(charger->chgin_irq);
	disable_irq(charger->vbus_irq);

	chgin_ilim = MIN_USB_CURRENT;
	switch (cable_status) {
		case CABLE_TYPE_USB:
#if 0
			if (!charger->fastcharging || charger->adb_open || charger->rndis_open) {
				chgin_ilim = max(MIN_USB_CURRENT, charger->chgin_ilim_usb);
				adjust_done = true;
			}

			if (adjust_done) {
				ret = regulator_set_current_limit(charger->ps,
						chgin_ilim * MA_TO_UA, MAX_AC_CURRENT * MA_TO_UA);
				if (ret) {
					pr_info("%s: set chgin current failed\n",
							__func__);
					enable_irq(charger->chgin_irq);
					enable_irq(charger->vbus_irq);
					wake_unlock(&charger->wake_lock);
					mutex_unlock(&charger->mutex_t);
					return;
				}
			}
			/* only fastcharging, and not open usb functions */
			if (!adjust_done && charger->fastcharging) {
				pr_info("%s: usb fastcharging start.\n", __func__);
				max77665_adjust_current(charger, chgin_ilim);
			}
#endif
			break;
		case CABLE_TYPE_AC: 
			ret = regulator_set_current_limit(charger->ps, 
					charger->chgin_ilim_ac * MA_TO_UA, 
					(charger->chgin_ilim_ac + CHG_CC_STEP) * MA_TO_UA);
			if (ret) {
				pr_info("%s:set ac chgin current failed\n", __func__);
				enable_irq(charger->chgin_irq);
				enable_irq(charger->vbus_irq);
				wake_unlock(&charger->wake_lock);
				mutex_unlock(&charger->mutex_t);
				return;
			}
			msleep(100);
			ret = max77665_remove_judge(charger);
			if (ret == DECREASE_CURRENT)
				pr_info("the vbusgpio is high level, so ignore the remove interrupt\n");
			pr_info("the AC charging current is %d\n", (regulator_get_current_limit(charger->ps) / MA_TO_UA));
			break;
		default:
			chgin_ilim = 0;
			break;
	}
	enable_irq(charger->chgin_irq);
	enable_irq(charger->vbus_irq);
	mutex_unlock(&charger->mutex_t);
	wake_unlock(&charger->wake_lock);
}

static void max77665_judge_insert_type(struct max77665_charger *charger)
{
	struct regmap *regmap_muic = charger->iodev->regmap_muic;
	enum cable_status_t cable_status = CABLE_TYPE_NONE;
	unsigned int reg_data = 0, cdetctrl1 = 0, int2 = 0;
	
    mutex_lock(&charger->mutex_t);
	if (charger->chgin) {
		pr_info("#######usb start detect\n");
		do {
			max77665_read_reg(regmap_muic, MAX77665_MUIC_REG_STATUS2, 
					&reg_data);
		} while (reg_data & 0x08);
		pr_info("########usb end detect (0x%02x)\n", reg_data);

		if ((reg_data & 0x07) == 0x01) {
			cable_status = CABLE_TYPE_USB;
		} else if ((reg_data & 0x07) == 0x00) {
            pr_info("%s:,nothing attached, because of DxOVP interrupt Occured,"
                    "before next detect we should clear it\n", __func__);
            /* clear the DxOVP by reading it*/
            max77665_read_reg(regmap_muic, MAX77665_MUIC_REG_INT2, &int2);
			max77665_read_reg(regmap_muic, MAX77665_MUIC_REG_CDETCTRL1, &cdetctrl1);
			max77665_write_reg(regmap_muic, MAX77665_MUIC_REG_CDETCTRL1, cdetctrl1 | 0x02);
			pr_info("usb start detect again\n");
            do {
                msleep(100);
                max77665_read_reg(regmap_muic, MAX77665_MUIC_REG_STATUS2, &reg_data);
            } while (reg_data & 0x08);
            pr_info("usb end detect (0x%02x)\n", reg_data);
            if (((reg_data & 0x07) == 0x01) || ((reg_data & 0x07) == 0x00)) {
                cable_status = CABLE_TYPE_USB;
            } else {
                cable_status = CABLE_TYPE_AC;
            }
        } else {
			cable_status = CABLE_TYPE_AC;
        }
	} else {
		regulator_set_current_limit(charger->ps,
				charger->chgin_ilim_usb * MA_TO_UA,
				MAX_AC_CURRENT * MA_TO_UA);
		cable_status = CABLE_TYPE_NONE;
	}
	charger->cable_status = cable_status;
    mutex_unlock(&charger->mutex_t);
}
	
static void max77665_poll_work_func(struct work_struct *work)
{
	struct max77665_charger *charger =
		container_of(work, struct max77665_charger, poll_dwork.work);
	struct power_supply *fuelgauge_ps
		= power_supply_get_by_name("fuelgauge");
	union power_supply_propval val;
	int battery_health = BATTERY_HEALTH_GOOD;
	struct regmap *regmap = charger->iodev->regmap;
	unsigned int reg_data;
	int soc = 100;
	bool disable_flag = false;

	mutex_lock(&charger->mutex_t);

	battery_health = get_battery_status(charger);

	if (charger->chg_status == CHG_STATUS_FAST ||
			charger->chg_status == CHG_STATUS_RECHG) {

		if(max77665_read_reg(regmap, MAX77665_CHG_REG_CHG_DETAILS_01, 
					&reg_data) >= 0) {
			if((reg_data & 0x0F) == 0x04) {
				charger->chg_status = CHG_STATUS_DONE;
			}
		}
	}
	if ( charger->cable_status == CABLE_TYPE_USB ||
			charger->cable_status == CABLE_TYPE_AC) {

		if (battery_health == BATTERY_HEALTH_COLD 
				|| battery_health == BATTERY_HEALTH_OVERHEAT 
				|| battery_health == BATTERY_HEALTH_UNKNOW) {
			if (regulator_is_enabled(charger->ps)) {
				pr_info("----------battery unhealthy, disable charging\n");
				regulator_disable(charger->ps);
			}
		} else {
			/* monitor whether it is enter "TIMER FAULT" state */
			if (max77665_read_reg(regmap, MAX77665_CHG_REG_CHG_DETAILS_01, &reg_data) >= 0) {
				if (((reg_data & 0x0F) == 0x06) || ((reg_data & 0x70) == 0x02)) {
					pr_info("%s:the charger state is enter TIMER FAULT state\n", __func__);
					/* we need to exit this state, first, we should set the charger off*/
					regulator_disable(charger->ps);
					msleep(500);
					/* then we need charger is on again */
					regulator_enable(charger->ps);
				}
			}

			if (charger->battery_health != battery_health) {
				if (!regulator_is_enabled(charger->ps))
					regulator_enable(charger->ps);
				queue_delayed_work_on(0, charger->irq_wq, 
						&charger->dwork, 0);
			}
			if (regulator_is_enabled(charger->ps)) {
				if (charger->chg_status == CHG_STATUS_DONE && fuelgauge_ps) {
					if(fuelgauge_ps->get_property(fuelgauge_ps, POWER_SUPPLY_PROP_CAPACITY, &val) == 0)
						soc = val.intval;
					if(soc <= 99) {
						regulator_disable(charger->ps);
						msleep(500);
						regulator_enable(charger->ps);
						charger->chg_status = CHG_STATUS_RECHG;
					}
				}
			} else {
				pr_info("----------battery healthy good, enable charging\n");
				regulator_enable(charger->ps);
			}
		}

		if (fuelgauge_ps) {
			if (fuelgauge_ps->get_property(fuelgauge_ps, POWER_SUPPLY_PROP_CAPACITY, &val) == 0) {
				soc = val.intval;
				if (soc == 100) {
					/* if the soc is 100%, disbale the max77665_charger irq */
					disable_irq(charger->chgin_irq);
					disable_flag = true;
				}
			}
		}

		schedule_delayed_work_on(0, &charger->poll_dwork, TEMP_CHECK_DELAY);
	} else {
		if (regulator_is_enabled(charger->ps)) {
			pr_info("--------------charger remove, disable charging\n");
			regulator_disable(charger->ps);
		}
		if (disable_flag)
			enable_irq(charger->chgin_irq);
	}
	pr_debug("battery_current = %d#######\n", regulator_get_current_limit(charger->battery));
	pr_debug("charging current = %d\n",regulator_get_current_limit(charger->ps));
	charger->battery_health = battery_health;
	mutex_unlock(&charger->mutex_t);
}

static int max77665_reg_dump(struct max77665_charger *charger)
{
	struct regmap *regmap = charger->iodev->regmap;
	unsigned int int_ok;
	u8 dtls_regs[3];
	unsigned int chgin_dtls, chg_dtls, bat_dtls, byp_dtls;
	int ret;
	ret = max77665_read_reg(regmap, MAX77665_CHG_REG_CHG_INT_OK,
			&int_ok);
	/* charger */
	ret = max77665_bulk_read(regmap, MAX77665_CHG_REG_CHG_DETAILS_00,
			dtls_regs, 3);

	chgin_dtls = ((dtls_regs[0] & MAX77665_CHGIN_DTLS) >>
			MAX77665_CHGIN_DTLS_SHIFT);
	chg_dtls = ((dtls_regs[1] & MAX77665_CHG_DTLS) >>
			MAX77665_CHG_DTLS_SHIFT);
	bat_dtls = ((dtls_regs[1] & MAX77665_BAT_DTLS) >>
			MAX77665_BAT_DTLS_SHIFT);
	byp_dtls = ((dtls_regs[2] & MAX77665_BYP_DTLS) >>
			MAX77665_BYP_DTLS_SHIFT);

	pr_info("INT_OK(0x%x), CHGIN(0x%x), CHG(0x%x), BAT(0x%x),BYP_DTLS(0x%02x)\n",
			int_ok, chgin_dtls, chg_dtls, bat_dtls, byp_dtls);

		return ret;
}

static void max77665_chgin_irq_handler(struct work_struct *work)
{
	struct max77665_charger *charger = container_of(work,
			struct max77665_charger, irq_dwork.work);
	int ret;
    int chgin_current = charger->chgin_ilim_usb;

	wake_lock(&charger->wake_lock);	

	/*First, adjust the insert type AC or USB*/
	max77665_judge_insert_type(charger);
	
	/*Obtain the appropriate charge current for different temperatures*/
	if (delayed_work_pending(&charger->poll_dwork))
		cancel_delayed_work(&charger->poll_dwork);
	schedule_delayed_work_on(0, &charger->poll_dwork, 0);
	
	/*First set a minimum charger current for usb*/
    if (charger->cable_status == CABLE_TYPE_AC) {
        chgin_current = charger->chgin_ilim_ac;
    }
	ret = regulator_set_current_limit(charger->ps, 
			chgin_current * MA_TO_UA,
			MAX_AC_CURRENT * MA_TO_UA);
	if (ret) {
		pr_info("%s:failed to set the chgin current\n", __func__);
		wake_unlock(&charger->wake_lock);
		return;
	} else {
		power_supply_changed(&charger->psy_usb);
		power_supply_changed(&charger->psy_ac);
	}

	set_alarm(&charger->delayed_alarm, delayedtime);

	wake_unlock(&charger->wake_lock);
}

static irqreturn_t max77665_charger_isr(int irq, void *dev_id)
{
	struct max77665_charger *charger = dev_id;
	struct regmap *regmap = charger->iodev->regmap;
	unsigned int prev_int_ok, int_ok;
	int chgin = 0;
	int ret;
	int current_now;

	pr_info("ENTER %s:****irq:%d******\n",__func__, irq);

	if (regulator_is_enabled(charger->reverse)) {
		pr_info("usb host insert, dismiss this isr\n");
		return IRQ_HANDLED;
	}

	wake_lock(&charger->wake_lock);
	
	ret = max77665_read_reg(regmap, MAX77665_CHG_REG_CHG_INT_OK, &prev_int_ok);
	if (unlikely(ret < 0)) {
		pr_err("Failed to read MAX77665_CHG_REG_CHG_INT: %d\n", ret);
	}
	msleep(20);
	ret = max77665_read_reg(regmap, MAX77665_CHG_REG_CHG_INT_OK, &int_ok);
	if (unlikely(ret < 0)) {
		pr_err("Failed to read MAX77665_CHG_REG_CHG_INT: %d\n", ret);
	}
	if ((charger->irq_reg == int_ok) && (prev_int_ok != int_ok)) {
		pr_info("%s:irq debounced(0x%x, 0x%x, 0x%x),return\n",
				__func__, charger->irq_reg,
				prev_int_ok, int_ok);
		wake_unlock(&charger->wake_lock);
		return IRQ_HANDLED;
	} else {
		chgin = !!(int_ok & 0x40);
	}
	charger->irq_reg = int_ok;

	if (charger->chgin == chgin) {
		wake_unlock(&charger->wake_lock);
		return IRQ_HANDLED;
	}

	max77665_reg_dump(charger);

	ret = max77665_remove_judge(charger);
	if (ret == DECREASE_CURRENT) {
		do {
			current_now = regulator_get_current_limit(charger->ps);
			if (current_now < MIN_USB_CURRENT * MA_TO_UA)
				break;
			regulator_set_current_limit(charger->ps,
					current_now - CURRENT_INCREMENT_STEP * MA_TO_UA,
					current_now);
			msleep(20);
			ret = max77665_read_reg(regmap, MAX77665_CHG_REG_CHG_INT_OK, &int_ok);
			pr_info("current_now %d, int_ok 0x%02x\n", current_now, int_ok);
			if (int_ok == 0x5d) {
                chgin = charger->chgin;
				wake_unlock(&charger->wake_lock);
				return IRQ_HANDLED;
			}
		} while (current_now > MIN_USB_CURRENT * MA_TO_UA);
	}

	if (charger->chgin != chgin) {
		charger->chgin = chgin;
		pr_info("-----%s %s\n", __func__, chgin ? "insert" : "remove");

        alarm_cancel(&charger->alarm);
        if (chgin)
            set_alarm(&charger->alarm, WAKE_ALARM_INT);

		charger->chg_status = CHG_STATUS_FAST;
		queue_delayed_work_on(0, charger->irq_wq, &charger->irq_dwork, 0);
	}

	wake_unlock(&charger->wake_lock);

	return IRQ_HANDLED;
}

static ssize_t usb_fastcharging_store(struct device *dev, 
		struct device_attribute *attr,
		const char *buf, size_t count)
{	
	struct max77665_charger *charger = dev_get_drvdata(dev);
#if 0
	charger->fastcharging = simple_strtol(buf, NULL, 10);
	if (charger->cable_status == CABLE_TYPE_USB)
		queue_delayed_work_on(0, charger->irq_wq, &charger->dwork, 0);
#endif
	pr_info("%s:fastcharging = %d\n", __func__, charger->fastcharging);
	
	return count;
}

static ssize_t usb_fastcharging_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{	
	struct max77665_charger *charger = dev_get_drvdata(dev);
	int fastcharging = 0;

	fastcharging = charger->fastcharging;

	return sprintf(buf, "%d\n", fastcharging);
}
static DEVICE_ATTR(fastcharging, 0644, usb_fastcharging_show, usb_fastcharging_store);

static ssize_t charger_delayedtime_store(struct class *class, 
		struct class_attribute *attr, const char *buf, size_t count)
{
	delayedtime = simple_strtol(buf, NULL, 10);
	
	pr_info("%s: set delayedtime %d\n", __func__, delayedtime);

	return count; 
}

static ssize_t charger_delayedtime_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", delayedtime);
	
}
static CLASS_ATTR(delayedtime, 0644, charger_delayedtime_show, charger_delayedtime_store);


static __devinit int max77665_init(struct max77665_charger *charger)
{
	struct max77665_dev *iodev = dev_get_drvdata(charger->dev->parent);
	struct max77665_platform_data *pdata = dev_get_platdata(iodev->dev);	
	struct regmap *regmap = iodev->regmap;
	int ret = EINVAL;
	unsigned int reg_data = 0;

	/* Unlock protected registers */
	ret = max77665_write_reg(regmap, MAX77665_CHG_REG_CHG_CNFG_06, 0x0C);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX8957_REG_CHG_CNFG_06: %d\n", ret);
		goto error;
	}

	reg_data = max((u8)MAX77665_FCHGTIME_4H, (u8)pdata->fast_charge_timer);
	ret = max77665_update_reg(regmap, MAX77665_CHG_REG_CHG_CNFG_01, 
			0x7<<0, reg_data);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX77665_CHG_REG_CHG_CNFG_01: %d\n", ret);
		goto error;
	}

	reg_data = min((u8)MAX77665_CHG_RSTRT_100MV, (u8)pdata->charging_restart_thresold);
	reg_data <<= 4;
	ret = max77665_update_reg(regmap, MAX77665_CHG_REG_CHG_CNFG_01, 0x3<<4, reg_data);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX77665_CHG_REG_CHG_CNFG_01: %d\n", ret);
		goto error;
	}

	reg_data = 1<<7;
	ret = max77665_update_reg(regmap, MAX77665_CHG_REG_CHG_CNFG_01, 0x1<<7, reg_data);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX77665_CHG_REG_CHG_CNFG_01: %d\n", ret);
		goto error;
	}

	reg_data = (min(MAX_AC_CURRENT, pdata->fast_charge_current) /CHG_CC_STEP);
	ret = max77665_update_reg(regmap, MAX77665_CHG_REG_CHG_CNFG_02, 0x3f<<0, reg_data);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX77665_CHG_REG_CHG_CNFG_02: %d\n", ret);
		goto error;
	}

	reg_data = 1<<7;
	ret = max77665_update_reg(regmap, MAX77665_CHG_REG_CHG_CNFG_02, 0x1<<7, reg_data);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX77665_CHG_REG_CHG_CNFG_02: %d\n", ret);
		goto error;
	}

	reg_data = max((u8)MAX77665_CHG_TO_ITH_100MA, (u8)pdata->top_off_current_thresold);
	ret = max77665_update_reg(regmap, MAX77665_CHG_REG_CHG_CNFG_03, 0x7<<0, reg_data);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX77665_CHG_REG_CHG_CNFG_03: %d\n", ret);
		goto error;
	}

	reg_data = max((u8)MAX77665_CHG_TO_TIME_10MIN, (u8)pdata->top_off_timer);
	reg_data <<= 3;
	ret = max77665_update_reg(regmap, MAX77665_CHG_REG_CHG_CNFG_03, 0x7<<3, reg_data);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX77665_CHG_REG_CHG_CNFG_03: %d\n", ret);
		goto error;
	}

	reg_data = max((u8)MAX77665_CHG_CV_PRM_4200MV, (u8)pdata->charger_termination_voltage);
	ret = max77665_update_reg(regmap, MAX77665_CHG_REG_CHG_CNFG_04, 0x1f<<0, reg_data);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX77665_CHG_REG_CHG_CNFG_04: %d\n", ret);
		goto error;
	}

	reg_data = 0 << 4;
	ret = max77665_update_reg(regmap, MAX77665_CHG_REG_CHG_CNFG_00, 0x1 << 4, reg_data);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX8957_REG_CHG_CNFG_00: %d\n", ret);
		goto error;
	}

	/* Lock protected registers */
	ret = max77665_write_reg(regmap, MAX77665_CHG_REG_CHG_CNFG_06, 0x00);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX8957_REG_CHG_CNFG_06: %d\n", ret);
		goto error;
	}

	/* disable muic ctrl */
	reg_data = 1<<5;
	ret = max77665_update_reg(regmap, MAX77665_CHG_REG_CHG_CNFG_00, 0x1<<5, reg_data);
	if (unlikely(ret)) {
		dev_err(charger->dev, "Failed to set MAX77665_CHG_REG_CHG_CNFG_00: %d\n", ret);
		goto error;
	}
	return 0;

error:
	return ret;
}

static int max77665_charger_event(struct notifier_block *this, unsigned long event,
		void *ptr)
{
	struct max77665_charger *charger = container_of(this,
			struct max77665_charger, usb_notifer);

	switch (event) {
#if 0
		case STORAGE_OPEN:
			pr_info("usb mass storage open######\n");
			charger->storage_open = true;
			break;
		case STORAGE_CLOSE:
			pr_info("usb mass stroage close######\n");
			if (!charger->storage_open)
				return event;
			charger->storage_open = false;
			break;
#endif
		case ADB_OPEN:
			pr_info("adb open##########\n");
			if (charger->cable_status != CABLE_TYPE_USB)
				return event;
			charger->adb_open = true;
			break;
		case RNDIS_OPEN:
			pr_info("rndis open########\n");
			charger->rndis_open = true;
			break;
		case ADB_CLOSE:
			pr_info("adb close##########\n");
			if (!charger->adb_open) 
				return event;
			charger->adb_open = false;
			break;
		case RNDIS_CLOSE:
			pr_info("rndis close########\n");
			if (!charger->rndis_open)
				return event;
			charger->rndis_open = false;
			break;
		default:
			break;
	}
	pr_info("adb_open:%d, rndis_open:%d\n",charger->adb_open, charger->rndis_open);
	
	if (charger->chgin) 
		queue_delayed_work_on(0, charger->irq_wq, &charger->dwork, HZ/10);

	return event;
}

static __devinit int max77665_charger_probe(struct platform_device *pdev)
{
	struct max77665_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct max77665_platform_data *pdata = dev_get_platdata(iodev->dev);	
	struct max77665_charger *charger;
	unsigned int reg_data;
	int ret = EINVAL;
	charger = kzalloc(sizeof(struct max77665_charger), GFP_KERNEL);
	if (unlikely(!charger))
		return -ENOMEM;

	platform_set_drvdata(pdev, charger);

	charger->iodev = iodev;
	charger->dev = &pdev->dev;
	charger->vbusdetect = pdata->vbusdetect_gpio;
	charger->chgin_ilim_usb = pdata->chgin_ilim_usb;
	charger->chgin_ilim_ac = pdata->chgin_ilim_ac;
	charger->fast_charge_current= pdata->fast_charge_current;
	charger->fastcharging_ilim_usb = pdata->fastcharging_ilim_usb;
	charger->adb_open = false;
	charger->storage_open = false;
	charger->rndis_open = false;
	charger->fastcharging = 0;
	charger->battery_health = BATTERY_HEALTH_GOOD;
	charger->usb_notifer.notifier_call = max77665_charger_event;
	register_usb_gadget_notifier(&charger->usb_notifer);
	
	mutex_init(&charger->mutex_t);
	wake_lock_init(&charger->wake_lock, WAKE_LOCK_SUSPEND, pdata->name);

	INIT_DELAYED_WORK_DEFERRABLE(&charger->poll_dwork, max77665_poll_work_func);
	charger->irq_wq = create_singlethread_workqueue("max77665_chgin_irq_handler");
	INIT_DELAYED_WORK(&charger->irq_dwork, max77665_chgin_irq_handler);
	INIT_DELAYED_WORK(&charger->dwork, max77665_start_adjust_current);

	charger->ps = regulator_get(charger->dev, pdata->supply);
	if (IS_ERR(charger->ps)) {
		dev_err(&pdev->dev, "Failed to regulator_get ps: %ld\n", 
				PTR_ERR(charger->ps));
		goto err_free;
	}

	charger->reverse = regulator_get(NULL, "reverse");
	if (IS_ERR(charger->reverse)) {
		dev_err(&pdev->dev, "Failed to regulator_get reverse: %ld\n",
				PTR_ERR(charger->reverse));
		goto err_put1;
	}

	charger->battery = regulator_get(NULL, "battery");
	if (IS_ERR(charger->battery)) {
		dev_err(&pdev->dev, "Failed to regulator_get battery: %ld\n",
				PTR_ERR(charger->battery));
		goto err_put0;
	}

	ret = class_create_file(charging, &class_attr_delayedtime);
	charger->dev = device_create(charging, charger->dev->parent,
			0, charger, "charging-switch");
	ret = device_create_file(charger->dev, &dev_attr_fastcharging);
	

	charger->psy_charger.name = "charger";
	charger->psy_charger.type = POWER_SUPPLY_TYPE_BATTERY;
	charger->psy_charger.properties = max77665_charger_props,
	charger->psy_charger.num_properties = ARRAY_SIZE(max77665_charger_props),
	charger->psy_charger.get_property = max77665_charger_get_property,
	ret = power_supply_register(&pdev->dev, &charger->psy_charger);
	if (unlikely(ret != 0)) {
		dev_err(&pdev->dev, "Failed to power_supply_register psy_charger: %d\n", ret);
		goto err_put;
	}

	charger->psy_usb.name = "usb";
	charger->psy_usb.type = POWER_SUPPLY_TYPE_USB;
	charger->psy_usb.supplied_to = supply_list,
	charger->psy_usb.num_supplicants = ARRAY_SIZE(supply_list),
	charger->psy_usb.properties = max77665_power_props,
	charger->psy_usb.num_properties = ARRAY_SIZE(max77665_power_props),
	charger->psy_usb.get_property = max77665_usb_get_property,
	charger->psy_usb.set_property = max77665_usb_set_property,
	ret = power_supply_register(&pdev->dev, &charger->psy_usb);
	if (unlikely(ret != 0)) {
		dev_err(&pdev->dev, "Failed to power_supply_register psy_usb: %d\n", ret);
		goto err_put;
	}

	charger->psy_ac.name = "ac";
	charger->psy_ac.type = POWER_SUPPLY_TYPE_MAINS;
	charger->psy_ac.supplied_to = supply_list,
	charger->psy_ac.num_supplicants = ARRAY_SIZE(supply_list),
	charger->psy_ac.properties = max77665_power_props,
	charger->psy_ac.num_properties = ARRAY_SIZE(max77665_power_props),
	charger->psy_ac.get_property = max77665_ac_get_property,
	charger->psy_ac.set_property = max77665_ac_set_property,
	ret = power_supply_register(&pdev->dev, &charger->psy_ac);
	if (unlikely(ret != 0)) {
		dev_err(&pdev->dev, "Failed to power_supply_register psy_ac: %d\n", ret);
		goto err_unregister0;
	}
	
	ret = max77665_init(charger);

	alarm_init(&charger->alarm, ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
			charger_bat_alarm);

	alarm_init(&charger->delayed_alarm, ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
			delayed_adjust_work);

	ret = max77665_read_reg(iodev->regmap, MAX77665_CHG_REG_CHG_INT_OK, &reg_data);
	if (unlikely(ret < 0))
		pr_err("Failed to read MAX77665_CHG_REG_CHG_INT: %d\n", ret);
	else {
		if (!(regulator_is_enabled(charger->reverse)) 
				&& (reg_data & 0x40)) {//CHGIN 
			charger->chgin = true;
			queue_delayed_work_on(0, charger->irq_wq,
					&charger->irq_dwork, 0);
			set_alarm(&charger->alarm, WAKE_ALARM_INT);
		}
	}

	charger->chgin_irq = pdata->irq_base + MAX77665_CHG_IRQ_CHGIN_I;
	ret = request_threaded_irq(charger->chgin_irq, 0, max77665_charger_isr,
			0, pdev->name, charger);
	if (unlikely(ret < 0)) {
		dev_err(&pdev->dev, "max77665: failed to request CHGIN IRQ %d\n",
				charger->chgin_irq);
		goto err_unregister1;
	}
	
	charger->vbus_irq = gpio_to_irq(charger->vbusdetect);
	    if (charger->vbus_irq) {
		    ret = request_threaded_irq(charger->vbus_irq, 0, max77665_charger_isr, 
				    IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_SHARED,
				    "CHARGER VBUS", charger);
		    if (ret < 0) { 
			    pr_err("%s:failed to request the vbus irq %d\n", 
					    __func__, charger->vbus_irq);
			    goto err_gpio_free;
		    }
	    }


	return 0;

err_gpio_free:
	gpio_free(charger->vbusdetect);
err_unregister1:
	alarm_cancel(&charger->alarm);
	power_supply_unregister(&charger->psy_ac);
err_unregister0:	
	power_supply_unregister(&charger->psy_usb);
err_put:
	regulator_put(charger->battery);
err_put0:
	regulator_put(charger->reverse);
err_put1:
	regulator_put(charger->ps);
err_free:
	wake_lock_destroy(&charger->wake_lock);
	platform_set_drvdata(pdev, NULL);
	kfree(charger);
	return ret;
}

static __devexit int max77665_charger_remove(struct platform_device *pdev)
{
	struct max77665_charger *charger = platform_get_drvdata(pdev);

	alarm_cancel(&charger->alarm);

	cancel_delayed_work_sync(&charger->poll_dwork);

	free_irq(charger->chgin_irq, charger);
	free_irq(charger->vbus_irq, charger);
	regulator_put(charger->battery);
	regulator_put(charger->reverse);
	regulator_put(charger->ps);
	power_supply_unregister(&charger->psy_usb);
	power_supply_unregister(&charger->psy_ac);
	power_supply_unregister(&charger->psy_charger);
	platform_set_drvdata(pdev, NULL);
	kfree(charger);
	return 0;
}

static void max77665_shutdown(struct platform_device *pdev)
{
	struct max77665_charger *charger = platform_get_drvdata(pdev);
	if(regulator_is_enabled(charger->reverse))
		regulator_disable(charger->reverse);
}

#ifdef CONFIG_PM
static int max77665_suspend(struct device *dev)
{
	struct max77665_charger *charger = dev_get_drvdata(dev);

	dev_dbg(charger->dev, "%s\n", __func__);
	cancel_delayed_work_sync(&charger->poll_dwork);

	return 0;
}

static int max77665_resume(struct device *dev)
{
	struct max77665_charger *charger = dev_get_drvdata(dev);

	dev_dbg(charger->dev, "%s\n", __func__);
	schedule_delayed_work_on(0, &charger->poll_dwork, HZ);

	return 0;
}

static const struct dev_pm_ops max77665_pm_ops = {
	.suspend		= max77665_suspend,
	.resume		= max77665_resume,
};
#else
#define max77665_pm_ops NULL
#endif

static struct platform_driver max77665_charger_driver =
{
	.driver = {
		.name  = "max77665-charger", 
		.owner = THIS_MODULE,        
		.pm    = &max77665_pm_ops,   
	},
		.probe    = max77665_charger_probe,               
		.remove   = __devexit_p(max77665_charger_remove), 
		.shutdown = max77665_shutdown,                    
};

static int __init max77665_charger_init(void)
{
	return platform_driver_register(&max77665_charger_driver);
}
late_initcall(max77665_charger_init);

static void __exit max77665_charger_exit(void)
{
	platform_driver_unregister(&max77665_charger_driver);
}
module_exit(max77665_charger_exit);

MODULE_DESCRIPTION("Charger driver for MAX77665");
MODULE_AUTHOR("Chwei <Chwei@meizu.com>");
MODULE_LICENSE("GPLV2");
