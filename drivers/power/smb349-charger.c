/*
 * drivers/power/smb349-charger.c
 *
 * Battery charger driver for smb349 from summit microelectronics
 *
 * Copyright (c) 2012, SamSung Electronic Corporation.
 *
 * SMB349 Battery Driver for TF4
 *
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/wakelock.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <plat/gpio-cfg.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <plat/map-base.h>

#undef  SMB349_DEBUG

#define SMB349_CHARGE		0x00
#define SMB349_CHRG_CRNTS	0x01
#define SMB349_VRS_FUNC		0x02
#define SMB349_FLOAT_VLTG	0x03
#define SMB349_CHRG_CTRL	0x04
#define SMB349_STAT_TIME_CTRL	0x05
#define SMB349_PIN_CTRL		0x06
#define SMB349_THERM_CTRL	0x07
#define SMB349_CTRL_REG		0x09

#define SMB349_OTG_TLIM_REG	0x0A
#define SMB349_HRD_SFT_TEMP	0x0B
#define SMB349_FAULT_INTR	0x0C
#define SMB349_STS_INTR_1	0x0D
#define SMB349_SYSOK_USB3	0x0E
#define SMB349_IN_CLTG_DET	0x10
#define SMB349_STS_INTR_2	0x11

#define SMB349_CMD_REG		0x30
#define SMB349_CMD_REG_B	0x31
#define SMB349_CMD_REG_c	0x33

#define SMB349_INTR_STS_A	0x35
#define SMB349_INTR_STS_B	0x36
#define SMB349_INTR_STS_C	0x37
#define SMB349_INTR_STS_D	0x38
#define SMB349_INTR_STS_E	0x39
#define SMB349_INTR_STS_F	0x3A

#define SMB349_STS_REG_A	0x3B
#define SMB349_STS_REG_B	0x3C
#define SMB349_STS_REG_C	0x3D
#define SMB349_STS_REG_D	0x3E
#define SMB349_STS_REG_E	0x3F

#define SMB349_ENABLE_WRITE	1
#define SMB349_DISABLE_WRITE	0
#define ENABLE_WRT_ACCESS	0x80
#define THERM_CTRL		0x10
#define BATTERY_MISSING		0x10
#define CHARGING		0x06
#define CHRG_DOWNSTRM_PORT	0x08
#define ENABLE_CHARGE		0x02

#define STANDARD_DOWNSTREAM_PORT	0x02
#define DEDICATED_CHARGER		0x04
#define CHARGING_DOWNSTREAM_PORT	0x08
#define ACCESORY_CHARGER_ADAPTER	0xf0
#define OTHER_CHARGING_PORT		0x01
#define APSD_COMPL_STA		0x40
#define PWR_OK_STA		0x01

#define SMB349_POLLING_INTERVAL				12*1000 // 12 sec

typedef enum {
	CHARGER_BATTERY = 0,
	CHARGER_AC,
	CHARGER_DISCHARGE
} charger_type_t;

struct battery_info {
	u32 batt_id;		/* Battery ID from ADC */
	u32 batt_vol;		/* Battery voltage from ADC */
	u32 batt_temp;		/* Battery Temperature (C) from ADC */
	u32 batt_temp_adc;	/* Battery Temperature ADC value */
	u32 batt_temp_adc_cal;	/* Battery Temperature ADC value (calibrated) */
	u32 level;		/* formula */
	u32 charging_source;	/* 0: no cable, 1:AC        */
	u32 charging_enabled;	/* 0: Disable, 1: Enable */
	u32 charging_error; /*0: ok, 1: should stop charging*/
	u32 batt_health;	/* Battery Health (Authority) */
	u32 batt_is_full;	/* 0 : Not full 1: Full */
};

struct smb349_battery_info {
	int present;
	int polling;
	int delay_work_time;
	struct delayed_work polling_work;		
	struct delayed_work charger_work;	
	struct battery_info bat_info;
};


static enum power_supply_property smb349_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
};

static enum power_supply_property smb349_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *supply_list[] = {
	"battery",
};

static int smb349_bat_get_property(struct power_supply *bat_ps,
					 enum power_supply_property psp,
					 union power_supply_propval *val);

static int smb349_power_get_property(struct power_supply *bat_ps,
				enum power_supply_property psp,
				union power_supply_propval *val);
	

static struct power_supply smb349_power_supplies[] = {
	{
		.name = "battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = smb349_battery_properties,
		.num_properties = ARRAY_SIZE(smb349_battery_properties),
		.get_property = smb349_bat_get_property,
	}
	, {
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = smb349_power_properties,
		.num_properties = ARRAY_SIZE(smb349_power_properties),
		.get_property = smb349_power_get_property,
	},
};


/* lock to protect the battery info */
static DEFINE_MUTEX(work_lock);

extern int smb349_bat_get_soc(void);
extern int smb349_bat_get_vol(void);

static struct smb349_battery_info smb349_bat_info;

				 
/* Prototypes */
static ssize_t smb349_bat_show_property(struct device *dev,
				struct device_attribute *attr,
				char *buf);
static ssize_t smb349_bat_store_property(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);
				
static int smb349_configure_charger(struct i2c_client *client, int value);
static int smb349_configure_interrupts(struct i2c_client *client);

static struct i2c_client *smb349_i2c_client = NULL;

static u32 smb349_bat_get_health(void)
{
	return smb349_bat_info.bat_info.batt_health;
}

static int smb349_bat_get_temp(void)
{
	int temp = 22;

	return temp;
}

int charger_ac_online(void) {
	return (smb349_bat_info.bat_info.charging_source==CHARGER_AC);
}
EXPORT_SYMBOL(charger_ac_online);


static int set_chg_led(int on)
{
    int err;

    err = gpio_request(EXYNOS5410_GPF1(6), "CHG_LED");
    if(err < 0){
     	return 0;
    }
    if (on){
    	s3c_gpio_cfgpin(EXYNOS5410_GPF1(6), S3C_GPIO_SFN(1));
    	s3c_gpio_setpull(EXYNOS5410_GPF1(6), S3C_GPIO_PULL_NONE);
    	gpio_direction_output(EXYNOS5410_GPF1(6), on); 
    }else{
	    s3c_gpio_cfgpin(EXYNOS5410_GPF1(6), S3C_GPIO_SFN(0));
	    s3c_gpio_setpull(EXYNOS5410_GPF1(6), S3C_GPIO_PULL_NONE);
	    gpio_direction_input(EXYNOS5410_GPF1(6));
    }
    gpio_free(EXYNOS5410_GPF1(6));
    return 1;
}

static int smb349_read(struct i2c_client *client, int reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int smb349_write(struct i2c_client *client, int reg, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, value);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int smb349_update_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret, retval;

	retval = smb349_read(client, reg);
	if (retval < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, retval);
		return retval;
	}

	ret = smb349_write(client, reg, retval | value);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	return ret;
}

int smb349_volatile_writes(struct i2c_client *client, uint8_t value)
{
	int ret = 0;

	if (value == SMB349_ENABLE_WRITE) {
		/* Enable volatile write to config registers */
		ret = smb349_update_reg(client, SMB349_CMD_REG,
						ENABLE_WRT_ACCESS);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing"
				"register 0x%02x\n", __func__, SMB349_CMD_REG);
			return ret;
		}
	} else {
		ret = smb349_read(client, SMB349_CMD_REG);
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return ret;
		}

		ret = smb349_write(client, SMB349_CMD_REG, ret & (~(1<<7)));
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return ret;
		}
	}
	return ret;
}

static void smb349_clear_interrupts(struct i2c_client *client)
{
	uint8_t val, buf[6];
#ifdef SMB349_DEBUG
	int debug_1, debug_2, debug_3 = 0;
	debug_1 = smb349_read(client,SMB349_PIN_CTRL));
	debug_2 = smb349_read(client,SMB349_FAULT_INTR));
	debug_3 = smb349_read(client,SMB349_STS_INTR_1));
#endif
	val = i2c_smbus_read_i2c_block_data(client, SMB349_INTR_STS_A, 6, buf);
	if(val < 0)
		dev_err(&client->dev, "%s(): Failed in clearing interrupts\n", __func__);
}

static int smb349_configure_otg(struct i2c_client *client, int enable)
{
	int ret = 0;

	/*Enable volatile writes to registers*/
	ret = smb349_volatile_writes(client, SMB349_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s error in configuring otg..\n",
								__func__);
		goto error;
	}

	if (enable) {
		/* Configure PGOOD to be active low if no 5V on VBUS */
		ret = smb349_read(client, SMB349_STS_REG_C);
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}

		if (!(ret & 0x01)) {
			ret = smb349_read(client, SMB349_SYSOK_USB3);
			if (ret < 0) {
				dev_err(&client->dev, "%s: err %d\n",
					__func__, ret);
				goto error;
			}

			ret = smb349_write(client, SMB349_SYSOK_USB3,
						(ret & (~(1))));
			if (ret < 0) {
				dev_err(&client->dev, "%s: err %d\n",
					__func__, ret);
				goto error;
			}
		}

		/* Enable OTG */
		ret = smb349_update_reg(client, SMB349_CMD_REG, 0x10);
		if (ret < 0) {
			dev_err(&client->dev, "%s: Failed in writing register"
				"0x%02x\n", __func__, SMB349_CMD_REG);
			goto error;
		}
	} else {
		/* Disable OTG */
		ret = smb349_read(client, SMB349_CMD_REG);
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}

		ret = smb349_write(client, SMB349_CMD_REG, (ret & (~(1<<4))));
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}

		/* Configure PGOOD to be active high */
		ret = smb349_write(client, SMB349_SYSOK_USB3, 0x00);
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}
	}

	/* Disable volatile writes to registers */
	ret = smb349_volatile_writes(client, SMB349_DISABLE_WRITE);
	if (ret < 0)
		dev_err(&client->dev, "%s error in configuring OTG..\n",
								__func__);
error:
	return ret;
}


int read_charger_status(void)
{
	if(smb349_bat_info.bat_info.charging_source == CHARGER_AC)
		return 1;
	return 0;		
}
EXPORT_SYMBOL(read_charger_status);


static int smb349_configure_charger(struct i2c_client *client, int value)
{
	int ret = 0;
#ifdef SMB349_DEBUG
  printk("[SMB349] smb349_configure_charger value=%d\n",value);
#endif
	/* Enable volatile writes to registers */
	ret = smb349_volatile_writes(client, SMB349_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}

	if (value) {
		 /* Enable charging */
		ret = smb349_update_reg(client, SMB349_CMD_REG, ENABLE_CHARGE);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing register"
					"0x%02x\n", __func__, SMB349_CMD_REG);
			goto error;
		}

		/* Configure THERM ctrl */
		ret = smb349_update_reg(client, SMB349_THERM_CTRL, THERM_CTRL);
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;	 
		}
		smb349_bat_info.bat_info.charging_source = CHARGER_AC;
	} else {
		ret = smb349_read(client, SMB349_CMD_REG);
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}

		ret = smb349_write(client, SMB349_CMD_REG, (ret & (~(1<<1))));
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}
		smb349_bat_info.bat_info.charging_source = CHARGER_BATTERY;
	}
	/* Disable volatile writes to registers */
	ret = smb349_volatile_writes(client, SMB349_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}
//	write_charger_status(value);
error:
	return ret;
}

void smb349_power_src_det(void)
{
	struct i2c_client *client = smb349_i2c_client;
	int val, val_1;

#ifdef SMB349_DEBUG
  printk("[SMB349] smb349_power_src_det\n");
#endif

	val = smb349_read(client, SMB349_INTR_STS_D);
	if(val < 0)
	{

		goto irq_error;
	}

	val_1 = smb349_read(client, SMB349_INTR_STS_F);
	if(val_1 < 0)
	{
		printk(KERN_ERR "[SMB349] Read SMB349_INTR_F failed..\n");
		goto irq_error;
	}

	// APSD completed status & Power ok status
	if((val & APSD_COMPL_STA) && (val_1 & PWR_OK_STA))
	{
		printk("[SMB349] Plug Charger\n");
		smb349_configure_charger(client, 1); 
    set_chg_led(1); 
	}
	else
	{
		printk("[SMB349] Unplug Charger\n");
  	smb349_configure_charger(client, 0); 
    set_chg_led(0);  
	}
		
	power_supply_changed(&smb349_power_supplies[CHARGER_AC]); 

irq_error:
	return;
}


static irqreturn_t smb349_status_isr(int irq, void *dev_id)
{
#ifdef SMB349_DEBUG	
	printk("[SMB349] smb349_status_isr\n");
#endif
	smb349_clear_interrupts(smb349_i2c_client);
	cancel_delayed_work_sync(&smb349_bat_info.charger_work);
	schedule_delayed_work(&smb349_bat_info.charger_work, msecs_to_jiffies(90));
	return IRQ_HANDLED;
}

static void smb349_delay_work_func(struct work_struct *work)
{
#ifdef SMB349_DEBUG					
  printk("[SMB349] smb349_delay_work_func\n");
#endif
	smb349_clear_interrupts(smb349_i2c_client);
	smb349_power_src_det();
}


static int smb349_configure_init_setting(struct i2c_client *client)
{
	int ret = 0;

	// Enable volatile writes to register
	ret = smb349_volatile_writes(client, SMB349_ENABLE_WRITE);
	if(ret < 0)
	{
		dev_err(&client->dev, "%s() error in configuring charger..\n", __func__);
		goto error;
	}
	
  ret = smb349_write(client, SMB349_CHARGE/*0x00*/, 0xFF); // Fast Charger Current : 4000mA , Input Current Linit : 3500mA
  if(ret < 0)
	{
		dev_err(&client->dev, "%s() Failed in writing register 0x%02x\n", __func__, SMB349_CMD_REG);
		goto error;
	}
	
	// Disbale volatile writes to register
	ret = smb349_volatile_writes(client, SMB349_DISABLE_WRITE);
	if(ret < 0)
	{
		dev_err(&client->dev, "%s() error in configuring charger..\n", __func__);
		goto error;
	}
	
error:
	return ret;
}

static int smb349_configure_interrupts(struct i2c_client *client)
{
	int ret = 0;

	// Enable volatile writes to register
	ret = smb349_volatile_writes(client, SMB349_ENABLE_WRITE);
	if(ret < 0)
	{
		dev_err(&client->dev, "%s() error in configuring charger..\n", __func__);
		goto error;
	}
  	ret = smb349_write(client, SMB349_FAULT_INTR, 0x00/*0xff8*/);
	if(ret < 0)
	{
		dev_err(&client->dev, "%s() Failed in writing register 0x%02x\n", __func__, SMB349_CMD_REG);
		goto error;
	}

	ret = smb349_write(client, SMB349_STS_INTR_1, 0x07/*0xff*/);
	if(ret < 0)
	{
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		goto error;
	}

	// Disbale volatile writes to register
	ret = smb349_volatile_writes(client, SMB349_DISABLE_WRITE);
	if(ret < 0)
	{
		dev_err(&client->dev, "%s() error in configuring charger..\n", __func__);
		goto error;
	}

error:
	return ret;
}

static int smb349_bat_get_charging_status(void)
{
	charger_type_t charger = CHARGER_BATTERY;
	int ret = 0;

	charger = smb349_bat_info.bat_info.charging_source;

	switch (charger) {
	case CHARGER_BATTERY:
		ret = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case CHARGER_AC:
		if (smb349_bat_info.bat_info.level == 100 &&
		    smb349_bat_info.bat_info.batt_is_full)
			ret = POWER_SUPPLY_STATUS_FULL;
		else
			ret = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case CHARGER_DISCHARGE:
		ret = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	default:
		ret = POWER_SUPPLY_STATUS_UNKNOWN;
	}
	return ret;
}

static int smb349_bat_get_property(struct power_supply *bat_ps,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = smb349_bat_get_charging_status();
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = smb349_bat_get_health();
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = smb349_bat_info.present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = smb349_bat_info.bat_info.level;	
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = smb349_bat_info.bat_info.batt_temp;
		dev_dbg(bat_ps->dev, "%s : temp = %d\n", __func__,
				val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = smb349_bat_info.bat_info.batt_vol;
		dev_dbg(bat_ps->dev, "%s : volt = %d\n", __func__,	val->intval);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}


static int smb349_power_get_property(struct power_supply *bat_ps,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	charger_type_t charger;

	dev_dbg(bat_ps->dev, "%s : psp = %d\n", __func__, psp);

	charger = smb349_bat_info.bat_info.charging_source;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (bat_ps->type == POWER_SUPPLY_TYPE_MAINS) {
			val->intval = (charger == CHARGER_AC ? 1 : 0);
   	}
		else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

#define SMB349_BAT_ATTR(_name)			\
{							\
	.attr = { .name = #_name,},			\
	.show = smb349_bat_show_property,		\
	.store = smb349_bat_store_property,	\
}

static struct device_attribute smb349_battery_attrs[] = {
	SMB349_BAT_ATTR(batt_temp),
	SMB349_BAT_ATTR(batt_temp_adc),
	SMB349_BAT_ATTR(batt_temp_adc_cal),
};

enum {
	BATT_VOL = 0,
	BATT_TEMP,
	BATT_TEMP_ADC,
	BATT_TEMP_ADC_CAL,
};

static int smb349_bat_create_attrs(struct device * dev)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(smb349_battery_attrs); i++) {
		rc = device_create_file(dev, &smb349_battery_attrs[i]);
		if (rc)
		goto attrs_failed;
	}
	goto succeed;

attrs_failed:
	while (i--)
	device_remove_file(dev, &smb349_battery_attrs[i]);
succeed:
	return rc;
}

static ssize_t smb349_bat_show_property(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int i = 0;
	const ptrdiff_t off = attr - smb349_battery_attrs;

	switch (off) {
	case BATT_VOL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			smb349_bat_info.bat_info.batt_vol);
	break;
	case BATT_TEMP:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			smb349_bat_info.bat_info.batt_temp);
		break;
	case BATT_TEMP_ADC:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			smb349_bat_info.bat_info.batt_temp_adc);
		break;
	case BATT_TEMP_ADC_CAL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			smb349_bat_info.bat_info.batt_temp_adc_cal);
		break;
	default:
		i = -EINVAL;
	}
	return i;
}

static ssize_t smb349_bat_store_property(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int x = 0;
	int ret = 0;
	const ptrdiff_t off = attr - smb349_battery_attrs;

	switch (off) {
	case BATT_TEMP_ADC_CAL:
		if (sscanf(buf, "%d\n", &x) == 1) {
			smb349_bat_info.bat_info.batt_temp_adc_cal = x;
			ret = count;
		}
		dev_info(dev, "%s : batt_temp_adc_cal = %d\n", __func__, x);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}


static void smb349_bat_status_update(void)
{
	int old_level, old_temp, old_batt_vol;
#ifdef SMB349_DEBUG		
  printk("[SMB349] smb349_bat_status_update\n");
#endif
	mutex_lock(&work_lock);

	old_temp = smb349_bat_info.bat_info.batt_temp;
	old_level = smb349_bat_info.bat_info.level;
	old_batt_vol = smb349_bat_info.bat_info.batt_vol;

	smb349_bat_info.bat_info.batt_temp = smb349_bat_get_temp( );
	smb349_bat_info.bat_info.level = smb349_bat_get_soc();	
	smb349_bat_info.bat_info.batt_vol = smb349_bat_get_vol();

	//temp protect battery, deatailed feature should be tested in the future
	if(smb349_bat_info.bat_info.level < 3 || smb349_bat_info.bat_info.batt_vol <= 3630)
		smb349_bat_info.bat_info.level = 0;

	if (old_level != smb349_bat_info.bat_info.level ||
	    old_temp != smb349_bat_info.bat_info.batt_temp ||
	    old_batt_vol != smb349_bat_info.bat_info.batt_vol) {

		power_supply_changed(&smb349_power_supplies[CHARGER_BATTERY]);
	}
	mutex_unlock(&work_lock);

	//temp for fake eoc
	if(smb349_bat_info.bat_info.level >= 95)	{
		smb349_bat_info.bat_info.batt_is_full = 1;
	}
	else if(smb349_bat_info.bat_info.level < 90)	{	
		smb349_bat_info.bat_info.batt_is_full = 0;
  } 
}

static void smb349_polling_work(struct work_struct *work)
{	
	smb349_bat_status_update();
	schedule_delayed_work(&smb349_bat_info.polling_work, msecs_to_jiffies(SMB349_POLLING_INTERVAL));	
}

int smb349_battery_online(void)
{
	int val, bOnline = 0;
  struct i2c_client *client = smb349_i2c_client;
  
	val = smb349_read(client, SMB349_INTR_STS_B);
	if (val < 0) {
		return val;
	}
	
	if (val & BATTERY_MISSING)
		bOnline = 0;
	else
		bOnline = 1;
		
	printk("[SMB349] battery_online=%d/ INTR_STS_B(0x36h)=0x%x\n",bOnline, val);
	return bOnline;
}


static int __devinit smb349_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct smb349_charger_platform_data *pdata;
	int ret, irq_num, err, i;

  int soc = smb349_bat_get_soc();
  int vol = smb349_bat_get_vol();
  
 
	smb349_i2c_client = client;
	printk("[SMB349] soc=%d, vol=%d\n",soc,vol);
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))	{
		return -EIO;
  }
	pdata = client->dev.platform_data;

	smb349_bat_info.present = 1;
	smb349_bat_info.bat_info.batt_id = 0;
	smb349_bat_info.bat_info.batt_vol = 0;
	smb349_bat_info.bat_info.batt_temp = 20;
	smb349_bat_info.bat_info.batt_temp_adc = 20;
	smb349_bat_info.bat_info.batt_temp_adc_cal = 20;
	smb349_bat_info.bat_info.level = 10;
	smb349_bat_info.bat_info.batt_health = POWER_SUPPLY_HEALTH_GOOD;
	smb349_bat_info.bat_info.charging_source = CHARGER_BATTERY;
	smb349_bat_info.bat_info.charging_enabled = 0;
	smb349_bat_info.bat_info.batt_is_full = 0;
	smb349_bat_info.bat_info.charging_error = 0;
	
		/* init power supplier framework */
	for (i = 0; i < ARRAY_SIZE(smb349_power_supplies); i++) {
		ret = power_supply_register(&client->dev,  &smb349_power_supplies[i]);
		if (ret) {
			goto error;
		}
	}
	
		/* create sec detail attributes */
	smb349_bat_create_attrs(smb349_power_supplies[CHARGER_BATTERY].dev);
	
	
	/* disable OTG */
	ret = smb349_configure_otg(smb349_i2c_client, 0);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring"
			"charger..\n", __func__);
		goto error;
	}

	/* SMB349 init value */
	ret = smb349_configure_init_setting(smb349_i2c_client);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring"
			"charger..\n", __func__);
		goto error;
	}

  if(smb349_battery_online()){
  	ret =  smb349_read(client, SMB349_STS_REG_D);
	  if (ret < 0) {
  		dev_err(&client->dev, "%s(): Failed in reading register"
  			"0x%02x\n", __func__, SMB349_STS_REG_D);
  		goto error;  
  	} else if (ret != 0) {
  		/* configure charger */
  		ret = smb349_configure_charger(client, 1);
  		if (ret < 0) {
  			dev_err(&client->dev, "%s() error in configuring"
  				"charger..\n", __func__);
  			goto error;
  		}
  	} else {
  		/* disable charger */
  		ret = smb349_configure_charger(client, 0);
  		if (ret < 0) {
  			printk("[test] fail 6\n");
  			dev_err(&client->dev, "%s() error in configuring"
  				"charger..\n", __func__);
  			goto error;
  		}
  	}
	} else {
		smb349_bat_info.present = 0;
		printk("No Battery present, exiting..\n");
		ret = -ENODEV;
		goto error;
	}
	
	smb349_bat_status_update();

	smb349_clear_interrupts(smb349_i2c_client);
	smb349_bat_info.delay_work_time = 2500; 
	INIT_DELAYED_WORK_DEFERRABLE(&smb349_bat_info.charger_work, smb349_delay_work_func);

	ret = smb349_configure_interrupts(client);
	if(ret < 0)	{
		dev_err(&client->dev, "%s(): error in configuring charger..\n", __func__);
		goto error;
	}

	smb349_power_src_det();

	err = gpio_request(EXYNOS5410_GPX1(2), "CHG_STAT#_349");
	if(err < 0)
		printk("[SMB349] GPIO request fail!!\n");
	s3c_gpio_cfgpin(EXYNOS5410_GPX1(2), S3C_GPIO_SFN(0));
	s3c_gpio_setpull(EXYNOS5410_GPX1(2), S3C_GPIO_PULL_UP);
	gpio_direction_input(EXYNOS5410_GPX1(2));

	irq_num = gpio_to_irq(EXYNOS5410_GPX1(2));
	ret = request_threaded_irq(irq_num, NULL, smb349_status_isr, 
		      IRQ_TYPE_EDGE_FALLING | IRQF_ONESHOT, "smb349", &smb349_bat_info);
	if(ret < 0) 	{
		dev_err(&client->dev, "%s(): Failed in requesting isr\n", __func__);
		goto error;
	}
	
	INIT_DELAYED_WORK_DEFERRABLE(&smb349_bat_info.polling_work, smb349_polling_work);		
  schedule_delayed_work(&smb349_bat_info.polling_work, HZ);
	return 0;

error:
	for (i = 0; i < ARRAY_SIZE(smb349_power_supplies); i++)
		power_supply_unregister(&smb349_power_supplies[i]);
	return ret;
}

static int __devexit smb349_remove(struct i2c_client *client)
{
	int i;
	cancel_delayed_work_sync(&smb349_bat_info.charger_work);
	for (i = 0; i < ARRAY_SIZE(smb349_power_supplies); i++)
		power_supply_unregister(&smb349_power_supplies[i]);
	return 0;
}

static const struct i2c_device_id smb349_id[] = {
	{ "smb349", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, smb349_id);

#ifdef CONFIG_PM
static int smb349_charger_suspend(struct device *dev);
static int smb349_charger_resume(struct device *dev);

static int smb349_charger_suspend(struct device *dev)
{
	cancel_delayed_work_sync(&smb349_bat_info.charger_work);
	printk(KERN_INFO "%s\n", __func__);
	return 0;
}

static int smb349_charger_resume(struct device *dev)
{
	schedule_delayed_work(&smb349_bat_info.charger_work, msecs_to_jiffies(500));
	printk(KERN_INFO "%s\n", __func__);
	return 0;
}

static const struct dev_pm_ops smb349_charger_pm_ops = {
	.suspend = smb349_charger_suspend,
	.resume  = smb349_charger_resume,
};
#endif

static struct i2c_driver smb349_i2c_driver = {
	.driver	= {
		.name	= "smb349",
#ifdef CONFIG_PM
		.pm	= &smb349_charger_pm_ops,
#endif
	},
	.probe		= smb349_probe,
	.remove		= __devexit_p(smb349_remove),
	.id_table	= smb349_id,
};

static int __init smb349_init(void)
{
	return i2c_add_driver(&smb349_i2c_driver);
}


static void __exit smb349_exit(void)
{
	i2c_del_driver(&smb349_i2c_driver);
}
module_init(smb349_init);
module_exit(smb349_exit);

MODULE_DESCRIPTION("SMB349 Battery-Charger");
MODULE_LICENSE("GPL");
