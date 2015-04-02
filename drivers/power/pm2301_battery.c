/*
 * PM2301 Battery Driver for SSCR TE4
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
//#include <mach/regs-pmu5.h>
#include <asm/io.h>
#include <plat/map-base.h>
//#include <plat/map-s5p.h>


#define PM2301_CHARGER_SCHEDULE_INTERVAL	200
#define PM2301_POLLING_INTERVAL				20*1000 // 20 sec

#define PM2301_DEBUG

#ifdef PM2301_DEBUG
#define Pm2301Dbg printk
#else
#define Pm2301Dbg(fmt, ...)						\
	do {								\
		if (0)							\
			printk(fmt, ##__VA_ARGS__);\
	} while (0)
#endif

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
	u32 charging_source;	/* 0: no cable, 1:usb, 2:AC */
	u32 charging_enabled;	/* 0: Disable, 1: Enable */
	u32 charging_error; /*0: ok, 1: should stop charging*/
	u32 batt_health;	/* Battery Health (Authority) */
	u32 batt_is_full;	/* 0 : Not full 1: Full */
};

struct pm2301_battery_info {
	int present;
	int polling;
	unsigned long polling_interval;
	struct delayed_work polling_work;		
	struct delayed_work charger_work;	

	struct battery_info bat_info;
};

static struct wake_lock vbus_wake_lock;
static struct i2c_client *pm2301 = NULL;
static int system_suspended = 0;
static int pm2301_battery_initial = 0;
static struct device *dev;

static char *status_text[] = {
	[POWER_SUPPLY_STATUS_UNKNOWN] =		"Unknown",
	[POWER_SUPPLY_STATUS_CHARGING] =	"Charging",
	[POWER_SUPPLY_STATUS_DISCHARGING] =	"Discharging",
	[POWER_SUPPLY_STATUS_NOT_CHARGING] =	"Not Charging",
	[POWER_SUPPLY_STATUS_FULL] =		"Full",
};

/* lock to protect the battery info */
static DEFINE_MUTEX(work_lock);

static struct pm2301_battery_info pm2301_bat_info;

/* Prototypes */
static ssize_t pm2301_bat_show_property(struct device *dev,
				struct device_attribute *attr,
				char *buf);
static ssize_t pm2301_bat_store_property(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);

static void pm2301_bat_stop_charging(void);

/* Externs */
extern int pm2301_bat_get_soc(void);
extern int pm2301_bat_get_vol(void);

static u32 pm2301_bat_get_health(void)
{
	return pm2301_bat_info.bat_info.batt_health;
}

static int pm2301_bat_get_temp(void)
{
	int temp = 22;

	return temp;
}

int pm2301_reg_read(struct i2c_client *i2c, u8 reg)
{
	int ret;
	ret = i2c_smbus_read_byte_data(i2c, reg);
	return ret;
}

int pm2301_reg_write(struct i2c_client *i2c, u8 reg, u8 val)
{
	int ret;
	ret = i2c_smbus_write_byte_data(i2c, reg, val);
	return ret;
}

static int pm2301_bat_get_charging_status(void)
{
	charger_type_t charger = CHARGER_BATTERY;
	int ret = 0;

	charger = pm2301_bat_info.bat_info.charging_source;

	switch (charger) {
	case CHARGER_BATTERY:
		ret = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case CHARGER_AC:
		if (pm2301_bat_info.bat_info.level == 100 &&
		    pm2301_bat_info.bat_info.batt_is_full)
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
	dev_dbg(dev, "%s : %s\n", __func__, status_text[ret]);

	return ret;
}

static int pm2301_bat_get_property(struct power_supply *bat_ps,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	dev_dbg(dev,"%s : psp = %d", __func__, psp);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = pm2301_bat_get_charging_status();
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = pm2301_bat_get_health();
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = pm2301_bat_info.present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = pm2301_bat_info.bat_info.level;	
		dev_dbg(dev, "%s : level = %d\n", __func__,
				val->intval);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = pm2301_bat_info.bat_info.batt_temp;
		dev_dbg(bat_ps->dev, "%s : temp = %d\n", __func__,
				val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = pm2301_bat_info.bat_info.batt_vol;
		dev_dbg(bat_ps->dev, "%s : volt = %d\n", __func__,	val->intval);
		break;
	default:
		return -EINVAL;
	}
	dev_dbg(dev,"  value = %d\n", val->intval);
	
	return 0;
}

static int pm2301_power_get_property(struct power_supply *bat_ps,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	charger_type_t charger;

	dev_dbg(bat_ps->dev, "%s : psp = %d\n", __func__, psp);

	charger = pm2301_bat_info.bat_info.charging_source;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (bat_ps->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = (charger == CHARGER_AC ? 1 : 0);
		else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

#define PM2301_BAT_ATTR(_name)			\
{							\
	.attr = { .name = #_name,},			\
	.show = pm2301_bat_show_property,		\
	.store = pm2301_bat_store_property,	\
}

static struct device_attribute pm2301_battery_attrs[] = {
	PM2301_BAT_ATTR(batt_temp),
	PM2301_BAT_ATTR(batt_temp_adc),
	PM2301_BAT_ATTR(batt_temp_adc_cal),
};

enum {
	BATT_VOL = 0,
	BATT_TEMP,
	BATT_TEMP_ADC,
	BATT_TEMP_ADC_CAL,
};

static int pm2301_bat_create_attrs(struct device * dev)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(pm2301_battery_attrs); i++) {
		rc = device_create_file(dev, &pm2301_battery_attrs[i]);
		if (rc)
		goto attrs_failed;
	}
	goto succeed;

attrs_failed:
	while (i--)
	device_remove_file(dev, &pm2301_battery_attrs[i]);
succeed:
	return rc;
}

static ssize_t pm2301_bat_show_property(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int i = 0;
	const ptrdiff_t off = attr - pm2301_battery_attrs;

	switch (off) {
	case BATT_VOL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			pm2301_bat_info.bat_info.batt_vol);
	break;
	case BATT_TEMP:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			pm2301_bat_info.bat_info.batt_temp);
		break;
	case BATT_TEMP_ADC:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			pm2301_bat_info.bat_info.batt_temp_adc);
		break;
	case BATT_TEMP_ADC_CAL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			pm2301_bat_info.bat_info.batt_temp_adc_cal);
		break;
	default:
		i = -EINVAL;
	}
	return i;
}

static ssize_t pm2301_bat_store_property(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int x = 0;
	int ret = 0;
	const ptrdiff_t off = attr - pm2301_battery_attrs;

	switch (off) {
	case BATT_TEMP_ADC_CAL:
		if (sscanf(buf, "%d\n", &x) == 1) {
			pm2301_bat_info.bat_info.batt_temp_adc_cal = x;
			ret = count;
		}
		dev_info(dev, "%s : batt_temp_adc_cal = %d\n", __func__, x);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static enum power_supply_property pm2301_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
};

static enum power_supply_property pm2301_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *supply_list[] = {
	"battery",
};

static struct power_supply pm2301_power_supplies[] = {
	{
		.name = "battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = pm2301_battery_properties,
		.num_properties = ARRAY_SIZE(pm2301_battery_properties),
		.get_property = pm2301_bat_get_property,
	}
	, {
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = pm2301_power_properties,
		.num_properties = ARRAY_SIZE(pm2301_power_properties),
		.get_property = pm2301_power_get_property,
	},
};

static int pm2301_cable_status_update(void)
{
	int ret = 0;
	const char *msg = "";

	dev_dbg(dev, "%s\n", __func__);

	if (!pm2301_battery_initial)
		return -EPERM;

	switch (pm2301_bat_info.bat_info.charging_source) {
	case CHARGER_BATTERY:
		msg = "Battery";
		break;
	case CHARGER_AC:
		msg = "cable AC";
		break;
	case CHARGER_DISCHARGE:
		msg = "Discharge";
		break;
	default:
		dev_err(dev, "%s : cable status is not supported\n", __func__);
		ret = -EINVAL;
		break;
	}

	wake_lock_timeout(&vbus_wake_lock, HZ / 2);

	/* if the power source changes, all power supplies may change state */
	power_supply_changed(&pm2301_power_supplies[CHARGER_AC]);

	dev_dbg(dev, "%s : call power_supply_changed\n", __func__);
	return ret;
}

static void pm2301_bat_status_update(void)
{
	int old_level, old_temp, old_batt_vol;
	dev_dbg(dev, "%s ++\n", __func__);

	if (!pm2301_battery_initial)
		return;	

	mutex_lock(&work_lock);

	old_temp = pm2301_bat_info.bat_info.batt_temp;
	old_level = pm2301_bat_info.bat_info.level;
	old_batt_vol = pm2301_bat_info.bat_info.batt_vol;

	pm2301_bat_info.bat_info.batt_temp = pm2301_bat_get_temp( );
	pm2301_bat_info.bat_info.level = pm2301_bat_get_soc();	
	pm2301_bat_info.bat_info.batt_vol = pm2301_bat_get_vol();

	//temp protect battery, deatailed feature should be tested in the future
	if(pm2301_bat_info.bat_info.level < 3 || pm2301_bat_info.bat_info.batt_vol < 3650)
		pm2301_bat_info.bat_info.level = 0;

#if 0 //temp for fake eoc 
	if(pm2301_bat_info.bat_info.batt_is_full == 1)
		pm2301_bat_info.bat_info.level = 100;
	else if(pm2301_bat_info.bat_info.level > 99)
		pm2301_bat_info.bat_info.level = 99;
#endif	

	if (old_level != pm2301_bat_info.bat_info.level ||
	    old_temp != pm2301_bat_info.bat_info.batt_temp ||
	    old_batt_vol != pm2301_bat_info.bat_info.batt_vol) {

		power_supply_changed(&pm2301_power_supplies[CHARGER_BATTERY]);
		dev_dbg(dev, "%s : call power_supply_changed\n", __func__);
	}
	mutex_unlock(&work_lock);

	//temp for fake eoc
	if(pm2301_bat_info.bat_info.level >= 92)
		pm2301_bat_info.bat_info.batt_is_full = 1;
	else if(pm2301_bat_info.bat_info.level < 85)
		pm2301_bat_info.bat_info.batt_is_full = 0;

	if(pm2301_bat_info.bat_info.batt_is_full)
		pm2301_bat_stop_charging();
	
	Pm2301Dbg("[pm2301] batt:%d \n", pm2301_bat_info.present);
}

static void pm2301_polling_work(struct work_struct *work)
{	
	pm2301_bat_status_update();
	schedule_delayed_work(&pm2301_bat_info.polling_work, msecs_to_jiffies(PM2301_POLLING_INTERVAL));	
}

#ifdef CONFIG_PM
static int pm2301_bat_suspend(struct device *dev)
{
	if(pm2301_bat_info.present==0)
		return 0;
	cancel_delayed_work(&pm2301_bat_info.charger_work);
	cancel_delayed_work(&pm2301_bat_info.polling_work);
	system_suspended = 1;
	return 0;
}


static void pm2301_bat_resume(struct device *dev)
{
	if(pm2301_bat_info.present==0)
		return 0;

	/*clear irqs*/
	pm2301_reg_read(pm2301, 0x40);
	pm2301_reg_read(pm2301, 0x41);
	pm2301_reg_read(pm2301, 0x42);
	pm2301_reg_read(pm2301, 0x43);
	pm2301_reg_read(pm2301, 0x44);
 	pm2301_reg_read(pm2301, 0x45);

	pm2301_reg_write(pm2301, 0x50, 0x0D); // bat_low edge info[not detect], bat disconnect info
	pm2301_reg_write(pm2301, 0x51, 0x03); // mask plug/unplug ir, only detect pwr1
	pm2301_reg_write(pm2301, 0x52, 0x08); // charging watchdog irq
	pm2301_reg_write(pm2301, 0x53, 0xB4); // bit7 cvphase bit6 bat_full bit5 resume bit4 charging_on bit3 low_pwr1 bit2 low_pwr2 bit1 hot bi0 cold
	pm2301_reg_write(pm2301, 0x54, 0x1A); // temp detail
	pm2301_reg_write(pm2301, 0x55, 0x3F); // ac adaptor vol abnormal
	schedule_delayed_work(&pm2301_bat_info.polling_work, msecs_to_jiffies(900));
	system_suspended = 0;
	return;
}
#else
#define pm2301_bat_suspend NULL
#define pm2301_bat_resume NULL
#endif /* CONFIG_PM */



int charger_ac_online(void) {
	return (pm2301_bat_info.bat_info.charging_source==CHARGER_AC);
}
EXPORT_SYMBOL(charger_ac_online);

static void pm2301_bat_start_charging(void)
{
	  int val=0;
	  
	  if(system_suspended == 1)	{
  		pm2301_reg_write(pm2301, 0x50, 0x0D); // bat_low edge info[not detect], bat disconnect info
      pm2301_reg_write(pm2301, 0x51, 0x03); // mask plug/unplug ir, only detect pwr1
	    pm2301_reg_write(pm2301, 0x52, 0x08); // charging watchdog irq
	    pm2301_reg_write(pm2301, 0x53, 0xB4); // bit7 cvphase bit6 bat_full bit5 resume bit4 charging_on bit3 low_pwr1 bit2 low_pwr2 bit1 hot bi0 cold
	    pm2301_reg_write(pm2301, 0x54, 0x1A); // temp detail
	    pm2301_reg_write(pm2301, 0x55, 0x3F); // ac adaptor vol abnormal
	  }
	  	
		pm2301_reg_write(pm2301, 0x0, 0x00); // chresume do_nothing
		pm2301_reg_write(pm2301, 0x2, 0x3F); // watchdog for cc, cv charging
		pm2301_reg_write(pm2301, 0x3, 0x07); // watchdog for pre-charging.
		pm2301_reg_write(pm2301, 0x4, 0x00); // charging mode related, not time_out
		pm2301_reg_write(pm2301, 0x5, 0xFF); // charging current programming
		/*0x6 relates to resume vol and precharge level, default uses 2.9V and 3.8V*/
		pm2301_reg_write(pm2301, 0x7, 0x1c); //4// 4.2V

		//pm2301_reg_write(pm2301, 0x70, 0xFF); // Watchdog
		pm2301_reg_write(pm2301, 0x26, 0x01); // FET setting
		pm2301_reg_write(pm2301, 0x23, 0x0F); // bat low level setting
		pm2301_reg_write(pm2301, 0x13, 0x00); // power optimization, comarator setting
		pm2301_reg_write(pm2301, 0x17, 0x00); // power optimization and comarator setting		
		   
	
		pm2301_reg_write(pm2301, 0x26, 0x01); // enable charging   	   
		pm2301_reg_write(pm2301, 0x01, 0x04); // enable charging

		pm2301_bat_info.bat_info.charging_source = CHARGER_AC;
		pm2301_bat_info.bat_info.charging_enabled = 1;
		Pm2301Dbg("DC Cable Plugged, Start Charging Automatically.\n");	
		
 		val = pm2301_reg_read(pm2301, 0xB);
		Pm2301Dbg("[pm2301] reg 0x0B is 0x%x\n", val);

	
	return;
}

static void pm2301_bat_stop_charging(void) {
		pm2301_reg_write(pm2301, 0x1, 0x0);
		pm2301_bat_info.bat_info.charging_source = CHARGER_BATTERY;
		pm2301_bat_info.bat_info.charging_enabled = 0;
		Pm2301Dbg("pm2301 stop charging\n");

}

static void pm2301_charger_irq_work(struct work_struct *work) {	
	int val;
	
	/*
	   check interrupt source here, this added the reading process of source register,
	   but it can reduce the processing time in the isr function.
	*/

	val = pm2301_reg_read(pm2301, 0x60);
	if(((val >> 1) & 0x1) == 1) {
		pm2301_bat_info.present = 0;
		pm2301_bat_stop_charging();
		Pm2301Dbg("pm2301 battery is absent\n");
	} else {
		pm2301_bat_info.present = 1;
	}
	
	val = pm2301_reg_read(pm2301, 0x61);
	if(val & 0x0C) {
					pm2301_bat_start_charging();		
	}

	if((val & 0x0C) == 0) {
		Pm2301Dbg("pm2301 charger unplugged\n");
		pm2301_bat_stop_charging();
	}

	val = pm2301_reg_read(pm2301, 0x62);
	if((val & 0x07)) {
		Pm2301Dbg("pm2301 charger timeout, battery may damaged\n");
		pm2301_bat_info.bat_info.batt_health = POWER_SUPPLY_HEALTH_DEAD;
		pm2301_bat_info.bat_info.charging_error = 1;	
		pm2301_bat_stop_charging();
	}

	val = pm2301_reg_read(pm2301, 0x63);
	if ((val & 0x40) == 0x40) {
		Pm2301Dbg("pm2301 EOC is reached\n");
		pm2301_reg_write(pm2301, 0x1, 0x0);
		pm2301_bat_info.bat_info.charging_enabled = 0;
		pm2301_bat_info.bat_info.batt_is_full = 1;
		pm2301_bat_info.bat_info.level = 100;		
	}	
	
	if ((val & 0x8) == 0x8) {		
		Pm2301Dbg("pm2301 over-voltage in pwr detected\n");
		pm2301_bat_stop_charging();
		pm2301_bat_info.bat_info.charging_error = 1;		
	}
	
	if ((val & 0x2) == 0x2) {
		Pm2301Dbg("pm2301 battery temerature too high\n");
		pm2301_bat_stop_charging();
		pm2301_bat_info.bat_info.batt_health = POWER_SUPPLY_HEALTH_OVERHEAT;
		pm2301_bat_info.bat_info.charging_error = 1;	
	}
	
	if ((val & 0x1) == 0x1) {
		Pm2301Dbg("pm2301 battery temerature too cold\n");
		pm2301_bat_stop_charging();		
		pm2301_bat_info.bat_info.batt_health = POWER_SUPPLY_HEALTH_COLD;
		pm2301_bat_info.bat_info.charging_error = 1;	
	}

	val = pm2301_reg_read(pm2301, 0x64);
	if((val & 0x4)) {
		Pm2301Dbg("pm2301 die temperature reached shutdown level\n");
		pm2301_bat_stop_charging();
		pm2301_bat_info.bat_info.charging_error = 1;			
	}

	pm2301_cable_status_update();
	power_supply_changed(&pm2301_power_supplies[CHARGER_BATTERY]);
	
}


static irqreturn_t pm2301_irq(int irq, void *data)
{
	int i, val;
	
	printk("$$$ pm2301 irq happend\n");
  if(pm2301_battery_initial == 1) {
	  for(i = 0; i < 6; i++) {
	  	val = pm2301_reg_read(pm2301, 0x40+i);
	  	Pm2301Dbg("pm2301 reg 0x4%d is 0x%x\n", i, val);
	  } 	
  	schedule_delayed_work(&pm2301_bat_info.charger_work, msecs_to_jiffies(PM2301_CHARGER_SCHEDULE_INTERVAL));
	}

	return IRQ_HANDLED;
}

static int __devinit pm2301_bat_probe(struct platform_device *pdev)
{
	int i, val;
	int ret = 0;

  int soc = pm2301_bat_get_soc();
  int vol = pm2301_bat_get_vol();
  
  Pm2301Dbg("[pm2301][pm2301_bat_probe] level=%d vol=%d\n",soc,vol);

  if(soc==0 || vol < 2400)  {
	   	Pm2301Dbg("Battery is not present.\n");
    	pm2301_bat_info.present = 0;		
    	goto __end__;
  }
  
	if (pm2301==NULL) {
		ret = -ENODEV;
		goto __end__;
	}

	// CHG_LPN
	ret = gpio_request(EXYNOS5410_GPY6(0), "GPY6(0)");
	if (ret) {
		Pm2301Dbg(KERN_ERR "[ERR] Failed to request GPX6(0).\n");
		goto __end__;
	}
	
	s3c_gpio_setpull(EXYNOS5410_GPY6(0), S3C_GPIO_PULL_NONE);
	gpio_direction_output(EXYNOS5410_GPY6(0), 1);
	gpio_free(EXYNOS5410_GPY6(0));

	val = pm2301_reg_read(pm2301, 0x0C); //device version checking
	if (val != 0x12) {
		printk("[PM2301] pm2301_bat_probe I2C Fail\n");
		Pm2301Dbg("[ERR] Failed to detect PM2301 Battery Charger(0x%X).\n", val);
		ret = -ENODEV;
		goto __end__;
	}
	Pm2301Dbg("[PM2301] pm2301_bat_probe I2C Success\n");
	
	pm2301_bat_info.present = 1;
	pm2301_bat_info.bat_info.batt_id = 0;
	pm2301_bat_info.bat_info.batt_vol = 0;
	pm2301_bat_info.bat_info.batt_temp = 20;
	pm2301_bat_info.bat_info.batt_temp_adc = 20;
	pm2301_bat_info.bat_info.batt_temp_adc_cal = 20;
	pm2301_bat_info.bat_info.level = 10;
	pm2301_bat_info.bat_info.batt_health = POWER_SUPPLY_HEALTH_GOOD;
	pm2301_bat_info.bat_info.charging_source = CHARGER_BATTERY;
	pm2301_bat_info.bat_info.charging_enabled = 0;
	pm2301_bat_info.bat_info.batt_is_full = 0;
	pm2301_bat_info.bat_info.charging_error = 0;
	
	s3c_gpio_setpull(EXYNOS5410_GPX1(2), S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(EXYNOS5410_GPX1(2), S3C_GPIO_INPUT);

	/*clear irqs*/
	pm2301_reg_read(pm2301, 0x40);
	pm2301_reg_read(pm2301, 0x41);
	pm2301_reg_read(pm2301, 0x42);
	pm2301_reg_read(pm2301, 0x43);
	pm2301_reg_read(pm2301, 0x44);
	pm2301_reg_read(pm2301, 0x45);


	pm2301_reg_write(pm2301, 0x50, 0x0D); // bat_low edge info[not detect], bat disconnect info
	pm2301_reg_write(pm2301, 0x51, 0x03); // mask plug/unplug ir, only detect pwr1
	pm2301_reg_write(pm2301, 0x52, 0x08); // charging watchdog irq
	pm2301_reg_write(pm2301, 0x53, 0xB4); // bit7 cvphase bit6 bat_full bit5 resume bit4 charging_on bit3 low_pwr1 bit2 low_pwr2 bit1 hot bi0 cold
	pm2301_reg_write(pm2301, 0x54, 0x1A); // temp detail
	pm2301_reg_write(pm2301, 0x55, 0x3F); // ac adaptor vol abnormal

	if((pm2301_reg_read(pm2301, 0x61)&0xC)==0xC) {	

		/*checking whether battery is present*/
		if ((pm2301_reg_read(pm2301, 0x60) & 0x2) == 0x2) {
			
			Pm2301Dbg("Battery is not present.\n");
			pm2301_bat_info.present = 0;			
			ret = -ENODEV;
			goto __end__;
		}
		pm2301_bat_start_charging();
	}
	
	// CHG_IRQ
	s3c_gpio_cfgpin(EXYNOS5410_GPX1(2), S3C_GPIO_SPECIAL(0xF));
	s3c_gpio_setpull(EXYNOS5410_GPX1(2), S3C_GPIO_PULL_NONE);
	irq_set_irq_wake(IRQ_EINT(10), 1);
	ret = request_irq(IRQ_EINT(10),
					pm2301_irq,
					IRQ_TYPE_LEVEL_LOW, // low level trigging
					"PM2301_IRQ",
					NULL);
	if (ret) {
		Pm2301Dbg("[ERR] Failed to request IRQ for CHG_IRQ for PM2301.\n");
		goto __end__;
	}
	
	dev = &pdev->dev;

	/* init power supplier framework */
	for (i = 0; i < ARRAY_SIZE(pm2301_power_supplies); i++) {
		ret = power_supply_register(&pdev->dev,  &pm2301_power_supplies[i]);
		if (ret) {
			dev_err(dev, "Failed to register power supply %d,%d\n", i, ret);
			goto __end__;
		}
	}

	/* create sec detail attributes */
	pm2301_bat_create_attrs(pm2301_power_supplies[CHARGER_BATTERY].dev);

	INIT_DELAYED_WORK(&pm2301_bat_info.charger_work, pm2301_charger_irq_work);	
	INIT_DELAYED_WORK_DEFERRABLE(&pm2301_bat_info.polling_work, pm2301_polling_work);		

	pm2301_battery_initial = 1;

	pm2301_cable_status_update( );
	schedule_delayed_work(&pm2301_bat_info.polling_work, HZ);

	Pm2301Dbg("[PM2301] Battery Charger Driver Registered Successfully.\n");

__end__:
	return ret;
}

static int __devexit pm2301_bat_remove(struct platform_device *pdev)
{
	int i;

	cancel_delayed_work(&pm2301_bat_info.charger_work);	
	cancel_delayed_work(&pm2301_bat_info.polling_work);

	for (i = 0; i < ARRAY_SIZE(pm2301_power_supplies); i++)
		power_supply_unregister(&pm2301_power_supplies[i]);

	return 0;
}

static const struct dev_pm_ops pm2301_bat_pm_ops = {
	.prepare	= pm2301_bat_suspend,
	.complete	= pm2301_bat_resume,
};

static struct platform_driver pm2301_bat_driver = {
	.driver = {
		.name	= "pm2301-battery",
		.owner	= THIS_MODULE,
		.pm	= &pm2301_bat_pm_ops,
	},
	.probe		= pm2301_bat_probe,
	.remove		= __devexit_p(pm2301_bat_remove),
};

static int pm2301_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	pm2301 = client;

	wake_lock_init(&vbus_wake_lock, WAKE_LOCK_SUSPEND, "vbus_present");

	return platform_driver_register(&pm2301_bat_driver);
}

static int pm2301_remove(struct i2c_client *client)
{
	platform_driver_unregister(&pm2301_bat_driver);
	wake_lock_destroy(&vbus_wake_lock);
	return 0;
}

static const struct i2c_device_id pm2301_id[] = {
	{ "pm2301", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pm2301_id);

static struct i2c_driver pm2301_i2c_driver = {
	.driver = {
		.name	= "pm2301",
		.owner	= THIS_MODULE,
	},
	.probe		= pm2301_probe,
	.remove		= __devexit_p(pm2301_remove),
	.id_table 	= pm2301_id,
};

static int __init pm2301_bat_init(void)
{
	return i2c_add_driver(&pm2301_i2c_driver);
}

static void __exit pm2301_bat_exit(void)
{
	i2c_del_driver(&pm2301_i2c_driver);
}

module_init(pm2301_bat_init);
module_exit(pm2301_bat_exit);

MODULE_DESCRIPTION("SSCR PM2301 Battery Driver for TE4 H/W Board");
MODULE_LICENSE("GPL");
