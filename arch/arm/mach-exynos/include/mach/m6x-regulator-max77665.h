#ifndef __M6X_MFD_MAX77665_H_H_
#define __M6X_MFD_MAX77665_H_H_

#ifdef CONFIG_MFD_MAX77665
static struct regulator_consumer_supply safeout1_supply[] = {
	REGULATOR_SUPPLY("safeout1", NULL),
};

static struct regulator_consumer_supply safeout2_supply[] = {
	REGULATOR_SUPPLY("safeout2", NULL),
};

static struct regulator_consumer_supply charger_supply[] = {
	REGULATOR_SUPPLY("vinchg1", "max77665-charger"),
};

static struct regulator_consumer_supply flash_led_supply[] = {
	REGULATOR_SUPPLY("flash_led", NULL),
};

static struct regulator_consumer_supply torch_led_supply[] = {
	REGULATOR_SUPPLY("torch_led", NULL),
};

static struct regulator_consumer_supply reverse_supply[] = {
	REGULATOR_SUPPLY("reverse", NULL),
	REGULATOR_SUPPLY("vbus_reverse", NULL),
};

static struct regulator_consumer_supply battery_supply[] = {
	REGULATOR_SUPPLY("battery", NULL),
};

static struct regulator_init_data safeout1_init_data = {
	.constraints	= {
		.name		= "safeout1 range",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.boot_on	= true,
		.state_mem	= {
			.enabled 	= true,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(safeout1_supply),
	.consumer_supplies	= safeout1_supply,
};

static struct regulator_init_data safeout2_init_data = {
	.constraints	= {
		.name		= "safeout2 range",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled 	= true,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(safeout2_supply),
	.consumer_supplies	= safeout2_supply,
};

static struct regulator_init_data charger_init_data = {
	.constraints	= {
		.name		= "CHARGER",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS |
					      REGULATOR_CHANGE_CURRENT,
		.min_uA		= 60000,
		.max_uA		= 2580000,
		.state_mem	= {
			.enabled	= true,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(charger_supply),
	.consumer_supplies	= charger_supply,
};

static struct regulator_init_data flash_led_init_data = {
	.constraints	= {
		.name		= "FLASH LED",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS |
					      REGULATOR_CHANGE_CURRENT,
		.boot_on	= false,
		.min_uA		= 15625,
		.max_uA		= 1000000,
		.state_mem	= {
			.disabled	= true,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(flash_led_supply),
	.consumer_supplies	= flash_led_supply,
};

static struct regulator_init_data torch_led_init_data = {
	.constraints	= {
		.name		= "TORCH LED",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS |
					      REGULATOR_CHANGE_CURRENT,
		.boot_on	= false,
		.min_uA		= 15625,
		.max_uA		= 250000,
		.state_mem	= {
			.disabled	= true,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(torch_led_supply),
	.consumer_supplies	= torch_led_supply,
};

static struct regulator_init_data reverse_init_data = {
	.constraints	= {
		.name		= "REVERSE",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.boot_on	= false,
		.state_mem	= {
			.disabled	= true,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(reverse_supply),
	.consumer_supplies	= reverse_supply,
};

static struct regulator_init_data battery_init_data = {
	.constraints	= {
		.name		= "BATTERY",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS |
					      REGULATOR_CHANGE_CURRENT,
		.boot_on	= true,
		.min_uA		= 0,
		.max_uA		= 2100000,
		.state_mem	= {
			.enabled	= true,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(battery_supply),
	.consumer_supplies	= battery_supply,
};

static struct max77665_regulator_data max77665_regulators[] = {
	{MAX77665_ESAFEOUT1, &safeout1_init_data,},
	{MAX77665_ESAFEOUT2, &safeout2_init_data,},
	{MAX77665_CHARGER, &charger_init_data,},
	{MAX77665_FLASH_LED, &flash_led_init_data,},
	{MAX77665_TORCH_LED, &torch_led_init_data,},
	{MAX77665_REVERSE, &reverse_init_data,},
	{MAX77665_BATTERY, &battery_init_data,},
};

#ifdef CONFIG_MOTOR_DRV_MAX77665
static struct max77665_haptic_platform_data max77665_haptic_pdata = {
	.pwm_channel_id = 0,
	.pwm_period = 38022,
	.pwm_duty = 41,
	.type = MAX77665_HAPTIC_LRA,
	.mode = MAX77665_EXTERNAL_MODE,
	.pwm_divisor = MAX77665_PWM_DIVISOR_128
};
#endif

static struct max77665_platform_data __initdata m6x_max77665_info = {
	.irq_base = IRQ_BOARD_START + 20,
	.wakeup = true,
	.name = "max77665-charger",

	/* charger */
	.supply = "vinchg1",
	.fast_charge_timer = MAX77665_FCHGTIME_8H,
	.charging_restart_thresold = MAX77665_CHG_RSTRT_150MV,
	.top_off_current_thresold = MAX77665_CHG_TO_ITH_100MA,
	.top_off_timer = MAX77665_CHG_TO_TIME_20MIN,
	.charger_termination_voltage = MAX77665_CHG_CV_PRM_4350MV,
	.fast_charge_current = 1166,		/* 0mA ~ 2100mA */
	.chgin_ilim_usb = 470,			/* 60mA ~ 2580mA */
	.chgin_ilim_ac = 1200,
	.fastcharging_ilim_usb = 1000,

	/* regulator */
	.regulators = max77665_regulators,
	.num_regulators = ARRAY_SIZE(max77665_regulators),

	/* haptic */
#ifdef CONFIG_MOTOR_DRV_MAX77665
	.haptic_pdata = &max77665_haptic_pdata,
#endif
};

static void __init max77665_rt_init_res(void) {
	m6x_max77665_info.vbusdetect_gpio = MEIZU_VBUS_DET_IRQ;
}
#endif /*CONFIG_MFD_MAX77665*/

#endif /*__M6X_MFD_MAX77665_H_H_*/
