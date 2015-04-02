#ifndef __MEIZU_M65_V2_REGULATOR_H_H__
#define __MEIZU_M65_V2_REGULATOR_H_H__

#ifdef CONFIG_REGULATOR_MAX77802

static struct regulator_consumer_supply max77802_buck1_consumer =
	REGULATOR_SUPPLY("vdd_mif", NULL);

static struct regulator_consumer_supply max77802_buck2_consumer =
	REGULATOR_SUPPLY("vdd_arm", NULL);

static struct regulator_consumer_supply max77802_buck3_consumer =
	REGULATOR_SUPPLY("vdd_int", NULL);

static struct regulator_consumer_supply max77802_buck4_consumer =
	REGULATOR_SUPPLY("vdd_g3d", NULL);

static struct regulator_consumer_supply max77802_buck6_consumer =
	REGULATOR_SUPPLY("vdd_kfc", NULL);

static struct regulator_consumer_supply max77802_buck8_consumer =
	REGULATOR_SUPPLY("dvdd285fnand", "dw_mmc.0");

static struct regulator_consumer_supply max77802_buck9_consumer =
	REGULATOR_SUPPLY("dvdd20", NULL);

static struct regulator_consumer_supply max77802_buck10_consumer =
	REGULATOR_SUPPLY("vdd_gps", "bcm4752");

static struct regulator_consumer_supply max77802_ldo1_consumer =
	REGULATOR_SUPPLY("vdd10_alive", NULL);

static struct regulator_consumer_supply max77802_ldo2_consumer =
	REGULATOR_SUPPLY("vdd12_memoff2", NULL);

static struct regulator_consumer_supply max77802_ldo3_consumer =
	REGULATOR_SUPPLY("dvdd18", NULL);

static struct regulator_consumer_supply max77802_ldo4_consumer =
	REGULATOR_SUPPLY("vdd28_mmc", NULL);

static struct regulator_consumer_supply max77802_ldo5_consumer =
	REGULATOR_SUPPLY("vdd18_hsic", NULL);

static struct regulator_consumer_supply max77802_ldo6_consumer =
	REGULATOR_SUPPLY("vdd18_pll", NULL);

static struct regulator_consumer_supply max77802_ldo7_consumer[] = {
	REGULATOR_SUPPLY("vdd18_ispa", NULL),
};

static struct regulator_consumer_supply max77802_ldo8_consumer[] = {
	REGULATOR_SUPPLY("vdd10_mipi", NULL),
};

static struct regulator_consumer_supply max77802_ldo9_consumer[] = {
	REGULATOR_SUPPLY("vdd18_au", NULL),
	REGULATOR_SUPPLY("AVDD", NULL),
	REGULATOR_SUPPLY("LDOVDD", NULL),
	REGULATOR_SUPPLY("DBVDD", NULL),
	REGULATOR_SUPPLY("DBVDD1", NULL),
	REGULATOR_SUPPLY("DBVDD2", NULL),
	REGULATOR_SUPPLY("DBVDD3", NULL),
	REGULATOR_SUPPLY("CPVDD", NULL),
};

static struct regulator_consumer_supply max77802_ldo10_consumer[] = {
	REGULATOR_SUPPLY("vdd18_mipi", NULL),
};

static struct regulator_consumer_supply max77802_ldo11_consumer =
	REGULATOR_SUPPLY("dvdd18_mmc01", NULL);

static struct regulator_consumer_supply max77802_ldo12_consumer[] = {
	REGULATOR_SUPPLY("vdd30_usb", NULL),
};

static struct regulator_consumer_supply max77802_ldo13_consumer =
	REGULATOR_SUPPLY("vdd18_pre", NULL);

static struct regulator_consumer_supply max77802_ldo14_consumer =
	REGULATOR_SUPPLY("vdd18_abb", NULL);

static struct regulator_consumer_supply max77802_ldo15_consumer[] = {
	REGULATOR_SUPPLY("vdd10_usb", NULL),
};

//NO LDO16

static struct regulator_consumer_supply max77802_ldo17_consumer =
	REGULATOR_SUPPLY("vdd12_ispa", NULL);

static struct regulator_consumer_supply max77802_ldo18_consumer =
	REGULATOR_SUPPLY("vdd18_lcd", NULL);

static struct regulator_consumer_supply max77802_ldo19_consumer =
	REGULATOR_SUPPLY("vdd18_fcamera", NULL);

static struct regulator_consumer_supply max77802_ldo20_consumer =
	REGULATOR_SUPPLY("vdd18_inandif", NULL);

static struct regulator_consumer_supply max77802_ldo21_consumer =
	REGULATOR_SUPPLY("vdd18_mcu", NULL);
//no LDO22
	
static struct regulator_consumer_supply max77802_ldo23_consumer =
	REGULATOR_SUPPLY("vcc28_af", NULL);
		
static struct regulator_consumer_supply max77802_ldo24_consumer =
	REGULATOR_SUPPLY("vcc28_tou", NULL);

static struct regulator_consumer_supply max77802_ldo25_consumer =
	REGULATOR_SUPPLY("vdd28_ir", NULL);

static struct regulator_consumer_supply max77802_ldo26_consumer[] = {
	REGULATOR_SUPPLY("vdd28_sen", NULL),
	REGULATOR_SUPPLY("vdd28_acc", NULL),    //accelerometer
	REGULATOR_SUPPLY("vdd28_gyr", NULL),    //gyroscope
};	

static struct regulator_consumer_supply max77802_ldo27_consumer =
	REGULATOR_SUPPLY("vdd12_fcamera", NULL);

static struct regulator_consumer_supply max77802_ldo28_consumer =
	REGULATOR_SUPPLY("vdd18_tou", NULL);

static struct regulator_consumer_supply max77802_ldo29_consumer =
	REGULATOR_SUPPLY("vdd18_8msen", NULL);

static struct regulator_consumer_supply max77802_ldo30_consumer =
	REGULATOR_SUPPLY("vdd12_memoff", NULL);

//no ldo31
static struct regulator_consumer_supply max77802_ldo32_consumer =
	REGULATOR_SUPPLY("vdd28_micbias", NULL);

static struct regulator_consumer_supply max77802_ldo33_consumer =
	REGULATOR_SUPPLY("vcc28_8msen", NULL);

static struct regulator_consumer_supply max77802_ldo34_consumer =
	REGULATOR_SUPPLY("vdd28_fcamera", NULL);

static struct regulator_consumer_supply max77802_ldo35_consumer =
	REGULATOR_SUPPLY("vdd12_8msen", NULL);

static struct regulator_consumer_supply max77802_en32khap =
	REGULATOR_SUPPLY("ap_32khz", NULL);

static struct regulator_consumer_supply max77802_en32khcp =
	REGULATOR_SUPPLY("gps_32.768k", "bcm4752");

static struct regulator_init_data max77802_buck1_data = {
	.constraints	= {
		.name		= "vdd_mif range",
		.min_uV		= 800000,
		.max_uV		= 1200000,
		.uV_offset      =   25000,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.always_on	= 1,
		.boot_on	= 1,
		.state_mem	= {
			.uV		= 1100000,
			.mode		= REGULATOR_MODE_NORMAL,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_buck1_consumer,
};

static struct regulator_init_data max77802_buck2_data = {
	.constraints	= {
		.name		= "vdd_arm range",
		.min_uV		=  900000,
		.max_uV		= 1300000,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.always_on 	= 1,
		.boot_on 	= 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_buck2_consumer,
};

static struct regulator_init_data max77802_buck3_data = {
	.constraints	= {
		.name		= "vdd_int range",
		.min_uV		=  800000,
		.max_uV		= 1250000,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.always_on 	= 1,
		.boot_on 	= 1,
		.state_mem	= {
			.uV		= 1200000,
			.mode		= REGULATOR_MODE_NORMAL,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_buck3_consumer,
};

static struct regulator_init_data max77802_buck4_data = {
	.constraints	= {
		.name		= "vdd_g3d range",
		.min_uV		=  850000,
		.max_uV		= 1300000,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.always_on 	= 1,
		.boot_on 	= 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_buck4_consumer,
};

static struct regulator_init_data max77802_buck6_data = {
	.constraints	= {
		.name		= "vdd_kfc range",
		.min_uV		=  900000,
		.max_uV		= 1400000,
		.uV_offset	=   25000,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.always_on 	= 1,
		.boot_on 	= 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_buck6_consumer,
};

static struct regulator_init_data max77802_buck8_data = {
	.constraints	= {
		.name		= "DVDD_285FNAND",
		.min_uV		= 2850000,
		.max_uV		= 2850000,
		.uV_offset	=  150000, /*offset 150mV for sandisk*/
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.always_on 	= 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_buck8_consumer,
};

static struct regulator_init_data max77802_buck9_data = {
	.constraints	= {
		.name		= "DVDD20",
		.min_uV		= 2000000,
		.max_uV		= 2000000,
		.apply_uV	= 1,
		.always_on 	= 1,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_buck9_consumer,
};

static struct regulator_init_data max77802_buck10_data = {
	.constraints	= {
		.name		= "DVDD_3V2",
		.min_uV		= 3200000,
		.max_uV		= 3200000,
		.always_on	= 1,
		.apply_uV	= 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_buck10_consumer,
};

static struct regulator_init_data max77802_ldo1_data = {
	.constraints	= {
		.name		= "VDD10_ALIVE",
		.min_uV		= 1000000,
		.max_uV		= 1000000,
		.apply_uV	= 1,
		.always_on	= 1,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_ldo1_consumer,
};

static struct regulator_init_data max77802_ldo2_data = {
	.constraints	= {
		.name		= "VDD12_MEMOFF2",
		.min_uV		= 1200000,
		.max_uV		= 1200000,
		.apply_uV	= 1,
		.always_on	= 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_ldo2_consumer,
};

static struct regulator_init_data max77802_ldo3_data = {
	.constraints	= {
		.name		= "DVDD_1V8",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.always_on	= 1,
		.boot_on	= 1,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_ldo3_consumer,
};

static struct regulator_init_data max77802_ldo4_data = {
	.constraints	= {
		.name		= "VDD28_MMC",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_ldo4_consumer,
};

static struct regulator_init_data max77802_ldo5_data = {
	.constraints	= {
		.name		= "VDD18_HSIC",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_ldo5_consumer,
};

static struct regulator_init_data max77802_ldo6_data = {
	.constraints	= {
		.name		= "VDD18_PLL",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.always_on	= 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_ldo6_consumer,
};

static struct regulator_init_data max77802_ldo7_data = {
	.constraints	= {
		.name		= "VDD18_ISPA",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= max77802_ldo7_consumer,
};

static struct regulator_init_data max77802_ldo8_data = {
	.constraints	= {
		.name		= "VDD10_MIPI",
		.min_uV		= 1000000,
		.max_uV		= 1000000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= max77802_ldo8_consumer,
};

static struct regulator_init_data max77802_ldo9_data = {
	.constraints	= {
		.name		= "VDD18_AU",
		.min_uV		= 1700000,
		.max_uV		= 1700000,
		.uV_offset      =   50000,
		.apply_uV	= 1,
		.always_on	= 1,
		.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(max77802_ldo9_consumer),
	.consumer_supplies	= max77802_ldo9_consumer,
};

static struct regulator_init_data max77802_ldo10_data = {
	.constraints	= {
		.name		= "VDD18_MIPI",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= max77802_ldo10_consumer,
};

static struct regulator_init_data max77802_ldo11_data = {
	.constraints	= {
		.name		= "DVDD18_MMC01",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.always_on	= 1,
		.boot_on	= 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_ldo11_consumer,
};

static struct regulator_init_data max77802_ldo12_data = {
	.constraints	= {
		.name		= "VDD30_USB",
		.min_uV		= 3000000,
		.max_uV		= 3000000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= max77802_ldo12_consumer,
};

static struct regulator_init_data max77802_ldo13_data = {
	.constraints	= {
		.name		= "VDD18_PRE",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV       = 1,
		.always_on	= 1,
		.boot_on	= 1,
		.state_mem	= {
			.enabled 	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_ldo13_consumer,
};

static struct regulator_init_data max77802_ldo14_data = {
	.constraints	= {
		.name		= "VDD18_ABB",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV       = 1,
		.always_on	= 1,
		.boot_on	= 1,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_ldo14_consumer,
};

static struct regulator_init_data max77802_ldo15_data = {
	.constraints	= {
		.name		= "VDD10_USB",
		.min_uV		= 1000000,
		.max_uV		= 1000000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= max77802_ldo15_consumer,
};

static struct regulator_init_data max77802_ldo17_data = {
	.constraints	= {
		.name		= "VDD12_ISPA",
		.min_uV		= 1200000,
		.max_uV		= 1200000,
		.uV_offset	=  100000, /*offset 100mV*/
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_ldo17_consumer,
};

static struct regulator_init_data max77802_ldo18_data = {
	.constraints	= {
		.name		= "VDD18_LCD",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.boot_on	= 0,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_ldo18_consumer,
};

static struct regulator_init_data max77802_ldo19_data = {
	.constraints	= {
		.name		= "VDD18_FCAMERA",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_ldo19_consumer,
};


static struct regulator_init_data max77802_ldo20_data = {
	.constraints	= {
		.name		= "VDD18_INANDIF",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.boot_on	= 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_ldo20_consumer,
};


#ifdef	CONFIG_MFD_MX_HUB
/*m065 MCU need this*/
static struct regulator_init_data max77802_ldo21_data = {
	.constraints	= {
		.name           = "VDD18_MCU",
		.min_uV         = 1800000,
		.max_uV         = 2500000,
		.boot_on		= 0,
		//.apply_uV	= 1,
		//.always_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS|REGULATOR_CHANGE_VOLTAGE,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_ldo21_consumer,
};
#else
static struct regulator_init_data max77802_ldo21_data = {
	.constraints	= {
		.name           = "VDD18_MCU",
		.min_uV		= 1900000,
		.max_uV		= 1900000,
		.apply_uV	= 1,
		.boot_on	= 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_ldo21_consumer,
};
#endif //CONFIG_MFD_MX_HUB

static struct regulator_init_data max77802_ldo23_data = {
	.constraints	= {
		.name		= "VCC28_AF",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_ldo23_consumer,
};

static struct regulator_init_data max77802_ldo24_data = {
	.constraints	= {
		.name		= "VDD28_TOU",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.apply_uV	= 1,
		//.always_on	= 1,
		//.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_ldo24_consumer,
};

static struct regulator_init_data max77802_ldo25_data = {
	.constraints	= {
		.name		= "VDD28_IR",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.apply_uV	= 1,
		.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_ldo25_consumer,
};

static struct regulator_init_data max77802_ldo26_data = {
	.constraints	= {
		.name		= "VDD28_SEN",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.apply_uV	= 1,
		.boot_on    = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(max77802_ldo26_consumer),
	.consumer_supplies	= max77802_ldo26_consumer,
};

static struct regulator_init_data max77802_ldo27_data = {
	.constraints	= {
		.name		= "VDD12_FCAMERA",
		.min_uV		= 1200000,
		.max_uV		= 1200000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_ldo27_consumer,
};

static struct regulator_init_data max77802_ldo28_data = {
	.constraints	= {
		.name		= "VDD18_TOU",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		//.always_on	= 1,
		//.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_ldo28_consumer,
};

static struct regulator_init_data max77802_ldo29_data = {
	.constraints	= {
		.name		= "VDD18_8MSEN",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_ldo29_consumer,
};

static struct regulator_init_data max77802_ldo30_data = {
	.constraints	= {
		.name		= "VDD12_MEMOFF",
		.min_uV		= 1200000,
		.max_uV		= 1200000,
		.apply_uV	= 1,
		.always_on	= 1,
		.boot_on 	= 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_ldo30_consumer,
};

//NO ldo31
static struct regulator_init_data max77802_ldo32_data = {
	.constraints	= {
		.name		= "VDD28_MICBIAS",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_ldo32_consumer,
};

static struct regulator_init_data max77802_ldo33_data = {
	.constraints	= {
		.name		= "VCC28_8MSEN",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_ldo33_consumer,
};

static struct regulator_init_data max77802_ldo34_data = {
	.constraints	= {
		.name		= "VDD28_FCAMERA",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_ldo34_consumer,
};

static struct regulator_init_data max77802_ldo35_data = {
	.constraints	= {
		.name		= "VDD12_8MSEN",
		.min_uV		= 1200000,
		.max_uV		= 1200000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled       = 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_ldo35_consumer,
};

static struct regulator_init_data max77802_en32khap_data = {
	.constraints	= {
		.name		= "PMIC_AP32K",
		.always_on	= 1,
		.state_mem	= {
			.enabled       = 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_en32khap,
};

static struct regulator_init_data max77802_en32khcp_data = {
	.constraints	= {
		.name		= "32KHZ_GPS",
		.always_on	= 1,
		.state_mem	= {
			.enabled       = 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max77802_en32khcp,
};

static struct max77802_regulator_data max77802_regulators[] = {
	{MAX77802_BUCK1, &max77802_buck1_data},
	{MAX77802_BUCK2, &max77802_buck2_data},
	{MAX77802_BUCK3, &max77802_buck3_data},
	{MAX77802_BUCK4, &max77802_buck4_data},
	{MAX77802_BUCK6, &max77802_buck6_data},
	{MAX77802_BUCK8, &max77802_buck8_data},
	{MAX77802_BUCK9, &max77802_buck9_data},
	{MAX77802_BUCK10, &max77802_buck10_data},
	{MAX77802_LDO1, &max77802_ldo1_data},
	{MAX77802_LDO2, &max77802_ldo2_data},
	{MAX77802_LDO3, &max77802_ldo3_data},
	{MAX77802_LDO4, &max77802_ldo4_data},
	{MAX77802_LDO5, &max77802_ldo5_data},
	{MAX77802_LDO6, &max77802_ldo6_data},
	{MAX77802_LDO7, &max77802_ldo7_data},
	{MAX77802_LDO8, &max77802_ldo8_data},
	{MAX77802_LDO9, &max77802_ldo9_data},
	{MAX77802_LDO10, &max77802_ldo10_data},
	{MAX77802_LDO11, &max77802_ldo11_data},
	{MAX77802_LDO12, &max77802_ldo12_data},
	{MAX77802_LDO13, &max77802_ldo13_data},
	{MAX77802_LDO14, &max77802_ldo14_data},
	{MAX77802_LDO15, &max77802_ldo15_data},
	{MAX77802_LDO17, &max77802_ldo17_data},
	{MAX77802_LDO18, &max77802_ldo18_data},
	{MAX77802_LDO19, &max77802_ldo19_data},
//	{MAX77802_LDO20, &max77802_ldo20_data},
	{MAX77802_LDO21, &max77802_ldo21_data},
	{MAX77802_LDO23, &max77802_ldo23_data},
	{MAX77802_LDO24, &max77802_ldo24_data},
	{MAX77802_LDO25, &max77802_ldo25_data},
	{MAX77802_LDO26, &max77802_ldo26_data},
	{MAX77802_LDO27, &max77802_ldo27_data},
	{MAX77802_LDO28, &max77802_ldo28_data},
	{MAX77802_LDO29, &max77802_ldo29_data},
	{MAX77802_LDO30, &max77802_ldo30_data},
	{MAX77802_LDO32, &max77802_ldo32_data},
	{MAX77802_LDO33, &max77802_ldo33_data},
	{MAX77802_LDO34, &max77802_ldo34_data},
	{MAX77802_LDO35, &max77802_ldo35_data},
	{MAX77802_EN32KHZ_AP, &max77802_en32khap_data},
	{MAX77802_EN32KHZ_CP, &max77802_en32khcp_data},
};

static struct max77802_opmode_data max77802_opmode_data[MAX77802_REG_MAX] = {
	[MAX77802_BUCK1] = {MAX77802_BUCK1, MAX77802_OPMODE_STANDBY},	/*vdd_mif*/
	[MAX77802_BUCK2] = {MAX77802_BUCK2, MAX77802_OPMODE_STANDBY},	/*arm*/
	[MAX77802_BUCK3] = {MAX77802_BUCK3, MAX77802_OPMODE_STANDBY},	/*int*/
	[MAX77802_BUCK4] = {MAX77802_BUCK4, MAX77802_OPMODE_STANDBY},	/*g3d*/
	[MAX77802_BUCK6] = {MAX77802_BUCK6, MAX77802_OPMODE_STANDBY},	/*kfc*/
	/* BUCK8 INAND2.85V, must be controlled by XMMC0CDN */
	[MAX77802_BUCK8] = {MAX77802_BUCK8, MAX77802_OPMODE_STANDBY},	/*dvdd285fnand*/
	[MAX77802_BUCK9] = {MAX77802_BUCK9, MAX77802_OPMODE_NORMAL},	/*dvdd20*/
	[MAX77802_BUCK10] = {MAX77802_BUCK10, MAX77802_OPMODE_NORMAL},	/*dvdd32*/
	[MAX77802_LDO1] = {MAX77802_LDO1, MAX77802_OPMODE_NORMAL},	/*alive*/
	[MAX77802_LDO2] = {MAX77802_LDO2, MAX77802_OPMODE_STANDBY},	/*memoff2*/
	[MAX77802_LDO3] = {MAX77802_LDO3, MAX77802_OPMODE_NORMAL},	/*dvdd18*/
	[MAX77802_LDO4] = {MAX77802_LDO4, MAX77802_OPMODE_STANDBY},	/*vdd28_mmc*/
	[MAX77802_LDO5] = {MAX77802_LDO5, MAX77802_OPMODE_STANDBY},	/*vdd18_hsic*/
	[MAX77802_LDO6] = {MAX77802_LDO6, MAX77802_OPMODE_STANDBY},	/*vdd18_pll*/
	[MAX77802_LDO7] = {MAX77802_LDO7, MAX77802_OPMODE_STANDBY},	/*vdd18_ispa*/
	[MAX77802_LDO8] = {MAX77802_LDO8, MAX77802_OPMODE_STANDBY},	/*vdd10_mipi*/
	[MAX77802_LDO9] = {MAX77802_LDO9, MAX77802_OPMODE_NORMAL},	/*vdd18_au*/
	[MAX77802_LDO10] = {MAX77802_LDO10, MAX77802_OPMODE_STANDBY},	/*vdd18_mipi*/
	[MAX77802_LDO11] = {MAX77802_LDO11, MAX77802_OPMODE_NORMAL},	/*vdd18_mmc01*/
	[MAX77802_LDO12] = {MAX77802_LDO12, MAX77802_OPMODE_STANDBY},	/*vdd30_usb*/
	[MAX77802_LDO13] = {MAX77802_LDO13, MAX77802_OPMODE_STANDBY},	/*vdd18_pre*/
	[MAX77802_LDO14] = {MAX77802_LDO14, MAX77802_OPMODE_STANDBY},	/*vdd18_abb*/
	[MAX77802_LDO15] = {MAX77802_LDO15, MAX77802_OPMODE_STANDBY},	/*vdd10_usb*/
	[MAX77802_LDO17] = {MAX77802_LDO17, MAX77802_OPMODE_STANDBY},	/*vdd12_ispa*/
	[MAX77802_LDO18] = {MAX77802_LDO18, MAX77802_OPMODE_STANDBY},	/*vdd18_lcd*/
	[MAX77802_LDO19] = {MAX77802_LDO19, MAX77802_OPMODE_STANDBY},	/*vdd18_fcamera*/
	/* LDO20 INAND_1.8V, must be controlled by XMMC0CDN*/
//	[MAX77802_LDO20] = {MAX77802_LDO20, MAX77802_OPMODE_NORMAL},	/*vdd18_inandif*/
	[MAX77802_LDO21] = {MAX77802_LDO21, MAX77802_OPMODE_NORMAL},	/*vdd18_mcu*/
	[MAX77802_LDO23] = {MAX77802_LDO23, MAX77802_OPMODE_STANDBY},	/*vcc28_af*/
	[MAX77802_LDO24] = {MAX77802_LDO24, MAX77802_OPMODE_NORMAL},	/*vcc28_tou*/
	[MAX77802_LDO25] = {MAX77802_LDO25, MAX77802_OPMODE_NORMAL},	/*vdd28_ir*/
	[MAX77802_LDO26] = {MAX77802_LDO26, MAX77802_OPMODE_NORMAL},	/*vdd28_sen*/
	[MAX77802_LDO27] = {MAX77802_LDO27, MAX77802_OPMODE_STANDBY},	/*vdd12_fcamera*/
	[MAX77802_LDO28] = {MAX77802_LDO28, MAX77802_OPMODE_NORMAL},	/*vdd18_tou*/
	[MAX77802_LDO29] = {MAX77802_LDO29, MAX77802_OPMODE_STANDBY},	/*vdd18_sen*/
	[MAX77802_LDO30] = {MAX77802_LDO30, MAX77802_OPMODE_STANDBY},	/*vdd12_memoff*/
	[MAX77802_LDO32] = {MAX77802_LDO32, MAX77802_OPMODE_STANDBY},	/*vdd28_micbias*/
	[MAX77802_LDO33] = {MAX77802_LDO33, MAX77802_OPMODE_STANDBY},	/*vcc28_sen*/
	[MAX77802_LDO34] = {MAX77802_LDO34, MAX77802_OPMODE_STANDBY},	/*vdd28_fcamera*/
	[MAX77802_LDO35] = {MAX77802_LDO35, MAX77802_OPMODE_STANDBY},	/*vdd12_8msen*/
};

static struct max77802_platform_data m65_max77802_pdata = {
	.irq_base		= IRQ_BOARD_START,
	.wakeup			= 1,
	.num_regulators		= ARRAY_SIZE(max77802_regulators),
	.regulators		= max77802_regulators,
	.opmode_data		= max77802_opmode_data, 
	.ramp_rate		= MAX77802_RAMP_RATE_50MV,
	.wtsr_smpl		= 1,

	.buck1_voltage[0]	= 1100000,
	.buck2_voltage[0]	= 1000000,
	.buck3_voltage[0]	= 1100000,
	.buck4_voltage[0]	= 1100000,
	.buck6_voltage[0]	= 1100000,
};

static void __init max77802_rt_init_res(void) {
	m65_max77802_pdata.irq_gpio = MEIZU_PMIC_IRQ;
	m65_max77802_pdata.buck12346_gpio_dvs[0].gpio = MEIZU_GPIO_DVS1;
	m65_max77802_pdata.buck12346_gpio_dvs[0].data = 0;
	m65_max77802_pdata.buck12346_gpio_dvs[1].gpio = MEIZU_GPIO_DVS2;
	m65_max77802_pdata.buck12346_gpio_dvs[1].data = 0;
	m65_max77802_pdata.buck12346_gpio_dvs[2].gpio = MEIZU_GPIO_DVS3;
	m65_max77802_pdata.buck12346_gpio_dvs[2].data = 0;
	m65_max77802_pdata.buck12346_gpio_selb[0] = MEIZU_BUCK1_SEL;
	m65_max77802_pdata.buck12346_gpio_selb[0] = MEIZU_BUCK2_SEL;
	m65_max77802_pdata.buck12346_gpio_selb[0] = MEIZU_BUCK3_SEL;
	m65_max77802_pdata.buck12346_gpio_selb[0] = MEIZU_BUCK4_SEL;
	m65_max77802_pdata.buck12346_gpio_selb[0] = MEIZU_BUCK6_SEL;
}
#endif /*CONFIG_REGULATOR_MAX77802*/

#endif //__MEIZU_M65_V2_REGULATOR_H_H__
