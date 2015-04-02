#ifndef __MEIZU_M69_V1_REGULATOR_H_H__
#define __MEIZU_M69_V1_REGULATOR_H_H__

#ifdef CONFIG_REGULATOR_S2MPS11

static struct regulator_consumer_supply s2mps11_buck1_consumer =
	REGULATOR_SUPPLY("vdd_mif", NULL);

static struct regulator_consumer_supply s2mps11_buck2_consumer =
	REGULATOR_SUPPLY("vdd_arm", NULL);

static struct regulator_consumer_supply s2mps11_buck3_consumer =
	REGULATOR_SUPPLY("vdd_int", NULL);

static struct regulator_consumer_supply s2mps11_buck4_consumer =
	REGULATOR_SUPPLY("vdd_g3d", NULL);

static struct regulator_consumer_supply s2mps11_buck5_consumer =
	REGULATOR_SUPPLY("vdd12_memon", NULL);

static struct regulator_consumer_supply s2mps11_buck5v123_consumer =
	REGULATOR_SUPPLY("vdd12_memoff", NULL);

static struct regulator_consumer_supply s2mps11_buck6_consumer =
	REGULATOR_SUPPLY("vdd_kfc", NULL);

static struct regulator_consumer_supply s2mps11_buck8_consumer =
	REGULATOR_SUPPLY("dvdd20", NULL);

static struct regulator_consumer_supply s2mps11_buck9_consumer =
	REGULATOR_SUPPLY("vdd_gps", "bcm4752");

static struct regulator_consumer_supply s2mps11_buck10_consumer =
	REGULATOR_SUPPLY("dvdd285fnand", "dw_mmc.0");

static struct regulator_consumer_supply s2mps11_ldo1_consumer =
	REGULATOR_SUPPLY("vdd10_alive", NULL);

static struct regulator_consumer_supply s2mps11_ldo2_consumer =
	REGULATOR_SUPPLY("dvdd18", NULL);

static struct regulator_consumer_supply s2mps11_ldo3_consumer =
	REGULATOR_SUPPLY("dvdd18_mmc0", NULL);

// for adc1
static struct regulator_consumer_supply s2mps11_ldo4_consumer =
	REGULATOR_SUPPLY("dvdd18_adc", NULL);

static struct regulator_consumer_supply s2mps11_ldo5_consumer =
	REGULATOR_SUPPLY("vdd18_pll", NULL);

static struct regulator_consumer_supply s2mps11_ldo6_consumer =
	REGULATOR_SUPPLY("vdd10_mipi", NULL);

static struct regulator_consumer_supply s2mps11_ldo7_consumer =
	REGULATOR_SUPPLY("vdd18_mipi", NULL);

static struct regulator_consumer_supply s2mps11_ldo8_consumer =
	REGULATOR_SUPPLY("vdd18_abb", NULL);

static struct regulator_consumer_supply s2mps11_ldo9_consumer =
	REGULATOR_SUPPLY("vdd30_usb", NULL);

static struct regulator_consumer_supply s2mps11_ldo10_consumer =
	REGULATOR_SUPPLY("vdd18_pre", NULL);

static struct regulator_consumer_supply s2mps11_ldo11_consumer =
	REGULATOR_SUPPLY("vdd10_usb", NULL);

static struct regulator_consumer_supply s2mps11_ldo12_consumer =
	REGULATOR_SUPPLY("vdd18_hsic", NULL);

static struct regulator_consumer_supply s2mps11_ldo13_consumer =
	REGULATOR_SUPPLY("vdd28_mmc", NULL);

static struct regulator_consumer_supply s2mps11_ldo14_consumer =
	REGULATOR_SUPPLY("vdd28_micbias", NULL);

static struct regulator_consumer_supply s2mps11_ldo15_consumer =
	REGULATOR_SUPPLY("vcc28_tou", NULL);

//NO LDO16

static struct regulator_consumer_supply s2mps11_ldo17_consumer[] = {
	REGULATOR_SUPPLY("vdd28_sen", NULL),
	REGULATOR_SUPPLY("vdd28_acc", NULL),    //accelerometer
	REGULATOR_SUPPLY("vdd28_gyr", NULL),    //gyroscope
};

static struct regulator_consumer_supply s2mps11_ldo18_consumer =
	REGULATOR_SUPPLY("vdd18_inandif", NULL);

// NO LDO19

static struct regulator_consumer_supply s2mps11_ldo20_consumer =
	REGULATOR_SUPPLY("dvdd18_mmc1", NULL);

static struct regulator_consumer_supply s2mps11_ldo21_consumer =
	REGULATOR_SUPPLY("vdd18_fcamera", NULL);

static struct regulator_consumer_supply s2mps11_ldo22_consumer =
	REGULATOR_SUPPLY("vdd12_fcamera", NULL);

// NO LDO23
		
static struct regulator_consumer_supply s2mps11_ldo24_consumer =
	REGULATOR_SUPPLY("vdd28_fcamera", NULL);

static struct regulator_consumer_supply s2mps11_ldo25_consumer =
	REGULATOR_SUPPLY("vdd18_ispa", NULL);

//NO LDO26

static struct regulator_consumer_supply s2mps11_ldo27_consumer =
	REGULATOR_SUPPLY("vdd12_ispa", NULL);

static struct regulator_consumer_supply s2mps11_ldo28_consumer =
	REGULATOR_SUPPLY("vcc28_8msen", NULL);

static struct regulator_consumer_supply s2mps11_ldo29_consumer =
	REGULATOR_SUPPLY("vdd18_lcd", NULL);

static struct regulator_consumer_supply s2mps11_ldo30_consumer[] = {
	REGULATOR_SUPPLY("vdd18_au", NULL),
	REGULATOR_SUPPLY("AVDD", NULL),
	REGULATOR_SUPPLY("LDOVDD", NULL),
	REGULATOR_SUPPLY("DBVDD", NULL),
	REGULATOR_SUPPLY("DBVDD1", NULL),
	REGULATOR_SUPPLY("DBVDD2", NULL),
	REGULATOR_SUPPLY("DBVDD3", NULL),
	REGULATOR_SUPPLY("CPVDD", NULL),
};

static struct regulator_consumer_supply s2mps11_ldo31_consumer =
	REGULATOR_SUPPLY("vdd18_8msen", NULL);

static struct regulator_consumer_supply s2mps11_ldo32_consumer =
	REGULATOR_SUPPLY("vdd18_mcu", NULL);

// NO LDO33

static struct regulator_consumer_supply s2mps11_ldo34_consumer =
	REGULATOR_SUPPLY("vdd28_ir", NULL);

static struct regulator_consumer_supply s2mps11_ldo35_consumer =
	REGULATOR_SUPPLY("vdd12_8msen", NULL);

static struct regulator_consumer_supply s2mps11_ldo36_consumer =
	REGULATOR_SUPPLY("vdd18_sen", NULL);

static struct regulator_consumer_supply s2mps11_ldo37_consumer =
	REGULATOR_SUPPLY("vdd18_tou", NULL);

static struct regulator_consumer_supply s2mps11_ldo38_consumer =
	REGULATOR_SUPPLY("vcc28_af", NULL);

static struct regulator_consumer_supply s2mps11_en32khap =
	REGULATOR_SUPPLY("ap_32khz", NULL);

static struct regulator_consumer_supply s2mps11_en32khcp =
	REGULATOR_SUPPLY("gps_32.768k", "bcm4752");

static struct regulator_init_data s2mps11_buck1_data = {
	.constraints	= {
		.name		= "vdd_mif range",
		.min_uV		= 800000,
		.max_uV		= 1300000,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.always_on = 1,
		.boot_on = 1,
		.state_mem	= {
			.uV		= 1100000,
			.mode		= REGULATOR_MODE_NORMAL,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2mps11_buck1_consumer,
};

static struct regulator_init_data s2mps11_buck2_data = {
	.constraints	= {
		.name		= "vdd_arm range",
		.min_uV		=  800000,
		.max_uV		= 1500000,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.always_on = 1,
		.boot_on = 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2mps11_buck2_consumer,
};

static struct regulator_init_data s2mps11_buck3_data = {
	.constraints	= {
		.name		= "vdd_int range",
		.min_uV		=  800000,
		.max_uV		= 1400000,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.always_on = 1,
		.boot_on = 1,
		.state_mem	= {
			.uV		= 1100000,
			.mode		= REGULATOR_MODE_NORMAL,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2mps11_buck3_consumer,
};

static struct regulator_init_data s2mps11_buck4_data = {
	.constraints	= {
		.name		= "vdd_g3d range",
		.min_uV		=  800000,
		.max_uV		= 1400000,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.always_on = 1,
		.boot_on = 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2mps11_buck4_consumer,
};

static struct regulator_init_data s2mps11_buck5_data = {
	.constraints	= {
		.name		= "VDD12_MEMON",
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
	.consumer_supplies	= &s2mps11_buck5_consumer,
};

static struct regulator_init_data s2mps11_buck5v123_data = {
	.constraints	= {
		.name		= "VDD12_MEMOFF",
		.min_uV		= 1200000,
		.max_uV		= 1200000,
		.apply_uV	= 1,
		.always_on	= 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2mps11_buck5v123_consumer,
};

static struct regulator_init_data s2mps11_buck6_data = {
	.constraints	= {
		.name		= "vdd_kfc range",
		.min_uV		=  800000,
		.max_uV		= 1500000,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.always_on = 1,
		.boot_on = 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2mps11_buck6_consumer,
};

static struct regulator_init_data s2mps11_buck8_data = {
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
	.consumer_supplies	= &s2mps11_buck8_consumer,
};

static struct regulator_init_data s2mps11_buck9_data = {
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
	.consumer_supplies	= &s2mps11_buck9_consumer,
};

static struct regulator_init_data s2mps11_buck10_data = {
	.constraints	= {
		.name		= "DVDD_285FNAND",
		.min_uV		= 2850000,
		.max_uV		= 2850000,
		.uV_offset	=  150000, /*offset 150mV for sandisk*/
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.always_on 	= 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2mps11_buck10_consumer,
};

static struct regulator_init_data s2mps11_ldo1_data = {
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
	.consumer_supplies	= &s2mps11_ldo1_consumer,
};

static struct regulator_init_data s2mps11_ldo2_data = {
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
	.consumer_supplies	= &s2mps11_ldo2_consumer,
};

static struct regulator_init_data s2mps11_ldo3_data = {
	.constraints	= {
		.name		= "DVDD18_MMC0",
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
	.consumer_supplies	= &s2mps11_ldo3_consumer,
};

static struct regulator_init_data s2mps11_ldo4_data = {
	.constraints	= {
		.name		= "DVDD18_ADC",
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
	.consumer_supplies	= &s2mps11_ldo4_consumer,
};

static struct regulator_init_data s2mps11_ldo5_data = {
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
	.consumer_supplies	= &s2mps11_ldo5_consumer,
};

static struct regulator_init_data s2mps11_ldo6_data = {
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
	.consumer_supplies	= &s2mps11_ldo6_consumer,
};

static struct regulator_init_data s2mps11_ldo7_data = {
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
	.consumer_supplies	= &s2mps11_ldo7_consumer,
};

static struct regulator_init_data s2mps11_ldo8_data = {
	.constraints	= {
		.name		= "VDD18_ABB",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.always_on	= 1,
		.boot_on	= 1,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2mps11_ldo8_consumer,
};

static struct regulator_init_data s2mps11_ldo9_data = {
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
	.consumer_supplies	= &s2mps11_ldo9_consumer,
};

static struct regulator_init_data s2mps11_ldo10_data = {
	.constraints	= {
		.name		= "VDD18_PRE",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.always_on	= 1,
		.boot_on	= 1,
		.state_mem	= {
			//.disabled	= 1,
			.enabled 	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2mps11_ldo10_consumer,
};

static struct regulator_init_data s2mps11_ldo11_data = {
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
	.consumer_supplies	= &s2mps11_ldo11_consumer,
};

static struct regulator_init_data s2mps11_ldo12_data = {
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
	.consumer_supplies	= &s2mps11_ldo12_consumer,
};

static struct regulator_init_data s2mps11_ldo13_data = {
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
	.consumer_supplies	= &s2mps11_ldo13_consumer,
};

static struct regulator_init_data s2mps11_ldo14_data = {
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
	.consumer_supplies	= &s2mps11_ldo14_consumer,
};

static struct regulator_init_data s2mps11_ldo15_data = {
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
	.consumer_supplies	= &s2mps11_ldo15_consumer,
};

// NO LDO16

static struct regulator_init_data s2mps11_ldo17_data = {
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
	.num_consumer_supplies	= ARRAY_SIZE(s2mps11_ldo17_consumer),
	.consumer_supplies	= s2mps11_ldo17_consumer,
};

static struct regulator_init_data s2mps11_ldo18_data = {
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
	.consumer_supplies	= &s2mps11_ldo18_consumer,
};

// NO LDO19

static struct regulator_init_data s2mps11_ldo20_data = {
	.constraints	= {
		.name		= "DVDD18_MMC1",
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
	.consumer_supplies	= &s2mps11_ldo20_consumer,
};

static struct regulator_init_data s2mps11_ldo21_data = {
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
	.consumer_supplies	= &s2mps11_ldo21_consumer,
};

static struct regulator_init_data s2mps11_ldo22_data = {
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
	.consumer_supplies	= &s2mps11_ldo22_consumer,
};

// NO LDO23

static struct regulator_init_data s2mps11_ldo24_data = {
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
	.consumer_supplies	= &s2mps11_ldo24_consumer,
};

static struct regulator_init_data s2mps11_ldo25_data = {
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
	.consumer_supplies	= &s2mps11_ldo25_consumer,
};

// NO LDO26

static struct regulator_init_data s2mps11_ldo27_data = {
	.constraints	= {
		.name		= "VDD12_ISPA",
		.min_uV		= 1300000,/* To remedy the PCB bug of m69 */
		.max_uV		= 1300000,
		.apply_uV	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2mps11_ldo27_consumer,
};

static struct regulator_init_data s2mps11_ldo28_data = {
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
	.consumer_supplies	= &s2mps11_ldo28_consumer,
};

static struct regulator_init_data s2mps11_ldo29_data = {
	.constraints	= {
		.name		= "VDD18_LCD",
		.min_uV 	= 1800000,
		.max_uV 	= 1800000,
		.apply_uV	= 1,
		.boot_on	= 0,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2mps11_ldo29_consumer,
};

static struct regulator_init_data s2mps11_ldo30_data = {
	.constraints	= {
		.name		= "VDD18_AU",
		.min_uV		= 1700000,
		.max_uV		= 1700000,
		.uV_offset	=   50000,
		.apply_uV	= 1,
		.always_on	= 1,
		.boot_on	= 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(s2mps11_ldo30_consumer),
	.consumer_supplies	= s2mps11_ldo30_consumer,
};

static struct regulator_init_data s2mps11_ldo31_data = {
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
	.consumer_supplies	= &s2mps11_ldo31_consumer,
};

#ifdef	CONFIG_MFD_MX_HUB
/*m069 MCU need this*/
static struct regulator_init_data s2mps11_ldo32_data = {
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
	.consumer_supplies	= &s2mps11_ldo32_consumer,
};
#else
static struct regulator_init_data s2mps11_ldo32_data = {
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
	.consumer_supplies	= &s2mps11_ldo32_consumer,
};
#endif //CONFIG_MFD_MX_HUB

// NO LDO33

static struct regulator_init_data s2mps11_ldo34_data = {
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
	.consumer_supplies	= &s2mps11_ldo34_consumer,
};

static struct regulator_init_data s2mps11_ldo35_data = {
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
	.consumer_supplies	= &s2mps11_ldo35_consumer,
};

static struct regulator_init_data s2mps11_ldo36_data = {
	.constraints	= {
		.name		= "VDD18_SEN",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		//.apply_uV	= 1,
		//.always_on	= 1,
		.boot_on	= 0,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled       = 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2mps11_ldo36_consumer,
};


static struct regulator_init_data s2mps11_ldo37_data = {
	.constraints	= {
		.name		= "VDD18_TOU",
#ifdef CONFIG_MEIZU_M69_V1
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		//.always_on	= 1,
		//.boot_on	= 1,
#else	/*just workaround keypad LED*/
		.min_uV		= 2400000,
		.max_uV		= 2400000,
		.apply_uV	= 1,
		.always_on	= 1,
		.boot_on	= 1,
#endif
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2mps11_ldo37_consumer,
};

static struct regulator_init_data s2mps11_ldo38_data = {
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
	.consumer_supplies	= &s2mps11_ldo38_consumer,
};

static struct regulator_init_data s2mps11_en32khap_data = {
	.constraints	= {
		.name		= "PMIC_AP32K",
		.always_on	= 1,
		.state_mem	= {
			.enabled       = 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2mps11_en32khap,
};

static struct regulator_init_data s2mps11_en32khcp_data = {
	.constraints	= {
		.name		= "32KHZ_GPS",
		.always_on	= 1,
		.state_mem	= {
			.enabled       = 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2mps11_en32khcp,
};

static struct sec_regulator_data s2mps11_regulators[] = {
	{S2MPS11_BUCK1, &s2mps11_buck1_data},
	{S2MPS11_BUCK2, &s2mps11_buck2_data},
	{S2MPS11_BUCK3, &s2mps11_buck3_data},
	{S2MPS11_BUCK4, &s2mps11_buck4_data},
	{S2MPS11_BUCK5, &s2mps11_buck5_data},
	{S2MPS11_BUCK5V123, &s2mps11_buck5v123_data},
	{S2MPS11_BUCK6, &s2mps11_buck6_data},
	{S2MPS11_BUCK8, &s2mps11_buck8_data},
	{S2MPS11_BUCK9, &s2mps11_buck9_data},
	{S2MPS11_BUCK10, &s2mps11_buck10_data},
	{S2MPS11_LDO1, &s2mps11_ldo1_data},
	{S2MPS11_LDO2, &s2mps11_ldo2_data},
	{S2MPS11_LDO3, &s2mps11_ldo3_data},
	{S2MPS11_LDO4, &s2mps11_ldo4_data},
	{S2MPS11_LDO5, &s2mps11_ldo5_data},
	{S2MPS11_LDO6, &s2mps11_ldo6_data},
	{S2MPS11_LDO7, &s2mps11_ldo7_data},
	{S2MPS11_LDO8, &s2mps11_ldo8_data},
	{S2MPS11_LDO9, &s2mps11_ldo9_data},
	{S2MPS11_LDO10, &s2mps11_ldo10_data},
	{S2MPS11_LDO11, &s2mps11_ldo11_data},
	{S2MPS11_LDO12, &s2mps11_ldo12_data},
	{S2MPS11_LDO13, &s2mps11_ldo13_data},
	{S2MPS11_LDO14, &s2mps11_ldo14_data},
	{S2MPS11_LDO15, &s2mps11_ldo15_data},
	// NO LDO16
	{S2MPS11_LDO17, &s2mps11_ldo17_data},
	{S2MPS11_LDO18, &s2mps11_ldo18_data},
	// NO LDO19
	{S2MPS11_LDO20, &s2mps11_ldo20_data},
	{S2MPS11_LDO21, &s2mps11_ldo21_data},
	{S2MPS11_LDO22, &s2mps11_ldo22_data},
	// NO LDO23
	{S2MPS11_LDO24, &s2mps11_ldo24_data},
	{S2MPS11_LDO25, &s2mps11_ldo25_data},
	// NO LDO26
	{S2MPS11_LDO27, &s2mps11_ldo27_data},
	{S2MPS11_LDO28, &s2mps11_ldo28_data},
	{S2MPS11_LDO29, &s2mps11_ldo29_data},
	{S2MPS11_LDO30, &s2mps11_ldo30_data},
	{S2MPS11_LDO31, &s2mps11_ldo31_data},
	{S2MPS11_LDO32, &s2mps11_ldo32_data},
	// NO LDO33
	{S2MPS11_LDO34, &s2mps11_ldo34_data},
	{S2MPS11_LDO35, &s2mps11_ldo35_data},
	{S2MPS11_LDO36, &s2mps11_ldo36_data},
	{S2MPS11_LDO37, &s2mps11_ldo37_data},
	{S2MPS11_LDO38, &s2mps11_ldo38_data},
	{S2MPS11_AP_EN32KHZ, &s2mps11_en32khap_data},
	{S2MPS11_CP_EN32KHZ, &s2mps11_en32khcp_data},
};

static struct sec_opmode_data s2mps11_opmode_data[S2MPS11_REG_MAX] = {
	[S2MPS11_BUCK1] = {S2MPS11_BUCK1, SEC_OPMODE_STANDBY},	   /*vdd_mif*/
	[S2MPS11_BUCK2] = {S2MPS11_BUCK2, SEC_OPMODE_STANDBY},	   /*arm*/
	[S2MPS11_BUCK3] = {S2MPS11_BUCK3, SEC_OPMODE_STANDBY},	   /*int*/
	[S2MPS11_BUCK4] = {S2MPS11_BUCK4, SEC_OPMODE_STANDBY},	   /*g3d*/
	[S2MPS11_BUCK5] = {S2MPS11_BUCK5, SEC_OPMODE_NORMAL},	   /*memon*/
	[S2MPS11_BUCK5V123] = {S2MPS11_BUCK5V123, SEC_OPMODE_STANDBY},	   /*memoff*/
	[S2MPS11_BUCK6] = {S2MPS11_BUCK6, SEC_OPMODE_STANDBY},	   /*kfc*/
	[S2MPS11_BUCK8] = {S2MPS11_BUCK8, SEC_OPMODE_NORMAL},	   /*dvdd20*/
	[S2MPS11_BUCK9] = {S2MPS11_BUCK9, SEC_OPMODE_NORMAL},	   /*dvdd32*/
	[S2MPS11_BUCK10] = {S2MPS11_BUCK10, SEC_OPMODE_STANDBY},	   /*dvdd285fnand*/

	[S2MPS11_LDO1] = {S2MPS11_LDO1, SEC_OPMODE_NORMAL},	       /*vdd10_alive*/
	[S2MPS11_LDO2] = {S2MPS11_LDO2, SEC_OPMODE_NORMAL},	       /*dvdd18*/
	[S2MPS11_LDO3] = {S2MPS11_LDO3, SEC_OPMODE_STANDBY},	       /*dvdd18_mmc0*/
	[S2MPS11_LDO4] = {S2MPS11_LDO4, SEC_OPMODE_STANDBY},	       /*dvdd18_adc*/
	[S2MPS11_LDO5] = {S2MPS11_LDO5, SEC_OPMODE_STANDBY},	   /*vdd18_pll*/
	[S2MPS11_LDO6] = {S2MPS11_LDO6, SEC_OPMODE_STANDBY},	   /*vdd10_mipi*/
	[S2MPS11_LDO7] = {S2MPS11_LDO7, SEC_OPMODE_STANDBY},	   /*vdd18_mipi*/
	[S2MPS11_LDO8] = {S2MPS11_LDO8, SEC_OPMODE_STANDBY},	   /*vdd18_abb*/
	[S2MPS11_LDO9] = {S2MPS11_LDO9, SEC_OPMODE_STANDBY},	   /*vdd30_usb*/
	[S2MPS11_LDO10] = {S2MPS11_LDO10, SEC_OPMODE_STANDBY},	   /*vdd18_pre*/
	[S2MPS11_LDO11] = {S2MPS11_LDO11, SEC_OPMODE_STANDBY},	   /*vdd10_usb*/
	[S2MPS11_LDO12] = {S2MPS11_LDO12, SEC_OPMODE_STANDBY},	   /*vdd18_hsic*/
	[S2MPS11_LDO13] = {S2MPS11_LDO13, SEC_OPMODE_STANDBY},	   /*vdd28_mmc*/
	[S2MPS11_LDO14] = {S2MPS11_LDO14, SEC_OPMODE_STANDBY},	   /*vdd28_micbias*/
	[S2MPS11_LDO15] = {S2MPS11_LDO15, SEC_OPMODE_NORMAL},	       /*vcc28_tou*/
	// NO LDO16
	[S2MPS11_LDO17] = {S2MPS11_LDO17, SEC_OPMODE_NORMAL},	   /*vdd28_sen*/
	[S2MPS11_LDO18] = {S2MPS11_LDO18, SEC_OPMODE_STANDBY},  /*vdd18_inandif*/
	// NO LDO19
	[S2MPS11_LDO20] = {S2MPS11_LDO20, SEC_OPMODE_NORMAL},	   /*dvdd18_mmc1*/
	[S2MPS11_LDO21] = {S2MPS11_LDO21, SEC_OPMODE_STANDBY},	   /*vdd18_fcamera*/
	[S2MPS11_LDO22] = {S2MPS11_LDO22, SEC_OPMODE_STANDBY},	   /*vcc12_fcamera*/
	// NO LDO23
	[S2MPS11_LDO24] = {S2MPS11_LDO24, SEC_OPMODE_STANDBY},	   /*vdd28_fcamera*/
	[S2MPS11_LDO25] = {S2MPS11_LDO25, SEC_OPMODE_STANDBY},	   /*vdd18_ispa*/
	// NO LDO26
	[S2MPS11_LDO27] = {S2MPS11_LDO27, SEC_OPMODE_STANDBY},	   /*vdd12_ispa*/
	[S2MPS11_LDO28] = {S2MPS11_LDO28, SEC_OPMODE_STANDBY},	   /*vcc28_8msen*/
	[S2MPS11_LDO29] = {S2MPS11_LDO29, SEC_OPMODE_STANDBY},	   /*vdd18_lcd*/
	[S2MPS11_LDO30] = {S2MPS11_LDO30, SEC_OPMODE_NORMAL},	   /*vdd18_au*/
	[S2MPS11_LDO31] = {S2MPS11_LDO31, SEC_OPMODE_STANDBY},	   /*vdd18_8msen*/
	[S2MPS11_LDO32] = {S2MPS11_LDO32, SEC_OPMODE_NORMAL},	   /*vdd18_mcu*/
	// NO LDO33
	[S2MPS11_LDO34] = {S2MPS11_LDO34, SEC_OPMODE_NORMAL},	   /*vdd28_ir*/
	[S2MPS11_LDO35] = {S2MPS11_LDO35, SEC_OPMODE_STANDBY},	   /*vdd12_8msen*/
	[S2MPS11_LDO36] = {S2MPS11_LDO36, SEC_OPMODE_STANDBY},	   /*vdd18_sen*/
	[S2MPS11_LDO37] = {S2MPS11_LDO37, SEC_OPMODE_NORMAL},	       /*vdd18_tou*/
	[S2MPS11_LDO38] = {S2MPS11_LDO38, SEC_OPMODE_STANDBY},	   /*vcc28_af*/
};

static int sec_cfg_irq(void)
{
	unsigned int pin = irq_to_gpio(M69_PMIC_EINT);

	s3c_gpio_cfgpin(pin, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(pin, S3C_GPIO_PULL_UP);

	return 0;
}

struct sec_pmic_platform_data m69_s2mps11_pdata = {
	.device_type	= S2MPS11X,
	.irq_base		= IRQ_BOARD_START,
	.num_regulators	= ARRAY_SIZE(s2mps11_regulators),
	.regulators		= s2mps11_regulators,
	.cfg_pmic_irq	= sec_cfg_irq,
	.wakeup			= 1,
	.wtsr_smpl		= 1,
	.opmode_data		= s2mps11_opmode_data,
	.buck16_ramp_delay	= 12,
	.buck2_ramp_delay	= 12,
	.buck34_ramp_delay	= 12,
	.buck2_ramp_enable	= 1,
	.buck3_ramp_enable	= 1,
	.buck4_ramp_enable	= 1,
	.buck6_ramp_enable	= 1,
};
#endif

#endif //__MEIZU_M65_V1_REGULATOR_H_H__
