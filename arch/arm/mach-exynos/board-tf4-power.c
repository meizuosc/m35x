/*
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/regulator/machine.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/gpio.h>
#include <linux/platform_data/exynos_thermal.h>

#include <asm/io.h>

#include <plat/devs.h>
#include <plat/iic.h>
#include <plat/gpio-cfg.h>

#include <mach/regs-pmu.h>
#include <mach/irqs.h>
#include <mach/hs-iic.h>
#include <mach/devfreq.h>
#include <mach/tmu.h>

#include <linux/mfd/samsung/core.h>
#include <linux/mfd/samsung/s2mps11.h>

#include "board-tf4.h"

#define TF4_PMIC_EINT	IRQ_EINT(0)

#if defined(CONFIG_BATTERY_MAX17040)
#include <linux/max17040_battery.h>
#endif

static struct regulator_consumer_supply s2m_buck1_consumer =
	REGULATOR_SUPPLY("vdd_mif", NULL);

static struct regulator_consumer_supply s2m_buck2_consumer =
	REGULATOR_SUPPLY("vdd_arm", NULL);

static struct regulator_consumer_supply s2m_buck3_consumer =
	REGULATOR_SUPPLY("vdd_int", NULL);

static struct regulator_consumer_supply s2m_buck4_consumer =
	REGULATOR_SUPPLY("vdd_g3d", NULL);

static struct regulator_consumer_supply s2m_buck6_consumer =
	REGULATOR_SUPPLY("vdd_kfc", NULL);

static struct regulator_consumer_supply s2m_buck10_consumer =
	REGULATOR_SUPPLY("vmmc", "dw_mmc.2");

static struct regulator_consumer_supply __initdata s2m_ldo1_consumer =
	REGULATOR_SUPPLY("vdd1_alive_1v0", NULL);

static struct regulator_consumer_supply __initdata s2m_ldo2_consumer =
	REGULATOR_SUPPLY("vdd2_APIO_1v8", NULL);

static struct regulator_consumer_supply __initdata s2m_ldo3_consumer =
	REGULATOR_SUPPLY("vdd3_APIO_MMCON_1v8", NULL);
		
static struct regulator_consumer_supply __initdata s2m_ldo4_consumer =
	REGULATOR_SUPPLY("vdd4_ADC_1v8", NULL);
	
static struct regulator_consumer_supply __initdata s2m_ldo5_consumer =
	REGULATOR_SUPPLY("vdd5_PLL_1v8", NULL);

static struct regulator_consumer_supply __initdata s2m_ldo6_consumer =
	REGULATOR_SUPPLY("vdd6_anaip_1v0", NULL);
	
static struct regulator_consumer_supply __initdata s2m_ldo7_consumer =
	REGULATOR_SUPPLY("vdd7_anaip_1v8", NULL);
	
static struct regulator_consumer_supply __initdata s2m_ldo8_consumer =
	REGULATOR_SUPPLY("vdd8_abb_1v8", NULL);
					
static struct regulator_consumer_supply __initdata s2m_ldo9_consumer =
	REGULATOR_SUPPLY("vdd9_usb_3v3", NULL);

static struct regulator_consumer_supply __initdata s2m_ldo10_consumer =
	REGULATOR_SUPPLY("vdd10_pre_1v8", NULL);

static struct regulator_consumer_supply __initdata s2m_ldo11_consumer =
	REGULATOR_SUPPLY("vdd11_usb_1v0", NULL);

static struct regulator_consumer_supply __initdata s2m_ldo12_consumer =
	REGULATOR_SUPPLY("vdd12_hsic_1v8", NULL);

static struct regulator_consumer_supply __initdata s2m_ldo13_consumer =
	REGULATOR_SUPPLY("vdd13_apio_mmcoff_2v8", NULL);

static struct regulator_consumer_supply __initdata s2m_ldo14_consumer =
	REGULATOR_SUPPLY("vdd14_cama_af_2v8", NULL);

static struct regulator_consumer_supply __initdata s2m_ldo15_consumer =
	REGULATOR_SUPPLY("vdd15_peri_2v8", NULL);

static struct regulator_consumer_supply __initdata s2m_ldo16_consumer =
	REGULATOR_SUPPLY("vdd16_peri_3v3", NULL);

static struct regulator_consumer_supply __initdata s2m_ldo17_consumer =
	REGULATOR_SUPPLY("vdd17_mipi2lvds_3v3", NULL);

static struct regulator_consumer_supply __initdata s2m_ldo18_consumer =
	REGULATOR_SUPPLY("vdd18_EMMC_1v8", NULL);

static struct regulator_consumer_supply __initdata s2m_ldo19_consumer =
	REGULATOR_SUPPLY("vdd19_tflash_2v8", NULL);

static struct regulator_consumer_supply __initdata s2m_ldo20_consumer =
	REGULATOR_SUPPLY("vdd20_ldo20", NULL);

static struct regulator_consumer_supply __initdata s2m_ldo21_consumer =
	REGULATOR_SUPPLY("vdd21_camaio_1v8", NULL);

static struct regulator_consumer_supply __initdata s2m_ldo22_consumer =
	REGULATOR_SUPPLY("vdd22_cama_1v2", NULL);

static struct regulator_consumer_supply __initdata s2m_ldo23_consumer =
	REGULATOR_SUPPLY("vdd23_lod23", NULL);

static struct regulator_consumer_supply __initdata s2m_ldo24_consumer =
	REGULATOR_SUPPLY("vdd24_cama_avdd_2v8", NULL);

static struct regulator_consumer_supply __initdata s2m_ldo25_consumer =
	REGULATOR_SUPPLY("vdd25_ldo25", NULL);

static struct regulator_consumer_supply __initdata s2m_ldo26_consumer =
	REGULATOR_SUPPLY("vdd26_camb_af_2v8", NULL);
									
static struct regulator_consumer_supply __initdata s2m_ldo27_consumer =
	REGULATOR_SUPPLY("vdd27_mipi2lvds_1v2", NULL);

static struct regulator_consumer_supply __initdata s2m_ldo28_consumer =
	REGULATOR_SUPPLY("vdd28_lcd_3v3", NULL);

static struct regulator_consumer_supply __initdata s2m_ldo29_consumer =
	REGULATOR_SUPPLY("vdd29_audio_1v8", NULL);

static struct regulator_consumer_supply __initdata s2m_ldo30_consumer =
	REGULATOR_SUPPLY("vdd30_btwifi_1v8", NULL);

static struct regulator_consumer_supply __initdata s2m_ldo31_consumer =
	REGULATOR_SUPPLY("vdd31_peri_1v8", NULL);

static struct regulator_consumer_supply __initdata s2m_ldo32_consumer =
	REGULATOR_SUPPLY("vdd32_mipi2lvds_1v8", NULL);

static struct regulator_consumer_supply __initdata s2m_ldo33_consumer =
	REGULATOR_SUPPLY("vdd33_cambio_1v8", NULL);
	
static struct regulator_consumer_supply __initdata s2m_ldo34_consumer =
	REGULATOR_SUPPLY("vdd34_btwifi_2v8", NULL);

static struct regulator_consumer_supply __initdata s2m_ldo35_consumer =
	REGULATOR_SUPPLY("vdd35_camb_1v2", NULL);
	
static struct regulator_consumer_supply __initdata s2m_ldo36_consumer =
	REGULATOR_SUPPLY("vdd36_ldo36", NULL);
	
static struct regulator_consumer_supply __initdata s2m_ldo37_consumer =
	REGULATOR_SUPPLY("vdd37_ldo37", NULL);
	
static struct regulator_consumer_supply __initdata s2m_ldo38_consumer =
	REGULATOR_SUPPLY("vdd38_camb_avdd_2v8", NULL);

static struct regulator_consumer_supply __initdata s2m_cp_en32khz_consumer =
	REGULATOR_SUPPLY("cp_32khz", NULL);

static struct regulator_init_data s2m_buck1_data = {
	.constraints	= {
		.name		= "vdd_mif range",
		.min_uV		=  800000,
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
	.consumer_supplies	= &s2m_buck1_consumer,
};

static struct regulator_init_data s2m_buck2_data = {
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
	.consumer_supplies	= &s2m_buck2_consumer,
};

static struct regulator_init_data s2m_buck3_data = {
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
	.consumer_supplies	= &s2m_buck3_consumer,
};

static struct regulator_init_data s2m_buck4_data = {
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
	.consumer_supplies	= &s2m_buck4_consumer,
};

static struct regulator_init_data s2m_buck6_data = {
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
	.consumer_supplies	= &s2m_buck6_consumer,
};

static struct regulator_init_data s2m_buck10_data = {
	.constraints	= {
		.name		= "vdd_emmcf range",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.always_on = 1,
		.boot_on = 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_buck10_consumer,
};

static struct regulator_init_data s2m_ldo1_data = {
	.constraints	= {
		.name		= "vdd_ldo1 range",
		.min_uV		= 1000000,
		.max_uV		= 1000000,
		.always_on	= 1,
		.boot_on    = 1,		
		.apply_uV	  = 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo1_consumer,
};

static struct regulator_init_data s2m_ldo2_data = {
	.constraints	= {
		.name		= "vdd_ldo2 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.always_on	= 1,
		.boot_on    = 1,		
		.apply_uV	  = 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,	
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo2_consumer,
};

static struct regulator_init_data s2m_ldo3_data = {
	.constraints	= {
		.name		= "vdd_ldo3 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.always_on	= 1,
		.boot_on    = 1,		
		.apply_uV	  = 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo3_consumer,
};

static struct regulator_init_data s2m_ldo4_data = {
	.constraints	= {
		.name		= "vdd_ldo4 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.always_on	= 1,
		.boot_on    = 1,		
		.apply_uV	  = 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo4_consumer,
};

static struct regulator_init_data s2m_ldo5_data = {
	.constraints	= {
		.name		= "vdd_ldo5 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.always_on	= 1,
		.boot_on    = 1,		
		.apply_uV	  = 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo5_consumer,
};

static struct regulator_init_data s2m_ldo6_data = {
	.constraints	= {
		.name		= "vdd_ldo6 range",
		.min_uV		= 1000000,
		.max_uV		= 1000000,
		.always_on	= 1,
		.boot_on    = 1,		
		.apply_uV	  = 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,			
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo6_consumer,
};

static struct regulator_init_data s2m_ldo7_data = {
	.constraints	= {
		.name		= "vdd_ldo7 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.always_on	= 1,
		.boot_on    = 1,		
		.apply_uV	  = 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo7_consumer,
};

static struct regulator_init_data s2m_ldo8_data = {
	.constraints	= {
		.name		= "vdd_ldo8 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.always_on	= 1,
		.boot_on    = 1,		
		.apply_uV	  = 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo8_consumer,
};

static struct regulator_init_data s2m_ldo9_data = {
	.constraints	= {
		.name		= "vdd_ldo9 range",
		.min_uV		= 3000000,
		.max_uV		= 3000000,
		.always_on	= 1,
		.boot_on    = 1,		
		.apply_uV	  = 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo9_consumer,
};

static struct regulator_init_data s2m_ldo10_data = {
	.constraints	= {
		.name		= "vdd_ldo10 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.always_on	= 1,
		.boot_on    = 1,		
		.apply_uV	  = 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
	 	.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo10_consumer,
};

static struct regulator_init_data s2m_ldo11_data = {
	.constraints	= {
		.name		= "vdd_ldo11 range",
		.min_uV		= 1000000,
		.max_uV		= 1000000,
		.always_on	= 1,
		.boot_on    = 1,		
		.apply_uV	  = 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo11_consumer,
};

static struct regulator_init_data s2m_ldo12_data = {
	.constraints	= {
		.name		= "vdd_ldo12 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.always_on	= 1,
		.boot_on    = 1,		
		.apply_uV	  = 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo12_consumer,
};

static struct regulator_init_data s2m_ldo13_data = {
	.constraints	= {
		.name		= "vdd_ldo13 range",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.always_on	= 1,
		.boot_on    = 1,		
		.apply_uV	  = 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,				
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo13_consumer,
};

static struct regulator_init_data s2m_ldo14_data = {
	.constraints	= {
		.name		= "vdd_ldo14 range",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.always_on	= 0,
		.boot_on    = 0,		
		.apply_uV	  = 0,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,				
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo14_consumer,
};

static struct regulator_init_data s2m_ldo15_data = {
	.constraints	= {
		.name		= "vdd_ldo15 range",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.always_on	= 1,
		.boot_on    = 1,		
		.apply_uV	  = 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo15_consumer,
};

static struct regulator_init_data s2m_ldo16_data = {
	.constraints	= {
		.name		= "vdd_ldo16 range",
		.min_uV		= 3300000,
		.max_uV		= 3300000,
		.always_on	= 1,
		.boot_on    = 1,		
		.apply_uV	  = 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo16_consumer,
};

static struct regulator_init_data s2m_ldo17_data = {
	.constraints	= {
		.name		= "vdd_ldo17 range",
		.min_uV		= 3300000,
		.max_uV		= 3300000,
		.always_on	= 0,
		.boot_on    = 1,		
		.apply_uV	  = 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo17_consumer,
};

static struct regulator_init_data s2m_ldo18_data = {
	.constraints	= {
		.name		= "vdd_ldo18 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.always_on	= 1,
		.boot_on    = 1,		
		.apply_uV	  = 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo18_consumer,
};

static struct regulator_init_data s2m_ldo19_data = {
	.constraints	= {
		.name		= "vdd_ldo19 range",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.always_on	= 1,
		.boot_on    = 1,		
		.apply_uV	  = 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo19_consumer,
};

static struct regulator_init_data s2m_ldo20_data = {
	.constraints	= {
		.name		= "vdd_ldo20 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.always_on	= 1,
		.boot_on    = 1,				
		.apply_uV	  = 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo20_consumer,
};

static struct regulator_init_data s2m_ldo21_data = {
	.constraints	= {
		.name		= "vdd_ldo21 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.always_on	= 0,
		.boot_on    = 0,		
		.apply_uV	  = 0,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo21_consumer,
};

static struct regulator_init_data s2m_ldo22_data = {
	.constraints	= {
		.name		= "vdd_ldo22 range",
		.min_uV		= 1200000,
		.max_uV		= 1200000,
		.always_on	= 0,
		.boot_on    = 0,		
		.apply_uV	  = 0,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo22_consumer,
};

static struct regulator_init_data s2m_ldo23_data = {
	.constraints	= {
		.name		= "vdd_ldo23 range",
		.min_uV		= 1200000,
		.max_uV		= 1200000,
		.always_on	= 1,
		.boot_on    = 1,				
		.apply_uV	  = 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo23_consumer,
};

static struct regulator_init_data s2m_ldo24_data = {
	.constraints	= {
		.name		= "vdd_ldo24 range",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.always_on	= 0,
		.boot_on    = 0,		
		.apply_uV	  = 0,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo24_consumer,
};

static struct regulator_init_data s2m_ldo25_data = {
	.constraints	= {
		.name		= "vdd_ldo25 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.always_on	= 1,
		.boot_on    = 1,				
		.apply_uV	  = 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo25_consumer,
};

static struct regulator_init_data s2m_ldo26_data = {
	.constraints	= {
		.name		= "vdd_ldo26 range",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.always_on	= 0,
		.boot_on    = 0,		
		.apply_uV	  = 0,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo26_consumer,
};

static struct regulator_init_data s2m_ldo27_data = {
	.constraints	= {
		.name		= "vdd_ldo27 range",
		.min_uV		= 1200000,
		.max_uV		= 1200000,
		.always_on	= 0,
		.boot_on    = 0,		
		.apply_uV	  = 0,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,	
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo27_consumer,
};

static struct regulator_init_data s2m_ldo28_data = {
	.constraints	= {
		.name		= "vdd_ldo28 range",
		.min_uV		= 3300000,
		.max_uV		= 3300000,
		.always_on	= 0,
		.boot_on    = 0,		
		.apply_uV	  = 0,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,			
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo28_consumer,
};
static struct regulator_init_data s2m_ldo29_data = {
	.constraints	= {
		.name		= "vdd_ldo29 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.always_on	= 1,
		.boot_on    = 1,		
		.apply_uV	  = 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo29_consumer,
};
static struct regulator_init_data s2m_ldo30_data = {
	.constraints	= {
		.name		= "vdd_ldo30 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.always_on	= 1,
		.boot_on    = 1,		
		.apply_uV	  = 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo30_consumer,
};
static struct regulator_init_data s2m_ldo31_data = {
	.constraints	= {
		.name		= "vdd_ldo31 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.always_on	= 0,
		.boot_on    = 0,		
		.apply_uV	  = 0,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo31_consumer,
};

static struct regulator_init_data s2m_ldo32_data = {
	.constraints	= {
		.name		= "vdd_ldo32 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.always_on	= 0,
		.boot_on    = 0,		
		.apply_uV	  = 0,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo32_consumer,
};

static struct regulator_init_data s2m_ldo33_data = {
	.constraints	= {
		.name		= "vdd_ldo33 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.always_on	= 0,
		.boot_on    = 0,		
		.apply_uV	  = 0,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo33_consumer,
};

static struct regulator_init_data s2m_ldo34_data = {
	.constraints	= {
		.name		= "vdd_ldo34 range",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.always_on	= 0,
		.boot_on    = 0,		
		.apply_uV	  = 0,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,				
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo34_consumer,
};

static struct regulator_init_data s2m_ldo35_data = {
	.constraints	= {
		.name		= "vdd_ldo35 range",
		.min_uV		= 1200000,
		.max_uV		= 1200000,
		.always_on	= 0,
		.boot_on    = 0,		
		.apply_uV	  = 0,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo35_consumer,
};

static struct regulator_init_data s2m_ldo36_data = {
	.constraints	= {
		.name		= "vdd_ldo36 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.always_on	= 1,
		.boot_on    = 1,		
		.apply_uV	  = 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo36_consumer,
};

static struct regulator_init_data s2m_ldo37_data = {
	.constraints	= {
		.name		= "vdd_ldo37 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.always_on	= 1,
		.boot_on    = 1,		
		.apply_uV	  = 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo37_consumer,
};

static struct regulator_init_data s2m_ldo38_data = {
	.constraints	= {
		.name		= "vdd_ldo38 range",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.always_on	= 0,
		.boot_on    = 0,		
		.apply_uV	  = 0,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,		
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_ldo38_consumer,
};

static struct regulator_init_data s2m_cp_en32khz_data = {
	.constraints	= {
		.name		= "cp_en32khz range",
		.always_on	= 0,
		.boot_on    = 0,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s2m_cp_en32khz_consumer,
};

static struct sec_regulator_data exynos_regulators[] = {
	{S2MPS11_BUCK1, &s2m_buck1_data},
	{S2MPS11_BUCK2, &s2m_buck2_data},
	{S2MPS11_BUCK3, &s2m_buck3_data},
	{S2MPS11_BUCK4, &s2m_buck4_data},
//	{S2MPS11_BUCK5, &s2m_buck5_data},   //
	{S2MPS11_BUCK6, &s2m_buck6_data},
//	{S2MPS11_BUCK7, &s2m_buck7_data},   //
//	{S2MPS11_BUCK8, &s2m_buck8_data},   //
//	{S2MPS11_BUCK9, &s2m_buck9_data},   //
	{S2MPS11_BUCK10,&s2m_buck10_data},	
	{S2MPS11_LDO1,  &s2m_ldo1_data},
	{S2MPS11_LDO2,  &s2m_ldo2_data},
//	{S2MPS11_LDO3,  &s2m_ldo3_data},	
	{S2MPS11_LDO4,  &s2m_ldo4_data},	
	{S2MPS11_LDO5,  &s2m_ldo5_data},		
	{S2MPS11_LDO6,  &s2m_ldo6_data},   		
	{S2MPS11_LDO7,  &s2m_ldo7_data},		
	{S2MPS11_LDO8,  &s2m_ldo8_data},
	{S2MPS11_LDO9,  &s2m_ldo9_data},
	{S2MPS11_LDO10, &s2m_ldo10_data},	
	{S2MPS11_LDO11, &s2m_ldo11_data},
	{S2MPS11_LDO12, &s2m_ldo12_data},	
	{S2MPS11_LDO13, &s2m_ldo13_data},	
	{S2MPS11_LDO14, &s2m_ldo14_data},	
	{S2MPS11_LDO15, &s2m_ldo15_data},
	{S2MPS11_LDO16, &s2m_ldo16_data},
	{S2MPS11_LDO17, &s2m_ldo17_data},
	{S2MPS11_LDO18, &s2m_ldo18_data},	
	{S2MPS11_LDO19, &s2m_ldo19_data}, 	
	{S2MPS11_LDO20, &s2m_ldo20_data},	 
	{S2MPS11_LDO21, &s2m_ldo21_data},	
	{S2MPS11_LDO22, &s2m_ldo22_data},	
	{S2MPS11_LDO23, &s2m_ldo23_data},	
	{S2MPS11_LDO24, &s2m_ldo24_data},	
	{S2MPS11_LDO25, &s2m_ldo25_data},	
	{S2MPS11_LDO26, &s2m_ldo26_data},	
	{S2MPS11_LDO27, &s2m_ldo27_data},
	{S2MPS11_LDO28, &s2m_ldo28_data},
	{S2MPS11_LDO29, &s2m_ldo29_data},
	{S2MPS11_LDO30, &s2m_ldo30_data},
	{S2MPS11_LDO31, &s2m_ldo31_data},
	{S2MPS11_LDO32, &s2m_ldo32_data},
	{S2MPS11_LDO33, &s2m_ldo33_data},  
	{S2MPS11_LDO34, &s2m_ldo34_data},
	{S2MPS11_LDO35, &s2m_ldo35_data}, 
	{S2MPS11_LDO36, &s2m_ldo36_data},
	{S2MPS11_LDO37, &s2m_ldo37_data},
	{S2MPS11_LDO38, &s2m_ldo38_data},
	{S2MPS11_CP_EN32KHZ, &s2m_cp_en32khz_data},
};

struct sec_opmode_data s2mps11_opmode_data[S2MPS11_REG_MAX] = {
	[S2MPS11_BUCK1] = {S2MPS11_BUCK1, SEC_OPMODE_STANDBY},
	[S2MPS11_BUCK2] = {S2MPS11_BUCK2, SEC_OPMODE_STANDBY},
	[S2MPS11_BUCK3] = {S2MPS11_BUCK3, SEC_OPMODE_STANDBY},
	[S2MPS11_BUCK4] = {S2MPS11_BUCK4, SEC_OPMODE_STANDBY},
	[S2MPS11_BUCK6] = {S2MPS11_BUCK6, SEC_OPMODE_STANDBY},
	[S2MPS11_BUCK7] = {S2MPS11_BUCK7, SEC_OPMODE_STANDBY},
	[S2MPS11_BUCK8] = {S2MPS11_BUCK8, SEC_OPMODE_STANDBY},
	[S2MPS11_BUCK9] = {S2MPS11_BUCK9, SEC_OPMODE_STANDBY},
	[S2MPS11_BUCK10]= {S2MPS11_BUCK10,SEC_OPMODE_STANDBY},
    [S2MPS11_LDO1] =  {S2MPS11_LDO1,  SEC_OPMODE_NORMAL},
	[S2MPS11_LDO2] =  {S2MPS11_LDO2,  SEC_OPMODE_NORMAL},
    [S2MPS11_LDO3] =  {S2MPS11_LDO3,  SEC_OPMODE_NORMAL},
	[S2MPS11_LDO4] =  {S2MPS11_LDO4,  SEC_OPMODE_NORMAL},	
	[S2MPS11_LDO5] =  {S2MPS11_LDO5,  SEC_OPMODE_STANDBY},			
    [S2MPS11_LDO6] =  {S2MPS11_LDO6,  SEC_OPMODE_STANDBY},
    [S2MPS11_LDO7] =  {S2MPS11_LDO7,  SEC_OPMODE_STANDBY},
    [S2MPS11_LDO8] =  {S2MPS11_LDO8,  SEC_OPMODE_STANDBY},
	[S2MPS11_LDO9] =  {S2MPS11_LDO9,  SEC_OPMODE_STANDBY},
	[S2MPS11_LDO10] = {S2MPS11_LDO10, SEC_OPMODE_STANDBY},
	[S2MPS11_LDO11] = {S2MPS11_LDO11, SEC_OPMODE_STANDBY},
	[S2MPS11_LDO12] = {S2MPS11_LDO12, SEC_OPMODE_STANDBY},
    [S2MPS11_LDO13] = {S2MPS11_LDO13, SEC_OPMODE_STANDBY},
    [S2MPS11_LDO14] = {S2MPS11_LDO14, SEC_OPMODE_STANDBY},
	[S2MPS11_LDO15] = {S2MPS11_LDO15, SEC_OPMODE_NORMAL},
	[S2MPS11_LDO16] = {S2MPS11_LDO16, SEC_OPMODE_NORMAL},
	[S2MPS11_LDO17] = {S2MPS11_LDO17, SEC_OPMODE_STANDBY},
	[S2MPS11_LDO18] = {S2MPS11_LDO18, SEC_OPMODE_NORMAL},	
	[S2MPS11_LDO19] = {S2MPS11_LDO19, SEC_OPMODE_NORMAL},	
	[S2MPS11_LDO20] = {S2MPS11_LDO20, SEC_OPMODE_STANDBY},	
	[S2MPS11_LDO21] = {S2MPS11_LDO21, SEC_OPMODE_STANDBY},	
	[S2MPS11_LDO22] = {S2MPS11_LDO22, SEC_OPMODE_STANDBY},	
	[S2MPS11_LDO23] = {S2MPS11_LDO23, SEC_OPMODE_STANDBY},	
	[S2MPS11_LDO24] = {S2MPS11_LDO24, SEC_OPMODE_STANDBY},	
	[S2MPS11_LDO25] = {S2MPS11_LDO25, SEC_OPMODE_STANDBY},	
	[S2MPS11_LDO26] = {S2MPS11_LDO26, SEC_OPMODE_NORMAL},	
	[S2MPS11_LDO27] = {S2MPS11_LDO27, SEC_OPMODE_STANDBY},
	[S2MPS11_LDO28] = {S2MPS11_LDO28, SEC_OPMODE_STANDBY},
	[S2MPS11_LDO29] = {S2MPS11_LDO29, SEC_OPMODE_NORMAL},
	[S2MPS11_LDO30] = {S2MPS11_LDO30, SEC_OPMODE_STANDBY},
	[S2MPS11_LDO31] = {S2MPS11_LDO31, SEC_OPMODE_NORMAL},
	[S2MPS11_LDO32] = {S2MPS11_LDO32, SEC_OPMODE_STANDBY},
	[S2MPS11_LDO33] = {S2MPS11_LDO33, SEC_OPMODE_NORMAL},
	[S2MPS11_LDO34] = {S2MPS11_LDO34, SEC_OPMODE_STANDBY},
	[S2MPS11_LDO35] = {S2MPS11_LDO35, SEC_OPMODE_NORMAL},
	[S2MPS11_LDO36] = {S2MPS11_LDO36, SEC_OPMODE_STANDBY},
	[S2MPS11_LDO37] = {S2MPS11_LDO37, SEC_OPMODE_STANDBY},
	[S2MPS11_LDO38] = {S2MPS11_LDO38, SEC_OPMODE_NORMAL},			
};

static int sec_cfg_irq(void)
{
	unsigned int pin = irq_to_gpio(TF4_PMIC_EINT);

	s3c_gpio_cfgpin(pin, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(pin, S3C_GPIO_PULL_UP);

	return 0;
}

static struct sec_pmic_platform_data exynos5_s2m_pdata = {
	.device_type		= S2MPS11X,
	.irq_base		= IRQ_BOARD_START,
	.num_regulators		= ARRAY_SIZE(exynos_regulators),
	.regulators		= exynos_regulators,
	.cfg_pmic_irq		= sec_cfg_irq,
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

static struct i2c_board_info hs_i2c_devs0[] __initdata = {
	{
		I2C_BOARD_INFO("sec-pmic", 0xCC >> 1),
		.platform_data = &exynos5_s2m_pdata,
		.irq		= TF4_PMIC_EINT,
	},
};

#ifdef CONFIG_BATTERY_SAMSUNG
static struct platform_device samsung_device_battery = {
	.name	= "samsung-fake-battery",
	.id	= -1,
};
#endif

#if defined(CONFIG_BATTERY_MAX17040_TF4_EVT_I2C) || defined(CONFIG_BATTERY_PM2301) || defined(CONFIG_CHARGER_SMB349)
static struct i2c_gpio_platform_data gpio_i2c11_data = {
	.sda_pin            = EXYNOS5410_GPY5(6),
	.scl_pin            = EXYNOS5410_GPY5(7),
	.udelay             = 2,
	.sda_is_open_drain  = 0,
	.scl_is_open_drain  = 0,
  .scl_is_output_only = 0,
};

static struct platform_device gpio_i2c11_dev = {
	.name               = "i2c-gpio",
	.id                 = 11,
	.dev.platform_data  = &gpio_i2c11_data,
};
#endif

#if defined(CONFIG_BATTERY_MAX17040_TF4_DVT_I2C)
static struct i2c_gpio_platform_data gpio_i2c12_data = {
	.sda_pin            = EXYNOS5410_GPG0(6),
	.scl_pin            = EXYNOS5410_GPG0(5),
	.udelay             = 2,
	.sda_is_open_drain  = 0,
	.scl_is_open_drain  = 0,
  .scl_is_output_only = 0,
};

static struct platform_device gpio_i2c12_dev = {
	.name               = "i2c-gpio",
	.id                 = 12,
	.dev.platform_data  = &gpio_i2c12_data,
};
#endif




#if defined(CONFIG_BATTERY_MAX17040)
struct platform_device pm2301_bat_dev = {
	.name		= "pm2301-battery",
	.id		= -1,
};
#endif


#if defined(CONFIG_CHARGER_SMB349)
struct platform_device smb349_bat_dev
 = {
	.name		= "smb349-battery",
	.id		= -1,
};
#endif



#ifdef CONFIG_PM_DEVFREQ
static struct platform_device exynos5_mif_devfreq = {
	.name	= "exynos5-busfreq-mif",
	.id	= -1,
};

static struct platform_device exynos5_int_devfreq = {
	.name	= "exynos5-busfreq-int",
	.id	= -1,
};

static struct exynos_devfreq_platdata smdk5410_qos_mif_pd __initdata = {
	.default_qos = 160000,
};

static struct exynos_devfreq_platdata smdk5410_qos_int_pd __initdata = {
	.default_qos = 100000,
};
#endif

#ifdef CONFIG_ARM_EXYNOS_IKS_CPUFREQ
static struct exynos_tmu_platform_data exynos5_tmu_data = {
	.trigger_levels[0] = 105,
	.trigger_levels[1] = 110,
	.trigger_levels[2] = 115,
	.trigger_levels[3] = 120,
	.trigger_level0_en = 1,
	.trigger_level1_en = 1,
	.trigger_level2_en = 1,
	.trigger_level3_en = 1,
	.gain = 5,
	.reference_voltage = 16,
	.noise_cancel_mode = 4,
	.cal_type = TYPE_ONE_POINT_TRIMMING,
	.efuse_value = 55,
	.freq_tab[0] = {
		.freq_clip_max = 1200 * 1000,
		.temp_level = 105,
	},
	.freq_tab[1] = {
		.freq_clip_max = 900 * 1000,
		.temp_level = 110,
	},
	.freq_tab[2] = {
		.freq_clip_max = 600 * 1000,
		.temp_level = 112,
	},
	.freq_tab[3] = {
		.freq_clip_max = 400 * 1000,
		.temp_level = 114,
	},
	.freq_tab[4] = {
		.freq_clip_max = 200 * 1000,
		.temp_level = 115,
	},
	.size[THERMAL_TRIP_ACTIVE] = 1,
	.size[THERMAL_TRIP_PASSIVE] = 3,
	.size[THERMAL_TRIP_HOT] = 1,
	.freq_tab_count = 5,
	.type = SOC_ARCH_EXYNOS5,
};
#else
static struct exynos_tmu_platform_data exynos5_tmu_data = {
	.trigger_levels[0] = 85,
	.trigger_levels[1] = 90,
	.trigger_levels[2] = 110,
	.trigger_levels[3] = 105,
	.trigger_level0_en = 1,
	.trigger_level1_en = 1,
	.trigger_level2_en = 1,
	.trigger_level3_en = 1,
	.gain = 5,
	.reference_voltage = 16,
	.noise_cancel_mode = 4,
	.cal_type = TYPE_ONE_POINT_TRIMMING,
	.efuse_value = 55,
	.freq_tab[0] = {
		.freq_clip_max = 1600 * 1000,
		.temp_level = 85,
	},
	.freq_tab[1] = {
		.freq_clip_max = 1400 * 1000,
		.temp_level = 90,
	},
	.freq_tab[2] = {
		.freq_clip_max = 1200 * 1000,
		.temp_level = 95,
	},
	.freq_tab[3] = {
		.freq_clip_max = 800 * 1000,
		.temp_level = 100,
	},
	.freq_tab[4] = {
		.freq_clip_max = 400 * 1000,
		.temp_level = 110,
	},
	.size[THERMAL_TRIP_ACTIVE] = 1,
	.size[THERMAL_TRIP_PASSIVE] = 3,
	.size[THERMAL_TRIP_HOT] = 1,
	.freq_tab_count = 5,
	.type = SOC_ARCH_EXYNOS5,
};
#endif

static struct platform_device *tf4_power_devices[] __initdata = {
	/* Samsung Power Domain */
	&exynos5_device_hs_i2c0,
#ifdef CONFIG_BATTERY_SAMSUNG
	&samsung_device_battery,
#endif
#ifdef CONFIG_PM_DEVFREQ
	&exynos5_mif_devfreq,
	&exynos5_int_devfreq,
#endif
	&exynos5410_device_tmu,
#if defined(CONFIG_BATTERY_MAX17040_TF4_EVT_I2C) || defined(CONFIG_BATTERY_PM2301) || defined(CONFIG_CHARGER_SMB349)
    &gpio_i2c11_dev,
#endif
#if defined(CONFIG_BATTERY_MAX17040_TF4_DVT_I2C)
    &gpio_i2c12_dev,
#endif     
#if defined(CONFIG_BATTERY_MAX17040)
    &pm2301_bat_dev,
#endif
#if defined(CONFIG_CHARGER_SMB349)
    &smb349_bat_dev,
#endif
};

#if defined(CONFIG_BATTERY_MAX17040)
static struct max17040_platform_data max17040_platform_data = {
	.battery_online = NULL,
	.charger_online = NULL,
	.charger_enable = NULL,
};
#endif
static struct i2c_board_info i2c_devs11[] __initdata = {
#if defined(CONFIG_BATTERY_MAX17040_TF4_EVT_I2C)
	{
		I2C_BOARD_INFO("max17040", 0x36 ),//(0x36 >> 1)),
		.platform_data = &max17040_platform_data,
	},
#endif
#if defined(CONFIG_BATTERY_PM2301)
	{
		I2C_BOARD_INFO("pm2301", 0x2C), //(0x58 >> 1)),
		.platform_data = NULL,
	},
#endif
#if defined(CONFIG_CHARGER_SMB349)
  {
 		I2C_BOARD_INFO("smb349", 0x1B), //(0x36 >> 1)),
		.platform_data = NULL,
  }
#endif  
};

#if defined(CONFIG_BATTERY_MAX17040_TF4_DVT_I2C)
static struct i2c_board_info i2c_devs12[] __initdata = {
	{
		I2C_BOARD_INFO("max17040", 0x36 ),//(0x36 >> 1)),
		.platform_data = &max17040_platform_data,
	},
};
#endif



void __init exynos5_tf4_power_init(void)
{
	exynos5_hs_i2c0_set_platdata(NULL);
	i2c_register_board_info(4, hs_i2c_devs0, ARRAY_SIZE(hs_i2c_devs0));
#if defined(CONFIG_BATTERY_MAX17040_TF4_EVT_I2C) || defined(CONFIG_BATTERY_PM2301) || defined(CONFIG_CHARGER_SMB349)
	i2c_register_board_info(11,i2c_devs11, ARRAY_SIZE(i2c_devs11));
#endif   
#if defined(CONFIG_BATTERY_MAX17040_TF4_DVT_I2C) 
	i2c_register_board_info(12,i2c_devs12, ARRAY_SIZE(i2c_devs12));
#endif  
#ifdef CONFIG_PM_DEVFREQ
	s3c_set_platdata(&smdk5410_qos_mif_pd, sizeof(struct exynos_devfreq_platdata),
			&exynos5_mif_devfreq);

	s3c_set_platdata(&smdk5410_qos_int_pd, sizeof(struct exynos_devfreq_platdata),
			&exynos5_int_devfreq);
#endif

	s3c_set_platdata(&exynos5_tmu_data, sizeof(struct exynos_tmu_platform_data),
			&exynos5410_device_tmu);
	platform_add_devices(tf4_power_devices,
			ARRAY_SIZE(tf4_power_devices));
}
