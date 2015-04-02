/*
 * Copyright (c) 2013 Meizu Tech Co., Ltd.
 *		http://www.meizu.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include <linux/platform_device.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>

#include <asm/mach-types.h>

#include <mach/gpio-meizu.h>
#include "board-m6x.h"

#if defined(CONFIG_BACKLIGHT_LM3630) || defined(CONFIG_BACKLIGHT_LM3695)
static struct regulator_consumer_supply lm3630_fixed_voltage_supplies[] = {
	REGULATOR_SUPPLY("bl-vol", NULL),
};

static struct regulator_init_data lm3630_fixed_voltage_init_data = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled = true,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(lm3630_fixed_voltage_supplies),
	.consumer_supplies		= lm3630_fixed_voltage_supplies,
};

static struct fixed_voltage_config lm3630_fixed_voltage_config = {
	.supply_name	= "bl-fixed",
	.microvolts		=  5000000,
	//.gpio			= MEIZU_LCD_BL_EN,
	.enable_high  = true,
	.init_data		= &lm3630_fixed_voltage_init_data,
};
static struct platform_device lm3630_fixed_voltage = {
	.name	= "reg-fixed-voltage",
	.id		= 4,
	.dev	= {
		.platform_data = &lm3630_fixed_voltage_config,
	},
};
#endif

#ifdef CONFIG_LCD_MIPI_MEIZU
static struct regulator_consumer_supply lcd5v_fixed_voltage_supplies[] = {
	REGULATOR_SUPPLY("lcd-5v", NULL),
};

static struct regulator_init_data lcd5v_fixed_voltage_init_data = {
	.constraints	= {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = 0,
		.state_mem = {
			.disabled = true,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(lcd5v_fixed_voltage_supplies),
	.consumer_supplies		= lcd5v_fixed_voltage_supplies,
};

static struct fixed_voltage_config lcd5v_fixed_voltage_config = {
	.supply_name	= "LCD5V",
	.microvolts		=  5000000,
	//.gpio			= MEIZU_LCD_5VEN,
	.enable_high  = true,
	.init_data		= &lcd5v_fixed_voltage_init_data,
};
static struct platform_device lcd5v_fixed_voltage = {
	.name	= "reg-fixed-voltage",
	.id		= 5,
	.dev	= {
		.platform_data = &lcd5v_fixed_voltage_config,
	},
};

static struct regulator_consumer_supply lcdn5v_fixed_voltage_supplies[] = {
	REGULATOR_SUPPLY("lcd-n5v", NULL),
};

static struct regulator_init_data lcdn5v_fixed_voltage_init_data = {
	.constraints	= {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = 0,
		.state_mem = {
			.disabled = true,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(lcdn5v_fixed_voltage_supplies),
	.consumer_supplies		= lcdn5v_fixed_voltage_supplies,
};

static struct fixed_voltage_config lcdn5v_fixed_voltage_config = {
	.supply_name	= "LCDN5V",
	.microvolts		=  5000000,
	//.gpio			= MEIZU_LCD_N5VEN,
	.enable_high  = true,
	.init_data		= &lcdn5v_fixed_voltage_init_data,
};
static struct platform_device lcdn5v_fixed_voltage = {
	.name	= "reg-fixed-voltage",
	.id		= 6,
	.dev	= {
		.platform_data = &lcdn5v_fixed_voltage_config,
	},
};
#endif

#ifdef CONFIG_VIDEO_EXYNOS_MIPI_CSIS
static struct regulator_consumer_supply mipi_csi_fixed_voltage_supplies[] = {
	REGULATOR_SUPPLY("mipi_csi", "s5p-mipi-csis.0"),
	REGULATOR_SUPPLY("mipi_csi", "s5p-mipi-csis.1"),
	REGULATOR_SUPPLY("mipi_csi", "s5p-mipi-csis.2"),
};

static struct regulator_init_data mipi_csi_fixed_voltage_init_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(mipi_csi_fixed_voltage_supplies),
	.consumer_supplies	= mipi_csi_fixed_voltage_supplies,
};

static struct fixed_voltage_config mipi_csi_fixed_voltage_config = {
	.supply_name	= "DC_5V",
	.microvolts	= 5000000,
	.gpio		= -EINVAL,
	.init_data	= &mipi_csi_fixed_voltage_init_data,
};

static struct platform_device mipi_csi_fixed_voltage = {
	.name		= "reg-fixed-voltage",
	.id		= 7,
	.dev		= {
		.platform_data	= &mipi_csi_fixed_voltage_config,
	},
};
#endif

static struct platform_device *m6x_fixed_voltage_devs[] __initdata = {
#if defined(CONFIG_BACKLIGHT_LM3630) || defined(CONFIG_BACKLIGHT_LM3695)
	&lm3630_fixed_voltage,
#endif
#ifdef CONFIG_LCD_MIPI_MEIZU
	&lcd5v_fixed_voltage,
	&lcdn5v_fixed_voltage,
#endif
#ifdef CONFIG_VIDEO_EXYNOS_MIPI_CSIS
	&mipi_csi_fixed_voltage,
#endif
};

static void __init fixed_regulator_rt_init_res(void) {
#if defined(CONFIG_BACKLIGHT_LM3630) || defined(CONFIG_BACKLIGHT_LM3695)
	lm3630_fixed_voltage_config.gpio = MEIZU_LCD_BL_EN;
#endif
#ifdef CONFIG_LCD_MIPI_MEIZU
	lcd5v_fixed_voltage_config.gpio = MEIZU_LCD_5VEN;
	lcdn5v_fixed_voltage_config.gpio = MEIZU_LCD_N5VEN;
#endif
}

void __init exynos5_m6x_fixed_voltage_init(void)
{
	fixed_regulator_rt_init_res();

	platform_add_devices(m6x_fixed_voltage_devs,
			ARRAY_SIZE(m6x_fixed_voltage_devs));
}
