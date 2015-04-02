/* linux/arch/arm/mach-exynos/board-tf4-audio.c
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/delay.h>
#include <mach/irqs.h>
#include <mach/pmu.h>

#ifdef CONFIG_SND_SOC_WM8994
#include <linux/mfd/wm8994/pdata.h>
#include <linux/mfd/wm8994/gpio.h>
#endif

#include <plat/gpio-cfg.h>
#include <plat/devs.h>
#include <plat/iic.h>

#include "board-tf4.h"

#ifdef CONFIG_SND_SOC_WM8994
/* PVDD_AUDIO */
static struct regulator_consumer_supply wm1811_pvdd_audio_supplies[] = {
	REGULATOR_SUPPLY("AVDD2", "10-001a"),
	REGULATOR_SUPPLY("CPVDD", "10-001a"),
	REGULATOR_SUPPLY("DBVDD1", "10-001a"),
	REGULATOR_SUPPLY("DBVDD2", "10-001a"),
	REGULATOR_SUPPLY("DBVDD3", "10-001a"),
};

static struct regulator_init_data wm1811_pvdd_audio_init_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies = ARRAY_SIZE(wm1811_pvdd_audio_supplies),
	.consumer_supplies = wm1811_pvdd_audio_supplies,
};

static struct fixed_voltage_config wm1811_pvdd_audio_config = {
	.supply_name = "PVDD_AUDIO",
	.microvolts = 1800000,
	.gpio = -EINVAL,
	.init_data = &wm1811_pvdd_audio_init_data,
};

static struct platform_device wm1811_pvdd_audio_device = {
	.name = "reg-fixed-voltage",
	.id = 0,
	.dev = {
		.platform_data	= &wm1811_pvdd_audio_config,
	},
};

/* VBAT */
static struct regulator_consumer_supply wm1811_vbat_supplies[] = {
	REGULATOR_SUPPLY("SPKVDD1", "10-001a"),
	REGULATOR_SUPPLY("SPKVDD2", "10-001a"),
};

static struct regulator_init_data wm1811_vbat_init_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies = ARRAY_SIZE(wm1811_vbat_supplies),
	.consumer_supplies = wm1811_vbat_supplies,
};

static struct fixed_voltage_config wm1811_vbat_config = {
	.supply_name = "VBAT",
	.microvolts = 5000000,
	.gpio = -EINVAL,
	.init_data = &wm1811_vbat_init_data,
};

static struct platform_device wm1811_vbat_device = {
	.name = "reg-fixed-voltage",
	.id = 1,
	.dev = {
		.platform_data = &wm1811_vbat_config,
	},
};

/* WM1811 LDO1 */
static struct regulator_consumer_supply wm1811_ldo1_supplies[] = {
	REGULATOR_SUPPLY("AVDD1", "10-001a"),
};

static struct regulator_init_data wm1811_ldo1_init_data = {
	.constraints = {
		.name = "WM1811 LDO1",
        .valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(wm1811_ldo1_supplies),
	.consumer_supplies = wm1811_ldo1_supplies,
};

/* WM1811 LDO2 */
static struct regulator_consumer_supply wm1811_ldo2_supplies[] = {
	REGULATOR_SUPPLY("DCVDD", "10-001a"),
};

static struct regulator_init_data wm1811_ldo2_init_data = {
	.constraints = {
		.name = "WM1811 LDO2",
		.always_on = 1,     /* Practically status is changed by LDO1 */
	},
	.num_consumer_supplies = ARRAY_SIZE(wm1811_ldo2_supplies),
	.consumer_supplies = wm1811_ldo2_supplies,
};

static struct wm8994_pdata wm1811_platform_data = {
	/* GPIO1 - IRQ output */
	.gpio_defaults[0] = WM8994_GP_FN_IRQ,
	/* If the i2s0 and i2s2 is enabled simultaneously */
	.gpio_defaults[7] = 0x8100,     /* GPIO8  DACDAT3 in */
	.gpio_defaults[8] = 0x0100,     /* GPIO9  ADCDAT3 out */
	.gpio_defaults[9] = 0x0100,     /* GPIO10 LRCLK3  out */
	.gpio_defaults[10] = 0x0100,    /* GPIO11 BCLK3   out */

	.ldo[0] = {
        .enable = EXYNOS5410_GPG1(0),  /* CODEC_LDO_EN */
        .init_data = &wm1811_ldo1_init_data
    },
	.ldo[1] = {
	    .enable = 0,
        .init_data = &wm1811_ldo2_init_data
    },

    .irq_base = IRQ_BOARD_START + 0x10, /* After SEC PMIC */

    /* Support external capacitors */
    .jd_ext_cap = 1,

    /* Regulated mode at highest output voltage */
    .micbias = {0x2f, 0x27},

    .micd_lvl_sel = 0xff,

    .ldo_ena_always_driven = 1,
};

/* I2C - GPIO */
static struct i2c_gpio_platform_data i2c_wm1811_platdata = {
	.sda_pin = EXYNOS5410_GPG1(7),	/* AUDIO_SDA */
	.scl_pin = EXYNOS5410_GPG1(6),	/* AUDIO_SCL */
	.udelay = 2,
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.scl_is_output_only = 0,
};

static struct platform_device s3c_device_i2c_wm1811 = {
	.name = "i2c-gpio",
	.id = 10,
	.dev.platform_data = &i2c_wm1811_platdata,
};

static struct i2c_board_info i2c_devs_wm1811[] __initdata = {
	{
		I2C_BOARD_INFO("wm1811", (0x34 >> 1)),  /* 0x1a */
		.platform_data  = &wm1811_platform_data,
		.irq = IRQ_EINT(11),    /* CODEC_INT */
	},
};

#endif  // CONFIG_SND_SOC_WM8994

static struct platform_device *tf4_audio_devices[] __initdata = {
#ifdef CONFIG_SND_SAMSUNG_I2S
	&exynos5_device_i2s0,
#endif
#ifdef CONFIG_SND_SAMSUNG_PCM
	&exynos5_device_pcm0,
#endif
#ifdef CONFIG_SND_SOC_SAMSUNG_TF4_MODEM
	&exynos5_device_pcm1,
#endif
#ifdef CONFIG_SND_SAMSUNG_SPDIF
	&exynos5_device_spdif,
#endif
#if defined(CONFIG_SND_SAMSUNG_RP) || defined(CONFIG_SND_SAMSUNG_ALP)
	&exynos5_device_srp,
#endif
#ifdef CONFIG_SND_SOC_WM8994
	&s3c_device_i2c_wm1811,
	&wm1811_pvdd_audio_device,
	&wm1811_vbat_device,
#endif
	&samsung_asoc_dma,
	&samsung_asoc_idma,
};

void __init exynos5_tf4_audio_init(void)
{
#ifdef CONFIG_SND_SOC_WM8994
	i2c_register_board_info(10, i2c_devs_wm1811, ARRAY_SIZE(i2c_devs_wm1811));
#endif

	platform_add_devices(tf4_audio_devices,
			ARRAY_SIZE(tf4_audio_devices));
}

