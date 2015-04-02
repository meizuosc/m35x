/*
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/arizona/pdata.h>
#include <linux/mfd/arizona/registers.h>
#include <linux/input.h>
#include <linux/i2c-gpio.h>
#include <mach/hardware.h>

#include <asm/mach-types.h>

#include <plat/gpio-cfg.h>
#include <plat/devs.h>
#include <plat/iic.h>
#include <plat/s3c64xx-spi.h>

#include <mach/gpio-i2c-m6x.h>
#include <mach/gpio-meizu.h>
#include <mach/spi-clocks.h>

#ifdef CONFIG_AUDIENCE_ES305B
#include <linux/es305b_soc.h>
#endif
#include "board-m6x.h"

static struct regulator_consumer_supply wm5102_fixed_voltage_supplies[] = {
	REGULATOR_SUPPLY("SPKVDD", NULL),
	REGULATOR_SUPPLY("SPKVDDL", NULL),
	REGULATOR_SUPPLY("SPKVDDR", NULL),
};

static struct regulator_init_data wm5102_fixed_voltage_init_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(wm5102_fixed_voltage_supplies),
	.consumer_supplies	= wm5102_fixed_voltage_supplies,
};

static struct fixed_voltage_config wm5102_fixed_voltage_config = {
	.supply_name	= "DC_5V",
	.microvolts	= 5000000,
	.gpio		= -EINVAL,
	.init_data	= &wm5102_fixed_voltage_init_data,
};

static struct platform_device wm5102_fixed_voltage_device = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev		= {
		.platform_data	= &wm5102_fixed_voltage_config,
	},
};

static const struct arizona_micd_config micd_default_modes[] = {
	{ 0, 2 << ARIZONA_MICD_BIAS_SRC_SHIFT, 0 },
};


/* This rangs for the resistor R306 present */
static const struct arizona_micd_range m65_version_1_rangs[] = {
	{ .max =   11, .key = KEY_MEDIA},
	{ .max =   73, .key = KEY_UNKNOWN},
	{ .max =  186, .key = KEY_VOLUMEUP},
	{ .max =  220, .key = KEY_UNKNOWN},
	{ .max =  321, .key = KEY_VOLUMEDOWN},
	{ .max =  752, .key = KEY_UNKNOWN},
	{ .max =  752, .key = KEY_UNKNOWN},
	{ .max =  752, .key = KEY_UNKNOWN},
};

/* This rangs for the resistor R306 absent */
static const struct arizona_micd_range m65_version_2_rangs[] = {
	{ .max =  11, .key = KEY_MEDIA},
	{ .max =  73, .key = KEY_UNKNOWN},
	{ .max = 220, .key = KEY_VOLUMEUP},
	{ .max = 270, .key = KEY_UNKNOWN},
	{ .max = 430, .key = KEY_VOLUMEDOWN},
	{ .max = 752, .key = KEY_UNKNOWN},
};

#define CODEC_GPIO_BASE	(S3C_GPIO_END + 8)
static struct arizona_pdata wm5102_pdata = {
	// .micvdd = &wm5102_micvdd_data,
	// .ldo1 = &wm5102_ldo1_data, // using default
	.gpio_base = CODEC_GPIO_BASE,
	// .irq_active_high = true,
	.micd_detect_debounce = 500,
	.micd_bias_start_time = 9,
	.micd_rate = 5,
	.micd_dbtime = 1,
	.micd_force_micbias = 1,
	.micd_configs = micd_default_modes,
	.num_micd_configs = ARRAY_SIZE(micd_default_modes),
    .irq_base = IRQ_BOARD_START + 0x60, /* After SEC PMIC */
	// .micd_pol_gpio = CODEC_GPIO_BASE + 4,
#ifdef CONFIG_MEIZU_M65_V31
	.clk32k_src = ARIZONA_32KZ_MCLK2, // for phone v2
#else
	.clk32k_src = ARIZONA_32KZ_NONE, // for phone v1
#endif
	.micbias = {
		[0] = {.bypass = 0},
		[1] = {.bypass = 0},
		[2] = {.bypass = 0},
	},
};

static void wm5102_set_micd_ranges(void)
{
	int board_version = m6x_get_board_version();

	if (machine_is_m65() && (board_version == 0 || board_version == 1)) {
		wm5102_pdata.micd_ranges = m65_version_1_rangs;
		wm5102_pdata.num_micd_ranges = ARRAY_SIZE(m65_version_1_rangs);
	} else {
		wm5102_pdata.micd_ranges = m65_version_2_rangs;
		wm5102_pdata.num_micd_ranges = ARRAY_SIZE(m65_version_2_rangs);
	}
}

static struct s3c64xx_spi_csinfo wm5102_spi1_csinfo[] = {
	[0] = {
		.fb_delay	= 0x8,
		.line		= EXYNOS5410_GPA2(5),
		.set_level	= gpio_set_value,
	},
};

#define CODEC_EINT   IRQ_EINT(31)
static struct spi_board_info wm5102_spi1_info[] __initdata = {
	{
		.modalias = "wm5102",
		.platform_data = &wm5102_pdata,
		.controller_data = &wm5102_spi1_csinfo[0],
		.irq		= CODEC_EINT,
		.max_speed_hz = 20 * 1000 * 1000,
		.bus_num = 1,
		.chip_select = 0,
		.mode = SPI_MODE_0,
	}
};

#ifdef CONFIG_AUDIENCE_ES305B
static struct es305b_platform_data __initdata es305b_pd = {
	//.gpio_es305b_wake	= MEIZU_NOISE_CANCELLER_WAKE,
	//.gpio_es305b_reset	= MEIZU_NOISE_CANCELLER_RST,
};
#endif

// #define CODEC_EINT   IRQ_EINT(31)
static struct i2c_board_info __initdata i2c_devs1[] = {
#ifdef CONFIG_MFD_ARIZONA_I2C
	{
		I2C_BOARD_INFO("wm5102", (0x34 >> 1)),
		.platform_data	= &wm5102_pdata,
		.irq		= CODEC_EINT,
	},
#endif
#ifdef CONFIG_AUDIENCE_ES305B
	{
		I2C_BOARD_INFO(ES305B_I2C_NAME, ES305B_I2S_SLAVE_ADDRESS),
		.platform_data	= &es305b_pd,
	},
#endif
};

static struct i2c_board_info __initdata i2c_devs19[] = {
#ifdef CONFIG_AUDIENCE_ES305B
	{
		I2C_BOARD_INFO(ES305B_I2C_NAME, ES305B_I2S_SLAVE_ADDRESS),
		.platform_data	= &es305b_pd,
	},
#endif
};

#ifdef CONFIG_PA_TFA9887
static struct i2c_board_info __initdata i2c_devs_tfa9887[]  = {
	{
		I2C_BOARD_INFO("tfa9887", 0x68>>1),
		.platform_data = NULL,
	},
};
#endif

#ifdef CONFIG_SND_SOC_BELLS
static struct platform_device bells_wm5102_device = {
	.name		= "bells",
	.id		= -1,
};
#endif
#ifdef CONFIG_SND_SOC_MEIZU_M6X_WM5102
static struct platform_device m6x_audio_device = {
	.name = "m6x-audio",
	.id = -1,
};
#endif

static struct platform_device *m6x_audio_devices[] __initdata = {
#ifdef CONFIG_SND_SAMSUNG_I2S
	&exynos5_device_i2s0,
#endif
#ifdef CONFIG_SND_SAMSUNG_PCM
	&exynos5_device_pcm0,
#endif
#ifdef CONFIG_SND_SAMSUNG_SPDIF
	&exynos5_device_spdif,
#endif
#if defined(CONFIG_SND_SAMSUNG_RP) || defined(CONFIG_SND_SAMSUNG_ALP)
	&exynos5_device_srp,
#endif
#ifdef CONFIG_SND_SOC_MEIZU_M6X_WM5102
#ifdef CONFIG_MFD_ARIZONA_SPI
	&s3c64xx_device_spi1,		//dognatas
#endif
	&wm5102_fixed_voltage_device,
	&m6x_audio_device,
#endif
#ifdef CONFIG_SND_SOC_BELLS
	&bells_wm5102_device,
#endif

	&samsung_asoc_dma,
	&samsung_asoc_idma,
};

#if defined(CONFIG_SND_SOC_MEIZU_M6X_WM5102) && defined(CONFIG_AUDIENCE_ES305B)
#ifdef CONFIG_MACH_M65
static void __init m65_es305b_i2c_bus_init(void) {
	int ret = 0;

	ret = platform_device_register(&s3c_device_i2c1);
	if (ret) {
		platform_device_unregister(&s3c_device_i2c1);
		pr_err("%s: Error register i2c1!\n", __func__);
	}
}

static void __init m65_es305b_i2c_board_init(void) {
	s3c_i2c1_set_platdata(NULL);
	es305b_pd.gpio_es305b_wake	= MEIZU_NOISE_CANCELLER_WAKE;
	es305b_pd.gpio_es305b_reset	= MEIZU_NOISE_CANCELLER_RST;
	i2c_register_board_info(1, i2c_devs1, ARRAY_SIZE(i2c_devs1));
}
#endif

#ifdef CONFIG_MACH_M69
static void __init m69_es305b_i2c_bus_init(void) {
	int ret = 0;

	ret = platform_device_register(&m6x_device_i2c19);
	if (ret) {
		platform_device_unregister(&m6x_device_i2c19);
		pr_err("%s: Error register i2c19!\n", __func__);
	}
}

static void __init m69_es305b_i2c_board_init(void) {
	es305b_pd.gpio_es305b_wake	= MEIZU_NOISE_CANCELLER_WAKE;
	es305b_pd.gpio_es305b_reset	= MEIZU_NOISE_CANCELLER_RST;
	i2c_register_board_info(19, i2c_devs19, ARRAY_SIZE(i2c_devs19));
}
#endif
#endif

#ifdef CONFIG_MFD_ARIZONA_SPI
static void mfd_spi_rt_init_res(void) {
	wm5102_pdata.reset = MEIZU_CODEC_RST;
	wm5102_pdata.ldoena = MEIZU_CODEC_AUEN;
}
#endif

void __init exynos5_m6x_audio_init(void)
{
#if defined(CONFIG_SND_SOC_MEIZU_M6X_WM5102) && defined(CONFIG_AUDIENCE_ES305B)
	if (machine_is_m65()) {
		m65_es305b_i2c_bus_init();
	} else if (machine_is_m69()) {
		m69_es305b_i2c_bus_init();
	} else {
		pr_err("Error: Please Configure Machine Version.\r\n");
		BUG();
	}
#endif
	
	platform_add_devices(m6x_audio_devices, ARRAY_SIZE(m6x_audio_devices));

#if defined(CONFIG_SND_SOC_MEIZU_M6X_WM5102) && defined(CONFIG_AUDIENCE_ES305B)
	if (machine_is_m65()) {
		m65_es305b_i2c_board_init();
	} else if (machine_is_m69()) {
		m69_es305b_i2c_board_init();
	} else {
		pr_err("Error: Please Configure Machine Version.\r\n");
		BUG();
	}
#endif

#ifdef CONFIG_MFD_ARIZONA_SPI
	exynos_spi_clock_setup(&s3c64xx_device_spi1.dev, 1);
	if (!exynos_spi_cfg_cs(wm5102_spi1_csinfo[0].line, 1)) {
		s3c64xx_spi1_set_platdata(&s3c64xx_spi1_pdata,
			EXYNOS_SPI_SRCCLK_SCLK, ARRAY_SIZE(wm5102_spi1_csinfo));

		mfd_spi_rt_init_res();
		wm5102_set_micd_ranges();
		
		spi_register_board_info(wm5102_spi1_info, ARRAY_SIZE(wm5102_spi1_info));
	} else {
		pr_err("%s: Error requesting gpio for SPI-CH1 CS\n", __func__);
	}
#endif
#ifdef CONFIG_PA_TFA9887
	i2c_register_board_info(11, i2c_devs_tfa9887, ARRAY_SIZE(i2c_devs_tfa9887));
#endif
}
