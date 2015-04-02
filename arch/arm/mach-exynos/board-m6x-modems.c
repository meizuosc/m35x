/**
 * arch/arm/mach-exynos/board-m6x-modems.c
 *
 * Copyright (C) 2010 Samsung Electronics.
 * Copyright (C) 2012 Zhuhai Meizu Inc.
 *
 * Original version by Samsung Electronics
 * Re-written by KarlZheng<zhengkl@meizu.com>
 * Maintained by Liu Jianping<heljoy@meizu.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <plat/devs.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/platform_data/modem.h>
#include <linux/bootmode.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <asm/mach-types.h>
#include <plat/s3c64xx-spi.h>

#include <mach/hardware.h>
#include <mach/gpio-meizu.h>
#include <mach/spi-clocks.h>
#ifdef CONFIG_MODEM_SC8803G
#include <mach/modem_sc8803g.h>
#endif

extern struct modem_data xmm_modem_data;

static struct platform_device umts_modem_device = {
	.name = "meizu_modem",
	.id = -1,
};

static void __init meizu_modem_set_platdata(void)
{
	if (meizu_board_mdm_type() == MEIZU_MDM_XMM6260) {
		umts_modem_device.name = "modem_ifx_6260";
		s3c_set_platdata(&xmm_modem_data,
			sizeof(struct modem_data), &umts_modem_device);
	}
}

#ifdef CONFIG_MODEM_SC8803G
static struct spt_modem_platform_data default_mdm_data = {
	.modem_type = SPT_MODEM_SC8803G,
	.mode		= SPI_MODE_3,
	.bit_per_word	= 32,
	.max_hz = 12*1000*1000,
	.use_dma = 1,
};

#ifdef CONFIG_MODEM_SC8803G_CONTROL
static struct platform_device m69_mdm_device = {
	.name		  = "sc880xg-modem",
	.id		  = -1,
	.dev = {
		.platform_data = &default_mdm_data,
	},
};
#endif // CONFIG_MODEM_SC8803G_CONTROL

static struct s3c64xx_spi_csinfo spi2_csi[] = {
	[0] = {
		.line = EXYNOS5410_GPB1(2),
		.set_level = gpio_set_value,
		.fb_delay = 0x1,
	},
};

static struct spi_board_info spi2_board_info[] __initdata = {
	{
		.modalias = "sc880xg-spi",
		.platform_data = &default_mdm_data,
		.max_speed_hz = 12*1000*1000,
		.bus_num = 2,
		.chip_select = 0,
		.mode = SPI_MODE_1,
		.data_not_swap = 1,
		.controller_data = &spi2_csi[0],
	}
};
#endif // CONFIG_MODEM_SC8803G

static int __init modem_device_init(void)
{
	int ret = 0;

	if (is_charging_mode())
		return ret;

	meizu_modem_set_platdata();
	ret = platform_device_register(&umts_modem_device);

	pr_info("[MODEM_IF] init_modem device over(ret=%d)\n", ret);

	return ret;
}

extern int modem_reset_init(void);
static int __init early_xmm_power(void)
{
	if (meizu_board_mdm_type() == MEIZU_MDM_XMM6260)
		return modem_reset_init();

	return 0;
}

#ifdef CONFIG_MODEM_SC8803G
static void __init sc8803g_rt_init_res(void) {
	default_mdm_data.pwr_on = MEIZU_GPIO_MODEM_POWER_ON;

	default_mdm_data.srdy = MEIZU_GPIO_MODEM_RDY;
	default_mdm_data.mrdy = MEIZU_GPIO_AP_RDY;
	default_mdm_data.srts = MEIZU_GPIO_MODEM_RTS;
	default_mdm_data.mrts = MEIZU_GPIO_AP_RTS;
	default_mdm_data.srsd = MEIZU_GPIO_MODEM_RESEND;
	default_mdm_data.mrsd = MEIZU_GPIO_AP_RESEND;
	default_mdm_data.salive = MEIZU_GPIO_MODEM_ALIVE;

	default_mdm_data.s2m1 = MEIZU_GPIO_MODEM_TO_AP1;
	default_mdm_data.s2m2 = MEIZU_GPIO_MODEM_TO_AP2;
	default_mdm_data.m2s1 = MEIZU_GPIO_AP_TO_MODEM1;
	default_mdm_data.m2s2 = MEIZU_GPIO_AP_TO_MODEM2;
}

static int  __init m6x_sc8803g_init(void) {
	int ret = 0;

	if (machine_is_m65())
		return 0;

	if (is_charging_mode())
		return 0;

	sc8803g_rt_init_res();

#ifdef CONFIG_MODEM_SC8803G_CONTROL
	ret = platform_device_register(&m69_mdm_device);
	if (ret) {
		platform_device_unregister(&m69_mdm_device);
		pr_err("%s: Error register sc8803g modem device!\n", __func__);
		return -EINVAL;
	}
#endif

	ret = platform_device_register(&s3c64xx_device_spi2);
	if (ret) {
		platform_device_unregister(&s3c64xx_device_spi2);
		pr_err("%s: Error register spi2!\n", __func__);
		return -EINVAL;
	}

	exynos_spi_clock_setup(&s3c64xx_device_spi2.dev, 2);
	if (!exynos_spi_cfg_cs(spi2_csi[0].line, 2)) {
		s3c64xx_spi2_set_platdata(&s3c64xx_spi2_pdata,
			EXYNOS_SPI_SRCCLK_SCLK, ARRAY_SIZE(spi2_csi));

		spi_register_board_info(spi2_board_info,
			ARRAY_SIZE(spi2_board_info));
	} else {
		pr_err("%s: Error requesting gpio for SPI-CH2 CS\n", __func__);
		return -EINVAL;
	}

	pr_info("[MODEM_IF] init modem device over\n");

	return 0;
}

arch_initcall(m6x_sc8803g_init);
#endif

device_initcall(early_xmm_power);
late_initcall(modem_device_init);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("KarlZheng<zhengkl@meizu.com>, Liu Jianping<heljoy@meizu.com>");
MODULE_DESCRIPTION("Meizu Modem Interface Driver");
