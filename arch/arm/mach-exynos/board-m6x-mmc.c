/*
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/gpio.h>
#include <linux/smsc911x.h>
#include <linux/mmc/host.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/clkdev.h>

#include <plat/gpio-cfg.h>
#include <plat/cpu.h>
#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/sdhci.h>

#include <mach/dwmci.h>
#include <linux/bootmode.h>

#include "board-m6x.h"

static struct dw_mci_clk exynos_dwmci_clk_rates[] = {
	{25 * 1000 * 1000, 100 * 1000 * 1000},
	{40 * 1000 * 1000, 160 * 1000 * 1000},
	{40 * 1000 * 1000, 160 * 1000 * 1000},
	{80 * 1000 * 1000, 320 * 1000 * 1000},
	{160 * 1000 * 1000, 640 * 1000 * 1000},
	{100 * 1000 * 1000, 400 * 1000 * 1000},
	{160 * 1000 * 1000, 640 * 1000 * 1000},
	{320 * 1000 * 1000, 640 * 1000 * 1000},
};

/*
 * see board-m6x-clock.c about mmc
 * change clk sclk_dwci1 parent to clk mout_epll(400MHz)
 */
static struct dw_mci_clk exynos_dwmci_clk_rates_for_wifi[] = {
	{25 * 1000 * 1000, 50 * 1000 * 1000},
	{50 * 1000 * 1000, 100 * 1000 * 1000},
	{50 * 1000 * 1000, 100 * 1000 * 1000},
	{100 * 1000 * 1000, 200 * 1000 * 1000},
	{200 * 1000 * 1000, 400 * 1000 * 1000},
	{100 * 1000 * 1000, 200 * 1000 * 1000},
	{200 * 1000 * 1000, 400 * 1000 * 1000},
	{200 * 1000 * 1000, 400 * 1000 * 1000},
};

static int exynos_dwmci0_get_bus_wd(u32 slot_id)
{
	return 8;
}

static void exynos_dwmci0_cfg_gpio(int width)
{
	unsigned int gpio;

	for (gpio = EXYNOS5410_GPC0(0);
			gpio < EXYNOS5410_GPC0(2); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV4);
	}

	switch (width) {
	case 8:
		for (gpio = EXYNOS5410_GPC3(0);
				gpio <= EXYNOS5410_GPC3(3); gpio++) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
			s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV4);
		}
	case 4:
		for (gpio = EXYNOS5410_GPC0(3);
				gpio <= EXYNOS5410_GPC0(6); gpio++) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
			s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV4);
		}
		break;
	case 1:
		gpio = EXYNOS5410_GPC0(3);
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV4);
	default:
		break;
	}

	gpio = EXYNOS5410_GPD1(0);
	s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
	s3c_gpio_setpull(gpio, S3C_GPIO_PULL_DOWN);
}

#ifdef CONFIG_ARM_EXYNOS5410_BUS_DEVFREQ
static struct dw_mci_mon_table exynos_dwmci_tp_mon0_tbl[] = {
	/* Byte/s,	MIF clk,	CPU clk*/
	{ 20000000,	800000,		1200000},	/* 20 MB / s */
#if 0
	{  2000000,	800000,		1000000},	/* 2 MB / s */
	{  1000000,	800000,		550000},	/* 1 MB / s */
	{   500000,	400000,		400000},	/* 0.5 MB / s */
	{   100000,	200000,	        200000},	/* 0.1 MB / s */
#endif
	{       0,	     0,		     0},
};
#endif

static struct dw_mci_board m6x_dwmci0_pdata __initdata = {
	.num_slots		= 1,
	.quirks			= DW_MCI_QUIRK_BROKEN_CARD_DETECTION |
				  DW_MCI_QUIRK_HIGHSPEED |
				  DW_MCI_QUIRK_NO_DETECT_EBIT,
	.bus_hz			= 160 * 1000 * 1000,
	.caps			= MMC_CAP_CMD23 | MMC_CAP_8_BIT_DATA |
				  MMC_CAP_UHS_DDR50 | MMC_CAP_1_8V_DDR |
				  MMC_CAP_ERASE,
	.caps2			= MMC_CAP2_HS200_1_8V_SDR | MMC_CAP2_HS200_1_8V_DDR,
	.fifo_depth		= 0x80,
	.detect_delay_ms	= 200,
	.hclk_name		= "dwmci",
	.cclk_name		= "sclk_dwmci",
	.cfg_gpio		= exynos_dwmci0_cfg_gpio,
	.get_bus_wd		= exynos_dwmci0_get_bus_wd,
	.sdr_timing		= 0x03040000,
	.ddr_timing		= 0x03020000,
	.clk_drv		= 0x3,
	.ddr200_timing		= 0x01020000,
	.clk_tbl		= exynos_dwmci_clk_rates,
#ifdef CONFIG_ARM_EXYNOS5410_BUS_DEVFREQ
	.tp_mon_tbl		= exynos_dwmci_tp_mon0_tbl,
#endif
};

static int exynos_dwmci1_get_bus_wd(u32 slot_id)
{
	return 8;
}

static void exynos_dwmci1_cfg_gpio(int width)
{
	unsigned int gpio;

	for (gpio = EXYNOS5410_GPC1(0); gpio < EXYNOS5410_GPC1(3); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		if (gpio == EXYNOS5410_GPC1(0))
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
		else
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV4);
	}

	gpio = EXYNOS5410_GPD1(1);
	s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
	s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
	s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV4);

	switch (width) {
	case 8:
	case 4:
		for (gpio = EXYNOS5410_GPC1(3);
				gpio <= EXYNOS5410_GPC1(6); gpio++) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
			s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV4);
		}

		for (gpio = EXYNOS5410_GPD1(4);
				gpio <= EXYNOS5410_GPD1(7); gpio++) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
			s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV4);
		}

		break;
	case 1:
		gpio = EXYNOS5410_GPC1(3);
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV4);
	default:
		break;
	}
}

typedef void (*notify_func)(struct platform_device *dev, int state);
static notify_func dwmci1_notify_func;

static int ext_cd_init_dwmci1(notify_func func)
{
	dwmci1_notify_func = func;
	return 0;
}

static int ext_cd_cleanup_dwmci1(notify_func func)
{
	dwmci1_notify_func = NULL;
	return 0;
}

#ifdef CONFIG_ARM_EXYNOS5410_BUS_DEVFREQ
static struct dw_mci_mon_table exynos_dwmci_tp_mon1_tbl[] = {
	/* Byte/s,      MIF clk,        CPU clk*/
	{ 4000000,      800000,        1200000},
	{ 2000000,      400000,              0},
	{       0,           0,              0},
};
#endif

static struct dw_mci_board m6x_dwmci1_pdata __initdata = {
	.num_slots		= 1,
	.quirks			= DW_MCI_QUIRK_HIGHSPEED,
	.bus_hz			= 200 * 1000 * 1000,
	.caps			= MMC_CAP_4_BIT_DATA | MMC_CAP_SDIO_IRQ |
						MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED,
	.pm_caps		= MMC_PM_KEEP_POWER | MMC_PM_IGNORE_PM_NOTIFY,
	.fifo_depth		= 0x80,
	.detect_delay_ms	= 200,
	.hclk_name		= "dwmci",
	.cclk_name		= "sclk_dwmci",
	.cfg_gpio		= exynos_dwmci1_cfg_gpio,
	.get_bus_wd		= exynos_dwmci1_get_bus_wd,
	.sdr_timing		= 0x81040001,
	.ddr_timing		= 0x81020000,
	.clk_tbl                = exynos_dwmci_clk_rates_for_wifi,

	.cd_type = DW_MCI_CD_EXTERNAL,
	.ext_cd_init = ext_cd_init_dwmci1,
	.ext_cd_cleanup = ext_cd_cleanup_dwmci1,
#ifdef CONFIG_ARM_EXYNOS5410_BUS_DEVFREQ
	.tp_mon_tbl		= exynos_dwmci_tp_mon1_tbl,
#endif
};

/*
 * call this when you need sd stack to recognize insertion or removal of card
 * that can't be told by SDHCI regs
 */
void mmc_force_presence_change(struct platform_device *pdev, int val)
{
	notify_func func = dwmci1_notify_func;

	if (pdev == &exynos5_device_dwmci1 && func)
		func(pdev, val);
	else
		pr_warn("%s: called for device with no notifier\n", __func__);
}

static struct platform_device *m6x_mmc_devices[] __initdata = {
#ifdef CONFIG_MMC_DW
	&exynos5_device_dwmci0,
	&exynos5_device_dwmci1,
#endif
};

/* Although we don't use mmc2, but iROM will access it, provide two helpers to
 * enable hclk while enter into LPA or sleep */
static struct clk *hclk_mmc2;

void enable_mmc2_hclk(void)
{
	clk_enable(hclk_mmc2);
}

void disable_mmc2_hclk(void)
{
	clk_disable(hclk_mmc2);
}

void __init exynos5_m6x_mmc_init(void)
{
#ifdef CONFIG_MMC_DW
	if (samsung_rev() < EXYNOS5410_REV_1_0)
		m6x_dwmci0_pdata.caps &=
			~(MMC_CAP_UHS_DDR50 | MMC_CAP_1_8V_DDR);
#ifndef CONFIG_EXYNOS_EMMC_HS200
	m6x_dwmci0_pdata.caps2 &=
		~MMC_CAP2_HS200_1_8V_SDR;
#endif
	hclk_mmc2 = clk_get_sys("dw_mmc.2", "dwmci");

	enable_mmc2_hclk();

	exynos_dwmci_set_platdata(&m6x_dwmci0_pdata, 0);
	exynos_dwmci_set_platdata(&m6x_dwmci1_pdata, 1);
#endif
	platform_add_devices(m6x_mmc_devices,
			ARRAY_SIZE(m6x_mmc_devices));
}
