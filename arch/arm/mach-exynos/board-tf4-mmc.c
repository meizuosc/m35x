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

#include <plat/gpio-cfg.h>
#include <plat/cpu.h>
#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/sdhci.h>

#include <mach/dwmci.h>

#include "board-tf4.h"

static struct dw_mci_clk exynos_dwmci_clk_rates[] = {
	{20 * 1000 * 1000, 80 * 1000 * 1000},
	{40 * 1000 * 1000, 160 * 1000 * 1000},
	{40 * 1000 * 1000, 160 * 1000 * 1000},
	{80 * 1000 * 1000, 320 * 1000 * 1000},
	{160 * 1000 * 1000, 640 * 1000 * 1000},
	{80 * 1000 * 1000, 320 * 1000 * 1000},
	{160 * 1000 * 1000, 640 * 1000 * 1000},
	{320 * 1000 * 1000, 640 * 1000 * 1000},
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

static struct dw_mci_board smdk5410_dwmci0_pdata __initdata = {
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

static struct dw_mci_board smdk5410_dwmci1_pdata __initdata = {
	.num_slots		= 1,
	.quirks			= DW_MCI_QUIRK_HIGHSPEED,
	.bus_hz			= 160 * 1000 * 1000,
	.caps			= MMC_CAP_UHS_SDR50 | MMC_CAP_UHS_DDR50 |
				  MMC_CAP_1_8V_DDR | MMC_CAP_8_BIT_DATA |
				  MMC_CAP_SDIO_IRQ,
	.fifo_depth		= 0x80,
	.detect_delay_ms	= 200,
	.hclk_name		= "dwmci",
	.cclk_name		= "sclk_dwmci",
	.cfg_gpio		= exynos_dwmci1_cfg_gpio,
	.get_bus_wd		= exynos_dwmci1_get_bus_wd,
	.sdr_timing		= 0x03040000,
	.ddr_timing		= 0x03020000,
	.clk_tbl                = exynos_dwmci_clk_rates,
};
#define MMC2_CD_GPIO (EXYNOS5410_GPX1(6))
static void exynos_dwmci2_cfg_gpio(int width)
{
	unsigned int gpio;

	/* set to pull up pin for write protection */
	gpio = EXYNOS5410_GPM5(0);
	s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
	s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV4);

	for (gpio = EXYNOS5410_GPC2(0); gpio < EXYNOS5410_GPC2(2); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV4);
	}

	switch (width) {
	case 4:
		for (gpio = EXYNOS5410_GPC2(3);
				gpio <= EXYNOS5410_GPC2(6); gpio++) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
			s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV4);
		}
		break;
	case 1:
		gpio = EXYNOS5410_GPC2(3);
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV4);
	default:
		break;
	}

	gpio = EXYNOS5410_GPC2(2);
	s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
	s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV4);
}

static int exynos_dwmci2_get_bus_wd(u32 slot_id)
{
	if (samsung_rev() < EXYNOS5410_REV_1_0)
		return 1;
	else
		return 4;
}

static int exynos_dwmci_get_cd(u32 slot_id)
{
	return gpio_get_value(MMC2_CD_GPIO);
}

static int __init exynos_dwmci2_init(u32 slot_id, irq_handler_t handler, void *data)
{
	struct dw_mci *host = (struct dw_mci *) data;
	struct dw_mci_board *pdata = host->pdata;
	struct device *dev = &host->dev;
	unsigned int gpio = pdata->ext_cd_gpio;
	if (pdata->cd_type == DW_MCI_CD_GPIO && gpio_is_valid(gpio)) {
		if (gpio_request(gpio, "DWMCI CD GPIO") == 0) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(0xf));
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
			s5p_register_gpio_interrupt(gpio);
			host->ext_cd_irq = gpio_to_irq(gpio);
			if (host->ext_cd_irq &&
				request_threaded_irq(gpio_to_irq(host->pdata->ext_cd_gpio), NULL,
					handler,
					IRQF_TRIGGER_RISING |
					IRQF_TRIGGER_FALLING,
					dev_name(dev), host) == 0) {
				dev_warn(dev,
					"success to request "
					"irq for card detect.i %#x, %#x, %#x, %s, %#x \n",
						host->ext_cd_irq, NULL ,handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
							dev_name(dev), host);
			} else {
				dev_warn(dev,
					"cannot request irq "
					"for card detect.\n");
				host->ext_cd_irq = 0;
			}
			gpio_free(gpio);
		} else {
			dev_err(dev, "cannot request gpio for card detect.\n");
		}
	}
	return 0;
}
static struct dw_mci_board smdk5410_dwmci2_pdata __initdata = {
	.num_slots		= 1,
	.quirks			= DW_MCI_QUIRK_HIGHSPEED,
	.bus_hz			= 160 * 1000 * 1000,
	.caps			= MMC_CAP_CMD23 | MMC_CAP_UHS_SDR104,
	.fifo_depth		= 0x80,
	.detect_delay_ms	= 200,
	.hclk_name		= "dwmci",
	.cclk_name		= "sclk_dwmci",
	.cfg_gpio		= exynos_dwmci2_cfg_gpio,
	.ext_cd_gpio	= MMC2_CD_GPIO,
	.get_bus_wd		= exynos_dwmci2_get_bus_wd,
	.sdr_timing		= 0x03040000,
	.ddr_timing		= 0x03020000,
	.clk_drv		= 0x3,
	.cd_type		= DW_MCI_CD_GPIO,
	.init			= exynos_dwmci2_init,
	.get_cd			= exynos_dwmci_get_cd,
	.clk_tbl                = exynos_dwmci_clk_rates,
};

static struct platform_device *smdk5410_mmc_devices[] __initdata = {
#ifdef CONFIG_MMC_DW
	&exynos5_device_dwmci0,
	&exynos5_device_dwmci1,
	&exynos5_device_dwmci2,
#endif
};

void __init exynos5_tf4_mmc_init(void)
{
#ifdef CONFIG_MMC_DW
	if (samsung_rev() < EXYNOS5410_REV_1_0)
		smdk5410_dwmci0_pdata.caps &=
			~(MMC_CAP_UHS_DDR50 | MMC_CAP_1_8V_DDR);
#ifndef CONFIG_EXYNOS_EMMC_HS200
	smdk5410_dwmci0_pdata.caps2 &=
		~MMC_CAP2_HS200_1_8V_SDR;
#endif
	exynos_dwmci_set_platdata(&smdk5410_dwmci0_pdata, 0);
	exynos_dwmci_set_platdata(&smdk5410_dwmci1_pdata, 1);
	exynos_dwmci_set_platdata(&smdk5410_dwmci2_pdata, 2);
#endif
	platform_add_devices(smdk5410_mmc_devices,
			ARRAY_SIZE(smdk5410_mmc_devices));
}
