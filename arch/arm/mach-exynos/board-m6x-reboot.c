/* linux/arch/arm/mach-exynos/m6x_reboot.c
 *
 * Copyright (C) 2013 Meizu Technology Co.Ltd, Zhuhai, China
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/sched.h>

#include <asm/cacheflush.h>
#include <asm/mach-types.h>
#include <asm/system_misc.h>

#include <mach/regs-pmu.h>
#include <mach/gpio.h>
#include <mach/gpio-meizu.h>
#include <asm-generic/gpio.h>

#include <plat/watchdog-reset.h>
#include <mach/resetreason.h>

#include "common.h"

#define REBOOT_MODE_CHARGE		0x0
#define REBOOT_MODE_WIPE		0x1
#define REBOOT_MODE_UPGRADE		0x2
/* Auto enter uboot fastboot mode */
#define REBOOT_MODE_FASTBOOT	0xFC
/* Auto enter uboot command line */
#define REBOOT_MODE_UBOOT		0xFE
/* custom reboot msg should be custom_xx */
#define REBOOT_MODE_CUSTOM		0xFF

static void __m6x_reboot(const char *cmd)
{
	unsigned long custom_val;
/*
	if(strstr(cmd, "help")) {
		show_reboot_help();
		return;
	}
*/
	local_irq_disable();

	/* Record the normal reboot reason */
	record_normal_reboot_reason(cmd);

	if (cmd) {
		if (strstr(cmd, "charge"))
			__raw_writel(REBOOT_MODE_CHARGE, EXYNOS_INFORM4);
		else if (strstr(cmd, "update_and_wipe"))
			__m6x_reboot("custom_1");
		else if (strstr(cmd, "wipe_sdcard"))
			__m6x_reboot("custom_2");
		else if (strstr(cmd, "wipe_userdata"))
			__m6x_reboot("custom_3");
		else if (strstr(cmd, "wipe_all"))
			__m6x_reboot("custom_4");
		else if (strstr(cmd, "update_locate"))
			__m6x_reboot("custom_5");
/*
		else if (strstr(cmd, "wipe"))
			__raw_writel(REBOOT_MODE_WIPE, EXYNOS_INFORM4);
*/
		else if (strstr(cmd, "upgrade"))
			__raw_writel(REBOOT_MODE_UPGRADE, EXYNOS_INFORM4);
		else if (strstr(cmd, "bootloader"))
			__raw_writel(REBOOT_MODE_UBOOT, EXYNOS_INFORM4);
		else if (strstr(cmd, "fastboot"))
			__raw_writel(REBOOT_MODE_FASTBOOT, EXYNOS_INFORM4);
		else if (strstr(cmd, "recovery"))
			__m6x_reboot("custom_8");
		else if (strstr(cmd, "custom")) {
			if (strict_strtoul(cmd+7, 10, &custom_val))
				custom_val = 0;
			custom_val = (custom_val << 8) | REBOOT_MODE_CUSTOM;
			__raw_writel(custom_val, EXYNOS_INFORM4);
		}
	}

	/* Soft Reset */
	exynos5_restart(0, NULL);

	/* Watchdog Reset if soft reset fails */
	arch_wdt_reset();

	pr_emerg("%s: waiting for reboot\n", __func__);
	unreachable();
}

static inline int is_cable_insert(void)
{
	return __gpio_get_value(MEIZU_VBUS_DET_IRQ);
}

void m6x_reboot(char str, const char *cmd)
{
	__m6x_reboot(cmd);
}

static void m6x_power_off_prepare(void)
{
	pr_info("power off prepare the deivce....\n");
}

static void m6x_power_off(void)
{
	int reg;

	if (is_cable_insert())
		__m6x_reboot("charge");
	else {
		reg = readl(EXYNOS_PS_HOLD_CONTROL);

		/* Dead loop to avoid sometimes auto restart */
		while (1) {
			pr_emerg("%s: waiting for power off\n", __func__);
			reg &= ~(0x1<<8);
			writel(reg, EXYNOS_PS_HOLD_CONTROL);
		}
	}
}

static int __init m6x_reboot_init(void)
{
	/* system shut down support */
	pm_power_off = m6x_power_off;
	pm_power_off_prepare = m6x_power_off_prepare;

	/* system reboot support */
	arm_pm_restart = m6x_reboot;

	return 0;
}

arch_initcall_sync(m6x_reboot_init);
