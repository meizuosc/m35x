/*
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Power mode for user space
 *
 * Three main power levels are defined and according interface is provided to
 * user space: /sys/class/exynos_power/exynos_power_mode/cur_power_mode
 *
 * 1. low: powersave mode for low-power, can be set via the power is low
 * 2. normal: balancing mode for daily use
 * 3. high: performance mode for large games
 *
 * Notes:
 *
 * 1. The current configuration is only for providing a interface to user
 * space, the detailed config for each level should be measured and considered
 * carefully. Differ from the old power mode driver, we limit the maximum power
 * consumption instead limit the minimum performance, so, we must make sure the
 * limitation should at least make the devices work especially in low power
 * mode.
 * 2. The limitation of cpu cores may not work currently for the cpu cores may
 * be hotpluged dynamically in the cpufreq driver.
 *
 * TODO: There is no interface to limit the maximum of the mif/int freq
 * currently, new PM Qos interface should be added and used here later.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * update by chenshb 2013-06-03
 * define cpufreq three level:600 1200 1600 for performance adjust.
 * update by chenshb@meizu.com 2013-08-06
 * define cpufreq three level:600 1200 1600 for performance adjust.
 */
#ifndef POWER_MODE
#define POWER_MODE
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pm_qos.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/kobject.h>
#include <linux/cpufreq.h>
#endif

#include <mach/exynos-power-mode.h>

#define TRANSITION_QOS		(1200000)
#define TRANSITION_DELAY	(1000)

struct kboject;
struct pm_qos_request power_mode_qos;
static struct exynos_power_info *cur_power_mode;
unsigned int cpu_power_mode;

/* Power mode setting information: Limit the maximum frequency */
struct exynos_power_info exynos_power_mode[POWER_MODE_END] = {
/* cpu_lock, mif_lock, int_lock, cpu0_online, cpu1_online, cpu2_online, cpu3_online */
	{ "low", 600000, 0, 0, 1, 1, 1, 1 },
	{ "normal", 1600000, 0, 0, 1, 1, 1, 1 },
	{ "high", 1600000, 0, 0, 1, 1, 1, 1 },
};

static void show_power_mode_list(void)
{
	int i;
	pr_info("Exynos Power mode List");
	pr_info("NAME\tCPU\tMIF\tINT\tCPU0\tCPU1\tCPU2\tCPU3\n");
	/* Show Exynos Power mode list */
	for (i = 0; i < POWER_MODE_END; i++ ) {
		pr_info("%s\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", exynos_power_mode[i].mode_name,
			exynos_power_mode[i].cpu_freq_lock, exynos_power_mode[i].mif_freq_lock,
			exynos_power_mode[i].int_freq_lock, exynos_power_mode[i].cpu_online_core0,
			exynos_power_mode[i].cpu_online_core1, exynos_power_mode[i].cpu_online_core2,
			exynos_power_mode[i].cpu_online_core3);
	}
}

static ssize_t show_power_mode(struct kobject *kobj, struct attribute *attr, char *buf)
{
	int ret;

	ret = snprintf(buf, POWER_MODE_LEN, "%s\n", cur_power_mode->mode_name);

	show_power_mode_list();

	return ret;
}

static unsigned int exynos_get_power_mode(char *target_mode, unsigned int *cpu_lock,
				unsigned int *int_lock, unsigned int *mif_lock, unsigned int *cpu_hotplug_mask)
{
	unsigned int i;

	for (i = 0; i < POWER_MODE_END; i++) {
		if (!strnicmp(exynos_power_mode[i].mode_name, target_mode, POWER_MODE_LEN)) {
			goto set_power_mode_info;
		}
	}

	return -EINVAL;
set_power_mode_info:
	cur_power_mode = &exynos_power_mode[i];
	if (cpu_power_mode == i) {
		pr_info("%s: already in %s mode\n", __func__, cur_power_mode->mode_name);
		return -EFAULT;
	}

	cpu_power_mode = i;

	*cpu_lock = exynos_power_mode[i].cpu_freq_lock;
	*int_lock = exynos_power_mode[i].int_freq_lock;
	*mif_lock = exynos_power_mode[i].mif_freq_lock;

	*cpu_hotplug_mask = ((exynos_power_mode[i].cpu_online_core0 << 0) |
			     (exynos_power_mode[i].cpu_online_core1 << 1) |
			     (exynos_power_mode[i].cpu_online_core2 << 2) |
			     (exynos_power_mode[i].cpu_online_core3 << 3));

	return 0;
}

static ssize_t __ref store_power_mode(struct kobject *kobj, struct attribute *attr,
			      const char *buf, size_t count)
{
	char str_power_mode[POWER_MODE_LEN];
	int ret;
	unsigned int cpu_lock, int_lock, mif_lock, cpu_hotplug_mask;

	ret = sscanf(buf, "%15s", str_power_mode);
	if (ret != 1)
		return -EINVAL;

	if (exynos_get_power_mode(str_power_mode, &cpu_lock, &int_lock, &mif_lock, &cpu_hotplug_mask)) {
		pr_err("%s Can not get power mode\n", __func__);
		return ret;
	}

	/* Update PM_QOS */
	pr_info("%s\t%d\t%d\t%d\n", str_power_mode, cpu_lock, mif_lock, int_lock);

	if (pm_qos_request_active(&power_mode_qos))
		pm_qos_update_request(&power_mode_qos, TRANSITION_QOS);
	else
		pm_qos_add_request(&power_mode_qos, PM_QOS_CPU_FREQ_MAX, TRANSITION_QOS);

	msleep(TRANSITION_DELAY);
	pm_qos_update_request(&power_mode_qos, cpu_lock);

	/* notify the client */
	power_mode_notifier_call_chain(cpu_power_mode, buf);
	return count;
}

define_one_global_rw(power_mode);

static int __init exynos_power_mode_init(void)
{
	int ret;

	ret = sysfs_create_file(power_kobj, &power_mode.attr);
	if (ret) {
		pr_err("%s: failed to create /sys/power/power_mode sysfs interface\n", __func__);
		return ret;
	}

	/* Setting init power mode info */
	cur_power_mode = &exynos_power_mode[POWER_MODE_1];
	cpu_power_mode = POWER_MODE_1;
	show_power_mode_list();

	return 0;
}
subsys_initcall(exynos_power_mode_init);
