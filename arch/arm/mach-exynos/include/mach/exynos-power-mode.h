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
#include <linux/notifier.h>
#endif

#define POWER_MODE_LEN	(16)

struct exynos_power_info {
	char		*mode_name;
	unsigned int	cpu_freq_lock;
	unsigned int	mif_freq_lock;
	unsigned int	int_freq_lock;
	bool		cpu_online_core0;
	bool		cpu_online_core1;
	bool		cpu_online_core2;
	bool		cpu_online_core3;
};

enum exynos_power_mode_idx {
	POWER_MODE_0,
	POWER_MODE_1,
	POWER_MODE_2,
	POWER_MODE_END,
};

#define power_mode_register_notifier(nb)	do { } while (0)
#define power_mode_unregister_notifier(nb)	do { } while (0)
#define power_mode_notifier_call_chain(val, v)	do { } while (0)
extern unsigned int cpu_power_mode;

