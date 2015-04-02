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
#include <linux/regulator/machine.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/gpio.h>
#include <linux/i2c-gpio.h>
#include <linux/platform_data/exynos_thermal.h>
#include <linux/mfd/samsung/core.h>
#include <linux/mfd/samsung/s2mps11.h>
#include <linux/usb/gadget.h>
#include <linux/mfd/max77802.h>
#include <linux/mfd/max77665.h>
#include <linux/power/max17047_battery.h>
#include <asm/io.h>
#include <asm/mach-types.h>

#include <plat/devs.h>
#include <plat/iic.h>
#include <plat/gpio-cfg.h>

#include <mach/regs-pmu.h>
#include <mach/irqs.h>
#include <mach/hs-iic.h>
#include <mach/devfreq.h>
#include <mach/tmu.h>
#include <mach/gpio-i2c-m6x.h>
#include <mach/gpio-meizu.h>

#include <mach/m6x-regulator-max77665.h>
#include <mach/m65-regulator-max77802.h>
#include <mach/m69-regulator-s2mps11.h>

#include "board-m6x.h"


#ifdef CONFIG_REGULATOR_S2MPS11
static struct i2c_board_info hs_i2c_devs2_0[] __initdata = {	
	{
		I2C_BOARD_INFO("sec-pmic", 0xCC >> 1),
		.platform_data	= &m69_s2mps11_pdata,
		.irq = M69_PMIC_EINT,
	}
};
#endif

#ifdef CONFIG_REGULATOR_MAX77802
static struct i2c_board_info hs_i2c_devs2_1[] __initdata = {
	{
		I2C_BOARD_INFO("max77802", 0x12 >> 1),
		.platform_data	= &m65_max77802_pdata,
		.irq = M65_PMIC_EINT,
	}
};
#endif

#if defined(CONFIG_MFD_MAX77665)
static struct i2c_board_info __initdata i2c_devs8[] = {
	{
		I2C_BOARD_INFO("max77665", (0xcc >> 1)),
		.platform_data	= &m6x_max77665_info,
		.irq = IRQ_EINT(10),
	}
};
#endif

#ifdef CONFIG_BATTERY_MAX17047
static struct max17047_platform_data __initdata m6x_max17047_info = {
	.r_sns = 10, /* 0.01 Ohm*/	
	.current_sensing = true,
};

static struct i2c_board_info __initdata i2c_devs13[] = {
	{
		I2C_BOARD_INFO(MAX17047_NAME, (0x6c >> 1)),
		.platform_data = &m6x_max17047_info,
	}
};

static void __init max17047_rt_init_res(void) {
	i2c_devs13[0].irq = MEIZU_FUELGAGE_IRQ;
}
#endif

#ifdef CONFIG_BATTERY_SAMSUNG
static struct platform_device samsung_device_battery = {
	.name	= "samsung-fake-battery",
	.id	= -1,
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

static struct exynos_devfreq_platdata m6x_qos_mif_pd __initdata = {
	.default_qos = 100000,
};

static struct exynos_devfreq_platdata m6x_qos_int_pd __initdata = {
	.default_qos = 50000,
};
#endif

#ifdef CONFIG_ARM_EXYNOS_IKS_CPUFREQ
static struct exynos_tmu_platform_data exynos5_tmu_data = {
	.low_trigger_levels[0] = 45,
	.low_trigger_levels[1] = 50,
	.low_trigger_levels[2] = 100,
	.low_trigger_levels[3] = 110,
	.normal_trigger_levels[0] = 50,
	.normal_trigger_levels[1] = 55,
	.normal_trigger_levels[2] = 100,
	.normal_trigger_levels[3] = 110,
	.boost_trigger_levels[0] = 70,
	.boost_trigger_levels[1] = 85,
	.boost_trigger_levels[2] = 100,
	.boost_trigger_levels[3] = 110,
	.trigger_level0_en = 1,
	.trigger_level1_en = 1,
	.trigger_level2_en = 1,
	.trigger_level3_en = 1,
	.gain = 5,
	.reference_voltage = 16,
	.noise_cancel_mode = 4,
	.cal_type = TYPE_ONE_POINT_TRIMMING,
	.efuse_value = 55,
	.low_freq_tab[0] = {
		.freq_clip_max = 1000 * 1000,
		.temp_level = 75,
	},
	.low_freq_tab[1] = {
		.freq_clip_max = 800 * 1000,
		.temp_level = 80,
	},
	.low_freq_tab[2] = {
		.freq_clip_max = 600 * 1000,
		.temp_level = 110,
	},
	.low_freq_tab[3] = {
		.freq_clip_max = 600 * 1000,
		.temp_level = 115,
	},
	.low_freq_tab[4] = {
		.freq_clip_max = 200 * 1000,
		.temp_level = 100,
	},
	.normal_freq_tab[0] = {
		.freq_clip_max = 1400 * 1000,
		.temp_level = 75,
	},
	.normal_freq_tab[1] = {
		.freq_clip_max = 1200 * 1000,
		.temp_level = 80,
	},
	.normal_freq_tab[2] = {
		.freq_clip_max = 1000 * 1000,
		.temp_level = 110,
	},
	.normal_freq_tab[3] = {
		.freq_clip_max = 550 * 1000,
		.temp_level = 115,
	},
	.normal_freq_tab[4] = {
		.freq_clip_max = 600 * 1000,
		.temp_level = 100,
	},
	.boost_freq_tab[0] = {
		.freq_clip_max = 1400 * 1000,
	},
	.boost_freq_tab[1] = {
		.freq_clip_max = 1200 * 1000,
	},
	.boost_freq_tab[2] = {
		.freq_clip_max = 1000 * 1000,
	},
	.boost_freq_tab[3] = {
		.freq_clip_max = 550 * 1000,
	},
	.boost_freq_tab[4] = {
		.freq_clip_max = 600 * 1000,
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

static struct platform_device *m6x_power_devices[] __initdata = {
	&m6x_device_i2c8,
#ifdef CONFIG_BATTERY_SAMSUNG
	&samsung_device_battery,
#endif
#ifdef CONFIG_PM_DEVFREQ
	&exynos5_mif_devfreq,
	&exynos5_int_devfreq,
#endif
	&exynos5410_device_tmu,

#ifdef CONFIG_BATTERY_MAX17047
	&m6x_device_i2c13,
#endif
};

#ifdef CONFIG_MFD_MAX77802
static __init void m65_max77802_i2c_init(void) {
	int ret = 0;

	ret = platform_device_register(&exynos5_device_hs_i2c2);
	if (ret) {
		platform_device_unregister(&exynos5_device_hs_i2c2);
		pr_err("%s: Error register i2c6!\n", __func__);
		return ;
	}

	max77802_rt_init_res();
	exynos5_hs_i2c2_set_platdata(NULL);
	i2c_register_board_info(6, hs_i2c_devs2_1, ARRAY_SIZE(hs_i2c_devs2_1));
}
#endif

#ifdef CONFIG_REGULATOR_S2MPS11
static void __init m69_s2mps11_i2c_init(void) {
	int ret = 0;

	ret = platform_device_register(&exynos5_device_hs_i2c3);
	if (ret) {
		platform_device_unregister(&exynos5_device_hs_i2c3);
		pr_err("%s: Error register i2c7!\n", __func__);
		return ;
	}

	exynos5_hs_i2c3_set_platdata(NULL);
	i2c_register_board_info(7, hs_i2c_devs2_0, ARRAY_SIZE(hs_i2c_devs2_0));
}
#endif

void __init exynos5_m6x_power_init(void)
{	
	if (machine_is_m65()) {
#ifdef CONFIG_MFD_MAX77802
		m65_max77802_i2c_init();
#endif
	} else if (machine_is_m69()) {
#ifdef CONFIG_REGULATOR_S2MPS11
		m69_s2mps11_i2c_init();
#endif
	} else {
		pr_err("Error: Please Configure Machine Version.\r\n");
		BUG();
	}

#ifdef CONFIG_MFD_MAX77665
	/*MAX77665 */
	max77665_rt_init_res();
	i2c_register_board_info(8, i2c_devs8, ARRAY_SIZE(i2c_devs8));
#endif

#ifdef CONFIG_BATTERY_MAX17047
	/* MAX17047 fuelgauge*/
	max17047_rt_init_res();
	i2c_register_board_info(13, i2c_devs13, ARRAY_SIZE(i2c_devs13));
#endif

#ifdef CONFIG_PM_DEVFREQ
	s3c_set_platdata(&m6x_qos_mif_pd, sizeof(struct exynos_devfreq_platdata),
			&exynos5_mif_devfreq);

	s3c_set_platdata(&m6x_qos_int_pd, sizeof(struct exynos_devfreq_platdata),
			&exynos5_int_devfreq);
#endif

	s3c_set_platdata(&exynos5_tmu_data, sizeof(struct exynos_tmu_platform_data),
			&exynos5410_device_tmu);

	platform_add_devices(m6x_power_devices,
			ARRAY_SIZE(m6x_power_devices));
}
