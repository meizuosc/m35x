/* arch/arm/mach-exynos/m6x_jack.c
 *
 * Copyright (C) 2012 Meizu Technology Co.Ltd, Zhuhai, China
 *
 * Based on mach-exynos/p4note-jack.c
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/m6x_jack.h>
#include <linux/delay.h>

#include <asm/mach-types.h>

#include <mach/gpio.h>
#include <linux/regulator/machine.h>

static int m6x_set_jack_micbias(bool on)
{
	struct regulator *regulator;
	int ret = 0;

	regulator = regulator_get(NULL, "vdd28_micbias");
	if (IS_ERR(regulator)) {
		pr_err("regulator_get failed");
		return -1;
	}

	ret = on ? regulator_enable(regulator) :
				// regulator_disable(regulator);
				regulator_enable(regulator) ;
	if (ret < 0) {
		pr_err("regulator_%sable failed\n", on ? "on" : "off");
		return ret;
	}

	regulator_put(regulator);

	return ret;
}

static struct m6x_jack_zone m6x_jack_zones[] = {
	{
		/* adc == 0, unstable zone, default to 3pole if it stays
		 * in this range for 300ms (15ms delays, 20 samples)
		 */
		.adc_high = 0,
		.delay_ms = 15,
		.check_count = 20,
		.jack_type = M6X_HEADSET_3POLE,
	},
	{
		/* 0 < adc <= 1200, unstable zone, default to 3pole if it stays
		 * in this range for 300ms (15ms delays, 20 samples)
		 */
		.adc_high = 1200,
		.delay_ms = 10,
		.check_count = 80,
		.jack_type = M6X_HEADSET_3POLE,
	},
	{
		/* 1200 < adc <= 2600, unstable zone, default to 4pole if it
		 * stays in this range for 800ms (10ms delays, 80 samples)
		 */
		.adc_high = 2600,
		.delay_ms = 10,
		.check_count = 10,
		.jack_type = M6X_HEADSET_4POLE,
	},
	{
		/* 2600 < adc <= 4000, 3 pole zone, default to 4pole if it
		 * stays in this range for 100ms (10ms delays, 10 samples)
		 */
		.adc_high = 4000,
		.delay_ms = 10,
		.check_count = 5,
		.jack_type = M6X_HEADSET_4POLE,
	},
	{
		/* adc > 3800, unstable zone, default to 3pole if it stays
		 * in this range for two seconds (10ms delays, 200 samples)
		 */
		.adc_high = 0x7fffffff,
		.delay_ms = 10,
		.check_count = 200,
		.jack_type = M6X_HEADSET_3POLE,
	},
};

/* To support 3-buttons earjack */
static struct m6x_jack_buttons_zone m6x_jack_buttons_zones3x[] = {
	{
		/* 0 <= adc <=10, stable zone */
		.code = KEY_MEDIA,
		.adc_low = 0,
		.adc_high = 10,
	},
	{
		/* 200 <= adc <= 300, stable zone */
		.code = KEY_VOLUMEUP,
		.adc_low = 200,
		.adc_high = 300,
	},
	{
		/* 300 <= adc <= 850, stable zone */
		.code = KEY_VOLUMEDOWN,
		.adc_low = 300,
		.adc_high = 850,
	},
};

static struct m6x_jack_buttons_zone m6x_jack_buttons_zones30[] = {
	{
		/* 0 <= adc <=10, stable zone */
		.code = KEY_MEDIA,
		.adc_low = 0,
		.adc_high = 10,
	},
	{
		/* 200 <= adc <= 300, stable zone */
		.code = KEY_VOLUMEUP,
		.adc_low = 100,
		.adc_high = 200,
	},
	{
		/* 300 <= adc <= 850, stable zone */
		.code = KEY_VOLUMEDOWN,
		.adc_low = 200,
		.adc_high = 850,
	},
};

static struct m6x_jack_platform_data m6x_jack_data = {
	.set_micbias_state = m6x_set_jack_micbias,
	.zones = m6x_jack_zones,
	.num_zones = ARRAY_SIZE(m6x_jack_zones),
	.buttons_zones = m6x_jack_buttons_zones3x,
	.num_buttons_zones = ARRAY_SIZE(m6x_jack_buttons_zones3x),
	// .det_gpio = EXYNOS5410_GPX3(0),
	.det_gpio = EXYNOS5410_GPX3(6),
	.adc_channel = 0,
};

static struct platform_device m6x_device_jack = {
	.name = "m6x_jack",
	.id = -1,
	.dev.platform_data = &m6x_jack_data,
};

static int  __init m6x_jack_init(void)
{
	return platform_device_register(&m6x_device_jack);
}
arch_initcall(m6x_jack_init);

