/*
 * linux/arch/arm/mach-exynos/factory_test.c
 *
 * Copyright (C) 2013 Meizu Technology Co.Ltd, Zhuhai, China
 * Author: 	Liu Jianping	<heljoy@meizu.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <linux/gpio.h>
#include <asm/mach-types.h>
#include <mach/gpio-common.h>
#include <mach/gpio-meizu.h>

/* Global variable for factory test */
int (*mx_is_factory_test_mode)(int type);
int (*mx_set_factory_test_led)(int on);

static int is_factory_test_mode(int type)
{
	int gpio1, gpio2, gpio3;
	int ret = 0;

	gpio1 = MEIZU_GPIO_FACTORY_MODE;
	gpio2 = MEIZU_VOLUP_IRQ;
	gpio3 = MEIZU_VOLDOWN_IRQ;

	switch(type) {
	case MX_FACTORY_TEST_BT:
		if(!gpio_get_value(gpio1) && !gpio_get_value(gpio2))
			ret = 1;
		break;
	case MX_FACTORY_TEST_CAMERA:
		if(!gpio_get_value(gpio1) && !gpio_get_value(gpio3))
			ret = 1;
		break;
	default:
		if(!gpio_get_value(gpio1))
			ret = 1;
		break;
	}

	return ret;

}

static int set_factory_test_led(int on)
{
	int gpio, ret;
	int gpio_value = on ? GPIOF_OUT_INIT_HIGH: GPIOF_OUT_INIT_LOW;

	gpio = MEIZU_GPIO_TEST_LED;

	ret = gpio_request_one(gpio, gpio_value, "mx_test_led");
	if (ret)
		return ret;

	gpio_free(gpio);

	return 0;
}
static int  __init m6x_init_factory(void)
{
	mx_is_factory_test_mode = is_factory_test_mode;
	mx_set_factory_test_led = set_factory_test_led;
	return 0;
}

arch_initcall(m6x_init_factory);
