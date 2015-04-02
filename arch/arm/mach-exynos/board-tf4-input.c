/*
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include <linux/delay.h>
#include <linux/gpio.h>
#if defined(CONFIG_KEYBOARD_GPIO)
#include <linux/gpio_keys.h>
#endif
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>

#include <plat/devs.h>
#include <plat/gpio-cfg.h>

#include <mach/irqs.h>
#include <mach/hs-iic.h>
#include <mach/regs-gpio.h>

#include "board-tf4.h"

#define GPIO_TSP_INT		EXYNOS5410_GPX0(1)
#define GPIO_TPPWR_TR_ON	EXYNOS5410_GPJ1(7)
#define GPIO_TSP_RST		EXYNOS5410_GPJ0(2)

#if defined(CONFIG_KEYBOARD_GPIO)

#define GPIO_KEY_POWER        EXYNOS5410_GPX0(2)
#define GPIO_KEY_VOLUMEDOWN   EXYNOS5410_GPX0(3)
#define GPIO_KEY_VOLUMEUP     EXYNOS5410_GPX0(4) 

struct gpio_keys_button tf4_button[] = {
	{
		.code = KEY_POWER,
		.gpio = GPIO_KEY_POWER,
		.active_low = 1,
		.wakeup = 1,
	},
	{
		.code = KEY_VOLUMEDOWN,
		.gpio = GPIO_KEY_VOLUMEUP,
		.active_low = 1,
		.wakeup = 1,
	},
	{
		.code = KEY_VOLUMEUP,
		.gpio = GPIO_KEY_VOLUMEDOWN,
		.active_low = 1,
		.wakeup = 1,
	},		
};

struct gpio_keys_platform_data tf4_gpiokeys_platform_data = {
	tf4_button,
	ARRAY_SIZE(tf4_button),
};

static struct platform_device tf4_gpio_keys = {
	.name	= "gpio-keys",
	.dev	= {
		.platform_data = &tf4_gpiokeys_platform_data,
	},
};
#endif

struct egalax_i2c_platform_data {
	unsigned int gpio_int;
	unsigned int gpio_en;
	unsigned int gpio_rst;
};

static struct egalax_i2c_platform_data exynos5_TP_data = {
	.gpio_int = GPIO_TSP_INT,
	.gpio_en  = GPIO_TPPWR_TR_ON,		
//	.gpio_rst = GPIO_TSP_RST,  /* reserved for Coasia's TP */
};


static struct i2c_board_info hs_i2c_devs1[] __initdata = {
	{
		I2C_BOARD_INFO("egalax_i2c", 0x04),
		.irq		= IRQ_EINT(1),
		.platform_data	= &exynos5_TP_data,
	},
};


/////////////////

static struct i2c_gpio_platform_data gpio_i2c15_data = {
	.sda_pin            = EXYNOS5410_GPA2(2),
	.scl_pin            = EXYNOS5410_GPA2(3),
	.udelay             = 2,
	.sda_is_open_drain  = 0,
	.scl_is_open_drain  = 0,
  .scl_is_output_only = 0,
};

static struct platform_device gpio_i2c15_dev = {
	.name               = "i2c-gpio",
	.id                 = 15,
	.dev.platform_data  = &gpio_i2c15_data,
};


static struct platform_device *tf4_input_devices[] __initdata = {
#if defined(CONFIG_KEYBOARD_GPIO)
	&tf4_gpio_keys,
#endif
  &gpio_i2c15_dev,
};


static struct i2c_board_info i2c_devs15[] __initdata = {
	{
		I2C_BOARD_INFO("egalax_i2c", 0x04), //(0x58 >> 1)),
		.irq		= IRQ_EINT(1),		
		.platform_data = &exynos5_TP_data,
	},
};
//////////////////	
	
#if defined(CONFIG_KEYBOARD_GPIO)
static void __init tf4_key_gpio_init(void)
{
	s3c_gpio_setpull(GPIO_KEY_VOLUMEDOWN, S3C_GPIO_PULL_UP);
	s3c_gpio_setpull(GPIO_KEY_VOLUMEUP  , S3C_GPIO_PULL_UP);
}
#endif

void __init exynos5_tf4_input_init(void)
{
	exynos5_hs_i2c1_set_platdata(NULL);

	
	i2c_register_board_info(15,i2c_devs15, ARRAY_SIZE(i2c_devs15));
		
#if defined(CONFIG_KEYBOARD_GPIO)
	tf4_key_gpio_init();
#endif
	platform_add_devices(tf4_input_devices,	ARRAY_SIZE(tf4_input_devices));
}
