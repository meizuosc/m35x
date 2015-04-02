/*
 * Copyright (C) 2013 Meizu Technology Co.Ltd, Zhuhai, China
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/syscore_ops.h>

#include <asm/mach-types.h>

#include <plat/pm.h>
#include <plat/gpio-cfg.h>

#include <asm/mach-types.h>

#include <mach/gpio.h>
#include <mach/gpio-common.h>

#include <mach/gpio-meizu.h>

#ifdef CONFIG_MACH_M65
#if defined(CONFIG_MEIZU_M65_V3)
#include <mach/m65-v3-gpio.h>
#elif defined(CONFIG_MEIZU_M65_V31)
#include <mach/m65-v31-gpio.h>
#else
#error "Please check m65 version config"
#endif
#endif

#ifdef CONFIG_MACH_M69
#if defined(CONFIG_MEIZU_M69_V1)
#include <mach/m69-v1-gpio.h>
#else
#error "Please check m69 version config"
#endif
#endif

const unsigned short *meizu_gpio;

inline static void config_gpio_pd(unsigned int pin, s5p_gpio_pd_cfg_t conpdn, s5p_gpio_pd_pull_t pudpdn)
{
	s5p_gpio_set_pd_cfg(pin, conpdn);
	s5p_gpio_set_pd_pull(pin, pudpdn);
}

static void m6x_config_special_gpio_pd(void)
{
}

static int m6x_config_gpio_pd(void)
{
	unsigned int i;
	unsigned int tbl_len;
	const struct gpio_pd_info *gpio;
	const struct gpio_pd_info *gpio_tbl;

	pr_debug("PM:-> Entering %s\n", __func__);

	if (machine_is_m65()) {
		gpio_tbl = m65_gpio_pd_table;
		tbl_len = ARRAY_SIZE(m65_gpio_pd_table);
	} else if (machine_is_m69()) {
		gpio_tbl = m69_gpio_pd_table;
		tbl_len = ARRAY_SIZE(m69_gpio_pd_table);
	} else {
		pr_err("Error: Please Configure Machine Version.\r\n");
		BUG();
	}

	for (i = 0; i < tbl_len; i++) {
		gpio = &gpio_tbl[i];
		config_gpio_pd(gpio->pin, gpio->type, gpio->pull);
	}

	/* runtime pin config */
	m6x_config_special_gpio_pd();

	return 0;
}

static struct syscore_ops m6x_gpio_pd_syscore_ops = {
	.suspend	= m6x_config_gpio_pd
};

static __init int m6x_init_gpio_cfg(void)
{
	const struct gpio_info *gpio;
	const struct gpio_info *gpio_info_tbl;
	unsigned int tbl_len;
	unsigned int i;

	if (machine_is_m65()) {
		gpio_info_tbl = m65_gpio_table;
		tbl_len = ARRAY_SIZE(m65_gpio_table);
	} else if (machine_is_m69()) {
		gpio_info_tbl = m69_gpio_table;
		tbl_len = ARRAY_SIZE(m69_gpio_table);
	} else {
		pr_err("Error: Please Configure Machine Version.\r\n");
		BUG();
	}

	for (i = 0; i < tbl_len; i++) {
		gpio = &gpio_info_tbl[i];

		s3c_gpio_cfgpin(gpio->pin, gpio->type);
		s3c_gpio_setpull(gpio->pin, gpio->pull);
		if (gpio->type == S3C_GPIO_OUTPUT)
			gpio_set_value(gpio->pin, !!gpio->data);
		s5p_gpio_set_drvstr(gpio->pin, gpio->drv);
	}

	return 0;
}

static int __init m6x_init_gpio(void)
{
	register_syscore_ops(&m6x_gpio_pd_syscore_ops);

	m6x_init_gpio_cfg();
	m6x_config_gpio_pd();

	return 0;
}

/* must be initialize gpio before init_machine function */
arch_initcall(m6x_init_gpio);
