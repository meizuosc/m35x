/*
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/i2c/at24.h>
#include <linux/i2c.h>
#include <linux/memory.h>
#include <linux/mutex.h>

#include <plat/gpio-cfg.h>
#include <plat/cpu.h>
#include <plat/clock.h>
#include <plat/devs.h>
#include <mach/gpio-meizu.h>

#include "board-m6x.h"

#ifdef CONFIG_SENSOR_GP2AP
static struct i2c_board_info __initdata i2c_devs_gp2ap[] = {
	{
		I2C_BOARD_INFO("gp2ap030a00f", 0x39),
		//.irq = MEIZU_IR_IRQ,
	}

};
#endif

#ifdef CONFIG_SENSOR_GP2AP
static void __init senor_gp2ap_rt_init_res(void) {
	i2c_devs_gp2ap[0].irq = MEIZU_IR_IRQ;
}
#endif

void __init exynos5_m6x_sensors_init(void)
{
#ifdef CONFIG_SENSOR_GP2AP
	senor_gp2ap_rt_init_res();
	i2c_register_board_info(15, i2c_devs_gp2ap, ARRAY_SIZE(i2c_devs_gp2ap));
#endif
}
