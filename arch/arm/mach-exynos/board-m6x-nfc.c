/*
 * Copyright (C) 2012 Meizu Technology Co.Ltd, Zhuhai, China
 *		http://www.meizu.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/


#include <linux/platform_device.h>
#include <linux/nfc/bcm2079x.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>

#include <mach/gpio.h>
#include <mach/gpio-i2c-m6x.h>
#include <mach/gpio-meizu.h>
#include <mach/hardware.h>

static struct bcm2079x_platform_data nfc_pdata = {
#if 0
	.irq_gpio = MEIZU_NFC_IRQ,
	.en_gpio = MEIZU_NFC_REG_PU,
	.wake_gpio = MEIZU_NFC_WAKE,
#endif
};

static struct i2c_board_info __initdata i2c16_board_info[] = {
	{
		I2C_BOARD_INFO("bcm2079x-i2c", 0x77),
		.platform_data	= &nfc_pdata,
		.irq		= IRQ_EINT(28),
	}
};

static void __init nfc_rt_init_res(void) {
	nfc_pdata.irq_gpio = MEIZU_NFC_IRQ;
	nfc_pdata.en_gpio = MEIZU_NFC_REG_PU;
	nfc_pdata.wake_gpio = MEIZU_NFC_WAKE;
}

int __init m6x_nfc_init(void)
{
	if (meizu_board_have_nfc()) {
		nfc_rt_init_res();

		i2c_register_board_info(16, i2c16_board_info,
					ARRAY_SIZE(i2c16_board_info));
		platform_device_register(&m6x_device_i2c16);
	}
	return 0;
}

device_initcall(m6x_nfc_init);
