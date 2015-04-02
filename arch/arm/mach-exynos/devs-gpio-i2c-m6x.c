/*
 * Copyright (C) 2013 Meizu Technology Co.Ltd, Zhuhai, China
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc.,
 *
 * Revision History
 *
 * Author: wangbo@meizu.com
 * Date:    2013-9-6
 *
 */

#include <mach/gpio-i2c-m6x.h>
#include <mach/gpio-meizu.h>


static void m6x_set_gpio_i2c_pin(int bus_num, unsigned int *sda_pin, unsigned int *scl_pin) {
	switch (bus_num) {
	case 8:
		*sda_pin = MEIZU_CHG_SDA;
		*scl_pin = MEIZU_CHG_SCL;
		break;
#if defined(CONFIG_BACKLIGHT_LM3695) || defined(CONFIG_BACKLIGHT_LM3630)
	case 9:
		*sda_pin = MEIZU_LCD_BL_SDA;
		*scl_pin = MEIZU_LCD_BL_SCL;
		break;
#endif
#ifdef CONFIG_PA_TFA9887
	case 11:
		*sda_pin = MEIZU_PA_SDA;
		*scl_pin = MEIZU_PA_SCL;
		break;
#endif
#ifdef CONFIG_BATTERY_MAX17047
	case 13:
		*sda_pin = MEIZU_FUEL_SDA;
		*scl_pin = MEIZU_FUEL_SCL;
		break;
#endif
#ifdef CONFIG_TPS65132
	case 14:
		*sda_pin = MEIZU_LCD_BIAS_SDA;
		*scl_pin = MEIZU_LCD_BIAS_SCL;
		break;
#endif
	case 15:
		*sda_pin = MEIZU_IR_SDA;
		*scl_pin = MEIZU_IR_SCL;
		break;
	case 16:
		*sda_pin = MEIZU_NFC_SDA;
		*scl_pin = MEIZU_NFC_SCL;
		break;
#ifdef CONFIG_MX_HUB_IIC18
	case 18:
		*sda_pin = MEIZU_MCU_SDA;
		*scl_pin = MEIZU_MCU_SCL;
		break;
#endif
	case 19:
		*sda_pin = MEIZU_ES305B_SDA;
		*scl_pin = MEIZU_ES305B_SCL;
		break;
	default:
		pr_err("%s, %d bus number gpio i2c device not exist!\r\n", __func__, bus_num);
		return ;
	};
}

#define COMMON_GPIO_I2C_PLATFORM_DATA_DEFINE \
	.sda_is_open_drain = 0, \
	.scl_is_open_drain = 0, \
	.scl_is_output_only = 0,

#define DEFINE_GPIO_I2C(bus_num, delay) \
	static struct i2c_gpio_platform_data _gpio_i2c##bus_num##_data = { \
		COMMON_GPIO_I2C_PLATFORM_DATA_DEFINE \
		.udelay = delay, /*the scl frequency is (500 / udelay) kHz*/ \
		.set_gpio = m6x_set_gpio_i2c_pin, \
	}; \
	struct platform_device m6x_device_i2c##bus_num = { \
		.name = "i2c-gpio", \
		.id = bus_num, \
		.dev.platform_data = &_gpio_i2c##bus_num##_data, \
	}

/* MEIZU Gpio I2C Bus Define */
DEFINE_GPIO_I2C(8, 2); /* MAX77665 */
#if defined(CONFIG_BACKLIGHT_LM3695) || defined(CONFIG_BACKLIGHT_LM3630)
DEFINE_GPIO_I2C(9, 2); /* LM3530 */
#endif
#ifdef CONFIG_PA_TFA9887
DEFINE_GPIO_I2C(11, 2); /* TFA9887 PA */
#endif
#ifdef CONFIG_BATTERY_MAX17047
DEFINE_GPIO_I2C(13, 5); /* FUELGAUGE */
#endif
#ifdef CONFIG_TPS65132
DEFINE_GPIO_I2C(14, 2); /* TPS65132 */
#endif
DEFINE_GPIO_I2C(15, 2); /* LIGHT AND PROXIMITY SENSOR (GP2AP) */
DEFINE_GPIO_I2C(16, 2); /* NFC */
DEFINE_GPIO_I2C(17, 2); /* AKM8963N FOR COMPASS*/
#ifdef CONFIG_MX_HUB_IIC18
DEFINE_GPIO_I2C(18, 1); /* MCU */
#endif
#ifdef CONFIG_AUDIENCE_ES305B
DEFINE_GPIO_I2C(19, 2); /* ES305B */
#endif
