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

#ifndef __H_GPIO_I2C_M6X_H__
#define __H_GPIO_I2C_M6X_H__

#include <linux/platform_device.h>
#include <linux/i2c-gpio.h>


#define DECLARE_GPIO_I2C(bus_num) \
	extern struct platform_device m6x_device_i2c##bus_num

/* MEIZU Gpio I2C Bus Declare */
DECLARE_GPIO_I2C(8); /* MAX77665 */
#if defined(CONFIG_BACKLIGHT_LM3695) || defined(CONFIG_BACKLIGHT_LM3630)
DECLARE_GPIO_I2C(9); /* LM3530 */
#endif
#ifdef CONFIG_PA_TFA9887
DECLARE_GPIO_I2C(11); /* TFA9887 PA */
#endif
#ifdef CONFIG_BATTERY_MAX17047
DECLARE_GPIO_I2C(13); /* FUELGAUGE */
#endif
#ifdef CONFIG_TPS65132
DECLARE_GPIO_I2C(14); /* TPS65132 */
#endif
DECLARE_GPIO_I2C(15); /* LIGHT AND PROXIMITY SENSOR*/
DECLARE_GPIO_I2C(16); /* NFC*/
#ifdef CONFIG_MX_HUB_IIC18
DECLARE_GPIO_I2C(18); /* MCU */
#endif
#ifdef CONFIG_AUDIENCE_ES305B
DECLARE_GPIO_I2C(19); /* ES305B */
#endif

#endif // __H_GPIO_I2C_M6X_H__
