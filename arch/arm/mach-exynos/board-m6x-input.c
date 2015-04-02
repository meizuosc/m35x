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
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/bootmode.h>
#include <linux/i2c-gpio.h>

#include <asm/mach-types.h>
#include <asm/tlbflush.h>
#include <asm/cacheflush.h>

#include <plat/devs.h>
#include <plat/iic.h>
#include <plat/gpio-cfg.h>

#include <mach/irqs.h>
#include <mach/hs-iic.h>
#include <mach/regs-gpio.h>
#include <mach/gpio-meizu.h>
#include <mach/gpio-i2c-m6x.h>

#include "board-m6x.h"

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_I2C
#include <linux/input/synaptics_dsx.h>
#endif

#ifdef CONFIG_MFD_MX_HUB
#include <linux/mfd/mx-hub.h>
#endif

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_I2C
#define RMI4_ADDR	(0x20)
static int synaptics_gpio_setup(unsigned gpio, bool configure)
{
	int retval=0;
	if (configure) {
		retval = gpio_request(gpio, "rmi4_attn");
		if (retval) {
			pr_err("%s: Failed to get attn gpio %d. Code: %d.",
			       __func__, gpio, retval);
			return retval;
		}

		retval = gpio_direction_input(gpio);
		if (retval) {
			pr_err("%s: Failed to setup attn gpio %d. Code: %d.",
			       __func__, gpio, retval);
			gpio_free(gpio);
		}
	} else {
		pr_warn("%s: No way to deconfigure gpio %d.",
		       __func__, gpio);
	}

	return retval;
}

static unsigned char rmi4_button_codes[] = {KEY_HOME,KEY_AGAIN,KEY_BACK,KEY_UP,KEY_LEFT,KEY_RIGHT,KEY_DOWN,KEY_END};
static struct synaptics_dsx_cap_button_map TM_SAMPLE2_cap_button_map = {
	.nbuttons 	= ARRAY_SIZE(rmi4_button_codes),
	.map 		= rmi4_button_codes,
};

static struct synaptics_dsx_platform_data dsx_platformdata = {
	.irq_flags 	= IRQF_TRIGGER_FALLING,
	.gpio_config 	= synaptics_gpio_setup,
 	.cap_button_map = &TM_SAMPLE2_cap_button_map,
	.regulator_en 	= 1,
};

static void __init dxs_rt_init_res(void) {
	dsx_platformdata.irq_gpio 	= MEIZU_TOUCH_IRQ;
	dsx_platformdata.vbus_gpio 	= MEIZU_VBUS_DET_IRQ;
}
#endif

static struct i2c_board_info __initdata i2c_devs7[] = {
#ifdef	CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_I2C
	{
 		I2C_BOARD_INFO("synaptics_dsx_i2c", RMI4_ADDR),
 		.platform_data = &dsx_platformdata,
     	},
#endif
};

#ifdef	CONFIG_MFD_MX_HUB
static struct acc_platform_data pacc = {
	.poll_interval = 10,   /*default poll delay 10 ms*/
	.min_interval = 1,

	.fs_range = LSM330DLC_ACC_G_2G,

	.axis_map_x = 1,  /*x=-x, y=-y, z=z*/
	.axis_map_y = 0,
	.axis_map_z = 2,

	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 1,
};

static struct gyr_platform_data pgyr = {
	.poll_interval = 10,
	.fs_range = LSM330DLC_GYR_FS_2000DPS,
	.min_interval = 1,

	.axis_map_x = 1,  /*x=-x, y=-y, z=-z*/
	.axis_map_y = 0,
	.axis_map_z = 2,

	.negate_x = 0,
	.negate_y = 1,
	.negate_z = 1,
};

static int hub_button_codes[] = {KEY_HOME,KEY_UP,KEY_DOWN};
static struct mx_sensor_hub_platform_data __initdata mx_hub_platformdata = {
	.irq_base = IRQ_BOARD_START + 80,
	.wakeup = true,
	.name = "mx_sensor_hub",
	
	.nbuttons = ARRAY_SIZE(hub_button_codes),
	.keymap = hub_button_codes,
	
	.compass_layout = 3,
	.pacc = &pacc,
	.pgyr = &pgyr,
};

static void __init mx_hub_rt_init_res(void) {

	if(machine_is_m69()){
		pacc.negate_x = 0;
		pacc.negate_y = 1;
		pacc.negate_z = 0;

		pgyr.negate_x = 0;
		pgyr.negate_y = 0;
		pgyr.negate_z = 0;
	}

	mx_hub_platformdata.gpio_wake 	= MEIZU_MCU_SLEEP;
	mx_hub_platformdata.gpio_reset 	= MEIZU_MCU_RST;
	mx_hub_platformdata.gpio_irq 	= MEIZU_MCU_IRQ;
	mx_hub_platformdata.gpio_busy 	= MEIZU_MCU_BUSY;
}

static struct i2c_board_info __initdata i2c_devs_mxhub[] = {
	{
		I2C_BOARD_INFO("mx_hub", 0x43),
		.platform_data	= &mx_hub_platformdata,
		.irq = IRQ_EINT(8),
	},
};
#endif

#if defined(CONFIG_KEYBOARD_GPIO)

#define MEIZU_WAKEUP_DEBUG /*just for test by lt*/
#ifdef MEIZU_WAKEUP_DEBUG
void meizu_wakeup_debug_set_val(int func, int val)
{
	int gpio_voldown = EXYNOS5410_GPX3(5);
	int gpio_volup = EXYNOS5410_GPX1(7);

	switch (func) {
	case 0x1:
		/*output*/
		s3c_gpio_cfgpin(gpio_voldown, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(gpio_voldown, S3C_GPIO_PULL_NONE);
		gpio_direction_output(gpio_voldown, (val & 0x1));

		s3c_gpio_cfgpin(gpio_volup, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(gpio_volup, S3C_GPIO_PULL_NONE);
		gpio_direction_output(gpio_volup, (val & (0x1 << 1)));
		break;

	case 0xF:
		/*interrupt*/
		s3c_gpio_cfgpin(gpio_voldown, S3C_GPIO_SFN(0xF));
		s3c_gpio_setpull(gpio_voldown, S3C_GPIO_PULL_UP);

		s3c_gpio_cfgpin(gpio_volup, S3C_GPIO_SFN(0xF));
		s3c_gpio_setpull(gpio_volup, S3C_GPIO_PULL_UP);
		break;

	default:
		pr_err("%s: not supported !!!!\n", __func__);
	}

	flush_cache_all();
	mdelay(1);
}
#else
void meizu_wakeup_debug_set_val(int func, int val) {}
#endif

static struct gpio_keys_button M6x_gpio_keys_tables[] = {
	{
		.code			= KEY_POWER,
		.desc			= "gpio-keys: KEY_POWER",
		.type			= EV_KEY,
		.active_low		= 1,
		.wakeup			= 1,
		.debounce_interval	= 1,
	}, {
		.code			= KEY_HOME,
		.desc			= "gpio-keys: KEY_HOME",
		.type			= EV_KEY,
		.active_low		= 1,
		.wakeup			= 1,
		.debounce_interval	= 1,
	}, {
		.code			= KEY_VOLUMEDOWN,
		.desc			= "gpio-keys: KEY_VOLUMEDOWN",
		.type			= EV_KEY,
		.active_low		= 1,
		.debounce_interval	= 100,
	}, {
		.code			= KEY_VOLUMEUP,
		.desc			= "gpio-keys: KEY_VOLUMEUP",
		.type			= EV_KEY,
		.active_low		= 1,
		.debounce_interval	= 100,
	},
};

static void __init gpio_keyboard_rt_init_res(void) {
	M6x_gpio_keys_tables[0].gpio = MEIZU_KEYOFF_IRQ;
	M6x_gpio_keys_tables[1].gpio = MEIZU_HOME_IRQ;
	M6x_gpio_keys_tables[2].gpio = MEIZU_VOLDOWN_IRQ;
	M6x_gpio_keys_tables[3].gpio = MEIZU_VOLUP_IRQ;
}

static struct gpio_keys_platform_data M6x_gpio_keys_data = {
	.buttons		= M6x_gpio_keys_tables,
	.nbuttons		= ARRAY_SIZE(M6x_gpio_keys_tables),
};

static struct platform_device m6x_gpio_keys = {
	.name			= "gpio-keys",
	.dev			= {
		.platform_data	= &M6x_gpio_keys_data,
	},
};
#endif

static struct platform_device *m6x_input_devices[] __initdata = {
#ifdef CONFIG_KEYBOARD_GPIO
	&m6x_gpio_keys,
#endif
};

#ifdef CONFIG_MACH_M65
static void __init m65_input_i2c_bus_init(void) {
	int ret = 0;

	ret = platform_device_register(&exynos5_device_hs_i2c3);
	if (ret) {
		platform_device_unregister(&exynos5_device_hs_i2c3);
		pr_err("%s: Error register i2c7!\n", __func__);
		return ;
	}

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_I2C
	dxs_rt_init_res();
#endif	
	exynos5_hs_i2c3_set_platdata(NULL);
	i2c_register_board_info(7, i2c_devs7, ARRAY_SIZE(i2c_devs7));
}
#endif

#ifdef CONFIG_MACH_M69
static void __init m69_input_i2c_bus_init(void) {
	int ret = 0;

	ret = platform_device_register(&s3c_device_i2c1);
	if (ret) {
		platform_device_unregister(&s3c_device_i2c1);
		pr_err("%s: Error register i2c1!\n", __func__);
		return ;
	}

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_I2C
	dxs_rt_init_res();
#endif
	s3c_i2c1_set_platdata(NULL);
	i2c_register_board_info(1, i2c_devs7, ARRAY_SIZE(i2c_devs7));
}
#endif

void __init exynos5_m6x_input_init(void)
{
	int ret = -EINVAL;

	if (machine_is_m65()) {
		m65_input_i2c_bus_init();
	} else if (machine_is_m69()) {
		m69_input_i2c_bus_init();
	} else {
		pr_err("Error: Please Configure Machine Version.\r\n");
		BUG();
	}

#ifdef	CONFIG_MFD_MX_HUB  
	/*mx sensor hub */
	mx_hub_rt_init_res();

#ifdef CONFIG_MX_HUB_IIC18
	ret = platform_device_register(&m6x_device_i2c18);
	if (ret) {
		platform_device_unregister(&m6x_device_i2c18);
		pr_err("%s: Error register i2c18!\n", __func__);
	}

	i2c_register_board_info(18, i2c_devs_mxhub, ARRAY_SIZE(i2c_devs_mxhub));
#else
	ret = platform_device_register(&s3c_device_i2c3);
	if (ret) {
		platform_device_unregister(&s3c_device_i2c3);
		pr_err("%s: Error register i2c3!\n", __func__);
	}

	i2c_register_board_info(3, i2c_devs_mxhub, ARRAY_SIZE(i2c_devs_mxhub));
#endif
#endif

#if defined(CONFIG_KEYBOARD_GPIO)
	gpio_keyboard_rt_init_res();
#endif
	platform_add_devices(m6x_input_devices,
			ARRAY_SIZE(m6x_input_devices));
}
