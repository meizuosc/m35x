/*
 * Copyright (C) 2012 Meizu Technology Co.Ltd, Zhuhai, China
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __GPIO_COMMON_H
#define __GPIO_COMMON_H

#define MX_FACTORY_TEST_BT	0
#define MX_FACTORY_TEST_CAMERA	1
#define MX_FACTORY_TEST_ALL	2

#define GPIO_SETPIN_LOW 	0
#define GPIO_SETPIN_HI   	1
#define GPIO_SETPIN_NONE	2

#define GPIO_PULL_NONE		S3C_GPIO_PULL_NONE
#define GPIO_PULL_DOWN		S3C_GPIO_PULL_DOWN
#define GPIO_PULL_UP		S3C_GPIO_PULL_UP

#define GPIO_DRVSTR_LV1		S5P_GPIO_DRVSTR_LV1
#define GPIO_DRVSTR_LV2		S5P_GPIO_DRVSTR_LV2
#define GPIO_DRVSTR_LV3		S5P_GPIO_DRVSTR_LV3
#define GPIO_DRVSTR_LV4		S5P_GPIO_DRVSTR_LV4

#define GPIO_PD_OUT0       ((__force s5p_gpio_pd_pull_t)0x00)
#define GPIO_PD_OUT1       ((__force s5p_gpio_pd_pull_t)0x01)
#define GPIO_PD_INPUT      ((__force s5p_gpio_pd_pull_t)0x02)
#define GPIO_PD_PREV       ((__force s5p_gpio_pd_pull_t)0x03)

struct gpio_info{
	unsigned int pin;
	unsigned int type;
	unsigned int data;
	unsigned int pull;
	unsigned int drv;
};

struct gpio_pd_info{
	unsigned char *name;
	unsigned int pin;
	unsigned int type;
	unsigned int pull;
};

extern int (*mx_is_factory_test_mode)(int type);
extern int (*mx_set_factory_test_led)(int on);

#endif /*__GPIO_COMMON_H*/
