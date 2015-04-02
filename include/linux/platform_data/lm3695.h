/*
 * LM3695 Backlight Driver
 *
 *			Copyright (C) 2013 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef _LM3695_H
#define _LM3695_H

enum lm3695_boost_freq {
	LM3695_BOOST_FREQ_443KHZ,
	LM3695_BOOST_FREQ_887KHZ,
	LM3695_BOOST_FREQ_500KHZ,
	LM3695_BOOST_FREQ_1000KHZ,
};

enum lm3695_ovp {
	LM3695_OVP_16V,
	LM3695_OVP_21V,
};

enum lm3695_string_mode {
	LM3695_DUAL_STRINGS,	/* LED1 and LED2 */
	LM3695_LED1_STRING,
};

enum lm3695_ramp_rate {
	LM3695_RAMP_125us,
	LM3695_RAMP_252us,
	LM3695_RAMP_500us,
	LM3695_RAMP_1ms,
	LM3695_RAMP_2ms,
	LM3695_RAMP_4ms,
	LM3695_RAMP_8ms,
	LM3695_RAMP_16ms,
	LM3695_RAMP_32ms,
	LM3695_RAMP_64ms,
	LM3695_RAMP_128ms = 0x0F,
};

/*
 * struct lm3695_platform_data
 * @name : Backlight driver name. If it is not defined, default name is set.
 * @initial_brightness : initial value of backlight brightness
 * @en_gpio : GPIO number for HWEN external pin
 * @boost_freq : Boost switching frequency selection
 * @ovp : OVP(Over-Voltage Protection) selection
 * @string : String mode
 * @disable_ramp : set true if current ramp is disabled
 * @ramp : Ramp settins. Only valid if disable_ramp is false
 */
struct lm3695_platform_data {
	/* Configurable Backlight Driver */
	const char *name;
	int initial_brightness;

	/* GPIO pin number for HWEN */
	int en_gpio;

	/* General Purpose Settings */
	enum lm3695_boost_freq boost_freq;
	enum lm3695_ovp ovp;
	enum lm3695_string_mode string;

	/* Ramp Rate Settings */
	bool disable_ramp;
	enum lm3695_ramp_rate ramp;
};

#endif
