/* linux/include/media/m6mo_platform.h
 *
 * Copyright (c) 2011 Meizu Co., Ltd.
 *		http://www.meizu.com/
 *
 * Driver for Milbeaut-S2 from Fujisu Electronics
 * supporting MIPI CSI-2
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#ifndef __MS2R_PLATFORM_DATA__H__
#define __MS2R_PLATFORM_DATA__H__

struct ms2r_platform_data {
	unsigned int default_width;
	unsigned int default_height;
	unsigned int pixelformat;
	int freq;	/* MCLK in KHz */

	/* This ISP supports Parallel & CSI-2 */
	int is_mipi;

	/* these members should be set */
	int (*init_gpio)(struct device *dev);
	int (*init_clock)(struct device *dev);
	int (*set_isp_power)(bool enable);
	int (*set_sensor_power)(int cam_id, bool enable);
	void (*reset)(void);
	int (*clock_enable)(struct device *dev, bool enable);
};

#endif

