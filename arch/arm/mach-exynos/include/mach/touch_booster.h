/* mach/touch_booster.h
 *
 * Copyright (C) 2012 ZhuHai MEIZU Technology Co., Ltd.
 *	  http://www.meizu.com
 *
 * Meizu touch booster interface
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __H_TOUCH_BOOSTER__
#define __H_TOUCH_BOOSTER__

#include <linux/sysdev.h>
#include <linux/input.h>
struct tb_private_info {
	struct class tb_class;
	unsigned int boost_time;
	unsigned int *boost_cpufreq;
	unsigned int boost_debug;
	struct input_handle handle;
	struct workqueue_struct *wq;
	struct delayed_work boost_work;
	atomic_t boost_lock;
};
#ifdef CONFIG_TOUCH_BOOTSER
extern int app_is_launching;

static inline int is_app_launching(void)
{
	return app_is_launching;
}
#else
#define is_app_launching()	(0)
#endif
#endif
