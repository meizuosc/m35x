/* arch/arm/mach-exynos/include/mach/modem.h
 *
 * Copyright (c) 2011 Meizu Technology Co., Ltd.
 *		http://www.meizu.com/
 *
 * Based on arch/arm/mach-s5p6442/include/mach/io.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __MODEM_H__
#define __MODEM_H__

#ifdef CONFIG_UMTS_MODEM_XMM6260
extern void modem_set_slave_wakeup(int state);
extern void modem_set_active_state(int state);
#else
static inline void modem_set_active_state(int state) {}
static inline void modem_set_slave_wakeup(int state) {}
#endif

#endif
