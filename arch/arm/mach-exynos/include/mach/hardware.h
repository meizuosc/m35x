/* linux/arch/arm/mach-exynos4/include/mach/hardware.h
 *
 * Copyright (c) 2010-2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS4 - Hardware support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_HARDWARE_H_
#define __ASM_ARCH_HARDWARE_H_

#include <linux/bootinfo.h>

#define MEIZU_MDM_XMM6260	0x0
#define MEIZU_MDM_SC8803G	0x1

extern unsigned int meizu_board_mdm_type(void);
extern bool meizu_board_have_nfc(void);

/*mach-m6x.c*/
extern u32 m6x_get_board_version(void);
#endif /* __ASM_ARCH_HARDWARE_H */
