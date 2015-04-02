/*
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __MACH_EXYNOS_BOARD_TF4_H
#define __MACH_EXYNOS_BOARD_TF4_H


enum board_version_type {
	BOARD_EVT = 0,
	BOARD_DVT,
	BOARD_PVT,
};
void exynos5_tf4_clock_init(void);
void exynos5_tf4_mmc_init(void);
void exynos5_tf4_power_init(void);
void exynos5_tf4_audio_init(void);
void exynos5_tf4_usb_init(void);
void exynos5_tf4_input_init(void);
void exynos5_tf4_media_init(void);
void exynos5_tf4_display_init(void);
enum board_version_type get_board_version(void);

#endif
