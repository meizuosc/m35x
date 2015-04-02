/*
 * Samsung Exynos5 SoC series FIMC-IS driver
 *
 * exynos5 fimc-is video functions
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef FIMC_IS_VIDEO_VDIS_CAPTURE_H
#define FIMC_IS_VIDEO_VDIS_CAPTURE_H

#include "fimc-is-video.h"

#define VIDEO_VDISC_READY_BUFFERS 4

struct fimc_is_video_vdisc {
	struct fimc_is_video_common common;
};

int fimc_is_vdisc_video_probe(void *data);

#endif
