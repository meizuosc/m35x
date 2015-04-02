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

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <mach/videonode.h>
#include <media/exynos_mc.h>
#include <linux/cma.h>
#include <asm/cacheflush.h>
#include <asm/pgtable.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/videodev2.h>
#include <linux/videodev2_exynos_camera.h>
#include <linux/videodev2_exynos_media.h>
#include <linux/v4l2-mediabus.h>

#include <mach/map.h>
#include <mach/regs-clock.h>

#include "fimc-is-core.h"
#include "fimc-is-err.h"
#include "fimc-is-video.h"
#include "fimc-is-framemgr.h"
#include "fimc-is-groupmgr.h"

int fimc_is_groupmgr_open(struct fimc_is_groupmgr *groupmgr,
	struct fimc_is_group *group_isp,
	struct fimc_is_group *group_dis,
	void *private_data)
{
	int ret = 0;

	mutex_init(&groupmgr->mutex_group);
	groupmgr->private_data = private_data;
	groupmgr->group_skip_cnt = 0;
	groupmgr->group_cnt = 0;

	groupmgr->group_isp = group_isp;
	groupmgr->group_dis = group_dis;
	group_isp->id = GROUP_ID_ISP;
	group_dis->id = GROUP_ID_DIS;

	group_isp->next = group_dis;
	group_dis->next = NULL;

	return ret;
}

int fimc_is_group_open(struct fimc_is_groupmgr *groupmgr,
	struct fimc_is_group *group, enum is_entry entry,
	struct fimc_is_video_ctx *video_ctx)
{
	int ret = 0;
	struct fimc_is_subdev *leader;

	groupmgr->group_skip_cnt = 2;
	groupmgr->group_cnt++;
	set_bit(FIMC_IS_GROUP_OPEN, &group->state);
	clear_bit(FIMC_IS_GROUP_ACTIVE, &group->state);
	clear_bit(FIMC_IS_GROUP_READY, &group->state);
	clear_bit(FIMC_IS_GROUP_RUN, &group->state);

	/* subdev init */
	leader = &group->leader;
	mutex_init(&leader->mutex_state);
	leader->entry = entry;
	leader->video_ctx = video_ctx;
	clear_bit(FIMC_IS_ISDEV_DSTART, &leader->state);

	return ret;
}

int fimc_is_group_close(struct fimc_is_groupmgr *groupmgr)
{
	int ret = 0;
	return ret;
}

int fimc_is_group_process_on(struct fimc_is_groupmgr *groupmgr,
	struct fimc_is_group *group)
{
	int ret = 0;

	set_bit(FIMC_IS_GROUP_ACTIVE, &group->state);

	return ret;
}

int fimc_is_group_process_off(struct fimc_is_groupmgr *groupmgr,
	struct fimc_is_group *group)
{
	int ret = 0;

	clear_bit(FIMC_IS_GROUP_ACTIVE, &group->state);

	return ret;
}

int fimc_is_group_buffer_start(struct fimc_is_groupmgr *groupmgr,
	struct fimc_is_group *group)
{
	int ret = 0;
	unsigned long flags;
	struct fimc_is_group *group_isp;
	struct fimc_is_group *group_dis;
	struct fimc_is_framemgr *isp_framemgr, *dis_framemgr;
	struct fimc_is_framemgr *framemgr;

	mutex_lock(&groupmgr->mutex_group);

	if (!groupmgr) {
		err("groupmgr is NULL");
		ret = -EINVAL;
		goto exit;
	}

	if (!group) {
		err("group is NULL");
		ret = -EINVAL;
		goto exit;
	}

	framemgr = &group->leader.framemgr;
	group_isp = groupmgr->group_isp;
	group_dis = groupmgr->group_dis;
	isp_framemgr = &group_isp->leader.framemgr;
	dis_framemgr = &group_dis->leader.framemgr;

	/* 1. check group ready */
	framemgr_e_barrier_irqs(framemgr, index, flags);

	if (framemgr->frame_request_cnt > 0)
		set_bit(FIMC_IS_GROUP_READY, &group->state);
	else
		clear_bit(FIMC_IS_GROUP_READY, &group->state);

	framemgr_x_barrier_irqr(framemgr, index, flags);

	/*
	 * TODO: this code will be used in the future
	 * if (groupmgr->group_skip_cnt)
	 * 	groupmgr->group_skip_cnt--;
	 * else {
	 * 	if (!test_bit(FIMC_IS_GROUP_READY, &group_isp->state) ||
	 * 		!test_bit(FIMC_IS_GROUP_READY, &group_dis->state)) {
	 * 		err("all group is not ready");
	 * 		goto exit;
	 * 	}
	 * }
	 */

	if (test_bit(FIMC_IS_GROUP_READY, &group_isp->state) &&
		!test_bit(FIMC_IS_GROUP_RUN, &group_isp->state)) {
		ret = fimc_is_ischain_isp_callback(groupmgr->private_data);
		if (ret) {
			err("fimc_is_ischain_isp_callback is fail");
			goto exit;
		}

		set_bit(FIMC_IS_GROUP_RUN, &group_isp->state);

		framemgr_e_barrier_irqs(isp_framemgr, 0, flags);
		if (!isp_framemgr->frame_request_cnt)
			clear_bit(FIMC_IS_GROUP_READY, &group_isp->state);
		framemgr_x_barrier_irqr(isp_framemgr, 0, flags);
	}

	if (test_bit(FIMC_IS_GROUP_READY, &group_dis->state) &&
		!test_bit(FIMC_IS_GROUP_RUN, &group_dis->state)) {
		ret = fimc_is_ischain_dis_callback(groupmgr->private_data);
		if (ret) {
			err("fimc_is_ischain_dis_callback is fail");
			goto exit;
		}

		set_bit(FIMC_IS_GROUP_RUN, &group_dis->state);

		framemgr_e_barrier_irqs(dis_framemgr, 0, flags);
		if (!dis_framemgr->frame_request_cnt)
			clear_bit(FIMC_IS_GROUP_READY, &group_dis->state);
		framemgr_x_barrier_irqr(dis_framemgr, 0, flags);
	}

exit:
	mutex_unlock(&groupmgr->mutex_group);
	return ret;
}

int fimc_is_group_done(struct fimc_is_groupmgr *groupmgr,
	struct fimc_is_group *group)
{
	int ret = 0;

	if (!group) {
		err("group is NULL");
		ret = -EINVAL;
		goto exit;
	}

	clear_bit(FIMC_IS_GROUP_RUN, &group->state);

exit:
	return ret;
}
