/*
 * Samsung Exynos5 SoC series FIMC-IS driver
 *
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef FIMC_IS_GROUP_MGR_H
#define FIMC_IS_GROUP_MGR_H

#include "fimc-is-device.h"

#define TRACE_GROUP
#define GROUP_ID_ISP		0
#define GROUP_ID_DIS		1
#define GROUP_ID_MAX		2
#define GROUP_ID_INVALID	(0xFF)
#define GROUP_ID_SHIFT		(16)
#define GROUP_ID_MASK		(0xFFFF0000)

enum fimc_is_group_state {
	FIMC_IS_GROUP_OPEN,
	FIMC_IS_GROUP_ACTIVE,
	FIMC_IS_GROUP_READY,
	FIMC_IS_GROUP_RUN,
};

struct fimc_is_group {
	struct list_head	list;
	struct fimc_is_group	*next;
	struct fimc_is_subdev	leader;

	bool			init_done;
	bool			done;

	u32			id;
	unsigned long		state;
};

struct fimc_is_groupmgr {
	struct fimc_is_group		*group_isp;
	struct fimc_is_group		*group_dis;
	struct mutex			mutex_group;

	u32				group_skip_cnt;
	u32				group_cnt;

	void				*private_data;
};

int fimc_is_groupmgr_open(struct fimc_is_groupmgr *groupmgr,
	struct fimc_is_group *group_isp,
	struct fimc_is_group *group_dis,
	void *private_data);
int fimc_is_group_open(struct fimc_is_groupmgr *groupmgr,
	struct fimc_is_group *group, enum is_entry entry,
	struct fimc_is_video_ctx *video_ctx);
int fimc_is_group_close(struct fimc_is_groupmgr *groupmgr);
int fimc_is_group_process_on(struct fimc_is_groupmgr *groupmgr,
	struct fimc_is_group *group);
int fimc_is_group_process_off(struct fimc_is_groupmgr *groupmgr,
	struct fimc_is_group *group);
int fimc_is_group_buffer_start(struct fimc_is_groupmgr *groupmgr,
	struct fimc_is_group *group);
int fimc_is_group_done(struct fimc_is_groupmgr *groupmgr,
	struct fimc_is_group *group);
#endif
