/*
 * Samsung Exynos5 SoC series FIMC-IS driver
 *
 * exynos5 fimc-is core functions
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
#include <linux/videodev2_exynos_camera.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>

#include "fimc-is-core.h"
#include "fimc-is-param.h"
#include "fimc-is-cmd.h"
#include "fimc-is-regs.h"
#include "fimc-is-err.h"
#include "fimc-is-framemgr.h"

static struct device *is_dev= NULL;

struct device *get_is_dev(void)
{
	return is_dev;
}

static int fimc_is_ischain_allocmem(struct fimc_is_core *this)
{
	int ret = 0;
	void *fw_cookie;

	dbg_ischain("Allocating memory for FIMC-IS firmware.\n");

	fw_cookie = vb2_ion_private_alloc(this->mem.alloc_ctx,
				FIMC_IS_A5_MEM_SIZE + FIMC_IS_A5_SEN_SIZE +
#ifdef ENABLE_ODC
				SIZE_ODC_INTERNAL_BUF * NUM_ODC_INTERNAL_BUF +
#endif
#ifdef ENABLE_VDIS
				SIZE_DIS_INTERNAL_BUF * NUM_DIS_INTERNAL_BUF +
#endif
#ifdef ENABLE_TDNR
				SIZE_DNR_INTERNAL_BUF * NUM_DNR_INTERNAL_BUF +
#endif
#ifdef ENABLE_3AA
				SIZE_ISP_INTERNAL_BUF * NUM_ISP_INTERNAL_BUF +
				SIZE_3AA_INTERNAL_BUF * NUM_3AA_INTERNAL_BUF +
#endif
				0);

	if (IS_ERR(fw_cookie)) {
		err("Allocating bitprocessor buffer failed");
		fw_cookie = NULL;
		ret = -ENOMEM;
		goto exit;
	}

	ret = vb2_ion_dma_address(fw_cookie, &this->minfo.dvaddr);
	if ((ret < 0) || (this->minfo.dvaddr  & FIMC_IS_FW_BASE_MASK)) {
		err("The base memory is not aligned to 64MB.");
		vb2_ion_private_free(fw_cookie);
		this->minfo.dvaddr = 0;
		fw_cookie = NULL;
		ret = -EIO;
		goto exit;
	}
	dbg_ischain("Device vaddr = %08x , size = %08x\n",
		this->minfo.dvaddr, FIMC_IS_A5_MEM_SIZE);

	this->minfo.kvaddr = (u32)vb2_ion_private_vaddr(fw_cookie);
	if (IS_ERR((void *)this->minfo.kvaddr)) {
		err("Bitprocessor memory remap failed");
		vb2_ion_private_free(fw_cookie);
		this->minfo.kvaddr = 0;
		fw_cookie = NULL;
		ret = -EIO;
		goto exit;
	}

exit:
	dbg_ischain("Virtual address for FW: %08lx\n",
		(long unsigned int)this->minfo.kvaddr);
	this->minfo.fw_cookie = fw_cookie;

	return ret;
}

static int fimc_is_ishcain_initmem(struct fimc_is_core *this)
{
	int ret = 0;
	u32 offset;

	dbg_ischain("fimc_is_init_mem - ION\n");

	ret = fimc_is_ischain_allocmem(this);
	if (ret) {
		err("Couldn't alloc for FIMC-IS firmware\n");
		ret = -ENOMEM;
		goto exit;
	}

	offset = FW_SHARED_OFFSET;
	this->minfo.dvaddr_fshared = this->minfo.dvaddr + offset;
	this->minfo.kvaddr_fshared = this->minfo.kvaddr + offset;

	offset = FIMC_IS_A5_MEM_SIZE - FIMC_IS_REGION_SIZE;
	this->minfo.dvaddr_region = this->minfo.dvaddr + offset;
	this->minfo.kvaddr_region = this->minfo.kvaddr + offset;

	offset = FIMC_IS_A5_MEM_SIZE;
#ifdef ENABLE_ODC
	this->minfo.dvaddr_odc = this->minfo.dvaddr + offset;
	this->minfo.kvaddr_odc = this->minfo.kvaddr + offset;
	offset += (SIZE_ODC_INTERNAL_BUF * NUM_ODC_INTERNAL_BUF);
#else
	this->minfo.dvaddr_odc = 0;
	this->minfo.kvaddr_odc = 0;
#endif

#ifdef ENABLE_VDIS
	this->minfo.dvaddr_dis = this->minfo.dvaddr + offset;
	this->minfo.kvaddr_dis = this->minfo.kvaddr + offset;
	offset += (SIZE_DIS_INTERNAL_BUF * NUM_DIS_INTERNAL_BUF);
#else
	this->minfo.dvaddr_dis = 0;
	this->minfo.kvaddr_dis = 0;
#endif

#ifdef ENABLE_TDNR
	this->minfo.dvaddr_3dnr = this->minfo.dvaddr + offset;
	this->minfo.kvaddr_3dnr = this->minfo.kvaddr + offset;
	offset += (SIZE_DNR_INTERNAL_BUF * NUM_DNR_INTERNAL_BUF);
#else
	this->minfo.dvaddr_3dnr = 0;
	this->minfo.kvaddr_3dnr = 0;
#endif

#ifdef ENABLE_3AA
	this->minfo.dvaddr_isp = this->minfo.dvaddr + offset;
	this->minfo.kvaddr_isp = this->minfo.kvaddr + offset;
#else
	this->minfo.dvaddr_isp = 0;
	this->minfo.kvaddr_isp = 0;
#endif

	dbg_ischain("fimc_is_init_mem done\n");

exit:
	return ret;
}

static struct fimc_is_sensor_dev *to_fimc_is_sensor_dev
				(struct v4l2_subdev *sdev)
{
	return container_of(sdev, struct fimc_is_sensor_dev, sd);
}

static struct fimc_is_front_dev *to_fimc_is_front_dev(struct v4l2_subdev *sdev)
{
	return container_of(sdev, struct fimc_is_front_dev, sd);
}

static struct fimc_is_back_dev *to_fimc_is_back_dev(struct v4l2_subdev *sdev)
{
	return container_of(sdev, struct fimc_is_back_dev, sd);
}

static int fimc_is_sensor_s_stream(struct v4l2_subdev *sd, int enable)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_front_s_stream(struct v4l2_subdev *sd, int enable)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_back_s_stream(struct v4l2_subdev *sd, int enable)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_sensor_subdev_get_fmt(struct v4l2_subdev *sd,
					struct v4l2_subdev_fh *fh,
					struct v4l2_subdev_format *fmt)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_sensor_subdev_set_fmt(struct v4l2_subdev *sd,
					struct v4l2_subdev_fh *fh,
					struct v4l2_subdev_format *fmt)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_sensor_subdev_get_crop(struct v4l2_subdev *sd,
					struct v4l2_subdev_fh *fh,
					struct v4l2_subdev_crop *crop)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_sensor_subdev_set_crop(struct v4l2_subdev *sd,
					struct v4l2_subdev_fh *fh,
					struct v4l2_subdev_crop *crop)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_front_subdev_get_fmt(struct v4l2_subdev *sd,
					struct v4l2_subdev_fh *fh,
					struct v4l2_subdev_format *fmt)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_front_subdev_set_fmt(struct v4l2_subdev *sd,
					struct v4l2_subdev_fh *fh,
					struct v4l2_subdev_format *fmt)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_front_subdev_get_crop(struct v4l2_subdev *sd,
					struct v4l2_subdev_fh *fh,
					struct v4l2_subdev_crop *crop)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_front_subdev_set_crop(struct v4l2_subdev *sd,
					struct v4l2_subdev_fh *fh,
					struct v4l2_subdev_crop *crop)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_back_subdev_get_fmt(struct v4l2_subdev *sd,
					struct v4l2_subdev_fh *fh,
					struct v4l2_subdev_format *fmt)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_back_subdev_set_fmt(struct v4l2_subdev *sd,
					struct v4l2_subdev_fh *fh,
					struct v4l2_subdev_format *fmt)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_back_subdev_get_crop(struct v4l2_subdev *sd,
					struct v4l2_subdev_fh *fh,
					struct v4l2_subdev_crop *crop)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_back_subdev_set_crop(struct v4l2_subdev *sd,
					struct v4l2_subdev_fh *fh,
					struct v4l2_subdev_crop *crop)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_sensor_s_ctrl(struct v4l2_subdev *sd,
				struct v4l2_control *ctrl)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_sensor_g_ctrl(struct v4l2_subdev *sd,
				struct v4l2_control *ctrl)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_front_s_ctrl(struct v4l2_subdev *sd,
				struct v4l2_control *ctrl)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_front_g_ctrl(struct v4l2_subdev *sd,
				struct v4l2_control *ctrl)
{
	dbg("%s\n", __func__);
	return 0;
}
static int fimc_is_back_s_ctrl(struct v4l2_subdev *sd,
				struct v4l2_control *ctrl)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_back_g_ctrl(struct v4l2_subdev *sd,
				struct v4l2_control *ctrl)
{
	dbg("%s\n", __func__);
	return 0;
}

static struct v4l2_subdev_pad_ops fimc_is_sensor_pad_ops = {
	.get_fmt	= fimc_is_sensor_subdev_get_fmt,
	.set_fmt	= fimc_is_sensor_subdev_set_fmt,
	.get_crop	= fimc_is_sensor_subdev_get_crop,
	.set_crop	= fimc_is_sensor_subdev_set_crop,
};

static struct v4l2_subdev_pad_ops fimc_is_front_pad_ops = {
	.get_fmt	= fimc_is_front_subdev_get_fmt,
	.set_fmt	= fimc_is_front_subdev_set_fmt,
	.get_crop	= fimc_is_front_subdev_get_crop,
	.set_crop	= fimc_is_front_subdev_set_crop,
};

static struct v4l2_subdev_pad_ops fimc_is_back_pad_ops = {
	.get_fmt	= fimc_is_back_subdev_get_fmt,
	.set_fmt	= fimc_is_back_subdev_set_fmt,
	.get_crop	= fimc_is_back_subdev_get_crop,
	.set_crop	= fimc_is_back_subdev_set_crop,
};

static struct v4l2_subdev_video_ops fimc_is_sensor_video_ops = {
	.s_stream	= fimc_is_sensor_s_stream,
};

static struct v4l2_subdev_video_ops fimc_is_front_video_ops = {
	.s_stream	= fimc_is_front_s_stream,
};

static struct v4l2_subdev_video_ops fimc_is_back_video_ops = {
	.s_stream	= fimc_is_back_s_stream,
};

static struct v4l2_subdev_core_ops fimc_is_sensor_core_ops = {
	.s_ctrl	= fimc_is_sensor_s_ctrl,
	.g_ctrl = fimc_is_sensor_g_ctrl,
};

static struct v4l2_subdev_core_ops fimc_is_front_core_ops = {
	.s_ctrl	= fimc_is_front_s_ctrl,
	.g_ctrl = fimc_is_front_g_ctrl,
};

static struct v4l2_subdev_core_ops fimc_is_back_core_ops = {
	.s_ctrl	= fimc_is_back_s_ctrl,
	.g_ctrl = fimc_is_back_g_ctrl,
};

static struct v4l2_subdev_ops fimc_is_sensor_subdev_ops = {
	.pad	= &fimc_is_sensor_pad_ops,
	.video	= &fimc_is_sensor_video_ops,
	.core	= &fimc_is_sensor_core_ops,
};

static struct v4l2_subdev_ops fimc_is_front_subdev_ops = {
	.pad	= &fimc_is_front_pad_ops,
	.video	= &fimc_is_front_video_ops,
	.core	= &fimc_is_front_core_ops,
};

static struct v4l2_subdev_ops fimc_is_back_subdev_ops = {
	.pad	= &fimc_is_back_pad_ops,
	.video	= &fimc_is_back_video_ops,
	.core	= &fimc_is_back_core_ops,
};

static int fimc_is_sensor_init_formats(struct v4l2_subdev *sd,
					struct v4l2_subdev_fh *fh)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_sensor_subdev_close(struct v4l2_subdev *sd,
					struct v4l2_subdev_fh *fh)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_sensor_subdev_registered(struct v4l2_subdev *sd)
{
	dbg("%s\n", __func__);
	return 0;
}

static void fimc_is_sensor_subdev_unregistered(struct v4l2_subdev *sd)
{
	dbg("%s\n", __func__);
}

static int fimc_is_front_init_formats(struct v4l2_subdev *sd,
					struct v4l2_subdev_fh *fh)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_front_subdev_close(struct v4l2_subdev *sd,
					 struct v4l2_subdev_fh *fh)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_front_subdev_registered(struct v4l2_subdev *sd)
{
	dbg("%s\n", __func__);
	return 0;
}

static void fimc_is_front_subdev_unregistered(struct v4l2_subdev *sd)
{
	dbg("%s\n", __func__);
}

static int fimc_is_back_init_formats(struct v4l2_subdev *sd,
					 struct v4l2_subdev_fh *fh)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_back_subdev_close(struct v4l2_subdev *sd,
					struct v4l2_subdev_fh *fh)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_back_subdev_registered(struct v4l2_subdev *sd)
{
	dbg("%s\n", __func__);
	return 0;
}

static void fimc_is_back_subdev_unregistered(struct v4l2_subdev *sd)
{
	dbg("%s\n", __func__);
}

static const struct v4l2_subdev_internal_ops
			fimc_is_sensor_v4l2_internal_ops = {
	.open = fimc_is_sensor_init_formats,
	.close = fimc_is_sensor_subdev_close,
	.registered = fimc_is_sensor_subdev_registered,
	.unregistered = fimc_is_sensor_subdev_unregistered,
};

static const struct v4l2_subdev_internal_ops fimc_is_front_v4l2_internal_ops = {
	.open = fimc_is_front_init_formats,
	.close = fimc_is_front_subdev_close,
	.registered = fimc_is_front_subdev_registered,
	.unregistered = fimc_is_front_subdev_unregistered,
};

static const struct v4l2_subdev_internal_ops fimc_is_back_v4l2_internal_ops = {
	.open = fimc_is_back_init_formats,
	.close = fimc_is_back_subdev_close,
	.registered = fimc_is_back_subdev_registered,
	.unregistered = fimc_is_back_subdev_unregistered,
};

static int fimc_is_sensor_link_setup(struct media_entity *entity,
			    const struct media_pad *local,
			    const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct fimc_is_sensor_dev *fimc_is_sensor;

	dbg("++%s\n", __func__);
	dbg("local->index : %d\n", local->index);
	dbg("media_entity_type(remote->entity) : %d\n",
				media_entity_type(remote->entity));

	fimc_is_sensor = to_fimc_is_sensor_dev(sd);

	switch (local->index | media_entity_type(remote->entity)) {
	case FIMC_IS_SENSOR_PAD_SOURCE_FRONT | MEDIA_ENT_T_V4L2_SUBDEV:
		if (flags & MEDIA_LNK_FL_ENABLED)
			fimc_is_sensor->output = FIMC_IS_SENSOR_OUTPUT_FRONT;
		else
			fimc_is_sensor->output = FIMC_IS_SENSOR_OUTPUT_NONE;
		break;

	default:
		v4l2_err(sd, "%s : ERR link\n", __func__);
		return -EINVAL;
	}
	dbg("--%s\n", __func__);
	return 0;
}

static int fimc_is_front_link_setup(struct media_entity *entity,
			    const struct media_pad *local,
			    const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct fimc_is_front_dev *fimc_is_front = to_fimc_is_front_dev(sd);

	dbg("++%s\n", __func__);
	dbg("local->index : %d\n", local->index);

	switch (local->index | media_entity_type(remote->entity)) {
	case FIMC_IS_FRONT_PAD_SINK | MEDIA_ENT_T_V4L2_SUBDEV:
		dbg("fimc_is_front sink pad\n");
		if (flags & MEDIA_LNK_FL_ENABLED) {
			if (fimc_is_front->input
				!= FIMC_IS_FRONT_INPUT_NONE) {
				dbg("BUSY\n");
				return -EBUSY;
			}
			if (remote->index == FIMC_IS_SENSOR_PAD_SOURCE_FRONT)
				fimc_is_front->input
					= FIMC_IS_FRONT_INPUT_SENSOR;
		} else {
			fimc_is_front->input = FIMC_IS_FRONT_INPUT_NONE;
		}
		break;

	case FIMC_IS_FRONT_PAD_SOURCE_BACK | MEDIA_ENT_T_V4L2_SUBDEV:
		if (flags & MEDIA_LNK_FL_ENABLED)
			fimc_is_front->output |= FIMC_IS_FRONT_OUTPUT_BACK;
		else
			fimc_is_front->output = FIMC_IS_FRONT_OUTPUT_NONE;
		break;

	case FIMC_IS_FRONT_PAD_SOURCE_BAYER | MEDIA_ENT_T_DEVNODE:
		if (flags & MEDIA_LNK_FL_ENABLED)
			fimc_is_front->output |= FIMC_IS_FRONT_OUTPUT_BAYER;
		else
			fimc_is_front->output = FIMC_IS_FRONT_OUTPUT_NONE;
		break;
	case FIMC_IS_FRONT_PAD_SOURCE_SCALERC | MEDIA_ENT_T_DEVNODE:
		if (flags & MEDIA_LNK_FL_ENABLED)
			fimc_is_front->output |= FIMC_IS_FRONT_OUTPUT_SCALERC;
		else
			fimc_is_front->output = FIMC_IS_FRONT_OUTPUT_NONE;
		break;

	default:
		v4l2_err(sd, "%s : ERR link\n", __func__);
		return -EINVAL;
	}
	dbg("--%s\n", __func__);
	return 0;
}

static int fimc_is_back_link_setup(struct media_entity *entity,
			    const struct media_pad *local,
			    const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct fimc_is_back_dev *fimc_is_back = to_fimc_is_back_dev(sd);

	dbg("++%s\n", __func__);
	switch (local->index | media_entity_type(remote->entity)) {
	case FIMC_IS_BACK_PAD_SINK | MEDIA_ENT_T_V4L2_SUBDEV:
		dbg("fimc_is_back sink pad\n");
		if (flags & MEDIA_LNK_FL_ENABLED) {
			if (fimc_is_back->input != FIMC_IS_BACK_INPUT_NONE) {
				dbg("BUSY\n");
				return -EBUSY;
			}
			if (remote->index == FIMC_IS_FRONT_PAD_SOURCE_BACK)
				fimc_is_back->input = FIMC_IS_BACK_INPUT_FRONT;
		} else {
			fimc_is_back->input = FIMC_IS_FRONT_INPUT_NONE;
		}
		break;
	case FIMC_IS_BACK_PAD_SOURCE_3DNR | MEDIA_ENT_T_DEVNODE:
		if (flags & MEDIA_LNK_FL_ENABLED)
			fimc_is_back->output |= FIMC_IS_BACK_OUTPUT_3DNR;
		else
			fimc_is_back->output = FIMC_IS_FRONT_OUTPUT_NONE;
		break;
	case FIMC_IS_BACK_PAD_SOURCE_SCALERP | MEDIA_ENT_T_DEVNODE:
		if (flags & MEDIA_LNK_FL_ENABLED)
			fimc_is_back->output |= FIMC_IS_BACK_OUTPUT_SCALERP;
		else
			fimc_is_back->output = FIMC_IS_FRONT_OUTPUT_NONE;
		break;
	default:
		v4l2_err(sd, "%s : ERR link\n", __func__);
		return -EINVAL;
	}
	dbg("--%s\n", __func__);
	return 0;
}

static const struct media_entity_operations fimc_is_sensor_media_ops = {
	.link_setup = fimc_is_sensor_link_setup,
};

static const struct media_entity_operations fimc_is_front_media_ops = {
	.link_setup = fimc_is_front_link_setup,
};

static const struct media_entity_operations fimc_is_back_media_ops = {
	.link_setup = fimc_is_back_link_setup,
};

int fimc_is_pipeline_s_stream_preview(struct media_entity *start_entity, int on)
{
	struct media_pad *pad = &start_entity->pads[0];
	struct v4l2_subdev *back_sd;
	struct v4l2_subdev *front_sd;
	struct v4l2_subdev *sensor_sd;
	int	ret;

	dbg("--%s\n", __func__);

	pad = media_entity_remote_source(pad);
	if (media_entity_type(pad->entity) != MEDIA_ENT_T_V4L2_SUBDEV
			|| pad == NULL)
		dbg("cannot find back entity\n");

	back_sd = media_entity_to_v4l2_subdev(pad->entity);

	pad = &pad->entity->pads[0];

	pad = media_entity_remote_source(pad);
	if (media_entity_type(pad->entity) != MEDIA_ENT_T_V4L2_SUBDEV
			|| pad == NULL)
		dbg("cannot find front entity\n");

	front_sd = media_entity_to_v4l2_subdev(pad->entity);

	pad = &pad->entity->pads[0];

	pad = media_entity_remote_source(pad);
	if (media_entity_type(pad->entity) != MEDIA_ENT_T_V4L2_SUBDEV
			|| pad == NULL)
		dbg("cannot find sensor entity\n");

	sensor_sd = media_entity_to_v4l2_subdev(pad->entity);

	if (on) {

		ret = v4l2_subdev_call(sensor_sd, video, s_stream, 1);
		if (ret < 0 && ret != -ENOIOCTLCMD)
			return ret;

		ret = v4l2_subdev_call(front_sd, video, s_stream, 1);
		if (ret < 0 && ret != -ENOIOCTLCMD)
			return ret;

		ret = v4l2_subdev_call(back_sd, video, s_stream, 1);
		if (ret < 0 && ret != -ENOIOCTLCMD)
			return ret;

	} else {
		ret = v4l2_subdev_call(back_sd, video, s_stream, 0);
		if (ret < 0 && ret != -ENOIOCTLCMD)
			return ret;
		ret = v4l2_subdev_call(front_sd, video, s_stream, 0);
		if (ret < 0 && ret != -ENOIOCTLCMD)
			return ret;
		ret = v4l2_subdev_call(sensor_sd, video, s_stream, 0);
	}

	return ret == -ENOIOCTLCMD ? 0 : ret;
}


static int fimc_is_suspend(struct device *dev)
{
	printk(KERN_INFO "FIMC_IS Suspend\n");
	return 0;
}

static int fimc_is_resume(struct device *dev)
{
	printk(KERN_INFO "FIMC_IS Resume\n");
	return 0;
}

int fimc_is_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct fimc_is_core *core
		= (struct fimc_is_core *)platform_get_drvdata(pdev);
	int ret = 0;

	printk(KERN_INFO "FIMC_IS runtime suspend\n");

#if defined(CONFIG_VIDEOBUF2_ION)
	if (core->mem.alloc_ctx)
		vb2_ion_detach_iommu(core->mem.alloc_ctx);
#endif

	if (core->pdata->clk_off) {
		core->pdata->clk_off(core->pdev);
	      printk(" ==>[MMKIM] FIMC_IS runtime suspend : clk off\n");
	} else {
		err("failed to clock on\n");
		ret = -EINVAL;
		goto end;
	}

end:
	pm_relax(dev);
	return ret;
}

int fimc_is_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct fimc_is_core *core
		= (struct fimc_is_core *)platform_get_drvdata(pdev);

	pm_stay_awake(dev);
	printk(KERN_INFO "FIMC_IS runtime resume\n");

	/* 1. Enable MIPI */
	enable_mipi();

	/* 2. Low clock setting */
	if (core->pdata->clk_cfg) {
		core->pdata->clk_cfg(core->pdev);
	      printk(" ==>[MMKIM] FIMC_IS runtime resume : clk cfg\n");
	} else {
		err("failed to config clock\n");
		goto err;
	}

	/* 4. Clock on */
	if (core->pdata->clk_on) {
		core->pdata->clk_on(core->pdev);
	      printk(" ==>[MMKIM] FIMC_IS runtime resume : clk off\n");
	} else {
		err("failed to clock on\n");
		goto err;
	}

#if defined(CONFIG_SOC_EXYNOS5250)
	/* 5. High clock setting */
	if (core->pdata->clk_cfg) {
		core->pdata->clk_cfg(core->pdev);
	} else {
		err("failed to config clock\n");
		goto err;
	}
#endif

#if defined(CONFIG_VIDEOBUF2_ION)
	if (core->mem.alloc_ctx)
		vb2_ion_attach_iommu(core->mem.alloc_ctx);
#endif
	return 0;

err:
	pm_relax(dev);
	return -EINVAL;
}

static int fimc_is_get_md_callback(struct device *dev, void *p)
{
	struct exynos_md **md_list = p;
	struct exynos_md *md = NULL;

	md = dev_get_drvdata(dev);

	if (md)
		*(md_list + md->id) = md;

	return 0; /* non-zero value stops iteration */
}

static struct exynos_md *fimc_is_get_md(enum mdev_node node)
{
	struct device_driver *drv;
	struct exynos_md *md[MDEV_MAX_NUM] = {NULL,};
	int ret;

	drv = driver_find(MDEV_MODULE_NAME, &platform_bus_type);
	if (!drv)
		return ERR_PTR(-ENODEV);

	ret = driver_for_each_device(drv, NULL, &md[0],
				     fimc_is_get_md_callback);
	/* put_driver(drv); */

	return ret ? NULL : md[node];

}

static int fimc_is_probe(struct platform_device *pdev)
{
	struct resource *mem_res;
	struct resource *regs_res;
	struct fimc_is_core *core;
	int ret = -ENODEV;

	printk(KERN_INFO "%s\n", __func__);

	core = kzalloc(sizeof(struct fimc_is_core), GFP_KERNEL);
	if (!core)
		return -ENOMEM;

	is_dev = &pdev->dev;

	core->pdev = pdev;
	core->pdata = pdev->dev.platform_data;
	core->id = pdev->id;
	core->ref_cnt = -1;
	core->debug_cnt = 0;
	device_init_wakeup(&pdev->dev, true);

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem_res) {
		dev_err(&pdev->dev, "Failed to get io memory region\n");
		goto p_err1;
	}

	regs_res = request_mem_region(mem_res->start, resource_size(mem_res),
				      pdev->name);
	if (!regs_res) {
		dev_err(&pdev->dev, "Failed to request io memory region\n");
		goto p_err1;
	}

	core->regs_res = regs_res;
	core->regs =  ioremap_nocache(mem_res->start, resource_size(mem_res));
	if (!core->regs) {
		dev_err(&pdev->dev, "Failed to remap io region\n");
		goto p_err2;
	}

	core->irq = platform_get_irq(pdev, 0);
	if (core->irq < 0) {
		dev_err(&pdev->dev, "Failed to get irq\n");
		goto p_err3;
	}

	core->mdev = fimc_is_get_md(MDEV_ISP);
	if (IS_ERR_OR_NULL(core->mdev))
		goto p_err3;

	fimc_is_mem_probe(&core->mem,
		core->pdev);

	fimc_is_interface_probe(&core->interface,
		(u32)core->regs,
		core->irq,
		core);

#if 1
	/*sensor entity*/
	v4l2_subdev_init(&core->dev_sensor.sd, &fimc_is_sensor_subdev_ops);
	core->dev_sensor.sd.owner = THIS_MODULE;
	core->dev_sensor.sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(core->dev_sensor.sd.name, sizeof(core->dev_sensor.sd.name),
					"%s\n",	FIMC_IS_SENSOR_ENTITY_NAME);

	core->dev_sensor.pads.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_init(&core->dev_sensor.sd.entity, 1,
				&core->dev_sensor.pads, 0);
	if (ret < 0)
		goto p_err3;

	fimc_is_sensor_init_formats(&core->dev_sensor.sd, NULL);

	core->dev_sensor.sd.internal_ops = &fimc_is_sensor_v4l2_internal_ops;
	core->dev_sensor.sd.entity.ops = &fimc_is_sensor_media_ops;

	ret = v4l2_device_register_subdev(&core->mdev->v4l2_dev,
						&core->dev_sensor.sd);
	if (ret)
		goto p_err3;
	/* This allows to retrieve the platform device id by the host driver */
	v4l2_set_subdevdata(&core->dev_sensor.sd, pdev);


	/*front entity*/
	v4l2_subdev_init(&core->front.sd, &fimc_is_front_subdev_ops);
	core->front.sd.owner = THIS_MODULE;
	core->front.sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(core->front.sd.name, sizeof(core->front.sd.name), "%s\n",
					FIMC_IS_FRONT_ENTITY_NAME);

	core->front.pads[FIMC_IS_FRONT_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	core->front.pads[FIMC_IS_FRONT_PAD_SOURCE_BACK].flags
							= MEDIA_PAD_FL_SOURCE;
	core->front.pads[FIMC_IS_FRONT_PAD_SOURCE_BAYER].flags
							= MEDIA_PAD_FL_SOURCE;
	core->front.pads[FIMC_IS_FRONT_PAD_SOURCE_SCALERC].flags
							= MEDIA_PAD_FL_SOURCE;
	ret = media_entity_init(&core->front.sd.entity, FIMC_IS_FRONT_PADS_NUM,
				core->front.pads, 0);
	if (ret < 0)
		goto p_err3;

	fimc_is_front_init_formats(&core->front.sd, NULL);

	core->front.sd.internal_ops = &fimc_is_front_v4l2_internal_ops;
	core->front.sd.entity.ops = &fimc_is_front_media_ops;

	ret = v4l2_device_register_subdev(&core->mdev->v4l2_dev,
							&core->front.sd);
	if (ret)
		goto p_err3;
	/* This allows to retrieve the platform device id by the host driver */
	v4l2_set_subdevdata(&core->front.sd, pdev);


	/*back entity*/
	v4l2_subdev_init(&core->back.sd, &fimc_is_back_subdev_ops);
	core->back.sd.owner = THIS_MODULE;
	core->back.sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(core->back.sd.name, sizeof(core->back.sd.name), "%s\n",
					FIMC_IS_BACK_ENTITY_NAME);

	core->back.pads[FIMC_IS_BACK_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	core->back.pads[FIMC_IS_BACK_PAD_SOURCE_3DNR].flags
							= MEDIA_PAD_FL_SOURCE;
	core->back.pads[FIMC_IS_BACK_PAD_SOURCE_SCALERP].flags
							= MEDIA_PAD_FL_SOURCE;
	ret = media_entity_init(&core->back.sd.entity, FIMC_IS_BACK_PADS_NUM,
				core->back.pads, 0);
	if (ret < 0)
		goto p_err3;

	fimc_is_front_init_formats(&core->back.sd, NULL);

	core->back.sd.internal_ops = &fimc_is_back_v4l2_internal_ops;
	core->back.sd.entity.ops = &fimc_is_back_media_ops;

	ret = v4l2_device_register_subdev(&core->mdev->v4l2_dev,
		&core->back.sd);
	if (ret)
		goto p_err3;

	v4l2_set_subdevdata(&core->back.sd, pdev);
#endif

	/*real start*/

	/*front video entity - scalerC */
	fimc_is_scc_video_probe(core);

	/* back video entity - scalerP*/
	fimc_is_scp_video_probe(core);

	/* back video entity - bayer*/
	fimc_is_sensor_video_probe(core);

	/* back video entity - isp*/
	fimc_is_isp_video_probe(core);

	/* dis video entity - vdis capture*/
	fimc_is_vdisc_video_probe(core);

	/* dis video entity - vdis output*/
	fimc_is_vdiso_video_probe(core);

	platform_set_drvdata(pdev, core);

	/* register subdev nodes*/
	ret = v4l2_device_register_subdev_nodes(&core->mdev->v4l2_dev);
	if (ret)
		err("v4l2_device_register_subdev_nodes failed\n");

	fimc_is_ishcain_initmem(core);

#if defined(CONFIG_PM_RUNTIME)
	pm_runtime_enable(&pdev->dev);
	//pm_runtime_get_sync(&pdev->dev); //mm78.kim -- delete..
#endif

	dbg("%s : fimc_is_front_%d probe success\n", __func__, pdev->id);
	return 0;

p_err3:
	iounmap(core->regs);
p_err2:
	release_mem_region(regs_res->start, resource_size(regs_res));
p_err1:
	kfree(core);
	return ret;
}

static int fimc_is_remove(struct platform_device *pdev)
{
	dbg("%s\n", __func__);
	return 0;
}

static const struct dev_pm_ops fimc_is_pm_ops = {
	.suspend		= fimc_is_suspend,
	.resume			= fimc_is_resume,
	.runtime_suspend	= fimc_is_runtime_suspend,
	.runtime_resume		= fimc_is_runtime_resume,
};

static struct platform_driver fimc_is_driver = {
	.probe		= fimc_is_probe,
	.remove	= __devexit_p(fimc_is_remove),
	.driver = {
		.name	= FIMC_IS_MODULE_NAME,
		.owner	= THIS_MODULE,
		.pm	= &fimc_is_pm_ops,
	}
};

static int __init fimc_is_init(void)
{
	int ret = platform_driver_register(&fimc_is_driver);
	if (ret)
		err("platform_driver_register failed: %d\n", ret);
	return ret;
}

static void __exit fimc_is_exit(void)
{
	platform_driver_unregister(&fimc_is_driver);
}
module_init(fimc_is_init);
module_exit(fimc_is_exit);

MODULE_AUTHOR("Jiyoung Shin<idon.shin@samsung.com>");
MODULE_DESCRIPTION("Exynos FIMC_IS2 driver");
MODULE_LICENSE("GPL");
