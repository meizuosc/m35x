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
#include <linux/videodev2_exynos_media.h>
#include <linux/videodev2_exynos_camera.h>
#include <linux/v4l2-mediabus.h>

#include "fimc-is-core.h"
#include "fimc-is-param.h"
#include "fimc-is-cmd.h"
#include "fimc-is-regs.h"
#include "fimc-is-err.h"
#include "fimc-is-video.h"
#include "fimc-is-metadata.h"

const struct v4l2_file_operations fimc_is_vdiso_video_fops;
const struct v4l2_ioctl_ops fimc_is_vdiso_video_ioctl_ops;
const struct vb2_ops fimc_is_vdiso_qops;

int fimc_is_vdiso_video_probe(void *data)
{
	int ret = 0;
	struct fimc_is_core *core = (struct fimc_is_core *)data;
	struct fimc_is_video_vdiso *video = &core->video_vdiso;

	dbg_vdiso("%s\n", __func__);

	ret = fimc_is_video_probe(&video->common,
		data,
		FIMC_IS_VIDEO_VDISO_NAME,
		FIMC_IS_VIDEO_NUM_VDISO,
		&fimc_is_vdiso_video_fops,
		&fimc_is_vdiso_video_ioctl_ops);

	if (ret != 0)
		dev_err(&(core->pdev->dev),
		"%s::Failed to fimc_is_video_probe()\n", __func__);

	return ret;
}

static int fimc_is_vdiso_video_open(struct file *file)
{
	int ret = 0;
	struct fimc_is_core *core = video_drvdata(file);
	struct fimc_is_device_ischain *ischain
		= core->ischain[core->ischain_index];
	struct fimc_is_groupmgr *groupmgr = &ischain->groupmgr;
	struct fimc_is_group *group_dis = &ischain->group_dis;
	struct fimc_is_video_ctx *video_ctx = NULL;

	dbg_vdiso("%s\n", __func__);

	video_ctx = kzalloc(sizeof *video_ctx, GFP_KERNEL);
	if (video_ctx == NULL) {
		dev_err(&(core->pdev->dev),
			"%s:: Failed to kzalloc\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}

	file->private_data = video_ctx;

	fimc_is_video_open(video_ctx,
		ischain,
		VIDEO_VDISO_READY_BUFFERS,
		&(core->video_vdiso.common),
		V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		&fimc_is_vdiso_qops,
		core->mem.vb2->ops);

	fimc_is_group_open(groupmgr, group_dis, ENTRY_DIS, video_ctx);

exit:
	return ret;
}

static int fimc_is_vdiso_video_close(struct file *file)
{
	int ret = 0;
	struct fimc_is_video_ctx *video;
	struct fimc_is_device_ischain *ischain;
	struct fimc_is_group *group;
	struct fimc_is_subdev *dis;

	printk(KERN_INFO "%s\n", __func__);

	video = file->private_data;
	if (!video) {
		err("video is NULL");
		ret = -EINVAL;
		goto exit;
	}

	ischain = video->device;
	if (!ischain) {
		err("ischain is NULL");
		ret = -EINVAL;
		goto exit;
	}
	group = &ischain->group_dis;
	dis = &group->leader;

	if (test_bit(FIMC_IS_VIDEO_STREAM_ON, &video->state))
		clear_bit(FIMC_IS_VIDEO_STREAM_ON, &video->state);

	fimc_is_video_close(video, &dis->framemgr);

exit:
	kfree(video);
	file->private_data = NULL;
	return ret;
}

static unsigned int fimc_is_vdiso_video_poll(struct file *file,
	struct poll_table_struct *wait)
{
	struct fimc_is_video_ctx *video_ctx = file->private_data;

	return vb2_poll(&video_ctx->vbq, file, wait);
}

static int fimc_is_vdiso_video_mmap(struct file *file,
	struct vm_area_struct *vma)
{
	struct fimc_is_video_ctx *video_ctx = file->private_data;

	return vb2_mmap(&video_ctx->vbq, vma);
}

static int fimc_is_vdiso_video_querycap(struct file *file, void *fh,
					struct v4l2_capability *cap)
{
	struct fimc_is_core *core = video_drvdata(file);

	strncpy(cap->driver, core->pdev->name, sizeof(cap->driver) - 1);

	dbg_vdiso("%s(devname : %s)\n", __func__, cap->driver);
	strncpy(cap->card, core->pdev->name, sizeof(cap->card) - 1);
	cap->bus_info[0] = 0;
	cap->version = KERNEL_VERSION(1, 0, 0);
	cap->capabilities = V4L2_CAP_STREAMING
				| V4L2_CAP_VIDEO_CAPTURE
				| V4L2_CAP_VIDEO_CAPTURE_MPLANE;

	return 0;
}

static int fimc_is_vdiso_video_enum_fmt_mplane(struct file *file, void *priv,
	struct v4l2_fmtdesc *f)
{
	/* Todo: add enum format control code */
	return 0;
}

static int fimc_is_vdiso_video_get_format_mplane(struct file *file, void *fh,
	struct v4l2_format *format)
{
	/* Todo: add get format control code */
	return 0;
}

static int fimc_is_vdiso_video_set_format_mplane(struct file *file, void *fh,
	struct v4l2_format *format)
{
	int ret = 0;
	struct fimc_is_video_ctx *video_ctx = file->private_data;

	dbg_vdiso("%s\n", __func__);

	ret = fimc_is_video_set_format_mplane(video_ctx, format);

	dbg_vdiso("req w : %d req h : %d\n",
		video_ctx->frame.width,
		video_ctx->frame.height);

	return ret;
}

static int fimc_is_vdiso_video_reqbufs(struct file *file, void *priv,
	struct v4l2_requestbuffers *buf)
{
	int ret;
	struct fimc_is_video_ctx *video_ctx = file->private_data;
	struct fimc_is_device_ischain *ischain = video_ctx->device;
	struct fimc_is_group *group = &ischain->group_dis;
	struct fimc_is_subdev *leader = &group->leader;

	dbg_vdiso("%s(buffers : %d)\n", __func__, buf->count);

	ret = fimc_is_video_reqbufs(video_ctx, &leader->framemgr, buf);
	if (ret)
		err("fimc_is_video_reqbufs is fail(error %d)", ret);

	return ret;
}

static int fimc_is_vdiso_video_querybuf(struct file *file, void *priv,
						struct v4l2_buffer *buf)
{
	int ret;
	struct fimc_is_video_ctx *video_ctx = file->private_data;

	dbg_vdiso("%s\n", __func__);
	ret = vb2_querybuf(&video_ctx->vbq, buf);

	return ret;
}

static int fimc_is_vdiso_video_qbuf(struct file *file, void *priv,
	struct v4l2_buffer *buf)
{
	int ret = 0;
	struct fimc_is_video_ctx *video_ctx = file->private_data;

#ifdef DBG_STREAMING
	dbg_vdiso("%s\n", __func__);
#endif

	if (test_bit(FIMC_IS_VIDEO_STREAM_ON, &video_ctx->state)) {
		ret = fimc_is_video_qbuf(video_ctx, buf);
	} else {
		err("stream off state, can NOT qbuf");
		ret = -EINVAL;
	}

	return ret;
}

static int fimc_is_vdiso_video_dqbuf(struct file *file, void *priv,
	struct v4l2_buffer *buf)
{
	int ret = 0;
	struct fimc_is_video_ctx *video_ctx = file->private_data;
	bool blocking = file->f_flags & O_NONBLOCK;

	ret = fimc_is_video_dqbuf(video_ctx, buf, blocking);

#ifdef DBG_STREAMING
	dbg_vdiso("%s(index : %d)\n", __func__, buf->index);
#endif

	return ret;
}

static int fimc_is_vdiso_video_streamon(struct file *file, void *priv,
	enum v4l2_buf_type type)
{
	int ret = 0;
	struct fimc_is_video_ctx *video_ctx = file->private_data;

	dbg_vdiso("%s\n", __func__);

	ret = fimc_is_video_streamon(video_ctx, type);

	return ret;
}

static int fimc_is_vdiso_video_streamoff(struct file *file, void *priv,
	enum v4l2_buf_type type)
{
	int ret = 0;
	struct fimc_is_video_ctx *video_ctx = file->private_data;

	dbg_vdiso("%s\n", __func__);

	ret = fimc_is_video_streamoff(video_ctx, type);

	return ret;
}

static int fimc_is_vdiso_video_enum_input(struct file *file, void *priv,
						struct v4l2_input *input)
{
	struct fimc_is_core *isp = video_drvdata(file);
	struct exynos5_fimc_is_sensor_info *sensor_info;

	sensor_info = isp->pdata->sensor_info[input->index];

	dbg_vdiso("index(%d) sensor(%s)\n",
		input->index, sensor_info->sensor_name);
	dbg_vdiso("pos(%d) sensor_id(%d)\n",
		sensor_info->sensor_position, sensor_info->sensor_id);
	dbg_vdiso("csi_id(%d) flite_id(%d)\n",
		sensor_info->csi_id, sensor_info->flite_id);
	dbg_vdiso("i2c_ch(%d)\n", sensor_info->i2c_channel);

	if (input->index >= FIMC_IS_MAX_CAMIF_CLIENTS)
		return -EINVAL;

	input->type = V4L2_INPUT_TYPE_CAMERA;

	strncpy(input->name, sensor_info->sensor_name,
					FIMC_IS_MAX_SENSOR_NAME_LEN);
	return 0;
}

static int fimc_is_vdiso_video_g_input(struct file *file, void *priv,
	unsigned int *input)
{
	/* Todo: add get input control code */
	return 0;
}

static int fimc_is_vdiso_video_s_input(struct file *file, void *priv,
	unsigned int input)
{
	/* Todo: add set input control code */
	return 0;
}

static int fimc_is_vdiso_video_s_ctrl(struct file *file, void *priv,
	struct v4l2_control *ctrl)
{
	/* Todo: add set control code */
	return 0;
}

static int fimc_is_vdiso_video_g_ctrl(struct file *file, void *priv,
	struct v4l2_control *ctrl)
{
	/* Todo: add get control code */
	return 0;
}

static int fimc_is_vdiso_video_g_ext_ctrl(struct file *file, void *priv,
	struct v4l2_ext_controls *ctrls)
{
	/* Todo: add get extra control code */
	return 0;
}

const struct v4l2_file_operations fimc_is_vdiso_video_fops = {
	.owner		= THIS_MODULE,
	.open		= fimc_is_vdiso_video_open,
	.release	= fimc_is_vdiso_video_close,
	.poll		= fimc_is_vdiso_video_poll,
	.unlocked_ioctl	= video_ioctl3,
	.mmap		= fimc_is_vdiso_video_mmap,
};

const struct v4l2_ioctl_ops fimc_is_vdiso_video_ioctl_ops = {
	.vidioc_querycap		= fimc_is_vdiso_video_querycap,
	.vidioc_enum_fmt_vid_out_mplane	= fimc_is_vdiso_video_enum_fmt_mplane,
	.vidioc_g_fmt_vid_out_mplane	= fimc_is_vdiso_video_get_format_mplane,
	.vidioc_s_fmt_vid_out_mplane	= fimc_is_vdiso_video_set_format_mplane,
	.vidioc_reqbufs			= fimc_is_vdiso_video_reqbufs,
	.vidioc_querybuf		= fimc_is_vdiso_video_querybuf,
	.vidioc_qbuf			= fimc_is_vdiso_video_qbuf,
	.vidioc_dqbuf			= fimc_is_vdiso_video_dqbuf,
	.vidioc_streamon		= fimc_is_vdiso_video_streamon,
	.vidioc_streamoff		= fimc_is_vdiso_video_streamoff,
	.vidioc_enum_input		= fimc_is_vdiso_video_enum_input,
	.vidioc_g_input			= fimc_is_vdiso_video_g_input,
	.vidioc_s_input			= fimc_is_vdiso_video_s_input,
	.vidioc_s_ctrl			= fimc_is_vdiso_video_s_ctrl,
	.vidioc_g_ctrl			= fimc_is_vdiso_video_g_ctrl,
	.vidioc_g_ext_ctrls		= fimc_is_vdiso_video_g_ext_ctrl,
};

static int fimc_is_vdiso_queue_setup(struct vb2_queue *vq,
	const struct v4l2_format *fmt,
	unsigned int *num_buffers, unsigned int *num_planes,
	unsigned int sizes[],
	void *allocators[])
{
	int ret = 0;
	struct fimc_is_video_ctx *video_ctx = vq->drv_priv;

	dbg_vdiso("%s\n", __func__);

	ret = fimc_is_video_queue_setup(video_ctx,
		num_planes,
		sizes,
		allocators);

	dbg_vdiso("(num_planes : %d)(size : %d)\n",
		(int)*num_planes, (int)sizes[0]);

	return ret;
}

static int fimc_is_vdiso_buffer_prepare(struct vb2_buffer *vb)
{
	/* Todo: add buffer prepare control code */
	return 0;
}

static inline void fimc_is_vdiso_wait_prepare(struct vb2_queue *q)
{
	/* Todo: add wait prepare control code */
}

static inline void fimc_is_vdiso_wait_finish(struct vb2_queue *q)
{
	/* Todo: add wait finish control code */
}

static int fimc_is_vdiso_start_streaming(struct vb2_queue *q,
	unsigned int count)
{
	int ret = 0;
	struct fimc_is_video_ctx *video_ctx = q->drv_priv;
	struct fimc_is_device_ischain *ischain = video_ctx->device;

	dbg_vdiso("%s\n", __func__);

	if (!test_bit(FIMC_IS_VIDEO_STREAM_ON, &video_ctx->state)) {
		set_bit(FIMC_IS_VIDEO_STREAM_ON, &video_ctx->state);
		fimc_is_ischain_vdiso_start(ischain);
	} else {
		err("already stream on or buffer is not ready(%ld)",
			video_ctx->state);
		ret = -EINVAL;
	}

	return ret;
}

static int fimc_is_vdiso_stop_streaming(struct vb2_queue *q)
{
	int ret = 0;
	struct fimc_is_video_ctx *video_ctx = q->drv_priv;
	struct fimc_is_device_ischain *ischain = video_ctx->device;

	dbg_vdiso("%s\n", __func__);

	if (test_bit(FIMC_IS_VIDEO_STREAM_ON, &video_ctx->state)) {
		clear_bit(FIMC_IS_VIDEO_STREAM_ON, &video_ctx->state);
		clear_bit(FIMC_IS_VIDEO_BUFFER_PREPARED, &video_ctx->state);
		fimc_is_ischain_vdiso_stop(ischain);
	} else {
		err("already stream off");
		ret = -EINVAL;
	}

	return ret;
}

static void fimc_is_vdiso_buffer_queue(struct vb2_buffer *vb)
{
	u32 index;
	struct fimc_is_video_ctx *video_ctx = vb->vb2_queue->drv_priv;
	struct fimc_is_device_ischain *ischain = video_ctx->device;
	struct fimc_is_group *group = &ischain->group_dis;
	struct fimc_is_subdev *leader = &group->leader;

	index = vb->v4l2_buf.index;

#ifdef DBG_STREAMING
	dbg_vdiso("%s(%d)\n", __func__, index);
#endif

	fimc_is_video_buffer_queue(video_ctx, vb, &leader->framemgr);
	fimc_is_ischain_vdiso_buffer_queue(ischain, vb->v4l2_buf.index);
}

static int fimc_is_vdiso_buffer_finish(struct vb2_buffer *vb)
{
	int ret = 0;
	struct fimc_is_video_ctx *video_ctx = vb->vb2_queue->drv_priv;
	struct fimc_is_device_ischain *ischain = video_ctx->device;
	struct fimc_is_group *group = &ischain->group_dis;
	struct fimc_is_subdev *leader = &group->leader;

#ifdef DBG_STREAMING
	dbg_vdiso("%s(%d)\n", __func__, vb->v4l2_buf.index);
#endif

	ret = fimc_is_subdev_buffer_finish(leader, vb->v4l2_buf.index);

	return ret;
}

const struct vb2_ops fimc_is_vdiso_qops = {
	.queue_setup		= fimc_is_vdiso_queue_setup,
	.buf_prepare		= fimc_is_vdiso_buffer_prepare,
	.buf_queue		= fimc_is_vdiso_buffer_queue,
	.buf_finish		= fimc_is_vdiso_buffer_finish,
	.wait_prepare		= fimc_is_vdiso_wait_prepare,
	.wait_finish		= fimc_is_vdiso_wait_finish,
	.start_streaming	= fimc_is_vdiso_start_streaming,
	.stop_streaming		= fimc_is_vdiso_stop_streaming,
};

