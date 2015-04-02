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

#include "fimc-is-time.h"
#include "fimc-is-core.h"
#include "fimc-is-param.h"
#include "fimc-is-cmd.h"
#include "fimc-is-regs.h"
#include "fimc-is-err.h"

#ifdef USE_FRAME_SYNC
#define SPARE_PLANE 1
#define SPARE_SIZE (16*1024)
#else
#define SPARE_PLANE 0
#define SPARE_SIZE 0
#endif

struct fimc_is_fmt fimc_is_formats[] = {
	 {
		.name		= "YUV 4:2:2 packed, YCbYCr",
		.pixelformat	= V4L2_PIX_FMT_YUYV,
		.num_planes	= 1 + SPARE_PLANE,
		.mbus_code	= V4L2_MBUS_FMT_YUYV8_2X8,
	}, {
		.name		= "YUV 4:2:2 packed, CbYCrY",
		.pixelformat	= V4L2_PIX_FMT_UYVY,
		.num_planes	= 1 + SPARE_PLANE,
		.mbus_code	= V4L2_MBUS_FMT_UYVY8_2X8,
	}, {
		.name		= "YUV 4:2:2 planar, Y/Cb/Cr",
		.pixelformat	= V4L2_PIX_FMT_YUV422P,
		.num_planes	= 1 + SPARE_PLANE,
	}, {
		.name		= "YUV 4:2:0 planar, YCbCr",
		.pixelformat	= V4L2_PIX_FMT_YUV420,
		.num_planes	= 1 + SPARE_PLANE,
	}, {
		.name		= "YUV 4:2:0 planar, YCbCr",
		.pixelformat	= V4L2_PIX_FMT_YVU420,
		.num_planes	= 1 + SPARE_PLANE,
	}, {
		.name		= "YUV 4:2:0 planar, Y/CbCr",
		.pixelformat	= V4L2_PIX_FMT_NV12,
		.num_planes	= 1 + SPARE_PLANE,
	}, {
		.name		= "YUV 4:2:0 planar, Y/CrCb",
		.pixelformat	= V4L2_PIX_FMT_NV21,
		.num_planes	= 1 + SPARE_PLANE,
	}, {
		.name		= "YUV 4:2:0 non-contiguous 2-planar, Y/CbCr",
		.pixelformat	= V4L2_PIX_FMT_NV12M,
		.num_planes	= 2 + SPARE_PLANE,
	}, {
		.name		= "YVU 4:2:0 non-contiguous 2-planar, Y/CrCb",
		.pixelformat	= V4L2_PIX_FMT_NV21M,
		.num_planes	= 2 + SPARE_PLANE,
	}, {
		.name		= "YUV 4:2:0 non-contiguous 3-planar, Y/Cb/Cr",
		.pixelformat	= V4L2_PIX_FMT_YUV420M,
		.num_planes	= 3 + SPARE_PLANE,
	}, {
		.name		= "YUV 4:2:0 non-contiguous 3-planar, Y/Cr/Cb",
		.pixelformat	= V4L2_PIX_FMT_YVU420M,
		.num_planes	= 3 + SPARE_PLANE,
	}, {
		.name		= "BAYER 10 bit",
		.pixelformat	= V4L2_PIX_FMT_SBGGR10,
		.num_planes	= 2,
	}, {
		.name		= "BAYER 12 bit",
		.pixelformat	= V4L2_PIX_FMT_SBGGR12,
		.num_planes	= 2,
	}, {
		.name		= "BAYER 16 bit",
		.pixelformat	= V4L2_PIX_FMT_SBGGR16,
		.num_planes	= 2,
	},
};

struct fimc_is_fmt *fimc_is_find_format(u32 *pixelformat,
	u32 *mbus_code, int index)
{
	struct fimc_is_fmt *fmt, *def_fmt = NULL;
	unsigned int i;

	if (index >= ARRAY_SIZE(fimc_is_formats))
		return NULL;

	for (i = 0; i < ARRAY_SIZE(fimc_is_formats); ++i) {
		fmt = &fimc_is_formats[i];
		if (pixelformat && fmt->pixelformat == *pixelformat)
			return fmt;
		if (mbus_code && fmt->mbus_code == *mbus_code)
			return fmt;
		if (index == i)
			def_fmt = fmt;
	}
	return def_fmt;

}

void fimc_is_set_plane_size(struct fimc_is_frame *frame, unsigned int sizes[])
{
	u32 plane;
	u32 width[FIMC_IS_MAX_PLANES];

	for (plane = 0; plane < FIMC_IS_MAX_PLANES; ++plane)
		width[plane] = frame->width + frame->width_stride[plane];

	switch (frame->format.pixelformat) {
	case V4L2_PIX_FMT_YUYV:
		dbg("V4L2_PIX_FMT_YUYV(w:%d)(h:%d)\n",
				frame->width, frame->height);
		sizes[0] = width[0]*frame->height*2;
#ifdef USE_FRAME_SYNC
		sizes[1] = SPARE_SIZE;
#endif
		break;
	case V4L2_PIX_FMT_NV12M:
		dbg("V4L2_PIX_FMT_NV12M(w:%d)(h:%d)\n",
				frame->width, frame->height);
		sizes[0] = width[0]*frame->height;
		sizes[1] = width[1]*frame->height/2;
#ifdef USE_FRAME_SYNC
		sizes[2] = SPARE_SIZE;
#endif
		break;
	case V4L2_PIX_FMT_YUV420M:
	case V4L2_PIX_FMT_YVU420M:
		dbg("V4L2_PIX_FMT_YVU420M(w:%d)(h:%d)\n",
				frame->width, frame->height);
		sizes[0] = width[0]*frame->height;
		sizes[1] = width[1]*frame->height/4;
		sizes[2] = width[2]*frame->height/4;
#ifdef USE_FRAME_SYNC
		sizes[3] = SPARE_SIZE;
#endif
		break;
	case V4L2_PIX_FMT_SBGGR10:
		dbg("V4L2_PIX_FMT_SBGGR10(w:%d)(h:%d)\n",
				frame->width, frame->height);
		sizes[0] = frame->width*frame->height*2;
		sizes[1] = SPARE_SIZE;
		break;
	case V4L2_PIX_FMT_SBGGR16:
		dbg("V4L2_PIX_FMT_SBGGR16(w:%d)(h:%d)\n",
				frame->width, frame->height);
		sizes[0] = frame->width*frame->height*2;
		sizes[1] = SPARE_SIZE;
		break;
	case V4L2_PIX_FMT_SBGGR12:
		dbg("V4L2_PIX_FMT_SBGGR12(w:%d)(h:%d)\n",
				frame->width, frame->height);
		sizes[0] = frame->width*frame->height*2;
		sizes[1] = SPARE_SIZE;
		break;
	default:
		err("unknown pixelformat\n");
		break;
	}
}

struct fimc_is_core * fimc_is_video_ctx_2_core(struct fimc_is_video_ctx *video_ctx)
{
	return (struct fimc_is_core *)video_ctx->video_common->core;
}

int fimc_is_video_probe(struct fimc_is_video_common *video,
	void *core_data,
	char *video_name,
	u32 video_number,
	const struct v4l2_file_operations *fops,
	const struct v4l2_ioctl_ops *ioctl_ops)
{
	int ret = 0;
	struct fimc_is_core *core = core_data;

	snprintf(video->vd.name, sizeof(video->vd.name),
		"%s", video_name);

	video->core			= core;
	video->vb2			= core->mem.vb2;
	video->vd.fops			= fops;
	video->vd.ioctl_ops		= ioctl_ops;
	video->vd.v4l2_dev		= &core->mdev->v4l2_dev;
	video->vd.minor			= -1;
	video->vd.release		= video_device_release;
	video->vd.lock			= NULL;
	video_set_drvdata(&video->vd, core);

	ret = video_register_device(&video->vd,
				VFL_TYPE_GRABBER,
				(EXYNOS_VIDEONODE_FIMC_IS + video_number));
	if (ret) {
		err("Failed to register video device");
		goto p_err;
	}

	video->pads.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_init(&video->vd.entity, 1, &video->pads, 0);
	if (ret) {
		err("Failed to media_entity_init ScalerP video device\n");
		goto p_err;
	}

p_err:
	return ret;
}

int fimc_is_video_open(struct fimc_is_video_ctx *video_ctx,
	void *device,
	 u32 buffers_ready,
	struct fimc_is_video_common *video_common,
	u32 vbq_type,
	const struct vb2_ops *vb2_ops,
	const struct vb2_mem_ops *mem_ops)
{
	int ret = 0;
	struct vb2_queue *vbq;

	video_ctx->buffers = 0;
	video_ctx->buffers_ready = buffers_ready;
	video_ctx->buf_ref_cnt = 0;
	video_ctx->device = device;
	video_ctx->video_common = video_common;
	mutex_init(&video_ctx->lock);

	vbq				= &video_ctx->vbq;
	memset(vbq, 0, sizeof(struct vb2_queue));
	vbq->type			= vbq_type;
	vbq->io_modes			= VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	vbq->drv_priv			= video_ctx;
	vbq->ops			= vb2_ops;
	vbq->mem_ops			= mem_ops;

	vb2_queue_init(vbq);

	clear_bit(FIMC_IS_VIDEO_BUFFER_PREPARED, &video_ctx->state);
	clear_bit(FIMC_IS_VIDEO_BUFFER_READY, &video_ctx->state);
	clear_bit(FIMC_IS_VIDEO_STREAM_ON, &video_ctx->state);

	memset(&video_ctx->frame, 0, sizeof(struct fimc_is_frame));

	return ret;
}

int fimc_is_video_close(struct fimc_is_video_ctx *video_ctx,
	struct fimc_is_framemgr *framemgr)
{
	int ret = 0;

	video_ctx->buffers = 0;
	video_ctx->buf_ref_cnt = 0;
	video_ctx->device = 0;

	clear_bit(FIMC_IS_VIDEO_BUFFER_PREPARED, &video_ctx->state);
	clear_bit(FIMC_IS_VIDEO_BUFFER_READY, &video_ctx->state);
	clear_bit(FIMC_IS_VIDEO_STREAM_ON, &video_ctx->state);
	fimc_is_frame_close(framemgr);

	vb2_queue_release(&video_ctx->vbq);

	return ret;
}

int fimc_is_video_reqbufs(struct fimc_is_video_ctx *video_ctx,
	struct fimc_is_framemgr *framemgr,
	struct v4l2_requestbuffers *request)
{
	int ret = 0;

	if (test_bit(FIMC_IS_VIDEO_STREAM_ON, &video_ctx->state)) {
		err("video is stream on, not applied");
		ret = -EINVAL;
		goto exit;
	}

	ret = vb2_reqbufs(&video_ctx->vbq, request);
	if (ret) {
		err("(type, mem) : vbq(%d,%d) != req(%d,%d)",
			video_ctx->vbq.type, video_ctx->vbq.memory,
			request->type, request->memory);
		goto exit;
	}

	video_ctx->buffers = request->count;

	if (video_ctx->buffers == 0) {
		video_ctx->buf_ref_cnt = 0;
		fimc_is_frame_close(framemgr);
	} else {
		if (video_ctx->buffers < video_ctx->buffers_ready) {
			err("buffer count is not invalid(%d < %d)",
				video_ctx->buffers, video_ctx->buffers_ready);
			ret = -EINVAL;
			goto exit;
		}
		fimc_is_frame_open(framemgr, video_ctx->buffers);
	}

exit:
	return ret;
}

int fimc_is_video_set_format_mplane(struct fimc_is_video_ctx *video_ctx,
	struct v4l2_format *format)
{
	int ret = 0;
	u32 plane;
	struct v4l2_pix_format_mplane *pix;
	struct fimc_is_fmt *frame;

	pix = &format->fmt.pix_mp;
	frame = fimc_is_find_format(&pix->pixelformat, NULL, 0);
	if (!frame) {
		err("pixel format is not found\n");
		ret = -EINVAL;
		goto p_err;
	}

	video_ctx->frame.format.pixelformat	= frame->pixelformat;
	video_ctx->frame.format.mbus_code	= frame->mbus_code;
	video_ctx->frame.format.num_planes	= frame->num_planes;
	video_ctx->frame.width			= pix->width;
	video_ctx->frame.height			= pix->height;

	for (plane = 0; plane < frame->num_planes; ++plane) {
		if (pix->plane_fmt[plane].bytesperline)
			video_ctx->frame.width_stride[plane] =
				pix->plane_fmt[plane].bytesperline - pix->width;
		else
			video_ctx->frame.width_stride[plane] = 0;
	}

p_err:
	return ret;
}

int fimc_is_video_qbuf(struct fimc_is_video_ctx *video_ctx,
	struct v4l2_buffer *buf)
{
	int ret = 0;

	buf->flags &= ~V4L2_BUF_FLAG_USE_SYNC;

	ret = vb2_qbuf(&video_ctx->vbq, buf);
	if (ret)
		err("vb2_qbuf is failed(%d)", ret);

	return ret;
}

int fimc_is_video_dqbuf(struct fimc_is_video_ctx *video_ctx,
	struct v4l2_buffer *buf, bool blocking)
{
	int ret = 0;

	ret = vb2_dqbuf(&video_ctx->vbq, buf, blocking);
	if (ret)
		err("vb2_dqbuf is failed(%d)", ret);

	video_ctx->buf_mask &= ~(1<<buf->index);

	return ret;
}

int fimc_is_video_streamon(struct fimc_is_video_ctx *video_ctx,
	enum v4l2_buf_type type)
{
	int ret = 0;

	ret = vb2_streamon(&video_ctx->vbq, type);
	if (ret)
		err("vb2_streamon is failed(%d)", ret);

	return ret;
}

int fimc_is_video_streamoff(struct fimc_is_video_ctx *video_ctx,
	enum v4l2_buf_type type)
{
	int ret = 0;

	ret = vb2_streamoff(&video_ctx->vbq, type);
	if (ret)
		err("vb2_streamoff is failed(%d)", ret);

	return ret;
}

int fimc_is_video_queue_setup(struct fimc_is_video_ctx *video_ctx,
	unsigned int *num_planes,
	unsigned int sizes[],
	void *allocators[])
{
	u32 ret = 0, i;
	struct fimc_is_core *core = video_ctx->video_common->core;

	*num_planes = (unsigned int)(video_ctx->frame.format.num_planes);
	fimc_is_set_plane_size(&video_ctx->frame, sizes);

	for (i = 0; i < *num_planes; i++) {
		allocators[i] =  core->mem.alloc_ctx;
		video_ctx->frame.size[i] = sizes[i];
	}

	return ret;
}

int fimc_is_video_buffer_queue(struct fimc_is_video_ctx *video_ctx,
	struct vb2_buffer *vb, struct fimc_is_framemgr *framemgr)
{
	u32 ret = 0, i;
	u32 index;
	u32 ext_size;
	u32 spare;
	struct fimc_is_core *core = video_ctx->video_common->core;
	struct fimc_is_frame_shot *frame;

	index = vb->v4l2_buf.index;

	if (!framemgr) {
		err("framemgr is null");
		ret = -EINVAL;
		goto exit;
	}

	/* plane address is updated for checking everytime */
	if (video_ctx->frame.format.pixelformat == V4L2_PIX_FMT_YVU420M) {
		video_ctx->buf_dva[index][0] = core->mem.vb2->plane_addr(vb, 0);

		video_ctx->buf_dva[index][1] = core->mem.vb2->plane_addr(vb, 2);

		video_ctx->buf_dva[index][2] = core->mem.vb2->plane_addr(vb, 1);

#ifdef USE_FRAME_SYNC
		video_ctx->buf_dva[index][3] = core->mem.vb2->plane_addr(vb, 3);
#endif
	} else {
		for (i = 0; i < vb->num_planes; i++) {
			video_ctx->buf_dva[index][i]
				= core->mem.vb2->plane_addr(vb, i);
		}
	}

	frame = &framemgr->frame[index];

	/* uninitialized frame need to get info */
	if (frame->init == FRAME_UNI_MEM)
		goto set_info;

	/* plane count check */
	if (frame->planes != vb->num_planes) {
		err("plane count is changed(%08X != %08X)",
			frame->planes, vb->num_planes);
		ret = -EINVAL;
		goto exit;
	}

	/* plane address check */
	for (i = 0; i < frame->planes; i++) {
		if (frame->dvaddr_buffer[i] !=
			video_ctx->buf_dva[index][i]) {
			err("buffer %d plane %d is changed(%08X != %08X)",
				index, i,
				frame->dvaddr_buffer[i],
				video_ctx->buf_dva[index][i]);
			ret = -EINVAL;
			goto exit;
		}
	}

	goto exit;

set_info:
	if (test_bit(FIMC_IS_VIDEO_BUFFER_PREPARED, &video_ctx->state)) {
		err("already prepared but new index(%d) is came", index);
		ret = -EINVAL;
		goto exit;
	}

	frame->vb = vb;
	frame->planes = vb->num_planes;
	spare = frame->planes - 1;

	for (i = 0; i < frame->planes; i++) {
		frame->dvaddr_buffer[i] = video_ctx->buf_dva[index][i];
#ifdef PRINT_BUFADDR
		printk(KERN_INFO "0x%04X buf[%d][%d] : 0x%08X\n", framemgr->id,
			index, i, frame->dvaddr_buffer[i]);
#endif
	}

	if (framemgr->id & (FRAMEMGR_ID_SENSOR | FRAMEMGR_ID_ISP_GRP |
		FRAMEMGR_ID_DIS_GRP)) {
		ext_size = sizeof(struct camera2_shot_ext) -
			sizeof(struct camera2_shot);

		/* Create Kvaddr for Metadata */
		video_ctx->buf_kva[index][spare]
			= core->mem.vb2->plane_kvaddr(vb, spare);

		frame->dvaddr_shot = video_ctx->buf_dva[index][spare] + ext_size;
		frame->kvaddr_shot = video_ctx->buf_kva[index][spare] + ext_size;
		frame->cookie_shot = (u32)vb2_plane_cookie(vb, spare);
		frame->shot = (struct camera2_shot *)frame->kvaddr_shot;
		frame->shot_ext = (struct camera2_shot_ext *)
			(video_ctx->buf_kva[index][spare]);
		frame->shot_size = video_ctx->frame.size[spare];
#ifdef MEASURE_TIME
		frame->tzone = (struct timeval *)frame->shot_ext->timeZone;
#endif
	} else {
#ifdef USE_FRAME_SYNC
		/* Create Kvaddr for frame sync */
		video_ctx->buf_kva[index][spare]
			= core->mem.vb2->plane_kvaddr(vb, spare);

		frame->stream = (struct camera2_stream *)
			video_ctx->buf_kva[index][spare];
		frame->stream->address = video_ctx->buf_kva[index][spare];
		frame->stream_size = video_ctx->frame.size[spare];
#else
		frame->stream = NULL;
		frame->stream_size = 0;
#endif
	}

	frame->init = FRAME_INI_MEM;

	video_ctx->buf_ref_cnt++;

	if (video_ctx->buffers_ready == video_ctx->buf_ref_cnt)
		set_bit(FIMC_IS_VIDEO_BUFFER_READY, &video_ctx->state);

	if (video_ctx->buffers == video_ctx->buf_ref_cnt)
		set_bit(FIMC_IS_VIDEO_BUFFER_PREPARED, &video_ctx->state);

exit:
	return ret;
}

int buffer_done(struct fimc_is_video_ctx *video_ctx, u32 index)
{
	int ret = 0;

	if (!video_ctx) {
		err("video_ctx is NULL");
		ret = -EINVAL;
		goto exit;
	}

	if (index == FIMC_IS_INVALID_BUF_INDEX) {
		err("buffer done had invalid index(%d)", index);
		ret = -EINVAL;
		goto exit;
	}

	if (!test_bit(FIMC_IS_VIDEO_STREAM_ON, &video_ctx->state)) {
		warn("video state is not stream on");
		ret = -EINVAL;
		goto exit;
	}

	vb2_buffer_done(video_ctx->vbq.bufs[index], VB2_BUF_STATE_DONE);

exit:
	return ret;
}

long video_ioctl3(struct file *file,
	       unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct fimc_is_video_ctx *video_ctx = file->private_data;

	if (mutex_lock_interruptible(&video_ctx->lock)) {
		err("mutex_lock_interruptible is fail");
		ret = -ERESTARTSYS;
		goto exit;
	}

	ret = video_ioctl2(file, cmd, arg);

	mutex_unlock(&video_ctx->lock);
exit:
	return ret;
}

EXPORT_SYMBOL(video_ioctl3);
