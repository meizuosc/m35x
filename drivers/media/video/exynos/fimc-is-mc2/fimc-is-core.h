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

#ifndef FIMC_IS_CORE_H
#define FIMC_IS_CORE_H

#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mediabus.h>
#include <media/exynos_fimc_is.h>
#include <media/v4l2-ioctl.h>
#include <media/exynos_mc.h>
#include <media/videobuf2-core.h>
#if defined(CONFIG_VIDEOBUF2_CMA_PHYS)
#include <media/videobuf2-cma-phys.h>
#elif defined(CONFIG_VIDEOBUF2_ION)
#include <media/videobuf2-ion.h>
#endif
#include "fimc-is-param.h"

#include "fimc-is-device-sensor.h"
#include "fimc-is-interface.h"
#include "fimc-is-framemgr.h"
#include "fimc-is-device-ischain.h"

#include "fimc-is-video-sensor.h"
#include "fimc-is-video-isp.h"
#include "fimc-is-video-scc.h"
#include "fimc-is-video-scp.h"
#include "fimc-is-video-vdisc.h"
#include "fimc-is-video-vdiso.h"
#include "fimc-is-mem.h"

#define FIMC_IS_MODULE_NAME			"exynos5-fimc-is"
#define FIMC_IS_SENSOR_ENTITY_NAME		"exynos5-fimc-is-sensor"
#define FIMC_IS_FRONT_ENTITY_NAME		"exynos5-fimc-is-front"
#define FIMC_IS_BACK_ENTITY_NAME		"exynos5-fimc-is-back"
#define FIMC_IS_VIDEO_BAYER_NAME		"exynos5-fimc-is-bayer"
#define FIMC_IS_VIDEO_ISP_NAME			"exynos5-fimc-is-isp"
#define FIMC_IS_VIDEO_SCALERC_NAME		"exynos5-fimc-is-scalerc"
#define FIMC_IS_VIDEO_3DNR_NAME			"exynos5-fimc-is-3dnr"
#define FIMC_IS_VIDEO_SCALERP_NAME		"exynos5-fimc-is-scalerp"
#define FIMC_IS_VIDEO_VDISC_NAME		"exynos5-fimc-is-vdisc"
#define FIMC_IS_VIDEO_VDISO_NAME		"exynos5-fimc-is-vdiso"

#define FIMC_IS_MAX_VNODES			(3)

#define FIMC_IS_COMMAND_TIMEOUT			(3*HZ)
#define FIMC_IS_STARTUP_TIMEOUT			(3*HZ)
#define FIMC_IS_SHUTDOWN_TIMEOUT		(10*HZ)
#define FIMC_IS_FLITE_STOP_TIMEOUT		(3*HZ)

#define FIMC_IS_SENSOR_MAX_ENTITIES		(1)
#define FIMC_IS_SENSOR_PAD_SOURCE_FRONT		(0)
#define FIMC_IS_SENSOR_PADS_NUM			(1)

#define FIMC_IS_FRONT_MAX_ENTITIES		(1)
#define FIMC_IS_FRONT_PAD_SINK			(0)
#define FIMC_IS_FRONT_PAD_SOURCE_BACK		(1)
#define FIMC_IS_FRONT_PAD_SOURCE_BAYER		(2)
#define FIMC_IS_FRONT_PAD_SOURCE_SCALERC	(3)
#define FIMC_IS_FRONT_PADS_NUM			(4)

#define FIMC_IS_BACK_MAX_ENTITIES		(1)
#define FIMC_IS_BACK_PAD_SINK			(0)
#define FIMC_IS_BACK_PAD_SOURCE_3DNR		(1)
#define FIMC_IS_BACK_PAD_SOURCE_SCALERP		(2)
#define FIMC_IS_BACK_PADS_NUM			(3)

#define FIMC_IS_MAX_SENSOR_NAME_LEN		(16)

#define FW_SHARED_OFFSET			(0xDC0000)
#define DEBUG_CNT				(500*1024)
#define DEBUG_OFFSET				(0xD40000)
#define DEBUGCTL_OFFSET				(0xDBD000)
#define DEBUG_FCOUNT				(0xDC64C0)
#define DEBUG_INSTANCE				0xF

/* configuration - default post processing */
/*#define ENABLE_DRC*/
/*#define ENABLE_ODC*/
#define ENABLE_VDIS
/*#define ENABLE_SWVDIS*/
#define ENABLE_TDNR
#define ENABLE_FD

#if (defined(ENABLE_VDIS) && defined(ENABLE_SWVDIS))
#error SW VDIS and HW VDIS should not be set together
#endif

/*
 * -----------------------------------------------------------------------------
 * Debug Message Configuration
 * -----------------------------------------------------------------------------
 */

/*#define DEBUG*/
/*#define DBG_STREAMING*/
/*#define DBG_FLITEISR*/
/*#define AUTO_MODE*/
#define FW_DEBUG
#define RESERVED_MEM
#define USE_FRAME_SYNC
#define USE_ADVANCED_DZOOM
/*#define TASKLET_MSG*/
/*#define PRINT_CAPABILITY*/
/*#define PRINT_BUFADDR*/
/*#define PRINT_DZOOM*/
#define CHECK_FDROP
#define ISDRV_VERSION 112

#define USE_TF4_SENSOR //## mm78.kim
#if defined(CONFIG_SOC_EXYNOS5410)
#define ENABLE_3AA
#endif

#ifdef err
#undef err
#endif
#define err(fmt, args...) \
	printk(KERN_ERR "[ERR]%s:%d: " fmt "\n", __func__, __LINE__, ##args)

#define merr(fmt, this, args...) \
	printk(KERN_ERR "[ERR:%d]%s:%d: " fmt "\n", \
		this->instance, __func__, __LINE__, ##args)

#ifdef warn
#undef warn
#endif
#define warn(fmt, args...) \
	printk(KERN_WARNING "[WRN] " fmt "\n", ##args)

#define mwarn(fmt, this, args...) \
	printk(KERN_WARNING "[WRN:%d] " fmt "\n", this->instance, ##args)

#define mdbg_common(prefix, fmt, instance, args...)			\
	do {								\
		if (instance & DEBUG_INSTANCE)				\
			printk(KERN_INFO prefix fmt, instance, ##args);	\
	} while (0)

#ifdef DEBUG
#define dbg(fmt, args...) \
	/*printk(KERN_DEBUG "%s:%d: " fmt "\n", __func__, __LINE__, ##args)*/

#define dbg_warning(fmt, args...) \
	printk(KERN_INFO "%s[WAR] Warning! " fmt, __func__, ##args)

/* debug message for video node */
#define dbg_sensor(fmt, args...) \
	printk(KERN_INFO "[SEN] " fmt, ##args)

#define dbg_isp(fmt, args...) \
	printk(KERN_INFO "[ISP] " fmt, ##args)

#define mdbgv_isp(fmt, this, args...)		\
	mdbg_common("[ISP:V:%d] ", fmt, this->instance, ##args)

#define dbg_scp(fmt, args...) \
	printk(KERN_INFO "[SCP] " fmt, ##args)

#define dbg_scc(fmt, args...) \
	printk(KERN_INFO "[SCC] " fmt, ##args)

#define dbg_vdisc(fmt, args...) \
	printk(KERN_INFO "[VDC] " fmt, ##args)

#define dbg_vdiso(fmt, args...) \
	printk(KERN_INFO "[VDO] " fmt, ##args)

/* debug message for device */
#define mdbgd_sensor(fmt, this, args...)	\
	mdbg_common("[SEN:D:%d] ", fmt, this->instance, ##args)

#define dbg_front(fmt, args...) \
	printk(KERN_INFO "[FRT] " fmt, ##args)

#define dbg_back(fmt, args...) \
	printk(KERN_INFO "[BAK] " fmt, ##args)

#define dbg_ischain(fmt, args...) \
	printk(KERN_INFO "[ISC] " fmt, ##args)

#define mdbgd_ischain(fmt, this, args...) \
	printk(KERN_INFO "[ISC:%d] " fmt, this->instance, ##args)

#define dbg_core(fmt, args...) \
	printk(KERN_INFO "[COR] " fmt, ##args)

#ifdef DBG_STREAMING
#define dbg_interface(fmt, args...) \
	printk(KERN_INFO "[ITF] " fmt, ##args)
#define dbg_frame(fmt, args...) \
	printk(KERN_INFO "[FRM] " fmt, ##args)
#else
#define dbg_interface(fmt, args...)
#define dbg_frame(fmt, args...)
#endif

#else
#define dbg(fmt, args...)
#define dbg_warning(fmt, args...)
/* debug message for video node */
#define dbg_sensor(fmt, args...)
#define dbg_isp(fmt, args...)
#define mdbgv_isp(fmt, this, args...)
#define dbg_scp(fmt, args...)
#define dbg_scc(fmt, args...)
#define dbg_vdisc(fmt, args...)
#define dbg_vdiso(fmt, args...)
/* debug message for device */
#define mdbgd_sensor(fmt, this, args...)
#define dbg_front(fmt, args...)
#define dbg_back(fmt, args...)
#define dbg_ischain(fmt, args...)
#define mdbgd_ischain(fmt, this, args...)
#define dbg_core(fmt, args...)
#define dbg_interface(fmt, args...)
#define dbg_frame(fmt, args...)
#endif

#define FIMC_IS_A5_MEM_SIZE		(0x00E00000)
#define FIMC_IS_A5_SEN_SIZE		(0x00100000)
#define FIMC_IS_REGION_SIZE		(0x5000)
#define FIMC_IS_SETFILE_SIZE		(0xc0d8)
#define FIMC_IS_FW_BASE_MASK		((1 << 26) - 1)
#define FIMC_IS_TDNR_MEM_SIZE		(1920*1080*4)
#define FIMC_IS_DEBUG_REGION_ADDR	(0x00D40000)
#define FIMC_IS_SHARED_REGION_ADDR	(0x00DC0000)

#define MAX_ISP_INTERNAL_BUF_WIDTH	(4128)  /* 4808 in HW */
#define MAX_ISP_INTERNAL_BUF_HEIGHT	(3096)  /* 3356 in HW */
#define SIZE_ISP_INTERNAL_BUF \
	(MAX_ISP_INTERNAL_BUF_WIDTH * MAX_ISP_INTERNAL_BUF_HEIGHT * 2)

#define MAX_3AA_INTERNAL_BUF_WIDTH	(2400)  /* 4808 in HW */
#define MAX_3AA_INTERNAL_BUF_HEIGHT	(1360)  /* 3356 in HW */
#define SIZE_3AA_INTERNAL_BUF \
	(MAX_3AA_INTERNAL_BUF_WIDTH * MAX_3AA_INTERNAL_BUF_HEIGHT * 2)

#define MAX_ODC_INTERNAL_BUF_WIDTH	(2560)  /* 4808 in HW */
#define MAX_ODC_INTERNAL_BUF_HEIGHT	(1920)  /* 3356 in HW */
#define SIZE_ODC_INTERNAL_BUF \
	(MAX_ODC_INTERNAL_BUF_WIDTH * MAX_ODC_INTERNAL_BUF_HEIGHT * 3)

#define MAX_DIS_INTERNAL_BUF_WIDTH	(2400)
#define MAX_DIS_INTERNAL_BUF_HEIGHT	(1360)
#define SIZE_DIS_INTERNAL_BUF \
	(MAX_DIS_INTERNAL_BUF_WIDTH * MAX_DIS_INTERNAL_BUF_HEIGHT * 2)

#define MAX_3DNR_INTERNAL_BUF_WIDTH	(1920)
#define MAX_3DNR_INTERNAL_BUF_HEIGHT	(1088)
#define SIZE_DNR_INTERNAL_BUF \
	(MAX_3DNR_INTERNAL_BUF_WIDTH * MAX_3DNR_INTERNAL_BUF_HEIGHT * 2)

#define NUM_ISP_INTERNAL_BUF		(4)
#define NUM_3AA_INTERNAL_BUF		(8)
#define NUM_ODC_INTERNAL_BUF		(2)
#define NUM_DIS_INTERNAL_BUF		(5)
#define NUM_DNR_INTERNAL_BUF		(2)

struct fimc_is_minfo {
	dma_addr_t	base;		/* buffer base */
	size_t		size;		/* total length */
	dma_addr_t	vaddr_base;	/* buffer base */
	dma_addr_t	vaddr_curr;	/* current addr */
	void		*bitproc_buf;
	void		*fw_cookie;

	u32		dvaddr;
	u32		kvaddr;
	u32		dvaddr_debug;
	u32		kvaddr_debug;
	u32		dvaddr_fshared;
	u32		kvaddr_fshared;
	u32		dvaddr_region;
	u32		kvaddr_region;
	u32		dvaddr_shared; /*shared region of is region*/
	u32		kvaddr_shared;
	u32		dvaddr_odc;
	u32		kvaddr_odc;
	u32		dvaddr_dis;
	u32		kvaddr_dis;
	u32		dvaddr_3dnr;
	u32		kvaddr_3dnr;
	u32		dvaddr_isp;
	u32		kvaddr_isp;
};

enum fimc_is_debug_device {
	FIMC_IS_DEBUG_MAIN = 0,
	FIMC_IS_DEBUG_EC,
	FIMC_IS_DEBUG_SENSOR,
	FIMC_IS_DEBUG_ISP,
	FIMC_IS_DEBUG_DRC,
	FIMC_IS_DEBUG_FD,
	FIMC_IS_DEBUG_SDK,
	FIMC_IS_DEBUG_SCALERC,
	FIMC_IS_DEBUG_ODC,
	FIMC_IS_DEBUG_DIS,
	FIMC_IS_DEBUG_TDNR,
	FIMC_IS_DEBUG_SCALERP
};

enum fimc_is_debug_target {
	FIMC_IS_DEBUG_UART = 0,
	FIMC_IS_DEBUG_MEMORY,
	FIMC_IS_DEBUG_DCC3
};

enum fimc_is_front_input_entity {
	FIMC_IS_FRONT_INPUT_NONE = 0,
	FIMC_IS_FRONT_INPUT_SENSOR,
};

enum fimc_is_front_output_entity {
	FIMC_IS_FRONT_OUTPUT_NONE = 0,
	FIMC_IS_FRONT_OUTPUT_BACK,
	FIMC_IS_FRONT_OUTPUT_BAYER,
	FIMC_IS_FRONT_OUTPUT_SCALERC,
};

enum fimc_is_back_input_entity {
	FIMC_IS_BACK_INPUT_NONE = 0,
	FIMC_IS_BACK_INPUT_FRONT,
};

enum fimc_is_back_output_entity {
	FIMC_IS_BACK_OUTPUT_NONE = 0,
	FIMC_IS_BACK_OUTPUT_3DNR,
	FIMC_IS_BACK_OUTPUT_SCALERP,
};

enum fimc_is_front_state {
	FIMC_IS_FRONT_ST_POWERED = 0,
	FIMC_IS_FRONT_ST_STREAMING,
	FIMC_IS_FRONT_ST_SUSPENDED,
};

struct fimc_is_core;

struct fimc_is_sensor_dev {
	struct v4l2_subdev		sd;
	struct media_pad		pads;
	struct v4l2_mbus_framefmt	mbus_fmt;
	enum fimc_is_sensor_output_entity	output;
};

struct fimc_is_front_dev {
	struct v4l2_subdev		sd;
	struct media_pad		pads[FIMC_IS_FRONT_PADS_NUM];
	struct v4l2_mbus_framefmt	mbus_fmt[FIMC_IS_FRONT_PADS_NUM];
	enum fimc_is_front_input_entity	input;
	enum fimc_is_front_output_entity	output;
	u32 width;
	u32 height;

};

struct fimc_is_back_dev {
	struct v4l2_subdev		sd;
	struct media_pad		pads[FIMC_IS_BACK_PADS_NUM];
	struct v4l2_mbus_framefmt	mbus_fmt[FIMC_IS_BACK_PADS_NUM];
	enum fimc_is_back_input_entity	input;
	enum fimc_is_back_output_entity	output;
	int	dis_on;
	int	odc_on;
	int	tdnr_on;
	u32 width;
	u32 height;
	u32 dis_width;
	u32 dis_height;
};

struct fimc_is_core {
	struct platform_device			*pdev;
	struct resource				*regs_res;
	void __iomem				*regs;
	int					irq;
	u32					id;
	int					ref_cnt;
	int					ischain_index;
	u32					debug_cnt;

	/* depended on isp */
	struct exynos5_platform_fimc_is		*pdata;
	struct exynos_md			*mdev;

	struct fimc_is_minfo			minfo;
	struct fimc_is_mem			mem;
	struct fimc_is_interface		interface;

	struct fimc_is_device_sensor		*sensor[FIMC_IS_MAX_VNODES];
	struct fimc_is_device_ischain		*ischain[FIMC_IS_MAX_VNODES];

	struct fimc_is_sensor_dev		dev_sensor;
	struct fimc_is_front_dev		front;
	struct fimc_is_back_dev			back;

	/* 0-bayer, 1-scalerC, 2-3DNR, 3-scalerP */
	struct fimc_is_video_sensor		video_sensor;
	struct fimc_is_video_isp		video_isp;
	struct fimc_is_video_scc		video_scc;
	struct fimc_is_video_scp		video_scp;
	struct fimc_is_video_vdisc		video_vdisc;
	struct fimc_is_video_vdiso		video_vdiso;
};

#if defined(CONFIG_VIDEOBUF2_CMA_PHYS)
extern const struct fimc_is_vb2 fimc_is_vb2_cma;
#elif defined(CONFIG_VIDEOBUF2_ION)
extern const struct fimc_is_vb2 fimc_is_vb2_ion;
#endif

struct device *get_is_dev(void);

void fimc_is_mem_suspend(void *alloc_ctxes);
void fimc_is_mem_resume(void *alloc_ctxes);
void fimc_is_mem_cache_clean(const void *start_addr, unsigned long size);
void fimc_is_mem_cache_inv(const void *start_addr, unsigned long size);
int fimc_is_pipeline_s_stream_preview
	(struct media_entity *start_entity, int on);
int fimc_is_init_set(struct fimc_is_core *dev , u32 val);
int fimc_is_load_fw(struct fimc_is_core *dev);
int fimc_is_load_setfile(struct fimc_is_core *dev);
int fimc_is_spi_read(void *buf, u32 rx_addr, size_t size);
int fimc_is_runtime_suspend(struct device *dev);
int fimc_is_runtime_resume(struct device *dev);
#endif /* FIMC_IS_CORE_H_ */
