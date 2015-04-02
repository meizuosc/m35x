/*
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/persistent_ram.h>
#include <linux/gpio.h>
#include <linux/gpio_event.h>
#include <linux/cma.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/notifier.h>
#include <linux/reboot.h>

#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>
#include <asm/mach-types.h>

#include <plat/clock.h>
#include <plat/cpu.h>
#include <plat/devs.h>
#include <plat/regs-serial.h>
#include <plat/regs-watchdog.h>
#include <plat/media.h>
#include <plat/gpio-cfg.h>
#include <plat/iic.h>

#include <mach/exynos_fiq_debugger.h>
#include <mach/map.h>
#include <mach/hs-iic.h>
#include <mach/dev.h>
#include <mach/regs-pmu.h>

#include "../../../drivers/staging/android/ram_console.h"
#include "common.h"
#include "board-tf4.h"
#include "resetreason.h"

#include <linux/i2c-gpio.h>

#ifdef CONFIG_INV_SENSORS
// shengliang: @2012-07-23
#include <linux/mpu.h>
#endif

#ifdef CONFIG_INV_SENSORS
// shengliang: @2012-07-23
// i2c6
static struct i2c_gpio_platform_data gpio_i2c6_platdata = {
	.sda_pin	= EXYNOS5410_GPB1(3),
	.scl_pin	= EXYNOS5410_GPB1(4),
	.udelay	= 2,
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};

static struct platform_device gpio_i2c6_dev = {
	.name	= "i2c-gpio",
	.id		= 6,
	.dev.platform_data = &gpio_i2c6_platdata,
};

// i2c7
static struct i2c_gpio_platform_data gpio_i2c7_platdata = {
	.sda_pin	= EXYNOS5410_GPB2(2),
	.scl_pin	= EXYNOS5410_GPB2(3),
	.udelay	= 2,
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};

static struct platform_device gpio_i2c7_dev = {
	.name	= "i2c-gpio",
	.id		= 7,
	.dev.platform_data = &gpio_i2c7_platdata,
};

static struct platform_device *tf4_sensor_i2c_devices[] __initdata = {
	&gpio_i2c6_dev,
	&gpio_i2c7_dev,
};

static struct mpu_platform_data mpu3050_data = {
	.int_config  = 0x10,
#if 0// shengliang, it's for TC4 screen in HZSSCR
#ifdef CONFIG_TF4_PORTRAIT_MODE
	// shengliang, need to be calibrated, @2012-08-02
	.orientation = {  0,  1,  0,
			  1,  0,  0,
			  0,  0, -1 },
#else
	// shengliang, has been calibrated, @2012-08-02
	.orientation = {  0,  1,  0,
			 -1,  0,  0,
			  0,  0,  1 },
#endif
#endif
	// shengliang, has been calibrated, @2013-1-9
	.orientation = { -1,  0,  0,
			  0,  1,  0,
			  0,  0, -1 },
};

/* accel */
static struct ext_slave_platform_data inv_mpu_bma250_data = {
	.bus         = EXT_SLAVE_BUS_SECONDARY,
#if 0// shengliang, it's for TC4 screen in HZSSCR
#ifdef CONFIG_TF4_PORTRAIT_MODE
	// shengliang, need to be calibrated, @2012-08-02
	.orientation = {  1,  0,  0,
			  0, -1,  0,
			  0,  0, -1 },
#else
	// shengliang, has been calibrated, @2012-08-02
	.orientation = {  0, -1,  0,
			  1,  0,  0,
			  0,  0,  1 },
#endif
#endif
	// shengliang, has been calibrated, @2013-1-9
	.orientation = {  1,  0,  0,
			  0, -1,  0,
			  0,  0, -1 },
};

/* compass */
static struct ext_slave_platform_data inv_mpu_hmc5883_data = {
	.bus         = EXT_SLAVE_BUS_PRIMARY,
#if 0// shengliang, it's for TC4 screen in HZSSCR
#ifdef CONFIG_TF4_PORTRAIT_MODE
	// shengliang, need to be calibrated, @2012-08-02
	.orientation = { -1,  0,  0,
			  0,  1,  0,
			  0,  0,  1 },
#else
	// shengliang, has been calibrated, @2012-08-02
	.orientation = {  1,  0,  0,
			  0,  1,  0,
			  0,  0,  1 },
#endif
#endif
	// shengliang, has been calibrated, @2013-1-9
	.orientation = {  0, -1,  0,
			 -1,  0,  0,
			  0,  0, -1 },
};

static struct i2c_board_info i2c_devs6[] __initdata = {
	// MPU3050 (Gyro)
	{
		I2C_BOARD_INFO(MPU_NAME, 0x68),
		.platform_data 	= &mpu3050_data,
		.irq		= IRQ_EINT(9),
	},
	// BMA250 (Accel)
	{
		I2C_BOARD_INFO("bma250", (0x30>>1)),
		.platform_data = &inv_mpu_bma250_data,
	},
};

static struct i2c_board_info i2c_devs7[] __initdata = {
	// HMC5883 (Mag)
	{
		I2C_BOARD_INFO("hmc5883", (0x3c>>1)),
		.platform_data = &inv_mpu_hmc5883_data,
	},
};

void __init exynos5_tf4_sensor_init(void)
{
	i2c_register_board_info(6, i2c_devs6, ARRAY_SIZE(i2c_devs6));
	i2c_register_board_info(7, i2c_devs7, ARRAY_SIZE(i2c_devs7));
	platform_add_devices(tf4_sensor_i2c_devices, ARRAY_SIZE(tf4_sensor_i2c_devices));
}
#endif

static struct ram_console_platform_data ramconsole_pdata;

static struct platform_device ramconsole_device = {
	.name	= "ram_console",
	.id	= -1,
	.dev	= {
		.platform_data = &ramconsole_pdata,
	},
};

static struct platform_device persistent_trace_device = {
	.name	= "persistent_trace",
	.id	= -1,
};

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define SMDK5410_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define SMDK5410_ULCON_DEFAULT	S3C2410_LCON_CS8

#define SMDK5410_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

static struct s3c2410_uartcfg smdk5410_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= SMDK5410_UCON_DEFAULT,
		.ulcon		= SMDK5410_ULCON_DEFAULT,
		.ufcon		= SMDK5410_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= SMDK5410_UCON_DEFAULT,
		.ulcon		= SMDK5410_ULCON_DEFAULT,
		.ufcon		= SMDK5410_UFCON_DEFAULT,
	},
	[2] = {
#ifndef CONFIG_EXYNOS_FIQ_DEBUGGER
	/*
	 * Don't need to initialize hwport 2, when FIQ debugger is
	 * enabled. Because it will be handled by fiq_debugger.
	 */
		.hwport		= 2,
		.flags		= 0,
		.ucon		= SMDK5410_UCON_DEFAULT,
		.ulcon		= SMDK5410_ULCON_DEFAULT,
		.ufcon		= SMDK5410_UFCON_DEFAULT,
	},
	[3] = {
#endif
		.hwport		= 3,
		.flags		= 0,
		.ucon		= SMDK5410_UCON_DEFAULT,
		.ulcon		= SMDK5410_ULCON_DEFAULT,
		.ufcon		= SMDK5410_UFCON_DEFAULT,
	},
};

static int exynos5_notifier_call(struct notifier_block *this,
		unsigned long code, void *_cmd)
{
	int mode = 0;

	if ((code == SYS_RESTART) && _cmd)
		if (!strcmp((char *)_cmd, "recovery"))
			mode = 0xf;

	__raw_writel(mode, EXYNOS_INFORM4);

	return NOTIFY_DONE;
}

static struct notifier_block exynos5_reboot_notifier = {
	.notifier_call = exynos5_notifier_call,
};

static struct platform_device *smdk5410_devices[] __initdata = {
	&ramconsole_device,
	&persistent_trace_device,
	&s3c_device_wdt,
	&s3c_device_rtc,
	&s3c_device_adc,
#ifdef CONFIG_PVR_SGX
	&exynos5_device_g3d,
#endif
#ifdef CONFIG_S5P_DEV_ACE
	&s5p_device_ace,
#endif
};

#if defined(CONFIG_CMA)
#include "reserve-mem.h"
static void __init exynos_reserve_mem(void)
{
	static struct cma_region regions[] = {
		{
			.name = "ion",
			.size = (
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_ROT
			CONFIG_VIDEO_SAMSUNG_MEMSIZE_ROT +
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMD
			CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMD +
#endif
			128) * SZ_1M,
			{ .alignment = SZ_1M },
		},
#ifdef CONFIG_EXYNOS_CONTENT_PATH_PROTECTION
#ifdef CONFIG_ION_EXYNOS_DRM_MFC_SH
		{
			.name = "drm_mfc_sh",
			.size = SZ_1M,
		},
#endif
#ifdef CONFIG_ION_EXYNOS_DRM_MSGBOX_SH
		{
			.name = "drm_msgbox_sh",
			.size = SZ_1M,
		},
#endif
#endif
		{
			.size = 0
		},
	};
#ifdef CONFIG_EXYNOS_CONTENT_PATH_PROTECTION
	static struct cma_region regions_secure[] = {
#ifdef CONFIG_ION_EXYNOS_DRM_MEMSIZE_FIMD_VIDEO
	       {
			.name = "drm_fimd_video",
			.size = CONFIG_ION_EXYNOS_DRM_MEMSIZE_FIMD_VIDEO *
				SZ_1K,
	       },
#endif
#ifdef CONFIG_ION_EXYNOS_DRM_MEMSIZE_MFC_OUTPUT
	       {
			.name = "drm_mfc_output",
			.size = CONFIG_ION_EXYNOS_DRM_MEMSIZE_MFC_OUTPUT *
				SZ_1K,
	       },
#endif
#ifdef CONFIG_ION_EXYNOS_DRM_MEMSIZE_MFC_INPUT
	       {
			.name = "drm_mfc_input",
			.size = CONFIG_ION_EXYNOS_DRM_MEMSIZE_MFC_INPUT *
				SZ_1K,
	       },
#endif
#ifdef CONFIG_ION_EXYNOS_DRM_MFC_FW
		{
			.name = "drm_mfc_fw",
			.size = SZ_1M,
		},
#endif
#ifdef CONFIG_ION_EXYNOS_DRM_SECTBL
		{
			.name = "drm_sectbl",
			.size = SZ_1M,
		},
#endif
		{
			.size = 0
		},
	};
#else /* !CONFIG_EXYNOS_CONTENT_PATH_PROTECTION */
	struct cma_region *regions_secure = NULL;
#endif /* CONFIG_EXYNOS_CONTENT_PATH_PROTECTION */
	static const char map[] __initconst =
#ifdef CONFIG_EXYNOS_C2C
		"samsung-c2c=c2c_shdmem;"
#endif
#ifdef CONFIG_EXYNOS_CONTENT_PATH_PROTECTION
		"ion-exynos/mfc_sh=drm_mfc_sh;"
		"ion-exynos/msgbox_sh=drm_msgbox_sh;"
		"ion-exynos/fimd_video=drm_fimd_video;"
		"ion-exynos/mfc_output=drm_mfc_output;"
		"ion-exynos/mfc_input=drm_mfc_input;"
		"ion-exynos/mfc_fw=drm_mfc_fw;"
		"ion-exynos/sectbl=drm_sectbl;"
		"s5p-smem/mfc_sh=drm_mfc_sh;"
		"s5p-smem/msgbox_sh=drm_msgbox_sh;"
		"s5p-smem/fimd_video=drm_fimd_video;"
		"s5p-smem/mfc_output=drm_mfc_output;"
		"s5p-smem/mfc_input=drm_mfc_input;"
		"s5p-smem/mfc_fw=drm_mfc_fw;"
		"s5p-smem/sectbl=drm_sectbl;"
#endif
		"ion-exynos=ion;";

	exynos_cma_region_reserve(regions, regions_secure, NULL, map);
}
#else /*!CONFIG_CMA*/
static inline void exynos_reserve_mem(void)
{
}
#endif

static void __init smdk5410_map_io(void)
{
	clk_xusbxti.rate = 24000000;
	clk_xxti.rate = 24000000;

	exynos_init_io(NULL, 0);
	s3c24xx_init_clocks(clk_xusbxti.rate);
	s3c24xx_init_uarts(smdk5410_uartcfgs, ARRAY_SIZE(smdk5410_uartcfgs));
}

#ifdef CONFIG_EXYNOS_HSI
struct exynos_hsi_platdata smdk5410_hsi_pdata = {
        .setup_gpio     = NULL,
};
#endif


static struct persistent_ram_descriptor smdk5410_prd[] __initdata = {
	{
		.name = "ram_console",
		.size = SZ_2M,
	},
#ifdef CONFIG_PERSISTENT_TRACER
	{
		.name = "persistent_trace",
		.size = SZ_1M,
	},
#endif
};


static unsigned int tf4_sleep_gpio_table[][3] = {

	{ EXYNOS5410_GPA0(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPA0(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPA0(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPA0(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPA0(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPA0(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPA0(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPA0(7),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

	{ EXYNOS5410_GPA1(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPA1(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPA1(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPA1(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPA1(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPA1(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

	{ EXYNOS5410_GPA2(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPA2(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPA2(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPA2(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ EXYNOS5410_GPA2(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPA2(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},  
	{ EXYNOS5410_GPA2(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPA2(7),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

	{ EXYNOS5410_GPB0(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPB0(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPB0(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPB0(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},  
	{ EXYNOS5410_GPB0(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	
	{ EXYNOS5410_GPB1(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPB1(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPB1(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPB1(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPB1(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, 

	{ EXYNOS5410_GPB2(0),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPB2(1),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPB2(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPB2(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, 

	{ EXYNOS5410_GPB3(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPB3(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPB3(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ EXYNOS5410_GPB3(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, 
///////////////////////////////////////////////////////////////////////
	{ EXYNOS5410_GPC0(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPC0(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPC0(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPC0(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPC0(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPC0(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPC0(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	


//#ifdef CONFIG_SND_SOC_SAMSUNG_TF4_WM1811
	{ EXYNOS5410_GPC1(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
//#endif
	{ EXYNOS5410_GPC1(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPC1(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPC1(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPC1(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPC1(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPC1(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

	{ EXYNOS5410_GPC2(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPC2(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPC2(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPC2(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPC2(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPC2(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPC2(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

	{ EXYNOS5410_GPC3(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPC3(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPC3(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPC3(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	
	{ EXYNOS5410_GPM5(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPM5(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	
	{ EXYNOS5410_GPD1(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPD1(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPD1(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPD1(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPD1(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPD1(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPD1(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPD1(7),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

	/*GPX0 - GPX3 alive part*/
	{ EXYNOS5410_GPE0(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},  
	{ EXYNOS5410_GPE0(1),  S3C_GPIO_SLP_OUT0, 	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPE0(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPE0(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPE0(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPE0(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	
	{ EXYNOS5410_GPE0(6),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},  
	{ EXYNOS5410_GPE0(7),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	
/////////////////////////////////////////////////////////////////////

	{ EXYNOS5410_GPE1(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPE1(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 

	{ EXYNOS5410_GPF0(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPF0(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPF0(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPF0(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPF0(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPF0(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 


	{ EXYNOS5410_GPF1(0),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPF1(1),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPF1(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPF1(3),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPF1(4),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPF1(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPF1(6),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPF1(7),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE}, 	

	{ EXYNOS5410_GPG0(0),  S3C_GPIO_SLP_OUT1,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPG0(1),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPG0(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPG0(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPG0(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPG0(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	
	{ EXYNOS5410_GPG0(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ EXYNOS5410_GPG0(7),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	

	{ EXYNOS5410_GPG1(0),  S3C_GPIO_SLP_OUT1,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPG1(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPG1(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPG1(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPG1(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPG1(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	
	{ EXYNOS5410_GPG1(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ EXYNOS5410_GPG1(7),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	

	{ EXYNOS5410_GPG2(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPG2(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 	
	
	{ EXYNOS5410_GPH0(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPH0(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPH0(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPH0(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	
	{ EXYNOS5410_GPH1(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPH1(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPH1(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPH1(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPH1(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPH1(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	
	{ EXYNOS5410_GPH1(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPH1(7),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

///////////////////////////////////////////////////////////////////////////
	{ EXYNOS5410_GPM7(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPM7(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPM7(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPM7(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPM7(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPM7(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	
	{ EXYNOS5410_GPM7(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPM7(7),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	
	{ EXYNOS5410_GPY0(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPY0(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPY0(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPY0(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPY0(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPY0(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	

	{ EXYNOS5410_GPY1(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},  
	{ EXYNOS5410_GPY1(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPY1(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPY1(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 

	{ EXYNOS5410_GPY2(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPY2(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPY2(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPY2(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPY2(4),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ EXYNOS5410_GPY2(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	

	{ EXYNOS5410_GPY3(0),  S3C_GPIO_SLP_OUT0, 	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPY3(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPY3(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPY3(3),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPY3(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPY3(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	
	{ EXYNOS5410_GPY3(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPY3(7),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	

	{ EXYNOS5410_GPY4(0),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPY4(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPY4(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPY4(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPY4(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPY4(5),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	
	{ EXYNOS5410_GPY4(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPY4(7),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	
	
////////////////////////////////////////////////////////////////////////
	{ EXYNOS5410_GPY5(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},  
	{ EXYNOS5410_GPY5(1),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPY5(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPY5(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPY5(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPY5(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	
	{ EXYNOS5410_GPY5(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ EXYNOS5410_GPY5(7),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},	

	{ EXYNOS5410_GPY6(0),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPY6(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPY6(2),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPY6(3),  S3C_GPIO_SLP_OUT1,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPY6(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},  
	{ EXYNOS5410_GPY6(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},  
	{ EXYNOS5410_GPY6(6),  S3C_GPIO_SLP_PREV,	S3C_GPIO_PULL_NONE},  
	{ EXYNOS5410_GPY6(7),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},	

	{ EXYNOS5410_GPY7(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPY7(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPY7(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPY7(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPY7(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},  
	{ EXYNOS5410_GPY7(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},  
	{ EXYNOS5410_GPY7(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},  
	{ EXYNOS5410_GPY7(7),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	
//	
	{ EXYNOS5410_GPV0(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPV0(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPV0(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPV0(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPV0(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPV0(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	
	{ EXYNOS5410_GPV0(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPV0(7),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	
//
	{ EXYNOS5410_GPV1(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPV1(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPV1(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPV1(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPV1(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPV1(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	
	{ EXYNOS5410_GPV1(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPV1(7),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

///////////////////////////////////////////////////////////////////////
	{ EXYNOS5410_GPV2(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPV2(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPV2(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPV2(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPV2(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPV2(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	
	{ EXYNOS5410_GPV2(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPV2(7),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

	{ EXYNOS5410_GPV3(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPV3(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPV3(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPV3(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPV3(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPV3(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	
	{ EXYNOS5410_GPV3(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPV3(7),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

	{ EXYNOS5410_GPV4(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPV4(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 

	{ EXYNOS5410_GPJ0(0),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPJ0(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPJ0(2),  S3C_GPIO_SLP_INPUT,  S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPJ0(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPJ0(4),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},

	{ EXYNOS5410_GPJ1(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPJ1(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPJ1(2),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPJ1(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPJ1(4),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},
	{ EXYNOS5410_GPJ1(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	
	{ EXYNOS5410_GPJ1(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPJ1(7),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE},

	{ EXYNOS5410_GPJ2(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPJ2(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPJ2(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPJ2(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPJ2(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPJ2(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	
	{ EXYNOS5410_GPJ2(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPJ2(7),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	
	{ EXYNOS5410_GPJ3(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPJ3(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPJ3(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPJ3(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPJ3(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPJ3(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	
	{ EXYNOS5410_GPJ3(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPJ3(7),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

///////////////////////////////////////////////////////////////////////	
	{ EXYNOS5410_GPJ4(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPJ4(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 	

	{ EXYNOS5410_GPK0(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPK0(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPK0(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPK0(3),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPK0(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ EXYNOS5410_GPK0(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	
	{ EXYNOS5410_GPK0(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPK0(7),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	
	{ EXYNOS5410_GPK1(0),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPK1(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPK1(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPK1(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPK1(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPK1(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	
	{ EXYNOS5410_GPK1(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE},
	{ EXYNOS5410_GPK1(7),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

	{ EXYNOS5410_GPK2(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPK2(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPK2(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPK2(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
	{ EXYNOS5410_GPK2(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPK2(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	
	{ EXYNOS5410_GPK2(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPK2(7),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	
	{ EXYNOS5410_GPK3(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPK3(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPK3(2),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPK3(3),  S3C_GPIO_SLP_OUT0,	S3C_GPIO_PULL_NONE}, 
	{ EXYNOS5410_GPK3(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
	{ EXYNOS5410_GPK3(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	
	{ EXYNOS5410_GPK3(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},

//	{ EXYNOS5410_GPZ(0),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
//	{ EXYNOS5410_GPZ(1),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
//	{ EXYNOS5410_GPZ(2),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
//	{ EXYNOS5410_GPZ(3),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN}, 
//	{ EXYNOS5410_GPZ(4),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
//	{ EXYNOS5410_GPZ(5),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},	
//	{ EXYNOS5410_GPZ(6),  S3C_GPIO_SLP_INPUT,	S3C_GPIO_PULL_DOWN},
};

extern void set_gpio_pd_reg_dirty(void);

static void config_sleep_gpio_table(int array_size, unsigned int (*gpio_table)[3])
{
        u32 i, gpio;

		set_gpio_pd_reg_dirty();

        for (i = 0; i < array_size; i++) {
                gpio = gpio_table[i][0];
                s5p_gpio_set_pd_cfg(gpio, gpio_table[i][1]);
                s5p_gpio_set_pd_pull(gpio, gpio_table[i][2]);
        }
}

static unsigned int tf4_sleep_alive_gpio_table[][4] =
{
		/*Need check later on TF4 EVT board!!, temp Here. */
	{EXYNOS5410_GPX0(0), S3C_GPIO_SFN(0xf),	S3C_GPIO_PULL_NONE},  //PMIC_IRQB
	{EXYNOS5410_GPX0(1), S3C_GPIO_INPUT,   	S3C_GPIO_PULL_DOWN},  //TSP_INT
	{EXYNOS5410_GPX0(2), S3C_GPIO_SFN(0xf), S3C_GPIO_PULL_NONE},  //PMIC_ONOB
	
	{EXYNOS5410_GPX0(3), S3C_GPIO_SFN(0xf),	S3C_GPIO_PULL_UP},    //VOLDOWN_KEY
	{EXYNOS5410_GPX0(4), S3C_GPIO_SFN(0xf),	S3C_GPIO_PULL_UP},    //VOLUP_KEY
	{EXYNOS5410_GPX0(5), S3C_GPIO_INPUT,	  S3C_GPIO_PULL_NONE},  //GS_INT1
	{EXYNOS5410_GPX0(6), S3C_GPIO_INPUT,	  S3C_GPIO_PULL_NONE},  //GS_INT2
	{EXYNOS5410_GPX0(7), S3C_GPIO_INPUT,	  S3C_GPIO_PULL_DOWN},  //DP_HPD

	{EXYNOS5410_GPX1(0), S3C_GPIO_INPUT,	  S3C_GPIO_PULL_NONE},  //3AXIS_RDY
	{EXYNOS5410_GPX1(1), S3C_GPIO_INPUT,	  S3C_GPIO_PULL_NONE},  //GYRO_INT
	{EXYNOS5410_GPX1(2), S3C_GPIO_SFN(0xf), S3C_GPIO_PULL_NONE},  //CHG_IRQ
	{EXYNOS5410_GPX1(3), S3C_GPIO_INPUT,	  S3C_GPIO_PULL_NONE},  //CODEC_INT
	{EXYNOS5410_GPX1(4), S3C_GPIO_INPUT,	  S3C_GPIO_PULL_DOWN},  //WIFI_INT
	{EXYNOS5410_GPX1(5), S3C_GPIO_INPUT,	  S3C_GPIO_PULL_DOWN},  //BGF_INT
	{EXYNOS5410_GPX1(6), S3C_GPIO_INPUT,	  S3C_GPIO_PULL_NONE},  //TF_nCD
	{EXYNOS5410_GPX1(7), S3C_GPIO_INPUT,	  S3C_GPIO_PULL_NONE},  //LIGHT_INT

	{EXYNOS5410_GPX2(0), S3C_GPIO_INPUT,	  S3C_GPIO_PULL_DOWN},  //MODEM_PWRSTATUS
	{EXYNOS5410_GPX2(1), S3C_GPIO_INPUT,	  S3C_GPIO_PULL_DOWN},  //MODEM_EINT3
	{EXYNOS5410_GPX2(2), S3C_GPIO_INPUT,	  S3C_GPIO_PULL_DOWN},  //MT11_INT
	{EXYNOS5410_GPX2(3), S3C_GPIO_SFN(0xf),	S3C_GPIO_PULL_DOWN}, // USB 3.0 ch0 vbus detect hunsoo.lee
	{EXYNOS5410_GPX2(4), S3C_GPIO_INPUT,	  S3C_GPIO_PULL_UP},    //USE3_IDDET
	{EXYNOS5410_GPX2(5), S3C_GPIO_INPUT,	  S3C_GPIO_PULL_DOWN},  //USB2_VBUSDET
	{EXYNOS5410_GPX2(6), S3C_GPIO_INPUT,	  S3C_GPIO_PULL_UP},    //USB2_IDDET
	{EXYNOS5410_GPX2(7), S3C_GPIO_INPUT,	  S3C_GPIO_PULL_NONE},  //TP

	{EXYNOS5410_GPX3(0), S3C_GPIO_INPUT,	  S3C_GPIO_PULL_DOWN},  //MODEM_EINT1
	{EXYNOS5410_GPX3(1), S3C_GPIO_INPUT,	  S3C_GPIO_PULL_NONE},  //NFC_INT
	{EXYNOS5410_GPX3(2), S3C_GPIO_INPUT,	  S3C_GPIO_PULL_DOWN},  //HSIC_AP_WAKEUP
	{EXYNOS5410_GPX3(3), S3C_GPIO_INPUT,	  S3C_GPIO_PULL_DOWN},  //MODEM_EINT2
	{EXYNOS5410_GPX3(4), S3C_GPIO_INPUT,	  S3C_GPIO_PULL_DOWN},  //TP
	{EXYNOS5410_GPX3(5), S3C_GPIO_INPUT,    S3C_GPIO_PULL_DOWN},  //HSIC_SUSPEND_REQ
	{EXYNOS5410_GPX3(6), S3C_GPIO_INPUT,	  S3C_GPIO_PULL_NONE},  //HDMI_CEC
	{EXYNOS5410_GPX3(7), S3C_GPIO_INPUT,	  S3C_GPIO_PULL_NONE},  //HDMI_HPD
	
};

extern void (*s3c_config_sleep_gpio_table)(void);

static void tf4_config_sleep_gpio_table(void)
{
	/*TO DO. unfinished*/
	int i=0, gpio;

	// MMC GPIO Control
	s3c_gpio_cfgpin(EXYNOS5410_GPC0(1), S3C_GPIO_INPUT);
	s3c_gpio_setpull(EXYNOS5410_GPC0(1), S3C_GPIO_PULL_DOWN);

	s3c_gpio_cfgpin(EXYNOS5410_GPC1(1), S3C_GPIO_INPUT);
	s3c_gpio_setpull(EXYNOS5410_GPC1(1), S3C_GPIO_PULL_DOWN);

	printk("-config alive gpio \n ");
   
	for (i = 0; i < ARRAY_SIZE(tf4_sleep_alive_gpio_table); i++)
  {
		  gpio = tf4_sleep_alive_gpio_table[i][0];
		  s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(tf4_sleep_alive_gpio_table[i][1]));
		  s3c_gpio_setpull(gpio, tf4_sleep_alive_gpio_table[i][2]);
 }	

	printk("-tf4_config_sleep_gpio_table\n ");	
	config_sleep_gpio_table(ARRAY_SIZE(tf4_sleep_gpio_table),
			tf4_sleep_gpio_table); 


}

static struct persistent_ram smdk5410_pr __initdata = {
	.descs		= smdk5410_prd,
	.num_descs	= ARRAY_SIZE(smdk5410_prd),
	.start		= PLAT_PHYS_OFFSET + SZ_512M,
#ifdef CONFIG_PERSISTENT_TRACER
	.size		= 3 * SZ_1M,
#else
	.size		= SZ_2M,
#endif
};

static void __init smdk5410_init_early(void)
{
	persistent_ram_early_init(&smdk5410_pr);
}
static void tf4_power_off(void)
{
	unsigned int reg;
	printk("power off the deivce....\n");
	
#ifdef CONFIG_CHARGER_SMB349
	extern int charger_ac_online(void);
	
	if (charger_ac_online()) {
		reg = readl(S3C2410_WTCON);
		reg &= ~(0x1<<5);
		writel(reg, S3C2410_WTCON);

		reg = readl(EXYNOS_AUTOMATIC_WDT_RESET_DISABLE);
		reg &= ~(0x1<<0);
		writel(reg, EXYNOS_AUTOMATIC_WDT_RESET_DISABLE);

		reg = readl(EXYNOS_MASK_WDT_RESET_REQUEST);
		reg &= ~(0x1<<0);
		writel(reg, EXYNOS_MASK_WDT_RESET_REQUEST);

		reg = readl(S3C2410_WTCNT);
		reg = (0x1);
		writel(reg, S3C2410_WTCNT);

		reg = readl(S3C2410_WTCON);
		reg |= (0x1<<0 | 0x1<<5 | 0x1<<15);
		writel(reg, S3C2410_WTCON);
	}
	else {
		reg = readl(EXYNOS_PS_HOLD_CONTROL);
		reg &= ~(0x1<<8);
		writel(reg, EXYNOS_PS_HOLD_CONTROL);
	}
#else
	reg = readl(EXYNOS_PS_HOLD_CONTROL);
	reg &= ~(0x1<<8);
	writel(reg, EXYNOS_PS_HOLD_CONTROL);
#endif
}

static void tf4_power_off_prepare(void)
{
	printk("power off prepare the deivce....\n");
}



static enum board_version_type board_version = BOARD_EVT;

enum board_version_type get_board_version(void)
{
	return board_version;
}

void check_board_version(void)
{
	unsigned int GPK1_5 = 0;
	unsigned int GPK1_6 = 0;
	enum board_version_type version = BOARD_EVT;
	
	s3c_gpio_cfgpin(EXYNOS5410_GPK1(5), S3C_GPIO_SLP_INPUT);  // input
	GPK1_5 = gpio_get_value(EXYNOS5410_GPK1(5));

	s3c_gpio_cfgpin(EXYNOS5410_GPK1(6), S3C_GPIO_SLP_INPUT);  // input
	GPK1_6 = gpio_get_value(EXYNOS5410_GPK1(6));
	
	if(GPK1_5 == 0 && GPK1_6 == 0)
		version = BOARD_EVT;
	else if (GPK1_5 == 1 && GPK1_6 == 0) {
		s3c_gpio_setpull(EXYNOS5410_GPK1(5), S3C_GPIO_PULL_NONE);
		version = BOARD_DVT;
	}
	else if (GPK1_5 == 0 && GPK1_6 == 1) {
		s3c_gpio_setpull(EXYNOS5410_GPK1(6), S3C_GPIO_PULL_NONE);
		version = BOARD_PVT;
	}
	else if (GPK1_5 == 1 && GPK1_6 == 1) {		
		s3c_gpio_setpull(EXYNOS5410_GPK1(5), S3C_GPIO_PULL_NONE);
		s3c_gpio_setpull(EXYNOS5410_GPK1(6), S3C_GPIO_PULL_NONE);
		version = BOARD_PVT;
	}
	else {
		version = BOARD_PVT;
		printk("Board Ver Error. We set PVT to Board Version by force\n");
	}
	printk("0:EVT 1:DVT 2:PVT Board Version = %d\n",version); 
	board_version = version;
}

static void __init smdk5410_machine_init(void)
{
	check_board_version();
#ifdef CONFIG_EXYNOS_FIQ_DEBUGGER
	exynos_serial_debug_init(2, 0);
#endif

	exynos5_tf4_clock_init();
	exynos5_tf4_mmc_init();

	s3c_i2c0_set_platdata(NULL);
	s3c_i2c2_set_platdata(NULL);
	s3c_i2c3_set_platdata(NULL);
	exynos5_hs_i2c1_set_platdata(NULL);
	exynos5_hs_i2c2_set_platdata(NULL);
	exynos5_hs_i2c3_set_platdata(NULL);
	exynos5_tf4_audio_init();
	exynos5_tf4_usb_init();
	exynos5_tf4_power_init();
	exynos5_tf4_input_init();
	exynos5_tf4_display_init();
	exynos5_tf4_media_init();

#ifdef CONFIG_INV_SENSORS
	exynos5_tf4_sensor_init(); // shengliang: @2012-07-25
#endif
  s3c_config_sleep_gpio_table = tf4_config_sleep_gpio_table;
	ramconsole_pdata.bootinfo = exynos_get_resetreason();
	platform_add_devices(smdk5410_devices, ARRAY_SIZE(smdk5410_devices));

	pm_power_off = tf4_power_off;
	pm_power_off_prepare = tf4_power_off_prepare;

	register_reboot_notifier(&exynos5_reboot_notifier);

}

MACHINE_START(TF4, "TF4")
	.atag_offset	= 0x100,
	.init_early	= smdk5410_init_early,
	.init_irq	= exynos5_init_irq,
	.map_io		= smdk5410_map_io,
	.handle_irq	= gic_handle_irq,
	.init_machine	= smdk5410_machine_init,
	.timer		= &exynos4_timer,
	.restart	= exynos5_restart,
	.reserve	= exynos_reserve_mem,
MACHINE_END
