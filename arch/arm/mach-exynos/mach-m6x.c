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
#include <linux/cma.h>
#include <linux/io.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/power/max17047_battery.h>
#include <linux/bootmode.h>
#include <linux/i2c-gpio.h>
#include <linux/delay.h>

#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>
#include <asm/mach-types.h>

#include <plat/clock.h>
#include <plat/cpu.h>
#include <plat/devs.h>
#include <plat/regs-serial.h>
#include <plat/iic.h>
#include <plat/watchdog.h>
#include <plat/gpio-cfg.h>

#include <mach/exynos_fiq_debugger.h>
#include <mach/map.h>
#include <mach/hs-iic.h>
#include <mach/regs-pmu.h>
#include <mach/pmu.h>
#include <mach/gpio-i2c-m6x.h>  /*for gpio emulated i2c adapter platfom_devices*/
#include <mach/gpio-meizu.h>

#include "../../../drivers/staging/android/ram_console.h"
#include "common.h"
#include "board-m6x.h"
#include <mach/resetreason.h>

static struct ram_console_platform_data ramconsole_pdata;

static struct platform_device ramconsole_device = {
	.name	= "ram_console",
	.id	= -1,
	.dev	= {
		.platform_data = &ramconsole_pdata,
	},
};

static struct platform_device reset_reason_mem_device = {
	.name	= "reset_reason",
	.id	= -1,
};

static struct platform_device persistent_trace_device = {
	.name	= "persistent_trace",
	.id	= -1,
};

extern void bt_uart_wake_peer(struct uart_port *uport);

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define M6X_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define M6X_ULCON_DEFAULT	S3C2410_LCON_CS8

#define M6X_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

static struct s3c2410_uartcfg m6x_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= M6X_UCON_DEFAULT,
		.ulcon		= M6X_ULCON_DEFAULT,
		.ufcon		= M6X_UFCON_DEFAULT,
		.wake_peer	= bt_uart_wake_peer,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= M6X_UCON_DEFAULT,
		.ulcon		= M6X_ULCON_DEFAULT,
		.ufcon		= M6X_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= M6X_UCON_DEFAULT,
		.ulcon		= M6X_ULCON_DEFAULT,
		.ufcon		= M6X_UFCON_DEFAULT,
	},
#ifndef CONFIG_EXYNOS_FIQ_DEBUGGER
	[3] = {
	/*
	 * Don't need to initialize hwport 2, when FIQ debugger is
	 * enabled. Because it will be handled by fiq_debugger.
	 */
		.hwport		= 3,
		.flags		= 0,
		.ucon		= M6X_UCON_DEFAULT,
		.ulcon		= M6X_ULCON_DEFAULT,
		.ufcon		= M6X_UFCON_DEFAULT,
	},
#endif
};

static struct platform_device bcm4752_gps = {
	.name		= "bcm4752-gps",
	.id		= -1,
};

static struct platform_device m6x_bt = {
	.name = "bcm43341_bluetooth",
	.id = -1,
};

static struct platform_device *m6x_devices[] __initdata = {
	&ramconsole_device,
	&reset_reason_mem_device,
	&persistent_trace_device,
	&s3c_device_wdt,
	&s3c_device_rtc,
	&s3c_device_adc,
	&bcm4752_gps,
	&m6x_bt,
	&s3c_device_i2c0,/*For ISP*/
#ifdef	CONFIG_EXYNOS4_SETUP_I2C7
	&s3c_device_i2c7,
#endif
#ifdef CONFIG_PVR_SGX
	&exynos5_device_g3d,
#endif
#ifdef CONFIG_S5P_DEV_ACE
	&s5p_device_ace,
#endif

#ifdef CONFIG_PA_TFA9887
	&m6x_device_i2c11,
#endif
#ifdef CONFIG_SENSOR_GP2AP
	&m6x_device_i2c15,
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
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMD
			CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMD +
#endif
			196) * SZ_1M,
			{ .alignment = SZ_1M },
		},
#ifdef CONFIG_EXYNOS_CONTENT_PATH_PROTECTION
#ifdef CONFIG_ION_EXYNOS_DRM_MFC_SH
		{
			.name = "drm_mfc_sh",
			.size = SZ_1M,
		},
#endif
#ifdef CONFIG_ION_EXYNOS_DRM_MEMSIZE_G2D_WFD
		{
			.name = "drm_g2d_wfd",
			.size = CONFIG_ION_EXYNOS_DRM_MEMSIZE_G2D_WFD * SZ_1K,
		},
#endif
#endif
#ifdef CONFIG_RESET_REASON
		{
			.name = "reset_reason",
			.size = SZ_128K,
		},
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
			{
				.alignment = SZ_1M,
			}
	       },
#endif
#ifdef CONFIG_ION_EXYNOS_DRM_MEMSIZE_MFC_OUTPUT
	       {
			.name = "drm_mfc_output",
			.size = CONFIG_ION_EXYNOS_DRM_MEMSIZE_MFC_OUTPUT *
				SZ_1K,
			{
				.alignment = SZ_1M,
			}
	       },
#endif
#ifdef CONFIG_ION_EXYNOS_DRM_MEMSIZE_MFC_INPUT
	       {
			.name = "drm_mfc_input",
			.size = CONFIG_ION_EXYNOS_DRM_MEMSIZE_MFC_INPUT *
				SZ_1K,
			{
				.alignment = SZ_1M,
			}
	       },
#endif
#ifdef CONFIG_ION_EXYNOS_DRM_MFC_FW
		{
			.name = "drm_mfc_fw",
			.size = SZ_1M,
			{
				.alignment = SZ_1M,
			}
		},
#endif
#ifdef CONFIG_ION_EXYNOS_DRM_SECTBL
		{
			.name = "drm_sectbl",
			.size = SZ_1M,
			{
				.alignment = SZ_1M,
			}
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
#ifdef CONFIG_EXYNOS_CONTENT_PATH_PROTECTION
#ifdef CONFIG_SND_SAMSUNG_ALP
		"samsung-rp=srp;"
#endif

		"reset_reason=reset_reason;"
		"ion-exynos/mfc_sh=drm_mfc_sh;"
		"ion-exynos/g2d_wfd=drm_g2d_wfd;"
		"ion-exynos/fimd_video=drm_fimd_video;"
		"ion-exynos/mfc_output=drm_mfc_output;"
		"ion-exynos/mfc_input=drm_mfc_input;"
		"ion-exynos/mfc_fw=drm_mfc_fw;"
		"ion-exynos/sectbl=drm_sectbl;"
		"s5p-smem/mfc_sh=drm_mfc_sh;"
		"s5p-smem/g2d_wfd=drm_g2d_wfd;"
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

static void __init m6x_map_io(void)
{
	clk_xusbxti.rate = 24000000;
	clk_xxti.rate = 24000000;

	exynos_init_io(NULL, 0);
	s3c24xx_init_clocks(clk_xusbxti.rate);
	s3c24xx_init_uarts(m6x_uartcfgs, ARRAY_SIZE(m6x_uartcfgs));
}

static struct persistent_ram_descriptor m6x_prd[] __initdata = {
	{
		.name = "ram_console",
		.size = SZ_128K,
	},
#ifdef CONFIG_PERSISTENT_TRACER
	{
		.name = "persistent_trace",
		.size = SZ_1M,
	},
#endif
};

static struct persistent_ram m6x_pr __initdata = {
	.descs		= m6x_prd,
	.num_descs	= ARRAY_SIZE(m6x_prd),
	.start		= PLAT_PHYS_OFFSET + SZ_512M,
#ifdef CONFIG_PERSISTENT_TRACER
	.size		= SZ_128K + SZ_1M,
#else
	.size		= SZ_128K,
#endif
};

/* WDT */
static struct s3c_watchdog_platdata smdk5410_watchdog_platform_data = {
	exynos_pmu_wdt_control,
	PMU_WDT_RESET_TYPE1,
};

static void __init m6x_init_early(void)
{
#ifdef CONFIG_ANDROID_PERSISTENT_RAM
	persistent_ram_early_init(&m6x_pr);
#endif
}

/*m6x board version*/
static u32 m6x_board_version;
u32 m6x_get_board_version(void)
{
	return m6x_board_version;
}

static void m6x_init_board_version(void)
{
	bool id1 = 0, id2 = 0, id3 = 0;

	s3c_gpio_cfgpin(MEIZU_BOARD_ID1, S3C_GPIO_INPUT);
	s3c_gpio_setpull(MEIZU_BOARD_ID1, S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(MEIZU_BOARD_ID2, S3C_GPIO_INPUT);
	s3c_gpio_setpull(MEIZU_BOARD_ID2, S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(MEIZU_BOARD_ID3, S3C_GPIO_INPUT);
	s3c_gpio_setpull(MEIZU_BOARD_ID3, S3C_GPIO_PULL_NONE);

	udelay(10);

	id1 = !!s3c_gpio_getpin(MEIZU_BOARD_ID1);
	id2 = !!s3c_gpio_getpin(MEIZU_BOARD_ID2);
	id3 = !!s3c_gpio_getpin(MEIZU_BOARD_ID3);

	m6x_board_version = ((id3<<2) | (id2<<1) | (id1<<0));

	pr_debug("M6X Board Version: [%d]\n", m6x_board_version);
}

static void __init m6x_machine_init(void)
{
	if (machine_is_m65()) {
		meizu_gpio = m65_gpio_v3x_tbl;
	} else if (machine_is_m69()) {
		meizu_gpio = m69_gpio_v1_tbl;
	} else {
		pr_err("Error: Please Configure Machine Version.\r\n");
		BUG();
	}

	/*get board id, after meizu_gpio init*/
	m6x_init_board_version();

#ifdef CONFIG_EXYNOS_FIQ_DEBUGGER
	exynos_serial_debug_init(3, 0);
#endif

	s3c_watchdog_set_platdata(&smdk5410_watchdog_platform_data);
	exynos5_m6x_clock_init();
	exynos5_m6x_mmc_init();

	s3c_i2c0_set_platdata(NULL);
	s3c_i2c3_set_platdata(NULL);
	exynos5_hs_i2c1_set_platdata(NULL);
	exynos5_hs_i2c2_set_platdata(NULL);
	exynos5_hs_i2c3_set_platdata(NULL);
#ifdef CONFIG_REGULATOR_FIXED_VOLTAGE
	exynos5_m6x_fixed_voltage_init();
#endif
#ifdef CONFIG_MEIZU_M6X_SENSORS
	exynos5_m6x_sensors_init();
#endif
	exynos5_m6x_audio_init();
	exynos5_m6x_power_init();
	exynos5_m6x_input_init();
	exynos5_m6x_media_init();
#ifdef CONFIG_FB_S3C
	exynos5_m6x_display_init();
#endif
	ramconsole_pdata.bootinfo = exynos_get_resetreason();

	platform_add_devices(m6x_devices, ARRAY_SIZE(m6x_devices));
}

#if defined(CONFIG_MACH_M65) || defined(CONFIG_MACH_M69)
#ifdef CONFIG_MACH_M65
//MACHINE_START(M65, "tf4")
MACHINE_START(M65, "MX3")
	.atag_offset	= 0x100,
	.init_early	= m6x_init_early,
	.init_irq	= exynos5_init_irq,
	.map_io		= m6x_map_io,
	.handle_irq	= gic_handle_irq,
	.init_machine	= m6x_machine_init,
	.timer		= &exynos4_timer,
	.reserve	= exynos_reserve_mem,
MACHINE_END
#endif
#ifdef CONFIG_MACH_M69
//MACHINE_START(M69, "tf4")
MACHINE_START(M69, "MX3")
	.atag_offset	= 0x100,
	.init_early	= m6x_init_early,
	.init_irq	= exynos5_init_irq,
	.map_io		= m6x_map_io,
	.handle_irq	= gic_handle_irq,
	.init_machine	= m6x_machine_init,
	.timer		= &exynos4_timer,
	.reserve	= exynos_reserve_mem,
MACHINE_END
#endif
#else
#error "Please Configure The Machine Version."
#endif
