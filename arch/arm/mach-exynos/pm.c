/* linux/arch/arm/mach-exynos/pm.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS - Power Management support
 *
 * Based on arch/arm/mach-s3c2410/pm.c
 * Copyright (c) 2006 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/syscore_ops.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>

#include <asm/tlbflush.h>
#include <asm/cacheflush.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/smp_scu.h>
#include <asm/cputype.h>

#include <plat/cpu.h>
#include <plat/pm.h>
#include <plat/pll.h>
#include <plat/regs-srom.h>

#include <plat/bts.h>

#include <mach/regs-irq.h>
#include <mach/regs-gpio.h>
#include <mach/regs-clock.h>
#include <mach/regs-pmu.h>
#include <mach/pm-core.h>
#include <mach/pmu.h>
#include <mach/smc.h>

#ifdef CONFIG_ARM_TRUSTZONE
#define REG_INFORM0            (S5P_VA_SYSRAM_NS + 0x8)
#define REG_INFORM1            (S5P_VA_SYSRAM_NS + 0xC)
#else
#define REG_INFORM0            (EXYNOS_INFORM0)
#define REG_INFORM1            (EXYNOS_INFORM1)
#endif

#define EXYNOS_I2C_CFG		(S3C_VA_SYS + 0x234)

#define EXYNOS_WAKEUP_STAT_EINT		(1 << 0)
#define EXYNOS_WAKEUP_STAT_RTC_ALARM	(1 << 1)

#define msecs_to_loops(t) (loops_per_jiffy / 1000 * HZ * t)

static struct sleep_save exynos4_set_clksrc[] = {
	{ .reg = EXYNOS4_CLKSRC_MASK_TOP		, .val = 0x00000001, },
	{ .reg = EXYNOS4_CLKSRC_MASK_CAM		, .val = 0x11111111, },
	{ .reg = EXYNOS4_CLKSRC_MASK_TV			, .val = 0x00000111, },
	{ .reg = EXYNOS4_CLKSRC_MASK_LCD0		, .val = 0x00001111, },
	{ .reg = EXYNOS4_CLKSRC_MASK_MAUDIO		, .val = 0x00000001, },
	{ .reg = EXYNOS4_CLKSRC_MASK_FSYS		, .val = 0x01011111, },
	{ .reg = EXYNOS4_CLKSRC_MASK_PERIL0		, .val = 0x01111111, },
	{ .reg = EXYNOS4_CLKSRC_MASK_PERIL1		, .val = 0x01110111, },
	{ .reg = EXYNOS4_CLKSRC_MASK_DMC		, .val = 0x00010000, },
};

static struct sleep_save exynos4210_set_clksrc[] = {
	{ .reg = EXYNOS4210_CLKSRC_MASK_LCD1		, .val = 0x00001111, },
};

static struct sleep_save exynos4_epll_save[] = {
	SAVE_ITEM(EXYNOS4_EPLL_CON0),
	SAVE_ITEM(EXYNOS4_EPLL_CON1),
};

static struct sleep_save exynos4_vpll_save[] = {
	SAVE_ITEM(EXYNOS4_VPLL_CON0),
	SAVE_ITEM(EXYNOS4_VPLL_CON1),
};

static struct sleep_save exynos5_set_clksrc[] = {
	{ .reg = EXYNOS5_CLKSRC_MASK_TOP,		.val = 0xffffffff, },
	{ .reg = EXYNOS5_CLKSRC_MASK_GSCL,		.val = 0xffffffff, },
	{ .reg = EXYNOS5_CLKSRC_MASK_DISP1_0,		.val = 0xffffffff, },
	{ .reg = EXYNOS5_CLKSRC_MASK_MAUDIO,		.val = 0xffffffff, },
	{ .reg = EXYNOS5_CLKSRC_MASK_FSYS,		.val = 0xffffffff, },
	{ .reg = EXYNOS5_CLKSRC_MASK_PERIC0,		.val = 0xffffffff, },
	{ .reg = EXYNOS5_CLKSRC_MASK_PERIC1,		.val = 0xffffffff, },
	{ .reg = EXYNOS5_CLKSRC_MASK_ISP,		.val = 0xffffffff, },
};

static struct sleep_save exynos5410_set_clksrc[] = {
	{ .reg = EXYNOS5410_CLKSRC_MASK_CPERI,		.val = 0xffffffff, },
	{ .reg = EXYNOS5_CLKSRC_MASK_DISP0_0,		.val = 0xffffffff, },
	{ .reg = EXYNOS5_CLKSRC_MASK_DISP0_1,		.val = 0xffffffff, },
	{ .reg = EXYNOS5_CLKSRC_MASK_DISP1_1,		.val = 0xffffffff, },
	{ .reg = EXYNOS5_CLKSRC_MASK_FSYS,		.val = 0xffffffff, },
	{ .reg = EXYNOS5_CLKGATE_BUS_DISP1,		.val = 0xffffffff, },
};

static struct sleep_save exynos_core_save[] = {
	/* SROM side */
	SAVE_ITEM(S5P_SROM_BW),
	SAVE_ITEM(S5P_SROM_BC0),
	SAVE_ITEM(S5P_SROM_BC1),
	SAVE_ITEM(S5P_SROM_BC2),
	SAVE_ITEM(S5P_SROM_BC3),

	/* I2C CFG */
	SAVE_ITEM(EXYNOS_I2C_CFG),
};

struct sleep_save cmu_dump_reg[] = {
	{ .reg = EXYNOS_CLKREG(0x0000),  .val = 0xffffffff, },	 //  APLL_LOCK
	{ .reg = EXYNOS_CLKREG(0x0100),  .val = 0xffffffff, },	 //  APLL_CON0
	{ .reg = EXYNOS_CLKREG(0x0104),  .val = 0xffffffff, },	 //  APLL_CON1
	{ .reg = EXYNOS_CLKREG(0x0200),  .val = 0xffffffff, },	 //  CLK_SRC_CPU
	{ .reg = EXYNOS_CLKREG(0x0400),  .val = 0xffffffff, },	 //  CLK_MUX_STAT_CPU
	{ .reg = EXYNOS_CLKREG(0x0500),  .val = 0xffffffff, },	 //  CLK_DIV_CPU0
	{ .reg = EXYNOS_CLKREG(0x0504),  .val = 0xffffffff, },	 //  CLK_DIV_CPU1
	{ .reg = EXYNOS_CLKREG(0x0600),  .val = 0xffffffff, },	 //  CLK_DIV_STAT_CPU0
	{ .reg = EXYNOS_CLKREG(0x0604),  .val = 0xffffffff, },	 //  CLK_DIV_STAT_CPU1
	{ .reg = EXYNOS_CLKREG(0x0700),  .val = 0xffffffff, },	 //  CLK_GATE_BUS_CPU
	{ .reg = EXYNOS_CLKREG(0x0800),  .val = 0xffffffff, },	 //  CLK_GATE_SCLK_CPU
	{ .reg = EXYNOS_CLKREG(0x0A00),  .val = 0xffffffff, },	 //  CLKOUT_CMU_CPU
	{ .reg = EXYNOS_CLKREG(0x0A04),  .val = 0xffffffff, },	 //  CLKOUT_CMU_CPU_DIV_STAT
	{ .reg = EXYNOS_CLKREG(0x1000),  .val = 0xffffffff, },	 //  ARMCLK_STOPCTRL
	{ .reg = EXYNOS_CLKREG(0x1008),  .val = 0xffffffff, },	 //  ARM_EMA_CTRL
	{ .reg = EXYNOS_CLKREG(0x100C),  .val = 0xffffffff, },	 //  ARM_EMA_STATUS
	{ .reg = EXYNOS_CLKREG(0x1020),  .val = 0xffffffff, },	 //  PWR_CTRL
	{ .reg = EXYNOS_CLKREG(0x1024),  .val = 0xffffffff, },	 //  PWR_CTRL2
	{ .reg = EXYNOS_CLKREG(0x1100),  .val = 0xffffffff, },	 //  APLL_CON0_L8
	{ .reg = EXYNOS_CLKREG(0x1104),  .val = 0xffffffff, },	 //  APLL_CON0_L7
	{ .reg = EXYNOS_CLKREG(0x1108),  .val = 0xffffffff, },	 //  APLL_CON0_L6
	{ .reg = EXYNOS_CLKREG(0x110C),  .val = 0xffffffff, },	 //  APLL_CON0_L5
	{ .reg = EXYNOS_CLKREG(0x1110),  .val = 0xffffffff, },	 //  APLL_CON0_L4
	{ .reg = EXYNOS_CLKREG(0x1114),  .val = 0xffffffff, },	 //  APLL_CON0_L3
	{ .reg = EXYNOS_CLKREG(0x1118),  .val = 0xffffffff, },	 //  APLL_CON0_L2
	{ .reg = EXYNOS_CLKREG(0x111C),  .val = 0xffffffff, },	 //  APLL_CON0_L1
	{ .reg = EXYNOS_CLKREG(0x1120),  .val = 0xffffffff, },	 //  IEM_CONTROL
	{ .reg = EXYNOS_CLKREG(0x1200),  .val = 0xffffffff, },	 //  APLL_CON1_L8
	{ .reg = EXYNOS_CLKREG(0x1204),  .val = 0xffffffff, },	 //  APLL_CON1_L7
	{ .reg = EXYNOS_CLKREG(0x1208),  .val = 0xffffffff, },	 //  APLL_CON1_L6
	{ .reg = EXYNOS_CLKREG(0x120C),  .val = 0xffffffff, },	 //  APLL_CON1_L5
	{ .reg = EXYNOS_CLKREG(0x1210),  .val = 0xffffffff, },	 //  APLL_CON1_L4
	{ .reg = EXYNOS_CLKREG(0x1214),  .val = 0xffffffff, },	 //  APLL_CON1_L3
	{ .reg = EXYNOS_CLKREG(0x1218),  .val = 0xffffffff, },	 //  APLL_CON1_L2
	{ .reg = EXYNOS_CLKREG(0x121C),  .val = 0xffffffff, },	 //  APLL_CON1_L1
	{ .reg = EXYNOS_CLKREG(0x1300),  .val = 0xffffffff, },	 //  CLKDIV_IEM_L8
	{ .reg = EXYNOS_CLKREG(0x1304),  .val = 0xffffffff, },	 //  CLKDIV_IEM_L7
	{ .reg = EXYNOS_CLKREG(0x1308),  .val = 0xffffffff, },	 //  CLKDIV_IEM_L6
	{ .reg = EXYNOS_CLKREG(0x130C),  .val = 0xffffffff, },	 //  CLKDIV_IEM_L5
	{ .reg = EXYNOS_CLKREG(0x1310),  .val = 0xffffffff, },	 //  CLKDIV_IEM_L4
	{ .reg = EXYNOS_CLKREG(0x1314),  .val = 0xffffffff, },	 //  CLKDIV_IEM_L3
	{ .reg = EXYNOS_CLKREG(0x1318),  .val = 0xffffffff, },	 //  CLKDIV_IEM_L2
	{ .reg = EXYNOS_CLKREG(0x131C),  .val = 0xffffffff, },	 //  CLKDIV_IEM_L1
	{ .reg = EXYNOS_CLKREG(0x1400),  .val = 0xffffffff, },	 //  L2_STATUS
	{ .reg = EXYNOS_CLKREG(0x1410),  .val = 0xffffffff, },	 //  CPU_STATUS
	{ .reg = EXYNOS_CLKREG(0x1420),  .val = 0xffffffff, },	 //  PTM_STATUS
	{ .reg = EXYNOS_CLKREG(0x2000),  .val = 0xffffffff, },	 //  CMU_CPU_SPARE0
	{ .reg = EXYNOS_CLKREG(0x2004),  .val = 0xffffffff, },	 //  CMU_CPU_SPARE1
	{ .reg = EXYNOS_CLKREG(0x2008),  .val = 0xffffffff, },	 //  CMU_CPU_SPARE2
	{ .reg = EXYNOS_CLKREG(0x200C),  .val = 0xffffffff, },	 //  CMU_CPU_SPARE3
	{ .reg = EXYNOS_CLKREG(0x2010),  .val = 0xffffffff, },	 //  CMU_CPU_SPARE4
	{ .reg = EXYNOS_CLKREG(0x3FF0),  .val = 0xffffffff, },	 //  CMU_CPU_VERSION
	{ .reg = EXYNOS_CLKREG(0x4000),  .val = 0xffffffff, },	 //  MPLL_LOCK
	{ .reg = EXYNOS_CLKREG(0x4100),  .val = 0xffffffff, },	 //  MPLL_CON0
	{ .reg = EXYNOS_CLKREG(0x4104),  .val = 0xffffffff, },	 //  MPLL_CON1
	{ .reg = EXYNOS_CLKREG(0x4200),  .val = 0xffffffff, },	 //  CLK_SRC_CPERI0
	{ .reg = EXYNOS_CLKREG(0x4204),  .val = 0xffffffff, },	 //  CLK_SRC_CPERI1
	{ .reg = EXYNOS_CLKREG(0x4300),  .val = 0xffffffff, },	 //  CLK_SRC_MASK_CPERI
	{ .reg = EXYNOS_CLKREG(0x4404),  .val = 0xffffffff, },	 //  CLK_MUX_STAT_CPERI1
	{ .reg = EXYNOS_CLKREG(0x4504),  .val = 0xffffffff, },	 //  CLK_DIV_CPERI1
	{ .reg = EXYNOS_CLKREG(0x4604),  .val = 0xffffffff, },	 //  CLK_DIV_STAT_CPERI1
	{ .reg = EXYNOS_CLKREG(0x4700),  .val = 0xffffffff, },	 //  CLK_GATE_BUS_CPERI0
	{ .reg = EXYNOS_CLKREG(0x4704),  .val = 0xffffffff, },	 //  CLK_GATE_BUS_CPERI1
	{ .reg = EXYNOS_CLKREG(0x4800),  .val = 0xffffffff, },	 //  CLK_GATE_SCLK_CPERI
	{ .reg = EXYNOS_CLKREG(0x4900),  .val = 0xffffffff, },	 //  CLK_GATE_IP_CPERI
	{ .reg = EXYNOS_CLKREG(0x4A00),  .val = 0xffffffff, },	 //  CLKOUT_CMU_CPERI
	{ .reg = EXYNOS_CLKREG(0x4A04),  .val = 0xffffffff, },	 //  CLKOUT_CMU_CPERI_DIV_STAT
	{ .reg = EXYNOS_CLKREG(0x5000),  .val = 0xffffffff, },	 //  DCGIDX_MAP0
	{ .reg = EXYNOS_CLKREG(0x5004),  .val = 0xffffffff, },	 //  DCGIDX_MAP1
	{ .reg = EXYNOS_CLKREG(0x5008),  .val = 0xffffffff, },	 //  DCGIDX_MAP2
	{ .reg = EXYNOS_CLKREG(0x5020),  .val = 0xffffffff, },	 //  DCGPERF_MAP0
	{ .reg = EXYNOS_CLKREG(0x5024),  .val = 0xffffffff, },	 //  DCGPERF_MAP1
	{ .reg = EXYNOS_CLKREG(0x5040),  .val = 0xffffffff, },	 //  DVCIDX_MAP
	{ .reg = EXYNOS_CLKREG(0x5060),  .val = 0xffffffff, },	 //  FREQ_CPU
	{ .reg = EXYNOS_CLKREG(0x5064),  .val = 0xffffffff, },	 //  FREQ_DPM
	{ .reg = EXYNOS_CLKREG(0x5080),  .val = 0xffffffff, },	 //  DVSEMCLK_EN
	{ .reg = EXYNOS_CLKREG(0x5084),  .val = 0xffffffff, },	 //  MAXPERF
	{ .reg = EXYNOS_CLKREG(0x7F00),  .val = 0xffffffff, },	 //  CMU_CPERI_SPARE0
	{ .reg = EXYNOS_CLKREG(0x7F04),  .val = 0xffffffff, },	 //  CMU_CPERI_SPARE1
	{ .reg = EXYNOS_CLKREG(0x7F08),  .val = 0xffffffff, },	 //  CMU_CPERI_SPARE2
	{ .reg = EXYNOS_CLKREG(0x7F0C),  .val = 0xffffffff, },	 //  CMU_CPERI_SPARE3
	{ .reg = EXYNOS_CLKREG(0x7F10),  .val = 0xffffffff, },	 //  CMU_CPERI_SPARE4
	{ .reg = EXYNOS_CLKREG(0x7F14),  .val = 0xffffffff, },	 //  CMU_CPERI_SPARE5
	{ .reg = EXYNOS_CLKREG(0x7F18),  .val = 0xffffffff, },	 //  CMU_CPERI_SPARE6
	{ .reg = EXYNOS_CLKREG(0x7F1C),  .val = 0xffffffff, },	 //  CMU_CPERI_SPARE7
	{ .reg = EXYNOS_CLKREG(0x7F20),  .val = 0xffffffff, },	 //  CMU_CPERI_SPARE8
	{ .reg = EXYNOS_CLKREG(0x7FF0),  .val = 0xffffffff, },	 //  CMU_CPERI_VERSION
	{ .reg = EXYNOS_CLKREG(0x8500),  .val = 0xffffffff, },	 //  CLK_DIV_G2D
	{ .reg = EXYNOS_CLKREG(0x8600),  .val = 0xffffffff, },	 //  CLK_DIV_STAT_G2D
	{ .reg = EXYNOS_CLKREG(0x8700),  .val = 0xffffffff, },	 //  CLK_GATE_BUS_G2D
	{ .reg = EXYNOS_CLKREG(0x8800),  .val = 0xffffffff, },	 //  CLK_GATE_IP_G2D
	{ .reg = EXYNOS_CLKREG(0x8A00),  .val = 0xffffffff, },	 //  CLKOUT_CMU_G2D
	{ .reg = EXYNOS_CLKREG(0x8A04),  .val = 0xffffffff, },	 //  CLKOUT_CMU_G2D_DIV_STAT
	{ .reg = EXYNOS_CLKREG(0x8B00),  .val = 0xffffffff, },	 //  CMU_G2D_SPARE0
	{ .reg = EXYNOS_CLKREG(0x8B04),  .val = 0xffffffff, },	 //  CMU_G2D_SPARE1
	{ .reg = EXYNOS_CLKREG(0x8B08),  .val = 0xffffffff, },	 //  CMU_G2D_SPARE2
	{ .reg = EXYNOS_CLKREG(0x8B0C),  .val = 0xffffffff, },	 //  CMU_G2D_SPARE3
	{ .reg = EXYNOS_CLKREG(0x8B10),  .val = 0xffffffff, },	 //  CMU_G2D_SPARE4
	{ .reg = EXYNOS_CLKREG(0xBFF0),  .val = 0xffffffff, },	 //  CMU_G2D_VERSION
//	{ .reg = EXYNOS_CLKREG(0xC300),  .val = 0xffffffff, },	 //  CLK_DIV_ISP0
//	{ .reg = EXYNOS_CLKREG(0xC304),  .val = 0xffffffff, },	 //  CLK_DIV_ISP1
//	{ .reg = EXYNOS_CLKREG(0xC308),  .val = 0xffffffff, },	 //  CLK_DIV_ISP2
//	{ .reg = EXYNOS_CLKREG(0xC400),  .val = 0xffffffff, },	 //  CLK_DIV_STAT_ISP0
//	{ .reg = EXYNOS_CLKREG(0xC404),  .val = 0xffffffff, },	 //  CLK_DIV_STAT_ISP1
//	{ .reg = EXYNOS_CLKREG(0xC408),  .val = 0xffffffff, },	 //  CLK_DIV_STAT_ISP2
//	{ .reg = EXYNOS_CLKREG(0xC700),  .val = 0xffffffff, },	 //  CLK_GATE_BUS_ISP0
//	{ .reg = EXYNOS_CLKREG(0xC704),  .val = 0xffffffff, },	 //  CLK_GATE_BUS_ISP1
//	{ .reg = EXYNOS_CLKREG(0xC708),  .val = 0xffffffff, },	 //  CLK_GATE_BUS_ISP2
//	{ .reg = EXYNOS_CLKREG(0xC70C),  .val = 0xffffffff, },	 //  CLK_GATE_BUS_ISP3
//	{ .reg = EXYNOS_CLKREG(0xC800),  .val = 0xffffffff, },	 //  CLK_GATE_IP_ISP0
//	{ .reg = EXYNOS_CLKREG(0xC804),  .val = 0xffffffff, },	 //  CLK_GATE_IP_ISP1
//	{ .reg = EXYNOS_CLKREG(0xC900),  .val = 0xffffffff, },	 //  CLK_GATE_SCLK_ISP
//	{ .reg = EXYNOS_CLKREG(0xC910),  .val = 0xffffffff, },	 //  MCUISP_PWR_CTRL
//	{ .reg = EXYNOS_CLKREG(0xCA00),  .val = 0xffffffff, },	 //  CLKOUT_CMU_ISP
//	{ .reg = EXYNOS_CLKREG(0xCA04),  .val = 0xffffffff, },	 //  CLKOUT_CMU_ISP_DIV_STAT
//	{ .reg = EXYNOS_CLKREG(0xCB00),  .val = 0xffffffff, },	 //  CMU_ISP_SPARE0
//	{ .reg = EXYNOS_CLKREG(0xCB04),  .val = 0xffffffff, },	 //  CMU_ISP_SPARE1
//	{ .reg = EXYNOS_CLKREG(0xCB08),  .val = 0xffffffff, },	 //  CMU_ISP_SPARE2
//	{ .reg = EXYNOS_CLKREG(0xCB0C),  .val = 0xffffffff, },	 //  CMU_ISP_SPARE3
//	{ .reg = EXYNOS_CLKREG(0xFFF0),  .val = 0xffffffff, },	 //  CMU_ISP_VERSION
	{ .reg = EXYNOS_CLKREG(0x10020),  .val = 0xffffffff, },   //  CPLL_LOCK
	{ .reg = EXYNOS_CLKREG(0x10030),  .val = 0xffffffff, },   //  DPLL_LOCK
	{ .reg = EXYNOS_CLKREG(0x10040),  .val = 0xffffffff, },   //  EPLL_LOCK
	{ .reg = EXYNOS_CLKREG(0x10050),  .val = 0xffffffff, },   //  VPLL_LOCK
	{ .reg = EXYNOS_CLKREG(0x10060),  .val = 0xffffffff, },   //  IPLL_LOCK
	{ .reg = EXYNOS_CLKREG(0x10120),  .val = 0xffffffff, },   //  CPLL_CON0
	{ .reg = EXYNOS_CLKREG(0x10124),  .val = 0xffffffff, },   //  CPLL_CON1
	{ .reg = EXYNOS_CLKREG(0x10128),  .val = 0xffffffff, },   //  DPLL_CON0
	{ .reg = EXYNOS_CLKREG(0x1012C),  .val = 0xffffffff, },   //  DPLL_CON1
	{ .reg = EXYNOS_CLKREG(0x10130),  .val = 0xffffffff, },   //  EPLL_CON0
	{ .reg = EXYNOS_CLKREG(0x10134),  .val = 0xffffffff, },   //  EPLL_CON1
	{ .reg = EXYNOS_CLKREG(0x10138),  .val = 0xffffffff, },   //  EPLL_CON2
	{ .reg = EXYNOS_CLKREG(0x10140),  .val = 0xffffffff, },   //  VPLL_CON0
	{ .reg = EXYNOS_CLKREG(0x10144),  .val = 0xffffffff, },   //  VPLL_CON1
	{ .reg = EXYNOS_CLKREG(0x10148),  .val = 0xffffffff, },   //  VPLL_CON2
	{ .reg = EXYNOS_CLKREG(0x10150),  .val = 0xffffffff, },   //  IPLL_CON0
	{ .reg = EXYNOS_CLKREG(0x10154),  .val = 0xffffffff, },   //  IPLL_CON1
	{ .reg = EXYNOS_CLKREG(0x10210),  .val = 0xffffffff, },   //  CLK_SRC_TOP0
	{ .reg = EXYNOS_CLKREG(0x10214),  .val = 0xffffffff, },   //  CLK_SRC_TOP1
	{ .reg = EXYNOS_CLKREG(0x10218),  .val = 0xffffffff, },   //  CLK_SRC_TOP2
	{ .reg = EXYNOS_CLKREG(0x1021C),  .val = 0xffffffff, },   //  CLK_SRC_TOP3
	{ .reg = EXYNOS_CLKREG(0x10220),  .val = 0xffffffff, },   //  CLK_SRC_GSCL
	{ .reg = EXYNOS_CLKREG(0x10224),  .val = 0xffffffff, },   //  CLK_SRC_DISP00
	{ .reg = EXYNOS_CLKREG(0x10228),  .val = 0xffffffff, },   //  CLK_SRC_DISP01
	{ .reg = EXYNOS_CLKREG(0x1022C),  .val = 0xffffffff, },   //  CLK_SRC_DISP10
	{ .reg = EXYNOS_CLKREG(0x10230),  .val = 0xffffffff, },   //  CLK_SRC_DISP11
	{ .reg = EXYNOS_CLKREG(0x10240),  .val = 0xffffffff, },   //  CLK_SRC_MAU
	{ .reg = EXYNOS_CLKREG(0x10244),  .val = 0xffffffff, },   //  CLK_SRC_FSYS
	{ .reg = EXYNOS_CLKREG(0x10250),  .val = 0xffffffff, },   //  CLK_SRC_PERIC0
	{ .reg = EXYNOS_CLKREG(0x10254),  .val = 0xffffffff, },   //  CLK_SRC_PERIC1
	{ .reg = EXYNOS_CLKREG(0x10270),  .val = 0xffffffff, },   //  SCLK_SRC_ISP
	{ .reg = EXYNOS_CLKREG(0x10310),  .val = 0xffffffff, },   //  CLK_SRC_MASK_TOP
	{ .reg = EXYNOS_CLKREG(0x10320),  .val = 0xffffffff, },   //  CLK_SRC_MASK_GSCL
	{ .reg = EXYNOS_CLKREG(0x10324),  .val = 0xffffffff, },   //  CLK_SRC_MASK_DISP00
	{ .reg = EXYNOS_CLKREG(0x10328),  .val = 0xffffffff, },   //  CLK_SRC_MASK_DISP01
	{ .reg = EXYNOS_CLKREG(0x1032C),  .val = 0xffffffff, },   //  CLK_SRC_MASK_DISP10
	{ .reg = EXYNOS_CLKREG(0x10330),  .val = 0xffffffff, },   //  CLK_SRC_MASK_DISP11
	{ .reg = EXYNOS_CLKREG(0x10334),  .val = 0xffffffff, },   //  CLK_SRC_MASK_MAU
	{ .reg = EXYNOS_CLKREG(0x10340),  .val = 0xffffffff, },   //  CLK_SRC_MASK_FSYS
	{ .reg = EXYNOS_CLKREG(0x10350),  .val = 0xffffffff, },   //  CLK_SRC_MASK_PERIC0
	{ .reg = EXYNOS_CLKREG(0x10354),  .val = 0xffffffff, },   //  CLK_SRC_MASK_PERIC1
	{ .reg = EXYNOS_CLKREG(0x10370),  .val = 0xffffffff, },   //  SCLK_SRC_MASK_ISP
	{ .reg = EXYNOS_CLKREG(0x10410),  .val = 0xffffffff, },   //  CLK_MUX_STAT_TOP0
	{ .reg = EXYNOS_CLKREG(0x10414),  .val = 0xffffffff, },   //  CLK_MUX_STAT_TOP1
	{ .reg = EXYNOS_CLKREG(0x10418),  .val = 0xffffffff, },   //  CLK_MUX_STAT_TOP2
	{ .reg = EXYNOS_CLKREG(0x1041C),  .val = 0xffffffff, },   //  CLK_MUX_STAT_TOP3
	{ .reg = EXYNOS_CLKREG(0x10420),  .val = 0xffffffff, },   //  CLK_MUX_STAT_TOP4
	{ .reg = EXYNOS_CLKREG(0x10510),  .val = 0xffffffff, },   //  CLK_DIV_TOP0
	{ .reg = EXYNOS_CLKREG(0x10514),  .val = 0xffffffff, },   //  CLK_DIV_TOP1
	{ .reg = EXYNOS_CLKREG(0x10518),  .val = 0xffffffff, },   //  CLK_DIV_TOP2
	{ .reg = EXYNOS_CLKREG(0x1051C),  .val = 0xffffffff, },   //  CLK_DIV_TOP3
	{ .reg = EXYNOS_CLKREG(0x10520),  .val = 0xffffffff, },   //  CLK_DIV_GSCL
	{ .reg = EXYNOS_CLKREG(0x10524),  .val = 0xffffffff, },   //  CLK_DIV_DISP00
	{ .reg = EXYNOS_CLKREG(0x10528),  .val = 0xffffffff, },   //  CLK_DIV_DISP01
	{ .reg = EXYNOS_CLKREG(0x1052C),  .val = 0xffffffff, },   //  CLK_DIV_DISP10
	{ .reg = EXYNOS_CLKREG(0x10530),  .val = 0xffffffff, },   //  CLK_DIV_DISP11
	{ .reg = EXYNOS_CLKREG(0x1053C),  .val = 0xffffffff, },   //  CLK_DIV_GEN
	{ .reg = EXYNOS_CLKREG(0x10544),  .val = 0xffffffff, },   //  CLK_DIV_MAU
	{ .reg = EXYNOS_CLKREG(0x10548),  .val = 0xffffffff, },   //  CLK_DIV_FSYS0
	{ .reg = EXYNOS_CLKREG(0x1054C),  .val = 0xffffffff, },   //  CLK_DIV_FSYS1
	{ .reg = EXYNOS_CLKREG(0x10550),  .val = 0xffffffff, },   //  CLK_DIV_FSYS2
	{ .reg = EXYNOS_CLKREG(0x10554),  .val = 0xffffffff, },   //  CLK_DIV_FSYS3
	{ .reg = EXYNOS_CLKREG(0x10558),  .val = 0xffffffff, },   //  CLK_DIV_PERIC0
	{ .reg = EXYNOS_CLKREG(0x1055C),  .val = 0xffffffff, },   //  CLK_DIV_PERIC1
	{ .reg = EXYNOS_CLKREG(0x10560),  .val = 0xffffffff, },   //  CLK_DIV_PERIC2
	{ .reg = EXYNOS_CLKREG(0x10564),  .val = 0xffffffff, },   //  CLK_DIV_PERIC3
	{ .reg = EXYNOS_CLKREG(0x10568),  .val = 0xffffffff, },   //  CLK_DIV_PERIC4
	{ .reg = EXYNOS_CLKREG(0x1056C),  .val = 0xffffffff, },   //  CLK_DIV_PERIC5
	{ .reg = EXYNOS_CLKREG(0x10580),  .val = 0xffffffff, },   //  SCLK_DIV_ISP
	{ .reg = EXYNOS_CLKREG(0x10584),  .val = 0xffffffff, },   //  SCLK_DIV_ISP1
	{ .reg = EXYNOS_CLKREG(0x10590),  .val = 0xffffffff, },   //  CLKDIV2_RATIO0
	{ .reg = EXYNOS_CLKREG(0x105A0),  .val = 0xffffffff, },   //  CLKDIV4_RATIO
	{ .reg = EXYNOS_CLKREG(0x10610),  .val = 0xffffffff, },   //  CLK_DIV_STAT_TOP0
	{ .reg = EXYNOS_CLKREG(0x10614),  .val = 0xffffffff, },   //  CLK_DIV_STAT_TOP1
	{ .reg = EXYNOS_CLKREG(0x10618),  .val = 0xffffffff, },   //  CLK_DIV_STAT_TOP2
	{ .reg = EXYNOS_CLKREG(0x10620),  .val = 0xffffffff, },   //  CLK_DIV_STAT_GSCL
	{ .reg = EXYNOS_CLKREG(0x10624),  .val = 0xffffffff, },   //  CLK_DIV_STAT_DISP00
	{ .reg = EXYNOS_CLKREG(0x10628),  .val = 0xffffffff, },   //  CLK_DIV_STAT_DISP01
	{ .reg = EXYNOS_CLKREG(0x1062C),  .val = 0xffffffff, },   //  CLK_DIV_STAT_DISP10
	{ .reg = EXYNOS_CLKREG(0x10630),  .val = 0xffffffff, },   //  CLK_DIV_STAT_DISP11
	{ .reg = EXYNOS_CLKREG(0x1063C),  .val = 0xffffffff, },   //  CLK_DIV_STAT_GEN
	{ .reg = EXYNOS_CLKREG(0x10644),  .val = 0xffffffff, },   //  CLK_DIV_STAT_MAU
	{ .reg = EXYNOS_CLKREG(0x10648),  .val = 0xffffffff, },   //  CLK_DIV_STAT_FSYS0
	{ .reg = EXYNOS_CLKREG(0x1064C),  .val = 0xffffffff, },   //  CLK_DIV_STAT_FSYS1
	{ .reg = EXYNOS_CLKREG(0x10650),  .val = 0xffffffff, },   //  CLK_DIV_STAT_FSYS2
	{ .reg = EXYNOS_CLKREG(0x10654),  .val = 0xffffffff, },   //  CLK_DIV_STAT_FSYS3
	{ .reg = EXYNOS_CLKREG(0x10658),  .val = 0xffffffff, },   //  CLK_DIV_STAT_PERIC0
	{ .reg = EXYNOS_CLKREG(0x1065C),  .val = 0xffffffff, },   //  CLK_DIV_STAT_PERIC1
	{ .reg = EXYNOS_CLKREG(0x10660),  .val = 0xffffffff, },   //  CLK_DIV_STAT_PERIC2
	{ .reg = EXYNOS_CLKREG(0x10664),  .val = 0xffffffff, },   //  CLK_DIV_STAT_PERIC3
	{ .reg = EXYNOS_CLKREG(0x10668),  .val = 0xffffffff, },   //  CLK_DIV_STAT_PERIC4
	{ .reg = EXYNOS_CLKREG(0x1066C),  .val = 0xffffffff, },   //  CLK_DIV_STAT_PERIC5
	{ .reg = EXYNOS_CLKREG(0x10680),  .val = 0xffffffff, },   //  SCLK_DIV_STAT_ISP
	{ .reg = EXYNOS_CLKREG(0x10684),  .val = 0xffffffff, },   //  SCLK_DIV_STAT_ISP1
	{ .reg = EXYNOS_CLKREG(0x10690),  .val = 0xffffffff, },   //  CLKDIV2_STAT0
	{ .reg = EXYNOS_CLKREG(0x106A0),  .val = 0xffffffff, },   //  CLKDIV4_STAT
	{ .reg = EXYNOS_CLKREG(0x10710),  .val = 0xffffffff, },   //  CLK_GATE_BUS_GSCL0
	{ .reg = EXYNOS_CLKREG(0x10720),  .val = 0xffffffff, },   //  CLK_GATE_BUS_GSCL1
	{ .reg = EXYNOS_CLKREG(0x10724),  .val = 0xffffffff, },   //  CLK_GATE_BUS_DISP0
	{ .reg = EXYNOS_CLKREG(0x10728),  .val = 0xffffffff, },   //  CLK_GATE_BUS_DISP1
	{ .reg = EXYNOS_CLKREG(0x10734),  .val = 0xffffffff, },   //  CLK_GATE_BUS_MFC
	{ .reg = EXYNOS_CLKREG(0x10738),  .val = 0xffffffff, },   //  CLK_GATE_BUS_G3D
	{ .reg = EXYNOS_CLKREG(0x1073C),  .val = 0xffffffff, },   //  CLK_GATE_BUS_GEN
	{ .reg = EXYNOS_CLKREG(0x10740),  .val = 0xffffffff, },   //  CLK_GATE_BUS_FSYS0
	{ .reg = EXYNOS_CLKREG(0x10744),  .val = 0xffffffff, },   //  CLK_GATE_BUS_FSYS1
	{ .reg = EXYNOS_CLKREG(0x1074C),  .val = 0xffffffff, },   //  CLK_GATE_BUS_GPS
	{ .reg = EXYNOS_CLKREG(0x10750),  .val = 0xffffffff, },   //  CLK_GATE_BUS_PERIC
	{ .reg = EXYNOS_CLKREG(0x10760),  .val = 0xffffffff, },   //  CLK_GATE_BUS_PERIS0
	{ .reg = EXYNOS_CLKREG(0x10764),  .val = 0xffffffff, },   //  CLK_GATE_BUS_PERIS1
	{ .reg = EXYNOS_CLKREG(0x10770),  .val = 0xffffffff, },   //  CLK_GATE_BUS_NOC
	{ .reg = EXYNOS_CLKREG(0x10820),  .val = 0xffffffff, },   //  CLK_GATE_TOP_SCLK_GSCL
	{ .reg = EXYNOS_CLKREG(0x10824),  .val = 0xffffffff, },   //  CLK_GATE_TOP_SCLK_DISP0
	{ .reg = EXYNOS_CLKREG(0x10828),  .val = 0xffffffff, },   //  CLK_GATE_TOP_SCLK_DISP1
	{ .reg = EXYNOS_CLKREG(0x1082C),  .val = 0xffffffff, },   //  CLK_GATE_TOP_SCLK_GEN
	{ .reg = EXYNOS_CLKREG(0x1083C),  .val = 0xffffffff, },   //  CLK_GATE_TOP_SCLK_MAU
	{ .reg = EXYNOS_CLKREG(0x10840),  .val = 0xffffffff, },   //  CLK_GATE_TOP_SCLK_FSYS
	{ .reg = EXYNOS_CLKREG(0x10850),  .val = 0xffffffff, },   //  CLK_GATE_TOP_SCLK_PERIC
	{ .reg = EXYNOS_CLKREG(0x10870),  .val = 0xffffffff, },   //  CLK_GATE_TOP_SCLK_ISP
	{ .reg = EXYNOS_CLKREG(0x10910),  .val = 0xffffffff, },   //  CLK_GATE_IP_GSCL0
	{ .reg = EXYNOS_CLKREG(0x10920),  .val = 0xffffffff, },   //  CLK_GATE_IP_GSCL1
	{ .reg = EXYNOS_CLKREG(0x10924),  .val = 0xffffffff, },   //  CLK_GATE_IP_DISP0
	{ .reg = EXYNOS_CLKREG(0x10928),  .val = 0xffffffff, },   //  CLK_GATE_IP_DISP1
	{ .reg = EXYNOS_CLKREG(0x1092C),  .val = 0xffffffff, },   //  CLK_GATE_IP_MFC
	{ .reg = EXYNOS_CLKREG(0x10930),  .val = 0xffffffff, },   //  CLK_GATE_IP_G3D
	{ .reg = EXYNOS_CLKREG(0x10934),  .val = 0xffffffff, },   //  CLK_GATE_IP_GEN
	{ .reg = EXYNOS_CLKREG(0x10944),  .val = 0xffffffff, },   //  CLK_GATE_IP_FSYS
	{ .reg = EXYNOS_CLKREG(0x1094C),  .val = 0xffffffff, },   //  CLK_GATE_IP_GPS
	{ .reg = EXYNOS_CLKREG(0x10950),  .val = 0xffffffff, },   //  CLK_GATE_IP_PERIC
	{ .reg = EXYNOS_CLKREG(0x10960),  .val = 0xffffffff, },   //  CLK_GATE_IP_PERIS
	{ .reg = EXYNOS_CLKREG(0x10980),  .val = 0xffffffff, },   //  CLK_GATE_BLOCK
	{ .reg = EXYNOS_CLKREG(0x109A0),  .val = 0xffffffff, },   //  MCUIOP_PWR_CTRL
	{ .reg = EXYNOS_CLKREG(0x10A00),  .val = 0xffffffff, },   //  CLKOUT_CMU_TOP
	{ .reg = EXYNOS_CLKREG(0x10A04),  .val = 0xffffffff, },   //  CLKOUT_CMU_TOP_DIV_STAT
	{ .reg = EXYNOS_CLKREG(0x10B00),  .val = 0xffffffff, },   //  CMU_TOP_SPARE0
	{ .reg = EXYNOS_CLKREG(0x10B04),  .val = 0xffffffff, },   //  CMU_TOP_SPARE1
	{ .reg = EXYNOS_CLKREG(0x10B08),  .val = 0xffffffff, },   //  CMU_TOP_SPARE2
	{ .reg = EXYNOS_CLKREG(0x10B0C),  .val = 0xffffffff, },   //  CMU_TOP_SPARE3
	{ .reg = EXYNOS_CLKREG(0x10B10),  .val = 0xffffffff, },   //  CMU_TOP_SPARE4
	{ .reg = EXYNOS_CLKREG(0x13FF0),  .val = 0xffffffff, },   //  CMU_TOP_VERSION
	{ .reg = EXYNOS_CLKREG(0x20010),  .val = 0xffffffff, },   //  BPLL_LOCK
	{ .reg = EXYNOS_CLKREG(0x20110),  .val = 0xffffffff, },   //  BPLL_CON0
	{ .reg = EXYNOS_CLKREG(0x20114),  .val = 0xffffffff, },   //  BPLL_CON1
	{ .reg = EXYNOS_CLKREG(0x20200),  .val = 0xffffffff, },   //  CLK_SRC_CDREX
	{ .reg = EXYNOS_CLKREG(0x20400),  .val = 0xffffffff, },   //  CLK_MUX_STAT_CDREX
	{ .reg = EXYNOS_CLKREG(0x20500),  .val = 0xffffffff, },   //  CLK_DIV_CDREX0
	{ .reg = EXYNOS_CLKREG(0x20504),  .val = 0xffffffff, },   //  CLK_DIV_CDREX1
	{ .reg = EXYNOS_CLKREG(0x20600),  .val = 0xffffffff, },   //  CLK_DIV_STAT_CDREX
	{ .reg = EXYNOS_CLKREG(0x20700),  .val = 0xffffffff, },   //  CLK_GATE_BUS_CDREX
	{ .reg = EXYNOS_CLKREG(0x20900),  .val = 0xffffffff, },   //  CLK_GATE_IP_CDREX
	{ .reg = EXYNOS_CLKREG(0x20910),  .val = 0xffffffff, },   //  C2C_MONITOR
	{ .reg = EXYNOS_CLKREG(0x20914),  .val = 0xffffffff, },   //  DMC_FREQ_CTRL
	{ .reg = EXYNOS_CLKREG(0x2091C),  .val = 0xffffffff, },   //  PAUSE
	{ .reg = EXYNOS_CLKREG(0x20920),  .val = 0xffffffff, },   //  DDRPHY_LOCK_CTRL
	{ .reg = EXYNOS_CLKREG(0x20A00),  .val = 0xffffffff, },   //  CLKOUT_CMU_CDREX
	{ .reg = EXYNOS_CLKREG(0x20A04),  .val = 0xffffffff, },   //  CLKOUT_CMU_CDREX_DIV_STAT
	{ .reg = EXYNOS_CLKREG(0x20A10),  .val = 0xffffffff, },   //  LPDDR3PHY_CTRL
	{ .reg = EXYNOS_CLKREG(0x20A14),  .val = 0xffffffff, },   //  LPDDR3PHY_CON0
	{ .reg = EXYNOS_CLKREG(0x20A18),  .val = 0xffffffff, },   //  LPDDR3PHY_CON1
	{ .reg = EXYNOS_CLKREG(0x20A1C),  .val = 0xffffffff, },   //  LPDDR3PHY_CON2
	{ .reg = EXYNOS_CLKREG(0x20A20),  .val = 0xffffffff, },   //  LPDDR3PHY_CON3
	{ .reg = EXYNOS_CLKREG(0x20A24),  .val = 0xffffffff, },   //  LPDDR3PHY_CON4
	{ .reg = EXYNOS_CLKREG(0x20A28),  .val = 0xffffffff, },   //  LPDDR3PHY_CON5
	{ .reg = EXYNOS_CLKREG(0x20A2C),  .val = 0xffffffff, },   //  RSVD
	{ .reg = EXYNOS_CLKREG(0x20B00),  .val = 0xffffffff, },   //  CMU_CDREX_SPARE0
	{ .reg = EXYNOS_CLKREG(0x20B04),  .val = 0xffffffff, },   //  CMU_CDREX_SPARE1
	{ .reg = EXYNOS_CLKREG(0x20B08),  .val = 0xffffffff, },   //  CMU_CDREX_SPARE2
	{ .reg = EXYNOS_CLKREG(0x20B0C),  .val = 0xffffffff, },   //  CMU_CDREX_SPARE3
	{ .reg = EXYNOS_CLKREG(0x20B10),  .val = 0xffffffff, },   //  CMU_CDREX_SPARE4
	{ .reg = EXYNOS_CLKREG(0x23FF0),  .val = 0xffffffff, },   //  CMU_CDREX_VERSION
	{ .reg = EXYNOS_CLKREG(0x28000),  .val = 0xffffffff, },   //  KPLL_LOCK
	{ .reg = EXYNOS_CLKREG(0x28100),  .val = 0xffffffff, },   //  KPLL_CON0
	{ .reg = EXYNOS_CLKREG(0x28104),  .val = 0xffffffff, },   //  KPLL_CON1
	{ .reg = EXYNOS_CLKREG(0x28200),  .val = 0xffffffff, },   //  CLK_SRC_KFC
	{ .reg = EXYNOS_CLKREG(0x28400),  .val = 0xffffffff, },   //  CLK_MUX_STAT_KFC
	{ .reg = EXYNOS_CLKREG(0x28500),  .val = 0xffffffff, },   //  CLK_DIV_KFC0
	{ .reg = EXYNOS_CLKREG(0x28600),  .val = 0xffffffff, },   //  CLK_DIV_STAT_KFC0
	{ .reg = EXYNOS_CLKREG(0x28700),  .val = 0xffffffff, },   //  CLK_GATE_BUS_CPU_KFC
	{ .reg = EXYNOS_CLKREG(0x28800),  .val = 0xffffffff, },   //  CLK_GATE_SCLK_CPU_KFC
	{ .reg = EXYNOS_CLKREG(0x28A00),  .val = 0xffffffff, },   //  CLKOUT_CMU_KFC
	{ .reg = EXYNOS_CLKREG(0x28A04),  .val = 0xffffffff, },   //  CLKOUT_CMU_KFC_DIV_STAT
	{ .reg = EXYNOS_CLKREG(0x29000),  .val = 0xffffffff, },   //  ARMCLK_STOPCTRL_KFC
	{ .reg = EXYNOS_CLKREG(0x29008),  .val = 0xffffffff, },   //  ARM_EMA_CTRL_KFC
	{ .reg = EXYNOS_CLKREG(0x2900C),  .val = 0xffffffff, },   //  ARM_EMA_STATUS_KFC
	{ .reg = EXYNOS_CLKREG(0x29020),  .val = 0xffffffff, },   //  PWR_CTRL_KFC
	{ .reg = EXYNOS_CLKREG(0x29024),  .val = 0xffffffff, },   //  PWR_CTRL2_KFC
	{ .reg = EXYNOS_CLKREG(0x29100),  .val = 0xffffffff, },   //  KPLL_CON0_L8
	{ .reg = EXYNOS_CLKREG(0x29104),  .val = 0xffffffff, },   //  KPLL_CON0_L7
	{ .reg = EXYNOS_CLKREG(0x29108),  .val = 0xffffffff, },   //  KPLL_CON0_L6
	{ .reg = EXYNOS_CLKREG(0x2910C),  .val = 0xffffffff, },   //  KPLL_CON0_L5
	{ .reg = EXYNOS_CLKREG(0x29110),  .val = 0xffffffff, },   //  KPLL_CON0_L4
	{ .reg = EXYNOS_CLKREG(0x29114),  .val = 0xffffffff, },   //  KPLL_CON0_L3
	{ .reg = EXYNOS_CLKREG(0x29118),  .val = 0xffffffff, },   //  KPLL_CON0_L2
	{ .reg = EXYNOS_CLKREG(0x2911C),  .val = 0xffffffff, },   //  KPLL_CON0_L1
	{ .reg = EXYNOS_CLKREG(0x29120),  .val = 0xffffffff, },   //  IEM_CONTROL_KFC
	{ .reg = EXYNOS_CLKREG(0x29200),  .val = 0xffffffff, },   //  KPLL_CON1_L8
	{ .reg = EXYNOS_CLKREG(0x29204),  .val = 0xffffffff, },   //  KPLL_CON1_L7
	{ .reg = EXYNOS_CLKREG(0x29208),  .val = 0xffffffff, },   //  KPLL_CON1_L6
	{ .reg = EXYNOS_CLKREG(0x2920C),  .val = 0xffffffff, },   //  KPLL_CON1_L5
	{ .reg = EXYNOS_CLKREG(0x29210),  .val = 0xffffffff, },   //  KPLL_CON1_L4
	{ .reg = EXYNOS_CLKREG(0x29214),  .val = 0xffffffff, },   //  KPLL_CON1_L3
	{ .reg = EXYNOS_CLKREG(0x29218),  .val = 0xffffffff, },   //  KPLL_CON1_L2
	{ .reg = EXYNOS_CLKREG(0x2921C),  .val = 0xffffffff, },   //  KPLL_CON1_L1
	{ .reg = EXYNOS_CLKREG(0x29300),  .val = 0xffffffff, },   //  CLKDIV_IEM_L8_KFC
	{ .reg = EXYNOS_CLKREG(0x29304),  .val = 0xffffffff, },   //  CLKDIV_IEM_L7_KFC
	{ .reg = EXYNOS_CLKREG(0x29308),  .val = 0xffffffff, },   //  CLKDIV_IEM_L6_KFC
	{ .reg = EXYNOS_CLKREG(0x2930C),  .val = 0xffffffff, },   //  CLKDIV_IEM_L5_KFC
	{ .reg = EXYNOS_CLKREG(0x29310),  .val = 0xffffffff, },   //  CLKDIV_IEM_L4_KFC
	{ .reg = EXYNOS_CLKREG(0x29314),  .val = 0xffffffff, },   //  CLKDIV_IEM_L3_KFC
	{ .reg = EXYNOS_CLKREG(0x29318),  .val = 0xffffffff, },   //  CLKDIV_IEM_L2_KFC
	{ .reg = EXYNOS_CLKREG(0x2931C),  .val = 0xffffffff, },   //  CLKDIV_IEM_L1_KFC
	{ .reg = EXYNOS_CLKREG(0x29400),  .val = 0xffffffff, },   //  L2_STATUS_KFC
	{ .reg = EXYNOS_CLKREG(0x29410),  .val = 0xffffffff, },   //  CPU_STATUS_KFC
	{ .reg = EXYNOS_CLKREG(0x29420),  .val = 0xffffffff, },   //  PTM_STATUS_KFC
	{ .reg = EXYNOS_CLKREG(0x2A000),  .val = 0xffffffff, },   //  CMU_KFC_SPARE0
	{ .reg = EXYNOS_CLKREG(0x2A004),  .val = 0xffffffff, },   //  CMU_KFC_SPARE1
	{ .reg = EXYNOS_CLKREG(0x2A008),  .val = 0xffffffff, },   //  CMU_KFC_SPARE2
	{ .reg = EXYNOS_CLKREG(0x2A00C),  .val = 0xffffffff, },   //  CMU_KFC_SPARE3
	{ .reg = EXYNOS_CLKREG(0x2A010),  .val = 0xffffffff, },   //  CMU_KFC_SPARE4
	{ .reg = EXYNOS_CLKREG(0x2BFF0),  .val = 0xffffffff, },   //  CMU_KFC_VERSION
};


struct sleep_save	pmu_dump_reg[] = {
	{ .reg = EXYNOS_PMUREG(0x0000),  .val = 0xffffffff, },	 //OM_STAT
	{ .reg = EXYNOS_PMUREG(0x001C),  .val = 0xffffffff, },	 //RTC_CLKO_SEL
	{ .reg = EXYNOS_PMUREG(0x0020),  .val = 0xffffffff, },	 //GNSS_RTC_OUT_CTRL
	{ .reg = EXYNOS_PMUREG(0x0200),  .val = 0xffffffff, },	 //CENTRAL_SEQ_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x0204),  .val = 0xffffffff, },	 //CENTRAL_SEQ_STATUS
	{ .reg = EXYNOS_PMUREG(0x0208),  .val = 0xffffffff, },	 //CENTRAL_SEQ_OPTION
	{ .reg = EXYNOS_PMUREG(0x0220),  .val = 0xffffffff, },	 //SEQ_TRANSITION0
	{ .reg = EXYNOS_PMUREG(0x0224),  .val = 0xffffffff, },	 //SEQ_TRANSITION1
	{ .reg = EXYNOS_PMUREG(0x0228),  .val = 0xffffffff, },	 //SEQ_TRANSITION2
	{ .reg = EXYNOS_PMUREG(0x022C),  .val = 0xffffffff, },	 //SEQ_TRANSITION3
	{ .reg = EXYNOS_PMUREG(0x0230),  .val = 0xffffffff, },	 //SEQ_TRANSITION4
	{ .reg = EXYNOS_PMUREG(0x0234),  .val = 0xffffffff, },	 //SEQ_TRANSITION5
	{ .reg = EXYNOS_PMUREG(0x0238),  .val = 0xffffffff, },	 //SEQ_TRANSITION6
	{ .reg = EXYNOS_PMUREG(0x023C),  .val = 0xffffffff, },	 //SEQ_TRANSITION7
	{ .reg = EXYNOS_PMUREG(0x0240),  .val = 0xffffffff, },	 //CENTRAL_SEQ_COREBLK_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x0244),  .val = 0xffffffff, },	 //CENTRAL_SEQ_COREBLK_STATUS
	{ .reg = EXYNOS_PMUREG(0x0248),  .val = 0xffffffff, },	 //CENTRAL_SEQ_COREBLK_OPTION
	{ .reg = EXYNOS_PMUREG(0x0260),  .val = 0xffffffff, },	 //SEQ_COREBLK_TRANSITION0
	{ .reg = EXYNOS_PMUREG(0x0264),  .val = 0xffffffff, },	 //SEQ_COREBLK_TRANSITION1
	{ .reg = EXYNOS_PMUREG(0x0268),  .val = 0xffffffff, },	 //SEQ_COREBLK_TRANSITION2
	{ .reg = EXYNOS_PMUREG(0x026C),  .val = 0xffffffff, },	 //SEQ_COREBLK_TRANSITION3
	{ .reg = EXYNOS_PMUREG(0x0270),  .val = 0xffffffff, },	 //SEQ_COREBLK_TRANSITION4
	{ .reg = EXYNOS_PMUREG(0x0274),  .val = 0xffffffff, },	 //SEQ_COREBLK_TRANSITION5
	{ .reg = EXYNOS_PMUREG(0x0278),  .val = 0xffffffff, },	 //SEQ_COREBLK_TRANSITION6
	{ .reg = EXYNOS_PMUREG(0x027C),  .val = 0xffffffff, },	 //SEQ_COREBLK_TRANSITION7
	{ .reg = EXYNOS_PMUREG(0x0400),  .val = 0xffffffff, },	 //SWRESET
	{ .reg = EXYNOS_PMUREG(0x0404),  .val = 0xffffffff, },	 //RST_STAT
	{ .reg = EXYNOS_PMUREG(0x0408),  .val = 0xffffffff, },	 //AUTOMATIC_WDT_RESET_DISABLE
	{ .reg = EXYNOS_PMUREG(0x040C),  .val = 0xffffffff, },	 //MASK_WDT_RESET_REQUEST
	{ .reg = EXYNOS_PMUREG(0x0500),  .val = 0xffffffff, },	 //RESET_SEQUENCER_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x0504),  .val = 0xffffffff, },	 //RESET_SEQUENCER_STATUS
	{ .reg = EXYNOS_PMUREG(0x0508),  .val = 0xffffffff, },	 //RESET_SEQUENCER_OPTION
	{ .reg = EXYNOS_PMUREG(0x0600),  .val = 0xffffffff, },	 //WAKEUP_STAT
	{ .reg = EXYNOS_PMUREG(0x0604),  .val = 0xffffffff, },	 //EINT_WAKEUP_MASK
	{ .reg = EXYNOS_PMUREG(0x0608),  .val = 0xffffffff, },	 //WAKEUP_MASK
	{ .reg = EXYNOS_PMUREG(0x060C),  .val = 0xffffffff, },	 //WAKEUP_INTERRUPT
	{ .reg = EXYNOS_PMUREG(0x0620),  .val = 0xffffffff, },	 //WAKEUP_STAT_COREBLK
	{ .reg = EXYNOS_PMUREG(0x0624),  .val = 0xffffffff, },	 //EINT_WAKEUP_MASK_COREBLK
	{ .reg = EXYNOS_PMUREG(0x0628),  .val = 0xffffffff, },	 //WAKEUP_MASK_COREBLK
	{ .reg = EXYNOS_PMUREG(0x062C),  .val = 0xffffffff, },	 //WAKEUP_INTERRUPT_COREBLK
	{ .reg = EXYNOS_PMUREG(0x0700),  .val = 0xffffffff, },	 //HDMI_PHY_CONTROL
	{ .reg = EXYNOS_PMUREG(0x0704),  .val = 0xffffffff, },	 //USBDEV_PHY_CONTROL
	{ .reg = EXYNOS_PMUREG(0x0708),  .val = 0xffffffff, },	 //USBDEV1_PHY_CONTROL
	{ .reg = EXYNOS_PMUREG(0x070C),  .val = 0xffffffff, },	 //USBHOST_PHY_CONTROL
	{ .reg = EXYNOS_PMUREG(0x0714),  .val = 0xffffffff, },	 //MIPI_PHY0_CONTROL
	{ .reg = EXYNOS_PMUREG(0x0718),  .val = 0xffffffff, },	 //MIPI_PHY1_CONTROL
	{ .reg = EXYNOS_PMUREG(0x071C),  .val = 0xffffffff, },	 //MIPI_PHY2_CONTROL
	{ .reg = EXYNOS_PMUREG(0x0720),  .val = 0xffffffff, },	 //ADC_PHY_CONTROL
	{ .reg = EXYNOS_PMUREG(0x0724),  .val = 0xffffffff, },	 //MTCADC_PHY_CONTROL
	{ .reg = EXYNOS_PMUREG(0x0728),  .val = 0xffffffff, },	 //DPTX_PHY_CONTROL
	{ .reg = EXYNOS_PMUREG(0x0800),  .val = 0xffffffff, },	 //INFORM0
	{ .reg = EXYNOS_PMUREG(0x0804),  .val = 0xffffffff, },	 //INFORM1
	{ .reg = EXYNOS_PMUREG(0x0808),  .val = 0xffffffff, },	 //INFORM2
	{ .reg = EXYNOS_PMUREG(0x080C),  .val = 0xffffffff, },	 //INFORM3
	{ .reg = EXYNOS_PMUREG(0x0810),  .val = 0xffffffff, },	 //SYSIP_DAT0
	{ .reg = EXYNOS_PMUREG(0x0814),  .val = 0xffffffff, },	 //SYSIP_DAT1
	{ .reg = EXYNOS_PMUREG(0x0818),  .val = 0xffffffff, },	 //SYSIP_DAT2
	{ .reg = EXYNOS_PMUREG(0x081C),  .val = 0xffffffff, },	 //SYSIP_DAT3
	{ .reg = EXYNOS_PMUREG(0x0900),  .val = 0xffffffff, },	 //PMU_SPARE0
	{ .reg = EXYNOS_PMUREG(0x0904),  .val = 0xffffffff, },	 //PMU_SPARE1
	{ .reg = EXYNOS_PMUREG(0x0908),  .val = 0xffffffff, },	 //PMU_SPARE2
	{ .reg = EXYNOS_PMUREG(0x090C),  .val = 0xffffffff, },	 //PMU_SPARE3
	{ .reg = EXYNOS_PMUREG(0x0980),  .val = 0xffffffff, },	 //IROM_DATA_REG0
	{ .reg = EXYNOS_PMUREG(0x0984),  .val = 0xffffffff, },	 //IROM_DATA_REG1
	{ .reg = EXYNOS_PMUREG(0x0988),  .val = 0xffffffff, },	 //IROM_DATA_REG2
//	{ .reg = EXYNOS_PMUREG(0x098C),  .val = 0xffffffff, },	 //IROM_DATA_REG3
	{ .reg = EXYNOS_PMUREG(0x1000),  .val = 0xffffffff, },	 //ARM_CORE0_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1010),  .val = 0xffffffff, },	 //ARM_CORE1_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1020),  .val = 0xffffffff, },	 //ARM_CORE2_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1030),  .val = 0xffffffff, },	 //ARM_CORE3_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1040),  .val = 0xffffffff, },	 //KFC_CORE0_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1050),  .val = 0xffffffff, },	 //KFC_CORE1_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1060),  .val = 0xffffffff, },	 //KFC_CORE2_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1070),  .val = 0xffffffff, },	 //KFC_CORE3_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1090),  .val = 0xffffffff, },	 //ISP_ARM_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x10A0),  .val = 0xffffffff, },	 //ARM_COMMON_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x10B0),  .val = 0xffffffff, },	 //KFC_COMMON_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x10C0),  .val = 0xffffffff, },	 //ARM_L2_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x10D0),  .val = 0xffffffff, },	 //KFC_L2_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1100),  .val = 0xffffffff, },	 //CMU_ACLKSTOP_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1104),  .val = 0xffffffff, },	 //CMU_SCLKSTOP_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x110C),  .val = 0xffffffff, },	 //CMU_RESET_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1120),  .val = 0xffffffff, },	 //CMU_ACLKSTOP_COREBLK_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1124),  .val = 0xffffffff, },	 //CMU_SCLKSTOP_COREBLK_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x112C),  .val = 0xffffffff, },	 //CMU_RESET_COREBLK_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1130),  .val = 0xffffffff, },	 //DRAM_FREQ_DOWN_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1134),  .val = 0xffffffff, },	 //DDRPHY_DLLOFF_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1138),  .val = 0xffffffff, },	 //DDRPHY_DLLLOCK_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1140),  .val = 0xffffffff, },	 //APLL_SYSCLK_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1144),  .val = 0xffffffff, },	 //MPLL_SYSCLK_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1148),  .val = 0xffffffff, },	 //VPLL_SYSCLK_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x114C),  .val = 0xffffffff, },	 //EPLL_SYSCLK_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1150),  .val = 0xffffffff, },	 //BPLL_SYSCLK_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1154),  .val = 0xffffffff, },	 //CPLL_SYSCLK_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1158),  .val = 0xffffffff, },	 //DPLL_SYSCLK_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x115C),  .val = 0xffffffff, },	 //IPLL_SYSCLK_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1160),  .val = 0xffffffff, },	 //KPLL_SYSCLK_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1164),  .val = 0xffffffff, },	 //MPLLUSER_SYSCLK_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1170),  .val = 0xffffffff, },	 //BPLLUSER_SYSCLK_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1180),  .val = 0xffffffff, },	 //TOP_BUS_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1184),  .val = 0xffffffff, },	 //TOP_RETENTION_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1188),  .val = 0xffffffff, },	 //TOP_PWR_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1190),  .val = 0xffffffff, },	 //TOP_BUS_COREBLK_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1194),  .val = 0xffffffff, },	 //TOP_RETENTION_COREBLK_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1198),  .val = 0xffffffff, },	 //TOP_PWR_COREBLK_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x11A0),  .val = 0xffffffff, },	 //LOGIC_RESET_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x11A4),  .val = 0xffffffff, },	 //OSCCLK_GATE_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x11B0),  .val = 0xffffffff, },	 //LOGIC_RESET_COREBLK_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x11B4),  .val = 0xffffffff, },	 //OSCCLK_GATE_COREBLK_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x11C0),  .val = 0xffffffff, },	 //OneNANDXL_MEM_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x11C8),  .val = 0xffffffff, },	 //G2D_MEM_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x11CC),  .val = 0xffffffff, },	 //USBDEV_MEM_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x11D0),  .val = 0xffffffff, },	 //USBDEV1_MEM_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x11D4),  .val = 0xffffffff, },	 //SDMMC_MEM_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x11D8),  .val = 0xffffffff, },	 //CSSYS_MEM_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x11DC),  .val = 0xffffffff, },	 //SECSS_MEM_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x11E0),  .val = 0xffffffff, },	 //ROTATOR_MEM_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x11E4),  .val = 0xffffffff, },	 //INTRAM_MEM_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x11E8),  .val = 0xffffffff, },	 //INTROM_MEM_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x11EC),  .val = 0xffffffff, },	 //JPEG_MEM_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x11F0),  .val = 0xffffffff, },	 //SFMC0_MEM_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x11F4),  .val = 0xffffffff, },	 //SFMC1_MEM_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x11F8),  .val = 0xffffffff, },	 //HSI_MEM_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x11FC),  .val = 0xffffffff, },	 //MCUIOP_MEM_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1200),  .val = 0xffffffff, },	 //PAD_RETENTION_DRAM_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1204),  .val = 0xffffffff, },	 //PAD_RETENTION_MAU_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1208),  .val = 0xffffffff, },	 //PAD_RETENTION_JTAG_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1210),  .val = 0xffffffff, },	 //PAD_RETENTION_GPIO_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1214),  .val = 0xffffffff, },	 //PAD_RETENTION_UART_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1218),  .val = 0xffffffff, },	 //PAD_RETENTION_MMC0_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x121C),  .val = 0xffffffff, },	 //PAD_RETENTION_MMC1_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1220),  .val = 0xffffffff, },	 //PAD_RETENTION_MMC2_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1224),  .val = 0xffffffff, },	 //PAD_RETENTION_HSI_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1228),  .val = 0xffffffff, },	 //PAD_RETENTION_EBIA_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x122C),  .val = 0xffffffff, },	 //PAD_RETENTION_EBIB_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1230),  .val = 0xffffffff, },	 //PAD_RETENTION_SPI_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1234),  .val = 0xffffffff, },	 //PAD_RETENTION_GPIO_COREBLK_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1240),  .val = 0xffffffff, },	 //PAD_ISOLATION_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1250),  .val = 0xffffffff, },	 //PAD_ISOLATION_COREBLK_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1260),  .val = 0xffffffff, },	 //PAD_ALV_SEL_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1280),  .val = 0xffffffff, },	 //XUSBXTI_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1284),  .val = 0xffffffff, },	 //XXTI_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x12C0),  .val = 0xffffffff, },	 //EXT_REGULATOR_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1300),  .val = 0xffffffff, },	 //GPIO_MODE_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1320),  .val = 0xffffffff, },	 //GPIO_MODE_COREBLK_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1340),  .val = 0xffffffff, },	 //GPIO_MODE_MAU_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1344),  .val = 0xffffffff, },	 //TOP_ASB_RESET_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1348),  .val = 0xffffffff, },	 //TOP_ASB_ISOLATION_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1400),  .val = 0xffffffff, },	 //GSCL_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1404),  .val = 0xffffffff, },	 //ISP_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1408),  .val = 0xffffffff, },	 //MFC_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x140C),  .val = 0xffffffff, },	 //G3D_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1410),  .val = 0xffffffff, },	 //DISP0_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1414),  .val = 0xffffffff, },	 //DISP1_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1418),  .val = 0xffffffff, },	 //MAU_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1480),  .val = 0xffffffff, },	 //CMU_CLKSTOP_GSCL_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1484),  .val = 0xffffffff, },	 //CMU_CLKSTOP_ISP_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1488),  .val = 0xffffffff, },	 //CMU_CLKSTOP_MFC_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x148C),  .val = 0xffffffff, },	 //CMU_CLKSTOP_G3D_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1490),  .val = 0xffffffff, },	 //CMU_CLKSTOP_DISP0_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1494),  .val = 0xffffffff, },	 //CMU_CLKSTOP_DISP1_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1498),  .val = 0xffffffff, },	 //CMU_CLKSTOP_MAU_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x14C0),  .val = 0xffffffff, },	 //CMU_SYSCLK_GSCL_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x14C4),  .val = 0xffffffff, },	 //CMU_SYSCLK_ISP_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x14C8),  .val = 0xffffffff, },	 //CMU_SYSCLK_MFC_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x14CC),  .val = 0xffffffff, },	 //CMU_SYSCLK_G3D_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x14D0),  .val = 0xffffffff, },	 //CMU_SYSCLK_DISP0_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x14D4),  .val = 0xffffffff, },	 //CMU_SYSCLK_DISP1_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x14D8),  .val = 0xffffffff, },	 //CMU_SYSCLK_MAU_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1580),  .val = 0xffffffff, },	 //CMU_RESET_GSCL_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1584),  .val = 0xffffffff, },	 //CMU_RESET_ISP_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1588),  .val = 0xffffffff, },	 //CMU_RESET_MFC_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x158C),  .val = 0xffffffff, },	 //CMU_RESET_G3D_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1590),  .val = 0xffffffff, },	 //CMU_RESET_DISP0_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1594),  .val = 0xffffffff, },	 //CMU_RESET_DISP1_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x1598),  .val = 0xffffffff, },	 //CMU_RESET_MAU_SYS_PWR_REG
	{ .reg = EXYNOS_PMUREG(0x2000),  .val = 0xffffffff, },	 //ARM_CORE0_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2004),  .val = 0xffffffff, },	 //ARM_CORE0_STATUS
	{ .reg = EXYNOS_PMUREG(0x2008),  .val = 0xffffffff, },	 //ARM_CORE0_OPTION
	{ .reg = EXYNOS_PMUREG(0x2020),  .val = 0xffffffff, },	 //DIS_IRQ_ARM_CORE0_LOCAL_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2024),  .val = 0xffffffff, },	 //DIS_IRQ_ARM_CORE0_LOCAL_STATUS
	{ .reg = EXYNOS_PMUREG(0x2028),  .val = 0xffffffff, },	 //DIS_IRQ_ARM_CORE0_LOCAL_OPTION
	{ .reg = EXYNOS_PMUREG(0x2040),  .val = 0xffffffff, },	 //DIS_IRQ_ARM_CORE0_CENTRAL_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2044),  .val = 0xffffffff, },	 //DIS_IRQ_ARM_CORE0_CENTRAL_STATUS
	{ .reg = EXYNOS_PMUREG(0x2048),  .val = 0xffffffff, },	 //DIS_IRQ_ARM_CORE0_CENTRAL_OPTION
	{ .reg = EXYNOS_PMUREG(0x2080),  .val = 0xffffffff, },	 //ARM_CORE1_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2084),  .val = 0xffffffff, },	 //ARM_CORE1_STATUS
	{ .reg = EXYNOS_PMUREG(0x2088),  .val = 0xffffffff, },	 //ARM_CORE1_OPTION
	{ .reg = EXYNOS_PMUREG(0x20A0),  .val = 0xffffffff, },	 //DIS_IRQ_ARM_CORE1_LOCAL_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x20A4),  .val = 0xffffffff, },	 //DIS_IRQ_ARM_CORE1_LOCAL_STATUS
	{ .reg = EXYNOS_PMUREG(0x20A8),  .val = 0xffffffff, },	 //DIS_IRQ_ARM_CORE1_LOCAL_OPTION
	{ .reg = EXYNOS_PMUREG(0x20C0),  .val = 0xffffffff, },	 //DIS_IRQ_ARM_CORE1_CENTRAL_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x20C4),  .val = 0xffffffff, },	 //DIS_IRQ_ARM_CORE1_CENTRAL_STATUS
	{ .reg = EXYNOS_PMUREG(0x20C8),  .val = 0xffffffff, },	 //DIS_IRQ_ARM_CORE1_CENTRAL_OPTION
	{ .reg = EXYNOS_PMUREG(0x2100),  .val = 0xffffffff, },	 //ARM_CORE2_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2104),  .val = 0xffffffff, },	 //ARM_CORE2_STATUS
	{ .reg = EXYNOS_PMUREG(0x2108),  .val = 0xffffffff, },	 //ARM_CORE2_OPTION
	{ .reg = EXYNOS_PMUREG(0x2120),  .val = 0xffffffff, },	 //DIS_IRQ_ARM_CORE2_LOCAL_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2124),  .val = 0xffffffff, },	 //DIS_IRQ_ARM_CORE2_LOCAL_STATUS
	{ .reg = EXYNOS_PMUREG(0x2128),  .val = 0xffffffff, },	 //DIS_IRQ_ARM_CORE2_LOCAL_OPTION
	{ .reg = EXYNOS_PMUREG(0x2140),  .val = 0xffffffff, },	 //DIS_IRQ_ARM_CORE2_CENTRAL_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2144),  .val = 0xffffffff, },	 //DIS_IRQ_ARM_CORE2_CENTRAL_STATUS
	{ .reg = EXYNOS_PMUREG(0x2148),  .val = 0xffffffff, },	 //DIS_IRQ_ARM_CORE2_CENTRAL_OPTION
	{ .reg = EXYNOS_PMUREG(0x2180),  .val = 0xffffffff, },	 //ARM_CORE3_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2184),  .val = 0xffffffff, },	 //ARM_CORE3_STATUS
	{ .reg = EXYNOS_PMUREG(0x2188),  .val = 0xffffffff, },	 //ARM_CORE3_OPTION
	{ .reg = EXYNOS_PMUREG(0x21A0),  .val = 0xffffffff, },	 //DIS_IRQ_ARM_CORE3_LOCAL_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x21A4),  .val = 0xffffffff, },	 //DIS_IRQ_ARM_CORE3_LOCAL_STATUS
	{ .reg = EXYNOS_PMUREG(0x21A8),  .val = 0xffffffff, },	 //DIS_IRQ_ARM_CORE3_LOCAL_OPTION
	{ .reg = EXYNOS_PMUREG(0x21C0),  .val = 0xffffffff, },	 //DIS_IRQ_ARM_CORE3_CENTRAL_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x21C4),  .val = 0xffffffff, },	 //DIS_IRQ_ARM_CORE3_CENTRAL_STATUS
	{ .reg = EXYNOS_PMUREG(0x21C8),  .val = 0xffffffff, },	 //DIS_IRQ_ARM_CORE3_CENTRAL_OPTION
	{ .reg = EXYNOS_PMUREG(0x2200),  .val = 0xffffffff, },	 //KFC_CORE0_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2204),  .val = 0xffffffff, },	 //KFC_CORE0_STATUS
	{ .reg = EXYNOS_PMUREG(0x2208),  .val = 0xffffffff, },	 //KFC_CORE0_OPTION
	{ .reg = EXYNOS_PMUREG(0x2220),  .val = 0xffffffff, },	 //DIS_IRQ_KFC_CORE0_LOCAL_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2224),  .val = 0xffffffff, },	 //DIS_IRQ_KFC_CORE0_LOCAL_STATUS
	{ .reg = EXYNOS_PMUREG(0x2228),  .val = 0xffffffff, },	 //DIS_IRQ_KFC_CORE0_LOCAL_OPTION
	{ .reg = EXYNOS_PMUREG(0x2240),  .val = 0xffffffff, },	 //DIS_IRQ_KFC_CORE0_CENTRAL_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2244),  .val = 0xffffffff, },	 //DIS_IRQ_KFC_CORE0_CENTRAL_STATUS
	{ .reg = EXYNOS_PMUREG(0x2248),  .val = 0xffffffff, },	 //DIS_IRQ_KFC_CORE0_CENTRAL_OPTION
	{ .reg = EXYNOS_PMUREG(0x2280),  .val = 0xffffffff, },	 //KFC_CORE1_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2284),  .val = 0xffffffff, },	 //KFC_CORE1_STATUS
	{ .reg = EXYNOS_PMUREG(0x2288),  .val = 0xffffffff, },	 //KFC_CORE1_OPTION
	{ .reg = EXYNOS_PMUREG(0x22A0),  .val = 0xffffffff, },	 //DIS_IRQ_KFC_CORE1_LOCAL_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x22A4),  .val = 0xffffffff, },	 //DIS_IRQ_KFC_CORE1_LOCAL_STATUS
	{ .reg = EXYNOS_PMUREG(0x22A8),  .val = 0xffffffff, },	 //DIS_IRQ_KFC_CORE1_LOCAL_OPTION
	{ .reg = EXYNOS_PMUREG(0x22C0),  .val = 0xffffffff, },	 //DIS_IRQ_KFC_CORE1_CENTRAL_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x22C4),  .val = 0xffffffff, },	 //DIS_IRQ_KFC_CORE1_CENTRAL_STATUS
	{ .reg = EXYNOS_PMUREG(0x22C8),  .val = 0xffffffff, },	 //DIS_IRQ_KFC_CORE1_CENTRAL_OPTION
	{ .reg = EXYNOS_PMUREG(0x2300),  .val = 0xffffffff, },	 //KFC_CORE2_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2304),  .val = 0xffffffff, },	 //KFC_CORE2_STATUS
	{ .reg = EXYNOS_PMUREG(0x2308),  .val = 0xffffffff, },	 //KFC_CORE2_OPTION
	{ .reg = EXYNOS_PMUREG(0x2320),  .val = 0xffffffff, },	 //DIS_IRQ_KFC_CORE2_LOCAL_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2324),  .val = 0xffffffff, },	 //DIS_IRQ_KFC_CORE2_LOCAL_STATUS
	{ .reg = EXYNOS_PMUREG(0x2328),  .val = 0xffffffff, },	 //DIS_IRQ_KFC_CORE2_LOCAL_OPTION
	{ .reg = EXYNOS_PMUREG(0x2340),  .val = 0xffffffff, },	 //DIS_IRQ_KFC_CORE2_CENTRAL_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2344),  .val = 0xffffffff, },	 //DIS_IRQ_KFC_CORE2_CENTRAL_STATUS
	{ .reg = EXYNOS_PMUREG(0x2348),  .val = 0xffffffff, },	 //DIS_IRQ_KFC_CORE2_CENTRAL_OPTION
	{ .reg = EXYNOS_PMUREG(0x2380),  .val = 0xffffffff, },	 //KFC_CORE3_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2384),  .val = 0xffffffff, },	 //KFC_CORE3_STATUS
	{ .reg = EXYNOS_PMUREG(0x2388),  .val = 0xffffffff, },	 //KFC_CORE3_OPTION
	{ .reg = EXYNOS_PMUREG(0x23A0),  .val = 0xffffffff, },	 //DIS_IRQ_KFC_CORE3_LOCAL_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x23A4),  .val = 0xffffffff, },	 //DIS_IRQ_KFC_CORE3_LOCAL_STATUS
	{ .reg = EXYNOS_PMUREG(0x23A8),  .val = 0xffffffff, },	 //DIS_IRQ_KFC_CORE3_LOCAL_OPTION
	{ .reg = EXYNOS_PMUREG(0x23C0),  .val = 0xffffffff, },	 //DIS_IRQ_KFC_CORE3_CENTRAL_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x23C4),  .val = 0xffffffff, },	 //DIS_IRQ_KFC_CORE3_CENTRAL_STATUS
	{ .reg = EXYNOS_PMUREG(0x23C8),  .val = 0xffffffff, },	 //DIS_IRQ_KFC_CORE3_CENTRAL_OPTION
	{ .reg = EXYNOS_PMUREG(0x2480),  .val = 0xffffffff, },	 //ISP_ARM_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2484),  .val = 0xffffffff, },	 //ISP_ARM_STATUS
	{ .reg = EXYNOS_PMUREG(0x2488),  .val = 0xffffffff, },	 //ISP_ARM_OPTION
	{ .reg = EXYNOS_PMUREG(0x24A0),  .val = 0xffffffff, },	 //DIS_IRQ_ISP_ARM_LOCAL_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x24A4),  .val = 0xffffffff, },	 //DIS_IRQ_ISP_ARM_LOCAL_STATUS
	{ .reg = EXYNOS_PMUREG(0x24A8),  .val = 0xffffffff, },	 //DIS_IRQ_ISP_ARM_LOCAL_OPTION
	{ .reg = EXYNOS_PMUREG(0x24C0),  .val = 0xffffffff, },	 //DIS_IRQ_ISP_ARM_CENTRAL_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x24C4),  .val = 0xffffffff, },	 //DIS_IRQ_ISP_ARM_CENTRAL_STATUS
	{ .reg = EXYNOS_PMUREG(0x24C8),  .val = 0xffffffff, },	 //DIS_IRQ_ISP_ARM_CENTRAL_OPTION
	{ .reg = EXYNOS_PMUREG(0x2500),  .val = 0xffffffff, },	 //ARM_COMMON_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2504),  .val = 0xffffffff, },	 //ARM_COMMON_STATUS
	{ .reg = EXYNOS_PMUREG(0x2508),  .val = 0xffffffff, },	 //ARM_COMMON_OPTION
	{ .reg = EXYNOS_PMUREG(0x2580),  .val = 0xffffffff, },	 //KFC_COMMON_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2584),  .val = 0xffffffff, },	 //KFC_COMMON_STATUS
	{ .reg = EXYNOS_PMUREG(0x2588),  .val = 0xffffffff, },	 //KFC_COMMON_OPTION
	{ .reg = EXYNOS_PMUREG(0x2600),  .val = 0xffffffff, },	 //ARM_L2_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2604),  .val = 0xffffffff, },	 //ARM_L2_STATUS
	{ .reg = EXYNOS_PMUREG(0x2608),  .val = 0xffffffff, },	 //ARM_L2_OPTION
	{ .reg = EXYNOS_PMUREG(0x2680),  .val = 0xffffffff, },	 //KFC_L2_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2684),  .val = 0xffffffff, },	 //KFC_L2_STATUS
	{ .reg = EXYNOS_PMUREG(0x2688),  .val = 0xffffffff, },	 //KFC_L2_OPTION
	{ .reg = EXYNOS_PMUREG(0x2800),  .val = 0xffffffff, },	 //CMU_ACLKSTOP_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2804),  .val = 0xffffffff, },	 //CMU_ACLKSTOP_STATUS
	{ .reg = EXYNOS_PMUREG(0x2808),  .val = 0xffffffff, },	 //CMU_ACLKSTOP_OPTION
	{ .reg = EXYNOS_PMUREG(0x2820),  .val = 0xffffffff, },	 //CMU_SCLKSTOP_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2824),  .val = 0xffffffff, },	 //CMU_SCLKSTOP_STATUS
	{ .reg = EXYNOS_PMUREG(0x2828),  .val = 0xffffffff, },	 //CMU_SCLKSTOP_OPTION
	{ .reg = EXYNOS_PMUREG(0x2860),  .val = 0xffffffff, },	 //CMU_RESET_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2864),  .val = 0xffffffff, },	 //CMU_RESET_STATUS
	{ .reg = EXYNOS_PMUREG(0x2868),  .val = 0xffffffff, },	 //CMU_RESET_OPTION
	{ .reg = EXYNOS_PMUREG(0x2900),  .val = 0xffffffff, },	 //CMU_ACLKSTOP_COREBLK_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2904),  .val = 0xffffffff, },	 //CMU_ACLKSTOP_COREBLK_STATUS
	{ .reg = EXYNOS_PMUREG(0x2908),  .val = 0xffffffff, },	 //CMU_ACLKSTOP_COREBLK_OPTION
	{ .reg = EXYNOS_PMUREG(0x2920),  .val = 0xffffffff, },	 //CMU_SCLKSTOP_COREBLK_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2924),  .val = 0xffffffff, },	 //CMU_SCLKSTOP_COREBLK_STATUS
	{ .reg = EXYNOS_PMUREG(0x2928),  .val = 0xffffffff, },	 //CMU_SCLKSTOP_COREBLK_OPTION
	{ .reg = EXYNOS_PMUREG(0x2960),  .val = 0xffffffff, },	 //CMU_RESET_COREBLK_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2964),  .val = 0xffffffff, },	 //CMU_RESET_COREBLK_STATUS
	{ .reg = EXYNOS_PMUREG(0x2968),  .val = 0xffffffff, },	 //CMU_RESET_COREBLK_OPTION
	{ .reg = EXYNOS_PMUREG(0x2980),  .val = 0xffffffff, },	 //DRAM_FREQ_DOWN_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2984),  .val = 0xffffffff, },	 //DRAM_FREQ_DOWN_STATUS
	{ .reg = EXYNOS_PMUREG(0x2988),  .val = 0xffffffff, },	 //DRAM_FREQ_DOWN_OPTION
	{ .reg = EXYNOS_PMUREG(0x29A0),  .val = 0xffffffff, },	 //DDRPHY_DLLOFF_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x29A4),  .val = 0xffffffff, },	 //DDRPHY_DLLOFF_STATUS
	{ .reg = EXYNOS_PMUREG(0x29A8),  .val = 0xffffffff, },	 //DDRPHY_DLLOFF_OPTION
	{ .reg = EXYNOS_PMUREG(0x29C0),  .val = 0xffffffff, },	 //DDRPHY_DLLLOCK_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x29C4),  .val = 0xffffffff, },	 //DDRPHY_DLLLOCK_STATUS
	{ .reg = EXYNOS_PMUREG(0x29C8),  .val = 0xffffffff, },	 //DDRPHY_DLLLOCK_OPTION
	{ .reg = EXYNOS_PMUREG(0x2A00),  .val = 0xffffffff, },	 //APLL_SYSCLK_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2A04),  .val = 0xffffffff, },	 //APLL_SYSCLK_STATUS
	{ .reg = EXYNOS_PMUREG(0x2A08),  .val = 0xffffffff, },	 //APLL_SYSCLK_OPTION
	{ .reg = EXYNOS_PMUREG(0x2A24),  .val = 0xffffffff, },	 //MPLL_SYSCLK_STATUS
	{ .reg = EXYNOS_PMUREG(0x2A28),  .val = 0xffffffff, },	 //MPLL_SYSCLK_OPTION
	{ .reg = EXYNOS_PMUREG(0x2A40),  .val = 0xffffffff, },	 //VPLL_SYSCLK_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2A44),  .val = 0xffffffff, },	 //VPLL_SYSCLK_STATUS
	{ .reg = EXYNOS_PMUREG(0x2A48),  .val = 0xffffffff, },	 //VPLL_SYSCLK_OPTION
	{ .reg = EXYNOS_PMUREG(0x2A60),  .val = 0xffffffff, },	 //EPLL_SYSCLK_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2A64),  .val = 0xffffffff, },	 //EPLL_SYSCLK_STATUS
	{ .reg = EXYNOS_PMUREG(0x2A68),  .val = 0xffffffff, },	 //EPLL_SYSCLK_OPTION
	{ .reg = EXYNOS_PMUREG(0x2A80),  .val = 0xffffffff, },	 //BPLL_SYSCLK_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2A84),  .val = 0xffffffff, },	 //BPLL_SYSCLK_STATUS
	{ .reg = EXYNOS_PMUREG(0x2A88),  .val = 0xffffffff, },	 //BPLL_SYSCLK_OPTION
	{ .reg = EXYNOS_PMUREG(0x2AA0),  .val = 0xffffffff, },	 //CPLL_SYSCLK_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2AA4),  .val = 0xffffffff, },	 //CPLL_SYSCLK_STATUS
	{ .reg = EXYNOS_PMUREG(0x2AA8),  .val = 0xffffffff, },	 //CPLL_SYSCLK_OPTION
	{ .reg = EXYNOS_PMUREG(0x2AC0),  .val = 0xffffffff, },	 //DPLL_SYSCLK_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2AC4),  .val = 0xffffffff, },	 //DPLL_SYSCLK_STATUS
	{ .reg = EXYNOS_PMUREG(0x2AC8),  .val = 0xffffffff, },	 //DPLL_SYSCLK_OPTION
	{ .reg = EXYNOS_PMUREG(0x2AE0),  .val = 0xffffffff, },	 //IPLL_SYSCLK_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2AE4),  .val = 0xffffffff, },	 //IPLL_SYSCLK_STATUS
	{ .reg = EXYNOS_PMUREG(0x2AE8),  .val = 0xffffffff, },	 //IPLL_SYSCLK_OPTION
	{ .reg = EXYNOS_PMUREG(0x2B00),  .val = 0xffffffff, },	 //KPLL_SYSCLK_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2B04),  .val = 0xffffffff, },	 //KPLL_SYSCLK_STATUS
	{ .reg = EXYNOS_PMUREG(0x2B08),  .val = 0xffffffff, },	 //KPLL_SYSCLK_OPTION
	{ .reg = EXYNOS_PMUREG(0x2B20),  .val = 0xffffffff, },	 //MPLLUSER_SYSCLK_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2B24),  .val = 0xffffffff, },	 //MPLLUSER_SYSCLK_STATUS
	{ .reg = EXYNOS_PMUREG(0x2B28),  .val = 0xffffffff, },	 //MPLLUSER_SYSCLK_OPTION
	{ .reg = EXYNOS_PMUREG(0x2B80),  .val = 0xffffffff, },	 //BPLLUSER_SYSCLK_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2B84),  .val = 0xffffffff, },	 //BPLLUSER_SYSCLK_STATUS
	{ .reg = EXYNOS_PMUREG(0x2B88),  .val = 0xffffffff, },	 //BPLLUSER_SYSCLK_OPTION
	{ .reg = EXYNOS_PMUREG(0x2C00),  .val = 0xffffffff, },	 //TOP_BUS_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2C04),  .val = 0xffffffff, },	 //TOP_BUS_STATUS
	{ .reg = EXYNOS_PMUREG(0x2C08),  .val = 0xffffffff, },	 //TOP_BUS_OPTION
	{ .reg = EXYNOS_PMUREG(0x2C20),  .val = 0xffffffff, },	 //TOP_RETENTION_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2C24),  .val = 0xffffffff, },	 //TOP_RETENTION_STATUS
	{ .reg = EXYNOS_PMUREG(0x2C28),  .val = 0xffffffff, },	 //TOP_RETENTION_OPTION
	{ .reg = EXYNOS_PMUREG(0x2C40),  .val = 0xffffffff, },	 //TOP_PWR_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2C44),  .val = 0xffffffff, },	 //TOP_PWR_STATUS
	{ .reg = EXYNOS_PMUREG(0x2C48),  .val = 0xffffffff, },	 //TOP_PWR_OPTION
	{ .reg = EXYNOS_PMUREG(0x2C80),  .val = 0xffffffff, },	 //TOP_BUS_COREBLK_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2C84),  .val = 0xffffffff, },	 //TOP_BUS_COREBLK_STATUS
	{ .reg = EXYNOS_PMUREG(0x2C88),  .val = 0xffffffff, },	 //TOP_BUS_COREBLK_OPTION
	{ .reg = EXYNOS_PMUREG(0x2CA0),  .val = 0xffffffff, },	 //TOP_RETENTION_COREBLK_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2CA4),  .val = 0xffffffff, },	 //TOP_RETENTION_COREBLK_STATUS
	{ .reg = EXYNOS_PMUREG(0x2CA8),  .val = 0xffffffff, },	 //TOP_RETENTION_COREBLK_OPTION
	{ .reg = EXYNOS_PMUREG(0x2CC0),  .val = 0xffffffff, },	 //TOP_PWR_COREBLK_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2CC4),  .val = 0xffffffff, },	 //TOP_PWR_COREBLK_STATUS
	{ .reg = EXYNOS_PMUREG(0x2CC8),  .val = 0xffffffff, },	 //TOP_PWR_COREBLK_OPTION
	{ .reg = EXYNOS_PMUREG(0x2D00),  .val = 0xffffffff, },	 //LOGIC_RESET_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2D04),  .val = 0xffffffff, },	 //LOGIC_RESET_STATUS
	{ .reg = EXYNOS_PMUREG(0x2D08),  .val = 0xffffffff, },	 //LOGIC_RESET_OPTION
	{ .reg = EXYNOS_PMUREG(0x2D20),  .val = 0xffffffff, },	 //OSCCLK_GATE_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2D24),  .val = 0xffffffff, },	 //OSCCLK_GATE_STATUS
	{ .reg = EXYNOS_PMUREG(0x2D28),  .val = 0xffffffff, },	 //OSCCLK_GATE_OPTION
	{ .reg = EXYNOS_PMUREG(0x2D80),  .val = 0xffffffff, },	 //LOGIC_RESET_COREBLK_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2D84),  .val = 0xffffffff, },	 //LOGIC_RESET_COREBLK_STATUS
	{ .reg = EXYNOS_PMUREG(0x2D88),  .val = 0xffffffff, },	 //LOGIC_RESET_COREBLK_OPTION
	{ .reg = EXYNOS_PMUREG(0x2DA0),  .val = 0xffffffff, },	 //OSCCLK_GATE_COREBLK_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2DA4),  .val = 0xffffffff, },	 //OSCCLK_GATE_COREBLK_STATUS
	{ .reg = EXYNOS_PMUREG(0x2DA8),  .val = 0xffffffff, },	 //OSCCLK_GATE_COREBLK_OPTION
	{ .reg = EXYNOS_PMUREG(0x2E00),  .val = 0xffffffff, },	 //OneNANDXL_MEM_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2E04),  .val = 0xffffffff, },	 //OneNANDXL_MEM_STATUS
	{ .reg = EXYNOS_PMUREG(0x2E08),  .val = 0xffffffff, },	 //OneNANDXL_MEM_OPTION
	{ .reg = EXYNOS_PMUREG(0x2E40),  .val = 0xffffffff, },	 //G2D_MEM_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2E44),  .val = 0xffffffff, },	 //G2D_MEM_STATUS
	{ .reg = EXYNOS_PMUREG(0x2E48),  .val = 0xffffffff, },	 //G2D_MEM_OPTION
	{ .reg = EXYNOS_PMUREG(0x2E60),  .val = 0xffffffff, },	 //USBDEV_MEM_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2E64),  .val = 0xffffffff, },	 //USBDEV_MEM_STATUS
	{ .reg = EXYNOS_PMUREG(0x2E68),  .val = 0xffffffff, },	 //USBDEV_MEM_OPTION
	{ .reg = EXYNOS_PMUREG(0x2E80),  .val = 0xffffffff, },	 //USBDEV1_MEM_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2E84),  .val = 0xffffffff, },	 //USBDEV1_MEM_STATUS
	{ .reg = EXYNOS_PMUREG(0x2E88),  .val = 0xffffffff, },	 //USBDEV1_MEM_OPTION
	{ .reg = EXYNOS_PMUREG(0x2EA0),  .val = 0xffffffff, },	 //SDMMC_MEM_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2EA4),  .val = 0xffffffff, },	 //SDMMC_MEM_STATUS
	{ .reg = EXYNOS_PMUREG(0x2EA8),  .val = 0xffffffff, },	 //SDMMC_MEM_OPTION
	{ .reg = EXYNOS_PMUREG(0x2EC0),  .val = 0xffffffff, },	 //CSSYS_MEM_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2EC4),  .val = 0xffffffff, },	 //CSSYS_MEM_STATUS
	{ .reg = EXYNOS_PMUREG(0x2EC8),  .val = 0xffffffff, },	 //CSSYS_MEM_OPTION
	{ .reg = EXYNOS_PMUREG(0x2EE0),  .val = 0xffffffff, },	 //SECSS_MEM_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2EE4),  .val = 0xffffffff, },	 //SECSS_MEM_STATUS
	{ .reg = EXYNOS_PMUREG(0x2EE8),  .val = 0xffffffff, },	 //SECSS_MEM_OPTION
	{ .reg = EXYNOS_PMUREG(0x2F00),  .val = 0xffffffff, },	 //ROTATOR_MEM_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2F04),  .val = 0xffffffff, },	 //ROTATOR_MEM_STATUS
	{ .reg = EXYNOS_PMUREG(0x2F08),  .val = 0xffffffff, },	 //ROTATOR_MEM_OPTION
	{ .reg = EXYNOS_PMUREG(0x2F20),  .val = 0xffffffff, },	 //INTRAM_MEM_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2F24),  .val = 0xffffffff, },	 //INTRAM_MEM_STATUS
	{ .reg = EXYNOS_PMUREG(0x2F28),  .val = 0xffffffff, },	 //INTRAM_MEM_OPTION
	{ .reg = EXYNOS_PMUREG(0x2F40),  .val = 0xffffffff, },	 //INTROM_MEM_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2F44),  .val = 0xffffffff, },	 //INTROM_MEM_STATUS
	{ .reg = EXYNOS_PMUREG(0x2F48),  .val = 0xffffffff, },	 //INTROM_MEM_OPTION
	{ .reg = EXYNOS_PMUREG(0x2F60),  .val = 0xffffffff, },	 //JPEG_MEM_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2F64),  .val = 0xffffffff, },	 //JPEG_MEM_STATUS
	{ .reg = EXYNOS_PMUREG(0x2F68),  .val = 0xffffffff, },	 //JPEG_MEM_OPTION
	{ .reg = EXYNOS_PMUREG(0x2F80),  .val = 0xffffffff, },	 //SFMC0_MEM_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2F84),  .val = 0xffffffff, },	 //SFMC0_MEM_STATUS
	{ .reg = EXYNOS_PMUREG(0x2F88),  .val = 0xffffffff, },	 //SFMC0_MEM_OPTION
	{ .reg = EXYNOS_PMUREG(0x2FA0),  .val = 0xffffffff, },	 //SFMC1_MEM_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2FA4),  .val = 0xffffffff, },	 //SFMC1_MEM_STATUS
	{ .reg = EXYNOS_PMUREG(0x2FA8),  .val = 0xffffffff, },	 //SFMC1_MEM_OPTION
	{ .reg = EXYNOS_PMUREG(0x2FC0),  .val = 0xffffffff, },	 //HSI_MEM_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2FC4),  .val = 0xffffffff, },	 //HSI_MEM_STATUS
	{ .reg = EXYNOS_PMUREG(0x2FC8),  .val = 0xffffffff, },	 //HSI_MEM_OPTION
	{ .reg = EXYNOS_PMUREG(0x2FE0),  .val = 0xffffffff, },	 //MCUIOP_MEM_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x2FE4),  .val = 0xffffffff, },	 //MCUIOP_MEM_STATUS
	{ .reg = EXYNOS_PMUREG(0x2FE8),  .val = 0xffffffff, },	 //MCUIOP_MEM_OPTION
	{ .reg = EXYNOS_PMUREG(0x3000),  .val = 0xffffffff, },	 //PAD_RETENTION_DRAM_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x3004),  .val = 0xffffffff, },	 //PAD_RETENTION_DRAM_STATUS
	{ .reg = EXYNOS_PMUREG(0x3008),  .val = 0xffffffff, },	 //PAD_RETENTION_DRAM_OPTION
	{ .reg = EXYNOS_PMUREG(0x3020),  .val = 0xffffffff, },	 //PAD_RETENTION_MAU_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x3024),  .val = 0xffffffff, },	 //PAD_RETENTION_MAU_STATUS
	{ .reg = EXYNOS_PMUREG(0x3028),  .val = 0xffffffff, },	 //PAD_RETENTION_MAU_OPTION
	{ .reg = EXYNOS_PMUREG(0x3040),  .val = 0xffffffff, },	 //PAD_RETENTION_JTAG_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x3044),  .val = 0xffffffff, },	 //PAD_RETENTION_JTAG_STATUS
	{ .reg = EXYNOS_PMUREG(0x3048),  .val = 0xffffffff, },	 //PAD_RETENTION_JTAG_OPTION
	{ .reg = EXYNOS_PMUREG(0x30C0),  .val = 0xffffffff, },	 //PAD_RETENTION_GPIO_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x30C4),  .val = 0xffffffff, },	 //PAD_RETENTION_GPIO_STATUS
	{ .reg = EXYNOS_PMUREG(0x30C8),  .val = 0xffffffff, },	 //PAD_RETENTION_GPIO_OPTION
	{ .reg = EXYNOS_PMUREG(0x30E0),  .val = 0xffffffff, },	 //PAD_RETENTION_UART_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x30E4),  .val = 0xffffffff, },	 //PAD_RETENTION_UART_STATUS
	{ .reg = EXYNOS_PMUREG(0x30E8),  .val = 0xffffffff, },	 //PAD_RETENTION_UART_OPTION
	{ .reg = EXYNOS_PMUREG(0x3100),  .val = 0xffffffff, },	 //PAD_RETENTION_MMC0_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x3104),  .val = 0xffffffff, },	 //PAD_RETENTION_MMC0_STATUS
	{ .reg = EXYNOS_PMUREG(0x3108),  .val = 0xffffffff, },	 //PAD_RETENTION_MMC0_OPTION
	{ .reg = EXYNOS_PMUREG(0x3120),  .val = 0xffffffff, },	 //PAD_RETENTION_MMC1_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x3124),  .val = 0xffffffff, },	 //PAD_RETENTION_MMC1_STATUS
	{ .reg = EXYNOS_PMUREG(0x3128),  .val = 0xffffffff, },	 //PAD_RETENTION_MMC1_OPTION
	{ .reg = EXYNOS_PMUREG(0x3140),  .val = 0xffffffff, },	 //PAD_RETENTION_MMC2_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x3144),  .val = 0xffffffff, },	 //PAD_RETENTION_MMC2_STATUS
	{ .reg = EXYNOS_PMUREG(0x3148),  .val = 0xffffffff, },	 //PAD_RETENTION_MMC2_OPTION
	{ .reg = EXYNOS_PMUREG(0x3160),  .val = 0xffffffff, },	 //PAD_RETENTION_HSI_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x3164),  .val = 0xffffffff, },	 //PAD_RETENTION_HSI_STATUS
	{ .reg = EXYNOS_PMUREG(0x3168),  .val = 0xffffffff, },	 //PAD_RETENTION_HSI_OPTION
	{ .reg = EXYNOS_PMUREG(0x3180),  .val = 0xffffffff, },	 //PAD_RETENTION_EBIA_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x3184),  .val = 0xffffffff, },	 //PAD_RETENTION_EBIA_STATUS
	{ .reg = EXYNOS_PMUREG(0x3188),  .val = 0xffffffff, },	 //PAD_RETENTION_EBIA_OPTION
	{ .reg = EXYNOS_PMUREG(0x31A0),  .val = 0xffffffff, },	 //PAD_RETENTION_EBIB_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x31A4),  .val = 0xffffffff, },	 //PAD_RETENTION_EBIB_STATUS
	{ .reg = EXYNOS_PMUREG(0x31A8),  .val = 0xffffffff, },	 //PAD_RETENTION_EBIB_OPTION
	{ .reg = EXYNOS_PMUREG(0x31C0),  .val = 0xffffffff, },	 //PAD_RETENTION_SPI_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x31C4),  .val = 0xffffffff, },	 //PAD_RETENTION_SPI_STATUS
	{ .reg = EXYNOS_PMUREG(0x31C8),  .val = 0xffffffff, },	 //PAD_RETENTION_SPI_OPTION
	{ .reg = EXYNOS_PMUREG(0x31E0),  .val = 0xffffffff, },	 //PAD_RETENTION_GPIO_COREBLK_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x31E4),  .val = 0xffffffff, },	 //PAD_RETENTION_GPIO_COREBLK_STATUS
	{ .reg = EXYNOS_PMUREG(0x31E8),  .val = 0xffffffff, },	 //PAD_RETENTION_GPIO_COREBLK_OPTION
	{ .reg = EXYNOS_PMUREG(0x3200),  .val = 0xffffffff, },	 //PAD_ISOLATION_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x3204),  .val = 0xffffffff, },	 //PAD_ISOLATION_STATUS
	{ .reg = EXYNOS_PMUREG(0x3208),  .val = 0xffffffff, },	 //PAD_ISOLATION_OPTION
	{ .reg = EXYNOS_PMUREG(0x3280),  .val = 0xffffffff, },	 //PAD_ISOLATION_COREBLK_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x3284),  .val = 0xffffffff, },	 //PAD_ISOLATION_COREBLK_STATUS
	{ .reg = EXYNOS_PMUREG(0x3288),  .val = 0xffffffff, },	 //PAD_ISOLATION_COREBLK_OPTION
	{ .reg = EXYNOS_PMUREG(0x3300),  .val = 0xffffffff, },	 //PAD_ALV_SEL_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x3304),  .val = 0xffffffff, },	 //PAD_ALV_SEL_STATUS
	{ .reg = EXYNOS_PMUREG(0x3308),  .val = 0xffffffff, },	 //PAD_ALV_SEL_OPTION0
	{ .reg = EXYNOS_PMUREG(0x330C),  .val = 0xffffffff, },	 //PS_HOLD_CONTROL
	{ .reg = EXYNOS_PMUREG(0x3400),  .val = 0xffffffff, },	 //XUSBXTI_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x3404),  .val = 0xffffffff, },	 //XUSBXTI_STATUS
	{ .reg = EXYNOS_PMUREG(0x3408),  .val = 0xffffffff, },	 //XUSBXTI_OPTION
	{ .reg = EXYNOS_PMUREG(0x341C),  .val = 0xffffffff, },	 //XUSBXTI_DURATION3
	{ .reg = EXYNOS_PMUREG(0x3420),  .val = 0xffffffff, },	 //XXTI_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x3424),  .val = 0xffffffff, },	 //XXTI_STATUS
	{ .reg = EXYNOS_PMUREG(0x3428),  .val = 0xffffffff, },	 //XXTI_OPTION
	{ .reg = EXYNOS_PMUREG(0x343C),  .val = 0xffffffff, },	 //XXTI_DURATION3
	{ .reg = EXYNOS_PMUREG(0x3600),  .val = 0xffffffff, },	 //EXT_REGULATOR_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x3604),  .val = 0xffffffff, },	 //EXT_REGULATOR_STATUS
	{ .reg = EXYNOS_PMUREG(0x3608),  .val = 0xffffffff, },	 //EXT_REGULATOR_OPTION
	{ .reg = EXYNOS_PMUREG(0x361C),  .val = 0xffffffff, },	 //EXT_REGULATOR_DURATION3
	{ .reg = EXYNOS_PMUREG(0x3800),  .val = 0xffffffff, },	 //GPIO_MODE_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x3804),  .val = 0xffffffff, },	 //GPIO_MODE_STATUS
	{ .reg = EXYNOS_PMUREG(0x3808),  .val = 0xffffffff, },	 //GPIO_MODE_OPTION
	{ .reg = EXYNOS_PMUREG(0x3900),  .val = 0xffffffff, },	 //GPIO_MODE_COREBLK_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x3904),  .val = 0xffffffff, },	 //GPIO_MODE_COREBLK_STATUS
	{ .reg = EXYNOS_PMUREG(0x3908),  .val = 0xffffffff, },	 //GPIO_MODE_COREBLK_OPTION
	{ .reg = EXYNOS_PMUREG(0x39E0),  .val = 0xffffffff, },	 //GPIO_MODE_MAU_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x39E4),  .val = 0xffffffff, },	 //GPIO_MODE_MAU_STATUS
	{ .reg = EXYNOS_PMUREG(0x39E8),  .val = 0xffffffff, },	 //GPIO_MODE_MAU_OPTION
	{ .reg = EXYNOS_PMUREG(0x3A00),  .val = 0xffffffff, },	 //TOP_ASB_RESET_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x3A04),  .val = 0xffffffff, },	 //TOP_ASB_RESET_STATUS
	{ .reg = EXYNOS_PMUREG(0x3A08),  .val = 0xffffffff, },	 //TOP_ASB_RESET_OPTION
	{ .reg = EXYNOS_PMUREG(0x3A20),  .val = 0xffffffff, },	 //TOP_ASB_ISOLATION_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x3A24),  .val = 0xffffffff, },	 //TOP_ASB_ISOLATION_STATUS
	{ .reg = EXYNOS_PMUREG(0x3A28),  .val = 0xffffffff, },	 //TOP_ASB_ISOLATION_OPTION
	{ .reg = EXYNOS_PMUREG(0x4000),  .val = 0xffffffff, },	 //GSCL_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x4004),  .val = 0xffffffff, },	 //GSCL_STATUS
	{ .reg = EXYNOS_PMUREG(0x4008),  .val = 0xffffffff, },	 //GSCL_OPTION
	{ .reg = EXYNOS_PMUREG(0x4020),  .val = 0xffffffff, },	 //ISP_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x4024),  .val = 0xffffffff, },	 //ISP_STATUS
	{ .reg = EXYNOS_PMUREG(0x4028),  .val = 0xffffffff, },	 //ISP_OPTION
	{ .reg = EXYNOS_PMUREG(0x4060),  .val = 0xffffffff, },	 //MFC_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x4064),  .val = 0xffffffff, },	 //MFC_STATUS
	{ .reg = EXYNOS_PMUREG(0x4068),  .val = 0xffffffff, },	 //MFC_OPTION
	{ .reg = EXYNOS_PMUREG(0x4080),  .val = 0xffffffff, },	 //G3D_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x4084),  .val = 0xffffffff, },	 //G3D_STATUS
	{ .reg = EXYNOS_PMUREG(0x4088),  .val = 0xffffffff, },	 //G3D_OPTION
	{ .reg = EXYNOS_PMUREG(0x40A0),  .val = 0xffffffff, },	 //DISP0_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x40A4),  .val = 0xffffffff, },	 //DISP0_STATUS
	{ .reg = EXYNOS_PMUREG(0x40A8),  .val = 0xffffffff, },	 //DISP0_OPTION
	{ .reg = EXYNOS_PMUREG(0x40C0),  .val = 0xffffffff, },	 //DISP1_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x40C4),  .val = 0xffffffff, },	 //DISP1_STATUS
	{ .reg = EXYNOS_PMUREG(0x40C8),  .val = 0xffffffff, },	 //DISP1_OPTION
	{ .reg = EXYNOS_PMUREG(0x40E0),  .val = 0xffffffff, },	 //MAU_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x40E4),  .val = 0xffffffff, },	 //MAU_STATUS
	{ .reg = EXYNOS_PMUREG(0x40E8),  .val = 0xffffffff, },	 //MAU_OPTION
	{ .reg = EXYNOS_PMUREG(0x4400),  .val = 0xffffffff, },	 //CMU_CLKSTOP_GSCL_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x4404),  .val = 0xffffffff, },	 //CMU_CLKSTOP_GSCL_STATUS
	{ .reg = EXYNOS_PMUREG(0x4408),  .val = 0xffffffff, },	 //CMU_CLKSTOP_GSCL_OPTION
	{ .reg = EXYNOS_PMUREG(0x4420),  .val = 0xffffffff, },	 //CMU_CLKSTOP_ISP_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x4424),  .val = 0xffffffff, },	 //CMU_CLKSTOP_ISP_STATUS
	{ .reg = EXYNOS_PMUREG(0x4428),  .val = 0xffffffff, },	 //CMU_CLKSTOP_ISP_OPTION
	{ .reg = EXYNOS_PMUREG(0x4460),  .val = 0xffffffff, },	 //CMU_CLKSTOP_MFC_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x4464),  .val = 0xffffffff, },	 //CMU_CLKSTOP_MFC_STATUS
	{ .reg = EXYNOS_PMUREG(0x4468),  .val = 0xffffffff, },	 //CMU_CLKSTOP_MFC_OPTION
	{ .reg = EXYNOS_PMUREG(0x4480),  .val = 0xffffffff, },	 //CMU_CLKSTOP_G3D_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x4484),  .val = 0xffffffff, },	 //CMU_CLKSTOP_G3D_STATUS
	{ .reg = EXYNOS_PMUREG(0x4488),  .val = 0xffffffff, },	 //CMU_CLKSTOP_G3D_OPTION
	{ .reg = EXYNOS_PMUREG(0x44A0),  .val = 0xffffffff, },	 //CMU_CLKSTOP_DISP0_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x44A4),  .val = 0xffffffff, },	 //CMU_CLKSTOP_DISP0_STATUS
	{ .reg = EXYNOS_PMUREG(0x44A8),  .val = 0xffffffff, },	 //CMU_CLKSTOP_DISP0_OPTION
	{ .reg = EXYNOS_PMUREG(0x44C0),  .val = 0xffffffff, },	 //CMU_CLKSTOP_DISP1_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x44C4),  .val = 0xffffffff, },	 //CMU_CLKSTOP_DISP1_STATUS
	{ .reg = EXYNOS_PMUREG(0x44C8),  .val = 0xffffffff, },	 //CMU_CLKSTOP_DISP1_OPTION
	{ .reg = EXYNOS_PMUREG(0x44E0),  .val = 0xffffffff, },	 //CMU_CLKSTOP_MAU_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x44E4),  .val = 0xffffffff, },	 //CMU_CLKSTOP_MAU_STATUS
	{ .reg = EXYNOS_PMUREG(0x44E8),  .val = 0xffffffff, },	 //CMU_CLKSTOP_MAU_OPTION
	{ .reg = EXYNOS_PMUREG(0x4600),  .val = 0xffffffff, },	 //CMU_SYSCLK_GSCL_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x4604),  .val = 0xffffffff, },	 //CMU_SYSCLK_GSCL_STATUS
	{ .reg = EXYNOS_PMUREG(0x4608),  .val = 0xffffffff, },	 //CMU_SYSCLK_GSCL_OPTION
	{ .reg = EXYNOS_PMUREG(0x4624),  .val = 0xffffffff, },	 //CMU_SYSCLK_ISP_STATUS
	{ .reg = EXYNOS_PMUREG(0x4628),  .val = 0xffffffff, },	 //CMU_SYSCLK_ISP_OPTION
	{ .reg = EXYNOS_PMUREG(0x4664),  .val = 0xffffffff, },	 //CMU_SYSCLK_MFC_STATUS
	{ .reg = EXYNOS_PMUREG(0x4668),  .val = 0xffffffff, },	 //CMU_SYSCLK_MFC_OPTION
	{ .reg = EXYNOS_PMUREG(0x4684),  .val = 0xffffffff, },	 //CMU_SYSCLK_G3D_STATUS
	{ .reg = EXYNOS_PMUREG(0x4688),  .val = 0xffffffff, },	 //CMU_SYSCLK_G3D_OPTION
	{ .reg = EXYNOS_PMUREG(0x46A4),  .val = 0xffffffff, },	 //CMU_SYSCLK_DISP0_STATUS
	{ .reg = EXYNOS_PMUREG(0x46A8),  .val = 0xffffffff, },	 //CMU_SYSCLK_DISP0_OPTION
	{ .reg = EXYNOS_PMUREG(0x46C4),  .val = 0xffffffff, },	 //CMU_SYSCLK_DISP1_STATUS
	{ .reg = EXYNOS_PMUREG(0x46C8),  .val = 0xffffffff, },	 //CMU_SYSCLK_DISP1_OPTION
	{ .reg = EXYNOS_PMUREG(0x46E4),  .val = 0xffffffff, },	 //CMU_SYSCLK_MAU_STATUS
	{ .reg = EXYNOS_PMUREG(0x46E8),  .val = 0xffffffff, },	 //CMU_SYSCLK_MAU_OPTION
	{ .reg = EXYNOS_PMUREG(0x4C00),  .val = 0xffffffff, },	 //CMU_RESET_GSCL_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x4C04),  .val = 0xffffffff, },	 //CMU_RESET_GSCL_STATUS
	{ .reg = EXYNOS_PMUREG(0x4C08),  .val = 0xffffffff, },	 //CMU_RESET_GSCL_OPTION
	{ .reg = EXYNOS_PMUREG(0x4C20),  .val = 0xffffffff, },	 //CMU_RESET_ISP_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x4C24),  .val = 0xffffffff, },	 //CMU_RESET_ISP_STATUS
	{ .reg = EXYNOS_PMUREG(0x4C28),  .val = 0xffffffff, },	 //CMU_RESET_ISP_OPTION
	{ .reg = EXYNOS_PMUREG(0x4C60),  .val = 0xffffffff, },	 //CMU_RESET_MFC_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x4C64),  .val = 0xffffffff, },	 //CMU_RESET_MFC_STATUS
	{ .reg = EXYNOS_PMUREG(0x4C68),  .val = 0xffffffff, },	 //CMU_RESET_MFC_OPTION
	{ .reg = EXYNOS_PMUREG(0x4C80),  .val = 0xffffffff, },	 //CMU_RESET_G3D_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x4C84),  .val = 0xffffffff, },	 //CMU_RESET_G3D_STATUS
	{ .reg = EXYNOS_PMUREG(0x4C88),  .val = 0xffffffff, },	 //CMU_RESET_G3D_OPTION
	{ .reg = EXYNOS_PMUREG(0x4CA0),  .val = 0xffffffff, },	 //CMU_RESET_DISP0_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x4CA4),  .val = 0xffffffff, },	 //CMU_RESET_DISP0_STATUS
	{ .reg = EXYNOS_PMUREG(0x4CA8),  .val = 0xffffffff, },	 //CMU_RESET_DISP0_OPTION
	{ .reg = EXYNOS_PMUREG(0x4CC0),  .val = 0xffffffff, },	//CMU_RESET_DISP1_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x4CC4),  .val = 0xffffffff, },	//CMU_RESET_DISP1_STATUS
	{ .reg = EXYNOS_PMUREG(0x4CC8),  .val = 0xffffffff, },	//CMU_RESET_DISP1_OPTION
	{ .reg = EXYNOS_PMUREG(0x4CE0),  .val = 0xffffffff, },	//CMU_RESET_MAU_CONFIGURATION
	{ .reg = EXYNOS_PMUREG(0x4CE4),  .val = 0xffffffff, },	//CMU_RESET_MAU_STATUS
	{ .reg = EXYNOS_PMUREG(0x4CE8),  .val = 0xffffffff, },	//CMU_RESET_MAU_OPTION
	{ .reg = EXYNOS_PMUREG(0x4D00),  .val = 0xffffffff, },	//VERSION_INFO
	{ .reg = EXYNOS_PMUREG(0x4D04),  .val = 0xffffffff, },	//I2S_BYPASS
};


struct debugging_info {
	unsigned int magic_value_1;
	unsigned int magic_value_2;
	unsigned int debugging_trace;
	unsigned int* pmu_address;
	unsigned int pmu_register_cnt;
	unsigned int* cmu_address;
	unsigned int cmu_register_cnt;
	unsigned int debugging_trace_in_blmon;
	unsigned int debugging_trace_in_tzsw;
};

/* For Cortex-A9 Diagnostic and Power control register */
static unsigned int save_arm_register[2];

static void exynos_clkgate_ctrl(bool on)
{
	unsigned int tmp;

	tmp = __raw_readl(EXYNOS5_CLKGATE_IP_GSCL0);
	tmp = on ? (tmp | EXYNOS5410_CLKGATE_GSCALER0_3) :
			(tmp & ~EXYNOS5410_CLKGATE_GSCALER0_3);

	__raw_writel(tmp, EXYNOS5_CLKGATE_IP_GSCL0);
}

static int read_mpidr(void)
{
	unsigned int id;
	asm volatile ("mrc\tp15, 0, %0, c0, c0, 5" : "=r" (id));
	return id;
}

#define MEIZU_MEM_DEBUG_TRACE
#ifdef MEIZU_MEM_DEBUG_TRACE
volatile struct debugging_info debugging_value = {0x12345678,0x87654321,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
void direct_access_to_debugging_info(unsigned int temp)
{
	static int first_time = 1;

	if(first_time == 1) {
		debugging_value.pmu_address = pmu_dump_reg;
		debugging_value.pmu_register_cnt = ARRAY_SIZE(pmu_dump_reg);
		debugging_value.cmu_address = cmu_dump_reg;
		debugging_value.cmu_register_cnt = ARRAY_SIZE(cmu_dump_reg);
		first_time = 0;
	}

	__raw_writel(&debugging_value,EXYNOS_INFORM2);

	debugging_value.debugging_trace = temp;
	flush_cache_all();
}

void store_pmu_cmu(struct sleep_save *ptr, int count)
{
	for (; count > 0; count--, ptr++) {
		ptr->val =  __raw_readl(ptr->reg);
	}
	flush_cache_all();
}
#else
void direct_access_to_debugging_info(unsigned int temp) {};
void store_pmu_cmu(struct sleep_save *ptr, int count) {};
#endif

extern void s3c_rtc_tick_wakeup_test(void);
extern void meizu_wakeup_debug_set_val(int func, int val);

static int exynos_cpu_suspend(unsigned long arg)
{
	unsigned int cluster_id = (read_mpidr() >> 8) & 0xf;
	unsigned int i, tmp, cpu_offset = ((cluster_id == 0) ? 0 : 4);
	int value = 0, loops = 0;

	direct_access_to_debugging_info(1);
#ifdef CONFIG_CACHE_L2X0
	outer_flush_all();
#endif
	/* flush cache back to ram */
	flush_cache_all();

	direct_access_to_debugging_info(0x05000005);
	/* RTC debug point: 5 */
	s3c_pm_rtc_debug_set(0x15);

	local_flush_tlb_all();

	direct_access_to_debugging_info(0x06000006);
	/* RTC debug point: 6 */
	s3c_pm_rtc_debug_set(0x16);

	if (soc_is_exynos5410()) {
		exynos_lpi_mask_ctrl(true);

		direct_access_to_debugging_info(0x07000007);
		/* RTC debug point: 7 */
		s3c_pm_rtc_debug_set(0x17);
		exynos_set_dummy_state(true);
	} else
		exynos_reset_assert_ctrl(false);

	direct_access_to_debugging_info(0x08000008);
	/* RTC debug point: 8 */
	s3c_pm_rtc_debug_set(0x18);

#ifdef CONFIG_ARM_TRUSTZONE
	exynos_smc(SMC_CMD_REG, SMC_REG_ID_SFR_W(0x02020028), value, 0);
#endif

	direct_access_to_debugging_info(0x09000009);
	/* RTC debug point: 9 */
	s3c_pm_rtc_debug_set(0x19);

	if (soc_is_exynos5410()) {
		for (i = 0; i < NR_CPUS; i++) {
			if (i == 0)
				continue;

			__raw_writel(0x3, EXYNOS_ARM_CORE_CONFIGURATION(cpu_offset + i));

			/* Wait until changing core status during 5ms */
			loops = msecs_to_loops(5);
			do {
				if (--loops == 0)
					BUG();
				tmp = __raw_readl(EXYNOS_ARM_CORE_STATUS(cpu_offset + i));
			} while ((tmp & 0x3) != 0x3);
		}
	}

	direct_access_to_debugging_info(0x0a00000a);
	/* RTC debug point: 10 */
	s3c_pm_rtc_debug_set(0x1A);

	s3c_pm_rtc_debug_sleep();

	store_pmu_cmu(pmu_dump_reg,ARRAY_SIZE(pmu_dump_reg));
	store_pmu_cmu(cmu_dump_reg,ARRAY_SIZE(cmu_dump_reg));

#ifdef CONFIG_ARM_TRUSTZONE
	meizu_wakeup_debug_set_val(0x1, 0);
	s3c_rtc_tick_wakeup_test();/*tick wake up test*/
	exynos_smc(SMC_CMD_SLEEP, 0, 0, 0);

	local_flush_tlb_all();
#else
	/* issue the standby signal into the pm unit. */
	cpu_do_idle();
#endif
	meizu_wakeup_debug_set_val(0xF, 3);
	direct_access_to_debugging_info(0x0b00000b);

	pr_debug("sleep resumed to originator?");

	if (soc_is_exynos5410()) {
		exynos_lpi_mask_ctrl(false);

		for (i = 0; i < NR_CPUS; i++) {
			if (i == 0)
				continue;

			__raw_writel(0x0, EXYNOS_ARM_CORE_CONFIGURATION(cpu_offset + i));

			/* Wait until changing core status during 5ms */
			loops = msecs_to_loops(5);
			do {
				if (--loops == 0)
					BUG();
				tmp = __raw_readl(EXYNOS_ARM_CORE_STATUS(cpu_offset + i));
			} while (tmp & 0x3);
		}
	}

	direct_access_to_debugging_info(0x0c00000c);
		/* flush cache back to ram */
	return 1; /* abort suspend */
}

static void exynos_pm_prepare(void)
{
	unsigned int tmp;

	if (soc_is_exynos5250()) {
		/* Decides whether to use retention capability */
		tmp = __raw_readl(EXYNOS5_ARM_L2_OPTION);
		tmp &= ~EXYNOS5_USE_RETENTION;
		__raw_writel(tmp, EXYNOS5_ARM_L2_OPTION);
	}

	if (!(soc_is_exynos5250() || soc_is_exynos5410())) {
		s3c_pm_do_save(exynos4_epll_save, ARRAY_SIZE(exynos4_epll_save));
		s3c_pm_do_save(exynos4_vpll_save, ARRAY_SIZE(exynos4_vpll_save));
	}

	/* Set value of power down register for sleep mode */
	exynos_sys_powerdown_conf(SYS_SLEEP);
	__raw_writel(EXYNOS_CHECK_SLEEP, REG_INFORM1);

	/* ensure at least INFORM0 has the resume address */
	__raw_writel(virt_to_phys(s3c_cpu_resume), REG_INFORM0);

	/*
	 * Before enter central sequence mode,
	 * clock src register have to set.
	 */
	if (!(soc_is_exynos5250() || soc_is_exynos5410()))
		s3c_pm_do_restore_core(exynos4_set_clksrc,
				ARRAY_SIZE(exynos4_set_clksrc));

	if (soc_is_exynos4210())
		s3c_pm_do_restore_core(exynos4210_set_clksrc, ARRAY_SIZE(exynos4210_set_clksrc));

	if (soc_is_exynos5250() || soc_is_exynos5410())
		s3c_pm_do_restore_core(exynos5_set_clksrc, ARRAY_SIZE(exynos5_set_clksrc));

	if (soc_is_exynos5410()) {
		s3c_pm_do_restore_core(exynos5410_set_clksrc, ARRAY_SIZE(exynos5410_set_clksrc));
		exynos_clkgate_ctrl(true);
	}
}

static int exynos_pm_add(struct device *dev, struct subsys_interface *sif)
{
	pm_cpu_prep = exynos_pm_prepare;
	pm_cpu_sleep = exynos_cpu_suspend;

	return 0;
}

static unsigned long pll_base_rate;

static void exynos4_restore_pll(void)
{
	unsigned long pll_con, locktime, lockcnt;
	unsigned long pll_in_rate;
	unsigned int p_div, epll_wait = 0, vpll_wait = 0;

	if (pll_base_rate == 0)
		return;

	pll_in_rate = pll_base_rate;

	/* EPLL */
	pll_con = exynos4_epll_save[0].val;

	if (pll_con & (1 << 31)) {
		pll_con &= (PLL46XX_PDIV_MASK << PLL46XX_PDIV_SHIFT);
		p_div = (pll_con >> PLL46XX_PDIV_SHIFT);

		pll_in_rate /= 1000000;

		locktime = (3000 / pll_in_rate) * p_div;
		lockcnt = locktime * 10000 / (10000 / pll_in_rate);

		__raw_writel(lockcnt, EXYNOS4_EPLL_LOCK);

		s3c_pm_do_restore_core(exynos4_epll_save,
					ARRAY_SIZE(exynos4_epll_save));
		epll_wait = 1;
	}

	pll_in_rate = pll_base_rate;

	/* VPLL */
	pll_con = exynos4_vpll_save[0].val;

	if (pll_con & (1 << 31)) {
		pll_in_rate /= 1000000;
		/* 750us */
		locktime = 750;
		lockcnt = locktime * 10000 / (10000 / pll_in_rate);

		__raw_writel(lockcnt, EXYNOS4_VPLL_LOCK);

		s3c_pm_do_restore_core(exynos4_vpll_save,
					ARRAY_SIZE(exynos4_vpll_save));
		vpll_wait = 1;
	}

	/* Wait PLL locking */

	do {
		if (epll_wait) {
			pll_con = __raw_readl(EXYNOS4_EPLL_CON0);
			if (pll_con & (1 << EXYNOS4_EPLLCON0_LOCKED_SHIFT))
				epll_wait = 0;
		}

		if (vpll_wait) {
			pll_con = __raw_readl(EXYNOS4_VPLL_CON0);
			if (pll_con & (1 << EXYNOS4_VPLLCON0_LOCKED_SHIFT))
				vpll_wait = 0;
		}
	} while (epll_wait || vpll_wait);
}

void exynos4_scu_enable(void __iomem *scu_base)
{
	u32 scu_ctrl;

#ifdef CONFIG_ARM_ERRATA_764369
	/* Cortex-A9 only */
	if ((read_cpuid(CPUID_ID) & 0xff0ffff0) == 0x410fc090) {
		scu_ctrl = __raw_readl(scu_base + 0x30);
		if (!(scu_ctrl & 1))
			__raw_writel(scu_ctrl | 0x1, scu_base + 0x30);
	}
#endif

	scu_ctrl = __raw_readl(scu_base);
	/* already enabled? */
	if (scu_ctrl & 1)
		return;

	if (soc_is_exynos4412() && (samsung_rev() >= EXYNOS4412_REV_1_0))
		scu_ctrl |= (1<<3);

	scu_ctrl |= 1;
	__raw_writel(scu_ctrl, scu_base);

	/*
	 * Ensure that the data accessed by CPU0 before the SCU was
	 * initialised is visible to the other CPUs.
	 */
	flush_cache_all();
}

static struct subsys_interface exynos4_pm_interface = {
	.name		= "exynos_pm",
	.subsys		= &exynos4_subsys,
	.add_dev	= exynos_pm_add,
};

static struct subsys_interface exynos5_pm_interface = {
	.name		= "exynos_pm",
	.subsys		= &exynos5_subsys,
	.add_dev	= exynos_pm_add,
};

static __init int exynos_pm_drvinit(void)
{
	struct clk *pll_base;

	s3c_pm_init();

	if (!(soc_is_exynos5250() || soc_is_exynos5410())) {
		pll_base = clk_get(NULL, "xtal");

		if (!IS_ERR(pll_base)) {
			pll_base_rate = clk_get_rate(pll_base);
			clk_put(pll_base);
		}
	}
	if (soc_is_exynos5250() || soc_is_exynos5410())
		return subsys_interface_register(&exynos5_pm_interface);
	else
		return subsys_interface_register(&exynos4_pm_interface);
}
arch_initcall(exynos_pm_drvinit);

#ifdef CONFIG_WAKEUP_REASON
static void exynos_show_wakeup_reason_eint(void)
{
	int bit;
	int reg_eintstart;
	long unsigned int ext_int_pend;
	unsigned long eint_wakeup_mask;
	bool found = 0;
	extern void __iomem *exynos_eint_base;

	eint_wakeup_mask = __raw_readl(EXYNOS_EINT_WAKEUP_MASK);

	for (reg_eintstart = 0; reg_eintstart <= 31; reg_eintstart += 8) {
		ext_int_pend =
			__raw_readl(EINT_PEND(exynos_eint_base,
					      IRQ_EINT(reg_eintstart)));

		for_each_set_bit(bit, &ext_int_pend, 8) {
			int irq = IRQ_EINT(reg_eintstart) + bit;
			struct irq_desc *desc = irq_to_desc(irq);

			if (eint_wakeup_mask & (1 << (reg_eintstart + bit)))
				continue;

			if (desc && desc->action && desc->action->name)
				pr_info("Resume caused by IRQ %d, %s\n", irq,
					desc->action->name);
			else
				pr_info("Resume caused by IRQ %d\n", irq);

			desc->wakeup_flag = 1;
			desc->wakeup_count++;

			found = 1;
		}
	}

	if (!found)
		pr_info("Resume caused by unknown EINT\n");
}

void exynos_show_wakeup_reason(void)
{
	unsigned long wakeup_stat;

	wakeup_stat = __raw_readl(EXYNOS_WAKEUP_STAT);

	if (wakeup_stat & EXYNOS_WAKEUP_STAT_RTC_ALARM)
		pr_info("Resume caused by RTC alarm\n");
	else if (wakeup_stat & EXYNOS_WAKEUP_STAT_EINT)
		exynos_show_wakeup_reason_eint();
	else
		pr_info("Resume caused by wakeup_stat=0x%08lx\n",
			wakeup_stat);
}

#else
#define exynos_show_wakeup_reason()	do { } while (0)
#endif

static int exynos_pm_suspend(void)
{
	unsigned long tmp;
	unsigned int cluster_id;

	s3c_pm_do_save(exynos_core_save, ARRAY_SIZE(exynos_core_save));

	if (!(soc_is_exynos4210() || soc_is_exynos5410()))
		exynos_reset_assert_ctrl(false);

#ifdef CONFIG_CPU_IDLE
	if (soc_is_exynos5410())
		exynos_disable_idle_clock_down(KFC);
#endif

	if (soc_is_exynos5410()) {
		cluster_id = (read_mpidr() >> 8) & 0xf;
		if (!cluster_id)
			__raw_writel(0x000F00F0, EXYNOS_CENTRAL_SEQ_OPTION);
		else
			__raw_writel(0x00F00F00, EXYNOS_CENTRAL_SEQ_OPTION);
	} else if (!soc_is_exynos5250()) {
		tmp = (EXYNOS4_USE_STANDBY_WFI0 | EXYNOS4_USE_STANDBY_WFE0);
		__raw_writel(tmp, EXYNOS_CENTRAL_SEQ_OPTION);

		/* Save Power control register */
		asm ("mrc p15, 0, %0, c15, c0, 0"
		     : "=r" (tmp) : : "cc");
		save_arm_register[0] = tmp;

		/* Save Diagnostic register */
		asm ("mrc p15, 0, %0, c15, c0, 1"
		     : "=r" (tmp) : : "cc");
		save_arm_register[1] = tmp;
	}

	/* Setting Central Sequence Register for power down mode */
	tmp = __raw_readl(EXYNOS_CENTRAL_SEQ_CONFIGURATION);
	tmp &= ~EXYNOS_CENTRAL_LOWPWR_CFG;
	__raw_writel(tmp, EXYNOS_CENTRAL_SEQ_CONFIGURATION);

	return 0;
}

static void exynos_pm_resume(void)
{
	unsigned long tmp;
#ifdef CONFIG_ARM_TRUSTZONE
	unsigned long p_reg, d_reg;
#endif

#ifdef CONFIG_EXYNOS5_CLUSTER_POWER_CONTROL
	unsigned int cluster_id = !((read_mpidr() >> 8) & 0xf);
#endif

	if (soc_is_exynos5410())
		__raw_writel(EXYNOS5410_USE_STANDBY_WFI_ALL,
			EXYNOS_CENTRAL_SEQ_OPTION);
	else
		exynos_reset_assert_ctrl(true);

	/*
	 * If PMU failed while entering sleep mode, WFI will be
	 * ignored by PMU and then exiting cpu_do_idle().
	 * S5P_CENTRAL_LOWPWR_CFG bit will not be set automatically
	 * in this situation.
	 */
	tmp = __raw_readl(EXYNOS_CENTRAL_SEQ_CONFIGURATION);
	if (!(tmp & EXYNOS_CENTRAL_LOWPWR_CFG)) {
		tmp |= EXYNOS_CENTRAL_LOWPWR_CFG;
		__raw_writel(tmp, EXYNOS_CENTRAL_SEQ_CONFIGURATION);
		/* No need to perform below restore code */
		goto early_wakeup;
	}
	if (!(soc_is_exynos5250() || soc_is_exynos5410())) {
#ifdef CONFIG_ARM_TRUSTZONE
		/* Restore Power control register */
		p_reg = save_arm_register[0];
		/* Restore Diagnostic register */
		d_reg = save_arm_register[1];
		exynos_smc(SMC_CMD_C15RESUME, p_reg, d_reg, 0);
#else
		/* Restore Power control register */
		tmp = save_arm_register[0];
		asm volatile ("mcr p15, 0, %0, c15, c0, 0"
			      : : "r" (tmp)
			      : "cc");

		/* Restore Diagnostic register */
		tmp = save_arm_register[1];
		asm volatile ("mcr p15, 0, %0, c15, c0, 1"
			      : : "r" (tmp)
			      : "cc");
#endif
	}

	/* For release retention */
	if (!soc_is_exynos5410()) {
		__raw_writel((1 << 28), EXYNOS_PAD_RET_MAUDIO_OPTION);
		__raw_writel((1 << 28), EXYNOS_PAD_RET_GPIO_OPTION);
		__raw_writel((1 << 28), EXYNOS_PAD_RET_UART_OPTION);
		__raw_writel((1 << 28), EXYNOS_PAD_RET_MMCA_OPTION);
		__raw_writel((1 << 28), EXYNOS_PAD_RET_MMCB_OPTION);
		__raw_writel((1 << 28), EXYNOS_PAD_RET_EBIA_OPTION);
		__raw_writel((1 << 28), EXYNOS_PAD_RET_EBIB_OPTION);
		__raw_writel((1 << 28), EXYNOS5_PAD_RETENTION_SPI_OPTION);
		__raw_writel((1 << 28), EXYNOS5_PAD_RETENTION_GPIO_SYSMEM_OPTION);
	} else {
		__raw_writel((1 << 28), EXYNOS_PAD_RET_DRAM_OPTION);
		__raw_writel((1 << 28), EXYNOS_PAD_RET_MAUDIO_OPTION);
		__raw_writel((1 << 28), EXYNOS_PAD_RET_JTAG_OPTION);
		__raw_writel((1 << 28), EXYNOS5410_PAD_RET_GPIO_OPTION);
		__raw_writel((1 << 28), EXYNOS5410_PAD_RET_UART_OPTION);
		__raw_writel((1 << 28), EXYNOS5410_PAD_RET_MMCA_OPTION);
		__raw_writel((1 << 28), EXYNOS5410_PAD_RET_MMCB_OPTION);
		__raw_writel((1 << 28), EXYNOS5410_PAD_RET_MMCC_OPTION);
		__raw_writel((1 << 28), EXYNOS5410_PAD_RET_HSI_OPTION);
		__raw_writel((1 << 28), EXYNOS_PAD_RET_EBIA_OPTION);
		__raw_writel((1 << 28), EXYNOS_PAD_RET_EBIB_OPTION);
		__raw_writel((1 << 28), EXYNOS5_PAD_RETENTION_SPI_OPTION);
	}

	s3c_pm_do_restore_core(exynos_core_save, ARRAY_SIZE(exynos_core_save));

	bts_initialize(NULL, true);

	if (!(soc_is_exynos5250() || soc_is_exynos5410())) {
		exynos4_restore_pll();
#ifdef CONFIG_SMP
	if (soc_is_exynos5250())
		scu_enable(S5P_VA_SCU);
	else
		exynos4_scu_enable(S5P_VA_SCU);
#endif
	}

#ifdef CONFIG_EXYNOS5_CLUSTER_POWER_CONTROL
	if (soc_is_exynos5410() && cluster_id == KFC &&
			(__raw_readl(EXYNOS_COMMON_STATUS(0)) & 0x3) == 0x0) {
		__raw_writel(0x3, EXYNOS_COMMON_CONFIGURATION(0));
		/* wait till cluster power control is applied */
		do {
			if (((__raw_readl(EXYNOS_COMMON_STATUS(0)) & 0x3) == 0x3) &&
						((__raw_readl(EXYNOS_L2_STATUS(0)) & 0x3) == 0x3))
					break;
			pr_debug("STEP1 : COMMON_STATUS[0x%x], L2_STATUS[0x%x]\n",
				__raw_readl(EXYNOS_COMMON_STATUS(0)),__raw_readl(EXYNOS_L2_STATUS(0)));
		} while (1);

		__raw_writel(0x0, EXYNOS_COMMON_CONFIGURATION(0));
		/* wait till cluster power control is applied */
		do {
			if (((__raw_readl(EXYNOS_COMMON_STATUS(0)) & 0x3) == 0x3) &&
						((__raw_readl(EXYNOS_L2_STATUS(0)) & 0x3) == 0x3))
				break;
			pr_debug("STEP1 : COMMON_STATUS[0x%x], L2_STATUS[0x%x]\n",
				__raw_readl(EXYNOS_COMMON_STATUS(0)), __raw_readl(EXYNOS_L2_STATUS(0)));
		} while (1);
	}
#endif

early_wakeup:
	if (!soc_is_exynos5410()) {
		exynos_reset_assert_ctrl(true);
		__raw_writel(0x0, REG_INFORM1);
#ifdef CONFIG_CPU_IDLE
	} else {
		exynos_enable_idle_clock_down(ARM);
		exynos_enable_idle_clock_down(KFC);
#endif
	}
	exynos_show_wakeup_reason();

	if (soc_is_exynos5410())
		exynos_set_dummy_state(false);

	return;
}

static struct syscore_ops exynos_pm_syscore_ops = {
	.suspend	= exynos_pm_suspend,
	.resume		= exynos_pm_resume,
};

#ifdef CONFIG_WAKEUP_REASON
/* /proc/wakeup_reason */
static int wakeup_reason_show(struct seq_file *p, void *v)
{
	unsigned long flags;
	int i = *(loff_t *) v;
	struct irqaction *action;
	struct irq_desc *desc;

	if (i == 0) {
		seq_printf(p, "IRQ\tWakeup\tCount\tName\n");
	}

	desc = irq_to_desc(i);
	if (!desc)
		return 0;

	raw_spin_lock_irqsave(&desc->lock, flags);
	action = desc->action;

	if (!action || !desc->wakeup_count)
		goto out;

	seq_printf(p, "%d\t", i);
	seq_printf(p, "%d\t", desc->wakeup_flag);
	seq_printf(p, "%lu\t", desc->wakeup_count);
	if (action) {
		seq_printf(p, "%s", action->name);
		while ((action = action->next) != NULL)
			seq_printf(p, ", %s", action->name);
	}

	seq_putc(p, '\n');
out:
	raw_spin_unlock_irqrestore(&desc->lock, flags);
	return 0;

	return 0;
}

static void *wakeup_reason_start(struct seq_file *m, loff_t *pos)
{
	return (*pos <= nr_irqs) ? pos : NULL;
}

static void *wakeup_reason_next(struct seq_file *m, void *v, loff_t *pos)
{
	(*pos)++;
	if (*pos > nr_irqs)
		return NULL;
	return pos;
}

static void wakeup_reason_stop(struct seq_file *m, void *v)
{
	/* Nothing to do */
}

const struct seq_operations wakeup_reason_op = {
	.start	= wakeup_reason_start,
	.next	= wakeup_reason_next,
	.stop	= wakeup_reason_stop,
	.show	= wakeup_reason_show
};

static int proc_wakeup_reason_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &wakeup_reason_op);
}

static const struct file_operations proc_wakeup_reason_operations = {
	.open		= proc_wakeup_reason_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};
#endif

static __init int exynos_pm_syscore_init(void)
{
	register_syscore_ops(&exynos_pm_syscore_ops);
#ifdef CONFIG_WAKEUP_REASON
	proc_create("wakeup_reason", S_IFREG | S_IRUGO, NULL, &proc_wakeup_reason_operations);
#endif

	return 0;
}
arch_initcall(exynos_pm_syscore_init);
