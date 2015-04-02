/* linux/drivers/video/s5p_mipi_dsi.c
 *
 * Samsung SoC MIPI-DSIM driver.
 *
 * Copyright (c) 2011 Samsung Electronics
 *
 * InKi Dae, <inki.dae@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/memory.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/regulator/consumer.h>
#include <linux/notifier.h>
#include <linux/pm_runtime.h>
#include <linux/lcd.h>
#include <linux/gpio.h>
#include <linux/reboot.h>

#include <video/mipi_display.h>

#include <plat/fb.h>
#include <plat/regs-mipidsim.h>
#include <plat/dsim.h>
#include <plat/cpu.h>
#include <plat/clock.h>

#include <mach/map.h>

#include "s5p_mipi_dsi_lowlevel.h"
#include "s5p_mipi_dsi.h"
#include <mach/regs-clock.h>
#ifdef CONFIG_ION_EXYNOS
#include <plat/iovmm.h>
#include <plat/sysmmu.h>
#include <mach/sysmmu.h>
#endif

#if	defined(CONFIG_ARM_EXYNOS5410_BUS_DEVFREQ) ||   \
	defined(CONFIG_ARM_EXYNOS5420_BUS_DEVFREQ)
#define CONFIG_FIMD_USE_BUS_DEVFREQ
#endif

#if defined(CONFIG_FIMD_USE_BUS_DEVFREQ)
#include <linux/pm_qos.h>
extern struct pm_qos_request *g_exynos5_mif_qos;
extern struct pm_qos_request *g_exynos5_int_qos;
static struct pm_qos_request  resume_cpu_qos;
#endif

extern unsigned int clk_down_request;
extern unsigned int clk_gating_request;
extern unsigned int *clk_enabled;
extern struct clk * clk_lcd;
extern struct clk * clk_axi_disp1;
extern struct clk * clk_fimd;
// defined in devfreq/exynos5410_volt_ctrl.c
extern unsigned int special_volt_mode;

static DEFINE_MUTEX(dsim_rd_wr_mutex);
static DEFINE_MUTEX(dsim_lcd_lock);
static DECLARE_COMPLETION(dsim_wr_comp);
static DECLARE_COMPLETION(dsim_wr_pl_comp);
static DECLARE_COMPLETION(dsim_rd_comp);

#define MIPI_WR_TIMEOUT msecs_to_jiffies(250)
#define MIPI_RD_TIMEOUT msecs_to_jiffies(250)

static unsigned int dpll_table[15] = {
	100, 120, 170, 220, 270,
	320, 390, 450, 510, 560,
	640, 690, 770, 870, 950
};

typedef struct __DSIM_PHYTIMING
{
    unsigned int Mbps;
    unsigned char r1_0;
    unsigned char r1_1;
    unsigned char r1_2;
    unsigned char r1_3;
    unsigned char r2_0;
    unsigned char r2_1;
    unsigned char r2_2;
    unsigned char r0_0;
    unsigned char r0_1;
} DSIM_PHYTIMING;

DSIM_PHYTIMING HSOperationTiming[143] = {
 {  80, 0x00, 0x02, 0x06, 0x01, 0x00, 0x00, 0x03, 0x00, 0x01}, //   80Mbps
 {  90, 0x00, 0x02, 0x07, 0x01, 0x00, 0x00, 0x03, 0x00, 0x01}, //   90Mbps
 { 100, 0x00, 0x03, 0x07, 0x01, 0x00, 0x00, 0x03, 0x00, 0x01}, //  100Mbps
 { 110, 0x00, 0x03, 0x07, 0x01, 0x00, 0x00, 0x04, 0x00, 0x01}, //  110Mbps
 { 120, 0x00, 0x04, 0x07, 0x01, 0x01, 0x00, 0x04, 0x00, 0x01}, //  120Mbps
 { 130, 0x00, 0x04, 0x07, 0x01, 0x01, 0x00, 0x04, 0x00, 0x01}, //  130Mbps
 { 140, 0x00, 0x05, 0x07, 0x01, 0x01, 0x00, 0x04, 0x01, 0x01}, //  140Mbps
 { 150, 0x00, 0x05, 0x07, 0x02, 0x01, 0x00, 0x04, 0x01, 0x01}, //  150Mbps
 { 160, 0x01, 0x06, 0x07, 0x02, 0x01, 0x00, 0x04, 0x01, 0x02}, //  160Mbps
 { 170, 0x01, 0x06, 0x07, 0x02, 0x01, 0x00, 0x04, 0x01, 0x02}, //  170Mbps
 { 180, 0x01, 0x06, 0x07, 0x02, 0x01, 0x00, 0x04, 0x01, 0x02}, //  180Mbps
 { 190, 0x01, 0x07, 0x07, 0x02, 0x01, 0x01, 0x04, 0x01, 0x02}, //  190Mbps
 { 200, 0x01, 0x07, 0x07, 0x02, 0x02, 0x01, 0x04, 0x01, 0x02}, //  200Mbps
 { 210, 0x01, 0x07, 0x07, 0x02, 0x02, 0x01, 0x04, 0x01, 0x02}, //  210Mbps
 { 220, 0x01, 0x08, 0x08, 0x02, 0x02, 0x01, 0x05, 0x01, 0x02}, //  220Mbps
 { 230, 0x01, 0x08, 0x08, 0x02, 0x02, 0x01, 0x05, 0x01, 0x02}, //  230Mbps
 { 240, 0x01, 0x09, 0x08, 0x02, 0x02, 0x01, 0x05, 0x01, 0x03}, //  240Mbps
 { 250, 0x01, 0x0A, 0x08, 0x02, 0x02, 0x01, 0x05, 0x01, 0x03}, //  250Mbps
 { 260, 0x01, 0x0B, 0x08, 0x03, 0x02, 0x02, 0x05, 0x01, 0x03}, //  260Mbps
 { 270, 0x02, 0x0B, 0x08, 0x03, 0x02, 0x02, 0x05, 0x02, 0x03}, //  270Mbps
 { 280, 0x02, 0x0C, 0x08, 0x03, 0x02, 0x02, 0x05, 0x02, 0x03}, //  280Mbps
 { 290, 0x02, 0x0C, 0x08, 0x03, 0x03, 0x02, 0x05, 0x02, 0x03}, //  290Mbps
 { 300, 0x02, 0x0D, 0x08, 0x03, 0x03, 0x03, 0x05, 0x02, 0x03}, //  300Mbps
 { 310, 0x02, 0x0D, 0x08, 0x03, 0x03, 0x03, 0x05, 0x02, 0x03}, //  310Mbps
 { 320, 0x02, 0x0E, 0x08, 0x03, 0x03, 0x03, 0x05, 0x02, 0x04}, //  320Mbps
 { 330, 0x02, 0x0E, 0x08, 0x03, 0x03, 0x03, 0x06, 0x02, 0x04}, //  330Mbps
 { 340, 0x02, 0x0E, 0x08, 0x03, 0x03, 0x03, 0x06, 0x02, 0x04}, //  340Mbps
 { 350, 0x02, 0x0F, 0x09, 0x03, 0x03, 0x03, 0x06, 0x02, 0x04}, //  350Mbps
 { 360, 0x02, 0x0F, 0x09, 0x03, 0x03, 0x04, 0x06, 0x02, 0x04}, //  360Mbps
 { 370, 0x02, 0x10, 0x09, 0x03, 0x03, 0x04, 0x06, 0x02, 0x04}, //  370Mbps
 { 380, 0x03, 0x10, 0x09, 0x04, 0x04, 0x04, 0x06, 0x02, 0x04}, //  380Mbps
 { 390, 0x03, 0x11, 0x09, 0x04, 0x04, 0x04, 0x06, 0x02, 0x04}, //  390Mbps
 { 400, 0x03, 0x11, 0x09, 0x04, 0x04, 0x04, 0x06, 0x03, 0x05}, //  400Mbps
 { 410, 0x03, 0x11, 0x09, 0x04, 0x04, 0x05, 0x06, 0x03, 0x05}, //  410Mbps
 { 420, 0x03, 0x12, 0x09, 0x04, 0x04, 0x05, 0x06, 0x03, 0x05}, //  420Mbps
 { 430, 0x03, 0x12, 0x09, 0x04, 0x04, 0x05, 0x07, 0x03, 0x05}, //  430Mbps
 { 440, 0x03, 0x13, 0x09, 0x04, 0x04, 0x05, 0x07, 0x03, 0x05}, //  440Mbps
 { 450, 0x03, 0x13, 0x09, 0x04, 0x04, 0x05, 0x07, 0x03, 0x05}, //  450Mbps
 { 460, 0x03, 0x14, 0x09, 0x04, 0x04, 0x05, 0x07, 0x03, 0x05}, //  460Mbps
 { 470, 0x03, 0x14, 0x09, 0x04, 0x05, 0x06, 0x07, 0x03, 0x05}, //  470Mbps
 { 480, 0x04, 0x15, 0x09, 0x04, 0x05, 0x06, 0x07, 0x03, 0x06}, //  480Mbps
 { 490, 0x04, 0x15, 0x0A, 0x05, 0x05, 0x06, 0x07, 0x03, 0x06}, //  490Mbps
 { 500, 0x04, 0x15, 0x0A, 0x05, 0x05, 0x06, 0x07, 0x03, 0x06}, //  500Mbps
 { 510, 0x04, 0x16, 0x0A, 0x05, 0x05, 0x06, 0x07, 0x03, 0x06}, //  510Mbps
 { 520, 0x04, 0x16, 0x0A, 0x05, 0x05, 0x07, 0x07, 0x03, 0x06}, //  520Mbps
 { 530, 0x04, 0x17, 0x0A, 0x05, 0x05, 0x07, 0x07, 0x03, 0x06}, //  530Mbps
 { 540, 0x04, 0x17, 0x0A, 0x05, 0x05, 0x07, 0x08, 0x04, 0x06}, //  540Mbps
 { 550, 0x04, 0x18, 0x0A, 0x05, 0x05, 0x07, 0x08, 0x04, 0x06}, //  550Mbps
 { 560, 0x04, 0x18, 0x0A, 0x05, 0x06, 0x07, 0x08, 0x04, 0x07}, //  560Mbps
 { 570, 0x04, 0x18, 0x0A, 0x05, 0x06, 0x07, 0x08, 0x04, 0x07}, //  570Mbps
 { 580, 0x04, 0x19, 0x0A, 0x05, 0x06, 0x08, 0x08, 0x04, 0x07}, //  580Mbps
 { 590, 0x05, 0x19, 0x0A, 0x05, 0x06, 0x08, 0x08, 0x04, 0x07}, //  590Mbps
 { 600, 0x05, 0x1A, 0x0A, 0x06, 0x06, 0x08, 0x08, 0x04, 0x07}, //  600Mbps
 { 610, 0x05, 0x1A, 0x0A, 0x06, 0x06, 0x08, 0x08, 0x04, 0x07}, //  610Mbps
 { 620, 0x05, 0x1B, 0x0B, 0x06, 0x06, 0x08, 0x08, 0x04, 0x07}, //  620Mbps
 { 630, 0x05, 0x1B, 0x0B, 0x06, 0x06, 0x09, 0x08, 0x04, 0x07}, //  630Mbps
 { 640, 0x05, 0x1C, 0x0B, 0x06, 0x06, 0x09, 0x08, 0x04, 0x08}, //  640Mbps
 { 650, 0x05, 0x1C, 0x0B, 0x06, 0x07, 0x09, 0x09, 0x04, 0x08}, //  650Mbps
 { 660, 0x05, 0x1C, 0x0B, 0x06, 0x07, 0x09, 0x09, 0x04, 0x08}, //  660Mbps
 { 670, 0x05, 0x1D, 0x0B, 0x06, 0x07, 0x09, 0x09, 0x05, 0x08}, //  670Mbps
 { 680, 0x05, 0x1D, 0x0B, 0x06, 0x07, 0x09, 0x09, 0x05, 0x08}, //  680Mbps
 { 690, 0x05, 0x1E, 0x0B, 0x06, 0x07, 0x0A, 0x09, 0x05, 0x08}, //  690Mbps
 { 700, 0x06, 0x1E, 0x0B, 0x06, 0x07, 0x0A, 0x09, 0x05, 0x08}, //  700Mbps
 { 710, 0x06, 0x1F, 0x0B, 0x06, 0x07, 0x0A, 0x09, 0x05, 0x08}, //  710Mbps
 { 720, 0x06, 0x1F, 0x0B, 0x07, 0x07, 0x0A, 0x09, 0x05, 0x09}, //  720Mbps
 { 730, 0x06, 0x1F, 0x0B, 0x07, 0x07, 0x0A, 0x09, 0x05, 0x09}, //  730Mbps
 { 740, 0x06, 0x20, 0x0B, 0x07, 0x08, 0x0B, 0x09, 0x05, 0x09}, //  740Mbps
 { 750, 0x06, 0x20, 0x0C, 0x07, 0x08, 0x0B, 0x09, 0x05, 0x09}, //  750Mbps
 { 760, 0x06, 0x21, 0x0C, 0x07, 0x08, 0x0B, 0x0A, 0x05, 0x09}, //  760Mbps
 { 770, 0x06, 0x21, 0x0C, 0x07, 0x08, 0x0B, 0x0A, 0x05, 0x09}, //  770Mbps
 { 780, 0x06, 0x22, 0x0C, 0x07, 0x08, 0x0B, 0x0A, 0x05, 0x09}, //  780Mbps
 { 790, 0x06, 0x22, 0x0C, 0x07, 0x08, 0x0B, 0x0A, 0x05, 0x09}, //  790Mbps
 { 800, 0x07, 0x23, 0x0C, 0x07, 0x08, 0x0C, 0x0A, 0x06, 0x0A}, //  800Mbps
 { 810, 0x07, 0x23, 0x0C, 0x07, 0x08, 0x0C, 0x0A, 0x06, 0x0A}, //  810Mbps
 { 820, 0x07, 0x23, 0x0C, 0x07, 0x08, 0x0C, 0x0A, 0x06, 0x0A}, //  820Mbps
 { 830, 0x07, 0x24, 0x0C, 0x08, 0x09, 0x0C, 0x0A, 0x06, 0x0A}, //  830Mbps
 { 840, 0x07, 0x24, 0x0C, 0x08, 0x09, 0x0C, 0x0A, 0x06, 0x0A}, //  840Mbps
 { 850, 0x07, 0x25, 0x0C, 0x08, 0x09, 0x0D, 0x0A, 0x06, 0x0A}, //  850Mbps
 { 860, 0x07, 0x25, 0x0C, 0x08, 0x09, 0x0D, 0x0B, 0x06, 0x0A}, //  860Mbps
 { 870, 0x07, 0x26, 0x0C, 0x08, 0x09, 0x0D, 0x0B, 0x06, 0x0A}, //  870Mbps
 { 880, 0x07, 0x26, 0x0C, 0x08, 0x09, 0x0D, 0x0B, 0x06, 0x0B}, //  880Mbps
 { 890, 0x07, 0x26, 0x0D, 0x08, 0x09, 0x0D, 0x0B, 0x06, 0x0B}, //  890Mbps
 { 900, 0x07, 0x27, 0x0D, 0x08, 0x09, 0x0D, 0x0B, 0x06, 0x0B}, //  900Mbps
 { 910, 0x08, 0x27, 0x0D, 0x08, 0x09, 0x0E, 0x0B, 0x06, 0x0B}, //  910Mbps
 { 920, 0x08, 0x28, 0x0D, 0x08, 0x0A, 0x0E, 0x0B, 0x06, 0x0B}, //  920Mbps
 { 930, 0x08, 0x28, 0x0D, 0x08, 0x0A, 0x0E, 0x0B, 0x06, 0x0B}, //  930Mbps
 { 940, 0x08, 0x29, 0x0D, 0x08, 0x0A, 0x0E, 0x0B, 0x07, 0x0B}, //  940Mbps
 { 950, 0x08, 0x29, 0x0D, 0x09, 0x0A, 0x0E, 0x0B, 0x07, 0x0B}, //  950Mbps
 { 960, 0x08, 0x2A, 0x0D, 0x09, 0x0A, 0x0F, 0x0B, 0x07, 0x0C}, //  960Mbps
 { 970, 0x08, 0x2A, 0x0D, 0x09, 0x0A, 0x0F, 0x0C, 0x07, 0x0C}, //  970Mbps
 { 980, 0x08, 0x2A, 0x0D, 0x09, 0x0A, 0x0F, 0x0C, 0x07, 0x0C}, //  980Mbps
 { 990, 0x08, 0x2B, 0x0D, 0x09, 0x0A, 0x0F, 0x0C, 0x07, 0x0C}, //  990Mbps
 {1000, 0x08, 0x2B, 0x0D, 0x09, 0x0B, 0x0F, 0x0C, 0x07, 0x0C}, // 1000Mbps
 {1010, 0x08, 0x2C, 0x0D, 0x09, 0x0B, 0x0F, 0x0C, 0x07, 0x0C}, // 1010Mbps
 {1020, 0x09, 0x2C, 0x0E, 0x09, 0x0B, 0x10, 0x0C, 0x07, 0x0C}, // 1020Mbps
 {1030, 0x09, 0x2D, 0x0E, 0x09, 0x0B, 0x10, 0x0C, 0x07, 0x0C}, // 1030Mbps
 {1040, 0x09, 0x2D, 0x0E, 0x09, 0x0B, 0x10, 0x0C, 0x07, 0x0D}, // 1040Mbps
 {1050, 0x09, 0x2D, 0x0E, 0x09, 0x0B, 0x10, 0x0C, 0x07, 0x0D}, // 1050Mbps
 {1060, 0x09, 0x2E, 0x0E, 0x0A, 0x0B, 0x10, 0x0C, 0x07, 0x0D}, // 1060Mbps
 {1070, 0x09, 0x2E, 0x0E, 0x0A, 0x0B, 0x11, 0x0C, 0x08, 0x0D}, // 1070Mbps
 {1080, 0x09, 0x2F, 0x0E, 0x0A, 0x0B, 0x11, 0x0D, 0x08, 0x0D}, // 1080Mbps
 {1090, 0x09, 0x2F, 0x0E, 0x0A, 0x0C, 0x11, 0x0D, 0x08, 0x0D}, // 1090Mbps
 {1100, 0x09, 0x30, 0x0E, 0x0A, 0x0C, 0x11, 0x0D, 0x08, 0x0D}, // 1100Mbps
 {1110, 0x09, 0x30, 0x0E, 0x0A, 0x0C, 0x11, 0x0D, 0x08, 0x0D}, // 1110Mbps
 {1120, 0x0A, 0x31, 0x0E, 0x0A, 0x0C, 0x11, 0x0D, 0x08, 0x0E}, // 1120Mbps
 {1130, 0x0A, 0x31, 0x0E, 0x0A, 0x0C, 0x12, 0x0D, 0x08, 0x0E}, // 1130Mbps
 {1140, 0x0A, 0x31, 0x0E, 0x0A, 0x0C, 0x12, 0x0D, 0x08, 0x0E}, // 1140Mbps
 {1150, 0x0A, 0x32, 0x0F, 0x0A, 0x0C, 0x12, 0x0D, 0x08, 0x0E}, // 1150Mbps
 {1160, 0x0A, 0x32, 0x0F, 0x0A, 0x0C, 0x12, 0x0D, 0x08, 0x0E}, // 1160Mbps
 {1170, 0x0A, 0x33, 0x0F, 0x0A, 0x0C, 0x12, 0x0D, 0x08, 0x0E}, // 1170Mbps
 {1180, 0x0A, 0x33, 0x0F, 0x0B, 0x0D, 0x13, 0x0D, 0x08, 0x0E}, // 1180Mbps
 {1190, 0x0A, 0x34, 0x0F, 0x0B, 0x0D, 0x13, 0x0E, 0x08, 0x0E}, // 1190Mbps
 {1200, 0x0A, 0x34, 0x0F, 0x0B, 0x0D, 0x13, 0x0E, 0x09, 0x0F}, // 1200Mbps
 {1210, 0x0A, 0x34, 0x0F, 0x0B, 0x0D, 0x13, 0x0E, 0x09, 0x0F}, // 1210Mbps
 {1220, 0x0A, 0x35, 0x0F, 0x0B, 0x0D, 0x13, 0x0E, 0x09, 0x0F}, // 1220Mbps
 {1230, 0x0B, 0x35, 0x0F, 0x0B, 0x0D, 0x13, 0x0E, 0x09, 0x0F}, // 1230Mbps
 {1240, 0x0B, 0x36, 0x0F, 0x0B, 0x0D, 0x14, 0x0E, 0x09, 0x0F}, // 1240Mbps
 {1250, 0x0B, 0x36, 0x0F, 0x0B, 0x0D, 0x14, 0x0E, 0x09, 0x0F}, // 1250Mbps
 {1260, 0x0B, 0x37, 0x0F, 0x0B, 0x0D, 0x14, 0x0E, 0x09, 0x0F}, // 1260Mbps
 {1270, 0x0B, 0x37, 0x0F, 0x0B, 0x0E, 0x14, 0x0E, 0x09, 0x0F}, // 1270Mbps
 {1280, 0x0B, 0x38, 0x0F, 0x0B, 0x0E, 0x14, 0x0E, 0x09, 0x10}, // 1280Mbps
 {1290, 0x0B, 0x38, 0x10, 0x0C, 0x0E, 0x15, 0x0F, 0x09, 0x10}, // 1290Mbps
 {1300, 0x0B, 0x38, 0x10, 0x0C, 0x0E, 0x15, 0x0F, 0x09, 0x10}, // 1300Mbps
 {1310, 0x0B, 0x39, 0x10, 0x0C, 0x0E, 0x15, 0x0F, 0x09, 0x10}, // 1310Mbps
 {1320, 0x0B, 0x39, 0x10, 0x0C, 0x0E, 0x15, 0x0F, 0x09, 0x10}, // 1320Mbps
 {1330, 0x0B, 0x3A, 0x10, 0x0C, 0x0E, 0x15, 0x0F, 0x09, 0x10}, // 1330Mbps
 {1340, 0x0C, 0x3A, 0x10, 0x0C, 0x0E, 0x15, 0x0F, 0x0A, 0x10}, // 1340Mbps
 {1350, 0x0C, 0x3B, 0x10, 0x0C, 0x0E, 0x16, 0x0F, 0x0A, 0x10}, // 1350Mbps
 {1360, 0x0C, 0x3B, 0x10, 0x0C, 0x0F, 0x16, 0x0F, 0x0A, 0x11}, // 1360Mbps
 {1370, 0x0C, 0x3B, 0x10, 0x0C, 0x0F, 0x16, 0x0F, 0x0A, 0x11}, // 1370Mbps
 {1380, 0x0C, 0x3C, 0x10, 0x0C, 0x0F, 0x16, 0x0F, 0x0A, 0x11}, // 1380Mbps
 {1390, 0x0C, 0x3C, 0x10, 0x0C, 0x0F, 0x16, 0x0F, 0x0A, 0x11}, // 1390Mbps
 {1400, 0x0C, 0x3D, 0x10, 0x0D, 0x0F, 0x17, 0x10, 0x0A, 0x11}, // 1400Mbps
 {1410, 0x0C, 0x3D, 0x10, 0x0D, 0x0F, 0x17, 0x10, 0x0A, 0x11}, // 1410Mbps
 {1420, 0x0C, 0x3E, 0x11, 0x0D, 0x0F, 0x17, 0x10, 0x0A, 0x11}, // 1420Mbps
 {1430, 0x0C, 0x3E, 0x11, 0x0D, 0x0F, 0x17, 0x10, 0x0A, 0x11}, // 1430Mbps
 {1440, 0x0D, 0x3F, 0x11, 0x0D, 0x0F, 0x17, 0x10, 0x0A, 0x12}, // 1440Mbps
 {1450, 0x0D, 0x3F, 0x11, 0x0D, 0x10, 0x17, 0x10, 0x0A, 0x12}, // 1450Mbps
 {1460, 0x0D, 0x3F, 0x11, 0x0D, 0x10, 0x18, 0x10, 0x0A, 0x12}, // 1460Mbps
 {1470, 0x0D, 0x40, 0x11, 0x0D, 0x10, 0x18, 0x10, 0x0B, 0x12}, // 1470Mbps
 {1480, 0x0D, 0x40, 0x11, 0x0D, 0x10, 0x18, 0x10, 0x0B, 0x12}, // 1480Mbps
 {1490, 0x0D, 0x41, 0x11, 0x0D, 0x10, 0x18, 0x10, 0x0B, 0x12}, // 1490Mbps
 {1500, 0x0D, 0x41, 0x11, 0x0D, 0x10, 0x18, 0x10, 0x0B, 0x12}, // 1500Mbps
};

static int s5p_mipi_clk_state(struct clk *clk)
{
	return clk->usage;
}

static int s5p_mipi_en_state(struct mipi_dsim_device *dsim)
{
	int ret = (!pm_runtime_suspended(dsim->dev) && s5p_mipi_clk_state(dsim->clock));

	return ret;
}

static int s5p_mipi_force_enable(struct mipi_dsim_device *dsim)
{
	if (pm_runtime_suspended(dsim->dev)){
		pm_runtime_get_sync(dsim->dev);
    }
	if (!s5p_mipi_clk_state(dsim->clock))
		clk_enable(dsim->clock);
	return 0;
}

unsigned int mipi_framedone_status=1; // init status is framedone
EXPORT_SYMBOL(mipi_framedone_status);
unsigned int mipi_reg=0;
EXPORT_SYMBOL(mipi_reg);


static void s5p_mipi_dsi_long_data_wr(struct mipi_dsim_device *dsim, unsigned int data0, unsigned int data1)
{
	unsigned int data_cnt = 0, payload = 0;

	/* in case that data count is more then 4 */
	for (data_cnt = 0; data_cnt < data1; data_cnt += 4) {
		/*
		 * after sending 4bytes per one time,
		 * send remainder data less then 4.
		 */
		if ((data1 - data_cnt) < 4) {
			if ((data1 - data_cnt) == 3) {
				payload = *(u8 *)(data0 + data_cnt) |
				    (*(u8 *)(data0 + (data_cnt + 1))) << 8 |
					(*(u8 *)(data0 + (data_cnt + 2))) << 16;
			dev_dbg(dsim->dev, "count = 3 payload = %x, %x %x %x\n",
				payload, *(u8 *)(data0 + data_cnt),
				*(u8 *)(data0 + (data_cnt + 1)),
				*(u8 *)(data0 + (data_cnt + 2)));
			} else if ((data1 - data_cnt) == 2) {
				payload = *(u8 *)(data0 + data_cnt) |
					(*(u8 *)(data0 + (data_cnt + 1))) << 8;
			dev_dbg(dsim->dev,
				"count = 2 payload = %x, %x %x\n", payload,
				*(u8 *)(data0 + data_cnt),
				*(u8 *)(data0 + (data_cnt + 1)));
			} else if ((data1 - data_cnt) == 1) {
				payload = *(u8 *)(data0 + data_cnt);
			}

			s5p_mipi_dsi_wr_tx_data(dsim, payload);
			mdelay(1);
		/* send 4bytes per one time. */
		} else {
			payload = *(u8 *)(data0 + data_cnt) |
				(*(u8 *)(data0 + (data_cnt + 1))) << 8 |
				(*(u8 *)(data0 + (data_cnt + 2))) << 16 |
				(*(u8 *)(data0 + (data_cnt + 3))) << 24;

			dev_dbg(dsim->dev,
				"count = 4 payload = %x, %x %x %x %x\n",
				payload, *(u8 *)(data0 + data_cnt),
				*(u8 *)(data0 + (data_cnt + 1)),
				*(u8 *)(data0 + (data_cnt + 2)),
				*(u8 *)(data0 + (data_cnt + 3)));

			s5p_mipi_dsi_wr_tx_data(dsim, payload);
		}
	}
}

int s5p_mipi_dsi_wr_data(struct mipi_dsim_device *dsim, unsigned int data_id,
	unsigned int data0, unsigned int data1)
{
	if (dsim->enabled == false) {
		dev_err(dsim->dev, "MIPI DSIM is disabled.\n");
		return -EINVAL;
	}

	if (dsim->state != DSIM_STATE_STOP && dsim->state != DSIM_STATE_HSCLKEN) {
			dev_err(dsim->dev, "MIPI DSIM is not ready.\n");
		return -EINVAL;
	}

	if (!s5p_mipi_en_state(dsim)) {
		s5p_mipi_force_enable(dsim);
		dev_warn(dsim->dev, "%s: MIPI state check!!\n", __func__);
	}

	mutex_lock(&dsim_rd_wr_mutex);
	switch (data_id) {
	/* short packet types of packet types for command. */
	case MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM:
	case MIPI_DSI_DCS_SHORT_WRITE:
	case MIPI_DSI_DCS_SHORT_WRITE_PARAM:
	case MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE:
		INIT_COMPLETION(dsim_wr_comp);
		s5p_mipi_dsi_wr_tx_header(dsim, data_id, data0, data1);
		if (!wait_for_completion_interruptible_timeout(&dsim_wr_comp,
			MIPI_WR_TIMEOUT)) {
				dev_err(dsim->dev, "MIPI DSIM write Timeout!\n");
				mutex_unlock(&dsim_rd_wr_mutex);
				return -1;
		}

		break;

	/* general command */
	case MIPI_DSI_COLOR_MODE_OFF:
	case MIPI_DSI_COLOR_MODE_ON:
	case MIPI_DSI_SHUTDOWN_PERIPHERAL:
	case MIPI_DSI_TURN_ON_PERIPHERAL:
//		INIT_COMPLETION(dsim_wr_comp);
		s5p_mipi_dsi_wr_tx_header(dsim, data_id, data0, data1);
/*		if (!wait_for_completion_interruptible_timeout(&dsim_wr_comp,
			MIPI_WR_TIMEOUT)) {
				dev_err(dsim->dev, "MIPI DSIM write Timeout!\n");
				mutex_unlock(&dsim_rd_wr_mutex);
				return -1;
		}*/
		break;

	/* packet types for video data */
	case MIPI_DSI_V_SYNC_START:
	case MIPI_DSI_V_SYNC_END:
	case MIPI_DSI_H_SYNC_START:
	case MIPI_DSI_H_SYNC_END:
	case MIPI_DSI_END_OF_TRANSMISSION:
		break;

	/* short and response packet types for command */
	case MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM:
	case MIPI_DSI_DCS_READ:
		s5p_mipi_dsi_clear_all_interrupt(dsim);
		s5p_mipi_dsi_wr_tx_header(dsim, data_id, data0, data1);
		/* process response func should be implemented. */
		break;

	/* long packet type and null packet */
	case MIPI_DSI_NULL_PACKET:
	case MIPI_DSI_BLANKING_PACKET:
		break;

	case MIPI_DSI_GENERIC_LONG_WRITE:
	case MIPI_DSI_DCS_LONG_WRITE:
	{
		unsigned int size, data_cnt = 0, payload = 0;

		size = data1 * 4;
		INIT_COMPLETION(dsim_wr_comp);
		/* if data count is less then 4, then send 3bytes data.  */
		if (data1 < 4) {
			payload = *(u8 *)(data0) |
				*(u8 *)(data0 + 1) << 8 |
				*(u8 *)(data0 + 2) << 16;

			s5p_mipi_dsi_wr_tx_data(dsim, payload);

			dev_dbg(dsim->dev, "count = %d payload = %x,%x %x %x\n",
				data1, payload,
				*(u8 *)(data0 + data_cnt),
				*(u8 *)(data0 + (data_cnt + 1)),
				*(u8 *)(data0 + (data_cnt + 2)));
		/* in case that data count is more then 4 */
		} else
			s5p_mipi_dsi_long_data_wr(dsim, data0, data1);

		/* put data into header fifo */
		s5p_mipi_dsi_wr_tx_header(dsim, data_id, data1 & 0xff,
			(data1 & 0xff00) >> 8);

		if (!wait_for_completion_interruptible_timeout(&dsim_wr_comp,
			MIPI_WR_TIMEOUT)) {
				dev_err(dsim->dev, "MIPI DSIM write Timeout!\n");
				mutex_unlock(&dsim_rd_wr_mutex);
				return -ETIMEDOUT;
		}
		break;
	}

	/* packet typo for video data */
	case MIPI_DSI_PACKED_PIXEL_STREAM_16:
	case MIPI_DSI_PACKED_PIXEL_STREAM_18:
	case MIPI_DSI_PIXEL_STREAM_3BYTE_18:
	case MIPI_DSI_PACKED_PIXEL_STREAM_24:
		break;
	default:
		dev_warn(dsim->dev,
			"data id %x is not supported current DSI spec.\n",
			data_id);

		mutex_unlock(&dsim_rd_wr_mutex);
		return -EINVAL;
	}
	mutex_unlock(&dsim_rd_wr_mutex);

	return 0;
}

static void s5p_mipi_dsi_rx_err_handler(struct mipi_dsim_device *dsim,
	u32 rx_fifo)
{
	/* Parse error report bit*/
	if (rx_fifo & (1 << 8))
		dev_err(dsim->dev, "SoT error!\n");
	if (rx_fifo & (1 << 9))
		dev_err(dsim->dev, "SoT sync error!\n");
	if (rx_fifo & (1 << 10))
		dev_err(dsim->dev, "EoT error!\n");
	if (rx_fifo & (1 << 11))
		dev_err(dsim->dev, "Escape mode entry command error!\n");
	if (rx_fifo & (1 << 12))
		dev_err(dsim->dev, "Low-power transmit sync error!\n");
	if (rx_fifo & (1 << 13))
		dev_err(dsim->dev, "HS receive timeout error!\n");
	if (rx_fifo & (1 << 14))
		dev_err(dsim->dev, "False control error!\n");
	/* Bit 15 is reserved*/
	if (rx_fifo & (1 << 16))
		dev_err(dsim->dev, "ECC error, single-bit(detected and corrected)!\n");
	if (rx_fifo & (1 << 17))
		dev_err(dsim->dev, "ECC error, multi-bit(detected, not corrected)!\n");
	if (rx_fifo & (1 << 18))
		dev_err(dsim->dev, "Checksum error(long packet only)!\n");
	if (rx_fifo & (1 << 19))
		dev_err(dsim->dev, "DSI data type not recognized!\n");
	if (rx_fifo & (1 << 20))
		dev_err(dsim->dev, "DSI VC ID invalid!\n");
	if (rx_fifo & (1 << 21))
		dev_err(dsim->dev, "Invalid transmission length!\n");
	/* Bit 22 is reserved */
	if (rx_fifo & (1 << 23))
		dev_err(dsim->dev, "DSI protocol violation!\n");
}

int s5p_mipi_dsi_rd_data(struct mipi_dsim_device *dsim, u32 data_id,
	 u32 addr, u32 count, u8 *buf)
{
	u32 rx_fifo,txhd, rx_size;
	int i, j;

	if (dsim->enabled == false) {
		dev_err(dsim->dev, "MIPI DSIM is disabled.\n");
		return -EINVAL;
	}

	if (dsim->state != DSIM_STATE_STOP && dsim->state != DSIM_STATE_HSCLKEN) {
			dev_err(dsim->dev, "MIPI DSIM is not ready.\n");
		return -EINVAL;
	}

	/*clear fifo firstly*/
	rx_fifo = readl(dsim->reg_base + S5P_DSIM_RXFIFO);
	mutex_lock(&dsim_rd_wr_mutex);

	INIT_COMPLETION(dsim_wr_comp);
	/* Set the maximum packet size returned */
	txhd = MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE | count << 8;
	writel(txhd, dsim->reg_base + S5P_DSIM_PKTHDR);
	if (!wait_for_completion_interruptible_timeout(&dsim_wr_comp,
				MIPI_RD_TIMEOUT)) {
		dev_err(dsim->dev, "MIPI DSIM set Timeout 2!\n");
		mutex_unlock(&dsim_rd_wr_mutex);
		return -ETIMEDOUT;
	}

	INIT_COMPLETION(dsim_rd_comp);
	 /* Read request */
	txhd = data_id | addr << 8;
	writel(txhd, dsim->reg_base + S5P_DSIM_PKTHDR);

	if (!wait_for_completion_interruptible_timeout(&dsim_rd_comp,
		MIPI_RD_TIMEOUT)) {
		dev_err(dsim->dev, "MIPI DSIM read Timeout!\n");
		mutex_unlock(&dsim_rd_wr_mutex);
		return -ETIMEDOUT;
	}

	rx_fifo = readl(dsim->reg_base + S5P_DSIM_RXFIFO);

	/* Parse the RX packet data types */
	switch (rx_fifo & 0xff) {
	case MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT:
		s5p_mipi_dsi_rx_err_handler(dsim, rx_fifo);
		goto rx_error;
	case MIPI_DSI_RX_END_OF_TRANSMISSION:
		dev_dbg(dsim->dev, "EoTp was received from LCD module.\n");
		break;
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
		dev_dbg(dsim->dev, "Short Packet was received from LCD module.\n");
		for (i = 0; i <= count; i++)
			buf[i] = (rx_fifo >> (8 + i * 8)) & 0xff;
		break;
	case MIPI_DSI_RX_DCS_LONG_READ_RESPONSE:
	case MIPI_DSI_RX_GENERIC_LONG_READ_RESPONSE:
		dev_dbg(dsim->dev, "Long Packet was received from LCD module.\n");
		rx_size = (rx_fifo & 0x00ffff00) >> 8;
		/* Read data from RX packet payload */
		for (i = 0; i < rx_size >> 2; i++) {
			rx_fifo = readl(dsim->reg_base + S5P_DSIM_RXFIFO);
			buf[0 + i] = (u8)(rx_fifo >> 0) & 0xff;
			buf[1 + i] = (u8)(rx_fifo >> 8) & 0xff;
			buf[2 + i] = (u8)(rx_fifo >> 16) & 0xff;
			buf[3 + i] = (u8)(rx_fifo >> 24) & 0xff;
		}
		if (rx_size % 4) {
			rx_fifo = readl(dsim->reg_base + S5P_DSIM_RXFIFO);
			for (j = 0; j < rx_size % 4; j++)
				buf[4 * i + j] =
					(u8)(rx_fifo >> (j * 8)) & 0xff;
		}
		break;
	default:
		dev_err(dsim->dev, "Packet format is invaild.\n");
		goto rx_error;
	}

	rx_fifo = readl(dsim->reg_base + S5P_DSIM_RXFIFO);
	if (rx_fifo != DSIM_RX_FIFO_READ_DONE) {
		dev_dbg(dsim->dev, "[DSIM:WARN]:%s Can't find RX FIFO READ DONE FLAG : %x\n",
			__func__, rx_fifo);
		goto clear_rx_fifo;
	}
	mutex_unlock(&dsim_rd_wr_mutex);
	return 0;

clear_rx_fifo:
	i = 0;
	while (1) {
		rx_fifo = readl(dsim->reg_base + S5P_DSIM_RXFIFO);
		if ((rx_fifo == DSIM_RX_FIFO_READ_DONE) ||
				(i > DSIM_MAX_RX_FIFO))
			break;
		dev_dbg(dsim->dev, "[DSIM:INFO] : %s clear rx fifo : %08x\n",
			__func__, rx_fifo);
		i++;
	}
	mutex_unlock(&dsim_rd_wr_mutex);
	return 0;

rx_error:
	s5p_mipi_dsi_force_dphy_stop_state(dsim, 1);
	usleep_range(3000, 4000);
	s5p_mipi_dsi_force_dphy_stop_state(dsim, 0);
	mutex_unlock(&dsim_rd_wr_mutex);
	return -1;
}

int s5p_mipi_dsi_pll_on(struct mipi_dsim_device *dsim, unsigned int enable)
{
	int sw_timeout;

	if (enable) {
		sw_timeout = 100;

		s5p_mipi_dsi_clear_interrupt(dsim, INTSRC_PLL_STABLE);
		s5p_mipi_dsi_enable_pll(dsim, 1);
		while (1) {
			usleep_range(1000, 1000);
			sw_timeout--;
			if (s5p_mipi_dsi_is_pll_stable(dsim))
				return 0;
			if (sw_timeout == 0) {
				pr_err("mipi pll on fail!!!\n");
				return -EINVAL;
			}
		}
	} else
		s5p_mipi_dsi_enable_pll(dsim, 0);

	return 0;
}

unsigned long s5p_mipi_dsi_change_pll(struct mipi_dsim_device *dsim,
	unsigned int pre_divider, unsigned int main_divider,
	unsigned int scaler)
{
	unsigned long dfin_pll, dfvco, dpll_out;
	unsigned int i, freq_band = 0xf;

	dfin_pll = (FIN_HZ / pre_divider);

	if (soc_is_exynos5250()) {
		if (dfin_pll < DFIN_PLL_MIN_HZ || dfin_pll > DFIN_PLL_MAX_HZ) {
			dev_warn(dsim->dev, "fin_pll range should be 6MHz ~ 12MHz\n");
			s5p_mipi_dsi_enable_afc(dsim, 0, 0);
		} else {
			if (dfin_pll < 7 * MHZ)
				s5p_mipi_dsi_enable_afc(dsim, 1, 0x1);
			else if (dfin_pll < 8 * MHZ)
				s5p_mipi_dsi_enable_afc(dsim, 1, 0x0);
			else if (dfin_pll < 9 * MHZ)
				s5p_mipi_dsi_enable_afc(dsim, 1, 0x3);
			else if (dfin_pll < 10 * MHZ)
				s5p_mipi_dsi_enable_afc(dsim, 1, 0x2);
			else if (dfin_pll < 11 * MHZ)
				s5p_mipi_dsi_enable_afc(dsim, 1, 0x5);
			else
				s5p_mipi_dsi_enable_afc(dsim, 1, 0x4);
		}
	}
	dfvco = dfin_pll * main_divider;
	dev_dbg(dsim->dev, "dfvco = %lu, dfin_pll = %lu, main_divider = %d\n",
				dfvco, dfin_pll, main_divider);

	if (soc_is_exynos5250()) {
		if (dfvco < DFVCO_MIN_HZ || dfvco > DFVCO_MAX_HZ)
			dev_warn(dsim->dev, "fvco range should be 500MHz ~ 1000MHz\n");
	}

	dpll_out = dfvco / (1 << scaler);
	dev_dbg(dsim->dev, "dpll_out = %lu, dfvco = %lu, scaler = %d\n",
		dpll_out, dfvco, scaler);

	if (soc_is_exynos5250()) {
		for (i = 0; i < ARRAY_SIZE(dpll_table); i++) {
			if (dpll_out < dpll_table[i] * MHZ) {
				freq_band = i;
				break;
			}
		}
	}

	dev_dbg(dsim->dev, "freq_band = %d\n", freq_band);

	s5p_mipi_dsi_pll_freq(dsim, pre_divider, main_divider, scaler);

	s5p_mipi_dsi_hs_zero_ctrl(dsim, 0);
	s5p_mipi_dsi_prep_ctrl(dsim, 0);

	if (soc_is_exynos5250()) {
		/* Freq Band */
		s5p_mipi_dsi_pll_freq_band(dsim, freq_band);
	}
	/* Stable time */
	s5p_mipi_dsi_pll_stable_time(dsim, dsim->dsim_config->pll_stable_time);

	/* Enable PLL */
	dev_dbg(dsim->dev, "FOUT of mipi dphy pll is %luMHz\n",
		(dpll_out / MHZ));

	return dpll_out;
}

static unsigned int s5p_mipi_dsi_get_esc_dphy_setting(unsigned int target_clock)
{
    u32 dphyctrl_value=0x003; // Default
    u32 dphyctrl_table[20]={ 0x019, 0x032, 0x04B, 0x064, 0x07D, 0x096, 0x0AF, 0x0C8, 0x0E1, 0x0FA,
                             0x113, 0x12C, 0x145, 0x15E, 0x177, 0x190, 0x1A9, 0x1C2, 0x1DB, 0x1F4};
    if(target_clock<=20 && target_clock >=1)
        dphyctrl_value=dphyctrl_table[target_clock-1];
    return dphyctrl_value;
}

static void s5p_mipi_dsi_set_hs_dphy_timing(struct mipi_dsim_device *dsim,
                                            unsigned int target_Mbps)
{
    DSIM_PHYTIMING ok;
    int i;

    for (i = 0; i < (sizeof(HSOperationTiming)/sizeof(DSIM_PHYTIMING)); i++) {
        if (HSOperationTiming[i].Mbps == target_Mbps) {
            ok=HSOperationTiming[i];
            s5p_mipi_dsi_set_timing_register0(dsim, ok.r0_0, ok.r0_1); //1080Mbps
            s5p_mipi_dsi_set_timing_register1(dsim, ok.r1_0, ok.r1_1, ok.r1_2, ok.r1_3);
            s5p_mipi_dsi_set_timing_register2(dsim, ok.r2_0, ok.r2_1, ok.r2_2);
            dev_dbg(dsim->dev, "found value for %dMbps: %x, %x, %x, %x, %x, %x, %x, %x, %x\n", 
               ok.Mbps, ok.r0_0, ok.r0_1, ok.r1_0, ok.r1_1, ok.r1_2, ok.r1_3, ok.r2_0, ok.r2_1, ok.r2_2);
            break;
        }
    }
    dev_dbg(dsim->dev, "Could not find proper DPHY HS TIMING Value: %d\n", target_Mbps);
}

static int s5p_mipi_dsi_set_clock(struct mipi_dsim_device *dsim,
	unsigned int byte_clk_sel, unsigned int enable)
{
	unsigned int esc_div;
	unsigned long esc_clk_error_rate;

	if (enable) {
		dsim->e_clk_src = byte_clk_sel;

		/* Escape mode clock and byte clock source */
		s5p_mipi_dsi_set_byte_clock_src(dsim, byte_clk_sel);

		/* DPHY, DSIM Link : D-PHY clock out */
		if (byte_clk_sel == DSIM_PLL_OUT_DIV8) {
			dsim->hs_clk = s5p_mipi_dsi_change_pll(dsim,
				dsim->dsim_config->p, dsim->dsim_config->m,
				dsim->dsim_config->s);
			if (dsim->hs_clk == 0) {
				dev_err(dsim->dev,
					"failed to get hs clock.\n");
				return -EINVAL;
			}
			if (!soc_is_exynos5250()) {
                s5p_mipi_dsi_set_b_dphyctrl(dsim, s5p_mipi_dsi_get_esc_dphy_setting(15)); 
                s5p_mipi_dsi_set_hs_dphy_timing(dsim, 980);
			}
			dsim->byte_clk = dsim->hs_clk / 8;
			s5p_mipi_dsi_enable_pll_bypass(dsim, 0);
			s5p_mipi_dsi_pll_on(dsim, 1);
		/* DPHY : D-PHY clock out, DSIM link : external clock out */
		} else if (byte_clk_sel == DSIM_EXT_CLK_DIV8)
			dev_warn(dsim->dev,
				"this project is not support external clock source for MIPI DSIM\n");
		else if (byte_clk_sel == DSIM_EXT_CLK_BYPASS)
			dev_warn(dsim->dev,
				"this project is not support external clock source for MIPI DSIM\n");

		/* escape clock divider */
		esc_div = dsim->byte_clk / (dsim->dsim_config->esc_clk);
		dev_dbg(dsim->dev,
			"esc_div = %d, byte_clk = %lu, esc_clk = %lu\n",
			esc_div, dsim->byte_clk, dsim->dsim_config->esc_clk);

		if (soc_is_exynos5250()) {
			if ((dsim->byte_clk / esc_div) >= (20 * MHZ) ||
					(dsim->byte_clk / esc_div) >
						dsim->dsim_config->esc_clk)
				esc_div += 1;
		} else {
			if ((dsim->byte_clk / esc_div) >= (10 * MHZ) ||
				(dsim->byte_clk / esc_div) >
					dsim->dsim_config->esc_clk)
				esc_div += 1;
		}
		dsim->escape_clk = dsim->byte_clk / esc_div;
		dev_dbg(dsim->dev,
			"escape_clk = %lu, byte_clk = %lu, esc_div = %d\n",
			dsim->escape_clk, dsim->byte_clk, esc_div);

		/* enable byte clock. */
		s5p_mipi_dsi_enable_byte_clock(dsim, DSIM_ESCCLK_ON);

		/* enable escape clock */
		s5p_mipi_dsi_set_esc_clk_prs(dsim, 1, esc_div);
		/* escape clock on lane */
		s5p_mipi_dsi_enable_esc_clk_on_lane(dsim,
			(DSIM_LANE_CLOCK | dsim->data_lane), 1);

		dev_dbg(dsim->dev, "byte clock is %luMHz\n",
			(dsim->byte_clk / MHZ));
		dev_dbg(dsim->dev, "escape clock that user's need is %lu\n",
			(dsim->dsim_config->esc_clk / MHZ));
		dev_dbg(dsim->dev, "escape clock divider is %x\n", esc_div);
		dev_dbg(dsim->dev, "escape clock is %luMHz\n",
			((dsim->byte_clk / esc_div) / MHZ));

		if ((dsim->byte_clk / esc_div) > dsim->escape_clk) {
			esc_clk_error_rate = dsim->escape_clk /
				(dsim->byte_clk / esc_div);
			dev_warn(dsim->dev, "error rate is %lu over.\n",
				(esc_clk_error_rate / 100));
		} else if ((dsim->byte_clk / esc_div) < (dsim->escape_clk)) {
			esc_clk_error_rate = (dsim->byte_clk / esc_div) /
				dsim->escape_clk;
			dev_warn(dsim->dev, "error rate is %lu under.\n",
				(esc_clk_error_rate / 100));
		}
	} else {
		s5p_mipi_dsi_enable_esc_clk_on_lane(dsim,
			(DSIM_LANE_CLOCK | dsim->data_lane), 0);
		s5p_mipi_dsi_set_esc_clk_prs(dsim, 0, 0);

		/* disable escape clock. */
		s5p_mipi_dsi_enable_byte_clock(dsim, DSIM_ESCCLK_OFF);

		if (byte_clk_sel == DSIM_PLL_OUT_DIV8)
			s5p_mipi_dsi_pll_on(dsim, 0);
	}

	return 0;
}

static void s5p_mipi_dsi_d_phy_onoff(struct mipi_dsim_device *dsim,
	unsigned int enable)
{
	if (dsim->pd->init_d_phy)
		dsim->pd->init_d_phy(dsim, enable);
}

extern unsigned int system_rev;

static int s5p_mipi_dsi_init_dsim(struct mipi_dsim_device *dsim)
{
	s5p_mipi_dsi_d_phy_onoff(dsim, 1);

	dsim->state = DSIM_STATE_INIT;

	switch (dsim->dsim_config->e_no_data_lane) {
	case DSIM_DATA_LANE_1:
		dsim->data_lane = DSIM_LANE_DATA0;
		break;
	case DSIM_DATA_LANE_2:
		dsim->data_lane = DSIM_LANE_DATA0 | DSIM_LANE_DATA1;
		break;
	case DSIM_DATA_LANE_3:
		dsim->data_lane = DSIM_LANE_DATA0 | DSIM_LANE_DATA1 |
			DSIM_LANE_DATA2;
		break;
	case DSIM_DATA_LANE_4:
		dsim->data_lane = DSIM_LANE_DATA0 | DSIM_LANE_DATA1 |
			DSIM_LANE_DATA2 | DSIM_LANE_DATA3;
		break;
	default:
		dev_dbg(dsim->dev, "data lane is invalid.\n");
		return -EINVAL;
	};

	s5p_mipi_dsi_sw_reset(dsim);
	s5p_mipi_dsi_dp_dn_swap(dsim, 0);

	return 0;
}

#if 0
static int s5p_mipi_dsi_enable_frame_done_int(struct mipi_dsim_device *dsim,
	unsigned int enable)
{
	/* enable only frame done interrupt */
	s5p_mipi_dsi_set_interrupt_mask(dsim, INTMSK_FRAME_DONE, enable);

	return 0;
}
#endif

static int s5p_mipi_dsi_set_display_mode(struct mipi_dsim_device *dsim,
	struct mipi_dsim_config *dsim_config)
{
	struct fb_videomode *lcd_video = NULL;
	struct s3c_fb_pd_win *pd;
	unsigned int width = 0, height = 0;
	pd = (struct s3c_fb_pd_win *)dsim->dsim_config->lcd_panel_info;
	lcd_video = (struct fb_videomode *)&pd->win_mode;

	width = dsim->pd->dsim_lcd_config->lcd_size.width;
	height = dsim->pd->dsim_lcd_config->lcd_size.height;

	/* in case of VIDEO MODE (RGB INTERFACE) */
	if (dsim->dsim_config->e_interface == (u32) DSIM_VIDEO) {
			s5p_mipi_dsi_set_main_disp_vporch(dsim,
				dsim->pd->dsim_lcd_config->rgb_timing.cmd_allow,
				dsim->pd->dsim_lcd_config->rgb_timing.stable_vfp,
				dsim->pd->dsim_lcd_config->rgb_timing.upper_margin);
			s5p_mipi_dsi_set_main_disp_hporch(dsim,
				dsim->pd->dsim_lcd_config->rgb_timing.right_margin,
				dsim->pd->dsim_lcd_config->rgb_timing.left_margin);
			s5p_mipi_dsi_set_main_disp_sync_area(dsim,
				dsim->pd->dsim_lcd_config->rgb_timing.vsync_len,
				dsim->pd->dsim_lcd_config->rgb_timing.hsync_len);
	}
	s5p_mipi_dsi_set_main_disp_resol(dsim, height, width);
	s5p_mipi_dsi_display_config(dsim);
	return 0;
}

static int s5p_mipi_dsi_init_link(struct mipi_dsim_device *dsim)
{
	unsigned int time_out = 100;
	unsigned int id;

	id = dsim->id;
	switch (dsim->state) {
	case DSIM_STATE_INIT:
		s5p_mipi_dsi_sw_reset(dsim);
		s5p_mipi_dsi_init_fifo_pointer(dsim, 0x1f);

		/* dsi configuration */
		s5p_mipi_dsi_init_config(dsim);
		s5p_mipi_dsi_enable_lane(dsim, DSIM_LANE_CLOCK, 1);
		s5p_mipi_dsi_enable_lane(dsim, dsim->data_lane, 1);

		/* set clock configuration */
		s5p_mipi_dsi_set_clock(dsim, dsim->dsim_config->e_byte_clk, 1);

		/* check clock and data lane state are stop state */
		while (!(s5p_mipi_dsi_is_lane_state(dsim))) {
			time_out--;
			if (time_out == 0) {
				dev_err(dsim->dev,
					"DSI Master is not stop state.\n");
				dev_err(dsim->dev,
					"Check initialization process\n");

				return -EINVAL;
			}
		}

		if (time_out != 0) {
			dev_dbg(dsim->dev,
				"DSI Master driver has been completed.\n");
			dev_dbg(dsim->dev, "DSI Master state is stop state\n");
		}

		dsim->state = DSIM_STATE_STOP;

		/* BTA sequence counters */
		s5p_mipi_dsi_set_stop_state_counter(dsim,
			dsim->dsim_config->stop_holding_cnt);
		s5p_mipi_dsi_set_bta_timeout(dsim,
			dsim->dsim_config->bta_timeout);
		s5p_mipi_dsi_set_lpdr_timeout(dsim,
			dsim->dsim_config->rx_timeout);

		return 0;
	default:
		dev_dbg(dsim->dev, "DSI Master is already init.\n");
		return 0;
	}

	return 0;
}

static int s5p_mipi_dsi_set_hs_enable(struct mipi_dsim_device *dsim)
{
	if (dsim->state == DSIM_STATE_STOP) {
		if (dsim->e_clk_src != DSIM_EXT_CLK_BYPASS) {
			dsim->state = DSIM_STATE_HSCLKEN;

			 /* set LCDC and CPU transfer mode to HS. */
			s5p_mipi_dsi_set_lcdc_transfer_mode(dsim, 0);
			s5p_mipi_dsi_set_cpu_transfer_mode(dsim, 0);

			s5p_mipi_dsi_enable_hs_clock(dsim, 1);

			return 0;
		} else
			dev_warn(dsim->dev,
				"clock source is external bypass.\n");
	} else
		dev_warn(dsim->dev, "DSIM is not stop state.\n");

	return 0;
}

static int s5p_mipi_dsi_set_data_transfer_mode(struct mipi_dsim_device *dsim,
		unsigned int mode)
{
	if (mode) {
		if (dsim->state != DSIM_STATE_HSCLKEN) {
			dev_err(dsim->dev, "HS Clock lane is not enabled.\n");
			return -EINVAL;
		}

		s5p_mipi_dsi_set_lcdc_transfer_mode(dsim, 0);
	} else {
		if (dsim->state == DSIM_STATE_INIT || dsim->state ==
			DSIM_STATE_ULPS) {
			dev_err(dsim->dev,
				"DSI Master is not STOP or HSDT state.\n");
			return -EINVAL;
		}

		s5p_mipi_dsi_set_cpu_transfer_mode(dsim, 0);
	}
	return 0;
}

#if 0
static int s5p_mipi_dsi_get_frame_done_status(struct mipi_dsim_device *dsim)
{
	return _s5p_mipi_dsi_get_frame_done_status(dsim);
}

static int s5p_mipi_dsi_clear_frame_done(struct mipi_dsim_device *dsim)
{
	_s5p_mipi_dsi_clear_frame_done(dsim);

	return 0;
}
#endif

static irqreturn_t s5p_mipi_dsi_interrupt_handler(int irq, void *dev_id)
{
	unsigned int int_src;
	struct mipi_dsim_device *dsim = dev_id;

	spin_lock(&dsim->slock);
	s5p_mipi_dsi_set_interrupt_mask(dsim, 0xffffffff, 1);

	int_src = readl(dsim->reg_base + S5P_DSIM_INTSRC);

	/* Test bit */
	if (int_src & SFR_PL_FIFO_EMPTY) {
		complete(&dsim_wr_pl_comp);
	}
	if (int_src & SFR_PH_FIFO_EMPTY) {
		complete(&dsim_wr_comp);
	}
	if (int_src & RX_DAT_DONE)
		complete(&dsim_rd_comp);
	if (int_src & ERR_RX_ECC)
		dev_err(dsim->dev, "RX ECC Multibit error was detected!\n");
#if defined(CONFIG_FB_VIDEO_PSR)
    if (int_src & INTSRC_FRAME_DONE){
        if(clk_gating_request)
        {
            clk_disable(clk_fimd); // SCLK_FIMD
            clk_disable(clk_axi_disp1); // ACLK_300
            clk_disable(clk_lcd); //LCD BUS CLK
            // IOVMM deactivate
            //iovmm_deactivate(dsim->pd->fimd1_device); 

            clk_disable(dsim->clock); // MIPI1 clock
            *clk_enabled=false;
            pr_info("%s gated\n", __func__);
        }
        mipi_framedone_status=1;
        clk_down_request=false;
        clk_gating_request=false;
    }
#endif
#if 0
    if(!(int_src & (RX_DAT_DONE|ERR_RX_ECC|INTSRC_FRAME_DONE)))
       dev_err(dsim->dev, "Other Int:%x\n",int_src);
#endif
	s5p_mipi_dsi_clear_interrupt(dsim, int_src);

	s5p_mipi_dsi_set_interrupt_mask(dsim, 0xfffffffe, 0);
	spin_unlock(&dsim->slock);

	return IRQ_HANDLED;
}

#ifdef CONFIG_PM
#ifdef CONFIG_HAS_EARLYSUSPEND
static void s5p_mipi_dsi_early_suspend(struct early_suspend *handler)
{
	struct mipi_dsim_device *dsim =
		container_of(handler, struct mipi_dsim_device, early_suspend);
	struct platform_device *pdev = to_platform_device(dsim->dev);

	mutex_lock(&dsim_lcd_lock);
	if (dsim->enabled == false) {
		mutex_unlock(&dsim_lcd_lock);
		return;
	}
	dsim->dsim_lcd_drv->suspend(dsim);
	dsim->state = DSIM_STATE_SUSPEND;
	s5p_mipi_dsi_d_phy_onoff(dsim, 0);
	if (clk_is_enabled(dsim->clock))
		clk_disable(dsim->clock);
	if (dsim->pd->mipi_power)
		dsim->pd->mipi_power(dsim, 0);
	pm_runtime_put_sync(&pdev->dev);
	dsim->dsim_lcd_drv->shutdown(dsim);
	dsim->enabled = false; // Above LCD Command need dsim control
	mutex_unlock(&dsim_lcd_lock);
}

static void s5p_mipi_dsi_late_resume(struct early_suspend *handler)
{
	struct mipi_dsim_device *dsim =
		container_of(handler, struct mipi_dsim_device, early_suspend);

	pr_info("+%s\n",__func__);
	mutex_lock(&dsim_lcd_lock);
	if (dsim->enabled == true) {
		mutex_unlock(&dsim_lcd_lock);
		return;
	}
	clk_enable(dsim->clock);
	pm_runtime_get_sync(dsim->dev);

	if (dsim->pd->mipi_power)
		dsim->pd->mipi_power(dsim, 1);
	if (dsim->dsim_lcd_drv->resume)
		dsim->dsim_lcd_drv->resume(dsim);
	s5p_mipi_dsi_init_dsim(dsim);
	s5p_mipi_dsi_init_link(dsim);
	dsim->enabled = true;
	s5p_mipi_dsi_enable_hs_clock(dsim, 1); //<< Required!
	s5p_mipi_dsi_set_cpu_transfer_mode(dsim, 1); // in Init Sequence LP
	s5p_mipi_dsi_set_display_mode(dsim, dsim->dsim_config);
	s5p_mipi_dsi_clear_int_status(dsim, INTSRC_SFR_FIFO_EMPTY);
	dsim->dsim_lcd_drv->read_id(dsim);
	dsim->dsim_lcd_drv->init_lcd(dsim);
	dsim->dsim_lcd_drv->displayon(dsim);
	s5p_mipi_dsi_set_cpu_transfer_mode(dsim, 0); // back
	pr_info("-%s\n",__func__);
	mutex_unlock(&dsim_lcd_lock);
}
#else
static int s5p_mipi_dsi_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mipi_dsim_device *dsim = platform_get_drvdata(pdev);

	if (!s5p_mipi_en_state(dsim)) {
		s5p_mipi_force_enable(dsim);
		dev_warn(dsim->dev, "%s: MIPI state check!!\n", __func__);
	} else
		dev_dbg(dsim->dev, "MIPI state is valied.\n");

	dev_dbg(dsim->dev, "dsim enabled?:%d\n",dsim->enabled);

	if (dsim->enabled == false)
		return 0;
	dsim->dsim_lcd_drv->suspend(dsim);
	dsim->dsim_lcd_drv->shutdown(dsim);

	dsim->state = DSIM_STATE_SUSPEND;
	s5p_mipi_dsi_d_phy_onoff(dsim, 0);
	if (dsim->pd->mipi_power)
		dsim->pd->mipi_power(dsim, 0);
	pm_runtime_put_sync(dev);
	if (clk_is_enabled(dsim->clock))
	    clk_disable(dsim->clock);
	dsim->enabled = false;
	pr_info("-%s\n",__func__);
	return 0;
}

static int s5p_mipi_dsi_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mipi_dsim_device *dsim = platform_get_drvdata(pdev);

	if (dsim->enabled == true)
		return 0;

	pm_runtime_get_sync(&pdev->dev);
	clk_enable(dsim->clock);

	if (dsim->pd->mipi_power)
		dsim->pd->mipi_power(dsim, 1);
	if (dsim->dsim_lcd_drv->resume)
		dsim->dsim_lcd_drv->resume(dsim);
	s5p_mipi_dsi_init_dsim(dsim);
	s5p_mipi_dsi_init_link(dsim);
	dsim->enabled = true; //<- this is required for LCD command writing.
#if defined(CONFIG_LCD_MIPI_MEIZU) //A
	s5p_mipi_dsi_enable_hs_clock(dsim, 1); //<< Required!
	s5p_mipi_dsi_set_cpu_transfer_mode(dsim, 1); // in Init Sequence LP
	s5p_mipi_dsi_set_display_mode(dsim, dsim->dsim_config);
	s5p_mipi_dsi_clear_int_status(dsim, INTSRC_SFR_FIFO_EMPTY);
	dsim->dsim_lcd_drv->read_id(dsim);
	dsim->dsim_lcd_drv->init_lcd(dsim);
	dsim->dsim_lcd_drv->displayon(dsim);
	s5p_mipi_dsi_set_cpu_transfer_mode(dsim, 0);
    //s5p_mipi_dsi_reg_dump(dsim);
#else
	s5p_mipi_dsi_set_data_transfer_mode(dsim, 0);
	s5p_mipi_dsi_set_display_mode(dsim, dsim->dsim_config);
	s5p_mipi_dsi_set_hs_enable(dsim);
	dsim->dsim_lcd_drv->displayon(dsim);
#endif
	pr_info("-%s\n",__func__);

	return 0;
}
#endif
static int s5p_mipi_dsi_runtime_suspend(struct device *dev)
{
	return 0;
}

static int s5p_mipi_dsi_runtime_resume(struct device *dev)
{
	return 0;
}
#else
#define s5p_mipi_dsi_suspend NULL
#define s5p_mipi_dsi_resume NULL
#define s5p_mipi_dsi_runtime_suspend NULL
#define s5p_mipi_dsi_runtime_resume NULL
#endif

static int s5p_mipi_dsi_enable(struct mipi_dsim_device *dsim)
{
	struct platform_device *pdev = to_platform_device(dsim->dev);

	pm_runtime_get_sync(&pdev->dev);
	clk_enable(dsim->clock);

	if (dsim->pd->mipi_power)
		dsim->pd->mipi_power(dsim, 1);
	if (dsim->dsim_lcd_drv->resume)
		dsim->dsim_lcd_drv->resume(dsim);
	s5p_mipi_dsi_init_dsim(dsim);
	s5p_mipi_dsi_init_link(dsim);
	dsim->enabled = true; //<- this is required for LCD command writing.
#if defined(CONFIG_LCD_MIPI_MEIZU) //A
	s5p_mipi_dsi_enable_hs_clock(dsim, 1); //<< Required!
	s5p_mipi_dsi_set_cpu_transfer_mode(dsim, 1); // in Init Sequence LP
	s5p_mipi_dsi_set_display_mode(dsim, dsim->dsim_config);
	s5p_mipi_dsi_clear_int_status(dsim, INTSRC_SFR_FIFO_EMPTY);
	dsim->dsim_lcd_drv->read_id(dsim);
	dsim->dsim_lcd_drv->init_lcd(dsim);
	dsim->dsim_lcd_drv->displayon(dsim);
	s5p_mipi_dsi_set_cpu_transfer_mode(dsim, 0);
//    s5p_mipi_dsi_reg_dump(dsim);
#else
	s5p_mipi_dsi_set_data_transfer_mode(dsim, 0);
	s5p_mipi_dsi_set_display_mode(dsim, dsim->dsim_config);
	s5p_mipi_dsi_set_hs_enable(dsim);

	usleep_range(18000, 18000);
	
	dsim->dsim_lcd_drv->displayon(dsim);
#endif

	return 0;
}

int s5p_mipi_dsi_enable_by_fimd(struct device *dsim_device)
{
	struct platform_device *pdev = to_platform_device(dsim_device);
	struct mipi_dsim_device *dsim = platform_get_drvdata(pdev);

	if (dsim->enabled == true)
		return 0;

	s5p_mipi_dsi_enable(dsim);
    pr_info("Mipi Enabled\n");
	return 0;
}

int s5p_mipi_dsi_clk_enable_by_fimd(struct device *dsim_device)
{
	struct platform_device *pdev = to_platform_device(dsim_device);
	struct mipi_dsim_device *dsim = platform_get_drvdata(pdev);

	if (dsim->enabled == false)
		return 0;

	clk_enable(dsim->clock);

	return 0;
}

static int s5p_mipi_dsi_disable(struct mipi_dsim_device *dsim)
{
	struct platform_device *pdev = to_platform_device(dsim->dev);

	if (!s5p_mipi_en_state(dsim)) {
		s5p_mipi_force_enable(dsim);
		dev_warn(dsim->dev, "%s: MIPI state check!!\n", __func__);
	} else
		dev_dbg(dsim->dev, "MIPI state is valied.\n");

	dev_dbg(dsim->dev, "dsim enabled?:%d\n",dsim->enabled);

	if (dsim->enabled == false)
		return 0;

	dsim->dsim_lcd_drv->suspend(dsim);
	dsim->state = DSIM_STATE_SUSPEND;
	s5p_mipi_dsi_d_phy_onoff(dsim, 0);


	if (dsim->pd->mipi_power){
		dsim->pd->mipi_power(dsim, 0);
	}
	pm_runtime_put_sync(&pdev->dev);

	clk_disable(dsim->clock);
	dsim->enabled = false;
	pr_info("Mipi disabled\n");

	return 0;
}

int s5p_mipi_dsi_disable_by_fimd(struct device *dsim_device)
{
	struct platform_device *pdev = to_platform_device(dsim_device);
	struct mipi_dsim_device *dsim = platform_get_drvdata(pdev);

	if (dsim->enabled == false)
		return 0;

	s5p_mipi_dsi_disable(dsim);
	return 0;
}

int s5p_mipi_dsi_clk_disable_by_fimd(struct device *dsim_device)
{
	struct platform_device *pdev = to_platform_device(dsim_device);
	struct mipi_dsim_device *dsim = platform_get_drvdata(pdev);

	if (dsim->enabled == false)
		return 0;

	clk_disable(dsim->clock);

	return 0;
}

static int s5p_dsim_reboot_event(struct notifier_block *this,
		unsigned long code, void *unused)
{
	struct mipi_dsim_device *dsim = container_of(this, struct mipi_dsim_device,
			reboot_notifier);

	if (dsim->enabled) {
		dsim->dsim_lcd_drv->suspend(dsim);
		dsim->dsim_lcd_drv->shutdown(dsim);
	}

	pr_info("REBOOT Notifier for MIPI DSIM\n");
	return NOTIFY_DONE;
}
extern void s3c_fb_disable_global(void);
extern void s3c_fb_enable_global(void);
static int s5p_mipi_dsi_fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	int blank = 0;
	struct fb_event *evdata = data;
	struct mipi_dsim_device *dsim = container_of(self, struct mipi_dsim_device, fb_notif);
	struct platform_device *pdev = to_platform_device(dsim->dev);
	blank = *(int*)(evdata->data);

	mutex_lock(&dsim_lcd_lock);
	switch (blank) {
	case FB_BLANK_POWERDOWN:
		{
			if (!dsim->enabled)
				goto out;
			pr_info("+%s powerdown\n",__func__);

			dsim->dsim_lcd_drv->suspend(dsim);
			dsim->state = DSIM_STATE_SUSPEND;
			s5p_mipi_dsi_d_phy_onoff(dsim, 0);
			if (clk_is_enabled(dsim->clock))
				clk_disable(dsim->clock);
			if (dsim->pd->mipi_power)
				dsim->pd->mipi_power(dsim, 0);
			pm_runtime_put_sync(&pdev->dev);
			dsim->dsim_lcd_drv->shutdown(dsim);
			dsim->enabled = false; // Above LCD Command need dsim control
			s3c_fb_disable_global();
#ifdef CONFIG_ARM_EXYNOS5410_BUS_DEVFREQ
			pm_qos_update_request(g_exynos5_mif_qos, 0);
			pm_qos_update_request(g_exynos5_int_qos, 0);
#endif
			pr_info("- %spowerdown\n",__func__);
		}
		break;
	case FB_BLANK_UNBLANK:
		{
			if (dsim->enabled)
				goto out;

#ifdef CONFIG_ARM_EXYNOS5410_BUS_DEVFREQ
			pm_qos_update_request(g_exynos5_mif_qos, 400000);
			pm_qos_update_request(g_exynos5_int_qos, 267000);
#endif
			pr_info("+%s unblank\n",__func__);
			clk_enable(dsim->clock);
			pm_runtime_get_sync(dsim->dev);

			if (dsim->pd->mipi_power)
				dsim->pd->mipi_power(dsim, 1);
			if (dsim->dsim_lcd_drv->resume)
				dsim->dsim_lcd_drv->resume(dsim);
			s5p_mipi_dsi_init_dsim(dsim);
			s5p_mipi_dsi_init_link(dsim);
			dsim->enabled = true;
			s5p_mipi_dsi_enable_hs_clock(dsim, 1); //<< Required!
			s5p_mipi_dsi_set_cpu_transfer_mode(dsim, 1); // in Init Sequence LP
			s5p_mipi_dsi_set_display_mode(dsim, dsim->dsim_config);
			s5p_mipi_dsi_clear_int_status(dsim, INTSRC_SFR_FIFO_EMPTY);
			dsim->dsim_lcd_drv->read_id(dsim);
			dsim->dsim_lcd_drv->init_lcd(dsim);
			dsim->dsim_lcd_drv->displayon(dsim);
			s5p_mipi_dsi_set_cpu_transfer_mode(dsim, 0); // back
#ifdef CONFIG_ARM_EXYNOS5410_BUS_DEVFREQ
			pm_qos_update_request(g_exynos5_mif_qos, 0);
			pm_qos_update_request(g_exynos5_int_qos, 0);
#endif
			pr_info("-%s unblank\n",__func__);
		}
		break;
	}
out:
	mutex_unlock(&dsim_lcd_lock);
	return NOTIFY_OK;
}

static int s5p_mipi_dsi_register_fb(struct mipi_dsim_device *dsim)
{
	memset(&dsim->fb_notif, 0, sizeof(dsim->fb_notif));
	dsim->fb_notif.notifier_call = s5p_mipi_dsi_fb_notifier_callback;

	return fb_register_client(&dsim->fb_notif);
}
static int s5p_mipi_dsi_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct mipi_dsim_device *dsim = NULL;
	struct mipi_dsim_config *dsim_config;
	struct s5p_platform_mipi_dsim *dsim_pd;
	int ret = -1;

	if (!dsim)
		dsim = kzalloc(sizeof(struct mipi_dsim_device),
			GFP_KERNEL);
	if (!dsim) {
		dev_err(&pdev->dev, "failed to allocate dsim object.\n");
		return -ENOMEM;
	}

	dsim->pd = to_dsim_plat(&pdev->dev);
	dsim->dev = &pdev->dev;
	dsim->id = pdev->id;

	spin_lock_init(&dsim->slock);

	ret = s5p_mipi_dsi_register_fb(dsim);
	if (ret) {
		dev_err(&pdev->dev, "failed to register fb notifier chain\n");
		kfree(dsim);
		return -EFAULT;
	}

	pm_runtime_enable(&pdev->dev);

	/* get s5p_platform_mipi_dsim. */
	dsim_pd = (struct s5p_platform_mipi_dsim *)dsim->pd;
	/* get mipi_dsim_config. */
	dsim_config = dsim_pd->dsim_config;
	dsim->dsim_config = dsim_config;

	if (dsim->dsim_config == NULL) {
		dev_err(&pdev->dev, "dsim_config is NULL.\n");
		ret = -EINVAL;
		goto err_dsim_config;
	}

	dsim->clock = clk_get(&pdev->dev, dsim->pd->clk_name);
	if (IS_ERR(dsim->clock)) {
		dev_err(&pdev->dev, "failed to get dsim clock source\n");
		goto err_clock_get;
	}
	clk_enable(dsim->clock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to get io memory region\n");
		ret = -EINVAL;
		goto err_platform_get;
	}
	res = request_mem_region(res->start, resource_size(res),
					dev_name(&pdev->dev));
	if (!res) {
		dev_err(&pdev->dev, "failed to request io memory region\n");
		ret = -EINVAL;
		goto err_mem_region;
	}

	dsim->res = res;
	dsim->reg_base = ioremap(res->start, resource_size(res));
	if (!dsim->reg_base) {
		dev_err(&pdev->dev, "failed to remap io region\n");
		ret = -EINVAL;
		goto err_mem_region;
	}
	mipi_reg = (unsigned int)dsim->reg_base;

	/*
	 * it uses frame done interrupt handler
	 * only in case of MIPI Video mode.
	 */
	if (dsim->pd->dsim_config->e_interface == DSIM_VIDEO
		|| dsim->pd->dsim_config->e_interface == DSIM_COMMAND) {
		dsim->irq = platform_get_irq(pdev, 0);
		if (request_irq(dsim->irq, s5p_mipi_dsi_interrupt_handler,
				IRQF_DISABLED, "mipi-dsi", dsim)) {
			dev_err(&pdev->dev, "request_irq failed.\n");
			ret = -EINVAL;
			goto err_irq;
		}
	}

	dsim->dsim_lcd_drv = dsim->dsim_config->dsim_ddi_pd;

	if (dsim->dsim_config == NULL) {
		dev_err(&pdev->dev, "dsim_config is NULL.\n");
		goto err_dsim_config;
	}
    
	mutex_init(&dsim_rd_wr_mutex);
	mutex_init(&dsim_lcd_lock);
	platform_set_drvdata(pdev, dsim);
	pm_runtime_get_sync(&pdev->dev);

	s5p_mipi_dsi_init_dsim(dsim);
	s5p_mipi_dsi_init_link(dsim);
	dsim->dsim_lcd_drv->probe(dsim);
	dsim->dsim_lcd_drv->resume(dsim);
	dsim->enabled = true;
#if defined(CONFIG_LCD_MIPI_MEIZU)
	s5p_mipi_dsi_enable_hs_clock(dsim, 1); //<< Required!
	s5p_mipi_dsi_set_cpu_transfer_mode(dsim, 1); // in Init Sequence LP
	s5p_mipi_dsi_set_display_mode(dsim, dsim->dsim_config);
	s5p_mipi_dsi_clear_int_status(dsim, INTSRC_SFR_FIFO_EMPTY);
	dsim->dsim_lcd_drv->read_id(dsim);
	dsim->dsim_lcd_drv->init_lcd(dsim);
	dsim->dsim_lcd_drv->displayon(dsim);
	s5p_mipi_dsi_set_cpu_transfer_mode(dsim, 0); // back to HS, should be after displayon
//    s5p_mipi_dsi_reg_dump(dsim);
#else
	s5p_mipi_dsi_set_hs_enable(dsim);
	s5p_mipi_dsi_set_display_mode(dsim, dsim->dsim_config);
	dsim->dsim_lcd_drv->read_id(dsim);
	dsim->dsim_lcd_drv->init_lcd(dsim);
	dsim->dsim_lcd_drv->displayon(dsim);
#endif
	dev_info(&pdev->dev, "mipi-dsi driver(%s mode) has been probed.\n",
		(dsim_config->e_interface == DSIM_COMMAND) ?
			"CPU" : "RGB");

#ifdef CONFIG_HAS_EARLYSUSPEND
	dsim->early_suspend.suspend = s5p_mipi_dsi_early_suspend;
	dsim->early_suspend.resume = s5p_mipi_dsi_late_resume;
	dsim->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
//	register_early_suspend(&(dsim->early_suspend));
#endif
	dsim->reboot_notifier.notifier_call = s5p_dsim_reboot_event;
	dsim->reboot_notifier.priority	= INT_MAX;
	if(register_reboot_notifier(&dsim->reboot_notifier))
		pr_err("register notifier call failed!\n");

	return 0;

#if defined(BOOT_LCD_INIT)
err_lcd_probe:
#endif
	if (dsim->pd->dsim_config->e_interface == DSIM_VIDEO
		|| dsim->pd->dsim_config->e_interface == DSIM_COMMAND)
		free_irq(dsim->irq, dsim);
err_irq:
	release_resource(dsim->res);
	kfree(dsim->res);

	iounmap((void __iomem *) dsim->reg_base);

err_mem_region:
err_platform_get:
	clk_disable(dsim->clock);
	clk_put(dsim->clock);

err_dsim_config:
err_clock_get:
	kfree(dsim);
	pm_runtime_put_sync(&pdev->dev);
	return ret;

}

static int __devexit s5p_mipi_dsi_remove(struct platform_device *pdev)
{
	struct mipi_dsim_device *dsim = platform_get_drvdata(pdev);

	if (dsim->dsim_config->e_interface == DSIM_VIDEO
		|| dsim->dsim_config->e_interface == DSIM_COMMAND)
		free_irq(dsim->irq, dsim);

	iounmap(dsim->reg_base);

	clk_disable(dsim->clock);
	clk_put(dsim->clock);

	release_resource(dsim->res);
	kfree(dsim->res);

	kfree(dsim);

	return 0;
}

static void s5p_mipi_dsi_shutdown(struct platform_device *pdev)
{
	struct mipi_dsim_device *dsim = platform_get_drvdata(pdev);

	if (dsim->enabled != false) {
		dsim->dsim_lcd_drv->suspend(dsim);
		dsim->state = DSIM_STATE_SUSPEND;

		s5p_mipi_dsi_set_interrupt_mask(dsim, 0xffffffff, 1);
		s5p_mipi_dsi_clear_interrupt(dsim, 0xffffffff);

		s5p_mipi_dsi_d_phy_onoff(dsim, 0);
		if (dsim->pd->mipi_power)
			dsim->pd->mipi_power(dsim, 0);
		pm_runtime_put_sync(&pdev->dev);
		clk_disable(dsim->clock);
		dsim->enabled = false;
	}
}

static const struct dev_pm_ops mipi_dsi_pm_ops = {
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = s5p_mipi_dsi_suspend,
	.resume = s5p_mipi_dsi_resume,
#endif
	.runtime_suspend	= s5p_mipi_dsi_runtime_suspend,
	.runtime_resume		= s5p_mipi_dsi_runtime_resume,
};

static struct platform_driver s5p_mipi_dsi_driver = {
	.probe = s5p_mipi_dsi_probe,
	.remove = __devexit_p(s5p_mipi_dsi_remove),
	.shutdown = s5p_mipi_dsi_shutdown,
	.driver = {
		   .name = "s5p-mipi-dsim",
		   .owner = THIS_MODULE,
		   .pm = &mipi_dsi_pm_ops,
	},
};

static int s5p_mipi_dsi_register(void)
{
	platform_driver_register(&s5p_mipi_dsi_driver);

	return 0;
}

static void s5p_mipi_dsi_unregister(void)
{
	platform_driver_unregister(&s5p_mipi_dsi_driver);
}
module_init(s5p_mipi_dsi_register);
module_exit(s5p_mipi_dsi_unregister);

MODULE_AUTHOR("InKi Dae <inki.dae@samsung.com>");
MODULE_DESCRIPTION("Samusung MIPI-DSI driver");
MODULE_LICENSE("GPL");
