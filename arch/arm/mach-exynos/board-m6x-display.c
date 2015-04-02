/*
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/gpio.h>
#include <linux/pwm_backlight.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/i2c-gpio.h>

#include <video/platform_lcd.h>
#include <video/s5p-dp.h>

#include <plat/cpu.h>
#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/fb.h>
#include <plat/fb-core.h>
#include <plat/regs-fb-v4.h>
#include <plat/dp.h>
#include <plat/backlight.h>
#include <plat/gpio-cfg.h>
#include <linux/i2c-gpio.h>
#ifdef CONFIG_BACKLIGHT_LM3630
#include <linux/platform_data/lm3630_bl.h>
#endif
#ifdef CONFIG_BACKLIGHT_LM3695
#include <linux/platform_data/lm3695.h>
#endif
#ifdef CONFIG_TPS65132
#include <linux/platform_data/tps65132.h>
#endif

#include <mach/map.h>
#include <mach/gpio-meizu.h>
#include <mach/gpio-i2c-m6x.h>

#ifdef CONFIG_FB_MIPI_DSIM
#include <plat/dsim.h>
#include <plat/mipi_dsi.h>
#endif
#include <linux/regulator/consumer.h>

#ifdef CONFIG_FB_MIPI_DSIM
#if defined(CONFIG_LCD_MIPI_MEIZU)
static struct s3c_fb_pd_win m6x_fb_win0 = {
	.win_mode = {
#if defined(CONFIG_FB_VIDEO_PSR)
#if defined(CONFIG_LCD_64HZ)
		.left_margin	= 45,
		.right_margin	= 100,
		.upper_margin	= 4,
		.lower_margin	= 4,
		.hsync_len	= 2,
		.vsync_len	= 4,
#else
		.left_margin	= 42,
		.right_margin	= 100,
		.upper_margin	= 7,
		.lower_margin	= 4,
		.hsync_len	= 4,
		.vsync_len	= 2,
#endif
#else  // Normal Video Mode
		.left_margin	= 42,//92,
		.right_margin	= 100,
		.upper_margin	= 8,//4,
		.lower_margin	= 4,//16,
		.hsync_len	= 4,
		.vsync_len	= 4,//2,
#endif
		.xres		= 1080,
		.yres		= 1800,
// Command mode settings
		.cs_setup_time	= 1,
		.wr_setup_time	= 0,
		.wr_act_time	= 1,
		.wr_hold_time	= 0,
		.rs_pol		= 0,
		.i80en		= 1,
	},
	.virtual_x		= 1080,

	.virtual_y		= 1800 * 2,
	.width			= 66,
	.height			= 110,
	.max_bpp		= 32,
	.default_bpp		= 24,
};
#endif
#endif  /* CONFIG_FB_MIPI_DSIM  */

#ifdef CONFIG_S5P_DP
static void dp_lcd_set_power(struct plat_lcd_data *pd,
				unsigned int power)
{
	printk("## %s, %d\n", __func__, __LINE__);

#ifndef CONFIG_BACKLIGHT_PWM
	/* LCD_PWM_IN_2.8V: LCD_B_PWM, GPB2_0 */
	gpio_request(EXYNOS5410_GPB2(0), "GPB2");
#endif

	gpio_request(EXYNOS5410_GPJ0(0), "GPJ0");
	if(power)
		gpio_direction_output(EXYNOS5410_GPJ0(0), 1);

#ifndef CONFIG_BACKLIGHT_PWM
	/* LCD_PWM_IN_2.8V: LCD_B_PWM, GPB2_0 */
	gpio_direction_output(EXYNOS5410_GPB2(0), power);

	gpio_free(EXYNOS5410_GPB2(0));
#endif

	gpio_free(EXYNOS5410_GPJ0(0));
}

static struct plat_lcd_data smdk5410_dp_lcd_data = {
	.set_power	= dp_lcd_set_power,
};

static struct platform_device smdk5410_dp_lcd = {
	.name	= "platform-lcd",
	.dev	= {
#ifdef CONFIG_EXYNOS_SETUP_FIMD0
		.parent		= &s5p_device_fimd0.dev,
#else
		.parent		= &s5p_device_fimd1.dev,
#endif
		.platform_data	= &smdk5410_dp_lcd_data,
	},
};

static struct s3c_fb_pd_win smdk5410_fb_win0 = {
	.win_mode = {
		.left_margin	= 80,
		.right_margin	= 48,
		.upper_margin	= 37,
		.lower_margin	= 3,
		.hsync_len	= 32,
		.vsync_len	= 6,
		.xres		= 2560,
		.yres		= 1600,
	},
	.virtual_x		= 2560,
	.virtual_y		= 1640 * 2,
	.max_bpp		= 32,
	.default_bpp		= 24,
};

static struct s3c_fb_pd_win smdk5410_fb_win1 = {
	.win_mode = {
		.left_margin	= 80,
		.right_margin	= 48,
		.upper_margin	= 37,
		.lower_margin	= 3,
		.hsync_len	= 32,
		.vsync_len	= 6,
		.xres		= 2560,
		.yres		= 1600,
	},
	.virtual_x		= 2560,
	.virtual_y		= 1640 * 2,
	.max_bpp		= 32,
	.default_bpp		= 24,
};

static struct s3c_fb_pd_win smdk5410_fb_win2 = {
	.win_mode = {
		.left_margin	= 80,
		.right_margin	= 48,
		.upper_margin	= 37,
		.lower_margin	= 3,
		.hsync_len	= 32,
		.vsync_len	= 6,
		.xres		= 2560,
		.yres		= 1600,
	},
	.virtual_x		= 2560,
	.virtual_y		= 1600 * 2,
	.max_bpp		= 32,
	.default_bpp		= 24,
};
#endif  /* CONFIG_S5P_DP */

#if defined(CONFIG_BACKLIGHT_LM3630)
static struct lm3630_platform_data bl_lm3630 = {
	.max_brt_led1 = 255,
	.max_brt_led2 = 255,
	.init_brt_led1 = 200,
	.init_brt_led2 = 200,
	.max_cur_a = 0x14,
	.max_cur_b = 0x14,
	.pwm_ctrl = PWM_CTRL_DISABLE,
	.pwm_active = PWM_ACTIVE_HIGH,
	.bank_a_ctrl = BANK_A_CTRL_ALL,
	.bank_b_ctrl = BANK_B_CTRL_DISABLE,
	.pwm_period = 0,
};
#endif
#if defined(CONFIG_BACKLIGHT_LM3695)
static struct lm3695_platform_data bl_lm3695 = {
	.name = "lm3630_bled",
	.initial_brightness = 1536,
	.boost_freq = LM3695_BOOST_FREQ_500KHZ,
	.ovp = LM3695_OVP_21V,
	.string = LM3695_DUAL_STRINGS,
	.ramp = LM3695_RAMP_125us,
};
#endif
#ifdef CONFIG_TPS65132
static struct tps65132_pd_data tps_pd = {
	.vpos_val = 0xb,
	.vneg_val = 0xb,
	.app_val  = 0,
	.disn_val = 1,
	.disp_val = 1,
	.stored_en = 0,
};
#endif
static struct i2c_board_info __initdata i2c_bl[]  = {
#if defined(CONFIG_BACKLIGHT_LM3630)
	{
		I2C_BOARD_INFO("bl-lm3630", 0x36),
		.platform_data = &bl_lm3630,
	},
#endif
#if defined(CONFIG_BACKLIGHT_LM3695)
	{
		I2C_BOARD_INFO("bl-lm3695", 0x63),
		.platform_data = &bl_lm3695,
	},
#endif
};
static struct i2c_board_info __initdata i2c_tps[]  = {
	{
		I2C_BOARD_INFO("tps-vol", 0x3e),
		.platform_data = &tps_pd,
	},
};
static void exynos_fimd_gpio_setup_24bpp(void)
{
	unsigned int reg = 0;

#if defined(CONFIG_S5P_DP)
	/* Set Hotplug detect for DP */
	gpio_request(EXYNOS5410_GPX0(7), "GPX0");
	s3c_gpio_cfgpin(EXYNOS5410_GPX0(7), S3C_GPIO_SFN(3));
#endif
#if defined(CONFIG_FB_VIDEO_PSR)
	// HW Trigger
	gpio_request(EXYNOS5410_GPJ4(0), "GPJ4");
	s3c_gpio_cfgpin(EXYNOS5410_GPJ4(0), S3C_GPIO_SFN(2));
	gpio_free(EXYNOS5410_GPJ4(0));
	gpio_request(MEIZU_LCD_TE, "GPJ1");
	s3c_gpio_cfgpin(MEIZU_LCD_TE, S3C_GPIO_SFN(0Xf));
	gpio_free(MEIZU_LCD_TE);
#endif
	/*
	 * Set DISP1BLK_CFG register for Display path selection
	 *
	 * FIMD of DISP1_BLK Bypass selection : DISP1BLK_CFG[15]
	 * ---------------------
	 *  0 | MIE/MDNIE
	 *  1 | FIMD : selected
	 * 
	 * Video Type Selection : DISPxBLK_CFG[24]
	 * 00 = RGB interface
	 * 01 = i80 interface
	 */
#if defined(CONFIG_S5P_DEV_FIMD0)

	__raw_writel(0, S3C_VA_SYS + 0x0210);
	reg = __raw_readl(S3C_VA_SYS + 0x0210);
	reg &= ~(1 << 15);	/* To save other reset values */
	reg |= (1 << 15);
	__raw_writel(reg, S3C_VA_SYS + 0x0210);
#else
	reg = __raw_readl(S3C_VA_SYS + 0x0214);
	reg &= ~(1 << 15);	/* To save other reset values */
	reg |= (1 << 15);
	__raw_writel(reg, S3C_VA_SYS + 0x0214);
#endif
#if defined(CONFIG_S5P_DP)
	/* Reference clcok selection for DPTX_PHY: PAD_OSC_IN */
	reg = __raw_readl(S3C_VA_SYS + 0x04d4);
	reg &= ~(1 << 0);
	__raw_writel(reg, S3C_VA_SYS + 0x04d4);

	/* DPTX_PHY: XXTI */
	reg = __raw_readl(S3C_VA_SYS + 0x04d8);
	reg &= ~(1 << 3);
	__raw_writel(reg, S3C_VA_SYS + 0x04d8);
#endif
}
#ifdef CONFIG_EXYNOS_SETUP_FIMD0
static struct s3c_fb_platdata m6x_lcd0_pdata __initdata = {
	.win[0]		= &m6x_fb_win0,
	.win[1]		= &m6x_fb_win0,
	.win[2]		= &m6x_fb_win0,
	.win[3]		= &m6x_fb_win0,
	.win[4]		= &m6x_fb_win0,
	.default_win	= 0,
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB,
	.vidcon1	= VIDCON1_INV_VCLK,
	.setup_gpio	= exynos_fimd_gpio_setup_24bpp,
	.ip_version	= EXYNOS5_813,
};

#else
static struct s3c_fb_platdata m6x_lcd1_pdata __initdata = {
	.win[0]		= &m6x_fb_win0,
	.win[1]		= &m6x_fb_win0,
	.win[2]		= &m6x_fb_win0,
	.win[3]		= &m6x_fb_win0,
	.win[4]		= &m6x_fb_win0,
	.default_win	= 0,
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB,
#if defined(CONFIG_S5P_DP)
	.vidcon1	= 0,
#else
	.vidcon1	= VIDCON1_INV_VCLK,
#endif
	.setup_gpio	= exynos_fimd_gpio_setup_24bpp,
	.ip_version	= EXYNOS5_813,

	.dsim_on	= s5p_mipi_dsi_enable_by_fimd,
	.dsim_off	= s5p_mipi_dsi_disable_by_fimd,
	.dsim_clk_on	= s5p_mipi_dsi_clk_enable_by_fimd,
	.dsim_clk_off	= s5p_mipi_dsi_clk_disable_by_fimd,

	.dsim1_device   = &s5p_device_mipi_dsim1.dev,
	.dp_exit        = s5p_dp_phy_exit,
};
#endif

#ifdef CONFIG_FB_MIPI_DSIM
#if defined(CONFIG_LCD_MIPI_MEIZU)
static struct mipi_dsim_config dsim_info = {
	.e_interface	= DSIM_VIDEO,
	.e_pixel_format	= DSIM_24BPP_888,

	.eot_disable	= false,

	/* main frame fifo auto flush at VSYNC pulse */
	// video mode parameters
	.auto_flush	= false,
	.auto_vertical_cnt = false,
	.hse = false,
	.hfp = false,
	.hbp = false,
	.hsa = true,

	.e_no_data_lane	= DSIM_DATA_LANE_4,
	.e_byte_clk	= DSIM_PLL_OUT_DIV8,
#if defined(CONFIG_FB_VIDEO_PSR)
	.e_burst_mode	= DSIM_NON_BURST_SYNC_EVENT,
#else
	.e_burst_mode	= DSIM_BURST,
#endif

	.p = 4,
#if defined(CONFIG_FB_VIDEO_PSR)
	.m = 75, // 85 with 980Mbps-PHY Timing is Byte SWAPPED
#else
	.m = 79,
#endif
	.s = 1,


	/* D-PHY PLL stable time spec :min = 200usec ~ max 400usec */
	.pll_stable_time = 400,		//500

	.esc_clk = 20 * 1000000,	//0.4	/* escape clk : 10MHz */

	/* stop state holding counter after bta change count 0 ~ 0xfff */
	.stop_holding_cnt	= 0xfff, 	//0x0f
	.bta_timeout		= 0xff,		/* bta timeout 0 ~ 0xff */
	.rx_timeout		= 0xffff,	/* lp rx timeout 0 ~ 0xffff */

	.dsim_ddi_pd = &meizu_mipi_lcd_driver,
};

static struct mipi_dsim_lcd_config dsim_lcd_info = {
	.rgb_timing.left_margin		= 42,//92,//42,
	.rgb_timing.right_margin	= 100,//110,//100,
#if defined(CONFIG_LCD_64HZ)
	.rgb_timing.upper_margin	= 4,
	.rgb_timing.lower_margin	= 4,
	.rgb_timing.hsync_len		= 2,
	.rgb_timing.vsync_len		= 4,
#else
	.rgb_timing.upper_margin	= 7,
	.rgb_timing.lower_margin	= 4,
	.rgb_timing.hsync_len		= 4,
	.rgb_timing.vsync_len		= 2,
#endif
	.rgb_timing.cmd_allow		= 0xf,
	.cpu_timing.cs_setup		= 1,
	.cpu_timing.wr_setup		= 0,
	.cpu_timing.wr_act		= 1,
	.cpu_timing.wr_hold		= 0,
	.lcd_size.width			= 1080,
	.lcd_size.height		= 1800,
};
#endif

static struct s5p_platform_mipi_dsim dsim_platform_data = {
#ifdef CONFIG_S5P_DEV_MIPI_DSIM0
	.clk_name		= "dsim0",
#else
	.clk_name		= "dsim1",
#endif
	.dsim_config		= &dsim_info,
	.dsim_lcd_config	= &dsim_lcd_info,

	.part_reset		= s5p_dsim_part_reset,
	.init_d_phy		= s5p_dsim_init_d_phy,
	.get_fb_frame_done	= NULL,
	.trigger		= NULL,

	/*
	 * The stable time of needing to write data on SFR
	 * when the mipi mode becomes LP mode.
	 */
	.delay_for_stabilization = 600,
	.fimd1_device    = &s5p_device_fimd1.dev,
};

#endif  /* CONFIG_FB_MIPI_DSIM */

#ifdef CONFIG_S5P_DP
static struct video_info smdk5410_dp_config = {
	.name			= "WQXGA(2560x1600) LCD, for SMDK TEST",

	.h_sync_polarity	= 0,
	.v_sync_polarity	= 0,
	.interlaced		= 0,

	.color_space		= COLOR_RGB,
	.dynamic_range		= VESA,
	.ycbcr_coeff		= COLOR_YCBCR601,
	.color_depth		= COLOR_8,

	.link_rate		= LINK_RATE_2_70GBPS,
	.lane_count		= LANE_COUNT4,
};

static void s5p_dp_backlight_on(void)
{
	printk("## %s, %d\n", __func__, __LINE__);
	/* EDP_BL_EN: GPK1_0 */
	gpio_request(EXYNOS5410_GPK1(0), "GPK1");
	gpio_direction_output(EXYNOS5410_GPK1(0), 1);
	msleep(20);

	gpio_free(EXYNOS5410_GPK1(0));
}

static void s5p_dp_backlight_off(void)
{
	printk("s5p_dp_backlight_off\n");
	/* EDP_BL_EN: GPK1_0 */
	gpio_request(EXYNOS5410_GPK1(0), "GPK1");

	gpio_direction_output(EXYNOS5410_GPK1(0), 0);
	msleep(20);

	gpio_free(EXYNOS5410_GPK1(0));
}

static struct s5p_dp_platdata smdk5410_dp_data __initdata = {
	.video_info	= &smdk5410_dp_config,
	.phy_init	= s5p_dp_phy_init,
	.phy_exit	= s5p_dp_phy_exit,
	.backlight_on	= s5p_dp_backlight_on,
	.backlight_off	= s5p_dp_backlight_off,
};
#endif  /* CONFIG_S5P_DP */

static struct platform_device *m6x_display_devices[] __initdata = {
#ifdef CONFIG_FB_MIPI_DSIM
#ifdef CONFIG_S5P_DEV_MIPI_DSIM0
	&s5p_device_mipi_dsim0,
#else
	&s5p_device_mipi_dsim1,
#endif
#endif
#ifdef CONFIG_S5P_DEV_FIMD0
	&s5p_device_fimd0,
#else
	&s5p_device_fimd1,
#endif
#ifdef CONFIG_S5P_DP
	&s5p_device_dp,
	&smdk5410_dp_lcd,
#endif
#if defined(CONFIG_BACKLIGHT_LM3695) || defined(CONFIG_BACKLIGHT_LM3630)
	&m6x_device_i2c9,
#endif
/* LCD Backlight data */
#ifdef CONFIG_TPS65132
	&m6x_device_i2c14,
#endif
};

#if defined(CONFIG_BACKLIGHT_LM3695) || defined(CONFIG_BACKLIGHT_LM3630)
static void __init lm3695_rt_init_res(void) {
	bl_lm3695.en_gpio = MEIZU_LCD_BL_EN;
}
#endif

void __init exynos5_m6x_display_init(void)
{
#if defined(CONFIG_FB_VIDEO_PSR)
	// For TE_VSYNC, GPIO initialization will be processed 
	// in m6x_init_gpio_cfg of gpio-m6x.c after this step
	// This interrupt will be chained to 77 (gpio_RT group)
	int irq;
	irq = s5p_register_gpio_interrupt(MEIZU_LCD_TE);
	if (IS_ERR_VALUE(irq)){
		pr_err("%s: Failed to configure GPJ1(7) \n", __func__);
		return;
	}
#endif

#ifdef CONFIG_FB_MIPI_DSIM
#ifdef CONFIG_S5P_DEV_MIPI_DSIM0
	s5p_dsim0_set_platdata(&dsim_platform_data);
#else
	s5p_dsim1_set_platdata(&dsim_platform_data);
#endif
#endif
#ifdef CONFIG_S5P_DP
	s5p_dp_set_platdata(&smdk5410_dp_data);
#endif

#ifdef CONFIG_S5P_DEV_FIMD0
	s5p_fimd0_set_platdata(&m6x_lcd0_pdata);
#else
	s5p_fimd1_set_platdata(&m6x_lcd1_pdata);
#endif
#if defined(CONFIG_BACKLIGHT_LM3695) || defined(CONFIG_BACKLIGHT_LM3630)
	lm3695_rt_init_res();
	i2c_register_board_info(9, i2c_bl, ARRAY_SIZE(i2c_bl)); 
#endif
#ifdef CONFIG_TPS65132
	i2c_register_board_info(14, i2c_tps, ARRAY_SIZE(i2c_tps)); 
#endif

	platform_add_devices(m6x_display_devices,
			ARRAY_SIZE(m6x_display_devices));

#ifdef CONFIG_S5P_DP
	exynos5_fimd1_setup_clock(&s5p_device_fimd1.dev,
			"sclk_fimd", "mout_mpll_bpll", 267 * MHZ);
#endif

#ifdef CONFIG_FB_MIPI_DSIM
#if defined(CONFIG_S5P_DEV_FIMD0)
	exynos5_fimd0_setup_clock(&s5p_device_fimd0.dev,
			"sclk_fimd", "mout_mpll_bpll", 140 * MHZ);
#else
	exynos5_fimd1_setup_clock(&s5p_device_fimd1.dev,
			"sclk_fimd", "mout_mpll_bpll", 140 * MHZ);
#endif
#endif
}

