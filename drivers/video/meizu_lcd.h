/* linux/drivers/video/samsung/meizu_lcd.h
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 
 *
 */
 
#ifndef __MEIZU_LCD_H__
#define __MEIZU_LCD_H__

#include <linux/gpio.h>
#include <mach/gpio-meizu.h>
#include <video/mipi_display.h>
#include <plat/mipi_dsi.h>
#include <plat/dsim.h>

/*The default value is 200us when esc_clk is 20MHZ, 
  *The value double if esc_clk is 10MHZ
  */
#define BTA_NONE 0
#define BTA_TIMEOUT 500
#define BTA_TIMEOUT_LONG 50000	/* 50ms */

#define write_cmd(lcd, type, cmd0, cmd1) \
	s5p_mipi_dsi_wr_data(lcd->dsim_dev, \
					type, cmd0, cmd1)

#define write_data(lcd, type, array, size)	\
	s5p_mipi_dsi_wr_data(lcd->dsim_dev, type,\
					(unsigned int)array, size)

#define read_data(lcd, addr, count, buf) \
	s5p_mipi_dsi_rd_data(lcd->dsim_dev, \
			MIPI_DSI_DCS_READ, addr, count, buf)

#define PP_NARG(...) \
    PP_NARG_(__VA_ARGS__,PP_RSEQ_N())
#define PP_NARG_(...) \
    PP_ARG_N(__VA_ARGS__)
#define PP_ARG_N( \
     _1, _2, _3, _4, _5, _6, _7, _8, _9,_10, \
    _11,_12,_13,_14,_15,_16,_17,_18,_19,_20, \
    _21,_22,_23,_24,_25,_26,_27,_28,_29,_30, \
    _31,_32,_33,_34,_35,_36,_37,_38,_39,_40,_41,_42,\
    _43,_44,_45,_46,_47,_48,N, ...) N

#define PP_RSEQ_N() \
    48,47,46,45,44,43,42,41,40,39,38,37,36,35,34,33,32,31,30,\
    29,28,27,26,25,24,23,22,21,20, \
    19,18,17,16,15,14,13,12,11,10, \
     9, 8, 7, 6, 5, 4, 3, 2, 1, 0

#define DCS_SHORT(mdelay, ...) {\
	.param = {__VA_ARGS__},\
	.delay = mdelay,\
	.type = MIPI_DSI_DCS_SHORT_WRITE,}
#define DCS_SHORT_PARAM(...) {\
	.param = {__VA_ARGS__},\
	.type = MIPI_DSI_DCS_SHORT_WRITE_PARAM,}
#define DCS_SHORT_PARAM_D(mdelay, ...) {\
	.delay = mdelay,\
	.param = {__VA_ARGS__},\
	.type = MIPI_DSI_DCS_SHORT_WRITE_PARAM,}

#define DCS_LONG( ...) {\
	.param = {__VA_ARGS__},\
	.size = PP_NARG(__VA_ARGS__),\
	.type = MIPI_DSI_DCS_LONG_WRITE,}

#define GCS_SHORT_PARAM(mdelay, ...) {\
	.delay = mdelay,\
	.param = {__VA_ARGS__},\
	.type = MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM,}

#define GCS_LONG(mdelay, ...) {\
	.delay = mdelay,\
	.param = {__VA_ARGS__},\
	.size = PP_NARG(__VA_ARGS__),\
	.type = MIPI_DSI_GENERIC_LONG_WRITE,}

#define GCS_SHORT_PARAM_0(...) {\
	.param = {__VA_ARGS__},\
	.type = MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM,}

#define GCS_LONG_0(...) {\
	.param = {__VA_ARGS__},\
	.size = PP_NARG(__VA_ARGS__),\
	.type = MIPI_DSI_GENERIC_LONG_WRITE,}

#define LCD_PARAM_DEF_END {.size = -1,}

#define LCD_TEST

struct lcd_param {
	char param[48];
	int size;
	int delay;	/* delay time ms */
	int type;
};

enum lcd_state {
	LCD_DISPLAY_SLEEP_IN,
	LCD_DISPLAY_DEEP_STAND_BY,
	LCD_DISPLAY_POWER_OFF,
};

enum lcd_id_reg {
	ID_CODE1 = 0, //0xda
	ID_CODE2, //0xdb
	ID_CODE3, //0xdc
	ID_FACTO, //0xdc
	ID_CODEMAX,
};

struct lcd_panel_info {
	struct device			*dev;
	struct lcd_device		*ld;

	struct mipi_dsim_device *dsim_dev;
	struct lcd_platform_data	*ddi_pd;

	enum lcd_state state;
	int ce_mode;

	int id_code[ID_CODEMAX];
	int cabc_en;
	/* power */
	struct regulator	*vddio;
	struct regulator	*vsp;
	struct regulator	*vsn;
};
/*For Sharp lcd module config.*/
static const struct lcd_param sharp_slpout_seq[] = {
	DCS_SHORT(120, MIPI_DCS_EXIT_SLEEP_MODE, 0x0),
	//GCS_LONG(0, 0xC2,0x30,0xF7,0x08,0x09,0x03,0x00,0x00), // Set NL(1800=0x708, 1920=0x780), BP(9), FP(3)
	LCD_PARAM_DEF_END,
};
static const struct lcd_param sharp_slpin_seq[] = {
	DCS_SHORT(80, MIPI_DCS_ENTER_SLEEP_MODE, 0x0),
	LCD_PARAM_DEF_END,
};
static const struct lcd_param sharp_dspon_seq[] = {
	DCS_SHORT(20, MIPI_DCS_SET_DISPLAY_ON, 0x0),
	//DCS_LONG(MIPI_DCS_SET_DISPLAY_ON, 0x0),
	//DCS_SHORT_PARAM(MIPI_DCS_BACKLIGHT_ON, 0x24),
	//DCS_SHORT_PARAM(0x35,0x00), // Te ON
	LCD_PARAM_DEF_END,
};
static const struct lcd_param sharp_dspoff_seq[] = {
	//DCS_SHORT_PARAM(MIPI_DCS_BACKLIGHT_ON, 0x0),
	DCS_SHORT(20, MIPI_DCS_SET_DISPLAY_OFF, 0x0),
	LCD_PARAM_DEF_END,
};
static const struct lcd_param sharp_cabc_seq[] = {
	DCS_LONG(0xDF,0x55,0xAA,0x52,0x08),
	DCS_LONG(0xB5,0x01,0xff,0x02,0x00,0x00,0x08,0x1c,0x00),
	DCS_SHORT_PARAM(0x5E, 0x3C),
	DCS_SHORT_PARAM(0x51, 0xFF),
	LCD_PARAM_DEF_END,
};
static const struct lcd_param sharp_cabc_seq_on[] = {
	DCS_SHORT_PARAM(0x55, 0x02),
	LCD_PARAM_DEF_END,
};

static const struct lcd_param sharp_cabc_seq_off[] = {
	DCS_SHORT_PARAM(0x55, 0x00),
	LCD_PARAM_DEF_END,
};
static const struct lcd_param sharp_cabc_gradient[] = {
	DCS_LONG(0xDF,0x55,0xAA,0x52,0x08,0x10),
	DCS_LONG(0xF7,0x24,0x00,0x3D,0x00),
	LCD_PARAM_DEF_END,
};
#define sat_lit_low  0x0F,0x0F,0x0F,0x0F,0x0F,0x0F
#define sat_lit_med  0x1F,0x1F,0x1F,0x1F,0x1F,0x1F
#define sat_lit_high 0x3F,0x3F,0x3F,0x3F,0x3F,0x3F
#define sat_lit_none 0x00,0x00,0x00,0x00,0x00,0x00

static const struct lcd_param sharp_sat_low_lit_low[] = {
	DCS_LONG(0xDF,0x55,0xAA,0x52,0x08),
	DCS_LONG(0xB0, 0x01),
	DCS_LONG(0x8A,sat_lit_low,sat_lit_low,0x00,0x06),
	DCS_LONG(0x8B,0x01,0x21,0x43,0x65,0x87,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00),
	DCS_LONG(0x88, 0x01),
	LCD_PARAM_DEF_END,
};

static const struct lcd_param sharp_sat_low_lit_med[] = {
	DCS_LONG(0xDF,0x55,0xAA,0x52,0x08),
	DCS_LONG(0xB0, 0x01),
	DCS_LONG(0x8A,sat_lit_low,sat_lit_med,0x00,0x06),
	DCS_LONG(0x8B,0x01,0x21,0x43,0x65,0x87,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00),
	DCS_LONG(0x88, 0x01),
	LCD_PARAM_DEF_END,
};

static const struct lcd_param sharp_sat_low_lit_high[] = {
	DCS_LONG(0xDF,0x55,0xAA,0x52,0x08),
	DCS_LONG(0xB0, 0x01),
	DCS_LONG(0x8A,sat_lit_low,sat_lit_high,0x00,0x06),
	DCS_LONG(0x8B,0x01,0x21,0x43,0x65,0x87,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00),
	DCS_LONG(0x88, 0x01),
	LCD_PARAM_DEF_END,
};
static const struct lcd_param sharp_sat_med_lit_low[] = {
	DCS_LONG(0xDF,0x55,0xAA,0x52,0x08),
	DCS_LONG(0xB0, 0x01),
	DCS_LONG(0x8A,sat_lit_med,sat_lit_low,0x00,0x06),
	DCS_LONG(0x8B,0x01,0x21,0x43,0x65,0x87,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00),
	DCS_LONG(0x88, 0x01),
	LCD_PARAM_DEF_END,
};

static const struct lcd_param sharp_sat_med_lit_med[] = {
	DCS_LONG(0xDF,0x55,0xAA,0x52,0x08),
	DCS_LONG(0xB0, 0x01),
	DCS_LONG(0x8A,sat_lit_med,sat_lit_med,0x00,0x06),
	DCS_LONG(0x8B,0x01,0x21,0x43,0x65,0x87,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00),
	DCS_LONG(0x88, 0x01),
	LCD_PARAM_DEF_END,
};

static const struct lcd_param sharp_sat_med_lit_high[] = {
	DCS_LONG(0xDF,0x55,0xAA,0x52,0x08),
	DCS_LONG(0xB0, 0x01),
	DCS_LONG(0x8A,sat_lit_med,sat_lit_high,0x00,0x06),
	DCS_LONG(0x8B,0x01,0x21,0x43,0x65,0x87,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00),
	DCS_LONG(0x88, 0x01),
	LCD_PARAM_DEF_END,
};
static const struct lcd_param sharp_sat_high_lit_low[] = {
	DCS_LONG(0xDF,0x55,0xAA,0x52,0x08),
	DCS_LONG(0xB0, 0x01),
	DCS_LONG(0x8A,sat_lit_high,sat_lit_low,0x00,0x06),
	DCS_LONG(0x8B,0x01,0x21,0x43,0x65,0x87,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00),
	DCS_LONG(0x88, 0x01),
	LCD_PARAM_DEF_END,
};

static const struct lcd_param sharp_sat_high_lit_med[] = {
	DCS_LONG(0xDF,0x55,0xAA,0x52,0x08),
	DCS_LONG(0xB0, 0x01),
	DCS_LONG(0x8A,sat_lit_high,sat_lit_med,0x00,0x06),
	DCS_LONG(0x8B,0x01,0x21,0x43,0x65,0x87,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00),
	DCS_LONG(0x88, 0x01),
	LCD_PARAM_DEF_END,
};

static const struct lcd_param sharp_sat_high_lit_high[] = {
	DCS_LONG(0xDF,0x55,0xAA,0x52,0x08),
	DCS_LONG(0xB0, 0x01),
	DCS_LONG(0x8A,sat_lit_high,sat_lit_high,0x00,0x06),
	DCS_LONG(0x8B,0x01,0x21,0x43,0x65,0x87,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00),
	DCS_LONG(0x88, 0x01),
	LCD_PARAM_DEF_END,
};

static const struct lcd_param sharp_sat_low[] = {
	DCS_LONG(0xDF,0x55,0xAA,0x52,0x08),
	DCS_LONG(0xB0, 0x01),
	DCS_LONG(0x8A,sat_lit_low,sat_lit_none,0x00,0x06),
	DCS_LONG(0x8B,0x01,0x21,0x43,0x65,0x87,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00),
	DCS_LONG(0x88, 0x01),
	LCD_PARAM_DEF_END,
};

static const struct lcd_param sharp_sat_med[] = {
	DCS_LONG(0xDF,0x55,0xAA,0x52,0x08),
	DCS_LONG(0xB0, 0x01),
	DCS_LONG(0x8A,sat_lit_med,sat_lit_none,0x00,0x06),
	DCS_LONG(0x8B,0x01,0x21,0x43,0x65,0x87,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00),
	DCS_LONG(0x88, 0x01),
	LCD_PARAM_DEF_END,
};

static const struct lcd_param sharp_sat_high[] = {
	DCS_LONG(0xDF,0x55,0xAA,0x52,0x08),
	DCS_LONG(0xB0, 0x01),
	DCS_LONG(0x8A,sat_lit_high,sat_lit_none,0x00,0x06),
	DCS_LONG(0x8B,0x01,0x21,0x43,0x65,0x87,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00),
	DCS_LONG(0x88, 0x01),
	LCD_PARAM_DEF_END,
};

static const struct lcd_param sharp_ce_off[] = {
	DCS_LONG(0x88, 0x02),
	LCD_PARAM_DEF_END,
};

static const struct lcd_param sharp_unlock[] = {
	GCS_LONG(0xDF,0x55,0xAA,0x52,0x08),
	GCS_LONG(0xD0,0x66),
	LCD_PARAM_DEF_END,
};

#ifdef LCD_TEST
static const struct lcd_param sharp_hsync_out_seq[] = {
	DCS_LONG(0xDF,0x55,0xAA,0x52,0x08),
	DCS_LONG(0x9B,0x02),
	DCS_LONG(0xCD,0x00,0x00,0x55,0x00,0x55,0x00),
	LCD_PARAM_DEF_END,
};
static const struct lcd_param sharp_vsync_out_seq[] = {
	DCS_LONG(0xDF,0x55,0xAA,0x52,0x08),
	DCS_LONG(0x9B,0x04),
	DCS_LONG(0xCD,0x00,0x00,0x55,0x00,0x55,0x00),
	LCD_PARAM_DEF_END,
};
#endif
static const struct lcd_param sharp_video_mode[] = {
	DCS_SHORT(3, 0X01, 0), // soft_reset min 3ms
	GCS_SHORT_PARAM(10, 0xB0,0x00), // MC_Manufacturer Command Access Protect
	GCS_LONG(10, 0xB3,0x14,0x00,0x00,0x22,0x00,0x00), // MC_Frame Memory Access and Interface Setting
	GCS_SHORT_PARAM(10, 0xD6,0x01),	// MC_Test_Register
	DCS_SHORT_PARAM(0x3A,0x77),	// set_pixel_format
	LCD_PARAM_DEF_END,
};

static const struct lcd_param sharp_video_RAM_mode[] = {
	DCS_SHORT(3, 0X01, 0),
	GCS_SHORT_PARAM(0, 0xB0,0x00),
	GCS_LONG(0, 0xB3,0x35,0x00,0x00,0x22,0x00,0x00), // 0x15=RAM capture mode, 0x35=RAM mode
	DCS_LONG(0x44,0x00,0x00), // set_tear_scanline fron 1 // Default 22line 200us
	GCS_SHORT_PARAM(0, 0xD6,0x01),
	DCS_SHORT_PARAM(0x3A,0x77),	// set_pixel_format
	DCS_LONG(0x2A,0x00,0x00,0x04,0x37),	// set_column_address
	DCS_LONG(0x2B,0x00,0x00,0x07,0x07),	// set_page_address
	LCD_PARAM_DEF_END,
};

// init setting for Command Mode but start with video mode
#if defined(CONFIG_LCD_64HZ) //64Hz LCD Tested
static const struct lcd_param sharp_video_psr_mode[] = {
	DCS_SHORT(3, 0X01, 0), // soft_reset min 3ms
	GCS_SHORT_PARAM(10, 0xB0,0x00), // MC_Manufacturer Command Access Protect
	GCS_LONG(10, 0xB3,0x14,0x00,0x00,0x22,0x00,0x00), // MC_Frame Memory Access and Interface Setting, Video Mode
  //  GCS_LONG(0, 0xC2,0x30,0x07,0x08,0x09,0x04,0x00,0x00), // Set NL(1800=0x708, 1920=0x780), BP(8), FP(8)
	GCS_LONG(10,0xC6,0x82,0x00,0x70,0x00,0x6C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x1C,0x05,\
			0x82,0x00,0x70,0x00,0x6C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x1C,0x05), // Set RTN0, RTNA0 = 0x80 = 128 clocks
	GCS_SHORT_PARAM(10, 0xD6,0x01),	// MC_Test_Register, No NVM load on exit_sleep_mode, default is 0x81(reload NVM)
	//GCS_LONG(0,0xC1, 0x08), // Red <-> Blue Color Swap
	DCS_SHORT_PARAM(0x3A,0x77),	// set_pixel_format
	DCS_LONG(0x2A,0x00,0x00,0x04,0x37),	// set_column_address for Command mode
	DCS_LONG(0x2B,0x00,0x00,0x07,0x07),	// set_page_address for Command mode
	DCS_SHORT_PARAM(0x35,0x00), // set_tear_on
	LCD_PARAM_DEF_END,
};
#else
// init setting for Command Mode but start with video mode
static const struct lcd_param sharp_video_psr_mode[] = {
	DCS_SHORT(3, 0X01, 0), // soft_reset min 3ms
	GCS_SHORT_PARAM(0, 0xB0,0x00), // MC_Manufacturer Command Access Protect
	GCS_LONG(0, 0xB3,0x14,0x00,0x00,0x22,0x00,0x00), // MC_Frame Memory Access and Interface Setting, Video Mode
	GCS_LONG(0, 0xC2,0x32,0xF7,0x08,0x09,0x03,0x00,0x00), // Set NL(1800=0x708, 1920=0x780), BP(9), FP(3)
	GCS_LONG(0, 0xC4,0x70,0x00,0x00,0x55,0x55,0x00,0x00,0x00,0x00,0x05,0x09,0x00,0x00,0x00,0x55,0x55,0x00,0x00,0x00,0x00,0x05,0x09),
	GCS_SHORT_PARAM(0, 0xD6,0x01),	// MC_Test_Register, No NVM load on exit_sleep_mode, default is 0x81(reload NVM)
	//GCS_LONG(0,0xC1, 0x08), // Red <-> Blue Color Swap
	DCS_SHORT_PARAM(0x3A,0x77),	// set_pixel_format
	DCS_LONG(0x2A,0x00,0x00,0x04,0x37),	// set_column_address for Command mode
	DCS_LONG(0x2B,0x00,0x00,0x07,0x07),	// set_page_address for Command mode
	DCS_SHORT_PARAM(0x35,0x00), // set_tear_on
	LCD_PARAM_DEF_END,
};
static const struct lcd_param jdi_video_psr_mode[] = {
	DCS_SHORT(3, 0X01, 0), // soft_reset min 3ms
	GCS_SHORT_PARAM(10, 0xB0,0x00), // MC_Manufacturer Command Access Protect
	GCS_LONG(0, 0xB3,0x14,0x00,0x00,0x22,0x00,0x00), // MC_Frame Memory Access and Interface Setting, Video Mode
	GCS_SHORT_PARAM(0, 0xC0, 0xFF),
	GCS_LONG(0, 0xC2,0x31,0xF7,0x08,0x09,0x03,0x00,0x00), // Set NL(1800=0x708, 1920=0x780), BP(9), FP(3)
	GCS_LONG(0, 0xC4,0x70,0x00,0x00,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x02),
	GCS_LONG(0, 0xC6,0x84,0x00,0x70,0x00,0x76,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x1C,0x05,\
			0x84,0x00,0x70,0x00,0x76,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x1C,0x05), // Set RTN0, RTNA0 = 0x80 = 128 clocks
	GCS_LONG(0, 0xD0,0x11,0x81,0xBB,0x16,0x8D,0x4C,0x19,0x19,0x0C,0x00), // Set NL(1800=0x708, 1920=0x780), BP(9), FP(3)
	GCS_SHORT_PARAM(0, 0xD6,0x01),	// MC_Test_Register, No NVM load on exit_sleep_mode, default is 0x81(reload NVM)
	//GCS_LONG(0,0xC1, 0x08), // Red <-> Blue Color Swap
	DCS_SHORT_PARAM(0x3A,0x77),	// set_pixel_format
	DCS_LONG(0x2A,0x00,0x00,0x04,0x37),	// set_column_address for Command mode
	DCS_LONG(0x2B,0x00,0x00,0x07,0x07),	// set_page_address for Command mode
	DCS_SHORT_PARAM(0x35,0x00), // set_tear_on
	LCD_PARAM_DEF_END,
};

#endif

static const struct lcd_param sharp_video_to_command[] = {
	GCS_SHORT_PARAM(10, 0xB0,0x00), // MC_Manufacturer Command Access Protect
	GCS_LONG(0, 0xB3,0x0C,0x00,0x00,0x22,0x00,0x00),
	LCD_PARAM_DEF_END,
};
static const struct lcd_param sharp_command_to_video[] = {
	GCS_SHORT_PARAM(10, 0xB0,0x00), // MC_Manufacturer Command Access Protect
	GCS_LONG(0, 0xB3,0x14,0x00,0x00,0x22,0x00,0x00),
	LCD_PARAM_DEF_END,
};

#if defined(CONFIG_LCD_64HZ) //64Hz LCD Tested
static const struct lcd_param sharp_command_mode[] = {
	DCS_SHORT(3, 0X01, 0), // soft_reset min 3ms
	GCS_SHORT_PARAM(0,0xB0,0x00), // Unlock
	//DCS_SHORT_PARAM(0xA1,0x40),
	GCS_LONG(0, 0xB3,0x04,0x00,0x00,0x22,0x00,0x00), // Set TE 60Hz
	//GCS_LONG(0,0xB3,0x04,0x00,0x01,0x22,0x00,0x00), // Set TE 30Hz
    //GCS_LONG(0, 0xC2,0x30,0x07,0x08,0x0F,0x0F,0x00,0x00), // Set NL(1800=0x708, 1920=0x780), BP(8), FP(8)
	GCS_LONG(10,0xC6,0x82,0x00,0x70,0x00,0x6C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x1C,0x05,\
			0x82,0x00,0x70,0x00,0x6C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x1C,0x05), // Set RTN0, RTNA0 = 0x80 = 128 clocks
    // Now Fps = 14Mhz / (RTN0 * (NL+BP+FP)) = 60Hz
	DCS_LONG(0x44,0x06,0xF4), // at 1780 line, wave

	GCS_SHORT_PARAM_0(0xD6,0x01), // no NVM load on exit_sleep_mode, default is 0x81(reload NVM)
    // Here Manufacturer Manual Settings.
	GCS_LONG(0,0xC1, 0x08), // Red <-> Blue Color Swap
	//GCS_LONG(0,0xB6,0x3B,0xD3), // Set DSITXDIV(11, /32), DSI_THSSET(5, 740~1000Mbps)
	//GCS_LONG(0,0xC0,????), // Slew rate adjustment, can down driving strength
	
	DCS_SHORT_PARAM(0x3A,0x77),
	DCS_LONG(0x2A,0x00,0x00,0x04,0x37),
	DCS_LONG(0x2B,0x00,0x00,0x07,0x07),
	//DCS_SHORT_PARAM(0x35,0x00), // Te ON
//  DCS_LONG(0x2C,0x00),
//	DCS_LONG(0x2C,0xFF,0x00,0xFF,0xFF,0x00,0xFF),
//	DCS_LONG(0x3C,0xFF,0x00,0xFF,0xFF,0x00,0xFF),
//	DCS_LONG(0x3C,0xFF,0x00,0xFF,0xFF,0x00,0xFF),
	//Backlight On 100% CABC OFF
//	DCS_SHORT_PARAM(0x51,0xff),	// Write_Display_Brightness
//	DCS_SHORT_PARAM(0x55,0x01),	// Write_Content_Adaptive_Brightness_Control
//	DCS_SHORT_PARAM(0x53,0x2C), // Write_Control_Display
	//Test Image Generator
//	DCS_LONG(0xDE,0x01,0x3F,0xFF,0x10),
	LCD_PARAM_DEF_END,
};
#else
static const struct lcd_param sharp_command_mode[] = {
	DCS_SHORT(3, 0X01, 0), // soft_reset min 3ms
	GCS_SHORT_PARAM(0,0xB0,0x00), // Unlock
	GCS_LONG(0, 0xB3,0x04,0x00,0x00,0x22,0x00,0x00), // Set TE 60Hz
	DCS_LONG(0x44,0x06,0xF4), // at 1780 line, wave
	GCS_SHORT_PARAM_0(0xD6,0x01), // no NVM load on exit_sleep_mode, default is 0x81(reload NVM)
    // Here Manufacturer Manual Settings.
	GCS_LONG(0,0xC1, 0x08), // Red <-> Blue Color Swap
	
	DCS_SHORT_PARAM(0x3A,0x77),
	DCS_LONG(0x2A,0x00,0x00,0x04,0x37),
	DCS_LONG(0x2B,0x00,0x00,0x07,0x07),
	LCD_PARAM_DEF_END,
};
#endif

static const struct lcd_param test_image[] = {
	DCS_SHORT_PARAM(0xB0,0x00),
	DCS_LONG(0xDE,0x01,0x3F,0xFF,0x10),
	LCD_PARAM_DEF_END,
};

static const struct lcd_param sharp_init_seq_0[] = {
	DCS_LONG(0xDF,0x55,0xAA,0x52,0x08),
	DCS_LONG(0xB5,0x01,0xff,0x02,0x00,0x00,0x08,0x1c,0x00),
	DCS_LONG(0xB8,0x33,0x00,0x27,0x00,0x00,0x06,0xB5,0x0A),
	DCS_LONG(0xC0,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF),
	DCS_LONG(0xC1,0x1E,0x3D,0x5D,0x8C,0xC8,0xCF,0x9C,0xD1,0x8C,0xB3,0x6F,0x9E,0xB3,0xC6,0xDF,0xFB),
	DCS_LONG(0xC2,0x1E,0x3D,0x5D,0x8C,0xC8,0xCF,0x9C,0xD1,0x8C,0xB3,0x6F,0x9E,0xB3,0xC6,0xDF,0xFB),
	DCS_LONG(0xC3,0x1E,0x3D,0x5D,0x8C,0xC8,0xCF,0x9C,0xD1,0x8C,0xB3,0x6F,0x9E,0xB3,0xC6,0xDF,0xFB),
	DCS_LONG(0xC4,0x1E,0x3D,0x5B,0x86,0xBA,0xBB,0x77,0xAE,0x72,0xA1,0x6B,0x9A,0xB1,0xC4,0xDE,0xFB),
	DCS_LONG(0xC5,0x1E,0x3D,0x5B,0x86,0xBA,0xBB,0x77,0xAE,0x72,0xA1,0x6B,0x9A,0xB1,0xC4,0xDE,0xFB),
	DCS_LONG(0xC6,0x1E,0x3D,0x5B,0x86,0xBA,0xBB,0x77,0xAE,0x72,0xA1,0x6B,0x9A,0xB1,0xC4,0xDE,0xFB),
	DCS_LONG(0xC8,0x11,0x19,0x06,0x0E,0x13,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00),
	LCD_PARAM_DEF_END,
};
static const struct lcd_param sharp_init_seq_0_4[] = {
	DCS_LONG(0xDF,0x55,0xAA,0x52,0x08),
	DCS_LONG(0xB5,0x01,0xff,0x02,0x00,0x00,0x08,0x1c,0x00),
	DCS_LONG(0xB8,0x38,0x00,0x22,0x00,0x00,0x06,0xB5,0x0A),
	DCS_LONG(0xC0,0x00,0xFF,0x00,0xFF,0x34,0xFF,0x00,0xFF,0x00,0xFF,0x38,0xFF),
	DCS_LONG(0xC1,0x1E,0x3D,0x5D,0x8C,0xCA,0xD2,0xA2,0xD7,0x90,0xB7,0x71,0x9F,0xB4,0xC7,0xE0,0xFB),
	DCS_LONG(0xC2,0x1E,0x3D,0x5D,0x8C,0xCA,0xD2,0xA2,0xD7,0x90,0xB7,0x71,0x9F,0xB4,0xC7,0xE0,0xFB),
	DCS_LONG(0xC3,0x41,0x51,0x6C,0x94,0xD0,0xDA,0xAF,0xE2,0x9E,0xC6,0x7A,0xA5,0xB5,0xC4,0xD9,0xEA),
	DCS_LONG(0xC4,0x1E,0x3D,0x5B,0x85,0xB7,0xB7,0x70,0xA8,0x6D,0x9C,0x69,0x99,0xB0,0xC3,0xDE,0xFB),
	DCS_LONG(0xC5,0x1E,0x3D,0x5B,0x85,0xB7,0xB7,0x70,0xA8,0x6D,0x9C,0x69,0x99,0xB0,0xC3,0xDE,0xFB),
	DCS_LONG(0xC6,0x43,0x53,0x6C,0x8E,0xBD,0xBF,0x7D,0xB3,0x7B,0xAC,0x72,0x9F,0xB1,0xC0,0xD7,0xE8),
	LCD_PARAM_DEF_END,
};
static const struct lcd_param sharp_init_seq_0_7[] = {
	DCS_LONG(0xDF,0x55,0xAA,0x52,0x08),
	DCS_LONG(0xB8,0x38,0x00,0x22,0x00,0x00,0x08,0xB5,0x0A),
	DCS_LONG(0xC0,0x00,0xFF,0x00,0xFF,0x5B,0xFF,0x00,0xFF,0x00,0xFF,0x61,0xFF),
	DCS_LONG(0xC1,0x1E,0x3D,0x5D,0x8C,0xCA,0xD2,0xA2,0xD7,0x90,0xB7,0x71,0x9F,0xB4,0xC7,0xE0,0xFB),
	DCS_LONG(0xC2,0x1E,0x3D,0x5D,0x8C,0xCA,0xD2,0xA2,0xD7,0x90,0xB7,0x71,0x9F,0xB4,0xC7,0xE0,0xFB),
	DCS_LONG(0xC3,0x64,0x72,0x85,0xA6,0xDA,0xDF,0xB8,0xE9,0xAA,0xD1,0x7C,0xA8,0xAf,0xB7,0xB9,0xDD),
	DCS_LONG(0xC4,0x1E,0x3D,0x5B,0x85,0xB7,0xB7,0x70,0xA8,0x6D,0x9C,0x69,0x99,0xB0,0xC3,0xDe,0xFB),
	DCS_LONG(0xC5,0x1E,0x3D,0x5B,0x85,0xB7,0xB7,0x70,0xA8,0x6D,0x9C,0x69,0x99,0xB0,0xC3,0xDE,0xFB),
	DCS_LONG(0xC6,0x68,0x76,0x85,0xA0,0xC8,0xC4,0x87,0xBB,0x87,0xB8,0x74,0xA2,0xAb,0xB3,0xB5,0xDB),
	LCD_PARAM_DEF_END,
};
/*For Sharp lcd config end*/

/*For JDI lcd module config.*/
static const struct lcd_param jdi_slpout_seq[] = {
	DCS_SHORT(120, MIPI_DCS_EXIT_SLEEP_MODE, 0x0),
	LCD_PARAM_DEF_END,
};
static const struct lcd_param jdi_slpin_seq[] = {
	DCS_SHORT(120, MIPI_DCS_ENTER_SLEEP_MODE, 0x0),
	LCD_PARAM_DEF_END,
};
static const struct lcd_param jdi_dspon_seq[] = {
	DCS_SHORT_PARAM(0xD3, 0x08),
	DCS_SHORT_PARAM(0xD4, 0x06),
	DCS_SHORT_PARAM(MIPI_DCS_SET_ADDRESS_MODE, 0x00), //NG
	DCS_SHORT(0, MIPI_DCS_SET_DISPLAY_ON, 0x0),
	LCD_PARAM_DEF_END,
};
static const struct lcd_param jdi_dspoff_seq[] = {
	DCS_SHORT(120, MIPI_DCS_SET_DISPLAY_OFF, 0x0),
	LCD_PARAM_DEF_END,
};

static const struct lcd_param jdi_gamma_cmd2page0[] = {
	DCS_SHORT_PARAM(0xFF, 0x01),
	DCS_SHORT_PARAM(0xFB, 0x01),
	DCS_SHORT_PARAM(0x75, 0x00),
	DCS_SHORT_PARAM(0x76, 0x00),
	DCS_SHORT_PARAM(0x77, 0x00),
	DCS_SHORT_PARAM(0x78, 0x0E),
	DCS_SHORT_PARAM(0x79, 0x00),
	DCS_SHORT_PARAM(0x7A, 0x27),
	DCS_SHORT_PARAM(0x7B, 0x00),
	DCS_SHORT_PARAM(0x7C, 0x3D),
	DCS_SHORT_PARAM(0x7D, 0x00),
	DCS_SHORT_PARAM(0x7E, 0x50),
	DCS_SHORT_PARAM(0x7F, 0x00),
	DCS_SHORT_PARAM(0x80, 0x61),
	DCS_SHORT_PARAM(0x81, 0x00),
	DCS_SHORT_PARAM(0x82, 0x71),
	DCS_SHORT_PARAM(0x83, 0x00),
	DCS_SHORT_PARAM(0x84, 0x7F),
	DCS_SHORT_PARAM(0x85, 0x00),
	DCS_SHORT_PARAM(0x86, 0x8D),
	DCS_SHORT_PARAM(0x87, 0x00),
	DCS_SHORT_PARAM(0x88, 0xBA),
	DCS_SHORT_PARAM(0x89, 0x00),
	DCS_SHORT_PARAM(0x8A, 0xE0),
	DCS_SHORT_PARAM(0x8B, 0x01),
	DCS_SHORT_PARAM(0x8C, 0x1D),
	DCS_SHORT_PARAM(0x8D, 0x01),
	DCS_SHORT_PARAM(0x8E, 0x4F),
	DCS_SHORT_PARAM(0x8F, 0x01),
	DCS_SHORT_PARAM(0x90, 0x9C),
	DCS_SHORT_PARAM(0x91, 0x01),
	DCS_SHORT_PARAM(0x92, 0xD8),
	DCS_SHORT_PARAM(0x93, 0x01),
	DCS_SHORT_PARAM(0x94, 0xDA),
	DCS_SHORT_PARAM(0x95, 0x02),
	DCS_SHORT_PARAM(0x96, 0x10),
	DCS_SHORT_PARAM(0x97, 0x02),
	DCS_SHORT_PARAM(0x98, 0x4A),
	DCS_SHORT_PARAM(0x99, 0x02),
	DCS_SHORT_PARAM(0x9A, 0x73),
	DCS_SHORT_PARAM(0x9B, 0x02),
	DCS_SHORT_PARAM(0x9C, 0xA6),
	DCS_SHORT_PARAM(0x9D, 0x02),
	DCS_SHORT_PARAM(0x9E, 0xCE),
	DCS_SHORT_PARAM(0x9F, 0x03),
	DCS_SHORT_PARAM(0xA0, 0x02),
	DCS_SHORT_PARAM(0xA2, 0x03),
	DCS_SHORT_PARAM(0xA3, 0x0F),
	DCS_SHORT_PARAM(0xA4, 0x03),
	DCS_SHORT_PARAM(0xA5, 0x20),
	DCS_SHORT_PARAM(0xA6, 0x03),
	DCS_SHORT_PARAM(0xA7, 0x34),
	DCS_SHORT_PARAM(0xA9, 0x03),
	DCS_SHORT_PARAM(0xAA, 0x4D),
	DCS_SHORT_PARAM(0xAB, 0x03),
	DCS_SHORT_PARAM(0xAC, 0x68),
	DCS_SHORT_PARAM(0xAD, 0x03),
	DCS_SHORT_PARAM(0xAE, 0x89),
	DCS_SHORT_PARAM(0xAF, 0x03),
	DCS_SHORT_PARAM(0xB0, 0xB4),
	DCS_SHORT_PARAM(0xB1, 0x03),
	DCS_SHORT_PARAM(0xB2, 0xCA),
	DCS_SHORT_PARAM(0xB3, 0x00),
	DCS_SHORT_PARAM(0xB4, 0x00),
	DCS_SHORT_PARAM(0xB5, 0x00),
	DCS_SHORT_PARAM(0xB6, 0x0E),
	DCS_SHORT_PARAM(0xB7, 0x00),
	DCS_SHORT_PARAM(0xB8, 0x27),
	DCS_SHORT_PARAM(0xB9, 0x00),
	DCS_SHORT_PARAM(0xBA, 0x3D),
	DCS_SHORT_PARAM(0xBB, 0x00),
	DCS_SHORT_PARAM(0xBC, 0x50),
	DCS_SHORT_PARAM(0xBD, 0x00),
	DCS_SHORT_PARAM(0xBE, 0x61),
	DCS_SHORT_PARAM(0xBF, 0x00),
	DCS_SHORT_PARAM(0xC0, 0x71),
	DCS_SHORT_PARAM(0xC1, 0x00),
	DCS_SHORT_PARAM(0xC2, 0x7F),
	DCS_SHORT_PARAM(0xC3, 0x00),
	DCS_SHORT_PARAM(0xC4, 0x8D),
	DCS_SHORT_PARAM(0xC5, 0x00),
	DCS_SHORT_PARAM(0xC6, 0xBA),
	DCS_SHORT_PARAM(0xC7, 0x00),
	DCS_SHORT_PARAM(0xC8, 0xE0),
	DCS_SHORT_PARAM(0xC9, 0x01),
	DCS_SHORT_PARAM(0xCA, 0x1D),
	DCS_SHORT_PARAM(0xCB, 0x01),
	DCS_SHORT_PARAM(0xCC, 0x4F),
	DCS_SHORT_PARAM(0xCD, 0x01),
	DCS_SHORT_PARAM(0xCE, 0x9C),
	DCS_SHORT_PARAM(0xCF, 0x01),
	DCS_SHORT_PARAM(0xD0, 0xD8),
	DCS_SHORT_PARAM(0xD1, 0x01),
	DCS_SHORT_PARAM(0xD2, 0xDA),
	DCS_SHORT_PARAM(0xD3, 0x02),
	DCS_SHORT_PARAM(0xD4, 0x10),
	DCS_SHORT_PARAM(0xD5, 0x02),
	DCS_SHORT_PARAM(0xD6, 0x4A),
	DCS_SHORT_PARAM(0xD7, 0x02),
	DCS_SHORT_PARAM(0xD8, 0x73),
	DCS_SHORT_PARAM(0xD9, 0x02),
	DCS_SHORT_PARAM(0xDA, 0xA6),
	DCS_SHORT_PARAM(0xDB, 0x02),
	DCS_SHORT_PARAM(0xDC, 0xCE),
	DCS_SHORT_PARAM(0xDD, 0x03),
	DCS_SHORT_PARAM(0xDE, 0x02),
	DCS_SHORT_PARAM(0xDF, 0x03),
	DCS_SHORT_PARAM(0xE0, 0x0F),
	DCS_SHORT_PARAM(0xE1, 0x03),
	DCS_SHORT_PARAM(0xE2, 0x20),
	DCS_SHORT_PARAM(0xE3, 0x03),
	DCS_SHORT_PARAM(0xE4, 0x34),
	DCS_SHORT_PARAM(0xE5, 0x03),
	DCS_SHORT_PARAM(0xE6, 0x4D),
	DCS_SHORT_PARAM(0xE7, 0x03),
	DCS_SHORT_PARAM(0xE8, 0x68),
	DCS_SHORT_PARAM(0xE9, 0x03),
	DCS_SHORT_PARAM(0xEA, 0x89),
	DCS_SHORT_PARAM(0xEB, 0x03),
	DCS_SHORT_PARAM(0xEC, 0xB4),
	DCS_SHORT_PARAM(0xED, 0x03),
	DCS_SHORT_PARAM(0xEE, 0xCA),
	DCS_SHORT_PARAM(0xEF, 0x00),
	DCS_SHORT_PARAM(0xF0, 0x89),
	DCS_SHORT_PARAM(0xF1, 0x00),
	DCS_SHORT_PARAM(0xF2, 0x8F),
	DCS_SHORT_PARAM(0xF3, 0x00),
	DCS_SHORT_PARAM(0xF4, 0x9A),
	DCS_SHORT_PARAM(0xF5, 0x00),
	DCS_SHORT_PARAM(0xF6, 0xA5),
	DCS_SHORT_PARAM(0xF7, 0x00),
	DCS_SHORT_PARAM(0xF8, 0xAF),
	DCS_SHORT_PARAM(0xF9, 0x00),
	DCS_SHORT_PARAM(0xFA, 0xB9),
	LCD_PARAM_DEF_END,
};
static const struct lcd_param jdi_gamma_cmd2page1[] = {
	DCS_SHORT_PARAM(0xFF, 0x02),
	DCS_SHORT_PARAM(0xFB, 0x01),
	DCS_SHORT_PARAM(0x00, 0x00),
	DCS_SHORT_PARAM(0x01, 0xC3),
	DCS_SHORT_PARAM(0x02, 0x00),
	DCS_SHORT_PARAM(0x03, 0xCC),
	DCS_SHORT_PARAM(0x04, 0x00),
	DCS_SHORT_PARAM(0x05, 0xD5),
	DCS_SHORT_PARAM(0x06, 0x00),
	DCS_SHORT_PARAM(0x07, 0xF5),
	DCS_SHORT_PARAM(0x08, 0x01),
	DCS_SHORT_PARAM(0x09, 0x11),
	DCS_SHORT_PARAM(0x0A, 0x01),
	DCS_SHORT_PARAM(0x0B, 0x42),
	DCS_SHORT_PARAM(0x0C, 0x01),
	DCS_SHORT_PARAM(0x0D, 0x6B),
	DCS_SHORT_PARAM(0x0E, 0x01),
	DCS_SHORT_PARAM(0x0F, 0xAE),
	DCS_SHORT_PARAM(0x10, 0x01),
	DCS_SHORT_PARAM(0x11, 0xE5),
	DCS_SHORT_PARAM(0x12, 0x01),
	DCS_SHORT_PARAM(0x13, 0xE7),
	DCS_SHORT_PARAM(0x14, 0x02),
	DCS_SHORT_PARAM(0x15, 0x19),
	DCS_SHORT_PARAM(0x16, 0x02),
	DCS_SHORT_PARAM(0x17, 0x52),
	DCS_SHORT_PARAM(0x18, 0x02),
	DCS_SHORT_PARAM(0x19, 0x7A),
	DCS_SHORT_PARAM(0x1A, 0x02),
	DCS_SHORT_PARAM(0x1B, 0xAF),
	DCS_SHORT_PARAM(0x1C, 0x02),
	DCS_SHORT_PARAM(0x1D, 0xD6),
	DCS_SHORT_PARAM(0x1E, 0x03),
	DCS_SHORT_PARAM(0x1F, 0x0A),
	DCS_SHORT_PARAM(0x20, 0x03),
	DCS_SHORT_PARAM(0x21, 0x19),
	DCS_SHORT_PARAM(0x22, 0x03),
	DCS_SHORT_PARAM(0x23, 0x2B),
	DCS_SHORT_PARAM(0x24, 0x03),
	DCS_SHORT_PARAM(0x25, 0x42),
	DCS_SHORT_PARAM(0x26, 0x03),
	DCS_SHORT_PARAM(0x27, 0x60),
	DCS_SHORT_PARAM(0x28, 0x03),
	DCS_SHORT_PARAM(0x29, 0x8A),
	DCS_SHORT_PARAM(0x2A, 0x03),
	DCS_SHORT_PARAM(0x2B, 0xE9),
	DCS_SHORT_PARAM(0x2D, 0x03),
	DCS_SHORT_PARAM(0x2F, 0xF8),
	DCS_SHORT_PARAM(0x30, 0x03),
	DCS_SHORT_PARAM(0x31, 0xFF),
	DCS_SHORT_PARAM(0x32, 0x00),
	DCS_SHORT_PARAM(0x33, 0x89),
	DCS_SHORT_PARAM(0x34, 0x00),
	DCS_SHORT_PARAM(0x35, 0x8F),
	DCS_SHORT_PARAM(0x36, 0x00),
	DCS_SHORT_PARAM(0x37, 0x9A),
	DCS_SHORT_PARAM(0x38, 0x00),
	DCS_SHORT_PARAM(0x39, 0xA5),
	DCS_SHORT_PARAM(0x3A, 0x00),
	DCS_SHORT_PARAM(0x3B, 0xAF),
	DCS_SHORT_PARAM(0x3D, 0x00),
	DCS_SHORT_PARAM(0x3F, 0xB9),
	DCS_SHORT_PARAM(0x40, 0x00),
	DCS_SHORT_PARAM(0x41, 0xC3),
	DCS_SHORT_PARAM(0x42, 0x00),
	DCS_SHORT_PARAM(0x43, 0xCC),
	DCS_SHORT_PARAM(0x44, 0x00),
	DCS_SHORT_PARAM(0x45, 0xD5),
	DCS_SHORT_PARAM(0x46, 0x00),
	DCS_SHORT_PARAM(0x47, 0xF5),
	DCS_SHORT_PARAM(0x48, 0x01),
	DCS_SHORT_PARAM(0x49, 0x11),
	DCS_SHORT_PARAM(0x4A, 0x01),
	DCS_SHORT_PARAM(0x4B, 0x42),
	DCS_SHORT_PARAM(0x4C, 0x01),
	DCS_SHORT_PARAM(0x4D, 0x6B),
	DCS_SHORT_PARAM(0x4E, 0x01),
	DCS_SHORT_PARAM(0x4F, 0xAE),
	DCS_SHORT_PARAM(0x50, 0x01),
	DCS_SHORT_PARAM(0x51, 0xE5),
	DCS_SHORT_PARAM(0x52, 0x01),
	DCS_SHORT_PARAM(0x53, 0xE7),
	DCS_SHORT_PARAM(0x54, 0x02),
	DCS_SHORT_PARAM(0x55, 0x19),
	DCS_SHORT_PARAM(0x56, 0x02),
	DCS_SHORT_PARAM(0x58, 0x52),
	DCS_SHORT_PARAM(0x59, 0x02),
	DCS_SHORT_PARAM(0x5A, 0x7A),
	DCS_SHORT_PARAM(0x5B, 0x02),
	DCS_SHORT_PARAM(0x5C, 0xAF),
	DCS_SHORT_PARAM(0x5D, 0x02),
	DCS_SHORT_PARAM(0x5E, 0xD6),
	DCS_SHORT_PARAM(0x5F, 0x03),
	DCS_SHORT_PARAM(0x60, 0x0A),
	DCS_SHORT_PARAM(0x61, 0x03),
	DCS_SHORT_PARAM(0x62, 0x19),
	DCS_SHORT_PARAM(0x63, 0x03),
	DCS_SHORT_PARAM(0x64, 0x2B),
	DCS_SHORT_PARAM(0x65, 0x03),
	DCS_SHORT_PARAM(0x66, 0x42),
	DCS_SHORT_PARAM(0x67, 0x03),
	DCS_SHORT_PARAM(0x68, 0x60),
	DCS_SHORT_PARAM(0x69, 0x03),
	DCS_SHORT_PARAM(0x6A, 0x8A),
	DCS_SHORT_PARAM(0x6B, 0x03),
	DCS_SHORT_PARAM(0x6C, 0xE9),
	DCS_SHORT_PARAM(0x6D, 0x03),
	DCS_SHORT_PARAM(0x6E, 0xF8),
	DCS_SHORT_PARAM(0x6F, 0x03),
	DCS_SHORT_PARAM(0x70, 0xFF),
	DCS_SHORT_PARAM(0x71, 0x00),
	DCS_SHORT_PARAM(0x72, 0xA0),
	DCS_SHORT_PARAM(0x73, 0x00),
	DCS_SHORT_PARAM(0x74, 0xA5),
	DCS_SHORT_PARAM(0x75, 0x00),
	DCS_SHORT_PARAM(0x76, 0xAF),
	DCS_SHORT_PARAM(0x77, 0x00),
	DCS_SHORT_PARAM(0x78, 0xB9),
	DCS_SHORT_PARAM(0x79, 0x00),
	DCS_SHORT_PARAM(0x7A, 0xC2),
	DCS_SHORT_PARAM(0x7B, 0x00),
	DCS_SHORT_PARAM(0x7C, 0xCB),
	DCS_SHORT_PARAM(0x7D, 0x00),
	DCS_SHORT_PARAM(0x7E, 0xD4),
	DCS_SHORT_PARAM(0x7F, 0x00),
	DCS_SHORT_PARAM(0x80, 0xDD),
	DCS_SHORT_PARAM(0x81, 0x00),
	DCS_SHORT_PARAM(0x82, 0xE5),
	DCS_SHORT_PARAM(0x83, 0x01),
	DCS_SHORT_PARAM(0x84, 0x02),
	DCS_SHORT_PARAM(0x85, 0x01),
	DCS_SHORT_PARAM(0x86, 0x1C),
	DCS_SHORT_PARAM(0x87, 0x01),
	DCS_SHORT_PARAM(0x88, 0x4B),
	DCS_SHORT_PARAM(0x89, 0x01),
	DCS_SHORT_PARAM(0x8A, 0x71),
	DCS_SHORT_PARAM(0x8B, 0x01),
	DCS_SHORT_PARAM(0x8C, 0xB3),
	DCS_SHORT_PARAM(0x8D, 0x01),
	DCS_SHORT_PARAM(0x8E, 0xE8),
	DCS_SHORT_PARAM(0x8F, 0x01),
	DCS_SHORT_PARAM(0x90, 0xE9),
	DCS_SHORT_PARAM(0x91, 0x02),
	DCS_SHORT_PARAM(0x92, 0x1C),
	DCS_SHORT_PARAM(0x93, 0x02),
	DCS_SHORT_PARAM(0x94, 0x53),
	DCS_SHORT_PARAM(0x95, 0x02),
	DCS_SHORT_PARAM(0x96, 0x7C),
	DCS_SHORT_PARAM(0x97, 0x02),
	DCS_SHORT_PARAM(0x98, 0xB2),
	DCS_SHORT_PARAM(0x99, 0x02),
	DCS_SHORT_PARAM(0x9A, 0xD8),
	DCS_SHORT_PARAM(0x9B, 0x03),
	DCS_SHORT_PARAM(0x9C, 0x0C),
	DCS_SHORT_PARAM(0x9D, 0x03),
	DCS_SHORT_PARAM(0x9E, 0x1C),
	DCS_SHORT_PARAM(0x9F, 0x03),
	DCS_SHORT_PARAM(0xA0, 0x2F),
	DCS_SHORT_PARAM(0xA2, 0x03),
	DCS_SHORT_PARAM(0xA3, 0x47),
	DCS_SHORT_PARAM(0xA4, 0x03),
	DCS_SHORT_PARAM(0xA5, 0x66),
	DCS_SHORT_PARAM(0xA6, 0x03),
	DCS_SHORT_PARAM(0xA7, 0x9E),
	DCS_SHORT_PARAM(0xA9, 0x03),
	DCS_SHORT_PARAM(0xAA, 0xF0),
	DCS_SHORT_PARAM(0xAB, 0x03),
	DCS_SHORT_PARAM(0xAC, 0xF8),
	DCS_SHORT_PARAM(0xAD, 0x03),
	DCS_SHORT_PARAM(0xAE, 0xFF),
	DCS_SHORT_PARAM(0xAF, 0x00),
	DCS_SHORT_PARAM(0xB0, 0xA0),
	DCS_SHORT_PARAM(0xB1, 0x00),
	DCS_SHORT_PARAM(0xB2, 0xA5),
	DCS_SHORT_PARAM(0xB3, 0x00),
	DCS_SHORT_PARAM(0xB4, 0xAF),
	DCS_SHORT_PARAM(0xB5, 0x00),
	DCS_SHORT_PARAM(0xB6, 0xB9),
	DCS_SHORT_PARAM(0xB7, 0x00),
	DCS_SHORT_PARAM(0xB8, 0xC2),
	DCS_SHORT_PARAM(0xB9, 0x00),
	DCS_SHORT_PARAM(0xBA, 0xCB),
	DCS_SHORT_PARAM(0xBB, 0x00),
	DCS_SHORT_PARAM(0xBC, 0xD4),
	DCS_SHORT_PARAM(0xBD, 0x00),
	DCS_SHORT_PARAM(0xBE, 0xDD),
	DCS_SHORT_PARAM(0xBF, 0x00),
	DCS_SHORT_PARAM(0xC0, 0xE5),
	DCS_SHORT_PARAM(0xC1, 0x01),
	DCS_SHORT_PARAM(0xC2, 0x02),
	DCS_SHORT_PARAM(0xC3, 0x01),
	DCS_SHORT_PARAM(0xC4, 0x1C),
	DCS_SHORT_PARAM(0xC5, 0x01),
	DCS_SHORT_PARAM(0xC6, 0x4B),
	DCS_SHORT_PARAM(0xC7, 0x01),
	DCS_SHORT_PARAM(0xC8, 0x71),
	DCS_SHORT_PARAM(0xC9, 0x01),
	DCS_SHORT_PARAM(0xCA, 0xB3),
	DCS_SHORT_PARAM(0xCB, 0x01),
	DCS_SHORT_PARAM(0xCC, 0xE8),
	DCS_SHORT_PARAM(0xCD, 0x01),
	DCS_SHORT_PARAM(0xCE, 0xE9),
	DCS_SHORT_PARAM(0xCF, 0x02),
	DCS_SHORT_PARAM(0xD0, 0x1C),
	DCS_SHORT_PARAM(0xD1, 0x02),
	DCS_SHORT_PARAM(0xD2, 0x53),
	DCS_SHORT_PARAM(0xD3, 0x02),
	DCS_SHORT_PARAM(0xD4, 0x7C),
	DCS_SHORT_PARAM(0xD5, 0x02),
	DCS_SHORT_PARAM(0xD6, 0xB2),
	DCS_SHORT_PARAM(0xD7, 0x02),
	DCS_SHORT_PARAM(0xD8, 0xD8),
	DCS_SHORT_PARAM(0xD9, 0x03),
	DCS_SHORT_PARAM(0xDA, 0x0C),
	DCS_SHORT_PARAM(0xDB, 0x03),
	DCS_SHORT_PARAM(0xDC, 0x1C),
	DCS_SHORT_PARAM(0xDD, 0x03),
	DCS_SHORT_PARAM(0xDE, 0x2F),
	DCS_SHORT_PARAM(0xDF, 0x03),
	DCS_SHORT_PARAM(0xE0, 0x47),
	DCS_SHORT_PARAM(0xE1, 0x03),
	DCS_SHORT_PARAM(0xE2, 0x66),
	DCS_SHORT_PARAM(0xE3, 0x03),
	DCS_SHORT_PARAM(0xE4, 0x9E),
	DCS_SHORT_PARAM(0xE5, 0x03),
	DCS_SHORT_PARAM(0xE6, 0xF0),
	DCS_SHORT_PARAM(0xE7, 0x03),
	DCS_SHORT_PARAM(0xE8, 0xF8),
	DCS_SHORT_PARAM(0xE9, 0x03),
	DCS_SHORT_PARAM(0xEA, 0xFF),
	DCS_SHORT_PARAM(0xFF, 0x00),
	LCD_PARAM_DEF_END,
};
#endif /* __MEIZU_LCD_H__*/
