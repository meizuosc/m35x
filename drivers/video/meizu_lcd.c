#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/ctype.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/lcd.h>
#include <linux/backlight.h>
#include <linux/regulator/consumer.h>
#include <mach/gpio-meizu.h>
#include <linux/gpio.h>

#include "meizu_lcd.h"

#define CHECK_PANEL_RET(func) do {\
	int ret = func;\
	if (ret) {\
		pr_err("#LCD WRITE ERROR: line %d\n", __LINE__);\
		return ret;}\
} while(0);

#define CHECK_LCD_NULL() do {\
	if (g_lcd_info == NULL)\
		return 0;\
}while(0);

struct sharp_ce_mode {
	const char *mode;
	int mode_val;
};

#ifdef LCD_TEST
static struct sharp_ce_mode lcd_mode_map[] = {
	{"lowlow", 0},
	{"lowmed", 1},
	{"lowhigh", 2},
	{"medlow", 3},
	{"medmed", 4},
	{"medhigh", 5},
	{"highlow", 6},
	{"highmed", 7},
	{"highhigh", 8},
	{"satlow", 9},
	{"satmed", 10},
	{"sathigh", 11},
	{"ceoff", 12},
};
#endif
static int lcd_id[ID_CODEMAX] = {0,};
module_param_array(lcd_id, int, NULL, S_IRUGO | S_IWUSR | S_IWGRP);
static int lcd_read_information(struct mipi_dsim_device *mipi_dev);

static void mipi_lcd_reset(unsigned int level)
{
	unsigned int gpio = MEIZU_LCD_RST;
	gpio_request(gpio, "GPJ0");
	gpio_set_value(gpio, level);
	gpio_free(gpio);
}

static int mipi_lcd_power_on(struct lcd_panel_info *lcd, int power)
{
	if (power) {
		mipi_lcd_reset(0);
		regulator_enable(lcd->vddio);
		usleep_range(5000,6000);
		regulator_enable(lcd->vsp);
		regulator_enable(lcd->vsn);
		usleep_range(5000,6000);
		mipi_lcd_reset(1);
		usleep_range(10000, 12000);
	} else {
		regulator_disable(lcd->vsn);
		regulator_disable(lcd->vsp);
		usleep_range(10000, 12000);
		mipi_lcd_reset(0);
		regulator_disable(lcd->vddio);
	}

	return 0;
}

static struct lcd_panel_info *g_lcd_info;
static int write_to_lcd(struct lcd_panel_info *lcd,
		const struct lcd_param *param)
{
	int i = 0, ret = 0;

	do {
		ret = param[i].size ?
			write_data(lcd, param[i].type, param[i].param, param[i].size) :
			write_cmd(lcd, param[i].type, param[i].param[0], param[i].param[1]);
		if (param[i].delay)
			usleep_range(param[i].delay *1000, param[i].delay *1000);

	} while (!ret && param[++i].size != -1);

	return ret;
}

static int lcd_panel_sharp_init_code(struct lcd_panel_info *lcd)
{
	switch (lcd_id[ID_CODE3]) {
		case 0:
#if defined(CONFIG_FB_VIDEO_PSR)
			if (lcd_id[ID_FACTO] == 0)
				return write_to_lcd(lcd, jdi_video_psr_mode);
			else
				return write_to_lcd(lcd, sharp_video_psr_mode);
#else
			return write_to_lcd(lcd, sharp_video_mode);
#endif
		default:
			pr_debug("ID Code(%d) Error! use default gamma settings.\n", lcd_id[ID_CODE3]);
			return 0;
	}
}

static int lcd_panel_sharp_sleep_in(struct lcd_panel_info *lcd)
{
	return write_to_lcd(lcd, sharp_slpin_seq);
}

static int lcd_panel_sharp_sleep_out(struct lcd_panel_info *lcd)
{
	return write_to_lcd(lcd, sharp_slpout_seq);
}

static int lcd_panel_sharp_display_on(struct lcd_panel_info *lcd)
{
	return write_to_lcd(lcd, sharp_dspon_seq); 
}

static int lcd_panel_sharp_display_off(struct lcd_panel_info *lcd)
{
	return write_to_lcd(lcd, sharp_dspoff_seq);
}

static int lcd_panel_sharp_command_to_video(struct lcd_panel_info *lcd)
{
//	pr_info("CtoV\n");
	return write_to_lcd(lcd, sharp_command_to_video);
}

static int lcd_panel_sharp_video_to_command(struct lcd_panel_info *lcd)
{
//	pr_info("VtoC\n");
	return write_to_lcd(lcd, sharp_video_to_command);
}

static int __maybe_unused lcd_panel_sharp_test_image(struct lcd_panel_info *lcd)
{
	return write_to_lcd(lcd, test_image);
}

static int lcd_panel_jdi_gamma_seq(struct lcd_panel_info *lcd)
{
	int ret = 0;
	
	ret = write_to_lcd(lcd, jdi_gamma_cmd2page0);
	ret = write_to_lcd(lcd, jdi_gamma_cmd2page1);
	return ret;
}

static int lcd_panel_jdi_sleep_in(struct lcd_panel_info *lcd)
{
	return write_to_lcd(lcd, jdi_slpin_seq);
}

static int lcd_panel_jdi_sleep_out(struct lcd_panel_info *lcd)
{
	return write_to_lcd(lcd, jdi_slpout_seq);
}

static int lcd_panel_jdi_display_on(struct lcd_panel_info *lcd)
{
	return write_to_lcd(lcd, jdi_dspon_seq); 
}

static int lcd_panel_jdi_display_off(struct lcd_panel_info *lcd)
{
	return write_to_lcd(lcd, jdi_dspoff_seq);
}
#ifdef LCD_TEST
static int lcd_panel_hsync_out(struct lcd_panel_info *lcd)
{
	return write_to_lcd(lcd, sharp_hsync_out_seq);
}
static int lcd_panel_vsync_out(struct lcd_panel_info *lcd)
{
	return write_to_lcd(lcd, sharp_vsync_out_seq);
}
static int lcd_panel_set_brightness(struct lcd_panel_info *lcd, int brt)
{
	struct lcd_param sharp_brightness[] = {
		DCS_SHORT_PARAM(0x51, brt),
		LCD_PARAM_DEF_END,
	};

	return write_to_lcd(lcd, sharp_brightness);
}
static int lcd_panel_sharp_set_ce_mode(struct lcd_panel_info *lcd)
{
	if (lcd->id_code[ID_CODE2] == 0x00) return 0;
	switch (lcd->ce_mode) {
	case 0xff:
		return 0;
	case 0:
		return write_to_lcd(lcd, sharp_sat_low_lit_low);
	break;
	case 1:
		return write_to_lcd(lcd, sharp_sat_low_lit_med);
	break;
	case 2:
		return write_to_lcd(lcd, sharp_sat_low_lit_high);
	break;
	case 3:
		return write_to_lcd(lcd, sharp_sat_med_lit_low);
	break;
	case 4:
		return write_to_lcd(lcd, sharp_sat_med_lit_med);
	break;
	case 5:
		return write_to_lcd(lcd, sharp_sat_med_lit_high);
	break;
	case 6:
		return write_to_lcd(lcd, sharp_sat_high_lit_low);
	break;
	case 7:
		return write_to_lcd(lcd, sharp_sat_high_lit_low);
	break;
	case 8:
		return write_to_lcd(lcd, sharp_sat_high_lit_high);
	break;
	case 9:
		return write_to_lcd(lcd, sharp_sat_low);
	break;
	case 10:
		return write_to_lcd(lcd, sharp_sat_med);
	break;
	case 11:
		return write_to_lcd(lcd, sharp_sat_high);
	break;
	case 12:
		return write_to_lcd(lcd, sharp_ce_off);
	default:
		return 0;
	break;
	}
}
#endif
static int lcd_panel_sharp_cabc_switch(struct lcd_panel_info *lcd, int enable)
{
	if (enable)
		return write_to_lcd(lcd, sharp_cabc_seq_on);
	else
		return write_to_lcd(lcd, sharp_cabc_seq_off);
}
// Read command : 0x06 DCS_NO_PARAM_SHORT, 0x37 SetMAxreturnpacketiisize
// 0x0A : get_power_mode, returnmax 1
// 0x0B : get_address_mode, returnmax 1
// 0x0C : get_pixel_format, returnmax 1
// 0x0D : get_display_mode, returnmax 1
// 0x0E : get_dignal_mode, returnmax 1
// 0x0F : get_diagnostic_result, returnmax 1
static int lcd_read_information(struct mipi_dsim_device *mipi_dev)
{
	struct lcd_panel_info *lcd =g_lcd_info;//dev_get_drvdata(mipi_dev->dev);
	int i;
	unsigned int val = 0;
	unsigned int ret = 0;

	//write_to_lcd(lcd, sharp_unlock); /*set password for ROnly*/
	//set_packet_size(lcd, 1); /* set return data size*/

	for(i = 0; i<6; i++){
		ret = read_data(lcd, 0x0a + i, 1, (u8*)&val);
		pr_info("return for 0X%x value 0x%x, ret= 0x%x\n", 0x0A+i, val, ret);
	}
	ret = read_data(lcd, 0x45, 2,(u8*) &val);// get_scanline)
	pr_info("return for 0X%x value 0x%x, ret= 0x%x\n", 0x0A+i, val, ret);
	udelay(3);
	return 0;
}

static int lcd_read_id(struct mipi_dsim_device *mipi_dev)
{
	unsigned int gpio = MEIZU_LCD_ID;
#if 0
	struct lcd_panel_info *lcd =g_lcd_info;//dev_get_drvdata(mipi_dev->dev);
	unsigned int val = 0;
	int i = 0;
	for (i = 0; i < 3; i++) {
		id_code = read_data(lcd, 0xda + i, 1, &val); /*read ID Code reg 0xda*/
		lcd->id_code[i] = val;
		lcd_id[i] = lcd->id_code[i];
		pr_info("id code 0X%x value 0x%x\n", 0xDA+i, lcd_id[i]);
	}
#endif
	gpio_request(gpio, "GPJ0");
	lcd_id[ID_FACTO] = gpio_get_value(gpio);
	if (lcd_id[ID_FACTO] == 1)
		pr_info("Sharp Module.\n");
	else
		pr_info("JDI Module.\n");
	gpio_free(gpio);
	return 0;
}
static int lcd_displayon(struct mipi_dsim_device *mipi_dev)
{
	struct lcd_panel_info *lcd = g_lcd_info;//dev_get_drvdata(mipi_dev->dev);
	
	if ((lcd->id_code[ID_CODE1] & 0x10)) {
		/*JDI init*/
		CHECK_PANEL_RET(lcd_panel_jdi_sleep_out(lcd));
		CHECK_PANEL_RET(lcd_panel_jdi_display_on(lcd));
	} else {
		/*Sharp init*/
		CHECK_PANEL_RET(lcd_panel_sharp_sleep_out(lcd));
		CHECK_PANEL_RET(lcd_panel_sharp_display_on(lcd));
	} 
	
	return 0;
}

static int lcd_init(struct mipi_dsim_device *mipi_dev)
{
	struct lcd_panel_info *lcd = g_lcd_info;//dev_get_drvdata(mipi_dev->dev);
	
	if ((lcd->id_code[ID_CODE1] & 0x10)) {
		/*JDI init*/
		CHECK_PANEL_RET(lcd_panel_jdi_sleep_out(lcd));
		CHECK_PANEL_RET(lcd_panel_jdi_gamma_seq(lcd));
		CHECK_PANEL_RET(lcd_panel_jdi_display_on(lcd));
	} else {
		/*Sharp init*/
		CHECK_PANEL_RET(lcd_panel_sharp_init_code(lcd));
	} 
	
	return 0;
}

static int lcd_remove(struct mipi_dsim_device *mipi_dev)
{
	struct lcd_panel_info *lcd = g_lcd_info;

	mipi_lcd_power_on(lcd, 0);
	regulator_put(lcd->vddio);
	regulator_put(lcd->vsp);
	regulator_put(lcd->vsn);

	if (g_lcd_info)
		kfree(lcd);

	return 0;
}
int lcd_cabc_opr(unsigned int brightness, unsigned int enable)
{
	CHECK_LCD_NULL(); 
	if ((g_lcd_info->id_code[ID_CODE1] & 0x10)) {
		pr_info("JDI panel.");
	} else {
		pr_info("Sharp panel.");
		CHECK_PANEL_RET(lcd_panel_sharp_cabc_switch(g_lcd_info, enable));
	}
	g_lcd_info->cabc_en = enable;
	return 0;
}
EXPORT_SYMBOL_GPL(lcd_cabc_opr);

#ifdef LCD_TEST
int lcd_cabc_set_brightness(unsigned int brightness)
{
	CHECK_LCD_NULL(); 
	CHECK_PANEL_RET(lcd_panel_set_brightness(g_lcd_info , brightness));
	return 0;
}
EXPORT_SYMBOL_GPL(lcd_cabc_set_brightness);

static ssize_t lcd_sync_enable(struct device *dev, struct device_attribute
					*attr, const char *buf, size_t size)
{
	struct lcd_panel_info *lcd = g_lcd_info;//dev_get_drvdata(dev);
	int num;

	sscanf(buf, "%d", &num);
	pr_info("cabc %s\n", num == 0 ? "vsync out" : "hsync out");

	if (num == 0) {
		CHECK_PANEL_RET(lcd_panel_vsync_out(lcd));
	} else {
		CHECK_PANEL_RET(lcd_panel_hsync_out(lcd));
	}

	return sizeof(num);
}
static DEVICE_ATTR(sync, 0644, NULL, lcd_sync_enable);

static ssize_t lcd_set_brt(struct device *dev, struct device_attribute
					*attr, const char *buf, size_t size)
{
	struct lcd_panel_info *lcd = g_lcd_info;//dev_get_drvdata(dev);
	int brt;

	sscanf(buf, "%d", &brt);
	pr_info("brightness set %d\n", brt);
	CHECK_PANEL_RET(lcd_panel_set_brightness(lcd, brt));

	return sizeof(brt);
}
static DEVICE_ATTR(brt, 0644, NULL, lcd_set_brt);

static ssize_t lcd_set_ce(struct device *dev, struct device_attribute
					*attr, const char *buf, size_t size)
{
	struct lcd_panel_info *lcd = g_lcd_info;//dev_get_drvdata(dev);
	int i;

	for (i = 0; i < ARRAY_SIZE(lcd_mode_map); i++) {
		if (sysfs_streq(buf, lcd_mode_map[i].mode)) {
			lcd->ce_mode = lcd_mode_map[i].mode_val;
			break;
		} else {
			lcd->ce_mode = 0xff;
		}
	}
	if ((g_lcd_info->id_code[ID_CODE1] & 0x10)) {
		pr_info("JDI panel.");
	} else {
		CHECK_PANEL_RET(lcd_panel_sharp_set_ce_mode(lcd));
	}
	pr_info("set mode %d name %s\n", lcd->ce_mode, buf);
	pr_info("please reset the LCD\n");

	return size;
}
static ssize_t lcd_get_ce(struct device *dev, struct device_attribute
					*attr, char *buf)
{
	struct lcd_panel_info *lcd = g_lcd_info;
	int i;

	for (i = 0; i < ARRAY_SIZE(lcd_mode_map); i++) {
		if (lcd->ce_mode == lcd_mode_map[i].mode_val) {
			return sprintf(buf, "the ce mode is %s\n", lcd_mode_map[i].mode);
		}
	}
	return sprintf(buf, "you don't set ce mode\n");
}

static DEVICE_ATTR(ce, 0644, lcd_get_ce, lcd_set_ce);

#endif

static int lcd_probe(struct mipi_dsim_device *dsim_dev)
{
	struct lcd_panel_info *lcd = NULL;
	int err = 0;

	lcd = kzalloc(sizeof(struct lcd_panel_info), GFP_KERNEL);
	if (!lcd) {
		pr_err("failed to allocate ls044k3sx01 structure.\n");
		return -ENOMEM;
	}

	lcd->dsim_dev = dsim_dev;
	lcd->dev = dsim_dev->dev;

	lcd->ld = lcd_device_register("lcd_panel", lcd->dev, lcd, NULL);
	if (IS_ERR(lcd->ld)) {
		dev_err(lcd->dev, "failed to register lcd ops.\n");
		goto err_dev_register;
	}

	lcd->state = LCD_DISPLAY_POWER_OFF;
	lcd->ce_mode = 0xff;
	lcd->cabc_en = false;

	lcd->vddio = regulator_get(NULL, "vdd18_lcd");
	if (IS_ERR(lcd->vddio)) {
		pr_err("lcd vddio regulator get failed!\n");
		return PTR_ERR(lcd->vddio);
	}

	lcd->vsp = regulator_get(NULL, "lcd-5v");
	if (IS_ERR(lcd->vsp)) {
		pr_err("lcd vsp regulator get failed!\n");
		return PTR_ERR(lcd->vsp);
	}

	lcd->vsn = regulator_get(NULL, "lcd-n5v");
	if (IS_ERR(lcd->vsp)) {
		pr_err("lcd vsn regulator get failed!\n");
		return PTR_ERR(lcd->vsn);
	}

#ifdef LCD_TEST
	err = device_create_file(lcd->dev, &dev_attr_sync);
	if (err < 0) {
		dev_err(lcd->dev, "Failed to create attr file cabc %d!\n", err);
	}

	err = device_create_file(lcd->dev, &dev_attr_brt);
	if (err < 0) {
		dev_err(lcd->dev, "Failed to create attr file cabc %d!\n", err);
	}

	err = device_create_file(lcd->dev, &dev_attr_ce);
	if (err < 0) {
		dev_err(lcd->dev, "Failed to create attr file cabc %d!\n", err);
	}
#endif
	g_lcd_info = lcd;

	return err;

err_dev_register:
	kfree(lcd);
	return -1;
}
static int lcd_shutdown(struct mipi_dsim_device *mipi_dev)
{
	/* lcd power off */
	mipi_lcd_power_on(g_lcd_info, 0);
	pr_info("%s shutdown\n", __func__);
	return 0;
}

static int lcd_suspend(struct mipi_dsim_device *mipi_dev)
{
	struct lcd_panel_info *lcd = g_lcd_info;//dev_get_drvdata(mipi_dev->dev);

	if ((g_lcd_info->id_code[ID_CODE1] & 0x10)) {
		CHECK_PANEL_RET(lcd_panel_jdi_display_off(lcd));
		CHECK_PANEL_RET(lcd_panel_jdi_sleep_in(lcd));
	} else {
		CHECK_PANEL_RET(lcd_panel_sharp_display_off(lcd));
		CHECK_PANEL_RET(lcd_panel_sharp_sleep_in(lcd));
	}
	lcd->state = LCD_DISPLAY_SLEEP_IN;

	return 0;
}

static int lcd_mode_change(struct mipi_dsim_device *mipi_dev, int mode)
{
	struct lcd_panel_info *lcd = g_lcd_info;

	if (mode){
		CHECK_PANEL_RET(lcd_panel_sharp_video_to_command(lcd));
	}
	else{
		CHECK_PANEL_RET(lcd_panel_sharp_command_to_video(lcd));
	}

	return 0;
}

static int lcd_resume(struct mipi_dsim_device *mipi_dev)
{
	struct lcd_panel_info *lcd = g_lcd_info;//dev_get_drvdata(mipi_dev->dev);

	/* lcd power on */

	mipi_lcd_power_on(lcd, 1);
	lcd->state = LCD_DISPLAY_SLEEP_IN;
	return 0;
}
int __lcd_read_id(void)
{
	 
	int value = -1 ;
	gpio_request(MEIZU_LCD_ID, "GPJ0");
	
	value =  gpio_get_value(MEIZU_LCD_ID);
	
	gpio_free(MEIZU_LCD_ID);
	return  value ;
}

struct mipi_dsim_lcd_driver meizu_mipi_lcd_driver = {
	.probe	= lcd_probe,
	.init_lcd	= lcd_init,
	.suspend	= lcd_suspend,
	.resume	= lcd_resume,
	.shutdown = lcd_shutdown,
	.remove	= lcd_remove,
	.read_id = lcd_read_id,
	.read_information = lcd_read_information,
	.displayon = lcd_displayon,
	.gopsr = lcd_mode_change,
};

