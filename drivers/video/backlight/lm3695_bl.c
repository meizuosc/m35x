/*
 * TI LM3695 Backlight Driver
 *
 *			Copyright (C) 2013 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/platform_data/lm3695.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

/* Registers */
#define LM3695_GENERAL_PURPOSE		0x10
#define LM3695_BOOST_FREQ_SHIFT		5
#define LM3695_OVP_SHIFT		4
#define LM3695_STRING_SHIFT		3
#define LM3695_ENABLE_BRT_RW		BIT(2)
#define LM3695_DISABLE_RAMP		BIT(1)
#define LM3695_CHIP_ENABLE		BIT(0)

#define LM3695_DIT_FREQ			0x11
#define LM3695_RAMP_RATE		0x12

#define LM3695_BRT_LSB			0x13
#define LM3695_BRT_MSB			0x14
#define LM3695_REG_MAX			0xff

#define DEFAULT_BL_NAME			"led-backlight"
#define MAX_BRIGHTNESS			2048

struct lm3695_device_config {
	enum lm3695_boost_freq boost_freq;
	enum lm3695_ovp ovp;
	enum lm3695_string_mode string;
	enum lm3695_ramp_rate ramp;
};

struct lm3695 {
	struct i2c_client *client;
	struct backlight_device *bl;
	struct device *dev;
	struct lm3695_platform_data *pdata;
	struct regmap *regmap;
	int    enabled;
	int    brightness_store;
	int    suspended;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

static struct lm3695_device_config lm3695_default_cfg = {
	.boost_freq = LM3695_BOOST_FREQ_443KHZ,
	.ovp = LM3695_OVP_16V,
	.string = LM3695_DUAL_STRINGS,
	.ramp = LM3695_RAMP_500us,
};

static int lm3695_write_byte(struct lm3695 *lm, u8 reg, u8 data)
{
	return regmap_write(lm->regmap, reg, data);
}
static int lm3695_power_on(struct lm3695 *pchip, int onoff)
{
        struct regulator * bl_en = regulator_get(NULL, "bl-vol");

        if (pchip->enabled == onoff)
                return 0;

        if (IS_ERR(bl_en)) {
                dev_err(pchip->dev, "Failed to get regulator!\n");
                return PTR_ERR(bl_en);
        }

        if (onoff) {
		if (regulator_is_enabled(bl_en)) {
        		regulator_put(bl_en);
			return -1;
		}
		regulator_enable(bl_en);
	} else if (regulator_is_enabled(bl_en))
                regulator_disable(bl_en);

        regulator_put(bl_en);

        pchip->enabled = onoff;

        return 0;
}

static int lm3695_init_device(struct lm3695 *lm)
{
	struct lm3695_platform_data *pdata = lm->pdata;
	struct lm3695_device_config *cfg = &lm3695_default_cfg;
	u8 val = 0;
	int ret;

	/*
	 * Sequence of device initialization
	 *
	 *  1) Control the GPIO for the HWEN pin
	 *  2) Update Ramp Rate Register
	 *  3) Set Chip Enable bit with GP register
	 *  4) 600us delay
	 */

	ret = lm3695_power_on(lm, true);
	if (ret)
		return ret;
	if (pdata) {
		cfg->boost_freq = pdata->boost_freq;
		cfg->ovp = pdata->ovp;
		cfg->string = pdata->string;
		cfg->ramp = pdata->ramp;
	}

	/* Update the RAMP RATE register */
	ret = lm3695_write_byte(lm, LM3695_RAMP_RATE, cfg->ramp);
	if (ret)
		goto err;

	/* Update the GENERAL PURPOSE register */
	val = (cfg->boost_freq << LM3695_BOOST_FREQ_SHIFT) |
		(cfg->ovp << LM3695_OVP_SHIFT) |
		(cfg->string << LM3695_STRING_SHIFT);

	if (pdata && pdata->disable_ramp)
		val |= LM3695_DISABLE_RAMP;

	val |= LM3695_ENABLE_BRT_RW | LM3695_CHIP_ENABLE;

	ret = lm3695_write_byte(lm, LM3695_GENERAL_PURPOSE, val);
	if (ret)
		goto err;

	mdelay(1);

	return 0;

err:
	return ret;
}

static int lm3695_bl_update_status(struct backlight_device *bl)
{
	struct lm3695 *lm = bl_get_data(bl);
	u8 brt_lsb = 0, brt_msb = 0;

	if (bl->props.state & BL_CORE_SUSPENDED)
		bl->props.brightness = 0;

	lm->brightness_store = bl->props.brightness;
	if (lm->suspended)
		return 0;
	if (!bl->props.brightness) {
		lm3695_write_byte(lm, LM3695_BRT_LSB, 0);
		lm3695_write_byte(lm, LM3695_BRT_MSB, 0);
		lm3695_power_on(lm, false);
		return bl->props.brightness;
	}

	if (!lm->enabled) {
		lm3695_init_device(lm);
	}
	/*
	 * Update LSB and MSB for the 11bit resolution
	 *
	 *  LSB: 0 (default)
	 *  MSB: 8bit brightness value
	 */
	brt_lsb = bl->props.brightness & 0x7;
	brt_msb = bl->props.brightness >> 3;
	
	lm3695_write_byte(lm, LM3695_BRT_LSB, brt_lsb);
	lm3695_write_byte(lm, LM3695_BRT_MSB, brt_msb);

	return bl->props.brightness;
}

static int lm3695_bl_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static const struct backlight_ops lm3695_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = lm3695_bl_update_status,
	.get_brightness = lm3695_bl_get_brightness,
};

static int lm3695_backlight_register(struct lm3695 *lm)
{
	struct backlight_device *bl;
	struct backlight_properties props;
	struct lm3695_platform_data *pdata = lm->pdata;
	const char *name;
	int brightness;

	if (pdata) {
		name = pdata->name ? : DEFAULT_BL_NAME;
		brightness = pdata->initial_brightness;
	} else {
		name = DEFAULT_BL_NAME;
		brightness = 0;
	}

	props.type = BACKLIGHT_PLATFORM;
	props.max_brightness = MAX_BRIGHTNESS;
	props.brightness = brightness;

	bl = backlight_device_register(name, lm->dev, lm,
				       &lm3695_bl_ops, &props);
	if (IS_ERR(bl))
		return PTR_ERR(bl);

	lm->bl = bl;

	return 0;
}

static void lm3695_backlight_unregister(struct lm3695 *lm)
{
	if (lm->bl)
		backlight_device_unregister(lm->bl);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void lm3695_early_suspend(struct early_suspend *handler)
{
        struct lm3695 *pchip = container_of(handler,
                                        struct lm3695, early_suspend);

	pr_info("%s %d\n", __func__, __LINE__);
        pchip->suspended = true;
        lm3695_power_on(pchip, false);
}
static void lm3695_late_resume(struct early_suspend *handler)
{
        struct lm3695 *pchip = container_of(handler,
                                        struct lm3695, early_suspend);

	pr_info("%s %d\n", __func__, __LINE__);
	lm3695_init_device(pchip);
        pchip->suspended = false;
	backlight_update_status(pchip->bl);
}
#endif
static const struct regmap_config lm3695_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = LM3695_REG_MAX,
};

static int lm3695_probe(struct i2c_client *cl, const struct i2c_device_id *id)
{
	struct lm3695 *lm;
	struct lm3695_platform_data *pdata = cl->dev.platform_data;
	int ret;

	if (!i2c_check_functionality(cl->adapter, I2C_FUNC_SMBUS_I2C_BLOCK))
		return -EIO;

	lm = devm_kzalloc(&cl->dev, sizeof(struct lm3695), GFP_KERNEL);
	if (!lm)
		return -ENOMEM;

	lm->client = cl;
	lm->dev = &cl->dev;
	lm->pdata = pdata;
	lm->brightness_store = pdata->initial_brightness;
	lm->regmap = devm_regmap_init_i2c(cl, &lm3695_regmap);
	if (IS_ERR(lm->regmap)) {
		ret = PTR_ERR(lm->regmap);
		dev_err(&cl->dev, "fail to allocate register map: %d\n", ret);
		goto err_dev;
	}
	i2c_set_clientdata(cl, lm);

	ret = lm3695_init_device(lm);
	if (ret) {
		dev_err(lm->dev, "failed to init device: %d", ret);
		goto err_dev;
	}

	ret = lm3695_backlight_register(lm);
	if (ret) {
		dev_err(lm->dev, "failed to register backlight: %d\n", ret);
		goto err_dev;
	}

	backlight_update_status(lm->bl);
        lm->suspended = false;

#ifdef CONFIG_HAS_EARLYSUSPEND
	lm->early_suspend.suspend = lm3695_early_suspend;
	lm->early_suspend.resume  = lm3695_late_resume;
	lm->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;	
	register_early_suspend(&lm->early_suspend);
#endif
	dev_info(&cl->dev,"TI backlight driver registerred OK!\n");

	return 0;
err_dev:
	devm_kfree(&cl->dev, lm);
	return ret;
}

static int __devexit lm3695_remove(struct i2c_client *cl)
{
	struct lm3695 *lm = i2c_get_clientdata(cl);

	lm->bl->props.brightness = 0;
	backlight_update_status(lm->bl);
	lm3695_backlight_unregister(lm);
	lm3695_power_on(lm, false);

	return 0;
}

static const struct i2c_device_id lm3695_ids[] = {
	{"bl-lm3695", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, lm3695_ids);

static struct i2c_driver lm3695_driver = {
	.driver = {
		.name = "bl-lm3695",
	},
	.probe = lm3695_probe,
	.remove = __devexit_p(lm3695_remove),
	.id_table = lm3695_ids,
};

module_i2c_driver(lm3695_driver);

MODULE_DESCRIPTION("Texas Instruments LM3695 Backlight driver");
MODULE_AUTHOR("Milo Kim <milo.kim@ti.com>");
MODULE_LICENSE("GPL");
