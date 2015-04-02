/*
* Simple driver for Texas Instruments LM3630 Backlight driver chip
* Copyright (C) 2012 Texas Instruments
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
*/
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/platform_data/lm3630_bl.h>
#include <linux/regulator/consumer.h>
#include <linux/earlysuspend.h>
#include <mach/gpio-meizu.h>
#include <linux/gpio.h>

#define REG_CTRL	0x00
#define REG_CONFIG	0x01
#define REG_BRT_A	0x03
#define REG_BRT_B	0x04
#define REG_CUR_A	0x05
#define REG_CUR_B	0x06
#define REG_ONOFF_RAMP	0x07
#define REG_RUN_RAMP	0x08
#define REG_INT_STATUS	0x09
#define REG_INT_EN	0x0A
#define REG_FAULT	0x0B
#define RGE_SOFT_RESET	0x0F
#define REG_PWM_OUTLOW	0x12
#define REG_PWM_OUTHIGH	0x13
#define REG_MAX		0x1F

#define INT_DEBOUNCE_MSEC	10

enum lm3630_leds {
	BLED_ALL = 0,
	BLED_1,
	BLED_2
};

static const char * const bled_name[] = {
	[BLED_ALL] = "lm3630_bled",	/*Bank1 controls all string */
	[BLED_1] = "lm3630_bled1",	/*Bank1 controls bled1 */
	[BLED_2] = "lm3630_bled2",	/*Bank1 or 2 controls bled2 */
};

struct lm3630_chip_data {
	struct device *dev;
	struct delayed_work work;
	int irq;
	struct workqueue_struct *irqthread;
	struct lm3630_platform_data *pdata;
	struct backlight_device *bled1;
	struct backlight_device *bled2;
	struct regmap *regmap;
	struct early_suspend early_suspend;
	struct regulator *regulator;
	int enabled;
	bool suspended;
	int brightness_store;
};

/* initialize chip */
static int lm3630_power_on(struct lm3630_chip_data *pchip, int onoff)
{
	struct regulator * lm3630_regulator = NULL;

	if (pchip->enabled == onoff)
		return 0;

	lm3630_regulator = regulator_get(NULL, "bl-vol");
	if (IS_ERR(lm3630_regulator)) {
		dev_err(pchip->dev, "Failed to get regulator!\n");
		return PTR_ERR(lm3630_regulator);
	}

	if (onoff)
		regulator_enable(lm3630_regulator);
	else if (regulator_is_enabled(lm3630_regulator))
		regulator_disable(lm3630_regulator);

	regulator_put(lm3630_regulator);

	pchip->enabled = onoff;

	return 0;
}
static int lm3630_chip_init(struct lm3630_chip_data *pchip)
{
	int ret;
	unsigned int reg_val;
	struct lm3630_platform_data *pdata = pchip->pdata;

	/*pwm control */
	reg_val = ((pdata->pwm_active & 0x01) << 2) | (pdata->pwm_ctrl & 0x03);
	ret = regmap_update_bits(pchip->regmap, REG_CONFIG, 0x07, reg_val);
	if (ret < 0) {
		dev_err(pchip->dev, "%s: i2c failed to access register REG_CONFIG\n", __func__);
		goto out;
	}

	/* bank control */
	reg_val = ((pdata->bank_b_ctrl & 0x01) << 1) |
			(pdata->bank_a_ctrl & 0x07);
	ret = regmap_update_bits(pchip->regmap, REG_CTRL, 0x07, reg_val);
	if (ret < 0) {
		dev_err(pchip->dev, "%s: i2c failed to access register REG_CTRL with mask 0x7\n", __func__);
		goto out;
	}

	ret = regmap_update_bits(pchip->regmap, REG_CTRL, 0x80, 0x00);
	if (ret < 0) {
		dev_err(pchip->dev, "%s: i2c failed to access register REG_CTRL with mask 0x80\n", __func__);
		goto out;
	}

	ret = regmap_update_bits(pchip->regmap, REG_CTRL, 0x18, 0x18);
	if (ret < 0) {
		dev_err(pchip->dev, "%s: i2c failed to access register REG_CTRL with mask 0x18\n", __func__);
		goto out;
	}

	/* set initial brightness */
	if (pdata->bank_a_ctrl != BANK_A_CTRL_DISABLE) {
		ret = regmap_write(pchip->regmap,
				   REG_BRT_A, pchip->brightness_store);
		if (ret < 0) {
			dev_err(pchip->dev, "%s: i2c failed to access register REG_BRT_A\n", __func__);
			goto out;
		}
		
		ret = regmap_update_bits(pchip->regmap,
					REG_CUR_A, 0x1f, pdata->max_cur_a);
		if (ret < 0) {
			dev_err(pchip->dev, "%s: i2c failed to access register REG_CUR_A\n", __func__);
			goto out;
		}
	}

	if (pdata->bank_b_ctrl != BANK_B_CTRL_DISABLE) {
		ret = regmap_write(pchip->regmap,
				   REG_BRT_B, pchip->brightness_store);
		if (ret < 0) {
			dev_err(pchip->dev, "%s: i2c failed to access register REG_BRT_B\n", __func__);
			goto out;
		}

		ret = regmap_update_bits(pchip->regmap,
					REG_CUR_B, 0x1f, pdata->max_cur_b);
		if (ret < 0) {
			dev_err(pchip->dev, "%s: i2c failed to access register REG_CUR_B\n", __func__);
			goto out;
		}
	}
	return ret;

out:
	return ret;
}

/* interrupt handling */
static void lm3630_delayed_func(struct work_struct *work)
{
	int ret;
	unsigned int reg_val;
	struct lm3630_chip_data *pchip;

	pchip = container_of(work, struct lm3630_chip_data, work.work);

	ret = regmap_read(pchip->regmap, REG_INT_STATUS, &reg_val);
	if (ret < 0) {
		dev_err(pchip->dev,
			"i2c failed to access REG_INT_STATUS Register\n");
		return;
	}

	dev_info(pchip->dev, "REG_INT_STATUS Register is 0x%x\n", reg_val);
}

static irqreturn_t lm3630_isr_func(int irq, void *chip)
{
	int ret;
	struct lm3630_chip_data *pchip = chip;
	unsigned long delay = msecs_to_jiffies(INT_DEBOUNCE_MSEC);

	queue_delayed_work(pchip->irqthread, &pchip->work, delay);

	ret = regmap_update_bits(pchip->regmap, REG_CTRL, 0x80, 0x00);
	if (ret < 0)
		goto out;

	return IRQ_HANDLED;
out:
	dev_err(pchip->dev, "%s: i2c failed to access register\n", __func__);
	return IRQ_HANDLED;
}

static int lm3630_intr_config(struct lm3630_chip_data *pchip)
{
	INIT_DELAYED_WORK(&pchip->work, lm3630_delayed_func);
	pchip->irqthread = create_singlethread_workqueue("lm3630-irqthd");
	if (!pchip->irqthread) {
		dev_err(pchip->dev, "create irq thread fail...\n");
		return -1;
	}
	if (request_threaded_irq
	    (pchip->irq, NULL, lm3630_isr_func,
	     IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "lm3630_irq", pchip)) {
		dev_err(pchip->dev, "request threaded irq fail..\n");
		return -1;
	}
	return 0;
}

static bool
set_intensity(struct backlight_device *bl, struct lm3630_chip_data *pchip)
{
	if (!pchip->pdata->pwm_set_intensity)
		return false;
	pchip->pdata->pwm_set_intensity(bl->props.brightness - 1,
					pchip->pdata->pwm_period);
	return true;
}

/* update and get brightness */
static int lm3630_bank_a_update_status(struct backlight_device *bl)
{
	int ret;
	struct lm3630_chip_data *pchip = bl_get_data(bl);
	enum lm3630_pwm_ctrl pwm_ctrl = pchip->pdata->pwm_ctrl;

	/* brightness 0 means disable */
	pchip->brightness_store = (bl->props.brightness >> 3);
	if (!bl->props.brightness) {
		ret = regmap_update_bits(pchip->regmap, REG_CTRL, 0x04, 0x00);
		if (ret < 0)
			goto out;
		pchip->enabled = false;
		lm3630_power_on(pchip, false);
		return bl->props.brightness;
	}

	if (pchip->enabled  == false) {
		lm3630_power_on(pchip, true);
		lm3630_chip_init(pchip);
		pchip->enabled = true;
	}

	/* pwm control */
	if (pwm_ctrl == PWM_CTRL_BANK_A || pwm_ctrl == PWM_CTRL_BANK_ALL) {
		if (!set_intensity(bl, pchip))
			dev_err(pchip->dev, "No pwm control func. in plat-data\n");
	} else {
		/* i2c control */
		ret = regmap_update_bits(pchip->regmap, REG_CTRL, 0x80, 0x00);
		if (ret < 0)
			goto out;
		mdelay(1);
		ret = regmap_write(pchip->regmap,
				   REG_BRT_A, bl->props.brightness >> 3);
		if (ret < 0)
			goto out;
	}
	return bl->props.brightness;
out:
	dev_err(pchip->dev, "%s: i2c failed to access REG_CTRL\n", __func__);
	return bl->props.brightness;
}

static int lm3630_bank_a_get_brightness(struct backlight_device *bl)
{
	unsigned int reg_val;
	int brightness, ret;
	struct lm3630_chip_data *pchip = bl_get_data(bl);
	enum lm3630_pwm_ctrl pwm_ctrl = pchip->pdata->pwm_ctrl;

	if (pwm_ctrl == PWM_CTRL_BANK_A || pwm_ctrl == PWM_CTRL_BANK_ALL) {
		ret = regmap_read(pchip->regmap, REG_PWM_OUTHIGH, &reg_val);
		if (ret < 0)
			goto out;
		brightness = reg_val & 0x01;
		ret = regmap_read(pchip->regmap, REG_PWM_OUTLOW, &reg_val);
		if (ret < 0)
			goto out;
		brightness = ((brightness << 8) | reg_val) + 1;
	} else {
		ret = regmap_update_bits(pchip->regmap, REG_CTRL, 0x80, 0x00);
		if (ret < 0)
			goto out;
		mdelay(1);
		ret = regmap_read(pchip->regmap, REG_BRT_A, &reg_val);
		if (ret < 0)
			goto out;
		brightness = reg_val + 1;
	}
	bl->props.brightness = (brightness << 3);
	return bl->props.brightness;
out:
	dev_err(pchip->dev, "%s: i2c failed to access register\n", __func__);
	return 0;
}

static const struct backlight_ops lm3630_bank_a_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = lm3630_bank_a_update_status,
	.get_brightness = lm3630_bank_a_get_brightness,
};

static int lm3630_bank_b_update_status(struct backlight_device *bl)
{
	int ret;
	struct lm3630_chip_data *pchip = bl_get_data(bl);
	enum lm3630_pwm_ctrl pwm_ctrl = pchip->pdata->pwm_ctrl;

	pr_info("%s brightness %d\n ", __func__, bl->props.brightness);
	if (pwm_ctrl == PWM_CTRL_BANK_B || pwm_ctrl == PWM_CTRL_BANK_ALL) {
		if (!set_intensity(bl, pchip))
			dev_err(pchip->dev,
				"no pwm control func. in plat-data\n");
	} else {
		ret = regmap_update_bits(pchip->regmap, REG_CTRL, 0x80, 0x00);
		if (ret < 0)
			goto out;
		mdelay(1);
		ret = regmap_write(pchip->regmap,
				   REG_BRT_B, bl->props.brightness - 1);
	}
	return bl->props.brightness;
out:
	dev_err(pchip->dev, "%s: i2c failed to access register\n", __func__);
	return bl->props.brightness;
}

static int lm3630_bank_b_get_brightness(struct backlight_device *bl)
{
	unsigned int reg_val;
	int brightness, ret;
	struct lm3630_chip_data *pchip = bl_get_data(bl);
	enum lm3630_pwm_ctrl pwm_ctrl = pchip->pdata->pwm_ctrl;

	if (pwm_ctrl == PWM_CTRL_BANK_B || pwm_ctrl == PWM_CTRL_BANK_ALL) {
		ret = regmap_read(pchip->regmap, REG_PWM_OUTHIGH, &reg_val);
		if (ret < 0)
			goto out;
		brightness = reg_val & 0x01;
		ret = regmap_read(pchip->regmap, REG_PWM_OUTLOW, &reg_val);
		if (ret < 0)
			goto out;
		brightness = ((brightness << 8) | reg_val) + 1;
	} else {
		ret = regmap_update_bits(pchip->regmap, REG_CTRL, 0x80, 0x00);
		if (ret < 0)
			goto out;
		mdelay(1);
		ret = regmap_read(pchip->regmap, REG_BRT_B, &reg_val);
		if (ret < 0)
			goto out;
		brightness = reg_val + 1;
	}
	bl->props.brightness = brightness;

	return bl->props.brightness;
out:
	dev_err(pchip->dev, "%s: i2c failed to access register\n", __func__);
	return bl->props.brightness;
}

static const struct backlight_ops lm3630_bank_b_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = lm3630_bank_b_update_status,
	.get_brightness = lm3630_bank_b_get_brightness,
};

static int lm3630_backlight_register(struct lm3630_chip_data *pchip,
				     enum lm3630_leds ledno)
{
	const char *name = bled_name[ledno];
	struct backlight_properties props;
	struct lm3630_platform_data *pdata = pchip->pdata;

	props.type = BACKLIGHT_RAW;
	switch (ledno) {
	case BLED_1:
	case BLED_ALL:
		props.brightness = pdata->init_brt_led1;
		props.max_brightness = 2048;//pdata->max_brt_led1;
		pchip->bled1 =
		    backlight_device_register(name, pchip->dev, pchip,
					      &lm3630_bank_a_ops, &props);
		if (IS_ERR(pchip->bled1))
			return -EIO;
		break;
	case BLED_2:
		props.brightness = pdata->init_brt_led2;
		props.max_brightness = pdata->max_brt_led2;
		pchip->bled2 =
		    backlight_device_register(name, pchip->dev, pchip,
					      &lm3630_bank_b_ops, &props);
		if (IS_ERR(pchip->bled2))
			return -EIO;
		break;
	}
	return 0;
}

static void lm3630_backlight_unregister(struct lm3630_chip_data *pchip)
{
	if (pchip->bled1)
		backlight_device_unregister(pchip->bled1);
	if (pchip->bled2)
		backlight_device_unregister(pchip->bled2);
}

static const struct regmap_config lm3630_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = REG_MAX,
};
#ifdef CONFIG_HAS_EARLYSUSPEND
static void lm3630_early_suspend(struct early_suspend *handler)
{
	struct lm3630_chip_data *pchip = container_of(handler,
					struct lm3630_chip_data, early_suspend);

	pchip->suspended = true;
	regmap_update_bits(pchip->regmap, REG_CTRL, 0x87, 0x80);

	lm3630_power_on(pchip, false);
}
static void lm3630_late_resume(struct early_suspend *handler)
{
	struct lm3630_chip_data *pchip = container_of(handler,
					struct lm3630_chip_data, early_suspend);

	lm3630_power_on(pchip, true);
	lm3630_chip_init(pchip);
	pchip->suspended = false;
}
#endif
static int lm3630_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct lm3630_platform_data *pdata = client->dev.platform_data;
	struct lm3630_chip_data *pchip;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "fail : i2c functionality check...\n");
		return -EOPNOTSUPP;
	}

	if (pdata == NULL) {
		dev_err(&client->dev, "fail : no platform data.\n");
		return -ENODATA;
	}

	pchip = devm_kzalloc(&client->dev, sizeof(struct lm3630_chip_data),
			     GFP_KERNEL);
	if (!pchip)
		return -ENOMEM;

	pchip->pdata = pdata;
	pchip->dev = &client->dev;
	pchip->brightness_store = pdata->init_brt_led1;

	pchip->regmap = devm_regmap_init_i2c(client, &lm3630_regmap);
	if (IS_ERR(pchip->regmap)) {
		ret = PTR_ERR(pchip->regmap);
		dev_err(&client->dev, "fail : allocate register map: %d\n",
			ret);
		return ret;
	}
	i2c_set_clientdata(client, pchip);

	/* power get*/
	ret = lm3630_power_on(pchip, true);
	if (ret)
		goto err_bl_pwr;

	/* chip initialize */
	ret = lm3630_chip_init(pchip);
	if (ret < 0) {
		dev_err(&client->dev, "fail : init chip\n");
		lm3630_power_on(pchip, false);
		goto err_bl_pwr;
	}

	switch (pdata->bank_a_ctrl) {
	case BANK_A_CTRL_ALL:
		ret = lm3630_backlight_register(pchip, BLED_ALL);
		pdata->bank_b_ctrl = BANK_B_CTRL_DISABLE;
		break;
	case BANK_A_CTRL_LED1:
		ret = lm3630_backlight_register(pchip, BLED_1);
		break;
	case BANK_A_CTRL_LED2:
		ret = lm3630_backlight_register(pchip, BLED_2);
		pdata->bank_b_ctrl = BANK_B_CTRL_DISABLE;
		break;
	default:
		break;
	}

	if (ret < 0)
		goto err_bl_reg;

	if (pdata->bank_b_ctrl && pchip->bled2 == NULL) {
		ret = lm3630_backlight_register(pchip, BLED_2);
		if (ret < 0)
			goto err_bl_reg;
	}
	/* interrupt enable  : irq 0 is not allowed for lm3630 */
	pchip->irq = client->irq;
	if (pchip->irq)
		lm3630_intr_config(pchip);
#ifdef CONFIG_HAS_EARLYSUSPEND
	pchip->early_suspend.suspend = lm3630_early_suspend;
	pchip->early_suspend.resume = lm3630_late_resume;
	pchip->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	register_early_suspend(&pchip->early_suspend);
#endif

	dev_info(&client->dev, "LM3630 backlight register OK.\n");

	return 0;

err_bl_reg:
	dev_err(&client->dev, "fail : backlight register.\n");
	lm3630_backlight_unregister(pchip);
err_bl_pwr:
	if (pchip)
		kfree(pchip);
err_chip_init:
	return ret;
}

static int lm3630_remove(struct i2c_client *client)
{
	int ret;
	struct lm3630_chip_data *pchip = i2c_get_clientdata(client);

	ret = regmap_write(pchip->regmap, REG_BRT_A, 0);
	if (ret < 0)
		dev_err(pchip->dev, "%s: i2c failed to access register REG_BRT_A\n", __func__);

	ret = regmap_write(pchip->regmap, REG_BRT_B, 0);
	if (ret < 0)
		dev_err(pchip->dev, "%s: i2c failed to access register REG_BRT_B\n", __func__);

	lm3630_backlight_unregister(pchip);
	if (pchip->irq) {
		free_irq(pchip->irq, pchip);
		flush_workqueue(pchip->irqthread);
		destroy_workqueue(pchip->irqthread);
	}
	return 0;
}

static const struct i2c_device_id lm3630_id[] = {
	{LM3630_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, lm3630_id);

static struct i2c_driver lm3630_i2c_driver = {
	.driver = {
		   .name = LM3630_NAME,
		   },
	.probe = lm3630_probe,
	.remove = lm3630_remove,
	.id_table = lm3630_id,
};
static int __init lm3630_init(void)
{ 
	return i2c_add_driver(&lm3630_i2c_driver);
}
module_init(lm3630_init);

static void __exit lm3630_exit(void)
{ 
	i2c_del_driver(&lm3630_i2c_driver);
} 
module_exit(lm3630_exit);

MODULE_DESCRIPTION("Texas Instruments Backlight driver for LM3630");
MODULE_AUTHOR("G.Shark Jeong <gshark.jeong@gmail.com>");
MODULE_AUTHOR("Daniel Jeong <daniel.jeong@ti.com>");
MODULE_LICENSE("GPL v2");