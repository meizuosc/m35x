/*
* Simple driver for Texas Instruments TPS65132 LCD Voltage Supply driver chip
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
#include <linux/platform_data/tps65132.h>

struct tps65132_chip_data {
	struct device *dev;
	struct tps65132_pd_data *pdata;
	struct regmap *regmap;
};

/* initialize chip */
static int tps65132_chip_init(struct tps65132_chip_data *pchip)
{
	int ret;
	unsigned int reg_val;
	struct tps65132_pd_data *pdata = pchip->pdata;

	/* VPOS control */
	reg_val = pdata->vpos_val;

	ret = regmap_write(pchip->regmap, TPS65132_VPOS_REG, reg_val);
	if (ret < 0)
		goto out;

	/* VNEG control */
	reg_val = pdata->vneg_val;
	ret = regmap_write(pchip->regmap, TPS65132_VNEG_REG, reg_val);
	if (ret < 0)
		goto out;

	/* app dis control */
	reg_val = 0;
	reg_val |= ((pdata->app_val << 6) | 
			(pdata->disp_val << 1) | pdata->disn_val);

	ret = regmap_write(pchip->regmap, TPS65132_APP_DIS_REG, reg_val);
	if (ret < 0)
		goto out;
	/* set to the non-volatile eprom and wait*/
	reg_val = (1 << 7);
	ret = regmap_write(pchip->regmap, TPS65132_CTRL_REG, reg_val);
	if (ret < 0)
		goto out;

	return ret;

out:
	dev_err(pchip->dev, "i2c failed to access register ret %d\n", ret);
	return ret;
}

static const struct regmap_config tps65132_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
};

static int tps65132_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct tps65132_pd_data *pdata = client->dev.platform_data;
	struct tps65132_chip_data *pchip = NULL;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "fail : i2c functionality check...\n");
		return -EOPNOTSUPP;
	}

	if (pdata == NULL) {
		dev_err(&client->dev, "fail : no platform data.\n");
		return -ENODATA;
	}

	pchip = devm_kzalloc(&client->dev, sizeof(struct tps65132_chip_data),
			     GFP_KERNEL);
	if (!pchip)
		return -ENOMEM;
	pchip->pdata = pdata;
	pchip->dev = &client->dev;

	pchip->regmap = devm_regmap_init_i2c(client, &tps65132_regmap);
	if (IS_ERR(pchip->regmap)) {
		ret = PTR_ERR(pchip->regmap);
		dev_err(&client->dev, "fail : allocate register map: %d\n",
			ret);
		return ret;
	}
	i2c_set_clientdata(client, pchip);

	/* chip initialize */
	tps65132_chip_init(pchip);
	dev_info(&client->dev, "TPS65132 register OK.\n");
	return 0;

err_bl_reg:
	dev_err(&client->dev, "fail : backlight register.\n");
err_chip_init:
	return ret;
}

static int tps65132_remove(struct i2c_client *client)
{
	int ret;
	struct tps65132_pd_data *pchip = i2c_get_clientdata(client);

	return 0;
}

static const struct i2c_device_id tps_id[] = {
	{"tps-vol", 0},
	{}
};
static struct i2c_driver tps65132_i2c_driver = {
	.driver = {
	   .name = "tps-vol",
	},
	.probe = tps65132_probe,
	.remove = tps65132_remove,
	.id_table = tps_id,
};

module_i2c_driver(tps65132_i2c_driver);

MODULE_DESCRIPTION("Texas Instruments TPS65132 driver");
MODULE_AUTHOR("Cao Ziqiang <caoziqiang@meizu.com>");
MODULE_LICENSE("GPL v2");
