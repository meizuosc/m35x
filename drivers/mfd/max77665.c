/*
 * max77665.c - mfd core driver for the MAX 77665
 *
 * Copyright (C) 2012 Meizu Technology Co.Ltd, Zhuhai, China
 *
 * This program is not provided / owned by Maxim Integrated Products.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * This driver is based on max8997.c
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max77665.h>
#include <linux/mfd/max77665-private.h>
#include <linux/regulator/machine.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/err.h>

#define I2C_ADDR_PMIC	(0xCC >> 1)	/* Charger, Flash LED */
#define I2C_ADDR_MUIC	(0x4A >> 1)	/* MUIC */
#define I2C_ADDR_HAPTIC	(0x90 >> 1)	/* Haptic Moto */

static struct mfd_cell max77665_devs[] = {
	{ .name = "max77665-charger", },
	{ .name = "max77665-haptic", },
	{ .name = "torch-led", },
	{ .name = "max77665-safeout", },
	{ .name = "max77665-muic", },
};

static bool max77665_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX77665_PMIC_REG_INTSRC:
	case MAX77665_PMIC_REG_TOPSYS_INT:
	case MAX77665_PMIC_REG_TOPSYS_STAT:
	case MAX77665_CHG_REG_CHG_INT:
	case MAX77665_CHG_REG_CHG_INT_OK:
	case MAX77665_CHG_REG_CHG_DETAILS_00:
	case MAX77665_CHG_REG_CHG_DETAILS_01:
	case MAX77665_CHG_REG_CHG_DETAILS_02:
	case MAX77665_CHG_REG_CHG_DETAILS_03:
	case MAX77665_LED_REG_FLASH_INT:
	case MAX77665_LED_REG_FLASH_INT_STATUS:
		return true;
	default:
		return false;
	}
}

static bool max77665_muic_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX77665_MUIC_REG_ID:
	case MAX77665_MUIC_REG_INT1:
	case MAX77665_MUIC_REG_INT2:
	case MAX77665_MUIC_REG_INT3:
	case MAX77665_MUIC_REG_STATUS1:
	case MAX77665_MUIC_REG_STATUS2:
	case MAX77665_MUIC_REG_STATUS3:
	case MAX77665_MUIC_REG_INTMASK1:
	case MAX77665_MUIC_REG_INTMASK2:
	case MAX77665_MUIC_REG_INTMASK3:
	case MAX77665_MUIC_REG_CDETCTRL1:
	case MAX77665_MUIC_REG_CDETCTRL2:
	case MAX77665_MUIC_REG_CTRL1:
	case MAX77665_MUIC_REG_CTRL2:
	case MAX77665_MUIC_REG_CTRL3:
		return true;
	default:
		return false;
	}
}

struct regmap_config max77665_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.max_register = MAX77665_PMIC_REG_END,
	.volatile_reg = max77665_volatile_register,
};

struct regmap_config max77665_muic_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.max_register = MAX77665_PMIC_REG_END,
	.volatile_reg = max77665_muic_volatile_register,
};

struct regmap_config max77665_haptic_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX77665_HAPTIC_REG_END,
};

int max77665_read_reg(struct regmap *regmap, unsigned int reg, unsigned int *val)
{
	int ret;

	ret = regmap_read(regmap, reg, val);
	if (ret < 0)
		return ret;

	return 0;
}
EXPORT_SYMBOL_GPL(max77665_read_reg);

int max77665_bulk_read(struct regmap *regmap, unsigned int reg,
		u8 *val, size_t val_count)
{
	int ret;

	ret = regmap_bulk_read(regmap, reg, val, val_count);
	
	if (ret < 0)
		return ret;

	return 0;
}
EXPORT_SYMBOL_GPL(max77665_bulk_read);

int max77665_write_reg(struct regmap *regmap, unsigned int reg, unsigned int val)
{
	int ret;

	ret = regmap_write(regmap, reg, val);
	if (ret < 0)
		return ret;

	return 0;
}
EXPORT_SYMBOL_GPL(max77665_write_reg);

int max77665_bulk_write(struct regmap *regmap, unsigned int reg, 
		unsigned int *val, size_t val_count)
{
	int ret;

	ret = regmap_bulk_write(regmap, reg, val, val_count);
	if (ret < 0)
		return ret;

	return 0;
}
EXPORT_SYMBOL_GPL(max77665_bulk_write);

int max77665_update_reg(struct regmap *regmap, unsigned int reg,
		unsigned int mask, unsigned int val)
{
	int ret;

	ret = regmap_update_bits(regmap, reg, mask, val);

	return ret;
}
EXPORT_SYMBOL_GPL(max77665_update_reg);

static int max77665_flash_setup(struct max77665_dev *max77665)
{
	int ret = 0;
	struct regmap *regmap = max77665->regmap;

	/*set led boost mode*/
	ret |= regmap_write(regmap, MAX77665_LED_REG_VOUT_CNTL,
		MAX77665_BOOST_FLED_FIXED_MODE | MAX77665_BOOST_FLED_BOTH_USED);
	
	/*set flash1 voltage, flash2 voltage is read-only accoding to the datasheet*/
	ret |= regmap_write(regmap, MAX77665_LED_REG_VOUT_FLASH1,
			MAX77665_BOOST_VOUT_FLASH_FROM_VOLT(5000));

	/*set flash timer*/
	ret |= regmap_write(regmap, MAX77665_LED_REG_FLASH_TIMER,
		MAX77665_FLASH_TIME_1000MS | MAX77665_FLASH_MAX_TIMER_MODE);

	/*set torch timer*/
	ret |= regmap_write(regmap, MAX77665_LED_REG_ITORCH_TIMER,
		MAX77665_TORCH_TIMER_DISABLE);

	/*default shutdown flash and torch mode*/
	ret |= regmap_write(regmap, MAX77665_LED_REG_FLASH_EN, 0);

	return ret;
}

static int max77665_i2c_probe(struct i2c_client *i2c,
			      const struct i2c_device_id *id)
{
	struct max77665_dev *max77665;
	struct max77665_platform_data *pdata = i2c->dev.platform_data;
	unsigned int reg_data;
	int ret = 0;

	max77665 = devm_kzalloc(&i2c->dev,
			sizeof(struct max77665_dev), GFP_KERNEL);
	if (max77665 == NULL)
		return -ENOMEM;

	max77665->regmap = devm_regmap_init_i2c(i2c, &max77665_regmap_config);	
	if (IS_ERR(max77665->regmap)) {
		ret = PTR_ERR(max77665->regmap);
		dev_err(max77665->dev, "failed to register regmap\n");
		goto err;
	}

	i2c_set_clientdata(i2c, max77665);
	max77665->dev = &i2c->dev;
	max77665->i2c = i2c;
	max77665->irq = i2c->irq;
	max77665->type = id->driver_data;

	if (!pdata)
		goto err;

	max77665->irq_base = pdata->irq_base;
	max77665->wakeup = pdata->wakeup;

	mutex_init(&max77665->iolock);

	/* Ensure I2C chip is active */
	if (regmap_read(max77665->regmap, MAX77665_PMIC_REG_PMIC_ID2, &reg_data) < 0) {
		dev_err(max77665->dev,
			"device not found on this channel\n");
		ret = -ENODEV;
		goto err;
	} else {
		dev_info(max77665->dev, "device ID: 0x%x\n", reg_data);
	}

	max77665->haptic = i2c_new_dummy(i2c->adapter, I2C_ADDR_HAPTIC);
	i2c_set_clientdata(max77665->haptic, max77665);

	max77665->muic = i2c_new_dummy(i2c->adapter, I2C_ADDR_MUIC);
	i2c_set_clientdata(max77665->muic, max77665);

	/*
	 * Initialize register map for MUIC device because use regmap-muic
	 * instance of MUIC device when irq of max77665 is initialized
	 * before call max77665-muic probe() function.
	 */
	max77665->regmap_muic = devm_regmap_init_i2c(max77665->muic,
					 &max77665_muic_regmap_config);
	if (IS_ERR(max77665->regmap_muic)) {
		ret = PTR_ERR(max77665->regmap_muic);
		dev_err(max77665->dev,
			"failed to allocate register map: %d\n", ret);
		goto err;
	}

	/*
	 * Initialize register map for HAPTIC device because use regmap-haptic
	 * instance of HAPTIC device when irq of max77665 is initialized
	 * before call max77665-haptic probe() function.
	 */
	max77665->regmap_haptic = devm_regmap_init_i2c(max77665->haptic,
					 &max77665_haptic_regmap_config);
	if (IS_ERR(max77665->regmap_haptic)) {
		ret = PTR_ERR(max77665->regmap_haptic);
		dev_err(max77665->dev,
			"failed to allocate register map: %d\n", ret);
		goto err;
	}

	ret = max77665_irq_init(max77665);
	if (ret < 0)
		goto err_mfd;

	pm_runtime_set_active(max77665->dev);

	ret = mfd_add_devices(max77665->dev, -1, max77665_devs,
			ARRAY_SIZE(max77665_devs), NULL, 0);
	if (ret < 0)
		goto err_mfd;

	device_init_wakeup(max77665->dev, pdata->wakeup);

	max77665_flash_setup(max77665);

	/* Enabe max77665 manual reset,
	 * set manual reset debounce timer = 6second
	 */
	max77665_update_reg(max77665->regmap, MAX77665_PMIC_REG_MAINCTRL1,
			MAX77665_PMIC_MRDBTIMER_MASK | MAX77665_PMIC_MREN_MASK,
			(MAX77665_MRDBTMER_6SEC << MAX77665_PMIC_MRDBTIMER_SHIFT)
			| (1 << MAX77665_PMIC_MREN_SHIFT));

	return ret;

err_mfd:
	i2c_unregister_device(max77665->muic);
	i2c_unregister_device(max77665->haptic);
err:
	kfree(max77665);
	return ret;
}

static int max77665_i2c_remove(struct i2c_client *i2c)
{
	struct max77665_dev *max77665 = i2c_get_clientdata(i2c);

	mfd_remove_devices(max77665->dev);
	i2c_unregister_device(max77665->muic);
	i2c_unregister_device(max77665->haptic);
	kfree(max77665);

	return 0;
}

static const struct i2c_device_id max77665_i2c_id[] = {
	{ "max77665", TYPE_MAX77665 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max77665_i2c_id);

#ifdef CONFIG_PM

extern int rtcalarm_wakeup_en;

static int max77665_suspend(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct max77665_dev *max77665 = i2c_get_clientdata(i2c);

	disable_irq(max77665->irq);

	if (device_may_wakeup(dev) && rtcalarm_wakeup_en)
		enable_irq_wake(max77665->irq);

	return 0;
}

static int max77665_resume(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct max77665_dev *max77665 = i2c_get_clientdata(i2c);

	if (device_may_wakeup(dev) && rtcalarm_wakeup_en)
		disable_irq_wake(max77665->irq);

	enable_irq(max77665->irq);

	return max77665_irq_resume(max77665);
}
#else
#define max77665_suspend	NULL
#define max77665_resume		NULL
#endif /* CONFIG_PM */

const struct dev_pm_ops max77665_pm = {
	.suspend = max77665_suspend,
	.resume = max77665_resume,
};

static struct i2c_driver max77665_i2c_driver = {
	.driver = {
		   .name = "max77665",
		   .owner = THIS_MODULE,
		   .pm = &max77665_pm,
	},
	.probe = max77665_i2c_probe,
	.remove = max77665_i2c_remove,
	.id_table = max77665_i2c_id,
};

static int __init max77665_i2c_init(void)
{
	return i2c_add_driver(&max77665_i2c_driver);
}
/* init early so consumer devices can complete system boot */
subsys_initcall(max77665_i2c_init);

static void __exit max77665_i2c_exit(void)
{
	i2c_del_driver(&max77665_i2c_driver);
}
module_exit(max77665_i2c_exit);

MODULE_DESCRIPTION("MAXIM 77665 multi-function core driver");
MODULE_AUTHOR("Sukdong Kim <Sukdong.Kim@samsung.com>");
MODULE_LICENSE("GPLV2");
