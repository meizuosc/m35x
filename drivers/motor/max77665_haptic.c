/*
 *  MAX77665-haptic controller driver
 *
 *  Copyright (C) 2012 Meizu Technology Co.Ltd, Zhuhai, China
 *  Chwei <chwei@meizu.com>
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
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/max77665.h>
#include <linux/mfd/max77665-private.h>
#include <linux/delay.h>
#include <linux/kdev_t.h>
#include <linux/timed_output.h>

#define HAPTIC_CONF2_PDIV_SHIFT		(0)
#define HAPTIC_CONF2_PDIV_MASK		(0x3<<HAPTIC_CONF2_PDIV_SHIFT)
#define HAPTIC_CONF2_HTYP_SHIFT		(5)
#define HAPTIC_CONF2_HTYP_MASK		(0x1<<HAPTIC_CONF2_HTYP_SHIFT)
#define HAPTIC_CONF2_MEN_SHIFT		(6)
#define HAPTIC_CONF2_MEN_MASK		(0x1<<HAPTIC_CONF2_MEN_SHIFT)
#define HAPTIC_CONF2_MODE_SHIFT		(7)
#define HAPTIC_CONF2_MODE_MASK		(0x1<<HAPTIC_CONF2_MODE_SHIFT)

#define MAX_TIMEOUT		1000
#define LSEN	(1<<7)	/* Low sys dac enable */
#define LSDEN	(0<<7)
#define LSEN_MASK	(1<<7)
#undef __CONFIG_DEBUG_HAPTIC__

struct haptic_data {
	struct device *dev;
	struct i2c_client *client;
	struct mutex haptic_mutex;
	struct timed_output_dev tdev;
	struct delayed_work disable_work;
	bool already_shutdown;
};

#ifdef __CONFIG_DEBUG_HAPTIC__
static void max77665_show_regs(struct i2c_client *i2c)
{
	struct haptic_data *haptic = container_of(i2c, struct haptic_data, client);
	struct max77665_dev *iodev = dev_get_drvdata(haptic->dev->parent);
	unsigned int i = 0;
	unsigned int val;

	pr_info("%s:reg 0~16=", __func__);
	for (i = 0; i < 17; i++) {
		max77665_read_reg(iodev->regmap_haptic, i, &val);
		pr_cont("0x%02x,", val);
	}
	pr_cont("\n");
}
#endif
static int max77665_haptic_on(struct haptic_data *chip, bool en)
{
	struct max77665_dev *iodev = dev_get_drvdata(chip->dev->parent);
	struct regmap *regmap = iodev->regmap;
	struct regmap *regmap_haptic = iodev->regmap_haptic;
	int ret = 0;
	int retry_count = 5;
	if(chip->already_shutdown)
		return;
	if (en) {
		pr_info("enable haptic motor.\n");
		do{
			ret = max77665_update_reg(regmap, MAX77665_PMIC_REG_LSCNFG,
					LSEN_MASK, LSEN);
			ret = max77665_update_reg(regmap_haptic,
					MAX77665_HAPTIC_REG_CONFIG2,
					HAPTIC_CONF2_MEN_MASK,
					(0x1 << HAPTIC_CONF2_MEN_SHIFT));
		}while(ret < 0 && retry_count-- > 0);
	} else {
		pr_info("disable haptic motor.\n");
		do{
			ret = max77665_update_reg(regmap, MAX77665_HAPTIC_REG_CONFIG2,
					HAPTIC_CONF2_MODE_MASK, (0x01 << HAPTIC_CONF2_MODE_SHIFT));
			ret = max77665_update_reg(regmap_haptic,
					MAX77665_HAPTIC_REG_CONFIG2,
					HAPTIC_CONF2_MEN_MASK,
					(0x0 << HAPTIC_CONF2_MEN_SHIFT));
			ret = max77665_update_reg(regmap, MAX77665_PMIC_REG_LSCNFG,
					LSEN_MASK, LSDEN);
			ret = max77665_update_reg(regmap, MAX77665_HAPTIC_REG_CONFIG2,
					HAPTIC_CONF2_MODE_MASK, (0x0 << HAPTIC_CONF2_MODE_SHIFT));
		}while(ret < 0 && retry_count-- > 0);
#ifdef __CONFIG_DEBUG_HAPTIC__
		max77665_show_regs(chip->client);
#endif
	}
	return ret;
}

static void motor_disable_work_func(struct work_struct *work)
{
	struct haptic_data *chip = container_of(work, struct haptic_data, disable_work.work);

#ifdef __CONFIG_DEBUG_HAPTIC__
	pr_info("write register to disable motor from delayed work\n");
#endif
	mutex_lock(&chip->haptic_mutex);
	max77665_haptic_on(chip, false);
	mutex_unlock(&chip->haptic_mutex);
}

static int haptic_get_time(struct timed_output_dev *tdev)
{
	return 0;
}

static void haptic_enable(struct timed_output_dev *tdev, int value)
{
	struct haptic_data *chip =
		container_of(tdev, struct haptic_data, tdev);

	mutex_lock(&chip->haptic_mutex);

	if(delayed_work_pending(&chip->disable_work))
		cancel_delayed_work(&chip->disable_work);

	if (value > 0) {
		value = min(value, MAX_TIMEOUT);
		schedule_delayed_work(&chip->disable_work,msecs_to_jiffies(value));
		max77665_haptic_on(chip, true);
	}else{
		max77665_haptic_on(chip, false);
	}
	mutex_unlock(&chip->haptic_mutex);
}

static __devinit int max77665_haptic_probe(struct platform_device *pdev)
{
	struct max77665_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct max77665_platform_data *max77665_pdata
		= dev_get_platdata(iodev->dev);
	struct max77665_haptic_platform_data *pdata
		= max77665_pdata->haptic_pdata;
	struct haptic_data *chip;
	unsigned int config = 0;
	int ret = 0;

	dev_info(&pdev->dev, "%s : MAX77665 Haptic Driver Loading\n", __func__);

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;
	chip->client = iodev->haptic;

	mutex_init(&chip->haptic_mutex);

	/* max77693_haptic_init */
	if (pdata->type == MAX77665_HAPTIC_LRA)
		config |= 1<<7;
	if (pdata->mode == MAX77665_INTERNAL_MODE)
		config |= 1<<5;
	config |= pdata->pwm_divisor;
	ret = max77665_write_reg(iodev->regmap_haptic, MAX77665_HAPTIC_REG_CONFIG2, config);

	chip->tdev.name = "vibrator";
	chip->tdev.get_time = haptic_get_time;
	chip->tdev.enable = haptic_enable;
	ret = timed_output_dev_register(&chip->tdev);
	INIT_DELAYED_WORK(&chip->disable_work, motor_disable_work_func);
	if (ret < 0) {
		pr_err("[VIB] Failed to register timed_output : %d\n", ret);
		ret = -EFAULT;
		goto err_timed_output;
	}
	platform_set_drvdata(pdev, chip);
	return 0;

err_timed_output:
	kfree(chip);
	return ret;
}

static int __devexit max77665_haptic_remove(struct platform_device *pdev)
{
	struct haptic_data *chip = platform_get_drvdata(pdev);

	mutex_destroy(&chip->haptic_mutex);
	platform_set_drvdata(pdev, NULL);
	timed_output_dev_unregister(&chip->tdev);
	if(chip)
		kfree(chip);
	return 0;
}

/*
	this function will be called when power off the machine,to avoid non-stop vibration problem when shutdown.
*/
static void max77665_haptic_shutdown(struct platform_device * pdev)
{
	struct haptic_data *chip = platform_get_drvdata(pdev);

	pr_info("%s disable motor when power off!", __func__);
	max77665_haptic_on(chip, false);
	chip->already_shutdown = true;
}

static int max77665_haptic_suspend(struct device *dev)
{
	struct haptic_data *chip = dev_get_drvdata(dev);
	max77665_haptic_on(chip, false);
	return 0;
}

static int max77665_haptic_resume(struct device *dev)
{
	struct haptic_data *chip = dev_get_drvdata(dev);
	max77665_haptic_on(chip, false);
	return 0;
}

static const struct dev_pm_ops max77665_haptic_pm_ops = {
	.suspend	= max77665_haptic_suspend,
	.resume	= max77665_haptic_resume,
};

static struct platform_driver max77665_haptic_driver = {
	.driver = {
		.name = "max77665-haptic",
		.owner = THIS_MODULE,
		.pm = &max77665_haptic_pm_ops,
	},
	.shutdown = max77665_haptic_shutdown,
	.probe = max77665_haptic_probe,
	.remove = __devexit_p(max77665_haptic_remove),
};

static int __init max77665_haptic_init(void)
{
	return platform_driver_register(&max77665_haptic_driver);
}

static void __exit max77665_haptic_exit(void)
{
	platform_driver_unregister(&max77665_haptic_driver);
}

module_init(max77665_haptic_init);
module_exit(max77665_haptic_exit);

MODULE_DESCRIPTION("MAXIM 77665 haptic control driver");
MODULE_AUTHOR("luochucheng@meizu.com");
MODULE_LICENSE("GPLV2");
