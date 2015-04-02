/*
 * max77665a-muic.c - MUIC driver for the Maxim 77665
 *
 *  Copyright (C) 2012 Meizu Technology Co.Ltd
 *  <jgmai@meizu.com>
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
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/mfd/max77665.h>
#include <linux/mfd/max77665-private.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <plat/gpio-cfg.h>
#include <plat/devs.h>
#include <linux/bootmode.h>

#define DEV_NAME	"max77665-muic"

enum {
	ADC_GND		= 0x00,
	ADC_UART1	= 0x11,	//ID 50K
	ADC_UART2	= 0x0C, //ID 20k
	ADC_REMOVE	= 0x10, //ID 20K
	UART_REMOVE	= 0x15, //ID 50K
	ADC_OPEN	= 0x1f,
};

enum {
	TO_UART		= 0,
	TO_USB		= 1,
	TO_BB_USB	= 2,
	TO_BB_UART	= 3,
	ENABLE_FIXED	= 10,
	DISABLE_FIXED	= 11,
};

struct max77665_muic_info {
	struct device		*dev;
	struct max77665_dev	*max77665;
	struct regmap *muic;
	int host_insert;
	struct regulator *reverse;
	struct regulator *safeout2; //BB_VBUS for BB debug
};

static struct max77665_muic_info *g_info;
static bool fixed_switch = false;
static struct class *otg_device = NULL;

static int init_max77665_muic(struct max77665_muic_info *info)
{
	int ret;
	struct regmap *regmap_muic = info->muic;
	u8 val, msk, adc;
	int status1;

	max77665_read_reg(regmap_muic, MAX77665_MUIC_REG_STATUS1, &status1);
	adc = status1 & STATUS1_ADC_MASK;
	if (adc == ADC_UART1 || adc == ADC_UART2) {
		/* switch to UART */
		val = (0x3 << COMN1SW_SHIFT) | (0x3 << COMP2SW_SHIFT);
	} else {
		/* switch to USB */
		val = (0x1 << COMN1SW_SHIFT) | (0x1 << COMP2SW_SHIFT);
	}

	val |= (0 << MICEN_SHIFT) | (0 << IDBEN_SHIFT);
	msk = COMN1SW_MASK | COMP2SW_MASK | MICEN_MASK | IDBEN_MASK;
	ret = max77665_update_reg(regmap_muic, MAX77665_MUIC_REG_CTRL1, msk, val);
	if (ret) {
		pr_err("[MUIC]: can not update reg_ctrl1\n");
		return ret;
	}

	val = msk = 0;
	val = (0 << ADCEN_SHIFT) | (0 << LOWPWR_SHIFT);
	msk = ADCEN_MASK | LOWPWR_SHIFT;
	ret = max77665_update_reg(regmap_muic, MAX77665_MUIC_REG_CTRL2, msk, val);
	if (ret) {
		pr_err("[MUIC]: can not disable adc\n");
		return ret;
	}

	val = msk = 0;
	val = (1 << ADCEN_SHIFT) | (1 << LOWPWR_SHIFT);
	msk = ADCEN_MASK | LOWPWR_SHIFT;
	ret = max77665_update_reg(regmap_muic, MAX77665_MUIC_REG_CTRL2, msk, val);
	if (ret) {
		pr_err("[MUIC]: can not enable adc\n");
		return ret;
	}

	val = msk = 0;
	val = (0x2 << ADCDBSET_SHIFT); /*set adc debounce time to 25ms*/
	msk = ADCDBSET_MASK;
	ret = max77665_update_reg(regmap_muic, MAX77665_MUIC_REG_CTRL3, msk, val);
	if (ret) {
		pr_err("[MUIC]: can not set adc debounce time\n");
		return ret;
	}

	return ret;
}

static int usb_switch(int usb)
{
	struct max77665_muic_info *info = g_info;
	u8 val, msk;

	if (!fixed_switch) {
		pr_info("MUIC switch to --> %s\n", (usb == TO_UART)? "Uart":"USB");
	} else {
		pr_info("MUIC fixed_switch, Can not be changed!!!\n");
		goto out;
	}

	msk = COMN1SW_MASK | COMP2SW_MASK;

	if (usb == TO_USB) {
		val = (0x1 << COMN1SW_SHIFT) | (0x1 << COMP2SW_SHIFT);
	} else if (usb == TO_UART) {
		val = (0x3 << COMN1SW_SHIFT) | (0x3 << COMP2SW_SHIFT);
	} else {
		goto out;
	}

	max77665_update_reg(info->muic, MAX77665_MUIC_REG_CTRL1, msk, val);

	msleep(100);
out:
	return 0;
}

static irqreturn_t max77665_muic_isr(int irq, void *dev_id)
{
	struct max77665_muic_info *info = dev_id;
	u8 adc, adclow ,adcerr, adc1k;
	int status1;

	max77665_read_reg(info->muic, MAX77665_MUIC_REG_STATUS1, &status1);
	adc = status1 & STATUS1_ADC_MASK;
	adclow = !!(status1 & STATUS1_ADCLOW_MASK);
	adcerr = !!(status1 & STATUS1_ADCERR_MASK);
	adc1k = !!(status1 & STATUS1_ADC1K_MASK);

	pr_info("adc 0x%02x, adclow %d, adcerr %d, adc1k %d\n",
			adc, adclow, adcerr, adc1k);

	if (adc == ADC_GND) {
		if(!info->host_insert) {
			pr_info("otg connect\n");

			if(!regulator_is_enabled(info->reverse))
				regulator_enable(info->reverse);
			info->host_insert = true;
		}
	} else if (adc == ADC_REMOVE || adc == ADC_OPEN || adc == UART_REMOVE) {
		if (info->host_insert) {
			pr_info("otg disconnect\n");

			if(regulator_is_enabled(info->reverse))
				regulator_disable(info->reverse);

			info->host_insert = false;
		}
	}
	
	if (adc == ADC_UART1 || adc == ADC_UART2)
		usb_switch(TO_UART);
	else
		usb_switch(TO_USB);

	return IRQ_HANDLED;
}

static ssize_t debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct max77665_muic_info *info = g_info;
	int ret;
	u8 val, msk;
	unsigned long debug;

	ret = strict_strtoul(buf, 10, &debug);
	if (ret)
		return count;

	switch (debug) {
	case TO_USB:
		val = (0x1 << COMN1SW_SHIFT) | (0x1 << COMP2SW_SHIFT);
		break;
	case TO_UART:
		val = (0x3 << COMN1SW_SHIFT) | (0x3 << COMP2SW_SHIFT);
		break;
	case TO_BB_USB:
		if (!regulator_is_enabled(info->safeout2))
			regulator_enable(info->safeout2);
		val = (0x4 << COMN1SW_SHIFT) | (0x4 << COMP2SW_SHIFT);
		break;
	case TO_BB_UART:
		val = (0x5 << COMN1SW_SHIFT) | (0x5 << COMP2SW_SHIFT);
		break;
	case ENABLE_FIXED:
		pr_info("%s-> enable fixed switch\n", __func__);
		fixed_switch = true;
		goto out;
	case DISABLE_FIXED:
		pr_info("%s-> disable fixed switch\n", __func__);
		fixed_switch = false;
		goto out;
	default:
		pr_err("switch channel failed\n");
		goto out;
	}

	if (debug != TO_BB_USB && regulator_is_enabled(info->safeout2))
		regulator_disable(info->safeout2);

	msk = COMN1SW_MASK | COMP2SW_MASK;

	ret = max77665_update_reg(info->muic, MAX77665_MUIC_REG_CTRL1, msk, val);
out:
	return count;
}

static ssize_t debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct max77665_muic_info *info = g_info;
	int usb_dm, usb_dp;
	int val;

	max77665_read_reg(info->muic, MAX77665_MUIC_REG_CTRL1, &val);
	usb_dm = (val & COMN1SW_MASK) >> COMN1SW_SHIFT;
	usb_dp = (val & COMP2SW_MASK) >> COMP2SW_SHIFT;

	return sprintf(buf, "usb_dm %d usb_dp %d\n", usb_dm, usb_dp);
}

static DEVICE_ATTR(debug, S_IRUGO|S_IWUSR|S_IWGRP,
		   				debug_show, debug_store);

static ssize_t otg_show(struct class *class, struct class_attribute *attr, char *buf)
{
	struct max77665_muic_info *info = g_info;

	sprintf(buf, "%d\n", info->host_insert);
}
static CLASS_ATTR(otg, S_IRUGO | S_IWUSR | S_IWGRP, otg_show, NULL);

static int __devinit max77665_muic_probe(struct platform_device *pdev)
{
	struct max77665_dev *max77665 = dev_get_drvdata(pdev->dev.parent);
	struct max77665_muic_info *info;
	int ret = 0;
	int irq;

	pr_info("func:%s\n", __func__);

	info = kzalloc(sizeof(struct max77665_muic_info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "%s: failed to allocate info\n", __func__);
		ret = -ENOMEM;
		goto err_return;
	}

	g_info = info;

	info->dev = &pdev->dev;
	info->max77665 = max77665;
	info->muic = max77665->regmap_muic;

	info->reverse = regulator_get(NULL, "reverse");
	if (IS_ERR(info->reverse)) {
		dev_err(&pdev->dev, "Failed to get reverse regulator\n");
		goto fail0;
	}

	info->safeout2 = regulator_get(NULL, "safeout2");
	if (IS_ERR(info->safeout2)) {
		dev_err(&pdev->dev, "Failed to get safeout2 regulator\n");
		regulator_put(info->reverse);
		goto fail0;
	}

	platform_set_drvdata(pdev, info);

	ret = init_max77665_muic(info);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to initialize MUIC:%d\n", ret);
		goto fail1;
	}

	irq = max77665->irq_base + MAX77665_MUIC_IRQ_INT1_ADC1K;
	ret = request_threaded_irq(irq, 0, max77665_muic_isr,
			0, "max77665_adc1k", info);

	irq = max77665->irq_base + MAX77665_MUIC_IRQ_INT1_ADCERR;
	ret = request_threaded_irq(irq, 0, max77665_muic_isr,
			0, "max77665_adcerr", info);

	irq = max77665->irq_base + MAX77665_MUIC_IRQ_INT1_ADCLOW;
	ret = request_threaded_irq(irq, 0, max77665_muic_isr,
			0, "max77665_adclow", info);

	irq = max77665->irq_base + MAX77665_MUIC_IRQ_INT1_ADC;
	ret = request_threaded_irq(irq, 0, max77665_muic_isr,
			0, "max77665_adc", info);

	/* create sysfs attributes */
	ret = sysfs_create_file(&pdev->dev.kobj, &dev_attr_debug.attr);
	if (ret < 0) {
		pr_debug("sysfs_create_group failed\n");
	}

	/* create the OTG interface for production testing */
	otg_device = class_create(THIS_MODULE, "otg_device");
	class_create_file(otg_device, &class_attr_otg);

	return 0;

fail1:
	regulator_put(info->safeout2);
	regulator_put(info->reverse);
fail0:
	platform_set_drvdata(pdev, NULL);
	kfree(info);
err_return:
	return ret;
}

static int __devexit max77665_muic_remove(struct platform_device *pdev)
{
	struct max77665_muic_info *info = platform_get_drvdata(pdev);

	if (info) {
		dev_info(info->dev, "func:%s\n", __func__);
		regulator_put(info->safeout2);
		platform_set_drvdata(pdev, NULL);
		kfree(info);
	}
	return 0;
}

void max77665_muic_shutdown(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct max77665_muic_info *info = platform_get_drvdata(pdev);

	if (regulator_is_enabled(info->safeout2))
		regulator_disable(info->safeout2);

	return ;
}

#ifdef CONFIG_PM
static int max77665_muic_suspend(struct device *dev)
{
#if 0
	int val = 0x55;
	int i;
	struct platform_device *pdev = to_platform_device(dev);
	struct max77665_muic_info *info = platform_get_drvdata(pdev);

	pr_info("%s##############\n", __func__);

	for (i = MAX77665_MUIC_REG_INT1; i < MAX77665_MUIC_REG_END; i++) {
		max77665_read_reg(info->muic, i, &val);
		pr_info("Reg[%d]: %02x\n", i, val);
	}
#endif
	return 0;
}

static int max77665_muic_resume(struct device *dev)
{
#ifdef CONFIG_MEIZU_M65_V3
	//int val = 0x55;
	//int i;
	u8 val, msk, adc;
	int status1;
	struct platform_device *pdev = to_platform_device(dev);
	struct max77665_muic_info *info = platform_get_drvdata(pdev);

	pr_info("%s##############\n", __func__);

	//max77665_write_reg(info->muic, MAX77665_MUIC_REG_CTRL2, 0x1C);
	max77665_write_reg(info->muic, MAX77665_MUIC_REG_INTMASK1, 0x0F);

	max77665_read_reg(info->muic, MAX77665_MUIC_REG_STATUS1, &status1);
	adc = status1 & STATUS1_ADC_MASK;
	if (adc == ADC_UART1 || adc == ADC_UART2) {
		val = (0x3 << COMN1SW_SHIFT) | (0x3 << COMP2SW_SHIFT);
	} else {
		val = (0x1 << COMN1SW_SHIFT) | (0x1 << COMP2SW_SHIFT);
	}
	msk = COMN1SW_MASK | COMP2SW_MASK;
	max77665_update_reg(info->muic, MAX77665_MUIC_REG_CTRL1, msk, val);

#if 0
	for (i = MAX77665_MUIC_REG_INT1; i < MAX77665_MUIC_REG_END; i++) {
		max77665_read_reg(info->muic, i, &val);
		pr_info("Reg[%d]: %02x\n", i, val);
	}
#endif
#endif /* CONFIG_MEIZU_M65_V3 */
	return 0;
}

static const struct dev_pm_ops max77665_pm_ops = {
	.suspend        = max77665_muic_suspend,
	.resume		= max77665_muic_resume,
};
#endif

static struct platform_driver max77665_muic_driver = {
	.driver		= {
		.name	= DEV_NAME,
		.owner	= THIS_MODULE,
		.shutdown = max77665_muic_shutdown,
#ifdef CONFIG_PM
		.pm       =  &max77665_pm_ops,
#endif
	},
	.probe		= max77665_muic_probe,
	.remove		= __devexit_p(max77665_muic_remove),
};

static int __init max77665_muic_init(void)
{
	if(is_charging_mode()){
		return 0;
	}
	pr_info("func:%s\n", __func__);
	return platform_driver_register(&max77665_muic_driver);
}
module_init(max77665_muic_init);

static void __exit max77665_muic_exit(void)
{
	pr_info("func:%s\n", __func__);
	platform_driver_unregister(&max77665_muic_driver);
}
module_exit(max77665_muic_exit);

MODULE_DESCRIPTION("Maxim MAX77665 MUIC driver");
MODULE_AUTHOR("<sukdong.kim@samsung.com>");
MODULE_LICENSE("GPL");
