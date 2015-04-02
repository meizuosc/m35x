/*
 * Broadcom gps driver, Author: heljoy liu [heljoy@meizu.com]
 * This is based UBLOX gps driver of jerrymo@meizu.com
 * Copyright (c) 2010 meizu Corporation
 *
 */

#define pr_fmt(fmt)	"BRCM_GPS: " fmt

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/gpio.h>
#include <mach/gpio.h>
#include <mach/gpio-meizu.h>
#include <mach/gpio-common.h>
#include <plat/gpio-cfg.h>

static int gps_power;

static ssize_t gps_power_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	int value;

	value = gpio_get_value(gps_power);
	pr_debug("%s():power %d\n", __func__, value);
	return sprintf(buf, "%d\n",  value);
}

static ssize_t gps_power_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	unsigned long value = simple_strtoul(buf, NULL, 10);

	pr_debug("%s():power %ld.\n", __func__, value);

	gpio_set_value(gps_power, !!value);

	return count;
}

static DEVICE_ATTR(pwr, S_IRUGO | S_IWUSR,
		   gps_power_show, gps_power_store);

static struct attribute *gps_attributes[] = {
	&dev_attr_pwr.attr,
	NULL
};

static struct attribute_group gps_attribute_group = {
	.attrs = gps_attributes
};

static int __devinit gps_probe(struct platform_device *pdev)
{
	int ret;

	gps_power = MEIZU_GPS_PWR_ON;

	ret = sysfs_create_group(&pdev->dev.kobj, &gps_attribute_group);
	if (ret < 0) {
		pr_err("%s():sys create group fail !!\n", __func__);
		return ret;
	}

	ret = gpio_request(gps_power, "GPS_PWR");
	if (ret) {
		pr_err("%s():fail to request gpio (GPS_nRST)\n", __func__);
		goto gpio_req;
	}

	s3c_gpio_setpull(gps_power, GPIO_PULL_NONE);
	gpio_direction_output(gps_power, 0);

	if (mx_is_factory_test_mode(MX_FACTORY_TEST_BT)) {
		s3c_gpio_cfgpin(MEIZU_GPS_RTS, S3C_GPIO_INPUT);
		s3c_gpio_cfgpin(MEIZU_GPS_CTS, S3C_GPIO_INPUT);
		s3c_gpio_cfgpin(MEIZU_GPS_RXD, S3C_GPIO_INPUT);
		s3c_gpio_cfgpin(MEIZU_GPS_TXD, S3C_GPIO_INPUT);
		s3c_gpio_setpull(MEIZU_GPS_RTS, GPIO_PULL_DOWN);
		s3c_gpio_setpull(MEIZU_GPS_CTS, GPIO_PULL_DOWN);
		s3c_gpio_setpull(MEIZU_GPS_RXD, GPIO_PULL_DOWN);
		gpio_direction_output(gps_power, 1);
		printk("GPS in test mode!\n");
	}

	pr_info("gps successfully probed!\n");

	return 0;

gpio_req:
	sysfs_remove_group(&pdev->dev.kobj, &gps_attribute_group);

	return ret;
}

static int __devexit gps_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &gps_attribute_group);
	gpio_free(gps_power);

	return 0;
}

static void gps_shutdown(struct platform_device *pdev)
{

	if (gpio_get_value(gps_power))
		gpio_set_value(gps_power, 0);
}
/*platform driver data*/
static struct platform_driver gps_driver = {
	.driver = {
		.name = "bcm4752-gps",
		.owner = THIS_MODULE,
	},
	.probe =   gps_probe,
	.remove = __devexit_p(gps_remove),
	.shutdown = gps_shutdown,
};

static int __init gps_init(void)
{
	return platform_driver_register(&gps_driver);
}

static void __exit gps_exit(void)
{
	platform_driver_unregister(&gps_driver);
}

module_init(gps_init);
module_exit(gps_exit);

MODULE_AUTHOR("Jianping Liu <heljoy@meizu.com>");
MODULE_DESCRIPTION("broadcom gps driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
