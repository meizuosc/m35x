/*  
 * arch/arm/mach-exynos/meizu_sys.c
 * 
 * meizu technology particular classes
 *
 * Author : Li Tao <litao@meizu.com>
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/err.h>


struct class *meizu_class;
EXPORT_SYMBOL_GPL(meizu_class);
struct class *sensors_class;
EXPORT_SYMBOL_GPL(sensors_class);

#if 0
/*
 * Create sysfs interface
 */
static void set_sensor_attr(struct device *dev,
	struct device_attribute *attributes[])
{
	int i;
	
	if (!attributes)
		return;

	for (i = 0; attributes[i] != NULL; i++) {
		if ((device_create_file(dev, attributes[i])) < 0)
			pr_err("[sensors class] failed to device_create_file"\
				"(dev, attributes[%d])\n", i);
	}
}

int sensors_register(struct device *dev, void *drvdata,
	struct device_attribute *attributes[], char *name)
{
	int ret = 0;

	if (!sensors_class) {
		sensors_class = class_create(THIS_MODULE, "sensors");
		if (IS_ERR(sensors_class))
			return PTR_ERR(sensors_class);
	}
	
	dev = device_create(sensors_class, NULL, 0, drvdata, "%s", name);
	
	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		pr_err("[sensors class] device_create failed [%d]\n", ret);
		return ret;
	}
	
	set_sensor_attr(dev, attributes);

	return 0;
}
EXPORT_SYMBOL_GPL(sensors_register);

void sensors_unregister(struct device *dev,
	struct device_attribute *attributes[])
{
	int i;

	for (i = 0; attributes[i] != NULL; i++)
		device_remove_file(dev, attributes[i]);
}
EXPORT_SYMBOL_GPL(sensors_unregister);
#endif

static int __init meizu_class_init(void)
{
	pr_info("%s\n", __func__);
        meizu_class = class_create(THIS_MODULE, "meizu");

        if (IS_ERR(meizu_class)) {
                pr_err("%s, create meizu_class failed.(err=%ld)\n",
                        __func__, IS_ERR(meizu_class));
                return PTR_ERR(meizu_class);
        }
	
	sensors_class = class_create(THIS_MODULE, "sensors");

        if (IS_ERR(meizu_class)) {
		pr_err("%s, create sensors_class failed.(err=%ld)\n",
			__func__, IS_ERR(meizu_class));
		return PTR_ERR(meizu_class);
	}
	
	return 0;
}

static void __exit meizu_class_exit(void)
{
	class_destroy(meizu_class);
	meizu_class = NULL;
	class_destroy(sensors_class);
	sensors_class = NULL;
}

subsys_initcall(meizu_class_init);
module_exit(meizu_class_exit);

MODULE_DESCRIPTION("Meizu technology particular class");
MODULE_AUTHOR("Li Tao <litao@meizu.com>");
MODULE_LICENSE("GPL");
