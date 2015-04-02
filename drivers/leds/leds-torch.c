#include <linux/module.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <plat/adc.h>

/*
* Modify maximum torch current to 105mA for m06x's security.
*/
#define TORCH_MAX_CURRENT 105000   /* unit uA */
#define HW_CURRENT_LIMIT_MAX 250000

struct torch_led {
	struct led_classdev cdev;
	struct regulator *regulator;
	bool is_enabled;
};

struct torch_led_attr {
	struct attribute attr;
	ssize_t (*show)(struct kobject *kobj,
			struct attribute *attr, char *buf);
	ssize_t (*store)(struct kobject *a, struct attribute *b,
			 const char *c, size_t count);
};

struct torch_led_temp_code_tbl {
	int temp;
	unsigned int code;
};

static struct torch_led_temp_code_tbl temp_code_tbl[] =
{
	{
		.temp = -40,
		.code = 18850,
	},
	{
		.temp = -35,
		.code = 14429,
	},
	{
		.temp = -30,
		.code = 11133,
	},
	{
		.temp = -25,
		.code = 8656,
	},
	{
		.temp = -20,
		.code = 6779,
	},
	{
		.temp = -15,
		.code = 5346,
	},
	{
		.temp = -10,
		.code = 4245,
	},
	{
		.temp = -5,
		.code = 3393,
	},
	{
		.temp = 0,
		.code = 2728,
	},
	{
		.temp = 5,
		.code = 2207,
	},
	{
		.temp = 10,
		.code = 1796,
	},
	{
		.temp = 15,
		.code = 1470,
	},
	{
		.temp = 20,
		.code = 1209,
	},
	{
		.temp = 25,
		.code = 1000,
	},
	{
		.temp = 30,
		.code = 831,
	},
	{
		.temp = 35,
		.code = 694,
	},
	{
		.temp = 40,
		.code = 583,
	},
	{
		.temp = 45,
		.code = 491,
	},
	{
		.temp = 50,
		.code = 416,
	},
	{
		.temp = 55,
		.code = 354,
	},
	{
		.temp = 60,
		.code = 302,
	},
	{
		.temp = 65,
		.code = 259,
	},
	{
		.temp = 70,
		.code = 223,
	},
	{
		.temp = 75,
		.code = 192,
	},
	{
		.temp = 80,
		.code = 167,
	},
	{
		.temp = 85,
		.code = 145,
	},
	{
		.temp = 90,
		.code = 127,
	},
	{
		.temp = 95,
		.code = 111,
	},
	{
		.temp = 100,
		.code = 97,
	},
	{
		.temp = 105,
		.code = 86,
	},
	{
		.temp = 110,
		.code = 76,
	},
	{
		.temp = 115,
		.code = 67,
	},
	{
		.temp = 120,
		.code = 59,
	},
	{
		.temp = 125,
		.code = 53,
	},

};

static struct s3c_adc_client *torch_led_adc;

static ssize_t show_torch_led_temperature(struct kobject *kobj,
			     struct attribute *attr, char *buf)
{
	unsigned int adcCode, code;
	int i;
	int retry_time = 0;

DO_RETRY:	
	adcCode = s3c_adc_read(torch_led_adc, 2);
	code = (unsigned int)((1000 * adcCode) / (4096 - adcCode));

	retry_time++;
	
	for (i = ARRAY_SIZE(temp_code_tbl) - 1; i >= 0; i--)
	{
		if (code <= temp_code_tbl[i].code)
			break;
	}
	if (i < 0)
	{
		if (retry_time <= 3)
			goto DO_RETRY;
		else
		{
			pr_err("%s():led get temperature failed\n", __FUNCTION__);
			return -EINVAL;
		}
	}
	return sprintf(buf, "%d\n", temp_code_tbl[i].temp);
}

static struct torch_led_attr torch_led_temperature =
		__ATTR(temperature, 0444, show_torch_led_temperature, NULL);

static void torch_led_generate_event(struct torch_led *led, bool enable)
{
	char *envp[2];

	if (enable) {
		envp[0] = "TORCH_LED_ON";
	} else {
		envp[0] = "TORCH_LED_OFF";
	}

	envp[1] = NULL;
	pr_info("%s(), send uevent\n", __func__);
	kobject_uevent_env(&led->cdev.dev->kobj, KOBJ_CHANGE, envp);
}

static int torch_led_enable(struct torch_led *led, bool enable)
{
	int ret = 0;

	if (led->is_enabled == enable) return 0;

	if (enable) 
		ret = regulator_enable(led->regulator);
	else 
		ret = regulator_disable(led->regulator);

	if (!ret) {
		torch_led_generate_event(led, enable);
		led->is_enabled = enable;
	}

	return ret;
}

static int torch_led_set_current(struct torch_led *led,
				enum led_brightness value)
{
	return regulator_set_current_limit(led->regulator, value, HW_CURRENT_LIMIT_MAX);
}

static void torch_led_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	struct torch_led *led =
			container_of(led_cdev, struct torch_led, cdev);
	int ret;
	pr_info("%s(), torch current is set to %dmA\n", __func__, value);
	ret = torch_led_set_current(led, value * 1000);  /*change current in uA unit*/
	if (ret) {
		pr_err("%s(), failed to torch_led_set_current: %d\n", __func__, ret);
	} else {
		torch_led_enable(led, !!value);
	}
}

static int __devinit torch_led_probe(struct platform_device *pdev)
{
	struct torch_led *led;
	struct regulator *regulator;
	int ret = 0;

	led = kzalloc(sizeof(*led), GFP_KERNEL);
	if (led == NULL) {
		pr_err("%s():request memory failed\n", __FUNCTION__);
		return -ENOMEM;
	}

	regulator = regulator_get(NULL, "torch_led");
	if (IS_ERR(regulator)) {
		pr_err("%s():regulator get failed\n", __FUNCTION__);
		ret = -EINVAL;
		goto err_regulator_get;
	}

	led->regulator = regulator;
	led->is_enabled = false;
	led->cdev.name = "torch_led";
	led->cdev.brightness_set = torch_led_brightness_set;
	//led->cdev.flags |= LED_CORE_SUSPENDRESUME;
	led->cdev.brightness = 0;
	led->cdev.max_brightness = TORCH_MAX_CURRENT/1000;
	platform_set_drvdata(pdev, led);

	ret = led_classdev_register(&pdev->dev, &led->cdev);
	if (ret < 0) {
		pr_err("%s():led register failed\n", __FUNCTION__);
		goto err_led_register;
	}

	torch_led_adc = s3c_adc_register(pdev, NULL, NULL, 0);
	if (IS_ERR(torch_led_adc)) {
		pr_err("%s, led temp get the adc failed\n", __FUNCTION__);
		goto err_led_register;
	}

	ret = sysfs_create_file(&led->cdev.dev->kobj, &torch_led_temperature.attr);
	if (ret) {
		pr_err("%s():led create temperature file failed\n", __FUNCTION__);
		goto err_led_register;
	}

	dev_info(&pdev->dev,"probed\n");
	
	return 0;

err_led_register:
	regulator_put(led->regulator);
err_regulator_get:
	kfree(led);
	return ret;
}
static void torch_led_shutdown(struct platform_device *pdev)
{
	struct torch_led *led = platform_get_drvdata(pdev);
	
	if (led) {
		if (regulator_is_enabled(led->regulator))
			regulator_disable(led->regulator);
	}
}

static int __devexit torch_led_remove(struct platform_device *pdev)
{
	struct torch_led *led = platform_get_drvdata(pdev);

	regulator_put(led->regulator);
	led_classdev_unregister(&led->cdev);
	kfree(led);

	return 0;
}

static struct platform_driver torch_led_driver = {
	.driver = {
		.name  = "torch-led",
		.owner = THIS_MODULE,
	},
	.probe  = torch_led_probe,
	.shutdown = torch_led_shutdown,
	.remove = __devexit_p(torch_led_remove),
};

static int __init torch_led_init(void)
{
	return platform_driver_register(&torch_led_driver);
}
module_init(torch_led_init);

static void __exit torch_led_exit(void)
{
	platform_driver_unregister(&torch_led_driver);
}
module_exit(torch_led_exit);


MODULE_AUTHOR("Jerry Mo");
MODULE_DESCRIPTION("MAX8997 and MAX77665 TORCH LED driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:torch-led");
