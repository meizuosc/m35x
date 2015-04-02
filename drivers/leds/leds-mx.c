/*
 * Led driver for meizu m6x
 *
 * Copyright (C) 2012 -2013 Meizu Technology Co.Ltd, Zhuhai, China
 * Author:		
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
	  
#include <linux/module.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/mfd/mx-hub.h>

#define	MODE_CURRENT			0x00
#define	MODE_SLOPE				0x01
#define	MODE_PWM				0x02
#define	MODE_FADE				0x03
#define	MODE_SLP_TOP			0x04
#define	MODE_FUNC				0x05

#define	GET_MODE(x)	((x>>8)&0x0F)

#define AUTO_SLOPE_AST	(1<<6)


 /*led private data*/
struct mx_hub_led {
	 struct mx_hub_dev *data;
	 struct led_classdev led_cdev;
	 int	 brightness;
	 struct mutex mutex;
	 int id;
#ifdef CONFIG_HAS_EARLYSUSPEND
	 struct early_suspend early_suspend;
#endif
};
static int gSlope = 0;
static int gPWM = 0;
 
 static int mx_hub_set_led_current(struct led_classdev *led_cdev, 
		 enum led_brightness cur)
 {
	 struct mx_hub_led *led =
			 container_of(led_cdev, struct mx_hub_led, led_cdev);
	 struct mx_hub_dev *hub = led->data;
	 int ret = 0;
	 	
	pr_debug("%s: cur  = 0x%X\n", __func__,cur); 
	
	ret = mx_hub_writebyte(hub->client,MX_HUB_REG_LED_BRN,cur);  

	return ret;
}
 
static int mx_hub_set_led_pwm(struct led_classdev *led_cdev,
		 enum led_brightness pwm)
{
	int ret = 0;
	
	gPWM = pwm;

	pr_debug("%s: pwm  = 0x%X\n", __func__,pwm); 

	ret = mx_hub_set_led_current(led_cdev,pwm);

	return ret;
}
 
static int mx_hub_set_led_slope(struct led_classdev *led_cdev, 
	int enable)
{
	struct mx_hub_led *led =
		 container_of(led_cdev, struct mx_hub_led, led_cdev);
	struct mx_hub_dev *hub = led->data;
	int ret = 0;

	 gSlope = !!enable;

	pr_debug("%s: enable  = 0x%X\n", __func__,enable); 

	ret = mx_hub_writebyte(hub->client,MX_HUB_REG_LED_SLP,enable);  
	//ret = mx_hub_writebyte(hub->client,MX_HUB_REG_LED_SLP,enable| AUTO_SLOPE_AST ); 

	return ret;
}  

static int mx_hub_set_led_fade(struct led_classdev *led_cdev, 
	int enable)
{
	struct mx_hub_led *led =
		 container_of(led_cdev, struct mx_hub_led, led_cdev);
	struct mx_hub_dev *hub = led->data;
	int ret = 0;

	pr_debug("%s: enable  = 0x%X\n", __func__,enable); 

	ret = mx_hub_writebyte(hub->client,MX_HUB_REG_LED_FADE,enable);  

	return ret;
} 

static int mx_hub_set_led_slope_top(struct led_classdev *led_cdev, 
	enum led_brightness brightness)
{
	struct mx_hub_led *led =
		 container_of(led_cdev, struct mx_hub_led, led_cdev);
	struct mx_hub_dev *hub = led->data;
	int ret = 0;

	pr_debug("%s: brightness  = 0x%X\n", __func__,brightness); 

	ret = mx_hub_writebyte(hub->client,MX_HUB_REG_LED_SLPTOP,brightness);  

	return ret;
} 

static int mx_hub_set_func_enable(struct led_classdev *led_cdev, 
	int enable)
{
	struct mx_hub_led *led =
		 container_of(led_cdev, struct mx_hub_led, led_cdev);
	struct mx_hub_dev *hub = led->data;
	int ret = 0;

	pr_info("%s: enable  = 0x%X\n", __func__,enable); 

	ret = mx_hub_writebyte(hub->client,MX_HUB_REG_LED_FUNC_EN,enable);  

	return ret;
} 

static void mx_hub_led_brightness_set(struct led_classdev *led_cdev,
		 enum led_brightness value)
{
	struct mx_hub_led *led =container_of(led_cdev, struct mx_hub_led, led_cdev);
	//struct mx_hub_dev *hub = led->data;
	int ret = 0;
	int mode;
	int data;

	dev_info(led_cdev->dev, "value = 0x%.4X \n",value);

	mode = GET_MODE(value);
	data = value & 0xFF; 

	switch( mode )
	{
	case MODE_CURRENT:
		led->brightness = data;
		ret = mx_hub_set_led_current(led_cdev,data);
		break;

	case MODE_SLOPE:
		ret = mx_hub_set_led_slope(led_cdev,data);
		break;

	case MODE_PWM:
		ret = mx_hub_set_led_pwm(led_cdev,data);
		break;

	case MODE_FADE:
		ret = mx_hub_set_led_fade(led_cdev,data);
		break;

	case MODE_SLP_TOP:
		ret = mx_hub_set_led_slope_top(led_cdev,data);
		break;
		
	case MODE_FUNC:
		ret = mx_hub_set_func_enable(led_cdev,data);
		break;

	default:		
		dev_err(led_cdev->dev, "mode  %d is valite \n",mode);
		ret = -EINVAL;			
		break;
	}

	if(ret < 0)
		dev_err(led_cdev->dev, "brightness set failed ret = %d \n",ret);

}
 
#ifdef CONFIG_HAS_EARLYSUSPEND
static void mx_hub_led_early_suspend(struct early_suspend *h)
{
	 struct mx_hub_led *led =
			 container_of(h, struct mx_hub_led, early_suspend);
	
	dev_dbg(led->led_cdev.dev, "early suspend \n");
}
 
static void mx_hub_led_late_resume(struct early_suspend *h)
{
	 struct mx_hub_led *led =
			 container_of(h, struct mx_hub_led, early_suspend);
	 
	dev_dbg(led->led_cdev.dev, "early resume \n");
}
#endif
 
static int __devinit mx_hub_led_probe(struct platform_device *pdev)
{
	 struct mx_hub_dev *hub = dev_get_drvdata(pdev->dev.parent);
	struct mx_sensor_hub_platform_data *pdata = dev_get_platdata(hub->dev);
	 struct mx_hub_led *led;
	 char name[20];
	 int ret;
 
	 if (!pdata) {
		 dev_err(&pdev->dev, "no platform data\n");
		 ret = -ENODEV;
	 }
 
	 led = kzalloc(sizeof(*led), GFP_KERNEL);
	 if (led == NULL) {
		 ret = -ENOMEM;
		 goto err_mem;
	 }
 
	 led->id = pdev->id; 
	 led->data = hub;
 
	 //snprintf(name, sizeof(name), "%s", pdev->name);
	 snprintf(name, sizeof(name), "mx-led");
	 led->led_cdev.name = name;
	 led->led_cdev.brightness = 0;
	 led->led_cdev.max_brightness= 0xFFF;
	 
	 led->led_cdev.brightness_set = mx_hub_led_brightness_set;
 
	 mutex_init(&led->mutex);
	 platform_set_drvdata(pdev, led);
 
	 ret = led_classdev_register(&pdev->dev, &led->led_cdev);
	 if (ret < 0)
		 goto err_register_led;
 
#ifdef CONFIG_HAS_EARLYSUSPEND
	 led->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	 led->early_suspend.suspend = mx_hub_led_early_suspend;
	 led->early_suspend.resume = mx_hub_led_late_resume;
	 register_early_suspend(&led->early_suspend);
#endif

	mx_hub_led_brightness_set(&led->led_cdev,MODE_SLOPE << 8);
	mx_hub_led_brightness_set(&led->led_cdev,led->led_cdev.brightness);
 	
	 return 0;
 
 err_register_led:
	 kfree(led);
 err_mem:
	 return ret;
}
 
static int __devexit mx_hub_led_remove(struct platform_device *pdev)
{
	 struct mx_hub_led *led = platform_get_drvdata(pdev);
 
#ifdef CONFIG_HAS_EARLYSUSPEND
	 unregister_early_suspend(&led->early_suspend);
#endif
	 led_classdev_unregister(&led->led_cdev);
	 kfree(led);
 
	 return 0;
}
 
const struct platform_device_id mx_hub_led_id[] = {
	 { "mx-hub-led",0 },
	 { },
};
 
static struct platform_driver mx_hub_led_driver = {
	 .driver = {
		 .name	= "mx-led",
		 .owner = THIS_MODULE,
	 },
	 .probe  = mx_hub_led_probe,
	 .remove = __devexit_p(mx_hub_led_remove),
	 .id_table = mx_hub_led_id,
};
 
static int __init mx_hub_led_init(void)
{
	return platform_driver_register(&mx_hub_led_driver);
}
module_init(mx_hub_led_init);
 
static void __exit mx_hub_led_exit(void)
{
	platform_driver_unregister(&mx_hub_led_driver);
}
module_exit(mx_hub_led_exit); 


MODULE_AUTHOR("Chwei <Chwei@meizu.com>");
MODULE_DESCRIPTION("MX Sensor Hub Leds");
MODULE_LICENSE("GPL");
