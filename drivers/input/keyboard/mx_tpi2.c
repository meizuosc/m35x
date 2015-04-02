/*
 * Touchpanel interface for meizu m6x
 *
 * Copyright (C) 2013 Meizu Technology Co.Ltd, Zhuhai, China
 * Author:		
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/gpio.h>
#include <linux/i2c-gpio.h>
#include <plat/gpio-cfg.h>
#include <mach/gpio-meizu.h>
#include <linux/firmware.h>
#include <linux/mfd/mx-hub.h>

struct mx_tpi_data {
	struct device *dev;
	struct mx_hub_dev *data;
	struct i2c_client *client;
	unsigned int irq;
	
	struct input_dev *input_key;
	struct mutex iolock;
	struct wake_lock wake_lock;
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

#define	K_HOME		0
#define	K_UNLOCK	1
#define	K_SLIDE		2

#define KEY_HOME_S		102
#define KEY_HOME_D		129
#define KEY_HOME_L		108
#define KEY_UNLOCK		103
#define KEY_SLIDE		108


static irqreturn_t mx_tpi_key_irq(int irq, void *dev_id)
{
	struct mx_tpi_data * tpi = dev_id;
	int key;
	int ret;
	
	key = KEY_SLIDE;
	ret = mx_hub_readbyte(tpi->client,MX_HUB_REG_WAKEUP_KEY);
	if(ret < 0)
		goto exit;
	
	key = ret & 0xFF;
	if( (key != KEY_SLIDE) &&  (key != KEY_UNLOCK)
		&&  (key != KEY_HOME_S)
		&&  (key != KEY_HOME_D)
		&&  (key != KEY_HOME_L))
	{
		goto exit;
	}
	
	if(tpi->data->debug)
		pr_info( "%s:gesture key = %d\n",__func__,key);
	
	if( (key != KEY_SLIDE) &&  (key != KEY_UNLOCK))
		key = KEY_SLIDE;

	if( tpi->input_key )
	{
		input_report_key(tpi->input_key,key,1);
		input_sync(tpi->input_key);
		input_report_key(tpi->input_key,key,0);
		input_sync(tpi->input_key);
	}

exit:
	pr_info( "%s:key  %d\n",__func__,key);	

	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
 static void mx_tpi_early_suspend(struct early_suspend *h)
 {
	 //struct mx_tpi_data *tpi =
	//		 container_of(h, struct mx_tpi_data, early_suspend);
 }
 
 static void mx_tpi_late_resume(struct early_suspend *h)
 {
	 //struct mx_tpi_data *tpi =
	//		 container_of(h, struct mx_tpi_data, early_suspend);
 }
#endif


static int __devinit mx_tpi_probe(struct platform_device *pdev)
{
	struct mx_hub_dev *hub = dev_get_drvdata(pdev->dev.parent);
	struct mx_sensor_hub_platform_data *pdata = dev_get_platdata(hub->dev);
	struct mx_tpi_data *tpi;
	struct input_dev *input_key;
	
	int err = 0,i;
	
	pr_debug("%s:++\n",__func__);
	
	tpi = kzalloc(sizeof(struct mx_tpi_data), GFP_KERNEL);
	if (!tpi) {
		dev_err(&pdev->dev, "insufficient memory\n");
		err = -ENOMEM;
		return err;
	}

	tpi->client = hub->client;
	tpi->data = hub;
	tpi->dev = &pdev->dev;
	
	mutex_init(&tpi->iolock);
	wake_lock_init(&tpi->wake_lock, WAKE_LOCK_SUSPEND, pdev->name);
		
	 input_key = input_allocate_device();
	 if (!input_key) {
		 dev_err(&pdev->dev, "insufficient memory\n");
		 err = -ENOMEM;
		 goto err_free_mem;
	 }
	 
	 tpi->input_key = input_key;	 
	 input_key->name = "mxhub-keys";
	 input_key->dev.parent = &pdev->dev;
	 input_key->id.bustype = BUS_I2C;
	 input_key->id.vendor = 0x1111;
	 
	input_key->keycodesize = sizeof(pdata->keymap[0]);
	input_key->keycodemax = pdata->nbuttons;
	input_key->keycode = pdata->keymap;
	 	
	 __set_bit(EV_KEY, input_key->evbit);
	 
	for (i = 0; i < pdata->nbuttons; i++)
		__set_bit(pdata->keymap[i], input_key->keybit);
	
	input_set_drvdata(input_key, tpi);

	/* Register the input_key device */
	err = input_register_device(tpi->input_key);
	if (err) {
		dev_err(&pdev->dev, "Failed to register input key device\n");
		goto err_free_mem;

	}	 	
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	 tpi->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN -5;
	 tpi->early_suspend.suspend = mx_tpi_early_suspend;
	 tpi->early_suspend.resume = mx_tpi_late_resume;
	 register_early_suspend(&tpi->early_suspend);
#endif

	tpi->irq = pdata->irq_base + MX_HUB_IRQ_KEY;
	err = request_threaded_irq(tpi->irq, NULL, mx_tpi_key_irq,
			IRQF_ONESHOT, input_key->name, tpi);
	if (err) {
		dev_err(&pdev->dev, "failed to register keypad interrupt\n");
		goto err_free_irq;
	}


	platform_set_drvdata(pdev, tpi);
	
#ifdef	CONFIG_TOUCHSCREEN_DSX_SWIPE_CONTROL	
	mx_hub_setkeytype(hub,MX_KEY_GESTURE2);
#endif

#if 0		
	err = mx_hub_writebyte(hub->client,MX_HUB_REG_GESTURE_Y_UNMASK,0b01111100);
	if (err) {
		dev_err(&pdev->dev, "failed to set y axis unmask \n");
	}
#endif	
	
	pr_debug("%s:--\n",__func__);
	return 0;
	
err_free_irq:	
	free_irq(tpi->irq,input_key);

err_free_mem:	
	pr_err("%s:init failed! \n",__func__);
	mutex_destroy(&tpi->iolock);
	wake_lock_destroy(&tpi->wake_lock);
	kfree(tpi);
	return err;
}

#ifdef CONFIG_PM
static int mx_tpi_suspend(struct device *dev)
{
	//struct mx_tpi_data * tpi  = dev_get_drvdata(dev);

	//disable_irq(tpi->irq);

	return 0;
}

static int mx_tpi_resume(struct device *dev)
{
	//struct mx_tpi_data * tpi  = dev_get_drvdata(dev);
	
	//enable_irq(tpi->irq);

	return 0;
}

#else
#define mx_tpi_suspend	NULL
#define mx_tpi_resume	NULL
#endif /* CONFIG_PM */

const struct dev_pm_ops mx_tpi_pm = {
	.suspend = mx_tpi_suspend,
	.resume = mx_tpi_resume,
};

static int __devexit mx_tpi_remove(struct platform_device *pdev)
{
	struct mx_tpi_data * tpi  = platform_get_drvdata(pdev);
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&tpi->early_suspend);
#endif

	mutex_destroy(&tpi->iolock);
	wake_lock_destroy(&tpi->wake_lock);
	
	kfree(tpi);

	return 0;
}

static const struct platform_device_id mx_tpi_id[] = {
	{ "mx_tpi", 0 },
	{ },
};

static struct platform_driver mx_tpi_driver = {
	.driver	= {
		.name	= "mx_tpi",
		.owner	= THIS_MODULE,
		.pm = &mx_tpi_pm,
	},
	.id_table	= mx_tpi_id,
	.probe		= mx_tpi_probe,
	.remove		= __devexit_p(mx_tpi_remove),
};

static int __init mx_tpi_init(void)
{
	return platform_driver_register(&mx_tpi_driver);
}
module_init(mx_tpi_init);

static void __exit mx_tpi_exit(void)
{
	platform_driver_unregister(&mx_tpi_driver);
}
module_exit(mx_tpi_exit);

MODULE_AUTHOR("Chwei <Chwei@meizu.com>");
MODULE_DESCRIPTION("MX touchpanel interface");
MODULE_LICENSE("GPL");
