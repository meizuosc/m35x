/**
 * linux/drivers/modem/meizu_modem.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2010 Samsung Electronics.
 * Copyright (C) 2012 Zhuhai Meizu Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>

#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/wakelock.h>

#include <linux/bootmode.h>

#include <linux/platform_data/modem.h>
#include "modem_prj.h"
#include "modem_variation.h"
#include "modem_utils.h"

int modem_debug = 0;

static int __init modem_debug_setup(char *args)
{
	int error;
	unsigned long val;

	error = strict_strtoul(args, 0, &val);
	if (!error)
		modem_debug = val;

	return error;
}
__setup("modem_debug=", modem_debug_setup);

static struct modem_ctl *create_modemctl_device(struct platform_device *pdev)
{
	int ret = 0;
	struct modem_data *pdata;
	struct modem_ctl *modemctl;
	struct device *dev = &pdev->dev;

	modemctl = kzalloc(sizeof(struct modem_ctl), GFP_KERNEL);
	if (!modemctl)
		return NULL;

	modemctl->dev = dev;
	pdata = pdev->dev.platform_data;
	modemctl->mdm_data = pdata;
	modemctl->name = pdata->name;
	modemctl->modem_type = pdata->modem_type;
	wake_lock_init(&modemctl->wakelock, WAKE_LOCK_SUSPEND, "modemctl");

	/* init modemctl device for getting modemctl operations */
	ret = modem_init_device(modemctl, pdata);
	if (ret) {
		kfree(modemctl);
		return NULL;
	}

	modem_tty_driver_init(modemctl);

	MIF_INFO("%s is created!!!\n", pdata->name);

	return modemctl;
}

static struct io_device *create_io_device(struct modem_io_t *io_t,
		struct modem_ctl *modemctl, struct modem_data *pdata)
{
	int ret = 0;
	struct io_device *iod = NULL;

	iod = kzalloc(sizeof(struct io_device), GFP_KERNEL);
	if (!iod) {
		MIF_ERR("iod == NULL\n");
		return NULL;
	}

	INIT_LIST_HEAD(&iod->list);
	iod->name = io_t->name;
	iod->id = io_t->id;
	iod->io_typ = io_t->io_type;
	iod->link_type = io_t->link;
	iod->phone_net_type = pdata->modem_net;
	atomic_set(&iod->opened, 0);
	wake_lock_init(&iod->wakelock, WAKE_LOCK_SUSPEND, iod->name);

	iod->mc = modemctl;

	/* link current io device to link deivce */
	set_current_link(iod, modemctl->ld);
	list_add_tail(&iod->list, &modemctl->ld->link_dev_list);

	/* register misc device or net device */
	ret = meizu_ipc_init_io_device(iod);
	if (ret) {
		kfree(iod);
		MIF_ERR("meizu_ipc_init_io_device fail (%d)\n", ret);
		return NULL;
	}

	MIF_DEBUG("%s is created!!!\n", iod->name);
	return iod;
}

static int __devinit modem_probe(struct platform_device *pdev)
{
	int i;
	struct modem_data *pdata = pdev->dev.platform_data;
	enum modem_t mdm_type = platform_get_device_id(pdev)->driver_data;
	struct modem_ctl *modemctl;
	struct io_device *iod[pdata->num_iodevs];
	struct link_device *ld;

	if (pdata->modem_type != mdm_type) {
		pr_err("%s: register error modem type\n", __func__);
		return -EINVAL;
	}

	memset(iod, 0, sizeof(iod));

	modemctl = create_modemctl_device(pdev);
	if (!modemctl) {
		MIF_ERR("modemctl == NULL\n");
		goto err_free_modemctl;
	}

	platform_set_drvdata(pdev, modemctl);

	modemctl->rx_wq = create_singlethread_workqueue("modem_rx_wq");
	if (!modemctl->rx_wq) {
		MIF_ERR("fail to create wq\n");
		return -EINVAL;
	}

	ld = init_link_device(pdev, pdata->link_type);
	if (!ld)
		goto err_free_modemctl;

	MIF_ERR("link created: %s\n", ld->name);
	ld->mc = modemctl;
	modemctl->ld = ld;

	/* create io deivces and connect to modemctl device */
	for (i = 0; i < pdata->num_iodevs; i++) {
		iod[i] = create_io_device(&pdata->iodevs[i], modemctl, pdata);
		if (!iod[i]) {
			MIF_ERR("iod[%d] == NULL\n", i);
			goto err_free_modemctl;
		}
	}

	modemctl->modem_state_changed(modemctl, MODEM_EVENT_BOOT_INIT);
	MIF_INFO("Complete!!!\n");

	return 0;

err_free_modemctl:
	for (i = 0; i < pdata->num_iodevs; i++)
		if (iod[i] != NULL)
			kfree(iod[i]);

	if (modemctl != NULL)
		kfree(modemctl);

	return -ENOMEM;
}

static void modem_shutdown(struct platform_device *pdev)
{
	struct modem_ctl *mc = platform_get_drvdata(pdev);

	free_irq(mc->irq_modem_reset, mc);
	mc->ops.modem_off(mc);
	mc->modem_state_changed(mc, MODEM_EVENT_POWEROFF);
}

static int modem_suspend(struct device *dev)
{
	struct modem_ctl *mc = dev_get_drvdata(dev);

	if (mc->modem_type == IMC_XMM6260) {
		disable_irq(mc->irq_hostwake);
		irq_set_irq_type(mc->irq_hostwake, IRQF_NO_SUSPEND |\
				IRQF_TRIGGER_LOW | IRQF_ONESHOT);
	}

	return 0;
}

static int modem_resume(struct device *dev)
{
	struct modem_ctl *mc = dev_get_drvdata(dev);

	if (mc->modem_type == IMC_XMM6260) {
		irq_set_irq_type(mc->irq_hostwake, IRQF_NO_SUSPEND |\
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING);
		enable_irq(mc->irq_hostwake);
	}

	return 0;
}

static const struct dev_pm_ops modem_pm_ops = {
	.suspend = modem_suspend,
	.resume  = modem_resume,
};

static struct platform_device_id modem_driver_ids[] = {
	{
		.name		= "modem_ifx_6260",
		.driver_data	= IMC_XMM6260,
	}, {
		.name		= "modem_spt_8803",
		.driver_data	= IMC_SC8803G,
	},
	{ }
};
MODULE_DEVICE_TABLE(platform, modem_driver_ids);

static struct platform_driver modem_driver = {
	.probe    = modem_probe,
	.shutdown = modem_shutdown,
	.id_table = modem_driver_ids,
	.driver   = {
		.name = "meizu_modem",
		.pm   = &modem_pm_ops,
	},
};

static int __init modem_driver_init(void)
{
	if(is_charging_mode())
		return 0;

	return platform_driver_register(&modem_driver);
}

module_init(modem_driver_init);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("KarlZheng<zhengkl@meizu.com>");
MODULE_DESCRIPTION("Meizu Modem Interface Driver");
