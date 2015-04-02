/*
 * Interrupt controller support for MAX8998
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mfd/mx-hub.h>

struct mx_hub_irq_data {
	int reg;
	int mask;
};

static struct mx_hub_irq_data mx_hub_irqs[] = {
	[MX_HUB_IRQ_KEY] = {
		.reg = 1,
		.mask = MX_HUB_IRQ_KEY_MASK,
	},
	[MX_HUB_IRQ_COMPASS] = {
		.reg = 1,
		.mask = MX_HUB_IRQ_COMPASS_MASK,
	},
	[MX_HUB_IRQ_ACC] = {
		.reg = 1,
		.mask = MX_HUB_IRQ_ACC_MASK,
	},
	[MX_HUB_IRQ_GYR] = {
		.reg = 1,
		.mask = MX_HUB_IRQ_GYR_MASK,
	},
};

static inline struct mx_hub_irq_data *
irq_to_mx_hub_irq(struct mx_hub_dev *mx_hub, int irq)
{
	return &mx_hub_irqs[irq - mx_hub->irq_base];
}

static void mx_hub_irq_lock(struct irq_data *data)
{
	struct mx_hub_dev *mx_hub = irq_data_get_irq_chip_data(data);

	mutex_lock(&mx_hub->irqlock);
}

static void mx_hub_irq_sync_unlock(struct irq_data *data)
{
	struct mx_hub_dev *mx_hub = irq_data_get_irq_chip_data(data);

	/*
	 * If there's been a change in the mask write it back
	 * to the hardware.
	 */
	if (mx_hub->irq_masks_cur != mx_hub->irq_masks_cache) {
		mx_hub->irq_masks_cache = mx_hub->irq_masks_cur;
		mx_hub_writebyte(mx_hub->client, MX_HUB_REG_IRQ_MASK,mx_hub->irq_masks_cur);
	}

	mutex_unlock(&mx_hub->irqlock);
}

static void mx_hub_irq_unmask(struct irq_data *data)
{
	struct mx_hub_dev *mx_hub = irq_data_get_irq_chip_data(data);
	struct mx_hub_irq_data *irq_data = irq_to_mx_hub_irq(mx_hub,
							       data->irq);

	mx_hub->irq_masks_cur &= ~irq_data->mask;
}

static void mx_hub_irq_mask(struct irq_data *data)
{
	struct mx_hub_dev *mx_hub = irq_data_get_irq_chip_data(data);
	struct mx_hub_irq_data *irq_data = irq_to_mx_hub_irq(mx_hub,
							       data->irq);

	mx_hub->irq_masks_cur |= irq_data->mask;
}

static struct irq_chip mx_hub_irq_chip = {
	.name = "mx_hub",
	.irq_bus_lock = mx_hub_irq_lock,
	.irq_bus_sync_unlock = mx_hub_irq_sync_unlock,
	.irq_mask = mx_hub_irq_mask,
	.irq_unmask = mx_hub_irq_unmask,
};

static irqreturn_t mx_hub_irq_thread(int irq, void *data)
{
	struct mx_hub_dev *mx_hub = data;
	u8 irq_reg;
	int ret;
	int i;

	ret = mx_hub_readdata(mx_hub->client, MX_HUB_REG_IRQ,
			1, &irq_reg);

	if(mx_hub->debug)
		dev_info(mx_hub->dev, "irq_reg: 0x%X\n",irq_reg);
	
	if (ret < 0) {
		dev_err(mx_hub->dev, "Failed to read interrupt register: %d\n",
				ret);
		return IRQ_NONE;
	}

	if(irq_reg == 0xFF)
		return IRQ_HANDLED;

	/* Apply masking */
	irq_reg &= ~mx_hub->irq_masks_cur;

	/* Report */
	for (i = 0; i < MX_HUB_IRQ_MAX; i++) {
		if (irq_reg & mx_hub_irqs[i].mask)
			handle_nested_irq(mx_hub->irq_base + i);
	}

	return IRQ_HANDLED;
}

int mx_hub_irq_resume(struct mx_hub_dev *mx_hub)
{
	if (mx_hub->irq && mx_hub->irq_base)
		mx_hub_irq_thread(mx_hub->irq_base, mx_hub);
	return 0;
}

int mx_hub_irq_init(struct mx_hub_dev *mx_hub)
{
	int i;
	int cur_irq;
	int ret;

	if (!mx_hub->irq) {
		dev_warn(mx_hub->dev,
			 "No interrupt specified, no interrupts\n");
		mx_hub->irq_base = 0;
		return 0;
	}

	if (!mx_hub->irq_base) {
		dev_err(mx_hub->dev,
			"No interrupt base specified, no interrupts\n");
		return 0;
	}

	mutex_init(&mx_hub->irqlock);

	/* Mask the individual interrupt sources */
	mx_hub->irq_masks_cur = 0xff;
	mx_hub->irq_masks_cache = 0xff;
	mx_hub_writebyte(mx_hub->client, MX_HUB_REG_IRQ, 0xff);


	/* register with genirq */
	for (i = 0; i < MX_HUB_IRQ_MAX; i++) {
		cur_irq = i + mx_hub->irq_base;
		irq_set_chip_data(cur_irq, mx_hub);
		irq_set_chip_and_handler(cur_irq, &mx_hub_irq_chip,
					 handle_edge_irq);
		irq_set_nested_thread(cur_irq, 1);		
		set_irq_flags(cur_irq, IRQF_VALID);
	}

	ret = request_threaded_irq(mx_hub->irq, NULL, mx_hub_irq_thread,
				   IRQF_TRIGGER_FALLING| IRQF_ONESHOT,
				   "mx-hub-irq", mx_hub);
	if (ret) {
		dev_err(mx_hub->dev, "Failed to request IRQ %d: %d\n",
			mx_hub->irq, ret);
		return ret;
	}

	return 0;
}

void mx_hub_irq_exit(struct mx_hub_dev *mx_hub)
{
	if (mx_hub->irq)
		free_irq(mx_hub->irq, mx_hub);
}
