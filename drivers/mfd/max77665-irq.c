/*
 * max77665-irq.c - Interrupt controller support for MAX77665
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
 * This driver is based on max77665-irq.c
 */

#include <linux/err.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mfd/max77665.h>
#include <linux/mfd/max77665-private.h>
#include <linux/regmap.h>

static const u8 max77665_mask_reg[] = {
	[CHG_INT] = MAX77665_CHG_REG_CHG_INT_MASK,
	[TOPSYS_INT] = MAX77665_PMIC_REG_TOPSYS_INT_MASK,
	[LED_INT] = MAX77665_LED_REG_FLASH_INT_MASK,
	[MUIC_INT1] = MAX77665_MUIC_REG_INTMASK1,
};

static struct regmap *max77665_get_regmap(struct max77665_dev *max77665, 
		enum max77665_irq_source src)
{
	switch (src) {
	case CHG_INT ... LED_INT:
		return max77665->regmap;
	case MUIC_INT1:
		return max77665->regmap_muic;
	default:
		return ERR_PTR(-EINVAL);
	}
}

struct max77665_irq_data {
	int mask;
	enum max77665_irq_source group;
};

#define DECLARE_IRQ(idx, _group, _mask)		\
	[(idx)] = { .group = (_group), .mask = (_mask) }
static const struct max77665_irq_data max77665_irqs[] = {
	DECLARE_IRQ(MAX77665_CHG_IRQ_BYP_I,		CHG_INT, 1 << 0),
	DECLARE_IRQ(MAX77665_CHG_IRQ_THM_I,		CHG_INT, 1 << 2),
	DECLARE_IRQ(MAX77665_CHG_IRQ_BAT_I,		CHG_INT, 1 << 3),
	DECLARE_IRQ(MAX77665_CHG_IRQ_CHG_I,		CHG_INT, 1 << 4),
	DECLARE_IRQ(MAX77665_CHG_IRQ_CHGIN_I,		CHG_INT, 1 << 6),

	DECLARE_IRQ(MAX77665_TOPSYS_IRQ_T120C_INT,	TOPSYS_INT, 1 << 0),
	DECLARE_IRQ(MAX77665_TOPSYS_IRQ_T140C_INT,	TOPSYS_INT, 1 << 1),
	DECLARE_IRQ(MAX77665_TOPSYS_IRQ_LOWSYS_INT,	TOPSYS_INT, 1 << 3),

	DECLARE_IRQ(MAX77665_LED_IRQ_FLED2_OPEN,	LED_INT, 1 << 0),
	DECLARE_IRQ(MAX77665_LED_IRQ_FLED2_SHORT,	LED_INT, 1 << 1),
	DECLARE_IRQ(MAX77665_LED_IRQ_FLED1_OPEN,	LED_INT, 1 << 2),
	DECLARE_IRQ(MAX77665_LED_IRQ_FLED1_SHORT,	LED_INT, 1 << 3),
	DECLARE_IRQ(MAX77665_LED_IRQ_MAX_FLASH,		LED_INT, 1 << 4),

	DECLARE_IRQ(MAX77665_MUIC_IRQ_INT1_ADC,	MUIC_INT1, 1 << 0),
	DECLARE_IRQ(MAX77665_MUIC_IRQ_INT1_ADCLOW,	MUIC_INT1, 1 << 1),
	DECLARE_IRQ(MAX77665_MUIC_IRQ_INT1_ADCERR,	MUIC_INT1, 1 << 2),
	DECLARE_IRQ(MAX77665_MUIC_IRQ_INT1_ADC1K,	MUIC_INT1, 1 << 3),
};

static void max77665_irq_lock(struct irq_data *data)
{
	struct max77665_dev *max77665 = irq_get_chip_data(data->irq);

	mutex_lock(&max77665->irqlock);
}

static void max77665_irq_sync_unlock(struct irq_data *data)
{
	struct max77665_dev *max77665 = irq_get_chip_data(data->irq);
	int i;

	for (i = 0; i < MAX77665_IRQ_GROUP_NR; i++) {
		u8 mask_reg = max77665_mask_reg[i];
		struct regmap *regmap = max77665_get_regmap(max77665, i);

		if (mask_reg == MAX77665_REG_INVALID ||
				IS_ERR_OR_NULL(regmap))
			continue;
		max77665->irq_masks_cache[i] = max77665->irq_masks_cur[i];

		if (max77665->irq_masks_cur[i] != 0xff) {
			unsigned int reg_data;
			regmap_read(regmap, MAX77665_PMIC_REG_INTSRC_MASK, &reg_data);
			reg_data &= ~(1<<i);
			regmap_write(regmap, MAX77665_PMIC_REG_INTSRC_MASK,reg_data);
		} else {
			unsigned int reg_data;
			regmap_read(regmap, MAX77665_PMIC_REG_INTSRC_MASK, &reg_data);
			reg_data |= (1<<i);
			regmap_write(regmap, MAX77665_PMIC_REG_INTSRC_MASK,reg_data);
		}
			
		regmap_write(regmap, max77665_mask_reg[i],
				max77665->irq_masks_cur[i]);
	}

	mutex_unlock(&max77665->irqlock);
}

static const inline struct max77665_irq_data *
irq_to_max77665_irq(struct max77665_dev *max77665, int irq)
{
	return &max77665_irqs[irq - max77665->irq_base];
}

static void max77665_irq_mask(struct irq_data *data)
{
	struct max77665_dev *max77665 = irq_get_chip_data(data->irq);
	const struct max77665_irq_data *irq_data =
				irq_to_max77665_irq(max77665, data->irq);

	if(irq_data->group == MUIC_INT1)
		max77665->irq_masks_cur[irq_data->group] &= ~irq_data->mask;
	else
		max77665->irq_masks_cur[irq_data->group] |= irq_data->mask;
}

static void max77665_irq_unmask(struct irq_data *data)
{
	struct max77665_dev *max77665 = irq_get_chip_data(data->irq);
	const struct max77665_irq_data *irq_data =
	    irq_to_max77665_irq(max77665, data->irq);

	if(irq_data->group == MUIC_INT1)
		max77665->irq_masks_cur[irq_data->group] |= irq_data->mask;
	else
		max77665->irq_masks_cur[irq_data->group] &= ~irq_data->mask;
}

static struct irq_chip max77665_irq_chip = {
	.name			= "max77665",
	.irq_bus_lock		= max77665_irq_lock,
	.irq_bus_sync_unlock= max77665_irq_sync_unlock,
	.irq_mask			= max77665_irq_mask,
	.irq_unmask		= max77665_irq_unmask,
};

static irqreturn_t max77665_irq_thread(int irq, void *data)
{
	struct max77665_dev *max77665 = data;
	unsigned int irq_reg[MAX77665_IRQ_GROUP_NR] = {0};
	unsigned int irq_src;
	int ret;
	int i;

	ret = regmap_read(max77665->regmap, MAX77665_PMIC_REG_INTSRC,
				&irq_src);
	if (ret < 0) {
		dev_err(max77665->dev, "Failed to read interrupt source: %d\n",
				ret);
		return IRQ_NONE;
	}

	if (irq_src & MAX77665_IRQSRC_CHG)
		/* CHG_INT */
		ret = regmap_read(max77665->regmap, MAX77665_CHG_REG_CHG_INT,
				&irq_reg[CHG_INT]);

	if (irq_src & MAX77665_IRQSRC_TOP)
		/* TOPSYS_INT */
		ret = regmap_read(max77665->regmap,
			MAX77665_PMIC_REG_TOPSYS_INT, &irq_reg[TOPSYS_INT]);

	if (irq_src & MAX77665_IRQSRC_FLASH)
		/* LED_INT */
		ret = regmap_read(max77665->regmap,
			MAX77665_LED_REG_FLASH_INT, &irq_reg[LED_INT]);

	if (irq_src & MAX77665_IRQSRC_MUIC) {
		/* MUIC INT1 */
		ret = regmap_read(max77665->regmap_muic, 
			MAX77665_MUIC_REG_INT1, &irq_reg[MUIC_INT1]);
	}

	/* Apply masking */
	for (i = 0; i < MAX77665_IRQ_GROUP_NR; i++) {
		if(i == MUIC_INT1)
			irq_reg[i] &= max77665->irq_masks_cur[i];
		else
			irq_reg[i] &= ~max77665->irq_masks_cur[i];
	}

	/* Report */
	for (i = 0; i < MAX77665_IRQ_NR; i++) {
		if (irq_reg[max77665_irqs[i].group] & max77665_irqs[i].mask)
			handle_nested_irq(max77665->irq_base + i);
	}

	return IRQ_HANDLED;
}

int max77665_irq_resume(struct max77665_dev *max77665)
{
	int ret = 0;
	if (max77665->irq && max77665->irq_base)
		ret = max77665_irq_thread(max77665->irq_base, max77665);

	return ret >= 0 ? 0 : ret;
}

int max77665_irq_init(struct max77665_dev *max77665)
{
	int i;
	int cur_irq;
	int ret;
	unsigned int intsrc_mask;

	if (!max77665->irq) {
		dev_warn(max77665->dev, "No interrupt specified.\n");
		max77665->irq_base = 0;
		return 0;
	}

	if (!max77665->irq_base) {
		dev_err(max77665->dev, "No interrupt base specified.\n");
		return 0;
	}

	mutex_init(&max77665->irqlock);

	/* Mask individual interrupt sources */
	for (i = 0; i < MAX77665_IRQ_GROUP_NR; i++) {
		struct regmap *regmap;;

		if(i == MUIC_INT1) {
			max77665->irq_masks_cur[i] = 0x00;
			max77665->irq_masks_cache[i] = 0x00;
		} else {
			max77665->irq_masks_cur[i] = 0xff;
			max77665->irq_masks_cache[i] = 0xff;
		}
		regmap = max77665_get_regmap(max77665, i);

		if (IS_ERR_OR_NULL(regmap))
			continue;
		if (max77665_mask_reg[i] == MAX77665_REG_INVALID)
			continue;
	
		if(i == MUIC_INT1) {
			regmap_write(regmap, max77665_mask_reg[i], 0x00);
		} else {
			regmap_write(regmap, max77665_mask_reg[i], 0xff);
		}	
	}

	/* Register with genirq */
	for (i = 0; i < MAX77665_IRQ_NR; i++) {
		cur_irq = i + max77665->irq_base;
		irq_set_chip_data(cur_irq, max77665);
		irq_set_chip_and_handler(cur_irq, &max77665_irq_chip,
					 handle_edge_irq);
		irq_set_nested_thread(cur_irq, 1);
#ifdef CONFIG_ARM
		set_irq_flags(cur_irq, IRQF_VALID);
#else
		irq_set_noprobe(cur_irq);
#endif
	}

	/* Unmask max77665 interrupt */
	ret = regmap_read(max77665->regmap, MAX77665_PMIC_REG_INTSRC_MASK,
			  &intsrc_mask);
	if (ret) {
		dev_err(max77665->dev, "%s: fail to read muic reg\n", __func__);
		return ret;
	}

	intsrc_mask &= ~(MAX77665_IRQSRC_CHG);	/* Unmask charger interrupt */
	intsrc_mask &= ~(MAX77665_IRQSRC_MUIC);	/* Unmask muic interrupt */
	regmap_write(max77665->regmap, MAX77665_PMIC_REG_INTSRC_MASK,
			   intsrc_mask);

	ret = request_threaded_irq(max77665->irq, NULL, max77665_irq_thread,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   "max77665-irq", max77665);

	if (ret) {
		dev_err(max77665->dev, "Failed to request IRQ %d: %d\n",
			max77665->irq, ret);
		return ret;
	}

	return 0;
}

void max77665_irq_exit(struct max77665_dev *max77665)
{
	if (max77665->irq)
		free_irq(max77665->irq, max77665);
}

MODULE_DESCRIPTION("MAXIM 77665 multi-function irq driver");
MODULE_AUTHOR("Sukdong Kim <Sukdong.Kim@samsung.com>");
MODULE_LICENSE("GPLV2");
