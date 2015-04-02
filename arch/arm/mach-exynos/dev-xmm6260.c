/**
 * arch/arm/mach-exynos/dev-xmm6260.c
 *
 * Copyright (C) 2010 Samsung Electronics.
 * Copyright (C) 2013 Zhuhai Meizu Inc.
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

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/platform_data/modem.h>
#include <linux/platform_device.h>
#include <plat/gpio-cfg.h>
#include <mach/modem.h>
#include <mach/gpio-meizu.h>

static struct modem_io_t m6x_io_devices[] = {
	[0] = {
		.id      = 0x0,
		.name    = "ttyACM0",
		.link    = LINKDEV_HSIC,
		.io_type = IODEV_TTY,
	},
	[1] = {
		.id      = 0x1,
		.name    = "rmnet0",
		.link    = LINKDEV_HSIC,
		.io_type = IODEV_NET,
	},
	[2] = {
		.id      = 0x2,
		.name    = "ttyACM2",
		.link    = LINKDEV_HSIC,
		.io_type = IODEV_TTY,
	},
	[3] = {
		.id      = 0x3,
		.name    = "ttyACM3",
		.link    = LINKDEV_HSIC,
		.io_type = IODEV_TTY,
	},
};

extern struct platform_device s5p_device_ehci;
#ifdef CONFIG_USB_EHCI_S5P
extern int s5p_wait_for_cp_resume(struct platform_device *pdev, int port);
extern void s5p_ehci_power(struct platform_device *pdev, int on);
#else
static inline int s5p_wait_for_cp_resume(struct platform_device *pdev, int port)
{
	return 0;
}
static inline void s5p_ehci_power(struct platform_device *pdev, int on) {}
#endif
static int xmm_wait_cp_resume(int port)
{
	return s5p_wait_for_cp_resume(&s5p_device_ehci, port);
}

static void xmm_set_ehci_power(int on)
{
	s5p_ehci_power(&s5p_device_ehci, on);
}

static struct modemlink_pm_data modem_link_pm_data = {
	.name = "link_pm",
	//.gpio_host_active = MEIZU_IPC_TRIGIN,
	//.gpio_hostwake  = MEIZU_IPC_HOST_WAKE_IRQ,
	//.gpio_slavewake = MEIZU_IPC_SLAVE_WAKEUP,
	.wait_cp_resume = xmm_wait_cp_resume,
};

struct modem_data xmm_modem_data = {
	.name                     = "xmm6260",
	//.gpio_cp_on               = MEIZU_MODEM_ON,
	//.gpio_reset_req_n         = MEIZU_PMU_RST,
	//.gpio_cp_reset            = MEIZU_BB_RST,
	//.gpio_cp_reset_int        = MEIZU_BB_RST_IRQ,
	.modem_type               = IMC_XMM6260,
	.link_type                = LINKDEV_HSIC,
	.modem_net                = UMTS_NETWORK,
	.num_iodevs               = ARRAY_SIZE(m6x_io_devices),
	.iodevs                   = m6x_io_devices,
	.ehci_power               = xmm_set_ehci_power,
	.link_pm_data             = &modem_link_pm_data,
};

void modem_set_active_state(int state)
{
	int val = gpio_get_value(xmm_modem_data.gpio_cp_reset);

	if (!val) {
		MIF_DEBUG("CP not ready, Active State low\n");
		return;
	}
	MIF_DEBUG("%s state=>:%d\n", __func__, state);
	gpio_direction_output(modem_link_pm_data.gpio_host_active, state);
}

void modem_set_slave_wakeup(int state)
{
	int val = gpio_get_value(xmm_modem_data.gpio_cp_reset);

	if (!val) {
		MIF_DEBUG("CP not ready, do not Slave wakeup\n");
		return;
	}

	if (!state)
		gpio_set_value(modem_link_pm_data.gpio_slavewake, 0);
	else {
		if (gpio_get_value(modem_link_pm_data.gpio_slavewake)) {
			MIF_INFO("warn.. slavewake toggle\n");
			gpio_set_value(modem_link_pm_data.gpio_slavewake, 0);
			msleep(20);
		}
		gpio_set_value(modem_link_pm_data.gpio_slavewake, 1);
	}
	MIF_DEBUG("%s state=>:%d\n", __func__, state);
}

static void umts_modem_cfg_gpio(void)
{
	int err = 0;
	unsigned gpio_cp_on        = xmm_modem_data.gpio_cp_on;
	unsigned gpio_cp_rst       = xmm_modem_data.gpio_cp_reset;
	unsigned gpio_reset_req_n  = xmm_modem_data.gpio_reset_req_n;
	unsigned gpio_cp_reset_int = xmm_modem_data.gpio_cp_reset_int;

	if (gpio_reset_req_n) {
		err = gpio_request(gpio_reset_req_n, "RESET_REQ_N");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "RESET_REQ_N", err);
		}
		gpio_direction_output(gpio_reset_req_n, 0);
		s3c_gpio_setpull(gpio_reset_req_n, S3C_GPIO_PULL_NONE);
	}

	if (gpio_cp_on) {
		err = gpio_request(gpio_cp_on, "CP_ON");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "CP_ON", err);
		}
		gpio_direction_output(gpio_cp_on, 0);
		s3c_gpio_setpull(gpio_cp_on, S3C_GPIO_PULL_NONE);
	}

	if (gpio_cp_rst) {
		err = gpio_request(gpio_cp_rst, "CP_RST");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "CP_RST", err);
		}
		gpio_direction_output(gpio_cp_rst, 0);
		s3c_gpio_setpull(gpio_cp_rst, S3C_GPIO_PULL_NONE);
	}

	if (gpio_cp_reset_int) {
		err = gpio_request(gpio_cp_reset_int, "GPIO_CP_RESET_INT");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "GPIO_CP_RESET_INT", err);
		}
		s3c_gpio_setpull(gpio_cp_reset_int, S3C_GPIO_PULL_UP);
	}

	printk(KERN_INFO "umts_modem_cfg_gpio done\n");
}

static void modem_hsic_pm_config_gpio(void)
{
	int err = 0;
	unsigned gpio_hostwake  = modem_link_pm_data.gpio_hostwake;
	unsigned gpio_slavewake = modem_link_pm_data.gpio_slavewake;
	unsigned gpio_host_active  = modem_link_pm_data.gpio_host_active;

	if (gpio_hostwake) {
		err = gpio_request(gpio_hostwake, "HOSTWAKE");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "HOSTWAKE", err);
		}
		gpio_direction_input(gpio_hostwake);
		s3c_gpio_cfgpin(gpio_hostwake, S3C_GPIO_SFN(0xF));
		s3c_gpio_setpull(gpio_hostwake, S3C_GPIO_PULL_NONE);
	}

	if (gpio_slavewake) {
		err = gpio_request(gpio_slavewake, "SLAVEWAKE");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "SLAVEWAKE", err);
		}
		gpio_direction_output(gpio_slavewake, 0);
		s3c_gpio_setpull(gpio_slavewake, S3C_GPIO_PULL_NONE);
	}

	if (gpio_host_active) {
		err = gpio_request(gpio_host_active, "HOST_ACTIVE");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "HOST_ACTIVE", err);
		}
		gpio_direction_output(gpio_host_active, 0);
	}

	if (gpio_hostwake)
		irq_set_irq_type(gpio_to_irq(gpio_hostwake),
							IRQ_TYPE_EDGE_BOTH);

	printk(KERN_INFO "modem_hsic_pm_config_gpio done\n");
}

static void __maybe_unused modem_power_reset(void)
{
	gpio_direction_output(xmm_modem_data.gpio_cp_reset, 1);
	mdelay(1);
	gpio_direction_output(xmm_modem_data.gpio_reset_req_n, 1);
	mdelay(2);
	gpio_direction_output(xmm_modem_data.gpio_cp_on, 1);
	udelay(80);
	gpio_direction_output(xmm_modem_data.gpio_cp_on, 0);
}


int __init modem_reset_init(void)
{
	pr_info("[MODEM_IF] init modem power\n");

	modem_link_pm_data.gpio_host_active = MEIZU_IPC_TRIGIN;
	modem_link_pm_data.gpio_hostwake  = MEIZU_IPC_HOST_WAKE_IRQ;
	modem_link_pm_data.gpio_slavewake = MEIZU_IPC_SLAVE_WAKEUP;

	xmm_modem_data.gpio_cp_on = MEIZU_MODEM_ON;
	xmm_modem_data.gpio_reset_req_n = MEIZU_PMU_RST;
	xmm_modem_data.gpio_cp_reset = MEIZU_BB_RST;
	xmm_modem_data.gpio_cp_reset_int = MEIZU_BB_RST_IRQ;
	umts_modem_cfg_gpio();
	modem_hsic_pm_config_gpio();
	modem_power_reset();
	return 0;
}
