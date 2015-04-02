/*
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/platform_data/dwc3-exynos.h>
#include <linux/regulator/consumer.h>
#include <linux/bootmode.h>

#include <plat/ehci.h>
#include <plat/devs.h>
#include <plat/usb-phy.h>
#include <plat/gpio-cfg.h>

#include <mach/ohci.h>
#include <mach/usb3-drd.h>
#include <mach/usb-switch.h>
#include <mach/gpio-meizu.h>

static struct regulator *reverse = NULL;

static int is_host_insert(void)
{
	int host = 0;

	if(reverse && regulator_is_enabled(reverse))
		host = 1;

	return host;
}

static int m6x_get_id_state(struct platform_device *pdev)
{
	if(is_host_insert()) {
		return 0;
	}
	return 1;
}

static bool m6x_get_bsession_valid(struct platform_device *pdev)
{
	int vbus = 0;

	if(is_host_insert()) {
		vbus = 0;
	} else {
		vbus = !!gpio_get_value(MEIZU_USB_IRQ);
	}

	return vbus;
}

#ifdef CONFIG_USB_OHCI_HCD
static struct exynos4_ohci_platdata m6x_ohci_pdata __initdata;
#endif
#ifdef CONFIG_USB_EHCI_HCD
static struct s5p_ehci_platdata m6x_ehci_pdata __initdata;
#endif
static struct dwc3_exynos_data m6x_drd_pdata __initdata = {
	.udc_name		= "exynos-ss-udc",
	.xhci_name		= "exynos-xhci",
	.phy_type		= S5P_USB_PHY_DRD,
	.phy_init		= s5p_usb_phy_init,
	.phy_exit		= s5p_usb_phy_exit,
	.phy_crport_ctrl	= exynos5_usb_phy_crport_ctrl,
	.get_id_state		= m6x_get_id_state,
	.get_bses_vld		= m6x_get_bsession_valid,
	.irq_flags		= IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_SHARED,
};


static void __init m6x_ohci_init(void)
{
#ifdef CONFIG_USB_OHCI_HCD
	exynos4_ohci_set_platdata(&m6x_ohci_pdata);
#endif
}


static void __init m6x_ehci_init(void)
{
#ifdef CONFIG_USB_EHCI_HCD
	s5p_ehci_set_platdata(&m6x_ehci_pdata);
#endif
}

static void __init m6x_drd_phy_shutdown(struct platform_device *pdev)
{
	int phy_num = pdev->id;
	struct clk *clk;

	switch (phy_num) {
	case 0:
		clk = clk_get_sys("exynos-dwc3.0", "usbdrd30");
		break;
	case 1:
		clk = clk_get_sys("exynos-dwc3.1", "usbdrd30");
		break;
	default:
		clk = NULL;
		break;
	}

	if (IS_ERR_OR_NULL(clk)) {
		printk(KERN_ERR "failed to get DRD%d phy clock\n", phy_num);
		return;
	}

	if (clk_enable(clk)) {
		printk(KERN_ERR "failed to enable DRD%d clock\n", phy_num);
		return;
	}

	s5p_usb_phy_exit(pdev, S5P_USB_PHY_DRD);

	clk_disable(clk);
}

static void __init __maybe_unused m6x_drd0_init(void)
{
	/* Initialize DRD0 gpio */
	if (gpio_request(EXYNOS5410_GPK3(0), "UDRD3_0_OVERCUR_U2")) {
		pr_err("failed to request UDRD3_0_OVERCUR_U2\n");
	} else {
		s3c_gpio_cfgpin(EXYNOS5410_GPK3(0), (0x2 << 0));
		s3c_gpio_setpull(EXYNOS5410_GPK3(0), S3C_GPIO_PULL_NONE);
		gpio_free(EXYNOS5410_GPK3(0));
	}
	if (gpio_request(EXYNOS5410_GPK3(1), "UDRD3_0_OVERCUR_U3")) {
		pr_err("failed to request UDRD3_0_OVERCUR_U3\n");
	} else {
		s3c_gpio_cfgpin(EXYNOS5410_GPK3(1), (0x2 << 4));
		s3c_gpio_setpull(EXYNOS5410_GPK3(1), S3C_GPIO_PULL_NONE);
		gpio_free(EXYNOS5410_GPK3(1));
	}

	if (gpio_request_one(EXYNOS5410_GPK3(2), GPIOF_OUT_INIT_LOW,
						"UDRD3_0_VBUSCTRL_U2")) {
		pr_err("failed to request UDRD3_0_VBUSCTRL_U2\n");
	} else {
		s3c_gpio_setpull(EXYNOS5410_GPK3(2), S3C_GPIO_PULL_NONE);
		gpio_free(EXYNOS5410_GPK3(2));
	}

	if (gpio_request_one(EXYNOS5410_GPK3(3), GPIOF_OUT_INIT_LOW,
						 "UDRD3_0_VBUSCTRL_U3")) {
		pr_err("failed to request UDRD3_0_VBUSCTRL_U3\n");
	} else {
		s3c_gpio_setpull(EXYNOS5410_GPK3(3), S3C_GPIO_PULL_NONE);
		gpio_free(EXYNOS5410_GPK3(3));
	}

	m6x_drd_pdata.id_irq = gpio_to_irq(MEIZU_USB_IRQ);
	m6x_drd_pdata.vbus_irq = gpio_to_irq(MEIZU_USB_IRQ);

	exynos5_usb3_drd0_set_platdata(&m6x_drd_pdata);
}

static void __init __maybe_unused m6x_drd1_init(void)
{
	/* Initialize DRD1 gpio */
	if (gpio_request(EXYNOS5410_GPK2(4), "UDRD3_1_OVERCUR_U2")) {
		pr_err("failed to request UDRD3_1_OVERCUR_U2\n");
	} else {
		s3c_gpio_cfgpin(EXYNOS5410_GPK2(4), (0x2 << 16));
		s3c_gpio_setpull(EXYNOS5410_GPK2(4), S3C_GPIO_PULL_NONE);
		gpio_free(EXYNOS5410_GPK2(4));
	}

	if (gpio_request(EXYNOS5410_GPK2(5), "UDRD3_1_OVERCUR_U3")) {
		pr_err("failed to request UDRD3_1_OVERCUR_U3\n");
	} else {
		s3c_gpio_cfgpin(EXYNOS5410_GPK2(5), (0x2 << 20));
		s3c_gpio_setpull(EXYNOS5410_GPK2(5), S3C_GPIO_PULL_NONE);
		gpio_free(EXYNOS5410_GPK2(5));
	}

	if (gpio_request_one(EXYNOS5410_GPK2(6), GPIOF_OUT_INIT_LOW,
				"UDRD3_1_VBUSCTRL_U2")) {
		pr_err("failed to request UDRD3_1_VBUSCTRL_U2\n");
	} else {
		s3c_gpio_setpull(EXYNOS5410_GPK2(6), S3C_GPIO_PULL_NONE);
		gpio_free(EXYNOS5410_GPK2(6));
	}

	if (gpio_request_one(EXYNOS5410_GPK2(7), GPIOF_OUT_INIT_LOW,
				"UDRD3_1_VBUSCTRL_U3")) {
		pr_err("failed to request UDRD3_1_VBUSCTRL_U3\n");
	} else {
		s3c_gpio_setpull(EXYNOS5410_GPK2(7), S3C_GPIO_PULL_NONE);
		gpio_free(EXYNOS5410_GPK2(7));
	}

	m6x_drd_pdata.quirks = DUMMY_DRD;

	exynos5_usb3_drd1_set_platdata(&m6x_drd_pdata);
}

static struct platform_device *m6x_usb_devices[] __initdata = {
#ifdef CONFIG_USB_OHCI_HCD
	&exynos4_device_ohci,
#endif
#ifdef CONFIG_USB_EHCI_HCD
	&s5p_device_ehci,
#endif
	&exynos5_device_usb3_drd0,
	&exynos5_device_usb3_drd1,
};

static int __init exynos5_m6x_usb_init(void)
{
	reverse = regulator_get(NULL, "vbus_reverse");
	if (IS_ERR(reverse)) {
		reverse = NULL;
	}

	m6x_ohci_init();
	m6x_ehci_init();

	if (soc_is_exynos5410() && samsung_rev() == 0)
		m6x_drd_pdata.quirks |= EXYNOS_PHY20_NO_SUSPEND;

	m6x_drd_pdata.quirks |= FORCE_PM_PERIPHERAL;

	/*
	 * Shutdown DRD PHYs to reduce power consumption.
	 * Later, DRD driver will turn on only the PHY it needs.
	 */
	m6x_drd_phy_shutdown(&exynos5_device_usb3_drd0);
	m6x_drd_phy_shutdown(&exynos5_device_usb3_drd1);
	m6x_drd0_init();
	m6x_drd1_init();
	platform_add_devices(m6x_usb_devices,
			ARRAY_SIZE(m6x_usb_devices));

	return 0;
}

module_init(exynos5_m6x_usb_init);
