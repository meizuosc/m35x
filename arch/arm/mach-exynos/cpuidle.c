/* linux/arch/arm/mach-exynos/cpuidle.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cpuidle.h>
#include <linux/cpu_pm.h>
#include <linux/io.h>
#include <linux/export.h>
#include <linux/time.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/suspend.h>
#include <linux/leds.h>
#include <linux/cpu.h>
#include <linux/fb.h>
#include <linux/pm_qos.h>
#include <linux/reboot.h>

#include <asm/proc-fns.h>
#include <asm/smp_scu.h>
#include <asm/suspend.h>
#include <asm/unified.h>
#include <asm/cputype.h>
#include <asm/cacheflush.h>
#include <asm/system_misc.h>
#include <asm/tlbflush.h>
#include <asm/bL_switcher.h>

#include <mach/regs-pmu.h>
#include <mach/regs-clock.h>
#include <mach/pmu.h>
#include <mach/smc.h>
#include <mach/asv.h>
#include <mach/gpio-meizu.h>

#include <plat/pm.h>
#include <plat/cpu.h>
#include <linux/gpio.h>

#include <plat/devs.h>
#include <plat/regs-serial.h>
#include <plat/gpio-cfg.h>
#include <plat/gpio-core.h>
#include <plat/usb-phy.h>
#include <plat/regs-watchdog.h>

/* Check wakeup causes */
#define LPA_CHECK_WAKEUP_CAUSES
#define AFTR_CHECK_WAKEUP_CAUSES

#ifdef CONFIG_WAKEUP_REASON
#include <linux/cpufreq.h>
static int idle_wakeup_debug;
static char idle_wakeup_debug_str[3][5]= {"none", "lpa", "aftr"};
extern void exynos_show_wakeup_reason(void);
#else
#define idle_wakeup_debug	(0)
#define exynos_show_wakeup_reason()	do { } while (0)
#endif

#ifdef LPA_CHECK_WAKEUP_CAUSES
#define lpa_exynos_show_wakeup_reason() do { if (idle_wakeup_debug == 1) exynos_show_wakeup_reason(); } while (0)
#define idle_pr_debug(fmt, ...) do { if (idle_wakeup_debug == 1) pr_info(fmt, ##__VA_ARGS__); } while (0)
#else
#define lpa_exynos_show_wakeup_reason() do { } while (0)
#define idle_pr_debug(fmt, ...) do { } while (0)
#endif

#ifdef AFTR_CHECK_WAKEUP_CAUSES
#define aftr_exynos_show_wakeup_reason() do { if (idle_wakeup_debug == 2) exynos_show_wakeup_reason(); } while (0)
#else
#define aftr_exynos_show_wakeup_reason() do { } while (0)
#endif

/* Only configure some of the gpio registers: MMC, Audio, Wifi, Modem. */
#define SIMPLIFY_GPIO_POWER_DOWN_CONFIG

/* LPA works with the enabling of RETENTION_HSI_SYS_PWR */
#undef LPA_IS_BROKEN

/* Allow enter into C2 state */
#define EXYNOS5410_ENTER_LOWPOWER

#ifdef CONFIG_ARM_TRUSTZONE
#define REG_DIRECTGO_ADDR	(soc_is_exynos4210() ? samsung_rev() == EXYNOS4210_REV_1_1 ? \
			EXYNOS_INFORM7 : (samsung_rev() == EXYNOS4210_REV_1_0 ? \
			(S5P_VA_SYSRAM_NS + 0x24) : EXYNOS_INFORM0) : (S5P_VA_SYSRAM_NS + 0x24))
#define REG_DIRECTGO_FLAG	(soc_is_exynos4210() ? samsung_rev() == EXYNOS4210_REV_1_1 ? \
			EXYNOS_INFORM6 : (samsung_rev() == EXYNOS4210_REV_1_0 ? \
			(S5P_VA_SYSRAM_NS + 0x20) : EXYNOS_INFORM1) : (S5P_VA_SYSRAM_NS + 0x20))
#else
#define REG_DIRECTGO_ADDR	(samsung_rev() == EXYNOS4210_REV_1_1 ? \
			EXYNOS_INFORM7 : (samsung_rev() == EXYNOS4210_REV_1_0 ? \
			(S5P_VA_SYSRAM + 0x24) : EXYNOS_INFORM0))
#define REG_DIRECTGO_FLAG	(samsung_rev() == EXYNOS4210_REV_1_1 ? \
			EXYNOS_INFORM6 : (samsung_rev() == EXYNOS4210_REV_1_0 ? \
			(S5P_VA_SYSRAM + 0x20) : EXYNOS_INFORM1))
#endif

#define EXYNOS_GPIO_END		(soc_is_exynos5410() ? \
					EXYNOS5410_GPIO_END : EXYNOS4_GPIO_END)
#define EXYNOS_CHECK_DIRECTGO		0xFCBA0D10
static struct cpumask out_cpus;

extern unsigned int scu_save[2];

static int exynos_enter_idle(struct cpuidle_device *dev,
			struct cpuidle_driver *drv,
			      int index);
static int exynos_enter_lowpower(struct cpuidle_device *dev,
				struct cpuidle_driver *drv,
				int index);
static int exynos5410_enter_lowpower(struct cpuidle_device *dev,
				 struct cpuidle_driver *drv,
				 int index);

static int __exynos5410_enter_lowpower(struct cpuidle_device *dev,
				struct cpuidle_driver *drv,
				int index);

struct check_reg_lpa {
	void __iomem	*check_reg;
	unsigned int	check_bit;
};

static inline void misc_before_idle(void)
{
	/* Disable watchdog */
	__watchdog_save();
}

static inline void misc_after_idle(void)
{
	__watchdog_restore();
}


/*
 * List of check power domain list for LPA mode
 * These register are have to power off to enter LPA mode
 */
static struct check_reg_lpa exynos5_power_domain[] = {
	{.check_reg = EXYNOS5_GSCL_STATUS,	.check_bit = 0x7},
	{.check_reg = EXYNOS5_ISP_STATUS,	.check_bit = 0x7},
	{.check_reg = EXYNOS5410_G3D_STATUS,	.check_bit = 0x7},
	{.check_reg = EXYNOS5410_MFC_STATUS,	.check_bit = 0x7},
};

/*
 * List of check clock gating list for LPA mode
 * If clock of list is not gated, system can not enter LPA mode.
 */
static struct check_reg_lpa exynos5_clock_gating[] = {
	{.check_reg = EXYNOS5_CLKGATE_IP_DISP1,		.check_bit = 0x00000008},
	{.check_reg = EXYNOS5_CLKGATE_IP_MFC,		.check_bit = 0x00000001},
	{.check_reg = EXYNOS5_CLKGATE_IP_GEN,		.check_bit = 0x00004016},
	{.check_reg = EXYNOS5_CLKGATE_BUS_FSYS0,	.check_bit = 0x00000002},
	{.check_reg = EXYNOS5_CLKGATE_IP_PERIC,		.check_bit = 0x003F7FC0},
};

#if defined(CONFIG_EXYNOS_DEV_DWMCI)
enum hc_type {
	HC_SDHC,
	HC_MSHC,
};

struct check_device_op {
	void __iomem		*base;
	struct platform_device	*pdev;
	enum hc_type		type;
};

#if defined(CONFIG_ARCH_EXYNOS4)
static struct check_device_op chk_sdhc_op[] = {
#if defined(CONFIG_MMC_DW)
	{.base = 0, .pdev = &exynos4_device_dwmci, .type = HC_MSHC},
#endif
#if defined(CONFIG_S5P_DEV_MSHC)
	{.base = 0, .pdev = &s3c_device_mshci, .type = HC_MSHC},
#endif
#if defined(CONFIG_S3C_DEV_HSMMC)
	{.base = 0, .pdev = &s3c_device_hsmmc0, .type = HC_SDHC},
#endif
#if defined(CONFIG_S3C_DEV_HSMMC1)
	{.base = 0, .pdev = &s3c_device_hsmmc1, .type = HC_SDHC},
#endif
#if defined(CONFIG_S3C_DEV_HSMMC2)
	{.base = 0, .pdev = &s3c_device_hsmmc2, .type = HC_SDHC},
#endif
#if defined(CONFIG_S3C_DEV_HSMMC3)
	{.base = 0, .pdev = &s3c_device_hsmmc3, .type = HC_SDHC},
#endif
};
#else
static struct check_device_op chk_sdhc_op[] = {
	{.base = 0, .pdev = &exynos5_device_dwmci0, .type = HC_MSHC},
	{.base = 0, .pdev = &exynos5_device_dwmci1, .type = HC_MSHC},
};
#endif

#define S3C_HSMMC_PRNSTS	(0x24)
#define S3C_HSMMC_CLKCON	(0x2c)
#define S3C_HSMMC_CMD_INHIBIT	0x00000001
#define S3C_HSMMC_DATA_INHIBIT	0x00000002
#define S3C_HSMMC_CLOCK_CARD_EN	0x0004

#define MSHCI_CLKENA	(0x10)  /* Clock enable */
#define MSHCI_STATUS	(0x48)  /* Status */
#define MSHCI_DATA_BUSY	(0x1<<9)
#define MSHCI_DATA_STAT_BUSY	(0x1<<10)
#define MSHCI_CMD_BUSY	(0xf<<4)
#define MSHCI_ENCLK	(0x1)

static int sdmmc_dev_num;
/* If SD/MMC interface is working: return = 1 or not 0 */
static int check_sdmmc_op(unsigned int ch)
{
	unsigned int reg1;
	void __iomem *base_addr;
	if (soc_is_exynos5410()) {
		if (unlikely(ch >= sdmmc_dev_num)) {
			pr_err("Invalid ch[%d] for SD/MMC\n", ch);
			return 0;
		}

		if (chk_sdhc_op[ch].type == HC_MSHC) {
			base_addr = chk_sdhc_op[ch].base;
			/* Check STATUS [9] for data busy */
			reg1 = readl(base_addr + MSHCI_STATUS);
			return (reg1 & (MSHCI_DATA_BUSY)) ||
				(reg1 & (MSHCI_DATA_STAT_BUSY)) || (reg1 & (MSHCI_CMD_BUSY));
		}
	} else {
		unsigned int reg2;
		if (unlikely(ch >= sdmmc_dev_num)) {
			printk(KERN_ERR "Invalid ch[%d] for SD/MMC\n", ch);
			return 0;
		}

		if (chk_sdhc_op[ch].type == HC_SDHC) {
			base_addr = chk_sdhc_op[ch].base;
			/* Check CLKCON [2]: ENSDCLK */
			reg2 = readl(base_addr + S3C_HSMMC_CLKCON);
			return !!(reg2 & (S3C_HSMMC_CLOCK_CARD_EN));
		} else if (chk_sdhc_op[ch].type == HC_MSHC) {
			base_addr = chk_sdhc_op[ch].base;
			/* Check STATUS [9] for data busy */
			reg1 = readl(base_addr + MSHCI_STATUS);
			return (reg1 & (MSHCI_DATA_BUSY)) ||
				(reg1 & (MSHCI_DATA_STAT_BUSY)) || (reg1 & (MSHCI_CMD_BUSY));
		}
	}
	/* should not be here */
	return 0;
}

/* Check all sdmmc controller */
static int loop_sdmmc_check(void)
{
	unsigned int iter;

	for (iter = 0; iter < sdmmc_dev_num; iter++) {
		if (check_sdmmc_op(iter)) {
			idle_pr_debug("SDMMC [%d] working\n", iter);
			return 1;
		}
	}
	return 0;
}
#endif

static int exynos_check_reg_status(struct check_reg_lpa *reg_list,
				    unsigned int list_cnt)
{
	unsigned int i;
	unsigned int tmp;

	for (i = 0; i < list_cnt; i++) {
		tmp = __raw_readl(reg_list[i].check_reg);
		if (tmp & reg_list[i].check_bit) {
			idle_pr_debug("%s: Check clock gate reg %d busy\n", __func__, i);
			return -EBUSY;
		}
	}

	return 0;
}

static int exynos_uart_fifo_check(void)
{
	unsigned int ret;
	unsigned int check_val;

	ret = 0;

	/* Check UART for console is empty */
	check_val = __raw_readl(S5P_VA_UART(CONFIG_S3C_LOWLEVEL_UART_PORT) +
				0x18);

	ret = ((check_val >> 16) & 0xff);

	return ret;
}

extern int bt_uart_rts_ctrl(int);
extern int check_bt_running(void);
static int check_gps_op(void)
{
	// gps_power pin is high when gps is working
	int gps_is_running = gpio_get_value(MEIZU_GPS_PWR_ON);
	return gps_is_running;
}

#ifdef CONFIG_USB_EHCI_HCD
extern int s5p_ehci_check_op(struct platform_device *pdev);
#else
static inline int s5p_ehci_check_op(struct platform_device *pdev) {return 0;}
#endif

static int check_power_domain(void)
{
	unsigned long tmp;

	tmp = __raw_readl(EXYNOS4_LCD0_CONFIGURATION);
	if ((tmp & EXYNOS_INT_LOCAL_PWR_EN) == EXYNOS_INT_LOCAL_PWR_EN)
		return 1;

	/*
	 * from REV 1.1, MFC power domain can turn off
	 */
	if (((soc_is_exynos4412()) && (samsung_rev() >= EXYNOS4412_REV_1_1)) ||
	    ((soc_is_exynos4212()) && (samsung_rev() >= EXYNOS4212_REV_1_0)) ||
	     soc_is_exynos4210()) {
		tmp = __raw_readl(EXYNOS4_MFC_CONFIGURATION);
		if ((tmp & EXYNOS_INT_LOCAL_PWR_EN) == EXYNOS_INT_LOCAL_PWR_EN)
			return 1;
	}

	tmp = __raw_readl(EXYNOS4_MFC_CONFIGURATION);
	if ((tmp & EXYNOS_INT_LOCAL_PWR_EN) == EXYNOS_INT_LOCAL_PWR_EN)
		return 1;

	tmp = __raw_readl(EXYNOS4_CAM_CONFIGURATION);
	if ((tmp & EXYNOS_INT_LOCAL_PWR_EN) == EXYNOS_INT_LOCAL_PWR_EN)
		return 1;

	tmp = __raw_readl(EXYNOS4_TV_CONFIGURATION);
	if ((tmp & EXYNOS_INT_LOCAL_PWR_EN) == EXYNOS_INT_LOCAL_PWR_EN)
		return 1;

	tmp = __raw_readl(EXYNOS4_GPS_CONFIGURATION);
	if ((tmp & EXYNOS_INT_LOCAL_PWR_EN) == EXYNOS_INT_LOCAL_PWR_EN)
		return 1;

	return 0;
}

static int exynos4_check_operation(void)
{
	if (check_power_domain())
		return 1;

	return 0;
}

static int __maybe_unused exynos_check_enter_mode(void)
{
	if (soc_is_exynos5410()) {
		/* Check power domain */
		if (exynos_check_reg_status(exynos5_power_domain,
				    ARRAY_SIZE(exynos5_power_domain))) {
			idle_pr_debug("%s: Check power domain\n", __func__);
			return EXYNOS_CHECK_DIDLE;
		}

		/* Check clock gating */
		if (exynos_check_reg_status(exynos5_clock_gating,
				    ARRAY_SIZE(exynos5_clock_gating)))
			return EXYNOS_CHECK_DIDLE;
	} else {
		if (exynos4_check_operation())
			return EXYNOS_CHECK_DIDLE;
	}

#if defined(CONFIG_EXYNOS_DEV_DWMCI)
	if (loop_sdmmc_check()) {
		idle_pr_debug("%s: Check sdmmc\n", __func__);
		return EXYNOS_CHECK_DIDLE;
	}
#endif
	if (check_gps_op()) {
		idle_pr_debug("%s: Check gps op\n", __func__);
		return EXYNOS_CHECK_DIDLE;
	}

	if (check_bt_running()) {
		idle_pr_debug("%s: Check bt running\n", __func__);
		return EXYNOS_CHECK_DIDLE;
	}

	if (s5p_ehci_check_op(&s5p_device_ehci)) {
		idle_pr_debug("%s: Check usb op\n", __func__);
		return EXYNOS_CHECK_DIDLE;
	}

	return EXYNOS_CHECK_LPA;
}

static struct cpuidle_state exynos_cpuidle_set[] __initdata = {
	[0] = {
		.enter			= exynos_enter_idle,
		.exit_latency		= 1,
		.target_residency	= 10000,
		.flags			= CPUIDLE_FLAG_TIME_VALID,
		.name			= "C0",
		.desc			= "ARM clock gating(WFI)",
	},
	[1] = {
		.enter			= exynos_enter_lowpower,
		.exit_latency		= 300,
		.target_residency	= 10000,
		.flags			= CPUIDLE_FLAG_TIME_VALID,
		.name			= "C1",
		.desc			= "ARM power down",
	},
};

static struct cpuidle_state exynos5410_cpuidle_set[] __initdata = {
	[0] = {
		.enter			= exynos_enter_idle,
		.exit_latency		= 1,
		.target_residency	= 1000,
		.flags			= CPUIDLE_FLAG_TIME_VALID,
		.name			= "C1",
		.desc			= "ARM clock gating(WFI)",
	},
#ifdef EXYNOS5410_ENTER_LOWPOWER
	[1] = {
		.enter			= __exynos5410_enter_lowpower,
		.exit_latency		= 30,
		.target_residency	= 1000,
		.flags			= CPUIDLE_FLAG_TIME_VALID,
		.name			= "C2",
		.desc			= "ARM power down",
	},
	[2] = {
#else
	[1] = {
#endif
		.enter                  = exynos_enter_lowpower,
		.exit_latency           = 300,
		.target_residency       = 5000,
		.flags                  = CPUIDLE_FLAG_TIME_VALID,
		.name                   = "C3",
		.desc                   = "ARM power down",
	},
};

static DEFINE_PER_CPU(struct cpuidle_device, exynos_cpuidle_device);

static struct cpuidle_driver exynos_idle_driver = {
	.name		= "exynos_idle",
	.owner		= THIS_MODULE,
};

/* Ext-GIC nIRQ/nFIQ is the only wakeup source in AFTR */
static void exynos_set_wakeupmask(void)
{
	if (soc_is_exynos5410())
		__raw_writel(0x40003ffe, EXYNOS_WAKEUP_MASK);
	else
		__raw_writel(0x0000ff3e, EXYNOS_WAKEUP_MASK);
}

#ifdef CONFIG_LEDS_TRIGGERS

static struct led_trigger *idle_led_trigger[4];
DEFINE_LED_TRIGGER(exynos5410_idle1_led_trigger)
DEFINE_LED_TRIGGER(exynos5410_idle2_led_trigger)
DEFINE_LED_TRIGGER(exynos5410_idle3_led_trigger)
DEFINE_LED_TRIGGER(aftr_led_trigger)
DEFINE_LED_TRIGGER(lpa_led_trigger)

static int exynos_leds_idle_notifier(
	struct notifier_block *nb, unsigned long val, void *data)
{
	int cpuid = smp_processor_id();

	switch (val) {
	case IDLE_START:
		led_trigger_event(idle_led_trigger[cpuid], LED_OFF);
		break;
	case IDLE_END:
		led_trigger_event(idle_led_trigger[cpuid], LED_FULL);
		break;
	}

	return 0;
}

static struct notifier_block exynos_leds_idle_nb = {
	.notifier_call = exynos_leds_idle_notifier,
};

static void __init exynos_init_leds_triggers(void)
{
	static char trig_name[4][16];
	int i;

	for (i = 0; i < ARRAY_SIZE(trig_name); i++) {
		snprintf(trig_name[i], sizeof(trig_name[i]), "cpu%d-idle", i);
		led_trigger_register_simple(trig_name[i], &idle_led_trigger[i]);
	}
	idle_notifier_register(&exynos_leds_idle_nb);

	led_trigger_register_simple("cpuidle_aftr", &aftr_led_trigger);
	led_trigger_register_simple("cpuidle_lpa", &lpa_led_trigger);

	led_trigger_register_simple("exynos5410-idle1", &exynos5410_idle1_led_trigger);
	led_trigger_register_simple("exynos5410-idle2", &exynos5410_idle2_led_trigger);
	led_trigger_register_simple("exynos5410-idle3", &exynos5410_idle3_led_trigger);
}
#else
#define exynos_init_leds_triggers()	do { } while (0)
#endif


#if !defined(CONFIG_ARM_TRUSTZONE)
static unsigned int g_pwr_ctrl, g_diag_reg;

static void save_cpu_arch_register(void)
{
	/*read power control register*/
	asm("mrc p15, 0, %0, c15, c0, 0" : "=r"(g_pwr_ctrl) : : "cc");
	/*read diagnostic register*/
	asm("mrc p15, 0, %0, c15, c0, 1" : "=r"(g_diag_reg) : : "cc");
	return;
}

static void restore_cpu_arch_register(void)
{
	/*write power control register*/
	asm("mcr p15, 0, %0, c15, c0, 0" : : "r"(g_pwr_ctrl) : "cc");
	/*write diagnostic register*/
	asm("mcr p15, 0, %0, c15, c0, 1" : : "r"(g_diag_reg) : "cc");
	return;
}
#else
static void save_cpu_arch_register(void)
{
}

static void restore_cpu_arch_register(void)
{
}
#endif

static int idle_finisher(unsigned long flags)
{

#ifdef CONFIG_CACHE_L2X0
	outer_flush_all();
#endif

#if defined(CONFIG_ARM_TRUSTZONE)
	if (soc_is_exynos5410()) {
		exynos_smc(SMC_CMD_SAVE, OP_TYPE_CORE, SMC_POWERSTATE_IDLE, 0);
		exynos_smc(SMC_CMD_SHUTDOWN, OP_TYPE_CLUSTER, SMC_POWERSTATE_IDLE, 0);
	} else {
		exynos_smc(SMC_CMD_CPU0AFTR, 0, 0, 0);
	}
#else
	cpu_do_idle();
#endif
	return 1;
}

static int c2_finisher(unsigned long flags)
{
#if defined(CONFIG_ARM_TRUSTZONE)
	exynos_smc(SMC_CMD_SAVE, OP_TYPE_CORE, SMC_POWERSTATE_IDLE, 0);
	exynos_smc(SMC_CMD_SHUTDOWN, OP_TYPE_CORE, SMC_POWERSTATE_IDLE, 0);
	/*
	 * Secure monitor disables the SMP bit and takes the CPU out of the
	 * coherency domain.
	 */
	local_flush_tlb_all();
#else
	cpu_do_idle();
#endif
	return 1;
}

static int exynos_enter_core0_aftr(struct cpuidle_device *dev,
				struct cpuidle_driver *drv,
				int index)
{
	struct timeval before, after;
	int idle_time;
	unsigned long tmp, abb_val;
	unsigned int ret = 0;
	unsigned int cpuid = smp_processor_id();

	raw_local_irq_disable();
	do_gettimeofday(&before);

	exynos_set_wakeupmask();

	/* Set value of power down register for aftr mode */
	exynos_sys_powerdown_conf(SYS_AFTR);

	__raw_writel(virt_to_phys(s3c_cpu_resume), REG_DIRECTGO_ADDR);
	__raw_writel(EXYNOS_CHECK_DIRECTGO, REG_DIRECTGO_FLAG);

	if (!soc_is_exynos5410()) {
		exynos_reset_assert_ctrl(0);

		abb_val = exynos4x12_get_abb_member(ABB_ARM);
		exynos4x12_set_abb_member(ABB_ARM, ABB_MODE_085V);
	}

	if (soc_is_exynos5410())
		exynos_disable_idle_clock_down(KFC);

	save_cpu_arch_register();

	/* Setting Central Sequence Register for power down mode */
	tmp = __raw_readl(EXYNOS_CENTRAL_SEQ_CONFIGURATION);
	tmp &= ~EXYNOS_CENTRAL_LOWPWR_CFG;
	__raw_writel(tmp, EXYNOS_CENTRAL_SEQ_CONFIGURATION);

	if (soc_is_exynos5410())
		set_boot_flag(cpuid, C2_STATE);

	cpu_pm_enter();

	ret = cpu_suspend(0, idle_finisher);
	if (ret) {
		tmp = __raw_readl(EXYNOS_CENTRAL_SEQ_CONFIGURATION);
		tmp |= EXYNOS_CENTRAL_LOWPWR_CFG;
		__raw_writel(tmp, EXYNOS_CENTRAL_SEQ_CONFIGURATION);
	}

#ifdef CONFIG_SMP
#if !defined(CONFIG_ARM_TRUSTZONE)
	scu_enable(S5P_VA_SCU);
#endif
#endif
	if (soc_is_exynos5410())
		clear_boot_flag(cpuid, C2_STATE);

	cpu_pm_exit();

	restore_cpu_arch_register();
	if (soc_is_exynos5410())
		exynos_enable_idle_clock_down(KFC);

	aftr_exynos_show_wakeup_reason();

	/* Clear wakeup state register */
	__raw_writel(0x0, EXYNOS_WAKEUP_STAT);

	if (!soc_is_exynos5410())
		exynos_reset_assert_ctrl(1);

	do_gettimeofday(&after);

	raw_local_irq_enable();
	idle_time = (after.tv_sec - before.tv_sec) * USEC_PER_SEC +
		    (after.tv_usec - before.tv_usec);

	dev->last_residency = idle_time;
	return index;
}

static struct sleep_save exynos4_lpa_save[] = {
	/* CMU side */
	SAVE_ITEM(EXYNOS4_CLKSRC_MASK_TOP),
	SAVE_ITEM(EXYNOS4_CLKSRC_MASK_CAM),
	SAVE_ITEM(EXYNOS4_CLKSRC_MASK_TV),
	SAVE_ITEM(EXYNOS4_CLKSRC_MASK_LCD0),
	SAVE_ITEM(EXYNOS4_CLKSRC_MASK_LCD1),
	SAVE_ITEM(EXYNOS4_CLKSRC_MASK_MAUDIO),
	SAVE_ITEM(EXYNOS4_CLKSRC_MASK_FSYS),
	SAVE_ITEM(EXYNOS4_CLKSRC_MASK_PERIL0),
	SAVE_ITEM(EXYNOS4_CLKSRC_MASK_PERIL1),
	SAVE_ITEM(EXYNOS4_CLKSRC_MASK_DMC),
};

static struct sleep_save exynos4_set_clksrc[] = {
	{ .reg = EXYNOS4_CLKSRC_MASK_TOP	, .val = 0x00000001, },
	{ .reg = EXYNOS4_CLKSRC_MASK_CAM	, .val = 0x11111111, },
	{ .reg = EXYNOS4_CLKSRC_MASK_TV		, .val = 0x00000111, },
	{ .reg = EXYNOS4_CLKSRC_MASK_LCD0	, .val = 0x00001111, },
	{ .reg = EXYNOS4_CLKSRC_MASK_MAUDIO	, .val = 0x00000001, },
	{ .reg = EXYNOS4_CLKSRC_MASK_FSYS	, .val = 0x01011111, },
	{ .reg = EXYNOS4_CLKSRC_MASK_PERIL0	, .val = 0x01111111, },
	{ .reg = EXYNOS4_CLKSRC_MASK_PERIL1	, .val = 0x01110111, },
	{ .reg = EXYNOS4_CLKSRC_MASK_DMC	, .val = 0x00010000, },
};

static struct sleep_save exynos5_lpa_save[] = {
	/* CMU side */
	SAVE_ITEM(EXYNOS5_CLKSRC_MASK_TOP),
	SAVE_ITEM(EXYNOS5_CLKSRC_MASK_GSCL),
	SAVE_ITEM(EXYNOS5_CLKSRC_MASK_DISP1_0),
	SAVE_ITEM(EXYNOS5_CLKSRC_MASK_MAUDIO),
	SAVE_ITEM(EXYNOS5_CLKSRC_MASK_FSYS),
	SAVE_ITEM(EXYNOS5_CLKSRC_MASK_PERIC0),
	SAVE_ITEM(EXYNOS5_CLKSRC_MASK_PERIC1),
	SAVE_ITEM(EXYNOS5_CLKSRC_TOP3),
};

static struct sleep_save exynos5_set_clksrc[] = {
	{ .reg = EXYNOS5_CLKSRC_MASK_TOP		, .val = 0xffffffff, },
	{ .reg = EXYNOS5_CLKSRC_MASK_GSCL		, .val = 0xffffffff, },
	{ .reg = EXYNOS5_CLKSRC_MASK_DISP1_0		, .val = 0xffffffff, },
	{ .reg = EXYNOS5_CLKSRC_MASK_MAUDIO		, .val = 0xffffffff, },
	{ .reg = EXYNOS5_CLKSRC_MASK_FSYS		, .val = 0xffffffff, },
	{ .reg = EXYNOS5_CLKSRC_MASK_PERIC0		, .val = 0xffffffff, },
	{ .reg = EXYNOS5_CLKSRC_MASK_PERIC1		, .val = 0xffffffff, },
};

#undef CONFIG_DETECT_IRQ_STORM

/* Check timer interrupts per second, if higher than a threshold, report the root cause! */
#define TIMER_IRQS_PER_SEC_THRESHOLD	100
/* To workaround LPA timer storm issue, define it. */
#undef STOP_LPA_AT_TIMER_STORM

/* Reboot while timer storm detected, defined it */
#undef REBOOT_AT_TIMER_STORM

int lpa_timer_storm = 0;

int lpa_timer_storm_detected(void)
{
	return lpa_timer_storm;
}

static inline int stop_lpa_at_timer_storm(void)
{
#ifdef STOP_LPA_AT_TIMER_STORM
	return lpa_timer_storm;
#else
	return 0;
#endif
}

static void lpa_work_func(struct work_struct *work)
{
	handle_sysrq('l');
	handle_sysrq('q');

#ifdef REBOOT_AT_TIMER_STORM
	//handle_sysrq('t');
	pr_info("%s: Reboot for LPA timer interrupts storm.\n", __func__);
	emergency_restart();
#endif
	enable_hlt();
}

struct delayed_work lpa_work;

static inline void lpa_check_timer_storm(void)
{
#ifdef CONFIG_DETECT_IRQ_STORM
	unsigned long wakeup_stat;
	static unsigned long prev_jiffies;
	static int timer_irqs;

	if (unlikely(lpa_timer_storm))
		return;

	wakeup_stat = __raw_readl(EXYNOS_WAKEUP_STAT);
	if (wakeup_stat & (1 << 12))
		timer_irqs++;

	pr_debug("timer_irqs = %d, jiffies = %lu, prev_jiffies = %lu\n", timer_irqs, jiffies, prev_jiffies);
	if (time_after(jiffies, prev_jiffies) || (prev_jiffies == 0)) {
		prev_jiffies = jiffies + msecs_to_jiffies(1000);
		if (timer_irqs > TIMER_IRQS_PER_SEC_THRESHOLD) {
			lpa_timer_storm = 1;
			disable_hlt();
			schedule_delayed_work(&lpa_work, HZ / 5);
		}
		timer_irqs = 0;
	}
#endif
}

extern void direct_access_to_debugging_info(unsigned int temp);

static int exynos_enter_core0_lpa(struct cpuidle_device *dev,
				struct cpuidle_driver *drv,
				int index)
{
	struct timeval before, after;
	int idle_time, ret = 0;
	unsigned long tmp, abb_val;
	unsigned int cpuid = smp_processor_id();

	//printk("[test] go lpa\n");

	direct_access_to_debugging_info(0x14000014);

	/*
	 * Before enter central sequence mode, clock src register have to set
	 */
	if (soc_is_exynos5410()) {
		s3c_pm_do_save(exynos5_lpa_save, ARRAY_SIZE(exynos5_lpa_save));
		s3c_pm_do_restore_core(exynos5_set_clksrc,
				       ARRAY_SIZE(exynos5_set_clksrc));
	} else {
		s3c_pm_do_save(exynos4_lpa_save, ARRAY_SIZE(exynos4_lpa_save));
		s3c_pm_do_restore_core(exynos4_set_clksrc,
					ARRAY_SIZE(exynos4_set_clksrc));
	}

	raw_local_irq_disable();
	do_gettimeofday(&before);

	/*
	 * Unmasking all wakeup source.
	 */
	if (soc_is_exynos5410()) {
	//	__raw_writel(0xFFFFFFFF, EXYNOS_EINT_WAKEUP_MASK);
		__raw_writel(0x7FFFE000, EXYNOS_WAKEUP_MASK);
	}
	else
		__raw_writel(0x3FF0000, EXYNOS_WAKEUP_MASK);

	/* Workaround: iRom need access mmc2 register, need hclk */
//	enable_mmc2_hclk();

	bt_uart_rts_ctrl(1);

	/* Configure GPIO Power down control register */
#ifdef SIMPLIFY_GPIO_POWER_DOWN_CONFIG
	exynos_gpio_set_pd_reg();
#else
	exynos_set_lpa_pdn(EXYNOS_GPIO_END);
#endif

	__raw_writel(virt_to_phys(s3c_cpu_resume), REG_DIRECTGO_ADDR);
	__raw_writel(EXYNOS_CHECK_DIRECTGO, REG_DIRECTGO_FLAG);

	if (!soc_is_exynos5410()) {
		exynos_reset_assert_ctrl(0);

		abb_val = exynos4x12_get_abb_member(ABB_ARM);
		exynos4x12_set_abb_member(ABB_ARM, ABB_MODE_085V);
	}

	direct_access_to_debugging_info(0x15000015);

	/* Set value of power down register for aftr mode */
	exynos_sys_powerdown_conf(SYS_LPA);

	if (soc_is_exynos5410())
		exynos_disable_idle_clock_down(KFC);

	save_cpu_arch_register();

	/* Setting Central Sequence Register for power down mode */
	tmp = __raw_readl(EXYNOS_CENTRAL_SEQ_CONFIGURATION);
	tmp &= ~EXYNOS_CENTRAL_LOWPWR_CFG;
	__raw_writel(tmp, EXYNOS_CENTRAL_SEQ_CONFIGURATION);

	do {
		/* Waiting for flushing UART fifo */
	} while (exynos_uart_fifo_check());

	if (soc_is_exynos5410())
		set_boot_flag(cpuid, C2_STATE);

	cpu_pm_enter();

	direct_access_to_debugging_info(0x16000016);

	ret = cpu_suspend(0, idle_finisher);

	direct_access_to_debugging_info(0x17000017);

	if (ret) {
		tmp = __raw_readl(EXYNOS_CENTRAL_SEQ_CONFIGURATION);
		tmp |= EXYNOS_CENTRAL_LOWPWR_CFG;
		__raw_writel(tmp, EXYNOS_CENTRAL_SEQ_CONFIGURATION);

		goto early_wakeup;
	}

#ifdef CONFIG_SMP
#if !defined(CONFIG_ARM_TRUSTZONE)
	scu_enable(S5P_VA_SCU);
#endif
#endif
	/* For release retention */
	if (!soc_is_exynos5410()) {
		__raw_writel((1 << 28), EXYNOS_PAD_RET_GPIO_OPTION);
		__raw_writel((1 << 28), EXYNOS_PAD_RET_UART_OPTION);
		__raw_writel((1 << 28), EXYNOS_PAD_RET_MMCA_OPTION);
		__raw_writel((1 << 28), EXYNOS_PAD_RET_MMCB_OPTION);
		__raw_writel((1 << 28), EXYNOS_PAD_RET_EBIA_OPTION);
		__raw_writel((1 << 28), EXYNOS_PAD_RET_EBIB_OPTION);
	} else {
		__raw_writel((1 << 28), EXYNOS5410_PAD_RET_GPIO_OPTION);
		__raw_writel((1 << 28), EXYNOS5410_PAD_RET_UART_OPTION);
		__raw_writel((1 << 28), EXYNOS5410_PAD_RET_MMCA_OPTION);
		__raw_writel((1 << 28), EXYNOS5410_PAD_RET_MMCB_OPTION);
		__raw_writel((1 << 28), EXYNOS5410_PAD_RET_MMCC_OPTION);
		__raw_writel((1 << 28), EXYNOS5410_PAD_RET_SPI_OPTION);
		__raw_writel((1 << 28), EXYNOS_PAD_RET_EBIA_OPTION);
		__raw_writel((1 << 28), EXYNOS_PAD_RET_EBIB_OPTION);
	}

early_wakeup:
	if (soc_is_exynos5410())
		clear_boot_flag(cpuid, C2_STATE);

	cpu_pm_exit();

	restore_cpu_arch_register();

	if (soc_is_exynos5410()) {
		exynos_enable_idle_clock_down(KFC);
		s3c_pm_do_restore_core(exynos5_lpa_save,
				       ARRAY_SIZE(exynos5_lpa_save));
	} else {
		s3c_pm_do_restore_core(exynos4_lpa_save,
			       ARRAY_SIZE(exynos4_lpa_save));
		exynos_reset_assert_ctrl(1);
	}

	direct_access_to_debugging_info(0x18000018);

	lpa_check_timer_storm();
	lpa_exynos_show_wakeup_reason();
	/* Clear wakeup state register */
	__raw_writel(0x0, EXYNOS_WAKEUP_STAT);

	bt_uart_rts_ctrl(0);
//	disable_mmc2_hclk();

	do_gettimeofday(&after);

	raw_local_irq_enable();
	idle_time = (after.tv_sec - before.tv_sec) * USEC_PER_SEC +
		    (after.tv_usec - before.tv_usec);

	dev->last_residency = idle_time;
	direct_access_to_debugging_info(0x19000019);

	return index;
}

static int exynos_enter_idle(struct cpuidle_device *dev,
				struct cpuidle_driver *drv,
				int index)
{
	struct timeval before, after;
	int idle_time;

	raw_local_irq_disable();
	do_gettimeofday(&before);

	cpu_do_idle();

	do_gettimeofday(&after);
	raw_local_irq_enable();
	idle_time = (after.tv_sec - before.tv_sec) * USEC_PER_SEC +
		    (after.tv_usec - before.tv_usec);

	dev->last_residency = idle_time;
	return index;
}

static unsigned int exynos_get_core_num(void)
{
	unsigned int cluster_id = read_cpuid_mpidr() & 0x100;
	unsigned int pwr_offset = 0;
	unsigned int cpu;
	unsigned int tmp;

	struct cpumask cpu_power_on_mask;

	cpumask_clear(&cpu_power_on_mask);

	if (samsung_rev() < EXYNOS5410_REV_1_0) {
		if (cluster_id == 0)
			pwr_offset = 4;
	} else {
		if (cluster_id != 0)
			pwr_offset = 4;
	}

	for (cpu = 0; cpu < num_possible_cpus(); cpu++) {
		tmp = __raw_readl(EXYNOS_ARM_CORE_STATUS(cpu + pwr_offset));

		if (tmp & EXYNOS_CORE_LOCAL_PWR_EN)
			cpumask_set_cpu(cpu, &cpu_power_on_mask);
	}

	return cpumask_weight(&cpu_power_on_mask);
}

static int exynos_enter_lowpower(struct cpuidle_device *dev,
				struct cpuidle_driver *drv,
				int index)
{
	int new_index = index;
	int idle_time;

	if (!soc_is_exynos5410())
		__raw_writel((EXYNOS4_USE_STANDBY_WFI0 |
				EXYNOS4_USE_STANDBY_WFE0),
				EXYNOS_CENTRAL_SEQ_OPTION);

	/* This mode only can be entered when other core's are offline */
	if (num_online_cpus() > 1) {
		if (soc_is_exynos5410()) {
			led_trigger_event(exynos5410_idle1_led_trigger, LED_FULL);
#ifndef EXYNOS5410_ENTER_LOWPOWER
			idle_time = exynos_enter_idle(dev, drv, 0);
#else
			idle_time = exynos5410_enter_lowpower(dev, drv, (new_index - 1));
#endif
			led_trigger_event(exynos5410_idle1_led_trigger, LED_OFF);
			return idle_time;
		} else
			return exynos_enter_idle(dev, drv, 0);
	}

	if (soc_is_exynos5410() && (exynos_get_core_num() > 1)) {
		led_trigger_event(exynos5410_idle2_led_trigger, LED_FULL);
#ifndef EXYNOS5410_ENTER_LOWPOWER
		idle_time = exynos_enter_idle(dev, drv, 0);
#else
		idle_time = exynos5410_enter_lowpower(dev, drv, (new_index - 1));
#endif
		led_trigger_event(exynos5410_idle2_led_trigger, LED_OFF);
		return idle_time;
	}

#ifdef LPA_IS_BROKEN
	led_trigger_event(aftr_led_trigger, LED_FULL);
	idle_time = exynos_enter_core0_aftr(dev, drv, new_index);
	led_trigger_event(aftr_led_trigger, LED_OFF);
#else
	//misc_before_idle();
	if (stop_lpa_at_timer_storm() || (exynos_check_enter_mode() == EXYNOS_CHECK_DIDLE)) {
		led_trigger_event(aftr_led_trigger, LED_FULL);
		idle_time = exynos_enter_core0_aftr(dev, drv, new_index);
		led_trigger_event(aftr_led_trigger, LED_OFF);
	} else {
		led_trigger_event(lpa_led_trigger, LED_FULL);
		idle_time = exynos_enter_core0_lpa(dev, drv, new_index);
		led_trigger_event(lpa_led_trigger, LED_OFF);
	}
	//misc_after_idle();
#endif

	return idle_time;
}

static int exynos5410_enter_lowpower(struct cpuidle_device *dev,
				struct cpuidle_driver *drv,
				int index)
{
	struct timeval before, after;
	int idle_time, ret = 0;
	unsigned int cpuid = smp_processor_id(), cpu_offset = 0;
	unsigned int cluster_id = read_cpuid(CPUID_MPIDR) >> 8 & 0xf;
	unsigned int value;

	raw_local_irq_disable();
	do_gettimeofday(&before);

	__raw_writel(virt_to_phys(s3c_cpu_resume), REG_DIRECTGO_ADDR);
	__raw_writel(EXYNOS_CHECK_DIRECTGO, REG_DIRECTGO_FLAG);

	set_boot_flag(cpuid, C2_STATE);
	cpu_pm_enter();

	if (samsung_rev() < EXYNOS5410_REV_1_0) {
		if (cluster_id == 0)
			cpu_offset = cpuid + 4;
		else
			cpu_offset = cpuid;
	} else {
		if (cluster_id == 0)
			cpu_offset = cpuid;
		else
			cpu_offset = cpuid + 4;
	}
	__raw_writel(0x0, EXYNOS_ARM_CORE_CONFIGURATION(cpu_offset));

	value = __raw_readl(EXYNOS5410_ARM_INTR_SPREAD_ENABLE);
	value &= ~(0x1 << cpu_offset);
	__raw_writel(value, EXYNOS5410_ARM_INTR_SPREAD_ENABLE);

	ret = cpu_suspend(0, c2_finisher);
	if (ret)
		__raw_writel(0x3, EXYNOS_ARM_CORE_CONFIGURATION(cpu_offset));

	value = __raw_readl(EXYNOS5410_ARM_INTR_SPREAD_ENABLE);
	value |= (0x1 << cpu_offset);
	__raw_writel(value, EXYNOS5410_ARM_INTR_SPREAD_ENABLE);

	clear_boot_flag(cpuid, C2_STATE);
	cpu_pm_exit();

	do_gettimeofday(&after);
	raw_local_irq_enable();
	idle_time = (after.tv_sec - before.tv_sec) * USEC_PER_SEC +
		    (after.tv_usec - before.tv_usec);

	dev->last_residency = idle_time;
	return index;
}

static int __exynos5410_enter_lowpower(struct cpuidle_device *dev,
				struct cpuidle_driver *drv,
				int index)
{
	int idle_time;

	led_trigger_event(exynos5410_idle3_led_trigger, LED_FULL);
	idle_time = exynos5410_enter_lowpower(dev, drv, index);
	led_trigger_event(exynos5410_idle3_led_trigger, LED_OFF);

	return idle_time;
}

void exynos_enable_idle_clock_down(unsigned int cluster)
{
	unsigned int tmp;

	if (cluster) {
		/* For A15 core */
		tmp = __raw_readl(EXYNOS5_PWR_CTRL1);
		tmp &= ~((0x7 << 28) | (0x7 << 16) | (1 << 9) | (1 << 8));
		tmp |= (0x7 << 28) | (0x7 << 16) | 0x3ff;
		__raw_writel(tmp, EXYNOS5_PWR_CTRL1);

		tmp = __raw_readl(EXYNOS5_PWR_CTRL2);
		tmp &= ~((0x3 << 24) | (0xffff << 8) | (0x77));
		tmp |= (1 << 16) | (1 << 8) | (1 << 4) | (1 << 0);
		tmp |= (1 << 25) | (1 << 24);
		__raw_writel(tmp, EXYNOS5_PWR_CTRL2);
	} else {
		/* For A7 core */
		tmp = __raw_readl(EXYNOS5_PWR_CTRL_KFC);
		tmp &= ~((0x7 << 16) | (1 << 8));
		tmp |= (0x7 << 16) | 0x1ff;
		__raw_writel(tmp, EXYNOS5_PWR_CTRL_KFC);

		tmp = __raw_readl(EXYNOS5_PWR_CTRL2_KFC);
		tmp &= ~((0x1 << 24) | (0xffff << 8) | (0x7));
		tmp |= (1 << 16) | (1 << 8) | (1 << 0);
		tmp |= 1 << 24;
		__raw_writel(tmp, EXYNOS5_PWR_CTRL2_KFC);
	}

	pr_debug("%s idle clock down is enabled\n", cluster ? "ARM" : "KFC");
}

void exynos_disable_idle_clock_down(unsigned int cluster)
{
	unsigned int tmp;

	if (cluster) {
		/* For A15 core */
		tmp = __raw_readl(EXYNOS5_PWR_CTRL1);
		tmp &= ~((0x7 << 28) | (0x7 << 16) | (1 << 9) | (1 << 8));
		__raw_writel(tmp, EXYNOS5_PWR_CTRL1);

		tmp = __raw_readl(EXYNOS5_PWR_CTRL2);
		tmp &= ~((0x3 << 24) | (0xffff << 8) | (0x77));
		__raw_writel(tmp, EXYNOS5_PWR_CTRL2);
	} else {
		/* For A7 core */
		tmp = __raw_readl(EXYNOS5_PWR_CTRL_KFC);
		tmp &= ~((0x7 << 16) | (1 << 8));
		__raw_writel(tmp, EXYNOS5_PWR_CTRL_KFC);

		tmp = __raw_readl(EXYNOS5_PWR_CTRL2_KFC);
		tmp &= ~((0x1 << 24) | (0xffff << 8) | (0x7));
		__raw_writel(tmp, EXYNOS5_PWR_CTRL2_KFC);
	}

	pr_debug("%s idle clock down is disabled\n", cluster ? "ARM" : "KFC");
}

#ifdef CONFIG_BL_SWITCHER
static int bl_notifier_event(struct notifier_block *this,
					  unsigned long event,
					  void *ptr)
{
	switch (event) {
	case SWITCH_ENTER:
		disable_hlt();
		pr_debug("@@@@ SWITCH_ENTER for cpuidle\n");
		return NOTIFY_OK;
	case SWITCH_EXIT:
		enable_hlt();
		pr_debug("@@@@ SWITCH_EXIT for cpuidle\n");
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static struct notifier_block bL_notifier = {
	.notifier_call = bl_notifier_event,
};
#endif

#ifdef CONFIG_HOTPLUG
static int cpu_notifier_event(struct notifier_block *this,
					  unsigned long event,
					  void *ptr)
{
	switch (event) {
	case CPU_UP_PREPARE:
		disable_hlt();
		pr_debug("@@@@ hotplug enter for cpuidle\n");
		return NOTIFY_OK;
	case CPU_UP_CANCELED:
	case CPU_ONLINE:
		enable_hlt();
		pr_debug("@@@@ hotplug exit for cpuidle\n");
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static struct notifier_block cpu_notifier = {
	.notifier_call = cpu_notifier_event,
};
#endif

static int exynos_cpuidle_notifier_event(struct notifier_block *this,
					  unsigned long event,
					  void *ptr)
{
	switch (event) {
	case PM_SUSPEND_PREPARE:
		disable_hlt();
		pr_debug("@@@@ PM_SUSPEND_PREPARE for CPUIDLE\n");
		return NOTIFY_OK;
	case PM_POST_RESTORE:
	case PM_POST_SUSPEND:
		enable_hlt();
		pr_debug("@@@@ PM_POST_SUSPEND for CPUIDLE\n");
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static struct notifier_block exynos_cpuidle_notifier = {
	.notifier_call = exynos_cpuidle_notifier_event,
};

static int __init exynos_init_cpuidle(void)
{
	int i, max_cpuidle_state, cpu_id;
	struct cpuidle_device *device;
	struct cpuidle_driver *drv = &exynos_idle_driver;
	struct cpuidle_state *idle_set;
	struct platform_device *pdev;
	struct resource *res;

	if (soc_is_exynos5410()) {
		exynos_enable_idle_clock_down(ARM);
		exynos_enable_idle_clock_down(KFC);
	}

	/* Setup cpuidle driver */
	if (soc_is_exynos5410()) {
		idle_set = exynos5410_cpuidle_set;
		drv->state_count = ARRAY_SIZE(exynos5410_cpuidle_set);
	} else {
		idle_set = exynos_cpuidle_set;
		drv->state_count = ARRAY_SIZE(exynos_cpuidle_set);
	}

	max_cpuidle_state = drv->state_count;
	for (i = 0; i < max_cpuidle_state; i++) {
		memcpy(&drv->states[i], &idle_set[i],
				sizeof(struct cpuidle_state));
	}
	drv->safe_state_index = 0;
	cpuidle_register_driver(&exynos_idle_driver);

	for_each_cpu(cpu_id, cpu_online_mask) {
		device = &per_cpu(exynos_cpuidle_device, cpu_id);
		device->cpu = cpu_id;

	if (cpu_id != 0 && !soc_is_exynos5410())
		device->state_count = 1;
	else
		 device->state_count = max_cpuidle_state;

	if (cpuidle_register_device(device)) {
		printk(KERN_ERR "CPUidle register device failed\n,");
		return -EIO;
		}
	}

#if defined(CONFIG_EXYNOS_DEV_DWMCI)
	sdmmc_dev_num = ARRAY_SIZE(chk_sdhc_op);

	for (i = 0; i < sdmmc_dev_num; i++) {

		pdev = chk_sdhc_op[i].pdev;

		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res) {
			pr_err("failed to get iomem region\n");
			return -EINVAL;
		}

		chk_sdhc_op[i].base = ioremap(res->start, resource_size(res));

		if (!chk_sdhc_op[i].base) {
			pr_err("failed to map io region\n");
			return -EINVAL;
		}
	}
#endif
	if (soc_is_exynos4412()) {
		/* Save register value for SCU */
		scu_save[0] = __raw_readl(S5P_VA_SCU + 0x30);
		scu_save[1] = __raw_readl(S5P_VA_SCU + 0x0);

		flush_cache_all();
		outer_clean_range(virt_to_phys(scu_save), ARRAY_SIZE(scu_save));
	}

	register_pm_notifier(&exynos_cpuidle_notifier);

#ifdef CONFIG_BL_SWITCHER
	register_bL_switcher_notifier(&bL_notifier);
#endif

#ifdef CONFIG_HOTPLUG
	register_cpu_notifier(&cpu_notifier);
#endif

	exynos_init_leds_triggers();

#ifdef CONFIG_DETECT_IRQ_STORM
	INIT_DELAYED_WORK(&lpa_work, lpa_work_func);
#endif

	cpumask_clear(&out_cpus);

	return 0;
}
device_initcall(exynos_init_cpuidle);

#ifdef CONFIG_WAKEUP_REASON
static ssize_t show_idle_debug(struct kobject *kobj, struct attribute *attr, char *buf)
{
	int ret;

	ret = sprintf(buf, "%s\n", idle_wakeup_debug_str[idle_wakeup_debug]);

	return ret;
}

static ssize_t __ref store_idle_debug(struct kobject *kobj, struct attribute *attr,
			      const char *buf, size_t count)
{
	int ret = sscanf(buf, "%d", &idle_wakeup_debug);
	if (ret != 1)
		return -EINVAL;

	if (idle_wakeup_debug < 0 || idle_wakeup_debug > 2) {
		pr_err("idle_wakeup_debug must be 0, 1, 2 for none, lpa and aftr wakeup debug\n");
		idle_wakeup_debug = 0;
	}

	return count;
}

define_one_global_rw(idle_debug);

static int __init exynos_idle_wakeup_debug_init(void)
{
	int ret;

	ret = sysfs_create_file(power_kobj, &idle_debug.attr);
	if (ret) {
		pr_err("%s: failed to create /sys/power/idle_wakeup_debug sysfs interface\n", __func__);
		return ret;
	}
	return 0;
}
subsys_initcall(exynos_idle_wakeup_debug_init);
#endif
