/*
 * arch/arm/mach-exynos/resetreason.c
 *
 * Copyright (C) 2012 Google, Inc.
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

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/bug.h>
#include <linux/io.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <mach/regs-pmu.h>
#include <mach/resetreason.h>
#include <linux/kmsg_dump.h>
#include <linux/platform_device.h>
#include <linux/cma.h>

#include <plat/regs-rtc.h>
#include <plat/pm.h>

#define EXYNOS_RST_STAT_ISP_ARM_WDTRESET	(1<<29)
#define EXYNOS_RST_STAT_SWRESET		(1<<11)
#define EXYNOS_RST_STAT_WRSET		(1<<10)
#define EXYNOS_RST_STAT_SYS_WDTRESET	(1<<9)
#define EXYNOS_RST_STAT_PINRESET	(1<<8)
#define EXYNOS_RST_STAT_KFC_WRESET3	(1<<7)
#define EXYNOS_RST_STAT_KFC_WRESET2	(1<<6)
#define EXYNOS_RST_STAT_KFC_WRESET1	(1<<5)
#define EXYNOS_RST_STAT_KFC_WRESET0	(1<<4)
#define EXYNOS_RST_STAT_EAGLE_WRESET3	(1<<3)
#define EXYNOS_RST_STAT_EAGLE_WRESET2	(1<<2)
#define EXYNOS_RST_STAT_EAGLE_WRESET1	(1<<1)
#define EXYNOS_RST_STAT_EAGLE_WRESET0	(1<<0)
#define BOOT_INFO_LABEL "Boot info: "
#define BOOT_FROM_LABEL "Boot from: "
#define BOOT_CMD_LABEL  "Boot cmd : "
#define BOOT_STAT_LABEL "Boot stat:\n"

#define RESET_REASON_SIG (0x53535655)

static char resetreason[1024];
u32 reasons;

#ifdef CONFIG_SAMSUNG_PM_RTC_DEBUG
static u32 rtc_mon, rtc_year;
#endif

enum reset_reason {
	SW_RESTART,		/* Software reboot */
	OOPS_RESTART,
	PANIC_RESTART,
	HALT_RESTART,
	POWEROFF_RESTART,
	EMERG_RESTART,
	CRIT_TEMP_RESTART,
	WR_RESTART,		/* Hardware reset */
	SYS_WDT_RESTART,
	ISP_WDT_RESTART,
	PIN_RESTART,
	KFC_WARM,
	EAGLE_WARM,
	UNKNOWN_REASON,
	END_REASON,		/* For end */
};

static char reset_reason_str[END_REASON][20] = {
	"software reboot",
	"oops reboot",
	"panic reboot",
	"halt reboot",
	"poweroff reboot",
	"emerg reboot",
	"crit temp reboot",
	"warm reset",
	"sys_wdt_reset",
	"isp_wdt reset",
	"pin reset",
	"kfc_wram reset",
	"eagle_warm reset",
	"unknow reason",
};

static struct {
	int flag;
	const char *str;
	u32 mask;
} hw_resetreason_flags[] = {
	{ SW_RESTART, "software", EXYNOS_RST_STAT_SWRESET },
	{ WR_RESTART, "warm", EXYNOS_RST_STAT_WRSET },
	{ ISP_WDT_RESTART, "isp_wachdog", EXYNOS_RST_STAT_ISP_ARM_WDTRESET },
	{ SYS_WDT_RESTART, "system_wachdog", EXYNOS_RST_STAT_SYS_WDTRESET },
	{ PIN_RESTART, "pin", EXYNOS_RST_STAT_PINRESET },
	{ KFC_WARM, "kfc_warm",	EXYNOS_RST_STAT_KFC_WRESET3 |
				EXYNOS_RST_STAT_KFC_WRESET2 |
				EXYNOS_RST_STAT_KFC_WRESET1 |
				EXYNOS_RST_STAT_KFC_WRESET0 },
	{ EAGLE_WARM, "eagle_warm", EXYNOS_RST_STAT_EAGLE_WRESET3 |
				EXYNOS_RST_STAT_EAGLE_WRESET2 |
				EXYNOS_RST_STAT_EAGLE_WRESET1 |
				EXYNOS_RST_STAT_EAGLE_WRESET0 },
};

static struct {
	int flag;
	int mask;
} sw_resetreason_flags[] = {
	{ OOPS_RESTART,		KMSG_DUMP_OOPS },
	{ PANIC_RESTART,	KMSG_DUMP_PANIC },
	{ SW_RESTART,		KMSG_DUMP_RESTART },
	{ HALT_RESTART,		KMSG_DUMP_HALT },
	{ POWEROFF_RESTART,	KMSG_DUMP_POWEROFF },
	{ EMERG_RESTART,	KMSG_DUMP_EMERG },
	{ CRIT_TEMP_RESTART,	KMSG_DUMP_CRIT_TEMP },
};

struct normal_reboot {
	int flag;
	char cmd[50];
};

/*The struct for record reset reason */
struct reset_info {
	uint32_t  sig;
	int resetreason;
	unsigned long long count[END_REASON];
	struct normal_reboot normal_reboot_info;
};

struct reset_info *resetinfoPtr;

static void reset_normal_reboot_reason(void)
{
	memset(&resetinfoPtr->normal_reboot_info, 0, sizeof(struct normal_reboot));
}

void record_normal_reboot_reason(const char *cmd)
{
	if (!resetinfoPtr) {
		pr_info("%s: Get the cam ram failed \n", __func__);
		return;
	}
	if (cmd) {
		resetinfoPtr->normal_reboot_info.flag = 1;
		strcpy(resetinfoPtr->normal_reboot_info.cmd, cmd);
	} else {
		reset_normal_reboot_reason();
	}
}

static void inc_reset_reason(enum kmsg_dump_reason reason)
{
	if (!resetinfoPtr) {
		pr_info("%s: Get the cam ram failed \n", __func__);
		return;
	}
	resetinfoPtr->resetreason = reason;
	resetinfoPtr->count[resetinfoPtr->resetreason]++;
	pr_info("%s resetreason = %s, count = %llu \n", __func__,
		reset_reason_str[resetinfoPtr->resetreason],
		resetinfoPtr->count[resetinfoPtr->resetreason]);
}

static DEFINE_SPINLOCK(recorded_spin);
static atomic_t recorded;
void record_reset_reason(enum kmsg_dump_reason reason)
{
	int i;
	spin_lock(&recorded_spin);
	if (atomic_read(&recorded)) {
		pr_debug("Already recorded the reset reason\n");
		spin_unlock(&recorded_spin);
		return;
	}
	/* Record once */
	atomic_set(&recorded, 1);
	spin_unlock(&recorded_spin);
	for (i = 0; i < ARRAY_SIZE(sw_resetreason_flags); i++) {
		if (reason == sw_resetreason_flags[i].mask)
			inc_reset_reason(sw_resetreason_flags[i].flag);
	}
}

static void check_hardware_reset_reason(void)
{
	int i;
	pr_info("%s: reset_reason = 0x%x\n", __func__, reasons);
	/*i = 1 to ignore software reset*/
	for (i = 1; i < ARRAY_SIZE(hw_resetreason_flags); i++)
		if (reasons & hw_resetreason_flags[i].mask)
			inc_reset_reason(hw_resetreason_flags[i].flag);
}

#define get_current_reset_reason() (resetinfoPtr->resetreason)
#define get_reset_reason_str(r)	(reset_reason_str[r])
#define get_reset_count(r) (resetinfoPtr->count[r])

const char *exynos_get_resetreason(void)
{
	return resetreason;
}

/* /proc/resetreason */
static int resetreason_show(struct seq_file *p, void *v)
{
	int i;
	pr_info("%s the resetreason is %s\n", __func__, resetreason);
	if (!resetinfoPtr) {
		pr_info("%s Get cma ram failed ....\n ", __func__);
		seq_printf(p, "%s", resetreason);
		return -EFAULT;
	}
	seq_printf(p, BOOT_FROM_LABEL "%d, %s, %llu\n",
		resetinfoPtr->resetreason,
		reset_reason_str[resetinfoPtr->resetreason],
		resetinfoPtr->count[resetinfoPtr->resetreason]);
	if (resetinfoPtr->normal_reboot_info.flag == 1) {
		seq_printf(p, BOOT_CMD_LABEL "reboot %s\n",
			resetinfoPtr->normal_reboot_info.cmd);
	}
	seq_printf(p, BOOT_STAT_LABEL);

	for (i = SW_RESTART; i < END_REASON; i++) {
		pr_debug("%d, %s, %llu\n", i,
			get_reset_reason_str(i),
			get_reset_count(i));
		seq_printf(p, "%d, %s, %llu\n", i,
			get_reset_reason_str(i),
			get_reset_count(i));
		}
#ifdef CONFIG_SAMSUNG_PM_RTC_DEBUG
	seq_printf(p, "\n");
	seq_printf(p, "rtc month = 0x%x\n", rtc_mon);
	seq_printf(p, "rtc year  = 0x%x\n", rtc_year);
#endif
	return 0;
}

static void *resetreason_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

static void *resetreason_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

static void resetreason_stop(struct seq_file *m, void *v)
{
	/* Nothing to do */
}

const struct seq_operations resetreason_op = {
	.start = resetreason_start,
	.next = resetreason_next,
	.stop = resetreason_stop,
	.show = resetreason_show
};

static int proc_resetreason_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &resetreason_op);
}

static const struct file_operations proc_resetreason_operations = {
	.open = proc_resetreason_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static int reset_mem_probe(struct platform_device *pdev)
{
	size_t start;
	size_t buffer_size;
	buffer_size = SZ_128K;

	start = cma_alloc(&pdev->dev, "reset_reason", buffer_size, 0);

	if (IS_ERR_VALUE(start)) {
		pr_err("%s: CMA Alloc Error!!!start = %d", __func__, start);
		return start;
	}

	/*Get the virtual addrs */
	resetinfoPtr = cma_get_virt(start, buffer_size, 1);
	if (!resetinfoPtr)
		return 0;
	if (resetinfoPtr->sig != RESET_REASON_SIG) {
		pr_info("%s The ram which record reasons was damaged \n",
			__func__);
		memset(resetinfoPtr, 0, sizeof(struct reset_info));
	}
	check_hardware_reset_reason();
	resetinfoPtr->sig = RESET_REASON_SIG;
	return 0;
}

static struct platform_driver reset_reason_mem_driver = {
	.driver = {
		   .name = "reset_reason",
		   .owner = THIS_MODULE,
		   },

	.probe = reset_mem_probe,
};

static int __init resetreason_init(void)
{
	pr_info("###############start register reset_reason_mem########\n");
	reasons = __raw_readl(EXYNOS_RST_STAT);
	platform_driver_register(&reset_reason_mem_driver);
	proc_create("reset_reason", S_IFREG | S_IRUGO, NULL,
		    &proc_resetreason_operations);
	return 0;
}

device_initcall_sync(resetreason_init);

static int __init _resetreason_init(void)
{
	int i;
	u32 reasons = __raw_readl(EXYNOS_RST_STAT);
	char buf[128];

	strlcpy(resetreason, "Last reset was ", sizeof(resetreason));

	for (i = 0; i < ARRAY_SIZE(hw_resetreason_flags); i++)
		if (reasons & hw_resetreason_flags[i].mask)
			strlcat(resetreason, hw_resetreason_flags[i].str,
				sizeof(resetreason));

	snprintf(buf, sizeof(buf), " reset (RST_STAT=0x%x)\n", reasons);
	strlcat(resetreason, buf, sizeof(resetreason));
	pr_info("%s", resetreason);

#ifdef CONFIG_SAMSUNG_PM_RTC_DEBUG
	rtc_mon  = __raw_readl(S3C_VA_RTC + S3C2410_RTCMON);
	rtc_year = __raw_readl(S3C_VA_RTC + S3C2410_RTCYEAR);

	pr_info("rtc month = 0x%x\n", rtc_mon);
	pr_info("rtc year  = 0x%x\n", rtc_year);
#endif
	return 0;
}

postcore_initcall(_resetreason_init);
