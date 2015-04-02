/*
 * RTC tick wake up test
 */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kobject.h>
#include <linux/device.h>

#include <asm/tlbflush.h>
#include <asm/cacheflush.h>
#include <asm/cputype.h>

#include <plat/regs-rtc.h>

#include <mach/regs-pmu.h>


extern struct kobject *power_kobj;
extern unsigned long s3c_irqwake_intmask;

int rtc_tick_wakeup_enable = 0;

static int tick_max = 0xD82;  // 20ms  0.02/0.00003=666, 29a I 50ms 0.05/0.00003=1666
static int tick_min = 0x42;   // 2ms   0.002/0.00003=1666
static int tick_curr = 0xD82;
static int tick_step = 0x42;
static u32 rtccon = 0;


static void s3c_rtc_tick_wakeup_enable(void)
{
	/*TIC Disable*/
	rtccon = __raw_readl(S3C_VA_RTC + S3C2410_RTCCON);
	rtccon &= ~(S3C64XX_RTCCON_TICEN);
	__raw_writel(rtccon, S3C_VA_RTC + S3C2410_RTCCON);

	/*RTC Disable*/
	rtccon = __raw_readl(S3C_VA_RTC + S3C2410_RTCCON);
	rtccon &= ~(S3C2410_RTCCON_RTCEN);
	__raw_writel(rtccon, S3C_VA_RTC + S3C2410_RTCCON);

	/*RTC EN*/
	rtccon = __raw_readl(S3C_VA_RTC + S3C2410_RTCCON);
	rtccon |= (S3C2410_RTCCON_RTCEN);
	__raw_writel(rtccon, S3C_VA_RTC + S3C2410_RTCCON);

	/*TICCKSEL*/
	rtccon = __raw_readl(S3C_VA_RTC + S3C2410_RTCCON);
	rtccon |= ~(0xF0);
	__raw_writel(rtccon, S3C_VA_RTC + S3C2410_RTCCON);

	tick_curr -= tick_step;
	if(tick_curr <= tick_min)
		tick_curr = tick_max;

	flush_cache_all();

	__raw_writel(tick_curr, S3C_VA_RTC + S3C2410_TICNT);
	pr_info("S3C2410_RTCCON: 0x%08x\n", __raw_readl(S3C_VA_RTC + S3C2410_RTCCON));
	pr_info("S3C2410_TICNT: 0x%08x\n",  __raw_readl(S3C_VA_RTC + S3C2410_TICNT));
	pr_info("S3C2410_CURTICCNT: 0x%08x\n", __raw_readl(S3C_VA_RTC + 0x90));

	/*TIC EN*/
	rtccon = __raw_readl(S3C_VA_RTC + S3C2410_RTCCON);
	rtccon |= S3C64XX_RTCCON_TICEN;
	__raw_writel(rtccon, S3C_VA_RTC + S3C2410_RTCCON);
}

static void s3c_rtc_tick_wakeup_disable(void)
{
	/*TIC Disable*/
	rtccon = __raw_readl(S3C_VA_RTC + S3C2410_RTCCON);
	rtccon &= ~(S3C64XX_RTCCON_TICEN);
	__raw_writel(rtccon, S3C_VA_RTC + S3C2410_RTCCON);
}

void s3c_rtc_tick_wakeup_test(void)
{
	if (rtc_tick_wakeup_enable) {
		s3c_rtc_tick_wakeup_enable();
		s3c_irqwake_intmask &= ~(0x1<<2);
		__raw_writel(s3c_irqwake_intmask, EXYNOS_WAKEUP_MASK);
	} else if (!(s3c_irqwake_intmask & (0x1<<2))){
		s3c_rtc_tick_wakeup_disable();
		s3c_irqwake_intmask |= (0x1<<2);
		__raw_writel(s3c_irqwake_intmask, EXYNOS_WAKEUP_MASK);
	}
}

static ssize_t s3c_rtc_tick_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", rtc_tick_wakeup_enable);
}

static ssize_t s3c_rtc_tick_test_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t n)
{
	int val;

	if (sscanf(buf, "%d", &val) == 1) {
		rtc_tick_wakeup_enable = !!val;
		return n;
	}

	return -EINVAL;
}
static DEVICE_ATTR(s3c_tick_wakeup_enable, 0666, s3c_rtc_tick_test_show, s3c_rtc_tick_test_store);

static int __init s3c_rtc_tick_test_init(void)
{
	int err = -EINVAL;
	
	if (!power_kobj)
		goto obj_err;

	err = sysfs_create_file(power_kobj, &dev_attr_s3c_tick_wakeup_enable.attr);
	
	if (err) {
		pr_err("%s: failed to create /sys/power/s3c_tick_wakeup_enable\n", __func__);
	}

obj_err:
	return err;
}

arch_initcall(s3c_rtc_tick_test_init);
