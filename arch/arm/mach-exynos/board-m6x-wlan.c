/*
 * Copyright (C) 2012 Meizu Technology Co.Ltd, Zhuhai, China
 *		http://www.meizu.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/wakelock.h>

#include <asm/mach-types.h>

#include <plat/devs.h>
#include <plat/irqs.h>
#include <linux/mmc/host.h>
#include <plat/sdhci.h>
#include <plat/gpio-cfg.h>
#include <mach/gpio-meizu.h>

#include <linux/fs.h>

static int wl_power;
static struct wake_lock wifi_wake_lock;
static DEFINE_MUTEX(wifi_mutex);


#define PREALLOC_WLAN_SEC_NUM		4
#define WLAN_STATIC_SCAN_BUF0		5
#define WLAN_STATIC_SCAN_BUF1		6
#define PREALLOC_WLAN_BUF_NUM		160
#define PREALLOC_WLAN_SECTION_HEADER	24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_BUF_NUM * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_BUF_NUM * 1024)

#define DHD_SKB_HDRSIZE			336
#define DHD_SKB_1PAGE_BUFSIZE	((PAGE_SIZE*1)-DHD_SKB_HDRSIZE)
#define DHD_SKB_2PAGE_BUFSIZE	((PAGE_SIZE*2)-DHD_SKB_HDRSIZE)
#define DHD_SKB_4PAGE_BUFSIZE	((PAGE_SIZE*4)-DHD_SKB_HDRSIZE)

#define WLAN_SKB_BUF_NUM	17
 
static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

struct wlan_mem_prealloc {
	void *mem_ptr;
	unsigned long size;
};

static struct wlan_mem_prealloc wlan_mem_array[PREALLOC_WLAN_SEC_NUM] = {
	{NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER)}
};

void *wlan_static_scan_buf0;
void *wlan_static_scan_buf1;

static void *brcm_wlan_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_SEC_NUM)
		return wlan_static_skb;
	if (section == WLAN_STATIC_SCAN_BUF0)
		return wlan_static_scan_buf0;
	if (section == WLAN_STATIC_SCAN_BUF1)
		return wlan_static_scan_buf1;
	if ((section < 0) || (section > PREALLOC_WLAN_SEC_NUM))
		return NULL;
	if (wlan_mem_array[section].size < size)
		return NULL;
	return wlan_mem_array[section].mem_ptr;
}

static int __init brcm_init_wifi_mem(void)
{
	int i;
	int j;

	for (i = 0; i < 8; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_1PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	for (; i < 16; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_2PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_4PAGE_BUFSIZE);
	if (!wlan_static_skb[i])
		goto err_skb_alloc;

	for (i = 0 ; i < PREALLOC_WLAN_SEC_NUM ; i++) {
		wlan_mem_array[i].mem_ptr =
				kmalloc(wlan_mem_array[i].size, GFP_KERNEL);

		if (!wlan_mem_array[i].mem_ptr)
			goto err_mem_alloc;
	}
	wlan_static_scan_buf0 = kmalloc(65536, GFP_KERNEL);
	if (!wlan_static_scan_buf0)
		goto err_mem_alloc;
	wlan_static_scan_buf1 = kmalloc(65536, GFP_KERNEL);
	if (!wlan_static_scan_buf1)
		goto err_mem_alloc;
	printk(KERN_INFO"%s: WIFI MEM Allocated\n", __func__);
	return 0;

 err_mem_alloc:
	pr_err("Failed to mem_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		kfree(wlan_mem_array[j].mem_ptr);

	i = WLAN_SKB_BUF_NUM;

 err_skb_alloc:
	pr_err("Failed to skb_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		dev_kfree_skb(wlan_static_skb[j]);

	return -ENOMEM;
}

static int wlan_power_en(int onoff)
{
	pr_info("## %s: %s power\n", __func__, onoff?"enable":"disable");
	wake_lock_timeout(&wifi_wake_lock, msecs_to_jiffies(5 * 1000));
	if (onoff) {
		s3c_gpio_cfgpin(wl_power, S3C_GPIO_OUTPUT);
		gpio_set_value(wl_power, 1);
		msleep(200);
	} else {
		gpio_set_value(wl_power, 0);
		msleep(10);
	}
	return 0;
}

static int wlan_reset_en(int onoff)
{
	pr_info("### %s %d\n", __func__, onoff);
	gpio_set_value(wl_power, onoff ? 1 : 0);
	if (onoff)
		msleep(200);
	return 0;
}

extern void mmc_force_presence_change(struct platform_device *pdev, int val);

static int wlan_carddetect_en(int onoff)
{
	pr_info("### %s %d\n", __func__, onoff);
	if(onoff) {
		s3c_gpio_cfgpin(EXYNOS5410_GPC1(0), S3C_GPIO_SFN(2));
		s3c_gpio_cfgpin(EXYNOS5410_GPC1(1), S3C_GPIO_SFN(2));
		s3c_gpio_cfgpin(EXYNOS5410_GPC1(2), S3C_GPIO_SFN(2));
		s3c_gpio_cfgpin(EXYNOS5410_GPC1(3), S3C_GPIO_SFN(2));
		s3c_gpio_cfgpin(EXYNOS5410_GPC1(4), S3C_GPIO_SFN(2));
		s3c_gpio_cfgpin(EXYNOS5410_GPC1(5), S3C_GPIO_SFN(2));
		s3c_gpio_cfgpin(EXYNOS5410_GPC1(6), S3C_GPIO_SFN(2));

	} else {
		s3c_gpio_cfgpin(EXYNOS5410_GPC1(0), S3C_GPIO_OUTPUT);
		s3c_gpio_cfgpin(EXYNOS5410_GPC1(1), S3C_GPIO_INPUT);
		s3c_gpio_cfgpin(EXYNOS5410_GPC1(2), S3C_GPIO_INPUT);
		s3c_gpio_cfgpin(EXYNOS5410_GPC1(3), S3C_GPIO_INPUT);
		s3c_gpio_cfgpin(EXYNOS5410_GPC1(4), S3C_GPIO_INPUT);
		s3c_gpio_cfgpin(EXYNOS5410_GPC1(5), S3C_GPIO_INPUT);
		s3c_gpio_cfgpin(EXYNOS5410_GPC1(6), S3C_GPIO_INPUT);

		gpio_set_value(EXYNOS5410_GPC1(0), 0);
	}
	msleep(10);

	mmc_force_presence_change(&exynos5_device_dwmci1, onoff);
	msleep(400);
	return 0;
}

/* Customized Locale table : OPTIONAL feature */
#define WLC_CNTRY_BUF_SZ        4
struct cntry_locales_custom {
	char iso_abbrev[WLC_CNTRY_BUF_SZ];
	char custom_locale[WLC_CNTRY_BUF_SZ];
	int  custom_locale_rev;
};

static struct cntry_locales_custom brcm_wlan_translate_custom_table[] = {
	/* Table should be filled out based
 on custom platform regulatory requirement */
	{"",   "XY", 4},  /* universal */
	{"US", "US", 69}, /* input ISO "US" to : US regrev 69 */
	{"CA", "US", 69}, /* input ISO "CA" to : US regrev 69 */
	{"EU", "EU", 5},  /* European union countries */
	{"AT", "EU", 5},
	{"BE", "EU", 5},
	{"BG", "EU", 5},
	{"CY", "EU", 5},
	{"CZ", "EU", 5},
	{"DK", "EU", 5},
	{"EE", "EU", 5},
	{"FI", "EU", 5},
	{"FR", "EU", 5},
	{"DE", "EU", 5},
	{"GR", "EU", 5},
	{"HU", "EU", 5},
	{"IE", "EU", 5},
	{"IT", "EU", 5},
	{"LV", "EU", 5},
	{"LI", "EU", 5},
	{"LT", "EU", 5},
	{"LU", "EU", 5},
	{"MT", "EU", 5},
	{"NL", "EU", 5},
	{"PL", "EU", 5},
	{"PT", "EU", 5},
	{"RO", "EU", 5},
	{"SK", "EU", 5},
	{"SI", "EU", 5},
	{"ES", "EU", 5},
	{"SE", "EU", 5},
	{"GB", "EU", 5},  /* input ISO "GB" to : EU regrev 05 */
	{"IL", "IL", 0},
	{"CH", "CH", 0},
	{"TR", "TR", 0},
	{"NO", "NO", 0},
	{"KR", "XY", 3},
	{"AU", "XY", 3},
	{"CN", "XY", 3},  /* input ISO "CN" to : XY regrev 03 */
	{"TW", "XY", 3},
	{"AR", "XY", 3},
	{"MX", "XY", 3}
};

static void __maybe_unused *brcm_wlan_get_country_code(char *ccode)
{
	int size = ARRAY_SIZE(brcm_wlan_translate_custom_table);
	int i;

	if (!ccode)
		return NULL;

	for (i = 0; i < size; i++)
		if (strcmp(ccode,
		brcm_wlan_translate_custom_table[i].iso_abbrev) == 0)
			return &brcm_wlan_translate_custom_table[i];
	return &brcm_wlan_translate_custom_table[0];
}

extern int get_mac_form_device(unsigned char *buf);

static int brcm_wlan_get_mac_addr(unsigned char *buf)
{
	struct file *fp      = NULL;
	char macbuffer[18]   = {0};
	mm_segment_t oldfs    = {0};
	char *mac_file       = "/data/calibration/mac_addr";
	int ret = 0;
	int no_mac = 0;

	no_mac = !!get_mac_form_device(buf);

	fp = filp_open(mac_file, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		if(no_mac) {
			get_random_bytes(buf, 6);
			buf[0] = 0x38;
			buf[1] = 0xBC;
			buf[2] = 0x1A;
		}

		pr_info("%s: write file %s\n", __func__, mac_file);

		snprintf(macbuffer, sizeof(macbuffer),"%02X:%02X:%02X:%02X:%02X:%02X\n",
				buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

		fp = filp_open(mac_file, O_RDWR | O_CREAT, 0644);
		if (IS_ERR(fp)) {
			pr_err("%s:create file %s error(%ld)\n", __func__, mac_file, IS_ERR(fp));
		} else {
			oldfs = get_fs();
			set_fs(get_ds());

			if (fp->f_mode & FMODE_WRITE) {
				ret = fp->f_op->write(fp, (const char *)macbuffer,
						sizeof(macbuffer), &fp->f_pos);
				if (ret < 0)
					pr_err("%s:write file %s error(%d)\n", __func__, mac_file, ret);
			}
			set_fs(oldfs);
			filp_close(fp, NULL);
		}

	} else {
		if(no_mac) {
			ret = kernel_read(fp, 0, macbuffer, 18);
			if(ret <= 17) {
				pr_info("%s: read mac_info error, get random mac address\n", __func__);
				get_random_bytes(buf, 6);
			} else {
				macbuffer[17] = '\0';

				sscanf(macbuffer, "%02X:%02X:%02X:%02X:%02X:%02X",
						(unsigned int *)&(buf[0]), (unsigned int *)&(buf[1]),
						(unsigned int *)&(buf[2]), (unsigned int *)&(buf[3]),
						(unsigned int *)&(buf[4]), (unsigned int *)&(buf[5]));
			}
			if (fp)
				filp_close(fp, NULL);

			buf[0] = 0x38;
			buf[1] = 0xBC;
			buf[2] = 0x1A;
		}
	}

	pr_info("mac address mac=%.2x:%.2x:%.2x:%.2x:%.2x:%.2x\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

	return 0;
}

static struct wifi_platform_data wifi_pdata = {
	.set_power = wlan_power_en,
	.set_reset = wlan_reset_en,
	.set_carddetect = wlan_carddetect_en,
	.mem_prealloc = brcm_wlan_mem_prealloc,
	//.get_country_code = brcm_wlan_get_country_code,
	.get_mac_addr = brcm_wlan_get_mac_addr,
};


static struct resource m6x_wifi_resources[] = {
	[0] = {
		.name	= "bcmdhd_wlan_irq",
		.start	= IRQ_EINT(16),
		.end	= IRQ_EINT(16),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE | IORESOURCE_IRQ_SHAREABLE,
	},
};

static struct platform_device m6x_wifi = {
	.name		= "bcmdhd_wlan",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(m6x_wifi_resources),
	.resource	= m6x_wifi_resources,
	.dev		= {
		.platform_data = &wifi_pdata,
	},
};

static int __init m6x_wifi_init(void)
{
	int ret;

	wake_lock_init(&wifi_wake_lock, WAKE_LOCK_SUSPEND, "wifi_ctrl_wake_lock");

	wl_power 	= MEIZU_WL_REGON;

	ret = platform_device_register(&m6x_wifi);
	if (ret)
		goto err1;

	ret = brcm_init_wifi_mem();
	if (ret)
		goto err2;

	return 0;

err2:
	platform_device_unregister(&m6x_wifi);
err1:
	return ret;
}

device_initcall(m6x_wifi_init);
