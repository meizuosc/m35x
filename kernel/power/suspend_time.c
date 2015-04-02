/*
 * debugfs file to track time spent in suspend
 *
 * Copyright (c) 2011, Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#define pr_fmt(fmt) "SUSPEND_TIME: " fmt

#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/seq_file.h>
#include <linux/syscore_ops.h>
#include <linux/time.h>
#include <linux/rtc.h>

static bool suspend_sucess;
static struct rtc_time suspend_time_before;
static unsigned long long sleep_time_bins[32];
static unsigned long long total_suspend;
static struct timespec total_sleep_time = {0, 0};

#ifdef CONFIG_DEBUG_FS
static int suspend_time_debug_show(struct seq_file *s, void *data)
{
	int bin;
	struct timespec boottime;

	seq_printf(s, "time (secs)  count\n");
	seq_printf(s, "------------------\n");
	for (bin = 0; bin < 32; bin++) {
		if (sleep_time_bins[bin] == 0)
			continue;
		seq_printf(s, "%4d - %4d %8llu\n",
			bin ? 1 << (bin - 1) : 0, 1 << bin,
				sleep_time_bins[bin]);
	}
	get_monotonic_boottime(&boottime);
	seq_printf(s, "total: %lu.%03lu/%lu.%03lu (1/%lu) seconds, %llu times\n", total_sleep_time.tv_sec,
		total_sleep_time.tv_nsec / NSEC_PER_MSEC,
		boottime.tv_sec, boottime.tv_nsec / NSEC_PER_MSEC,
		total_sleep_time.tv_sec ? boottime.tv_sec / total_sleep_time.tv_sec : boottime.tv_sec,
		total_suspend);
	return 0;
}

static int suspend_time_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, suspend_time_debug_show, NULL);
}

static const struct file_operations suspend_time_debug_fops = {
	.open		= suspend_time_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init suspend_time_debug_init(void)
{
	struct dentry *d;

	d = debugfs_create_file("suspend_time", 0755, NULL, NULL,
		&suspend_time_debug_fops);
	if (!d) {
		pr_err("Failed to create suspend_time debug file\n");
		return -ENOMEM;
	}

	return 0;
}

late_initcall(suspend_time_debug_init);
#endif

static int suspend_time_syscore_suspend(void)
{
	suspend_sucess = 1;

	return 0;
}

static void suspend_time_syscore_resume(void)
{
}

static struct syscore_ops suspend_time_syscore_ops = {
	.suspend = suspend_time_syscore_suspend,
	.resume = suspend_time_syscore_resume,
};

int suspend_time_suspend(struct rtc_time before)
{
	suspend_time_before = before;

	return 0;
}

void suspend_time_resume(struct rtc_time suspend_time_after)
{
	struct rtc_time tm;
	struct timespec before, after, boottime;

	if (!suspend_sucess)
		return;
	else
		suspend_sucess = 0;


	tm = suspend_time_before;
	pr_debug("Suspend @ %d-%02d-%02d %02d:%02d:%02d Shanghai\n",
		tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		(tm.tm_hour + 8) % 24, tm.tm_min, tm.tm_sec);
	tm = suspend_time_after;
	pr_debug("Resume @ %d-%02d-%02d %02d:%02d:%02d Shanghai\n",
		tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		(tm.tm_hour + 8) % 24, tm.tm_min, tm.tm_sec);

	before.tv_nsec = NSEC_PER_SEC >> 1;
	after.tv_nsec = NSEC_PER_SEC >> 1;
	rtc_tm_to_time(&suspend_time_before, &before.tv_sec);
	rtc_tm_to_time(&suspend_time_after, &after.tv_sec);

	after = timespec_sub(after, before);

	if ((long)after.tv_sec < 0)
		after.tv_sec = 0;

	sleep_time_bins[fls(after.tv_sec)]++;
	total_sleep_time = timespec_add(total_sleep_time, after);
	total_suspend++;
	get_monotonic_boottime(&boottime);

	pr_debug("Suspend %lu.%03lu seconds, total %lu.%03lu/%lu.%03lu (1/%lu) seconds, total %llu times\n",
		after.tv_sec, after.tv_nsec / NSEC_PER_MSEC,
		total_sleep_time.tv_sec, total_sleep_time.tv_nsec / NSEC_PER_MSEC,
		boottime.tv_sec, boottime.tv_nsec / NSEC_PER_MSEC,
		total_sleep_time.tv_sec ? boottime.tv_sec / total_sleep_time.tv_sec : boottime.tv_sec,
		total_suspend);
}

static int suspend_time_syscore_init(void)
{
	register_syscore_ops(&suspend_time_syscore_ops);

	return 0;
}

static void suspend_time_syscore_exit(void)
{
	unregister_syscore_ops(&suspend_time_syscore_ops);
}
module_init(suspend_time_syscore_init);
module_exit(suspend_time_syscore_exit);
