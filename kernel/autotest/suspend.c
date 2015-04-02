/*
 * Suspend test case for Built-in autotest framework
 *
 * Copyright (C) 2012 Meizu Co,. Ltd.
 *
 * Author: Wu Zhangjin <falcon@meizu.com> or <wuzhangjin@gmail.com>
 * Update: Sun Apr 29 13:14:29 CST 2012
 */

#define pr_fmt(fmt) "AUTOTEST_SUSPEND: " fmt

#include <linux/autotest/core.h>
#include <linux/wakelock.h>
#include <linux/syscalls.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/rtc.h>

extern int setup_test_suspend(unsigned long suspend_time);
struct task_struct *suspend_task;

unsigned int  __read_mostly sysctl_suspend_test = 0;
unsigned long __read_mostly sysctl_suspend_count = 0;
unsigned long __read_mostly sysctl_suspend_time_secs = CONFIG_AUTOTEST_SUSPEND_TIME;
unsigned long __read_mostly sysctl_suspend_cycle_secs = CONFIG_AUTOTEST_SUSPEND_CYCLE;

int proc_dosuspend_test(struct ctl_table *table, int write,
				  void __user *buffer,
				  size_t *lenp, loff_t *ppos)
{
	int ret;

	ret = proc_doulongvec_minmax(table, write, buffer, lenp, ppos);

	if (ret || !write)
		goto out;

	if (sysctl_suspend_test)
		setup_test_suspend(sysctl_suspend_time_secs);
	else
		setup_test_suspend(0);

	wake_up_process(suspend_task);

 out:
	return ret;
}

static unsigned long timeout_jiffies(unsigned long timeout)
{
	/* timeout of 0 will disable the watchdog */
	return timeout ? timeout * HZ : MAX_SCHEDULE_TIMEOUT;
}

static int suspend_thread(void *data)
{
	unsigned int suspend_cycle, suspend_time, suspend_count = 0;

	pr_info("%s: Enter into suspend_thread\n", __func__);

	while (1) {
		if (sysctl_suspend_test) {
			/* The setup_test_suspend() function accept secs as unit */
			suspend_time = get_random_secs(1, sysctl_suspend_time_secs) + 5;
			pr_info("%s: Allow go to suspend, suspend time = %u s\n", __func__, suspend_time);
			setup_test_suspend(suspend_time);

			suspend_cycle = get_random_secs(1, sysctl_suspend_cycle_secs) + 6;
			pr_info("%s: Sleep %d seconds for suspend\n", __func__, suspend_cycle);
		} else {
			suspend_cycle = 0;
			suspend_count = 0;
		}

		/* allow suspend for CONFIG_AUTOTEST_SUSPEND_CYCLE / 2 seconds */
		while (schedule_timeout_interruptible(timeout_jiffies(suspend_cycle)))
			suspend_cycle = sysctl_suspend_test ? get_random_secs(1, sysctl_suspend_cycle_secs) + 1 : 0;

		suspend_count++;
		pr_info("Suspend tested %d times\n", suspend_count);
		/* Stop suspend if suspend count */
		if (sysctl_suspend_count != 0 && suspend_count >= sysctl_suspend_count) {
			pr_info("Suspend test reaches %ld times, stop it.\n", sysctl_suspend_count);
			/* Wake it up immediatly for late operations */
			setup_test_suspend(1);
			msleep_interruptible(3);
			setup_test_suspend(0);
			sysctl_suspend_test = 0;
		}
	}

	return 0;
}

void start_suspend_thread(void)
{
	pr_info("%s: Start suspend test thread\n", __func__);
	suspend_task = kthread_run(suspend_thread, NULL, "suspendtest/daemon");
	if (IS_ERR(suspend_task))
		pr_err("%s: Fail to create suspend_thread\n", __func__);
}
