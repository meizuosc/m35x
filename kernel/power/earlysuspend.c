/* kernel/power/earlysuspend.c
 *
 * Copyright (C) 2005-2008 Google, Inc.
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

#include <linux/earlysuspend.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/rtc.h>
#include <linux/syscalls.h> /* sys_sync */
#include <linux/workqueue.h>
#include <linux/syscore_ops.h>

#include "power.h"

enum {
	DEBUG_USER_STATE = 1U << 0,
	DEBUG_SUSPEND = 1U << 2,
	DEBUG_VERBOSE = 1U << 3,
};

#ifndef CONFIG_PM_DEBUG_EARLY_SUSPEND
static int debug_mask = DEBUG_USER_STATE;
#else
static int debug_mask = DEBUG_USER_STATE|DEBUG_SUSPEND|DEBUG_VERBOSE;
#endif

module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

static DEFINE_MUTEX(early_suspend_lock);
static LIST_HEAD(early_suspend_handlers);
static void early_suspend(struct work_struct *work);
static void late_resume(struct work_struct *work);
static DECLARE_WORK(early_suspend_work, early_suspend);
static DECLARE_WORK(late_resume_work, late_resume);
static DEFINE_SPINLOCK(state_lock);
enum {
	SUSPEND_REQUESTED = 0x1,
	SUSPENDED = 0x2,
	SUSPEND_REQUESTED_AND_SUSPENDED = SUSPEND_REQUESTED | SUSPENDED,
};
static int state;

/*new*/
static suspend_state_t requested_early_suspend_state = PM_SUSPEND_ON;
static struct workqueue_struct *early_suspend_wq;
static struct wakeup_source *early_suspend_ws;

void register_early_suspend(struct early_suspend *handler)
{
	struct list_head *pos;

	mutex_lock(&early_suspend_lock);
	list_for_each(pos, &early_suspend_handlers) {
		struct early_suspend *e;
		e = list_entry(pos, struct early_suspend, link);
		if (e->level > handler->level)
			break;
	}
	list_add_tail(&handler->link, pos);
	if ((state & SUSPENDED) && handler->suspend)
		handler->suspend(handler);
	mutex_unlock(&early_suspend_lock);
}
EXPORT_SYMBOL(register_early_suspend);

void unregister_early_suspend(struct early_suspend *handler)
{
	mutex_lock(&early_suspend_lock);
	list_del(&handler->link);
	mutex_unlock(&early_suspend_lock);
}
EXPORT_SYMBOL(unregister_early_suspend);

static void early_suspend(struct work_struct *work)
{
	struct early_suspend *pos;
	unsigned long irqflags;
	int abort = 0;

	mutex_lock(&early_suspend_lock);
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPEND_REQUESTED)
		state |= SUSPENDED;
	else
		abort = 1;
	spin_unlock_irqrestore(&state_lock, irqflags);

	if (abort) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("%s: abort, state %d\n", __func__, state);
		mutex_unlock(&early_suspend_lock);
		return;
	}

	if (debug_mask & DEBUG_SUSPEND)
		pr_info("%s: call handlers\n", __func__);
	list_for_each_entry(pos, &early_suspend_handlers, link) {
		if (pos->suspend != NULL) {
			if (debug_mask & DEBUG_VERBOSE) {
				pr_info("%s: calling %pf\n",
					__func__, pos->suspend);
			}
			pos->suspend(pos);
		}
	}
	mutex_unlock(&early_suspend_lock);

#ifndef CONFIG_PM_DEBUG_DISABLE_SUSPEND
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPEND_REQUESTED_AND_SUSPENDED)
		__pm_relax(early_suspend_ws);
	spin_unlock_irqrestore(&state_lock, irqflags);
#endif
}

static void late_resume(struct work_struct *work)
{
	struct early_suspend *pos;
	unsigned long irqflags;
	int abort = 0;

	mutex_lock(&early_suspend_lock);
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPENDED)
		state &= ~SUSPENDED;
	else
		abort = 1;
	spin_unlock_irqrestore(&state_lock, irqflags);

	if (abort) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("%s: abort, state %d\n", __func__, state);
		goto abort;
	}

	if (debug_mask & DEBUG_SUSPEND)
		pr_info("%s: call handlers\n", __func__);

	list_for_each_entry_reverse(pos, &early_suspend_handlers, link)
		if (pos->resume != NULL) {
			if (debug_mask & DEBUG_VERBOSE) {
				pr_info("%s: calling %pf\n",
						__func__, pos->resume);
			}
			pos->resume(pos);
		}

	if (debug_mask & DEBUG_SUSPEND)
		pr_info("%s: done\n", __func__);
abort:
	mutex_unlock(&early_suspend_lock);
}

void pm_request_early_suspend_state(suspend_state_t new_state)
{
	unsigned long irqflags;
	int old_sleep;

	spin_lock_irqsave(&state_lock, irqflags);
	old_sleep = state & SUSPEND_REQUESTED;
	if (debug_mask & DEBUG_USER_STATE) {
		struct timespec ts;
		struct rtc_time tm;
		getnstimeofday(&ts);
		rtc_time_to_tm(ts.tv_sec, &tm);
		pr_info("pm_request_early_suspend_state: %s (%d->%d) "
				" at %lld (%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n",
				new_state != PM_SUSPEND_ON ? "sleep" : "wakeup",
				requested_early_suspend_state, new_state,
				ktime_to_ns(ktime_get()),
				tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
				tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
	}
	if (!old_sleep && new_state != PM_SUSPEND_ON) {
		state |= SUSPEND_REQUESTED;
		queue_work(early_suspend_wq, &early_suspend_work);
	} else if (old_sleep && new_state == PM_SUSPEND_ON) {
		state &= ~SUSPEND_REQUESTED;
		__pm_stay_awake(early_suspend_ws);
		queue_work(early_suspend_wq, &late_resume_work);
	}
	requested_early_suspend_state = new_state;
	spin_unlock_irqrestore(&state_lock, irqflags);
}

/*Add early_suspend ws*/
static int __init pm_early_suspend_init(void)
{
	early_suspend_wq = create_singlethread_workqueue("early_suspend");
	if (early_suspend_wq == NULL)
		return -ENOMEM;
	
	early_suspend_ws = wakeup_source_register("early_suspend");

	if (!early_suspend_ws) {
		pr_err("wakeup_source_register early_suspend faild!\n");
		goto err_ws;
	}
	
	__pm_stay_awake(early_suspend_ws);
	
	return 0;

err_ws:
	destroy_workqueue(early_suspend_wq);
	return -ENOMEM;
}

static void __exit pm_early_suspend_exit(void)
{
	destroy_workqueue(early_suspend_wq);

	if (early_suspend_ws)
		wakeup_source_unregister(early_suspend_ws);
}

core_initcall(pm_early_suspend_init);
module_exit(pm_early_suspend_exit);
