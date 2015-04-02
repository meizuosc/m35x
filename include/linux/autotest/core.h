#ifndef _LINUX_AUTOTEST_H
#define _LINUX_AUTOTEST_H

#include <linux/kthread.h>
#include <linux/random.h>

/* From 0 to max, max <= 255 */
extern int get_random(unsigned int max);

/* From (time) to (time * max) */
static inline int get_random_val(unsigned int max, unsigned int val)
{
	return get_random(max) * val;
}

#define get_random_secs(max, val) get_random_val(max, val)

static inline int get_random_msecs(unsigned int max, unsigned int secs)
{
	return get_random_secs(max, secs) * MSEC_PER_SEC;
}

#ifdef CONFIG_AUTOTEST_SUSPEND
extern void start_suspend_thread(void);
#else
#define start_suspend_thread()	do { } while (0)
#endif

#endif /* _LINUX_AUTOTEST_H */
