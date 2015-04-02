#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/time.h>

#include "fimc-is-time.h"

#ifdef MEASURE_TIME
#ifdef INTERNAL_TIME

#define INSTANCES 3

static u32 time_count[INSTANCES];
static u32 time1_min[INSTANCES];
static u32 time1_max[INSTANCES];
static u32 time1_avg[INSTANCES];
static u32 time2_min[INSTANCES];
static u32 time2_max[INSTANCES];
static u32 time2_avg[INSTANCES];
static u32 time3_min[INSTANCES];
static u32 time3_max[INSTANCES];
static u32 time3_avg[INSTANCES];
static u32 time4_cur[INSTANCES];
static u32 time4_old[INSTANCES];
static u32 time4_avg[INSTANCES];

void measure_init(u32 instance)
{
	time_count[instance] = 0;
	time1_min[instance] = 0;
	time1_max[instance] = 0;
	time1_avg[instance] = 0;
	time2_min[instance] = 0;
	time2_max[instance] = 0;
	time2_avg[instance] = 0;
	time3_min[instance] = 0;
	time3_max[instance] = 0;
	time3_avg[instance] = 0;
	time4_cur[instance] = 0;
	time4_old[instance] = 0;
	time4_avg[instance] = 0;
}

void measure_internal_time(
	u32 instance,
	struct timeval *time_queued,
	struct timeval *time_shot,
	struct timeval *time_shotdone,
	struct timeval *time_dequeued)
{
	u32 temp1, temp2, temp3;

	if (!((1<<instance) && INSTANCE_MASK))
		return;

	temp1 = (time_shot->tv_sec - time_queued->tv_sec)*1000000 +
		(time_shot->tv_usec - time_queued->tv_usec);
	temp2 = (time_shotdone->tv_sec - time_shot->tv_sec)*1000000 +
		(time_shotdone->tv_usec - time_shot->tv_usec);
	temp3 = (time_dequeued->tv_sec - time_shotdone->tv_sec)*1000000 +
		(time_dequeued->tv_usec - time_shotdone->tv_usec);

	if (!time_count) {
		time1_min[instance] = temp1;
		time1_max[instance] = temp1;
		time2_min[instance] = temp2;
		time2_max[instance] = temp2;
		time3_min[instance] = temp3;
		time3_max[instance] = temp3;
	} else {
		if (time1_min[instance] > temp1)
			time1_min[instance] = temp1;

		if (time1_max[instance] < temp1)
			time1_max[instance] = temp1;

		if (time2_min[instance] > temp2)
			time2_min[instance] = temp2;

		if (time2_max[instance] < temp2)
			time2_max[instance] = temp2;

		if (time3_min[instance] > temp3)
			time3_min[instance] = temp3;

		if (time3_max[instance] < temp3)
			time3_max[instance] = temp3;
	}

	time1_avg[instance] += temp1;
	time2_avg[instance] += temp2;
	time3_avg[instance] += temp3;

	time4_cur[instance] = time_queued->tv_sec*1000000 + time_queued->tv_usec;
	time4_avg[instance] += (time4_cur[instance] - time4_old[instance]);
	time4_old[instance] = time4_cur[instance];

	time_count[instance]++;

	if (time_count[instance] % 33)
		return;

	pr_info("I%d t1(%d,%d,%d), t2(%d,%d,%d), t3(%d,%d,%d) : %d(%dfps)",
		instance,
		temp1, time1_max[instance], time1_avg[instance]/time_count[instance],
		temp2, time2_max[instance], time2_avg[instance]/time_count[instance],
		temp3, time3_max[instance], time3_avg[instance]/time_count[instance],
		time4_avg[instance]/33, 33000000/time4_avg[instance]);

	time4_avg[instance] = 0;
}

#endif
#endif
