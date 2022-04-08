/*!
 *****************************************************************************
 *
 *  Copyright © 2016-2022 CHASING INNOVATION . All Rights Reserved.
 *
 * \file      perf.c
 * \author    johnson@chasing-innovation.com
 * \version   1.0
 * \date      2022年4月3日星期日
 * \brief     性能统计
 *
 *----------------------------------------------------------------------------
 * \attention
 *
 *
 *****************************************************************************
 */

/*****************************************************************************
 change history: 
    1.date  : 2022年4月3日星期日
      author: johnson@chasing-innovation.com
      change: create file

*****************************************************************************/
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "system.h"
#include "perf.h"
#include "shell.h"
#include "usbcfg.h"
#include "task.h"
#include "scheduler.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

typedef struct perf_stru_count_s 
{
    const char *name;

    /* Everything below is in microseconds */
    uint32_t start;
    uint32_t min;
    uint32_t max;
    uint32_t count;
    uint32_t total;

    float avg;
    float m2;
} PERF_STRU_COUNT_T;

uint16_t loop_rate_hz;
uint16_t overtime_threshold_micros;
uint16_t loop_count;
uint32_t max_time; // in microseconds
uint32_t min_time; // in microseconds
uint64_t sigma_time;
uint64_t sigmasquared_time;
uint16_t long_running;
uint32_t last_check_us;
float filtered_loop_time;
bool ignore_loop;



uint16_t get_num_loops(void);
uint32_t get_max_time(void);
uint32_t get_min_time(void);
uint16_t get_num_long_running(void);
uint32_t get_avg_time(void);
uint32_t get_stddev_time(void);
float	 get_filtered_time(void);

//
//  high level performance monitoring
//
//  we measure the main loop time
//

// reset - reset all records of loop time to zero
void perf_reset(void)
{
    loop_count = 0;
    max_time = 0;
    min_time = 0;
    long_running = 0;
    sigma_time = 0;
    sigmasquared_time = 0;
}

// ignore_loop - ignore this loop from performance measurements (used to reduce false positive when arming)
void ignore_this_loop(void)
{
    ignore_loop = true;
}

// check_loop_time - check latest loop time vs min, max and overtime threshold
void perf_check_loop_time(uint32_t time_in_micros)
{
    loop_count++;

    // exit if this loop should be ignored
    if (ignore_loop) {
        ignore_loop = false;
        return;
    }

    if( time_in_micros > max_time) {
        max_time = time_in_micros;
    }
    if( min_time == 0 || time_in_micros < min_time) {
        min_time = time_in_micros;
    }
    if (time_in_micros > overtime_threshold_micros) {
        long_running++;
    }
    sigma_time += time_in_micros;
    sigmasquared_time += time_in_micros * time_in_micros;

    /* we keep a filtered loop time for use as G_Dt which is the
       predicted time for the next loop. We remove really excessive
       times from this calculation so as not to throw it off too far
       in case we get a single long loop

       Note that the time we use here is the time between calls to
       check_loop_time() not the time from loop start to loop
       end. This is because we are using the time for time between
       calls to controllers, which has nothing to do with cpu speed.
    */
    const uint32_t now = micros();
    const uint32_t loop_time_us = now - last_check_us;
    last_check_us = now;
    if (loop_time_us < overtime_threshold_micros + 10000UL) {
        filtered_loop_time = 0.99f * filtered_loop_time + 0.01f * loop_time_us * 1.0e-6f;
    }
}

// get_num_loops: return number of loops used for recording performance
uint16_t get_num_loops(void)
{
    return loop_count;
}

// get_max_time - return maximum loop time (in microseconds)
uint32_t get_max_time(void)
{
    return max_time;
}

// get_min_time - return minumum loop time (in microseconds)
uint32_t get_min_time(void)
{
    return min_time;
}

// get_num_long_running - get number of long running loops
uint16_t get_num_long_running(void)
{
    return long_running;
}

// get_avg_time - return average loop time (in microseconds)
uint32_t get_avg_time(void)
{
    return (sigma_time / loop_count);
}

// get_stddev_time - return stddev of average loop time (in us)
uint32_t get_stddev_time(void)
{
    return (uint32_t)sqrtf((float)((sigmasquared_time - (sigma_time*sigma_time)/loop_count) / loop_count));
}

// get_filtered_time - return low pass filtered loop time in seconds
float get_filtered_time(void)
{
    return filtered_loop_time;
}

void update_logging()
{
#if 0
    gcs().send_text(MAV_SEVERITY_WARNING,
                    "PERF: %u/%u [%lu:%lu] F=%uHz sd=%lu Ex=%lu",
                    (unsigned)get_num_long_running(),
                    (unsigned)get_num_loops(),
                    (unsigned long)get_max_time(),
                    (unsigned long)get_min_time(),
                    (unsigned)(0.5+(1.0f/get_filtered_time())),
                    (unsigned long)get_stddev_time(),
                    (unsigned long)AP::scheduler().get_extra_loop_us());
#endif
}

void perf_set_loop_rate(uint16_t rate_hz)
{
    // allow a 20% overrun before we consider a loop "slow":
    overtime_threshold_micros = 1000000/rate_hz * 1.2f;

    if (loop_rate_hz != rate_hz) {
        loop_rate_hz = rate_hz;
        filtered_loop_time = 1.0f / rate_hz;
    }
}

void *perf_counter_create(const char *name)
{
	PERF_STRU_COUNT_T *p;

	p = malloc(sizeof(PERF_STRU_COUNT_T));
	if (NULL == p) {
		return NULL;
	} else {
		memset(p, 0, sizeof(PERF_STRU_COUNT_T));
		p->name = name;
		return p;
	}
}

void perf_begin(void *p)
{
	PERF_STRU_COUNT_T *counterp = p;
	
	if (NULL == counterp) {
        debug(2, "counterp null.\r\n");
		return;
	}

	counterp->start = micros();
}

void perf_end(void *p)
{
	PERF_STRU_COUNT_T *counterp = p;
	
	if ((NULL == counterp) || (0 == counterp->start)) {
        debug(2, "0 == counterp->start.\r\n");
		return;
	}	
	
    uint32_t elapsed = (int32_t)(micros() - counterp->start);
    counterp->count++;
    counterp->total += elapsed;
    //debug(2, "start %u elapsed %u.\r\n", counterp->start, elapsed);
    if (counterp->min > elapsed) {
        counterp->min = elapsed;
    }

    if (counterp->max < elapsed) {
        counterp->max = elapsed;
    }

    /*
     * Maintain avg and variance of interval in nanoseconds
     * Knuth/Welford recursive avg and variance of update intervals (via Wikipedia)
     * Same implementation of PX4.
     */
    const double delta_intvl = elapsed - counterp->avg;
    counterp->avg += (delta_intvl / counterp->count);
    counterp->m2 += (delta_intvl * (elapsed - counterp->avg));
    counterp->start = 0;
	return;
}

void perf_counter_show(BaseSequentialStream *chp, TASK_STRU_T *task)
{
	PERF_STRU_COUNT_T *c;

	if (task && task->perf_counter_p) {
		c = task->perf_counter_p;
	} else {
	    chprintf(chp, "(no task or no perf counter)\r\n");
		return;
	}

	if (!c->count) {
	    chprintf(chp, "%-30s\t"
	            "(no events)\r\n", c->name);
	} else {
	    chprintf(chp, "%-30s\t"
                "start: %u\t"
                "total: %u\t"
	            "count: %u\t"
	            "min: %u\t"
	            "max: %u\t"
	            "avg: %f\t"
	            "stddev: %f\r\n",
	            c->name, c->start, c->total, c->count, c->min, c->max, c->avg, sqrt(c->m2));
	}

	return;
   
}

void perf_counters_show(BaseSequentialStream *chp)
{
	TASK_STRU_T *task;
	
	for(uint8_t i = 0; i < task_get_tasksize(); i++) {
		task = task_get_tasks(i);
		perf_counter_show(chp, task);
	}
}

void perf_cmd(BaseSequentialStream *chp, int argc, char *argv[])
{
	int32_t cmd_type;
	
	if (argc != 1) {
		shellUsage(chp, "perf [sche | task]");
		return;
	} else {
		if (strcmp(argv[0], "sche") == 0) {
			cmd_type = 1;
		} else if (strcmp(argv[0], "task") == 0){
			cmd_type = 2;
		} else {
			cmd_type = -1;
			shellUsage(chp, "perf [sche | task]");
			return;
		}
	}

	switch (cmd_type) {
		case 1:
			chprintf(chp, "PERF: %u/%u [%lu:%lu] F=%uHz sd=%lu Ex=%lu",
						(unsigned)get_num_long_running(),
						(unsigned)get_num_loops(),
						(unsigned long)get_max_time(),
						(unsigned long)get_min_time(),
						(unsigned)(0.5+(1.0f/get_filtered_time())),
						(unsigned long)get_stddev_time(),
						(unsigned long)scheduler_get_extra_loop_us());
			break;
		case 2:
			perf_counters_show(chp);
			break;
		default:
			break;
	}
	
	return;
}


