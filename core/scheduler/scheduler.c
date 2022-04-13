/*!
 *****************************************************************************
 *
 *  Copyright © 2016-2022 CHASING INNOVATION . All Rights Reserved.
 *
 * \file      scheduler.c
 * \author    johnson@chasing-innovation.com
 * \version   1.0
 * \date      2022年4月3日星期日
 * \brief     时间片调度器
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
#include "scheduler.h"
#ifndef UNIT_TEST
#include "usbcfg.h"
#endif
#include "imu.h"
#include "task.h"
#ifndef UNIT_TEST
#include "mpu6000.h"
#include "qmc5883l.h"
#endif
#include "ahrs.h"
#include <math.h>
#include <float.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define SCHE_MONITOR_PRIORITY      183
#define SCHE_MAIN_PRIORITY_BOOST   182
#define SCHE_MAIN_PRIORITY         180

#define SCHE_DEFAULT_LOOP_RATE     400

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

void sche_cmd(BaseSequentialStream *chp, int argc, char *argv[]);

#ifndef UNIT_TEST
static const ShellCommand commands[] = {
	{"scher", sche_cmd},
	{"perf", perf_cmd},
    {"mpu6k", mpu6k_cmd},
    {"qmc5883l", qmc5883l_cmd},
    {"imu", imu_cmd},   
	{NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&SDU1,
  commands
};
#endif

 
static thread_t* daemon_task;
static uint16_t _loop_rate_hz;
static bool     _called_boost;
static bool     _priority_boosted;

// loop rate in Hz as set at startup
uint16_t _active_loop_rate_hz;

// calculated loop period in usec
uint16_t _loop_period_us;

// calculated loop period in seconds
float _loop_period_s;

// total number of tasks in _tasks and _common_tasks list
uint8_t _num_tasks;

// number of tasks in _tasks list
uint8_t _num_unshared_tasks;

// number of 'ticks' that have passed (number of times that
// tick() has been called
uint16_t _tick_counter;

// number of microseconds allowed for the current task
uint32_t _task_time_allowed;

// the time in microseconds when the task started
uint32_t _task_time_started;

// number of spare microseconds accumulated
uint32_t _spare_micros;

// number of ticks that _spare_micros is counted over
uint8_t _spare_ticks;

// start of loop timing
uint32_t _loop_timer_start_us;

// time of last loop in seconds
float _last_loop_time_s;

// bitmask bit which indicates if we should log PERF message
uint32_t _log_performance_bit;

// maximum task slowdown compared to desired task rate before we
// start giving extra time per loop
const uint8_t max_task_slowdown = 4;

// counters to handle dynamically adjusting extra loop time to
// cope with low CPU conditions
uint32_t task_not_achieved;
uint32_t task_all_achieved;

// extra time available for each loop - used to dynamically adjust
// the loop rate in case we are well over budget
uint32_t extra_loop_us;

uint32_t _debug;
/*
 *    function declare
 */
static thread_t* _get_main_thread(void);
static bool in_main_thread(void);
static uint32_t get_loop_period_us(void);

/*
 *    function imp
 */
static bool in_main_thread(void) 
{ 
	return (_get_main_thread() == chThdGetSelfX()); 
}

// one tick has passed
void tick(void)
{
    _tick_counter++;
}

static void fast_loop(void)
{
	imu_update();

    ahrs_update();
	return;
}

/*
  run one tick
  this will run as many scheduler tasks as we can in the specified time
 */
static void run(uint32_t time_available)
{
	TASK_STRU_T *task;

    uint32_t run_started_usec = micros();
    uint32_t now = run_started_usec;

    for (uint8_t i = 0; i < task_get_tasksize(); i++) {
        task = task_get_tasks(i);

		if (_debug > 1 && task) {
			if (NULL == task->perf_counter_p) {
				task->perf_counter_p = perf_counter_create(task->name);
                debug(2, "task:%s get counter @0x%x.\r\n", task->name, task->perf_counter_p);
			}
		}
		
        uint32_t dt = (int16_t)(_tick_counter - task->_last_run);
        uint32_t interval_ticks = _loop_rate_hz / task->rate_hz;
        if (interval_ticks < 1) {
            interval_ticks = 1;
        }
        if (dt < interval_ticks) {
            // this task is not yet scheduled to run again
            continue;
        }
        // this task is due to run. Do we have enough time to run it?
        _task_time_allowed = task->max_time_micros;

        if (dt >= interval_ticks*2) {
            // we've slipped a whole run of this task!
            debug(2, "Scheduler slip task[%u-%s] (%u/%u/%u/%u/%u)\n",
                  (unsigned)i,
                  task->name,
                  (unsigned)dt,
                  (unsigned)interval_ticks,
                  (unsigned)_task_time_allowed,
                  _tick_counter,
                  task->_last_run);
        }

        if (dt >= interval_ticks*max_task_slowdown) {
            // we are going beyond the maximum slowdown factor for a
            // task. This will trigger increasing the time budget
            task_not_achieved++;
        }

        if (_task_time_allowed > time_available) {
            // not enough time to run this task.  Continue loop -
            // maybe another task will fit into time remaining
            continue;
        }

        // run it
        _task_time_started = now;

        if (_debug > 1 && task && task->perf_counter_p) {
            perf_begin(task->perf_counter_p);
        }

        task->function();

        if (_debug > 1 && task && task->perf_counter_p) {
            perf_end(task->perf_counter_p);
        }

        // record the tick counter when we ran. This drives
        // when we next run the event
        task->_last_run = _tick_counter;

        // work out how long the event actually took
        now = micros();
        uint32_t time_taken = now - _task_time_started;

        if (time_taken > _task_time_allowed) {
            // the event overran!
            debug(3, "Scheduler overrun task[%u-%s] (%u/%u)\n",
                  (unsigned)i,
                  task->name,
                  (unsigned)time_taken,
                  (unsigned)_task_time_allowed);
        }
        if (time_taken >= time_available) {
            time_available = 0;
            break;
        }
        time_available -= time_taken;
    }

    // update number of spare microseconds
    _spare_micros += time_available;

    _spare_ticks++;
    if (_spare_ticks == 32) {
        _spare_ticks /= 2;
        _spare_micros /= 2;
    }
}

bool is_equalf(float a)
{
    return fabsf(a) < FLT_EPSILON;
}


uint16_t get_loop_rate_hz(void) 
{
	if (_active_loop_rate_hz == 0) {
		_active_loop_rate_hz = _loop_rate_hz;
	}
	return _active_loop_rate_hz;
}

static uint32_t get_loop_period_us(void) 
{
	if (_loop_period_us == 0) {
		_loop_period_us = 1000000UL / _loop_rate_hz;
	}
	return _loop_period_us;
}

float get_loop_period_s(void) 
{
	if (is_equalf(_loop_period_s)) {
		_loop_period_s = 1.0f / _loop_rate_hz;
	}
	return _loop_period_s;
}

static void set_high_priority(void)
{
    chThdSetPriority(SCHE_MAIN_PRIORITY_BOOST);
}


static thread_t* _get_main_thread(void)
{
    return daemon_task;
}

void scheduler_set_main_thread(thread_t* thd)
{
    daemon_task = thd;
	chThdSetPriority(SCHE_MAIN_PRIORITY);
}

bool scheduler_check_called_boost(void)
{
    if (!_called_boost) {
        return false;
    }
    _called_boost = false;
    return true;
}

 void scheduler_delay_microseconds(uint16_t usec)
 {
	 if (usec == 0) { //chibios faults with 0us sleep
		 return;
	 }
	 uint32_t ticks;
	 ticks = chTimeUS2I(usec);
	 if (ticks == 0) {
		 // calling with ticks == 0 causes a hard fault on ChibiOS
		 ticks = 1;
	 }
	 chThdSleep(ticks); //Suspends Thread for desired microseconds
 }

 void scheduler_delay(uint16_t ms)
 {
     uint64_t start = micros64();
 
     while ((micros64() - start)/1000 < ms) {
         scheduler_delay_microseconds(1000);
     }
 }

 /*
	a variant of delay_microseconds that boosts priority to
	APM_MAIN_PRIORITY_BOOST for APM_MAIN_PRIORITY_BOOST_USEC
	microseconds when the time completes. This significantly improves
	the regularity of timing of the main loop
 */
void scheduler_delay_microseconds_boost(uint16_t usec)
{
    if (!_priority_boosted && in_main_thread()) {
        set_high_priority();
        _priority_boosted = true;
        _called_boost = true;
    }
    scheduler_delay_microseconds(usec); //Suspends Thread for desired microseconds
}

/*
  return the main thread to normal priority
 */
void scheduler_boost_end(void)
{
    if (in_main_thread() && _priority_boosted) {
        _priority_boosted = false;
        chThdSetPriority(SCHE_MAIN_PRIORITY);
    }
}

void scheduler_loop(void)
{
	// wait for an INS sample
	imu_wait_for_sample();

	const uint32_t sample_time_us = micros();
	
	if (_loop_timer_start_us == 0) {
		_loop_timer_start_us = sample_time_us;
		_last_loop_time_s = get_loop_period_s();
	} else {
		_last_loop_time_s = (sample_time_us - _loop_timer_start_us) * 1.0e-6;
	}

	// Execute the fast loop
	// ---------------------
	fast_loop();

#ifdef BOARD_SITL
	{
		/*
		  for testing low CPU conditions we can add an optional delay in SITL
		*/
		auto *sitl = AP::sitl();
		uint32_t loop_delay_us = sitl->loop_delay.get();
		hal.scheduler->delay_microseconds(loop_delay_us);
	}
#endif

	// tell the scheduler one tick has passed
	tick();

	// run all the tasks that are due to run. Note that we only
	// have to call this once per loop, as the tasks are scheduled
	// in multiples of the main loop tick. So if they don't run on
	// the first call to the scheduler they won't run on a later
	// call until scheduler.tick() is called again
	const uint32_t loop_us = get_loop_period_us();
	uint32_t now = micros();
	uint32_t time_available = 0;
	if (now - sample_time_us < loop_us) {
		// get remaining time available for this loop
		time_available = loop_us - (now - sample_time_us);
	}

	// add in extra loop time determined by not achieving scheduler tasks
	time_available += extra_loop_us;

	// run the tasks
	run(time_available);

#ifdef BOARD_SITL
	// move result of AP_HAL::micros() forward:
	scheduler_delay_microseconds(1);
#endif

	if (task_not_achieved > 0) {
		// add some extra time to the budget
		extra_loop_us = MIN(extra_loop_us+100U, 5000U);
		task_not_achieved = 0;
		task_all_achieved = 0;
	} else if (extra_loop_us > 0) {
		task_all_achieved++;
		if (task_all_achieved > 50) {
			// we have gone through 50 loops without a task taking too
			// long. CPU pressure has eased, so drop the extra time we're
			// giving each loop
			task_all_achieved = 0;
			// we are achieving all tasks, slowly lower the extra loop time
			extra_loop_us = MAX((int32_t)0, (int32_t)(extra_loop_us-50U));
		}
	}

	// check loop time
	perf_check_loop_time(sample_time_us - _loop_timer_start_us);
		
	_loop_timer_start_us = sample_time_us;

}

void scheduler_shellThdCreate(void)
{
#ifndef UNIT_TEST
	static bool isCreate = false;
    if ((!isCreate) && (SDU1.config->usbp->state == USB_ACTIVE)) {
		thread_t *shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
		                                      "shell", SCHE_SHELL_PRIORITY,
		                                      shellThread, (void *)&shell_cfg1);
		if(shelltp != NULL) {
			isCreate = true;
		} else {
			debug(0, "shell thread create failed!\r\n");
		}
    }

	return;
#endif
}

void scheduler_init(void)
{
	_loop_rate_hz = SCHE_DEFAULT_LOOP_RATE;
	perf_set_loop_rate(get_loop_rate_hz());
	perf_reset();
	imu_init(get_loop_rate_hz());

	return;
}

uint32_t scheduler_get_extra_loop_us(void)
{
	return extra_loop_us;
}

void sche_cmd(BaseSequentialStream *chp, int argc, char *argv[])
{
	int v;
	(void)chp;

    if (argc != 2) {
        shellUsage(chp, "scher debug [level]");
        return;
    }

	if (strcmp(argv[0], "debug") == 0)
	{
		v = atoi(argv[1]);
		_debug = v;
	} else {
        printk("unknown cmd.\r\n");
    }

	return;
}


