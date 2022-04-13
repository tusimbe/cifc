/*!
 *****************************************************************************
 *
 *  Copyright © 2016-2022 CHASING INNOVATION . All Rights Reserved.
 *
 * \file      task.c
 * \author    johnson@chasing-innovation.com
 * \version   1.0
 * \date      2022年4月3日星期日
 * \brief     任务管理
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
#ifndef UNIT_TEST
#include "usbcfg.h"
#endif
#include "task.h"

#define TASK_NAME_INITIALIZER(_name) .name = #_name

/*
  useful macro for creating scheduler task table
 */
#define TASK_ENTRY(func, _rate_hz, _max_time_micros) { \
    .function = func,                     \
    TASK_NAME_INITIALIZER(func),          \
    .rate_hz = _rate_hz,                  \
    .max_time_micros = _max_time_micros,  \
    ._last_run = 0,                       \
    .perf_counter_p = 0                   \
}

void test_func0(void)
{
#ifndef UNIT_TEST
    static bool flag = false;

    if (flag) {
        palClearPad(GPIOG, GPIOG_LED_BLUE);
    }
    else {
        palSetPad(GPIOG, GPIOG_LED_BLUE);
    }
    
    flag = !flag;
#endif
}

void test_func1(void)
{
#ifndef UNIT_TEST

    static bool flag = false;

    if (flag) {
        palClearPad(GPIOF, GPIOF_LED_GREEN);
    }
    else {
        palSetPad(GPIOF, GPIOF_LED_GREEN);
    }
    
    flag = !flag;
#endif
}

void test_func2(void)
{
#ifndef UNIT_TEST
    static bool flag = false;

    if (flag) {
        palClearPad(GPIOG, GPIOG_LED_AMBER);
    }
    else {
        palSetPad(GPIOG, GPIOG_LED_AMBER);
    }
    
    flag = !flag;
#endif

}


void test_func3(void)
{
#ifndef UNIT_TEST

    static bool flag = false;

    if (flag) {
        palClearPad(GPIOG, GPIOG_LED_SAFETY);
    }
    else {
        palSetPad(GPIOG, GPIOG_LED_SAFETY);
    }
    
    flag = !flag;
#endif

}


// progmem list of tasks to run
TASK_STRU_T tasks[] = {
    TASK_ENTRY(test_func0,         20,     75),
    TASK_ENTRY(test_func1,         50,    200),
    TASK_ENTRY(test_func2,          5,     50),
    TASK_ENTRY(test_func3,          1,     50),

};

uint32_t _tasksize = sizeof(tasks) / sizeof(tasks[0]);

TASK_STRU_T *task_get_tasks(uint32_t taskid)
{
	if (taskid >= _tasksize) {
		return NULL;
	}
	
	return &tasks[taskid];
}

int task_get_tasksize(void)
{
	return(_tasksize);
}


