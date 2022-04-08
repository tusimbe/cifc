/*!
 *****************************************************************************
 *
 *  Copyright © 2016-2022 CHASING INNOVATION . All Rights Reserved.
 *
 * \file      task.h
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
#ifndef _TASK_H_
#define _TASK_H_

#include "ch.h"

typedef struct task_stru_s {
	void (*function)(void);
	const char *name;
	float rate_hz;
	uint16_t max_time_micros;
	// tick counter at the time we last ran each task
	uint16_t _last_run;
	void *perf_counter_p;
} TASK_STRU_T;

TASK_STRU_T *task_get_tasks(uint32_t taskid);
int task_get_tasksize(void);


#endif
