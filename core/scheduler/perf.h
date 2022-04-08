/*!
 *****************************************************************************
 *
 *  Copyright © 2016-2022 CHASING INNOVATION . All Rights Reserved.
 *
 * \file      perf.h
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

#ifndef _PERF_H_
#define _PERF_H_

void perf_reset(void);
void ignore_this_loop(void);
void perf_set_loop_rate(uint16_t rate_hz);
void update_logging(void);
void perf_check_loop_time(uint32_t time_in_micros);
void perf_cmd(BaseSequentialStream *chp, int argc, char *argv[]);
void *perf_counter_create(const char *name);
void perf_begin(void *p);
void perf_end(void *p);

#endif
