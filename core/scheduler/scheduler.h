/*!
 *****************************************************************************
 *
 *  Copyright © 2016-2022 CHASING INNOVATION . All Rights Reserved.
 *
 * \file      scheduler.h
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


#ifndef SCHEDULER_H
#define SCHEDULER_H

// SPI priority needs to be above main priority to ensure fast sampling of IMUs can keep up
// with the data rate
#define SCHE_SPI_PRIORITY          181 
#define SCHE_TIMER_PRIORITY        181
#define SCHE_SHELL_PRIORITY        179
#define SCHE_CAN_PRIORITY          178
#define SCHE_RCIN_PRIORITY         177
#define SCHE_I2C_PRIORITY          176
#define SCHE_UART_PRIORITY          60
#define SCHE_STORAGE_PRIORITY       59
#define SCHE_IO_PRIORITY            58
#define SCHE_STARTUP_PRIORITY       10
#define SCHE_SCRIPTING_PRIORITY    LOWPRIO


void scheduler_init(void);
void scheduler_shellThdCreate(void);
void scheduler_set_main_thread(thread_t* thd);
bool scheduler_check_called_boost(void);
void scheduler_boost_end(void);
void scheduler_delay(uint16_t ms);
void scheduler_delay_microseconds(uint16_t usec);
void scheduler_delay_microseconds_boost(uint16_t usec);
void scheduler_loop(void);
uint32_t scheduler_get_extra_loop_us(void);


#endif
