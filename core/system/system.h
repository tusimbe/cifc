/*!
 *****************************************************************************
 *
 *  Copyright © 2016-2022 CHASING INNOVATION . All Rights Reserved.
 *
 * \file      system.h
 * \author    johnson@chasing-innovation.com
 * \version   1.0
 * \date      2022年4月3日星期日
 * \brief     系统基础函数
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

#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#define ARRAY_SIZE(_arr) (sizeof(_arr) / sizeof(_arr[0]))

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

#define MHZ (1000 * 1000)

#ifndef UNIT_TEST
#define debug(level, fmt, args...)   do {chprintf((BaseSequentialStream *)&SDU1, fmt, ##args);} while(0)
#define printk(fmt, args...)   do {chprintf((BaseSequentialStream *)&SDU1, fmt, ##args);} while(0)
#else
#define debug(level, fmt, args...)   
#define printk(fmt, args...)  
#endif

uint32_t micros(void);
uint32_t millis(void);
uint16_t millis16(void);
uint64_t micros64(void);
uint64_t millis64(void);

#endif
