/*!
 *****************************************************************************
 *
 *  Copyright © 2016-2022 CHASING INNOVATION . All Rights Reserved.
 *
 * \file      system.c
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
#include "ch.h"
#include "hrt.h"
#include "system.h"

uint32_t micros(void)
{
    return hrt_micros32();
}

uint32_t millis(void)
{
    return hrt_millis32();
}

uint16_t millis16(void)
{
    return hrt_millis32() & 0xFFFF;
}

uint64_t micros64(void)
{
    return hrt_micros64();
}

uint64_t millis64(void)
{
    return hrt_micros64() / 1000U;
}

