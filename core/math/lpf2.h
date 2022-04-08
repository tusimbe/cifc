/*!
 *****************************************************************************
 *
 *  Copyright © 2016-2022 CHASING INNOVATION . All Rights Reserved.
 *
 * \file      lpf2.h
 * \author    johnson@chasing-innovation.com
 * \version   1.0
 * \date      2022年4月7日星期四
 * \brief     二阶低通滤波器
 *
 *----------------------------------------------------------------------------
 * \attention
 *
 *
 *****************************************************************************
 */

/*****************************************************************************
 change history: 
    1.date  : 2022年4月7日星期四
      author: johnson@chasing-innovation.com
      change: create file

*****************************************************************************/
#ifndef _MATH_LPF2_H_
#define _MATH_LPF2_H_

float lpf2_apply(void *p, const float sample);
void lpf2_reset(void *p);
void *lpf2_create(float sample_freq, float cutoff_freq);
void lpf2_destory(void *p);


#endif
