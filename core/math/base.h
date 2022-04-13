/*!
 *****************************************************************************
 *
 *  Copyright © 2016-2022 CHASING INNOVATION . All Rights Reserved.
 *
 * \file      base.h
 * \author    johnson@chasing-innovation.com
 * \version   1.0
 * \date      2022年4月7日星期四
 * \brief     基本数学计算
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
#ifndef _MATH_BASE_H_
#define _MATH_BASE_H_
#include <float.h>
#include <math.h>
static inline bool is_zero(const float fVal1) {
    return (fabsf(fVal1) < FLT_EPSILON);
}

/* 
 * @brief: Check whether a float is greater than zero
 */
static inline bool is_positive(const float fVal1) {
    return (fVal1 >= FLT_EPSILON);
}

/* 
 * @brief: Check whether a float is less than zero
 */
static inline bool is_negative(const float fVal1) {
    return (fVal1 <= (-1.0 * FLT_EPSILON));
}

static inline float constrain_float(const float amt, const float low, const float high)
{
    // the check for NaN as a float prevents propagation of floating point
    // errors through any function that uses constrain_value(). The normal
    // float semantics already handle -Inf and +Inf
    if (isnan(amt)) {
        return (low + high) / 2;
    }

    if (amt < low) {
        return low;
    }

    if (amt > high) {
        return high;
    }

    return amt;
}

#endif

