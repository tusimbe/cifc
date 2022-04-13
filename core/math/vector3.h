/*!
 *****************************************************************************
 *
 *  Copyright © 2017-2018 CHASING INNOVATION. All Rights Reserved.
 *
 * \file      vector3.h
 * \author    johnson@chasing-innovation.com
 * \version   1.0
 * \date      2022年4月11日星期一
 * \brief     3维向量计算
 *
 *----------------------------------------------------------------------------
 * \attention
 *
 *
 *****************************************************************************
 */

/*****************************************************************************
 change history: 
    1.date  : 2022年4月11日星期一
      author: johnson@chasing-innovation.com
      change: create file

*****************************************************************************/
#ifndef _VECTOR3_H_
#define _VECTOR3_H_

typedef struct stru_vector3 {
    float v[3];
} VECTOR3;

VECTOR3 vector3_cross(VECTOR3 *v1, VECTOR3 *v2);
VECTOR3 vector3_product(VECTOR3 *v1, float v2);
VECTOR3 vector3_add(VECTOR3 *v1, VECTOR3 *v2);
VECTOR3 vector3_div(VECTOR3 *v1, float d);
void vector3_zero(VECTOR3 *v);
void vector3_copy(VECTOR3 *dst, VECTOR3 *src);
bool vector3_is_nan(VECTOR3 *v);
bool vector3_is_inf(VECTOR3 *v);
float vector3_length(VECTOR3 *v);
bool vector3_is_zero(VECTOR3 *v);

#endif
