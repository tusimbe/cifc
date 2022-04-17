/*!
 *****************************************************************************
 *
 *  Copyright © 2017-2018 CHASING INNOVATION. All Rights Reserved.
 *
 * \file      vector3.c
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
#include "ch.h"
#include "vector3.h"
#include <float.h>
#include <math.h>

VECTOR3 vector3_new(float x, float y, float z)
{
    VECTOR3 v3;

    v3.v[0] = x; v3.v[1] = y; v3.v[2] = z;
    return v3;
}

VECTOR3 vector3_cross(VECTOR3 *v1, VECTOR3 *v2)
{
    VECTOR3 ret;
    
    ret.v[0] = v1->v[1]*v2->v[2] - v1->v[2]*v2->v[1];
    ret.v[1] = v1->v[2]*v2->v[0] - v1->v[0]*v2->v[2];
    ret.v[2] = v1->v[0]*v2->v[1] - v1->v[1]*v2->v[0];
    
    return ret;
}

VECTOR3 vector3_product(VECTOR3 *v1, float v2)
{
    uint8_t i;
    VECTOR3 ret;

    for (i = 0; i < 3; i++) {
        ret.v[i] = v1->v[i] * v2;
    }

    return ret;
}

VECTOR3 vector3_add(VECTOR3 *v1, VECTOR3 *v2)
{
    uint8_t i;
    VECTOR3 r;

    for (i = 0; i < 3; i++) {
        r.v[i] = v1->v[i] + v2->v[i];
    }

    return r;
}

VECTOR3 vector3_sub(VECTOR3 *v1, VECTOR3 *v2)
{
    uint8_t i;
    VECTOR3 r;

    for (i = 0; i < 3; i++) {
        r.v[i] = v1->v[i] - v2->v[i];
    }

    return r;
}

VECTOR3 vector3_div(VECTOR3 *v1, float d)
{
    uint8_t i;
    VECTOR3 r;

    for (i = 0; i < 3; i++) {
        r.v[i] = v1->v[i] / d;
    }

    return r;
}

void vector3_set(VECTOR3 *v, float v0, float v1, float v2)
{
    v->v[0] = v0;
    v->v[1] = v1;
    v->v[2] = v2;
}

VECTOR3 vector3_neg(VECTOR3 *v)
{
    uint8_t i;
    VECTOR3 tmp;

    for (i = 0; i < 3; i++) {
        tmp.v[i] = -v->v[i];
    }
    return tmp;
}

void vector3_zero(VECTOR3 *v)
{
    v->v[0] = v->v[1] = v->v[2] = 0;
}

void vector3_copy(VECTOR3 *dst, VECTOR3 *src)
{
    uint8_t i;
    
    for (i = 0; i < 3; i++) {
        dst->v[i] = src->v[i];
    }

    return;
}

bool vector3_is_nan(VECTOR3 *v)
{
    return isnan(v->v[0]) || isnan(v->v[1]) || isnan(v->v[2]);
}

bool vector3_is_inf(VECTOR3 *v)
{
    return isinf(v->v[0]) || isinf(v->v[1]) || isinf(v->v[2]);
}

float vector3_length(VECTOR3 *v)
{
    return sqrtf(v->v[0] * v->v[0] + v->v[1] * v->v[1] + v->v[2] * v->v[2]);
}

bool vector3_is_zero(VECTOR3 *v)
{
    // check if all elements are zero
    return (fabsf(v->v[0]) < FLT_EPSILON) &&
           (fabsf(v->v[1]) < FLT_EPSILON) &&
           (fabsf(v->v[2]) < FLT_EPSILON);

}

