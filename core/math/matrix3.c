/*!
 *****************************************************************************
 *
 *  Copyright © 2017-2018 CHASING INNOVATION. All Rights Reserved.
 *
 * \file      matrix3.c
 * \author    johnson@chasing-innovation.com
 * \version   1.0
 * \date      2022年4月13日星期三
 * \brief     3x3矩阵运算
 *
 *----------------------------------------------------------------------------
 * \attention
 *
 *
 *****************************************************************************
 */

/*****************************************************************************
 change history: 
    1.date  : 2022年4月13日星期三
      author: johnson@chasing-innovation.com
      change: create file

*****************************************************************************/
#include "ch.h"
#include "vector3.h"
#include "matrix3.h"

VECTOR3 matrix3_mul_vectro3(MATRIX3 *m3, VECTOR3 *v)
{
    VECTOR3 v3r;
    v3r.v[0] = m3->m[0][0] * v->v[0] + m3->m[0][1] * v->v[1] + m3->m[0][2] * v->v[2];
    v3r.v[1] = m3->m[1][0] * v->v[0] + m3->m[1][1] * v->v[1] + m3->m[1][2] * v->v[2];
    v3r.v[2] = m3->m[2][0] * v->v[0] + m3->m[2][1] * v->v[1] + m3->m[2][2] * v->v[2];

    return v3r; 
}

