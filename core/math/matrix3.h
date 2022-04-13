/*!
 *****************************************************************************
 *
 *  Copyright © 2017-2018 CHASING INNOVATION. All Rights Reserved.
 *
 * \file      matrix3.h
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
#ifndef _MATRIX3_H_
#define _MATRIX3_H_

typedef struct stru_matrix3 {
    float m[3][3];
} MATRIX3;

VECTOR3 matrix3_mul_vectro3(MATRIX3 *m3, VECTOR3 *v);

#endif
