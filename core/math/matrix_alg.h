/*!
 *****************************************************************************
 *
 *  Copyright © 2017-2018 CHASING INNOVATION. All Rights Reserved.
 *
 * \file      matrix_alg.h
 * \author    johnson@chasing-innovation.com
 * \version   1.0
 * \date      2022年4月15日星期五
 * \brief     矩阵运算
 *
 *----------------------------------------------------------------------------
 * \attention
 *
 *
 *****************************************************************************
 */

/*****************************************************************************
 change history: 
    1.date  : 2022年4月15日星期五
      author: johnson@chasing-innovation.com
      change: create file

*****************************************************************************/
#ifndef _MATRIX_ALG_H_
#define _MATRIX_ALG_H_

/*
 *    generic matrix inverse code
 *
 *    @param     x,     input nxn matrix
 *    @param     y,     Output inverted nxn matrix
 *    @param     n,     dimension of square matrix
 *    @returns          false = matrix is Singular, true = matrix inversion successful
 */
bool inverse(float x[], float y[], uint16_t dim);

#endif
