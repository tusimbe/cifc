/*!
 *****************************************************************************
 *
 *  Copyright © 2016-2022 CHASING INNOVATION . All Rights Reserved.
 *
 * \file      mpu6000.h
 * \author    johnson@chasing-innovation.com
 * \version   1.0
 * \date      2022年4月5日星期二
 * \brief     MPU6000驱动
 *
 *----------------------------------------------------------------------------
 * \attention
 *
 *
 *****************************************************************************
 */

/*****************************************************************************
 change history: 
    1.date  : 2022年4月5日星期二
      author: johnson@chasing-innovation.com
      change: create file

*****************************************************************************/

#ifndef _DRV_MPU6000_H_
#define _DRV_MPU6000_H_

int32_t mpu6k_init(uint32_t spi_id);
void mpu6k_cmd(BaseSequentialStream *chp, int argc, char *argv[]);

#endif
