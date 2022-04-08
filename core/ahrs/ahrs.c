/*!
 *****************************************************************************
 *
 *  Copyright © 2016-2022 CHASING INNOVATION . All Rights Reserved.
 *
 * \file      ahrs.c
 * \author    johnson@chasing-innovation.com
 * \version   1.0
 * \date      2022年4月7日星期四
 * \brief     AHRS模块
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
#include "ch.h"
#include "hal.h"
#include "imu.h"

#include <stdlib.h>
#include <string.h>

void ahrs_update(void)
{
    imu_print_data();
}

