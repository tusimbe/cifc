/*!
 *****************************************************************************
 *
 *  Copyright © 2017-2018 CHASING INNOVATION. All Rights Reserved.
 *
 * \file      compass_calibrator.h
 * \author    johnson@chasing-innovation.com
 * \version   1.0
 * \date      2022年4月14日星期四
 * \brief     罗盘校准
 *
 *----------------------------------------------------------------------------
 * \attention
 *
 *
 *****************************************************************************
 */

/*****************************************************************************
 change history: 
    1.date  : 2022年4月14日星期四
      author: johnson@chasing-innovation.com
      change: create file

*****************************************************************************/
#ifndef _COMPASS_CALIBRATOR_H_
#define _COMPASS_CALIBRATOR_H_
#include "vector3.h"

void cmps_clbrt_start(bool retry, float delay, uint16_t offset_max);
void cmps_clbrt_new_sample(VECTOR3 *sample);
void cmps_clbrt_stop(void);
void compass_calibrator_cmd(BaseSequentialStream *chp, int argc, char *argv[]);
void cmps_clbrt_update(void);


#endif
