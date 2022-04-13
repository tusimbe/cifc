/*!
 *****************************************************************************
 *
 *  Copyright © 2017-2018 CHASING INNOVATION. All Rights Reserved.
 *
 * \file      compass.h
 * \author    johnson@chasing-innovation.com
 * \version   1.0
 * \date      2022年4月12日星期二
 * \brief     compass抽象层
 *
 *----------------------------------------------------------------------------
 * \attention
 *
 *
 *****************************************************************************
 */

/*****************************************************************************
 change history: 
    1.date  : 2022年4月12日星期二
      author: johnson@chasing-innovation.com
      change: create file

*****************************************************************************/
#ifndef _COMPASS_H_
#define _COMPASS_H_

int32_t compass_init(void);
void compass_notify_new_data(VECTOR3 *magData);
void hal_compass_timer(void);


#endif
