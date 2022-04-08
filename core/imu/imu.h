/*!
 *****************************************************************************
 *
 *  Copyright © 2016-2022 CHASING INNOVATION . All Rights Reserved.
 *
 * \file      imu.h
 * \author    johnson@chasing-innovation.com
 * \version   1.0
 * \date      2022年4月3日星期日
 * \brief     imu框架
 *
 *----------------------------------------------------------------------------
 * \attention
 *
 *
 *****************************************************************************
 */

/*****************************************************************************
 change history: 
    1.date  : 2022年4月3日星期日
      author: johnson@chasing-innovation.com
      change: create file

*****************************************************************************/
#ifndef _IMU_H_
#define _IMU_H_

void imu_wait_for_sample(void);
void imu_update(void);
void imu_init(int16_t sample_rate);
void imu_notify_new_accel_raw_sample(float accel[], float gyro[], uint64_t sample_us);
void imu_notify_new_gyro_raw_sample(float gyro[], uint64_t sample_us);
void imu_print_data(void);
void imu_cmd(BaseSequentialStream *chp, int argc, char *argv[]);


#endif
