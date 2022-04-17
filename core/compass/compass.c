/*!
 *****************************************************************************
 *
 *  Copyright © 2017-2018 CHASING INNOVATION. All Rights Reserved.
 *
 * \file      compass.c
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
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "system.h"
#include "scheduler.h"
#ifndef UNIT_TEST
#include "usbcfg.h"
#include "qmc5883l.h"
#endif
#include "base.h"
#include "vector3.h"
#include "compass.h"
#include "compass_calibrator.h"
#include "matrix3.h"
#include <string.h>
#include <math.h>

#define COMPASS_SAMPLE_HZ               (100)
#define COMPASS_SAMPLE_US               ((1000 * 1000) / COMPASS_SAMPLE_HZ)
#define COMPASS_MAX_SAMPLES             (20)
#define FILTER_KOEF                     (0.1f)
#define COMPASS_MIN_SCALE_FACTOR         0.85
#define COMPASS_MAX_SCALE_FACTOR         1.3


typedef struct stru_compass {
    bool        healthy;
    
    VECTOR3       offset;
    VECTOR3       diagonals;
    VECTOR3       offdiagonals;
    float       scale_factor;

    uint32_t    use_for_yaw;

    // when we last got data
    uint32_t    last_update_ms;
    uint32_t    last_update_usec;

    // accumulated samples, protected by _sem, used by AP_Compass_Backend
    VECTOR3    accum;
    uint32_t accum_count;
    
    // mean field length for range filter
    float _mean_field_length;
    
    // number of dropped samples. Not used for now, but can be usable to choose more reliable sensor
    uint32_t _error_count;
} STRU_COMPASS;

static STRU_COMPASS compass; 

// parameters
uint8_t param_compass_range = 0;

static bool compass_sample_thread_start(void);
static bool have_scale_factor(void);
void publish_raw_field(VECTOR3 *mag);
void correct_field(VECTOR3 *mag);
bool field_ok(VECTOR3 *field);


static THD_FUNCTION(compass_sample_thd, arg) {

    (void)arg;
    while (true) {
        chThdSleepMicroseconds(COMPASS_SAMPLE_HZ);
        hal_compass_timer();
    }
}


int32_t compass_init(void)
{
    memset(&compass, 0, sizeof(STRU_COMPASS));
    
    if (!compass_sample_thread_start()) {
        printk("[%s, %d] compass thread start failed.\r\n", __func__, __LINE__);
        return -1;
    }

    return 0;
}

static bool compass_sample_thread_start(void)
{
    thread_t* thread_ctx;

	thread_ctx = chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(1024), "compass_period",
				      SCHE_I2C_PRIORITY, compass_sample_thd, NULL);
    if (thread_ctx == NULL) {
        return false;
    }

    return true;
}

void compass_notify_new_data(VECTOR3 *magData)
{
    /* rotate raw_field from sensor frame to body frame */
    //rotate_field(magData);

    /* publish raw_field (uncorrected point sample) for calibration use */
    publish_raw_field(magData);

    /* correct raw_field for known errors */
    correct_field(magData);

    if (!field_ok(magData)) {
        return;
    }

    compass.accum = vector3_add(&compass.accum, magData);
    compass.accum_count++;
    if (compass.accum_count >= COMPASS_MAX_SAMPLES) {
        compass.accum_count /= 2;
        compass.accum = vector3_div(&compass.accum, 2);
    }

}


void publish_raw_field(VECTOR3 *mag)
{
    // note that we do not set last_update_usec here as otherwise the
    // EKF and DCM would end up consuming compass data at the full
    // sensor rate. We want them to consume only the filtered fields
    compass.last_update_ms = millis();
    cmps_clbrt_new_sample(mag);
}

void correct_field(VECTOR3 *mag)
{
    if (vector3_is_zero(&compass.diagonals)) {
        compass.diagonals.v[0] = compass.diagonals.v[1] = compass.diagonals.v[2] = 1.0f;
    }

    VECTOR3 *offsets = &compass.offset;
    VECTOR3 *diagonals = &compass.diagonals;
    VECTOR3 *offdiagonals = &compass.offdiagonals;

    // add in the basic offsets
    *mag = vector3_add(mag, offsets);

    // add in scale factor, use a wide sanity check. The calibrator
    // uses a narrower check.
    if (have_scale_factor()) {
        *mag = vector3_product(mag, compass.scale_factor);
    }

    // apply eliptical correction
    MATRIX3 mat;
    mat.m[0][0] = diagonals->v[0];
    mat.m[0][1] = offdiagonals->v[0];
    mat.m[0][2] = offdiagonals->v[1];

    mat.m[1][0] = offdiagonals->v[0];
    mat.m[1][1] = diagonals->v[1];
    mat.m[1][2] = offdiagonals->v[2];

    mat.m[2][0] = offdiagonals->v[1];
    mat.m[2][1] = offdiagonals->v[2];
    mat.m[2][2] = diagonals->v[2];

    *mag = matrix3_mul_vectro3(&mat, mag);

}

/* Check that the compass value is valid by using a mean filter. If
 * the value is further than filtrer_range from mean value, it is
 * rejected. 
*/
bool field_ok(VECTOR3 *field)
{
    if (vector3_is_inf(field)|| vector3_is_nan(field)) {
        return false;
    }

    const float range = (float)param_compass_range;
    if (range <= 0) {
        return true;
    }

    const float length = vector3_length(field);

    if (is_zero(compass._mean_field_length)) {
        compass._mean_field_length = length;
        return true;
    }

    bool ret = true;
    // diff divide by mean value in percent ( with the *200.0f on later line)
    const float d = fabsf(compass._mean_field_length - length) / (compass._mean_field_length + length);  
    float koeff = FILTER_KOEF;

    if (d * 200.0f > range) {  // check the difference from mean value outside allowed range
        // printf("\nCompass field length error: mean %f got %f\n", (double)_mean_field_length, (double)length );
        ret = false;
        koeff /= (d * 10.0f);  // 2.5 and more, so one bad sample never change mean more than 4%
        compass._error_count++;
    }
    compass._mean_field_length = compass._mean_field_length * (1 - koeff) + length * koeff;  // complimentary filter 1/k

    return ret;
}

/*
  return true if we have a valid scale factor
 */
static bool have_scale_factor(void)
{
    if (compass.scale_factor < COMPASS_MIN_SCALE_FACTOR ||
        compass.scale_factor > COMPASS_MAX_SCALE_FACTOR) {
        return false;
    }
    return true;
}

