/*!
 *****************************************************************************
 *
 *  Copyright © 2016-2022 CHASING INNOVATION . All Rights Reserved.
 *
 * \file      imu.c
 * \author    johnson@chasing-innovation.com
 * \version   1.0
 * \date      2022年4月3日星期日
 * \brief     imu抽象层
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

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "system.h"
#include "shell.h"
#ifndef UNIT_TEST
#include "usbcfg.h"
#endif
#include "scheduler.h"
#include "base.h"
#include "lpf2.h"
#include "imu.h"
#include "vector3.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// has wait_for_sample() found a sample?
bool _have_sample;

// the delta time in seconds for the last sample
float _delta_time;

// last time a wait_for_sample() returned a sample
uint32_t _last_sample_usec;

// target time for next wait_for_sample() return
uint32_t _next_sample_usec;

// time between samples in microseconds
uint32_t _sample_period_usec;

// last time update() completed
uint32_t _last_update_usec;

// the selected sample rate
uint16_t _sample_rate;
float _loop_delta_t;
float _loop_delta_t_max;

// accelerometer and gyro raw sample rate in units of Hz
float  _accel_raw_sample_rates;
float  _gyro_raw_sample_rates;

/* raw imu data */
VECTOR3  _gyro;
VECTOR3  _accel;

/* data passed through lfp2 */
VECTOR3  _accel_filtered;
VECTOR3  _gyro_filtered;

void *accel_lpf2[3];
void *gyro_lpf2[3];

bool _new_accel_data;
bool _new_gyro_data;

uint64_t _accel_last_sample_us;
uint64_t _gyro_last_sample_us;

// sample times for checking real sensor rate for FIFO sensors
uint16_t _sample_accel_count;
uint32_t _sample_accel_start_us;
uint16_t _sample_gyro_count;
uint32_t _sample_gyro_start_us;

// delta velocity accumulator
VECTOR3 _delta_velocity_acc;
// time accumulator for delta velocity accumulator
float _delta_velocity_acc_dt;


// Most recent accelerometer reading
VECTOR3 _accel;
VECTOR3 _delta_velocity;
float _delta_velocity_dt;
bool _delta_velocity_valid;

// Most recent gyro reading
VECTOR3 _gyro;
VECTOR3 _delta_angle;
float _delta_angle_dt;
bool _delta_angle_valid;

// time accumulator for delta angle accumulator
float _delta_angle_acc_dt;
VECTOR3 _delta_angle_acc;
VECTOR3 _last_delta_angle;
VECTOR3 _last_raw_gyro;


uint32_t imu_debug;

static void _update_gyro(void);
static void _update_accel(void);


// return true if the sensors are still converging and sampling rates could change significantly
static inline bool sensors_converging(void)  
{ 
    return millis() < 30000; 
}

/*
  update the sensor rate for FIFO sensors

  FIFO sensors produce samples at a fixed rate, but the clock in the
  sensor may vary slightly from the system clock. This slowly adjusts
  the rate to the observed rate
*/
void _update_sensor_rate(uint16_t *count, uint32_t *start_us, float *rate_hz)
{
    uint16_t tmp_cnt = *count;
    uint32_t tmp_start_us = *start_us;
    float tmp_rate_hz = *rate_hz;
    
    uint32_t now = micros();
    
    if (tmp_start_us == 0) {
        (*count) = 0;
        (*start_us) = now;
    } else {
        tmp_cnt++;
        if (now - tmp_start_us > 1000000UL) {
            float observed_rate_hz = tmp_cnt * 1.0e6f / (now - tmp_start_us);
#ifdef SENSOR_RATE_DEBUG
            printf("RATE: %.1f should be %.1f\n", observed_rate_hz, tmp_rate_hz);
#endif
            float filter_constant = 0.98f;
            float upper_limit = 1.05f;
            float lower_limit = 0.95f;
            if (sensors_converging()) {
                // converge quickly for first 30s, then more slowly
                filter_constant = 0.8f;
                upper_limit = 2.0f;
                lower_limit = 0.5f;
            }
            
            observed_rate_hz = constrain_float(observed_rate_hz, tmp_rate_hz*lower_limit, tmp_rate_hz*upper_limit);
            (*rate_hz) = filter_constant * tmp_rate_hz + (1-filter_constant) * observed_rate_hz;
            (*count) = 0;
            (*start_us) = now;
        }
        (*count) = tmp_cnt;
    }
}

/*
  wait for a sample to be available. This is the function that
  determines the timing of the main loop in ardupilot.

  Ideally this function would return at exactly the rate given by the
  sample_rate argument given to AP_InertialSensor::init().

  The key output of this function is _delta_time, which is the time
  over which the gyro and accel integration will happen for this
  sample. We want that to be a constant time if possible, but if
  delays occur we need to cope with them. The long term sum of
  _delta_time should be exactly equal to the wall clock elapsed time
 */
void imu_wait_for_sample(void)
{
    if (_have_sample) {
        // the user has called wait_for_sample() again without
        // consuming the sample with update()
        return;
    }

    uint32_t now = micros();

    if (_next_sample_usec == 0 && _delta_time <= 0) {
        // this is the first call to wait_for_sample()
        _last_sample_usec = now - _sample_period_usec;
        _next_sample_usec = now + _sample_period_usec;
        goto check_sample;
    }

    uint32_t wait_usec = (int32_t)(_next_sample_usec - now);
    // see how long it is till the next sample is due
    if (wait_usec <= _sample_period_usec) {
        // we're ahead on time, schedule next sample at expected period
        scheduler_delay_microseconds_boost(wait_usec);
        uint32_t now2 =  micros();
        if ((int32_t)(_next_sample_usec - now2) > 100) {
            printk("shortsleep %u wait_usec=%u\n", (unsigned int)(_next_sample_usec - now2), (unsigned int)wait_usec);
        }
        if ((int32_t)(now2 - _next_sample_usec) > 400) {
            printk("longsleep %u wait_usec=%u\n", (unsigned int)(now2 - _next_sample_usec), (unsigned int)wait_usec);
        }
        _next_sample_usec += _sample_period_usec;
    } else if ((int32_t)(now - _next_sample_usec) < (int32_t)(_sample_period_usec/8)) {
        // we've overshot, but only by a small amount, keep on
        // schedule with no delay
        //printk("overshoot1 %u\n", (unsigned)(now-_next_sample_usec));
        _next_sample_usec += _sample_period_usec;
    } else {
        // we've overshot by a larger amount, re-zero scheduling with
        // no delay
        //printk("overshoot2 %u\n", (unsigned)(now-_next_sample_usec));
        _next_sample_usec = now + _sample_period_usec;
    }

check_sample:		
	{
	    // now we wait until we have the gyro and accel samples we need
	    #if 1
	    while (true) {
            if (_new_gyro_data && _new_accel_data) {
                break;
            }

	        scheduler_delay_microseconds_boost(100);
	    }
        #endif
	}
    now = micros();
    _delta_time = (now - _last_sample_usec) * 1.0e-6f;
    _last_sample_usec = now;

#if 0
    {
        static uint64_t delta_time_sum;
        static uint16_t counter;
        if (delta_time_sum == 0) {
            delta_time_sum = _sample_period_usec;
        }
        delta_time_sum += _delta_time * 1.0e6f;
        if (counter++ == 400) {
            counter = 0;
            hal.console->printf("now=%lu _delta_time_sum=%lu diff=%ld\n",
                                (unsigned long)now,
                                (unsigned long)delta_time_sum,
                                (long)(now - delta_time_sum));
        }
    }
#endif

    _have_sample = true;
}

void imu_update(void)
{
    // during initialisation update() may be called without
    // wait_for_sample(), and a wait is implied
    imu_wait_for_sample();

    _delta_velocity_valid = false;
    _delta_angle_valid = false;

    _update_accel();
    _update_gyro();

    // clear accumulators
    vector3_zero(&_delta_velocity_acc);
    _delta_velocity_acc_dt = 0;
    vector3_zero(&_delta_angle_acc);
    _delta_angle_acc_dt = 0;

    _last_update_usec = micros();
    _have_sample = false;
}

void imu_init(int16_t sample_rate)
{
    // remember the sample rate
    _sample_rate = sample_rate;
    _loop_delta_t = 1.0f / sample_rate;

    // we don't allow deltat values greater than 10x the normal loop
    // time to be exposed outside of INS. Large deltat values can
    // cause divergence of state estimators
    _loop_delta_t_max = 10 * _loop_delta_t;

    _sample_period_usec = 1000*1000UL / _sample_rate;

    _accel_raw_sample_rates = 1000;
    _gyro_raw_sample_rates = 1000;

    // establish the baseline time between samples
    _delta_time = 0;
    _next_sample_usec = 0;
    _last_sample_usec = 0;
    _have_sample = false;

    for (uint8_t i = 0; i < 3; i++) {
        accel_lpf2[i] = lpf2_create(1000, 20);
        if (accel_lpf2[i] == NULL) {
            goto fail;
        }
        gyro_lpf2[i] = lpf2_create(1000, 20);
        if (gyro_lpf2[i] == NULL) {
            goto fail;
        }
    }

    return;

fail:
    for (uint8_t i = 0; i < 3; i++) {
        if (accel_lpf2[i] != NULL) {
            lpf2_destory(accel_lpf2[i]);
        }
    }

    for (uint8_t i = 0; i < 3; i++) {
        if (gyro_lpf2[i] != NULL) {
            lpf2_destory(gyro_lpf2[i]);
        }
    }
	return;
}

void imu_notify_new_accel_raw_sample(float accel[], float gyro[], uint64_t sample_us)
{
    float dt;
    uint64_t now;
    uint64_t last_sample_us;
    (void)gyro;
    
    _update_sensor_rate(&_sample_accel_count, &_sample_accel_start_us, &_accel_raw_sample_rates);

    last_sample_us = _accel_last_sample_us;

    /*
      we have two classes of sensors. FIFO based sensors produce data
      at a very predictable overall rate, but the data comes in
      bunches, so we use the provided sample rate for deltaT. Non-FIFO
      sensors don't bunch up samples, but also tend to vary in actual
      rate, so we use the provided sample_us to get the deltaT. The
      difference between the two is whether sample_us is provided.
     */
    if (sample_us != 0 && _accel_last_sample_us != 0) {
        dt = (sample_us - _accel_last_sample_us) * 1.0e-6f;
        _accel_last_sample_us = sample_us;
    } else {
        // don't accept below 100Hz
        if (_accel_raw_sample_rates < 100) {
            //printk("raw accel samle rates too low %d.\r\n", _accel_raw_sample_rates);
            return;
        }

        dt = 1.0f / _accel_raw_sample_rates;
        _accel_last_sample_us = micros64();
        sample_us = _accel_last_sample_us;
    }

    now = micros64();

    if (now - last_sample_us > 100000U) {
        // zero accumulator if sensor was unhealthy for 0.1s
        //_imu._delta_velocity_acc[instance].zero();
        //_imu._delta_velocity_acc_dt[instance] = 0;
        dt = 0;
    }
    
    // delta velocity
    for (uint8_t i = 0; i < 3; i++) {
        _delta_velocity_acc.v[i] += accel[i] * dt;
    }
    _delta_velocity_acc_dt += dt;

    for (uint8_t i = 0; i < 3; i++) {
        _accel_filtered.v[i] = lpf2_apply(accel_lpf2[i], accel[i]);
        if (isnan(_accel_filtered.v[i])  || isinf(_accel_filtered.v[i])) {
            lpf2_reset(accel_lpf2[i]);
        }
    }
    if (imu_debug == 1) {
        printk("a(%4.4f, %4.4f, %4.4f)\r", 
            _accel_filtered.v[0], _accel_filtered.v[1], _accel_filtered.v[2]);
    }

    _new_accel_data = true;

}

void imu_notify_new_gyro_raw_sample(float gyro[], uint64_t sample_us)
{
    float dt;
    uint64_t last_sample_us;
    uint64_t now;
    uint8_t i;

    _update_sensor_rate(&_sample_gyro_count, &_sample_gyro_start_us, &_gyro_raw_sample_rates);
    last_sample_us = _gyro_last_sample_us;

    /*
      we have two classes of sensors. FIFO based sensors produce data
      at a very predictable overall rate, but the data comes in
      bunches, so we use the provided sample rate for deltaT. Non-FIFO
      sensors don't bunch up samples, but also tend to vary in actual
      rate, so we use the provided sample_us to get the deltaT. The
      difference between the two is whether sample_us is provided.
     */
    if (sample_us != 0 && _gyro_last_sample_us != 0) {
        dt = (sample_us - _gyro_last_sample_us) * 1.0e-6f;
        _gyro_last_sample_us = sample_us;
    } else {
        // don't accept below 100Hz
        if (_gyro_raw_sample_rates < 100) {
            //printk("gyro raw samle rates too low %d.\r\n", _gyro_raw_sample_rates);
            return;
        }

        dt = 1.0f / _gyro_raw_sample_rates;
        _gyro_last_sample_us = micros64();
        sample_us = _gyro_last_sample_us;
    }


    // compute delta angle
    VECTOR3 delta_angle;

    for (i = 0; i < 3; i++) {
        delta_angle.v[i] = (gyro[i] + _last_raw_gyro.v[i]) * 0.5f * dt;
    }

    // compute coning correction
    // see page 26 of:
    // Tian et al (2010) Three-loop Integration of GPS and Strapdown INS with Coning and Sculling Compensation
    // Available: http://www.sage.unsw.edu.au/snap/publications/tian_etal2010b.pdf
    // see also examples/coning.py
    VECTOR3 delta_coning;
    VECTOR3 tmp;

    tmp = vector3_product(&_last_delta_angle, (1.0f / 6.0f));
    delta_coning = vector3_add(&_delta_angle_acc, &tmp);
    delta_coning = vector3_cross(&delta_coning, &delta_angle);
    delta_coning = vector3_product(&delta_coning, 0.5f);

    {
        now = micros64();

        if (now - last_sample_us > 100000U) {
            // zero accumulator if sensor was unhealthy for 0.1s
            vector3_zero(&_delta_angle_acc);
            _delta_angle_acc_dt = 0;
            dt = 0;
            vector3_zero(&delta_angle);
        }

        // integrate delta angle accumulator
        // the angles and coning corrections are accumulated separately in the
        // referenced paper, but in simulation little difference was found between
        // integrating together and integrating separately (see examples/coning.py)
        tmp = vector3_add(&delta_angle, &delta_coning);
        _delta_angle_acc = vector3_add(&_delta_angle_acc, &tmp);
        _delta_angle_acc_dt += dt;

        // save previous delta angle for coning correction
        vector3_copy(&_last_delta_angle, &delta_angle);
        tmp.v[0] = gyro[0];
        tmp.v[1] = gyro[1];
        tmp.v[2] = gyro[2];
        vector3_copy(&_last_raw_gyro, &tmp);

        // apply the low pass filter
        VECTOR3 gyro_filtered;
        for (uint8_t i = 0; i < 3; i++) {
            gyro_filtered.v[i] = lpf2_apply(gyro_lpf2[i], gyro[i]);

            // if the filtering failed in any way then reset the filters and keep the old value
            if (isnan(gyro_filtered.v[i]) || isinf(gyro_filtered.v[i])) {
                lpf2_reset(gyro_lpf2[i]);
            } else {
                vector3_copy(&_gyro_filtered, &gyro_filtered);
            }
        }
        _new_gyro_data = true;
    }
}

/*
  common gyro update function for all backends
 */
static void _update_gyro(void)
{    
    if (_new_gyro_data) {
        vector3_copy(&_gyro, &_gyro_filtered);
        
        // publish delta angle
        vector3_copy(&_delta_angle, &_delta_angle_acc);
        _delta_angle_dt = _delta_angle_acc_dt;
        _delta_angle_valid = true;
        _new_gyro_data = false;
    }
}

/*
  common accel update function for all backends
 */
static void _update_accel(void)
{    

    if (_new_accel_data) {
        vector3_copy(&_accel, &_accel_filtered);
        
        // publish delta velocity
        vector3_copy(&_delta_velocity, &_delta_velocity_acc);
        _delta_velocity_dt = _delta_velocity_acc_dt;
        _delta_velocity_valid = true;

        _new_accel_data = false;
    }
}

float get_delta_time(void)
{ 
    return MIN(_delta_time, _loop_delta_t_max); 
}

/*
  get delta velocity if available
*/
bool imu_get_delta_velocity(VECTOR3 *delta_velocity)
{
    if (_delta_velocity_valid) {
        vector3_copy(delta_velocity, &_delta_velocity);
        return true;
    } 
    return false;
}

/*
  return delta_time for the delta_velocity
 */
float imu_get_delta_velocity_dt(void)
{
    float ret;
    if (_delta_velocity_valid) {
        ret = _delta_velocity_dt;
    } else {
        ret = get_delta_time();
    }
    
    ret = MIN(ret, _loop_delta_t_max);
    return ret;
}

void imu_print_data(void)
{
    if (imu_debug == 3) {
        printk("a(%4.4f, %4.4f, %4.4f) g(%4.4f, %4.4f, %4.4f)\r", 
            _accel.v[0], _accel.v[1], _accel.v[2], _gyro.v[0], _gyro.v[1], _gyro.v[2]);
    }
}

void imu_cmd(BaseSequentialStream *chp, int argc, char *argv[])
{
	int v;
    uint32_t cmd_type;
	(void)chp;
    
    if (argc > 2) {
        shellUsage(chp, "imu [debug | show] [level]");
        return;
    } else {
        
    }

    if (argc == 1) {
		if (strcmp(argv[0], "show") == 0) {
			cmd_type = 1;
		} else if (strcmp(argv[0], "init") == 0){
			cmd_type = 2;
		} else if (strcmp(argv[0], "fifo") == 0){
			cmd_type = 3;
		} else {
			cmd_type = -1;
			shellUsage(chp, "mpu6k [id | init | fifo]");
			return;
		}
    } else if (argc == 2){
    	if (strcmp(argv[0], "debug") == 0)
    	{
    		v = atoi(argv[1]);
    		imu_debug = v;
    	} else {
            printk("unknown cmd.\r\n");
        }

        return;
    }

    if (cmd_type == 1) {
        printk("raw_sample_rates: %4.4f %4.4f.\r\n", _accel_raw_sample_rates, _gyro_raw_sample_rates);
    } else if (cmd_type == 2) {

    } else if (cmd_type == 3) {

    }
}

