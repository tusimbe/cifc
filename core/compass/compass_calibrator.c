/*!
 *****************************************************************************
 *
 *  Copyright © 2017-2018 CHASING INNOVATION. All Rights Reserved.
 *
 * \file      compass_calibrator.c
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
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "system.h"
#include "usbcfg.h"
#include "shell.h"
#include "base.h"
#include "vector3.h"
#include "matrix3.h"
#include "geodisicGrip.h"
#include "matrix_alg.h"
#include <string.h>
#include <stdlib.h>

#define      COMPASS_CAL_NUM_SPHERE_PARAMS       4
#define      COMPASS_CAL_NUM_ELLIPSOID_PARAMS    9
#define      COMPASS_CAL_NUM_SAMPLES             300

#define      FIELD_RADIUS_MIN                    150
#define      FIELD_RADIUS_MAX                    950

// compass calibration states
enum Status {
    NOT_STARTED = 0,
    WAITING_TO_START = 1,
    RUNNING_STEP_ONE = 2,
    RUNNING_STEP_TWO = 3,
    CC_SUCCESS = 4,
    FAILED = 5,
    BAD_ORIENTATION = 6,
    BAD_RADIUS = 7,
};

typedef struct {
    float radius;        // magnetic field strength calculated from samples
    VECTOR3 offset;      // offsets
    VECTOR3 diag;        // diagonal scaling
    VECTOR3 offdiag;     // off diagonal scaling
    float scale_factor;  // scaling factor to compensate for radius error
} param_t;

enum Status _status;                    // current state of calibrator

// behavioral state
uint32_t _start_time_ms;                // system time start() function was last called
uint8_t _attempt;                       // number of attempts have been made to calibrate
uint32_t _last_sample_ms;               // system time of last sample received for timeout
VECTOR3 *_sample_buffer;                // buffer of sensor values
uint16_t _samples_collected;            // number of samples in buffer
uint16_t _samples_thinned;              // number of samples removed by the thin_samples() call (called before step 2 begins)
uint8_t _completion_mask[10];

// fit state
param_t _params;                        // latest calibration outputs
uint16_t _fit_step;                     // step during RUNNING_STEP_ONE/TWO which performs sphere fit and ellipsoid fit
float _fitness;                         // fitness (mean squared residuals) of current parameters
float _initial_fitness;                 // fitness before latest "fit" was attempted (used to determine if fit was an improvement)
float _sphere_lambda;                   // sphere fit's lambda
float _ellipsoid_lambda;                // ellipsoid fit's lambda

// values provided by caller
float _delay_start_sec;                 // seconds to delay start of calibration (provided by caller)
bool _retry;                            // true if calibration should be restarted on failured (provided by caller)
float _tolerance = 8.0;                 // worst acceptable RMS tolerance (aka fitness).  see set_tolerance()
uint16_t _offset_max;                   // maximum acceptable offsets (provided by caller)

bool accept_sample(VECTOR3 *sample, uint16_t skip_index);
bool set_status(enum Status status);
bool calculate_orientation(void);
bool fit_acceptable(void);


bool running(void)
{
    return ((_status == RUNNING_STEP_ONE) || (_status == RUNNING_STEP_TWO));
}

bool fitting(void)
{
    return running() && (_samples_collected == COMPASS_CAL_NUM_SAMPLES);
}

// update completion mask based on latest sample
// used to ensure we have collected samples in all directions
void update_completion_mask(VECTOR3 *v)
{
    VECTOR3 *offset = &_params.offset;
    VECTOR3 *diag = &_params.diag;
    VECTOR3 *offdiag = &_params.offdiag;
    MATRIX3 softiron;
    VECTOR3 v3tmp, corrected;
    softiron = matrix3_new(
        diag->v[0]    , offdiag->v[0]  , offdiag->v[1],
        offdiag->v[0] , diag->v[1]     , offdiag->v[2],
        offdiag->v[1] , offdiag->v[2]  , diag->v[2]
    );
    
    v3tmp = vector3_add(v, offset);
    corrected = matrix3_mul_vectro3(&softiron, &v3tmp);

    int sect = section(&corrected, true);
    if (sect < 0) {
        return;
    }
    _completion_mask[sect / 8] |= 1 << (sect % 8);
}

// reset and update the completion mask using all samples in the sample buffer
void update_completion_mask_all(void)
{
    memset(_completion_mask, 0, sizeof(_completion_mask));
    for (int i = 0; i < _samples_collected; i++) {
        update_completion_mask(&_sample_buffer[i]);
    }
}

void thin_samples(void)
{
    VECTOR3 temp;
    
    if (_sample_buffer == NULL) {
        return;
    }

    _samples_thinned = 0;
    // shuffle the samples http://en.wikipedia.org/wiki/Fisher%E2%80%93Yates_shuffle
    // this is so that adjacent samples don't get sequentially eliminated
    for (uint16_t i=_samples_collected-1; i>=1; i--) {
        uint16_t j = get_random16() % (i+1);
        vector3_copy(&temp, &_sample_buffer[i]);
        vector3_copy(&_sample_buffer[i], &_sample_buffer[j]);
        vector3_copy(&_sample_buffer[j], &temp);
    }

    // remove any samples that are close together
    for (uint16_t i = 0; i < _samples_collected; i++) {
        if (!accept_sample(&_sample_buffer[i], i)) {
            vector3_copy(&_sample_buffer[i], &_sample_buffer[_samples_collected-1]);
            _samples_collected--;
            _samples_thinned++;
        }
    }

    update_completion_mask_all();
}

/*
 * The sample acceptance distance is determined as follows:
 * For any regular polyhedron with triangular faces, the angle theta subtended
 * by two closest points is defined as
 *
 *      theta = arccos(cos(A)/(1-cos(A)))
 *
 * Where:
 *      A = (4pi/F + pi)/3
 * and
 *      F = 2V - 4 is the number of faces for the polyhedron in consideration,
 *      which depends on the number of vertices V
 *
 * The above equation was proved after solving for spherical triangular excess
 * and related equations.
 */
bool accept_sample(VECTOR3 *sample, uint16_t skip_index)
{
    static const uint16_t faces = (2 * COMPASS_CAL_NUM_SAMPLES - 4);
    static const float a = (4.0f * M_PI / (3.0f * faces)) + M_PI / 3.0f;
    static const float theta = 0.5f * acosf(cosf(a) / (1.0f - cosf(a)));

    VECTOR3 tmp;

    if (_sample_buffer == NULL) {
        return false;
    }

    float min_distance = _params.radius * 2*sinf(theta/2);

    for (uint16_t i = 0; i<_samples_collected; i++) {
        if (i != skip_index) {
            tmp = vector3_sub(sample, &_sample_buffer[i]);
            float distance = vector3_length(&tmp);
            if (distance < min_distance) {
                return false;
            }
        }
    }
    return true;
}

float calc_residual(VECTOR3 *sample, param_t *params)
{
    MATRIX3 softiron;
    VECTOR3 v3tmp;
    softiron = matrix3_new(
        params->diag.v[0]    , params->offdiag.v[0] , params->offdiag.v[1],
        params->offdiag.v[0] , params->diag.v[1]    , params->offdiag.v[2],
        params->offdiag.v[1] , params->offdiag.v[2] , params->diag.v[2]
    );

    v3tmp = vector3_add(sample, &params->offset);
    v3tmp = matrix3_mul_vectro3(&softiron, &v3tmp);
    
    return params->radius - vector3_length(&v3tmp);
}

// calc the fitness given a set of parameters (offsets, diagonals, off diagonals)
float calc_mean_squared_residuals(param_t *params)
{
    VECTOR3 sample;
    
    if (_sample_buffer == NULL || _samples_collected == 0) {
        return 1.0e30f;
    }
    float sum = 0.0f;
    for (uint16_t i=0; i < _samples_collected; i++) {
        vector3_copy(&sample, &_sample_buffer[i]);
        float resid = calc_residual(&sample, params);
        sum += sq(resid);
    }
    sum /= _samples_collected;
    return sum;
}

// calculate initial offsets by simply taking the average values of the samples
void calc_initial_offset(void)
{
    // Set initial offset to the average value of the samples
    vector3_zero(&_params.offset);
    for (uint16_t k = 0; k < _samples_collected; k++) {
        _params.offset = vector3_sub(&_params.offset, &_sample_buffer[k]);
    }
    _params.offset = vector3_div(&_params.offset, _samples_collected);
}

void calc_sphere_jacob(VECTOR3 *sample, param_t *params, float* ret)
{
    VECTOR3 *offset = &params->offset;
    VECTOR3 *diag = &params->diag;
    VECTOR3 *offdiag = &params->offdiag;
    MATRIX3 softiron;
    VECTOR3 v3tmp;
    softiron = matrix3_new(
        diag->v[0]    , offdiag->v[0]  , offdiag->v[1],
        offdiag->v[0] , diag->v[1]     , offdiag->v[2],
        offdiag->v[1] , offdiag->v[2]  , diag->v[2]
    );

    float A =  (diag->v[0] * (sample->v[0] + offset->v[0])) + (offdiag->v[0] * (sample->v[1] + offset->v[1])) + (offdiag->v[1] * (sample->v[2] + offset->v[2]));
    float B =  (offdiag->v[0] * (sample->v[0] + offset->v[0])) + (diag->v[1]    * (sample->v[1] + offset->v[1])) + (offdiag->v[2] * (sample->v[2] + offset->v[2]));
    float C =  (offdiag->v[1] * (sample->v[0] + offset->v[0])) + (offdiag->v[2] * (sample->v[1] + offset->v[1])) + (diag->v[2]    * (sample->v[2] + offset->v[2]));

    v3tmp = vector3_add(sample, offset);
    v3tmp = matrix3_mul_vectro3(&softiron, &v3tmp);
    float length = vector3_length(&v3tmp);

    // 0: partial derivative (radius wrt fitness fn) fn operated on sample
    ret[0] = 1.0f;
    // 1-3: partial derivative (offsets wrt fitness fn) fn operated on sample
    ret[1] = -1.0f * (((diag->v[0]    * A) + (offdiag->v[0] * B) + (offdiag->v[1] * C))/length);
    ret[2] = -1.0f * (((offdiag->v[0] * A) + (diag->v[1]    * B) + (offdiag->v[2] * C))/length);
    ret[3] = -1.0f * (((offdiag->v[1] * A) + (offdiag->v[2] * B) + (diag->v[2]    * C))/length);
}

// run sphere fit to calculate diagonals and offdiagonals
void run_sphere_fit(void)
{
    VECTOR3 sample;
    if (_sample_buffer == NULL) {
        return;
    }

    const float lma_damping = 10.0f;

    // take backup of fitness and parameters so we can determine later if this fit has improved the calibration
    float fitness = _fitness;
    float fit1, fit2;
    param_t fit1_params, fit2_params;
    fit1_params = fit2_params = _params;

    float JTJ[COMPASS_CAL_NUM_SPHERE_PARAMS*COMPASS_CAL_NUM_SPHERE_PARAMS] = { };
    float JTJ2[COMPASS_CAL_NUM_SPHERE_PARAMS*COMPASS_CAL_NUM_SPHERE_PARAMS] = { };
    float JTFI[COMPASS_CAL_NUM_SPHERE_PARAMS] = { };

    // Gauss Newton Part common for all kind of extensions including LM
    for (uint16_t k = 0; k<_samples_collected; k++) {
        vector3_copy(&sample, &_sample_buffer[k]);

        float sphere_jacob[COMPASS_CAL_NUM_SPHERE_PARAMS];

        calc_sphere_jacob(&sample, &fit1_params, sphere_jacob);

        for (uint8_t i = 0;i < COMPASS_CAL_NUM_SPHERE_PARAMS; i++) {
            // compute JTJ
            for (uint8_t j = 0; j < COMPASS_CAL_NUM_SPHERE_PARAMS; j++) {
                JTJ[i*COMPASS_CAL_NUM_SPHERE_PARAMS+j] += sphere_jacob[i] * sphere_jacob[j];
                JTJ2[i*COMPASS_CAL_NUM_SPHERE_PARAMS+j] += sphere_jacob[i] * sphere_jacob[j];   //a backup JTJ for LM
            }
            // compute JTFI
            JTFI[i] += sphere_jacob[i] * calc_residual(&sample, &fit1_params);
        }
    }

    //------------------------Levenberg-Marquardt-part-starts-here---------------------------------//
    // refer: http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm#Choice_of_damping_parameter
    for (uint8_t i = 0; i < COMPASS_CAL_NUM_SPHERE_PARAMS; i++) {
        JTJ[i*COMPASS_CAL_NUM_SPHERE_PARAMS+i] += _sphere_lambda;
        JTJ2[i*COMPASS_CAL_NUM_SPHERE_PARAMS+i] += _sphere_lambda/lma_damping;
    }

    if (!inverse(JTJ, JTJ, 4)) {
        return;
    }

    if (!inverse(JTJ2, JTJ2, 4)) {
        return;
    }

    float *pfit1_radius = &fit1_params.radius;
    float *pfit2_radius = &fit2_params.radius;
    // extract radius, offset, diagonals and offdiagonal parameters
    for (uint8_t row=0; row < COMPASS_CAL_NUM_SPHERE_PARAMS; row++) {
        for (uint8_t col=0; col < COMPASS_CAL_NUM_SPHERE_PARAMS; col++) {
            pfit1_radius[row] -= JTFI[col] * JTJ[row*COMPASS_CAL_NUM_SPHERE_PARAMS+col];
            pfit2_radius[row] -= JTFI[col] * JTJ2[row*COMPASS_CAL_NUM_SPHERE_PARAMS+col];
        }
    }

    // calculate fitness of two possible sets of parameters
    fit1 = calc_mean_squared_residuals(&fit1_params);
    fit2 = calc_mean_squared_residuals(&fit2_params);

    // decide which of the two sets of parameters is best and store in fit1_params
    if (fit1 > _fitness && fit2 > _fitness) {
        // if neither set of parameters provided better results, increase lambda
        _sphere_lambda *= lma_damping;
    } else if (fit2 < _fitness && fit2 < fit1) {
        // if fit2 was better we will use it. decrease lambda
        _sphere_lambda /= lma_damping;
        fit1_params = fit2_params;
        fitness = fit2;
    } else if (fit1 < _fitness) {
        fitness = fit1;
    }
    //--------------------Levenberg-Marquardt-part-ends-here--------------------------------//

    // store new parameters and update fitness
    if (!isnan(fitness) && fitness < _fitness) {
        _fitness = fitness;
        _params = fit1_params;
        update_completion_mask_all();
    }
}

void calc_ellipsoid_jacob(VECTOR3 *sample, param_t *params, float* ret)
{
    VECTOR3 *offset = &params->offset;
    VECTOR3 *diag = &params->diag;
    VECTOR3 *offdiag = &params->offdiag;
    MATRIX3 softiron;
    VECTOR3 v3tmp;
    softiron = matrix3_new(
        diag->v[0]    , offdiag->v[0]  , offdiag->v[1],
        offdiag->v[0] , diag->v[1]     , offdiag->v[2],
        offdiag->v[1] , offdiag->v[2]  , diag->v[2]
    );

    float A =  (diag->v[0] * (sample->v[0] + offset->v[0])) + (offdiag->v[0] * (sample->v[1] + offset->v[1])) + (offdiag->v[1] * (sample->v[2] + offset->v[2]));
    float B =  (offdiag->v[0] * (sample->v[0] + offset->v[0])) + (diag->v[1]    * (sample->v[1] + offset->v[1])) + (offdiag->v[2] * (sample->v[2] + offset->v[2]));
    float C =  (offdiag->v[1] * (sample->v[0] + offset->v[0])) + (offdiag->v[2] * (sample->v[1] + offset->v[1])) + (diag->v[2]    * (sample->v[2] + offset->v[2]));

    v3tmp = vector3_add(sample, offset);
    v3tmp = matrix3_mul_vectro3(&softiron, &v3tmp);
    float length = vector3_length(&v3tmp);

    // 0-2: partial derivative (offset wrt fitness fn) fn operated on sample
    ret[0] = -1.0f * (((diag->v[0]    * A) + (offdiag->v[0] * B) + (offdiag->v[1] * C))/length);
    ret[1] = -1.0f * (((offdiag->v[0] * A) + (diag->v[1]    * B) + (offdiag->v[2] * C))/length);
    ret[2] = -1.0f * (((offdiag->v[1] * A) + (offdiag->v[2] * B) + (diag->v[2]    * C))/length);
    // 3-5: partial derivative (diag offset wrt fitness fn) fn operated on sample
    ret[3] = -1.0f * ((sample->v[0] + offset->v[0]) * A)/length;
    ret[4] = -1.0f * ((sample->v[1] + offset->v[1]) * B)/length;
    ret[5] = -1.0f * ((sample->v[2] + offset->v[2]) * C)/length;
    // 6-8: partial derivative (off-diag offset wrt fitness fn) fn operated on sample
    ret[6] = -1.0f * (((sample->v[1] + offset->v[1]) * A) + ((sample->v[0] + offset->v[0]) * B))/length;
    ret[7] = -1.0f * (((sample->v[2] + offset->v[2]) * A) + ((sample->v[0] + offset->v[0]) * C))/length;
    ret[8] = -1.0f * (((sample->v[2] + offset->v[2]) * B) + ((sample->v[1] + offset->v[1]) * C))/length;
}

void run_ellipsoid_fit(void)
{
    VECTOR3 sample;
    if (_sample_buffer == NULL) {
        return;
    }

    const float lma_damping = 10.0f;

    // take backup of fitness and parameters so we can determine later if this fit has improved the calibration
    float fitness = _fitness;
    float fit1, fit2;
    param_t fit1_params, fit2_params;
    fit1_params = fit2_params = _params;

    float JTJ[COMPASS_CAL_NUM_ELLIPSOID_PARAMS*COMPASS_CAL_NUM_ELLIPSOID_PARAMS] = { };
    float JTJ2[COMPASS_CAL_NUM_ELLIPSOID_PARAMS*COMPASS_CAL_NUM_ELLIPSOID_PARAMS] = { };
    float JTFI[COMPASS_CAL_NUM_ELLIPSOID_PARAMS] = { };

    // Gauss Newton Part common for all kind of extensions including LM
    for (uint16_t k = 0; k<_samples_collected; k++) {
        vector3_copy(&sample, &_sample_buffer[k]);

        float ellipsoid_jacob[COMPASS_CAL_NUM_ELLIPSOID_PARAMS];

        calc_ellipsoid_jacob(&sample, &fit1_params, ellipsoid_jacob);

        for (uint8_t i = 0;i < COMPASS_CAL_NUM_ELLIPSOID_PARAMS; i++) {
            // compute JTJ
            for (uint8_t j = 0; j < COMPASS_CAL_NUM_ELLIPSOID_PARAMS; j++) {
                JTJ [i*COMPASS_CAL_NUM_ELLIPSOID_PARAMS+j] += ellipsoid_jacob[i] * ellipsoid_jacob[j];
                JTJ2[i*COMPASS_CAL_NUM_ELLIPSOID_PARAMS+j] += ellipsoid_jacob[i] * ellipsoid_jacob[j];
            }
            // compute JTFI
            JTFI[i] += ellipsoid_jacob[i] * calc_residual(&sample, &fit1_params);
        }
    }

    //------------------------Levenberg-Marquardt-part-starts-here---------------------------------//
    //refer: http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm#Choice_of_damping_parameter
    for (uint8_t i = 0; i < COMPASS_CAL_NUM_ELLIPSOID_PARAMS; i++) {
        JTJ[i*COMPASS_CAL_NUM_ELLIPSOID_PARAMS+i] += _ellipsoid_lambda;
        JTJ2[i*COMPASS_CAL_NUM_ELLIPSOID_PARAMS+i] += _ellipsoid_lambda/lma_damping;
    }

    if (!inverse(JTJ, JTJ, 9)) {
        return;
    }

    if (!inverse(JTJ2, JTJ2, 9)) {
        return;
    }

    float *pfit1_offset = (float *)&fit1_params.offset;
    float *pfit2_offset = (float *)&fit2_params.offset;
    // extract radius, offset, diagonals and offdiagonal parameters
    for (uint8_t row=0; row < COMPASS_CAL_NUM_ELLIPSOID_PARAMS; row++) {
        for (uint8_t col=0; col < COMPASS_CAL_NUM_ELLIPSOID_PARAMS; col++) {
            pfit1_offset[row] -= JTFI[col] * JTJ[row*COMPASS_CAL_NUM_ELLIPSOID_PARAMS+col];
            pfit2_offset[row] -= JTFI[col] * JTJ2[row*COMPASS_CAL_NUM_ELLIPSOID_PARAMS+col];
        }
    }

    // calculate fitness of two possible sets of parameters
    fit1 = calc_mean_squared_residuals(&fit1_params);
    fit2 = calc_mean_squared_residuals(&fit2_params);

    // decide which of the two sets of parameters is best and store in fit1_params
    if (fit1 > _fitness && fit2 > _fitness) {
        // if neither set of parameters provided better results, increase lambda
        _ellipsoid_lambda *= lma_damping;
    } else if (fit2 < _fitness && fit2 < fit1) {
        // if fit2 was better we will use it. decrease lambda
        _ellipsoid_lambda /= lma_damping;
        fit1_params = fit2_params;
        fitness = fit2;
    } else if (fit1 < _fitness) {
        fitness = fit1;
    }
    //--------------------Levenberg-part-ends-here--------------------------------//

    // store new parameters and update fitness
    if (fitness < _fitness) {
        _fitness = fitness;
        _params = fit1_params;
        update_completion_mask_all();
    }
}

// initialize fitness before starting a fit
void initialize_fit(void)
{
    if (_samples_collected != 0) {
        _fitness = calc_mean_squared_residuals(&_params);
    } else {
        _fitness = 1.0e30f;
    }
    _initial_fitness = _fitness;
    _sphere_lambda = 1.0f;
    _ellipsoid_lambda = 1.0f;
    _fit_step = 0;
}

void reset_state(void)
{
    _samples_collected = 0;
    _samples_thinned = 0;
    _params.radius = 200;
    vector3_zero(&_params.offset);
    vector3_set(&_params.diag, 1.0f, 1.0f, 1.0f);
    vector3_zero(&_params.offdiag);
    _params.scale_factor = 0;

    initialize_fit();
}

bool set_status(enum Status status)
{
    if (status != NOT_STARTED && _status == status) {
        return true;
    }

    switch (status) {
        case NOT_STARTED:
            reset_state();
            _status = NOT_STARTED;
            if (_sample_buffer != NULL) {
                free(_sample_buffer);
                _sample_buffer = NULL;
            }
            return true;

        case WAITING_TO_START:
            reset_state();
            _status = WAITING_TO_START;
            set_status(RUNNING_STEP_ONE);
            return true;

        case RUNNING_STEP_ONE:
            if (_status != WAITING_TO_START) {
                return false;
            }

            // on first attempt delay start if requested by caller
            if (_attempt == 1 && (millis()-_start_time_ms)*1.0e-3f < _delay_start_sec) {
                return false;
            }

            if (_sample_buffer == NULL) {
                _sample_buffer = (VECTOR3*)calloc(COMPASS_CAL_NUM_SAMPLES, sizeof(VECTOR3));
            }
            if (_sample_buffer != NULL) {
                initialize_fit();
                _status = RUNNING_STEP_ONE;
                return true;
            }
            return false;

        case RUNNING_STEP_TWO:
            if (_status != RUNNING_STEP_ONE) {
                return false;
            }
            thin_samples();
            initialize_fit();
            _status = RUNNING_STEP_TWO;
            return true;

        case CC_SUCCESS:
            if (_status != RUNNING_STEP_TWO) {
                return false;
            }

            if (_sample_buffer != NULL) {
                free(_sample_buffer);
                _sample_buffer = NULL;
            }

            _status = CC_SUCCESS;
            return true;

        case FAILED:
        case BAD_ORIENTATION:
        case BAD_RADIUS:
            if ((status == FAILED) 
                && (_status == BAD_ORIENTATION || _status == BAD_RADIUS)) {
                // don't overwrite bad orientation status
                return false;
            }
                
            if (_status == NOT_STARTED) {
                return false;
            }

            if (_retry && set_status(WAITING_TO_START)) {
                _attempt++;
                return true;
            }

            if (_sample_buffer != NULL) {
                free(_sample_buffer);
                _sample_buffer = NULL;
            }

            _status = status;
            return true;

        default:
            return false;
    };
}

/*
  fix radius of the fit to compensate for sensor scale factor errors
  return false if radius is outside acceptable range
 */
bool fix_radius(void)
{
    _params.scale_factor = 0;
    return true;
#if 0
    if (AP::gps().status() < AP_GPS::GPS_OK_FIX_2D) {
        // we don't have a position, leave scale factor as 0. This
        // will disable use of WMM in the EKF. Users can manually set
        // scale factor after calibration if it is known
        _params.scale_factor = 0;
        return true;
    }
    const struct Location &loc = AP::gps().location();
    float intensity;
    float declination;
    float inclination;
    AP_Declination::get_mag_field_ef(loc.lat * 1e-7f, loc.lng * 1e-7f, intensity, declination, inclination);

    float expected_radius = intensity * 1000; // mGauss
    float correction = expected_radius / _params.radius;

    if (correction > COMPASS_MAX_SCALE_FACTOR || correction < COMPASS_MIN_SCALE_FACTOR) {
        // don't allow more than 30% scale factor correction
        gcs().send_text(MAV_SEVERITY_ERROR, "Mag(%u) bad radius %.0f expected %.0f",
                        _compass_idx,
                        _params.radius,
                        expected_radius);
        set_status(Status::BAD_RADIUS);
        return false;
    }

    _params.scale_factor = correction;

    return true;
#endif
}

void _cmps_clbrt_update(bool *failure)
{
    *failure = false;

    // collect the minimum number of samples
    if (!fitting()) {
        return;
    }

    if (_status == RUNNING_STEP_ONE) {
        if (_fit_step >= 10) {
            if (is_equal(_fitness, _initial_fitness) || isnan(_fitness)) {  // if true, means that fitness is diverging instead of converging
                set_status(FAILED);
                *failure = true;
            } else {
                set_status(RUNNING_STEP_TWO);
            }
        } else {
            if (_fit_step == 0) {
                calc_initial_offset();
            }
            run_sphere_fit();
            _fit_step++;
        }
    } else if (_status == RUNNING_STEP_TWO) {
        if (_fit_step >= 35) {
            if (fit_acceptable() && fix_radius() && calculate_orientation()) {
                set_status(CC_SUCCESS);
            } else {
                set_status(FAILED);
                *failure = true;
            }
        } else if (_fit_step < 15) {
            run_sphere_fit();
            _fit_step++;
        } else {
            run_ellipsoid_fit();
            _fit_step++;
        }
    }
}

bool fit_acceptable(void)
{
    if (!isnan(_fitness) &&
        _params.radius > FIELD_RADIUS_MIN && _params.radius < FIELD_RADIUS_MAX &&
        fabsf(_params.offset.v[0]) < _offset_max &&
        fabsf(_params.offset.v[1]) < _offset_max &&
        fabsf(_params.offset.v[2]) < _offset_max &&
        _params.diag.v[0] > 0.2f && _params.diag.v[0] < 5.0f &&
        _params.diag.v[1] > 0.2f && _params.diag.v[1] < 5.0f &&
        _params.diag.v[2] > 0.2f && _params.diag.v[2] < 5.0f &&
        fabsf(_params.offdiag.v[0]) < 1.0f &&      //absolute of sine/cosine output cannot be greater than 1
        fabsf(_params.offdiag.v[1]) < 1.0f &&
        fabsf(_params.offdiag.v[2]) < 1.0f ) {
            return _fitness <= sq(_tolerance);
        }
    return false;
}


bool calculate_orientation(void)
{
    return true;
}

void cmps_clbrt_new_sample(VECTOR3 *sample)
{
    _last_sample_ms = millis();

    if (_status == WAITING_TO_START) {
        set_status(RUNNING_STEP_ONE);
    }

    if (running() && _samples_collected < COMPASS_CAL_NUM_SAMPLES && accept_sample(sample, UINT16_MAX)) {
        update_completion_mask(sample);
        vector3_copy(&_sample_buffer[_samples_collected], sample);
        //_sample_buffer[_samples_collected].att.set_from_ahrs();
        _samples_collected++;
    }
}

void cmps_clbrt_start(bool retry, float delay, uint16_t offset_max)
{
    if (running()) {
        return;
    }
    _offset_max = offset_max;
    _attempt = 1;
    _retry = retry;
    _delay_start_sec = delay;
    _start_time_ms = millis();
    set_status(WAITING_TO_START);
}


void cmps_clbrt_stop(void)
{
    set_status(NOT_STARTED);
}

void cmps_clbrt_update(void)
{
    bool failure;
    _cmps_clbrt_update(&failure);
}


void calibrator_show(void)
{
    printk("--------------compass calibrator-----------------\r\n");
    printk("status: %d. samples: %d thin samples:%d. \r\n", _status, _samples_collected, _samples_thinned);
    printk("mask:");
    for (uint8_t i = 0; i < 10; i++) {
        printk("%02x ", _completion_mask[i]);
    }
    
    printk("fit step:%d \r\n", _fit_step);
    printk("params:r(%f) offset(%f, %f, %f) diag(%f, %f, %f) offdiag(%f, %f, %f) scale factor(%f).\r\n",
        _params.radius, _params.offset.v[0], _params.offset.v[1], _params.offset.v[2], 
        _params.diag.v[0], _params.diag.v[1], _params.diag.v[2], 
        _params.offdiag.v[0], _params.offdiag.v[1], _params.offdiag.v[2], _params.scale_factor);
    printk("fitness %f initial fitness %f sphere_lambda %f ellipsoid_lambda %f.\r\n", 
        _fitness, _initial_fitness, _sphere_lambda, _ellipsoid_lambda);
    printk("-------------------------------------------------\r\n");
}

void compass_calibrator_cmd(BaseSequentialStream *chp, int argc, char *argv[])
{
	int32_t cmd_type;

	if (argc != 1) {
		shellUsage(chp, "cpcl [show | start | stop]");
		return;
	} else {
		if (strcmp(argv[0], "show") == 0) {
			cmd_type = 1;
		} else if (strcmp(argv[0], "start") == 0){
			cmd_type = 2;
		} else if (strcmp(argv[0], "stop") == 0){
            cmd_type = 3;
        } else {
			cmd_type = -1;
			shellUsage(chp, "cpcl [show | start | stop]");
			return;
		}
	}

    if (cmd_type == 1) {
        calibrator_show();
    } else if (cmd_type == 2) {
        cmps_clbrt_start(false, 10, 2200);
    } else if (cmd_type == 3) {
        cmps_clbrt_stop();
    }

    return;
}

