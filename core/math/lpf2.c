/*!
 *****************************************************************************
 *
 *  Copyright © 2016-2022 CHASING INNOVATION . All Rights Reserved.
 *
 * \file      lpf2.c
 * \author    johnson@chasing-innovation.com
 * \version   1.0
 * \date      2022年4月7日星期四
 * \brief     2阶低通滤波器
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
#include "base.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

typedef struct biquad_params_s {
    float cutoff_freq;
    float sample_freq;
    float a1;
    float a2;
    float b0;
    float b1;
    float b2;
    float _delay_element_1;
    float _delay_element_2;
} biquad_params_t;

float lpf2_apply(void *p, const float sample)
{
    biquad_params_t *params = p;

    if (NULL  == p) {
        return sample;
    }
 
    if(is_zero(params->cutoff_freq) || is_zero(params->sample_freq)) {
        return sample;
    }

    float delay_element_0 = sample - params->_delay_element_1 * params->a1 - params->_delay_element_2 * params->a2;
    float output = delay_element_0 * params->b0 + params->_delay_element_1 * params->b1 + params->_delay_element_2 * params->b2;

    params->_delay_element_2 = params->_delay_element_1;
    params->_delay_element_1 = delay_element_0;

    return output;

}

void lpf2_reset(void *p)
{
    biquad_params_t *para = p;
    
    if (NULL == para) {
        return;
    }
    
    para->_delay_element_1 = 0;
    para->_delay_element_2 = 0;
    return;
}

static void _compute_params(float sample_freq, float cutoff_freq, biquad_params_t *ret)
{
    ret->cutoff_freq = cutoff_freq;
    ret->sample_freq = sample_freq;
    if (!is_positive(ret->cutoff_freq)) {
        // zero cutoff means pass-thru
        return;
    }

    float fr = sample_freq/cutoff_freq;
    float ohm = tanf(M_PI/fr);
    float c = 1.0f+2.0f*cosf(M_PI/4.0f)*ohm + ohm*ohm;

    ret->b0 = ohm*ohm/c;
    ret->b1 = 2.0f*ret->b0;
    ret->b2 = ret->b0;
    ret->a1 = 2.0f*(ohm*ohm-1.0f)/c;
    ret->a2 = (1.0f-2.0f*cosf(M_PI/4.0f)*ohm+ohm*ohm)/c;

}

void *lpf2_create(float sample_freq, float cutoff_freq)
{
    biquad_params_t *p;

    p = malloc(sizeof(biquad_params_t));
    if (NULL == p) {
        return NULL;
    }

    memset(p, 0, sizeof(biquad_params_t));
    _compute_params(sample_freq, cutoff_freq, p);

    return p;
}

void lpf2_destory(void *p)
{
    if (NULL != p) {
        free(p);
    }
    return;
}

