/*!
 *****************************************************************************
 *
 *  Copyright © 2017-2018 CHASING INNOVATION. All Rights Reserved.
 *
 * \file      params.h
 * \author    johnson@chasing-innovation.com
 * \version   1.0
 * \date      2022年4月16日星期六
 * \brief     参数管理
 *
 *----------------------------------------------------------------------------
 * \attention
 *
 *
 *****************************************************************************
 */

/*****************************************************************************
 change history: 
    1.date  : 2022年4月16日星期六
      author: johnson@chasing-innovation.com
      change: create file

*****************************************************************************/
#ifndef _PARAMS_H_
#define _PARAMS_H_
#include "vector3.h"

enum param_module {
    param_module_compass = 1,
};

enum param_type {
    param_type_int = 0,
    param_type_float,
    param_type_vector3,
};

typedef union param_data {
    int i;
    float f;
    VECTOR3 v;
} PARAM_DATA;
    
typedef struct param_entry_s {
    uint16_t paramId;
    char *entry_name;
    enum param_type type;
    PARAM_DATA data;
} PARAM_ENTRY_T;

/* modules call back interfaces */
typedef uint32_t (*param_numbers_get)(uint32_t moduleid);
typedef uint32_t (*param_default_load)(uint32_t moduleid, uint32_t paramid, PARAM_ENTRY_T *data);

/* 参数管理模块对外接口 */
uint32_t param_cb_register(uint32_t moduleid, param_numbers_get func_get, param_default_load func_load);
uint32_t param_data_find(uint32_t moduleid, char *name, PARAM_ENTRY_T *data);
uint32_t param_data_set(uint32_t moduleid, char *name, PARAM_ENTRY_T *data);

int params_init(void);
void param_cmd(BaseSequentialStream *chp, int argc, char *argv[]);


#endif
