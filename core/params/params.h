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

enum param_module {
    param_module_compass = 1,
};

enum param_type {
    param_type_int = 0,
    param_type_float,
    param_type_invalid = -1,
};

typedef union param_data {
    int i;
    float f;
} PARAM_DATA;

typedef struct param_lst {
    char *name;
    uint16_t type;
    PARAM_DATA val;
} PARAM_LST;

typedef struct param_entry_s {
    char *entry_name;
    enum param_type type;
    PARAM_DATA data;
} PARAM_ENTRY_T;

/* modules call back interfaces */
typedef uint32_t (*param_numbers_get)(uint32_t moduleid);
typedef int32_t (*param_default_load)(uint32_t moduleid, uint32_t paramid, PARAM_ENTRY_T *data);

/* 参数管理模块对外接口 */
int param_cb_register(uint16_t moduleid, char *name, param_numbers_get func_get, param_default_load func_load);


int params_init(void);
int params_update(char *name, int32_t type, PARAM_DATA *data);
int params_get(char *name, int32_t type, PARAM_DATA *val);
void param_cmd(BaseSequentialStream *chp, int argc, char *argv[]);


#endif
