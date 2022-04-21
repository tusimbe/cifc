/*!
 *****************************************************************************
 *
 *  Copyright © 2017-2018 CHASING INNOVATION. All Rights Reserved.
 *
 * \file      cli.c
 * \author    johnson@chasing-innovation.com
 * \version   1.0
 * \date      2022年4月19日星期二
 * \brief     cli命令行工具
 *
 *----------------------------------------------------------------------------
 * \attention
 *
 *
 *****************************************************************************
 */

/*****************************************************************************
 change history: 
    1.date  : 2022年4月19日星期二
      author: johnson@chasing-innovation.com
      change: create file

*****************************************************************************/
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "system.h"
#include "scheduler.h"
#include <stdio.h>

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

extern void sche_cmd(BaseSequentialStream *chp, int argc, char *argv[]);
extern void perf_cmd(BaseSequentialStream *chp, int argc, char *argv[]);
extern void mpu6k_cmd(BaseSequentialStream *chp, int argc, char *argv[]);
extern void qmc5883l_cmd(BaseSequentialStream *chp, int argc, char *argv[]);
extern void imu_cmd(BaseSequentialStream *chp, int argc, char *argv[]);
extern void compass_calibrator_cmd(BaseSequentialStream *chp, int argc, char *argv[]);
extern void fram_cmd(BaseSequentialStream *chp, int argc, char *argv[]);
extern void lfs_cmd(BaseSequentialStream *chp, int argc, char *argv[]);
extern void param_cmd(BaseSequentialStream *chp, int argc, char *argv[]);
extern void sdcard_cmd(BaseSequentialStream *chp, int argc, char *argv[]);



#ifndef UNIT_TEST
static const ShellCommand commands[] = {
	{"scher", sche_cmd},
	{"perf", perf_cmd},
    {"mpu6k", mpu6k_cmd},
    {"qmc", qmc5883l_cmd},
    {"imu", imu_cmd},
    {"cpcl", compass_calibrator_cmd},
    {"fram", fram_cmd},
    {"lfs", lfs_cmd},
    {"param", param_cmd},
    {"sd", sdcard_cmd},   
	{NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&SD8,
  commands
};
#endif

void cli_ThdCreate(void)
{
#ifndef UNIT_TEST
	static bool isCreate = false;
    if (!isCreate) {
		thread_t *shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
		                                      "shell", SCHE_SHELL_PRIORITY,
		                                      shellThread, (void *)&shell_cfg1);
		if(shelltp != NULL) {
			isCreate = true;
		} else {
			printf("shell thread create failed!\r\n");
		}
    }

	return;
#endif
}

