/*!
 *****************************************************************************
 *
 *  Copyright © 2017-2018 CHASING INNOVATION. All Rights Reserved.
 *
 * \file      sdcard.c
 * \author    johnson@chasing-innovation.com
 * \version   1.0
 * \date      2022年4月21日星期四
 * \brief     SD卡驱动
 *
 *----------------------------------------------------------------------------
 * \attention
 *
 *
 *****************************************************************************
 */

/*****************************************************************************
 change history: 
    1.date  : 2022年4月21日星期四
      author: johnson@chasing-innovation.com
      change: create file

*****************************************************************************/
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "ff.h"
#include "shell.h"
#include "sdcard.h"
#include <stdio.h>
#include <string.h>

static FATFS SDC_FS; // FATFS object
FIL testFile;
static bool sdcard_running;
char *test_string = "Hello, world!!";

int sdcard_init(void)
{
    uint8_t i, tries = 3;
    uint32_t testBytes;
    FRESULT ret;
    for (i = 0; i < tries; i++) {
        printf("start SDCard %d times.\r\n", i);
        sdcStart(&SDCD1, NULL);
        printf("connecting SDCard.\r\n");
        if(sdcConnect(&SDCD1) == HAL_FAILED) {
            sdcStop(&SDCD1);
            continue;
        }
        printf("mount SDCard.\r\n");
        ret = f_mount(&SDC_FS, "0:", 1);
        if (ret != FR_OK) {
            sdcDisconnect(&SDCD1);
            sdcStop(&SDCD1);
            printf("f_mount return %d.\r\n", ret);
            continue;
        }
        printf("Successfully mounted SDCard.\r\n");

        // Create CHASING Directory if needed
        f_mkdir("/CHASING");
        sdcard_running = true;
        return 0;
    }


    
    sdcard_running = false;
    printf("mounted SDCard failed.\r\n");
    return -1;
}


#define  sd_cmd_usage()                    \
    shellUsage(chp, "sd init\r\n")
    
void sdcard_cmd(BaseSequentialStream *chp, int argc, char *argv[])
{
    int cmd_type;
    
	if (argc > 3) {
        sd_cmd_usage();
		return;
	} else if (argc == 1) {
		if (strcmp(argv[0], "show") == 0) {
			cmd_type = 2;
		} else if (strcmp(argv[0], "init") == 0) {
			cmd_type = 3;
		} else if (strcmp(argv[0], "save") == 0) {
            cmd_type = 4;
        } else if (strcmp(argv[0], "format") == 0) {
            cmd_type = 8;
        } else {
			cmd_type = -1;
            sd_cmd_usage();
			return;
		}
	}

    if (cmd_type == 3) {
        sdcard_init();
    }
}
