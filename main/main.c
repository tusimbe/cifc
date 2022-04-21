/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include "cmsis_os.h"
#include "chprintf.h"
#include "shell.h"
#include "usbcfg.h"
#include "scheduler.h"
#include "mpu6000.h"
#include "qmc5883l.h"
#include "compass.h"
#include "fram.h"
#include "lfs.h"
#include "params.h"
#include "compass_calibrator.h"
#include "cli.h"
#include <stdio.h>
/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/
static void _init(void) 
{
	/*
	 * System initializations.
	 * - HAL initialization, this also initializes the configured device drivers
	 *	 and performs the board-specific initializations.
	 * - Kernel initialization, the main() function becomes a thread and the
	 *	 RTOS is active.
	 */

    sdStart(&SD8, NULL);
	/*
	 * Shell manager initialization.
	 */
	shellInit();
	
	/*
	 * Initializes a serial-over-USB CDC driver.
	 */
	sduObjectInit(&SDU1);
	sduStart(&SDU1, &serusbcfg);
	
	/*
	 * Activates the USB driver and then the USB bus pull-up on D+.
	 * Note, a delay is inserted in order to not have to disconnect the cable
	 * after a reset.
	 */
	usbDisconnectBus(serusbcfg.usbp);
	chThdSleepMilliseconds(1000);
	usbStart(serusbcfg.usbp, &usbcfg);
	usbConnectBus(serusbcfg.usbp);

    /* pramaters init */
    cmps_params_register();
    fram_init(1);
    params_init();

    scheduler_init();

    /* driver init */
    mpu6k_init(0);
    qmc5883l_init(0);
    sdcard_init();
    
    /* core init */
    compass_init();
}

/*===========================================================================*/
/* Initialization and main thread.                                           */
/*===========================================================================*/

/*
 * Application entry point.
 */
int main(void) 
{
	_init();

	scheduler_set_main_thread(chThdGetSelfX());

	/*
	 *  Normal main() thread activity, spawning shells.
	 */
  	while (true) {
		cli_ThdCreate();
		scheduler_loop();

		if (!scheduler_check_called_boost()) {
            scheduler_delay_microseconds(50);
        }
        //scheduler_delay_microseconds(100);
  	}

	return 0;
}
