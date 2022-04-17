/*!
 *****************************************************************************
 *
 *  Copyright © 2016-2022 CHASING INNOVATION . All Rights Reserved.
 *
 * \file      qmc5883l.c
 * \author    johnson@chasing-innovation.com
 * \version   1.0
 * \date      2022年4月6日星期三
 * \brief     qmc5883l驱动
 *
 *----------------------------------------------------------------------------
 * \attention
 *
 *
 *****************************************************************************
 */

/*****************************************************************************
 change history: 
    1.date  : 2022年4月6日星期三
      author: johnson@chasing-innovation.com
      change: create file

*****************************************************************************/
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "usbcfg.h"
#include "system.h"
#include "shell.h"
#include "scheduler.h"
#include "vector3.h"
#include "drv_i2c.h"
#include "qmc5883l.h"
#include "compass.h"
#include <string.h>
#include <stdlib.h>

#define QMC5883L_I2C_ADDR 0x0D

#define QMC5883L_REG_CONF1 0x09
#define QMC5883L_REG_CONF2 0x0A

// data output rates for 5883L
#define QMC5883L_ODR_10HZ (0x00 << 2)
#define QMC5883L_ODR_50HZ  (0x01 << 2)
#define QMC5883L_ODR_100HZ (0x02 << 2)
#define QMC5883L_ODR_200HZ (0x03 << 2)

// Sensor operation modes
#define QMC5883L_MODE_STANDBY 0x00
#define QMC5883L_MODE_CONTINUOUS 0x01

#define QMC5883L_RNG_2G (0x00 << 4)
#define QMC5883L_RNG_8G (0x01 << 4)

#define QMC5883L_OSR_512 (0x00 << 6)
#define QMC5883L_OSR_256 (0x01 << 6)
#define QMC5883L_OSR_128	(0x10 << 6)
#define QMC5883L_OSR_64	(0x11	<< 6)

#define QMC5883L_RST 0x80

#define QMC5883L_REG_DATA_OUTPUT_X 0x00
#define QMC5883L_REG_STATUS 0x06

#define QMC5883L_REG_ID 0x0D
#define QMC5883_ID_VAL 0xFF

static int qmc_debug;

static void *g_drvp_i2c;

static bool _check_whoami(void);
void _timer(void);


int qmc5883l_init(uint32_t busid)
{
    g_drvp_i2c = drv_i2c_create(busid, false, QMC5883L_I2C_ADDR, 3);
    if (!g_drvp_i2c) {
        printk("[%s, %d] spi driver create failed.\r\n", __func__, __LINE__);
        return -1;
    }

    drv_i2c_start(g_drvp_i2c);

    if (!_check_whoami()) {
        printk("[%s, %d] error chi ip.\r\n", __func__, __LINE__);
        return -1;
    }

    if (!drv_i2c_write_register(g_drvp_i2c, 0x0B, 0x01) ||
    	 !drv_i2c_write_register(g_drvp_i2c, 0x20, 0x40) ||
		 !drv_i2c_write_register(g_drvp_i2c, 0x21, 0x01) ||
		 !drv_i2c_write_register(g_drvp_i2c, QMC5883L_REG_CONF1,
						QMC5883L_MODE_CONTINUOUS|
						QMC5883L_ODR_100HZ|
						QMC5883L_OSR_512|
						QMC5883L_RNG_8G)) {
        printk("[%s, %d] config qmc5883l failed.\r\n", __func__, __LINE__);
        return -1;
    }

    return 0;
}

static bool _check_whoami(void)
{
    uint8_t val;

    (void)drv_i2c_read_register(g_drvp_i2c, 0x00);
    val = drv_i2c_read_register(g_drvp_i2c, 0x0C);
    if (val != 0x01) {
        return false;
    }
    val = drv_i2c_read_register(g_drvp_i2c, 0x0D);
    if (val != 0xff) {
        return false;
    }
    
    return true;
}

void qmc5883l_cmd(BaseSequentialStream *chp, int argc, char *argv[])
{
    int v;
	int32_t cmd_type;
	uint8_t chip_id;

    if (argc > 2) {
        shellUsage(chp, "qmc [debug | id | init] [level]");
        return;
    } else {
        
    }

    if (argc == 1) {
		if (strcmp(argv[0], "id") == 0) {
			cmd_type = 1;
		} else if (strcmp(argv[0], "init") == 0){
			cmd_type = 2;
		} else if (strcmp(argv[0], "fifo") == 0){
			cmd_type = 3;
		} else {
			cmd_type = -1;
			shellUsage(chp, "qmc [id | init | fifo]");
			return;
		}
    } else if (argc == 2){
    	if (strcmp(argv[0], "debug") == 0)
    	{
    		v = atoi(argv[1]);
    		qmc_debug = v;
    	} else {
            printk("unknown cmd.\r\n");
        }

        return;
    }
    

    if (cmd_type == 1) {
        chip_id = drv_i2c_read_register(g_drvp_i2c, QMC5883L_REG_ID);
        printk("qmc5883l init chip id:0x%x.\r\n", chip_id); 

    } else if (cmd_type == 2) {
        hal_compass_timer();
    }

    return;
}

void hal_compass_timer(void)
{
    uint8_t buf[6];
    int16_t magData[3];

    const float range_scale = 1000.0f / 3000.0f;

    uint8_t status;
    if(!drv_i2c_read_registers(g_drvp_i2c, QMC5883L_REG_STATUS, &status, 1)){
    	return;
    }

    //new data is ready
    if (!(status & 0x04)) {
    	return;
    }

    if(!drv_i2c_read_registers(g_drvp_i2c, QMC5883L_REG_DATA_OUTPUT_X, buf, sizeof(buf))) {
        return;
    }

    magData[0] = -((int16_t)(buf[1] << 8 | buf[0]));
    magData[1] = (int16_t)(buf[3] << 8 | buf[2]);
    magData[2] = -((int16_t)(buf[5] << 8 | buf[4]));


    if (qmc_debug == 1) {
        printk("mag(%05d %05d %05d)\r", magData[0], magData[1], magData[2]);
    }

    VECTOR3 field;
    field.v[0] = magData[0] * range_scale;
    field.v[1] = magData[1] * range_scale;
    field.v[2] = magData[2] * range_scale;
    
    if (qmc_debug == 2) {
        float radius = vector3_length(&field);
        printk("mag(%04.4f %04.4f %04.4f) r:%04.04f\r", field.v[0], field.v[1], field.v[2], radius);
    }    
    compass_notify_new_data(&field);
}


