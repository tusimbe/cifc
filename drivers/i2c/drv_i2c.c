/*!
 *****************************************************************************
 *
 *  Copyright © 2016-2022 CHASING INNOVATION . All Rights Reserved.
 *
 * \file      drv_i2c.c
 * \author    johnson@chasing-innovation.com
 * \version   1.0
 * \date      2022年4月6日星期三
 * \brief     i2c驱动抽象层
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
#include "drv_i2c.h"
#include <stdlib.h>
#include <string.h>

#define DRV_I2C_MAX_CLOCK  (100000)


static I2CDriver *drv_i2c_table[] = 
{
    /** @brief I2C1 driver identifier.*/
#if STM32_I2C_USE_I2C1
    &I2CD1,
#endif
    
    /** @brief I2C2 driver identifier.*/
#if STM32_I2C_USE_I2C2
    &I2CD2,
#endif
    
    /** @brief I2C3 driver identifier.*/
#if STM32_I2C_USE_I2C3
    &I2CD3,
#endif
};

static uint32_t  drv_i2c_size = ARRAY_SIZE(drv_i2c_table);

I2CConfig g_i2c_config;

typedef struct {
    uint32_t busid;
    uint32_t retries;
    uint8_t devAddr;
    bool started;
    bool smbus;
    bool (*transfer)(void *p, const uint8_t * send, 
                uint32_t send_len, uint8_t * recv, uint32_t len);
} drv_i2c_vmt;


static bool _do_transfer(void *p, const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len);

static bool _transfer(void *p, const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len);


void* drv_i2c_create(uint32_t busid, bool smbus, uint8_t devAddr, uint32_t retries)
{
    if (busid >= drv_i2c_size) {
        printk("[%s, %d] busid %d is bigger than bus table size %d.\r\n", 
                                    __func__, __LINE__, busid, drv_i2c_size);
        return false;
    }

    drv_i2c_vmt *p = malloc(sizeof(drv_i2c_vmt));
    if (!p) {
        printk("[%s, %d] no memory.\r\n", __func__, __LINE__);
        return NULL;
    }
    
    p->busid = busid;
    p->started = false;
    p->smbus = smbus;
    p->devAddr = devAddr;
    p->retries = retries;
    p->transfer = _do_transfer;

    return p;
}

void drv_i2c_start(void *p)
{
    drv_i2c_vmt *i2cp = p;

    if (i2cp->started) {
        return;
    }

    if (i2cp->busid >= drv_i2c_size) {
        printk("[%s, %d] busid %d is bigger than bus table size %d.\r\n", 
                                    __func__, __LINE__, i2cp->busid, drv_i2c_size);
        return;
    }

    I2CDriver *drv = drv_i2c_table[i2cp->busid];
    
    g_i2c_config.clock_speed = DRV_I2C_MAX_CLOCK;
    if (g_i2c_config.clock_speed <= 100000) {
        g_i2c_config.duty_cycle = STD_DUTY_CYCLE;
    } else {
        g_i2c_config.duty_cycle = FAST_DUTY_CYCLE_2;
    }

    if (i2cp->smbus) {
        g_i2c_config.op_mode = OPMODE_SMBUS_HOST;
    } else {
        g_i2c_config.op_mode = OPMODE_I2C;
    }

    i2cAcquireBus(drv);              /* Acquire ownership of the bus.    */
    i2cStart(drv, &g_i2c_config);
    i2cReleaseBus(drv);
    i2cp->started = true;

    return;
}

bool drv_i2c_read_registers(void *p, uint8_t first_reg, uint8_t *recv, uint32_t recv_len)
{
    return _transfer(p, &first_reg, 1, recv, recv_len);
}

/**
 * Wrapper function over #transfer() to write a byte to the register reg.
 * The transfer is done by sending reg and val in that order.
 *
 * Return: true on a successful transfer, false on failure.
 */
bool drv_i2c_write_register(void *p, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return _transfer(p, buf, sizeof(buf), NULL, 0);
}


uint8_t drv_i2c_read_register(void *p, uint8_t reg)
{
    uint8_t val = 0;
    drv_i2c_read_registers(p, reg, &val, 1);
    return val;
}

bool drv_i2c_transfer(void *p, const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    return _transfer(p, send, send_len, recv, recv_len);
}

static bool _do_transfer(void *p, const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    drv_i2c_vmt *i2c_vmt = p;
    I2CDriver *i2cp = drv_i2c_table[i2c_vmt->busid];

    for (uint8_t i = 0; i <= i2c_vmt->retries; i++) {
        int ret;
        // calculate a timeout as twice the expected transfer time, and set as min of 4ms
        uint32_t timeout_ms = 1+2*(((8*1000000UL/g_i2c_config.clock_speed)*(send_len+recv_len))/1000);
        timeout_ms = MAX(timeout_ms, 4);

        osalDbgAssert(i2cp->state == I2C_READY, "i2cStart state");

        if(send_len == 0) {
            ret = i2cMasterReceiveTimeout(i2cp, i2c_vmt->devAddr, recv, recv_len, chTimeMS2I(timeout_ms));
        } else {
            ret = i2cMasterTransmitTimeout(i2cp, i2c_vmt->devAddr, send, send_len,
                                           recv, recv_len, chTimeMS2I(timeout_ms));
        }

        osalDbgAssert(i2cp->state == I2C_STOP, "i2cStart state");


        if (MSG_OK != ret) {
            printk("[%s, L%d] i2c transfer ret %d.\r\n", __func__, __LINE__, ret);
            break;
        }

        if (ret == MSG_OK) {
            return true;
        }
    }

    return false;
}

static bool _transfer(void *p, const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    bool ret;
    drv_i2c_vmt *i2c_vmt = p;

    if (!i2c_vmt || (i2c_vmt->busid >= drv_i2c_size)) {
        printk("[%s, L%d] i2c_vmt is not exsit or busid invalid.\r\n", __func__, __LINE__);
        return false;
    }

    I2CDriver *i2cp = drv_i2c_table[i2c_vmt->busid];
    i2cAcquireBus(i2cp);
    ret = i2c_vmt->transfer(i2c_vmt, send, send_len, recv, recv_len);
    i2cReleaseBus(i2cp);
    return ret;
}

