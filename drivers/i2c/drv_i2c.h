/*!
 *****************************************************************************
 *
 *  Copyright © 2016-2022 CHASING INNOVATION . All Rights Reserved.
 *
 * \file      drv_i2c.h
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
#ifndef _DRV_I2C_H_
#define _DRV_I2C_H_

void* drv_i2c_create(uint32_t busid, bool smbus, uint8_t devAddr, uint32_t retries);
bool drv_i2c_transfer(void *p, const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len);

bool drv_i2c_read_registers(void *p, uint8_t first_reg, uint8_t *recv, uint32_t recv_len);
void drv_i2c_start(void *p);
bool drv_i2c_write_register(void *p, uint8_t reg, uint8_t val);
uint8_t drv_i2c_read_register(void *p, uint8_t reg);

#endif
