/*!
 *****************************************************************************
 *
 *  Copyright © 2016-2022 CHASING INNOVATION . All Rights Reserved.
 *
 * \file      drv_spi.h
 * \author    johnson@chasing-innovation.com
 * \version   1.0
 * \date      2022年4月5日星期二
 * \brief     SPI驱动抽象层
 *
 *----------------------------------------------------------------------------
 * \attention
 *
 *
 *****************************************************************************
 */

/*****************************************************************************
 change history: 
    1.date  : 2022年4月5日星期二
      author: johnson@chasing-innovation.com
      change: create file

*****************************************************************************/
#ifndef _DRV_SPI_H_
#define _DRV_SPI_H_

bool drv_spi_transfer(void *p, const uint8_t *send, 
                uint32_t send_len, uint8_t *recv, uint32_t recv_len);
void* drv_spi_create(uint32_t busid);
void drv_spi_start(void *p, uint32_t freq);
bool drv_spi_read_registers(void *p, uint8_t first_reg, uint8_t *recv, uint32_t recv_len);
bool drv_spi_write_register(void *p, uint8_t reg, uint8_t val);
void drv_spi_freq_set(void *p, uint32_t freq);
uint8_t drv_spi_register_read(void *p, uint8_t reg);
void drv_spi_set_speed(void *p, uint32_t freq);

#endif
