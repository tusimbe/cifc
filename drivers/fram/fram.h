/*!
 *****************************************************************************
 *
 *  Copyright © 2017-2018 CHASING INNOVATION. All Rights Reserved.
 *
 * \file      fram.h
 * \author    johnson@chasing-innovation.com
 * \version   1.0
 * \date      2022年4月17日星期日
 * \brief     fram驱动
 *
 *----------------------------------------------------------------------------
 * \attention
 *
 *
 *****************************************************************************
 */

/*****************************************************************************
 change history: 
    1.date  : 2022年4月17日星期日
      author: johnson@chasing-innovation.com
      change: create file

*****************************************************************************/
#ifndef _DRV_FRAM_H_
#define _DRV_FRAM_H_

int32_t fram_init(uint32_t spi_id);
int32_t fram_read(uint32_t addr, uint8_t *buf, uint32_t size);
int32_t fram_write(uint32_t addr, uint8_t *buf, uint32_t size);
void fram_cmd(BaseSequentialStream *chp, int argc, char *argv[]);

#endif
