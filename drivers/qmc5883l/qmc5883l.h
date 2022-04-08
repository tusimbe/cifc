/*!
 *****************************************************************************
 *
 *  Copyright © 2016-2022 CHASING INNOVATION . All Rights Reserved.
 *
 * \file      qmc5883l.h
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
#ifndef _DRV_QMC5883L_H_
#define _DRV_QMC5883L_H_

int qmc5883l_init(uint32_t busid);
void qmc5883l_cmd(BaseSequentialStream *chp, int argc, char *argv[]);

#endif
