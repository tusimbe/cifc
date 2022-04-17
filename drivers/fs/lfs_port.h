/*!
 *****************************************************************************
 *
 *  Copyright © 2017-2018 CHASING INNOVATION. All Rights Reserved.
 *
 * \file      lfs_port.h
 * \author    johnson@chasing-innovation.com
 * \version   1.0
 * \date      2022年4月17日星期日
 * \brief     lfs文件系统PORTING
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
#ifndef _LFS_PORT_H_
#define _LFS_PORT_H_

int lfs_port_mount(lfs_t *lfs);
void lfs_port_unmount(lfs_t *lfs);

#endif
