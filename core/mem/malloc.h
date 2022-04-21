/*!
 *****************************************************************************
 *
 *  Copyright © 2017-2018 CHASING INNOVATION. All Rights Reserved.
 *
 * \file      malloc.h
 * \author    johnson@chasing-innovation.com
 * \version   1.0
 * \date      2022年4月19日星期二
 * \brief     内存管理
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
#ifndef _MALLOC_H_
#define _MALLOC_H_

void malloc_init(void);
size_t mem_available(void);
void *malloc_dma(size_t size);
void *malloc_sdcard_dma(size_t size);
void *malloc_fastmem(size_t size);
thread_t *thread_create_alloc(size_t size, const char *name, tprio_t prio, tfunc_t pf, void *arg);

// flush all dcache
void memory_flush_all(void);

#endif
