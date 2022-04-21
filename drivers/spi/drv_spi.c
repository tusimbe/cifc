/*!
 *****************************************************************************
 *
 *  Copyright © 2016-2022 CHASING INNOVATION . All Rights Reserved.
 *
 * \file      drv_spi.c
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
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "usbcfg.h"
#include "system.h"
#include "drv_spi.h"
#include <stdlib.h>
#include <string.h>

#define SPIDEV_MODE0    0
#define SPIDEV_MODE1    SPI_CR1_CPHA
#define SPIDEV_MODE2    SPI_CR1_CPOL
#define SPIDEV_MODE3    SPI_CR1_CPOL | SPI_CR1_CPHA

#define SPI1_CLOCK  STM32_PCLK2
#define SPI2_CLOCK  STM32_PCLK1
#define SPI3_CLOCK  STM32_PCLK1
#define SPI4_CLOCK  STM32_PCLK2
#define SPI5_CLOCK  STM32_PCLK2
#define SPI6_CLOCK  STM32_PCLK2

typedef struct {
    uint32_t busid;
    uint32_t cs;
    uint32_t spimode;
    SPIConfig config;
    bool started;
    bool (*transfer)(void *p, const uint8_t * send, 
                uint32_t send_len, uint8_t * recv, uint32_t len);
} drv_spi_vmt;

// expected bus clock speeds
static const uint32_t bus_clocks[6] = {
    SPI1_CLOCK, SPI2_CLOCK, SPI3_CLOCK, SPI4_CLOCK, SPI5_CLOCK, SPI6_CLOCK
};


static SPIDriver *drv_bus_table[] = 
{
#if STM32_SPI_USE_SPI1 
    &SPID1,
#endif

/** @brief SPI2 driver identifier.*/
#if STM32_SPI_USE_SPI2 
    &SPID2,
#endif

/** @brief SPI3 driver identifier.*/
#if STM32_SPI_USE_SPI3 
    &SPID3,
#endif

/** @brief SPI4 driver identifier.*/
#if STM32_SPI_USE_SPI4 
    &SPID4,
#endif

/** @brief SPI5 driver identifier.*/
#if STM32_SPI_USE_SPI5 
    &SPID5,
#endif

/** @brief SPI6 driver identifier.*/
#if STM32_SPI_USE_SPI6 
    &SPID6,
#endif    
};

uint32_t  drv_bus_size = (sizeof(drv_bus_table) / sizeof(drv_bus_table[0]));

static uint32_t derive_freq_flag_bus(uint8_t busid, uint32_t _frequency);


static bool do_transfer(uint32_t busid, const uint8_t *send, uint8_t *recv, uint32_t len)
{
    bool ret = true;
    uint32_t flag;
    if (busid >= drv_bus_size) {
        printk("[%s, %d] busid %d is bigger than bus table size %d.\r\n", 
                                    __func__, __LINE__, (int)busid, (int)drv_bus_size);
        return false;
    }

    SPIDriver *drv = drv_bus_table[busid];

    flag = irqsave();
    //printk("1.\r\n");
    if (send == NULL) {
        spiStartReceiveI(drv, len, recv);
    } else if (recv == NULL) {
        spiStartSendI(drv, len, send);
    } else {
        spiStartExchangeI(drv, len, send, recv);
    }
    
    // we allow SPI transfers to take a maximum of 20ms plus 32us per
    // byte. This covers all use cases in ArduPilot. We don't ever
    // expect this timeout to trigger unless there is a severe MCU
    // error
    const uint32_t timeout_us = 20000U + len * 32U;
    msg_t msg = osalThreadSuspendTimeoutS(&drv->sync_transfer, TIME_US2I(timeout_us));
    //printk("2.\r\n");
    irqrestore(flag);
    if (msg == MSG_TIMEOUT) {
        printk("[%s, %d] spi transfer time out.\r\n", __func__, __LINE__);
        ret = false;
        spiAbort(drv);
    }

    return ret;
}

static bool transfer(void *p, const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    drv_spi_vmt *spip = p;

#ifdef _DEBUG_
    printk("[%s, L%d] send@%p sl:%d, recv@%p, rl:%d.\r\n", 
        __func__, __LINE__, send, send_len, recv, recv_len);
#endif
    if (!spip || !spip->started) {
        printk("[%s, %d] spip null or no started.\r\n", __func__, __LINE__);
        return false;
    }

    if (spip->busid >= drv_bus_size) {
        printk("[%s, %d] busid %d is bigger than bus table size %d.\r\n", 
                                    __func__, __LINE__, (int)spip->busid, (int)drv_bus_size);
        return false;
    }

    if ((send_len == recv_len && send == recv) || !send || !recv) {
        // simplest cases, needed for DMA
        return do_transfer(spip->busid, send, recv, recv_len?recv_len:send_len);
    }

    uint8_t *buf = malloc(send_len+recv_len);
    if (!buf) {
        printk("[%s:L%d malloc %d failed.\r\n]", __func__, __LINE__, (int)(send_len+recv_len));
        return false;
    }

    if (send_len > 0) {
        memcpy(buf, send, send_len);
    }
    if (recv_len > 0) {
        memset(&buf[send_len], 0, recv_len);
    }

    bool ret = do_transfer(spip->busid, buf, buf, send_len+recv_len);

    if (ret && recv_len > 0) {
        memcpy(recv, &buf[send_len], recv_len);
    }

    free(buf);

    return ret;
}


void* drv_spi_create(uint32_t busid)
{
    if (busid >= drv_bus_size) {
        printk("[%s, %d] busid %d is bigger than bus table size %d.\r\n", 
                                    __func__, __LINE__, (int)busid, (int)drv_bus_size);
        return NULL;
    }

    drv_spi_vmt *p = malloc(sizeof(drv_spi_vmt));
    if (!p) {
        printk("[%s, %d] no memory.\r\n", __func__, __LINE__);
        return NULL;
    }

    memset(&p->config, 0, sizeof(SPIConfig));
    p->busid = busid;
    p->cs = 0;
    p->spimode = SPIDEV_MODE3;
    p->started = false;
    p->transfer = transfer;

    return p;
}


static bool _transfer(void *p, const uint8_t *send, 
                uint32_t send_len, uint8_t *recv, uint32_t recv_len)
{
    bool ret;
    drv_spi_vmt *spip = p;

    if (!spip || !spip->started) {
        return false;
    }

    if (spip->busid >= drv_bus_size) {
        printk("[%s, %d] busid %d is bigger than bus table size %d.\r\n", 
                                    __func__, __LINE__, (int)spip->busid, (int)drv_bus_size);
        return false;
    }

    SPIDriver *drv = drv_bus_table[spip->busid];
    spiAcquireBus(drv);
    spiSelect(drv);
    ret = spip->transfer(p, send, send_len, recv, recv_len);
    spiUnselect(drv);
    spiReleaseBus(drv);
    return ret;
}

bool drv_spi_transfer(void *p, const uint8_t *send, 
                uint32_t send_len, uint8_t *recv, uint32_t recv_len)
{
#ifdef _DEBUG_
    drv_spi_vmt *spip = p;
    printk("[%s, L%d] bus %d send@%p sl:%d, recv@%p, rl:%d.\r\n", 
        __func__, __LINE__, spip->busid, send, (int)send_len, recv, (int)recv_len);
#endif
    return _transfer(p, send, send_len, recv, recv_len);
}

void drv_spi_start(void *p, uint32_t freq, uint32_t cs_io, uint32_t mode)
{
    drv_spi_vmt *spip = p;
    uint32_t freq_flag;
    uint32_t spimode;

    spip->started = false;

    if (spip->busid >= drv_bus_size) {
        printk("[%s, %d] busid %d is bigger than bus table size %d.\r\n", 
                                    __func__, __LINE__, (int)spip->busid, (int)drv_bus_size);
        return;
    }

    SPIDriver *drv = drv_bus_table[spip->busid];
    
    memset(&spip->config, 0, sizeof(SPIConfig));
    spip->config.data_cb = NULL;
    spip->config.ssport = PAL_PORT(cs_io);
    spip->config.sspad = PAL_PAD(cs_io);

    freq_flag = derive_freq_flag_bus(spip->busid, freq);
    switch (mode) {
        case DRV_SPI_MODE0:
            spimode = SPIDEV_MODE0;
            break;
        case DRV_SPI_MODE1:
            spimode = SPIDEV_MODE1;
            break;
        case DRV_SPI_MODE2:
            spimode = SPIDEV_MODE2;
            break;
        case DRV_SPI_MODE3:
            spimode = SPIDEV_MODE3;
            break;           
        default:
            spimode = SPIDEV_MODE3;
            break;
    }
    
    spip->config.cr1 = (uint16_t)(freq_flag | spimode);
    //printk("cr1:0x%x.\r\n", g_config.cr1);
    spip->config.cr2 = 0;

    spiAcquireBus(drv);              /* Acquire ownership of the bus.    */
    spiStart(drv, &spip->config);
    spiReleaseBus(drv);
    spip->cs = cs_io;
    spip->spimode = mode;
    spip->started = true;

    return;
}

void drv_spi_stop(void *p)
{
    drv_spi_vmt *spip = p;

    if (!spip->started) {
        return;
    }

    if (spip->busid >= drv_bus_size) {
        printk("[%s, %d] busid %d is bigger than bus table size %d.\r\n", 
                                    __func__, __LINE__, (int)spip->busid, (int)drv_bus_size);
        return;
    }

    SPIDriver *drv = drv_bus_table[spip->busid];
    spiAcquireBus(drv);              /* Acquire ownership of the bus.    */
    spiStop(drv);
    spiReleaseBus(drv);
    spip->started = false;
    return;
}

static uint32_t derive_freq_flag_bus(uint8_t busid, uint32_t _frequency)
{
    uint32_t spi_clock_freq = SPI1_CLOCK;
    if (busid > 0 && (uint8_t)(busid-1) < ARRAY_SIZE(bus_clocks)) {
        spi_clock_freq = bus_clocks[busid-1] / 2;
    }

    // find first divisor that brings us below the desired SPI clock
    uint32_t i = 0;
    while (spi_clock_freq > _frequency && i<7) {
        spi_clock_freq >>= 1;
        i++;
    }

    // assuming the bitrate bits are consecutive in the CR1 register,
    // we can just multiply by BR_0 to get the right bits for the desired
    // scaling

    return i * SPI_CR1_BR_0;
}

/**
 * Wrapper function over #transfer() to read recv_len registers, starting
 * by first_reg, into the array pointed by recv. The read flag passed to
 * #set_read_flag(uint8_t) is ORed with first_reg before performing the
 * transfer.
 *
 * Return: true on a successful transfer, false on failure.
 */
bool drv_spi_read_registers(void *p, uint8_t first_reg, uint8_t *recv, uint32_t recv_len)
{
    first_reg |= 0x80;
    return _transfer(p, &first_reg, 1, recv, recv_len);
}

/**
 * Wrapper function over #transfer() to write a byte to the register reg.
 * The transfer is done by sending reg and val in that order.
 *
 * Return: true on a successful transfer, false on failure.
 */
bool drv_spi_write_register(void *p, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return _transfer(p, buf, sizeof(buf), NULL, 0);
}


uint8_t drv_spi_register_read(void *p, uint8_t reg)
{
    uint8_t val = 0;
    drv_spi_read_registers(p, reg, &val, 1);
    return val;
}

void drv_spi_set_speed(void *p, uint32_t freq)
{
    drv_spi_vmt *spip = p;
    drv_spi_stop(p);
    drv_spi_start(p, freq, spip->cs, spip->spimode);
}

