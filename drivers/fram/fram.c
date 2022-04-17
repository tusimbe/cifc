/*!
 *****************************************************************************
 *
 *  Copyright © 2017-2018 CHASING INNOVATION. All Rights Reserved.
 *
 * \file      fram.c
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
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "usbcfg.h"
#include "shell.h"
#include "drv_spi.h"
#include "system.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdlib.h>

#define  FRAM_SPI_FREQ    (8*MHZ)
#define  FRAM_BUFF_SIZE   (128)

// register numbers
#define RAMTRON_RDID      0x9f
#define RAMTRON_READ      0x03
#define RAMTRON_RDSR      0x05
#define RAMTRON_WREN      0x06
#define RAMTRON_WRITE     0x02

typedef struct rdid {
    uint8_t manufacturer[6];
    uint8_t memory;
    uint8_t id1;
    uint8_t id2;
} RDID;

typedef struct ramtron_id {
    uint8_t id1, id2;
    uint16_t size_kbyte;
    uint8_t addrlen;
} RAMTRON_ID;

/*
  list of supported devices. Thanks to NuttX ramtron driver
 */
const RAMTRON_ID ramtron_ids[] = {
    { 0x21, 0x00, 16,  2}, // FM25V01
    { 0x21, 0x08, 16,  2}, // FM25V01A
    { 0x22, 0x00, 32,  2}, // FM25V02
    { 0x22, 0x08, 32,  2}, // FM25V02A
    { 0x22, 0x01, 32,  2}, // FM25VN02
    { 0x23, 0x00, 64,  2}, // FM25V05
    { 0x23, 0x01, 64,  2}, // FM25VN05
    { 0x24, 0x00, 128, 3}, // FM25V10
    { 0x24, 0x01, 128, 3}, // FM25VN10
    { 0x25, 0x08, 256, 3}, // FM25V20A
    { 0x26, 0x08, 512, 3}, // CY15B104Q
    { 0x27, 0x03, 128, 3}, // MB85RS1MT
    { 0x05, 0x09, 32,  3}, // B85RS256B
};

static void *g_fram_spi;
osSemaphoreId fram_sem;
uint8_t addrlen;

int32_t fram_init(uint32_t spi_id)
{
    RDID rdid;
    bool isFind = false;

    fram_sem = osSemaphoreCreate(NULL, 1);
    if (!fram_sem) {
        printk("[%s, %d] spi %d osSemaphoreCreate failed.\r\n", __func__, __LINE__, spi_id);
        return -1;
    }

    g_fram_spi = drv_spi_create(spi_id);
    if (!g_fram_spi) {
        printk("[%s, %d] spi %d driver create failed.\r\n", __func__, __LINE__, spi_id);
        return -2;
    }

    drv_spi_start(g_fram_spi, FRAM_SPI_FREQ, LINE_FRAM_CS, DRV_SPI_MODE3);

    if (!drv_spi_read_registers(g_fram_spi, RAMTRON_RDID, (uint8_t*)&rdid, sizeof(RDID))) {
        printk("[%s, %d] read rdid register failed.\r\n", __func__, __LINE__);
        return -3;
    }

    printk("RAMTRON manufacturer=%02x memory=%02x id1=%02x id2=%02x\r\n",
                        rdid.manufacturer[0], rdid.memory, rdid.id1, rdid.id2);

    for (uint8_t i = 0; i < ARRAY_SIZE(ramtron_ids); i++) {
        if (ramtron_ids[i].id1 == rdid.id1 &&
            ramtron_ids[i].id2 == rdid.id2) {
            addrlen = ramtron_ids[i].addrlen;
            isFind = true;
            break;
        }
    }

    if (!isFind) {
        return -4;
    }

    printk("RAMTRON addr len %d.\r\n", addrlen);
    return 0;

}

void _cmd_addr_buf(uint32_t addr, uint8_t cmd, uint8_t *buf)
{
    buf[0] = cmd;
    if (addrlen == 3) {
        buf[1] = (uint8_t)((addr>>16) & 0xFF);
        buf[2] = (uint8_t)((addr>>8) & 0xFF);
        buf[3] = (uint8_t)(addr & 0xFF);
    } else /* len 2 */ {
        buf[1] = (uint8_t)((addr>>8) & 0xFF);
        buf[2] = (uint8_t)(addr & 0xFF);
    }
}

int32_t fram_read(uint32_t addr, uint8_t *buf, uint32_t size)
{
    uint8_t buf_sd[4];
    uint8_t cmd_size = addrlen + 1;

    _cmd_addr_buf(addr, RAMTRON_READ, buf_sd);
#ifdef _FRAM_DEBUG_
    printk("fram_read send %02x %02x %02x sds %d, revsize %d.\r\n", 
        buf_sd[0], buf_sd[1], buf_sd[2], cmd_size, size);
#endif
    osSemaphoreWait(fram_sem, osWaitForever);
    drv_spi_transfer(g_fram_spi, buf_sd, cmd_size, buf, size);
    osSemaphoreRelease(fram_sem);
    
    return 0;
}

int32_t fram_write(uint32_t addr, uint8_t *buf, uint32_t size)
{
    uint8_t *buf_sd;
    uint8_t cmd;
    uint8_t cmd_size = addrlen + 1;

    buf_sd = malloc(size + cmd_size);
    if (!buf_sd) {
        return -1;
    }

    osSemaphoreWait(fram_sem, osWaitForever);
    cmd = RAMTRON_WREN;
    drv_spi_transfer(g_fram_spi, &cmd, 1, NULL, 0);
    
    _cmd_addr_buf(addr, RAMTRON_WRITE, buf_sd);
#ifdef _FRAM_DEBUG_
    printk("fram_read send %02x %02x %02x sds %d, revsize %d.\r\n", 
        buf_sd[0], buf_sd[1], buf_sd[2], cmd_size, size);
#endif
    memcpy(buf_sd + cmd_size, buf, size);
    drv_spi_transfer(g_fram_spi, buf_sd, size + cmd_size, NULL, 0);
    osSemaphoreRelease(fram_sem);
    free(buf_sd);
    
    return 0;
}

void fram_cmd(BaseSequentialStream *chp, int argc, char *argv[])
{
	int32_t cmd_type;
    RDID rdid;
    uint32_t addr;
    uint32_t test_c = 0xdeadbeef;

	if (argc > 3) {
		shellUsage(chp, "fram [id | init | test]");
		return;
	} else if (argc == 1){
		if (strcmp(argv[0], "id") == 0) {
			cmd_type = 1;
		} else if (strcmp(argv[0], "init") == 0){
			cmd_type = 2;
		} else {
			cmd_type = -1;
			shellUsage(chp, "fram [id | init | test]");
			return;
		}
	} else {
        if (strcmp(argv[0], "test") == 0){
            cmd_type = 3;
            addr = atoi(argv[1]);
            if (argc == 3) {
                test_c = atoi(argv[2]);
            }
        }
    }

    if (cmd_type == 1) {

        if (!drv_spi_read_registers(g_fram_spi, RAMTRON_RDID, (uint8_t*)&rdid, sizeof(RDID))) {
            printk("[%s, %d] read rdid register failed.\r\n", __func__, __LINE__);
            return;
        }
        
        printk("RAMTRON manufacturer=%02x memory=%02x id1=%02x id2=%02x\n",
                            rdid.manufacturer[0], rdid.memory, rdid.id1, rdid.id2);

    } else if (cmd_type == 2) {
        int ret = fram_init(1);
        printk("fram_init ret %d", ret);
    } else if (cmd_type == 3) {
        
        uint32_t r;
        fram_read(addr, (uint8_t*)&r, 4);
        printk("read addr %08x : %08x.\r\n", addr, r);
        fram_write(addr, (uint8_t*)&test_c, 4);
        printk("write addr %08x : %08x.\r\n", addr, test_c);
        fram_read(addr, (uint8_t*)&r, 4);
        printk("read addr %08x : %08x.\r\n", addr, r);
    }

    return;
}

