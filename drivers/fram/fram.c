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
#include "malloc.h"
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
mutex_t fram_sem;
uint8_t addrlen;
uint32_t size_kbyte;

int32_t fram_init(uint32_t spi_id)
{
    RDID rdid;
    bool isFind = false;

    osalMutexObjectInit(&fram_sem);

    g_fram_spi = drv_spi_create(spi_id);
    if (!g_fram_spi) {
        printk("[%s, %d] spi %d driver create failed.\r\n", __func__, __LINE__, (int)spi_id);
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
            size_kbyte = ramtron_ids[i].size_kbyte;
            isFind = true;
            break;
        }
    }

    if (!isFind) {
        return -4;
    }

    printk("RAMTRON addr len %d.\r\n", (int)addrlen);
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
    osalMutexLock(&fram_sem);
    drv_spi_transfer(g_fram_spi, buf_sd, cmd_size, buf, size);
    osalMutexUnlock(&fram_sem);
    
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

    osalMutexLock(&fram_sem);
    cmd = RAMTRON_WREN;
    drv_spi_transfer(g_fram_spi, &cmd, 1, NULL, 0);
    
    _cmd_addr_buf(addr, RAMTRON_WRITE, buf_sd);
#ifdef _FRAM_DEBUG_
    printk("fram_read send %02x %02x %02x sds %d, revsize %d.\r\n", 
        buf_sd[0], buf_sd[1], buf_sd[2], cmd_size, size);
#endif
    memcpy(buf_sd + cmd_size, buf, size);
    drv_spi_transfer(g_fram_spi, buf_sd, size + cmd_size, NULL, 0);
    osalMutexUnlock(&fram_sem);
    free(buf_sd);
    
    return 0;
}


void _hexDump(char *desc, void *addr, int len) 
{
    int i;
    unsigned char buff[17];
    unsigned char *pc = (unsigned char*)addr;

    // Output description if given.
    if (desc != NULL)
        printk ("%s:\n", desc);

    // Process every byte in the data.
    for (i = 0; i < len; i++) {
        // Multiple of 16 means new line (with line offset).

        if ((i % 16) == 0) {
            // Just don't print ASCII for the zeroth line.
            if (i != 0)
                printk("  %s\n", buff);

            // Output the offset.
            printk("  %04x ", i);
        }

        // Now the hex code for the specific character.
        printk(" %02x", pc[i]);

        // And store a printable ASCII character for later.
        if ((pc[i] < 0x20) || (pc[i] > 0x7e)) {
            buff[i % 16] = '.';
        } else {
            buff[i % 16] = pc[i];
        }

        buff[(i % 16) + 1] = '\0';
    }

    // Pad out last line if not exactly 16 characters.
    while ((i % 16) != 0) {
        printk("   ");
        i++;
    }

    // And print the final ASCII bit.
    printk("  %s\n", buff);
}

#define FRAM_DUMP_SIZE  (1024)
void _dump(uint32_t start, uint32_t end)
{
    uint8_t *buf;
    uint32_t addr;
    char desc[24];
    uint32_t i;

    if (start > size_kbyte || end > size_kbyte) {
        return;
    }
    
    buf = malloc(FRAM_DUMP_SIZE);
    if (!buf) {
        return;
    }

    for (i = start; i < end; i++) {
        addr = i * FRAM_DUMP_SIZE;
        fram_read(addr, buf, FRAM_DUMP_SIZE);
        sprintf(desc, "block %d.\r\n", (int)i);
        _hexDump(desc, buf, FRAM_DUMP_SIZE);
    }

    free(buf);
}

void _erase(uint8_t val)
{
    uint32_t addr;
    
    for (addr = 0;addr < size_kbyte * 1024; addr++) {
        //printf("addr 0x%x val %d.\r\n", (int)addr, val);
        fram_write(addr, &val, 1);
    }
}

void fram_cmd(BaseSequentialStream *chp, int argc, char *argv[])
{
	int32_t cmd_type;
    RDID rdid;
    uint32_t addr;
    uint32_t test_c = 0xdeadbeef;
    uint32_t start, end;
    uint32_t val;

	if (argc > 3) {
		shellUsage(chp, "fram [id | init | test | dump]");
		return;
	} 

    if (argc == 1){
		if (strcmp(argv[0], "id") == 0) {
			cmd_type = 1;
		} else if (strcmp(argv[0], "init") == 0){
			cmd_type = 2;
		} else {
			cmd_type = -1;
			shellUsage(chp, "fram [id | init | test | dump]");
			return;
		}
	} else if (argc == 2) {
        if (strcmp(argv[0], "erase") == 0){
            cmd_type = 5;
            val = atoi(argv[1]);
        }
    } else {
        if (strcmp(argv[0], "test") == 0){
            cmd_type = 3;
            addr = atoi(argv[1]);
            if (argc == 3) {
                test_c = atoi(argv[2]);
            }
        } else if (strcmp(argv[0], "dump") == 0){
            start = atoi(argv[1]);
            end = atoi(argv[2]);
            cmd_type = 4;
        }
    }

    if (cmd_type == 1) {

        if (!drv_spi_read_registers(g_fram_spi, RAMTRON_RDID, (uint8_t*)&rdid, sizeof(RDID))) {
            printk("[%s, %d] read rdid register failed.\r\n", __func__, __LINE__);
            return;
        }
        
        printk("RAMTRON manufacturer=%02x memory=%02x id1=%02x id2=%02x\r\n",
                            rdid.manufacturer[0], rdid.memory, rdid.id1, rdid.id2);

    } else if (cmd_type == 2) {
        int ret = fram_init(1);
        printk("fram_init ret %d", ret);
    } else if (cmd_type == 3) {
        
        uint32_t r;
        fram_read(addr, (uint8_t*)&r, 4);
        printk("read addr %08x : %08x.\r\n", (int)addr, (int)r);
        fram_write(addr, (uint8_t*)&test_c, 4);
        printk("write addr %08x : %08x.\r\n", (int)addr, (int)test_c);
        fram_read(addr, (uint8_t*)&r, 4);
        printk("read addr %08x : %08x.\r\n", (int)addr, (int)r);
    } else if (cmd_type == 4) {
        _dump(start, end);
    } else if (cmd_type == 5) {
        _erase((uint8_t)val);
    }

    return;
}

