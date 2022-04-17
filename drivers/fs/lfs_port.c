/*!
 *****************************************************************************
 *
 *  Copyright © 2017-2018 CHASING INNOVATION. All Rights Reserved.
 *
 * \file      lfs_port.c
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
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "usbcfg.h"
#include "shell.h"
#include "system.h"
#include "lfs.h"
#include "fram.h"
#include "lfs_port.h"

// Read a region in a block. Negative error codes are propagated
// to the user.
int lfs_port_read(const struct lfs_config *c, lfs_block_t block,
        lfs_off_t off, void *buffer, lfs_size_t size)
{
    return fram_read(c->block_size * block + off, (uint8_t*)buffer, size);
}

// Program a region in a block. The block must have previously
// been erased. Negative error codes are propagated to the user.
// May return LFS_ERR_CORRUPT if the block should be considered bad.
int lfs_port_prog(const struct lfs_config *c, lfs_block_t block,
        lfs_off_t off, const void *buffer, lfs_size_t size)
{
    return fram_write(c->block_size * block + off, (uint8_t*)buffer, size);
}

// Erase a block. A block must be erased before being programmed.
// The state of an erased block is undefined. Negative error codes
// are propagated to the user.
// May return LFS_ERR_CORRUPT if the block should be considered bad.
int lfs_port_erase(const struct lfs_config *c, lfs_block_t block)
{
    (void)c;
    (void)block;
    
    return LFS_ERR_OK;
}

// Sync the state of the underlying block device. Negative error codes
// are propagated to the user.
int lfs_port_sync(const struct lfs_config *c)
{
    (void)c;
    
    return LFS_ERR_OK;
}

// configuration of the filesystem is provided by this struct
const struct lfs_config cfg = {
    // block device operations
    .read  = lfs_port_read,
    .prog  = lfs_port_prog,
    .erase = lfs_port_erase,
    .sync  = lfs_port_sync,

    // block device configuration
    .read_size = 16,
    .prog_size = 16,
    .block_size = 1024,
    .block_count = 32,
    .cache_size = 16,
    .lookahead_size = 16,
    .block_cycles = 500,
};

int lfs_port_mount(lfs_t *lfs)
{
    // mount the filesystem
    int err = lfs_mount(lfs, &cfg);

    // reformat if we can't mount the filesystem
    // this should only happen on the first boot
    if (err) {
        lfs_format(lfs, &cfg);
        lfs_mount(lfs, &cfg);
    }

    return 0;
}

void lfs_port_unmount(lfs_t *lfs)
{
    lfs_unmount(lfs);
}

// entry point
void lfs_port_test(void) 
{
#if 0
    // mount the filesystem
    int err = lfs_mount(&lfs, &cfg);

    // reformat if we can't mount the filesystem
    // this should only happen on the first boot
    if (err) {
        lfs_format(&lfs, &cfg);
        lfs_mount(&lfs, &cfg);
    }

    // read current count
    uint32_t boot_count = 0;
    lfs_file_open(&lfs, &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&lfs, &file, &boot_count, sizeof(boot_count));

    // update boot count
    boot_count += 1;
    lfs_file_rewind(&lfs, &file);
    lfs_file_write(&lfs, &file, &boot_count, sizeof(boot_count));

    // remember the storage is not updated until the file is closed successfully
    lfs_file_close(&lfs, &file);

    // release any resources we were using
    lfs_unmount(&lfs);

    // print the boot count
    printk("boot_count: %d\r\n", boot_count);
#endif
}

void lfs_cmd(BaseSequentialStream *chp, int argc, char *argv[])
{
	int32_t cmd_type;

	if (argc != 1) {
		shellUsage(chp, "lfs [test]");
		return;
	} else {
		if (strcmp(argv[0], "test") == 0) {
			cmd_type = 1;
		} else if (strcmp(argv[0], "init") == 0){
			cmd_type = 2;
		} else if (strcmp(argv[0], "fifo") == 0){
			cmd_type = 3;
		} else {
			cmd_type = -1;
			shellUsage(chp, "lfs [test]");
			return;
		}
	}

    if (cmd_type == 1) {
        lfs_port_test();
    } else if (cmd_type == 2) {

    } else if (cmd_type == 3) {
    }

    return;
}


