/*!
 *****************************************************************************
 *
 *  Copyright © 2017-2018 CHASING INNOVATION. All Rights Reserved.
 *
 * \file      params.c
 * \author    johnson@chasing-innovation.com
 * \version   1.0
 * \date      2022年4月16日星期六
 * \brief     参数管理模块
 *
 *----------------------------------------------------------------------------
 * \attention
 *
 *
 *****************************************************************************
 */

/*****************************************************************************
 change history: 
    1.date  : 2022年4月16日星期六
      author: johnson@chasing-innovation.com
      change: create file

*****************************************************************************/
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
//#include "list.h"
#include "shell.h"
#include "usbcfg.h"
#include "system.h"
#include "lfs.h"
#include "lfs_port.h"
#include "cJSON.h"

// variables used by the filesystem
lfs_t lfs;
lfs_file_t file;

#if 0

#define  PARAMS_MAGIC_NUMBER         (0xaa5555aa)
#define  PARAMS_VERSION_MINOR        (0)
#define  PARAMS_VERSION_MAIN         (1)
#define  PARAMS_VERSION              (PARAMS_VERSION_MAIN << 8 | PARAMS_VERSION_MINOR)

#define  PARAMS_ID(moduleId, paramId)  (moduleId << 16 | paramId)
#define  PARAMS_MODULEID(id)             ((id >> 16) & 0xffff)
#define  PARAMS_PARAMID(id)              (id & 0xffff)

typedef struct params_module_s {
    struct list_head head;
    uint16_t moduleId;
    uint16_t numOfParams;
    param_numbers_get numbers_get_fn;
    param_default_load default_load_fn;
    PARAM_ENTRY_T entrys[];
} PARAMS_MODULE_T;

typedef struct params_head_s {
    uint32_t magic;
    uint16_t version;
    uint16_t res1;
    uint16_t numOfModule;
    uint16_t numOfParams;
    uint32_t res2;
} PARAMS_HEAD_T;

typedef struct param_entry_store_s {
    uint32_t id;
    enum param_type type;
    PARAM_DATA data;
} PARAM_ENTRY_STORE_T;

typedef struct param_module_head_s {
    uint32_t moduleId;
    uint32_t numOfParams;
} PARAM_MODULE_HEAD_T;


struct list_head param_list_head;
osSemaphoreId param_sem;
uint32_t start_addr;
uint32_t storage_size;
PARAMS_HEAD_T param_head;


int32_t params_init(void)
{
    param_sem = osSemaphoreCreate(NULL, 1);
    if (!param_sem) {
        return -1;
    }

    param_head.magic = PARAMS_MAGIC_NUMBER;
    param_head.version = PARAMS_VERSION;
    param_head.numOfModule = 0;
    param_head.numOfParams = 0;

    return 0;
}

PARAMS_MODULE_T *_param_module_find(uint32_t moduleId)
{
    struct list_head *pos;
    PARAMS_MODULE_T *entry;
    bool isFind = false;

    osSemaphoreWait(param_sem, osWaitForever);
    list_for_each(pos, &param_list_head) {
        entry = list_entry(pos, PARAMS_MODULE_T, head);
        if (entry->moduleId == moduleId) {
            isFind = true;
            break;
        }
    }

    osSemaphoreRelease(param_sem);
    if (isFind) {
        return entry;
    } else {
        return NULL;
    }
}


uint32_t param_cb_register(uint32_t moduleid, param_numbers_get func_get, param_default_load func_load)
{
    PARAMS_MODULE_T *entry;
    
    entry = _param_module_find(moduleid);
    if (entry != NULL) {
        return (-1);
    }

    entry = malloc(sizeof(PARAMS_MODULE_T));
    if (!entry) {
        return (-2);
    }

    entry->moduleId = moduleid;
    INIT_LIST_HEAD(&entry->head);
    entry->default_load_fn = func_load;
    entry->numbers_get_fn = func_get;
    entry->entrys = NULL;

    osSemaphoreWait(param_sem, osWaitForever);
    list_add(&entry->head, &param_list_head);
    param_head.numOfModule++;
    osSemaphoreRelease(param_sem);

    return 0;
}

void param_def_load(void)
{
    struct list_head *pos;
    PARAMS_MODULE_T *module;
    PARAM_ENTRY_T entry;
    uint32_t maxOfparam;
    uint32 i;

    osSemaphoreWait(param_sem, osWaitForever);
    list_for_each(pos, &param_list_head) {
        module = list_entry(pos, PARAMS_MODULE_T, head);
        maxOfparam = module->numbers_get_fn(module->moduleId);
        module->entrys = malloc(sizeof(PARAM_ENTRY_T) * maxOfparam);
        if (!module->entrys) {
            break;
        }
        module->numOfParams = maxOfparam;
        param_head.numOfParams += maxOfparam;
        for (i = 0; i < maxOfparam; i++) {
            module->default_load_fn(module->moduleId, i, &module->entrys[i]);
        }
    }

    osSemaphoreRelease(param_sem);
    return;
}

void param_load(void)
{
    PARAMS_HEAD_T phdr;
    
    storage_read(start_addr, &phdr ,sizeof(PARAMS_HEAD_T));

    /* new storage */
    if (phdr.magic != PARAMS_MAGIC_NUMBER) {
        _format();
    } else {
        _load();
    }
}

int32_t _data_find(uint16_t moduleId, uint16_t paramId, PARAM_ENTRY_STORE_T *data)
{
    uint32_t id = PARAMS_ID(moduleId, paramId);
    uint32_t addr;
    PARAM_ENTRY_STORE_T entry;
    bool isFind = false;
    int32_t ret;

    for (addr = start_addr + sizeof(PARAMS_HEAD_T); 
         addr < start_addr + storage_size; addr += sizeof(PARAM_ENTRY_STORE_T)) {
        storage_read(addr, &entry ,sizeof(PARAM_ENTRY_STORE_T));
        if (entry.id == id) {
            isFind = true;
            memcpy(data, &entry, sizeof(PARAM_ENTRY_STORE_T));
            break;
        }
    }

    ret = isFind ? 0 : -1;
    return ret;
}

int32_t _data_find_by_name(char *name, PARAM_ENTRY_STORE_T *data)
{
    struct list_head *pos;
    PARAMS_MODULE_T *module;
    PARAM_ENTRY_T *entry;
    uint32_t i;
    bool isFind = false;
    int32_t ret;
    
    list_for_each(pos, &param_list_head) {
        module = list_entry(pos, PARAMS_MODULE_T, head);
        for (i = 0; i < module->numOfParams; i++) {
            entry = module->entrys[i];
            if (strcmp(name, entry->entry_name) == 0) {
                isFind = true;
                break;
            }
        }
    }

    if (isFind) {
        ret = _data_find(module->moduleId, entry->paramId, data);
    } else {
        ret = -1;
    }

    return ret;
}

int32_t _data_set(uint16_t moduleId, uint16_t paramId, enum param_type type, PARAM_DATA *data)
{
    uint32_t id = PARAMS_ID(moduleId, paramId);
    uint32_t addr;
    PARAM_ENTRY_STORE_T entry;
    bool isFind = false;
    int32_t ret;

    for (addr = start_addr + sizeof(PARAMS_HEAD_T); 
         addr < start_addr + storage_size; addr += sizeof(PARAM_ENTRY_STORE_T)) {
        storage_read(addr, &entry ,sizeof(PARAM_ENTRY_STORE_T));
        if (entry.id == id) {
            isFind = true;
            memcpy(&entry, data, sizeof(PARAM_ENTRY_STORE_T));
            break;
        }
    }

    if (isFind) {
        
    }
}



void _format(void) 
{
    struct list_head *pos;
    PARAMS_MODULE_T *module;
    PARAM_ENTRY_T *entry;
    PARAM_ENTRY_STORE_T entryStore;
    uint32_t i;
    uint32_t addr = start_addr;

    storage_write(addr, &param_head, sizeof(PARAMS_HEAD_T));
    addr += sizeof(PARAMS_HEAD_T);
    
    list_for_each(pos, &param_list_head) {
        module = list_entry(pos, PARAMS_MODULE_T, head);
        for (i = 0; i < module->numOfParams; i++) {
            entry = &module->entrys[i];
            entryStore.id = PARAMS_ID(module->moduleId, entry->paramId);
            entryStore.type = entry.type;
            memcpy(&entryStore.data, &entry->data, sizeof(PARAM_DATA));
            
            storage_write(addr, sizeof(PARAM_DATA), &entryStore);
            addr += sizeof(PARAM_DATA);
        }
    }

    return;
}

void _load(void)
{
    struct list_head *pos;
    PARAMS_MODULE_T *module;
    PARAM_ENTRY_T *entry;
    PARAM_ENTRY_STORE_T entryStore;
    uint32_t i;
    uint32_t ret;

    list_for_each(pos, &param_list_head) {
        module = list_entry(pos, PARAMS_MODULE_T, head);
        for (i = 0; i < module->numOfParams; i++) {
            entry = &module->entrys[i];
            ret = _data_find(module->moduleId, entry->paramId, &entryStore);
            if (0 == ret) {
                if (entryStore.type != entry->type) {
                    continue;
                } else {
                    memcpy(&entry->data, &entryStore.data, sizeof(PARAM_DATA));
                }
            }
        }
    }

    return;
}

#endif

cJSON *json;

void _rebuild_params(void);

int params_init(void)
{
    int ret;
    int32_t fsize;
    lfs_port_mount(&lfs);
    char *string;

    ret = lfs_file_open(&lfs, &file, "cifc_params", LFS_O_RDWR | LFS_O_CREAT);
    if (LFS_ERR_OK != ret) {
        printk("lfs_file_open fail ret %d.\r\n", ret);
        return ret;
    }

    fsize = lfs_file_size(&lfs, &file);
    if (fsize < 0) {
        printk("file size get err fsize = %d.\r\n", fsize);
        return -1;
    }

    if (fsize > 0) {
        string = malloc(fsize);
        if (!string) {
            printk("no memory\r\n");
            return -2;
        }

        lfs_file_read(&lfs, &file, string, fsize);

        json = cJSON_Parse(string);
        if (NULL == json) {
            printk("JSON parse fail.\r\n");
            free(string);
            return -3;
        }
    } else {
        /* rebuild params */
        _rebuild_params();
    }

    return 0;
}

void _rebuild_params(void)
{
    json = cJSON_CreateObject();
    cJSON_AddItemToObject(json, "version", cJSON_CreateString("1.0.0"));

    char *buf = cJSON_Print(json);
    if (!buf) {
        printk("JSON print fail.\r\n");
        return;
    }

    lfs_file_write(&lfs, &file, buf, strlen(buf));
    free(buf);
    lfs_file_close(&lfs, &file);

    return;
}

int params_add_val(char *name, int32_t val)
{
    if (!cJSON_HasObjectItem(json, name)) {
        cJSON_AddItemToObject(json, name, cJSON_CreateNumber(val));
    } else {
        cJSON_ReplaceItemInObject(json, name, cJSON_CreateNumber(val));
    }
    return 0;
}

void params_del(char *name)
{
    cJSON_DeleteItemFromObject(json, name);
    return;
}

void params_save(void)
{
    char *buf = cJSON_Print(json);
    if (!buf) {
        printk("JSON print fail.\r\n");
        return;
    }
    
    lfs_file_rewind(&lfs, &file);
    lfs_file_write(&lfs, &file, buf, strlen(buf));
    free(buf);
    lfs_file_sync(&lfs, &file);
}

void params_show(void)
{
    char *buf = cJSON_Print(json);
    if (!buf) {
        printk("JSON print fail.\r\n");
        return;
    }

    printk("%s \e\n", buf);
}

#define  param_cmd_usage()                \
    shellUsage(chp, "param add name val"  \
                    "param show"          \
                    "param init"          \
                    "param save"          \
                    "param del name")
                     
void param_cmd(BaseSequentialStream *chp, int argc, char *argv[])
{
	int32_t cmd_type;
    int32_t val;

	if (argc > 3) {
        param_cmd_usage();
		return;
	} else if (argc == 1) {
		if (strcmp(argv[0], "show") == 0) {
			cmd_type = 2;
		} else if (strcmp(argv[0], "init") == 0) {
			cmd_type = 3;
		} else if (strcmp(argv[0], "save") == 0) {
            cmd_type = 4;
        } else {
			cmd_type = -1;
            param_cmd_usage();
			return;
		}
	} else if (argc == 3) {
		if (strcmp(argv[0], "add") == 0) {
			cmd_type = 1;
            val = atoi(argv[2]);
        }
    } else {
        if (strcmp(argv[0], "del") == 0) {
            cmd_type = 5;
        } else {
            param_cmd_usage();
            cmd_type = -1;
        }
    } 

    if (cmd_type == 1) {
        params_add_val(argv[1], val);
    } else if (cmd_type == 2) {
        params_show();
    } else if (cmd_type == 3) {
        params_init();
    } else if (cmd_type == 4) {
        params_save();
    } else if (cmd_type == 5) {
        params_del(argv[1]);
    }

    return;
}

