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
#include "list.h"
#include "shell.h"
#include "usbcfg.h"
#include "cmsis_os.h"
#include "system.h"
#include "lfs.h"
#include "lfs_port.h"
#include "cJSON.h"
#include "params.h"
#include "re.h"

// variables used by the filesystem
lfs_t lfs;
lfs_file_t file;

/*  internal list */
LIST_HEAD(param_list_head);
//osSemaphoreId param_sem;
cJSON *json;

typedef struct params_module_s {
    struct list_head head;
    uint16_t moduleId;
    uint16_t numOfParams;
    char *module_name;
    param_numbers_get numbers_get_fn;
    param_default_load default_load_fn;
} PARAMS_MODULE_T;


static void _rebuild_params(void);
static int params_add_val(char *name, int32_t type, PARAM_DATA *data);
static void params_del(char *name);
static void params_file_del(void);
static void params_save(void);


PARAMS_MODULE_T *_param_module_find(uint16_t moduleId)
{
    struct list_head *pos;
    PARAMS_MODULE_T *entry;
    bool isFind = false;

    //osSemaphoreWait(param_sem, osWaitForever);
    list_for_each(pos, &param_list_head) {
        entry = list_entry(pos, PARAMS_MODULE_T, head);
        if (entry->moduleId == moduleId) {
            isFind = true;
            break;
        }
    }

    //osSemaphoreRelease(param_sem);
    if (isFind) {
        return entry;
    } else {
        return NULL;
    }
}


int param_cb_register(uint16_t moduleid, char *name, param_numbers_get func_get, param_default_load func_load)
{
    PARAMS_MODULE_T *entry;
    uint32_t len = strlen(name);
    
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
    entry->module_name = malloc(len + 1);
    if (!entry->module_name) {
        free(entry);
        return -3;
    }
    memcpy(entry->module_name, name, len);
    entry->module_name[len] = '\0';

    //osSemaphoreWait(param_sem, osWaitForever);
    list_add(&entry->head, &param_list_head);
    //osSemaphoreRelease(param_sem);

    return 0;
}


int params_init(void)
{
    int ret;
    int32_t fsize;
    char *string;
    static bool inited = false;
    
    if (inited) {
        return 0;
    }

    /*
    param_sem = osSemaphoreCreate(NULL, 1);
    if (!param_sem) {
        return -1;
    }
    */
    
    lfs_port_mount(&lfs);

    ret = lfs_file_open(&lfs, &file, "cifc_params", LFS_O_RDWR | LFS_O_CREAT);
    if (LFS_ERR_OK != ret) {
        printk("lfs_file_open fail ret %d.\r\n", ret);
        return ret;
    }

    printk("file open success.\r\n");

    fsize = lfs_file_size(&lfs, &file);
    if (fsize < 0) {
        printk("file size get err fsize = %d.\r\n", (int)fsize);
        return -1;
    }

    printk("Get file size %d.\r\n", (int)fsize);

    if (fsize > 0) {
        string = malloc(fsize);
        if (!string) {
            printk("no memory\r\n");
            lfs_file_close(&lfs, &file);
            return -2;
        }

        lfs_file_read(&lfs, &file, string, fsize);

        json = cJSON_Parse(string);
        if (NULL == json) {
            printk("JSON parse fail.\r\n");
            lfs_file_close(&lfs, &file);
            free(string);
            return -3;
        }
        free(string);
    } else {
        printk("rebuild params.\r\n");
        /* rebuild params */
        _rebuild_params();
    }

    lfs_file_close(&lfs, &file);
    inited = true;
    return 0;
}

void _param_def_load(void)
{
    struct list_head *pos;
    PARAMS_MODULE_T *module;
    PARAM_ENTRY_T entry;
    uint32_t maxOfparam;
    uint32_t i;
    char *pname;
    uint32_t len1, len2;

    //osSemaphoreWait(param_sem, osWaitForever);
    list_for_each(pos, &param_list_head) {
        module = list_entry(pos, PARAMS_MODULE_T, head);
        maxOfparam = module->numbers_get_fn(module->moduleId);
        module->numOfParams = maxOfparam;
        for (i = 0; i < maxOfparam; i++) {
            module->default_load_fn(module->moduleId, i, &entry);
            len1 = strlen(module->module_name); 
            len2 = strlen(entry.entry_name);
            pname = malloc(len1 + len2 + 2);
            if (!pname) {
                return;
            }
            memcpy(pname, module->module_name, len1);
            pname[len1] = '_';
            memcpy(pname + len1 + 1, entry.entry_name, len2);
            pname[len1 + len2 + 1] = '\0';

            switch (entry.type) {
                case param_type_int:
                    (void)params_add_val(pname, entry.type, &entry.data);
                    break;
                case param_type_float:
                    (void)params_add_val(pname, entry.type, &entry.data);
                    break;
                default:
                    break;
            }

            if (pname) {
                free(pname);
            }
        }
    }

    //osSemaphoreRelease(param_sem);
    return;
}

void _rebuild_params(void)
{
    json = cJSON_CreateObject();
    cJSON_AddItemToObject(json, "version", cJSON_CreateString("1.0.0"));

    _param_def_load();

    char *buf = cJSON_Print(json);
    if (!buf) {
        printk("JSON print fail.\r\n");
        return;
    }

    lfs_file_write(&lfs, &file, buf, strlen(buf));
    free(buf);

    return;
}

int params_add_val(char *name, int32_t type, PARAM_DATA *data)
{
    bool ret;
    cJSON *item;

    switch(type) {
        case param_type_int:
            item = cJSON_CreateNumber(data->i);
            break;
        case param_type_float:
            item = cJSON_CreateNumber(data->f);
            break;
        default:
            return -1;
    }

    if (!cJSON_HasObjectItem(json, name)) {
        ret = cJSON_AddItemToObject(json, name, item);
    } else {
        ret = cJSON_ReplaceItemInObject(json, name, item);
    }

    if (!ret) {
        return -2;
    }
    
    return 0;
}

void params_del(char *name)
{
    cJSON_DeleteItemFromObject(json, name);
    return;
}

int params_update(char *name, int32_t type, PARAM_DATA *data)
{
    cJSON *item;
    
    switch(type) {
        case param_type_int:
            item = cJSON_CreateNumber(data->i);
            break;
        case param_type_float:
            item = cJSON_CreateNumber(data->f);
            break;
        default:
            return -1;
    }
    
    if (!cJSON_HasObjectItem(json, name)) {
        return -1;
    }

    if (!cJSON_ReplaceItemInObject(json, name, item)) {
        return -2;
    }

    params_save();
    return 0;
}

int params_get(char *name, int32_t type, PARAM_DATA *val)
{
    cJSON *item;
    
    item = cJSON_GetObjectItemCaseSensitive(json, name);
    if (!item) {
        return -1;
    }

    if (!cJSON_IsNumber(item)) {
        return -2;
    }

    switch(type) {
        case param_type_int:
            val->i = (uint32_t)cJSON_GetNumberValue(item);
            break;
        case param_type_float:
            val->f = (float)cJSON_GetNumberValue(item);
            break;
        default:
            return -3;
    }
            
    return 0;
}

void params_save(void)
{
    int ret;
    char *buf = cJSON_Print(json);
    if (!buf) {
        printk("JSON print fail.\r\n");
        return;
    }

    ret = lfs_file_open(&lfs, &file, "cifc_params", LFS_O_RDWR | LFS_O_CREAT);
    if (LFS_ERR_OK != ret) {
        printk("lfs_file_open fail ret %d.\r\n", ret);
        free(buf);
        return;
    }
    
    lfs_file_rewind(&lfs, &file);
    lfs_file_write(&lfs, &file, buf, strlen(buf));
    lfs_file_close(&lfs, &file);
    free(buf);
}

void params_file_del(void)
{
    lfs_remove(&lfs, "cifc_params");
}

void params_format(void) 
{
    lfs_port_format(&lfs);
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

#define  param_cmd_usage()                    \
    shellUsage(chp, "param add name val\r\n"  \
                    "param show\r\n"          \
                    "param init\r\n"          \
                    "param save\r\n"          \
                    "param del name\r\n"      \
                    "param get name type\r\n" \
                    "param file del\r\n"      \
                    "param format")
                     
void param_cmd(BaseSequentialStream *chp, int argc, char *argv[])
{
	int32_t cmd_type;
    int32_t type;
    re_t pattern;
    PARAM_DATA data;
    int match_len, ret;

    pattern = re_compile("[+-]?[0-9]+[.][0-9]+");

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
        } else if (strcmp(argv[0], "format") == 0) {
            cmd_type = 8;
        } else {
			cmd_type = -1;
            param_cmd_usage();
			return;
		}
	} else if (argc == 3) {
		if (strcmp(argv[0], "add") == 0) {
			cmd_type = 1;
            
            ret = re_matchp(pattern, argv[2], &match_len);
            if (ret != -1) {
                type = param_type_float;
                data.f = atof(argv[2]);
            } else {
                type = param_type_int;
                data.i = atoi(argv[2]);
            }
        } else if (strcmp(argv[0], "get") == 0) {
            cmd_type = 6;
            type = atoi(argv[2]);
        } 
    } else {
        if (strcmp(argv[0], "del") == 0) {
            cmd_type = 5;
        } else if (strcmp(argv[0], "file") == 0) {
            cmd_type = 7;
        } else {
            param_cmd_usage();
            cmd_type = -1;
        }
    } 

    if (cmd_type == 1) {
        params_add_val(argv[1], type, &data);
    } else if (cmd_type == 2) {
        params_show();
    } else if (cmd_type == 3) {
        params_init();
    } else if (cmd_type == 4) {
        params_save();
    } else if (cmd_type == 5) {
        params_del(argv[1]);
    } else if (cmd_type == 6) {
        params_get(argv[1], type, &data);
        switch (type) {
            case param_type_int:
                printk("param name %s val %d.\r\n", argv[1], data.i);
                break;
            case param_type_float:
                printk("param name %s val %f.\r\n", argv[1], data.f);
                break;
            default:
                printk("param name %s unknown type.\r\n", argv[1]);
                break;
        }
    } else if (cmd_type == 7) {
        params_file_del();
    } else if (cmd_type == 8) {
        params_format();
    }

    return;
}

