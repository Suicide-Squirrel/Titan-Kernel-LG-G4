/* Copyright (c) 2018, steadfasterX <steadfasterX * gmail DOT com>.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * version 3, or any later versions as published by the Free Software
 * Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/fs.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>

#include <linux/init.h>
#include <linux/string.h>
#include <linux/printk.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/ctype.h>

static char usu_partition[200] __initdata=CONFIG_USU_PARTITION;
//static size_t usu_offset[200] __initdata=CONFIG_USU_PARTITION_OFFSET;
static size_t usu_offset __initdata=3145722;
static char usu_model[200] __initdata="undef";
struct file *f;

struct __init file *file_open(const char *path, int flags, int rights) 
{
    pr_info("UsU (%s): reading file %s\n", __func__, path);

    struct file *filp = NULL;
    mm_segment_t oldfs;
    int err = 0;

    oldfs = get_fs();
    set_fs(KERNEL_DS);

    filp = filp_open(path, flags, rights);
    set_fs(oldfs);
    if (IS_ERR(filp)) {
    	pr_info("UsU (%s): error %d while opening file\n", __func__, PTR_ERR(filp));
        err = PTR_ERR(filp);
        return NULL;
    }

    pr_info("UsU (%s): end\n", __func__);
    return filp;
    //return 0;
}

int __init file_read(struct file *file, unsigned long long offset, unsigned char *data, unsigned int size) 
{
    pr_info("UsU (%s): reading offset %llu\n", __func__, offset);

    mm_segment_t oldfs;
    int ret;

    oldfs = get_fs();
    set_fs(get_ds());

    ret = vfs_read(file, data, size, &offset);

    set_fs(oldfs);
    pr_info("UsU (%s): end\n", __func__);
    return ret;
}

void __init file_close(struct file *file) 
{
    pr_info("UsU (%s): closing disk\n", __func__);
    //filp_close(file, NULL);
    pr_info("UsU (%s): end\n", __func__);
}

static int __init get_usu_model(void)
{
    pr_info("UsU (%s): usu_partition: %s, usu_offset: %lu, usu_model: %s\n", __func__, usu_partition, usu_offset, usu_model);
    f = file_open(usu_partition, O_RDONLY, 0);
    //file_read(f, usu_offset, usu_model, 7);
    file_close(f);
    pr_info("UsU (%s): model: %s\n", __func__, usu_model);
    pr_info("UsU (%s): end\n", __func__);
    return 0;
}

/** command-line is loaded at core_initcall() **/
//subsys_initcall(get_usu_model);

// https://stackoverflow.com/questions/18605653/module-init-vs-core-initcall-vs-early-initcall#18606561
late_initcall(get_usu_model);

