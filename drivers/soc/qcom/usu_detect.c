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

int __init file_read(struct file *file, unsigned long long offset, unsigned char *data, unsigned int size) 
{
    printk("UsU: (%s) reading offset %llu\n", __func__, offset);
/**
    mm_segment_t oldfs;
    int ret;

    oldfs = get_fs();
    set_fs(get_ds());

    ret = vfs_read(file, data, size, &offset);

    set_fs(oldfs);
    return ret;
**/
    return 0;
}  

void __init file_close(struct file *file) 
{
    printk("UsU (%s): closing disk\n", __func__);
    //filp_close(file, NULL);
}

static int __init get_usu_model(void)
{
    pr_info("UsU (%s): usu_partition: %s, usu_offset: %lu, usu_model: %s\n", __func__, usu_partition, usu_offset, usu_model);
    f = filp_open(usu_partition,O_RDONLY,0644);
    file_read(f, usu_offset, usu_model, 7);
    file_close(f);
    printk("UsU (%s): model: %s\n", __func__, usu_model);
    return 0;
}

/** command-line is loaded at core_initcall() **/
subsys_initcall(get_usu_model);
