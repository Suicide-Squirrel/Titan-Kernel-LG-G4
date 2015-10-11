/*
 * drivers/soc/qcom/lge/lge_misc_checker.c
 *
 * Copyright (C) 2014 LGE, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <asm/uaccess.h>
#include <soc/qcom/lge/board_lge.h>


#define MODULE_NAME "misc_checker"
#define MISC_PATH "/dev/block/platform/f9824900.sdhci/by-name/misc"
#define FTM_ITEM_NAME_MAX_LEN   32
#define LGFTM_ITEM_MAX          3585

struct ftm_header {
	uint32_t header_length;
	uint32_t total_length;
	uint32_t item_size;
	uint32_t item_count;
};

struct ftm_header *header_info;

struct ftm_item {
	char name[FTM_ITEM_NAME_MAX_LEN];
	unsigned int index;
	unsigned int size;
};

struct ftm_items {
	struct ftm_item *data;
	struct list_head item;
};

enum {
	FTM_VALUE_BYTE,
	FTM_VALUE_STRING,
};

static unsigned int index_read;
static unsigned int index_write;
static unsigned int value_type;
static int dummy_arg;
static int misc_checker_index;

static DEFINE_MUTEX(ftm_lock);
static LIST_HEAD(ftm_item_list);

struct file *misc_open(void)
{
	struct file *filp;

	filp = filp_open(MISC_PATH, O_RDWR, S_IRUSR|S_IWUSR);

	if (IS_ERR(filp)) {
		pr_err("Can't open misc partition : %ld\n", IS_ERR(filp));
		return NULL;
	}

	return filp;
}

static int check_index(int id)
{
	if (id < 1 || id > LGFTM_ITEM_MAX)
		return -1;

	return 0;
}

int set_ftm_item(int id, int size, void *in)
{
	struct file *filp;
	int ret = 0;
	int len;
	void *data;
	mm_segment_t oldfs;

	if (check_index(id)) {
		pr_err("Invalid id of ftm_item, %d\n", id);
		return -1;
	}

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	filp = misc_open();

	if (filp != NULL) {
		filp->f_pos = id * 1024 * 2;

		data = kmalloc(size, GFP_KERNEL);
		if (data == NULL) {
			pr_err("Failed to allocate buffer\n");
			ret = -1;
			goto fail_mem_allocate;
		}

		memcpy(data, in, size);

		len = vfs_write(filp, (char *)data, size, &filp->f_pos);
		if (len < 0) {
			pr_err("Failed to read ftm_item\n");
			ret = -1;
			goto fail_vfs_read;
		}
	} else {
		ret = -1;
		goto fail_misc_open;
	}

fail_vfs_read:
	kfree(data);
fail_mem_allocate:
	filp_close(filp, NULL);
fail_misc_open:
	set_fs(oldfs);
	return ret;
}
EXPORT_SYMBOL(set_ftm_item);

int get_ftm_item(int id, int size, void *out)
{
	struct file *filp;
	int ret = 0;
	int len;
	void *data;
	mm_segment_t oldfs;

	if (check_index(id)) {
		pr_err("Invalid id of ftm_item, %d\n", id);
		return -1;
	}

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	filp = misc_open();

	if (filp != NULL) {
		filp->f_pos = id * 1024 * 2;
		data = kmalloc(size, GFP_KERNEL);
		if (data == NULL) {
			pr_err("Failed to allocate buffer\n");
			ret = -1;
			goto fail_mem_allocate;
		}

		len = vfs_read(filp, (char *)data, size, &filp->f_pos);
		if (len < 0) {
			pr_err("Failed to read ftm_item\n");
			ret = -1;
			goto fail_vfs_read;
		}

		memcpy(out, data, size);
	} else {
		ret = -1;
		goto fail_misc_open;
	}

fail_vfs_read:
	kfree(data);
fail_mem_allocate:
	filp_close(filp, NULL);
fail_misc_open:
	set_fs(oldfs);
	return ret;
}
EXPORT_SYMBOL(get_ftm_item);

static struct ftm_item *get_ftm_item_entry(int id)
{
	struct ftm_items *node;

	list_for_each_entry(node, &ftm_item_list, item) {
		if (id == node->data->index)
			return node->data;
	}

	return NULL;
}

static int ftm_header_init(void)
{
	char *value;
	int size;

	size = sizeof(struct ftm_header);
	value = kmalloc(size, GFP_KERNEL);
	if (value == NULL)
		return -1;

	if (get_ftm_item(misc_checker_index, size, value)) {
		kfree(value);
		return -1;
	}

	header_info = (struct ftm_header *)value;

	return 0;
}

static int get_ftm_item_all(int size, void *out)
{
	struct file *filp;
	int ret = 0;
	int len;
	void *data;
	mm_segment_t oldfs;

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	filp = misc_open();


	if (filp != NULL) {
		filp->f_pos = misc_checker_index * 1024 * 2 + header_info->header_length;
		data = kmalloc(size, GFP_KERNEL);
		if (data == NULL) {
			pr_err("Failed to allocate buffer\n");
			ret = -1;
			goto fail_mem_allocate;
		}

		len = vfs_read(filp, (char *)data, size, &filp->f_pos);
		if (len < 0) {
			pr_err("Failed to read ftm_item\n");
			ret = -1;
			goto fail_vfs_read;
		}

		memcpy(out, data, size);
	} else {
		ret = -1;
		goto fail_misc_open;
	}

fail_vfs_read:
	kfree(data);
fail_mem_allocate:
	filp_close(filp, NULL);
fail_misc_open:
	set_fs(oldfs);
	return ret;
}

static int ftm_item_list_init(void)
{
	struct ftm_items *node;
	char *value;
	int size;
	int i;
	struct ftm_item *item;

	size = header_info->total_length - header_info->header_length;
	value = kmalloc(size, GFP_KERNEL);
	if (value == NULL)
		return -1;

	get_ftm_item_all(size, value);
	item = (struct ftm_item *)value;

	mutex_lock(&ftm_lock);
	for (i = 0; i < header_info->item_count; i++) {
		node = kmalloc(sizeof(struct ftm_items), GFP_KERNEL);
		if (node == NULL)
			return -1;

		node->data = item++;
		list_add_tail(&node->item, &ftm_item_list);
	}
	mutex_unlock(&ftm_lock);

	return 0;
}

static int create_ftm_item_list(void)
{
	if (ftm_header_init()) {
		pr_err("ftm_header_init() fail\n");
		return -1;
	}

	if (ftm_item_list_init()) {
		pr_err("ftm_item_list_init() fail\n");
		return -1;
	}

	return 0;
}

static int get_ftm_list(char *buffer, const struct kernel_param *kp)
{
	struct ftm_items *node;
	int len = 0;

	if (list_empty(&ftm_item_list)) {
		if (create_ftm_item_list()) {
			len += snprintf(buffer, PAGE_SIZE-len, "No data\n");
			return len;
		}
	}

	list_for_each_entry(node, &ftm_item_list, item) {
		len += snprintf(buffer+len, PAGE_SIZE-len, "%s, index=%d, size=%d\n",
			node->data->name, node->data->index, node->data->size);
	}

	return len;
}

module_param_call(ftm_list, NULL, get_ftm_list,
					&dummy_arg, S_IWUSR | S_IRUGO);


static int set_ftm_item_read(const char *val, struct kernel_param *kp)
{
	int ret = 0;
	char index_str[10];
	int index;

	if (list_empty(&ftm_item_list)) {
		if (create_ftm_item_list()) {
			pr_err("create ftm item list fail\n");
			return -1;
		}
	}

	if (1 == sscanf(val, "%s", index_str)) {
		ret = kstrtoint(index_str, 10, &index);
		if (ret == 0) {
			if (check_index(index) != 0) {
				pr_err("invalid index : %d\n", index);
				return -1;
			} else {
				index_read = index;
			}
		} else {
			pr_err("invalid index type : %d\n", index);
			return -1;
		}
	} else {
		pr_err("invalid format\n");
		return -1;
	}

	return ret;
}

static int get_ftm_item_read(char *buffer, const struct kernel_param *kp)
{
	struct ftm_item *item;
	char *value;

	if (list_empty(&ftm_item_list)) {
		pr_err("No data\n");
		return -1;
	}

	item = get_ftm_item_entry(index_read);
	if (item == NULL) {
		pr_err("invalid index : %d\n", index_read);
		return -1;
	}

	value = kmalloc(item->size + 1, GFP_KERNEL);
	if (value == NULL)
		return -1;

	get_ftm_item(index_read, item->size, value);
	value[item->size] = '\0';

	if (item->size == 1)
		return sprintf(buffer, "%s = %d", item->name, (char)value[0]);
	else
		return sprintf(buffer, "%s = %s", item->name, value);
}

module_param_call(ftm_item_read, set_ftm_item_read, get_ftm_item_read,
					&dummy_arg, S_IWUSR | S_IRUGO);

static int set_ftm_item_write(const char *val, struct kernel_param *kp)
{
	struct ftm_item *item;
	int ret = 0;
	char index_str[10];
	char value_str[512];
	int index;
	int value;
	char num;

	if (list_empty(&ftm_item_list)) {
		if (create_ftm_item_list()) {
			pr_err("create ftm item list fail\n");
			return -1;
		}
	}

	if (2 == sscanf(val, "%s %s", index_str, value_str)) {
		ret = kstrtoint(index_str, 10, &index);
		if (ret == 0) {
			if (check_index(index) != 0) {
				pr_err("invalid index : %d\n", index);
				return -1;
			} else {
				index_write = index;
				item = get_ftm_item_entry(index_write);
				if (item == NULL) {
					pr_err("invalid index : %d\n", index_write);
					return -1;
				}
			}
		} else {
			pr_err("invalid index type : %d\n", index);
			return -1;
		}

		ret = kstrtoint(value_str, 10, &value);
		if (ret == 0) {
			num = (char)value;
			value_type = FTM_VALUE_BYTE;
			set_ftm_item(index, item->size, &num);
		} else {
			value_type = FTM_VALUE_STRING;
			set_ftm_item(index, item->size, value_str);
		}
	} else {
		pr_err("invalid format\n");
		return -1;
	}

	return ret;
}

static int get_ftm_item_write(char *buffer, const struct kernel_param *kp)
{
	struct ftm_item *item;
	char *value;

	if (list_empty(&ftm_item_list)) {
		pr_err("No data\n");
		return -1;
	}

	item = get_ftm_item_entry(index_write);
	if (item == NULL) {
		pr_err("invalid index : %d\n", index_write);
		return -1;
	}

	value = kmalloc(item->size + 1, GFP_KERNEL);
	if (value == NULL)
		return -1;

	get_ftm_item(index_write, item->size, value);
	value[item->size] = '\0';

	if (value_type == FTM_VALUE_BYTE)
		return sprintf(buffer, "%s = %d", item->name, (char)value[0]);
	else
		return sprintf(buffer, "%s = %s", item->name, value);
}

module_param_call(ftm_item_write, set_ftm_item_write, get_ftm_item_write,
					&dummy_arg, S_IWUSR | S_IRUGO);

static int misc_checker_probe(struct platform_device *pdev)
{
	struct device_node *node;

	node = of_find_node_by_path("/chosen");
	if (node == NULL) {
		pr_err("%s: chosen node not found\n", __func__);
		return -ENODEV;
	}

	if(of_property_read_u32(node, "lge,misc_checker_index", (u32 *)&misc_checker_index)) {
		pr_err("%s: misc_checker_index not found\n", __func__);
		return -ENODEV;
	}

	return 0;
}

static int misc_checker_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver misc_checker_driver = {
	.probe = misc_checker_probe,
	.remove = misc_checker_remove,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
	},
};

static struct platform_device misc_checker_device = {
	.name = MODULE_NAME,
	.dev = {
		.platform_data = NULL,
	}
};

static int __init misc_checker_init(void)
{
	platform_device_register(&misc_checker_device);
	return platform_driver_register(&misc_checker_driver);
}

static void __exit misc_checker_exit(void)
{
	platform_driver_unregister(&misc_checker_driver);
}

module_init(misc_checker_init);
module_exit(misc_checker_exit);

MODULE_DESCRIPTION("LGE misc checker");
MODULE_AUTHOR("TaeBum Kim <taebum81.kim@lge.com>");
MODULE_LICENSE("GPL");
