/* touch_synaptics.h
 *
 * Copyright (C) 2015 LGE.
 *
 * Author: hoyeon.jang@lge.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef LGE_TOUCH_SYNAPTICS_H
#define LGE_TOUCH_SYNAPTICS_H

#include <linux/device.h>

#define MAX_NUM_OF_FINGERS			10
#define PAGE_SELECT_REG				0xff
#define PAGES_TO_SERVICE			10
#define PDT_START				0x00e9
#define PDT_END					0x00D0

struct synaptics_f12_reg {
	u8 ctrl[32];
	u8 data[16];
};

struct synaptics_f51_reg {
	u8 lpwg_status_reg;
	u8 lpwg_data_reg;
	u8 lpwg_tapcount_reg;
	u8 lpwg_min_intertap_reg;
	u8 lpwg_max_intertap_reg;
	u8 lpwg_touch_slop_reg;
	u8 lpwg_tap_distance_reg;
	u8 lpwg_interrupt_delay_reg;
	u8 lpwg_tapcount_reg2;
	u8 lpwg_min_intertap_reg2;
	u8 lpwg_max_intertap_reg2;
	u8 lpwg_touch_slop_reg2;
	u8 lpwg_tap_distance_reg2;
	u8 lpwg_interrupt_delay_reg2;
	u8 overtap_cnt_reg;
	u8 request_reset_reg;
	u8 lpwg_partial_reg;
	u8 lpwg_fail_count_reg;
	u8 lpwg_fail_index_reg;
	u8 lpwg_fail_reason_reg;
	u8 lpwg_adc_offset_reg;
	u8 lpwg_adc_fF_reg1;
	u8 lpwg_adc_fF_reg2;
	u8 lpwg_adc_fF_reg3;
	u8 lpwg_adc_fF_reg4;
};

struct synaptics_f54_reg {
	u8 interference__metric_LSB;
	u8 interference__metric_MSB;
	u8 current_noise_status;
	u8 cid_im;
	u8 freq_scan_im;
	u8 incell_statistic;
};

struct synaptics_fw_info {
	u8 version[5];
	u8 product_id[11];
	u8 img_version[5];
	u8 img_product_id[11];
	u8 *fw_start;
	u8 family;
	u8 revision;
	unsigned long fw_size;
	u8 need_rewrite_firmware;
};

struct function_descriptor {
	u8 query_base;
	u8 command_base;
	u8 control_base;
	u8 data_base;
	u8 int_source_count;
	u8 fn_number;
};

struct synaptics_function {
	struct function_descriptor dsc;
	u8 page;
};

struct synaptics_object {
	u8 type;
	u8 x_lsb;
	u8 x_msb;
	u8 y_lsb;
	u8 y_msb;
	u8 z;
	u8 wx;
	u8 wy;
};

struct synaptics_data {
	struct synaptics_function f01;
	struct synaptics_function f11;
	struct synaptics_function f12;
	struct synaptics_function f1a;
	struct synaptics_function f34;
	struct synaptics_function f51;
	struct synaptics_function f54;
	struct synaptics_function f55;
	struct synaptics_f12_reg f12_reg;
	struct synaptics_f51_reg f51_reg;
	struct synaptics_f54_reg f54_reg;
	struct synaptics_fw_info fw;
	u8 curr_page;
	u8 object_report;
	u8 num_of_fingers;
	u8 irq_mask;
};

static inline struct synaptics_data *to_synaptics_data(struct device *dev)
{
	return (struct synaptics_data *)touch_get_device(to_touch_core(dev));
}
#endif /* LGE_TOUCH_SYNAPTICS_H */
