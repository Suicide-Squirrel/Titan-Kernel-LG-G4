/* touch_synaptics.c
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include "touch_hwif.h"
#include "touch_core.h"
#include "touch_synaptics.h"

/*
 * PLG349 - Z2
 * PLG446 - P1(JDI)
 * PLG469 - P1(LGD)
 */

#define DEFAULT_PAGE			0x00
#define COMMON_PAGE			(d->f01.page)
#define FINGER_PAGE			(d->f12.page)
#define ANALOG_PAGE			(d->f54.page)
#define FLASH_PAGE			(d->f34.page)
#define LPWG_PAGE			(d->f51.page)

#define DEVICE_CONTROL_REG		(d->f01.dsc.control_base + 0)
#define DEVICE_CONTROL_SLEEP		0x01
#define	DEVICE_CONTROL_NOSLEEP		0x04

#define INTERRUPT_ENABLE_REG		(d->f01.dsc.control_base + 1)
#define DOZE_INTERVAL_REG		(d->f01.dsc.control_base + 2)

#define DEVICE_STATUS_REG		(d->f01.dsc.data_base + 0)
#define INTERRUPT_STATUS_REG		(d->f01.dsc.data_base + 1)
#define INTERRUPT_MASK_FLASH		(1 << 0)
#define INTERRUPT_MASK_STATUS		(1 << 1)
#define INTERRUPT_MASK_ABS0		(1 << 2)
#define INTERRUPT_MASK_BUTTON		(1 << 4)
#define INTERRUPT_MASK_CUSTOM		(1 << 6)
#define INTERRUPT_MASK_LPWG		INTERRUPT_MASK_CUSTOM

#define FINGER_DATA_REG			(d->f12.dsc.data_base + 0)
#define F12_NO_OBJECT_STATUS		(0x00)
#define F12_FINGER_STATUS		(0x01)
#define F12_STYLUS_STATUS		(0x02)
#define F12_PALM_STATUS			(0x03)
#define F12_HOVERING_FINGER_STATUS	(0x05)
#define F12_GLOVED_FINGER_STATUS	(0x06)
#define F12_MAX_OBJECT			(0x06)

#define LPWG_STATUS_REG			(d->f51.dsc.data_base)
#define LPWG_STATUS_DOUBLETAP		(1 << 0)
#define LPWG_STATUS_PASSWORD		(1 << 1)
#define LPWG_DATA_REG			(d->f51.dsc.data_base + 1)
#define LPWG_OVER_TAPCOUNT		(d->f51.dsc.data_base + 73)

#define LPWG_TAPCOUNT_REG		(d->f51.dsc.control_base)
#define LPWG_MIN_INTERTAP_REG		(d->f51.dsc.control_base + 1)
#define LPWG_MAX_INTERTAP_REG		(d->f51.dsc.control_base + 2)
#define LPWG_TOUCH_SLOP_REG		(d->f51.dsc.control_base + 3)
#define LPWG_TAP_DISTANCE_REG		(d->f51.dsc.control_base + 4)
#define LPWG_INTERRUPT_DELAY_REG	(d->f51.dsc.control_base + 6)

#define LPWG_TAPCOUNT_REG2		(d->f51.dsc.control_base + 7)
#define LPWG_MIN_INTERTAP_REG2		(d->f51.dsc.control_base + 8)
#define LPWG_MAX_INTERTAP_REG2		(d->f51.dsc.control_base + 9)
#define LPWG_TOUCH_SLOP_REG2		(d->f51.dsc.control_base + 10)
#define LPWG_TAP_DISTANCE_REG2		(d->f51.dsc.control_base + 11)
#define LPWG_INTERRUPT_DELAY_REG2	(d->f51.dsc.control_base + 13)

#if defined(ENABLE_TCI_DEBUG)
#define LPWG_TCI1_FAIL_COUNT_REG	(d->f51.dsc.data_base + 49)
#define LPWG_TCI1_FAIL_INDEX_REG	(d->f51.dsc.data_base + 50)
#define LPWG_TCI1_FAIL_BUFFER_REG	(d->f51.dsc.data_base + 51)
#define LPWG_TCI2_FAIL_COUNT_REG	(d->f51.dsc.data_base + 61)
#define LPWG_TCI2_FAIL_INDEX_REG	(d->f51.dsc.data_base + 62)
#define LPWG_TCI2_FAIL_BUFFER_REG	(d->f51.dsc.data_base + 63)
#endif

#define FINGER_REPORT_REG		(d->f12_reg.ctrl[20])


static int synaptics_read(struct device *dev, u16 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct touch_bus_msg msg;
	int ret = 0;

	ts->tx_buf[0] = addr;

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = 1;

	msg.rx_buf = ts->rx_buf;
	msg.rx_size = size;

	ret = touch_bus_read(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus read error : %d\n", ret);
		return ret;
	}

	memcpy(data, &ts->rx_buf[0], size);
	return 0;
}

static int synaptics_write(struct device *dev, u16 addr, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct touch_bus_msg msg;
	int ret = 0;

	ts->tx_buf[0] = addr;
	memcpy(&ts->tx_buf[1], data, size);

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = size+1;
	msg.rx_buf = NULL;
	msg.rx_size = 0;

	ret = touch_bus_write(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus write error : %d\n", ret);
		return ret;
	}

	return 0;
}

static int synaptics_irq_enable(struct device *dev, bool enable)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	u8 val;
	int ret;

	ret = synaptics_read(dev, INTERRUPT_ENABLE_REG, &val, sizeof(val));
	if (ret < 0) {
		TOUCH_E("failed to read interrupt enable - ret:%d\n", ret);
		return ret;
	}

	if (enable)
		val |= (INTERRUPT_MASK_ABS0 | INTERRUPT_MASK_CUSTOM);
	else
		val &= ~INTERRUPT_MASK_ABS0;


	ret = synaptics_write(dev, INTERRUPT_ENABLE_REG, &val, sizeof(val));
	if (ret < 0) {
		TOUCH_E("failed to write interrupt enable - ret:%d\n", ret);
		return ret;
	}

	TOUCH_I("write interrupt : enable:%d, val:%02X\n", enable, val);

	return 0;
}

static int synaptics_set_page(struct device *dev, u8 page)
{
	int ret = synaptics_write(dev, PAGE_SELECT_REG, &page, 1);

	if (ret >= 0)
		to_synaptics_data(dev)->curr_page = page;

	return ret;
}

static int synaptics_get_f12(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	u8 query_5_data[5];
	u8 query_8_data[3];
	u8 ctrl_23_data[2];
	u8 ctrl_8_data[14];

	u32 query_5_present = 0;
	u16 query_8_present = 0;

	u8 offset;
	int i;
	int ret;

	ret = synaptics_read(dev, d->f12.dsc.query_base + 5,
			     query_5_data, sizeof(query_5_data));

	if (ret < 0) {
		TOUCH_E("faied to get query5 (ret: %d)\n", ret);
		return ret;
	}

	query_5_present = (query_5_present << 8) | query_5_data[4];
	query_5_present = (query_5_present << 8) | query_5_data[3];
	query_5_present = (query_5_present << 8) | query_5_data[2];
	query_5_present = (query_5_present << 8) | query_5_data[1];
	TOUCH_I("qeury_5_present=0x%08X [%02X %02X %02X %02X %02X]\n",
			query_5_present, query_5_data[0], query_5_data[1],
			query_5_data[2], query_5_data[3], query_5_data[4]);

	for (i = 0, offset = 0; i < 32; i++) {
		d->f12_reg.ctrl[i] = d->f12.dsc.control_base + offset;

		if (query_5_present & (1 << i)) {
			TOUCH_I("f12_reg.ctrl[%d]=0x%02X (0x%02x+%d)\n",
					i, d->f12_reg.ctrl[i],
					d->f12.dsc.control_base, offset);
			offset++;
		}
	}

	ret = synaptics_read(dev, d->f12.dsc.query_base + 8,
			query_8_data, sizeof(query_8_data));

	if (ret < 0) {
		TOUCH_E("faied to get query8 (ret: %d)\n", ret);
		return ret;
	}

	query_8_present = (query_8_present << 8) | query_8_data[2];
	query_8_present = (query_8_present << 8) | query_8_data[1];
	TOUCH_I("qeury_8_present=0x%08X [%02X %02X %02X]\n",
			query_8_present, query_8_data[0],
			query_8_data[1], query_8_data[2]);

	for (i = 0, offset = 0; i < 16; i++) {
		d->f12_reg.data[i] = d->f12.dsc.data_base + offset;

		if (query_8_present & (1 << i)) {
			TOUCH_I("d->f12_reg.data[%d]=0x%02X (0x%02x+%d)\n",
					i, d->f12_reg.data[i],
					d->f12.dsc.data_base, offset);
			offset++;
		}
	}

	ret = synaptics_read(dev, d->f12_reg.ctrl[23],
			     ctrl_23_data, sizeof(ctrl_23_data));

	if (ret < 0) {
		TOUCH_E("faied to get f12_ctrl32_data (ret: %d)\n", ret);
		return ret;
	}

	d->object_report = ctrl_23_data[0];
	d->num_of_fingers = min_t(u8, ctrl_23_data[1], (u8) MAX_NUM_OF_FINGERS);

	TOUCH_I("object_report[0x%02X], num_of_fingers[%d]\n",
			d->object_report, d->num_of_fingers);

	ret = synaptics_read(dev, d->f12_reg.ctrl[8],
			     ctrl_8_data, sizeof(ctrl_8_data));

	if (ret < 0) {
		TOUCH_E("faied to get f12_ctrl8_data (ret: %d)\n", ret);
		return ret;
	}

	TOUCH_I("ctrl_8-sensor_max_x[%d], sensor_max_y[%d]\n",
			((u16)ctrl_8_data[0] << 0) |
			((u16)ctrl_8_data[1] << 8),
			((u16)ctrl_8_data[2] << 0) |
			((u16)ctrl_8_data[3] << 8));

	return 0;
}

static int synaptics_page_description(struct device *dev)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	struct function_descriptor dsc;
	u8 page;

	unsigned short pdt;
	int ret;

	TOUCH_TRACE();

	memset(&d->f01, 0, sizeof(struct synaptics_function));
	memset(&d->f11, 0, sizeof(struct synaptics_function));
	memset(&d->f12, 0, sizeof(struct synaptics_function));
	memset(&d->f1a, 0, sizeof(struct synaptics_function));
	memset(&d->f34, 0, sizeof(struct synaptics_function));
	memset(&d->f51, 0, sizeof(struct synaptics_function));
	memset(&d->f54, 0, sizeof(struct synaptics_function));
	memset(&d->f55, 0, sizeof(struct synaptics_function));

	for (page = 0; page < PAGES_TO_SERVICE; page++) {
		ret = synaptics_set_page(dev, page);

		if (ret < 0) {
			TOUCH_E("faied to set page %d (ret: %d)\n", page, ret);
			return ret;
		}

		for (pdt = PDT_START; pdt > PDT_END; pdt -= sizeof(dsc)) {
			ret = synaptics_read(dev, pdt, &dsc, sizeof(dsc));

			if (ret < 0) {
				TOUCH_E("read descrptore %d (ret: %d)\n",
					pdt, ret);
				return ret;
			}

			if (!dsc.fn_number)
				break;

			TOUCH_I("dsc - %02x, %02x, %02x, %02x, %02x, %02x\n",
				dsc.query_base, dsc.command_base,
				dsc.control_base, dsc.data_base,
				dsc.int_source_count, dsc.fn_number);

			switch (dsc.fn_number) {
			case 0x01:
				d->f01.dsc = dsc;
				d->f01.page = page;
				break;

			case 0x11:
				d->f11.dsc = dsc;
				d->f11.page = page;
				break;

			case 0x12:
				d->f12.dsc = dsc;
				d->f12.page = page;
				synaptics_get_f12(dev);
				break;

			case 0x1a:
				d->f1a.dsc = dsc;
				d->f1a.page = page;
				break;

			case 0x34:
				d->f34.dsc = dsc;
				d->f34.page = page;
				break;

			case 0x51:
				d->f51.dsc = dsc;
				d->f51.page = page;
				break;

			case 0x54:
				d->f54.dsc = dsc;
				d->f54.page = page;
				break;

			default:
				break;
			}
		}
	}

	TOUCH_D(BASE_INFO,
		"common[%dP:0x%02x] finger_f12[%dP:0x%02x] flash[%dP:0x%02x] analog[%dP:0x%02x] lpwg[%dP:0x%02x]\n",
		d->f01.page, d->f01.dsc.fn_number,
		d->f12.page, d->f12.dsc.fn_number,
		d->f34.page, d->f34.dsc.fn_number,
		d->f54.page, d->f54.dsc.fn_number,
		d->f51.page, d->f51.dsc.fn_number);

	if (!(d->f01.dsc.fn_number &&
	      d->f12.dsc.fn_number &&
	      d->f34.dsc.fn_number &&
	      d->f54.dsc.fn_number &&
	      d->f51.dsc.fn_number))
		return -EINVAL;

	ret = synaptics_set_page(dev, 0);

	if (ret) {
		TOUCH_E("faied to set page %d (ret: %d)\n", 0, ret);
		return ret;
	}

	return 0;
}

static int synaptics_ic_info(struct device *dev)
{
	int ret;

	ret = synaptics_page_description(dev);
	return 0;
}

static int synaptics_sleep_control(struct device *dev, u8 mode)
{
/*
	struct synaptics_data *d = to_synaptics_data(dev);
	u8 val;
	int ret;

	ret = synaptics_read(dev, DEVICE_CONTROL_REG, &val, sizeof(val));
	if (ret < 0) {
		TOUCH_E("failed to read finger report enable - ret:%d\n", ret);
		return ret;
	}

	val &= 0xf8;

	if (mode)
		val |= 1;

	ret = synaptics_write(dev, DEVICE_CONTROL_REG, &val, sizeof(val));
	if (ret < 0) {
		TOUCH_E("failed to write finger report enable - ret:%d\n", ret);
		return ret;
	}

	TOUCH_I("%s - mode:%d\n", __func__, mode);
*/
	return 0;
}

static int synaptics_tci_report_enable(struct device *dev, bool enable)
{
	struct synaptics_data *d = to_synaptics_data(dev);

	u8 val[3];
	int ret;

	synaptics_irq_enable(dev, enable ? false : true);

	ret = synaptics_read(dev, FINGER_REPORT_REG, val, sizeof(val));
	if (ret < 0) {
		TOUCH_E("failed to read finger report enable - ret:%d\n", ret);
		return ret;
	}

	val[2] &= 0xfc;

	if (enable)
		val[2] |= 0x2;

	ret = synaptics_write(dev, FINGER_REPORT_REG, val, sizeof(val));
	if (ret < 0) {
		TOUCH_E("failed to write finger report enable - ret:%d\n", ret);
		return ret;
	}

	TOUCH_I("%s - enable:%d\n", __func__, enable);

	return 0;
}

static int synaptics_tci_knock(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	struct tci_info *info;
	u8 lpwg_data[6];
	int ret;

	TOUCH_TRACE();

	ret = synaptics_set_page(dev, LPWG_PAGE);

	if (ret < 0) {
		TOUCH_E("failed to set page to LPWG_PAGE\n");
		return ret;
	}

	ret = synaptics_read(dev, LPWG_TAPCOUNT_REG,
			      lpwg_data, sizeof(lpwg_data));

	TOUCH_I("0 : %d,%d,%d,%d,%d,%d\n",
		lpwg_data[0],
		lpwg_data[1],
		lpwg_data[2],
		lpwg_data[3],
		lpwg_data[4],
		lpwg_data[5]);

	info = &ts->tci.info[0];

	info->tap_count &= 0x7;
	info->intr_delay = 68;
	info->tap_distance = 10;

	lpwg_data[0] |= ((info->tap_count << 3) | 1);
	lpwg_data[1] = info->min_intertap;
	lpwg_data[2] = info->max_intertap;
	lpwg_data[3] = info->touch_slop;
	lpwg_data[4] = info->tap_distance;
	lpwg_data[5] = 0;

	ret = synaptics_write(dev, LPWG_TAPCOUNT_REG,
			      lpwg_data, sizeof(lpwg_data));

	ret = synaptics_read(dev, LPWG_TAPCOUNT_REG2,
			      &lpwg_data[0], sizeof(u8));

	TOUCH_I("1 : %d\n", lpwg_data[0]);

	lpwg_data[0] &= 0xfe;

	ret = synaptics_write(dev, LPWG_TAPCOUNT_REG2,
			      lpwg_data, sizeof(u8));
	if (ret < 0) {
		TOUCH_E("failed to write LPWG_TAPCOUNT_REG2\n");
		return ret;
	}

	ret = synaptics_set_page(dev, DEFAULT_PAGE);

	if (ret < 0) {
		TOUCH_E("failed to set page to DEFAULT_PAGE\n");
		return ret;
	}

	return ret;
}

static int synaptics_tci_password(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	struct tci_info *info;
	u8 lpwg_data[6];
	int ret;

	TOUCH_TRACE();

	ret = synaptics_set_page(dev, LPWG_PAGE);

	if (ret < 0) {
		TOUCH_E("failed to set page to LPWG_PAGE\n");
		return ret;
	}

	ret = synaptics_read(dev, LPWG_TAPCOUNT_REG,
			      lpwg_data, sizeof(lpwg_data));

	TOUCH_I("0: %d,%d,%d,%d,%d,%d\n",
		lpwg_data[0],
		lpwg_data[1],
		lpwg_data[2],
		lpwg_data[3],
		lpwg_data[4],
		lpwg_data[5]);

	info = &ts->tci.info[0];

	info->tap_count &= 0x7;
	info->intr_delay = ts->tci.double_tap_check ? 68 : 0;
	info->tap_distance = 7;

	lpwg_data[0] |= ((info->tap_count << 3) | 1);
	lpwg_data[1] = info->min_intertap;
	lpwg_data[2] = info->max_intertap;
	lpwg_data[3] = info->touch_slop;
	lpwg_data[4] = info->tap_distance;
	lpwg_data[5] = (info->intr_delay << 1 | 1);

	ret = synaptics_write(dev, LPWG_TAPCOUNT_REG,
			      lpwg_data, sizeof(lpwg_data));

	ret = synaptics_read(dev, LPWG_TAPCOUNT_REG2,
			      lpwg_data, sizeof(lpwg_data));

	TOUCH_I("1: %d,%d,%d,%d,%d,%d\n",
		lpwg_data[0],
		lpwg_data[1],
		lpwg_data[2],
		lpwg_data[3],
		lpwg_data[4],
		lpwg_data[5]);

	info = &ts->tci.info[1];
	info->tap_count &= 0x7;
	lpwg_data[0] |= ((info->tap_count << 3) | 1);
	lpwg_data[1] = info->min_intertap;
	lpwg_data[2] = info->max_intertap;
	lpwg_data[3] = info->touch_slop;
	lpwg_data[4] = info->tap_distance;
	lpwg_data[5] = (info->intr_delay << 1 | 1);

	ret = synaptics_write(dev, LPWG_TAPCOUNT_REG2,
			      lpwg_data, sizeof(lpwg_data));

	if (ret < 0) {
		TOUCH_E("failed to write LPWG_TAPCOUNT_REG2\n");
		return ret;
	}

	ret = synaptics_set_page(dev, DEFAULT_PAGE);

	if (ret < 0) {
		TOUCH_E("failed to set page to DEFAULT_PAGE\n");
		return ret;
	}

	return ret;
}

static int synaptics_tci_active_area(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	u8 buffer[8];

	buffer[0] = (ts->lpwg.area[0].x >> 0) & 0xff;
	buffer[1] = (ts->lpwg.area[0].x >> 8) & 0xff;
	buffer[2] = (ts->lpwg.area[0].x >> 0) & 0xff;
	buffer[3] = (ts->lpwg.area[0].x >> 8) & 0xff;
	buffer[4] = (ts->lpwg.area[1].x >> 0) & 0xff;
	buffer[5] = (ts->lpwg.area[1].x >> 8) & 0xff;
	buffer[6] = (ts->lpwg.area[1].x >> 0) & 0xff;
	buffer[7] = (ts->lpwg.area[1].x >> 8) & 0xff;

	synaptics_write(dev, d->f12_reg.ctrl[18], buffer, sizeof(buffer));

	return 0;
}

static int synaptics_lpwg_control(struct device *dev, int mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	TOUCH_I("synaptics_lpwg_control mode=%d\n", mode);

	switch (mode) {
	case LPWG_DOUBLE_TAP:
		ts->tci.mode = 0x01;
		synaptics_tci_knock(dev);
		synaptics_tci_report_enable(dev, true);
		break;

	case LPWG_PASSWORD:
		ts->tci.mode = 0x02;
		synaptics_tci_password(dev);
		synaptics_tci_report_enable(dev, true);
		break;

	default:
		ts->tci.mode = 0;
		ret = synaptics_tci_report_enable(dev, false);
		break;
	}

	return 0;
}

static int synaptics_lpwg_mode(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	if (atomic_read(&ts->state.fb) == FB_SUSPEND) {
		if (ts->lpwg.mode == LPWG_NONE) {
			/* deep sleep */
			TOUCH_I("%s(%d) - deep sleep\n",
				__func__, __LINE__);
			synaptics_sleep_control(dev, 1);
			synaptics_lpwg_control(dev, LPWG_NONE);
		} else if (ts->lpwg.screen) {
			TOUCH_I("%s(%d) - FB_SUSPEND & screen on -> skip\n",
				__func__, __LINE__);
			return 0;
		} else if (ts->lpwg.sensor == PROX_NEAR) {
			/* deep sleep */
			TOUCH_I("%s(%d) - deep sleep by prox\n",
				__func__, __LINE__);
			synaptics_sleep_control(dev, 1);
			synaptics_lpwg_control(dev, LPWG_NONE);
		} else if (ts->lpwg.qcover == HOLE_NEAR) {
			/* knock on */
			TOUCH_I("%s(%d) - knock on by hole\n",
				__func__, __LINE__);
			synaptics_sleep_control(dev, 1);
			synaptics_lpwg_control(dev, LPWG_DOUBLE_TAP);
		} else {
			/* knock on/code */
			TOUCH_I("%s(%d) - knock %d\n",
				__func__, __LINE__, ts->lpwg.mode);
			synaptics_sleep_control(dev, 1);
			synaptics_lpwg_control(dev, ts->lpwg.mode);
		}
		return 0;
	}

	/* resume */
	if (ts->lpwg.screen) {
		/* normal */
		TOUCH_I("%s(%d) - normal\n",
				__func__, __LINE__);
		synaptics_sleep_control(dev, 1);
		synaptics_lpwg_control(dev, LPWG_NONE);
	} else if (ts->lpwg.mode == LPWG_NONE) {
		/* normal */
		TOUCH_I("%s(%d) - normal on screen off\n",
				__func__, __LINE__);
		synaptics_sleep_control(dev, 1);
		synaptics_lpwg_control(dev, LPWG_NONE);
	} else if (ts->lpwg.sensor == PROX_NEAR) {
		/* wake up */
		TOUCH_I("%s(%d) - wake up on screen off and prox\n",
				__func__, __LINE__);
		TOUCH_I("%s - wake up is not ready\n", __func__);
		synaptics_sleep_control(dev, 0);
		synaptics_lpwg_control(dev, LPWG_NONE);
	} else {
		/* partial */
		TOUCH_I("%s(%d) - parial mode\n",
				__func__, __LINE__);
		TOUCH_I("%s - partial is not ready\n", __func__);
		synaptics_sleep_control(dev, 0);
		synaptics_lpwg_control(dev, LPWG_NONE);
	}

	return 0;
}

static void synaptic_init_tci_info(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	ts->tci.info[0].tap_count = 2;
	ts->tci.info[0].min_intertap = 6;
	ts->tci.info[0].max_intertap = 70;
	ts->tci.info[0].touch_slop = 100;
	ts->tci.info[0].tap_distance = 10;
	ts->tci.info[0].intr_delay = 0;

	ts->tci.info[1].tap_count = 2;
	ts->tci.info[1].min_intertap = 6;
	ts->tci.info[1].max_intertap = 70;
	ts->tci.info[1].touch_slop = 100;
	ts->tci.info[1].tap_distance = 255;
	ts->tci.info[1].intr_delay = 68;
}

static int synaptics_probe(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d;

	TOUCH_TRACE();

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);

	if (!d) {
		TOUCH_E("failed to allocate synaptics data\n");
		return -ENOMEM;
	}

	touch_set_device(ts, d);

	touch_gpio_init(ts->reset_pin, "touch_reset");
	touch_gpio_direction_output(ts->reset_pin, 0);

	touch_gpio_init(ts->int_pin, "touch_int");
	touch_gpio_direction_input(ts->int_pin);

	touch_power_init(dev);
	touch_bus_init(dev, 4096);

	synaptic_init_tci_info(dev);

	return 0;
}

static int synaptics_remove(struct device *dev)
{
	TOUCH_TRACE();

	return 0;
}

static int synaptics_suspend(struct device *dev)
{
	TOUCH_TRACE();
	synaptics_lpwg_mode(dev);

	return 0;
}

static int synaptics_resume(struct device *dev)
{
	TOUCH_TRACE();
	synaptics_lpwg_mode(dev);
	return 0;
}


static int synaptics_get_status(struct device *dev, u8 *device, u8 *interrupt)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	u8 dev_status;
	u8 irq_status;
	int ret;

	ret = synaptics_read(dev, DEVICE_STATUS_REG,
			&dev_status, sizeof(dev_status));

	if (ret < 0) {
		TOUCH_E("failed to read device status - ret:%d\n", ret);
		return ret;
	}

	ret = synaptics_read(dev, INTERRUPT_STATUS_REG,
			&irq_status, sizeof(irq_status));

	if (ret < 0) {
		TOUCH_E("failed to read interrupt status - ret:%d\n", ret);
		return ret;
	}

	TOUCH_I("status[device:%02x, interrupt:%02x]\n",
		dev_status, irq_status);

	if (device)
		*device = dev_status;

	if (interrupt)
		*interrupt = irq_status;

	return ret;
}

static int synaptics_irq_clear(struct device *dev)
{
	return synaptics_get_status(dev, NULL, NULL);
}

int synaptics_get_object_count(struct device *dev, u8 *object)
{
	struct synaptics_data *d = to_synaptics_data(dev);
	u8 object_to_read = d->num_of_fingers;
	u16 object_attention = 0;
	int ret;

	ret = synaptics_read(dev, d->f12_reg.data[15],
			&object_attention, sizeof(object_attention));

	if (ret < 0) {
		TOUCH_E("%s, %d : get object_attention data failed\n",
			__func__, __LINE__);
		return ret;
	}
	for (; object_to_read > 0 ;) {
		if (object_attention & (0x1 << (object_to_read - 1)))
			break;

		object_to_read--;
	}

	*object = object_to_read;
	return 0;
}

static int synaptics_irq_abs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	struct synaptics_object objects[MAX_NUM_OF_FINGERS];
	struct synaptics_object *obj;
	struct touch_data *tdata;
	u8 i;
	u8 finger_index;
	u8 object_to_read = 0;
	int ret;

	ts->new_mask = 0;
	ret = synaptics_get_object_count(dev, &object_to_read);

	if (ret < 0) {
		TOUCH_E("faied to read object count\n");
		return ret;
	}

	TOUCH_D(ABS, "object_to_read: %d\n", object_to_read);

	if (object_to_read > 0) {
		ret = synaptics_read(dev, FINGER_DATA_REG,
				     objects, sizeof(*obj) * object_to_read);
		if (ret < 0) {
			TOUCH_E("faied to read finger data\n");
			return ret;
		}

		finger_index = 0;

		for (i = 0; i < object_to_read; i++) {
			obj = objects + i;

			if (obj->type > F12_MAX_OBJECT)
				TOUCH_D(ABS, "id : %d, type : %d\n",
					i, obj->type);

			if (obj->type == F12_FINGER_STATUS) {
				ts->new_mask |= (1 << i);
				tdata = ts->tdata + i;

				tdata->id = i;
				tdata->type = obj->type;
				tdata->x = obj->x_lsb | obj->x_msb << 8;
				tdata->y = obj->y_lsb | obj->y_msb << 8;
				tdata->pressure = obj->z;

				if (obj->wx > obj->wy) {
					tdata->width_major = obj->wx;
					tdata->width_minor = obj->wy;
					tdata->orientation = 0;
				} else {
					tdata->width_major = obj->wy;
					tdata->width_minor = obj->wx;
					tdata->orientation = 1;
				}

				finger_index++;

				TOUCH_D(ABS,
					"tdata [id:%d t:%d x:%d y:%d z:%d-%d,%d,%d]\n",
					tdata->id,
					tdata->type,
					tdata->x,
					tdata->y,
					tdata->pressure,
					tdata->width_major,
					tdata->width_minor,
					tdata->orientation);
			}
		}

		ts->tcount = finger_index;
	}

	ts->intr_status |= TOUCH_IRQ_FINGER;

	return 0;
}

static int synaptics_tci_getdata(struct device *dev, int count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	u32 buffer[12];
	int i = 0;
	int ret;

	ret = synaptics_read(dev, LPWG_DATA_REG,
					 buffer, sizeof(u32) * count);

	if (ret < 0)
		return ret;

	for (i = 0; i < count; i++) {
		ts->lpwg.code[i].x = buffer[0] & 0xffff;
		ts->lpwg.code[i].y = (buffer[0] >> 16) & 0xffff;

		TOUCH_I("LPWG data %d, %d\n",
			ts->lpwg.code[i].x, ts->lpwg.code[i].y);
	}

	ts->lpwg.code[i].x = -1;
	ts->lpwg.code[i].y = -1;

	return 0;
}

static int synaptics_irq_lpwg(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct synaptics_data *d = to_synaptics_data(dev);
	u8 status;
	int ret;

	ret = synaptics_set_page(dev, LPWG_PAGE);

	if (ret < 0)
		return ret;

	ret = synaptics_read(dev, LPWG_STATUS_REG, &status, 1);
	if (ret < 0)
		return ret;

	if (status & LPWG_STATUS_DOUBLETAP) {
		synaptics_tci_getdata(dev, 2);
		ts->intr_status |= TOUCH_IRQ_KNOCK;
	} else if (status & LPWG_STATUS_PASSWORD) {
		synaptics_tci_getdata(dev, ts->lpwg.code_num);
		ts->intr_status |= TOUCH_IRQ_PASSWD;
	} else {
		/* Overtab */
		ts->lpwg.code_num = 1;
		synaptics_tci_getdata(dev, ts->lpwg.code_num);
		ts->intr_status |= TOUCH_IRQ_PASSWD;
	}

	return synaptics_set_page(dev, DEFAULT_PAGE);
}

static int synaptics_irq_handler(struct device *dev)
{
	u8 dev_status;
	u8 irq_status;
	int ret;

	TOUCH_TRACE();

	ret = synaptics_get_status(dev, &dev_status, &irq_status);

	if (irq_status & INTERRUPT_MASK_ABS0)
		ret = synaptics_irq_abs(dev);

	if (irq_status & INTERRUPT_MASK_LPWG)
		ret = synaptics_irq_lpwg(dev);

	return ret;
}

static int synaptics_f51_remap(struct device *dev)
{
	return 0;
}

static int synaptics_init(struct device *dev)
{
	TOUCH_TRACE();

	synaptics_ic_info(dev);

	synaptics_f51_remap(dev);

	synaptics_irq_enable(dev, true);
	synaptics_irq_clear(dev);

	return 0;
}

static int synaptics_power(struct device *dev, int ctrl)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();

	switch (ctrl) {
	case POWER_OFF:
		TOUCH_I("%s, off\n", __func__);
		touch_gpio_direction_output(ts->reset_pin, 0);
		touch_power_vio(dev, 0);
		touch_power_vdd(dev, 0);
		break;

	case POWER_ON:
		TOUCH_I("%s, on\n", __func__);
		touch_power_vdd(dev, 1);
		touch_power_vio(dev, 1);
		touch_gpio_direction_output(ts->reset_pin, 1);
		break;


	case POWER_SLEEP:
		TOUCH_I("%s, sleep\n", __func__);
		break;

	case POWER_WAKE:
		TOUCH_I("%s, wake\n", __func__);
		break;
	}

	return 0;
}


static int synaptics_lpwg(struct device *dev, u32 code, void *param)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int *value = (int *)param;

	TOUCH_TRACE();

	switch (code) {
	case LPWG_ACTIVE_AREA:
		ts->lpwg.area[0].x = value[0];
		ts->lpwg.area[0].y = value[2];
		ts->lpwg.area[1].x = value[1];
		ts->lpwg.area[1].y = value[3];
		TOUCH_I("LPWG AREA (%d,%d)-(%d,%d)\n",
			ts->lpwg.area[0].x, ts->lpwg.area[0].y,
			ts->lpwg.area[1].x, ts->lpwg.area[1].y);
		synaptics_tci_active_area(dev);
		break;

	case LPWG_UPDATE_ALL:
		ts->lpwg.mode = value[0];
		ts->lpwg.screen = value[1];
		ts->lpwg.sensor = value[2];
		ts->lpwg.qcover = value[3];
		TOUCH_I(
			"LPWG_UPDATE_ALL: mode[%d], screen[%s], sensor[%s], qcover[%s]\n",
			ts->lpwg.mode,
			ts->lpwg.screen ? "ON" : "OFF",
			ts->lpwg.sensor ? "FAR" : "NEAR",
			ts->lpwg.qcover ? "CLOSE" : "OPEN");
		synaptics_lpwg_mode(dev);
		break;
	}

	return 0;
}

static int synaptics_notify(struct device *dev, ulong event, void *data)
{
	TOUCH_TRACE();

	return 0;
}

static int synaptics_register_sysfs(struct device *dev)
{
	TOUCH_TRACE();

	return 0;
}

static int synaptics_set(struct device *dev, long cmd, long data)
{
	TOUCH_TRACE();

	return 0;
}

static int synaptics_get(struct device *dev, long cmd, long *data)
{
	TOUCH_TRACE();

	return 0;
}

static struct touch_driver touch_driver = {
	.probe = synaptics_probe,
	.remove = synaptics_remove,
	.suspend = synaptics_suspend,
	.resume = synaptics_resume,
	.init = synaptics_init,
	.irq_handler = synaptics_irq_handler,
	.power = synaptics_power,
	.lpwg = synaptics_lpwg,
	.notify = synaptics_notify,
	.register_sysfs = synaptics_register_sysfs,
	.set = synaptics_set,
	.get = synaptics_get,
};


#define MATCH_NAME			"lge,synaptics"

static struct of_device_id touch_match_ids[] = {
	{ .compatible = MATCH_NAME, },
};

static struct touch_hwif hwif = {
	.bus_type = HWIF_I2C,
	.name = LGE_TOUCH_NAME,
	.owner = THIS_MODULE,
	.of_match_table = touch_match_ids,
};

static int __init touch_device_init(void)
{
	TOUCH_TRACE();
	return touch_bus_device_init(&hwif, &touch_driver);
}

static void __exit touch_device_exit(void)
{
	TOUCH_TRACE();
	touch_bus_device_exit(&hwif);
}

module_init(touch_device_init);
module_exit(touch_device_exit);

MODULE_AUTHOR("hoyeon.jang@lge.com");
MODULE_DESCRIPTION("LGE touch driver v3");
MODULE_LICENSE("GPL");
