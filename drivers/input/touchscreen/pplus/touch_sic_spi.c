/* touch_sic_spi.c
 *
 * Copyright (C) 2015 LGE.
 *
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

#include <linux/err.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/regulator/machine.h>
#include <linux/async.h>
#include <linux/atomic.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>
#include <linux/syscalls.h>
#include <linux/gpio.h>
#include <linux/file.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/interrupt.h>
#include "lge_touch_core_spi.h"
#include "touch_sic_spi.h"
#include "touch_sic_watch_font.h"
#include "touch_sic_bl_code.h"
#include <soc/qcom/lge/board_lge.h>

#if USE_ABT_MONITOR_APP
#include "touch_sic_abt_spi.h"
u16 frame_num = 0;
#endif
static char power_state;
static int sensor_state;	/* near 0, far 1 */
static int doze_state;

#define USE_I2C_BURST	1

/* LPWG Control Value */
#define DRIVING_STOP			5
#define DOZE1_MODE_SET			1
#define DOZE2_MODE_SET			2
#define TCI_ENABLE_CTRL			3
#define TAP_COUNT_CTRL			4
#define MIN_INTERTAP_CTRL		5
#define MAX_INTERTAP_CTRL		6
#define TOUCH_SLOP_CTRL			7
#define TAP_DISTANCE_CTRL		8
#define INTERRUPT_DELAY_CTRL		9
#define ACTIVE_AREA_CTRL		10
#define PARTIAL_LPWG_ON			11
#define LOW_POWER_CTRL			12
#define ACTIVE_AREA_RESET_CTRL		13

#define SWIPE_ENABLE_CTRL		30
#define SWIPE_DISABLE_CTRL		31
#define SWIPE_DIST_THR_CTRL		32
#define SWIPE_RATIO_THR_CTRL		33
#define SWIPE_RATIO_DIST_MIN_CTRL	34
#define SWIPE_RATIO_PERIOD_CTRL		35
#define SWIPE_TIME_MIN_CTRL		36
#define SWIPE_TIME_MAX_CTRL		37
#define SWIPE_AREA_CTRL			38

#define FAIL_REASON_CTRL		100
#define FAIL_REASON_CTRL2		101
#define OVERTAP_CTRL			102


static const char const *sic_tci_fail_reason_str[] = {
	"NONE",
	"DISTANCE_INTER_TAP",
	"DISTANCE_TOUCHSLOP",
	"TIMEOUT_INTER_TAP_LONG",
	"MULTI_FINGER",
	"DELAY_TIME",/* It means Over Tap */
	"TIMEOUT_INTER_TAP_SHORT",
	"PALM_STATE",
	"TAP_TIMEOVER",
	"DEBUG9",
	"DEBUG10"
};
static const char const *sic_swipe_fail_reason_str[] = {
	"ERROR",
	"1FINGER_FAST_RELEASE",
	"MULTI_FINGER",
	"FAST_SWIPE",
	"SLOW_SWIPE",
	"OUT_OF_AREA",
	"RATIO_FAIL",
};

static int sic_mfts_mode;
/*   0 : normal mode
	1 : mfts_folder
	2 : mfts_flat
	3 : mfts_curved   */
static char line[BUFFER_SIZE];
static char Write_Buffer[BUFFER_SIZE];
static u16 sicImage[COL_SIZE][ROW_SIZE];
unsigned int detect_flash_ic(struct sic_ts_data *ts);

int sic_spi_read(struct spi_device *spi, u16 addr, u8 *data, u16 size)
{
	struct spi_transfer t = {0};
	struct spi_message m;
	int ret = 0;
	struct sic_ts_data *ts = (struct sic_ts_data *)get_touch_handle(spi);

	mutex_lock(&ts->spi_lock);

	spi_message_init(&m);

	/* set header */
	ts->input[0] = 0;	/* read command */

	if(size > 4)
		ts->input[1] = 0x03; /* burst read command */
	else
		ts->input[1] = 0x01; /* single word read command */

	ts->input[2] = (addr & 0xFF00) >> 8; /* address hi */
	ts->input[3] = (addr & 0x00FF) >> 0; /* address lo */
	ts->input[4] = 0;	/* read dummy */
	ts->input[5] = 0;	/* read dummy */

	/* read data */
	t.tx_buf = ts->input;
	t.rx_buf = ts->output;
	t.len = SPI_READ_HEADER_SIZE + size;
	t.cs_change = 0;
	t.bits_per_word = 8;
	spi_message_add_tail(&t, &m);
	ret = spi_sync(spi, &m);

	if (ret) {
		TOUCH_E("spi_sync read data error:ret=[%d]\n", ret);
		mutex_unlock(&ts->spi_lock);
		return -EIO;
	}

	memcpy(data, ts->output + SPI_READ_HEADER_SIZE, size);
	mutex_unlock(&ts->spi_lock);

	return ret;

}

int sic_spi_write(struct spi_device *spi, u16 addr, u8 *data, u16 size)
{
	struct spi_transfer t = {0};
	struct spi_message m;
	int ret = 0;
	struct sic_ts_data *ts = (struct sic_ts_data *)get_touch_handle(spi);

	mutex_lock(&ts->spi_lock);

	spi_message_init(&m);

	/* set header */
	ts->input[0] = 0; /* write commad */

	if(size == 1)
		ts->input[1] = 0x02; /* write byte command */
	else
		ts->input[1] = 0x00; /* write command */

	ts->input[2] = (addr & 0xFF00) >> 8; /* address hi */
	ts->input[3] = (addr & 0x00FF) >> 0; /* address lo */

	memcpy(&ts->input[4], data, size);

	/* write data */
	t.tx_buf = ts->input;
	t.rx_buf = NULL;
	t.len = SPI_WRITE_HEADER_SIZE + size;
	t.cs_change = 0;
	t.bits_per_word = 8;
	spi_message_add_tail(&t, &m);
	ret = spi_sync(spi, &m);

	if (ret) {
		TOUCH_E("spi_sync write data error:ret=[%d]\n", ret);
		mutex_unlock(&ts->spi_lock);
		return -EIO;
	}
	mutex_unlock(&ts->spi_lock);

	return ret;

}

int sic_spi_font_write(struct spi_device *spi, u8 *font_sel, u8 *data, u16 size)
{
	int ret = 0;
	u8 select[12] = {0};
	u8 *offset = NULL;
	struct spi_transfer t[2];
	struct spi_message m;
	struct sic_ts_data *ts = (struct sic_ts_data *)get_touch_handle(spi);

	mutex_lock(&ts->spi_lock);

	spi_message_init(&m);
	memset(t, 0, sizeof(t));

	offset = ts->input;

	/* set header */
	select[0] = 0;
	select[1] = 0;
	select[2] = (EXT_WATCH_FONT_SEL & 0xFF00) >> 8;
	select[3] = (EXT_WATCH_FONT_SEL & 0x00FF) >> 0;
	memcpy(&select[4], font_sel, sizeof(u32));

	offset[0] = 0;
	offset[1] = 0;
	offset[2] = (EXT_WATCH_FONT_OFFSET_ADDR & 0xFF00) >> 8;
	offset[3] = (EXT_WATCH_FONT_OFFSET_ADDR & 0x00FF) >> 0;
	memcpy(&offset[4], data, size);

	/* write data */
	t[0].rx_buf = NULL;
	t[0].tx_buf = select;
	t[0].cs_change = 1;
	t[0].bits_per_word = 8;
	t[0].len = SPI_WRITE_HEADER_SIZE + sizeof(u32);
	spi_message_add_tail(&t[0], &m);

	t[1].rx_buf = NULL;
	t[1].tx_buf = offset;
	t[1].cs_change = 1;
	t[1].bits_per_word = 8;
	t[1].len = SPI_WRITE_HEADER_SIZE + size;
	spi_message_add_tail(&t[1], &m);

	ret = spi_sync(spi, &m);
	if (ret) {
		TOUCH_E("spi_sync write data error:ret=[%d]\n", ret);
		mutex_unlock(&ts->spi_lock);
		return -EIO;
	}
	mutex_unlock(&ts->spi_lock);

	return ret;
}

int sic_spi_font_read(struct spi_device *spi, u8 *font_sel, u8 *data, u16 size)
{
	int ret;
	u8 select[12] = {0};
	u8 *offset = NULL;
	struct spi_transfer t[2];
	struct spi_message m;
	struct sic_ts_data *ts = (struct sic_ts_data *)get_touch_handle(spi);

	mutex_lock(&ts->spi_lock);

	spi_message_init(&m);
	memset(t, 0, sizeof(t));

	offset = ts->input;

	/* set header */
	select[0] = 0;
	select[1] = 0;
	select[2] = (EXT_WATCH_FONT_SEL & 0xFF00) >> 8;
	select[3] = (EXT_WATCH_FONT_SEL & 0x00FF) >> 0;
	memcpy(&select[4], font_sel, sizeof(u32));

	memset(offset, 0, SPI_READ_HEADER_SIZE);
	offset[1] = 0x03;
	offset[2] = (EXT_WATCH_FONT_OFFSET_ADDR & 0xFF00) >> 8;
	offset[3] = (EXT_WATCH_FONT_OFFSET_ADDR & 0x00FF) >> 0;

	/* write data */
	t[0].rx_buf = NULL;
	t[0].tx_buf = select;
	t[0].cs_change = 1;
	t[0].bits_per_word = 8;
	t[0].len = SPI_WRITE_HEADER_SIZE + sizeof(u32);
	spi_message_add_tail(&t[0], &m);

	t[1].rx_buf = ts->output;
	t[1].tx_buf = offset;
	t[1].cs_change = 1;
	t[1].bits_per_word = 8;
	t[1].len = SPI_READ_HEADER_SIZE + size;
	spi_message_add_tail(&t[1], &m);

	ret = spi_sync(spi, &m);
	if (ret) {
		TOUCH_E("spi_sync write data error:ret=[%d]\n", ret);
		mutex_unlock(&ts->spi_lock);
		return -EIO;
	}

	memcpy(data, ts->output + SPI_READ_HEADER_SIZE, size);
	mutex_unlock(&ts->spi_lock);

	return ret;
}

static void set_fail_reason(struct sic_ts_data *ts, int type)
{
	u32 wdata = (u32) type;
	wdata |=  ts->fail_reason[0] == 1 ? 0x01 << 2 : 0x01 << 3;
	TOUCH_I("TCI%d-type:%d\n", type + 1, wdata);
	DO_SAFE(sic_spi_write(ts->spi_device, CMD_ABT_LPWG_TCI_FAIL_DEBUG_WRITE,
		(u8 *)&wdata, sizeof(u32)), error);

	wdata = ts->fail_type;
	TOUCH_I("TCI%d-dbg:%d\n", type + 1, wdata);
	DO_SAFE(sic_spi_write(ts->spi_device, CMD_ABT_LPWG_TCI_FAIL_BIT_WRITE,
		(u8 *)&wdata, sizeof(u32)), error);

	return;
error:
	TOUCH_E("set_fail_reason FAIL!!!\n");
}

/**
 * Knock on
 *
 * Type		Value
 *
 * 1		WakeUp_gesture_only=1 / Normal=0
 * 2		TCI enable=1 / disable=0
 * 3		Tap Count
 * 4		Min InterTap
 * 5		Max InterTap
 * 6		Touch Slop
 * 7		Tap Distance
 * 8		Interrupt Delay
 */
static int tci_control(struct sic_ts_data *ts, int type)
{
	struct tci_ctrl_info *tci1 = &ts->tci_ctrl.tci1;
	struct tci_ctrl_info *tci2 = &ts->tci_ctrl.tci2;

	u32 wdata = 0;

	switch (type) {
	case FAIL_REASON_CTRL:
		set_fail_reason(ts, 0);	/* TCI-1 */
		if (ts->lpwg_ctrl.password_enable)
			set_fail_reason(ts, 1);	/* TCI-2 */

		break;
	case FAIL_REASON_CTRL2:
		wdata = ts->fail_reason[1];
		DO_SAFE(sic_spi_write(ts->spi_device, CMD_ABT_LPWG_SWIPE_FAIL_DEBUG_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		break;
	case OVERTAP_CTRL:
		/* OverTap - Always Report for DELAY_TIME */
		wdata = 0x01; /* TCI - 2 requires */
		wdata |= 0x01 << 3;
		TOUCH_I("TCI2-type:%d\n", wdata);
		DO_SAFE(sic_spi_write(ts->spi_device,
		CMD_ABT_LPWG_TCI_FAIL_DEBUG_WRITE,
		(u8 *)&wdata, sizeof(u32)), error);

		wdata = ts->fail_overtap;
		TOUCH_I("TCI2-dbg:%d\n", wdata);
		DO_SAFE(sic_spi_write(ts->spi_device,
		CMD_ABT_LPWG_TCI_FAIL_BIT_WRITE,
		(u8 *)&wdata, sizeof(u32)), error);
		break;
	case ACTIVE_AREA_CTRL:
		wdata = ts->pdata->role->quickcover_filter->X1;
		DO_SAFE(sic_spi_write(ts->spi_device, CMD_ABT_ACT_AREA_X1_CTRL_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		wdata = ts->pdata->role->quickcover_filter->X2;
		DO_SAFE(sic_spi_write(ts->spi_device, CMD_ABT_ACT_AREA_X2_CTRL_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		wdata = ts->pdata->role->quickcover_filter->Y1;
		DO_SAFE(sic_spi_write(ts->spi_device, CMD_ABT_ACT_AREA_Y1_CTRL_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		wdata = ts->pdata->role->quickcover_filter->Y2;
		DO_SAFE(sic_spi_write(ts->spi_device, CMD_ABT_ACT_AREA_Y2_CTRL_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		break;
	case ACTIVE_AREA_RESET_CTRL:
		wdata = 0x410041;	/* 65 */
		DO_SAFE(sic_spi_write(ts->spi_device, CMD_ABT_ACT_AREA_X1_CTRL_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		wdata = 0x55e055e;	/* 1374 */
		DO_SAFE(sic_spi_write(ts->spi_device, CMD_ABT_ACT_AREA_X2_CTRL_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		wdata = 0x410041;	/* 65 */
		DO_SAFE(sic_spi_write(ts->spi_device, CMD_ABT_ACT_AREA_Y1_CTRL_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		wdata = 0x9be09be;	/* 2494 */
		DO_SAFE(sic_spi_write(ts->spi_device, CMD_ABT_ACT_AREA_Y2_CTRL_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		break;
#if 0
	case PARTIAL_LPWG_ON:
		wdata = 0x06;
		DO_SAFE(sic_spi_write(ts->spi_device, tc_device_ctl,
					(u8 *)&wdata, sizeof(u32)), error);
		doze_state = DOZE1_PARTIAL_STATUS;
		TOUCH_I("PARTIAL_LPWG_ON Set\n");
		break;
	case LOW_POWER_CTRL:
		wdata = 0x40;
		DO_SAFE(sic_spi_write(ts->spi_device, tc_device_ctl,
					(u8 *)&wdata, sizeof(u32)), error);
		doze_state = LOW_POWER_STATUS;
		TOUCH_I("LOW_POWER Set\n");
		break;
	case DOZE1_MODE_SET:
		/* doze1 = 0x2, doze2 = 0x4 */
		wdata = 0x02;
		DO_SAFE(sic_spi_write(ts->spi_device, tc_device_ctl,
					(u8 *)&wdata, sizeof(u32)), error);
		doze_state = DOZE1_STATUS;
		TOUCH_I("Doze1 Mode Set\n");
		break;
	case DOZE2_MODE_SET:
		/* doze1 = 0x2, doze2 = 0x4 */
		wdata = ts->debug_mode ? 0x04 : 0x44;
		DO_SAFE(sic_spi_write(ts->spi_device, tc_device_ctl,
					(u8 *)&wdata, sizeof(u32)), error);
		doze_state = ts->debug_mode ? DOZE2_DEBUG_STATUS : DOZE2_STATUS;
		TOUCH_I("Doze2 Mode Set\n");
		break;
#endif

	/* TCI-1 (Knock on) setting */
	case TCI_ENABLE_CTRL:
		wdata = ts->tci_ctrl.tci_mode;
		DO_SAFE(sic_spi_write(ts->spi_device, CMD_ABT_TCI_ENABLE_CTRL_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		break;
	case TAP_COUNT_CTRL:
		wdata = (tci1->tap_count) | (tci2->tap_count << 16);
		DO_SAFE(sic_spi_write(ts->spi_device, CMD_ABT_TAP_COUNT_CTRL_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		break;
	case MIN_INTERTAP_CTRL:
		wdata = (tci1->min_intertap) | (tci2->min_intertap << 16);
		DO_SAFE(sic_spi_write(ts->spi_device, CMD_ABT_MIN_INTERTAP_CTRL_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		break;
	case MAX_INTERTAP_CTRL:
		wdata = (tci1->max_intertap) | (tci2->max_intertap << 16);
		DO_SAFE(sic_spi_write(ts->spi_device, CMD_ABT_MAX_INTERTAP_CTRL_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		break;
	case TOUCH_SLOP_CTRL:
		wdata = (tci1->touch_slop) | (tci2->touch_slop << 16);
		DO_SAFE(sic_spi_write(ts->spi_device, CMD_ABT_TOUCH_SLOP_CTRL_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		break;
	case TAP_DISTANCE_CTRL:
		wdata = (tci1->tap_distance) | (tci2->tap_distance << 16);
		DO_SAFE(sic_spi_write(ts->spi_device, CMD_ABT_TAP_DISTANCE_CTRL_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		break;
	case INTERRUPT_DELAY_CTRL:
		wdata = (tci1->intr_delay) | (tci2->intr_delay << 16);
		DO_SAFE(sic_spi_write(ts->spi_device, CMD_ABT_INT_DELAY_CTRL_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		break;
	default:
		break;
	}
	return 0;
error:
	TOUCH_E("tci_control FAIL!!!\n");
	return -ERROR;
}

static int swipe_control(struct sic_ts_data *ts, int type)
{
	u32 wdata = 0;
	struct swipe_data *swp = &ts->swipe;
	struct swipe_ctrl_info *down = &swp->down;
	struct swipe_ctrl_info *up = &swp->up;

	switch (type) {
	case SWIPE_ENABLE_CTRL:
		wdata = swp->swipe_mode;
		DO_SAFE(sic_spi_write(ts->spi_device,
					CMD_ABT_LPWG_SWIPE_ON_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		break;
	case SWIPE_DISABLE_CTRL:
		wdata = 0;
		DO_SAFE(sic_spi_write(ts->spi_device,
					CMD_ABT_LPWG_SWIPE_ON_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		break;
	case SWIPE_DIST_THR_CTRL:
		wdata = (down->min_distance) | (up->min_distance << 16);
		DO_SAFE(sic_spi_write(ts->spi_device,
					CMD_ABT_LPWG_SWIPE_DIST_THRESHOLD_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		break;
	case SWIPE_RATIO_THR_CTRL:
		wdata = (down->ratio_thres) | (up->ratio_thres << 16);
		DO_SAFE(sic_spi_write(ts->spi_device,
					CMD_ABT_LPWG_SWIPE_RATIO_THRESHOLD_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		break;
	case SWIPE_RATIO_DIST_MIN_CTRL:
		wdata = (down->ratio_chk_min_distance) |
					(up->ratio_chk_min_distance << 16);
		DO_SAFE(sic_spi_write(ts->spi_device,
					CMD_ABT_LPWG_SWIPE_RATIO_CHECK_DIST_MIN_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		break;
	case SWIPE_RATIO_PERIOD_CTRL:
		wdata = (down->ratio_chk_period) | (up->ratio_chk_period << 16);
		DO_SAFE(sic_spi_write(ts->spi_device,
					CMD_ABT_LPWG_SWIPE_RATIO_CHECK_PERIOD_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		break;
	case SWIPE_TIME_MIN_CTRL:
		wdata = (down->min_time_thres) | (up->min_time_thres << 16);
		DO_SAFE(sic_spi_write(ts->spi_device,
					CMD_ABT_LPWG_SWIPE_TIME_MIN_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		break;
	case SWIPE_TIME_MAX_CTRL:
		wdata = (down->max_time_thres) | (up->max_time_thres << 16);
		DO_SAFE(sic_spi_write(ts->spi_device,
					CMD_ABT_LPWG_SWIPE_TIME_MAX_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		break;
	case SWIPE_AREA_CTRL:
		wdata = (down->active_area_x0) | (up->active_area_x0 << 16);
		DO_SAFE(sic_spi_write(ts->spi_device,
					CMD_ABT_LPWG_SWIPE_ACT_AREA_X1_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		wdata = (down->active_area_y0) | (up->active_area_y0 << 16);
		DO_SAFE(sic_spi_write(ts->spi_device,
					CMD_ABT_LPWG_SWIPE_ACT_AREA_Y1_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		wdata = (down->active_area_x1) | (up->active_area_x1 << 16);
		DO_SAFE(sic_spi_write(ts->spi_device,
					CMD_ABT_LPWG_SWIPE_ACT_AREA_X2_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		wdata = (down->active_area_y1) | (up->active_area_y1 << 16);
		DO_SAFE(sic_spi_write(ts->spi_device,
					CMD_ABT_LPWG_SWIPE_ACT_AREA_Y2_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		break;
	default:
		break;
	}
	return 0;
error:
	TOUCH_E("swipe_control FAIL!!!\n");
	return -ERROR;
}


static int swipe_enable(struct sic_ts_data *ts)
{
	if (ts->fail_reason[1])
		DO_SAFE(tci_control(ts, FAIL_REASON_CTRL2), error);
	if (ts->swipe.swipe_mode != 0) {
		DO_SAFE(swipe_control(ts, SWIPE_ENABLE_CTRL), error);
		DO_SAFE(swipe_control(ts, SWIPE_DIST_THR_CTRL), error);
		DO_SAFE(swipe_control(ts, SWIPE_RATIO_THR_CTRL), error);
		DO_SAFE(swipe_control(ts, SWIPE_RATIO_DIST_MIN_CTRL), error);
		DO_SAFE(swipe_control(ts, SWIPE_RATIO_PERIOD_CTRL), error);
		DO_SAFE(swipe_control(ts, SWIPE_TIME_MIN_CTRL), error);
		DO_SAFE(swipe_control(ts, SWIPE_TIME_MAX_CTRL), error);
		DO_SAFE(swipe_control(ts, SWIPE_AREA_CTRL), error);
	} else {
		TOUCH_I("Swipe_Mode is not set\n");
	}

	TOUCH_I("%s\n", __func__);
	return 0;
error:
	TOUCH_E("swipe_enable failed\n");
	return -EPERM;
}

static int swipe_disable(struct sic_ts_data *ts)
{
	DO_SAFE(swipe_control(ts, SWIPE_DISABLE_CTRL), error);

	TOUCH_I("%s\n", __func__);
	return 0;
error:
	TOUCH_E("swipe_disable failed\n");
	return -EPERM;
}

static int get_tci_data(struct sic_ts_data *ts, int count)
{
	struct spi_device *spi = ts->spi_device;
	u8 i = 0;
	u32 rdata[MAX_POINT_SIZE_FOR_LPWG];

	ts->pw_data.data_num = count;

	if (!count)
		return 0;

	DO_SAFE(sic_spi_read(spi, report_base+1,
		(u8 *)&rdata[0], count*4), error);

	for (i = 0; i < count; i++) {
		ts->pw_data.data[i].x = rdata[i] & 0xffff;
		ts->pw_data.data[i].y = rdata[i]  >> 16;

		if (ts->pdata->role->use_security_mode) {
			if (ts->lpwg_ctrl.password_enable) {
				TOUCH_I("LPWG data xxxx, xxxx\n");
			} else {
				TOUCH_I("LPWG data %d, %d\n",
						ts->pw_data.data[i].x,
						ts->pw_data.data[i].y);
			}
		} else {
			TOUCH_I("LPWG data %d, %d\n",
					ts->pw_data.data[i].x,
					ts->pw_data.data[i].y);
		}
	}

	return 0;
error:
	return -ERROR;
}

static int set_tc_driving(struct spi_device *spi, int value)
{
	u32 driving_ctl = 0;
	u8 skip_driving = 0;
	u32 rdata = 0;
	u32 retry = 0;
	u32 disp_status;

	struct sic_ts_data *ts = (struct sic_ts_data *)get_touch_handle(spi);

	switch (value) {
	case LCD_MODE_U0 :
		driving_ctl = 0x1;
		disp_status = 0x00;
		doze_state = DOZE2_STATUS;
		TOUCH_I("%s: LCD_MODE U0!\n", __func__);
		break;
	case LCD_MODE_U1 :
		if (ts->pdata->mode.prev_mode == LCD_MODE_U3)
			skip_driving = 1;
		driving_ctl = 0x81;
		disp_status = 0x20;
		doze_state = DOZE1_STATUS;
		TOUCH_I("%s: LCD_MODE U1!\n", __func__);
		break;
	case LCD_MODE_U2 :
		driving_ctl = 0x101;
		disp_status = 0x40;
		doze_state = DOZE2_STATUS;
		swipe_enable(ts);
		TOUCH_I("%s: LCD_MODE U2!\n", __func__);
		break;
	case LCD_MODE_U3 :
		if (ts->pdata->mode.prev_mode == LCD_MODE_U1)
			skip_driving = 1;
		driving_ctl = 0x181;
		disp_status = 0x60;
		doze_state = DOZE1_STATUS;
		TOUCH_I("%s: LCD_MODE U3!\n", __func__);
		break;
	case LCD_MODE_U3_PARTIAL:
		driving_ctl = 0x381;
		disp_status = 0x60;
		doze_state = DOZE1_STATUS;
		TOUCH_I("%s: LCD_MODE U3 PARTIAL!\n", __func__);
		break;
	case DRIVING_STOP :
		driving_ctl = 0x02;
		doze_state = LOW_POWER_STATUS;
		break;
	}

	if (!skip_driving) {
		msleep(25);
		do {
			retry++;
			msleep(5);
			TOUCH_I("%s, delay until touch ic ready for mode change. retry cnt = %d",
				__func__, retry);
			DO_SAFE(sic_spi_read(spi, SYS_DISPMODE_ST, (u8 *)&rdata, sizeof(u32)),
				error);
			TOUCH_I("%s, rdata = %d, disp_status = %d\n",__func__, rdata, disp_status);
		} while ((rdata & disp_status) != disp_status
			&& retry < 5
			&& value != DRIVING_STOP);

		DO_SAFE(sic_spi_write(spi, tc_driving_ctl,
			(u8 *)&driving_ctl, sizeof(u32)), error);
	}

	TOUCH_I("TC_Driving %s\n",
		(doze_state == DOZE1_STATUS) ? "DOZE1_MODE_SET" :
		(doze_state == LOW_POWER_STATUS) ? "DRIVING_STOP" : "DOZE2_MODE_SET");

	return 0;
error:
	TOUCH_E("[%s] spi fail\n", __func__);
	return -ERROR;
}

static int spi_clock_ctrl(struct spi_device *spi, int mode) {
	struct spi_transfer t = {0};
	struct spi_message m;
	int ret = 0;
	u8 input[2] = {0, };

	spi_message_init(&m);

	/* set header */
	input[0] = 0x0;

	if(mode == POWER_ON)
		input[1] = 0x98; /* Oscillator power on */
	else
		input[1] = 0x90; /* Oscillator power off */

	/* write data */
	t.tx_buf = input;
	t.rx_buf = NULL;
	t.len = 2;
	t.cs_change = 0;
	t.bits_per_word = 8;
	spi_message_add_tail(&t, &m);
	ret = spi_sync(spi, &m);

	if (ret) {
		TOUCH_E("spi_sync write data error:ret=[%d]\n", ret);
		return -EIO;
	}

	return ret;

}

static int spi_osc_power_ctrl(struct spi_device *spi, int mode) {
	struct spi_transfer t = {0};
	struct spi_message m;
	int ret = 0;
	u8 input[2] = {0, };

	spi_message_init(&m);

	/* set header */
	input[0] = 0x0;

	if(mode == POWER_ON)
		input[1] = 0x88; /* Oscillator power on */
	else
		input[1] = 0x80; /* Oscillator power off */

	/* write data */
	t.tx_buf = input;
	t.rx_buf = NULL;
	t.len = 2;
	t.cs_change = 0;
	t.bits_per_word = 8;
	spi_message_add_tail(&t, &m);
	ret = spi_sync(spi, &m);

	if (ret) {
		TOUCH_E("spi_sync write data error:ret=[%d]\n", ret);
		return -EIO;
	}

	return ret;
}

void tci_deep_sleep(struct spi_device *spi, int value)
{
	if (value == 1) {
		set_tc_driving(spi, DRIVING_STOP);
		spi_clock_ctrl(spi, POWER_OFF);
		spi_osc_power_ctrl(spi, POWER_OFF);
	} else if (value == 0) {
		spi_osc_power_ctrl(spi, POWER_ON);
		spi_clock_ctrl(spi, POWER_ON);
		//set_tc_driving(spi, LCD_MODE_U0);
	}
}
EXPORT_SYMBOL(tci_deep_sleep);

static void set_lpwg_mode(struct lpwg_control *ctrl, int mode)
{
	ctrl->double_tap_enable =
		((mode == LPWG_DOUBLE_TAP) | (mode == LPWG_PASSWORD)) ? 1 : 0;
	ctrl->password_enable = (mode == LPWG_PASSWORD) ? 1 : 0;
	ctrl->signature_enable = (mode == LPWG_SIGNATURE) ? 1 : 0;
	ctrl->lpwg_is_enabled = ctrl->double_tap_enable
		|| ctrl->password_enable || ctrl->signature_enable;
}

static int lpwg_control(struct sic_ts_data *ts, int mode)
{
	struct tci_ctrl_info *tci1 = &ts->tci_ctrl.tci1;
	struct tci_ctrl_info *tci2 = &ts->tci_ctrl.tci2;

	if (ts->pdata->mode.curr_mode == LCD_MODE_U3 && !ts->lpwg_ctrl.screen) {
		ts->pdata->mode.prev_mode = LCD_MODE_U3;
		ts->pdata->mode.curr_mode = LCD_MODE_U3_PARTIAL;
		mode = ts->lpwg_ctrl.lpwg_mode;
	}

	set_lpwg_mode(&ts->lpwg_ctrl, mode);
	sensor_state = 1;


	switch (mode) {
	case LPWG_SIGNATURE:
		break;
	case LPWG_DOUBLE_TAP:
		ts->tci_ctrl.tci_mode = 1;
		tci1->intr_delay = 0;
		tci1->tap_distance = 10;

		if (ts->fail_reason[0])
			DO_SAFE(tci_control(ts, FAIL_REASON_CTRL), error);
		if (ts->lpwg_ctrl.qcover == QUICKCOVER_CLOSE)
			DO_SAFE(tci_control(ts, ACTIVE_AREA_CTRL), error);
		DO_SAFE(tci_control(ts, TCI_ENABLE_CTRL), error);
		/* tap count = 2 */
		DO_SAFE(tci_control(ts, TAP_COUNT_CTRL), error);
		/* min inter_tap, 60ms*/
		DO_SAFE(tci_control(ts, MIN_INTERTAP_CTRL), error);
		/* max inter_tap, 700ms*/
		DO_SAFE(tci_control(ts, MAX_INTERTAP_CTRL), error);
		/* touch_slop = 10mm */
		DO_SAFE(tci_control(ts, TOUCH_SLOP_CTRL), error);
		/* tap distance = 10mm*/
		DO_SAFE(tci_control(ts, TAP_DISTANCE_CTRL), error);
		/* interrupt delay = 10ms unit*/
		DO_SAFE(tci_control(ts, INTERRUPT_DELAY_CTRL), error);

		if (atomic_read(&ts->lpwg_ctrl.is_suspend)) {
			/* qcover not close swipe enable*/
			if (ts->lpwg_ctrl.qcover != QUICKCOVER_CLOSE)
				swipe_enable(ts);
			else
				swipe_disable(ts);

			//DO_SAFE(tci_control(ts, DOZE2_MODE_SET), error);
			//DO_SAFE(set_tc_driving(ts->spi_device, ts->pdata->mode.curr_mode)	, error);
		}
#if 0
		else {
			DO_SAFE(tci_control(ts, PARTIAL_LPWG_ON), error);
		}
#endif
		break;
	case LPWG_PASSWORD:
		ts->tci_ctrl.tci_mode = 0x01 | 0x01 << 16;
		tci1->intr_delay = ts->pw_data.double_tap_check ? 68 : 0;
		tci1->tap_distance = 7;
		tci2->tap_count = ts->pw_data.tap_count;

		if (ts->fail_reason[0])
			DO_SAFE(tci_control(ts, FAIL_REASON_CTRL), error);
		DO_SAFE(tci_control(ts, OVERTAP_CTRL), error);
		DO_SAFE(tci_control(ts, TCI_ENABLE_CTRL), error);
		DO_SAFE(tci_control(ts, TAP_COUNT_CTRL), error);
		/* min inter_tap, 60ms*/
		DO_SAFE(tci_control(ts, MIN_INTERTAP_CTRL), error);
		/* max inter_tap, 700ms*/
		DO_SAFE(tci_control(ts, MAX_INTERTAP_CTRL), error);
		/* touch_slop = 0.1mm unit, 10mm */
		DO_SAFE(tci_control(ts, TOUCH_SLOP_CTRL), error);
		/* tap distance = 7mm*/
		DO_SAFE(tci_control(ts, TAP_DISTANCE_CTRL), error);
		/* interrupt delay = 10ms unit, 700ms */
		DO_SAFE(tci_control(ts, INTERRUPT_DELAY_CTRL), error);

		if (atomic_read(&ts->lpwg_ctrl.is_suspend)) {
			/* qcover not close swipe enable*/
			if (ts->lpwg_ctrl.qcover != QUICKCOVER_CLOSE)
				swipe_enable(ts);
			else
				swipe_disable(ts);

			//DO_SAFE(tci_control(ts, DOZE2_MODE_SET), error);
			//DO_SAFE(set_tc_driving(ts->spi_device, ts->pdata->mode.curr_mode)	, error);
		}
#if 0
		else {
			DO_SAFE(tci_control(ts, PARTIAL_LPWG_ON), error);
		}
#endif
		break;
	default:
		ts->tci_ctrl.tci_mode = 0;
		DO_SAFE(tci_control(ts, TCI_ENABLE_CTRL), error);
		swipe_disable(ts);
		/* because of swipe - notify to Touch IC about swipe resume*/
		/*
		if (ts->pdata->swipe_pwr_ctr == WAIT_TOUCH_PRESS) {
			u8 wdata = 0x08;
			if (sic_spi_write(ts->spi_device, tc_device_ctl,
					(u8 *)&wdata, sizeof(u32)) < 0) {
				TOUCH_I("%s swipe resume i2c write fail\n",
					__func__);
			}
			TOUCH_I("%s notify Touch IC swipe resume\n", __func__);
		} else {
			//DO_SAFE(tci_control(ts, DOZE1_MODE_SET), error);
			//DO_SAFE(set_tc_driving(ts->spi_device, ts->pdata->mode.curr_mode)
					, error);
		}*/
		break;
	}

	DO_SAFE(set_tc_driving(ts->spi_device, ts->pdata->mode.curr_mode)
						, error);

	TOUCH_I("%s : lpwg_mode[%d/%d]\n", __func__,
			mode, ts->lpwg_ctrl.lpwg_mode);

	return 0;
error:
	TOUCH_D(DEBUG_BASE_INFO | DEBUG_LPWG, "%s : FAIL!!!\n"
	, __func__);
	return -ERROR;
}

static int lpwg_update_all(struct sic_ts_data *ts)
{
	int ic_status = -1;
	int lpwg_status = -1;

	if (ts->lpwg_ctrl.screen) {
		if (ts->pdata->mode.curr_mode == LCD_MODE_U3_PARTIAL) {
			ts->pdata->mode.prev_mode = LCD_MODE_U3_PARTIAL;
			ts->pdata->mode.curr_mode = LCD_MODE_U3;
			lpwg_status = 0;
		}
#if 0
		if ((atomic_read(&ts->lpwg_ctrl.is_suspend) == TC_STATUS_RESUME) &&
				(ts->is_init == 1)) {
			/* because of partial
			   mode change to doze1 from doze1-partial */
			if (sensor_state == 0)
				set_tc_driving(ts->spi_device, TC_RESTART);
			tci_control(ts, ACTIVE_AREA_RESET_CTRL);
			lpwg_status = 0;
			TOUCH_I("lpwg_update_all, Doze 1 set\n");
		} else {
			TOUCH_I("skip lpwg_update_all\n");
		}
#endif
	} else {
		if (ts->pdata->swipe_pwr_ctr != SKIP_PWR_CON)
			ts->pdata->swipe_pwr_ctr = WAIT_SWIPE_WAKEUP;
		TOUCH_I("%s : swipe_pwr_ctr = %d\n", __func__,
				ts->pdata->swipe_pwr_ctr);

		/* lpwg_mode is 0, No Set LPWG */
		if (ts->lpwg_ctrl.lpwg_mode == 0) {
			ic_status = 0;
		} else {
			if (ts->lpwg_ctrl.qcover == QUICKCOVER_CLOSE) {
				TOUCH_I("QUICKCOVER CLOSE-DOUBLE_TAP MODE SET\n");
				if (sensor_state == 0) { /* Near -> QuickCover LPWG */
					ic_status = 1;
				} else { /* QuickCover */
					if (doze_state == DOZE2_STATUS) /* LPWG -> QuickCover LPWG*/
						ic_status = 2;
					else	/* QuickCover LPWG */
						lpwg_status = ts->lpwg_ctrl.lpwg_mode;
				}
			} else {
				TOUCH_I("QUICKCOVER OPEN\n");
				if (ts->lpwg_ctrl.sensor == SENSOR_STATUS_NEAR) { /* Near -> Deep Sleep */
					TOUCH_I("PROX SENSOR NEAR - TC OFF\n");
					ic_status = 0;
				} else {
					if (sensor_state == 0) { /* Deep Sleep -> Far -> Restart LPWG */
						TOUCH_I("TC ON\n");
						ic_status = 1;
					} else {	/* LPWG Normal Set*/
						TOUCH_I("NORMAL MODE SET\n");
						lpwg_status = ts->lpwg_ctrl.lpwg_mode;
					}
				}
			}
		}
	}

	if (ic_status == 0) {	/* NEAR, no need to LPWG Set */
		if (ts->pdata->mode.curr_mode == LCD_MODE_U0) {
			tci_deep_sleep(ts->spi_device, 1);
		} else {
			set_tc_driving(ts->spi_device, DRIVING_STOP);
		}

		sensor_state = 0;
	} else if (ic_status == 1) {	/* FAR, LPWG Set */
		tci_deep_sleep(ts->spi_device, 0);
		lpwg_status = ts->lpwg_ctrl.lpwg_mode;
	} else if (ic_status == 2) {
#if 0
		sic_ts_power(ts->spi_device, POWER_OFF);
		sic_ts_power(ts->spi_device, POWER_ON);
		msleep(ts->pdata->role->booting_delay);
		DO_SAFE(sic_ts_init(ts->spi_device), error);
		atomic_set(&ts->state->device_init, INIT_DONE);
#endif
	}

	if (lpwg_status > -1)
		DO_SAFE(lpwg_control(ts, lpwg_status), error);

	return NO_ERROR;
error:
	TOUCH_I("%s : FAIL!!!\n", __func__);
	return ERROR;
}


static int ts_interrupt_clear(struct sic_ts_data *ts)
{
	u32 wdata = 1;
	TOUCH_TRACE();

	DO_SAFE(sic_spi_write(ts->spi_device,
			tc_interrupt_status, (u8 *)&wdata, sizeof(u32)), error);
	return 0;
error:
	TOUCH_D(DEBUG_BASE_INFO | DEBUG_LPWG, "%s : FAIL!!!\n", __func__);
	return -ERROR;
}

/*
 * show_atcmd_fw_ver
 *
 * show only firmware version.
 * It will be used for AT-COMMAND
 */
static ssize_t show_atcmd_fw_ver(struct spi_device *spi, char *buf)
{
	struct sic_ts_data *ts =
			(struct sic_ts_data *)get_touch_handle(spi);

	int ret = 0;

	ret = snprintf(buf, PAGE_SIZE, "V%d.%02d\n",
			ts->fw_info.fw_version[0], ts->fw_info.fw_version[1]);

	return ret;
}

static void get_tci_info(struct sic_ts_data *ts)
{
	struct tci_ctrl_data *tci_ctrl = &ts->tci_ctrl;

	tci_ctrl->tci1.tap_count = 2;
	tci_ctrl->tci1.min_intertap = 6;
	tci_ctrl->tci1.max_intertap = 70;
	tci_ctrl->tci1.touch_slop = 100;
	tci_ctrl->tci1.tap_distance = 10;
	tci_ctrl->tci1.intr_delay = 0;

	tci_ctrl->tci2.min_intertap = 6;
	tci_ctrl->tci2.max_intertap = 70;
	tci_ctrl->tci2.touch_slop = 100;
	tci_ctrl->tci2.tap_distance = 255;
	tci_ctrl->tci2.intr_delay = 20;
}

static void get_swipe_info(struct sic_ts_data *ts)
{
	struct swipe_data *swp = &ts->swipe;

	swp->down.min_distance = ts->pdata->swp_down_caps->min_distance;
	swp->down.ratio_thres = ts->pdata->swp_down_caps->ratio_thres;
	swp->down.ratio_chk_period = ts->pdata->swp_down_caps->ratio_chk_period;
	swp->down.ratio_chk_min_distance =
		ts->pdata->swp_down_caps->ratio_chk_min_distance;
	swp->down.min_time_thres = ts->pdata->swp_down_caps->min_time_thres;
	swp->down.max_time_thres = ts->pdata->swp_down_caps->max_time_thres;
	swp->down.active_area_x0 = ts->pdata->swp_down_caps->active_area_x0;
	swp->down.active_area_y0 = ts->pdata->swp_down_caps->active_area_y0;
	swp->down.active_area_x1 = ts->pdata->swp_down_caps->active_area_x1;
	swp->down.active_area_y1 = ts->pdata->swp_down_caps->active_area_y1;

	swp->up.min_distance = ts->pdata->swp_up_caps->min_distance;
	swp->up.ratio_thres = ts->pdata->swp_up_caps->ratio_thres;
	swp->up.ratio_chk_period =
		ts->pdata->swp_up_caps->ratio_chk_period;
	swp->up.ratio_chk_min_distance =
		ts->pdata->swp_up_caps->ratio_chk_min_distance;
	swp->up.min_time_thres = ts->pdata->swp_up_caps->min_time_thres;
	swp->up.max_time_thres = ts->pdata->swp_up_caps->max_time_thres;
	swp->up.active_area_x0 = ts->pdata->swp_up_caps->active_area_x0;
	swp->up.active_area_y0 = ts->pdata->swp_up_caps->active_area_y0;
	swp->up.active_area_x1 = ts->pdata->swp_up_caps->active_area_x1;
	swp->up.active_area_y1 = ts->pdata->swp_up_caps->active_area_y1;

	swp->swipe_mode = SWIPE_UP_BIT;	/*SWIPE_DOWN_BIT;*//* | SWIPE_UP_BIT*/;
}

static int get_ic_info(struct sic_ts_data *ts)
{
	u32 rdata = 0;

	DO_SAFE(sic_spi_read(ts->spi_device, tc_version,
				(u8 *)&rdata, sizeof(u32)), error);

	ts->fw_info.fw_version[0] = ((rdata >> 8) & 0xFF);
	ts->fw_info.fw_version[1] = (rdata & 0xFF);
	TOUCH_D(DEBUG_BASE_INFO,
		"fw version : v%d.%02d, chip version : %d, protocol ver : %d\n",
		ts->fw_info.fw_version[0],
		ts->fw_info.fw_version[1],
		(rdata >> 16) & 0xFF,
		(rdata >> 24) & 0xFF);

	DO_SAFE(sic_spi_read(ts->spi_device, tc_revision_info,
				(u8 *)&rdata, sizeof(u32)), error);
	ts->fw_info.fw_revision = rdata & 0xFF;
	if (ts->fw_info.fw_revision == 0xFF)
		TOUCH_D(DEBUG_BASE_INFO, "IC_Revision: Unknown. Flash Erased\n");
	else
		TOUCH_D(DEBUG_BASE_INFO, "IC_Revision: %d\n", ts->fw_info.fw_revision);

	DO_SAFE(sic_spi_read(ts->spi_device, tc_product_id,
				(u8 *)&ts->fw_info.fw_product_id, 8), error);
	TOUCH_D(DEBUG_BASE_INFO, "IC_product_id: %s\n",
			ts->fw_info.fw_product_id);
#if 0
	DO_SAFE(sic_spi_read(ts->spi_device, tc_doze1_offset,
				(u8 *)&rdata, sizeof(u32)), error);
	doze1Offset[0] = rdata;
	doze1Offset[1] = rdata >> 16;
	DO_SAFE(sic_spi_read(ts->spi_device, tc_doze2_offset,
				(u8 *)&rdata, sizeof(u32)), error);
	doze2Offset[0] = rdata;
	doze2Offset[1] = rdata >> 16;
	DO_SAFE(sic_spi_read(ts->spi_device, tc_runinfo,
				(u8 *)&rdata, sizeof(u32)), error);
	runInfo = rdata & 0xffff;
#endif
	get_tci_info(ts);
	get_swipe_info(ts);

	if (detect_flash_ic(ts) != 0) {
		TOUCH_E("%s Flash device probing fail!!! \n", __func__);
		return -EIO;
	}

	return 0;

error:
	TOUCH_E("%s Fail\n", __func__);
	return -EIO;

}

static int get_binFW_version(struct sic_ts_data *ts)
{
	const struct firmware *fw_entry = NULL;
	const u8 *firmware = NULL;
	int rc = 0;
	u32 fw_idx;

	rc = request_firmware(&fw_entry,
		ts->pdata->inbuilt_fw_name,
		&ts->spi_device->dev);
	if (rc != 0) {
		TOUCH_E("request_firmware() failed %d\n", rc);
		return -EIO;
	}

	firmware = fw_entry->data;
	fw_idx = *((u32 *)&firmware[ts->pdata->fw_ver_addr]);
	memcpy(ts->fw_info.fw_image_version, &firmware[fw_idx], 2);
	fw_idx = *((u32 *)&firmware[ts->pdata->fw_pid_addr]);
	memcpy(ts->fw_info.fw_image_product_id, &firmware[fw_idx], 8);

	release_firmware(fw_entry);

	return rc;
}

static ssize_t show_firmware(struct spi_device *spi, char *buf)
{
	struct sic_ts_data *ts =
			(struct sic_ts_data *)get_touch_handle(spi);
	int ret = 0;
	int rc = 0;

	rc = get_ic_info(ts);
	rc += get_binFW_version(ts);

	if (rc < 0) {
		ret += snprintf(buf+ret,
				PAGE_SIZE,
				"-1\n");
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"Read Fail Touch IC Info.\n");
		return ret;
	}

	ret = snprintf(buf, PAGE_SIZE, "\n======== Firmware Info ========\n");
	ret += snprintf(buf+ret, PAGE_SIZE - ret,
				"=== ic_fw_version info ===\n");
	ret += snprintf(buf+ret, PAGE_SIZE - ret, "IC_fw_version : v%d.%02d\n",
				ts->fw_info.fw_version[0],
				ts->fw_info.fw_version[1]);

	if (ts->fw_info.fw_revision == 0xFF) {
		ret += snprintf(buf+ret, PAGE_SIZE - ret,
			"IC_revision[Unknown. Flash Erased]\n");
	} else {
		ret += snprintf(buf+ret, PAGE_SIZE - ret,
			"IC_revision[%d]\n", ts->fw_info.fw_revision);
	}

	ret += snprintf(buf+ret, PAGE_SIZE - ret, "IC_product_id[%s]\n",
				ts->fw_info.fw_product_id);
#if 0
	ret += snprintf(buf+ret, PAGE_SIZE - ret, "Run info : %d\n",
				runInfo);
	ret += snprintf(buf+ret, PAGE_SIZE - ret,
			"Offset1 Left : %d, Right : %d\n",
			(int16_t)doze1Offset[0], (int16_t)doze1Offset[1]);
	ret += snprintf(buf+ret, PAGE_SIZE - ret,
			"Offset2 Left : %d, Right : %d\n\n",
			(int16_t)doze2Offset[0], (int16_t)doze2Offset[1]);
#endif
	ret += snprintf(buf+ret, PAGE_SIZE - ret,
				"=== img_fw_version info ===\n");
	ret += snprintf(buf+ret, PAGE_SIZE - ret, "Img_fw_version : v%d.%02d\n",
				ts->fw_info.fw_image_version[0],
				ts->fw_info.fw_image_version[1]);
	ret += snprintf(buf+ret, PAGE_SIZE - ret, "Img_product_id[%s]\n\n",
				ts->fw_info.fw_image_product_id);

	return ret;
}

static ssize_t show_sic_fw_version(struct spi_device *spi, char *buf)
{
	struct sic_ts_data *ts =
			(struct sic_ts_data *)get_touch_handle(spi);
	int ret = 0;
	int rc = 0;

	rc = get_ic_info(ts);

	if (rc < 0) {
		ret += snprintf(buf+ret, PAGE_SIZE,
				"-1\n");
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"Read Fail Touch IC Info.\n");
		return ret;
	}

	ret = snprintf(buf + ret, PAGE_SIZE - ret,
			"\n======== Auto Touch Test ========\n");
	ret += snprintf(buf+ret, PAGE_SIZE - ret,
				"version : v%d.%02d\n",
				ts->fw_info.fw_version[0],
				ts->fw_info.fw_version[1]);

	if (ts->fw_info.fw_revision == 0xFF) {
		ret += snprintf(buf+ret, PAGE_SIZE - ret,
			"IC_revision[Unknown. Flash Erased]\n");
	} else {
		ret += snprintf(buf+ret, PAGE_SIZE - ret,
			"IC_revision[%d]\n", ts->fw_info.fw_revision);
	}

	ret += snprintf(buf+ret, PAGE_SIZE - ret,
				"IC_product_id[%s]\n\n",
				ts->fw_info.fw_product_id);
	return ret;

}
static ssize_t store_tci(struct spi_device *spi,
	const char *buf, size_t count)
{
	struct sic_ts_data *ts
		= (struct sic_ts_data *)get_touch_handle(spi);
	struct tci_ctrl_data *tci_ctrl = &ts->tci_ctrl;
	struct tci_ctrl_info *tci = NULL;
	char tci_info;
	u16 tci_type;
	u16 value = 0;

	if (sscanf(buf, "%hu %c %hu", &tci_type, &tci_info, &value) <= 0)
		return count;


	if ((tci_type < 1) || (tci_type > 2) ||
		(tci_info < 'a') ||	(tci_info > 'g')) {
		TOUCH_I("<writing tci guide>\n");
		TOUCH_I("echo [tci_type] [tci_info] [value] > tci\n");
		TOUCH_I("[tci_type]: 1(tci-1), 2(tci-2)\n");
		TOUCH_I("[tci_info],\n");
		TOUCH_I("a(enable),\n");
		TOUCH_I("b(tap_count),\n");
		TOUCH_I("c(min_intertap),\n");
		TOUCH_I("d(max_intertap),\n");
		TOUCH_I("e(touch_slop),\n");
		TOUCH_I("f(tap_distance),\n");
		TOUCH_I("g(intr_delay),\n");
		TOUCH_I("[value]: (0x00~0xFF)\n");
		return count;
	}

	switch (tci_type) {
	case 1:
		tci = &tci_ctrl->tci1;
		break;
	case 2:
		tci = &tci_ctrl->tci2;
		break;
	default:
		TOUCH_I("unknown tci_type(%d)\n", tci_type);
		return count;
	}

	switch (tci_info) {
	case 'a':
		tci_ctrl->tci_mode = (tci_type == 1 ? 1 : 1 | (1 << 16));
		break;
	case 'b':
		tci->tap_count = value;
		break;
	case 'c':
		tci->min_intertap = value;
		break;
	case 'd':
		tci->max_intertap = value;
		break;
	case 'e':
		tci->touch_slop = value;
		break;
	case 'f':
		tci->tap_distance = value;
		break;
	case 'g':
		tci->intr_delay = value;
		break;
	}

	return count;
}

static ssize_t show_tci(struct spi_device *spi, char *buf)
{
	struct sic_ts_data *ts
		= (struct sic_ts_data *)get_touch_handle(spi);

	int ret = 0;
	int i = 0;
	u32 rdata = 0;
	u32 buffer[7];

	DO_SAFE(sic_spi_read(ts->spi_device, tc_status,
			(u8 *)&rdata, sizeof(u32)),
			error);

	rdata = (rdata >> 6) & 0x03;
	ret += snprintf(buf+ret, PAGE_SIZE - ret, "report_mode [%s]\n",
				rdata == 2 ? "WAKEUP_ONLY" : "NORMAL");

	DO_SAFE(sic_spi_read(ts->spi_device, CMD_ABT_TCI_ENABLE_CTRL_READ,
				(u8 *)&buffer[0], sizeof(u32)),
		error);
	ret += snprintf(buf+ret, PAGE_SIZE - ret,
				"wakeup_gesture [%d]\n\n",
					(buffer[0] >> 16) ? 2 :
					(buffer[0] ? 1 : 0));

	DO_SAFE(sic_spi_read(ts->spi_device, CMD_ABT_TAP_COUNT_CTRL_READ,
				(u8 *)&buffer[1], sizeof(u32)),
		error);
	DO_SAFE(sic_spi_read(ts->spi_device, CMD_ABT_MIN_INTERTAP_CTRL_READ,
				(u8 *)&buffer[2], sizeof(u32)),
		error);
	DO_SAFE(sic_spi_read(ts->spi_device, CMD_ABT_MAX_INTERTAP_CTRL_READ,
				(u8 *)&buffer[3], sizeof(u32)),
		error);
	DO_SAFE(sic_spi_read(ts->spi_device, CMD_ABT_TOUCH_SLOP_CTRL_READ,
				(u8 *)&buffer[4], sizeof(u32)),
		error);
	DO_SAFE(sic_spi_read(ts->spi_device, CMD_ABT_TAP_DISTANCE_CTRL_READ,
				(u8 *)&buffer[5], sizeof(u32)),
		error);
	DO_SAFE(sic_spi_read(ts->spi_device, CMD_ABT_INT_DELAY_CTRL_READ,
				(u8 *)&buffer[6], sizeof(u32)),
		error);

	for (i = 0; i < 2; i++) {
		ret += snprintf(buf+ret, PAGE_SIZE - ret, "TCI - %d\n", i+1);
		ret += snprintf(buf+ret, PAGE_SIZE - ret, "TCI [%s]\n",
		((buffer[0] >> (i*16)) & 0x1) == 1 ? "enabled" : "disabled");
		ret += snprintf(buf+ret, PAGE_SIZE - ret, "Tap Count [%d]\n",
					(buffer[1] >> (i*16)) & 0xFF);
		ret += snprintf(buf+ret, PAGE_SIZE - ret, "Min InterTap [%d]\n",
					(buffer[2] >> (i*16)) & 0xFF);
		ret += snprintf(buf+ret, PAGE_SIZE - ret, "Max InterTap [%d]\n",
					(buffer[3] >> (i*16)) & 0xFF);
		ret += snprintf(buf+ret, PAGE_SIZE - ret, "Touch Slop [%d]\n",
					(buffer[4] >> (i*16)) & 0xFF);
		ret += snprintf(buf+ret, PAGE_SIZE - ret, "Tap Distance [%d]\n",
					(buffer[5] >> (i*16)) & 0xFF);
		ret += snprintf(buf+ret, PAGE_SIZE - ret,
					"Interrupt Delay [%d]\n\n",
					(buffer[6] >> (i*16)) & 0xFF);
	}
	return ret;

error:
	TOUCH_D(DEBUG_BASE_INFO, "%s : FAIL!!!\n", __func__);
	return -ERROR;
}

static ssize_t store_reg_ctrl(struct spi_device *spi,
	const char *buf, size_t count)
{
	char command[6] = {0};
	u32 reg = 0;
	int value = 0;
	u32 wdata = 1;
	u32 rdata;

	if (sscanf(buf, "%5s %x %d", command, &reg, &value) <= 0)
		return count;

	if (!strcmp(command, "write")) {
		u16 reg_addr = reg;
		wdata = value;
		if (sic_spi_write(spi, reg_addr,
				(u8 *)&wdata, sizeof(u32)) < 0)
			TOUCH_E("reg addr 0x%x write fail\n", reg_addr);
		else
			TOUCH_D(DEBUG_BASE_INFO, "reg[%x] = 0x%x\n",
						reg_addr, wdata);
	} else if (!strcmp(command, "read")) {
		u16 reg_addr = reg;
		if (sic_spi_read(spi, reg_addr,
				(u8 *)&rdata, sizeof(u32)) < 0)
			TOUCH_E("reg addr 0x%x read fail\n", reg_addr);
		else
			TOUCH_D(DEBUG_BASE_INFO, "reg[%x] = 0x%x\n",
						reg_addr, rdata);
	} else {
		TOUCH_D(DEBUG_BASE_INFO, "Usage\n");
		TOUCH_D(DEBUG_BASE_INFO, "Write reg value\n");
		TOUCH_D(DEBUG_BASE_INFO, "Read reg\n");
	}
	return count;
}

static ssize_t show_object_report(struct spi_device *spi, char *buf)
{
	struct sic_ts_data *ts
		= (struct sic_ts_data *)get_touch_handle(spi);

	u8 object_report_enable_reg = 0;
	u8 temp[5];

	int ret = 0;
	int i;

	DO_SAFE(sic_spi_read(ts->spi_device, OBJECT_REPORT_ENABLE_REG_READ,
					(u8 *)&object_report_enable_reg,
					sizeof(object_report_enable_reg)),
			error);

	for (i = 0; i < 5; i++)
		temp[i] = (object_report_enable_reg >> i) & 0x01;

	ret = snprintf(buf, PAGE_SIZE,
		"\n======= read object_report_enable register =======\n");
	ret += snprintf(buf+ret, PAGE_SIZE - ret,
		" Addr Bit4 Bit3 Bit2 Bit1 Bit0 HEX\n");
	ret += snprintf(buf+ret, PAGE_SIZE - ret,
		"--------------------------------------------------\n");
	ret += snprintf(buf+ret, PAGE_SIZE - ret,
		" 0x%02X %4d %4d %4d %4d %4d 0x%02X\n",
		OBJECT_REPORT_ENABLE_REG_READ, temp[4], temp[3],
		temp[2], temp[1], temp[0], object_report_enable_reg);
	ret += snprintf(buf+ret, PAGE_SIZE - ret,
		"--------------------------------------------------\n");
	ret += snprintf(buf+ret, PAGE_SIZE - ret,
		" Bit0  : [F]inger -> %7s\n", temp[0] ? "Enable" : "Disable");
	ret += snprintf(buf+ret, PAGE_SIZE - ret,
		" Bit1  : [S]tylus -> %7s\n", temp[1] ? "Enable" : "Disable");
	ret += snprintf(buf+ret, PAGE_SIZE - ret,
		" Bit2  : [P]alm -> %7s\n", temp[2] ? "Enable" : "Disable");
	ret += snprintf(buf+ret, PAGE_SIZE - ret,
		" Bit3  : [G]loved Finger -> %7s\n",
			temp[3] ? "Enable" : "Disable");
	ret += snprintf(buf+ret, PAGE_SIZE - ret,
		" Bit4  : Hand[E]dge  -> %7s\n",
			temp[4] ? "Enable" : "Disable");
	ret += snprintf(buf+ret, PAGE_SIZE - ret,
		"==================================================\n\n");

	return ret;

error:
	TOUCH_D(DEBUG_BASE_INFO, "%s : FAIL!!!\n", __func__);
	return -ERROR;
}

static ssize_t store_object_report(struct spi_device *spi,
	const char *buf, size_t count)
{
	struct sic_ts_data *ts
		= (struct sic_ts_data *)get_touch_handle(spi);

	char select[16];
	u8 value = 2;
	int select_cnt;
	int i;
	u8 bit_select = 0;
	u8 object_report_enable_reg_old;
	u8 object_report_enable_reg_new;
	u32 wdata = 0;

	if (sscanf(buf, "%15s %hhu", select, &value) <= 0)
		return count;

	if ((strlen(select) > 5) || (value > 1)) {
		TOUCH_I("<writing object_report guide>\n");
		TOUCH_I("echo [select] [value] > object_report\n");
		TOUCH_I(
			"select: [F]inger, [S]tylus, [P]alm, [G]loved Finger, Hand[E]dge\n");
		TOUCH_I("select length: 1~5, value: 0~1\n");
		TOUCH_I("ex) echo F 1 > object_report (enable [F]inger)\n");
		TOUCH_I("ex) echo s 1 > object_report (enable [S]tylus)\n");
		TOUCH_I("ex) echo P 0 > object_report (disable [P]alm)\n");
		TOUCH_I(
			"ex) echo gE 1 > object_report (enable [G]loved Finger, Hand[E]dge)\n");
		TOUCH_I(
			"ex) echo eFGp 1 > object_report (enable Hand[E]dge, [F]inger, [G]loved Finger, [P]alm)\n");
		TOUCH_I(
			"ex) echo gPsF 0 > object_report (disable [G]loved Finger, [P]alm, [S]tylus, [F]inger)\n");
		TOUCH_I(
			"ex) echo GPSfe 1 > object_report (enable all object)\n");
		TOUCH_I(
			"ex) echo egpsf 0 > object_report (disbale all object)\n");
	} else {
		select_cnt = strlen(select);

		for (i = 0; i < select_cnt; i++) {
			switch ((char)(*(select + i))) {
			case 'F': case 'f': /* (F)inger */
				bit_select |= (0x01 << 0);
				break;
			case 'S': case 's': /* (S)tylus */
				bit_select |= (0x01 << 1);
				break;
			case 'P': case 'p': /* (P)alm */
				bit_select |= (0x01 << 2);
				break;
			case 'G': case 'g': /* (G)loved Finger */
				bit_select |= (0x01 << 3);
				break;
			case 'E': case 'e': /* Hand (E)dge */
				bit_select |= (0x01 << 4);
				break;
			default:
				break;
			}
		}

		DO_SAFE(sic_spi_read(ts->spi_device, OBJECT_REPORT_ENABLE_REG_READ,
					(u8 *)&object_report_enable_reg_old,
					sizeof(object_report_enable_reg_old)),
			error);

		object_report_enable_reg_new = object_report_enable_reg_old;

		if (value > 0)
			object_report_enable_reg_new |= bit_select;
		else
			object_report_enable_reg_new &= ~(bit_select);

		wdata = object_report_enable_reg_new;

		DO_SAFE(sic_spi_write(spi, OBJECT_REPORT_ENABLE_REG_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
	}

	return count;

error:
	TOUCH_D(DEBUG_BASE_INFO, "%s : FAIL!!!\n", __func__);
	return -ERROR;
}

static void write_file(char *filename, char *data, int time, int append)
{
	int fd = 0;
	char *fname = NULL;
	char time_string[64] = {0};
	struct timespec my_time;
	struct tm my_date;
	mm_segment_t old_fs = get_fs();

	set_fs(KERNEL_DS);

	if (filename == NULL) {
		switch (sic_mfts_mode) {
		case 0:
			fname = "/cache/touch_self_test.txt";
			break;
		case 1:
			fname = "/cache/touch_self_test_mfts_folder.txt";
			break;
		case 2:
			fname = "/cache/touch_self_test_mfts_flat.txt";
			break;
		case 3:
			fname = "/cache/touch_self_test_mfts_curved.txt";
			break;
		default:
			TOUCH_I("%s : not support mfts_mode\n", __func__);
			break;
		}
	} else
		fname = filename;

	if (fname) {
		if (append)
			fd = sys_open(fname, O_WRONLY|O_CREAT|O_APPEND, 0666);
		else
			fd = sys_open(fname, O_WRONLY|O_CREAT, 0666);
	} else {
		TOUCH_E("%s : fname is NULL, can not open FILE\n", __func__);
		set_fs(old_fs);
		return;
	}

	if (fd >= 0) {
		if (time > 0) {
			my_time = __current_kernel_time();
			time_to_tm(my_time.tv_sec,
				sys_tz.tz_minuteswest * 60 * (-1), &my_date);
			snprintf(time_string, 64,
				"\n%02d-%02d %02d:%02d:%02d.%03lu\n\n",
				my_date.tm_mon + 1,
				my_date.tm_mday,
				my_date.tm_hour,
				my_date.tm_min,
				my_date.tm_sec,
				(unsigned long) my_time.tv_nsec / 1000000);
			sys_write(fd, time_string, strlen(time_string));
		}
		sys_write(fd, data, strlen(data));
		sys_close(fd);
	} else {
		TOUCH_I("File open failed\n");
	}
	set_fs(old_fs);
}

static ssize_t get_data(struct sic_ts_data *ts, int16_t *buf, u32 wdata)
{
	u32 rdata = 1;
	int retry = 40;
	int __frame_size = ROW_SIZE*COL_SIZE*RAWDATA_SIZE;

	/* write 1 : GETRAWDATA
	   write 2 : GETDELTA */
	TOUCH_I("======== get data ========\n");

	DO_SAFE(sic_spi_write(ts->spi_device,
		rawdata_ctl_write, (u8 *)&wdata, sizeof(wdata)), error);
	TOUCH_I("i2c_write, wdata = %d\n", wdata);

	/* wait until 0 is written */
	do {
		msleep(200);
		DO_SAFE(sic_spi_read(ts->spi_device,
			rawdata_ctl_read, (u8 *)&rdata, sizeof(rdata)), error);

	} while ((rdata != 0) && retry--);
	/* check whether 0 is written or not */

	if (rdata != 0) {
		TOUCH_E("== get data time out! ==\n");
		return -EIO;
	}

	/*read data*/
	if (__frame_size % 4)
		__frame_size = (((__frame_size >> 2) + 1) << 2);

	DO_SAFE(sic_spi_read(ts->spi_device, M2_RAWDATA_ADDR,
				(u8 *)buf, __frame_size), error);

	return 0;

error:
	return -EIO;
}

static ssize_t get_baseline_data
	(struct sic_ts_data *ts, int16_t *buf, u32 wdata)
{
	int __frame_size = ROW_SIZE*COL_SIZE*RAWDATA_SIZE;

	/* write 1 : GETBASELINEDATA */

	TOUCH_I("======== get data ========\n");

	DO_SAFE(sic_spi_write(ts->spi_device,
		tc_mem_sel, (u8 *)&wdata, sizeof(wdata)), error);
	TOUCH_I("spi_write, wdata = %d\n", wdata);

	/*read data*/
	if (__frame_size % 4)
		__frame_size = (((__frame_size >> 2) + 1) << 2);

	DO_SAFE(sic_spi_read(ts->spi_device, BASELINE_ADDR,
				(u8 *)buf, __frame_size), error);

	return 0;

error:
	return -EIO;
}

static ssize_t show_tci_fail_reason(struct spi_device *spi, char *buf)
{
	int ret = 0;
	u32 rdata = -1;
	struct sic_ts_data *ts =
			(struct sic_ts_data *)get_touch_handle(spi);
	char *type[3] = { "Disable Type", "Buffer Type", "Always Report Type" };
	int i = 1;

	if (sic_spi_read(spi, CMD_ABT_LPWG_TCI_FAIL_DEBUG_READ,
				(u8 *)&rdata, 4) < 0) {
		TOUCH_I("Fail to Read TCI Debug Reason type\n");
	} else {
		ret = snprintf(buf + ret, PAGE_SIZE,
				"Read TCI Debug Reason type[IC] = %s\n",
				type[rdata & 0x8 ? 2 : (rdata & 0x4 ? 1 : 0)]);
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Read TCI Debug Reason type[Driver] = %s\n",
				type[ts->fail_reason[0]]);
		TOUCH_I("Read TCI Debug Reason type = %s\n",
				type[ts->fail_reason[0]]);
	}

	ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Currently set Debug Reasons\n");
	for (i = 1; i < 9; i++) {
		if (ts->fail_type & (0x1 << i))
			ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"%s\n", sic_tci_fail_reason_str[i]);
	}

	return ret;
}

static ssize_t store_tci_fail_reason(struct spi_device *spi,
						const char *buf, size_t count)
{
	int value = 0, reason = 0;
	struct sic_ts_data *ts =
			(struct sic_ts_data *)get_touch_handle(spi);
	char *type[3] = { "Disable Type", "Buffer Type", "Always Report Type" };

	if (sscanf(buf, "%d %d", &value, &reason) <= 0)
		return count;

	if (value > 2 || value < 0) {
		TOUCH_I("SET TCI debug reason wrong, 0, 1, 2 only\n");
		return count;
	} else
		TOUCH_I("SET TCI debug reason type = %s\n", type[value]);

	if (!reason || !value)
		ts->fail_type = TCI_DEBUG_REASON_ALL;
	else
		ts->fail_type &= ~reason;

	ts->fail_reason[0] = (u32)value;

	return count;
}
static ssize_t show_swipe_fail_reason(struct spi_device *spi, char *buf)
{
	int ret = 0;
	u32 rdata = -1;
	struct sic_ts_data *ts =
			(struct sic_ts_data *)get_touch_handle(spi);
	char *type[3] = { "Disable Type", "Buffer Type", "Always Report Type" };

	if (sic_spi_read(spi, CMD_ABT_LPWG_SWIPE_FAIL_DEBUG_READ,
				(u8 *)&rdata, 4) < 0) {
		TOUCH_I("Fail to Read SWIPE fail reason type\n");
	} else {
		ret = snprintf(buf + ret, PAGE_SIZE,
				"Read SWIPE Fail reason type[IC] = %s\n",
				type[rdata]);
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Read SWIPE Fail reason type[Driver] = %s\n",
				type[ts->fail_reason[1]]);
		TOUCH_I("Read SWIPE Fail reason type = %s\n",
				type[ts->fail_reason[1]]);
	}

	return ret;
}

static ssize_t store_swipe_fail_reason(struct spi_device *spi,
						const char *buf, size_t count)
{
	int value = 0;
	u32 wdata;
	struct sic_ts_data *ts =
			(struct sic_ts_data *)get_touch_handle(spi);
	char *type[3] = { "Disable Type", "Buffer Type", "Always Report Type" };

	if (sscanf(buf, "%d", &value) <= 0)
		return count;
	wdata = (u32)value;

	if (sic_spi_write(spi, CMD_ABT_LPWG_SWIPE_FAIL_DEBUG_WRITE,
				(u8 *)&wdata, 4) < 0) {
		TOUCH_I("Fail to Write SWIPE fail reason type\n");
	} else {
		TOUCH_I("Write SWIPE Fail reason type = %s\n", type[wdata]);
		ts->fail_reason[1] = wdata;
	}

	return count;
}

static ssize_t show_swipe_param(struct spi_device *spi, char *buf)
{
	struct sic_ts_data *ts =
			(struct sic_ts_data *)get_touch_handle(spi);
	struct swipe_data *swp = &ts->swipe;
	int ret = 0;

	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"swipe_mode = 0x%02X\n",
			swp->swipe_mode);

	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"=================================================\n");

	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"down.min_distance = 0x%02X (%dmm)\n",
			swp->down.min_distance,
			swp->down.min_distance);

	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"down.ratio_thres = 0x%02X (%d%%)\n",
			swp->down.ratio_thres,
			swp->down.ratio_thres);

	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"down.ratio_chk_period = 0x%02X (%dframes)\n",
			swp->down.ratio_chk_period,
			swp->down.ratio_chk_period);

	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"down.ratio_chk_min_distance = 0x%02X (%dmm)\n",
			swp->down.ratio_chk_min_distance,
			swp->down.ratio_chk_min_distance);

	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"down.min_time_thres =	0x%02X (%d0ms)\n",
			swp->down.min_time_thres,
			swp->down.min_time_thres);

	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"down.max_time_thres = 0x%02X (%d0ms)\n",
			swp->down.max_time_thres,
			swp->down.max_time_thres);

	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"down.active_area = x0,y0(%d,%d) x1,y1(%d,%d)\n",
			swp->down.active_area_x0, swp->down.active_area_y0,
			swp->down.active_area_x1, swp->down.active_area_y1);

	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"=================================================\n");

	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"up.min_distance = 0x%02X (%dmm)\n",
			swp->up.min_distance,
			swp->up.min_distance);

	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"up.ratio_thres = 0x%02X (%d%%)\n",
			swp->up.ratio_thres,
			swp->up.ratio_thres);

	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"up.ratio_chk_period = 0x%02X (%dframes)\n",
			swp->up.ratio_chk_period,
			swp->up.ratio_chk_period);

	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"up.ratio_chk_min_distance = 0x%02X (%dmm)\n",
			swp->up.ratio_chk_min_distance,
			swp->up.ratio_chk_min_distance);

	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"up.min_time_thres =  0x%02X (%d0ms)\n",
			swp->up.min_time_thres,
			swp->up.min_time_thres);

	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"up.max_time_thres = 0x%02X (%d0ms)\n",
			swp->up.max_time_thres,
			swp->up.max_time_thres);

	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"up.active_area = x0,y0(%d,%d) x1,y1(%d,%d)\n",
			swp->up.active_area_x0, swp->up.active_area_y0,
			swp->up.active_area_x1, swp->up.active_area_y1);

	return ret;
}
static ssize_t store_swipe_param(struct spi_device *spi,
		const char *buf, size_t count)
{
	struct sic_ts_data *ts =
			(struct sic_ts_data *)get_touch_handle(spi);
	struct swipe_data *swp = &ts->swipe;
	struct swipe_ctrl_info *swpd = NULL;
	char direction;
	char select;
	u16 value;

	if (sscanf(buf, "%c %c %hu", &direction, &select, &value) <= 0)
		return count;

	if (((direction != 'd') && (direction != 'u'))
			|| (select < 'a')
			|| (select > 'j')) {
		TOUCH_I("<writing swipe_param guide>\n");
		TOUCH_I("echo [direction] [select] [value] > swipe_param\n");
		TOUCH_I("[direction]: d(down), u(up)\n");
		TOUCH_I("[select]:\n");
		TOUCH_I("a(min_distance),\n");
		TOUCH_I("b(ratio_thres),\n");
		TOUCH_I("c(ratio_chk_period),\n");
		TOUCH_I("d(ratio_chk_min_distance),\n");
		TOUCH_I("e(min_time_thres),\n");
		TOUCH_I("f(max_time_thres),\n");
		TOUCH_I("g(active_area_x0),\n");
		TOUCH_I("h(active_area_y0),\n");
		TOUCH_I("i(active_area_x1),\n");
		TOUCH_I("j(active_area_y1)\n");
		TOUCH_I("[value]: (0x00~0xFF) or (0x00~0xFFFF)\n");
		return count;
	}

	switch (direction) {
	case 'd':
		swpd = &swp->down;
		break;
	case 'u':
		swpd = &swp->up;
		break;
	default:
		TOUCH_I("unknown direction(%c)\n", direction);
		return count;
	}

	switch (select) {
	case 'a':
		swpd->min_distance = value;
		break;
	case 'b':
		swpd->ratio_thres = value;
		break;
	case 'c':
		swpd->ratio_chk_period = value;
		break;
	case 'd':
		swpd->ratio_chk_min_distance = value;
		break;
	case 'e':
		swpd->min_time_thres = value;
		break;
	case 'f':
		swpd->max_time_thres = value;
		break;
	case 'g':
		swpd->active_area_x0 = value;
		break;
	case 'h':
		swpd->active_area_y0 = value;
		break;
	case 'i':
		swpd->active_area_x1 = value;
		break;
	case 'j':
		swpd->active_area_y1 = value;
		break;
	default:
		break;
	}

	return count;
}

static ssize_t show_swipe_mode(struct spi_device *spi, char *buf)
{
	struct sic_ts_data *ts =
			(struct sic_ts_data *)get_touch_handle(spi);
	struct swipe_data *swp = &ts->swipe;
	int ret = 0;

	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"swipe_mode = 0x%02X\n",
			swp->swipe_mode);
	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"Down[%s] UP[%s]\n",
			swp->swipe_mode & SWIPE_DOWN_BIT ? "Enable" : "Disable",
			swp->swipe_mode & SWIPE_UP_BIT ? "Enable" : "Disable");

	return ret;
}

static ssize_t store_swipe_mode(struct spi_device *spi,
		const char *buf, size_t count)
{
	struct sic_ts_data *ts =
		(struct sic_ts_data *)get_touch_handle(spi);
	struct swipe_data *swp = &ts->swipe;
	int down = 0;
	int up = 0;
	u32 mode = 0;

	if (sscanf(buf, "%d %d", &down, &up) <= 0)
		return count;

	if (down)
		mode |= SWIPE_DOWN_BIT;
	else
		mode &= ~(SWIPE_DOWN_BIT);

	if (up)
		mode |= SWIPE_UP_BIT;
	else
		mode &= ~(SWIPE_UP_BIT);

	swp->swipe_mode = mode;

	return count;
}

static ssize_t store_boot_mode(struct spi_device *spi,
		const char *buf, size_t count)
{
	struct sic_ts_data *ts =
		(struct sic_ts_data *)get_touch_handle(spi);
	int current_mode;

	if (sscanf(buf, "%d", &current_mode) <= 0)
		return count;

	mutex_lock(&ts->pdata->thread_lock);
	switch (current_mode) {
	case CHARGERLOGO_MODE:
		TOUCH_I("%s: Charger mode!!! Disable irq\n",
				__func__);

		sic_ts_power(spi, POWER_OFF);
		atomic_set(&ts->state->power, POWER_OFF);
		atomic_set(&ts->state->device_init, INIT_NONE);
		boot_mode = current_mode;
		break;
	case NORMAL_BOOT_MODE:
		TOUCH_I("%s: Normal boot mode!!! Enable irq\n",
				__func__);
		if (boot_mode != current_mode) {
			sic_ts_power(spi, POWER_ON);
			msleep(ts->pdata->role->booting_delay);
			DO_SAFE(sic_ts_init(ts->spi_device), error);
			atomic_set(&ts->state->device_init, INIT_DONE);
		}
		boot_mode = current_mode;
		break;
	default:
		break;
	}
	mutex_unlock(&ts->pdata->thread_lock);

	return count;

error:
	mutex_unlock(&ts->pdata->thread_lock);
	TOUCH_E("%s, failed init\n",
			__func__);
	return count;
}

int prd_os_result_get(struct spi_device *spi, u32 *buf)
{
	struct sic_ts_data *ts =
				(struct sic_ts_data *)get_touch_handle(spi);
	DO_SAFE(sic_spi_read(ts->spi_device,
				tc_tsp_test_os_result,
				(u8 *)buf,
				sizeof(u32)*COL_SIZE), error);

	return 0;
error:
	TOUCH_E("prd_os_result_get() SPI fail\n");
	return -ERROR;
}

int prd_os_xline_result_read(struct spi_device *spi,
					u8 (*result_buffer)[ROW_SIZE], int type)
{
	struct sic_ts_data *ts =
				(struct sic_ts_data *)get_touch_handle(spi);
	int i;
	int j;
	u32 buffer[COL_SIZE] = {0,};
	int ret = 0;
	u8 write_value = 0x0;

	switch (type) {
		case OPEN_NODE_TEST:
			write_value = 0x1;
			break;
		case SHORT_NODE_TEST:
			write_value = 0x2;
			break;
	}

	ret = prd_os_result_get(spi, buffer);

	if (ret == 0) {
		for (i = 0; i < COL_SIZE; i++) {
			for (j = 0; j < ROW_SIZE; j++) {
				if ((buffer[i] & (0x1 << j)) != 0) {
					result_buffer[i][j] = (result_buffer[i][j] | write_value);
				}
			}
		}
	}
	sic_spi_read(ts->spi_device, 0x8C40,
				(u8 *)sicImage, COL_SIZE*ROW_SIZE*2);

	for (i = 0 ; i < COL_SIZE ; i++) {
		char log_buf[LOG_BUF_SIZE] = {0,};
		int log_ret = 0;

		log_ret += snprintf(log_buf + log_ret,
					LOG_BUF_SIZE - log_ret, "[%2d]  ", i);

		for (j = 0 ; j < ROW_SIZE ; j++) {
			log_ret += snprintf(log_buf + log_ret,
						LOG_BUF_SIZE - log_ret, "%5d ",
						(int)(sicImage[i][j]));
		}
		TOUCH_I("%s\n", log_buf);
	}
	return ret;
}

static int sdcard_spec_file_read(void)
{
	int ret = 0;
	int fd;
	char *path[4] = { "/cache/p1_limit.txt",
			"/cache/p1_limit_mfts_folder.txt",
			"/cache/p1_limit_mfts_flat.txt",
			"/cache/p1_limit_mfts_curved.txt" };
	mm_segment_t old_fs = get_fs();

	set_fs(KERNEL_DS);
	fd = sys_open(path[sic_mfts_mode], O_RDONLY, 0);
	if (fd >= 0) {
		sys_read(fd, line, sizeof(line));
		sys_close(fd);
		TOUCH_I("%s file existing\n", path[sic_mfts_mode]);
		ret = 1;
	}
	set_fs(old_fs);

	return ret;
}

static int spec_file_read(struct spi_device *spi)
{
	int ret = 0;
	const struct firmware *fwlimit = NULL;
	struct sic_ts_data *ts =
			(struct sic_ts_data *)get_touch_handle(spi);
	const char *path[4] = { ts->pdata->panel_spec,
			ts->pdata->panel_spec_mfts_folder,
			ts->pdata->panel_spec_mfts_flat,
			ts->pdata->panel_spec_mfts_curved };

	if (ts->pdata->panel_spec == NULL
		|| ts->pdata->panel_spec_mfts_folder == NULL
		|| ts->pdata->panel_spec_mfts_flat == NULL
		|| ts->pdata->panel_spec_mfts_curved == NULL) {
		TOUCH_I("panel_spec_file name is null\n");
		ret = -1;
		goto exit;
	}

	if (request_firmware(&fwlimit, path[sic_mfts_mode],
		&spi->dev) < 0) {
		TOUCH_I("request ihex is failed in normal mode\n");
		ret = -1;
		goto exit;
	}

	if (fwlimit->data == NULL) {
		ret = -1;
		TOUCH_I("fwlimit->data is NULL\n");
		goto exit;
	}

	strlcpy(line, fwlimit->data, sizeof(line));

exit:
	if (fwlimit)
		release_firmware(fwlimit);

	return ret;
}

int sic_get_limit(unsigned char Tx, unsigned char Rx, struct spi_device *spi,
		char *breakpoint, u16 limit_data[][ROW_SIZE])
{
	int p = 0;
	int q = 0;
	int r = 0;
	int cipher = 1;
	int ret = 0;
	int rx_num = 0;
	int tx_num = 0;
	char *found;

	int file_exist = 0;

	if (breakpoint == NULL) {
		ret = -1;
		goto exit;
	}

	if (sic_mfts_mode > 3 || sic_mfts_mode < 0) {
		ret = -1;
		goto exit;
	}

	file_exist = sdcard_spec_file_read();

	if (!file_exist) {
		ret = spec_file_read(spi);
		if (ret == -1)
			goto exit;
	}

	if (line == NULL) {
		ret =  -1;
		goto exit;
	}

	found = strnstr(line, breakpoint, sizeof(line));
	if (found != NULL) {
		q = found - line;
	} else {
		TOUCH_I(
			"failed to find breakpoint. The panel_spec_file is wrong");
		ret = -1;
		goto exit;
	}

	memset(limit_data, 0, (ROW_SIZE * COL_SIZE) * 2);

	while (1) {
		if (line[q] == ',') {
			cipher = 1;
			for (p = 1; (line[q - p] >= '0') &&
					(line[q - p] <= '9'); p++) {
				limit_data[tx_num][rx_num] +=
					((line[q - p] - '0') * cipher);
				cipher *= 10;
			}
			if (line[q - p] == '-') {
				limit_data[tx_num][rx_num] = (-1) *
					(limit_data[tx_num][rx_num]);
			}
			r++;

			if (r % (int)Rx == 0) {
				rx_num = 0;
				tx_num++;
			} else {
				rx_num++;
			}
		}
		q++;

		if (r == (int)Tx * (int)Rx) {
			TOUCH_I(
				"panel_spec_file scanning is success, breakpoint = %s value = %d\n",
				breakpoint, limit_data[0][0]);
			break;
		}
	}

exit:
	return ret;
}

int prd_compare_rawdata(struct spi_device *spi,
			char *buf, int is_prod_test)
{
	int i, j;
	int ret = 0;
	int result = 0; /*pass : 0 fail : 1*/
#if 0
	int ret2 = 0;
#endif
	int buffer_length = BUFFER_SIZE;
	int min = 9999;
	int max = 0;

#if 0
	/* spec reading */
	int lower_ret = 0;
	int upper_ret = 0;

	lower_ret = sic_get_limit(COL_SIZE,
				ROW_SIZE,
				spi,
				"LowerImageLimit",
				LowerLimit);
	upper_ret = sic_get_limit(COL_SIZE,
				ROW_SIZE,
				spi,
				"UpperImageLimit",
				UpperLimit);

	if (lower_ret < 0 || upper_ret < 0) {
		TOUCH_I(
			"[%s][FAIL] Can not check the limit of raw cap, lower return = %d upper return = %d\n",
			__func__, lower_ret, upper_ret);
		result = 1;
		return result;
	}
#endif
	if (is_prod_test == NORMAL_MODE)
		buffer_length = PAGE_SIZE;

	/* print a frame data */
	ret = snprintf(buf, buffer_length, "\n   : ");

	for (i = 0; i < ROW_SIZE; i++)
		ret += snprintf(buf + ret,
				buffer_length - ret,
				" [%2d] ", i);

	for (i = 0; i < COL_SIZE; i++) {
		ret += snprintf(buf + ret,
				buffer_length - ret,  "\n[%2d] ", i);
		for (j = 0; j < ROW_SIZE; j++) {
			ret += snprintf(buf + ret,
					buffer_length - ret, "%5d ",
					sicImage[i][j]);
			if (sicImage[i][j] != 0 && sicImage[i][j] < min)
				min = sicImage[i][j];
			if (sicImage[i][j] > max)
				max = sicImage[i][j];
		}
	}

	ret += snprintf(buf + ret,
		buffer_length - ret, "\n");

	/* production test : file write */
	if (is_prod_test == PRODUCTION_MODE) {
		ret += snprintf(buf + ret, buffer_length - ret, "\n[RAWDATA] min : %5d , max : %5d\n", min, max);
		write_file(NULL, buf, 0, 1);
		memset(buf, 0, sizeof(Write_Buffer));
		ret = result;
	}
#if 0
	/* compare result(pass : 0 fail : 1) */
	for (i = 0; i < COL_SIZE; i++) {
		for (j = 0; j < ROW_SIZE; j++) {
			if ((sicImage[i][j] < LowerLimit[i][j]) ||
				(sicImage[i][j] > UpperLimit[i][j])) {
				result = 1;

				if (is_prod_test == PRODUCTION_MODE)
					ret2 += snprintf(buf + ret2,
							buffer_length - ret2,
							"F %d,%d,%d\n",
							i, j, sicImage[i][j]);
				else
					TOUCH_I("F %d,%d,%d\n",
							i, j, sicImage[i][j]);
			}
		}
	}

	/* production test only */
	if (is_prod_test == PRODUCTION_MODE) {
		ret2 += snprintf(buf + ret2,
				buffer_length - ret2,
				"\nRawdata min : %d , max : %d\n", min, max);
		write_file(NULL, buf, 0, 1);
		ret = result;
	} else {
		TOUCH_I("Rawdata min : %d , max : %d\n", min, max);
		TOUCH_I("[Rawdata Test] - %s\n\n", result ? "Fail" : "Pass");
	}
#endif
	return ret;
}

int prd_frame_read(struct spi_device *spi, u8 type)
{
	struct sic_ts_data *ts =
			(struct sic_ts_data *)get_touch_handle(spi);

	int __m1_frame_size = COL_SIZE*2;
	int __m2_frame_size = ROW_SIZE*COL_SIZE*RAWDATA_SIZE;
//	int wdata;
//	u16 readbuf[2][COL_SIZE];
	memset(sicImage, 0, sizeof(sicImage));

	if (__m1_frame_size % 4)
		__m1_frame_size = (((__m1_frame_size >> 2) + 1) << 2);
	if (__m2_frame_size % 4)
		__m2_frame_size = (((__m2_frame_size >> 2) + 1) << 2);
	switch (type) {
	case DOZE1_M1_RAWDATA_TEST:
	case DOZE2_M1_RAWDATA_TEST:
		DO_SAFE(sic_spi_read(ts->spi_device,
				tc_tsp_test_m1_raw_data,
				(u8 *)&sicImage,
				__m1_frame_size), error);
		/*
		wdata = 0;
		DO_SAFE(sic_spi_write(ts->spi_device,
				tc_mem_sel,(u8 *)&wdata, sizeof(u32)), error);
		if(DO_SAFE(sic_spi_read(ts->spi_device,
					tc_tsp_test_m1_raw_data,
					(u8 *)&readbuf[0][0], COL_SIZE*2), error) != COL_SIZE*2) {
			return -ERROR;
		}
		wdata = 1;
		DO_SAFE(sic_spi_write(ts->spi_device,
				tc_mem_sel,(u8 *)&wdata, sizeof(u32)), error);
		if(DO_SAFE(sic_spi_read(ts->spi_device,
					tc_tsp_test_m1_raw_data,
					(u8 *)&readbuf[1][0], COL_SIZE*2), error) != COL_SIZE*2) {
			return -ERROR;
		}
		*/
		break;

	case DOZE1_M2_RAWDATA_TEST:
	case DOZE2_M2_RAWDATA_TEST:
		DO_SAFE(sic_spi_read(ts->spi_device,
				tc_tsp_test_m2_raw_data,
				(u8 *)&sicImage,
				__m2_frame_size), error);
		break;
	}
	return NO_ERROR;
error:
	TOUCH_E("i2c fail\n");
	return -ERROR;
}

void prd_pass_fail_result_print(u8 type, u32 result)
{
	int ret = 0;
	unsigned char buffer[LOG_BUF_SIZE] = {0,};

	switch (type) {
		case OPEN_SHORT_ALL_TEST:
			TOUCH_I("[Open Short All Test] - %s (%d/%d)\n",
					(result == 0) ?	"Pass" : "Fail",
					((result & 0x1) == 0x1) ? 0 : 1,
					((result & 0x2) == 0x2) ? 0 : 1);
			ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
					"Open Short All Test : %s (%d/%d)\n",
					(result == 0) ?	"Pass" : "Fail",
					((result & 0x1) == 0x1) ? 0 : 1,
					((result & 0x2) == 0x2) ? 0 : 1);
			break;

		case DOZE1_M1_RAWDATA_TEST:
		case DOZE2_M1_RAWDATA_TEST:
			TOUCH_I("[M1 Rawdata Test] - %s\n\n", result ? "Fail" : "Pass");
			ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
					"M1 Rawdata Test %s\n\n",
					result ? "Fail" : "Pass");
			break;

		case DOZE1_M2_RAWDATA_TEST:
		case DOZE2_M2_RAWDATA_TEST:
			TOUCH_I("[M2 Rawdata Test] - %s\n\n", result ? "Fail" : "Pass");
			ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
					"M2 Rawdata Test %s\n\n",
					result ? "Fail" : "Pass");
	}

	write_file(NULL, buffer, 0, 1);
}

int write_test_mode(struct spi_device *spi, u8 type)
{
	struct sic_ts_data *ts =
			(struct sic_ts_data *)get_touch_handle(spi);
	u32 testmode = 0;
	u8 doze_mode = 0x1;
	int retry = 20;
	u32 rdata = 0x01;
	int waiting_time = 10;

	switch (type) {
		case OPEN_NODE_TEST:
			testmode = (doze_mode << 8) + type;
			waiting_time = 300;
			break;

		case SHORT_NODE_TEST:
			testmode = (doze_mode << 8) + type;
			waiting_time = 100;
			break;

		case DOZE1_M1_RAWDATA_TEST:
		case DOZE1_M2_RAWDATA_TEST:
			testmode = (doze_mode << 8) + type;
			break;

		case DOZE2_M1_RAWDATA_TEST:
			type = 0x6;
			testmode = (doze_mode << 9) + type;
			break;

		case DOZE2_M2_RAWDATA_TEST:
			type = 0x5;
			testmode = (doze_mode << 9) + type;
			break;
	}

	/* TestType Set */
	DO_SAFE(sic_spi_write(ts->spi_device,
			tc_tsp_test_ctl,
			(u8 *)&testmode, sizeof(testmode)), error);
	TOUCH_I("i2c_write, testmode = %x\n", testmode);

	/* Check Test Result - wait until 0 is written */
	do {
		msleep(waiting_time);
		DO_SAFE(sic_spi_read(ts->spi_device,
				tc_tsp_test_sts,
				(u8 *)&rdata, sizeof(rdata)), error);
		TOUCH_I("rdata = 0x%x\n", rdata);
	} while ((rdata != 0xAA) && retry--);

	if (rdata != 0xAA) {
		TOUCH_I("ProductionTest Type [%d] Time out\n", type);
		return -ERROR;
	}
	return NO_ERROR;
error:
	TOUCH_E("[%s] i2c fail\n", __func__);
	return -ERROR;
}

int open_short_all_test(struct spi_device *spi) {
	int type = 0;
	int ret = 0;
	int ret2 = 0;
	u32 open_result = 0;
	u32 short_result = 0;
	u32 openshort_all_result = 0;
	u8 result_buffer[COL_SIZE][ROW_SIZE];
	int i = 0;
	int j = 0;

	memset(&result_buffer, 0x0, sizeof(result_buffer));

	/* 1. open_test */
	type = OPEN_NODE_TEST;
	ret2 = write_test_mode(spi, type);
	if (ret2 < 0) {
		TOUCH_E("write_test_mode fail\n");
		return 0x3;
	}

	sic_spi_read(spi, tc_tsp_test_pf_result,
		(u8 *)&open_result, sizeof(open_result));
	TOUCH_I("open_result = %d\n", open_result);

	if (open_result) {
		ret2 = prd_os_xline_result_read(spi, result_buffer, type);	// ret2 == -value is fail to read data
		openshort_all_result |= 0x1;
	}


	/* 2.short_test */
	type = SHORT_NODE_TEST;
	ret2 = write_test_mode(spi, type);
	if (ret2 < 0) {
		TOUCH_E("write_test_mode fail\n");
		return 0x3;
	}

	sic_spi_read(spi, tc_tsp_test_pf_result,
		(u8 *)&short_result, sizeof(short_result));
	TOUCH_I("short_result = %d\n", short_result);

	if (short_result) {
		ret2 = prd_os_xline_result_read(spi, result_buffer, type);
		openshort_all_result |= 0x2;
	}

	// fail case
	if (openshort_all_result != 0) {
		ret = snprintf(Write_Buffer, BUFFER_SIZE, "\n   : ");
		for (i = 0; i < ROW_SIZE; i++)
			ret += snprintf(Write_Buffer + ret,
				BUFFER_SIZE - ret, " [%2d] ", i);

		for (i = 0; i < COL_SIZE; i++) {
			ret += snprintf(Write_Buffer + ret,
			BUFFER_SIZE - ret,  "\n[%2d] ", i);

			for (j = 0; j < ROW_SIZE; j++) {
				ret += snprintf(Write_Buffer + ret, BUFFER_SIZE - ret,
					"%5s ",
					((result_buffer[i][j] & 0x3) == 0x3) ?  "O,S" :
					((result_buffer[i][j] & 0x1) == 0x1) ?  "O" :
					((result_buffer[i][j] & 0x2) == 0x2) ?  "S" : "-");
			}
		}
		ret += snprintf(Write_Buffer + ret, BUFFER_SIZE - ret, "\n");
	}

	write_file(NULL, Write_Buffer, 0, 1);

	return openshort_all_result;
}

void tune_display(char *tc_tune_code, int offset, int type)
{
	char log_buf[tc_tune_code_size] = {0,};
	int ret = 0;
	int i;
	char temp[tc_tune_code_size] = {0,};
	switch(type) {
		case 1:
			ret = snprintf(log_buf, tc_tune_code_size,
					"GOFT tune_code_read : ");
			if((tc_tune_code[offset] >> 4) == 1) {
				temp[offset] = tc_tune_code[offset] - (0x1 << 4);
				ret += snprintf(log_buf + ret, tc_tune_code_size - ret,
						"-%d  ",
						temp[offset]);
			}
			else {
				ret += snprintf(log_buf + ret, tc_tune_code_size - ret,
						" %d  ",
						tc_tune_code[offset]);
			}
			TOUCH_I("%s\n", log_buf);
			ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
			write_file(NULL, log_buf, 0, 1);
			break;
		case 2:
			ret = snprintf(log_buf, tc_tune_code_size,
					"LOFT tune_code_read : ");
			for(i = 0; i < tc_total_ch_size; i++) {
				if((tc_tune_code[offset+i]) >> 5 == 1) {
					temp[offset+i] = tc_tune_code[offset+i] - (0x1 << 5);
					ret += snprintf(log_buf + ret, tc_tune_code_size - ret,
							"-%d  ",
							temp[offset+i]);
				}
				else {
					ret += snprintf(log_buf + ret, tc_tune_code_size - ret,
							" %d  ",
							tc_tune_code[offset+i]);
				}
			}
			TOUCH_I("%s\n", log_buf);
			ret += snprintf(log_buf + ret, tc_tune_code_size - ret, "\n");
			write_file(NULL, log_buf, 0, 1);
			break;
	}
}


void read_tune_code(struct sic_ts_data *ts, u8 type)
{
	u8 tune_code_read_buf[276] = {0,};
	sic_spi_read(ts->spi_device, tc_tune_code_base,
				(u8 *)&tune_code_read_buf[0], tc_tune_code_size);

	switch(type) {
		case DOZE1_M1_RAWDATA_TEST:
		case DOZE2_M1_RAWDATA_TEST:
			write_file(NULL, "[Read Tune Code]\n", 0, 1);
			tune_display(tune_code_read_buf, TSP_TUNE_CODE_L_GOFT_OFFSET, 1);
			tune_display(tune_code_read_buf, TSP_TUNE_CODE_R_GOFT_OFFSET, 1);
			tune_display(tune_code_read_buf, TSP_TUNE_CODE_L_M1_OFT_OFFSET, 2);
			tune_display(tune_code_read_buf, TSP_TUNE_CODE_R_M1_OFT_OFFSET, 2);
			break;

		case DOZE1_M2_RAWDATA_TEST:
		case DOZE2_M2_RAWDATA_TEST:
			write_file(NULL, "[Read Tune Code]\n", 0, 1);
			tune_display(tune_code_read_buf, TSP_TUNE_CODE_L_GOFT_OFFSET+1, 1);
			tune_display(tune_code_read_buf, TSP_TUNE_CODE_R_GOFT_OFFSET+1, 1);
			tune_display(tune_code_read_buf, TSP_TUNE_CODE_L_G1_OFT_OFFSET, 2);
			tune_display(tune_code_read_buf, TSP_TUNE_CODE_L_G2_OFT_OFFSET, 2);
			tune_display(tune_code_read_buf, TSP_TUNE_CODE_L_G3_OFT_OFFSET, 2);
			tune_display(tune_code_read_buf, TSP_TUNE_CODE_R_G1_OFT_OFFSET, 2);
			tune_display(tune_code_read_buf, TSP_TUNE_CODE_R_G2_OFT_OFFSET, 2);
			tune_display(tune_code_read_buf, TSP_TUNE_CODE_R_G3_OFT_OFFSET, 2);
			break;
		}

}
int production_test(struct spi_device *spi, u8 type)
{
	u32 result = 1;
	int ret;
	struct sic_ts_data *ts =
			(struct sic_ts_data *)get_touch_handle(spi);

	memset(Write_Buffer, 0, BUFFER_SIZE);

	/* Read Test Result : pass(0),fail(1) */
	switch (type) {
		case OPEN_SHORT_ALL_TEST:
			result = open_short_all_test(spi);
			break;

		case DOZE1_M1_RAWDATA_TEST:
		case DOZE2_M1_RAWDATA_TEST:
			/*
			ret = write_test_mode(spi, type);
			if (ret < 0) {
				TOUCH_E("production test couldn't be done\n");
				goto error;
			}
			DO_SAFE(prd_frame_read(spi, type), error);
			result = prd_compare_rawdata(spi, Write_Buffer,
					PRODUCTION_MODE);
			break;
			*/
		case DOZE1_M2_RAWDATA_TEST:
		case DOZE2_M2_RAWDATA_TEST:
			ret = write_test_mode(spi, type);
			if (ret < 0) {
				TOUCH_E("production test couldn't be done\n");
				goto error;
			}
			DO_SAFE(prd_frame_read(spi, type), error);
			result = prd_compare_rawdata(spi, Write_Buffer,
					PRODUCTION_MODE);
			break;
	}

error:
	prd_pass_fail_result_print(type, result);
	read_tune_code(ts, type);
	return result;
}

void firmware_version_log(struct sic_ts_data *ts)
{
	int ret = 0;
	unsigned char buffer[LOG_BUF_SIZE] = {0,};

	if (sic_mfts_mode)
		ret = get_ic_info(ts);

	ret = snprintf(buffer, LOG_BUF_SIZE,
		"======== Firmware Info ========\n");
	ret += snprintf(buffer+ret, LOG_BUF_SIZE - ret,
		"IC_fw_version : v%d.%02d\n",
				ts->fw_info.fw_version[0],
				ts->fw_info.fw_version[1]);
	ret += snprintf(buffer+ret, LOG_BUF_SIZE - ret,
		"IC_product_id[%s]\n\n",
				ts->fw_info.fw_product_id);

	write_file(NULL, buffer, 0, 1);
}

void sic_ts_init_prod_test(struct sic_ts_data *ts)
{
	touch_disable_irq(ts->spi_device->irq);
	return;
}

void sic_ts_exit_prod_test(struct sic_ts_data *ts)
{
	sic_ts_power(ts->spi_device, POWER_OFF);
	sic_ts_power(ts->spi_device, POWER_ON);
	msleep(ts->pdata->role->booting_delay);
	DO_SAFE(sic_ts_init(ts->spi_device), error);
	atomic_set(&ts->state->device_init, INIT_DONE);
	touch_enable_irq(ts->spi_device->irq);
	return;
error:
	TOUCH_E("%s\n", __func__);
	return;
}

static ssize_t show_sd(struct spi_device *spi, char *buf)
{
	struct sic_ts_data *ts =
			(struct sic_ts_data *)get_touch_handle(spi);

	int openshort_ret = 0;
	int m2_rawdata_ret = 0;
	int ret = 0;
	u32 trim_result = 0;
	u32 wdata;
	int result = 0;
	/* file create , time log */
	write_file(NULL, "", 1, 1);

	// LCD off
	if (atomic_read(&ts->lpwg_ctrl.is_suspend)) {
		ret = snprintf(buf + ret,
				PAGE_SIZE - ret,
				"LCD Off. Test Result: Fail\n");
		return ret;
	}

	/* firmware version log */
	firmware_version_log(ts);
	mutex_lock(&ts->pdata->thread_lock);
	sic_ts_init_prod_test(ts);

	/* OSC Trimming */
	wdata = 0xEAEA0800;
	sic_spi_write(spi, tc_device_ctl,
			(u8 *)&wdata, sizeof(u32));
	result = sic_spi_read(spi, 0xD6A9, (u8 *)&trim_result, sizeof(trim_result));
	if (result > 10) {
		TOUCH_I("OSC Trimming fail. trim_result = %d\n", result);
		return ret;
	}

	write_file(NULL, "[M2_RAWDATA_TEST]", 0, 1);
	m2_rawdata_ret = production_test(spi, DOZE1_M2_RAWDATA_TEST);
	write_file(NULL, "\n[OPEN_SHORT_ALL_TEST]\n", 0, 1);
	openshort_ret = production_test(spi, OPEN_SHORT_ALL_TEST);
	ret = snprintf(buf,PAGE_SIZE,"========RESULT=======\n");
	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"M2 Raw Data : %s\n", m2_rawdata_ret ? "Fail" : "Pass");
	if (openshort_ret == 0) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Channel Status : %s\n", "Pass");
	}
	else {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Channel Status : %s (%d/%d)\n", "Fail",
				((openshort_ret & 0x1) == 0x1) ? 0 : 1,
				((openshort_ret & 0x2) == 0x2) ? 0 : 1);
	}
	sic_ts_exit_prod_test(ts);
	mutex_unlock(&ts->pdata->thread_lock);

	write_file(NULL, "", 1, 1);

	return ret;
}


static ssize_t show_lpwg_sd(struct spi_device *spi, char *buf)
{
	struct sic_ts_data *ts =
			(struct sic_ts_data *)get_touch_handle(spi);

	int m1_rawdata_ret = 0;
	int m2_rawdata_ret = 0;
	int ret = 0;

	/* file create , time log */
	write_file(NULL, "", 1, 1);

	mutex_lock(&ts->pdata->thread_lock);
	sic_ts_init_prod_test(ts);

	write_file(NULL, "[M2_RAWDATA_TEST]\n", 0, 1);
	m2_rawdata_ret = production_test(spi, DOZE2_M2_RAWDATA_TEST);
	write_file(NULL, "[M1_RAWDATA_TEST]\n", 0, 1);
	m1_rawdata_ret = production_test(spi, DOZE2_M1_RAWDATA_TEST);

	ret = snprintf(buf,
		PAGE_SIZE,
		"========RESULT=======\n");

	ret += snprintf(buf + ret,
			PAGE_SIZE - ret,
			"M1 RawData : %s\n",
			m1_rawdata_ret ? "FAIL" : "PASS");

	ret += snprintf(buf + ret,
			PAGE_SIZE - ret,
			"M2 RawData : %s\n",
			m2_rawdata_ret ? "FAIL" : "PASS");


	sic_ts_exit_prod_test(ts);
	mutex_unlock(&ts->pdata->thread_lock);

	write_file(NULL, "", 1, 1);

	return ret;
}

static ssize_t show_fdata(struct spi_device *spi, char *buf)
{
	struct sic_ts_data *ts =
			(struct sic_ts_data *)get_touch_handle(spi);
	int ret = 0;
	int ret2 = 0;
	u8 type = DOZE1_M2_RAWDATA_TEST;

	if (atomic_read(&ts->lpwg_ctrl.is_suspend)) {
		ret = snprintf(buf + ret,
				PAGE_SIZE - ret,
				"LCD Off. Test Result: Fail\n");
		return ret;
	}
#if 0
	production_test_offset(NORMAL_MODE);
#endif
	mutex_lock(&ts->pdata->thread_lock);
	sic_ts_init_prod_test(ts);

	ret2 = write_test_mode(spi, type);
	if (ret2 < 0) {
		TOUCH_E("write_test_mode fail\n");
		sic_ts_exit_prod_test(ts);
		mutex_unlock(&ts->pdata->thread_lock);
		return ret;
	}

	prd_frame_read(spi, type);
	ret = prd_compare_rawdata(spi, buf, NORMAL_MODE);

	sic_ts_exit_prod_test(ts);
	mutex_unlock(&ts->pdata->thread_lock);
	return ret;
}

static ssize_t store_rawdata(struct spi_device *spi,
				const char *buf, size_t count)
{
	struct sic_ts_data *ts =
			(struct sic_ts_data *)get_touch_handle(spi);
	int ret = 0;
	int ret2 = 0;
	int value;
	int i;
	int j;
	int16_t *rawdata = NULL;
	char data_path[PATH_SIZE] = {0,};
	char *read_buf;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if (atomic_read(&ts->lpwg_ctrl.is_suspend)) {
		TOUCH_E("LCD Off. Test Result: Fail\n");
		return ret;
	}

	read_buf = kzalloc(sizeof(u8)*PAGE_SIZE, GFP_KERNEL);
	rawdata = kzalloc(sizeof(int16_t) * (COL_SIZE*ROW_SIZE), GFP_KERNEL);

	if (read_buf == NULL) {
		TOUCH_E("read_buf mem_error\n");
		return count;
	}
	if (rawdata == NULL) {
		TOUCH_E("rawdata mem_error\n");
		return count;
	}


	snprintf(data_path, PATH_SIZE, "/sdcard/%d.csv", value);
	TOUCH_I("data_path : %s\n", data_path);

	ret2 = get_data(ts, rawdata, 1);

	if (ret2 < 0) {
		goto error;
	}

	for (i = 0; i < COL_SIZE; i++) {
		for (j = 0; j < ROW_SIZE; j++) {
			ret += snprintf(read_buf + ret, PAGE_SIZE - ret,
					"%5d,", rawdata[i*ROW_SIZE+j]);
		}
	}
	write_file(data_path, read_buf, 0, 0);

error:
	if (read_buf != NULL)
		kfree(read_buf);
	if (rawdata != NULL)
		kfree(rawdata);

	return count;
}

static ssize_t show_rawdata(struct spi_device *spi, char *buf)
{
	struct sic_ts_data *ts =
			(struct sic_ts_data *)get_touch_handle(spi);
	int ret = 0;
	int ret2 = 0;
	int16_t *rawdata = NULL;
	int i = 0;
	int j = 0;

	rawdata = kzalloc(sizeof(int16_t) * (COL_SIZE*ROW_SIZE), GFP_KERNEL);

	if (rawdata == NULL) {
		TOUCH_E("mem_error\n");
		return ret;
	}

	ret = snprintf(buf, PAGE_SIZE, "======== rawdata ========\n");

	ret2 = get_data(ts, rawdata, 1);  /* 2 == deltadata */
	if (ret2 < 0) {
		TOUCH_E("Test fail (Check if LCD is OFF)\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Test fail (Check if LCD is OFF)\n");
		goto error;
	}

	for (i = 0 ; i < COL_SIZE ; i++) {
		char log_buf[LOG_BUF_SIZE] = {0,};
		int log_ret = 0;

		ret += snprintf(buf + ret, PAGE_SIZE - ret, "[%2d] ", i);
		log_ret += snprintf(log_buf + log_ret,
					LOG_BUF_SIZE - log_ret, "[%2d]  ", i);

		for (j = 0 ; j < ROW_SIZE ; j++) {
			ret += snprintf(buf + ret, PAGE_SIZE - ret,
					"%5d ", rawdata[i*ROW_SIZE+j]);
			log_ret += snprintf(log_buf + log_ret,
						LOG_BUF_SIZE - log_ret, "%5d ",
						rawdata[i*ROW_SIZE+j]);
		}
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");
		TOUCH_I("%s\n", log_buf);
	}

error:
	if (rawdata != NULL)
		kfree(rawdata);

	return ret;
}

static ssize_t show_delta(struct spi_device *spi, char *buf)
{
	struct sic_ts_data *ts =
			(struct sic_ts_data *)get_touch_handle(spi);
	int ret = 0;
	int ret2 = 0;
	int16_t *delta = NULL;
	int i = 0;
	int j = 0;

	delta = kzalloc(sizeof(int16_t) * (COL_SIZE*ROW_SIZE), GFP_KERNEL);

	if (delta == NULL) {
		TOUCH_E("delta mem_error\n");
		return ret;
	}

	ret = snprintf(buf, PAGE_SIZE, "======== Deltadata ========\n");

	ret2 = get_data(ts, delta, 2);  /* 2 == deltadata */
	if (ret2 < 0) {
		TOUCH_E("Test fail (Check if LCD is OFF)\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Test fail (Check if LCD is OFF)\n");
		goto error;
	}

	for (i = 0 ; i < COL_SIZE ; i++) {
		char log_buf[LOG_BUF_SIZE] = {0,};
		int log_ret = 0;

		ret += snprintf(buf + ret, PAGE_SIZE - ret, "[%2d] ", i);
		log_ret += snprintf(log_buf + log_ret,
					LOG_BUF_SIZE - log_ret, "[%2d]  ", i);

		for (j = 0 ; j < ROW_SIZE ; j++) {
			ret += snprintf(buf + ret, PAGE_SIZE - ret,
					"%5d ", delta[i*ROW_SIZE+j]);
			log_ret += snprintf(log_buf + log_ret,
						LOG_BUF_SIZE - log_ret, "%5d ",
						delta[i*ROW_SIZE+j]);
		}
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");
		TOUCH_I("%s\n", log_buf);
	}

error:
	if (delta != NULL)
		kfree(delta);

	return ret;
}

static ssize_t show_baseline_data(struct spi_device *spi, char *buf)
{
	struct sic_ts_data *ts =
			(struct sic_ts_data *)get_touch_handle(spi);
	int ret = 0;
	int ret2 = 0;
	int16_t *rawdata = NULL;
	int i = 0;
	int j = 0;
	u8 flat = 0;

	rawdata = kzalloc(sizeof(int16_t) * (COL_SIZE*ROW_SIZE), GFP_KERNEL);

	if (rawdata == NULL) {
		TOUCH_E("mem_error\n");
		return ret;
	}

	ret = snprintf(buf, PAGE_SIZE, "======== baseline_data ========\n");

	ret2 = get_baseline_data(ts, rawdata, 1);  /* 1 == baseline_data */
	if (ret2 < 0) {
		TOUCH_E("Test fail (Check if LCD is OFF)\n");
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Test fail (Check if LCD is OFF)\n");
		goto error;
	}

	for (i = 0 ; i < COL_SIZE ; i++) {
		char log_buf[LOG_BUF_SIZE] = {0,};
		int log_ret = 0;

		ret += snprintf(buf + ret, PAGE_SIZE - ret, "[%2d] ", i);
		log_ret += snprintf(log_buf + log_ret,
					LOG_BUF_SIZE - log_ret, "[%2d]  ", i);

		for (j = 0 ; j < ROW_SIZE; j++) {
			flat = ROW_SIZE-1-j;	/* the data be flatted */
			ret += snprintf(buf + ret, PAGE_SIZE - ret,
					"%5d ", rawdata[i*ROW_SIZE+flat]);
			log_ret += snprintf(log_buf + log_ret,
						LOG_BUF_SIZE - log_ret, "%5d ",
						rawdata[i*ROW_SIZE+flat]);
		}
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");
		TOUCH_I("%s\n", log_buf);
	}

error:
	if (rawdata != NULL)
		kfree(rawdata);

	return ret;
}

static ssize_t show_open_short(struct spi_device *spi, char *buf)
{
	struct sic_ts_data *ts =
			(struct sic_ts_data *)get_touch_handle(spi);
	int ret = 0;
	int ret2 = 0;
	u8 type = OPEN_SHORT_ALL_TEST;
	// if result is 0, test result pass
	u32 open_result = 0;
	u32 short_result = 0;
	u32 openshort_all_result = 0;
	u8 result_buffer[COL_SIZE][ROW_SIZE];
	int i = 0;
	int j = 0;

	memset(&result_buffer, 0x0, sizeof(result_buffer));
	if (atomic_read(&ts->lpwg_ctrl.is_suspend)) {
		ret = snprintf(buf + ret,
				PAGE_SIZE - ret,
				"state=[suspend]. we cannot use I2C, now. Test Result: Fail\n");
		return ret;
	}

	mutex_lock(&ts->pdata->thread_lock);
	sic_ts_init_prod_test(ts);

	// 1. open_test
	type = OPEN_NODE_TEST;
	ret2 = write_test_mode(spi, type);
	if (ret2 < 0) {
		TOUCH_E("write_test_mode fail\n");
		sic_ts_exit_prod_test(ts);
		mutex_unlock(&ts->pdata->thread_lock);
		return ret;
	}

	DO_SAFE(sic_spi_read(ts->spi_device,
		tc_tsp_test_pf_result,
		(u8 *)&open_result, sizeof(open_result)), error);
	TOUCH_I("open_result = %d\n", open_result);

	/* if fail, show f/p result of each node */
	if (open_result) {
		ret2 = prd_os_xline_result_read(spi, result_buffer, type);	// ret2 == -value is fail to read data
		openshort_all_result |= 0x1;
	}

	// 2.short_test
	type = SHORT_NODE_TEST;
	ret2 = write_test_mode(spi, type);
	if (ret2 < 0) {
		TOUCH_E("write_test_mode fail\n");
		sic_ts_exit_prod_test(ts);
		mutex_unlock(&ts->pdata->thread_lock);
		return ret2;
	}

	DO_SAFE(sic_spi_read(ts->spi_device,
		tc_tsp_test_pf_result,
		(u8 *)&short_result, sizeof(short_result)), error);
	TOUCH_I("short_result = %d\n", short_result);

	/* if fail, show f/p result of each node */
	if (short_result) {
		ret2 = prd_os_xline_result_read(spi, result_buffer, type);
		openshort_all_result |= 0x2;
	}

	// case
	if ( openshort_all_result != 0 ) {
		ret = snprintf(buf, PAGE_SIZE, "\n   : ");
		for (i = 0; i < ROW_SIZE; i++)
			ret += snprintf(buf + ret,
				PAGE_SIZE - ret, " [%2d] ", i);

		for (i = 0; i < COL_SIZE; i++) {
			ret += snprintf(buf + ret,
			PAGE_SIZE - ret,  "\n[%2d] ", i);

			for (j = 0; j < ROW_SIZE; j++) {
				ret += snprintf(buf + ret, PAGE_SIZE - ret,
					"%5s ",
					((result_buffer[i][j] & 0x3) == 0x3) ?  "O,S" :
					((result_buffer[i][j] & 0x1) == 0x1) ?  "O" :
					((result_buffer[i][j] & 0x2) == 0x2) ?  "S" : "-");
			}
		}
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");
	}
	ret += snprintf(buf + ret, PAGE_SIZE - ret,
		"[Open Short All Test] - %s (%d/%d)\n",
		(openshort_all_result == 0) ? "Pass" : "Fail",
		(open_result != 0) ? 0 : 1,
		(short_result != 0) ? 0 : 1);
	TOUCH_I("[Open Short All Test] - %s (%d/%d)\n",
		(openshort_all_result == 0) ? "Pass" : "Fail",
		(open_result != 0) ? 0 : 1,
		(short_result != 0) ? 0 : 1);
error:
	sic_ts_exit_prod_test(ts);
	mutex_unlock(&ts->pdata->thread_lock);
	return ret;
}

static ssize_t show_mfts_mode(struct spi_device *spi, char *buf)
{
	int ret = 0;
	ret = snprintf(buf, PAGE_SIZE, "%d\n", sic_mfts_mode);
	return ret;
}

static ssize_t store_mfts_mode(struct spi_device *spi,
		const char *buf, size_t count)
{
	int value;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	sic_mfts_mode = value;
	TOUCH_I("mfts_mode:%d\n", sic_mfts_mode);

	return count;
}

static ssize_t show_debug_mode(struct spi_device *spi, char *buf)
{
	int ret = 0;
	struct sic_ts_data *ts =
			(struct sic_ts_data *)get_touch_handle(spi);

	ret = snprintf(buf + ret, PAGE_SIZE, "Debug_mode = %s\n",
			ts->debug_mode ? "Enable" : "Disable");
	TOUCH_I("Debug_mode = %s\n", ts->debug_mode ? "Enable" : "Disable");

	return ret;
}

static ssize_t store_debug_mode(struct spi_device *spi,
						const char *buf, size_t count)
{
	int value = 0;
	struct sic_ts_data *ts =
			(struct sic_ts_data *)get_touch_handle(spi);

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	ts->debug_mode = value;

	TOUCH_I("Debug_mode = %s\n", ts->debug_mode ? "Enable" : "Disable");
	return count;
}


int DeepPowerDown(struct sic_ts_data *ts);
int ReleaseDeepPowerDown(struct sic_ts_data *ts);
int spi_flash_read(struct sic_ts_data *ts, u32 addr, u8 *read_data_buf, int byte_cnt);

static ssize_t store_fw_dump(struct spi_device *spi,
						const char *buf, size_t count)
{
	struct sic_ts_data *ts =
		(struct sic_ts_data *) get_touch_handle(spi);
	u8 *pDump = NULL;
	u32 wdata;
	int fd = 0;
	int i;
	char *dump_path = "/sdcard/touch_dump.fw";
	mm_segment_t old_fs = get_fs();

	TOUCH_I("F/W Dumping...\n");

	pDump = kzalloc(SIZEOF_FLASH_IMAGE, GFP_KERNEL);

	if (pDump == NULL) {
		TOUCH_E("pDump mem_error\n");
		return count;
	}

	wdata = 2;
	DO_SAFE(sic_spi_write(ts->spi_device, SYS_RST_CTL, (u8 *)&wdata, sizeof(u32)), error);

	for (i = 0; i < SIZEOF_FLASH_IMAGE; i+=0x800) {
		DO_SAFE(sic_spi_read(ts->spi_device, i/4, &pDump[i], 0x800), error);
	}

	set_fs(KERNEL_DS);
	fd = sys_open(dump_path, O_WRONLY|O_CREAT, 0666);
	if (fd >= 0) {
		sys_write(fd, pDump, SIZEOF_FLASH_IMAGE);
		sys_close(fd);
	}

	set_fs(old_fs);

	if (pDump != NULL)
		kfree(pDump);

	sic_ts_power(ts->spi_device, POWER_OFF);
	sic_ts_power(ts->spi_device, POWER_ON);
	msleep(ts->pdata->role->booting_delay);
	DO_SAFE(sic_ts_init(ts->spi_device), error);
	atomic_set(&ts->state->device_init, INIT_DONE);
	touch_enable_irq(ts->spi_device->irq);
	TOUCH_I("%s:%d\n", __func__, __LINE__);
	return count;

error:
	if (pDump != NULL)
		kfree(pDump);
	return count;
}

static ssize_t show_spi_speed(struct spi_device *spi, char *buf)
{
	int ret = 0;

	ret = snprintf(buf, PAGE_SIZE, "Clock = %dMhz\n", (spi->max_speed_hz)/1000000);

	TOUCH_I("%s :  Clock[%d Mhz] \n", __func__, (spi->max_speed_hz)/1000000);

	return ret;
}

static ssize_t store_spi_speed(struct spi_device *spi,
						const char *buf, size_t count)
{
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	spi->max_speed_hz = value * 1000000;

	TOUCH_I("%s :  Clock[%d Mhz] \n", __func__, (spi->max_speed_hz)/1000000);

	return count;
}

static ssize_t store_deep_sleep(struct spi_device *spi,
						const char *buf, size_t count)
{
	int value = 0;
	struct sic_ts_data *ts =
		(struct sic_ts_data *) get_touch_handle(spi);

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if (atomic_read(&ts->lpwg_ctrl.is_suspend) != TC_STATUS_SUSPEND) {
		TOUCH_I("Can't go to deep_sleep mode\n");
		return count;
	}

	mutex_lock(&ts->pdata->thread_lock);
	tci_deep_sleep(spi, value);
	mutex_unlock(&ts->pdata->thread_lock);

	return count;
}

static ssize_t store_u3fake(struct spi_device *spi,
						const char *buf, size_t count)
{
	struct sic_ts_data *ts =
			(struct sic_ts_data *)get_touch_handle(spi);
	int is_u3fake = 0;

	if (sscanf(buf, "%d", &is_u3fake) <= 0)
		return count;

	TOUCH_I("%s : is_u3fake = %d\n", __func__, is_u3fake);

	ts->pdata->is_u3fake = is_u3fake;

	return count;
}

#if 0
static int offset_calibration(struct spi_device *spi, int is_lcd_on)
{
	struct sic_ts_data *ts =
		(struct sic_ts_data *) get_touch_handle(spi);
	u32 wdata = 0;
	u32 rdata = 0;
	u32 trim_addr[7] = {0x6015, 0x601D, 0x601E,
				0x6020, 0x6000, 0x6003, 0x6004};
	u32 trim_value[7] = {0, };
	u32 run_info = 0;
	u32 cal_status = 0;
	u32 right_offset = 0;
	u32 left_offset = 0;
	u16 reg_doze = 0;

	int i = 0;
	int result = 0;
	int retry_count = 3;

	/* 0. Disable irq */
	sic_ts_init_prod_test(ts);

	/* Doze2 Calibration need IC Reset before cal */
	if (is_lcd_on == 0) {
		sic_ts_power(ts->spi_device, POWER_OFF);
		sic_ts_power(ts->spi_device, POWER_ON);
		msleep(ts->pdata->role->booting_delay);
	}

	/* 1. Read flash info */
	TOUCH_I("System Hold for calibration\n");
	wdata = 2;
	DO_SAFE(sic_spi_write(ts->spi_device, spr_rst_ctl,
			(u8 *)&wdata, sizeof(u32)), error);

	/* trim value read */
	wdata = 0x1 << 3;	/* for READ */
	for (i = 0; i < 7; i++) {
		DO_SAFE(sic_spi_write(ts->spi_device, fc_addr,
				(u8 *)&trim_addr[i], sizeof(u32)), error);
		DO_SAFE(sic_spi_write(ts->spi_device, fc_ctl,
				(u8 *)&wdata, sizeof(u32)), error);
		DO_SAFE(sic_spi_read(ts->spi_device, fc_rdata,
				(u8 *)&trim_value[i], sizeof(u32)), error);
	}

	/* 2. erase flash */
	wdata = 0x6000;
	DO_SAFE(sic_spi_write(ts->spi_device, fc_addr,
			(u8 *)&wdata, sizeof(u32)), error);
	wdata = 0x1;	/* for Erase */
	DO_SAFE(sic_spi_write(ts->spi_device, fc_ctl,
			(u8 *)&wdata, sizeof(u32)), error);
	msleep(50);

	/* 3. write flash */
	/* trim value write */
	for (i = 0; i < 7; i++)	{
		DO_SAFE(sic_spi_write(ts->spi_device, fc_addr,
				(u8 *)&trim_addr[i], sizeof(u32)), error);
		DO_SAFE(sic_spi_write(ts->spi_device, fc_wdata,
				(u8 *)&trim_value[i], sizeof(u32)), error);
	}

	/* 4. reset */
	sic_ts_power(ts->spi_device, POWER_OFF);
	sic_ts_power(ts->spi_device, POWER_ON);
	msleep(ts->pdata->role->booting_delay);

	/* 5. read runinfo */
	DO_SAFE(sic_spi_read(ts->spi_device, 0xD01F,
			(u8 *)&rdata, sizeof(u32)), error);
	run_info = rdata & 0xFFFFFFFF;
	TOUCH_I("run_info = %d\n", run_info);

	/* 6. offset cal */
	if (is_lcd_on) {
		wdata = 1;
		reg_doze = 0xD01B;
	} else {
		wdata = 2;
		reg_doze = 0xD01C;
	}
	DO_SAFE(sic_spi_write(ts->spi_device, 0xD01A,
			(u8 *)&wdata, sizeof(u32)), error);
	do {
		msleep(2300);
		DO_SAFE(sic_spi_read(ts->spi_device, 0xD01A,
			(u8 *)&cal_status, sizeof(u32)), error);
	} while ((cal_status != 0) && (retry_count--));

	DO_SAFE(sic_spi_read(ts->spi_device, reg_doze,
			(u8 *)&rdata, sizeof(u32)), error);
	TOUCH_I("rdata = 0x%x\n", rdata);

	right_offset = rdata & 0xFFFF;
	left_offset = (rdata >> 16) & 0xFFFF;

	right_offset = ((right_offset & 0xFF) << 8) |
			((right_offset & 0xFF00) >> 8);
	left_offset = ((left_offset & 0xFF) << 8) |
			((left_offset & 0xFF00) >> 8);

	/* 7. runinfo & cal value write */
	TOUCH_I("System Hold for calibration\n");
	wdata = 2;
	DO_SAFE(sic_spi_write(ts->spi_device, spr_rst_ctl,
			(u8 *)&wdata, sizeof(u32)), error);

	/* doze2 left offset */
	wdata = 0x6024;
	DO_SAFE(sic_spi_write(ts->spi_device, fc_addr,
			(u8 *)&wdata, sizeof(u32)), error);
	wdata = is_lcd_on ? (doze2Offset[0] & 0xFFFF) : left_offset;
	DO_SAFE(sic_spi_write(ts->spi_device, fc_wdata,
			(u8 *)&wdata, sizeof(u32)), error);

	wdata = 0x6025;
	DO_SAFE(sic_spi_write(ts->spi_device, fc_addr,
			(u8 *)&wdata, sizeof(u32)), error);
	wdata = is_lcd_on ? (doze2Offset[1] & 0xFFFF) : right_offset;
	DO_SAFE(sic_spi_write(ts->spi_device, fc_wdata,
			(u8 *)&wdata, sizeof(u32)), error);
	/* doze1 left offset	 */
	wdata = 0x6026;
	DO_SAFE(sic_spi_write(ts->spi_device, fc_addr,
			(u8 *)&wdata, sizeof(u32)), error);
	wdata = is_lcd_on ? left_offset : (doze1Offset[0] & 0xFFFF);
	DO_SAFE(sic_spi_write(ts->spi_device, fc_wdata,
			(u8 *)&wdata, sizeof(u32)), error);

	wdata = 0x6027;
	DO_SAFE(sic_spi_write(ts->spi_device, fc_addr,
			(u8 *)&wdata, sizeof(u32)), error);
	wdata = is_lcd_on ? right_offset : (doze1Offset[1] & 0xFFFF);
	DO_SAFE(sic_spi_write(ts->spi_device, fc_wdata,
			(u8 *)&wdata, sizeof(u32)), error);

	wdata = 0x6028;
	DO_SAFE(sic_spi_write(ts->spi_device, fc_addr,
			(u8 *)&wdata, sizeof(u32)), error);
	wdata = run_info;
	DO_SAFE(sic_spi_write(ts->spi_device, fc_wdata,
			(u8 *)&wdata, sizeof(u32)), error);
	result = 1;

error:
	/* 99. reset */
	ts->is_probed = 0;
	sic_ts_exit_prod_test(ts);
	return result;
}

static ssize_t show_doze1_calibration(struct spi_device *spi, char *buf)
{
	int ret = 0;
	int result = 0;

	/* 1 is doze1 calibration */
	result = offset_calibration(spi, 1);

	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"Doze1 Calibration Result : %s\n",
			result ? "PASS" : "FAIL");

	return ret;
}

static ssize_t show_doze2_calibration(struct spi_device *spi, char *buf)
{
	int ret = 0;
	int result = 0;

	/* 0 is doze2 calibration */
	if (doze_state == DOZE2_STATUS || doze_state == DOZE2_DEBUG_STATUS)
		result = offset_calibration(spi, 0);

	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"Doze2 Calibration Result : %s\n",
			result ? "PASS" : "FAIL");

	return ret;
}
#endif

static LGE_TOUCH_ATTR(firmware, S_IRUGO | S_IWUSR, show_firmware, NULL);
static LGE_TOUCH_ATTR(testmode_ver, S_IRUGO | S_IWUSR, show_atcmd_fw_ver, NULL);
static LGE_TOUCH_ATTR(version, S_IRUGO | S_IWUSR,
		show_sic_fw_version, NULL);
static LGE_TOUCH_ATTR(tci, S_IRUGO | S_IWUSR, show_tci, store_tci);
static LGE_TOUCH_ATTR(reg_ctrl, S_IRUGO | S_IWUSR, NULL, store_reg_ctrl);
static LGE_TOUCH_ATTR(object_report, S_IRUGO | S_IWUSR,
			show_object_report, store_object_report);
static LGE_TOUCH_ATTR(rawdata, S_IRUGO | S_IWUSR,
	show_rawdata, store_rawdata);
static LGE_TOUCH_ATTR(baseline, S_IRUGO | S_IWUSR,
	show_baseline_data, NULL);
static LGE_TOUCH_ATTR(fdata, S_IRUGO | S_IWUSR, show_fdata, NULL);
static LGE_TOUCH_ATTR(open_short, S_IRUGO | S_IWUSR, show_open_short, NULL);
static LGE_TOUCH_ATTR(delta, S_IRUGO | S_IWUSR, show_delta, NULL);
static LGE_TOUCH_ATTR(debug_mode, S_IRUGO | S_IWUSR,
			show_debug_mode, store_debug_mode);
static LGE_TOUCH_ATTR(tci_fail_reason, S_IRUGO | S_IWUSR,
			show_tci_fail_reason, store_tci_fail_reason);
static LGE_TOUCH_ATTR(swipe_fail_reason, S_IRUGO | S_IWUSR,
			show_swipe_fail_reason, store_swipe_fail_reason);
static LGE_TOUCH_ATTR(swipe_param, S_IRUGO | S_IWUSR,
			show_swipe_param, store_swipe_param);
static LGE_TOUCH_ATTR(swipe_mode, S_IRUGO | S_IWUSR,
			show_swipe_mode, store_swipe_mode);
static LGE_TOUCH_ATTR(bootmode, S_IRUGO | S_IWUSR, NULL, store_boot_mode);
static LGE_TOUCH_ATTR(sd, S_IRUGO | S_IWUSR, show_sd, NULL);
static LGE_TOUCH_ATTR(mfts, S_IRUGO | S_IWUSR,
		show_mfts_mode, store_mfts_mode);
static LGE_TOUCH_ATTR(fw_dump, S_IRUSR | S_IWUSR, NULL, store_fw_dump);
static LGE_TOUCH_ATTR(spi_speed, S_IRUSR | S_IWUSR, show_spi_speed, store_spi_speed);
static LGE_TOUCH_ATTR(deep_sleep, S_IRUSR | S_IWUSR, NULL, store_deep_sleep);
static LGE_TOUCH_ATTR(u3fake, S_IRUSR | S_IWUSR, NULL, store_u3fake);
static LGE_TOUCH_ATTR(lpwg_sd, S_IRUGO | S_IWUSR, show_lpwg_sd, NULL);
#if 0
static LGE_TOUCH_ATTR(doze1_calibration, S_IRUSR | S_IWUSR,
		show_doze1_calibration, NULL);
static LGE_TOUCH_ATTR(doze2_calibration, S_IRUSR | S_IWUSR,
		show_doze2_calibration, NULL);
#endif

#if USE_ABT_MONITOR_APP
static LGE_TOUCH_ATTR(abt_monitor, S_IRUGO | S_IWUGO,
			show_abtApp, store_abtApp);
static LGE_TOUCH_ATTR(raw_report, S_IRUGO | S_IWUGO,
			show_abtTool, store_abtTool);
#endif


static struct attribute *sic_ts_attribute_list[] = {
	&lge_touch_attr_firmware.attr,
	&lge_touch_attr_tci.attr,
	&lge_touch_attr_reg_ctrl.attr,
	&lge_touch_attr_object_report.attr,
	&lge_touch_attr_testmode_ver.attr,
	&lge_touch_attr_version.attr,
	&lge_touch_attr_rawdata.attr,
	&lge_touch_attr_baseline.attr,
	&lge_touch_attr_fdata.attr,
	&lge_touch_attr_delta.attr,
	&lge_touch_attr_debug_mode.attr,
	&lge_touch_attr_tci_fail_reason.attr,
	&lge_touch_attr_swipe_fail_reason.attr,
	&lge_touch_attr_bootmode.attr,
	&lge_touch_attr_sd.attr,
	&lge_touch_attr_open_short.attr,
	&lge_touch_attr_mfts.attr,
	&lge_touch_attr_swipe_param.attr,
	&lge_touch_attr_swipe_mode.attr,
	&lge_touch_attr_fw_dump.attr,
	&lge_touch_attr_spi_speed.attr,
	&lge_touch_attr_deep_sleep.attr,
	&lge_touch_attr_u3fake.attr,
	&lge_touch_attr_lpwg_sd.attr,
#if 0
	&lge_touch_attr_doze1_calibration.attr,
	&lge_touch_attr_doze2_calibration.attr,
#endif
#if USE_ABT_MONITOR_APP
	&lge_touch_attr_abt_monitor.attr,
	&lge_touch_attr_raw_report.attr,
#endif
	NULL,
};

static int compare_fw_version(struct spi_device *spi,
				struct touch_fw_info *fw_info)
{
	struct sic_ts_data *ts =
			(struct sic_ts_data *)get_touch_handle(spi);
	u8 ic_ver[2], fw_img_ver[2];

	ic_ver[0] = ts->fw_info.fw_version[0];
	ic_ver[1] = ts->fw_info.fw_version[1];

	fw_img_ver[0] = ts->fw_info.fw_image_version[0];
	fw_img_ver[1] = ts->fw_info.fw_image_version[1];

	TOUCH_D(DEBUG_BASE_INFO,
		"fw_version(ic)[V%d.%02d]\n",
		ic_ver[0],
		ic_ver[1]);

	TOUCH_D(DEBUG_BASE_INFO,
		"fw_version(fw)[V%d.%02d]\n",
		fw_img_ver[0],
		fw_img_ver[1]);

	if (ic_ver[0] == 0 && ic_ver[1] == 0) {
		TOUCH_I("fw_version is 0.00, fw write\n");
		return 1;
	}
	/* both official version*/
	if (ic_ver[0] && fw_img_ver[0]) {
		if (ic_ver[1] != fw_img_ver[1]) {
			TOUCH_D(DEBUG_BASE_INFO,
					"fw_version mismatch. Update Official FW\n");
			return 1;
		}
		TOUCH_D(DEBUG_BASE_INFO | DEBUG_FW_UPGRADE,
				"official fw version: match.\n");
	} else if (ic_ver[0] && !fw_img_ver[0]) {
		/* ic official, dt test -> 1 and 0*/
		TOUCH_D(DEBUG_BASE_INFO,
				"fw_version mismatch. Update from Official to Test FW\n");
		return 1;
	} else if (!ic_ver[0] && fw_img_ver[0]) {
		/* ic test, dt official -> 0 and 1*/
		TOUCH_D(DEBUG_BASE_INFO,
				"fw_version mismatch. Update from Test to Official FW\n");
		return 1;
	} else if (!ic_ver[0] && !fw_img_ver[0]) {
		/* ic test, dt test -> 0 and 0*/
		if (ic_ver[1] < fw_img_ver[1]) {
			TOUCH_D(DEBUG_BASE_INFO | DEBUG_FW_UPGRADE,
					"fw_version mismatch. Update from Test to Test FW\n");
			return 1;
		}
	}

	return 0;
}

int write_spi(struct sic_ts_data *ts, u8 *data_buf, int w_cnt)
{
	int i, j, buf_idx;
	u8 buf[SPIM_MAX_TRANSFER_SIZE];
	u32 wdata = 0;

	if(data_buf == NULL || w_cnt < 1) {
		return -1;
	}

	if (w_cnt < 5) {
		for (i = 0; i < 4; i++) {
			if (i < w_cnt)
				buf[i] = data_buf[i];
			else
				buf[i] = 0;
		}
		wdata =	(buf[3] << 24 | buf[2] << 16 | buf[1] << 8 | buf[0]);
		DO_SAFE(sic_spi_write(ts->spi_device, SFLASHCTRL_WDATA0, (u8 *)&wdata, sizeof(u32)), error);
		wdata = ((w_cnt-1) << 4) | SPI_FC_WRITE;
		DO_SAFE(sic_spi_write(ts->spi_device, SFLASHCTRL_NORMCTRL, (u8 *)&wdata, sizeof(u32)), error);
	} else {
		for (i = 0; i < w_cnt; ) {
			memset(buf, 0, SPIM_MAX_TRANSFER_SIZE);
			for (buf_idx = 0; buf_idx < SPIM_MAX_TRANSFER_SIZE; buf_idx++, i++) {
				if (i >= w_cnt)
					break;
				buf[buf_idx] = data_buf[i];
			}
			for (j = 0; j < buf_idx; j+=4){
				wdata =	(buf[j+3] << 24 | buf[j+2] << 16 | buf[j+1] << 8 | buf[j+0]);
				DO_SAFE(sic_spi_write(ts->spi_device, SFLASHCTRL_WDATA0 + j/4, (u8 *)&wdata, sizeof(u32)), error);
			}

			wdata =	((buf_idx-1) << 4) | SPI_FC_WRITE;
			DO_SAFE(sic_spi_write(ts->spi_device, SFLASHCTRL_NORMCTRL, (u8 *)&wdata, sizeof(u32)), error);
		}
	}
	return 0;
error:
	TOUCH_D(DEBUG_BASE_INFO, "%s : FAIL!!!\n", __func__);
	return -ERROR;
}

int read_spi(struct sic_ts_data *ts, u8 *data_buf, int r_cnt)
{
	u32 wdata = 0;
//	u32 rdata = 0;
	int ret;

	if (data_buf == NULL || r_cnt < 1) {
		return -1;
	}

	if (r_cnt < 5) {
		wdata =	((r_cnt-1) << 4) | SPI_FC_READ;
		DO_SAFE(sic_spi_write(ts->spi_device, SFLASHCTRL_NORMCTRL, (u8 *)&wdata, sizeof(u32)), error);
		ret = sic_spi_read(ts->spi_device, SFLASHCTRL_RDATA0, (u8 *)&data_buf[0], sizeof(u32));
		if (ret < 0) {
			return -2;
		}
	} else {
		int i;
		int cur_buf_idx = 0;
		int read_cnt;
#if 0
		int k;
		int buf_idx;
		int remain_cnt, remain_no_cnt;
#endif
		for (i = r_cnt; i > 0; ) {
#if 0
			if (i > (SPIM_MAX_TRANSFER_SIZE-1)) {
				wdata =	((SPIM_MAX_TRANSFER_SIZE-1) << 4) | SPI_FC_READ;
				DO_SAFE(sic_spi_write(ts->spi_device, SFLASHCTRL_NORMCTRL, (u8 *)&wdata, sizeof(u32)), error);
				read_cnt		= SPIM_MAX_TRANSFER_SIZE;
				remain_cnt		= 4;
				remain_no_cnt	= 0;
			} else {
				wdata =	((i - 1) << 4) | SPI_FC_READ;
				DO_SAFE(sic_spi_write(ts->spi_device, SFLASHCTRL_NORMCTRL, (u8 *)&wdata, sizeof(u32)), error);
				read_cnt = i;
				remain_cnt = read_cnt % 4;
				remain_no_cnt = remain_cnt;
				if (remain_cnt)
					read_cnt += (4-remain_cnt);
			}

			for (buf_idx = 0; buf_idx <(read_cnt/4); buf_idx++, cur_buf_idx+=4) {

				ret = sic_spi_read(ts->spi_device, SFLASHCTRL_RDATA0 + buf_idx, (u8 *)&rdata, sizeof(u32));
				if (ret < 0) {
					return -2;
				}
				if (remain_no_cnt == 0) {
					for (k = 0; k < remain_cnt; k++) {
						data_buf[cur_buf_idx+k] = (rdata >> (k*8)) & 0xFF;
					}
				} else {
					data_buf[cur_buf_idx	] = (rdata) & 0xFF;
					data_buf[cur_buf_idx + 1] = (rdata >> 8) & 0xFF;
					data_buf[cur_buf_idx + 2] = (rdata >> 16) & 0xFF;
					data_buf[cur_buf_idx + 3] = (rdata >> 24) & 0xFF;
				}
				if (remain_no_cnt > 0){
					TOUCH_I("%s:%d sic_spi_read  remain_no_cnt=[%d]\n", __func__, __LINE__, remain_no_cnt); // no case
					remain_no_cnt--;
				}
			}
#else
			if (i > (SPIM_MAX_TRANSFER_SIZE-1)) {
				wdata =	((SPIM_MAX_TRANSFER_SIZE-1) << 4) | SPI_FC_READ;
				read_cnt = SPIM_MAX_TRANSFER_SIZE;
			} else {
				wdata =	((i - 1) << 4) | SPI_FC_READ;
				read_cnt = i;
			}
			DO_SAFE(sic_spi_write(ts->spi_device, SFLASHCTRL_NORMCTRL, (u8 *)&wdata, sizeof(u32)), error);
			ret = sic_spi_read(ts->spi_device, SFLASHCTRL_RDATA0, (u8 *)&data_buf[cur_buf_idx], read_cnt);
			if (ret < 0) {
				TOUCH_E("%s : sic_spi_read error!!!\n", __func__);
				return -2;
			}
			cur_buf_idx += read_cnt;
#endif
			i -= read_cnt;
		}
	}

	return 0;
error:
	TOUCH_D(DEBUG_BASE_INFO, "%s : FAIL!!!\n", __func__);
	return -ERROR;
}

void spi_cs_on(struct sic_ts_data *ts)
{
	u32 wdata = SPI_FC_CS_START;
	sic_spi_write(ts->spi_device, SFLASHCTRL_NORMCTRL, (u8 *)&wdata, sizeof(u32));
}

void spi_cs_off(struct sic_ts_data *ts)
{
	u32 wdata = SPI_FC_CS_STOP;
	sic_spi_write(ts->spi_device, SFLASHCTRL_NORMCTRL, (u8 *)&wdata, sizeof(u32));
}

int write_enable_spi_flash(struct sic_ts_data *ts)
{
	int ret;
	u8 buf[1];
	spi_cs_on(ts);
	buf[0] = WREN;
	ret = write_spi(ts, buf, 1);
	spi_cs_off(ts);
	if(ret < 0) {
		return -2;
	}
	return 0;
}

uint32_t read_busy_spi_flash(struct sic_ts_data *ts)
{
#define READ_TRY_CNT 10000
	int trycnt = READ_TRY_CNT;
	uint8_t buf[5];

	spi_cs_on(ts);
	buf[0] = RDSR_CMD;
	write_spi(ts, buf, 1);

	do {
		read_spi(ts, buf, 2);
		trycnt--;
	} while (trycnt > 0 && buf[0] & 1);
	spi_cs_off(ts);

	return READ_TRY_CNT - trycnt;
}

int spi_flash_write(struct sic_ts_data *ts, u32 addr, u8 *write_data_buf, int byte_cnt)
{
	u8 buf[4];
	int ret;

	write_enable_spi_flash(ts);

	spi_cs_on(ts);
	buf[0] = PAGE_PROGRAM;
	buf[1] = (addr >> 16) & 0xFF;
	buf[2] = (addr >> 8) & 0xFF;
	buf[3] = addr & 0xFF;
	ret = write_spi(ts, buf, 4);
	if(ret < 0) {
		spi_cs_off(ts);
		return -1;
	}
	ret = write_spi(ts, write_data_buf, byte_cnt);
	spi_cs_off(ts);
	if(ret < 0) {
		return -2;
	}
	if (read_busy_spi_flash(ts) < 0) {
		return -3;
	}

	return byte_cnt;
}

int spi_flash_read(struct sic_ts_data *ts, u32 addr, u8 *read_data_buf, int byte_cnt)
{
	u8 buf[4];
	int ret;
	spi_cs_on(ts);
	buf[0] = READ_CMD;
	buf[1] = (addr >> 16) & 0xFF;
	buf[2] = (addr >> 8) & 0xFF;
	buf[3] = addr & 0xFF;
	ret = write_spi(ts, buf, 4);
	if(ret < 0) {
		spi_cs_off(ts);
		return -2;
	}
	ret = read_spi(ts, read_data_buf,byte_cnt);
	spi_cs_off(ts);
	if(ret < 0) {
		return -3;
	}
	return byte_cnt;
}

int EraseChip(struct sic_ts_data *ts)
{
	u8 buf = CHIP_ERASE;
	int ret = 0;

	TOUCH_I("%s start\n", __func__);

	write_enable_spi_flash(ts);

	spi_cs_on(ts);
	ret = write_spi(ts, &buf, 1);
	spi_cs_off(ts);
	if(ret < 0) {
		TOUCH_I("%s fail -2\n", __func__);
		return -2;
	}
	if (read_busy_spi_flash(ts) < 0) {
		TOUCH_I("%s fail -3\n", __func__);
		return -3;
	}
	TOUCH_I("%s done\n", __func__);
	return 0;
}

int EraseBlock(struct sic_ts_data *ts, u32 addr)
{
	u8 buf[4];
	int ret;

	write_enable_spi_flash(ts);

	spi_cs_on(ts);
	buf[0] = BLOCK_ERASE;
	buf[1] = (addr >> 16) & 0xFF;
	buf[2] = (addr >> 8) & 0xFF;
	buf[3] = addr & 0xFF;
	ret = write_spi(ts, buf, 4);
	spi_cs_off(ts);
	if(ret < 0) {
		return -2;
	}
	if (read_busy_spi_flash(ts) < 0) {
		return -3;
	}
	return 0;
}

int EraseSector(struct sic_ts_data *ts, u32 addr)
{
	u8 buf[4];
	int ret;

	write_enable_spi_flash(ts);
	spi_cs_on(ts);
	buf[0] = SECTOR_ERASE;
	buf[1] = (addr >> 16) & 0xFF;
	buf[2] = (addr >> 8) & 0xFF;
	buf[3] = addr & 0xFF;
	ret = write_spi(ts, buf, 4);
	spi_cs_off(ts);
	if(ret < 0) {
		return -2;
	}
	if (read_busy_spi_flash(ts) < 0) {
		return -3;
	}
	return 0;
}

int DeepPowerDown(struct sic_ts_data *ts)
{
	int ret;
	u8 buf[1];
	spi_cs_on(ts);
	buf[0] = DP_CMD;
	ret = write_spi(ts, buf, 1);
	spi_cs_off(ts);
	if (ret < 0) {
		return -3;
	}
	return 0;
}

int ReleaseDeepPowerDown(struct sic_ts_data *ts)
{
	int ret;
	u8 buf[1];
	spi_cs_on(ts);
	buf[0] = RDP_CMD;
	ret = write_spi(ts, buf, 1);
	spi_cs_off(ts);
	if (ret < 0) {
		return -3;
	}
	return 0;
}

unsigned int detect_flash_ic(struct sic_ts_data *ts)
{
	int ret;
	unsigned char buf[8];
	u32 rdata=0;

	// clk enable for spim
	DO_SAFE(sic_spi_read(ts->spi_device, SYS_CLK_CTL,(u8 *)&rdata, sizeof(u32)), error);
	rdata = rdata | 0x40;
	DO_SAFE(sic_spi_write(ts->spi_device, SYS_CLK_CTL, (u8 *)&rdata, sizeof(u32)), error);

	spi_cs_on(ts);
	buf[0] = RDP_CMD;
	ret = write_spi(ts, buf, 1);
	spi_cs_off(ts);
	if (ret < 0)
	{
		return -1;
	}

	spi_cs_on(ts);
	buf[0] = RDID_CMD;
	buf[1] = buf[2] = buf[3] = 0x0;
	ret = write_spi(ts, buf, 1);
	if(ret < 0) {
		spi_cs_off(ts);
		return -2;
	}
	ret = read_spi(ts, buf, 2);
	spi_cs_off(ts);
	if(ret < 0) {
		return -3;
	}

	if (MANUFACTURE_ID == buf[0] && MX25V1006E_MEMORY_TYPE == buf[1]) {
		TOUCH_I("MX25V1006E probe ok! manufacturer= [0x%x] memory type =[0x%x]", buf[0], buf[1]);
		return 0;
	}
	else  {
		TOUCH_E("MX25V1006E detection failed : manufacturer[0x%x] != [0x%x] memory type[0x%x] != [0x%x]", MANUFACTURE_ID, buf[0], MX25V1006E_MEMORY_TYPE, buf[1]);
		return -1;
	}
error:
	TOUCH_D(DEBUG_BASE_INFO, "%s : FAIL!!!\n", __func__);
	return -ERROR;
}

int SicFirmwareUpgrade(struct sic_ts_data *ts,
					const struct firmware *fw_entry)
{
	unsigned char *my_image_bin = NULL;
	unsigned int my_image_size = 0;
	int ret = 0;
	int i = 0;
	u32 wdata = 0;
	u32 rdata = 0;

	u32 nCheckCrcStatus = 0;
	int tryCnt = 10;
	u32 trsize = 0;
	int try_cnt2 = 0;

	my_image_size = fw_entry->size;
	my_image_bin = (unsigned char *)(fw_entry->data);

	TOUCH_I("%s:%d Size = %d KB\n", __func__, __LINE__, my_image_size/1024);
	if (my_image_bin == NULL) {
		TOUCH_E("FW_IMAGE NULL\n");
		goto error;
	}

	if (my_image_size != SIZEOF_FLASH_IMAGE) {
		TOUCH_E("FW_IMAGE Size Error, Size = %x\n", my_image_size);
		goto error;
	}

	// CM3 Hold
	wdata = 2;
	DO_SAFE(sic_spi_write(ts->spi_device, SYS_RST_CTL,
		(u8 *)&wdata, sizeof(u32)), error);

	// code sram write enable
	wdata = 1;
	DO_SAFE(sic_spi_write(ts->spi_device, SYS_SRAM_CTRL,
		(u8 *)&wdata, sizeof(u32)), error);

	// DN FW download
	DO_SAFE(sic_spi_write(ts->spi_device, code_start_addr,
		(u8 *)&pplus_bl_code_v1[0], sizeof(pplus_bl_code_v1)), error);

	// Boot done
	wdata = 1;
	DO_SAFE(sic_spi_write(ts->spi_device, SYS_BOOT_CTL,
		(u8 *)&wdata, sizeof(u32)), error);

	// CM3 Release
	wdata = 0;
	DO_SAFE(sic_spi_write(ts->spi_device, SYS_RST_CTL,
		(u8 *)&wdata, sizeof(u32)), error);

	// BL boot done check
	try_cnt2 = 10;
	do {
		usleep(10000);
		ret = sic_spi_read(ts->spi_device, flashdownload_boot_chk,
			(u8 *)&rdata, sizeof(u32));
		try_cnt2--;
		if (try_cnt2 == 0) {
			TOUCH_E("%s:%d BL boot checking failed, read reg : %X",
				__func__, __LINE__, rdata);
			goto error;
		}
	} while (rdata != flash_bl_boot_chk);

	// flash erase command
	wdata = 2;
	DO_SAFE(sic_spi_write(ts->spi_device, flashdownload_ctl,
		(u8 *)&wdata, sizeof(u32)), error);

	// flash erase busy check
	try_cnt2 = 100;
	do {
		usleep(10000);
		ret = sic_spi_read(ts->spi_device, flashdownload_sts,
			(u8 *)&rdata, sizeof(u32));
		try_cnt2--;
		if (try_cnt2 == 0) {
			TOUCH_E("%s:%d erasee flash busy checking try cnt : %d",
				__func__, __LINE__,try_cnt2);
			goto error;
		}
	} while(rdata & 1);

	/*Firmware download*/
	for (i = 0; i < SIZEOF_FW; i += NUM_OF_FW_DN_SIZE) {
		if ((i + NUM_OF_FW_DN_SIZE) < SIZEOF_FW) {
			trsize = NUM_OF_FW_DN_SIZE;
		} else {
			trsize = SIZEOF_FW - i;
		}
		TOUCH_I("%s:%d [Firmware Writing.. %X] = trsize : %X\n",
			__func__, __LINE__, i, trsize);
		// Flash Dest addr set
		wdata = i;
		DO_SAFE(sic_spi_write(ts->spi_device, flashdownload_dest,
			(u8 *)&wdata, sizeof(u32)), error);
		// data write to data sram
		DO_SAFE(sic_spi_write(ts->spi_device, flashdownload_start_addr,
			&my_image_bin[i], trsize), error);
		// transfer command
		wdata = ((trsize << 8) | 1);
		DO_SAFE(sic_spi_write(ts->spi_device, flashdownload_ctl,
			(u8 *)&wdata, sizeof(u32)), error);
		// transfer busy check
		try_cnt2 = 100;
		do {
			usleep(10000);
			ret = sic_spi_read(ts->spi_device, flashdownload_sts,
				(u8 *)&rdata, sizeof(u32));
			try_cnt2--;
			if (try_cnt2 == 0) {
				TOUCH_E("%s:%d download code flash busy checking try cnt : %d",
					__func__, __LINE__,try_cnt2);
				goto error;
			}
		} while(rdata & 1);
	}

	/*Conf download*/
	trsize = SIZEOF_CFG;
	// Flash Dest addr set
	wdata = CFG_SECTOR_NUM*SIZEOF_SECTOR;
	DO_SAFE(sic_spi_write(ts->spi_device, flashdownload_dest,
		(u8 *)&wdata, sizeof(u32)), error);
	// data write to data sram
	DO_SAFE(sic_spi_write(ts->spi_device, flashdownload_start_addr,
	&my_image_bin[SIZEOF_FW], trsize), error);
	// transfer command
	wdata = ((trsize << 8) | 1);
	DO_SAFE(sic_spi_write(ts->spi_device, flashdownload_ctl,
		(u8 *)&wdata, sizeof(u32)), error);
	TOUCH_I("%s:%d [Config Writing.. %X] = trsize : %X\n",
		__func__, __LINE__, CFG_SECTOR_NUM*SIZEOF_SECTOR, trsize);
	// transfer busy check
	try_cnt2 = 100;
	do {
		usleep(10000);
		ret = sic_spi_read(ts->spi_device, flashdownload_sts,
			(u8 *)&rdata, sizeof(u32));
		try_cnt2--;
		if (try_cnt2 == 0) {
			TOUCH_E("%s:%d download cfg flash busy checking try cnt : %d",
				__func__, __LINE__,try_cnt2);
			goto error;
		}
	} while((rdata & 1) == 1);

	// crc check start
	/*
	crc control
	[0] crc check - 1 : crc check start
	[2:1] crc region select : 11 - full region, 01 - code region, 10 - configuration region, 00 - no region
	[3] sram/flash crc select : 0 - flash, 1 - codesram
	*/
	wdata = 0x7;
	DO_SAFE(sic_spi_write(ts->spi_device, SYS_CRC_CTL,
		(u8 *)&wdata, sizeof(u32)), error);

	do {
		usleep(10000);
		/*
		crc status
		[0] CRC busy
		[1] Code CRC result (1: pass / 0: fail)
		[2] Conf CRC Result (1: pass / 0: fail)
		*/
		ret = sic_spi_read(ts->spi_device, SYS_CRC_ST,
			(u8 *)&nCheckCrcStatus, sizeof(u32));
		tryCnt--;
		if (ret < 0 || tryCnt == 0) {
			TOUCH_I("%s:%d CRC Error status : %x, ret : %d, cnt : %d\n",
				__func__, __LINE__, nCheckCrcStatus, ret, tryCnt);
			goto error;
		}
	} while(nCheckCrcStatus & 1);

	if( ( nCheckCrcStatus >> 1) != 3) {
		TOUCH_E("	%s:%d Error! Code CRC result: %d, Conf CRC Result; %d\n",
			__func__, __LINE__, (nCheckCrcStatus >> 1) & 0x1, (nCheckCrcStatus >> 2) & 0x1);
		goto error;
	}

	// release cm3 reset
	wdata = 0x0;
	DO_SAFE(sic_spi_write(ts->spi_device, SYS_RST_CTL,
		(u8 *)&wdata, sizeof(u32)), error);

	// Download Okay!
	TOUCH_I("==============Firmware download Okay==============\n");
	TOUCH_I("Code CRC result: %d, Config CRC Result: %d\n",
		(nCheckCrcStatus >> 1) & 0x1, (nCheckCrcStatus >> 2) & 0x1);
	TOUCH_I("==================================================\n");

	return 0;

error:
	TOUCH_E("%s Firmware Download Failed!!", __func__);
	return -EIO;
}

int check_tci_debug_result(struct sic_ts_data *ts, u8 wake_up_type)
{
	u32 rdata = 0;
	u8 buf = 0;
	u8 i = 0;
	u32 addr = 0;
	u8 count0 = 0;
	u8 count1 = 0;
	u8 count_max = 0;
	u8 fail_reason_buf[TCI_MAX_NUM][LPWG_FAIL_BUFFER_MAX_NUM];
	int result = 0;

	DO_SAFE(sic_spi_read(ts->spi_device, CMD_ABT_LPWG_TCI_FAIL_COUNT_READ,
				(u8 *)&rdata, sizeof(u32)), error);

	count0 = (rdata & 0xFFFF);
	count1 = ((rdata >> 16) & 0xFFFF);
	count_max = (count0 > count1) ? (count0) : (count1);

	if (count_max == 0) {
		return 0;
	} else if (count_max > LPWG_FAIL_BUFFER_MAX_NUM) {
		count_max = LPWG_FAIL_BUFFER_MAX_NUM;
		if (count0 > LPWG_FAIL_BUFFER_MAX_NUM)
			count0 = LPWG_FAIL_BUFFER_MAX_NUM;
		if (count1 > LPWG_FAIL_BUFFER_MAX_NUM)
			count1 = LPWG_FAIL_BUFFER_MAX_NUM;
	}

	addr = CMD_ABT_LPWG_TCI_FAIL_BUFFER_READ;

	for (i = 0; i < count_max; i++) {

		DO_SAFE(sic_spi_write(ts->spi_device, CMD_ABT_LPWG_TCI_FAIL_BUFFER_ACCESS_WRITE,
				(u8 *)&i, sizeof(u32)), error);

		result = sic_spi_read(ts->spi_device, addr,
					(u8 *)&rdata, sizeof(u32));
		if (!result) {
			if (i < count0) {
				buf = (rdata & 0xFFFF);
				fail_reason_buf[0][i] = buf;
			}
			if (i < count1) {
				buf = ((rdata >> 16) & 0xFFFF);
				fail_reason_buf[1][i] = buf;
			}
		}
	}
	for (i = count1; 0 < i; i--) {
		buf = fail_reason_buf[1][i-1];
		if ((DELAY_TIME == (0x01 << buf)) &&
			wake_up_type == TCI_FAIL_DEBUG) {
			TOUCH_I("TCI(2) OverTap Detected\n");
			ts->pw_data.data_num = 1;
			ts->pw_data.data[0].x = 0;
			ts->pw_data.data[0].y = 0;
			send_uevent_lpwg(ts->spi_device, LPWG_PASSWORD);
			break;
		}
	}

	if (ts->fail_reason[0]) {
		for (i = 0; i < count0; i++) {
			buf = fail_reason_buf[0][i];
			TOUCH_I("TCI(1)-DBG[%d/%d]: %s\n",
				i + 1, count0, (buf > 0 && buf < 11) ?
				sic_tci_fail_reason_str[buf] :
				sic_tci_fail_reason_str[0]);
		}
		for (i = 0; i < count1; i++) {
			buf = fail_reason_buf[1][i];
			TOUCH_I("TCI(2)-DBG[%d/%d]: %s\n",
				i + 1, count1, (buf > 0 && buf < 11) ?
				sic_tci_fail_reason_str[buf] :
				sic_tci_fail_reason_str[0]);
		}
	}
	return 0;
error:
	return -ERROR;
}

int print_swipe_debug_result(struct sic_ts_data *ts)
{
	u32 rdata = 0;
	u8 i = 0;
	u32 addr = 0;
	u8 count = 0;
	int result = 0;

	DO_SAFE(sic_spi_read(ts->spi_device, CMD_ABT_LPWG_SWIPE_FAIL_COUNT_READ,
				(u8 *)&rdata, sizeof(u32)), error);

	count = rdata;
	addr = CMD_ABT_LPWG_SWIPE_FAIL_BUFFER_READ;

	for (i = 0; i < count; i++) {
		DO_SAFE(sic_spi_write(ts->spi_device, CMD_ABT_LPWG_SWIPE_FAIL_BUFFER_ACCESS_WRITE,
				(u8 *)&i, sizeof(u32)), error);

		result = sic_spi_read(ts->spi_device, addr,
					(u8 *)&rdata, sizeof(u32));
		if (!result) {
			TOUCH_I("SWIPE Down Debug[%d/%d]: %s\n",
				i + 1, count, (rdata > 0 && rdata < 7) ?
				sic_swipe_fail_reason_str[rdata] :
				sic_swipe_fail_reason_str[0]);

			TOUCH_I("SWIPE Up Debug[%d/%d]: %s\n",
				i + 1, count, ((rdata << 16) > 0
						&& (rdata << 16) < 7) ?
				sic_swipe_fail_reason_str[rdata] :
				sic_swipe_fail_reason_str[0]);
		}
	}

	return 0;
error:
	return -ERROR;
}

static int get_swipe_data(struct spi_device *spi)
{
	struct sic_ts_data *ts =
		(struct sic_ts_data *)get_touch_handle(spi);

	u32 rdata[3];

	u16 swipe_start_x = 0;
	u16 swipe_start_y = 0;
	u16 swipe_end_x = 0;
	u16 swipe_end_y = 0;
	u16 swipe_time = 0;

	DO_SAFE(sic_spi_read(spi, report_base+1,
		(u8 *)&rdata[0], 3*sizeof(u32)), error);

	swipe_start_x = rdata[0] & 0xffff;
	swipe_start_y = rdata[0]  >> 16;
	swipe_end_x = rdata[1] & 0xffff;
	swipe_end_y = rdata[1]  >> 16;
	swipe_time = rdata[2] & 0xffff;

	TOUCH_D(DEBUG_BASE_INFO || DEBUG_LPWG,
			"LPWG Swipe Gesture: start(%4d,%4d) end(%4d,%4d) swipe_time(%dms)\n",
			swipe_start_x, swipe_start_y,
			swipe_end_x, swipe_end_y,
			swipe_time);

	ts->pw_data.data_num = 1;
	ts->pw_data.data[0].x = swipe_end_x;
	ts->pw_data.data[0].y = swipe_end_y;

	return 0;
error:
	TOUCH_E("failed to read swipe data.\n");
	return -ERROR;
}

static void charger_state_func(struct work_struct *work_charger)
{
	struct sic_ts_data *ts = container_of(to_delayed_work(work_charger),
			struct sic_ts_data, work_charger);

	u32 charger_state = 0;
	int ta_status = atomic_read(&ts->state->touch_ta_status);

	TOUCH_TRACE();

	if (ta_status == 0)
		charger_state = 0x00;
	else if ((ta_status == 2) || (ta_status == 4))
		charger_state = 0x02;
	else
		charger_state = 0x01;


	TOUCH_I("%s: charger_state = %d\n", __func__, charger_state);

	mutex_lock(&ts->pdata->thread_lock);
	sic_spi_write(ts->spi_device, spr_charger_sts, (u8 *)&charger_state, sizeof(u32));
	TOUCH_I("%s: write charger_state info at TOUCH IC\n", __func__);
	mutex_unlock(&ts->pdata->thread_lock);

	return;
}
enum error_type sic_ts_probe(struct spi_device *spi,
	struct touch_platform_data *lge_ts_data,
	struct state_info *state)
{
	struct sic_ts_data *ts;

	TOUCH_TRACE();
	TOUCH_I("sic_ts_probe\n");

	ASSIGN(ts = devm_kzalloc(&spi->dev, sizeof(struct sic_ts_data),
					GFP_KERNEL), error);
	set_touch_handle(spi, ts);

	ts->spi_device = spi;
	ts->pdata = lge_ts_data;
	ts->state = state;

	if (ts->pdata->pwr->use_regulator) {
		DO_IF(IS_ERR(ts->regulator_vdd = regulator_get(&spi->dev,
						ts->pdata->pwr->vdd)), error);
		DO_IF(IS_ERR(ts->regulator_vio = regulator_get(&spi->dev,
						ts->pdata->pwr->vio)), error);
		if (ts->pdata->pwr->vdd_voltage > 0)
			DO_SAFE(regulator_set_voltage(ts->regulator_vdd,
						ts->pdata->pwr->vdd_voltage,
						ts->pdata->pwr->vdd_voltage),
				error);
		if (ts->pdata->pwr->vio_voltage > 0)
			DO_SAFE(regulator_set_voltage(ts->regulator_vio,
						ts->pdata->pwr->vio_voltage,
						ts->pdata->pwr->vio_voltage),
				error);
	}

	ts->is_probed = 0;
	ts->is_init = 0;
	ts->is_palm = 0;

	ts->lpwg_ctrl.screen = 1;
	ts->lpwg_ctrl.sensor = 1;

	mutex_init(&ts->spi_lock);

#if USE_ABT_MONITOR_APP
	mutex_init(&abt_i2c_comm_lock);
#endif

	atomic_set(&ts->lpwg_ctrl.is_suspend, 0);
	INIT_DELAYED_WORK(&ts->work_charger, charger_state_func);
	return NO_ERROR;
error:
	return ERROR;
}

enum error_type sic_ts_remove(struct spi_device *spi)
{
	struct sic_ts_data *ts =
			(struct sic_ts_data *)get_touch_handle(spi);

	TOUCH_TRACE();

	if (ts->pdata->pwr->use_regulator) {
		regulator_put(ts->regulator_vio);
		regulator_put(ts->regulator_vdd);
	}

#if USE_ABT_MONITOR_APP
	mutex_destroy(&abt_i2c_comm_lock);
	if (abt_socket_mutex_flag == 1)
		mutex_destroy(&abt_socket_lock);
#endif

	return NO_ERROR;
}

enum error_type sic_ts_init(struct spi_device *spi)
{
	struct sic_ts_data *ts
		= (struct sic_ts_data *)get_touch_handle(spi);
	u32 wdata = 1;
	u8 mode = ts->lpwg_ctrl.lpwg_mode;
	int is_suspend = atomic_read(&ts->lpwg_ctrl.is_suspend);

	TOUCH_TRACE();
	TOUCH_D(DEBUG_BASE_INFO, "%s start\n", __func__);

	if (!ts->is_probed) {
		get_ic_info(ts);
		ts->fail_overtap = DELAY_TIME;
		if (!ts->fail_reason[0])
			ts->fail_type = TCI_DEBUG_REASON_ALL;
		//ts->fail_reason[0] = 1; /* Buffer Type */
		ts->is_probed = 1;
	}

#if	USE_ABT_MONITOR_APP
	if (abt_report_mode != 0)
		DO_SAFE(abt_force_set_report_mode(spi, abt_report_mode),
			error);
#endif

	/* touch device configured */
	DO_SAFE(sic_spi_write(spi, tc_device_ctl,
				(u8 *)&wdata, sizeof(u32)), error);


	/* touch interrupt enable */
	DO_SAFE(sic_spi_write(spi, tc_interrupt_ctl,
				(u8 *)&wdata, sizeof(u32)), error);

	ts->is_init = 1;
	doze_state = DOZE1_STATUS;

#if 0
	/* driving control*/
	wdata = 0x181;
	DO_SAFE(sic_spi_write(spi, tc_driving_ctl,
				(u8 *)&wdata, sizeof(u32)), error);
#endif

	/* set lpwg mode SIC TODO */
	DO_SAFE(lpwg_control(ts, is_suspend ? mode : 0), error);

	TOUCH_D(DEBUG_BASE_INFO, "%s end\n", __func__);
	return NO_ERROR;
error:
	TOUCH_D(DEBUG_BASE_INFO, "%s error\n", __func__);
	return ERROR;
}

enum error_type sic_ts_get_data(struct spi_device *spi,
	struct touch_data *curr_data, const struct touch_data *prev_data)
{
	struct sic_ts_data *ts
		= (struct sic_ts_data *)get_touch_handle(spi);
	struct T_TouchData *t_Data;
	struct t_data *c_data;

	u32  i;
	u8  f_index = 0;
	u32 touchUpCnt = 0;
	int is_normal_operation = 1;
	//u32 interrupt_data[5] = {0x0, };
	u32 interrupt_status_reg = 0x0;
#if USE_ABT_MONITOR_APP
	struct T_ABTLog_FileHead tHeadBuffer;
	struct T_ReportP local_reportP;
	u8 local_reportP_144[256];
	u32 wdata;
	u16 addr;
	int ocd_piece_size;
	int node;
	u32 head_loc[4];
	static int abt_monitor_head	= 1;
	static int abt_ocd_off		= 1;
	static u32 prev_rnd_piece_no = DEF_RNDCPY_EVERY_Nth_FRAME;

	struct T_TouchInfo *touchInfo = &local_reportP.touchInfo;
	struct T_TouchData *ocd_touchData = &local_reportP.touchData[0];
	struct T_OnChipDebug *ocd = &local_reportP.ocd;

	/* only once after booting */
	if (abt_monitor_head) {
		DO_SAFE(sic_spi_read(spi, CMD_ABT_LOC_X_START_READ,
				(u8 *)&head_loc[0], sizeof(u32)), error);
		DO_SAFE(sic_spi_read(spi, CMD_ABT_LOC_X_END_READ,
				(u8 *)&head_loc[1], sizeof(u32)), error);
		DO_SAFE(sic_spi_read(spi, CMD_ABT_LOC_Y_START_READ,
				(u8 *)&head_loc[2], sizeof(u32)), error);
		DO_SAFE(sic_spi_read(spi, CMD_ABT_LOC_Y_END_READ,
				(u8 *)&head_loc[3], sizeof(u32)), error);

		abt_monitor_head = 0;

		tHeadBuffer.resolution_x = 1440;
		tHeadBuffer.resolution_y = 2720;

		tHeadBuffer.node_cnt_x = ACTIVE_SCREEN_CNT_X;
		tHeadBuffer.node_cnt_y = ACTIVE_SCREEN_CNT_Y;
		tHeadBuffer.additional_node_cnt = 0;

		tHeadBuffer.rn_min = 1000;
		tHeadBuffer.rn_max = 1300;

		tHeadBuffer.raw_data_size = sizeof(u16);
		tHeadBuffer.rn_data_size = sizeof(u16);
		tHeadBuffer.frame_data_type = DATA_TYPE_RN_ORG;
		tHeadBuffer.frame_data_size = sizeof(u16);

		tHeadBuffer.loc_x[0] = (u16) head_loc[0];
		tHeadBuffer.loc_x[1] = (u16) head_loc[1];
		tHeadBuffer.loc_y[0] = (u16) head_loc[2];
		tHeadBuffer.loc_y[1] = (u16) head_loc[3];

		memcpy((u8 *)&abt_head[0], (u8 *)&tHeadBuffer,
					sizeof(struct T_ABTLog_FileHead));
	}

	if (abt_show_mode < REPORT_RNORG || REPORT_DEBUG_ONLY < abt_show_mode)
		is_normal_operation = 1;
	else
		is_normal_operation = 0;

#endif /* USE_ABT_MONITOR_APP */

	TOUCH_TRACE();

	if (!ts->is_init) {
		ts_interrupt_clear(ts);
		return IGNORE_EVENT;
	}

	curr_data->total_num = 0;
	curr_data->id_mask = 0;

#if 0
	/* read 4 byte * 5 = 20 bytes */
	DO_SAFE(sic_spi_read(spi, tc_status, (u8 *)interrupt_data,
				sizeof(u32)*5), error);

	memcpy(&ts->ts_data.device_status_reg, &interrupt_data[0], sizeof(u32));
	memcpy(&ts->ts_data.report.touchInfo, &interrupt_data[1], sizeof(u32));
#endif

	DO_SAFE(sic_spi_read(spi, tc_status,
			(u8 *)&ts->ts_data.device_status_reg, sizeof(u32)),
		error);

	TOUCH_D(DEBUG_GET_DATA,
		"device_status_regs 0x%x\n", ts->ts_data.device_status_reg);

	DO_IF(!((ts->ts_data.device_status_reg >> 5) & 0x1), error);

	if ((ts->ts_data.device_status_reg >> 10) & 0x1) {
		TOUCH_I("ESD Error detected");
		goto error;
	}

	/* read wakeUpType */
	DO_SAFE(sic_spi_read(spi, report_base,
			(u8 *)&ts->ts_data.report.touchInfo,
			sizeof(ts->ts_data.report.touchInfo)),
		error);

	if (ts->ts_data.report.touchInfo.wakeUpType != TCI_NOTHING)
		interrupt_status_reg = INTERRUPT_MASK_CUSTOM;
	else
		interrupt_status_reg = INTERRUPT_MASK_ABS0;


	if (interrupt_status_reg & INTERRUPT_MASK_CUSTOM) {
		u8 wake_up_type;

		wake_up_type = ts->ts_data.report.touchInfo.wakeUpType;

		/* handle wakeUpType */
		if (wake_up_type == TCI_1) {
			if (ts->lpwg_ctrl.double_tap_enable) {
				get_tci_data(ts, 2);
				send_uevent_lpwg(ts->spi_device, LPWG_DOUBLE_TAP);
			}
		} else if (wake_up_type == TCI_2) {
			if (ts->lpwg_ctrl.password_enable) {
				get_tci_data(ts, ts->pw_data.tap_count);
				send_uevent_lpwg(ts->spi_device, LPWG_PASSWORD);
			}
		} else if (wake_up_type == SWIPE_DOWN) {
			if (get_swipe_data(spi) == 0) {
				ts->pdata->swipe_pwr_ctr = SKIP_PWR_CON;
				if (ts->pdata->lockscreen_stat == 1) {
					send_uevent_lpwg(spi,
						LPWG_SWIPE_DOWN);
					swipe_disable(ts);
				} else {
					ts->pdata->swipe_pwr_ctr =
						WAIT_SWIPE_WAKEUP;
					TOUCH_I("drop SWP(type:%d swp_pwr:%d\n",
					LPWG_SWIPE_DOWN, WAIT_SWIPE_WAKEUP);
				}
			}
		} else if (wake_up_type == SWIPE_UP) {
			if (get_swipe_data(spi) == 0) {
				ts->pdata->swipe_pwr_ctr = SKIP_PWR_CON;
				send_uevent_lpwg(spi, LPWG_SWIPE_UP);
				swipe_disable(ts);
			}
		} else if (wake_up_type == TCI_FAIL_DEBUG) {
			TOUCH_D(DEBUG_BASE_INFO | DEBUG_LPWG,
				"LPWG wakeUpType is TCI DEBUG\n");
		} else {
			TOUCH_D(DEBUG_BASE_INFO | DEBUG_LPWG,
				"LPWG wakeUpType is not support type![%d]\n",
				wake_up_type);
		}
		if ((ts->fail_reason[0] &&
			(wake_up_type == TCI_1 || wake_up_type == TCI_2))
			|| wake_up_type == TCI_FAIL_DEBUG)
			check_tci_debug_result(ts, wake_up_type);
		if (ts->fail_reason[1] &&
			(wake_up_type == SWIPE_DOWN ||
				wake_up_type == SWIPE_UP ||
				wake_up_type == TCI_FAIL_DEBUG))
			print_swipe_debug_result(ts);

		ts_interrupt_clear(ts);
		return IGNORE_EVENT;
	} else if (interrupt_status_reg & INTERRUPT_MASK_ABS0) {

		/*
		 * NORMAL OPERATION (no on-chip-debug)
		 */
		if (is_normal_operation) {
			#if USE_ABT_MONITOR_APP
			/* Write onchipdebug off (once) */
			if (abt_ocd_off) {
				wdata = 0;
				DO_SAFE(sic_spi_write(spi, CMD_ABT_OCD_ON_WRITE,
					(u8 *)&wdata, sizeof(wdata)), error);

				DO_SAFE(sic_spi_read(spi, CMD_ABT_OCD_ON_READ,
					(u8 *)&wdata, sizeof(wdata)), error);
				TOUCH_I("[ABT] onchipdebug off: wdata=%d\n",
						wdata);

				abt_ocd_off = 0;
			}
			#endif

			/* check if touch cnt is valid */
			if (!ts->ts_data.report.touchInfo.touchCnt ||
				ts->ts_data.report.touchInfo.touchCnt >
					MAX_REPORT_FINGER) {
				ts_interrupt_clear(ts);
				return IGNORE_EVENT;
			}

#if 0
			/* read touchData - touchCnt-1 */
			memcpy(&ts->ts_data.report.touchData[0],
			&interrupt_data[2], sizeof(struct T_TouchData));
			if (ts->ts_data.report.touchInfo.touchCnt > 1) {
			DO_SAFE(sic_spi_read(ts->spi_device, report_base+4,
				(u8 *)&ts->ts_data.report.touchData[1],
				sizeof(struct T_TouchData) *
				(ts->ts_data.report.touchInfo.touchCnt-1)),
				error);
			}
#endif

			/* read touchData*/
			DO_SAFE(sic_spi_read(ts->spi_device, report_base+1,
					(u8 *)&ts->ts_data.report.touchData[0],
					sizeof(struct T_TouchData) *
					ts->ts_data.report.touchInfo.touchCnt),
				error);
			t_Data = &ts->ts_data.report.touchData[0];

			/* palm touch case */
			if (t_Data[0].track_id == PALM_ID) {
				if (t_Data[0].event == TOUCHSTS_DOWN)
					TOUCH_I("Palm Detected\n");
				else if (t_Data[0].event == TOUCHSTS_UP)
					TOUCH_I("Palm Released\n");

				ts_interrupt_clear(ts);
				return NO_ERROR;
			}
			/* check if doze_mode is doze1 */
			if (doze_state != DOZE1_STATUS) {
				TOUCH_I("Don't report ABS except doze1\n");
				memset(curr_data, 0, sizeof(struct touch_data));
				return NO_ERROR;
			}

			for (i = 0; i < ts->ts_data.report.touchInfo.touchCnt;
					i++) {
				if ((t_Data[i].event == TOUCHSTS_MOVE ||
					t_Data[i].event == TOUCHSTS_DOWN) &&
					t_Data[i].track_id <
						MAX_REPORT_FINGER) {
					c_data = &curr_data->touch[f_index];
					c_data->id = t_Data[i].track_id;
					c_data->type =
						t_Data[i].toolType;
					c_data->x = c_data->raw_x =
						t_Data[i].x;
					c_data->y = c_data->raw_y =
						t_Data[i].y;
					c_data->width_major =
						t_Data[i].width_major;
					c_data->width_minor =
						t_Data[i].width_minor;
					if (t_Data[i].width_major
						== t_Data[i].width_minor)
						c_data->orientation = 1;
					else
						c_data->orientation =
							t_Data[i].angle;
					c_data->pressure =
						t_Data[i].byte.pressure;

					curr_data->id_mask |=
						(0x1 << (c_data->id));
					curr_data->total_num++;

					f_index++;
				} else if (t_Data[i].event == TOUCHSTS_UP) {
					touchUpCnt++;
					TOUCH_D(DEBUG_GET_DATA,
						"<%d> type[%d] pos(%4d,%4d) w_m[%2d] w_n[%2d] o[%2d] p[%2d] reported touch_up\n",
						t_Data[i].track_id,
						t_Data[i].toolType,
						t_Data[i].x, t_Data[i].y,
						t_Data[i].width_major,
						t_Data[i].width_minor,
						t_Data[i].angle,
						t_Data[i].byte.pressure);
				}
			}

			TOUCH_D(DEBUG_GET_DATA,
				"ID[0x%x] Total_num[%d]\n",
				curr_data->id_mask, curr_data->total_num);
		}
#if USE_ABT_MONITOR_APP
		/*
		 * ON-CHIP-DEBUG OPERATION
		 */
		else { /*if (is_normal_operation==0)*/
			abt_ocd_off = 1;

			/* Read all report data */
			for (i = 0; i < 2; i++) {
				if (sic_spi_read(ts->spi_device,
					report_base+(128*i/4),
					(u8 *)&(local_reportP_144[(i*128)]),
					128) < 0)
					TOUCH_I(
						"report data reg addr read fail 1\n");
			}
			memcpy((u8 *)&local_reportP,
				(u8 *)&local_reportP_144[0],
				sizeof(struct T_ReportP));

			/* Write onchipdebug on */
			if (abt_ocd_on) {
				wdata = abt_show_mode;
				TOUCH_I(
					"[ABT] onchipdebug on(before write): wdata=%d\n",
						wdata);
				DO_SAFE(sic_spi_write(spi, CMD_ABT_OCD_ON_WRITE,
					(u8 *)&wdata, sizeof(wdata)), error);

				DO_SAFE(sic_spi_read(spi, CMD_ABT_OCD_ON_READ,
					(u8 *)&wdata, sizeof(wdata)), error);
				TOUCH_I(
					"[ABT] onchipdebug on(after write): wdata=%d\n",
						wdata);

				abt_ocd_on = 0;
			} else if (abt_show_mode < REPORT_DEBUG_ONLY) {
				if (ocd->rnd_piece_no == 0 &&
					ocd->rnd_piece_no != prev_rnd_piece_no)
					abt_ocd_read ^= 1;

				ocd_piece_size =
						ACTIVE_SCREEN_CNT_X*
						ACTIVE_SCREEN_CNT_Y/DEF_RNDCPY_EVERY_Nth_FRAME;
				if(ocd_piece_size%2)
					ocd_piece_size -= 1;
				addr = ((ocd->rnd_addr-0x20000000)/4 +
					((ocd->rnd_piece_no)*ocd_piece_size)/2);
				addr += 0x8000;
				node = (ocd->rnd_piece_no)*
						ocd_piece_size;

				if (ocd->rnd_piece_no != prev_rnd_piece_no) {
					int ret;

					if((ocd->rnd_piece_no)!=(DEF_RNDCPY_EVERY_Nth_FRAME-1)) {
						ret = sic_spi_read(ts->spi_device, addr,
							(u8 *)&abt_ocd[abt_ocd_read][node],
							ocd_piece_size*2);
						if (ret < 0)
							TOUCH_I(
							"RNdata reg addr write fail [%d]\n",
							ocd->rnd_piece_no);
					} else {
						i = ACTIVE_SCREEN_CNT_X*ACTIVE_SCREEN_CNT_Y;
						i -= ocd_piece_size*(DEF_RNDCPY_EVERY_Nth_FRAME-1);
						ret = sic_spi_read(ts->spi_device, addr,
							(u8 *)&abt_ocd[abt_ocd_read][node],
							i*2);
						if (ret < 0)
							TOUCH_I(
							"RNdata reg addr write fail [%d]\n",
							ocd->rnd_piece_no);
					}
				}
				prev_rnd_piece_no = ocd->rnd_piece_no;
			}

			t_Data = ocd_touchData;

			/* palm touch case */
			if (t_Data[0].track_id == PALM_ID) {
				if (t_Data[0].event == TOUCHSTS_DOWN) {
					TOUCH_I("Palm Detected : [%d]->[%d]\n",
						ts->is_palm, 1);
					ts->is_palm = 1;
				} else if (t_Data[0].event == TOUCHSTS_UP) {
					TOUCH_I("Palm Released : [%d]->[%d]\n",
						ts->is_palm, 0);
					ts->is_palm = 0;
				}

				ts_interrupt_clear(ts);
				return NO_ERROR;
			} else if (touchInfo->touchCnt > 0 &&
					touchInfo->touchCnt <
						(MAX_REPORT_FINGER+1)) {
				for (i = 0; i < touchInfo->touchCnt; i++) {
					if ((t_Data[i].event ==
							TOUCHSTS_MOVE ||
						t_Data[i].event ==
							TOUCHSTS_DOWN) &&
						t_Data[i].track_id <
							MAX_REPORT_FINGER) {
						c_data =
						&curr_data->touch[f_index];
						c_data->id =
							t_Data[i].track_id;
						c_data->type =
							t_Data[i].toolType;
						c_data->x = c_data->raw_x =
							t_Data[i].x;
						c_data->y = c_data->raw_y =
							t_Data[i].y;
						c_data->width_major =
							t_Data[i].width_major;
						c_data->width_minor =
							t_Data[i].width_minor;
						c_data->orientation =
								t_Data[i].angle;
						c_data->pressure =
							t_Data[i].byte.pressure;

						curr_data->id_mask |=
							(0x1 << (c_data->id));
						curr_data->total_num++;

						TOUCH_D(DEBUG_GET_DATA,
							"<%d> type[%d] pos(%4d,%4d) w_m[%2d] w_n[%2d] o[%2d] p[%2d]\n",
							c_data->id,
							c_data->type,
							c_data->x,
							c_data->y,
							c_data->width_major,
							c_data->width_minor,
							c_data->orientation,
							c_data->pressure);

						f_index++;
					} else if (t_Data[i].event ==
								TOUCHSTS_UP) {
						touchUpCnt++;

						TOUCH_D(DEBUG_GET_DATA,
						"<%d> type[%d] pos(%4d,%4d) w_m[%2d] w_n[%2d] o[%2d] p[%2d] reported touch_up\n",
						t_Data[i].track_id,
						t_Data[i].toolType,
						t_Data[i].x,
						t_Data[i].y,
						t_Data[i].width_major,
						t_Data[i].width_minor,
						t_Data[i].angle,
						t_Data[i].byte.pressure);
					}
				}

				TOUCH_D(DEBUG_GET_DATA,
					"ID[0x%x] Total_num[%d]\n",
					curr_data->id_mask,
					curr_data->total_num);
			}
		}
#endif /* USE_ABT_MONITOR_APP */
	} else {
		ts_interrupt_clear(ts);
		return IGNORE_EVENT;
	}

	ts_interrupt_clear(ts);

#if USE_ABT_MONITOR_APP
	if (!is_normal_operation)
		memcpy((u8 *)&abt_reportP, (u8 *)&local_reportP,
			sizeof(struct T_ReportP));
#endif

	return NO_ERROR;
error:
	ts_interrupt_clear(ts);
	return ERROR;
}

#if USE_ABT_MONITOR_APP

enum error_type sic_ts_get_data_debug_mode(struct spi_device *spi,
	struct touch_data *curr_data, const struct touch_data *prev_data)
{
	struct sic_ts_data *ts =
			(struct sic_ts_data *)get_touch_handle(spi);

	struct t_data *c_data;

	u32  i;
	u8  f_index = 0;
	u32 touchUpCnt = 0;
#if 0
	u32 interrupt_data[5] = {0x0, };
#endif
	u32 interrupt_status_reg = 0x0;

	struct T_TouchInfo *touchInfo = &ts->ts_data.report.touchInfo;
	struct T_TouchData *t_Data = &ts->ts_data.report.touchData[0];

	struct send_data_t *packet_ptr = &abt_comm.data_send;
	struct debug_report_header *d_header =
		(struct debug_report_header *)(&abt_comm.data_send.data[0]);
	int d_header_size = sizeof(struct debug_report_header);
	u8 *d_data_ptr = (u8 *)d_header + d_header_size;
	u32 TC_debug_data_ptr = CMD_GET_ABT_DEBUG_REPORT;
	u32 i2c_pack_count = 0;
	struct timeval t_stamp;

	mutex_lock(&abt_i2c_comm_lock);

	TOUCH_TRACE();

	if (!ts->is_init)
		goto ignore;

	curr_data->total_num = 0;
	curr_data->id_mask = 0;

#if 0
	/* read 4 byte * 5 = 20 bytes */
	DO_SAFE(sic_spi_read(spi, tc_status, (u8 *)interrupt_data,
				sizeof(u32)*5), error);

	memcpy(&ts->ts_data.device_status_reg, &interrupt_data[0], sizeof(u32));
	memcpy(&ts->ts_data.report.touchInfo, &interrupt_data[1], sizeof(u32));
#else
	DO_SAFE(sic_spi_read(spi, tc_status,
			(u8 *)&ts->ts_data.device_status_reg, sizeof(u32)),
		error);
	/* read wakeUpType */
	DO_SAFE(sic_spi_read(spi, report_base,
			(u8 *)&ts->ts_data.report.touchInfo,
			sizeof(ts->ts_data.report.touchInfo)),
		error);
#endif

	DO_IF(!((ts->ts_data.device_status_reg >> 5) & 0x1), error);
#if 0
	if ((ts->ts_data.device_status_reg >> 10) & 0x1) {
		TOUCH_I("ESD Error detected");
		goto error;
	}
#endif

	if (ts->ts_data.report.touchInfo.wakeUpType != TCI_NOTHING)
		interrupt_status_reg = INTERRUPT_MASK_CUSTOM;
	else
		interrupt_status_reg = INTERRUPT_MASK_ABS0;

	if (abt_report_mode) {
		DO_SAFE(sic_spi_read(spi, TC_debug_data_ptr,
				(u8 *)d_header, d_header_size), error);
		if (d_header->type == abt_report_mode) {
			i2c_pack_count = (d_header->data_size)/I2C_PACKET_SIZE;
			TC_debug_data_ptr += d_header_size/4;
			for (i = 0; i < i2c_pack_count; i++) {
				DO_SAFE(sic_spi_read(spi, TC_debug_data_ptr,
					(u8 *)d_data_ptr, I2C_PACKET_SIZE),
					error);
				d_data_ptr += I2C_PACKET_SIZE;
				TC_debug_data_ptr += I2C_PACKET_SIZE/4;
			}

			if (d_header->data_size % I2C_PACKET_SIZE != 0) {
				DO_SAFE(sic_spi_read(spi, TC_debug_data_ptr,
					d_data_ptr,
					d_header->data_size % I2C_PACKET_SIZE),
					error);
				d_data_ptr +=
					d_header->data_size % I2C_PACKET_SIZE;
			}
		} else {
			TOUCH_I("debug data load error !!\n");
			TOUCH_I("type : %d\n", d_header->type);
			TOUCH_I("size : %d\n", d_header->data_size);
		}
	} else
		d_header->data_size = 0;

	if (interrupt_status_reg & INTERRUPT_MASK_CUSTOM) {
		u8 wake_up_type;

		wake_up_type = ts->ts_data.report.touchInfo.wakeUpType;

		packet_ptr->touchCnt = 0;

		if (wake_up_type == TCI_1) {
			if (ts->lpwg_ctrl.double_tap_enable) {
				get_tci_data(ts, 2);
				send_uevent_lpwg(ts->spi_device, LPWG_DOUBLE_TAP);
			}
		} else if (wake_up_type == TCI_2) {
			if (ts->lpwg_ctrl.password_enable) {
				get_tci_data(ts, ts->pw_data.tap_count);
				send_uevent_lpwg(ts->spi_device, LPWG_PASSWORD);
			}
		} else if (wake_up_type == SWIPE_DOWN) {
			if (get_swipe_data(spi) == 0) {
				ts->pdata->swipe_pwr_ctr = SKIP_PWR_CON;
				if (ts->pdata->lockscreen_stat == 1) {
					send_uevent_lpwg(spi,
						LPWG_SWIPE_DOWN);
					swipe_disable(ts);
				} else {
					ts->pdata->swipe_pwr_ctr =
						WAIT_SWIPE_WAKEUP;
					TOUCH_I("drop SWP(type:%d swp_pwr:%d\n",
					LPWG_SWIPE_DOWN, WAIT_SWIPE_WAKEUP);
				}
			}
		} else if (wake_up_type == SWIPE_UP) {
			if (get_swipe_data(spi) == 0) {
				ts->pdata->swipe_pwr_ctr = SKIP_PWR_CON;
				send_uevent_lpwg(spi, LPWG_SWIPE_UP);
				swipe_disable(ts);
			}
		} else if (wake_up_type == TCI_FAIL_DEBUG) {
			TOUCH_D(DEBUG_BASE_INFO | DEBUG_LPWG,
				"LPWG wakeUpType is TCI DEBUG\n");
		} else {
			TOUCH_D(DEBUG_BASE_INFO | DEBUG_LPWG,
				"LPWG wakeUpType is not support type![%d]\n",
				wake_up_type);
		}
		if ((ts->fail_reason[0] &&
			(wake_up_type == TCI_1 || wake_up_type == TCI_2))
			|| wake_up_type == TCI_FAIL_DEBUG)
			check_tci_debug_result(ts, wake_up_type);
		if (ts->fail_reason[1] &&
			(wake_up_type == SWIPE_DOWN ||
				wake_up_type == SWIPE_UP ||
				wake_up_type == TCI_FAIL_DEBUG))
			print_swipe_debug_result(ts);
	} else if (interrupt_status_reg & INTERRUPT_MASK_ABS0) {
		if (abt_report_ocd) {
			DO_SAFE(sic_spi_read(ts->spi_device, report_base+34,
				(u8 *)d_data_ptr,
				60), error);
			d_data_ptr += 60;
		}

		if (touchInfo->touchCnt > 0 &&
				touchInfo->touchCnt < (MAX_REPORT_FINGER+1)) {
#if 0
			/* read touchData - touchCnt-1 */
			memcpy(&ts->ts_data.report.touchData[0],
				&interrupt_data[2], sizeof(struct T_TouchData));
			if (ts->ts_data.report.touchInfo.touchCnt > 1) {
				DO_SAFE(sic_spi_read(ts->spi_device, report_base+4,
				(u8 *)&ts->ts_data.report.touchData[1],
				sizeof(struct T_TouchData) *
				(ts->ts_data.report.touchInfo.touchCnt-1)),
				error);
			}
#else
			/* read touchData*/
			DO_SAFE(sic_spi_read(ts->spi_device, report_base+1,
					(u8 *)&ts->ts_data.report.touchData[0],
					sizeof(struct T_TouchData) *
					ts->ts_data.report.touchInfo.touchCnt),
				error);
#endif

			t_Data = &ts->ts_data.report.touchData[0];

			/* palm touch case */
			if (t_Data[0].track_id == PALM_ID) {
				if (t_Data[0].event == TOUCHSTS_DOWN) {
					TOUCH_I("Palm Detected : [%d]->[%d]\n",
						ts->is_palm, 1);
					ts->is_palm = 1;
				} else if (t_Data[0].event == TOUCHSTS_UP) {
					TOUCH_I("Palm Released : [%d]->[%d]\n",
						ts->is_palm, 0);
					ts->is_palm = 0;
				}
			} else {
				if (abt_report_point) {
					packet_ptr->touchCnt =
						touchInfo->touchCnt;
					memcpy((u8 *)d_data_ptr,
						(u8 *)t_Data,
						sizeof(struct T_TouchData)*
							(touchInfo->touchCnt));
					d_data_ptr +=
						sizeof(struct T_TouchData)*
							(touchInfo->touchCnt);
				} else
					packet_ptr->touchCnt = 0;

				for (i = 0; i < touchInfo->touchCnt; i++) {
					if ((t_Data[i].event == TOUCHSTS_MOVE ||
							t_Data[i].event ==
							TOUCHSTS_DOWN)
						&& t_Data[i].track_id <
							MAX_REPORT_FINGER) {
						c_data =
						&curr_data->touch[f_index];
						c_data->id =
							t_Data[i].track_id;
						c_data->type =
							t_Data[i].toolType;
						c_data->x = c_data->raw_x =
							t_Data[i].x;
						c_data->y = c_data->raw_y =
							t_Data[i].y;
						c_data->width_major =
							t_Data[i].width_major;
						c_data->width_minor =
							t_Data[i].width_minor;
						c_data->orientation =
							t_Data[i].angle;
						c_data->pressure =
							t_Data[i].byte.pressure;

						curr_data->id_mask |=
							(0x1 << (c_data->id));
						curr_data->total_num++;

						f_index++;
					} else if (t_Data[i].event ==
								TOUCHSTS_UP) {
						touchUpCnt++;

						TOUCH_D(DEBUG_GET_DATA,
						"<%d> type[%d] pos(%4d,%4d) w_m[%2d] w_n[%2d] o[%2d] p[%2d] reported touch_up\n",
						t_Data[i].track_id,
						t_Data[i].toolType,
						t_Data[i].x,
						t_Data[i].y,
						t_Data[i].width_major,
						t_Data[i].width_minor,
						t_Data[i].angle,
						t_Data[i].byte.pressure);
					}
				}

				TOUCH_D(DEBUG_GET_DATA,
					"ID[0x%x] Total_num[%d]\n",
					curr_data->id_mask,
					curr_data->total_num);
			}
		}
	} else
		goto ignore;

	if ((u8 *)d_data_ptr - (u8 *)packet_ptr > 0) {
		do_gettimeofday(&t_stamp);

		frame_num++;

		packet_ptr->type = DEBUG_DATA;
		packet_ptr->mode = abt_report_mode;
		packet_ptr->frame_num = frame_num;
		packet_ptr->timestamp =
				t_stamp.tv_sec*1000000 + t_stamp.tv_usec;

		packet_ptr->flag = 0;
		if (abt_report_point)
			packet_ptr->flag |= 0x1;
		if (abt_report_ocd)
			packet_ptr->flag |= (0x1)<<1;

		abt_ksocket_raw_data_send((u8 *)packet_ptr,
			(u8 *)d_data_ptr - (u8 *)packet_ptr);
	}

	ts_interrupt_clear(ts);
	mutex_unlock(&abt_i2c_comm_lock);
	return NO_ERROR;

ignore:
	ts_interrupt_clear(ts);
	mutex_unlock(&abt_i2c_comm_lock);
	return IGNORE_EVENT;

error:
	ts_interrupt_clear(ts);
	mutex_unlock(&abt_i2c_comm_lock);
	return ERROR;
}
#endif /* USE_ABT_MONITOR_APP */


enum error_type sic_ts_filter(struct spi_device *spi,
	struct touch_data *curr_data, const struct touch_data *prev_data)
{
	return NO_ERROR;
}

enum error_type sic_ts_power(struct spi_device *spi, int power_ctrl)
{
	struct sic_ts_data *ts =
			(struct sic_ts_data *)get_touch_handle(spi);
	int ret;

	TOUCH_TRACE();

	if (ts->pdata->swipe_pwr_ctr == SKIP_PWR_CON) {
		TOUCH_I("%s : Skip power_control (swipe_pwr_ctr = %d)\n",
				__func__, ts->pdata->swipe_pwr_ctr);

		if ((power_ctrl == POWER_OFF) || (power_ctrl == POWER_ON))
			power_state = power_ctrl;

		TOUCH_I("%s : power_state[%d]\n", __func__, power_state);
		return NO_ERROR;
	}

	switch (power_ctrl) {
	case POWER_OFF:
		ts->is_init = 0;
		ts->is_palm = 0;
		TOUCH_I("POWER_OFF\n");
		if (ts->pdata->reset_pin > 0)
			gpio_direction_output(ts->pdata->reset_pin, 0);
		if (ts->pdata->pwr->use_regulator) {
			if (regulator_is_enabled(ts->regulator_vio))
				regulator_disable(ts->regulator_vio);
			if (regulator_is_enabled(ts->regulator_vdd))
				regulator_disable(ts->regulator_vdd);
		}
		usleep(ts->pdata->role->reset_delay*1000);
		break;
	case POWER_ON:
		ts->is_init = 0;
		ts->is_palm = 0;
		TOUCH_I("POWER_ON\n");

		if (ts->pdata->pwr->use_regulator) {
			if (!regulator_is_enabled(ts->regulator_vdd)) {
				ret = regulator_enable(ts->regulator_vdd);
				if (ret != 0)
					TOUCH_E("vdd enable failed");
			}
			if (!regulator_is_enabled(ts->regulator_vio)) {
				ret = regulator_enable(ts->regulator_vio);
				if (ret != 0)
					TOUCH_E("vdd enable failed");
			}
		}
		if (ts->pdata->reset_pin > 0)
			gpio_direction_output(ts->pdata->reset_pin, 1);
		break;
	case POWER_SLEEP:
		TOUCH_I("POWER_SLEEP - SIC Don't Use SleepMode\n");
		break;
	case POWER_WAKE:
		TOUCH_I("POWER_WAKE\n");
		break;
	default:
		break;
	}

	if ((power_ctrl == POWER_OFF) || (power_ctrl == POWER_ON))
		power_state = power_ctrl;

	return NO_ERROR;
}

enum error_type sic_ts_ic_ctrl(struct spi_device *spi,
				u8 code, u32 value, u32 *ret)
{
	struct sic_ts_data *ts
		= (struct sic_ts_data *)get_touch_handle(spi);
	u16 addr = value & 0xFFFF;
	u32 wdata;
	switch (code) {
	case IC_CTRL_READ:
		DO_SAFE(sic_spi_read(spi,
				addr, (u8 *)ret, sizeof(u32)), error);
		break;
	case IC_CTRL_WRITE:
		DO_SAFE(sic_spi_write(spi,
				addr, (u8 *)ret, sizeof(u32)), error);
		break;
	case IC_CTRL_RESET:
		ts->is_init = 0;
		wdata = 0xCACA0010;
		DO_SAFE(sic_spi_write(ts->spi_device, tc_device_ctl,
					(u8 *)&wdata, sizeof(u32)), error);
		break;
	default:
		break;
	}
	return NO_ERROR;
error:
	return ERROR;
}

enum error_type sic_ts_fw_upgrade(struct spi_device *spi,
					struct touch_fw_info *info,
					struct touch_firmware_module *fw)
{
	struct sic_ts_data *ts
		= (struct sic_ts_data *)get_touch_handle(spi);
	int need_upgrade = 0;
	int rc = 0;
	int id_compare = 0, fw_compare = 0;
	char path[256];
	const struct firmware *fw_entry = NULL;
	const u8 *firmware = NULL;
	u32 fw_idx;

	memcpy(path, info->fw_path, sizeof(path));
	if (path != NULL) {

		if (memcmp(path, "erase", 5) == 0) {
			EraseChip(ts);
			return NO_ERROR;
		}

		TOUCH_D(DEBUG_BASE_INFO, "IC_product_id: %s\n",
			ts->fw_info.fw_product_id);
		rc = request_firmware(&fw_entry, path,
			&ts->spi_device->dev);

		TOUCH_D(DEBUG_BASE_INFO | DEBUG_FW_UPGRADE,
			"FW: file[%s]\n", path);
		if (rc != 0) {
			TOUCH_E("request_firmware() failed %d\n", rc);
			goto error;
		}
	} else {
		TOUCH_E("error get fw_path\n");
		goto error;
	}

	firmware = fw_entry->data;
	fw_idx = *((u32 *)&firmware[ts->pdata->fw_ver_addr]);
	memcpy(ts->fw_info.fw_image_version, &firmware[fw_idx], 2);
	fw_idx = *((u32 *)&firmware[ts->pdata->fw_pid_addr]);
	memcpy(ts->fw_info.fw_image_product_id, &firmware[fw_idx], 8);

	if (info->force_upgrade || info->force_upgrade_cat) {
		TOUCH_D(DEBUG_BASE_INFO | DEBUG_FW_UPGRADE,
			"FW: need_upgrade[%d] force[%d] force_cat[%d] path[%s]\n",
			fw->need_upgrade, info->force_upgrade,
			info->force_upgrade_cat, path);
		goto firmware_up;
	}

#if 1 //code for an existing deployment targets due to changing the product_id
	id_compare = 1;
#else
	id_compare = !strncmp(ts->fw_info.fw_product_id,
			ts->fw_info.fw_image_product_id,
			sizeof(ts->fw_info.fw_product_id));
#endif

	if (ts->fw_info.fw_product_id[0] == 0) {
		id_compare = 1;
		TOUCH_I("product_id is empty, fw write\n");
	}

	TOUCH_D(DEBUG_BASE_INFO, "product_id[%s(ic):%s(fw)]\n ",
		ts->fw_info.fw_product_id, ts->fw_info.fw_image_product_id);

	if (memcmp(ts->fw_info.fw_product_id, ts->fw_info.fw_image_product_id, 8) != 0)
		msleep(200); // LG4945 200ms waiting if flash is empty.

	fw_compare = compare_fw_version(spi, info);
	need_upgrade = fw->need_upgrade && id_compare && fw_compare;

	if (need_upgrade) {
		TOUCH_D(DEBUG_BASE_INFO | DEBUG_FW_UPGRADE,
			"FW Upgrade[%d] = id[%d] & fw[%d] & need_upgrade[%d]\n",
			need_upgrade, id_compare, fw_compare, fw->need_upgrade);
		goto firmware_up;
	}

	release_firmware(fw_entry);
	return NO_UPGRADE;

firmware_up:
	ts->is_probed = 0;
	ts->is_init = 0; /* During upgrading, interrupt will be ignored. */
	DO_SAFE(SicFirmwareUpgrade(ts, fw_entry), firmware_up_retry);
	release_firmware(fw_entry);
	return NO_ERROR;

firmware_up_retry:
	TOUCH_D(DEBUG_BASE_INFO, "firmware_up_retry\n");
	msleep(200); // to download code from flash to LG4945
	if(SicFirmwareUpgrade(ts, fw_entry) < 0){
		release_firmware(fw_entry);
		goto error;
	}
	release_firmware(fw_entry);
	return NO_ERROR;

error:
	return ERROR;
}

enum error_type  sic_ts_notify(struct spi_device *spi, u8 code, u32 value)
{
	struct sic_ts_data *ts
		= (struct sic_ts_data *)get_touch_handle(spi);

	switch (code) {
	case NOTIFY_TA_CONNECTION:
		queue_delayed_work(touch_wq,
			&ts->work_charger, msecs_to_jiffies(0));
		break;
	case NOTIFY_TEMPERATURE_CHANGE:
		break;
	case NOTIFY_PROXIMITY:
		break;
	case NOTIFY_HALL_IC:
		break;
	default:
		break;
	}

	return NO_ERROR;
}

enum error_type sic_ts_suspend(struct spi_device *spi)
{
	struct sic_ts_data *ts
		= (struct sic_ts_data *)get_touch_handle(spi);

	TOUCH_D(DEBUG_BASE_INFO, "%s\n", __func__);

	atomic_set(&ts->lpwg_ctrl.is_suspend, TC_STATUS_SUSPEND);
	DO_SAFE(lpwg_update_all(ts), error);

	return NO_ERROR;
error:
	return ERROR;
}

enum error_type sic_ts_resume(struct spi_device *spi)
{
	struct sic_ts_data *ts
		= (struct sic_ts_data *)get_touch_handle(spi);

#if USE_ABT_MONITOR_APP
	sic_spi_write(spi, CMD_RAW_DATA_COMPRESS_WRITE,
						(u8 *)&abt_compress_flag,
						sizeof(u32));
#endif
	atomic_set(&ts->lpwg_ctrl.is_suspend, TC_STATUS_RESUME);

	return NO_ERROR;
}

enum error_type sic_ts_lpwg(struct spi_device *spi,
	u32 code, int64_t value, struct point *data)
{
	struct sic_ts_data *ts =
		(struct sic_ts_data *)get_touch_handle(spi);

	u8 mode = ts->lpwg_ctrl.lpwg_mode;

	switch (code) {
	case LPWG_READ:
		memcpy(data, ts->pw_data.data,
			sizeof(struct point) * ts->pw_data.data_num);
		data[ts->pw_data.data_num].x = -1;
		data[ts->pw_data.data_num].y = -1;
		/* '-1' should be assigned to the last data.
		 Each data should be converted to LCD-resolution.*/
		memset(ts->pw_data.data, -1,
			sizeof(struct point)*ts->pw_data.data_num);
		break;
	case LPWG_ENABLE:
		if (!atomic_read(&ts->lpwg_ctrl.is_suspend))
			ts->lpwg_ctrl.lpwg_mode = value;
		break;
	case LPWG_LCD_X:
	case LPWG_LCD_Y:
		/* If touch-resolution is not same with LCD-resolution,
		 position-data should be converted to LCD-resolution.*/
		break;
	case LPWG_ACTIVE_AREA_X1:
		/*
		wdata = value;
		DO_SAFE(sic_spi_write(ts->spi_device, CMD_ABT_ACT_AREA_X1_CTRL_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		*/
		break;
	case LPWG_ACTIVE_AREA_X2:
		/*
		wdata = value;
		DO_SAFE(sic_spi_write(ts->spi_device, CMD_ABT_ACT_AREA_X2_CTRL_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		*/
		break;
	case LPWG_ACTIVE_AREA_Y1:
		/*
		wdata = value;
		DO_SAFE(sic_spi_write(ts->spi_device, CMD_ABT_ACT_AREA_Y1_CTRL_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		*/
		break;
	case LPWG_ACTIVE_AREA_Y2:
		/*
		wdata = value;
		DO_SAFE(sic_spi_write(ts->spi_device, CMD_ABT_ACT_AREA_Y2_CTRL_WRITE,
					(u8 *)&wdata, sizeof(u32)), error);
		*/
		break;
	case LPWG_TAP_COUNT:
		ts->pw_data.tap_count = value;
		break;
	case LPWG_LENGTH_BETWEEN_TAP:
		break;
	case LPWG_EARLY_SUSPEND:
		break;
	case LPWG_SENSOR_STATUS:
		break;
	case LPWG_DOUBLE_TAP_CHECK:
		ts->pw_data.double_tap_check = value;
		break;
	case LPWG_REPLY:
		if (ts->pdata->role->use_lpwg_all) {
			if (atomic_read(&ts->lpwg_ctrl.is_suspend) == 0) {
				TOUCH_I("%s : screen on\n", __func__);
				break;
			}
		} else {
			if (ts->lpwg_ctrl.password_enable && !value)
				DO_SAFE(lpwg_control(ts, mode), error);
		}
		break;
	case LPWG_UPDATE_ALL:
	{
		int *v = (int *) value;
		int mode = *(v + 0);
		int screen = *(v + 1);
		int sensor = *(v + 2);
		int qcover = *(v + 3);

		ts->lpwg_ctrl.lpwg_mode = mode;
		ts->lpwg_ctrl.screen = screen;
		ts->lpwg_ctrl.sensor = sensor;
		ts->lpwg_ctrl.qcover = qcover;

		TOUCH_I(
			"LPWG_UPDATE_ALL: mode[%d], screen[%s], sensor[%s], qcover[%s]\n",
			ts->lpwg_ctrl.lpwg_mode,
			ts->lpwg_ctrl.screen ? "ON" : "OFF",
			ts->lpwg_ctrl.sensor ? "FAR" : "NEAR",
			ts->lpwg_ctrl.qcover ? "CLOSE" : "OPEN");
		DO_SAFE(lpwg_update_all(ts), error);
		break;
	}
	break;
	default:
		break;
	}

	return NO_ERROR;
error:
	TOUCH_D(DEBUG_BASE_INFO | DEBUG_LPWG, "%s error\n", __func__);
	return ERROR;
}

static void sic_ts_ime_status(struct spi_device *spi, int ime_status)
{
	u32 wdata;
	wdata = ime_status;

	DO_SAFE(sic_spi_write(spi, CMD_ABT_IME_INFO_WRITE,
			(u8 *)&wdata, sizeof(u32)), error);
	TOUCH_I("IME status Write [%d]\n", wdata);
	return;

error:
	TOUCH_E("IME status Write Fail\n");
	return;
}

static void sic_toggle_swipe(struct spi_device *spi)
{
	return;
}


enum window_status sic_ts_check_crack(struct spi_device *spi)
{
	TOUCH_TRACE();
	return NO_CRACK;
}

static const struct attribute_group sic_ts_attribute_group = {
	.attrs = sic_ts_attribute_list,
};

enum error_type sic_ts_shutdown(struct spi_device *spi)
{

	TOUCH_TRACE();

	return NO_ERROR;
}

static void sic_ts_incoming_call(struct spi_device *spi, int value)
{
	TOUCH_TRACE();
}

static int sic_ts_register_sysfs(struct kobject *k)
{
	return sysfs_create_group(k, &sic_ts_attribute_group);
}

static void sic_ts_set_touch_mode(struct spi_device *spi, int lcd_mode)
{
	TOUCH_TRACE();
	TOUCH_I("%s\n", __func__);
	set_tc_driving(spi, lcd_mode);
}
struct touch_device_driver sic_ts_driver = {
	.probe			= sic_ts_probe,
	.remove			= sic_ts_remove,
	.shutdown		= sic_ts_shutdown,
	.suspend		= sic_ts_suspend,
	.resume			= sic_ts_resume,
	.init			= sic_ts_init,
	.data			= sic_ts_get_data,
	.filter			= sic_ts_filter,
	.power			= sic_ts_power,
	.ic_ctrl		= sic_ts_ic_ctrl,
	.fw_upgrade		= sic_ts_fw_upgrade,
	.notify			= sic_ts_notify,
	.lpwg			= sic_ts_lpwg,
	.ime_drumming		= sic_ts_ime_status,
	.toggle_swipe		= sic_toggle_swipe,
	.inspection_crack	= sic_ts_check_crack,
	.register_sysfs		= sic_ts_register_sysfs,
	.incoming_call		= sic_ts_incoming_call,
	.touch_mode		= sic_ts_set_touch_mode,
	/* Font */
	.etw_onoff	= ext_watch_onoff,
	.etw_shutdown	= ext_watch_shutdown,
	.etw_font_download		= ext_watch_font_download,
	.etw_font_dump		= ext_watch_font_dump,
	.etw_synctime			= ext_watch_set_current_time,
	.etw_gettime			= ext_watch_get_current_time,
	.etw_set_postion			= ext_watch_set_position,
	.etw_set_mode			= ext_watch_set_mode,
	.etw_get_cfg			= ext_watch_get_cfg,
	.etw_set_cfg			= ext_watch_set_cfg,
};

#if USE_ABT_MONITOR_APP
void sic_set_get_data_func(u8 mode)
{
	if (mode == 1) {
		sic_ts_driver.data = sic_ts_get_data_debug_mode;
		TOUCH_D(DEBUG_BASE_INFO,
			"(%s)change get_data func(sic_ts_get_data_debug_mode)\n",
			__func__);
	} else {
		sic_ts_driver.data = sic_ts_get_data;
		TOUCH_D(DEBUG_BASE_INFO,
			"(%s)change get_data func(sic_ts_get_data)\n",
			__func__);
	}
}
#endif /* USE_ABT_MONITOR_APP */


static struct of_device_id sic_match_table[] = {
	{ .compatible = "lgesic,lg4945",},
	{ },
};

static void async_sic_touch_init(void *data, async_cookie_t cookie)
{
	int panel_type = lge_get_panel();

	TOUCH_D(DEBUG_BASE_INFO, "panel type is %d\n", panel_type);

	if (panel_type != 4)
		return;

	TOUCH_D(DEBUG_BASE_INFO, "async_sic_touch_init START!!!!!!\n");
	touch_driver_register(&sic_ts_driver, sic_match_table);
	return;
}
static int __init sic_touch_init(void)
{
	TOUCH_TRACE();
	async_schedule(async_sic_touch_init, NULL);
	return 0;
}

static void __exit sic_touch_exit(void)
{
	TOUCH_TRACE();
	touch_driver_unregister();
}

module_init(sic_touch_init);
module_exit(sic_touch_exit);

MODULE_AUTHOR("sunkwi.kim@lge.com");
MODULE_DESCRIPTION("LGE Touch Driver");
MODULE_LICENSE("GPL");
