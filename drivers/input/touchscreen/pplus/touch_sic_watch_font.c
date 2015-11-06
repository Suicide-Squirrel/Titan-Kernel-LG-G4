/* touch_sic_watch_font.c
 *
 * Copyright (C) 2015 LGE.
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
#include <linux/syscalls.h>
#include <linux/gpio.h>
#include <linux/file.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/interrupt.h>
#include "lge_touch_core_spi.h"
#include "touch_sic_watch_font.h"

static int ext_watch_set_mcs_ctrl(struct spi_device *spi, u8 enable)
{
	u8 mcs_value = 0;

	if (enable)
		mcs_value = 0xAC;
	else
		mcs_value = 0xCA;

	DO_SAFE(sic_spi_write(spi, EXT_WATCH_MCS_ACCESS, (u8 *)&mcs_value, sizeof(u8)), error);
	//TOUCH_I("MCS Access onoff : %X  \n", mcs_value);
	return NO_ERROR;
error:
	TOUCH_E("MCS Access Fail\n");
	return ERROR;
}

enum error_type ext_watch_get_mode(struct spi_device *spi, struct ext_watch_cfg *cfg)
{
	u8 *ptr = NULL;
	u16 offset = EXT_WATCH_CTRL;
	u16 idx = 0;
	u32 size = sizeof(u8);

	TOUCH_I("%s start \n", __func__);

	ext_watch_set_mcs_ctrl(spi, EXT_WATCH_MCS_ENABLE);
	
	ptr = (u8 *)(&cfg->mode.watch_ctrl);
	offset = EXT_WATCH_CTRL;
	DO_SAFE(sic_spi_read(spi,  offset++, &ptr[0], size), error);
	DO_SAFE(sic_spi_read(spi,  offset++, &ptr[1], size), error);
	TOUCH_I("Get Offet[%X] watch_ctrl %02X %02X\n",
		EXT_WATCH_CTRL, ptr[0], ptr[1]);

	ptr = (u8 *)(&cfg->mode.watch_area);
	offset = EXT_WATCH_AREA;
	DO_SAFE(sic_spi_read(spi,  offset++, &ptr[0], size), error);
	DO_SAFE(sic_spi_read(spi,  offset++, &ptr[1], size), error);
	DO_SAFE(sic_spi_read(spi,  offset++, &ptr[2], size), error);
	TOUCH_I("Get Offet[%X] watch_area %02X %02X %02X\n",
		EXT_WATCH_AREA, ptr[0], ptr[1], ptr[2]);

	ptr = (u8 *)(&cfg->mode.blink_area);
	offset = EXT_WATCH_BLINK_AREA;
	DO_SAFE(sic_spi_read(spi,  offset++, &ptr[0], size), error);
	DO_SAFE(sic_spi_read(spi,  offset++, &ptr[1], size), error);
	DO_SAFE(sic_spi_read(spi,  offset++, &ptr[2], size), error);
	TOUCH_I("Get Offet[%X] blink_area %02X %02X %02X\n",
		EXT_WATCH_BLINK_AREA, ptr[0], ptr[1], ptr[2]);

	ptr = (u8 *)(&cfg->mode.grad);
	offset = EXT_WATCH_GRAD;
	DO_SAFE(sic_spi_read(spi,  offset++, &ptr[0], size), error);
	DO_SAFE(sic_spi_read(spi,  offset++, &ptr[1], size), error);
	DO_SAFE(sic_spi_read(spi,  offset++, &ptr[2], size), error);
	DO_SAFE(sic_spi_read(spi,  offset++, &ptr[3], size), error);
	DO_SAFE(sic_spi_read(spi,  offset++, &ptr[4], size), error);
	DO_SAFE(sic_spi_read(spi,  offset++, &ptr[5], size), error);
	TOUCH_I("Get Offet[%X] grad %02X %02X %02X %02X %02X %02X\n",
		EXT_WATCH_GRAD, ptr[0], ptr[1], ptr[2], ptr[3], ptr[4], ptr[5]);

	offset = EXT_WATCH_LUT;
	for (idx = 0; idx < EXT_WATCH_LUT_NUM; idx++) {
		ptr = (u8 *)(&cfg->mode.lut[idx]);
		DO_SAFE(sic_spi_read(spi,  offset++, &ptr[0], size), error);
		DO_SAFE(sic_spi_read(spi,  offset++, &ptr[1], size), error);
		DO_SAFE(sic_spi_read(spi,  offset++, &ptr[2], size), error);
		TOUCH_I("Get Offet[%X] LUT[%d] : B[%02X] G[%02X] R[%02X] \n",
			offset - 3, idx, ptr[0],ptr[1],ptr[2]);
	}

	ext_watch_set_mcs_ctrl(spi, EXT_WATCH_MCS_DISABLE);
	TOUCH_I("%s end \n", __func__);
	return NO_ERROR;
	
error:
	TOUCH_I("%s failed \n", __func__);
	return ERROR;
}

enum error_type ext_watch_set_mode(struct spi_device *spi, struct ext_watch_cfg *cfg)
{
	u8 *ptr = NULL;
	u16 offset = EXT_WATCH_CTRL;
	u16 idx = 0;
	u32 size = sizeof(u8);

	TOUCH_I("%s \n", __func__);

	ext_watch_set_mcs_ctrl(spi, EXT_WATCH_MCS_ENABLE);
	
	ptr = (u8 *)(&cfg->mode.watch_ctrl);
	offset = EXT_WATCH_CTRL;
	DO_SAFE(sic_spi_write(spi,  offset++, &ptr[0], size), error);
	DO_SAFE(sic_spi_write(spi,  offset++, &ptr[1], size), error);
	#if 0
	TOUCH_I("Set Offet[%X] watch_ctrl %02X %02X\n",
		EXT_WATCH_CTRL, ptr[0], ptr[1]);
	#endif

	ptr = (u8 *)(&cfg->mode.watch_area);
	offset = EXT_WATCH_AREA;
	DO_SAFE(sic_spi_write(spi,  offset++, &ptr[0], size), error);
	DO_SAFE(sic_spi_write(spi,  offset++, &ptr[1], size), error);
	DO_SAFE(sic_spi_write(spi,  offset++, &ptr[2], size), error);
	#if 0
	TOUCH_I("Set Offet[%X] watch_area %02X %02X %02X\n",
		EXT_WATCH_AREA, ptr[0], ptr[1], ptr[2]);
	#endif

	ptr = (u8 *)(&cfg->mode.blink_area);
	offset = EXT_WATCH_BLINK_AREA;
	DO_SAFE(sic_spi_write(spi,  offset++, &ptr[0], size), error);
	DO_SAFE(sic_spi_write(spi,  offset++, &ptr[1], size), error);
	DO_SAFE(sic_spi_write(spi,  offset++, &ptr[2], size), error);
	#if 0
	TOUCH_I("Set Offet[%X] blink_area %02X %02X %02X\n",
		EXT_WATCH_BLINK_AREA, ptr[0], ptr[1], ptr[2]);
	#endif

	ptr = (u8 *)(&cfg->mode.grad);
	offset = EXT_WATCH_GRAD;
	DO_SAFE(sic_spi_write(spi,  offset++, &ptr[0], size), error);
	DO_SAFE(sic_spi_write(spi,  offset++, &ptr[1], size), error);
	DO_SAFE(sic_spi_write(spi,  offset++, &ptr[2], size), error);
	DO_SAFE(sic_spi_write(spi,  offset++, &ptr[3], size), error);
	DO_SAFE(sic_spi_write(spi,  offset++, &ptr[4], size), error);
	DO_SAFE(sic_spi_write(spi,  offset++, &ptr[5], size), error);
	#if 0
	TOUCH_I("Set Offet[%X] grad %02X %02X %02X %02X %02X %02X\n",
		EXT_WATCH_GRAD, ptr[0], ptr[1], ptr[2], ptr[3], ptr[4], ptr[5]);
	#endif

	offset = EXT_WATCH_LUT;
	for (idx = 0; idx < EXT_WATCH_LUT_NUM; idx++) {
		ptr = (u8 *)(&cfg->mode.lut[idx]);
		DO_SAFE(sic_spi_write(spi,  offset++, &ptr[0], size), error);
		DO_SAFE(sic_spi_write(spi,  offset++, &ptr[1], size), error);
		DO_SAFE(sic_spi_write(spi,  offset++, &ptr[2], size), error);
		#if 0
		TOUCH_I("Set Offet[%X] LUT[%d] : B[%02X] G[%02X] R[%02X] \n",
			offset - 3, idx, ptr[0],ptr[1],ptr[2]);
		#endif
	}

	ext_watch_set_mcs_ctrl(spi, EXT_WATCH_MCS_DISABLE);
	#if 0
	TOUCH_I("%s end \n", __func__);
	#endif
	return NO_ERROR;
	
error:
	TOUCH_I("%s failed \n", __func__);
	return ERROR;
}

enum error_type ext_watch_set_current_time(struct spi_device *spi, struct ext_watch_cfg *cfg)
{
	u32 rtc_ctrl = EXT_WATCH_RTC_STOP;
	u16 rtc_count = 305;		/* for time, 1 /rtc_ecnt */


	cfg->time.rtc_ecnt = 32764;
	cfg->time.rtc_sctcnt = (int)((cfg->time.rtc_sctcnt * rtc_count) / 10);

	DO_SAFE(sic_spi_write(spi, EXT_WATCH_RTC_RUN,
		(u8 *)&rtc_ctrl, sizeof(u32)), error);

	DO_SAFE(sic_spi_write(spi, EXT_WATCH_RTC_SCT,
		(u8 *)&cfg->time.rtc_sct, sizeof(u32)), error);

	DO_SAFE(sic_spi_write(spi, EXT_WATCH_RTC_SCTCNT,
		(u8 *)&cfg->time.rtc_sctcnt, sizeof(u32)), error);

	rtc_ctrl = cfg->time.rtc_ecnt & 0xFFFF;
	DO_SAFE(sic_spi_write(spi, EXT_WATCH_RTC_ECNT,
		(u8 *)&rtc_ctrl, sizeof(u32)), error);

	rtc_ctrl = EXT_WATCH_RTC_START;
	DO_SAFE(sic_spi_write(spi, EXT_WATCH_RTC_RUN,
		(u8 *)&rtc_ctrl, sizeof(u32)), error);

	TOUCH_I("%s : %02d:%02d:%02d CLK[%d Hz]\n", __func__,
		cfg->time.rtc_sct.hour, cfg->time.rtc_sct.min,
		cfg->time.rtc_sct.sec, cfg->time.rtc_ecnt);

	return NO_ERROR;

error:
	TOUCH_I("%s Fail \n", __func__);
	return ERROR;
}

enum error_type ext_watch_get_current_time(struct spi_device *spi, struct ext_watch_cfg *cfg)
{
	DO_SAFE(sic_spi_read(spi, EXT_WATCH_RTC_CTST,
		(u8 *)&cfg->time.rtc_ctst, sizeof(u32)), error);

	TOUCH_I("%s : %02d:%02d:%02d\n", __func__,
		cfg->time.rtc_ctst.hour, cfg->time.rtc_ctst.min, cfg->time.rtc_ctst.sec);

	return NO_ERROR;

error:
	TOUCH_I("%s Fail \n", __func__);
	return ERROR;
}

enum error_type ext_watch_get_position(struct spi_device *spi, struct ext_watch_cfg *cfg)
{
	u8 *ptr = (u8 *)(&cfg->position);
	struct ext_watch_status_cfg status_cfg;

	TOUCH_I("%s start \n", __func__);

	DO_SAFE(sic_spi_read(spi, EXT_WATCH_POSITION_R, ptr, sizeof(u32) * 3), error);
	TOUCH_I("Get Hour Position [%d][%d] \n",
		cfg->position.h10x_pos, cfg->position.h1x_pos);
	TOUCH_I("Get Min Position [%d][%d] \n",
		cfg->position.m10x_pos, cfg->position.m1x_pos);
	TOUCH_I("Get Colon Position [%d] \n", cfg->position.clx_pos);

	DO_SAFE(sic_spi_read(spi, EXT_WATCH_SATATE, (u8 *)&status_cfg, sizeof(u32)), error);
	cfg->position.zero_disp = status_cfg.zero_en;
	cfg->position.h24_en = status_cfg.en_24;
	cfg->position.clock_disp_mode = status_cfg.disp_mode;
	cfg->position.bhprd = status_cfg.bhprd;

	TOUCH_I("Get Zero Display [%d] \n", cfg->position.zero_disp);
	TOUCH_I("Get 24H Mode [%d] \n", cfg->position.h24_en);
	TOUCH_I("Get Clock Mode [%d] \n", cfg->position.clock_disp_mode);
	TOUCH_I("Get Blink period [%d] \n", cfg->position.bhprd);
	TOUCH_I("Get Current Watch[%d] \n", status_cfg.step);
	TOUCH_I("Get Watch Enable[%d] \n", status_cfg.en);

	if ( cfg->position.clock_disp_mode )
		TOUCH_I("Get Current Time[%02d][%02d] \n",
			status_cfg.cur_min, status_cfg.cur_sec);
	else
		TOUCH_I("Get Current Time[%02d][%02d][%03d] \n", status_cfg.cur_hour,
			status_cfg.cur_min, cfg->time.rtc_sctcnt);

	return NO_ERROR;
error:
	TOUCH_I("%s Fail \n", __func__);
	return ERROR;
}

enum error_type ext_watch_set_position(struct spi_device *spi, struct ext_watch_cfg *cfg)
{
	u8 *ptr = (u8 *)(&cfg->position);	

	TOUCH_I("%s \n", __func__);

	DO_SAFE(sic_spi_write(spi, EXT_WATCH_POSITION, ptr, sizeof(u32) * 5), error);
	#if 0
	TOUCH_I("Set Hour Position [%d][%d] \n",
		cfg->position.h10x_pos, cfg->position.h1x_pos);
	TOUCH_I("Set Min Position [%d][%d] \n",
		cfg->position.m10x_pos, cfg->position.m1x_pos);
	TOUCH_I("Set Colon Position [%d] \n", cfg->position.clx_pos);
	TOUCH_I("Set Zero Display [%d] \n", cfg->position.zero_disp);
	TOUCH_I("Set 24H Mode [%d] \n", cfg->position.h24_en);
	TOUCH_I("Set Display Mode [%d] \n", cfg->position.clock_disp_mode);
	TOUCH_I("Set Clock Mode [%d] \n", cfg->position.clock_disp_mode);
	TOUCH_I("Set Blink period [%d] \n", cfg->position.bhprd);
	#endif

	return NO_ERROR;
error:
	TOUCH_I("%s Fail \n", __func__);
	return ERROR;
}


/* ext_watch_shutdown
 *
 * 'power state' can has only  'SLEEP' or 'WAKE' (not 'ON' or 'OFF')
 */
enum error_type ext_watch_shutdown(struct spi_device *spi, u8 onoff)
{
	u32 rtc_ctrl = EXT_WATCH_RTC_STOP;
	TOUCH_I("%s start \n", __func__ );

	if ( onoff == EXT_WATCH_RTC_START )
		rtc_ctrl = EXT_WATCH_RTC_START;

	DO_SAFE(sic_spi_write(spi, EXT_WATCH_RTC_RUN,
		(u8 *)&rtc_ctrl, sizeof(u32)), error);

	return NO_ERROR;
error:
	TOUCH_I("%s Fail \n", __func__);
	return ERROR;

}

/* ext_watch_set_onoff
 *
 * 'power state' can has only 'ON' or 'OFF'. (not 'SLEEP' or 'WAKE') 
 */
enum error_type ext_watch_onoff(struct spi_device *spi, u32 onoff)
{
	TOUCH_I("%s %d\n", __func__, onoff);

	DO_SAFE(sic_spi_write(spi, EXT_WATCH_RTC_CTRL,
			(u8 *)&onoff, sizeof(u32)), error);

	return NO_ERROR;
error:
	TOUCH_I("%s Fail \n", __func__);
	return ERROR;
}

enum error_type ext_watch_font_download(struct spi_device *spi, char *font_data)
{
	u32 font_sel = 0;
	u32 font_data_offset = 0;
	u32 wdata = 0;
	u32 size = 0;

	TOUCH_I("%s start\n", __func__);
	// Font memory access enable
	wdata = 1;
	DO_SAFE(sic_spi_write(spi, EXT_WATCH_FONT_ACC_EN,
		(u8 *)&wdata, sizeof(u32)), error);

	size = sizeof(u32) * EXT_WATCH_FONT_NUM_SIZE;

	for (font_sel = 0; font_sel < 10; font_sel++) {
		// Font select : '0' ~ '9'
		font_data_offset = font_sel * size;
		DO_SAFE(sic_spi_font_write(spi, (u8 *)&font_sel,
			font_data+font_data_offset, size), error);
	}

	// Font select : ':'
	font_data_offset = font_sel * size;
	size = sizeof(u32) * EXT_WATCH_FONT_CHAR_SIZE;
	DO_SAFE(sic_spi_font_write(spi, (u8 *)&font_sel,
		font_data+font_data_offset, size), error);

	// Font memory access disable
	wdata = 0;
	DO_SAFE(sic_spi_write(spi, EXT_WATCH_FONT_ACC_EN,
		(u8 *)&wdata, sizeof(u32)), error);

	TOUCH_I("%s done\n", __func__);
	return NO_ERROR;

error:
	TOUCH_I("%s Fail\n", __func__);
	return ERROR;
}

enum error_type ext_watch_font_dump(struct spi_device *spi, char *font_dump)
{
	u32 font_sel = 0;
	u32 font_data_offset = 0;
	u32 wdata = 0;
	u32 size = 0;

	TOUCH_I("%s start\n", __func__);
	// Font memory access enable
	wdata = 1;
	DO_SAFE(sic_spi_write(spi, EXT_WATCH_FONT_ACC_EN,
		(u8 *)&wdata, sizeof(u32)), error);

	size = sizeof(u32) * EXT_WATCH_FONT_NUM_SIZE;

	for (font_sel = 0; font_sel < 10; font_sel++) {
		// Font select : '0' ~ '9'
		font_data_offset = font_sel * size;
		DO_SAFE(sic_spi_font_read(spi, (u8 *)&font_sel,
			font_dump+font_data_offset, size), error);
	}

	// Font select : ':'
	font_data_offset = font_sel * size;
	size = sizeof(u32) * EXT_WATCH_FONT_CHAR_SIZE;
	DO_SAFE(sic_spi_font_read(spi, (u8 *)&font_sel,
		font_dump+font_data_offset, size), error);

	// Font memory access disable
	wdata = 0;
	DO_SAFE(sic_spi_write(spi, EXT_WATCH_FONT_ACC_EN,
		(u8 *)&wdata, sizeof(u32)), error);

	TOUCH_I("%s done\n", __func__);
	return NO_ERROR;

error:
	TOUCH_I("%s Fail\n", __func__);
	return ERROR;
}

enum error_type ext_watch_get_cfg(struct spi_device *spi, struct ext_watch_cfg *cfg)
{
	TOUCH_I("%s \n", __func__);

	DO_IF(ext_watch_get_mode(spi, cfg) != 0, error);
	DO_IF(ext_watch_get_position(spi, cfg) != 0, error);
	DO_IF(ext_watch_get_current_time(spi,cfg)!= 0, error);

	return NO_ERROR;
error:
	TOUCH_I("%s Fail \n", __func__);
	return ERROR;
}

enum error_type ext_watch_set_cfg(struct spi_device *spi,struct ext_watch_cfg *cfg)
{
	TOUCH_I("%s \n", __func__);

	cfg->mode.watch_ctrl.alpha = 1; // bypass foreground
	DO_IF(ext_watch_set_mode(spi, cfg) != 0, error);
	DO_IF(ext_watch_set_position(spi, cfg) != 0, error);

	return NO_ERROR;
error:
	TOUCH_I("%s Fail \n", __func__);
	return ERROR;
}

