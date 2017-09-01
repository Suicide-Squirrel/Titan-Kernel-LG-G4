/* ext_watch.h
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

#ifndef EXT_WATCH_H
#define EXT_WATCH_H

#define FONT_DATA_SIZE	(13280 * 4)

/****************************************************************************
* Defines
****************************************************************************/
#define EXT_WATCH_LUT_MAX               7

enum {
	NOTSURPPORT=0,
	SURPPORT,
};

/****************************************************************************
* CAPABILITY QUERY
****************************************************************************/
#pragma pack(push, 1)
struct ExtWatchFontDataQuery{
	bool	Font_supported;		// 0: not supported, 1: supported
	u8	max_font_x_size;	// 1~9 number X max size.
	u8	max_font_y_size;	// 1~9 number Y max size.
	u8	max_cln_x_size;		// ":" X max size. (ex. 23:47)
	u8	max_cln_y_size;		// ":" Y max size. (ex. 23:47)
};
#pragma pack(pop)

struct ExtWatchFontPositionQuery{
	bool	vertical_position_supported;	// 0: not supported, 1: supported
	bool	horizontal_position_supported;	// 0: not supported, 1: supported
};

struct ExtWatchFontTimeQuery{
	bool	h24_supported;	// 0: not supported, 1: supported
	bool	AmPm_supported;	// 0: not supported, 1: supported
};

#pragma pack(push, 1)
struct ExtWatchFontColorQuery{
	u8	max_num;		// The number of LUT
	bool	LUT_supported;		// 0: not supported, 1: supported
	bool	alpha_supported;	// 0: not supported, 1: supported
	bool	gradation_supported;	// 0: not supported, 1: supported
};
#pragma pack(pop)

#pragma pack(push, 1)
struct ExtWatchFontEffectQuery{
	bool	zero_supported;	// 0: display off, 1: display on
	u8	blink_type;	// 0: blink disable, 1: 500ms, 2: 1sec, 3: 2sec.
};
#pragma pack(pop)

/****************************************************************************
* FONT MODE SETTING
****************************************************************************/

struct ExtWatchFontEffectBlinkConfig
{
	u32	blink_type;	// 0: blink disable, 1: 500ms, 2: 1sec, 3: 2sec.
	u32	bstartx;	// blink startx position. watstartx <= bstartx, bstartx <= bendx
	u32	bendx;		// blink end position. bendx <= watendx
};

struct ExtWatchFontEffectConfig
{
	u32	len;
	u32	watchon;		// 0: watch off, 1: watch on
	u32	h24_en;			// 0: 12 hour display, 1: 24 hour display
	u32	zero_disp;		// 0: display off, 1: display on
	u32	clock_disp_type;	// 0: hour and min display mode, 1: min and sec display mode
	struct ExtWatchFontEffectBlinkConfig	blink;	//for blink effect
};

struct ExtWatchFontLUTConfig
{
	/* LUT */
	u32	RGB_blue;
	u32	RGB_green;
	u32	RGB_red;
};

struct ExtWatchFontPropertyConfig
{
	u32	len;
	u32	max_num;		// The number of LUT
	struct ExtWatchFontLUTConfig	LUT[EXT_WATCH_LUT_MAX];
};

struct ExtWatchFontPostionConfig
{
	u32	len;
	u32	watstartx;	// watch start postion. 400 <= watstartx, watstartx <= watendx
	u32	watendx;	// watch end positon. watendx <= 1440
	u32	h1x_pos;	// 1 ~ 9hour position
	u32	h10x_pos;	// 10, 20 hour position
	u32	m1x_pos;	// 1 ~ 9min position
	u32	m10x_pos;	// 10 ~ 50 min position
	u32	clx_pos;	// 1 ~ 60 second position
};

struct ExtWatchTimeSyncConfig //to sync with AP's current time
{
	u32	len;
	u32	rtc_cwhour;	// for hour
	u32	rtc_cwmin;	// for min
	u32	rtc_cwsec;	// for sec
	u32	rtc_cwmilli;	// for millisecond
};
struct ExtWatchFontDataConfig
{
	u8	*Data;		// Font Data (53120 bytes)
};

#endif //EXT_WATCH_H
