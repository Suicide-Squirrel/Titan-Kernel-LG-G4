/* touch_sic_watch_font.h
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

#ifndef TOUCH_SIC_WATCH_FONT_H
#define TOUCH_SIC_WATCH_FONT_H

#define EXT_WATCH_FONT_ACC_EN		0xC010	/* Font memory access enable : 0 : disable, 1 : enable */
#define EXT_WATCH_FONT_SEL		0xD07A	/* 4'h 0~9: font '0'~'9', 4'hA : font ':' */
#define EXT_WATCH_FONT_OFFSET_ADDR	0xD800	/* 0~9:D800-DCFF, A(:):D800-D9DF */
#define EXT_WATCH_MCS_ACCESS		0xF600	/* mcs_ctrl : Watch configuration register access enable :Enable - 0xAC, Disable - 0xCA */

#define EXT_WATCH_CTRL			0x8A00
#define EXT_WATCH_AREA			0x8A02
#define EXT_WATCH_BLINK_AREA		0x8A05
#define EXT_WATCH_GRAD			0x8A08
#define EXT_WATCH_LUT			0x8A0E

#define EXT_WATCH_RTC_CTRL		0xD027	/* Watch display off : 0, on : 1*/
#define EXT_WATCH_RTC_SCT		0xD08B	/* Synchronous current time */
#define EXT_WATCH_RTC_SCTCNT		0xD08C	/* Synchronous current time for milesecound*/
#define EXT_WATCH_RTC_DIT		0xD08E	/* Target time for occurring date change interrupt */
#define EXT_WATCH_RTC_CTST		0xD092	/* Current time */
#define EXT_WATCH_RTC_ECNT		0xD093	/* end count */
#define EXT_WATCH_HOUR_DISP		0xC014	/* 0bit : zero_disp, 1bit : h24_en, 2bit : clock_disp_mode */
#define EXT_WATCH_BLINK_PRD		0xC015	/* Stop : 0x00, 1sec : 0x01, 1.5sec : 0x10 */
#define EXT_WATCH_RTC_RUN		0xC016	/* Watch RTC Stop : 0x10, Start 0x01 */

#define EXT_WATCH_POSITION		0xC011	/*Write only*/
#define EXT_WATCH_POSITION_R		0x8A48	/*Read only*/
#define EXT_WATCH_SATATE		0x8A47	/*Watch state, Read only*/

#define EXT_WATCH_LUT_NUM		7
#define EXT_WATCH_FONT_NUM_SIZE		0x500	// 1280	bytes per each '0' ~ '9'
#define EXT_WATCH_FONT_CHAR_SIZE	0x1E0	// 480 bytes for ':'

#define EXT_WATCH_ON			1
#define EXT_WATCH_OFF			0

#define EXT_WATCH_RTC_START		1
#define EXT_WATCH_RTC_STOP		2

#define EXT_WATCH_MCS_ENABLE		1
#define EXT_WATCH_MCS_DISABLE		0


struct ext_watch_ctrl_bits {	/* 0xF9C0 */
	u8	dispmode	: 1;	/* 0 : Alpha blending mode, 1 : Gradation mode */
	u8			: 3;
	u8	grad_mode	: 1;
	u8			: 2;
	u16	alpha		: 9;
} __attribute__((packed));

struct ext_watch_area_bits { /* 0xF9C2 */
	u16	watstartx	: 11;
	u8			: 1;
	u16	watendx		: 11;
	u8			: 1;
}__attribute__((packed));

struct ext_watch_blink_area_bits { /* 0xF9C5 */
	u16	  bstartx	: 11;
	u8			: 1;
	u16	  bendx		: 11;
	u8			: 1;
}__attribute__((packed));

struct ext_watch_grad_bits {	/* 0xF9C8 */
    u32	grad_l		: 24;
    u32	grad_r		: 24;
}__attribute__((packed));

struct  ext_watch_lut_bits {	/* 0xF9E0 */
	u8	b;
	u8	g;
	u8	r;
}__attribute__((packed));

struct ext_watch_time_bits {
	u16	hour		: 5;
	u16	min		: 6;
	u16	sec		: 6;
	u16			: 15;
}__attribute__((packed));

struct ext_watch_mode_cfg {	/* 36 bytes */
	u8					mcs_ctrl;		/* 1 bytes */
	struct ext_watch_ctrl_bits		watch_ctrl;		/* 2 bytes */
	struct ext_watch_area_bits		watch_area;		/* 3 bytes */
	struct ext_watch_blink_area_bits	blink_area;		/* 3 bytes */
	struct ext_watch_grad_bits		grad;			/* 6 bytes */
	struct ext_watch_lut_bits		lut[EXT_WATCH_LUT_NUM];	/* 21 bytes */
}__attribute__((packed));

struct ext_watch_time_cfg {	/* 36 bytes */
	u32				disp_waton;		/* 0xD027 watch display off : 0, on : 1*/
	struct ext_watch_time_bits	rtc_sct;		/* 0xD08B Synchronous current time */
	u16				rtc_sctcnt;		/* 0xD08C Synchronous current time for millisecound */
	u16						: 16;
	struct ext_watch_time_bits	rtc_dit;		/* 0xD08E Target time for occurring date change interrupt */
	struct ext_watch_time_bits	rtc_ctst;		/* 0xD092 Current time */
	u16				rtc_ecnt	: 16;	/* 0xD093 end count */
	u16						: 16;
}__attribute__((packed));

struct ext_watch_position_cfg {	/* 0xC011 20 bytes W-only*/
	u16	h10x_pos	: 9;
	u16	h1x_pos		: 9;
	u16			: 14;
	u16	m10x_pos	: 9;
	u16	m1x_pos		: 9;
	u16			: 14;
	u16	clx_pos		: 9;
	u32			: 23;
	u8	zero_disp	: 1;
	u8	h24_en		: 1;
	u8	clock_disp_mode	: 1;
	u32			: 29;
	u8	bhprd		: 2;
	u32			: 30;
}__attribute__((packed));

struct ext_watch_status_cfg {	/* 0x8A47 4 bytes R-only*/
	u8	step	: 3;
	u8	en		: 1;
	u8	en_24		: 1;
	u8	zero_en : 1;
	u8	disp_mode	: 1;
	u8	bhprd		: 2;
	u8	cur_hour	: 5;
	u8	cur_min		: 6;
	u8	cur_sec		: 6;
	u8			: 6;
}__attribute__((packed));


struct ext_watch_cfg {
	u8				*font_data;
	struct ext_watch_mode_cfg	mode;
	struct ext_watch_time_cfg	time;
	struct ext_watch_position_cfg	position;
}__attribute__((packed));

enum error_type ext_watch_onoff(struct spi_device *spi, u32 onoff);
enum error_type ext_watch_shutdown(struct spi_device *spi, u8 onoff);
enum error_type ext_watch_font_download(struct spi_device *spi, char *font_data);
enum error_type ext_watch_font_dump(struct spi_device *spi, char *font_dump);
enum error_type ext_watch_set_current_time(struct spi_device *spi, struct ext_watch_cfg *cfg);
enum error_type ext_watch_get_current_time(struct spi_device *spi, struct ext_watch_cfg *cfg);
enum error_type ext_watch_set_mode(struct spi_device *spi, struct ext_watch_cfg *cfg);
enum error_type ext_watch_set_position(struct spi_device *spi, struct ext_watch_cfg *cfg);
enum error_type ext_watch_get_cfg(struct spi_device *spi, struct ext_watch_cfg *cfg);
enum error_type ext_watch_set_cfg(struct spi_device *spi,struct ext_watch_cfg *cfg);

extern int sic_spi_write(struct spi_device *spi, u16 addr, u8 *data, u16 size);
extern int sic_spi_read(struct spi_device *spi, u16 addr, u8 *data, u16 size);
extern int sic_spi_font_write(struct spi_device *spi, u8 *font_sel, u8 *data, u16 size);
extern int sic_spi_font_read(struct spi_device *spi, u8 *font_sel, u8 *data, u16 size);

#endif //TOUCH_SIC_WATCH_FONT_H
