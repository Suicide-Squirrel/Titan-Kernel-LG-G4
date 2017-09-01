/* include/linux/touch_sic.h
 *
 * Copyright (C) 2014 LGE.
 *
 * Author: sunkwi.kim@lge.com
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

#ifndef TOUCH_SIC_H
#define TOUCH_SIC_H

#define USE_ABT_MONITOR_APP	0

/*----------------------------------------------------------------------
 * lg2411 register map
 *--------------------------------------------------------------------*/
#define fw_i2c_base		(0x0UL)
#define fw_img_size		(0x13800)
#define fpm_i2c_base		(fw_img_size/4)
#define fpm_size		(0x2000)
#define fdm_i2c_base		((fw_img_size + fpm_size)/4)
#define fdm_size		(0x2000)
#define flash_size		(0x18000)
/* flash controller*/
#define fc_base			(0xC800u)
#define tc_device_ctl_base	(0xD000u)
#define tc_status		(0x0BFFu)
#define report_base		(0x0C00u)

#define tc_version		(tc_device_ctl_base)
#define tc_product_code		(tc_device_ctl_base + 0x1u)
#define tc_product_id		(tc_device_ctl_base + 0x2u)
#define tc_device_ctl		(tc_device_ctl_base + 0x6u)

#define INTERRUPT_MASK_ABS0	(0x10000u)
#define INTERRUPT_MASK_BUTTON	(0x20000u)
#define INTERRUPT_MASK_CUSTOM	(0x40000u)
#define tc_interrupt_status	(tc_device_ctl_base + 0x08u)
#define tc_interrupt_ctl	(tc_device_ctl_base + 0x09u)
#define tc_driving_ctl		(tc_device_ctl_base + 0x0au)
#define tc_sp_ctl		(tc_device_ctl_base + 0x13u)
#define spr_rst_ctl		(0xc40cu)
#define spr_flash_crc_ctl	(0xc411u)
#define spr_flash_crc_val	(0xc412u)
#define spr_osc_ctl		(0xc417u)
#define spr_clk_ctl		(0xc406u)


#define serial_if_ctl		(0xc010u)
#define fc_addr			(fc_base)
#define fc_rdata		(fc_base + 0x01u)
#define fc_wdata		(fc_base + 0x02u)
#define fc_ctl			(fc_base + 0x03u)
#define fc_stat			(fc_base + 0x04u)

/* production test */
#define tc_tsp_test_mode_ctl			(tc_device_ctl_base + 0x0fu)
#define tc_tsp_test_mode_node_result		(tc_device_ctl_base + 0x12u)
#define tc_tsp_test_mode_sts			(tc_device_ctl_base + 0x17u)
#define tc_tsp_test_mode_pf_result		(tc_device_ctl_base + 0x18u)
#define tc_tsp_test_mode_node_os_result		(tc_device_ctl_base + 0x19u)
#define tc_serial_if_ctl			(tc_device_ctl_base + 0x30u)
#define RAWDATA_ADDR				(0x2a00u)
#define rawdata_ctl				(0xd629u)
#define tc_doze1_offset				(tc_device_ctl_base + 0x22u)
#define tc_doze2_offset				(tc_device_ctl_base + 0x23u)
#define tc_runinfo				(tc_device_ctl_base + 0x24u)


#define CMD_ABT_TCI_ENABLE_CTRL			(tc_device_ctl_base + 0x610)
#define CMD_ABT_TAP_COUNT_CTRL			(tc_device_ctl_base + 0x611)
#define CMD_ABT_MIN_INTERTAP_CTRL		(tc_device_ctl_base + 0x612)
#define CMD_ABT_MAX_INTERTAP_CTRL		(tc_device_ctl_base + 0x613)
#define CMD_ABT_TOUCH_SLOP_CTRL			(tc_device_ctl_base + 0x614)
#define CMD_ABT_TAP_DISTANCE_CTRL		(tc_device_ctl_base + 0x615)
#define CMD_ABT_INT_DELAY_CTRL			(tc_device_ctl_base + 0x616)
#define CMD_ABT_ACT_AREA_X1_CTRL		(tc_device_ctl_base + 0x617)
#define CMD_ABT_ACT_AREA_Y1_CTRL		(tc_device_ctl_base + 0x618)
#define CMD_ABT_ACT_AREA_X2_CTRL		(tc_device_ctl_base + 0x619)
#define CMD_ABT_ACT_AREA_Y2_CTRL		(tc_device_ctl_base + 0x61a)

#define CMD_ABT_CHARGER_INFO			(tc_device_ctl_base + 0x660)
#define CMD_ABT_IME_INFO			(tc_device_ctl_base + 0x661)
#define CMD_ABT_OCD_ON				(tc_device_ctl_base + 0x670)

#define CMD_ABT_LPWG_SWIPE_ON			(tc_device_ctl_base + 0x680)
#define CMD_ABT_LPWG_SWIPE_DIST_THRESHOLD	(tc_device_ctl_base + 0x681)
#define CMD_ABT_LPWG_SWIPE_RATIO_THRESHOLD	(tc_device_ctl_base + 0x682)
#define CMD_ABT_LPWG_SWIPE_RATIO_CHECK_DIST_MIN	(tc_device_ctl_base + 0x683)
#define CMD_ABT_LPWG_SWIPE_RATIO_CHECK_PERIOD	(tc_device_ctl_base + 0x684)
#define CMD_ABT_LPWG_SWIPE_TIME_MIN		(tc_device_ctl_base + 0x685)
#define CMD_ABT_LPWG_SWIPE_TIME_MAX		(tc_device_ctl_base + 0x686)
#define CMD_ABT_LPWG_SWIPE_ACT_AREA_X1		(tc_device_ctl_base + 0x687)
#define CMD_ABT_LPWG_SWIPE_ACT_AREA_Y1		(tc_device_ctl_base + 0x688)
#define CMD_ABT_LPWG_SWIPE_ACT_AREA_X2		(tc_device_ctl_base + 0x689)
#define CMD_ABT_LPWG_SWIPE_ACT_AREA_Y2		(tc_device_ctl_base + 0x68a)
#define CMD_ABT_LPWG_TCI_FAIL_DEBUG		(tc_device_ctl_base + 0x61c)
#define CMD_ABT_LPWG_TCI_FAIL_BIT_CTRL		(tc_device_ctl_base + 0x61d)

#define CMD_ABT_LPWG_TCI_FAIL_COUNT		(tc_device_ctl_base + 0x61e)
#define CMD_ABT_LPWG_TCI_FAIL_BUFFER		(tc_device_ctl_base + 0x61f)
#define CMD_ABT_LPWG_SWIPE_FAIL_DEBUG		(tc_device_ctl_base + 0x68d)
#define CMD_ABT_LPWG_SWIPE_FAIL_COUNT		(tc_device_ctl_base + 0x68e)
#define CMD_ABT_LPWG_SWIPE_FAIL_BUFFER		(tc_device_ctl_base + 0x68f)

#define OBJECT_REPORT_ENABLE_REG		(tc_device_ctl_base + 0x6a0)

#define DISTANCE_INTER_TAP		(0x1 << 1) /* 2 */
#define DISTANCE_TOUCHSLOP		(0x1 << 2) /* 4 */
#define TIMEOUT_INTER_TAP_LONG		(0x1 << 3) /* 8 */
#define MULTI_FINGER			(0x1 << 4) /* 16 */
#define DELAY_TIME			(0x1 << 5) /* 32 */
#define TIMEOUT_INTER_TAP_SHORT		(0x1 << 6) /* 64 */
#define PALM_STATE			(0x1 << 7) /* 128 */
#define TAP_TIMEOVER			(0x1 << 8) /* 256 */

#define TCI_DEBUG_REASON_ALL (DISTANCE_INTER_TAP |\
	DISTANCE_TOUCHSLOP | TIMEOUT_INTER_TAP_LONG |\
	MULTI_FINGER | DELAY_TIME |\
	TIMEOUT_INTER_TAP_SHORT |\
	PALM_STATE | TAP_TIMEOVER)

#define MAX_REPORT_SLOT		16
#define MAX_REPORT_FINGER	10
#define FW_VER_INFO_NUM		4
#define P_CONTOUR_POINT_MAX	8
#define PALM_ID			15

#define PATH_SIZE		64
#define BURST_SIZE		512
#define RAWDATA_SIZE		2
#define ROW_SIZE		18
#define COL_SIZE		32
#define LOG_BUF_SIZE		256
#define BUFFER_SIZE (PAGE_SIZE * 2)

#define TCI_MAX_NUM		2
#define LPWG_FAIL_BUFFER_MAX_NUM	10
#define DOZE1_OFFSET_MIN	-1600
#define DOZE1_OFFSET_MAX	1000
#define DOZE2_OFFSET_MIN	-1900
#define DOZE2_OFFSET_MAX	1000


enum {
	TOUCHSTS_IDLE = 0,
	TOUCHSTS_DOWN,
	TOUCHSTS_MOVE,
	TOUCHSTS_UP,
};

enum {
	TCI_NOTHING = 0,
	TCI_1,
	TCI_2,
	SWIPE_DOWN,
	SWIPE_UP,
	TCI_FAIL_DEBUG = 200,
};

enum {
	TC_STATUS_RESUME = 0,
	TC_STATUS_SUSPEND,
};

enum {
	SENSOR_STATUS_NEAR = 0,
	SENSOR_STATUS_FAR,
};

enum {
	DOZE1_STATUS = 0,
	DOZE1_PARTIAL_STATUS,
	DOZE2_STATUS,
	DOZE2_DEBUG_STATUS,
	LOW_POWER_STATUS,
};

enum {
	TC_RESTART = 1,
	TC_STOP,
};

struct T_TouchInfo {
	u8 wakeUpType;
	u8 touchCnt:5;
	u8 buttonCnt:3;
	u16 palmBit;
} __packed;

struct T_TouchData {
	u8 toolType:4;
	u8 event:4;
	s8 track_id;
	u16 x;
	u16 y;
	union {
		u8 pressure;
		u8 contourPcnt;
	} byte;
	u8 angle;
	u16 width_major;
	u16 width_minor;
} __packed;

struct T_TCRegCopy2Report {
	u32 tc_reg_copy[5];
};

struct T_OnChipDebug {
	u32 rnd_addr;
	u32 rnd_piece_no;
};

struct T_ReportP {
	struct T_TouchInfo touchInfo;
	struct T_TouchData touchData[MAX_REPORT_SLOT];
	u16 contourP[P_CONTOUR_POINT_MAX];
	struct T_TCRegCopy2Report tc_reg;
	struct T_OnChipDebug ocd;
	u8 dummy[16];
};

struct cur_touch_data {
	u32 device_status_reg;
	u32 test_status_reg;
	struct T_ReportP report;
};

struct sic_ts_fw_info {
	u8	fw_version[2];
	u8	fw_product_id[8];
	u8	fw_image_version[2];
	u8	fw_image_product_id[8];
	u8	*fw_start;
	unsigned char   family;
	unsigned char   fw_revision;
	u32	fw_size;
	u8	need_rewrite_firmware;
};

struct tci_ctrl_info {
	u8	tap_count;
	u8	min_intertap;
	u8	max_intertap;
	u8	touch_slop;
	u8	tap_distance;
	u8	intr_delay;
	u16	active_area_x0;
	u16	active_area_y0;
	u16	active_area_x1;
	u16	active_area_y1;
};

struct tci_ctrl_data {
	u32	tci_mode;
	struct tci_ctrl_info	tci1;
	struct tci_ctrl_info	tci2;
};

struct lpwg_control {
	u8		lpwg_mode;
	u8		screen;
	u8		sensor;
	u8		qcover;
	u8		double_tap_enable;
	u8		password_enable;
	u8		signature_enable;
	u8		lpwg_is_enabled;
	atomic_t	is_suspend;
};

struct lpwg_password_data {
	u8	 tap_count;
	u8	 data_num;
	u8	 double_tap_check;
	struct point	data[MAX_POINT_SIZE_FOR_LPWG];
};

struct swipe_ctrl_info {
	u8	min_distance;
	u8	ratio_thres;
	u8	ratio_chk_period;
	u8	ratio_chk_min_distance;
	u16	min_time_thres;
	u16	max_time_thres;
	u16	active_area_x0;
	u16	active_area_y0;
	u16	active_area_x1;
	u16	active_area_y1;
};

struct swipe_data {
	u32	swipe_mode;
	struct swipe_ctrl_info	down;
	struct swipe_ctrl_info	up;
};

enum {
	SWIPE_DOWN_BIT	= 1,
	SWIPE_UP_BIT	= 1 << 16,
};

enum {
	NO_TEST = 0,
	OPEN_SHORT_ALL_TEST,
	OPEN_NODE_TEST,
	ADJACENCY_SHORT_TEST,
	SAME_MUX_SHORT_TEST,
	RAWDATA_TEST,
	NOISE_TEST,
	RAWDATA_DOZE2,
};

enum {
	NORMAL_MODE = 0,
	PRODUCTION_MODE,
};

struct sic_ts_data {
	u8  is_probed;
	u8  is_init;
	u8  touch_panel_type;
	u8  is_palm;
	u32 debug_mode;
	u32 fail_reason[2];
	u32 fail_type;
	u32 fail_overtap;
	struct lpwg_control lpwg_ctrl;
	struct lpwg_password_data	pw_data;
	struct regulator		*regulator_vdd;
	struct regulator		*regulator_vio;
	struct i2c_client		*client;
	struct cur_touch_data		ts_data;
	struct sic_ts_fw_info	fw_info;
	struct touch_platform_data	*pdata;
	struct state_info		*state;
	struct swipe_data	swipe;
	struct tci_ctrl_data tci_ctrl;
	struct wake_lock		touch_rawdata;
};

enum error_type sic_ts_init(struct i2c_client *client);
enum error_type sic_ts_power(struct i2c_client *client, int power_ctrl);

extern struct workqueue_struct *touch_wq;

#if USE_ABT_MONITOR_APP
extern u16 frame_num;

extern int sic_i2c_write(struct i2c_client *client,	u16 reg,
							u8 *data, u32 len);
extern int sic_i2c_read(struct i2c_client *client, u16 reg,
							u8 *data, u32 len);
extern void sic_set_get_data_func(u8 mode);
#endif

#endif

