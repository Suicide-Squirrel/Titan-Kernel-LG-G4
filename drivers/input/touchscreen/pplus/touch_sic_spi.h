/* touch_sic_spi.h
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

#ifndef TOUCH_SIC_H
#define TOUCH_SIC_H

#define USE_ABT_MONITOR_APP	1

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
//#define report_base		(0x0C00u)
#define report_base                  (0x8C00u)

/*#define tc_version		(tc_device_ctl_base)
#define tc_product_code		(tc_device_ctl_base + 0x1u)
#define tc_product_id		(tc_device_ctl_base + 0x2u)
#define tc_device_ctl		(tc_device_ctl_base + 0x6u)
#define tc_status		(tc_device_ctl_base + 0x7u)
*/
#define tc_version                    (0x8A40u)
#define tc_product_code            (0x8A41u)
#define tc_product_id                (0x8A42u)
#define tc_device_ctl                  (0xC000u)
#define tc_status                    (0x8A45u)

#define INTERRUPT_MASK_ABS0	(0x10000u)
#define INTERRUPT_MASK_BUTTON	(0x20000u)
#define INTERRUPT_MASK_CUSTOM	(0x40000u)
//#define tc_interrupt_status	(tc_device_ctl_base + 0x08u)
//#define tc_interrupt_ctl	(tc_device_ctl_base + 0x09u)
#define tc_interrupt_ctl             (0xC001u)
#define tc_interrupt_status          (0xC021u)
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

#define tc_driving_ctl		0xC002
/* production test */
#define tc_tsp_test_ctl			(0xC003)
#define tc_tsp_test_sts			(0x8A63)
#define tc_tsp_test_raw_data		(0x8A80)
#define tc_tsp_test_pf_result		(0x8A64)
#define tc_tsp_test_os_result		(0x8EA4)

#define RAWDATA_ADDR				(0x8A80u)
#define rawdata_ctl_read				(0x8BE2)
#define rawdata_ctl_write				(0xC229u)
#if 0
#define tc_doze1_offset				(tc_device_ctl_base + 0x22u)
#define tc_doze2_offset				(tc_device_ctl_base + 0x23u)
#define tc_runinfo				(tc_device_ctl_base + 0x24u)
#endif

#define CMD_ABT_TCI_ENABLE_CTRL_READ		(0x8BC0)
#define CMD_ABT_TAP_COUNT_CTRL_READ			(0x8BC1)
#define CMD_ABT_MIN_INTERTAP_CTRL_READ		(0x8BC2)
#define CMD_ABT_MAX_INTERTAP_CTRL_READ		(0x8BC3)
#define CMD_ABT_TOUCH_SLOP_CTRL_READ		(0x8BC4)
#define CMD_ABT_TAP_DISTANCE_CTRL_READ		(0x8BC5)
#define CMD_ABT_INT_DELAY_CTRL_READ			(0x8BC6)
#define CMD_ABT_ACT_AREA_X1_CTRL_READ		(0x8BC7)
#define CMD_ABT_ACT_AREA_Y1_CTRL_READ		(0x8BC8)
#define CMD_ABT_ACT_AREA_X2_CTRL_READ		(0x8BC9)
#define CMD_ABT_ACT_AREA_Y2_CTRL_READ		(0x8BCA)

#define CMD_ABT_TCI_ENABLE_CTRL_WRITE		(0xC200)
#define CMD_ABT_TAP_COUNT_CTRL_WRITE		(0xC201)
#define CMD_ABT_MIN_INTERTAP_CTRL_WRITE		(0xC202)
#define CMD_ABT_MAX_INTERTAP_CTRL_WRITE		(0xC203)
#define CMD_ABT_TOUCH_SLOP_CTRL_WRITE		(0xC204)
#define CMD_ABT_TAP_DISTANCE_CTRL_WRITE		(0xC205)
#define CMD_ABT_INT_DELAY_CTRL_WRITE		(0xC206)
#define CMD_ABT_ACT_AREA_X1_CTRL_WRITE		(0xC207)
#define CMD_ABT_ACT_AREA_Y1_CTRL_WRITE		(0xC208)
#define CMD_ABT_ACT_AREA_X2_CTRL_WRITE		(0xC209)
#define CMD_ABT_ACT_AREA_Y2_CTRL_WRITE		(0xC20A)

#define CMD_ABT_OCD_ON_READ					(0x8BE0)

#define CMD_ABT_CHARGER_INFO_WRITE			(0xC260)
#define CMD_ABT_IME_INFO_WRITE				(0xC261)
#define CMD_ABT_OCD_ON_WRITE				(0xC270)

#define CMD_ABT_LPWG_SWIPE_ON_READ						(0x8BD0)
#define CMD_ABT_LPWG_SWIPE_DIST_THRESHOLD_READ			(0x8BD1)
#define CMD_ABT_LPWG_SWIPE_RATIO_THRESHOLD_READ			(0x8BD2)
#define CMD_ABT_LPWG_SWIPE_RATIO_CHECK_DIST_MIN_READ	(0x8BD3)
#define CMD_ABT_LPWG_SWIPE_RATIO_CHECK_PERIOD_READ		(0x8BD4)
#define CMD_ABT_LPWG_SWIPE_TIME_MIN_READ				(0x8BD5)
#define CMD_ABT_LPWG_SWIPE_TIME_MAX_READ				(0x8BD6)
#define CMD_ABT_LPWG_SWIPE_ACT_AREA_X1_READ				(0x8BD7)
#define CMD_ABT_LPWG_SWIPE_ACT_AREA_Y1_READ				(0x8BD8)
#define CMD_ABT_LPWG_SWIPE_ACT_AREA_X2_READ				(0x8BD9)
#define CMD_ABT_LPWG_SWIPE_ACT_AREA_Y2_READ				(0x8BDA)

#define CMD_ABT_LPWG_SWIPE_ON_WRITE						(0xC210)
#define CMD_ABT_LPWG_SWIPE_DIST_THRESHOLD_WRITE			(0xC211)
#define CMD_ABT_LPWG_SWIPE_RATIO_THRESHOLD_WRITE		(0xC212)
#define CMD_ABT_LPWG_SWIPE_RATIO_CHECK_DIST_MIN_WRITE	(0xC213)
#define CMD_ABT_LPWG_SWIPE_RATIO_CHECK_PERIOD_WRITE		(0xC214)
#define CMD_ABT_LPWG_SWIPE_TIME_MIN_WRITE				(0xC215)
#define CMD_ABT_LPWG_SWIPE_TIME_MAX_WRITE				(0xC216)
#define CMD_ABT_LPWG_SWIPE_ACT_AREA_X1_WRITE			(0xC217)
#define CMD_ABT_LPWG_SWIPE_ACT_AREA_Y1_WRITE			(0xC218)
#define CMD_ABT_LPWG_SWIPE_ACT_AREA_X2_WRITE			(0xC219)
#define CMD_ABT_LPWG_SWIPE_ACT_AREA_Y2_WRITE			(0xC21A)

#define CMD_ABT_LPWG_TCI_FAIL_DEBUG_READ				(0x8BCB)
#define CMD_ABT_LPWG_TCI_FAIL_BIT_READ					(0x8BCC)
#define CMD_ABT_LPWG_TCI_FAIL_COUNT_READ				(0x8BCD)
#define CMD_ABT_LPWG_TCI_FAIL_BUFFER_READ				(0x8BCE)
#define CMD_ABT_LPWG_SWIPE_FAIL_DEBUG_READ				(0x8BDB)
#define CMD_ABT_LPWG_SWIPE_FAIL_COUNT_READ				(0x8BDC)
#define CMD_ABT_LPWG_SWIPE_FAIL_BUFFER_READ				(0x8BDD)

#define CMD_ABT_LPWG_TCI_FAIL_DEBUG_WRITE				(0xC20C)
#define CMD_ABT_LPWG_TCI_FAIL_BIT_WRITE					(0xC20D)
#define CMD_ABT_LPWG_TCI_FAIL_BUFFER_ACCESS_WRITE		(0xC20F)
#define CMD_ABT_LPWG_SWIPE_FAIL_DEBUG_WRITE				(0xC21D)
#define CMD_ABT_LPWG_SWIPE_FAIL_BUFFER_ACCESS_WRITE		(0xC20F)

#define OBJECT_REPORT_ENABLE_REG_READ		(0x8BE1)
#define OBJECT_REPORT_ENABLE_REG_WRITE		(0xC2A0)

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
#define COL_SIZE		34
#define LOG_BUF_SIZE		256
#define BUFFER_SIZE (PAGE_SIZE * 2)

#define TCI_MAX_NUM		2
#define LPWG_FAIL_BUFFER_MAX_NUM	10
#define DOZE1_OFFSET_MIN	-1500
#define DOZE1_OFFSET_MAX	500
#define DOZE2_OFFSET_MIN	-1900
#define DOZE2_OFFSET_MAX	800

//[start] firmware download
#define SIZEOF_FLASH_IMAGE				(68*1024)
#define NUM_OF_FW_SECTOR				(17)
#define SYSCFG_I2CBASE_ADDR				(0xD000)

#define SYS_CHIP_ID						(SYSCFG_I2CBASE_ADDR + 0)
#define SYS_CHIP_VERSION				(SYSCFG_I2CBASE_ADDR + 1)
#define SYS_CLK_CTL                     (SYSCFG_I2CBASE_ADDR + 0x4)
#define SYS_RST_CTL                     (SYSCFG_I2CBASE_ADDR + 0x8)

#define SYS_BOOT_CTL                    (SYSCFG_I2CBASE_ADDR + 0x12)
#define SYS_SRAM_CTRL                   (SYSCFG_I2CBASE_ADDR + 0x13)

#define SYS_CRC_CTL                     (SYSCFG_I2CBASE_ADDR + 0xE)
#define SYS_CRC_ST                      (SYSCFG_I2CBASE_ADDR + 0xF)
#define SFLASHCTRL_NORMCTRL		(0xD05F)
#define SFLASHCTRL_STATUS		(SFLASHCTRL_NORMCTRL+1)
#define SFLASHCTRL_WDATA0		(SFLASHCTRL_NORMCTRL+2)
#define SFLASHCTRL_WDATA1		(SFLASHCTRL_NORMCTRL+3)
#define SFLASHCTRL_WDATA2		(SFLASHCTRL_NORMCTRL+4)
#define SFLASHCTRL_WDATA3		(SFLASHCTRL_NORMCTRL+5)
#define SFLASHCTRL_RDATA0		(SFLASHCTRL_NORMCTRL+6)
#define SFLASHCTRL_RDATA1		(SFLASHCTRL_NORMCTRL+7)
#define SFLASHCTRL_RDATA2		(SFLASHCTRL_NORMCTRL+8)
#define SFLASHCTRL_RDATA3		(SFLASHCTRL_NORMCTRL+9)

#define MX25V1006E_DEVICE_ID		0x10	// electronic id
#define MX25V1006E_MEMORY_TYPE	0x20

//manufacturer ID memory type  memory density
//		C2			20				11
#define MANUFACTURE_ID	(0xC2)	// manufacture id

/* Page Program Sequence */
#define PAGE_PROGRAM	0x2

/* Sector Erase Sequence */
#define SECTOR_ERASE	0x20

/* Block Erase Sequence */
#define	BLOCK_ERASE		(0x52) // 0xD8

/* Chip Erase Sequence */
#define	CHIP_ERASE		(0x60)

/* Write Enable Sequence */
#define WREN			(0x06)

/* Read Status Register Sequence */
#define RDSR_CMD		(0x05)

/* Write Status Register Sequence */
#define WRSR_CMD		(0x01)

/* Read Data */
#define READ_CMD		(0x3)

/* Deep Power Down Sequence */
#define	DP_CMD			(0xB9)

/* Release from Deep Power Down Sequence */
#define	RDP_CMD			(0xAB)

/* Read Electronic Manufacturer & Device ID(REMS) Sequence */
#define REMS_CMD		(0x90)
#define RDID_CMD		(0x9F)

#define FLASH_STS_CHECK_CNT	3
#define SPIM_MAX_TRANSFER_SIZE		16
#define FW_LAST_SECTOR_NUM				(16)
#define CFG_SECTOR_NUM					(17)
#define SIZEOF_SECTOR					(4096)
#define SIZEOF_PAGE						(256)
#define SIZEOF_FW						(67*1024)
#define SIZEOF_CFG						(1024)
#define SPI_TRX_SIZE	(0x4010)//(8192)


#define NUM_OF_FW_DN_SIZE			(16*1024)
#define code_start_addr				(0x0)
#define flashdownload_ctl				(0xc090)
#define flashdownload_start_addr			(0x8200)
#define flashdownload_src			(0x8100)
#define flashdownload_dest			(0x8101)
#define flashdownload_sts			(0x8102)
#define flashdownload_boot_chk			(0x8103)
#define flashdownload_run_cnt 			(0x8104)
#define flash_bl_boot_chk 			(0xA0A0A0A0)

enum {
	SPI_FC_CS_START = 1,
	SPI_FC_CS_STOP = 2,
	SPI_FC_WRITE = 4,
	SPI_FC_READ = 8
};

enum {
	SPI_FC_STS_BUSY = 0x1
};
//[end] firmware download

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
	u32 interrupt_status_reg;
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
	SHORT_NODE_TEST,
	RAWDATA_TEST = 5,
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
	struct spi_device		*spi_device;
	struct cur_touch_data		ts_data;
	struct sic_ts_fw_info	fw_info;
	struct touch_platform_data	*pdata;
	struct state_info		*state;
	struct swipe_data	swipe;
	struct tci_ctrl_data tci_ctrl;
	u8 input[SPI_TRX_SIZE];
	u8 output[SPI_TRX_SIZE];
	struct mutex			spi_lock;
};

enum error_type sic_ts_init(struct spi_device *spi);
enum error_type sic_ts_power(struct spi_device *spi, int power_ctrl);

extern struct workqueue_struct *touch_wq;

#if USE_ABT_MONITOR_APP
extern u16 frame_num;
extern int sic_spi_write(struct spi_device *spi, u16 addr, u8 *data, u16 size);
extern int sic_spi_read(struct spi_device *spi, u16 addr, u8 *data, u16 size);
extern void sic_set_get_data_func(u8 mode);
#endif

#endif

