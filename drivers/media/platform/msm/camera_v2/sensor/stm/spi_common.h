/* FPC1020 Touch sensor driver
 *
 * Copyright (c) 2013,2014 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#ifndef LINUX_SPI_MH1_COMMON_H
#define LINUX_SPI_MH1_COMMON_H

#define DEBUG

#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/wait.h>


#define GPIO_IDX_AP2SH  0
#define GPIO_IDX_SH2AP  1
#define GPIO_IDX_BOOT0  2
#define GPIO_IDX_NRST   3
#define GPIO_IDX_WAKEUP 4
#define GPIO_IDX_LDOEN 5
#define GPIO_IDX_FIXED_ARRAY 6
#define GPIO_IDX_CS0 6
#define GPIO_IDX_CS2 7
#if defined(CONFIG_MACH_MSM8992_PPLUS_KR) 
#define GPIO_IDX_CS3 8
#define GPIO_IDX_MAX 9
#else
#define GPIO_IDX_MAX 8
#endif


enum {
	SPICH_FLAG_TIMESTAMPS_ENABLED = 1,
};

struct spich_data {
	dev_t devno;
	spinlock_t spi_lock;
	struct spi_device *spi;
	struct class *class;
	struct device *device;
	struct cdev cdev;
	unsigned users;
	struct mutex buf_lock;
	u8 *buffer;
	u8 *bufferrx;
	u32 flags;
	int sh2ap_irq;
	struct completion sh2ap_completion;
	struct gpio gpio_array[GPIO_IDX_MAX];
	u32 spi_freq_mhz;
	int gpio_cs0;
	int gpio_cs2;
	struct regulator *sensor_vdd;
    u32 pre_reset_delay;
};
enum stm_mode {
	STM_SYSTEM=0x01,
	STM_RESET,
	STM_SHUTDOWN
};

/* -------------------------------------------------------------------- */
/* driver constants						*/
/* -------------------------------------------------------------------- */
#define DRV_CLASS_NAME "contexthub"
#define DRV_NAME "spich"
#define SPICH_BUFFER_SIZE       4096

/* -------------------------------------------------------------------- */
/* extern variable							*/
/* -------------------------------------------------------------------- */
#if defined(CONFIG_SPI_MH1)
extern const char *mh1_inbuilt_fw_name_list;
extern struct spich_data *mh1_spich;
#endif
extern const char *stm_inbuilt_fw_name_list;
extern struct spich_data *spich_p;
/* -------------------------------------------------------------------- */
/* function prototypes							*/
/* -------------------------------------------------------------------- */
#if defined(CONFIG_SPI_MH1)
extern int mh1_spi_read_set(struct spich_data *spich);
extern int mh1_spi_write_set(struct spich_data *spich);
extern int mh1_fetch_image(struct spich_data *spich);
extern int mh1_fetch_image_from_sd(struct spich_data *spich);
extern int mh1_fetch_shading_tbl(struct spich_data *spich, uint8_t *table, int size);
#endif
#define STM_NOT_SLEEP
extern int try_download_firmware(struct spich_data *spich);
extern void  StmResetHub( uint8_t tmp);
extern void Spi_Cs_Configuration(int number);
extern int firmware_header_check(uint8_t *header);
extern int stm_calibration(void);
#endif /* LINUX_SPI_MH1_COMMON_H */

