/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef MSM_MH1_H
#define MSM_MH1_H

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <soc/qcom/camera2.h>
#include <media/v4l2-subdev.h>
#include <media/msmb_camera.h>
#include "msm_camera_i2c.h"
#include "msm_camera_dt_util.h"
#include "msm_camera_io_util.h"

#define DEFINE_MSM_MUTEX(mutexname) \
static struct mutex mutexname = __MUTEX_INITIALIZER(mutexname)
#define	MSM_MH1_MAX_VREGS (10)

#define MH1_SUCCESS 0
#define MH1_FAIL    -1
#define MH1_INIT_OLD_MODULE		1
#define MH1_INIT_NOT_SUPPORTED  -2
#define MH1_INIT_CHECKSUM_ERROR -3
#define MH1_INIT_EEPROM_ERROR   -4
#define MH1_INIT_I2C_ERROR      -5
#define MH1_INIT_TIMEOUT		-6
#define MH1_INIT_LOAD_BIN_ERROR -7
#define MH1_INIT_NOMEM			-8
#define MH1_INIT_GYRO_ADJ_FAIL	 2

enum msm_mh1_state_t {
	MH1_POWER_UP,
	MH1_POWER_DOWN,
};

struct msm_mh1_fn_t {
	int (*mh1_off) (void);
};

struct msm_mh1_vreg {
	struct camera_vreg_t *cam_vreg;
	void *data[MSM_MH1_MAX_VREGS];
	int num_vreg;
};

struct msm_mh1_ctrl_t {
	struct i2c_driver *i2c_driver;
	struct platform_driver *pdriver;
	struct platform_device *pdev;
	struct msm_camera_i2c_client *i2c_client;
	enum msm_camera_device_type_t mh1_device_type;
	struct msm_sd_subdev msm_sd;
	struct mutex *mh1_mutex;
	struct v4l2_subdev sdev;
	struct v4l2_subdev_ops *mh1_v4l2_subdev_ops;
	struct msm_mh1_info_t mh1_stat;

	uint16_t sid_mh1;
	struct msm_mh1_fn_t *mh1_func_tbl;
	struct work_struct mh1_work;
	struct workqueue_struct *work_thread;;
	uint8_t exit_workqueue;
	uint8_t pause_workqueue;
	uint8_t wq_init_success;
	uint32_t max_i2c_fail_thres;
	uint32_t i2c_fail_cnt;
	uint32_t subdev_id;
	uint32_t irq;
	wait_queue_head_t wait;
	uint32_t issued;
};

int32_t msm_init_mh1(struct msm_mh1_ctrl_t *m_ctrl);

int32_t msm_mh1_hdr_on(struct msm_mh1_ctrl_t *m_ctrl);
int msm_mh1_set_shading_tbl(struct msm_mh1_ctrl_t *m_ctrl, uint8_t *shading_tbl, int size );

int msm_mh1_start_stream(bool is_first);
int msm_mh1_stop_stream(void);
int msm_mh1_af_vcm_code(int16_t UsVcmCod);
int msm_mh1_ois_mode(uint8_t set_mode);
int msm_mh1_ois_enable(void);
int msm_mh1_proxy_i2c_e2p_read(int32_t eeprom_addr, int16_t *value);
int msm_mh1_proxy_i2c_e2p_write(int32_t eeprom_addr, int16_t value);

#endif
