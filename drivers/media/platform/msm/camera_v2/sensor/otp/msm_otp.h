/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
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
#ifndef MSM_OTP_H
#define MSM_OTP_H

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <soc/qcom/camera2.h>
#include <media/v4l2-subdev.h>
#include <media/msmb_camera.h>
#include "msm_camera_i2c.h"
#include "msm_camera_spi.h"
#include "msm_camera_io_util.h"
#include "msm_camera_dt_util.h"

struct msm_otp_ctrl_t;

#define DEFINE_MSM_MUTEX(mutexname) \
	static struct mutex mutexname = __MUTEX_INITIALIZER(mutexname)

#define PROPERTY_MAXSIZE 32

struct msm_otp_fn_t {
	int32_t (*otp_read) (struct msm_otp_ctrl_t *,
							struct msm_otp_memory_block_t *);
	int32_t (*otp_checksum) (struct msm_otp_ctrl_t *);
	int32_t (*otp_write) (struct msm_otp_ctrl_t *);
};

struct msm_otp_ctrl_t {
	struct platform_device *pdev;
	struct mutex *otp_mutex;

	struct v4l2_subdev sdev;
	struct v4l2_subdev_ops *otp_v4l2_subdev_ops;
	enum msm_camera_device_type_t otp_device_type;
	struct msm_sd_subdev msm_sd;
	enum cci_i2c_master_t cci_master;

	struct msm_camera_i2c_client i2c_client;
	struct msm_otp_memory_block_t cal_data;
	uint8_t is_supported;
	struct msm_otp_board_info *oboard_info;
	uint32_t subdev_id;
	struct msm_otp_fn_t *otp_func;
	uint8_t otp_read;
	enum camb_position_t position;
};

int msm_otp_platform_remove(struct platform_device *);
int msm_otp_platform_probe(struct platform_device *, struct msm_otp_fn_t *);
int32_t msm_eeprom_find(const char *, enum camb_position_t);
#endif
