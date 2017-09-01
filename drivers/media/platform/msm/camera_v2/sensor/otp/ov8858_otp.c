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

#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/crc32.h>
#include "msm_sd.h"
#include "msm_cci.h"
#include "msm_otp.h"
#include "msm.h"

#undef CDBG
#define CDBG(fmt, args...) pr_debug("%s %d "fmt, __func__, __LINE__, ##args)
#include <linux/device.h>

/**
  * read_xxx_otp_memory() - read map data into buffer
  * @o_ctrl:	otp control struct
  * @block:	block to be read
  *
  * This function iterates through blocks stored in block->map, reads each
  * region and concatenate them into the pre-allocated block->mapdata
  */
static int32_t ov8858_read_otp_memory(struct msm_otp_ctrl_t *o_ctrl,
			      struct msm_otp_memory_block_t *block)
{
	int rc = 0;
	int j;
	struct msm_otp_memory_map_t *emap = block->map;
	struct msm_otp_board_info *ob_info;
	uint8_t *memptr = block->mapdata;
	uint16_t buf = 0;

	if (!o_ctrl) {
		pr_err("%s o_ctrl is NULL", __func__);
		return -EINVAL;
	}

	ob_info = o_ctrl->oboard_info;

	for (j = 0; j < block->num_map; j++) {
		if (emap[j].saddr.addr) {
			ob_info->i2c_slaveaddr = emap[j].saddr.addr;
			o_ctrl->i2c_client.cci_client->sid =
					ob_info->i2c_slaveaddr >> 1;
			pr_err("qcom,slave-addr = 0x%X\n",
				ob_info->i2c_slaveaddr);
		}

		if (emap[j].mem.valid_size) {
			o_ctrl->i2c_client.addr_type = emap[j].mem.addr_t;

			rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_read(
					&(o_ctrl->i2c_client), 0x300b,
					&buf, MSM_CAMERA_I2C_WORD_DATA);
			if(rc < 0) {
				pr_err("%s %d i2c fail\n", __func__, __LINE__);
				return rc;
			}

			pr_err("%s %d chipid = 0x%x\n", __func__, __LINE__, buf);

			rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write(
					&(o_ctrl->i2c_client),
					0x0100,
					1, MSM_CAMERA_I2C_BYTE_DATA);
			if(rc < 0) {
				pr_err("%s %d i2c failed\n", __func__, __LINE__);
				return rc;
			}

			buf = 0;
			rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_read(
					&(o_ctrl->i2c_client), 0x5002,
					&buf, MSM_CAMERA_I2C_BYTE_DATA);
			if(rc < 0) {
				pr_err("%s %d i2c fail\n", __func__, __LINE__);
				return rc;
			}

			buf &= ~BIT(3);
			rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write(
					&(o_ctrl->i2c_client),
					0x5002,
					buf, MSM_CAMERA_I2C_BYTE_DATA);
			if(rc < 0) {
				pr_err("%s %d i2c failed\n", __func__, __LINE__);
				return rc;
			}

			buf = 0;
			rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_read(
					&(o_ctrl->i2c_client), 0x3d84,
					&buf, MSM_CAMERA_I2C_BYTE_DATA);
			if(rc < 0) {
				pr_err("%s %d i2c fail\n", __func__, __LINE__);
				return rc;
			}

			buf &= ~(BIT(6) | BIT(7));
			rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write(
					&(o_ctrl->i2c_client),
					0x3d84,
					buf, MSM_CAMERA_I2C_BYTE_DATA);
			if(rc < 0) {
				pr_err("%s %d i2c failed\n", __func__, __LINE__);
				return rc;
			}

			rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write(
					&(o_ctrl->i2c_client),
					0x3d88,
					0x7010, MSM_CAMERA_I2C_WORD_DATA);
			if(rc < 0) {
				pr_err("%s %d i2c failed\n", __func__, __LINE__);
				return rc;
			}

			rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write(
					&(o_ctrl->i2c_client),
					0x3d8a,
					0x720e, MSM_CAMERA_I2C_WORD_DATA);
			if(rc < 0) {
				pr_err("%s %d i2c failed\n", __func__, __LINE__);
				return rc;
			}

			buf = 0;
			rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_read(
					&(o_ctrl->i2c_client), 0x3d81,
					&buf, MSM_CAMERA_I2C_BYTE_DATA);
			if(rc < 0) {
				pr_err("%s %d i2c fail\n", __func__, __LINE__);
				return rc;
			}

			buf |= BIT(0);
			rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write(
					&(o_ctrl->i2c_client),
					0x3d81,
					buf, MSM_CAMERA_I2C_BYTE_DATA);
			if(rc < 0) {
				pr_err("%s %d i2c failed\n", __func__, __LINE__);
				return rc;
			}
			msleep(10);

			rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_read_seq(
				&(o_ctrl->i2c_client), emap[j].mem.addr,
				memptr, emap[j].mem.valid_size);

			if(rc < 0) {
				pr_err("%s %d i2c failed\n", __func__, __LINE__);
				return rc;
			}

			buf = 0;
			rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_read(
					&(o_ctrl->i2c_client), 0x5002,
					&buf, MSM_CAMERA_I2C_BYTE_DATA);
			if(rc < 0) {
				pr_err("%s %d i2c fail\n", __func__, __LINE__);
				return rc;
			}

			buf |= BIT(3);
			rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write(
					&(o_ctrl->i2c_client),
					0x5002,
					buf, MSM_CAMERA_I2C_BYTE_DATA);
			if(rc < 0) {
				pr_err("%s %d i2c failed\n", __func__, __LINE__);
				return rc;
			}

			rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write(
					&(o_ctrl->i2c_client),
					0x0100,
					0, MSM_CAMERA_I2C_BYTE_DATA);
			if(rc < 0) {
				pr_err("%s %d i2c failed\n", __func__, __LINE__);
				return rc;
			}
		}
	}

	return rc;
}

static int32_t ov8858_write_otp_memory(struct msm_otp_ctrl_t *o_ctrl)
{
	int rc = 0;
	//int i;

	struct msm_otp_board_info *ob_info;
	uint16_t buf = 0;
	int bank_offset_for_lsc = 0;

      pr_err("%s ENTER\n", __func__);

	if (!o_ctrl) {
		pr_err("%s o_ctrl is NULL", __func__);
		return -EINVAL;
	}
	ob_info = o_ctrl->oboard_info;

#if 1
    pr_err("%s num_data = %d\n", __func__, o_ctrl->cal_data.num_data);

	pr_err("%s mapdata = %d\n", __func__, o_ctrl->cal_data.mapdata[0]);
	pr_err("%s mapdata = %d\n", __func__, o_ctrl->cal_data.mapdata[1]);
	pr_err("%s mapdata = %d\n", __func__, o_ctrl->cal_data.mapdata[2]);
#endif

	if(o_ctrl->cal_data.mapdata[1] == 1) {
		bank_offset_for_lsc = 16;
	} else if(o_ctrl->cal_data.mapdata[1] == 3) {
		bank_offset_for_lsc = 269;
	} else {
		pr_err("%s %d Invalid otp data\n", __func__, __LINE__);
		return -EINVAL;
	}

	buf = 0;
	rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_read(
			&(o_ctrl->i2c_client), 0x5002,
			&buf, MSM_CAMERA_I2C_BYTE_DATA);
	if(rc < 0) {
		pr_err("%s %d i2c fail\n", __func__, __LINE__);
		return rc;
	}

	buf &= ~BIT(3);
	rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write(
			&(o_ctrl->i2c_client),
			0x5002,
			buf, MSM_CAMERA_I2C_BYTE_DATA);
	if(rc < 0) {
		pr_err("%s %d i2c failed\n", __func__, __LINE__);
		return rc;
	}
	msleep(10);

#if 1
	rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write_seq(
		&(o_ctrl->i2c_client), 0x5800,
		&(o_ctrl->cal_data.mapdata[bank_offset_for_lsc]), 240);

	if(rc < 0) {
		pr_err("%s %d i2c failed\n", __func__, __LINE__);
		return rc;
	}
#else // if there is a error of sequential writing, use 12byte writing
	for (i = 0; i < 20; i++){
		rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write_seq(
			&(o_ctrl->i2c_client), 0x5800 + (i*12),
			&(o_ctrl->cal_data.mapdata[bank_offset_for_lsc + (i*12)]), 12);

		if(rc < 0) {
			pr_err("%s %d i2c failed(%d th)\n", __func__, __LINE__, i);
			return rc;
		}
	}
#endif
	buf = 0;
	rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_read(
			&(o_ctrl->i2c_client), 0x5002,
			&buf, MSM_CAMERA_I2C_BYTE_DATA);
	if(rc < 0) {
		pr_err("%s %d i2c fail\n", __func__, __LINE__);
		return rc;
	}

	buf |= BIT(3);
	rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write(
			&(o_ctrl->i2c_client),
			0x5002,
			buf, MSM_CAMERA_I2C_BYTE_DATA);
	if(rc < 0) {
		pr_err("%s %d i2c failed\n", __func__, __LINE__);
		return rc;
	}

      pr_err("%s EXIT\n", __func__);

	return rc;
}

static int32_t ov8858_otp_checksum(struct msm_otp_ctrl_t *o_ctrl) {
	int32_t rc = -EFAULT;
	int bank_offset = 0;
	int i;

	uint8_t sum_img = 0, chk_img = 0;
	uint16_t sum_sensitivity = 0, chk_sensitivity = 0;
	uint16_t sum_shading = 0, chk_shading = 0;

	if(o_ctrl->cal_data.mapdata[0] == 1) {
		bank_offset = 0;
	} else if(o_ctrl->cal_data.mapdata[0] == 3) {
		bank_offset = 253;
	} else {
		pr_err("%s %d Invalid otp data\n", __func__, __LINE__);
	}

	for(i = 5 + bank_offset; i < 7 + bank_offset; i++) {
		sum_img += o_ctrl->cal_data.mapdata[i];
	}
	chk_img = o_ctrl->cal_data.mapdata[7+bank_offset];

	for(i = 8 + bank_offset; i < 14 + bank_offset; i++) {
		sum_sensitivity += o_ctrl->cal_data.mapdata[i];
	}
	chk_sensitivity = (o_ctrl->cal_data.mapdata[15 + bank_offset] << 8)
						| o_ctrl->cal_data.mapdata[14 + bank_offset];

	for(i = 16 + bank_offset; i < 256 + bank_offset; i++) {
		sum_shading += o_ctrl->cal_data.mapdata[i];
	}
	chk_shading = (o_ctrl->cal_data.mapdata[257 + bank_offset] << 8)
						| o_ctrl->cal_data.mapdata[256 + bank_offset];

	if(sum_img == chk_img && sum_sensitivity == chk_sensitivity &&
		sum_shading == chk_shading) {
		pr_err("%s %d Checksum verified\n", __func__, __LINE__);
		rc = 0;
	} else {
		pr_err("%s %d sum_img = 0x%x, chk_img = 0x%x\n",
			__func__, __LINE__, sum_img, chk_img);
		pr_err("%s %d sum_sensitivity = 0x%x, chk_sensitivity = 0x%x\n",
			__func__, __LINE__, sum_sensitivity, chk_sensitivity);
		pr_err("%s %d sum_shading = 0x%x, chk_shading = 0x%x\n",
			__func__, __LINE__, sum_shading, chk_shading);
	}

	return rc;
};

static const struct of_device_id ov8858_otp_dt_match[] = {
	{ .compatible = "ovt,ov8858-otp" },
	{ }
};

MODULE_DEVICE_TABLE(of, ov8858_otp_dt_match);

static struct platform_driver ov8858_otp_platform_driver = {
	.driver = {
		.name = "ov8858,otp",
		.owner = THIS_MODULE,
		.of_match_table = ov8858_otp_dt_match,
	},
	.remove = msm_otp_platform_remove,
};

struct msm_otp_fn_t ov8858_otp_func = {
	.otp_read = ov8858_read_otp_memory,
	.otp_checksum = ov8858_otp_checksum,
	.otp_write = ov8858_write_otp_memory,
};

static int32_t ov8858_otp_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	CDBG("E\n");
	rc = msm_otp_platform_probe(pdev, &ov8858_otp_func);
	CDBG("msm_otp_platform_probe rc = %d\n", rc);
	return rc;
}


static int32_t __init ov8858_otp_init_module(void)
{
	int rc = 0;

	CDBG("E\n");
	rc = platform_driver_probe(&ov8858_otp_platform_driver,
		ov8858_otp_platform_probe);
	CDBG("platform rc %d\n", rc);

	return rc;
}

static void __exit ov8858_otp_exit_module(void)
{
	platform_driver_unregister(&ov8858_otp_platform_driver);
}

module_init(ov8858_otp_init_module);
module_exit(ov8858_otp_exit_module);
MODULE_DESCRIPTION("ov8858 OTP driver");
MODULE_LICENSE("GPL v2");
