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

DEFINE_MSM_MUTEX(msm_otp_mutex);
#ifdef CONFIG_COMPAT
static struct v4l2_file_operations msm_otp_v4l2_subdev_fops;
#endif

#include <linux/device.h>

#define MODULE_VENDOR_ID 0x05

static struct class *camera_vendor_id_class = NULL;
static int8_t main_sensor_id = -1;

static ssize_t show_LGCameraMainID(struct device *dev,struct device_attribute *attr, char *buf)
{
	pr_err("show_LGCameraMainID: main_camera_id [%d] \n", main_sensor_id);
	switch (main_sensor_id) {
		case 0x01:
		case 0x02:
		case 0x05:
		case 0x06:
		case 0x07:
			return sprintf(buf, "id:0x%x, %s\n", main_sensor_id, "LGIT");
		case 0x03:
			return sprintf(buf, "id:0x%x, %s\n", main_sensor_id, "Fujifilm");
		case 0x04:
			return sprintf(buf, "id:0x%x, %s\n", main_sensor_id, "Minolta");
		case 0x10:
		case 0x11:
		case 0x12:
		case 0x13:
			return sprintf(buf, "id:0x%x, %s\n", main_sensor_id, "Cowell");
		case 0x14:
		case 0x15:
		case 0x16:
		case 0x17:
			return sprintf(buf, "id:0x%x, %s\n", main_sensor_id, "IM-tech");
		case 0x20:
		case 0x21:
		case 0x22:
		case 0x23:
			return sprintf(buf, "id:0x%x, %s\n", main_sensor_id, "Sunny");
		default:
			return sprintf(buf, "id:0x%x, %s\n", main_sensor_id, "Reserved for future");
	}
}

static DEVICE_ATTR(vendor_id, S_IRUGO, show_LGCameraMainID, NULL);


/**
  * msm_otp_verify_sum - verify crc32 checksum
  * @mem:	data buffer
  * @size:	size of data buffer
  * @sum:	expected checksum
  *
  * Returns 0 if checksum match, -EINVAL otherwise.
  */
static int msm_otp_verify_sum(const char *mem, uint32_t size, uint32_t sum)
{
	uint32_t crc = ~0;

	/* check overflow */
	if (size > crc - sizeof(uint32_t))
		return -EINVAL;

	crc = crc32_le(crc, mem, size);
	if (~crc != sum) {
		CDBG("%s: expect 0x%x, result 0x%x\n", __func__, sum, ~crc);
		return -EINVAL;
	}
	CDBG("%s: checksum pass 0x%x\n", __func__, sum);
	return 0;
}

/**
  * msm_otp_match_crc - verify multiple regions using crc
  * @data:	data block to be verified
  *
  * Iterates through all regions stored in @data.  Regions with odd index
  * are treated as data, and its next region is treated as checksum.  Thus
  * regions of even index must have valid_size of 4 or 0 (skip verification).
  * Returns a bitmask of verified regions, starting from LSB.  1 indicates
  * a checksum match, while 0 indicates checksum mismatch or not verified.
  */
static uint32_t msm_otp_match_crc(struct msm_otp_memory_block_t *data)
{
	int j, rc;
	uint32_t *sum;
	uint32_t ret = 0;
	uint8_t *memptr;
	struct msm_otp_memory_map_t *map;

	if (!data) {
		pr_err("%s data is NULL", __func__);
		return -EINVAL;
	}
	map = data->map;
	memptr = data->mapdata;

	for (j = 0; j + 1 < data->num_map; j += 2) {
		/* empty table or no checksum */
		if (!map[j].mem.valid_size || !map[j+1].mem.valid_size) {
			memptr += map[j].mem.valid_size
				+ map[j+1].mem.valid_size;
			continue;
		}
		if (map[j+1].mem.valid_size != sizeof(uint32_t)) {
			CDBG("%s: malformatted data mapping\n", __func__);
			return -EINVAL;
		}
		sum = (uint32_t *) (memptr + map[j].mem.valid_size);
		rc = msm_otp_verify_sum(memptr, map[j].mem.valid_size,
					   *sum);
		if (!rc)
			ret |= 1 << (j/2);
		memptr += map[j].mem.valid_size + map[j+1].mem.valid_size;
	}
	return ret;
}

static int msm_otp_get_cmm_data(struct msm_otp_ctrl_t *o_ctrl,
				       struct msm_otp_cfg_data *cdata)
{
	int rc = 0;
	struct msm_otp_cmm_t *cmm_data = &o_ctrl->oboard_info->cmm_data;
	cdata->cfg.get_cmm_data.cmm_support = cmm_data->cmm_support;
	cdata->cfg.get_cmm_data.cmm_compression = cmm_data->cmm_compression;
	cdata->cfg.get_cmm_data.cmm_size = cmm_data->cmm_size;
	return rc;
}

static int otp_config_read_cal_data(struct msm_otp_ctrl_t *o_ctrl,
	struct msm_otp_cfg_data *cdata)
{
	int rc;

	/* check range */
	if (cdata->cfg.read_data.num_bytes >
		o_ctrl->cal_data.num_data) {
		CDBG("%s: Invalid size. exp %u, req %u\n", __func__,
			o_ctrl->cal_data.num_data,
			cdata->cfg.read_data.num_bytes);
		return -EINVAL;
	}
	if (!o_ctrl->cal_data.mapdata)
		return -EFAULT;

	rc = copy_to_user(cdata->cfg.read_data.dbuffer,
		o_ctrl->cal_data.mapdata,
		cdata->cfg.read_data.num_bytes);

	return rc;
}

static int msm_otp_write_data(struct msm_otp_ctrl_t *o_ctrl)
{
	int rc = 0;

	if(!o_ctrl->otp_read) {
		pr_err("%s data is not prepared\n", __func__);
		return rc;
	}

	if(o_ctrl->otp_func->otp_write) {
		rc = o_ctrl->otp_func->otp_write(o_ctrl);
		if(rc < 0) {
			pr_err("%s otp write failed\n", __func__);
			rc = -ENODEV;
		}
       }
       return rc;
}

static int msm_otp_prepare_data(struct msm_otp_ctrl_t *o_ctrl)
{
	struct msm_camera_power_ctrl_t *power_info =
		&o_ctrl->oboard_info->power_info;
	struct msm_otp_board_info *ob_info = o_ctrl->oboard_info;
	struct msm_camera_i2c_reg_setting boot_array;
	int rc = 0;
	int j = 0;

	if(o_ctrl->otp_read) {
		pr_info("%s data is already prepared\n", __func__);
		return rc;
	}

	rc = msm_camera_power_up(power_info, o_ctrl->otp_device_type,
		&o_ctrl->i2c_client);
	if (rc) {
		pr_err("failed rc %d\n", rc);
		goto memdata_free;
	}

	o_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;

	if(ob_info->i2c_init && ob_info->cnt_i2c_init) {
		boot_array.reg_setting = ob_info->i2c_init;
		boot_array.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
		boot_array.data_type = MSM_CAMERA_I2C_BYTE_DATA;
		boot_array.delay = 0;
		boot_array.size = ob_info->cnt_i2c_init;

		rc = o_ctrl->i2c_client.i2c_func_tbl->i2c_write_table(&(o_ctrl->i2c_client), &boot_array);
		if(rc < 0)
			pr_err("%s %d i2c write fail\n", __func__, __LINE__);
	}

	/* TODO : Need to implement otp read & checksum function for each sensor */
	if(o_ctrl->otp_func->otp_read && o_ctrl->otp_func->otp_checksum) {
		rc = o_ctrl->otp_func->otp_read(o_ctrl, &o_ctrl->cal_data);
		if(rc < 0) {
			pr_err("%s otp read failed\n", __func__);
			rc = -ENODEV;
			goto power_down;
		}

		rc = o_ctrl->otp_func->otp_checksum(o_ctrl);
		if(rc < 0) {
			pr_err("%s otp checksum failed\n", __func__);
		}
	} else {
		pr_err("%s otp function is null\n", __func__);
		rc = -ENODEV;
	}

	if (rc < 0) {
		pr_err("%s read_otp_memory failed\n", __func__);
		goto power_down;
	}

	CDBG("[CHECK] o_ctrl->cal_data.num_data: %d\n", o_ctrl->cal_data.num_data);
	for (j = 0; j < o_ctrl->cal_data.num_data - 1  ; j = j+2) {
		CDBG("data[%d]: 0x%x, 0x%x\n", j,
				o_ctrl->cal_data.mapdata[j],
				o_ctrl->cal_data.mapdata[j+1]);
	}

	o_ctrl->is_supported |= msm_otp_match_crc(&o_ctrl->cal_data);

	if(!rc) {
		main_sensor_id = o_ctrl->cal_data.mapdata[MODULE_VENDOR_ID];
		pr_info("%s:main_sensor_id 0x%x\n", __func__, main_sensor_id);
	}

	rc = msm_camera_power_down(power_info, o_ctrl->otp_device_type,
		&o_ctrl->i2c_client);
	if (rc) {
		pr_err("failed rc %d\n", rc);
		goto memdata_free;
	}

	o_ctrl->otp_read = 1;

	return rc;

power_down:
	msm_camera_power_down(power_info, o_ctrl->otp_device_type,
		&o_ctrl->i2c_client);
memdata_free:
	kfree(o_ctrl->cal_data.mapdata);
	kfree(o_ctrl->cal_data.map);
	return rc;
}

static int msm_otp_config(struct msm_otp_ctrl_t *o_ctrl,
	void __user *argp)
{
	struct msm_otp_cfg_data *cdata =
		(struct msm_otp_cfg_data *)argp;
	int rc = 0;

	CDBG("%s E\n", __func__);
	switch (cdata->cfgtype) {
	case CFG_OTP_GET_INFO:
		CDBG("%s E CFG_OTP_GET_INFO\n", __func__);
		cdata->is_supported = o_ctrl->is_supported;
		memcpy(cdata->cfg.otp_name,
			o_ctrl->oboard_info->otp_name,
			sizeof(cdata->cfg.otp_name));
		break;
	case CFG_OTP_GET_CAL_DATA:
		CDBG("%s E CFG_OTP_GET_CAL_DATA\n", __func__);
		cdata->cfg.get_data.num_bytes =
			o_ctrl->cal_data.num_data;
		break;
	case CFG_OTP_READ_CAL_DATA:
		CDBG("%s E CFG_OTP_READ_CAL_DATA\n", __func__);
		rc = msm_otp_prepare_data(o_ctrl);
		if(!rc)
			rc = otp_config_read_cal_data(o_ctrl, cdata);
		break;
	case CFG_OTP_GET_MM_INFO:
		CDBG("%s E CFG_OTP_GET_MM_INFO\n", __func__);
		rc = msm_otp_get_cmm_data(o_ctrl, cdata);
		break;
    case CFG_OTP_WRITE_DATA:
        CDBG("%s E CFG_OTP_WRITE_DATA\n", __func__);
		rc = msm_otp_write_data(o_ctrl);
		break;
	default:
		break;
	}

	CDBG("%s X rc: %d\n", __func__, rc);
	return rc;
}

static int msm_otp_get_subdev_id(struct msm_otp_ctrl_t *o_ctrl,
				    void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;
	CDBG("%s E\n", __func__);
	if (!subdev_id) {
		pr_err("%s failed\n", __func__);
		return -EINVAL;
	}
	*subdev_id = o_ctrl->subdev_id;
	CDBG("subdev_id %d\n", *subdev_id);
	CDBG("%s X\n", __func__);
	return 0;
}

static long msm_otp_subdev_ioctl(struct v4l2_subdev *sd,
		unsigned int cmd, void *arg)
{
	struct msm_otp_ctrl_t *o_ctrl = v4l2_get_subdevdata(sd);
	void __user *argp = (void __user *)arg;
	CDBG("%s E\n", __func__);
	CDBG("%s:%d a_ctrl %p argp %p\n", __func__, __LINE__, o_ctrl, argp);
	switch (cmd) {
	case VIDIOC_MSM_SENSOR_GET_SUBDEV_ID:
		return msm_otp_get_subdev_id(o_ctrl, argp);
	case VIDIOC_MSM_OTP_CFG:
		return msm_otp_config(o_ctrl, argp);
	default:
		return -ENOIOCTLCMD;
	}

	CDBG("%s X\n", __func__);
}

static struct msm_camera_i2c_fn_t msm_otp_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_seq = msm_camera_cci_i2c_write_seq,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
	msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
	.i2c_poll = msm_camera_cci_i2c_poll,
};

static int msm_otp_open(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh) {
	int rc = 0;
	struct msm_otp_ctrl_t *o_ctrl =  v4l2_get_subdevdata(sd);
	CDBG("%s E\n", __func__);
	if (!o_ctrl) {
		pr_err("%s failed o_ctrl is NULL\n", __func__);
		return -EINVAL;
	}
	CDBG("%s X\n", __func__);
	return rc;
}

static int msm_otp_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh) {
	int rc = 0;
	struct msm_otp_ctrl_t *o_ctrl =  v4l2_get_subdevdata(sd);
	CDBG("%s E\n", __func__);
	if (!o_ctrl) {
		pr_err("%s failed o_ctrl is NULL\n", __func__);
		return -EINVAL;
	}
	CDBG("%s X\n", __func__);
	return rc;
}

static const struct v4l2_subdev_internal_ops msm_otp_internal_ops = {
	.open = msm_otp_open,
	.close = msm_otp_close,
};

/**
  * msm_otp_parse_memory_map() - parse memory map in device node
  * @of:	device node
  * @data:	memory block for output
  *
  * This functions parses @of to fill @data.  It allocates map itself, parses
  * the @of node, calculate total data length, and allocates required buffer.
  * It only fills the map, but does not perform actual reading.
  */
static int msm_otp_parse_memory_map(struct device_node *of,
				       struct msm_otp_memory_block_t *data)
{
	int i, rc = 0;
	char property[PROPERTY_MAXSIZE];
	uint32_t count = 6;
	struct msm_otp_memory_map_t *map;

	snprintf(property, PROPERTY_MAXSIZE, "qcom,num-blocks");
	rc = of_property_read_u32(of, property, &data->num_map);
	CDBG("%s: %s %d\n", __func__, property, data->num_map);
	if (rc < 0) {
		pr_err("%s failed rc %d\n", __func__, rc);
		return rc;
	}

	map = kzalloc((sizeof(*map) * data->num_map), GFP_KERNEL);
	if (!map) {
		pr_err("%s failed line %d\n", __func__, __LINE__);
		return -ENOMEM;
	}
	data->map = map;

	for (i = 0; i < data->num_map; i++) {
		snprintf(property, PROPERTY_MAXSIZE, "qcom,page%d", i);
		rc = of_property_read_u32_array(of, property,
				(uint32_t *) &map[i].page, count);
		if (rc < 0) {
			pr_err("%s: failed %d\n", __func__, __LINE__);
			goto ERROR;
		}

		snprintf(property, PROPERTY_MAXSIZE,
					"qcom,pageen%d", i);
		rc = of_property_read_u32_array(of, property,
			(uint32_t *) &map[i].pageen, count);
		if (rc < 0)
			pr_err("%s: pageen not needed\n", __func__);

		snprintf(property, PROPERTY_MAXSIZE, "qcom,saddr%d", i);
		rc = of_property_read_u32_array(of, property,
			(uint32_t *) &map[i].saddr.addr, 1);
		if (rc < 0)
			CDBG("%s: saddr not needed - block %d\n", __func__, i);

		snprintf(property, PROPERTY_MAXSIZE, "qcom,poll%d", i);
		rc = of_property_read_u32_array(of, property,
				(uint32_t *) &map[i].poll, count);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR;
		}

		snprintf(property, PROPERTY_MAXSIZE, "qcom,mem%d", i);
		rc = of_property_read_u32_array(of, property,
				(uint32_t *) &map[i].mem, count);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR;
		}
		data->num_data += map[i].mem.valid_size;
	}

	CDBG("%s num_bytes %d\n", __func__, data->num_data);

	data->mapdata = kzalloc(data->num_data, GFP_KERNEL);
	if (!data->mapdata) {
		pr_err("%s failed line %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto ERROR;
	}
	return rc;

ERROR:
	kfree(data->map);
	memset(data, 0, sizeof(*data));
	return rc;
}

#if 0
static struct msm_cam_clk_info cam_8960_clk_info[] = {
	[SENSOR_CAM_MCLK] = {"cam_clk", 24000000},
};
#endif

static struct msm_cam_clk_info cam_8974_clk_info[] = {
	/* LGE_CHANGE_S, camera bringup*/
	[SENSOR_CAM_MCLK] = {"cam_src_clk", 24000000},
	/* LGE_CHANGE_E, camera bringup*/
	[SENSOR_CAM_CLK] = {"cam_clk", 0},
};

static struct v4l2_subdev_core_ops msm_otp_subdev_core_ops = {
	.ioctl = msm_otp_subdev_ioctl,
};

static struct v4l2_subdev_ops msm_otp_subdev_ops = {
	.core = &msm_otp_subdev_core_ops,
};

static int msm_otp_get_dt_data(struct msm_otp_ctrl_t *o_ctrl)
{
	int rc = 0, i = 0;
	struct msm_otp_board_info *ob_info;
	struct msm_camera_power_ctrl_t *power_info =
		&o_ctrl->oboard_info->power_info;
	struct device_node *of_node = NULL;
	struct msm_camera_gpio_conf *gconf = NULL;
	//uint16_t gpio_array_size = 0;
	int gpio_array_size = 0;
	uint16_t *gpio_array = NULL;
	uint32_t count = 0;
	uint32_t *val_array = 0;

	ob_info = o_ctrl->oboard_info;

	if (o_ctrl->otp_device_type == MSM_CAMERA_PLATFORM_DEVICE)
		of_node = o_ctrl->pdev->dev.of_node;
	else
		return -ENODEV;

	if (!of_node) {
		pr_err("%s: %d of_node is NULL\n", __func__ , __LINE__);
		return -ENOMEM;
	}

	if(of_get_property(of_node, "qcom,i2c-init-addr", &count)) {
		count /= sizeof(uint32_t);
		CDBG("%s i2c init register cnt %d, rc %d\n", __func__,
				count, rc);

		if(count > 0) {
			ob_info->i2c_init = kzalloc(sizeof(struct msm_camera_i2c_reg_array) * count, GFP_KERNEL);
			if(!ob_info->i2c_init) {
				pr_err("%s %d Not enough memory\n", __func__, __LINE__);
				return -ENOMEM;
			}

			val_array = kzalloc(sizeof(uint32_t) * count, GFP_KERNEL);
			if(!val_array) {
				pr_err("%s %d Not enough memory\n", __func__, __LINE__);
				goto ERROR_I2C_INIT;
			}

			ob_info->cnt_i2c_init = count;

			rc = of_property_read_u32_array(of_node, "qcom,i2c-init-addr", val_array, count);
			if(rc < 0) {
				pr_err("%s Can't read qcom,i2c-init-addr, rc = %d\n", __func__, rc);
				goto ERROR0;
			}

			for(i=0; i<count; i++) {
				ob_info->i2c_init[i].reg_addr = (uint16_t)val_array[i];
			}

			rc = of_property_read_u32_array(of_node, "qcom,i2c-init-data", val_array, count);
			if(rc < 0) {
				pr_err("%s Can't read qcom,i2c-init-data, rc = %d\n", __func__, rc);
				goto ERROR0;
			}

			for(i=0; i<count; i++) {
				ob_info->i2c_init[i].reg_data = (uint16_t)val_array[i];
				CDBG("%s OTP i2c init, addr = 0x%x, data = 0x%x\n", __func__, ob_info->i2c_init[i].reg_addr, ob_info->i2c_init[i].reg_data);
			}

			kfree(val_array);
		}
	}

	rc = msm_camera_get_dt_vreg_data(of_node, &power_info->cam_vreg,
					     &power_info->num_vreg);
	if (rc < 0)
		return rc;

	rc = msm_camera_get_dt_power_setting_data(of_node,
		power_info->cam_vreg, power_info->num_vreg,
		power_info);
	if (rc < 0)
		goto ERROR1;

	power_info->gpio_conf = kzalloc(sizeof(struct msm_camera_gpio_conf),
					GFP_KERNEL);
	if (!power_info->gpio_conf) {
		rc = -ENOMEM;
		goto ERROR2;
	}
	gconf = power_info->gpio_conf;
	gpio_array_size = of_gpio_count(of_node);
	CDBG("%s gpio count %d\n", __func__, gpio_array_size);

	//LGE_CHANGE, fix invalid routine caused by unsigned type of gpio_array_size which
	//            makes -ENOENT return value from -2 to 65534, 2014-11-20, jongkwon.chae@lge.com
	if (gpio_array_size > 0) {
		gpio_array = kzalloc(sizeof(uint16_t) * gpio_array_size,
			GFP_KERNEL);
		if (!gpio_array) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR3;
		}
		for (i = 0; i < gpio_array_size; i++) {
			gpio_array[i] = of_get_gpio(of_node, i);
			CDBG("%s gpio_array[%d] = %d\n", __func__, i,
				gpio_array[i]);
		}

		rc = msm_camera_get_dt_gpio_req_tbl(of_node, gconf,
			gpio_array, gpio_array_size);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR4;
		}

		rc = msm_camera_init_gpio_pin_tbl(of_node, gconf,
			gpio_array, gpio_array_size);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR4;
		}
		kfree(gpio_array);
	}

	return rc;
ERROR4:
	kfree(gpio_array);
ERROR3:
	kfree(power_info->gpio_conf);
ERROR2:
	kfree(power_info->cam_vreg);
ERROR1:
	kfree(power_info->power_setting);
ERROR0:
	if(!val_array)
		kfree(val_array);
ERROR_I2C_INIT:
	if(!ob_info->i2c_init)
		kfree(ob_info->i2c_init);
	return rc;
}


static int msm_otp_cmm_dts(struct msm_otp_board_info *ob_info,
				struct device_node *of_node)
{
	int rc = 0;
	struct msm_otp_cmm_t *cmm_data = &ob_info->cmm_data;

	cmm_data->cmm_support =
		of_property_read_bool(of_node, "qcom,cmm-data-support");
	if (!cmm_data->cmm_support)
		return -EINVAL;
	cmm_data->cmm_compression =
		of_property_read_bool(of_node, "qcom,cmm-data-compressed");
	if (!cmm_data->cmm_compression)
		CDBG("No MM compression data\n");

	rc = of_property_read_u32(of_node, "qcom,cmm-data-offset",
				  &cmm_data->cmm_offset);
	if (rc < 0)
		CDBG("No MM offset data\n");

	rc = of_property_read_u32(of_node, "qcom,cmm-data-size",
				  &cmm_data->cmm_size);
	if (rc < 0)
		CDBG("No MM size data\n");

	CDBG("cmm_support: cmm_compr %d, cmm_offset %d, cmm_size %d\n",
		cmm_data->cmm_compression,
		cmm_data->cmm_offset,
		cmm_data->cmm_size);
	return 0;
}

#ifdef CONFIG_COMPAT
static int otp_config_read_cal_data32(struct msm_otp_ctrl_t *o_ctrl,
	void __user *arg)
{
	int rc;
	uint8_t *ptr_dest = NULL;
	struct msm_otp_cfg_data32 *cdata32 =
		(struct msm_otp_cfg_data32 *) arg;
	struct msm_otp_cfg_data cdata;

	cdata.cfgtype = cdata32->cfgtype;
	cdata.is_supported = cdata32->is_supported;
	cdata.cfg.read_data.num_bytes = cdata32->cfg.read_data.num_bytes;
	/* check range */
	if (cdata.cfg.read_data.num_bytes >
	    o_ctrl->cal_data.num_data) {
		CDBG("%s: Invalid size. exp %u, req %u\n", __func__,
			o_ctrl->cal_data.num_data,
			cdata.cfg.read_data.num_bytes);
		return -EINVAL;
	}
	if (!o_ctrl->cal_data.mapdata)
		return -EFAULT;

	ptr_dest = (uint8_t *) compat_ptr(cdata32->cfg.read_data.dbuffer);

	rc = copy_to_user(ptr_dest, o_ctrl->cal_data.mapdata,
		cdata.cfg.read_data.num_bytes);

	return rc;
}

static int msm_otp_config32(struct msm_otp_ctrl_t *o_ctrl,
	void __user *argp)
{
	struct msm_otp_cfg_data *cdata = (struct msm_otp_cfg_data *)argp;
	int rc = 0;

	CDBG("%s E\n", __func__);
	switch (cdata->cfgtype) {
	case CFG_OTP_GET_INFO:
		CDBG("%s E CFG_OTP_GET_INFO\n", __func__);
		cdata->is_supported = o_ctrl->is_supported;
		memcpy(cdata->cfg.otp_name,
			o_ctrl->oboard_info->otp_name,
			sizeof(cdata->cfg.otp_name));
		break;
	case CFG_OTP_GET_CAL_DATA:
		CDBG("%s E CFG_OTP_GET_CAL_DATA\n", __func__);
		cdata->cfg.get_data.num_bytes =
			o_ctrl->cal_data.num_data;
		break;
	case CFG_OTP_READ_CAL_DATA:
		CDBG("%s E CFG_OTP_READ_CAL_DATA\n", __func__);
		rc = msm_otp_prepare_data(o_ctrl);
		if(!rc)
			rc = otp_config_read_cal_data32(o_ctrl, argp);
		break;
    case CFG_OTP_WRITE_DATA:
        CDBG("%s E CFG_OTP_WRITE_DATA\n", __func__);
		rc = msm_otp_write_data(o_ctrl);
		break;
	default:
		break;
	}

	CDBG("%s X rc: %d\n", __func__, rc);
	return rc;
}

static long msm_otp_subdev_ioctl32(struct v4l2_subdev *sd,
		unsigned int cmd, void *arg)
{
	struct msm_otp_ctrl_t *o_ctrl = v4l2_get_subdevdata(sd);
	void __user *argp = (void __user *)arg;

	CDBG("%s E\n", __func__);
	CDBG("%s:%d a_ctrl %p argp %p\n", __func__, __LINE__, o_ctrl, argp);
	switch (cmd) {
	case VIDIOC_MSM_SENSOR_GET_SUBDEV_ID:
		return msm_otp_get_subdev_id(o_ctrl, argp);
	case VIDIOC_MSM_OTP_CFG32:
		return msm_otp_config32(o_ctrl, argp);
	default:
		return -ENOIOCTLCMD;
	}

	CDBG("%s X\n", __func__);
}

static long msm_otp_subdev_do_ioctl32(
	struct file *file, unsigned int cmd, void *arg)
{
	struct video_device *vdev = video_devdata(file);
	struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);

	return msm_otp_subdev_ioctl32(sd, cmd, arg);
}

static long msm_otp_subdev_fops_ioctl32(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	return video_usercopy(file, cmd, arg, msm_otp_subdev_do_ioctl32);
}

#endif

static void msm_otp_create_sysfs(void)
{
	struct device*  camera_vendor_id_dev;

	camera_vendor_id_class = class_create(THIS_MODULE, "camera");
	camera_vendor_id_dev = device_create(camera_vendor_id_class, NULL,
		0, NULL, "vendor_id");
	device_create_file(camera_vendor_id_dev, &dev_attr_vendor_id);
	return;
}

int msm_otp_platform_probe(struct platform_device *pdev, struct msm_otp_fn_t *func)
{
	int rc = 0;

	uint32_t temp;

	struct msm_camera_cci_client *cci_client = NULL;
	struct msm_otp_ctrl_t *o_ctrl = NULL;
	struct msm_otp_board_info *ob_info = NULL;
	struct device_node *of_node = pdev->dev.of_node;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	CDBG("%s E\n", __func__);

	o_ctrl = kzalloc(sizeof(*o_ctrl), GFP_KERNEL);
	if (!o_ctrl) {
		pr_err("%s:%d kzalloc failed\n", __func__, __LINE__);
		return -ENOMEM;
	}
	o_ctrl->otp_v4l2_subdev_ops = &msm_otp_subdev_ops;
	o_ctrl->otp_mutex = &msm_otp_mutex;

	o_ctrl->is_supported = 0;
	if (!of_node) {
		pr_err("%s dev.of_node NULL\n", __func__);
		return -EINVAL;
	}

	rc = of_property_read_u32(of_node, "cell-index",
		&pdev->id);
	CDBG("cell-index %d, rc %d\n", pdev->id, rc);
	if (rc < 0) {
		pr_err("failed rc %d\n", rc);
		return rc;
	}
	o_ctrl->subdev_id = pdev->id;

	rc = of_property_read_u32(of_node, "qcom,cci-master",
		&o_ctrl->cci_master);
	CDBG("qcom,cci-master %d, rc %d\n", o_ctrl->cci_master, rc);
	if (rc < 0) {
		pr_err("%s failed rc %d\n", __func__, rc);
		return rc;
	}
	rc = of_property_read_u32(of_node, "qcom,slave-addr",
		&temp);
	if (rc < 0) {
		pr_err("%s failed rc %d\n", __func__, rc);
		return rc;
	}

	/* Set platform device handle */
	o_ctrl->pdev = pdev;
	/* Set device type as platform device */
	o_ctrl->otp_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	o_ctrl->i2c_client.i2c_func_tbl = &msm_otp_cci_func_tbl;
	o_ctrl->i2c_client.cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!o_ctrl->i2c_client.cci_client) {
		pr_err("%s failed no memory\n", __func__);
		return -ENOMEM;
	}

	o_ctrl->oboard_info = kzalloc(sizeof(
		struct msm_otp_board_info), GFP_KERNEL);
	if (!o_ctrl->oboard_info) {
		pr_err("%s failed line %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto cciclient_free;
	}
	ob_info = o_ctrl->oboard_info;
	power_info = &ob_info->power_info;
	ob_info->i2c_slaveaddr = temp;

	power_info->clk_info = cam_8974_clk_info;
	power_info->clk_info_size = ARRAY_SIZE(cam_8974_clk_info);
	power_info->dev = &pdev->dev;


	rc = of_property_read_u32(of_node, "qcom,i2c-freq-mode",
		&ob_info->i2c_freq_mode);
	if (rc < 0 || (ob_info->i2c_freq_mode >= I2C_MAX_MODES)) {
		ob_info->i2c_freq_mode = I2C_STANDARD_MODE;
		CDBG("%s Default I2C standard speed mode.\n", __func__);
	}

	CDBG("qcom,slave-addr = 0x%X\n", ob_info->i2c_slaveaddr);
	cci_client = o_ctrl->i2c_client.cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->cci_i2c_master = o_ctrl->cci_master;
	cci_client->sid = ob_info->i2c_slaveaddr >> 1;
	cci_client->retries = 3;
	cci_client->id_map = 0;
	cci_client->i2c_freq_mode =  ob_info->i2c_freq_mode;

	rc = of_property_read_string(of_node, "qcom,otp-name",
		&ob_info->otp_name);
	CDBG("%s qcom,otp-name %s, rc %d\n", __func__,
		ob_info->otp_name, rc);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto board_free;
	}

	rc = of_property_read_u32(of_node, "qcom,otp-position",
				  &temp);
	CDBG("qcom,otp-position %d, rc %d\n", o_ctrl->position, rc);
	if (rc < 0) {
		o_ctrl->position = BACK_CAMERA_B;
		pr_info("Default OTP position. %d\n", rc);
	} else {
		o_ctrl->position = temp;
	}

#if 0
	if(msm_eeprom_find(ob_info->otp_name, o_ctrl->position)) {
		pr_info("%s %d EEPROM module\n", __func__, __LINE__);
		goto board_free;
	}
#endif

	rc = msm_otp_cmm_dts(o_ctrl->oboard_info, of_node);
	if (rc < 0)
		CDBG("%s MM data miss:%d\n", __func__, __LINE__);

	rc = msm_otp_get_dt_data(o_ctrl);
	if (rc)
		goto board_free;

	rc = msm_otp_parse_memory_map(of_node, &o_ctrl->cal_data);
	if (rc < 0)
		goto board_free;

	o_ctrl->otp_func = func;

	v4l2_subdev_init(&o_ctrl->msm_sd.sd,
		o_ctrl->otp_v4l2_subdev_ops);
	v4l2_set_subdevdata(&o_ctrl->msm_sd.sd, o_ctrl);
	platform_set_drvdata(pdev, &o_ctrl->msm_sd.sd);
	o_ctrl->msm_sd.sd.internal_ops = &msm_otp_internal_ops;
	o_ctrl->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(o_ctrl->msm_sd.sd.name,
		ARRAY_SIZE(o_ctrl->msm_sd.sd.name), "msm_otp");
	media_entity_init(&o_ctrl->msm_sd.sd.entity, 0, NULL, 0);
	o_ctrl->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	o_ctrl->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_OTP;
	msm_sd_register(&o_ctrl->msm_sd);

#ifdef CONFIG_COMPAT
	msm_otp_v4l2_subdev_fops = v4l2_subdev_fops;
	msm_otp_v4l2_subdev_fops.compat_ioctl32 =
		msm_otp_subdev_fops_ioctl32;
	o_ctrl->msm_sd.sd.devnode->fops = &msm_otp_v4l2_subdev_fops;
#endif

	o_ctrl->is_supported = (o_ctrl->is_supported << 1) | 1;
	CDBG("[CHECK] [FINAL]  o_ctrl->is_supported: 0x%X", o_ctrl->is_supported);
	CDBG("%s X\n", __func__);

	if(o_ctrl->position == BACK_CAMERA_B && !camera_vendor_id_class) {
		msm_otp_create_sysfs();
	}

	return rc;

board_free:
	if(o_ctrl->oboard_info->i2c_init)
		kfree(o_ctrl->oboard_info->i2c_init);
	kfree(o_ctrl->oboard_info);
cciclient_free:
	kfree(o_ctrl->i2c_client.cci_client);
	kfree(o_ctrl);
	return rc;
}

int msm_otp_platform_remove(struct platform_device *pdev)
{
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct msm_otp_ctrl_t  *o_ctrl;
	if (!sd) {
		pr_err("%s: Subdevice is NULL\n", __func__);
		return 0;
	}

	o_ctrl = (struct msm_otp_ctrl_t *)v4l2_get_subdevdata(sd);
	if (!o_ctrl) {
		pr_err("%s: otp device is NULL\n", __func__);
		return 0;
	}

	kfree(o_ctrl->i2c_client.cci_client);
	kfree(o_ctrl->cal_data.mapdata);
	kfree(o_ctrl->cal_data.map);
	if (o_ctrl->oboard_info) {
		kfree(o_ctrl->oboard_info->power_info.gpio_conf);
		if(o_ctrl->oboard_info->i2c_init)
			kfree(o_ctrl->oboard_info->i2c_init);
		kfree(o_ctrl->oboard_info);
	}
	kfree(o_ctrl);

	class_destroy(camera_vendor_id_class);
	return 0;
}

#if 0
static const struct of_device_id msm_otp_dt_match[] = {
	{ .compatible = "qcom,otp" },
	{ }
};

MODULE_DEVICE_TABLE(of, msm_otp_dt_match);

static struct platform_driver msm_otp_platform_driver = {
	.driver = {
		.name = "qcom,otp",
		.owner = THIS_MODULE,
		.of_match_table = msm_otp_dt_match,
	},
	.remove = msm_otp_platform_remove,
};

static const struct i2c_device_id msm_otp_i2c_id[] = {
	{ "msm_otp", (kernel_ulong_t)NULL},
	{ }
};

static int __init msm_otp_init_module(void)
{
	int rc = 0;
	struct device*  camera_vendor_id_dev;

	CDBG("%s E\n", __func__);
	rc = platform_driver_probe(&msm_otp_platform_driver,
		msm_otp_platform_probe);
	CDBG("%s:%d platform rc %d\n", __func__, __LINE__, rc);

	if(!rc) {
		camera_vendor_id_class = class_create(THIS_MODULE, "camera");
		camera_vendor_id_dev = device_create(camera_vendor_id_class, NULL,
			0, NULL, "vendor_id");
		device_create_file(camera_vendor_id_dev, &dev_attr_vendor_id);
	}
	return rc;
}

static void __exit msm_otp_exit_module(void)
{
	platform_driver_unregister(&msm_otp_platform_driver);
	class_destroy(camera_vendor_id_class);
}

module_init(msm_otp_init_module);
module_exit(msm_otp_exit_module);
MODULE_DESCRIPTION("MSM OTP driver");
MODULE_LICENSE("GPL v2");
#endif
