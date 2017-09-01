/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "msm_sensor.h"
#define IMX234_SENSOR_NAME "imx234"
#include <soc/qcom/lge/board_lge.h>	//to use lge_get_board_revno()
DEFINE_MSM_MUTEX(imx234_mut);

static struct msm_sensor_ctrl_t imx234_s_ctrl;

static struct msm_sensor_power_setting imx234_power_setting_reva[] = {
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_OIS_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
#ifdef CONFIG_SPI_MH1
	{	//MH1_LDO_EN(1.8V)
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_MH1_LDO_EN,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
	{	//MH1_DCDC_EN(1.0V)
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_MH1_DCDC_EN,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
	{	//MH1_RESET
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_MH1_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
#endif
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VAF,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_OISVDD,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_LDAF_EN,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{	// OIS_RESET
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_OIS_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
#ifdef CONFIG_SPI_MH1
	{   //MH1 vio for i2c pullup
		.seq_type = SENSOR_VREG,
		.seq_val = MH1_VIO,
		.config_val = 0,
		.delay = 1,
	},
	{	//MH1_LDO_EN(1.8V)
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_MH1_LDO_EN,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	{   //MH1 VDDE12
		.seq_type = SENSOR_VREG,
		.seq_val = MH1_VDDE12,
		.config_val = 0,
		.delay = 1,
	},
	{	//MH1_DCDC_EN(1.0V)
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_MH1_DCDC_EN,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	{    //GYRO VDD 3.3V
		.seq_type = SENSOR_VREG,
		.seq_val = GYRO_VDD,
		.config_val = 0,
		.delay = 1,
	},
#endif
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 1,
	},
#ifdef CONFIG_SPI_MH1
	{	//MH1_RESET
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_MH1_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
#endif
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 10,
	},
};

static struct msm_sensor_power_setting imx234_power_down_setting_reva[] = {
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 2,
	},
#ifdef CONFIG_SPI_MH1
	{	//MH1_RESET
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_MH1_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
#endif
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 1,
	},
#ifdef CONFIG_SPI_MH1
	{   //GYRO VDD 3.3V
		.seq_type = SENSOR_VREG,
		.seq_val = GYRO_VDD,
		.config_val = 0,
		.delay = 1,
	},
	{	//MH1_DCDC_EN(1.0V)
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_MH1_DCDC_EN,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
	{   //MH1 VDDE12
		.seq_type = SENSOR_VREG,
		.seq_val = MH1_VDDE12,
		.config_val = 0,
		.delay = 1,
	},
	{	//MH1_LDO_EN(1.8V)
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_MH1_LDO_EN,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
	{   //MH1 vio for i2c pullup
		.seq_type = SENSOR_VREG,
		.seq_val = MH1_VIO,
		.config_val = 0,
		.delay = 1,
	},
#endif
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{	// OIS_RESET
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_OIS_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_OISVDD,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VAF,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_LDAF_EN,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
};

static struct msm_sensor_power_setting imx234_power_setting[] = {
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_OIS_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VAF,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_OISVDD,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_LDAF_EN,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{	// OIS_RESET
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_OIS_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 10,
	},
};

static struct msm_sensor_power_setting imx234_power_down_setting[] = {
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 2,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{	// OIS_RESET
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_OIS_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_OISVDD,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VAF,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_LDAF_EN,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
};

static struct v4l2_subdev_info imx234_subdev_info[] = {
	{
		.code = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt = 1,
		.order = 0,
	},
};

static const struct i2c_device_id imx234_i2c_id[] = {
	{IMX234_SENSOR_NAME, (kernel_ulong_t)&imx234_s_ctrl},
	{ }
};

static int32_t msm_imx234_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &imx234_s_ctrl);
}

static struct i2c_driver imx234_i2c_driver = {
	.id_table = imx234_i2c_id,
	.probe  = msm_imx234_i2c_probe,
	.driver = {
		.name = IMX234_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client imx234_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id imx234_dt_match[] = {
	{.compatible = "qcom,imx234", .data = &imx234_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, imx234_dt_match);

static struct platform_driver imx234_platform_driver = {
	.driver = {
		.name = "qcom,imx234",
		.owner = THIS_MODULE,
		.of_match_table = imx234_dt_match,
	},
};

static int32_t imx234_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	pr_err("%s: E\n", __func__);
	match = of_match_device(imx234_dt_match, &pdev->dev);
	if (match)
		rc = msm_sensor_platform_probe(pdev, match->data);
	else {
		pr_err("%s:%d match is null\n", __func__, __LINE__);
		rc = -EINVAL;
	}
	pr_err("%s: X\n", __func__);
	return rc;
}

static int __init imx234_init_module(void)
{
	int32_t rc = 0;

	int32_t rev = lge_get_board_revno();

	pr_err("%s:HW rev = %d\n", __func__, rev);

	switch(rev) {
		case 3: //rev0
		case 4: //revA
			imx234_s_ctrl.power_setting_array.power_setting = imx234_power_setting_reva;
			imx234_s_ctrl.power_setting_array.size = ARRAY_SIZE(imx234_power_setting_reva);
			imx234_s_ctrl.power_setting_array.power_down_setting= imx234_power_down_setting_reva;
			imx234_s_ctrl.power_setting_array.size_down = ARRAY_SIZE(imx234_power_down_setting_reva);
			break;
		default:
			imx234_s_ctrl.power_setting_array.power_setting = imx234_power_setting;
			imx234_s_ctrl.power_setting_array.size = ARRAY_SIZE(imx234_power_setting);
			imx234_s_ctrl.power_setting_array.power_down_setting= imx234_power_down_setting;
			imx234_s_ctrl.power_setting_array.size_down = ARRAY_SIZE(imx234_power_down_setting);
			break;
	}
	rc = platform_driver_probe(&imx234_platform_driver,
		imx234_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&imx234_i2c_driver);
}

static void __exit imx234_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (imx234_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&imx234_s_ctrl);
		platform_driver_unregister(&imx234_platform_driver);
	} else
		i2c_del_driver(&imx234_i2c_driver);
	return;
}

static struct msm_sensor_ctrl_t imx234_s_ctrl = {
	.sensor_i2c_client = &imx234_sensor_i2c_client,
	//.power_setting_array.power_setting = imx234_power_setting,
	//.power_setting_array.size = ARRAY_SIZE(imx234_power_setting),
	.msm_sensor_mutex = &imx234_mut,
	.sensor_v4l2_subdev_info = imx234_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(imx234_subdev_info),
};

module_init(imx234_init_module);
module_exit(imx234_exit_module);
MODULE_DESCRIPTION("imx234");
MODULE_LICENSE("GPL v2");
