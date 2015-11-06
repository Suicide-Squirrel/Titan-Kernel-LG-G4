/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>

#include "oem_mdss_dsi_common.h"
#include "oem_mdss_dsi.h"
#include <soc/qcom/lge/board_lge.h>
#include <linux/input/lge_touch_notify.h>
#include <linux/mfd/p1_dsv.h>

#define PIN_RESET "RESET"
#define PANEL_SEQUENCE(name, state) do { pr_debug("[PanelSequence][%s] %d\n", name, state); } while (0)

int panel_not_connected;
bool first_touch_power_on = false;
int swipe_status;
int proxy_sensor_status;
int lgd_rsp_lcd_on_off = 1;
int SIC_is_doze =1;
int jdi_deep_sleep = 0;

static int lcd_notifier_callback(struct notifier_block *this,
					unsigned long event, void *data)
{
	int mode  = 0;
	int param;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = container_of(this,
		struct mdss_dsi_ctrl_pdata, notif);

	switch (event) {
	case LCD_EVENT_TOUCH_DRIVER_REGISTERED:
		ctrl_pdata->lge_pan_data->touch_driver_registered = true;
		pr_err("%s: TOUCH_EVENT_REGISTER_DONE received ndx: %d [%d]\n",
				__func__, ctrl_pdata->ndx,
				ctrl_pdata->lge_pan_data->touch_driver_registered);
		break;

	case LCD_EVENT_TOUCH_PROXY_STATUS:
		if (ctrl_pdata->ndx) {
			mode = *(int *)(unsigned long) data;
			switch (mode) {
			case PROXY_NEAR:
				if (ctrl_pdata->panel_data.panel_info.lge_pan_info.panel_type
					== LGD_SIC_INCELL_CMD_PANEL) {
					if (!ctrl_pdata->ndx || SIC_is_doze == 1)
						break;

					mdss_dsi_lcd_reset(&(ctrl_pdata->panel_data), 0);
					mdelay(2);
					mdss_dsi_lcd_reset(&(ctrl_pdata->panel_data), 1);
					mdelay(2);

					if (ctrl_pdata->lge_pan_data->clk_off_cmds.cmd_cnt) {
						pr_err("lge_log %s cmd cnt:%d\n", __func__, ctrl_pdata->lge_pan_data->clk_off_cmds.cmd_cnt);
						mdss_dsi_stub_cmds_send(ctrl_pdata, &ctrl_pdata->lge_pan_data->clk_off_cmds);
					}
					dw8768_fast_discharge();
					pr_err("%s: At PROXY_NEAR event clk off, ndx: %d\n", __func__, ctrl_pdata->ndx);
				} else if (ctrl_pdata->panel_data.panel_info.lge_pan_info.panel_type
					== LGD_INCELL_CMD_PANEL) {
					proxy_sensor_status = PROXY_NEAR;
					pr_err("%s: TOUCH_EVENT_PROXY_STATUS received : PROXY_NEAR\n",
							__func__);
					if (lgd_rsp_lcd_on_off == 0) {
						pr_info("%s: LPWG to DEEP\n", __func__);
						lgd_deep_sleep(ctrl_pdata,
							LPWG_TO_DEEP_SLEEP,
						       LPWG_TO_DEEP_SLEEP);
					}
				} else if (ctrl_pdata->panel_data.panel_info.lge_pan_info.panel_type
					== JDI_INCELL_CMD_PANEL) {
					proxy_sensor_status = PROXY_NEAR;
					pr_err("%s: TOUCH_EVENT_PROXY_STATUS received : PROXY_NEAR\n",
							__func__);
					if (lgd_rsp_lcd_on_off == 0) {
						param = 0;
						pr_info("%s: LPWG to DEEP\n", __func__);
						jdi_deep_sleep = 1;
						touch_notifier_call_chain(
						LCD_EVENT_TOUCH_SLEEP_STATUS,
							(void *)&param);
					}
				}
				break;

			case PROXY_FAR:
				if (ctrl_pdata->panel_data.panel_info.lge_pan_info.panel_type
					== LGD_SIC_INCELL_CMD_PANEL) {
					if (!ctrl_pdata->ndx || SIC_is_doze == 1)
						break;

					mdss_dsi_lcd_reset(&(ctrl_pdata->panel_data), 0);
					mdelay(2);
					mdss_dsi_lcd_reset(&(ctrl_pdata->panel_data), 1);
					mdelay(2);

					if (ctrl_pdata->lge_pan_data->clk_on_cmds.cmd_cnt) {
						pr_err("lge_log %s cmd cnt:%d\n", __func__, ctrl_pdata->lge_pan_data->clk_on_cmds.cmd_cnt);
						mdss_dsi_stub_cmds_send(ctrl_pdata, &ctrl_pdata->lge_pan_data->clk_on_cmds);
					}
					pr_err("%s: At PROXY_FAR event clk on, ndx: %d\n", __func__, ctrl_pdata->ndx);
				} else if (ctrl_pdata->panel_data.panel_info.lge_pan_info.panel_type
					== LGD_INCELL_CMD_PANEL) {
					proxy_sensor_status = PROXY_FAR;
					pr_err("%s: TOUCH_EVENT_PROXY_STATUS received : PROXY_FAR\n",
							__func__);
					if (lgd_rsp_lcd_on_off == 0) {
						pr_info("%s: DEEP SLEEP to LPWG\n",
							       __func__);
						lgd_deep_sleep(ctrl_pdata,
							DEEP_SLEEP_TO_LPWG,
						       DEEP_SLEEP_TO_LPWG);
					}
				} else if (ctrl_pdata->panel_data.panel_info.lge_pan_info.panel_type
					== JDI_INCELL_CMD_PANEL) {
					proxy_sensor_status = PROXY_FAR;
					pr_err("%s: TOUCH_EVENT_PROXY_STATUS received : PROXY_FAR\n",
							__func__);
					if (lgd_rsp_lcd_on_off == 0) {
						param = 1;
						pr_info("%s: DEEP SLEEP to LPWG\n",
							       __func__);
						jdi_deep_sleep = 0;
						touch_notifier_call_chain(
						LCD_EVENT_TOUCH_SLEEP_STATUS,
							(void *)&param);
					}

				}
				break;
			default:
				break;
				}
			}
			break;
	case LCD_EVENT_TOUCH_SWIPE_STATUS:
		if (ctrl_pdata->ndx) {
			mode = *(int *)(unsigned long) data;
			switch (mode) {
			case SWIPE_DONE:
				swipe_status = 0;
				pr_err("TOUCH_EVENT_SWIPE received : %d\n",
						       SWIPE_DONE);
				break;
			case DO_SWIPE:
				swipe_status = 1;
				pr_err("TOUCH_EVENT_SWIPE received : %d\n",
						       DO_SWIPE);
				break;
			default:
				break;
			}
		}
		break;
	default:
		break;
	}
	return 0;
}

void mdss_dsi_ctrl_shutdown(struct platform_device *pdev)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = platform_get_drvdata(pdev);
	struct mdss_panel_info *pinfo = &(ctrl_pdata->panel_data.panel_info);
	int ret = 0;

	if (!ctrl_pdata) {
		pr_err("%s: no driver data\n", __func__);
		return;
	}

	if (ctrl_pdata->ndx == DSI_CTRL_LEFT)
		return;

	if (pinfo->lge_pan_info.panel_type == LGD_SIC_INCELL_CMD_PANEL) {
		struct dss_vreg *in_vreg;
		int i;
		int ldo_list[] = {
			-1,		// p1 : ldo_tpvci
			-1,		// p1 : ldo_vpnl
			-1		// pplus : ldo_vpnl_touch
		};

		in_vreg = ctrl_pdata->power_data[DSI_PANEL_PM].vreg_config;

		for(i=0; i<ctrl_pdata->power_data[DSI_PANEL_PM].num_vreg; i++){
			if(!strcmp(in_vreg[i].vreg_name, "tpvci"))
				ldo_list[P1_LDO_TPVCI] = i;
			if(!strcmp(in_vreg[i].vreg_name, "vdd_l19"))
				ldo_list[P1_LDO_VPNL] = i;
			if(!strcmp(in_vreg[i].vreg_name, "vpnl_touch"))
				ldo_list[PP_LDO_VPNL_TOUCH] = i;
		}

#if defined(CONFIG_MACH_MSM8992_P1_CN) \
|| defined(CONFIG_MACH_MSM8992_P1_GLOBAL_COM)
		if (lge_get_sim_type() == 2) {
			if (gpio_is_valid(ctrl_pdata->lge_pan_data->touch_io)) {
				gpio_set_value((ctrl_pdata->lge_pan_data->touch_io), 0);
				gpio_free(ctrl_pdata->lge_pan_data->touch_io);
			}
		} else {
			pr_err("%s: sigle sim model doesn't use external touch io\n",
					__func__);
		}
#endif

		if (ldo_list[P1_LDO_TPVCI] != -1) {
			if (regulator_is_enabled(in_vreg[ldo_list[P1_LDO_TPVCI]].vreg)) {
				if (in_vreg[ldo_list[P1_LDO_TPVCI]].pre_off_sleep)
					msleep(in_vreg[ldo_list[P1_LDO_TPVCI]].pre_off_sleep);
				regulator_set_optimum_mode(in_vreg[ldo_list[P1_LDO_TPVCI]].vreg,
					in_vreg[ldo_list[P1_LDO_TPVCI]].disable_load);
				regulator_disable(in_vreg[ldo_list[P1_LDO_TPVCI]].vreg);
				PANEL_SEQUENCE(in_vreg[ldo_list[P1_LDO_TPVCI]].vreg_name, 0);
				if (in_vreg[ldo_list[P1_LDO_TPVCI]].post_off_sleep)
					msleep(in_vreg[ldo_list[P1_LDO_TPVCI]].post_off_sleep);
			}
		}

		if (ldo_list[PP_LDO_VPNL_TOUCH] != -1) {
			if (regulator_is_enabled(in_vreg[ldo_list[PP_LDO_VPNL_TOUCH]].vreg)) {
				if (in_vreg[ldo_list[PP_LDO_VPNL_TOUCH]].pre_off_sleep)
					msleep(in_vreg[ldo_list[PP_LDO_VPNL_TOUCH]].pre_off_sleep);
				regulator_set_optimum_mode(in_vreg[ldo_list[PP_LDO_VPNL_TOUCH]].vreg,
					in_vreg[ldo_list[PP_LDO_VPNL_TOUCH]].disable_load);
				regulator_disable(in_vreg[ldo_list[PP_LDO_VPNL_TOUCH]].vreg);
				PANEL_SEQUENCE(in_vreg[ldo_list[PP_LDO_VPNL_TOUCH]].vreg_name, 0);
				if (in_vreg[ldo_list[PP_LDO_VPNL_TOUCH]].post_off_sleep)
					msleep(in_vreg[ldo_list[PP_LDO_VPNL_TOUCH]].post_off_sleep);
			}
		}

		dw8768_mode_change(0);
		if (gpio_is_valid(ctrl_pdata->rst_gpio)) {
			gpio_set_value((ctrl_pdata->rst_gpio), 0);
			PANEL_SEQUENCE(PIN_RESET, 0);
			gpio_free(ctrl_pdata->rst_gpio);
		}

		if (gpio_is_valid(ctrl_pdata->lge_pan_data->dsv_ena)) {
			gpio_set_value((ctrl_pdata->lge_pan_data->dsv_ena), 0);
			gpio_free(ctrl_pdata->lge_pan_data->dsv_ena);
		}

		if (ldo_list[P1_LDO_VPNL] != -1) {
			if (regulator_is_enabled(in_vreg[ldo_list[P1_LDO_VPNL]].vreg)) {
				if (in_vreg[ldo_list[P1_LDO_VPNL]].pre_off_sleep)
					msleep(in_vreg[ldo_list[P1_LDO_VPNL]].pre_off_sleep);
				regulator_set_optimum_mode(in_vreg[ldo_list[P1_LDO_VPNL]].vreg,
					in_vreg[ldo_list[P1_LDO_VPNL]].disable_load);
				regulator_disable(in_vreg[ldo_list[P1_LDO_VPNL]].vreg);
				PANEL_SEQUENCE(in_vreg[ldo_list[P1_LDO_VPNL]].vreg_name, 0);
				if (in_vreg[ldo_list[P1_LDO_VPNL]].post_off_sleep)
					msleep(in_vreg[ldo_list[P1_LDO_VPNL]].post_off_sleep);
			}
		}

		if (gpio_is_valid(ctrl_pdata->lge_pan_data->vddio_en)) {
			gpio_set_value((ctrl_pdata->lge_pan_data->vddio_en), 0);
			gpio_free(ctrl_pdata->lge_pan_data->vddio_en);
		}
	} else if (pinfo->lge_pan_info.panel_type == LGD_INCELL_CMD_PANEL) {

		ret = msm_dss_enable_vreg(
			ctrl_pdata->power_data[DSI_PANEL_PM].vreg_config,
			ctrl_pdata->power_data[DSI_PANEL_PM].num_vreg, 0);
		if (ret)
			pr_err("%s: failed to disable vregs for PANEL_PM\n",
					__func__);

#if defined(CONFIG_MACH_MSM8992_P1_CN) \
|| defined(CONFIG_MACH_MSM8992_P1_GLOBAL_COM)
		if (lge_get_sim_type() == 2) {
			if (gpio_is_valid(ctrl_pdata->lge_pan_data->touch_io)) {
				gpio_set_value((ctrl_pdata->lge_pan_data->touch_io), 0);
				gpio_free(ctrl_pdata->lge_pan_data->touch_io);
			}
		} else {
			pr_err("%s: sigle sim model doesn't use external touch io\n",
					__func__);
		}
#endif
		if (gpio_is_valid(ctrl_pdata->rst_gpio)) {
			gpio_set_value((ctrl_pdata->rst_gpio), 0);
			PANEL_SEQUENCE(PIN_RESET, 0);
			gpio_free(ctrl_pdata->rst_gpio);
		}

		mdelay(2);

		dw8768_lgd_dsv_setting(2);

		if (gpio_is_valid(ctrl_pdata->lge_pan_data->dsv_ena)) {
			gpio_set_value((ctrl_pdata->lge_pan_data->dsv_ena), 0);
			gpio_free(ctrl_pdata->lge_pan_data->dsv_ena);
		}

		mdelay(2);

		if (gpio_is_valid(ctrl_pdata->lge_pan_data->vddio_en)) {
			gpio_set_value((ctrl_pdata->lge_pan_data->vddio_en), 0);
			gpio_free(ctrl_pdata->lge_pan_data->vddio_en);
		}
	} else if (pinfo->lge_pan_info.panel_type == JDI_INCELL_CMD_PANEL) {
		if (gpio_is_valid(ctrl_pdata->lge_pan_data->dsv_ena)) {
			gpio_set_value((ctrl_pdata->lge_pan_data->dsv_ena), 0);
			gpio_free(ctrl_pdata->lge_pan_data->dsv_ena);
		}

		mdelay(1);

		if (gpio_is_valid(ctrl_pdata->rst_gpio)) {
			gpio_set_value((ctrl_pdata->rst_gpio), 0);
			PANEL_SEQUENCE(PIN_RESET, 0);
			gpio_free(ctrl_pdata->rst_gpio);
		}

		mdelay(1);

		if (gpio_is_valid(ctrl_pdata->lge_pan_data->vddio_en)) {
			gpio_set_value((ctrl_pdata->lge_pan_data->vddio_en), 0);
			gpio_free(ctrl_pdata->lge_pan_data->vddio_en);
		}

		ret = msm_dss_enable_vreg(
			ctrl_pdata->power_data[DSI_PANEL_PM].vreg_config,
			ctrl_pdata->power_data[DSI_PANEL_PM].num_vreg, 0);
		if (ret)
			pr_err("%s: failed to disable vregs for PANEL_PM\n",
						__func__);
	}

}

int lgd_qhd_command_mdss_dsi_event_handler(struct mdss_panel_data *pdata, int event, void *arg)
{
	int rc=0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	u32 mode;
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
			panel_data);

	switch (event) {
	case MDSS_EVENT_LINK_READY:
		if (!ctrl_pdata->ndx) {
			lgd_rsp_lcd_on_off = 1;
			lgd_deep_sleep(ctrl_pdata,
					DEEP_SLEEP_TO_LPWG,
					DEEP_SLEEP_TO_ACTIVE);

		}
		break;
	case MDSS_EVENT_BLANK:
		if (swipe_status) {
			mode = ABNORMAL_SWIPE;
			touch_notifier_call_chain(LCD_EVENT_TOUCH_SLEEP_STATUS,
					(void *)&mode);
		}
		break;
	case MDSS_EVENT_PANEL_OFF:
		if (ctrl_pdata->ndx) {
			dw8768_lgd_dsv_setting(0);    /* LPWG mode */
			if (ctrl_pdata->lge_pan_data->touch_driver_registered)
				touch_notifier_call_chain(
						LCD_EVENT_TOUCH_LPWG_ON, NULL);
			mdelay(20);
			if (proxy_sensor_status == PROXY_NEAR) {
				pr_info("%s: ACTIVE to DEEP\n",
						__func__);
				lgd_deep_sleep(ctrl_pdata,
						LPWG_TO_DEEP_SLEEP,
						LPWG_TO_DEEP_SLEEP);
			}
			lgd_rsp_lcd_on_off = 0;
		}
		break;
	}
	return rc;
}

int jdi_qhd_command_mdss_dsi_event_handler(struct mdss_panel_data *pdata, int event, void *arg)
{
	int rc=0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
     u32 mode;
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
			panel_data);

	switch (event) {
	case MDSS_EVENT_LINK_READY:
		if (!ctrl_pdata->ndx) {
			lgd_rsp_lcd_on_off = 1;
			if (jdi_deep_sleep == 1) {
				int param = 4;
				touch_notifier_call_chain(
						LCD_EVENT_TOUCH_SLEEP_STATUS,
						(void *)&param);
			}
		}
		break;
	case MDSS_EVENT_BLANK:
		if (swipe_status) {
			mode = ABNORMAL_SWIPE;
			touch_notifier_call_chain(LCD_EVENT_TOUCH_SLEEP_STATUS,
					(void *)&mode);
		}
		break;
	case MDSS_EVENT_PANEL_OFF:
		if (ctrl_pdata->ndx) {
			if (ctrl_pdata->lge_pan_data->touch_driver_registered)
				touch_notifier_call_chain(
						LCD_EVENT_TOUCH_LPWG_ON, NULL);
			msleep(50);
			if (proxy_sensor_status == PROXY_NEAR) {
				int param = 0;
				pr_info("%s: ACTIVE to DEEP\n",
						__func__);
				touch_notifier_call_chain(
						LCD_EVENT_TOUCH_SLEEP_STATUS,
						(void *)&param);
				jdi_deep_sleep = 1;
			}
			lgd_rsp_lcd_on_off = 0;

		}
		break;
	}
	return rc;
}

int lgd_sic_qhd_command_mdss_dsi_event_handler(struct mdss_panel_data *pdata, int event, void *arg)
{
	int rc=0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
			panel_data);

	switch (event) {
	case MDSS_EVENT_PANEL_OFF:
		if (ctrl_pdata->ndx) {
			SIC_is_doze = 2;
		}
		break;
	}
	return rc;
}

int lgd_qhd_command_msm_dss_enable_vreg(struct mdss_dsi_ctrl_pdata *ctrl_pdata, int enable)
{
	int ret =0;
	int i=0;


	if(enable)
	{
		if (ctrl_pdata->ndx == 0) {
			for (i = 0; i < DSI_MAX_PM; i++) {
				/*
				 * Core power module will be enabled when the
				 * clocks are enabled
				 */
				if (DSI_CORE_PM == i)
					continue;
				if (DSI_PANEL_PM == i) {
					if (first_touch_power_on == true)
						continue;
					else
						first_touch_power_on = true;
				}
				ret = msm_dss_enable_vreg(
						ctrl_pdata->power_data[i].vreg_config,
						ctrl_pdata->power_data[i].num_vreg, 1);
				if (ret) {
					pr_err("%s: failed to enable vregs for %s\n",
							__func__, __mdss_dsi_pm_name(i));
					return ret;
				}
			}

		}

	}else{

		if (ctrl_pdata->ndx) {
			for (i = DSI_MAX_PM - 1; i >= 0; i--) {
				/*
				 * Core power module will be disabled when the
				 * clocks are disabled
				 */
				if (DSI_CORE_PM == i)
					continue;
				if (DSI_PANEL_PM == i)
					continue;
				ret = msm_dss_enable_vreg(
						ctrl_pdata->power_data[i].vreg_config,
						ctrl_pdata->power_data[i].num_vreg, 0);
				if (ret)
					pr_err("%s: failed to disable vregs for %s\n",
							__func__, __mdss_dsi_pm_name(i));
			}
		}
	}
	return ret;

}

int lgd_qhd_command_pre_mdss_dsi_panel_power_ctrl(struct mdss_panel_data *pdata, int enable)
{
	int ret;
	if(enable)
	{
		struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

		if (pdata == NULL) {
			pr_err("%s: Invalid input data\n", __func__);
			return -EINVAL;
		}
		ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
		if (!ctrl_pdata->ndx) {
			if (!swipe_status) {
				if (ctrl_pdata->lge_pan_data->touch_driver_registered
						&& (proxy_sensor_status
							== PROXY_FAR))
					touch_notifier_call_chain(
							LCD_EVENT_TOUCH_LPWG_OFF, NULL);
			} else {
				pr_info("LPWG OFF Skipped:%d\n", swipe_status);
			}
			mdss_dsi_lcd_reset(pdata, 0);
			mdelay(2);
			dw8768_lgd_dsv_setting(1);
			mdelay(2);
		}

	}else{

	}

	return ret;

}

int jdi_qhd_command_pre_mdss_dsi_panel_power_ctrl(struct mdss_panel_data *pdata, int enable)
{
	int ret;
	if(enable)
	{
		struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

		if (pdata == NULL) {
			pr_err("%s: Invalid input data\n", __func__);
			return -EINVAL;
		}
		ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
		if (!ctrl_pdata->ndx) {
			if (!ctrl_pdata->ndx)
				if (ctrl_pdata->lge_pan_data->touch_driver_registered) {
					if(jdi_deep_sleep == 0)
						touch_notifier_call_chain(
								LCD_EVENT_TOUCH_LPWG_OFF, NULL);
				}
			jdi_deep_sleep = 0;
		}

	}else{

	}

	return ret;

}

int lgd_sic_qhd_command_pre_mdss_dsi_panel_power_ctrl(struct mdss_panel_data *pdata, int enable)
{
	int ret;
	if(enable)
	{
		struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

		if (pdata == NULL) {
			pr_err("%s: Invalid input data\n", __func__);
			return -EINVAL;
		}
		ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
		if (!ctrl_pdata->ndx) {
			//noti to touch hw reset process
			if (ctrl_pdata->lge_pan_data->touch_driver_registered) {
				touch_notifier_call_chain(LCD_EVENT_HW_RESET, NULL);
			}
			SIC_is_doze = 1;
			mdss_dsi_lcd_reset(pdata, 0);
			mdelay(2);
			mdss_dsi_lcd_reset(pdata, 1);
			mdelay(2);
		}

	}else{

	}

	return ret;

}

int lgd_qhd_command_post_mdss_dsi_panel_power_ctrl(struct mdss_panel_data *pdata, int enable)
{
	int ret;
	if(enable)
	{
		struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

		if (pdata == NULL) {
			pr_err("%s: Invalid input data\n", __func__);
			return -EINVAL;
		}

		ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

		if (ctrl_pdata->ndx)
			mdss_dsi_lcd_reset(pdata, 1);
	}else{

	}
	return ret;

}

int lgd_qhd_command_post_mdss_dsi_blank(struct mdss_panel_data *pdata, int power_state)
{
	int ret=0;
	return ret;
}

int lgd_qhd_command_mdss_dsi_ctrl_probe(struct platform_device *pdev)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	ctrl_pdata = platform_get_drvdata(pdev);

	ctrl_pdata->lge_pan_data->touch_driver_registered = false;

	ctrl_pdata->notif.notifier_call = lcd_notifier_callback;
	if (touch_register_client(&ctrl_pdata->notif) != 0)
		pr_err("Failed to register callback\n");

	return 0;
}

int lgd_qhd_command_dsi_panel_device_register(struct device_node *pan_node,
						struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	struct device_node *dsi_ctrl_np = NULL;
	struct platform_device *ctrl_pdev = NULL;
	char *dsv_vendor = lge_get_dsv_vendor();

	pr_info("%s: dsv_vendor = %s\n", __func__, dsv_vendor);

	dsi_ctrl_np = of_parse_phandle(pan_node,
	"qcom,mdss-dsi-panel-controller", 0);

	if (!dsi_ctrl_np) {
		pr_err("%s: Dsi controller node not initialized\n", __func__);
		return -EPROBE_DEFER;
	}

	ctrl_pdev = of_find_device_by_node(dsi_ctrl_np);

	if (!strcmp(dsv_vendor, "TI"))
		ctrl_pdata->lge_pan_data->dsv_manufacturer = DSV_TPS65132;
	else if (!strcmp(dsv_vendor, "SM"))
		ctrl_pdata->lge_pan_data->dsv_manufacturer = DSV_SM5107;
	else if (!strcmp(dsv_vendor, "DW"))
		ctrl_pdata->lge_pan_data->dsv_manufacturer = DSV_DW8768;

	panel_not_connected = lge_get_lk_panel_status();
	pr_debug("%s: lk panel init fail[%d]\n",
			__func__, panel_not_connected);

	ctrl_pdata->lge_pan_data->vddio_en = of_get_named_gpio(ctrl_pdev->dev.of_node,
			 "qcom,platform-vddio_en-gpio", 0);

	ctrl_pdata->lge_pan_data->dsv_ena = of_get_named_gpio(ctrl_pdev->dev.of_node,
			 "qcom,platform-avdd-gpio", 0);

#if defined(CONFIG_MACH_MSM8992_P1_CN) \
	|| defined(CONFIG_MACH_MSM8992_P1_GLOBAL_COM)
	if (lge_get_sim_type() == 2) {
		ctrl_pdata->lge_pan_data->touch_io = of_get_named_gpio(ctrl_pdev->dev.of_node,
				"lge,platform-touch-io", 0);
	}
#endif
	if (!gpio_is_valid(ctrl_pdata->lge_pan_data->dsv_ena))
		pr_err("%s:%d, dsv_ena gpio(%d) not specified\n",
				__func__, __LINE__, ctrl_pdata->lge_pan_data->dsv_ena);

	if (ctrl_pdata->lge_pan_data->dsv_manufacturer == DSV_TPS65132) {
		ctrl_pdata->lge_pan_data->dsv_enb = of_get_named_gpio(ctrl_pdev->dev.of_node,
				"qcom,platform-avee-gpio", 0);

		if (!gpio_is_valid(ctrl_pdata->lge_pan_data->dsv_enb))
			pr_err("%s:%d, dsv_enb gpio(%d) not specified\n",
				__func__, __LINE__, ctrl_pdata->lge_pan_data->dsv_enb);
	}

	pr_info("%s vddio_en = %d\n", __func__, ctrl_pdata->lge_pan_data->vddio_en);
	pr_info("%s dsv_ena = %d, dsv_enb = %d, dsv_manufacturer = %d\n",
			__func__, ctrl_pdata->lge_pan_data->dsv_ena,
			ctrl_pdata->lge_pan_data->dsv_enb, ctrl_pdata->lge_pan_data->dsv_manufacturer);

	return 0;
}
