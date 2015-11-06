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

#ifndef OEM_MDSS_DSI_H
#define OEM_MDSS_DSI_H

#include <linux/list.h>
#include <linux/mdss_io_util.h>
#include <linux/irqreturn.h>
#include <linux/pinctrl/consumer.h>
#include <linux/gpio.h>

int lgd_qhd_command_pre_msm_dss_enable_vreg(struct mdss_panel_data *pdata, int enable);
int lgd_qhd_command_post_msm_dss_enable_vreg(struct mdss_panel_data *pdata, int enable);
int lgd_qhd_command_pre_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable);
int lgd_qhd_command_post_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable);

int lgd_qhd_command_pre_mdss_dsi_panel_power_ctrl(struct mdss_panel_data *pdata, int enable);
int jdi_qhd_command_pre_mdss_dsi_panel_power_ctrl(struct mdss_panel_data *pdata, int enable);
int lgd_sic_qhd_command_pre_mdss_dsi_panel_power_ctrl(struct mdss_panel_data *pdata, int enable);

int lgd_qhd_command_post_mdss_dsi_panel_power_ctrl(struct mdss_panel_data *pdata, int enable);
int lgd_qhd_command_post_mdss_dsi_blank(struct mdss_panel_data *pdata, int power_state);
int lgd_qhd_command_post_mdss_dsi_panel_on(struct mdss_panel_data *pdata);
int lgd_qhd_command_post_mdss_dsi_panel_off(struct mdss_panel_data *pdata);

int lgd_qhd_command_mdss_dsi_event_handler(struct mdss_panel_data *pdata, int event, void *arg);
int jdi_qhd_command_mdss_dsi_event_handler(struct mdss_panel_data *pdata, int event, void *arg);
int lgd_sic_qhd_command_mdss_dsi_event_handler(struct mdss_panel_data *pdata, int event, void *arg);

int lgd_qhd_command_msm_dss_enable_vreg(struct mdss_dsi_ctrl_pdata *ctrl_pdata, int enable);
int lgd_qhd_command_mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int lgd_qhd_command_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable);
int jdi_qhd_command_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable);
int lgd_sic_qhd_command_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable);

int lgd_qhd_command_mdss_dsi_lane_config(struct mdss_panel_data *pdata, int enable);

int lgd_qhd_command_dsi_panel_device_register(struct device_node *pan_node,
						struct mdss_dsi_ctrl_pdata *ctrl_pdata);

int lgd_qhd_command_mdss_dsi_ctrl_probe(struct platform_device *pdev);

int lgd_qhd_command_mdss_panel_parse_dt(struct device_node *pan_node,
					struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int lgd_qhd_command_panel_device_create(struct device_node *pan_node,
					struct mdss_dsi_ctrl_pdata *ctrl_pdata);

int lgd_qhd_command_mdss_dsi_cmdlist_commit(struct mdss_dsi_ctrl_pdata *ctrl, int from_mdp);
int lgd_qhd_command_mdss_dsi_panel_init(struct device_node *node, struct mdss_dsi_ctrl_pdata *ctrl_pdata, bool cmd_cfg_cont_splash);
int lgd_qhd_command_dump_mdss_reg(void);
int lgd_qhd_command_mdss_xlog_tout_handler_default(void);
int lgd_qhd_command_mdss_create_xlog_debug(struct mdss_debug_data *mdd);
int lgd_qhd_command_mdss_dsi_panel_bl_ctrl(struct mdss_panel_data *pdata, u32 bl_level);

enum {
LGD_R69007_QHD_DUALDSI_CMD_PANEL,
JDI_NT36850_QHD_DUALDSI_CMD_PANEL,
LGD_LG4943_QHD_DUALDSI_CMD_PANEL,
LGD_LG4945_QHD_DUALDSI_CMD_PANEL,
UNKNOWN_PANEL
};

enum {
	P1_LDO_TPVCI = 0,
	P1_LDO_VPNL,
	PP_LDO_VPNL_TOUCH,
	MAX_LDO,
};

#endif /* OEM_MDSS_DSI_H */
