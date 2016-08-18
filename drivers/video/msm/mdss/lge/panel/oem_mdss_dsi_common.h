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

#ifndef OEM_MDSS_DSI_COMMON_H
#define OEM_MDSS_DSI_COMMON_H

#include <linux/list.h>
#include <linux/mdss_io_util.h>
#include <linux/irqreturn.h>
#include <linux/pinctrl/consumer.h>
#include <linux/gpio.h>
#include <linux/of_device.h>
#include <linux/of_device.h>
#include <linux/kernel.h>

#include "../../mdss_debug.h"
#include "../../mdss_panel.h"
#include "../../mdss_dsi.h"
#include "../../mdss_fb.h"


#define DSV_TPS65132 1
#define DSV_SM5107 2
#define DSV_DW8768 3

#define LPWG_TO_DEEP_SLEEP 0
#define DEEP_SLEEP_TO_LPWG 1
#define DEEP_SLEEP_TO_ACTIVE 3

#define PROXY_FAR 0
#define PROXY_NEAR 1

//LGE_UPDATE_S (june1014.lee@lge.com. 2015.03.04). SRE
#if defined(CONFIG_LGE_P1_SRE_SUPPORTED)
#define	SRE_CHANGE_OFF	0
#define	SRE_CHANGE_ON	1
int mdss_dsi_panel_sre_apply(unsigned int enabled);
#endif
//LGE_UPDATE_E (june1014.lee@lge.com. 2015.03.04). SRE

#define XLOG_DEFAULT_PANIC 0
#define XLOG_DEFAULT_REGDUMP_ENABLE 0

extern int mdss_dsi_lcd_reset(struct mdss_panel_data *pdata, int enable);
extern int lgd_deep_sleep(struct mdss_dsi_ctrl_pdata *ctrl_pdata, int mode,
	       int is_no_sleep);
extern void mdss_dsi_ctrl_shutdown(struct platform_device *pdev);
void mdss_dsi_stub_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds);
void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
		struct dsi_panel_cmds *pcmds);
extern int swipe_status;
//extern struct mdss_dbg_xlog mdss_dbg_xlog;

struct lge_pan_data {
	int dsv_ena;
	int dsv_enb;
	int dsv_manufacturer;
	int vdd_ldo;
	int vddio_en;
#if defined(CONFIG_MACH_MSM8992_P1_CN) \
|| defined(CONFIG_MACH_MSM8992_P1_GLOBAL_COM)
	int touch_io;
#endif
	bool touch_driver_registered;
	struct notifier_block notif;
	struct dsi_panel_cmds clk_on_cmds;
	struct dsi_panel_cmds clk_off_cmds;
	struct dsi_panel_cmds rsp_nvm_write;
	bool do_rsp_nvm_write;
};

#define MAX_PANEL_ID_LEN 64

struct panel_list {
	char name[MAX_PANEL_ID_LEN];
	uint32_t id;
};

struct lge_mdss_dsi_interface {
	int (*pre_msm_dss_enable_vreg)(struct mdss_panel_data *pdata, int enable);
	int (*post_msm_dss_enable_vreg)(struct mdss_panel_data *pdata, int enable);
	int (*pre_mdss_dsi_panel_reset)(struct mdss_panel_data *pdata, int enable);
	int (*post_mdss_dsi_panel_reset)(struct mdss_panel_data *pdata, int enable);
	int (*pre_mdss_dsi_panel_power_ctrl)(struct mdss_panel_data *pdata, int enable);
	int (*post_mdss_dsi_panel_power_ctrl)(struct mdss_panel_data *pdata, int enable);
	int (*post_mdss_dsi_blank)(struct mdss_panel_data *pdata, int power_state);
	int (*post_mdss_dsi_panel_on)(struct mdss_panel_data *pdata);
	int (*post_mdss_dsi_panel_off)(struct mdss_panel_data *pdata);
	int (*lge_mdss_dsi_event_handler)(struct mdss_panel_data *pdata, int event, void *arg);
	int (*lge_msm_dss_enable_vreg)(struct mdss_dsi_ctrl_pdata *ctrl_pdata, int enable);
	int (*lge_mdss_dsi_request_gpios)(struct mdss_dsi_ctrl_pdata *ctrl_pdata);
	int (*lge_mdss_dsi_panel_reset)(struct mdss_panel_data *pdata, int enable);
	int (*lge_mdss_dsi_lane_config)(struct mdss_panel_data *pdata, int enable);
	int (*lge_mdss_dsi_ctrl_probe)(struct platform_device *pdev);
	int (*lge_dsi_panel_device_register)(struct device_node *pan_node, struct mdss_dsi_ctrl_pdata *ctrl_pdata);
	int (*lge_mdss_panel_parse_dt)(struct device_node *pan_node, struct mdss_dsi_ctrl_pdata *ctrl_pdata);
	int (*lge_panel_device_create)(struct device_node *pan_node, struct mdss_dsi_ctrl_pdata *ctrl_pdata);
	int (*lge_mdss_dsi_cmdlist_commit)(struct mdss_dsi_ctrl_pdata *ctrl, int from_mdp);
	int (*lge_mdss_dsi_panel_init)(struct device_node *node, struct mdss_dsi_ctrl_pdata *ctrl_pdata, bool cmd_cfg_cont_splash);
	int (*lge_dump_mdss_reg)(void);
	int (*lge_mdss_xlog_tout_handler_default)(void);
	int (*lge_mdss_create_xlog_debug)(struct mdss_debug_data *mdd);
	int (*lge_mdss_dsi_panel_bl_ctrl)(struct mdss_panel_data *pdata, u32 bl_level);
};

int pre_msm_dss_enable_vreg(struct mdss_panel_data *pdata, int enable);
int post_msm_dss_enable_vreg(struct mdss_panel_data *pdata, int enable);
int pre_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable);
int post_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable);
int pre_mdss_dsi_panel_power_ctrl(struct mdss_panel_data *pdata, int enable);
int post_mdss_dsi_panel_power_ctrl(struct mdss_panel_data *pdata, int enable);
int post_mdss_dsi_blank(struct mdss_panel_data *pdata, int power_state);
int post_mdss_dsi_panel_on(struct mdss_panel_data *pdata);
int post_mdss_dsi_panel_off(struct mdss_panel_data *pdata);

int lge_mdss_dsi_event_handler(struct mdss_panel_data *pdata, int event, void *arg);
int lge_msm_dss_enable_vreg(struct mdss_dsi_ctrl_pdata *ctrl_pdata, int enable);
int lge_mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int lge_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable);

int lge_mdss_dsi_lane_config(struct mdss_panel_data *pdata, int enable);
int lge_dsi_panel_device_register(struct device_node *pan_node, struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int lge_mdss_dsi_ctrl_probe(struct platform_device *pdev);
int lge_mdss_panel_parse_dt(struct device_node *pan_node, struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int lge_panel_device_create(struct device_node *pan_node, struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int lge_mdss_dsi_cmdlist_commit(struct mdss_dsi_ctrl_pdata *ctrl, int from_mdp);
int lge_mdss_dsi_panel_init(struct device_node *node, struct mdss_dsi_ctrl_pdata *ctrl_pdata, bool cmd_cfg_cont_splash);
int lge_dump_mdss_reg(void);
int lge_mdss_xlog_tout_handler_default(void);
int lge_mdss_create_xlog_debug(struct mdss_debug_data *mdd);
int lge_mdss_dsi_panel_bl_ctrl(struct mdss_panel_data *pdata, u32 bl_level);

void lge_mdss_dsi_seperate_panel_api_init(struct lge_mdss_dsi_interface *pdata, struct device_node *dsi_pan_node);

int get_lge_panel_id(void);

#if IS_ENABLED(CONFIG_LGE_DISPLAY_DUAL_BACKLIGHT)
int lge_get_bklt_type(void);
int lge_set_bklt_type(int bklt_type);
int lge_get_mode_type(void);
#endif

extern struct mdss_panel_data *pdata_base;
extern struct lge_mdss_dsi_interface lge_mdss_dsi;
#endif /* OEM_MDSS_DSI_COMMON_H */
