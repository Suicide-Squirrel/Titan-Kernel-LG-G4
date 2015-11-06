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

#ifndef OEM_MDSS_PANEL_INFO_H
#define OEM_MDSS_PANEL_INFO_H

#include <linux/list.h>
#include <linux/mdss_io_util.h>
#include <linux/irqreturn.h>
#include <linux/pinctrl/consumer.h>
#include <linux/gpio.h>

enum lcd_panel_type {
	JDI_INCELL_CMD_PANEL,
	JDI_INCELL_VIDEO_PANEL,
	LGD_INCELL_CMD_PANEL,
	LGD_SIC_INCELL_CMD_PANEL,
	UNKOWN_PANEL
};

enum lge_panel_mode_switch_state {
	LGE_PANEL_MODE_NONE = -1,
	LGE_PANEL_MODE_U0 = 0,
	LGE_PANEL_MODE_U1,
	LGE_PANEL_MODE_U2,
	LGE_PANEL_MODE_U3,
	LGE_PANEL_MODE_U3_READY,
	LGE_PANEL_MODE_NUM
};

struct lge_pan_info {
	int blmap_size;
	int *blmap;
	int bl_store_mode;
#if defined(CONFIG_LGE_BLMAP_STORE_MODE)
	int *blmap_store_mode;
	int panel_type;
#endif
#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTENDED_PANEL)
	/* Following is used for lcd mode switch, u0 u1 u2 u3 */
	enum lge_panel_mode_switch_state cur_panel_mode;
	int lge_panel_send_on_cmd;
	int lge_panel_send_off_cmd;
#endif
};

#endif /* OEM_MDSS_PANEL_INFO_H */
