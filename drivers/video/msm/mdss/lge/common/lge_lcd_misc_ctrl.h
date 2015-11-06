/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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

#ifndef LGE_LCD_MISC_CTRL_H
#define LGE_LCD_MISC_CTRL_H

#include <linux/list.h>
#include "../../mdss_mdp.h"
#include "../../mdss_dsi.h"

struct lcd_platform_data {
	int (*set_values) (int *tun_lcd_t);
	int (*get_values) (int *tun_lcd_t);
};

void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds);
ssize_t mdss_get_porch_show(struct device *dev,
		struct device_attribute *attr, char *buf);
ssize_t mdss_set_porch_store(struct device *dev,
		struct device_attribute *addr, const char *buf, size_t count);
ssize_t mdss_get_timing_show(struct device *dev,
		struct device_attribute *attr, char *buf);
ssize_t mdss_set_timing_store(struct device *dev,
		struct device_attribute *addr, const char *buf, size_t count);
ssize_t mdss_get_tclk_show(struct device *dev,
		struct device_attribute *attr, char *buf);
ssize_t mdss_set_tclk_store(struct device *dev,
		struct device_attribute *addr, const char *buf, size_t count);

#endif /* LGE_LCD_MISC_CTRL_H */
