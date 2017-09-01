/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved
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
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/iopoll.h>
#include <linux/kthread.h>

#include "oem_mdss_dsi_common.h"
#include "oem_mdss_dsi.h"

int lgd_qhd_command_mdss_dsi_cmdlist_commit(struct mdss_dsi_ctrl_pdata *ctrl, int from_mdp)
{
	u32 ctrl_rev;
	if (!from_mdp) {
		mdss_dsi_clk_ctrl(ctrl, DSI_BUS_CLKS, 1);
		ctrl_rev = MIPI_INP(ctrl->ctrl_base);
		mdss_dsi_clk_ctrl(ctrl, DSI_BUS_CLKS, 0);
	} else {
		ctrl_rev = MIPI_INP(ctrl->ctrl_base);
	}
	return ctrl_rev;
}
