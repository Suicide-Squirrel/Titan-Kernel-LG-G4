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

#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/msm_mdp.h>
#include <linux/memblock.h>
#include <linux/sort.h>
#include <linux/sw_sync.h>

#include "oem_mdss_dsi_common.h"
#include "oem_mdss_dsi.h"

extern int panel_not_connected;
extern unsigned int reg_dump_enable;

int lgd_qhd_command_dump_mdss_reg(void)
{
	int rc=0;

	if(!reg_dump_enable)
	{
		rc=1;
		return rc;
	}
	if(panel_not_connected) {
		rc=1;
		pr_debug("[%s] reg-dump skipped \n", __func__);
		return rc;
	}
	return rc;
}
