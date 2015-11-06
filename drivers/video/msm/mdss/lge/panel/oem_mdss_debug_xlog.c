/* Copyright (c) 2014-2015, The Linux Foundation. All rights reserved
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

#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/ktime.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>

#include "oem_mdss_dsi_common.h"
#include "oem_mdss_dsi.h"
#include "mdss_debug.h"

unsigned int reg_dump_enable;
extern int panel_not_connected;

int lgd_qhd_command_mdss_xlog_tout_handler_default(void)
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

int lgd_qhd_command_mdss_create_xlog_debug(struct mdss_debug_data *mdd)
{
	int rc=0;
	return rc;
}
