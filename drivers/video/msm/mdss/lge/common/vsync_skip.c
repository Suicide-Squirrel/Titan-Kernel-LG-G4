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

#include "vsync_skip.h"
#include "lge_display_feature.h"
#include <linux/kallsyms.h>

#if IS_ENABLED(CONFIG_LGE_DISPLAY_VSYNC_SKIP)
struct oem_mdss_data_type oem_vsync_skip;

ssize_t fps_store(struct device *dev,
	 struct device_attribute *attr, const char *buf, size_t count)
{
	ulong fps;

	if (!count)
		return -EINVAL;

	fps = simple_strtoul(buf, NULL, 10);

	if (fps == 0 || fps >= LGE_DISPLAY_MAX_FPS) {
		oem_vsync_skip.enable_skip_vsync = 0;
		oem_vsync_skip.skip_value = 0;
		oem_vsync_skip.weight = 0;
		oem_vsync_skip.bucket = 0;
		oem_vsync_skip.skip_count = 0;
		oem_vsync_skip.skip_ratio = LGE_DISPLAY_MAX_FPS;
		oem_vsync_skip.skip_first = false;
		pr_info("Disable frame skip.\n");
	} else {
		oem_vsync_skip.enable_skip_vsync = 1;
		oem_vsync_skip.skip_value = (LGE_DISPLAY_MAX_FPS<<16)/fps;
		oem_vsync_skip.weight = (1<<16);
		oem_vsync_skip.bucket = 0;
		oem_vsync_skip.skip_ratio = fps;
		oem_vsync_skip.skip_first = false;
		pr_info("Enable frame skip: Set to %lu fps.\n", fps);
	}
	return count;
}

ssize_t fps_show(struct device *dev,
	 struct device_attribute *attr, char *buf)
{
	int r = 0;
	r = snprintf(buf, PAGE_SIZE, "enable_skip_vsync=%d\nweight=%lu\nskip_value=%lu\nbucket=%lu\nskip_count=%lu\n",
	    oem_vsync_skip.enable_skip_vsync,
	    oem_vsync_skip.weight,
	    oem_vsync_skip.skip_value,
	    oem_vsync_skip.bucket,
	    oem_vsync_skip.skip_count);
	return r;
}

ssize_t fps_ratio_show(struct device *dev,
	 struct device_attribute *attr, char *buf)
{
	int r = 0;
	r = snprintf(buf, PAGE_SIZE, "%d\n", oem_vsync_skip.skip_ratio);
	return r;
}

static struct fb_info **fbi_list;
int fps_cnt_before_lge = 0;

ssize_t fps_fcnt_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int r = 0;
	struct msm_fb_data_type *mfd;
	struct mdss_overlay_private *mdp5_data;
	struct mdss_mdp_ctl *ctl;

	fbi_list = (struct fb_info **)kallsyms_lookup_name(
			"fbi_list");

	if (fbi_list[0] == NULL)
		goto ERROR;

	mfd = fbi_list[0]->par;
	if (mfd == NULL)
		goto ERROR;

	mdp5_data = mfd_to_mdp5_data(mfd);
	if (mdp5_data == NULL)
		goto ERROR;

	ctl = mdp5_data->ctl;
	if (ctl == NULL)
		goto ERROR;

	r = snprintf(buf, PAGE_SIZE, "%d\n", ctl->play_cnt-fps_cnt_before_lge);
	fps_cnt_before_lge = ctl->play_cnt;
	return r;

ERROR:
	r = snprintf(buf, PAGE_SIZE, "0\n");
	return r;
}
#endif
