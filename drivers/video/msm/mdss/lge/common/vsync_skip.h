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

#ifndef VSYNC_SKIP_H
#define VSYNC_SKIP_H

#include <linux/list.h>
#include "../../mdss_mdp.h"

#if IS_ENABLED(CONFIG_LGE_DISPLAY_VSYNC_SKIP)
struct oem_mdss_data_type {
	char enable_skip_vsync;
	ulong skip_value;
	ulong weight;
	ulong bucket;
	ulong skip_count;
	int skip_ratio;
	bool skip_first;
};

ssize_t fps_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
ssize_t fps_show(struct device *dev,
	struct device_attribute *attr, char *buf);
ssize_t fps_ratio_show(struct device *dev,
	struct device_attribute *attr, char *buf);
ssize_t fps_fcnt_show(struct device *dev,
	struct device_attribute *attr, char *buf);

extern struct oem_mdss_data_type oem_vsync_skip;

#endif

#endif /* VSYNC_SKIP_H */
