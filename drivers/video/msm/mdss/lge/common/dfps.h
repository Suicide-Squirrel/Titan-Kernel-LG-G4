/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*

	Display driver for dynamic scaling support

Copyright(c) 2013 by LG Electronics. All Rights Reserved.
	Sangduck Kim <sangduck.kim@lge.com>
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
/*==========================================================================

	 EDIT HISTORY FOR FILE

  This section contains comments describing changes made to this file.
  Notice that changes are listed in reverse chronological order.

when       who        what, where, why
--------   --------   -------------------------------------------------------
12/01/13   sdkim   Created file
======================================================-====================*/
#ifndef DFPS_H
#define DFPS_H

#include <linux/devfreq.h>
#include "../../mdss_panel.h"

struct display_fps_data {
	u32 prev_fcount;
	unsigned int fps;
};

struct lge_display_data {
	struct devfreq *devfreq;
	struct opp *curr_opp;
	struct device *dev;

	struct delayed_work wq_lowfreq;
	struct delayed_work wq_highfreq;
	struct notifier_block nb_pm;
	struct mutex lock;
	struct display_fps_data fps_data;
};

struct display_opp_table {
	unsigned int idx;
	unsigned long freq;
	unsigned long volt;
};

enum lge_display_clk_level_idx {
	LV_0 = 0,
	LV_1,
/*	LV_2,*/
	_LV_END_
};

enum lge_display_type {
	TYPE_DISPLAY_LGE,
};

#if defined (CONFIG_LGE_DYNAMIC_FPS_LUT)
/*#define MDSS_FPS_LUT_SIZE 21*/
#endif

#ifdef CONFIG_Z2_LGD_POLED_PANEL
struct display_opp_table lge_display_opp_table[] = {
	{LV_0, 48, 0},
	{LV_1, 58, 0},
};

struct devfreq_simple_ondemand_data lge_display_ondemand_data = {
	.downdifferential = 33,
	.upthreshold = 46,
};
#else
struct display_opp_table lge_display_opp_table[] = {
	{LV_0, 50, 0},
	{LV_1, 60, 0},
};
struct devfreq_simple_ondemand_data lge_display_ondemand_data = {
	.downdifferential = 35,
	.upthreshold = 48,
};
#endif

void lge_release_pm(struct device *device);
#ifdef CONFIG_PM
int lge_display_suspend(struct device *dev);
int lge_display_resume(struct device *dev);
#endif
int lge_display_remove(struct platform_device *pdev);
int lge_display_probe(struct platform_device *pdev);
int lge_display_profile_target(struct device *dev,
unsigned long *_freq, u32 options);
int lge_display_get_dev_status(struct device *dev,
struct devfreq_dev_status *stat);
void lge_display_profile_exit(struct device *dev);
int lge_opp_get_idx(unsigned long new_freq,
struct display_opp_table *table);
void lge_display_read_fps(struct lge_display_data *data,
struct devfreq_dev_status *stat);
void lge_display_set_lowfreq(struct work_struct *work);
void lge_display_set_highfreq(struct work_struct *work);
int lge_display_send_event_to_mdss_display(unsigned long val, void *v);
int lge_display_pm_notifier_callback(struct notifier_block *this,
unsigned long event, void *_data);
int lge_display_dev_platform_register(void);
void lge_display_dev_platform_unregister(void);

#endif
