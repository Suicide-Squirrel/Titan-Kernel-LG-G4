/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*

       Display driver for dynamic scaling support

GENERAL DESCRIPTION
  LG G3 LCD DEVFREQ Drrver for Dynamic refresh rate

Copyright(c) 2013 by LG Electronics. All Rights Reserved.
	Sangduck Kim <sangduck.kim@lge.com>
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
/*==========================================================================

	EDIT HISTORY FOR FILE

  This section contains comments describing changes made to this file.
  Notice that changes are listed in reverse chronological order.

when       who	  what, where, why
--------   --------   -------------------------------------------------------
12/01/13   sdkim   Created file
======================================================-====================*/
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/devfreq.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/pm_opp.h>
#include <linux/kallsyms.h>

#include "dfps.h"
#include "lge_display_feature.h"
#include "../../mdss_fb.h"
#include "../../mdss_mdp.h"

#define _trace_macro_(tag, a, ...)      printk(KERN_INFO "[DFPSv1|"#tag"]"a, ##__VA_ARGS__)
#define trace(...)             _trace_macro_(INFO, ##__VA_ARGS__)

extern int mdss_mdp_ctl_update_fps(struct mdss_mdp_ctl *ctl, int fps);
struct fb_info **fbi_list;
int pdev_list_cnt;

struct platform_device lge_display_device = {
	.name = "lge-display",
	.id = TYPE_DISPLAY_LGE,
	.dev.release = lge_release_pm,
};

const struct platform_device_id display_ids[] = {
	{"lge-display", TYPE_DISPLAY_LGE},
};

#ifdef CONFIG_PM
const struct dev_pm_ops lge_display_dev_pm_ops = {
	.suspend = lge_display_suspend,
	.resume = lge_display_resume,
};
#endif

struct platform_driver lge_display_driver = {
	.probe = lge_display_probe,
	.remove = lge_display_remove,
	.id_table = display_ids,
	.driver = {
		.name = "lge-display",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &lge_display_dev_pm_ops,
#endif
	},
};

static int lge_cur_level = _LV_END_ - 1;

static int lge_get_cur_freq(struct device *dev, unsigned long *freq)
{
	*freq = lge_display_opp_table[lge_cur_level].freq;
/*	trace("%s, cur_freq=%lu\n", __FUNCTION__, lge_display_opp_table[lge_cur_level].freq);*/
	return 0;
}

/* Profile Display OPP*/
struct devfreq_dev_profile lge_display_profile = {
	.initial_freq = LGE_DISPLAY_MAX_FPS,
	.polling_ms = 1000,
	.get_dev_status = lge_display_get_dev_status,
	.target = lge_display_profile_target,
	.exit = lge_display_profile_exit,
	.get_cur_freq = lge_get_cur_freq,
};

void lge_release_pm(struct device *dev)
{
	/*trace("%s\n", __FUNCTION__);*/
}

#ifdef CONFIG_PM
int lge_display_suspend(struct device *dev)
{
	/*trace("%s\n", __FUNCTION__);*/
	return 0;
}

int lge_display_resume(struct device *dev)
{
	/*trace("%s\n", __FUNCTION__);*/
	return 0;
}
#endif

int lge_display_probe(struct platform_device *pdev)
{
	struct lge_display_data *data;
	struct device *dev = &pdev->dev;
	int ret = 0;
	struct opp *opp;
	int i;

	data = kzalloc(sizeof(struct lge_display_data), GFP_KERNEL);
	if (!data) {
		dev_err(dev, "cannot_allocate memory.\n");
		return -ENOMEM;
	}
	data->dev = dev;
	mutex_init(&data->lock);

	/* register opp entries */
	for (i = 0; i < _LV_END_; i++) {
		ret = dev_pm_opp_add(dev, lge_display_opp_table[i].freq,
				lge_display_opp_table[i].volt);
		if (ret) {
			dev_err(dev, "cannot add opp entries.\n");
			goto err_alloc_mem;
		}
	}

	/* find opp entry with init frequency */

	opp = dev_pm_opp_find_freq_floor(dev, &lge_display_profile.initial_freq);
	if (IS_ERR(opp)) {
		dev_err(dev, "invalid initial frequency %lu.\n",
				lge_display_profile.initial_freq);
		ret = PTR_ERR(opp);
		goto err_alloc_mem;
	}
	data->curr_opp = opp;

	/* initialize qos */
	/* TODO*/

	/* register lge_display to devfreq framework */
	data->devfreq = devfreq_add_device(dev, &lge_display_profile,
			"simple_ondemand", &lge_display_ondemand_data);
	if (IS_ERR(data->devfreq)) {
		ret = PTR_ERR(data->devfreq);
		dev_err(dev, "failed to add devfreq: %d\n", ret);
		goto err_alloc_mem;
	}

	devfreq_register_opp_notifier(dev, data->devfreq);

	/* register lge_display as client to pm notifier */
	memset(&data->nb_pm, 0, sizeof(data->nb_pm));
	data->nb_pm.notifier_call = lge_display_pm_notifier_callback;
	ret = register_pm_notifier(&data->nb_pm);
	if (ret < 0) {
		dev_err(dev, "failed to get pm notifier: %d\n", ret);
		goto err_add_devfreq;
	}

	platform_set_drvdata(pdev, data);

	return 0;
err_add_devfreq:
	devfreq_remove_device(data->devfreq);
err_alloc_mem:
	kfree(data);
	return ret;
}

int lge_display_remove(struct platform_device *pdev)
{
	struct lge_display_data *data = pdev->dev.platform_data;

	unregister_pm_notifier(&data->nb_pm);
	devfreq_remove_device(data->devfreq);
	kfree(data);

	return 0;
}

u32 msm_fb_read_frame_count(void)
{
	struct msm_fb_data_type *mfd = fbi_list[0]->par;
	struct mdss_overlay_private *mdp5_data = mfd_to_mdp5_data(mfd);
	struct mdss_mdp_ctl *ctl = mdp5_data->ctl;
	return ctl->play_cnt;
/*	struct mdss_panel_info *mdss_panel = mfd->panel_info;
	trace("%s, %d\n", __FUNCTION__, mdss_panel->frame_count);
	return mdss_panel->frame_count;*/
}

u32 msm_fb_read_frame_rate(void)
{
	int rdfps = 0;
	struct msm_fb_data_type *mfd = fbi_list[0]->par;
	struct mdss_panel_info *mdss_panel = mfd->panel_info;

	struct mdss_overlay_private *mdp5_data = mfd_to_mdp5_data(mfd);
	mutex_lock(&mdp5_data->dfps_lock);
	rdfps = mdss_panel_get_framerate(mdss_panel);
	mutex_unlock(&mdp5_data->dfps_lock);

	return rdfps;
}

void lge_display_read_fps(struct lge_display_data *data, struct devfreq_dev_status *stat)
{
	unsigned long fcount = msm_fb_read_frame_count();

	data->fps_data.fps = (fcount - data->fps_data.prev_fcount);
	data->fps_data.prev_fcount = fcount;

	stat->current_frequency = dev_pm_opp_get_freq(data->curr_opp);
	stat->total_time = msm_fb_read_frame_rate();
/*	stat->total_time = lge_display_opp_table[lge_cur_level].freq;*/
	stat->busy_time = (data->fps_data.fps) * (1000 / lge_display_profile.polling_ms);

	/* trace("total_time=%lu, busy_time=%lu, util=%lu\n",
		stat->total_time, stat->busy_time, (stat->busy_time * 100 / stat->total_time));
	*/
}

int lge_display_send_event_to_mdss_display(unsigned long val, void *v)
{
	int wdfps = 0;
	int ret = 0;
	struct mdss_panel_data *pdata;
	struct msm_fb_data_type *mfd = fbi_list[0]->par;
	struct mdss_overlay_private *mdp5_data = mfd_to_mdp5_data(mfd);
	struct mdss_mdp_ctl *ctl = mdp5_data->ctl;
	pr_debug("send_event_to_mdss_display, val=%lu, freq=%lu\n", val, lge_display_opp_table[val].freq);

	if (!ctl || !(ctl->power_state) || !(mfd->panel_power_state)) {
		pr_err("Panel is off...FPS will not be changed\n");
		return -EPERM;
	}
	if (ctl->play_cnt == 0) {
		pr_err("Vsync is not enabled...FPS will not be changed\n");
		return -EPERM;
	}
	pdata = dev_get_platdata(&mfd->pdev->dev);

	mutex_lock(&mdp5_data->dfps_lock);
	wdfps = lge_display_opp_table[val].freq;
	if (wdfps == pdata->panel_info.mipi.frame_rate) {
		pr_debug("%s: FPS is already %d\n", __func__, wdfps);
		mutex_unlock(&mdp5_data->dfps_lock);
		return 0;
	}

	if (wdfps < lge_display_opp_table[LV_0].freq) {
		pr_err("Unsupported FPS. Configuring to min_fps = lge_display_opp_table[LV_0].freq\n");
		wdfps = lge_display_opp_table[LV_0].freq;
		ret = mdss_mdp_ctl_update_fps(mdp5_data->ctl, wdfps);
	} else if (wdfps > lge_display_opp_table[_LV_END_ - 1].freq) {
		pr_err("Unsupported FPS. Configuring to max_fps = lge_display_opp_table[_LV_END_ - 1].freq\n");
		wdfps = lge_display_opp_table[_LV_END_ - 1].freq;
		ret = mdss_mdp_ctl_update_fps(mdp5_data->ctl, wdfps);
	} else {
		ret = mdss_mdp_ctl_update_fps(mdp5_data->ctl, wdfps);
	}
	if (!ret) {
		pr_debug("%s: configured to '%d' FPS\n", __func__, wdfps);
	} else {
		pr_err("Failed to configure '%d' FPS. ret = %d\n", wdfps, ret);
		mutex_unlock(&mdp5_data->dfps_lock);
		return ret;
	}
	pdata->panel_info.new_fps = wdfps;
	mutex_unlock(&mdp5_data->dfps_lock);

	lge_cur_level = val;
	return 0;
}

int lge_display_pm_notifier_callback(struct notifier_block *this,
		unsigned long event, void *_data)
{
	/* TODO */
	/*trace("%s\n", __FUNCTION__);*/
	return 0;
}

int lge_opp_get_idx(unsigned long new_freq,
		struct display_opp_table *table)
{
	int i;
	for (i = 0; i < _LV_END_; i++) {
		if (new_freq == table[i].freq)
			return i;
	}
	return -EPERM;
}

int lge_display_profile_target(struct device *dev,
		unsigned long *_freq, u32 options)
{
	int ret = 0;
	struct lge_display_data *data = dev_get_drvdata(dev);
	struct opp *opp = devfreq_recommended_opp(dev, _freq, options);
	unsigned long old_freq = dev_pm_opp_get_freq(data->curr_opp);
	unsigned long new_freq = dev_pm_opp_get_freq(opp);
	unsigned int new_freq_idx = -1, old_freq_idx = -1;

	if (old_freq == new_freq)
		return ret;

	old_freq_idx = lge_opp_get_idx(old_freq, lge_display_opp_table);
	new_freq_idx = lge_opp_get_idx(new_freq, lge_display_opp_table);

	/*trace("%lu => %lu, %u => %u\n", old_freq, new_freq, old_freq_idx, new_freq_idx);*/

	if (new_freq_idx > old_freq_idx) {
		if (lge_cur_level < _LV_END_ - 1) {
			*_freq = lge_display_opp_table[lge_cur_level + 1].freq;
			opp = dev_pm_opp_find_freq_floor(dev, _freq);
			data->curr_opp = opp;

			ret = lge_display_send_event_to_mdss_display(lge_cur_level + 1, NULL);
		}
	} else {
		if (lge_cur_level > 0) {
			*_freq = lge_display_opp_table[lge_cur_level - 1].freq;
			opp = dev_pm_opp_find_freq_floor(dev, _freq);
			data->curr_opp = opp;

			ret = lge_display_send_event_to_mdss_display(lge_cur_level - 1, NULL);
		}
	}

	/* return to previous freq & opp if failed to change fps*/
	if (ret) {
		*_freq = lge_display_opp_table[lge_cur_level].freq;
		opp = dev_pm_opp_find_freq_floor(dev, _freq);
		data->curr_opp = opp;
	}

	return ret;
}

int lge_display_get_dev_status(struct device *dev,
		struct devfreq_dev_status *stat)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);

	struct lge_display_data *data = platform_get_drvdata(pdev);
	struct msm_fb_data_type *mfd = fbi_list[0]->par;

	if (!mfd->panel_power_state)
		return -EPERM;
	else
		lge_display_read_fps(data, stat);

	return 0;
}

void lge_display_profile_exit(struct device *dev)
{
	struct platform_device *pdev = container_of(dev, struct platform_device,
			dev);
	struct lge_display_data *data = platform_get_drvdata(pdev);

	/*trace("%s\n", __FUNCTION__);*/

	devfreq_unregister_opp_notifier(dev, data->devfreq);
}

int lge_display_dev_platform_register(void)
{
	fbi_list = (struct fb_info **)kallsyms_lookup_name(
			"fbi_list");
	pdev_list_cnt = *((int *)kallsyms_lookup_name(
				"pdev_list_cnt"));

	platform_device_register(&lge_display_device);
	platform_driver_register(&lge_display_driver);
	return 0;
}
late_initcall(lge_display_dev_platform_register);

void lge_display_dev_platform_unregister(void)
{
	platform_driver_unregister(&lge_display_driver);
	platform_device_unregister(&lge_display_device);
}

#if defined (CONFIG_LGE_DYNAMIC_FPS_LUT)
int mdss_dsi_parse_dfps_lut(struct device_node *np,
		u32 dfps_lut[MDSS_FPS_LUT_SIZE], u32 *dfps_lut_size,
		const char *name)
{
	int num = 0, i;
	int rc;
	struct property *data;
	u32 tmp[MDSS_FPS_LUT_SIZE];
	*dfps_lut_size = 0;
	data = of_find_property(np, name, &num);
	num /= sizeof(u32);
	if (!data || !num || num > MDSS_FPS_LUT_SIZE || num % 7) {
		pr_debug("%s:%d, error reading %s, length found = %d\n",
			__func__, __LINE__, name, num);
	} else {
		rc = of_property_read_u32_array(np, name, tmp, num);
		if (rc)
			pr_debug("%s:%d, error reading %s, rc = %d\n",
				__func__, __LINE__, name, rc);
		else {
			for (i = 0; i < num; ++i)
				dfps_lut[i] = tmp[i];
			*dfps_lut_size = num;
		}
	}
	return 0;
}
#endif

module_exit(lge_display_dev_platform_unregister);
