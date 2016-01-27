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


#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/leds.h>

#include "oem_mdss_dsi_common.h"
#include "oem_mdss_dsi.h"
#include <linux/input/lge_touch_notify.h>
#include <soc/qcom/lge/board_lge.h>
#include <linux/mfd/p1_dsv.h>

DEFINE_LED_TRIGGER(bl_led_trigger);
#define PIN_DSV_ENA "DSV_ENA"
#define PIN_DISP_EN "DISP_EN"
#define PIN_RESET "RESET"
#define PANEL_SEQUENCE(name, state) do { pr_info("[PanelSequence][%s] %d\n", name, state); } while (0)

//LGE_UPDATE_S (june1014.lee@lge.com. 2015.03.04). SRE
#if defined(CONFIG_LGE_P1_SRE_SUPPORTED)
struct mdss_panel_data *pdata_sre = NULL;
static unsigned int sre_mode = 0;
struct sre_cmds_desc {
	struct dsi_panel_cmds sre_cmds[2];
};
static char *sre_lgd_dt[] = {
	"lgd,sre-cmds-off",	// sre off
	"lgd,sre-cmds-on",	// sre on
};
static char *sre_jdi_dt[] = {
	"jdi,sre-cmds-off",	// sre off
	"jdi,sre-cmds-on",	// sre on
};
static char *sre_lgdsic_dt[] = {
	"lge,sre-cmds-off",     // sre off
	"lge,sre-cmds-on",      // sre on
};
static struct sre_cmds_desc *sre_cmds_set;
#endif
//LGE_UPDATE_E (june1014.lee@lge.com. 2015.03.04). SRE

#if defined(CONFIG_LGE_BLMAP_STORE_MODE)
struct mdss_panel_info *pinfo_store_mode = NULL;
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_DUAL_BACKLIGHT)
int req_bklt_type = 1; // BL_PWM, BL_WLED, BL_DCS_CMD, UNKNOWN_CTRL
#if IS_ENABLED(CONFIG_BACKLIGHT_LM3697)
extern void lm3697_lcd_backlight_set_level(int level);
#endif
#else
extern void mdss_dsi_panel_bklt_pwm(struct mdss_dsi_ctrl_pdata *ctrl, int level);
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTENDED_PANEL)
#define FB_BLANK_U2 FB_BLANK_VSYNC_SUSPEND
#define FB_BLANK_U1 FB_BLANK_NORMAL

struct mdss_panel_data *pdata_mode = NULL;

struct mode_cmds_desc {
	struct dsi_panel_cmds mode_cmds[LGE_PANEL_MODE_NUM];
};
static struct mode_cmds_desc *mode_cmds_set;
static char *mode_lg4945_dt[] = {
	"lge,mode-change-cmds-u0",
	"lge,mode-change-cmds-u1",
	"lge,mode-change-cmds-u2",
	"lge,mode-change-cmds-u3",
	"lge,mode-change-cmds-u3-ready"
};

enum img_tune_mode {
	LGE_PANEL_SH_ON = 0,
	LGE_PANEL_SH_OFF,
	LGE_PANEL_CE_ON,
	LGE_PANEL_CE_OFF,
	LGE_PANEL_IMG_TUNE_NUM
};
struct img_tune_cmds_desc {
	struct dsi_panel_cmds img_tune_cmds[LGE_PANEL_IMG_TUNE_NUM];
};
static struct img_tune_cmds_desc *img_tune_cmds_set;
static char *img_tune_lg4945_dt[] = {
	"lge,sharpness-cmds-on",
	"lge,sharpness-cmds-off",
	"lge,color_enhancement-cmds-on",
	"lge,color_enhancement-cmds-off",
};
#endif

extern int mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key);

//LGE_UPDATE_S (june1014.lee@lge.com. 2015.03.04). SRE
#if defined(CONFIG_LGE_P1_SRE_SUPPORTED)
int mdss_dsi_panel_sre_apply(unsigned int enabled) {
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_dsi_ctrl_pdata *other = NULL;

	if (pdata_sre == NULL) {
		pr_err("%s: invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata_sre, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (pdata_sre->panel_info.panel_power_state == 0) {
		pr_err("%s: Panel off state. Ignore sre command\n", __func__);
		return -EINVAL;
	}

	if (enabled == SRE_CHANGE_OFF || enabled == SRE_CHANGE_ON) {
		// sre mode changed
		pr_info("%s: send the sre cmd (%d) from kernel\n", __func__, enabled);

		if (sre_cmds_set->sre_cmds[enabled].cmd_cnt) {
			mdss_dsi_panel_cmds_send(ctrl, &sre_cmds_set->sre_cmds[enabled]);
			other = mdss_dsi_get_other_ctrl(ctrl);
			mdss_dsi_panel_cmds_send(other, &sre_cmds_set->sre_cmds[enabled]);
			pr_info("%s: SRE is changed\n", __func__);
		} else {
			pr_err("%s: sre is changed, but cmd is empty\n", __func__);
			return -EINVAL;
		}
	} else {
		pr_info("%s: SRE is not changed\n", __func__);
	}
	return 0;
}
EXPORT_SYMBOL(mdss_dsi_panel_sre_apply);

static ssize_t sre_mode_get(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sre_mode);
}

static ssize_t sre_mode_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int param;
	ssize_t ret = strnlen(buf, PAGE_SIZE);

	sscanf(buf, "%u", &param);
	sre_mode = param;
	pr_debug("%s: sre mode change[%d]\n", __func__, sre_mode);
	mdss_dsi_panel_sre_apply(param);
	return ret;
}

static struct device_attribute sre_status_attrs[] = {
	__ATTR(sre_status, 0644, sre_mode_get, sre_mode_set),
};
#endif
//LGE_UPDATE_E (june1014.lee@lge.com. 2015.03.04). SRE

#if defined(CONFIG_LGE_BLMAP_STORE_MODE)
static ssize_t blmap_store_mode_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (pinfo_store_mode == NULL) {
		pr_err("%s: No panel information\n", __func__);
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", pinfo_store_mode->lge_pan_info.bl_store_mode);
}

static ssize_t blmap_store_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int param;
    ssize_t ret = 0;

	if (pinfo_store_mode == NULL) {
		pr_err("%s: No panel information\n", __func__);
		return -EINVAL;
	}

	ret = strnlen(buf, PAGE_SIZE);
    if( ret>0 ) {
        sscanf(buf, "%x", &param);
        pinfo_store_mode->lge_pan_info.bl_store_mode = param;
        pr_info("%s: bl_store_mode[%d]\n", __func__, pinfo_store_mode->lge_pan_info.bl_store_mode);
    }

	return ret;
}

static struct device_attribute blmap_adjust_status_attrs[] = {
	__ATTR(enable, 0644, blmap_store_mode_show, blmap_store_mode_store),
};
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTENDED_PANEL)
int lge_lg4945_panel_img_tune_cmd_send(int mode) {
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_dsi_ctrl_pdata *other = NULL;

	if (pdata_mode == NULL) {
		pr_err("%s: invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata_mode, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (img_tune_cmds_set->img_tune_cmds[mode].cmd_cnt) {
		mdss_dsi_panel_cmds_send(ctrl, &img_tune_cmds_set->img_tune_cmds[mode]);
		other = mdss_dsi_get_other_ctrl(ctrl);
		if (other)
			mdss_dsi_panel_cmds_send(other, &img_tune_cmds_set->img_tune_cmds[mode]);
		pr_debug("%s: img tune mode %d cmds send\n", __func__, mode);
	} else {
		pr_err("%s: img tune %d cmd is empty\n", __func__, mode);
		return -EINVAL;
	}

	return 0;
}

static ssize_t sharpness_get(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%x\n", img_tune_cmds_set->img_tune_cmds[LGE_PANEL_SH_ON].cmds[1].payload[3]);
}

static ssize_t sharpness_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int param;
	ssize_t ret = strnlen(buf, PAGE_SIZE);

	sscanf(buf, "%x", &param);
	img_tune_cmds_set->img_tune_cmds[LGE_PANEL_SH_ON].cmds[1].payload[3] = param;
	pr_info("%s: strength=0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", __func__,
		img_tune_cmds_set->img_tune_cmds[LGE_PANEL_SH_ON].cmds[0].payload[0],
		img_tune_cmds_set->img_tune_cmds[LGE_PANEL_SH_ON].cmds[0].payload[1],
		img_tune_cmds_set->img_tune_cmds[LGE_PANEL_SH_ON].cmds[1].payload[0],
		img_tune_cmds_set->img_tune_cmds[LGE_PANEL_SH_ON].cmds[1].payload[1],
		img_tune_cmds_set->img_tune_cmds[LGE_PANEL_SH_ON].cmds[1].payload[2],
		img_tune_cmds_set->img_tune_cmds[LGE_PANEL_SH_ON].cmds[1].payload[3],
		img_tune_cmds_set->img_tune_cmds[LGE_PANEL_SH_ON].cmds[1].payload[4]);
	lge_lg4945_panel_img_tune_cmd_send(0);

	return ret;
}

static ssize_t color_enhancement_get(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int i=0;

	for(i=0; i<24; i++){
		sprintf(buf, "%s 0x%02X", buf, img_tune_cmds_set->img_tune_cmds[LGE_PANEL_CE_ON].cmds[1].payload[i]);
		if(((i+1)%6) == 0)
			sprintf(buf, "%s \n", buf);
	}

	return sprintf(buf, "%s\n", buf);
}

static ssize_t color_enhancement_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int set_color_enhancement[128];
	int i;

	memset(set_color_enhancement,0,24*sizeof(int));
	set_color_enhancement[0] = img_tune_cmds_set->img_tune_cmds[LGE_PANEL_CE_ON].cmds[1].payload[0];
	sscanf(buf, "%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",
		&set_color_enhancement[1],&set_color_enhancement[2],&set_color_enhancement[3],
		&set_color_enhancement[4],&set_color_enhancement[5],&set_color_enhancement[6],&set_color_enhancement[7],
		&set_color_enhancement[8],&set_color_enhancement[9],&set_color_enhancement[10],&set_color_enhancement[11],
		&set_color_enhancement[12],&set_color_enhancement[13],&set_color_enhancement[14],&set_color_enhancement[15],
		&set_color_enhancement[16],&set_color_enhancement[17],&set_color_enhancement[18],&set_color_enhancement[19],
		&set_color_enhancement[20],&set_color_enhancement[21],&set_color_enhancement[22],&set_color_enhancement[23]);

	pr_info("%s:", __func__);
	for(i=0; i<24; i++){
		img_tune_cmds_set->img_tune_cmds[LGE_PANEL_CE_ON].cmds[1].payload[i] = set_color_enhancement[i];
		pr_info("0x%02X ", img_tune_cmds_set->img_tune_cmds[LGE_PANEL_CE_ON].cmds[1].payload[i]);
	}
	pr_info("\n");

	lge_lg4945_panel_img_tune_cmd_send(2);
	return ret;
}

static struct device_attribute panel_tuning_device_attrs[] = {
	__ATTR(sharpness, 0644, sharpness_get, sharpness_set),
	__ATTR(color_enhancement, 0644, color_enhancement_get, color_enhancement_set),
};

int lge_lg4945_panel_mode_cmd_send(int mode) {
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_dsi_ctrl_pdata *other = NULL;

	if (pdata_mode == NULL) {
		pr_err("%s: invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata_mode, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (pdata_mode->panel_info.panel_power_state == 0) {
		pr_err("%s: Panel off state. Ignore u%d mode change command\n", __func__, mode);
		return -EINVAL;
	}

	pr_debug("[PowerMode] %s: send the u%d mode cmd from kernel\n", __func__, mode);

	if (mode_cmds_set->mode_cmds[mode].cmd_cnt) {
		mdss_dsi_panel_cmds_send(ctrl, &mode_cmds_set->mode_cmds[mode]);
		other = mdss_dsi_get_other_ctrl(ctrl);
		if (other)
			mdss_dsi_panel_cmds_send(other, &mode_cmds_set->mode_cmds[mode]);
		pr_info("[PowerMode] %s: u%d mode cmds send\n", __func__, mode);
	} else {
		pr_err("[PowerMode] %s: u%d mode cmd is empty\n", __func__, mode);
		return -EINVAL;
	}

	return 0;
}

int lge_lg4945_panel_mode_switch(struct msm_fb_data_type *mfd, unsigned int req_blank_mode) {
	int cur_panel_mode;
	int req_panel_mode;

	cur_panel_mode = mfd->panel_info->lge_pan_info.cur_panel_mode;
	switch(req_blank_mode) {
	case FB_BLANK_UNBLANK: /* u1->u3 and u2->u3 */
		if((cur_panel_mode == LGE_PANEL_MODE_U1) || (cur_panel_mode == LGE_PANEL_MODE_U2))
			req_panel_mode = LGE_PANEL_MODE_U3;
		else
			req_panel_mode = LGE_PANEL_MODE_NONE;
		break;
	case FB_BLANK_U2: /* u3 ->u2 */
		req_panel_mode = LGE_PANEL_MODE_U2;
		break;
	case FB_BLANK_U1: /* u3 ->u1 */
		req_panel_mode = LGE_PANEL_MODE_U1;
		break;
	case FB_BLANK_POWERDOWN: //U0
		req_panel_mode = LGE_PANEL_MODE_NONE;
		break;
	}
	if (LGE_PANEL_MODE_NONE == req_panel_mode || req_panel_mode == cur_panel_mode)
		return 0;

	pr_debug("%s: U%d mode requested from U%d\n", __func__, req_panel_mode, cur_panel_mode);
	lge_lg4945_panel_mode_cmd_send(req_panel_mode);

	return 0;
}

int lge_lg4945_check_skip_onoff_cmd(struct msm_fb_data_type *mfd, int req_blank_mode){
	int cur_panel_mode;
	struct mdss_panel_info *panel_info = mfd->panel_info;

	cur_panel_mode = panel_info->lge_pan_info.cur_panel_mode;
	switch(req_blank_mode) {
	case FB_BLANK_UNBLANK: // U3 unblank
		switch (cur_panel_mode) {
		case LGE_PANEL_MODE_U0: // fall through
			panel_info->lge_pan_info.lge_panel_send_on_cmd = true;
			break;
		case LGE_PANEL_MODE_U1:
		case LGE_PANEL_MODE_U2:
			panel_info->lge_pan_info.lge_panel_send_on_cmd = false;
			break;
		}
		break;
	case FB_BLANK_VSYNC_SUSPEND: // U2 blank
		switch (cur_panel_mode) {
		case LGE_PANEL_MODE_U0:
			pr_err("error, can't called.");
			break;
		case LGE_PANEL_MODE_U1: // fall through
		case LGE_PANEL_MODE_U3:
			panel_info->lge_pan_info.lge_panel_send_off_cmd = false;
			break;
		}
	case FB_BLANK_HSYNC_SUSPEND: // U1 unblank
		switch (cur_panel_mode) {
		case LGE_PANEL_MODE_U0:
			panel_info->lge_pan_info.lge_panel_send_on_cmd = true;
			break;
		case LGE_PANEL_MODE_U2: // fall through
		case LGE_PANEL_MODE_U3:
			break;
		}
		break;
	case FB_BLANK_POWERDOWN: // U0 blank
		switch (cur_panel_mode) {
		case LGE_PANEL_MODE_U1: // fall through
		case LGE_PANEL_MODE_U2:
			break;
		case LGE_PANEL_MODE_U3:
			panel_info->lge_pan_info.lge_panel_send_off_cmd = true;
			break;
		}
		break;
	}

	return 0;
}
#endif

int lgd_qhd_command_mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc = 0;
	return rc;
}

int lgd_qhd_command_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;
	int rc = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (!gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
	}

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
		return rc;
	}

	pr_info("%s: enable = %d, ndx = %d\n", __func__,
			enable, ctrl_pdata->ndx);
	pinfo = &(ctrl_pdata->panel_data.panel_info);

	if (enable) {
		rc = lgd_qhd_command_mdss_dsi_request_gpios(ctrl_pdata);
		if (rc) {
			pr_err("gpio request failed\n");
			return rc;
		}
		if (!pinfo->cont_splash_enabled) {
			if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
				gpio_set_value((ctrl_pdata->disp_en_gpio), 1);
				PANEL_SEQUENCE(PIN_DISP_EN, 1);
			}

			if (gpio_is_valid(ctrl_pdata->bklt_en_gpio))
				gpio_set_value((ctrl_pdata->bklt_en_gpio), 1);
		}

		if (gpio_is_valid(ctrl_pdata->mode_gpio)) {
			if (pinfo->mode_gpio_state == MODE_GPIO_HIGH)
				gpio_set_value((ctrl_pdata->mode_gpio), 1);
			else if (pinfo->mode_gpio_state == MODE_GPIO_LOW)
				gpio_set_value((ctrl_pdata->mode_gpio), 0);
		}
		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("%s: Panel Not properly turned OFF\n",
						__func__);
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_debug("%s: Reset panel done\n", __func__);
		}
	} else {
		if (gpio_is_valid(ctrl_pdata->bklt_en_gpio)) {
			gpio_set_value((ctrl_pdata->bklt_en_gpio), 0);
			gpio_free(ctrl_pdata->bklt_en_gpio);
		}
		if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
			gpio_set_value((ctrl_pdata->disp_en_gpio), 0);
			PANEL_SEQUENCE(PIN_DISP_EN, 0);
			gpio_free(ctrl_pdata->disp_en_gpio);
		}
		if (gpio_is_valid(ctrl_pdata->mode_gpio))
			gpio_free(ctrl_pdata->mode_gpio);
	}
	return rc;
}

int jdi_qhd_command_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;
	int rc = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (!gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
	}

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
		return rc;
	}
	if (!ctrl_pdata->ndx) {
		pr_debug("%s:%d, do not inplement mdss_dsi_panel_reset\n",
				__func__, __LINE__);
		return rc;
	}
	pr_info("%s: enable = %d, ndx = %d\n", __func__,
			enable, ctrl_pdata->ndx);
	pinfo = &(ctrl_pdata->panel_data.panel_info);

	if (enable) {
		rc = lgd_qhd_command_mdss_dsi_request_gpios(ctrl_pdata);
		if (rc) {
			pr_err("gpio request failed\n");
			return rc;
		}
		if (!pinfo->cont_splash_enabled) {
			if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
				gpio_set_value((ctrl_pdata->disp_en_gpio), 1);
				PANEL_SEQUENCE(PIN_DISP_EN, 1);
			}
			if (ctrl_pdata->ndx) {
				if (ctrl_pdata->lge_pan_data->dsv_manufacturer ==
						DSV_SM5107)
					sm5107_mode_change(1);
				else if (ctrl_pdata->lge_pan_data->dsv_manufacturer ==
						DSV_DW8768)
					dw8768_mode_change(1);
			}

			if (gpio_is_valid(ctrl_pdata->bklt_en_gpio))
				gpio_set_value((ctrl_pdata->bklt_en_gpio), 1);
		}

		if (gpio_is_valid(ctrl_pdata->mode_gpio)) {
			if (pinfo->mode_gpio_state == MODE_GPIO_HIGH)
				gpio_set_value((ctrl_pdata->mode_gpio), 1);
			else if (pinfo->mode_gpio_state == MODE_GPIO_LOW)
				gpio_set_value((ctrl_pdata->mode_gpio), 0);
		}
		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("%s: Panel Not properly turned OFF\n",
						__func__);
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_debug("%s: Reset panel done\n", __func__);
		}
	} else {
		if (gpio_is_valid(ctrl_pdata->bklt_en_gpio)) {
			gpio_set_value((ctrl_pdata->bklt_en_gpio), 0);
			gpio_free(ctrl_pdata->bklt_en_gpio);
		}
		if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
			gpio_set_value((ctrl_pdata->disp_en_gpio), 0);
			PANEL_SEQUENCE(PIN_DISP_EN, 0);
			gpio_free(ctrl_pdata->disp_en_gpio);
		}
		if (ctrl_pdata->ndx) {
			if (ctrl_pdata->lge_pan_data->dsv_manufacturer ==
					DSV_SM5107)
				sm5107_mode_change(0);
			else if (ctrl_pdata->lge_pan_data->dsv_manufacturer ==
					DSV_DW8768)
				dw8768_mode_change(0);
          }
		if (gpio_is_valid(ctrl_pdata->mode_gpio))
			gpio_free(ctrl_pdata->mode_gpio);
	}
	return rc;
}

int lgd_sic_qhd_command_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;
	int rc = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (!gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
	}

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
		return rc;
	}
	pr_info("%s: enable = %d, ndx = %d\n", __func__,
			enable, ctrl_pdata->ndx);
	pinfo = &(ctrl_pdata->panel_data.panel_info);

	if (enable) {
		rc = lgd_qhd_command_mdss_dsi_request_gpios(ctrl_pdata);
		if (rc) {
			pr_err("gpio request failed\n");
			return rc;
		}
		if (!pinfo->cont_splash_enabled) {
			if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
				gpio_set_value((ctrl_pdata->disp_en_gpio), 1);
				PANEL_SEQUENCE(PIN_DISP_EN, 1);
			}
			if (ctrl_pdata->ndx) {
				if (ctrl_pdata->lge_pan_data->dsv_manufacturer ==
						DSV_SM5107)
					sm5107_mode_change(1);
				else if (ctrl_pdata->lge_pan_data->dsv_manufacturer ==
						DSV_DW8768)
					dw8768_mode_change(1);
			}

			if (gpio_is_valid(ctrl_pdata->bklt_en_gpio))
				gpio_set_value((ctrl_pdata->bklt_en_gpio), 1);
		}

		if (gpio_is_valid(ctrl_pdata->mode_gpio)) {
			if (pinfo->mode_gpio_state == MODE_GPIO_HIGH)
				gpio_set_value((ctrl_pdata->mode_gpio), 1);
			else if (pinfo->mode_gpio_state == MODE_GPIO_LOW)
				gpio_set_value((ctrl_pdata->mode_gpio), 0);
		}
		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("%s: Panel Not properly turned OFF\n",
						__func__);
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_debug("%s: Reset panel done\n", __func__);
		}
	} else {
		if (gpio_is_valid(ctrl_pdata->bklt_en_gpio)) {
			gpio_set_value((ctrl_pdata->bklt_en_gpio), 0);
			gpio_free(ctrl_pdata->bklt_en_gpio);
		}
		if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
			gpio_set_value((ctrl_pdata->disp_en_gpio), 0);
			PANEL_SEQUENCE(PIN_DISP_EN, 0);
			gpio_free(ctrl_pdata->disp_en_gpio);
		}
		if (ctrl_pdata->ndx) {
			if (ctrl_pdata->lge_pan_data->dsv_manufacturer ==
					DSV_SM5107)
				sm5107_mode_change(0);
			else if (ctrl_pdata->lge_pan_data->dsv_manufacturer ==
					DSV_DW8768)
				dw8768_mode_change(0);
		}
		if (gpio_is_valid(ctrl_pdata->mode_gpio))
			gpio_free(ctrl_pdata->mode_gpio);
	}
	return rc;
}

int lgd_qhd_command_mdss_dsi_panel_bl_ctrl(struct mdss_panel_data *pdata, u32 bl_level)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return 0;
	}
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
			panel_data);

#if IS_ENABLED(CONFIG_LGE_DISPLAY_DUAL_BACKLIGHT)
	ctrl_pdata->bklt_ctrl = lge_get_bklt_type();
	pr_info("%s: bklt_ctrl=%d\n", __func__, ctrl_pdata->bklt_ctrl);
#endif
	switch (ctrl_pdata->bklt_ctrl) {
	case BL_WLED:
		led_trigger_event(bl_led_trigger, bl_level);
		break;
	case BL_PWM:
#if IS_ENABLED(CONFIG_LGE_DISPLAY_DUAL_BACKLIGHT)
#if IS_ENABLED(CONFIG_BACKLIGHT_LM3697)
		lm3697_lcd_backlight_set_level(bl_level);
#endif
#else
		mdss_dsi_panel_bklt_pwm(ctrl_pdata, bl_level);
#endif
		break;
	default:
		pr_err("%s: Unknown bl_ctrl configuration\n",
				__func__);
		break;
	}


	return 0;
}

int lgd_qhd_command_post_mdss_dsi_panel_on(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
			panel_data);

	//LGE_UPDATE_S (june1014.lee@lge.com. 2015.03.04). SRE
#if defined(CONFIG_LGE_P1_SRE_SUPPORTED)
	if (pdata_sre == NULL)
		pdata_sre = pdata;
#endif
	//LGE_UPDATE_E (june1014.lee@lge.com. 2015.03.04). SRE

	if (ctrl->ndx) {
		if (swipe_status) {
			touch_notifier_call_chain(
					LCD_EVENT_TOUCH_LPWG_OFF, NULL);
		}
	}
	return 0;
}

int lgd_qhd_command_post_mdss_dsi_panel_off(struct mdss_panel_data *pdata)
{
	int rc=0;
	return rc;
}

int lgd_qhd_command_mdss_dsi_panel_init(struct device_node *node, struct mdss_dsi_ctrl_pdata *ctrl_pdata, bool cmd_cfg_cont_splash)
{
	struct mdss_panel_info *pinfo;

//LGE_UPDATE_S (june1014.lee@lge.com. 2015.03.04). SRE
#if defined(CONFIG_LGE_P1_SRE_SUPPORTED)
	static struct class *sre = NULL;
	static struct device *sre_sysfs_dev = NULL;
#endif
//LGE_UPDATE_E (june1014.lee@lge.com. 2015.03.04). SRE

#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTENDED_PANEL)
	static struct class *panel;
	static struct device *panel_sysfs_dev;
#endif

	pinfo = &ctrl_pdata->panel_data.panel_info;

	/* [LGD_SIC] 20141216. Add code for LGD SIC module */
	if (strncmp(pinfo->panel_name, "LGD SIC", 7) == 0) {
		pr_info("%s: panel_type is LGD_SIC_INCELL_CMD_PANEL\n",
				__func__);
		pinfo->lge_pan_info.panel_type = LGD_SIC_INCELL_CMD_PANEL;
	} else if (strncmp(pinfo->panel_name, "JDI", 3) == 0) {
		pinfo->lge_pan_info.panel_type = JDI_INCELL_CMD_PANEL;
	} else if (strncmp(pinfo->panel_name, "LGD", 3) == 0) {
		pr_info("%s: panel_type is LGD_INCELL_CMD_PANEL\n",
				__func__);
		pinfo->lge_pan_info.panel_type = LGD_INCELL_CMD_PANEL;
	} else {
		pr_err("%s: Invalid panel type\n", __func__);
	}
	lge_set_panel(pinfo->lge_pan_info.panel_type);

//LGE_UPDATE_S (june1014.lee@lge.com. 2015.03.04). SRE
#if defined(CONFIG_LGE_P1_SRE_SUPPORTED)
		if(!sre) {
			sre = class_create(THIS_MODULE, "sre");
			if (IS_ERR(sre))
				pr_err("%s: Failed to create sre class\n", __func__);
		}

		if ( !sre_sysfs_dev ) {
			sre_sysfs_dev = device_create(sre, NULL, 0, NULL, "sre_func");
			if (IS_ERR(sre_sysfs_dev)) {
				pr_err("%s: Failed to create dev(sre_sysfs_dev)!",
						__func__);
			} else {
				if (device_create_file(sre_sysfs_dev,
						&sre_status_attrs[0]) < 0)
					pr_err("%s: Fail!", __func__);
			}
		}
#endif

#if defined(CONFIG_LGE_BLMAP_STORE_MODE)
		{
			static struct class *blmap_adjust = NULL;
			static struct device *blmode_sysfs_dev = NULL;

			if(!blmap_adjust) {
				blmap_adjust = class_create(THIS_MODULE, "blmap_adjust");
				if (IS_ERR(blmap_adjust))
					pr_err("%s: Failed to create blmap_adjust class\n", __func__);
			}

			if ( !blmode_sysfs_dev ) {
				blmode_sysfs_dev = device_create(blmap_adjust, NULL, 0, NULL, "bl_store_mode");
				if (IS_ERR(blmode_sysfs_dev)) {
					pr_err("%s: Failed to create dev(blmode_sysfs_dev)!",
							__func__);
				} else {
					if (device_create_file(blmode_sysfs_dev,
								&blmap_adjust_status_attrs[0]) < 0)
						pr_err("%s: Fail!", __func__);
				}
			}
		}
#endif

//LGE_UPDATE_E (june1014.lee@lge.com. 2015.03.04). SRE
#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTENDED_PANEL)
	pinfo->lge_pan_info.lge_panel_send_on_cmd = true;
	pinfo->lge_pan_info.lge_panel_send_off_cmd = true;

	panel = class_create(THIS_MODULE, "panel");
	if (IS_ERR(panel))
		pr_err("%s: Failed to create panel class\n", __func__);

	panel_sysfs_dev = device_create(panel, NULL, 0, NULL, "img_tune");
	if (IS_ERR(panel_sysfs_dev)) {
		pr_err("%s: Failed to create dev(panel_sysfs_dev)!", __func__);
	} else {
		if (device_create_file(panel_sysfs_dev, &panel_tuning_device_attrs[0]) < 0)
			pr_err("%s: #1 add panel tuning node fail!", __func__);
		if (device_create_file(panel_sysfs_dev, &panel_tuning_device_attrs[1]) < 0)
			pr_err("%s: #2 add panel tuning node fail!!", __func__);
	}
#endif

#if defined(CONFIG_LGE_BLMAP_STORE_MODE)
	if (pinfo_store_mode == NULL)
		pinfo_store_mode = pinfo;
#endif

	return 0;
}

int lgd_qhd_command_mdss_panel_parse_dt(struct device_node *np,	struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	u32 tmp;
	int rc, i;
	u32 *array;
	const char *data;

	struct mdss_panel_info *pinfo = &(ctrl_pdata->panel_data.panel_info);


	ctrl_pdata->bklt_ctrl = UNKNOWN_CTRL;
	data = of_get_property(np, "qcom,mdss-dsi-bl-pmic-control-type", NULL);
	if (data) {
		if (!strncmp(data, "bl_ctrl_wled", 12)) {
			led_trigger_register_simple("bkl-trigger",
					&bl_led_trigger);
			pr_debug("%s: SUCCESS-> WLED TRIGGER register\n",
					__func__);
			ctrl_pdata->bklt_ctrl = BL_WLED;
		} else if (!strncmp(data, "bl_ctrl_pwm", 11)) {
			ctrl_pdata->bklt_ctrl = BL_PWM;
			ctrl_pdata->pwm_pmi = of_property_read_bool(np,
					"qcom,mdss-dsi-bl-pwm-pmi");
			rc = of_property_read_u32(np,
					"qcom,mdss-dsi-bl-pmic-pwm-frequency", &tmp);
			if (rc) {
				pr_err("%s:%d, Error, panel pwm_period\n",
						__func__, __LINE__);
				return -EINVAL;
			}
			ctrl_pdata->pwm_period = tmp;
			if (ctrl_pdata->pwm_pmi) {
				ctrl_pdata->pwm_bl = of_pwm_get(np, NULL);
				if (IS_ERR(ctrl_pdata->pwm_bl)) {
					pr_err("%s: Error, pwm device\n",
							__func__);
					ctrl_pdata->pwm_bl = NULL;
					return -EINVAL;
				}
			} else {
				rc = of_property_read_u32(np,
						"qcom,mdss-dsi-bl-pmic-bank-select",
						&tmp);
				if (rc) {
					pr_err("%s:%d, Error, lpg channel\n",
							__func__, __LINE__);
					return -EINVAL;
				}
				ctrl_pdata->pwm_lpg_chan = tmp;
				tmp = of_get_named_gpio(np,
						"qcom,mdss-dsi-pwm-gpio", 0);
				ctrl_pdata->pwm_pmic_gpio = tmp;
				pr_debug("%s: Configured PWM bklt ctrl\n",
						__func__);
			}
		} else if (!strncmp(data, "bl_ctrl_dcs", 11)) {
			ctrl_pdata->bklt_ctrl = BL_DCS_CMD;
			pr_debug("%s: Configured DCS_CMD bklt ctrl\n",
					__func__);
		}
	}
	rc = of_property_read_u32(np, "qcom,blmap-size", &tmp);
	pinfo->lge_pan_info.blmap_size = (!rc ? tmp : 0);

	if (pinfo->lge_pan_info.blmap_size) {
		array = kzalloc(sizeof(u32) * pinfo->lge_pan_info.blmap_size, GFP_KERNEL);

		if (!array)
			return -ENOMEM;
		rc = of_property_read_u32_array(np,
			"qcom,blmap", array, pinfo->lge_pan_info.blmap_size);

		if (rc) {
			pr_err("%s:%d, unable to read backlight map\n",
					__func__, __LINE__);
			kfree(array);
			return rc;
		}

		pinfo->lge_pan_info.blmap = kzalloc(sizeof(int) * pinfo->lge_pan_info.blmap_size,
					GFP_KERNEL);
		if (!pinfo->lge_pan_info.blmap)
		{
			kfree(array);
			return -ENOMEM;
		}

		for (i = 0; i < pinfo->lge_pan_info.blmap_size; i++)
			pinfo->lge_pan_info.blmap[i] = array[i];

		kfree(array);
	} else {
		pinfo->lge_pan_info.blmap = NULL;
	}
#if defined(CONFIG_LGE_BLMAP_STORE_MODE)
	if (pinfo->lge_pan_info.blmap_size) {
		array = kzalloc(sizeof(u32) * pinfo->lge_pan_info.blmap_size, GFP_KERNEL);

		if (!array)
			return -ENOMEM;
		rc = of_property_read_u32_array(np,
			"qcom,blmap_store_mode", array, pinfo->lge_pan_info.blmap_size);

		if (rc) {
			pr_err("%s:%d, unable to read backlight map for store_mode\n",
					__func__, __LINE__);
			kfree(array);
			return rc;
		}

		pinfo->lge_pan_info.blmap_store_mode = kzalloc(sizeof(int) * pinfo->lge_pan_info.blmap_size,
					GFP_KERNEL);
		if (!pinfo->lge_pan_info.blmap_store_mode)
			return -ENOMEM;

		for (i = 0; i < pinfo->lge_pan_info.blmap_size; i++)
			pinfo->lge_pan_info.blmap_store_mode[i] = array[i];
		pinfo->lge_pan_info.bl_store_mode = 0;

		kfree(array);
	} else {
		pinfo->lge_pan_info.blmap_store_mode = NULL;
	}
#endif
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->lge_pan_data->clk_on_cmds,
		"lge,mdss-dsi-clk_on-command", "lge,mdss-dsi-clk_on-command-state");

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->lge_pan_data->clk_off_cmds,
		"lge,mdss-dsi-clk_off-command", "lge,mdss-dsi-clk_off-command-state");

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->lge_pan_data->rsp_nvm_write,
		"lgd,rsp-write-nvm", "qcom,mdss-dsi-on-command-state");
//LGE_UPDATE_S (june1014.lee@lge.com. 2015.03.04). SRE
#if defined(CONFIG_LGE_P1_SRE_SUPPORTED)
	sre_cmds_set = kzalloc(sizeof(struct sre_cmds_desc), GFP_KERNEL);
	switch (pinfo->lge_pan_info.panel_type) {
		case LGD_INCELL_CMD_PANEL:	// LGD panel
			for (i = 0; i < 2; i++) {
				mdss_dsi_parse_dcs_cmds(np, &sre_cmds_set->sre_cmds[i],
						sre_lgd_dt[i], "qcom,sre-control-dsi-state");
			}
			pr_debug("%s: std_cmd=0x%02X 0x%02X\n", __func__,
				sre_cmds_set->sre_cmds[0].cmds->payload[0],
				sre_cmds_set->sre_cmds[0].cmds->payload[1]);
			break;
		case JDI_INCELL_CMD_PANEL:	// JDI panel
			for (i = 0; i < 2; i++) {
				mdss_dsi_parse_dcs_cmds(np, &sre_cmds_set->sre_cmds[i],
						sre_jdi_dt[i], "qcom,sre-control-dsi-state");
			}
			pr_debug("%s: std_cmd=0x%02X 0x%02X\n", __func__,
				sre_cmds_set->sre_cmds[0].cmds->payload[0],
				sre_cmds_set->sre_cmds[0].cmds->payload[1]);
			break;
		case LGD_SIC_INCELL_CMD_PANEL:	// LGD-SIC panel
			for (i = 0; i < 2; i++) {
				mdss_dsi_parse_dcs_cmds(np, &sre_cmds_set->sre_cmds[i],
						sre_lgdsic_dt[i], "qcom,sre-control-dsi-state");
			}
			pr_debug("%s: std_cmd=0x%02X 0x%02X\n", __func__,
				sre_cmds_set->sre_cmds[0].cmds->payload[0],
				sre_cmds_set->sre_cmds[0].cmds->payload[1]);
			break;
		default:
			pr_err("%s: not chosen panel type in SRE\n", __func__);
			break;
	}

	if (pdata_sre == NULL)
		pdata_sre = &(ctrl_pdata->panel_data);
#endif
//LGE_UPDATE_E (june1014.lee@lge.com. 2015.03.04). SRE

#if IS_ENABLED(CONFIG_LGE_DISPLAY_EXTENDED_PANEL)
	mode_cmds_set = kzalloc(sizeof(struct mode_cmds_desc), GFP_KERNEL);
	for (i = 0; i < LGE_PANEL_MODE_NUM; i++) {
		mdss_dsi_parse_dcs_cmds(np, &mode_cmds_set->mode_cmds[i],
			mode_lg4945_dt[i], "qcom,mode-control-dsi-state");
	}

	img_tune_cmds_set = kzalloc(sizeof(struct img_tune_cmds_desc), GFP_KERNEL);
	for (i = 0; i < LGE_PANEL_IMG_TUNE_NUM; i++) {
		mdss_dsi_parse_dcs_cmds(np, &img_tune_cmds_set->img_tune_cmds[i],
			img_tune_lg4945_dt[i], "qcom,img-tune-control-dsi-state");
		pr_info("%s: parse img tune %d cmd = 0x%02X \n", __func__, i, img_tune_cmds_set->img_tune_cmds[i].cmds->payload[0]);
	}

	if (pdata_mode == NULL)
		pdata_mode = &(ctrl_pdata->panel_data);

#endif
		return rc;
}

void mdss_dsi_stub_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds)
{
		mdss_dsi_panel_cmds_send(ctrl, pcmds);
}

int lgd_deep_sleep(struct mdss_dsi_ctrl_pdata *ctrl_pdata, int mode,
	       int is_no_sleep)
{
	int rc = 0;
	pr_info("%s: mode : %d\n", __func__, mode);
	if (!mode) { /* NEAR :  LPWG -> DEEP SLEEP */
		if (gpio_get_value(ctrl_pdata->lge_pan_data->dsv_ena)) {
			pr_info("%s: LPWG to DEEP SLEEP\n", __func__);
			if (gpio_is_valid(ctrl_pdata->rst_gpio)) {
				gpio_set_value((ctrl_pdata->rst_gpio), 0);
				PANEL_SEQUENCE(PIN_RESET, 0);
				gpio_free(ctrl_pdata->rst_gpio);
			}

			dw8768_lgd_fd_mode_change(1); /* fd_on */

			if (gpio_is_valid(ctrl_pdata->lge_pan_data->dsv_ena)) {
				gpio_set_value((ctrl_pdata->lge_pan_data->dsv_ena), 0);
				PANEL_SEQUENCE(PIN_DSV_ENA, 0);
				gpio_free(ctrl_pdata->lge_pan_data->dsv_ena);
			}
			if (ctrl_pdata->lge_pan_data->touch_driver_registered)
				touch_notifier_call_chain(
						LCD_EVENT_TOUCH_SLEEP_STATUS,
						(void *)&mode);
		}
	} else { /* FAR : DEEP SLEEP -> LPWG */
		if (!gpio_get_value(ctrl_pdata->lge_pan_data->dsv_ena)) {
			if (gpio_is_valid(ctrl_pdata->lge_pan_data->dsv_ena)) {
				rc = gpio_request(ctrl_pdata->lge_pan_data->dsv_ena,
						"dsv_ena");
				if (rc) {
					pr_err("%s:request dsv_ena gpio failed, rc=%d\n",
							__func__, rc);
					goto dsv_ena_err;
				}
				gpio_set_value((ctrl_pdata->lge_pan_data->dsv_ena), 1);
				PANEL_SEQUENCE(PIN_DSV_ENA, 1);
				if (gpio_is_valid(ctrl_pdata->rst_gpio)) {
					rc = gpio_request(ctrl_pdata->rst_gpio,
							"disp_enable");
					if (rc) {
						pr_err("%s:request rst_gpio gpio failed, rc=%d\n",
								__func__, rc);
						goto disp_reset_err;
					}
					gpio_set_value((ctrl_pdata->rst_gpio),
							1);
					PANEL_SEQUENCE(PIN_RESET, 1);
				}

				dw8768_lgd_fd_mode_change(0); /* fd_off */
				if (ctrl_pdata->lge_pan_data->touch_driver_registered) {
					if (is_no_sleep == DEEP_SLEEP_TO_LPWG) {
						pr_info("%s: DEEP SLEEP to LPWG\n",
						       __func__);

						touch_notifier_call_chain(
						LCD_EVENT_TOUCH_SLEEP_STATUS,
								(void *)&mode);
					} else if (is_no_sleep
						== DEEP_SLEEP_TO_ACTIVE) {
						pr_info("%s: DEEP SLEEP to ACTIVE\n",
						       __func__);
						touch_notifier_call_chain(
						LCD_EVENT_TOUCH_SLEEP_STATUS,
							(void *)&is_no_sleep);
					}
				}
			}
		}
	}

disp_reset_err:
dsv_ena_err:
	return 0;

}
int mdss_dsi_lcd_reset(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	int rc = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	pr_info("%s: Reset = %d\n", __func__, enable);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);

	if (enable) {
		if (gpio_is_valid(ctrl_pdata->rst_gpio)) {
			rc = gpio_request(ctrl_pdata->rst_gpio, "disp_enable");
			if (rc) {
				pr_err("%s:request disp_en gpio failed, rc=%d\n",
					__func__, rc);
				goto disp_reset_err;
			}
			gpio_set_value((ctrl_pdata->rst_gpio), 1);
			PANEL_SEQUENCE(PIN_RESET, 1);
		}
	} else {
		if (gpio_is_valid(ctrl_pdata->rst_gpio)) {
			gpio_set_value((ctrl_pdata->rst_gpio), 0);
			PANEL_SEQUENCE(PIN_RESET, 0);
		}

		gpio_free(ctrl_pdata->rst_gpio);
	}
disp_reset_err:
	return rc;
}

#if IS_ENABLED(CONFIG_LGE_DISPLAY_DUAL_BACKLIGHT)
int lge_get_bklt_type()
{
	return req_bklt_type;
}

int lge_set_bklt_type(int bklt_type)
{
	req_bklt_type = bklt_type;
	return 0;
}
#endif
