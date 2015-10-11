/*
 * P1 DSV MFD Driver
 *
 * Copyright 2014 LG Electronics Inc,
 *
 * Author:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __MFD_P1_DSV_H__
#define __MFD_P1_DSV_H__

#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/regulator/machine.h>

#define DW8768_DISCHARGE_REG 0x03
#define DW8768_ENABLE_REG 0x05

#define SM5107_CTRL_SET_REG 0xFF

#define P1_DSV_MAX_REGISTERS 0x04

struct p1_dsv_platform_data {
	const char *name;
};

struct p1_dsv {
	struct device *dev;
	struct regmap *regmap;
	struct p1_dsv_platform_data *pdata;
};

extern int sm5107_mode_change(int mode);
extern int dw8768_mode_change(int mode);
extern int dw8768_lgd_dsv_setting(int enable);
extern int dw8768_fast_discharge(void);
extern int dw8768_lgd_fd_mode_change(int enable);
#endif
