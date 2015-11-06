/*
 * Copyright (C) 2009 Samsung Electronics
 * Copyright (C) 2012 Nvidia Cooperation
 * Minkyu Kang <mk7.kang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MAX17048_BATTERY_H_
#define __MAX17048_BATTERY_H_

#define CONFIG_MAX17048_SOC_ALERT
#define CONFIG_MAX17048_CUSTOM_MODEL
#define CONFIG_LGE_PM_MAX17048_SHOW_OCV
#define CONFIG_LGE_PM_MAX17048_EVL
#define CONFIG_LGE_PM_MAX17048_BMS_DEBUG
#define CONFIG_LGE_PM_BATT_PROFILE
//#define CONFIG_LGE_PM_BATT_PROFILE_DEBUG // compare level between origin and tune
#if defined(CONFIG_MACH_MSM8992_P1) || defined(CONFIG_MACH_MSM8992_P1A4WP)
#define CONFIG_LGE_PM_MAX17048_RECHARGING
#endif

struct max17048_battery_model {
	int empty;
#ifdef CONFIG_LGE_PM
	int rcomp;
	int temp_co_hot;
	int temp_co_cold;
	int alert_threshold;
	int alert_gpio;
	int full_design;
#else
	uint8_t rcomp;
	uint8_t soccheck_a;
	uint8_t soccheck_b;
	uint8_t bits;
	uint8_t alert_threshold;
	uint8_t one_percent_alerts;
	uint8_t alert_on_reset;
	uint16_t rcomp_seg;
	uint16_t hibernate;
	uint16_t vreset;
	uint16_t valert;
	uint16_t ocvtest;
#endif
#ifdef CONFIG_LGE_PM_BATT_PROFILE
	int batt_profile_enabled;
#endif
};

struct max17048_platform_data {
	int (*battery_online)(void);
	int (*charging_status)(void);
	int (*charger_online)(void);
};

#ifdef CONFIG_LGE_PM
extern int max17048_get_voltage(void);
extern int max17048_get_capacity(void);
extern int max17048_get_fulldesign(void);
#endif
#endif
