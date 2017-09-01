/*
 * Copyright (C) 2012, Kyungtae Oh <kyungtae.oh@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under  the terms of the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the License, or (at your
 * option) any later version.
 *
 */

#ifndef __LINUX_POWER_UNIFIED_WLC_CHARGER_H__
#define __LINUX_POWER_UNIFIED_WLC_CHARGER_H__

#define UNIFIED_WLC_DEV_NAME "unified_wlc"

struct unified_wlc_platform_data {
	unsigned int wlc_full_chg;
	unsigned int wlc_rx_off;
};

#if defined(CONFIG_LGE_TOUCH_CORE)
extern void touch_notify_wireless(u32 type);
#endif

#ifdef CONFIG_LGE_PM_UNIFIED_WLC_ALIGNMENT
#define WLC_ALIGN_INTERVAL	(300)
#endif
#endif
