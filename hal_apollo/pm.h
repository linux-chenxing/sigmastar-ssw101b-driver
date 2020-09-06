/*
 * Mac80211 power management interface for sigmastar APOLLO mac80211 drivers
 * *
 * Copyright (c) 2016, sigmastar
 * Author:
 *
 * Based on apollo code
 * Copyright (c) 2011, ST-Ericsson
 * Author: Dmitry Tarnyagin <dmitry.tarnyagin@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef PM_H_INCLUDED
#define PM_H_INCLUDED
#include <linux/platform_device.h>
#ifdef CONFIG_WAKELOCK
#include <linux/wakelock.h>
#endif

/* ******************************************************************** */
/* mac80211 API								*/

#ifdef CONFIG_PM

/* extern */  struct Sstar_common;
/* private */ struct Sstar_suspend_state;
struct Sstar_pm_stayawake_lock{
	struct Sstar_common *hw_priv;
	int stayawak_cnt;
	spinlock_t stayawak_spinlock;
	#ifdef CONFIG_WAKELOCK
	struct wake_lock stayawak_lock;
	#endif
};
struct Sstar_pm_state {
	struct platform_driver pm_driver;
#ifdef CONFIG_WAKELOCK
	struct wake_lock wakelock;
#else
	struct timer_list stay_awake;
#endif
	struct platform_device *pm_dev;
	spinlock_t lock;
	struct Sstar_pm_stayawake_lock stayawake_lock;
	int b_init;
};

struct Sstar_pm_state_vif {
	struct Sstar_suspend_state *suspend_state;
};

int Sstar_pm_init(struct Sstar_pm_state *pm,
		    struct Sstar_common *priv);
void Sstar_pm_deinit(struct Sstar_pm_state *pm);
void Sstar_pm_stay_awake(struct Sstar_pm_state *pm,
			  unsigned long tmo);
int Sstar_wow_suspend(struct ieee80211_hw *hw,
		       struct cfg80211_wowlan *wowlan);
int Sstar_wow_resume(struct ieee80211_hw *hw);
void Sstar_pm_stay_awake_lock(struct Sstar_pm_state *pm);
void Sstar_pm_stay_awake_unlock(struct Sstar_pm_state *pm);
#endif
#endif
