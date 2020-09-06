/*
 * mac80211 STA and AP API for mac80211 sigmastar APOLLO drivers
 * *
 * Copyright (c) 2016, sigmastar
 * Author:
 *
 *Based on apollo code
 * Copyright (c) 2010, ST-Ericsson
 * Author: Dmitry Tarnyagin <dmitry.tarnyagin@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef AP_H_INCLUDED
#define AP_H_INCLUDED

#define SSTAR_APOLLO_NOA_NOTIFICATION_DELAY 10

int Sstar_set_tim(struct ieee80211_hw *dev, struct ieee80211_sta *sta,
		   bool set);
int Sstar_sta_add(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		   struct ieee80211_sta *sta);
int Sstar_sta_remove(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		      struct ieee80211_sta *sta);
void Sstar_sta_notify(struct ieee80211_hw *dev, struct ieee80211_vif *vif,
		       enum sta_notify_cmd notify_cmd,
		       struct ieee80211_sta *sta);
void Sstar_bss_info_changed(struct ieee80211_hw *dev,
			     struct ieee80211_vif *vif,
			     struct ieee80211_bss_conf *info,
			     u32 changed);
int Sstar_ampdu_action(struct ieee80211_hw *hw,
			struct ieee80211_vif *vif,
			enum ieee80211_ampdu_mlme_action action,
			struct ieee80211_sta *sta, u16 tid, u16 *ssn,
			u8 buf_size);

void Sstar_suspend_resume(struct Sstar_vif *priv,
			  struct wsm_suspend_resume *arg);
void Sstar_set_tim_work(struct work_struct *work);
void Sstar_set_cts_work(struct work_struct *work);
void Sstar_multicast_start_work(struct work_struct *work);
void Sstar_multicast_stop_work(struct work_struct *work);
void Sstar_mcast_timeout(unsigned long arg);
int Sstar_find_link_id(struct Sstar_vif *priv, const u8 *mac);
int Sstar_alloc_link_id(struct Sstar_vif *priv, const u8 *mac);
void Sstar_link_id_work(struct work_struct *work);
void Sstar_link_id_gc_work(struct work_struct *work);
void Sstar_notify_noa(struct Sstar_vif *priv, int delay);
int ABwifi_unmap_link(struct Sstar_vif *priv, int link_id);
void Sstar_ht_info_update_work(struct work_struct *work);
int Sstar_start_monitor_mode(struct Sstar_vif *priv,
				struct ieee80211_channel *chan);
int Sstar_stop_monitor_mode(struct Sstar_vif *priv);

#endif
