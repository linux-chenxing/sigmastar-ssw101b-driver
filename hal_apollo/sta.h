/*
 * Mac80211 STA interface for sigmastar APOLLO mac80211 drivers
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

#ifndef STA_H_INCLUDED
#define STA_H_INCLUDED

/* ******************************************************************** */
/* mac80211 API								*/

int Sstar_start(struct ieee80211_hw *dev);
void Sstar_stop(struct ieee80211_hw *dev);
int Sstar_add_interface(struct ieee80211_hw *dev,
			 struct ieee80211_vif *vif);
void Sstar_remove_interface(struct ieee80211_hw *dev,
			     struct ieee80211_vif *vif);
int Sstar_change_interface(struct ieee80211_hw *dev,
				struct ieee80211_vif *vif,
				enum nl80211_iftype new_type,
				bool p2p);

int Sstar_config(struct ieee80211_hw *dev, u32 changed);
int Sstar_change_interface(struct ieee80211_hw *dev,
                                struct ieee80211_vif *vif,
                                enum nl80211_iftype new_type,
                                bool p2p);
void Sstar_configure_filter(struct ieee80211_hw *dev,
			     struct ieee80211_vif *vif,
			     unsigned int changed_flags,
			     unsigned int *total_flags,
			     u64 multicast);
int Sstar_conf_tx(struct ieee80211_hw *dev, struct ieee80211_vif *vif,
		u16 queue, const struct ieee80211_tx_queue_params *params);
int Sstar_get_stats(struct ieee80211_hw *dev,
		     struct ieee80211_low_level_stats *stats);
/* Not more a part of interface?
int Sstar_get_tx_stats(struct ieee80211_hw *dev,
			struct ieee80211_tx_queue_stats *stats);
*/
int Sstar_set_key(struct ieee80211_hw *dev, enum set_key_cmd cmd,
		   struct ieee80211_vif *vif, struct ieee80211_sta *sta,
		   struct ieee80211_key_conf *key);

int Sstar_set_rts_threshold(struct ieee80211_hw *hw,
		struct ieee80211_vif *vif, u32 value);

void Sstar_flush(struct ieee80211_hw *hw,
		  struct ieee80211_vif *vif,
		  bool drop);

int Sstar_remain_on_channel(struct ieee80211_hw *hw,
				struct ieee80211_vif *vif,
				struct ieee80211_channel *chan,
				enum nl80211_channel_type channel_type,
				int duration, u64 cookie);

int Sstar_cancel_remain_on_channel(struct ieee80211_hw *hw);

int Sstar_set_arpreply(struct ieee80211_hw *hw, struct ieee80211_vif *vif);

u64 Sstar_prepare_multicast(struct ieee80211_hw *hw,
			     struct ieee80211_vif *vif,
			     struct netdev_hw_addr_list *mc_list);

int Sstar_set_pm(struct Sstar_vif *priv, const struct wsm_set_pm *arg);
void Sstar_dhcp_retry_work(struct work_struct *work);

void Sstar_set_data_filter(struct ieee80211_hw *hw,
			   struct ieee80211_vif *vif,
			   void *data,
			   int len);
/* ******************************************************************** */
/* WSM callbacks							*/

/* void Sstar_set_pm_complete_cb(struct Sstar_common *hw_priv,
	struct wsm_set_pm_complete *arg); */
void Sstar_channel_switch_cb(struct Sstar_common *hw_priv);

/* ******************************************************************** */
/* WSM events								*/

void Sstar_free_event_queue(struct Sstar_common *hw_priv);
void Sstar_event_handler(struct work_struct *work);
void Sstar_bss_loss_work(struct work_struct *work);
void Sstar_connection_loss_work(struct work_struct *work);
void Sstar_keep_alive_work(struct work_struct *work);
void Sstar_tx_failure_work(struct work_struct *work);

/* ******************************************************************** */
/* Internal API								*/
void Sstar_pending_offchanneltx_work(struct work_struct *work);
int Sstar_setup_mac(struct Sstar_common *hw_priv);
void Sstar_join_work(struct work_struct *work);
void Sstar_restart_join_bss(struct Sstar_vif *priv,struct cfg80211_bss *bss);
void Sstar_join_timeout(struct work_struct *work);
void Sstar_unjoin_work(struct work_struct *work);
void Sstar_offchannel_work(struct work_struct *work);
void Sstar_wep_key_work(struct work_struct *work);
void Sstar_update_filtering(struct Sstar_vif *priv);
void Sstar_update_filtering_work(struct work_struct *work);
int __Sstar_flush(struct Sstar_common *hw_priv, bool drop, int if_id);
void Sstar_set_beacon_wakeup_period_work(struct work_struct *work);
int Sstar_enable_listening(struct Sstar_vif *priv,
			struct ieee80211_channel *chan);
int Sstar_disable_listening(struct Sstar_vif *priv);
int Sstar_set_uapsd_param(struct Sstar_vif *priv,
				const struct wsm_edca_params *arg);
#ifdef CONFIG_SSTAR_BA_STATUS
void Sstar_ba_work(struct work_struct *work);
void Sstar_ba_timer(unsigned long arg);
#endif
const u8 *Sstar_get_ie(u8 *start, size_t len, u8 ie);
int Sstar_vif_setup(struct Sstar_vif *priv);
void Sstar_vif_setup_params(struct Sstar_vif *priv);
int Sstar_setup_mac_pvif(struct Sstar_vif *priv);
void Sstar_iterate_vifs(void *data, u8 *mac,
			 struct ieee80211_vif *vif);
void Sstar_rem_chan_timeout(struct work_struct *work);
int Sstar_set_macaddrfilter(struct Sstar_common *hw_priv, struct Sstar_vif *priv, u8 *data);
#ifdef CONFIG_SSTAR_STA_LISTEN
int Sstar_sta_triger_listen(struct ieee80211_hw *hw,struct ieee80211_vif *vif,struct ieee80211_channel *chan);
void Sstar_sta_listen_int(struct Sstar_common *hw_priv);
int Sstar_sta_stop_listen(struct ieee80211_hw *hw,struct ieee80211_vif *vif);
#endif

#ifdef ROAM_OFFLOAD
int Sstar_testmode_event(struct wiphy *wiphy, const u32 msg_id,
				const void *data, int len, gfp_t gfp);
#endif /*ROAM_OFFLOAD*/
#ifdef IPV6_FILTERING
int Sstar_set_na(struct ieee80211_hw *hw,
			struct ieee80211_vif *vif);
#endif /*IPV6_FILTERING*/
#ifdef CONFIG_NL80211_TESTMODE

int Sstar_altmtest_cmd(struct ieee80211_hw *hw, void *data, int len);
#endif
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
void Sstar_device_power_calc(struct Sstar_common *priv,
			      s16 max_output_power, s16 fe_cor, u32 band);
int Sstar_testmode_cmd(struct ieee80211_hw *hw, void *data, int len);
int Sstar_tesmode_event(struct wiphy *wiphy, const u32 msg_id,
			 const void *data, int len, gfp_t gfp);
int Sstar_get_tx_power_range(struct ieee80211_hw *hw);
int Sstar_get_tx_power_level(struct ieee80211_hw *hw);

#endif /* CONFIG_SSTAR_APOLLO_TESTMODE */
#endif
