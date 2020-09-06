#ifndef __INTERNAL_CMD__
#define __INTERNAL_CMD__
#include <linux/hash.h>
#include "mac80211/ieee80211_i.h"

bool Sstar_internal_cmd_scan_triger(struct ieee80211_sub_if_data *sdata,struct ieee80211_internal_scan_request *req);
bool Sstar_internal_cmd_stainfo(struct ieee80211_local *local,struct ieee80211_internal_sta_req *sta_req);
bool Sstar_internal_cmd_monitor_req(struct ieee80211_sub_if_data *sdata,struct ieee80211_internal_monitor_req *monitor_req);
bool Sstar_internal_cmd_stop_monitor(struct ieee80211_sub_if_data *sdata);
bool Sstar_internal_wsm_adaptive(struct Sstar_common *hw_priv,struct ieee80211_internal_wsm_adaptive *adaptive);
bool Sstar_internal_wsm_txpwr_dcxo(struct Sstar_common *hw_priv,struct ieee80211_internal_wsm_txpwr_dcxo *txpwr_dcxo);
bool Sstar_internal_wsm_txpwr(struct Sstar_common *hw_priv,struct ieee80211_internal_wsm_txpwr *txpwr);
bool Sstar_internal_freq_set(struct ieee80211_hw *hw,struct ieee80211_internal_set_freq_req *req);
bool Sstar_internal_cmd_scan_build(struct ieee80211_local *local,struct ieee80211_internal_scan_request *req,
											   u8* channels,int n_channels,struct cfg80211_ssid *ssids,int n_ssids,
											   struct ieee80211_internal_mac *macs,int n_macs);
bool Sstar_internal_channel_auto_select_results(struct ieee80211_sub_if_data *sdata,
												struct ieee80211_internal_channel_auto_select_results *results);
bool Sstar_internal_channel_auto_select(struct ieee80211_sub_if_data *sdata,
													  struct ieee80211_internal_channel_auto_select_req *req);
bool Sstar_internal_request_chip_cap(struct ieee80211_hw *hw,struct ieee80211_internal_req_chip *req);
bool Sstar_internal_update_ap_conf(struct ieee80211_sub_if_data *sdata,struct ieee80211_internal_ap_conf *conf_req,bool clear);
bool Sstar_internal_wsm_set_rate(struct Sstar_common *hw_priv,struct ieee80211_internal_rate_req *req);
bool Sstar_internal_wsm_set_rate_power(struct Sstar_common *hw_priv,struct ieee80211_internal_rate_power_req *req);

#endif
