/*
 * Scanning implementation
 *
 * Copyright 2003, Jouni Malinen <jkmaline@cc.hut.fi>
 * Copyright 2004, Instant802 Networks, Inc.
 * Copyright 2005, Devicescape Software, Inc.
 * Copyright 2006-2007	Jiri Benc <jbenc@suse.cz>
 * Copyright 2007, Michael Wu <flamingice@sourmilk.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/if_arp.h>
#include <linux/rtnetlink.h>
#ifdef CONFIG_SSTAR_PM_QOS
#include <linux/pm_qos.h>
#endif
#include <net/sch_generic.h>
#include <linux/slab.h>
#include <linux/export.h>
#include <net/Sstar_mac80211.h>
#ifdef SSTAR_CHANGE_AP_TIMESTAMP_TO_BOOTTIME
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#endif
#include <linux/hash.h>
#include <linux/sched.h>

#include "ieee80211_i.h"
#include "driver-ops.h"
#include "mesh.h"


#define IEEE80211_PROBE_DELAY (HZ / 33)
#define IEEE80211_CHANNEL_TIME (HZ / 33)
#define IEEE80211_PASSIVE_CHANNEL_TIME (HZ / 8)

struct ieee80211_bss *
ieee80211_rx_bss_get(struct ieee80211_local *local, u8 *bssid, int freq,
		     u8 *ssid, u8 ssid_len)
{
	struct cfg80211_bss *cbss;

	cbss = ieee80211_Sstar_get_bss(local->hw.wiphy,
				ieee80211_get_channel(local->hw.wiphy, freq),
				bssid, ssid, ssid_len, 0, 0);
	if (!cbss)
		return NULL;
	return (void *)cbss->priv;
}
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
static void ieee80211_rx_bss_free(struct cfg80211_bss *cbss)
{
	struct ieee80211_bss *bss = (void *)cbss->priv;

	Sstar_kfree(bss_mesh_id(bss));
	Sstar_kfree(bss_mesh_cfg(bss));
}
#endif
void ieee80211_rx_bss_put(struct ieee80211_local *local,
			  struct ieee80211_bss *bss)
{
	if (!bss)
		return;
	ieee80211_Sstar_put_bss(local->hw.wiphy,container_of((void *)bss, struct cfg80211_bss, priv));
}

static bool is_uapsd_supported(struct ieee802_Sstar_11_elems *elems)
{
	u8 qos_info;

	if (elems->wmm_info && elems->wmm_info_len == 7
	    && elems->wmm_info[5] == 1)
		qos_info = elems->wmm_info[6];
	else if (elems->wmm_param && elems->wmm_param_len == 24
		 && elems->wmm_param[5] == 1)
		qos_info = elems->wmm_param[6];
	else
		/* no valid wmm information or parameter element found */
		return false;

	return qos_info & IEEE80211_WMM_IE_AP_QOSINFO_UAPSD;
}

struct ieee80211_bss *
ieee80211_bss_info_update(struct ieee80211_local *local,
			  struct ieee80211_rx_status *rx_status,
			  struct Sstar_ieee80211_mgmt *mgmt,
			  size_t len,
			  struct ieee802_Sstar_11_elems *elems,
			  struct ieee80211_channel *channel,
			  bool beacon)
{
	#ifdef SSTAR_CHANGE_AP_TIMESTAMP_TO_BOOTTIME
	/*
	*Fix android 7.0 bug!!!!!
	*In the framework,when it starts to scan,it will record a starttime(signed 64 bit boottime) which will be used at can end.
	*after scan end,the framework will process probe response/beacon, and compare tsf in the probe response/beacon
	*with the scan starttime, and the tsf must be greater than the scan starttime. so here change the tsf
	*to the max signed inter value(0x7fffffffffffffff).
	*/
	#define SSTAR_BOOT_TIME 0x7fffffffffffffffLL//((u64)(ktime_to_ns(ktime_get_boottime())>>10))
	#endif
	
	struct cfg80211_bss *cbss;
	struct ieee80211_bss *bss;
	int clen, srlen;
	s32 signal = 0;

	if (local->hw.flags & IEEE80211_HW_SIGNAL_DBM)
		signal = rx_status->signal * 100;
	else if (local->hw.flags & IEEE80211_HW_SIGNAL_UNSPEC)
		signal = (rx_status->signal * 100) / local->hw.max_signal;
	#ifdef SSTAR_CHANGE_AP_TIMESTAMP_TO_BOOTTIME
	{
		__le16 fc = mgmt->frame_control;
		if(ieee80211_is_probe_resp(fc)){
			mgmt->u.probe_resp.timestamp = cpu_to_le64(SSTAR_BOOT_TIME);
		}
		else if(ieee80211_is_beacon(fc)){
			mgmt->u.beacon.timestamp = cpu_to_le64(SSTAR_BOOT_TIME);
		}else{
			WARN_ON(1);
		}
	}
	#undef SSTAR_BOOT_TIME
	#endif
	cbss = cfg80211_inform_bss_frame(local->hw.wiphy, channel,
					 (struct ieee80211_mgmt*)mgmt, len, signal, GFP_ATOMIC);

	if (!cbss)
		return NULL;
	#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
	cbss->free_priv = ieee80211_rx_bss_free;
	#endif
	bss = (void *)cbss->priv;

	/* save the ERP value so that it is available at association time */
	if (elems->erp_info && elems->erp_info_len >= 1) {
		bss->erp_value = elems->erp_info[0];
		bss->has_erp_value = 1;
	}

	if (elems->tim) {
		struct ieee80211_tim_ie *tim_ie =
			(struct ieee80211_tim_ie *)elems->tim;
		bss->dtim_period = tim_ie->dtim_period;
	}

	/* If the beacon had no TIM IE, or it was invalid, use 1 */
	if (beacon && !bss->dtim_period)
		bss->dtim_period = 1;

	/* replace old supported rates if we get new values */
	srlen = 0;
	if (elems->supp_rates) {
		clen = IEEE80211_MAX_SUPP_RATES;
		if (clen > elems->supp_rates_len)
			clen = elems->supp_rates_len;
		memcpy(bss->supp_rates, elems->supp_rates, clen);
		srlen += clen;
	}
	if (elems->ext_supp_rates) {
		clen = IEEE80211_MAX_SUPP_RATES - srlen;
		if (clen > elems->ext_supp_rates_len)
			clen = elems->ext_supp_rates_len;
		memcpy(bss->supp_rates + srlen, elems->ext_supp_rates, clen);
		srlen += clen;
	}
	if (srlen)
		bss->supp_rates_len = srlen;

	bss->wmm_used = elems->wmm_param || elems->wmm_info;
	bss->uapsd_supported = is_uapsd_supported(elems);
	bss->noise_floor     = local->noise_floor[(channel_hw_value(channel)-1)%13];//-92;
	if(bss->noise_floor > -80)
		bss->noise_floor = -80;

	if (!beacon)
		bss->last_probe_resp = jiffies;

	return bss;
}
bool  ieee80211_scan_internal_req_results(struct ieee80211_local *local,struct Sstar_internal_scan_results_req *req)
{
	struct hlist_head *hlist;
	struct hlist_node *node;
	struct hlist_node *node_temp;
	struct Sstar_internal_scan_sta_node *sta_node;
	struct ieee80211_internal_scan_sta sta;
	int hash_index = 0;
	bool ret = true;

	spin_lock_bh(&local->internal_scan_list_lock);
	for(hash_index = 0;hash_index<SSTAR_COMMON_HASHENTRIES;hash_index++){
		hlist = &local->internal_scan_list[hash_index];
		hlist_for_each_safe(node,node_temp,hlist){
			
			sta_node = hlist_entry(node,struct Sstar_internal_scan_sta_node,hnode);
			
			if(req->flush == true){
				local->internal_scan_n_results--;
				hlist_del(&sta_node->hnode);
			}
			
			memcpy(&sta,&sta_node->sta,sizeof(struct ieee80211_internal_scan_sta));
			spin_unlock_bh(&local->internal_scan_list_lock);
			if(ret == true){
				if(req->result_handle)
					ret = req->result_handle(&local->hw,req,&sta);
			}
			spin_lock_bh(&local->internal_scan_list_lock);
			
			if(req->flush == true){
				if(sta_node->sta.ie){
					Sstar_kfree(sta_node->sta.ie);
					sta_node->sta.ie = NULL;
					sta_node->sta.ie_len = 0;
				}
				Sstar_kfree(sta_node);
			}
		}
	}
	spin_unlock_bh(&local->internal_scan_list_lock);

	return ret;
}
static void  ieee80211_scan_internal_list_flush(struct ieee80211_local *local)
{
	struct hlist_head *hlist;
	struct hlist_node *node;
	struct hlist_node *node_temp;
	struct Sstar_internal_scan_sta_node *sta_node;
	int hash_index = 0;

	spin_lock_bh(&local->internal_scan_list_lock);
	for(hash_index = 0;hash_index<SSTAR_COMMON_HASHENTRIES;hash_index++){
		hlist = &local->internal_scan_list[hash_index];
		hlist_for_each_safe(node,node_temp,hlist){
			sta_node = hlist_entry(node,struct Sstar_internal_scan_sta_node,hnode);
			local->internal_scan_n_results--;
			hlist_del(&sta_node->hnode);
			if(sta_node->sta.ie){
				Sstar_kfree(sta_node->sta.ie);
				sta_node->sta.ie = NULL;
				sta_node->sta.ie_len = 0;
			}
			Sstar_kfree(sta_node);
		}
	}
	spin_unlock_bh(&local->internal_scan_list_lock);
}

void ieee80211_scan_internal_deinit(struct ieee80211_local *local)
{
	u8 *ie = NULL;
	
	ieee80211_scan_internal_list_flush(local);

	mutex_lock(&local->mtx);
	
	ie = rcu_dereference(local->internal_scan_ie);
	rcu_assign_pointer(local->internal_scan_ie,NULL);
	local->internal_scan_ie_len = 0;
	synchronize_rcu();

	if(ie)
		Sstar_kfree(ie);
	mutex_unlock(&local->mtx);
}

void ieee80211_scan_internal_int(struct ieee80211_local *local)
{
	/*****internal scan init **********/
	spin_lock_init(&local->internal_scan_list_lock);
	init_waitqueue_head(&local->internal_scan_wq);
	atomic_set(&local->internal_scan_status,IEEE80211_INTERNAL_SCAN_STATUS__IDLE);
	spin_lock_bh(&local->internal_scan_list_lock);
	Sstar_common_hash_list_init(local->internal_scan_list,SSTAR_COMMON_HASHENTRIES);
	spin_unlock_bh(&local->internal_scan_list_lock);
}
static void ieee80211_scan_rx_internal_update(struct ieee80211_local *local,
															 struct ieee80211_internal_scan_result *result)
{
	int hash_index = 0;
	struct hlist_head *hlist;
	struct hlist_node *node;
	struct Sstar_internal_scan_sta_node *sta_node;
	struct Sstar_internal_scan_sta_node *sta_node_target = NULL;
	
	Sstar_printk_debug("%s:ssid[%s],mac[%pM],channel[%d],signal[%d]\n",__func__,result->sta.ssid,result->sta.bssid,result->sta.channel,result->sta.signal);
	spin_lock_bh(&local->internal_scan_list_lock);
	/*
	*find target sta from hash list
	*/
	if(result->sta.ssid_len)
		hash_index = Sstar_hash_index(result->sta.ssid,result->sta.ssid_len,SSTAR_COMMON_HASHBITS);
	else
		hash_index = 0;

	hlist = &local->internal_scan_list[hash_index];

	hlist_for_each(node,hlist){
		sta_node = hlist_entry(node,struct Sstar_internal_scan_sta_node,hnode);
		if ((result->sta.ssid_len == sta_node->sta.ssid_len)&&(!memcmp(result->sta.bssid,sta_node->sta.bssid,6))){
			/*
			*hidden ssid
			*/
			if((result->sta.ssid_len == 0) || (!memcmp(sta_node->sta.ssid,result->sta.ssid,sta_node->sta.ssid_len)))
				sta_node_target = sta_node;
			break;
		}
	}
	/*
	*insert new sta to hash list
	*/
	if(sta_node_target == NULL){
		sta_node_target = Sstar_kzalloc(sizeof(struct Sstar_internal_scan_sta_node),GFP_ATOMIC);

		if(sta_node_target == NULL){
			Sstar_printk_always("%s:sta_node_target == NULL\n",__func__);
			spin_unlock_bh(&local->internal_scan_list_lock);
			return;
		}
		local->internal_scan_n_results++;
		hlist_add_head(&sta_node_target->hnode,hlist);
	}else {
		if(sta_node_target->sta.ie){
			/*
			*only save the new special ie,here free the old one.
			*/
			Sstar_kfree(sta_node_target->sta.ie);
			sta_node_target->sta.ie = NULL;
			sta_node_target->sta.ie_len = 0;
		}
	}
	/*
	*update sta infor
	*/
	BUG_ON(sta_node_target == NULL);
	memcpy(&sta_node_target->sta,&result->sta,sizeof(struct ieee80211_internal_scan_sta));
	sta_node_target->sta.ssid_len = result->sta.ssid_len;
	if(result->sta.ssid_len)
		memcpy(sta_node_target->sta.ssid,result->sta.ssid,result->sta.ssid_len);
	if(sta_node_target->sta.ie)
		Sstar_printk_debug("%s:ie(%s)\n",__func__,sta_node_target->sta.ie);
	spin_unlock_bh(&local->internal_scan_list_lock);

}
bool ieee80211_scan_rx_internal_default(struct ieee80211_sub_if_data *sdata,
													   struct ieee80211_internal_scan_result *result,bool finish)
{
	struct ieee80211_local *local = sdata->local;

	if(finish == false){
		ieee80211_scan_rx_internal_update(local,result);
	}else {
	}

	return true;
}
void ieee80211_scan_cca_notify(struct ieee80211_hw *hw,struct ieee80211_internal_scan_notity *notify)
{
	struct ieee80211_local *local = hw_to_local(hw);
	struct ieee80211_scan_req_wrap *req_wrap = &local->scan_req_wrap;
	struct cfg80211_scan_request *req = req_wrap->req;
	u8 index;
	u8 len = notify->cca.val_len;
	u8* cca_val = notify->cca.val;
	/*
	*cca only can run in internal scan mode
	*/
	Sstar_printk_debug("%s\n",__func__);
	WARN_ON(test_bit(SCAN_INTERNAL_SCANNING, &local->scanning) == 0);
	WARN_ON((req_wrap->flags & IEEE80211_SCAN_REQ_CCA) == 0);

	if(req == NULL){
		WARN_ON(1);
		return;
	}

	if(len > IEEE80211_SSTAR_MAX_SCAN_CHANNEL_INDEX){
		WARN_ON(1);
		return;
	}
	
	for(index = 0;index<req->n_channels;index++){
		
		if(index>len){
			WARN_ON(1);
			break;
		}
		req_wrap->cca_val[channel_hw_value(req->channels[index])-1] = cca_val[index];
		
	}
}
void ieee80211_scan_cca_val_put(struct ieee80211_hw *hw)
{
	struct ieee80211_local *local = hw_to_local(hw);
	struct ieee80211_scan_req_wrap *req_wrap = &local->scan_req_wrap;

	memset(req_wrap->cca_val,0,IEEE80211_SSTAR_MAX_SCAN_CHANNEL_INDEX);
}

u8* ieee80211_scan_cca_val_get(struct ieee80211_hw *hw)
{
	struct ieee80211_local *local = hw_to_local(hw);
	struct ieee80211_scan_req_wrap *req_wrap = &local->scan_req_wrap;


	return req_wrap->cca_val;
}
static ieee80211_rx_result 
ieee80211_scan_rx_internal_sta_info(struct ieee80211_sub_if_data *sdata, struct sk_buff *skb)
{
	struct ieee80211_rx_status *rx_status = IEEE80211_SKB_RXCB(skb);
	struct Sstar_ieee80211_mgmt *mgmt;
	u8 *elements;
	size_t baselen;
	int freq;
	__le16 fc;
	bool presp, beacon = false;
	struct ieee802_Sstar_11_elems elems;
	bool (*func)(struct ieee80211_sub_if_data *sdata,void *fs_data,struct ieee80211_internal_scan_result *result,bool finish);
	void *scan_data;
	ieee80211_rx_result handle = RX_QUEUED;
	struct ieee80211_internal_scan_result scan_info;
	
	rcu_read_lock();
	
	memset(&scan_info,0,sizeof(struct ieee80211_internal_scan_result));
	func = rcu_dereference(sdata->local->internal_scan.req.result_handle);
	scan_data = rcu_dereference(sdata->local->internal_scan.req.priv);
	
	if (skb->len < 2){
		handle = RX_DROP_UNUSABLE;
		goto err;
	}

	mgmt = (struct Sstar_ieee80211_mgmt *) skb->data;
	fc = mgmt->frame_control;

	if (ieee80211_is_ctl(fc)){
		handle = RX_CONTINUE;
		goto err;
	}
	if (skb->len < 24){
		handle = RX_CONTINUE;
		goto err;
	}

	presp = ieee80211_is_probe_resp(fc);
	if (presp) {
		/* ignore ProbeResp to foreign address */
		if (memcmp(mgmt->da, sdata->vif.addr, ETH_ALEN)){
			handle = RX_DROP_MONITOR;
			goto err;
		}

		presp = true;
		elements = mgmt->u.probe_resp.variable;
		baselen = offsetof(struct Sstar_ieee80211_mgmt, u.probe_resp.variable);
	} else {
		beacon = ieee80211_is_beacon(fc);
		baselen = offsetof(struct Sstar_ieee80211_mgmt, u.beacon.variable);
		elements = mgmt->u.beacon.variable;
	}
	if (!presp && !beacon){
		handle = RX_CONTINUE;
		goto err;
	}
	if (baselen > skb->len){
		handle = RX_DROP_MONITOR;
		goto err;
	}
	if(sdata->local->internal_scan.req.n_macs){
		struct ieee80211_internal_mac *sta_mac;
		struct hlist_head *hlist;
		struct hlist_node *node;
		u8 hash_index = 0;
		bool found = false;

		hash_index = Sstar_hash_index(mgmt->bssid,6,IEEE80211_INTERNAL_SCAN_HASHBITS);
		hlist = &sdata->local->internal_scan.mac_hash_list[hash_index];

		hlist_for_each(node,hlist){
			sta_mac = hlist_entry(node,struct ieee80211_internal_mac,hnode);
			if (!memcmp(sta_mac->mac, mgmt->bssid, 6)){
				found = true;
				break;
			}
		}
		if(found == false){
			Sstar_printk_debug("%s,mac[%pM] not in list\n",__func__,mgmt->bssid);
			handle = RX_DROP_MONITOR;
			goto err;
		}
	}
	ieee802_11_parse_elems(elements, skb->len - baselen, &elems);

	if (elems.ds_params && elems.ds_params_len == 1)
		freq = ieee80211_channel_to_frequency(elems.ds_params[0],
						      rx_status->band);
	else
		freq = rx_status->freq;

	if(elems.rsn && elems.rsn_len && elems.wpa && elems.wpa_len){
		scan_info.sta.enc_type = IEEE80211_ENC_WPA_WPA2;
	}else if(elems.rsn && elems.rsn_len)
		scan_info.sta.enc_type = IEEE80211_ENC_WPA2;
	else if(elems.wpa && elems.wpa_len)
		scan_info.sta.enc_type = IEEE80211_ENC_WPA;
	else if(((presp == true) && (mgmt->u.probe_resp.capab_info & SSTAR_WLAN_CAPABILITY_PRIVACY)) ||
		    ((beacon == true)&& (mgmt->u.beacon.capab_info & SSTAR_WLAN_CAPABILITY_PRIVACY))){
		scan_info.sta.enc_type = IEEE80211_ENC_WEP;
	}else {
		scan_info.sta.enc_type = IEEE80211_ENC_OPEN;
	}
	memcpy(scan_info.sta.bssid,mgmt->bssid,ETH_ALEN);
	scan_info.sta.ssid_len = elems.ssid_len;
	if(elems.ssid_len>0)
		memcpy(scan_info.sta.ssid,elems.ssid,elems.ssid_len);
	
	scan_info.sta.channel = channel_hw_value(ieee80211_get_channel(sdata->local->hw.wiphy,freq));
	scan_info.sta.signal = rx_status->signal;

	if(elems.Sstar_special_len && elems.Sstar_special){
		scan_info.sta.ie = Sstar_kzalloc(elems.Sstar_special_len,GFP_ATOMIC);

		if(scan_info.sta.ie == NULL){
			Sstar_printk_err("%s,scan_info.sta.ie alloc err\n",__func__);
			handle = RX_DROP_MONITOR;
			goto err;
		}
		memcpy(scan_info.sta.ie,elems.Sstar_special,elems.Sstar_special_len);
		scan_info.sta.ie_len = elems.Sstar_special_len;
	}
	if(sdata->local->scan_req_wrap.flags & IEEE80211_SCAN_REQ_CCA){
		struct ieee80211_channel *channel = ieee80211_get_channel(sdata->local->hw.wiphy,freq);

		if(channel == NULL){
			handle = RX_DROP_MONITOR;
			if(scan_info.sta.ie)
				Sstar_kfree(scan_info.sta.ie);
			goto err;
		}	

		if(channel_in_cca(channel) == false){
			handle = RX_DROP_MONITOR;
			if(scan_info.sta.ie)
				Sstar_kfree(scan_info.sta.ie);
			goto err;
		}
		scan_info.sta.cca = true;
	}
	if(func){
		if(func(sdata->local->scan_sdata,scan_data,&scan_info,false) == false){
			handle = RX_DROP_MONITOR;
			if(scan_info.sta.ie)
				Sstar_kfree(scan_info.sta.ie);
			goto err;
		}
	}else if(sdata->local->scan_req_wrap.flags & IEEE80211_SCAN_REQ_RESULTS_HANDLE){
			ieee80211_scan_rx_internal_default(sdata->local->scan_sdata,&scan_info,false);
	}else {
		if(scan_info.sta.ie)
			Sstar_kfree(scan_info.sta.ie);
	}
	Sstar_dev_kfree_skb(skb);
err:
	rcu_read_unlock();
	return handle;	
}

static ieee80211_rx_result 
ieee80211_scan_rx_internal_skb(struct ieee80211_sub_if_data *sdata, struct sk_buff *skb)
{
	struct Sstar_ieee80211_mgmt *mgmt;
	ieee80211_rx_result handle = RX_QUEUED;
	bool (*func)(struct ieee80211_sub_if_data *sdata,void *fs_data,struct ieee80211_internal_scan_result *result,bool finish);
	void *scan_data;
	struct ieee80211_internal_scan_result scan_info;
	__le16 fc;
	
	rcu_read_lock();
	
	memset(&scan_info,0,sizeof(struct ieee80211_internal_scan_result));
	func = rcu_dereference(sdata->local->internal_scan.req.result_handle);
	scan_data = rcu_dereference(sdata->local->internal_scan.req.priv);
	
	if (skb->len < 2){
		handle = RX_DROP_UNUSABLE;
		goto err;
	}

	if(func==NULL){
		handle = RX_DROP_UNUSABLE;
		goto err;
	}

	mgmt = (struct Sstar_ieee80211_mgmt *) skb->data;
	fc = mgmt->frame_control;

	if (ieee80211_is_ctl(fc)){
		handle = RX_CONTINUE;
		goto err;
	}
	if (skb->len < 24){
		handle = RX_CONTINUE;
		goto err;
	}

	if((!ieee80211_is_probe_resp(fc))&&(!ieee80211_is_beacon(fc))){
		Sstar_printk_err("%s:not beacon or probe respons(%x)\n",__func__,fc);
		handle = RX_CONTINUE;
		goto err;
	}

	if(sdata != sdata->local->scan_sdata){
		Sstar_printk_err("%s:[%s]->[%s]\n",__func__,sdata->name,sdata->local->scan_sdata->name);
		handle = RX_CONTINUE;
		goto err;
	}
	scan_info.sta.skb = skb;
	
	if(func(sdata->local->scan_sdata,scan_data,&scan_info,false) == false){
		handle = RX_CONTINUE;
		goto err;
	}	
err:
	Sstar_printk_debug("%s:handle(%zu)\n",__func__,(size_t)handle);
	rcu_read_unlock();
	return handle;	
}
ieee80211_rx_result
ieee80211_scan_rx_internal(struct ieee80211_sub_if_data *sdata, struct sk_buff *skb)
{
	ieee80211_rx_result handle = RX_QUEUED;
	if(sdata->local->scan_req_wrap.flags & IEEE80211_SCAN_REQ_RESULTS_SKB){	
		Sstar_printk_debug("%s receive skb\n",__func__);
		handle = ieee80211_scan_rx_internal_skb(sdata,skb);
	}else{		
		Sstar_printk_debug("%s receive stainfo\n",__func__);
		handle = ieee80211_scan_rx_internal_sta_info(sdata,skb);
	}
	return handle;	
}
ieee80211_rx_result
ieee80211_scan_rx(struct ieee80211_sub_if_data *sdata, struct sk_buff *skb)
{
	struct ieee80211_rx_status *rx_status = IEEE80211_SKB_RXCB(skb);
	struct Sstar_ieee80211_mgmt *mgmt;
	struct ieee80211_bss *bss;
	u8 *elements;
	struct ieee80211_channel *channel;
	size_t baselen;
	int freq;
	__le16 fc;
	bool presp, beacon = false;
	struct ieee802_Sstar_11_elems elems;
	if (skb->len < 2)
		return RX_DROP_UNUSABLE;

	mgmt = (struct Sstar_ieee80211_mgmt *) skb->data;
	fc = mgmt->frame_control;

	if (ieee80211_is_ctl(fc))
		return RX_CONTINUE;

	if (skb->len < 24)
		return RX_CONTINUE;

	presp = ieee80211_is_probe_resp(fc);
	if (presp) {
		/* ignore ProbeResp to foreign address */
		if (memcmp(mgmt->da, sdata->vif.addr, ETH_ALEN))
			return RX_DROP_MONITOR;

		presp = true;
		elements = mgmt->u.probe_resp.variable;
		baselen = offsetof(struct Sstar_ieee80211_mgmt, u.probe_resp.variable);
	} else {
		beacon = ieee80211_is_beacon(fc);
		baselen = offsetof(struct Sstar_ieee80211_mgmt, u.beacon.variable);
		elements = mgmt->u.beacon.variable;
	}
	if (!presp && !beacon)
		return RX_CONTINUE;

	if (baselen > skb->len)
		return RX_DROP_MONITOR;

	ieee802_11_parse_elems(elements, skb->len - baselen, &elems);

	if (elems.ds_params && elems.ds_params_len == 1)
		freq = ieee80211_channel_to_frequency(elems.ds_params[0],
						      rx_status->band);
	else
		freq = rx_status->freq;

	channel = ieee80211_get_channel(sdata->local->hw.wiphy, freq);

	if (!channel || channel->flags & IEEE80211_CHAN_DISABLED)
		return RX_DROP_MONITOR;

	bss = ieee80211_bss_info_update(sdata->local, rx_status,
					mgmt, skb->len, &elems,
					channel, beacon);
	if (bss)
		ieee80211_rx_bss_put(sdata->local, bss);

	Sstar_dev_kfree_skb(skb);
	return RX_QUEUED;
}

/* return false if no more work */
static bool ieee80211_prep_hw_scan(struct ieee80211_local *local)
{
	struct cfg80211_scan_request *req = local->scan_req;
	enum ieee80211_band band = IEEE80211_BAND_2GHZ;
	int i, ielen, n_chans;
	struct ieee80211_internal_ap_conf *conf = NULL;
	bool use_req = true;
	bool abort = false;
	
	rcu_read_lock();
	conf = rcu_dereference(local->scan_sdata->internal_ap_conf);
	while(conf){
		struct ieee80211_channel *ap_channel = ieee8011_chnum_to_channel(&local->hw,conf->channel);

		if(local->hw_scan_band){
			abort = true;
			break;
		}

		if(ap_channel == NULL){
			break;
		}
		
		local->hw_scan_band++;
		local->hw_scan_req->channels[0] = ap_channel;
		n_chans = 1;
		use_req = false;
		band = ap_channel->band;
		Sstar_printk_debug("%s: use ap channel[%d]\n",__func__,conf->channel);
		break;
	}
	rcu_read_unlock();
	
	do {
		if (local->hw_scan_band == IEEE80211_NUM_BANDS)
			return false;
		
		if (abort == true)
			return false;
		
		if (use_req == false)
			break;
		
		band = local->hw_scan_band;
		n_chans = 0;
		for (i = 0; i < req->n_channels; i++) {
			if (req->channels[i]->band == band) {
				local->hw_scan_req->channels[n_chans] =
							req->channels[i];
				n_chans++;
			}
		}

		local->hw_scan_band++;
	} while (!n_chans);

	local->hw_scan_req->n_channels = n_chans;
	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 1, 0))
	ielen = ieee80211_build_preq_ies(local, (u8 *)local->hw_scan_req->ie,
					 req->ie, req->ie_len, band,
					 req->rates[band], 0);
	#else
	ielen = ieee80211_build_preq_ies(local, (u8 *)local->hw_scan_req->ie,
					 req->ie, req->ie_len, band,
					 (u32)(~0), 0);

	#endif
	local->hw_scan_req->ie_len = ielen;
	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 1, 0))
	local->hw_scan_req->no_cck = req->no_cck;
	#endif

	return true;
}
void ieee80211_run_pending_scan(struct ieee80211_local *local)
{
	lockdep_assert_held(&local->mtx);
	if ( local->scanning)
	{
		Sstar_printk_scan("%s:scanning,so cannot active pennding scan\n",__func__);
		WARN_ON(1);
		return;
	}

	
	if(local->pending_scan_req&&local->pending_scan_sdata)
	{
		if(ieee80211_sdata_running(local->pending_scan_sdata))
		{
			Sstar_printk_scan( "%s\n",__func__);
			local->scan_req = local->pending_scan_req;
			local->scan_sdata = local->pending_scan_sdata;
			local->pending_scan_req = NULL;
			local->pending_scan_sdata = NULL;
			ieee80211_queue_delayed_work(&local->hw, &local->scan_work, round_jiffies_relative(0));
		}
		else
		{
			Sstar_printk_scan("%s:sdata is not running,but scan,err!!!!!!\n",__func__);
			Sstar_notify_scan_done(local,local->pending_scan_req, true);
			local->pending_scan_req = NULL;
			local->pending_scan_sdata = NULL;
		}
	}
}
static void ieee80211_internal_scan_completed(struct ieee80211_hw *hw,bool aborted)
{	
	struct ieee80211_local *local = hw_to_local(hw);
	struct ieee80211_internal_scan_request *req = &local->internal_scan.req;
	u8 index = 0;
	bool (*func)(struct ieee80211_sub_if_data *sdata,void *fs_data,struct ieee80211_internal_scan_result *result,bool finish);
	void *scan_data;

	lockdep_assert_held(&local->mtx);
	
	func = rcu_dereference(local->internal_scan.req.result_handle);
	scan_data = rcu_dereference(local->internal_scan.req.priv);

	rcu_assign_pointer(local->internal_scan.req.result_handle,NULL);
	rcu_assign_pointer(local->internal_scan.req.priv,NULL);
	local->scanning = 0;
	synchronize_rcu();

	for(index = 0;index<req->n_macs;index++){
		hlist_del(&req->macs[index].hnode);
	}
	if(local->scan_req_wrap.flags & IEEE80211_SCAN_REQ_CCA){
		for(index = 0;index<local->scan_req->n_channels;index++){
			channel_clear_cca(local->scan_req->channels[index]);
		}
	}
	req->channels = NULL;
	req->n_channels = 0;
	req->ies = NULL;
	req->ie_len = 0;
	req->macs = NULL;
	req->n_macs = 0;
	req->ssids = NULL;
	req->n_ssids = 0;
	req->req_flags = 0;
	
	if(func)
		func(local->scan_sdata,scan_data,NULL,true);
	else if(local->scan_req_wrap.flags & IEEE80211_SCAN_REQ_RESULTS_HANDLE){
		WARN_ON(atomic_read(&local->internal_scan_status) != IEEE80211_INTERNAL_SCAN_STATUS__WAIT);
		if(aborted == true)
			atomic_set(&local->internal_scan_status,IEEE80211_INTERNAL_SCAN_STATUS__ABORT);
		else 
			atomic_set(&local->internal_scan_status,IEEE80211_INTERNAL_SCAN_STATUS__FINISHED);

		wake_up(&local->internal_scan_wq);
	}else {
		WARN_ON(1);
	}
	synchronize_rcu();
	Sstar_kfree(local->scan_req);
}
static void __ieee80211_scan_completed(struct ieee80211_hw *hw, bool aborted,
				       bool was_hw_scan)
{
	struct ieee80211_local *local = hw_to_local(hw);
	struct ieee80211_sub_if_data *sdata;

	lockdep_assert_held(&local->mtx);

	/*
	 * It's ok to abort a not-yet-running scan (that
	 * we have one at all will be verified by checking
	 * local->scan_req next), but not to complete it
	 * successfully.
	 */
	if (WARN_ON(!local->scanning && !aborted))
		aborted = true;

	if (WARN_ON(!local->scan_req))
		return;
	if (was_hw_scan && !aborted && ieee80211_prep_hw_scan(local)) {
		int rc = drv_hw_scan(local, local->scan_sdata, &local->scan_req_wrap);
		if (rc == 0)
		{
			Sstar_printk_scan( "%s:%d\n",__func__,__LINE__);
			return;
		}
	}

	Sstar_kfree(local->hw_scan_req);
	local->hw_scan_req = NULL;	
	local->scan_req_wrap.req = NULL;
	if (local->scan_req != local->int_scan_req){
		if(test_bit(SCAN_INTERNAL_SCANNING, &local->scanning)){
			ieee80211_internal_scan_completed(hw,aborted);
		}else 
			Sstar_notify_scan_done(local,local->scan_req, aborted);
	}
	local->scan_req_wrap.flags = 0;
	local->scan_req = NULL;
	sdata = local->scan_sdata;
	local->scan_sdata = NULL;

	local->scanning = 0;
	local->scan_channel = NULL;

	ieee80211_hw_config(local, IEEE80211_CONF_CHANGE_CHANNEL);
	/*scan stuck kernel panic wa*/
	if (!was_hw_scan && !aborted) {
		ieee80211_configure_filter(sdata);
		drv_sw_scan_complete(local);
		ieee80211_offchannel_return(local, true);
	}

	ieee80211_recalc_idle(local);

	ieee80211_mlme_notify_scan_completed(local);
	ieee80211_ibss_notify_scan_completed(local);
	ieee80211_mesh_notify_scan_completed(local);
	ieee80211_start_next_roc(local);
	ieee80211_queue_work(&local->hw, &local->work_work);
}

void ieee80211_scan_completed(struct ieee80211_hw *hw, bool aborted)
{
	struct ieee80211_local *local = hw_to_local(hw);

	trace_api_scan_completed(local, aborted);

	set_bit(SCAN_COMPLETED, &local->scanning);
	if (aborted)
		set_bit(SCAN_ABORTED, &local->scanning);
	ieee80211_queue_delayed_work(&local->hw, &local->scan_work, 0);
}
//EXPORT_SYMBOL(ieee80211_scan_completed);

static int ieee80211_start_sw_scan(struct ieee80211_local *local)
{
	/*
	 * Hardware/driver doesn't support hw_scan, so use software
	 * scanning instead. First send a nullfunc frame with power save
	 * bit on so that AP will buffer the frames for us while we are not
	 * listening, then send probe requests to each channel and wait for
	 * the responses. After all channels are scanned, tune back to the
	 * original channel and send a nullfunc frame with power save bit
	 * off to trigger the AP to send us all the buffered frames.
	 *
	 * Note that while local->sw_scanning is true everything else but
	 * nullfunc frames and probe requests will be dropped in
	 * ieee80211_tx_h_check_assoc().
	 */
	drv_sw_scan_start(local);

	ieee80211_offchannel_stop_beaconing(local);

	local->leave_oper_channel_time = 0;
	local->next_scan_state = SCAN_DECISION;
	local->scan_channel_idx = 0;

	drv_flush(local, local->scan_sdata, false);

	ieee80211_configure_filter(local->scan_sdata);

	/* We need to set power level at maximum rate for scanning. */
	ieee80211_hw_config(local, 0);

	ieee80211_queue_delayed_work(&local->hw,
				     &local->scan_work,
				     IEEE80211_CHANNEL_TIME);

	return 0;
}


static int __ieee80211_start_scan(struct ieee80211_sub_if_data *sdata,
				  struct cfg80211_scan_request *req)
{
	struct ieee80211_local *local = sdata->local;
	int rc;

	lockdep_assert_held(&local->mtx);

	if (local->scan_req)
	{
		Sstar_printk_scan("%s:%d\n",__func__,__LINE__);
		return -EBUSY;
	}

	if (!list_empty(&local->roc_list))
	{
#if 0
		printk( "%s:%d\n",__func__,__LINE__);
		return -EBUSY;
#endif
		/*
		*android 5.0 we can not return err, because the p2p function can not 
		*reset the p2p station mechine.
		*/

		if((local->pending_scan_req != NULL ) || (local->pending_scan_sdata != NULL))
		{
			Sstar_printk_scan("%s:only can pending one scan request\n",__func__);
			return -EBUSY;
		}
		else
		{
			local->pending_scan_req = req;
			local->pending_scan_sdata = sdata;
			Sstar_printk_scan( "%s:%d: offch runing ,so delay scanning\n",__func__,__LINE__);
		}
		return 0;
	}

	if (!list_empty(&local->work_list)) {
		/* wait for the work to finish/time out */
		local->scan_req = req;
		local->scan_sdata = sdata;
		local->pending_scan_start_time = jiffies;
		Sstar_printk_scan("%s(%s):work_list is not empty,pend scan\n",__func__,sdata->name);
		return 0;
	}
	memset(&local->scan_info,0,sizeof(struct cfg80211_scan_info));
	if (local->ops->hw_scan) {
		u8 *ies;

		local->hw_scan_req = Sstar_kmalloc(
				sizeof(*local->hw_scan_req) +
				req->n_channels * sizeof(req->channels[0]) +
				2 + IEEE80211_MAX_SSID_LEN + local->scan_ies_len +
				req->ie_len, GFP_KERNEL);
		if (!local->hw_scan_req)
			return -ENOMEM;

		local->hw_scan_req->ssids = req->ssids;
		local->hw_scan_req->n_ssids = req->n_ssids;
		ies = (u8 *)local->hw_scan_req +
			sizeof(*local->hw_scan_req) +
			req->n_channels * sizeof(req->channels[0]);
		local->hw_scan_req->ie = ies;

		local->hw_scan_band = 0;

		/*
		 * After allocating local->hw_scan_req, we must
		 * go through until ieee80211_prep_hw_scan(), so
		 * anything that might be changed here and leave
		 * this function early must not go after this
		 * allocation.
		 */
	}

	local->scan_req = req;
	local->scan_sdata = sdata;

	if (local->ops->hw_scan)
		__set_bit(SCAN_HW_SCANNING, &local->scanning);
	else
		__set_bit(SCAN_SW_SCANNING, &local->scanning);
	/*
	*cfg80211 triger scan
	*/
	__set_bit(SCAN_CFG80211_SCANNING, &local->scanning);
	
	ieee80211_recalc_idle(local);

	if (local->ops->hw_scan) {
		WARN_ON(!ieee80211_prep_hw_scan(local));
		if(sdata->last_scan_ie_len < local->hw_scan_req->ie_len){
			if(sdata->last_scan_ie){
				Sstar_kfree(sdata->last_scan_ie);
				sdata->last_scan_ie = NULL;
			}
			sdata->last_scan_ie = Sstar_kmalloc(local->hw_scan_req->ie_len,GFP_KERNEL);
			WARN_ON(sdata->last_scan_ie == NULL);
		}
		if(sdata->last_scan_ie&&local->hw_scan_req->ie){
			memcpy(sdata->last_scan_ie,local->hw_scan_req->ie,local->hw_scan_req->ie_len);
			sdata->last_scan_ie_len = local->hw_scan_req->ie_len;
		}
		local->scan_req_wrap.flags = 0;
		local->scan_req_wrap.req = local->hw_scan_req;
		rc = drv_hw_scan(local, sdata, &local->scan_req_wrap);
	} else
		rc = ieee80211_start_sw_scan(local);

	if (rc) {
		Sstar_kfree(local->hw_scan_req);
		local->hw_scan_req = NULL;
		local->scanning = 0;

		ieee80211_recalc_idle(local);

		local->scan_req = NULL;
		local->scan_sdata = NULL;
	}

	return rc;
}

static unsigned long
ieee80211_scan_get_channel_time(struct ieee80211_channel *chan)
{
	/*
	 * TODO: channel switching also consumes quite some time,
	 * add that delay as well to get a better estimation
	 */
	#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0))
	if (chan->flags & IEEE80211_CHAN_PASSIVE_SCAN)
	#else
	if (chan->flags & (IEEE80211_CHAN_NO_IR | IEEE80211_CHAN_RADAR))
	#endif
		return IEEE80211_PASSIVE_CHANNEL_TIME;
	return IEEE80211_PROBE_DELAY + IEEE80211_CHANNEL_TIME;
}

static void ieee80211_scan_state_decision(struct ieee80211_local *local,
					  unsigned long *next_delay)
{
	bool associated = false;
	bool tx_empty = true;
	bool bad_latency;
	bool listen_int_exceeded = 0;
	unsigned long min_beacon_int = 0;
	struct ieee80211_sub_if_data *sdata;
	struct ieee80211_channel *next_chan;

	/*
	 * check if at least one STA interface is associated,
	 * check if at least one STA interface has pending tx frames
	 * and grab the lowest used beacon interval
	 */
	mutex_lock(&local->iflist_mtx);
	list_for_each_entry(sdata, &local->interfaces, list) {
		if (!ieee80211_sdata_running(sdata))
			continue;

		if (sdata->vif.type == NL80211_IFTYPE_STATION) {
			if (sdata->u.mgd.associated) {
				associated = true;

				if (sdata->vif.bss_conf.beacon_int <
				    min_beacon_int || min_beacon_int == 0)
					min_beacon_int =
						sdata->vif.bss_conf.beacon_int;

				if (!qdisc_all_tx_empty(sdata->dev)) {
					tx_empty = false;
					break;
				}
			}
		}
	}
	mutex_unlock(&local->iflist_mtx);

	if (local->scan_channel) {
		/*
		 * we're currently scanning a different channel, let's
		 * see if we can scan another channel without interfering
		 * with the current traffic situation.
		 *
		 * Since we don't know if the AP has pending frames for us
		 * we can only check for our tx queues and use the current
		 * pm_qos requirements for rx. Hence, if no tx traffic occurs
		 * at all we will scan as many channels in a row as the pm_qos
		 * latency allows us to. Additionally we also check for the
		 * currently negotiated listen interval to prevent losing
		 * frames unnecessarily.
		 *
		 * Otherwise switch back to the operating channel.
		 */
		next_chan = local->scan_req->channels[local->scan_channel_idx];
#ifdef CONFIG_SSTAR_PM_QOS
		bad_latency = time_after(jiffies +
				ieee80211_scan_get_channel_time(next_chan),
				local->leave_oper_channel_time +
				usecs_to_jiffies(pm_qos_request(PM_QOS_NETWORK_LATENCY)));
#else
		bad_latency = time_after(jiffies +
				ieee80211_scan_get_channel_time(next_chan),
				local->leave_oper_channel_time +
				HZ/4);

#endif

		list_for_each_entry(sdata, &local->interfaces, list) {
			listen_int_exceeded = time_after(jiffies +
					ieee80211_scan_get_channel_time(next_chan),
					local->leave_oper_channel_time + // XXX: leave_oper_channel_time ?
					usecs_to_jiffies(min_beacon_int * 1024) *
					sdata->vif.bss_conf.listen_interval);
			if (listen_int_exceeded)
				break;
		}

		if (associated && ( !tx_empty || bad_latency ||
		    listen_int_exceeded))
			local->next_scan_state = SCAN_ENTER_OPER_CHANNEL;
		else
			local->next_scan_state = SCAN_SET_CHANNEL;
	} else {
		/*
		 * we're on the operating channel currently, let's
		 * leave that channel now to scan another one
		 */
		local->next_scan_state = SCAN_LEAVE_OPER_CHANNEL;
	}

	*next_delay = 0;
}

static void ieee80211_scan_state_leave_oper_channel(struct ieee80211_local *local,
						    unsigned long *next_delay)
{
	struct ieee80211_sub_if_data *sdata;
	ieee80211_offchannel_stop_station(local);

	__set_bit(SCAN_OFF_CHANNEL, &local->scanning);

	/*
	 * What if the nullfunc frames didn't arrive?
	 */
	mutex_lock(&local->iflist_mtx);

	list_for_each_entry(sdata, &local->interfaces, list)
		drv_flush(local, sdata, false);

	mutex_unlock(&local->iflist_mtx);
	if (local->ops->flush)
		*next_delay = 0;
	else
		*next_delay = HZ / 10;

	/* remember when we left the operating channel */
	local->leave_oper_channel_time = jiffies;

	/* advance to the next channel to be scanned */
	local->next_scan_state = SCAN_SET_CHANNEL;
}

static void ieee80211_scan_state_enter_oper_channel(struct ieee80211_local *local,
						    unsigned long *next_delay)
{
	/* switch back to the operating channel */
	local->scan_channel = NULL;
	ieee80211_hw_config(local, IEEE80211_CONF_CHANGE_CHANNEL);

	/*
	 * Only re-enable station mode interface now; beaconing will be
	 * re-enabled once the full scan has been completed.
	 */
	ieee80211_offchannel_return(local, false);

	__clear_bit(SCAN_OFF_CHANNEL, &local->scanning);

	*next_delay = HZ / 5;
	local->next_scan_state = SCAN_DECISION;
}

static void ieee80211_scan_state_set_channel(struct ieee80211_local *local,
					     unsigned long *next_delay)
{
	struct ieee80211_channel_state *chan_state = ieee80211_get_channel_state(local, local->scan_sdata);
	int skip;
	struct ieee80211_channel *chan;

	skip = 0;
	chan = local->scan_req->channels[local->scan_channel_idx];

	local->scan_channel = chan;

	/* Only call hw-config if we really need to change channels. */
	if (chan != chan_state->conf.channel)
		if (ieee80211_hw_config(local, IEEE80211_CONF_CHANGE_CHANNEL))
			skip = 1;

	/* advance state machine to next channel/band */
	local->scan_channel_idx++;

	if (skip) {
		/* if we skip this channel return to the decision state */
		local->next_scan_state = SCAN_DECISION;
		return;
	}

	/*
	 * Probe delay is used to update the NAV, cf. 11.1.3.2.2
	 * (which unfortunately doesn't say _why_ step a) is done,
	 * but it waits for the probe delay or until a frame is
	 * received - and the received frame would update the NAV).
	 * For now, we do not support waiting until a frame is
	 * received.
	 *
	 * In any case, it is not necessary for a passive scan.
	 */
	if (
		#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0))
		chan->flags & IEEE80211_CHAN_PASSIVE_SCAN
		#else
		chan->flags & IEEE80211_CHAN_NO_IR
		#endif
		||
	    !local->scan_req->n_ssids) {
		*next_delay = IEEE80211_PASSIVE_CHANNEL_TIME;
		local->next_scan_state = SCAN_DECISION;
		return;
	}

	/* active scan, send probes */
	*next_delay = IEEE80211_PROBE_DELAY;
	local->next_scan_state = SCAN_SEND_PROBE;
}

static void ieee80211_scan_state_send_probe(struct ieee80211_local *local,
					    unsigned long *next_delay)
{
	int i;
	struct ieee80211_sub_if_data *sdata = local->scan_sdata;
	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 1, 0))
	struct ieee80211_channel_state *chan_state = ieee80211_get_channel_state(local, sdata);
	enum ieee80211_band band = chan_state->conf.channel->band;
	#endif

	for (i = 0; i < local->scan_req->n_ssids; i++)
		#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 1, 0))
		ieee80211_send_probe_req(
			sdata, NULL,
			local->scan_req->ssids[i].ssid,
			local->scan_req->ssids[i].ssid_len,
			local->scan_req->ie, local->scan_req->ie_len,
			local->scan_req->rates[band], false,
			local->scan_req->no_cck);
		#else
		ieee80211_send_probe_req(
			sdata, NULL,
			local->scan_req->ssids[i].ssid,
			local->scan_req->ssids[i].ssid_len,
			local->scan_req->ie, local->scan_req->ie_len,
			(u32)(~0), false,
			0);

		#endif
	/*
	 * After sending probe requests, wait for probe responses
	 * on the channel.
	 */
	*next_delay = IEEE80211_CHANNEL_TIME;
	local->next_scan_state = SCAN_DECISION;
}

void ieee80211_scan_work(struct work_struct *work)
{
	struct ieee80211_local *local =
		container_of(work, struct ieee80211_local, scan_work.work);
	struct ieee80211_sub_if_data *sdata;
	unsigned long next_delay = 0;
	bool aborted, hw_scan;

	mutex_lock(&local->mtx);

	sdata = local->scan_sdata;

	if (test_and_clear_bit(SCAN_COMPLETED, &local->scanning)) {
		aborted = test_and_clear_bit(SCAN_ABORTED, &local->scanning);
		goto out_complete;
	}

	if (!sdata || !local->scan_req)
		goto out;

	if (local->scan_req && !local->scanning) {
		struct cfg80211_scan_request *req = local->scan_req;
		int rc;

		local->scan_req = NULL;
		local->scan_sdata = NULL;
		Sstar_printk_scan( "%s:[%s] start pending scan\n",__func__,sdata->name);
		rc = __ieee80211_start_scan(sdata, req);
		if (rc) {
			/* need to complete scan in cfg80211 */
			local->scan_req = req;
			aborted = true;
			goto out_complete;
		} else
			goto out;
	}
	if(test_bit(SCAN_HW_SCANNING,&local->scanning)){
		Sstar_printk_scan( "%s:[%s]: hw scan running\n",__func__,sdata->name);
		goto out;
	}
	/*
	 * Avoid re-scheduling when the sdata is going away.
	 */
	if (!ieee80211_sdata_running(sdata)) {
		aborted = true;
		goto out_complete;
	}

	/*
	 * as long as no delay is required advance immediately
	 * without scheduling a new work
	 */
	do {
		if (!ieee80211_sdata_running(sdata)) {
			aborted = true;
			goto out_complete;
		}

		switch (local->next_scan_state) {
		case SCAN_DECISION:
			/* if no more bands/channels left, complete scan */
			if (local->scan_channel_idx >= local->scan_req->n_channels) {
				aborted = false;
				goto out_complete;
			}
			ieee80211_scan_state_decision(local, &next_delay);
			break;
		case SCAN_SET_CHANNEL:
			ieee80211_scan_state_set_channel(local, &next_delay);
			break;
		case SCAN_SEND_PROBE:
			ieee80211_scan_state_send_probe(local, &next_delay);
			break;
		case SCAN_LEAVE_OPER_CHANNEL:
			ieee80211_scan_state_leave_oper_channel(local, &next_delay);
			break;
		case SCAN_ENTER_OPER_CHANNEL:
			ieee80211_scan_state_enter_oper_channel(local, &next_delay);
			break;
		}
	} while (next_delay == 0);

	ieee80211_queue_delayed_work(&local->hw, &local->scan_work, next_delay);
	goto out;

out_complete:
	hw_scan = test_bit(SCAN_HW_SCANNING, &local->scanning);
	__ieee80211_scan_completed(&local->hw, aborted, hw_scan);
out:
	mutex_unlock(&local->mtx);
}

int ieee80211_request_scan(struct ieee80211_sub_if_data *sdata,
			   struct cfg80211_scan_request *req)
{
	int res;

	mutex_lock(&sdata->local->mtx);
	res = __ieee80211_start_scan(sdata, req);
	mutex_unlock(&sdata->local->mtx);

	return res;
}
bool ieee80211_internal_scan_triger(struct ieee80211_sub_if_data *sdata,struct cfg80211_scan_request *req)
{
	struct ieee80211_local *local = sdata->local;
	int rc;
	u8 *ies;
	int i = 0;
	
	lockdep_assert_held(&local->mtx);

	if(local->internal_scan.req.req_flags & IEEE80211_INTERNAL_SCAN_FLAGS__CCA){
		
		Sstar_printk_err("%s:hw.conf.flags(%x)\n",__func__,local->hw.conf.flags);
		if(!!(local->hw.conf.flags & IEEE80211_CONF_IDLE) == 0){
			Sstar_printk_err("%s:not idle,associated with ap ,or started ap mode(%x)\n",__func__,local->hw.conf.flags);
			return false;
		}
	}
	ieee80211_scan_internal_list_flush(local);
	memset(&local->scan_info,0,sizeof(struct cfg80211_scan_info));
	local->hw_scan_req = Sstar_kmalloc(
			sizeof(*local->hw_scan_req) +
			req->n_channels * sizeof(req->channels[0]) +
			2 + IEEE80211_MAX_SSID_LEN + local->scan_ies_len +
			req->ie_len, GFP_KERNEL);
	if (!local->hw_scan_req)
		return false;

	local->hw_scan_req->ssids = req->ssids;
	local->hw_scan_req->n_ssids = req->n_ssids;
	ies = (u8 *)local->hw_scan_req +
		sizeof(*local->hw_scan_req) +
		req->n_channels * sizeof(req->channels[0]);
	local->hw_scan_req->ie = ies;

	local->hw_scan_band = 0;

	/*
	 * After allocating local->hw_scan_req, we must
	 * go through until ieee80211_prep_hw_scan(), so
	 * anything that might be changed here and leave
	 * this function early must not go after this
	 * allocation.
	 */

	local->scan_req = req;
	local->scan_sdata = sdata;

	__set_bit(SCAN_HW_SCANNING, &local->scanning);
	__set_bit(SCAN_INTERNAL_SCANNING, &local->scanning);
	atomic_set(&local->internal_scan_status,IEEE80211_INTERNAL_SCAN_STATUS__IDLE);
	ieee80211_recalc_idle(local);
	
	memset(&local->scan_req_wrap,0,sizeof(struct ieee80211_scan_req_wrap));

	local->scan_req_wrap.flags = IEEE80211_SCAN_REQ_INTERNAL;
	
	if(local->internal_scan.req.req_flags & IEEE80211_INTERNAL_SCAN_FLAGS__CCA){
		local->scan_req_wrap.flags |= IEEE80211_SCAN_REQ_CCA;

		for(i = 0;i<req->n_channels;i++){
			channel_mask_cca(req->channels[i]);
		}
	}
	if(!local->internal_scan.req.result_handle)
		local->scan_req_wrap.flags |= IEEE80211_SCAN_REQ_RESULTS_HANDLE;

	if(local->internal_scan.req.req_flags & IEEE80211_INTERNAL_SCAN_FLAGS__PASSAVI_SCAN)
		local->scan_req_wrap.flags |= IEEE80211_SCAN_REQ_PASSIVE_SCAN;
	if(local->internal_scan.req.req_flags & IEEE80211_INTERNAL_SCAN_FLAGS__NEED_SKB)
		local->scan_req_wrap.flags |= IEEE80211_SCAN_REQ_RESULTS_SKB;
	
	local->scan_req_wrap.req = local->hw_scan_req;
	WARN_ON(!ieee80211_prep_hw_scan(local));
	rc = drv_hw_scan(local, sdata, &local->scan_req_wrap);

	if (rc) {
		Sstar_kfree(local->hw_scan_req);
		local->hw_scan_req = NULL;
		local->scanning = 0;

		ieee80211_recalc_idle(local);
		for(i = 0;i<req->n_channels;i++){
			channel_clear_cca(req->channels[i]);
		}
		local->scan_req = NULL;
		local->scan_sdata = NULL;
		memset(&local->scan_req_wrap,0,sizeof(struct ieee80211_scan_req_wrap));
	}else{
		if(local->scan_req_wrap.flags & IEEE80211_SCAN_REQ_RESULTS_HANDLE){
			atomic_set(&local->internal_scan_status,IEEE80211_INTERNAL_SCAN_STATUS__WAIT);
		}else {
			atomic_set(&local->internal_scan_status,IEEE80211_INTERNAL_SCAN_STATUS__FINISHED);
		}
	}

	return rc == 0?true:false;
}

int ieee80211_request_internal_scan(struct ieee80211_sub_if_data *sdata,
				    const u8 *ssid, u8 ssid_len,
				    struct ieee80211_channel *chan)
{
	struct ieee80211_local *local = sdata->local;
	int ret = -EBUSY;
	enum ieee80211_band band;

	mutex_lock(&local->mtx);

	/* busy scanning */
	if (local->scan_req)
		goto unlock;

	/* fill internal scan request */
	if (!chan) {
		int i, nchan = 0;

		for (band = 0; band < IEEE80211_NUM_BANDS; band++) {
			if (!local->hw.wiphy->bands[band])
				continue;
			for (i = 0;
			     i < local->hw.wiphy->bands[band]->n_channels;
			     i++) {
				local->int_scan_req->channels[nchan] =
				    &local->hw.wiphy->bands[band]->channels[i];
				nchan++;
			}
		}

		local->int_scan_req->n_channels = nchan;
	} else {
		local->int_scan_req->channels[0] = chan;
		local->int_scan_req->n_channels = 1;
	}

	local->int_scan_req->ssids = &local->scan_ssid;
	local->int_scan_req->n_ssids = 1;
	memcpy(local->int_scan_req->ssids[0].ssid, ssid, IEEE80211_MAX_SSID_LEN);
	local->int_scan_req->ssids[0].ssid_len = ssid_len;

	ret = __ieee80211_start_scan(sdata, sdata->local->int_scan_req);
 unlock:
	mutex_unlock(&local->mtx);
	return ret;
}

/*
 * Only call this function when a scan can't be queued -- under RTNL.
 */
void ieee80211_scan_cancel(struct ieee80211_local *local)
{
	/*
	 * We are canceling software scan, or deferred scan that was not
	 * yet really started (see __ieee80211_start_scan ).
	 *
	 * Regarding hardware scan:
	 * - we can not call  __ieee80211_scan_completed() as when
	 *   SCAN_HW_SCANNING bit is set this function change
	 *   local->hw_scan_req to operate on 5G band, what race with
	 *   driver which can use local->hw_scan_req
	 *
	 * - we can not cancel scan_work since driver can schedule it
	 *   by ieee80211_scan_completed(..., true) to finish scan
	 *
	 * Hence we only call the cancel_hw_scan() callback, but the low-level
	 * driver is still responsible for calling ieee80211_scan_completed()
	 * after the scan was completed/aborted.
	 */

	mutex_lock(&local->mtx);
	if (!local->scan_req)
		goto out;

	if (test_bit(SCAN_HW_SCANNING, &local->scanning)) {
		if (local->ops->cancel_hw_scan)
			drv_cancel_hw_scan(local, local->scan_sdata);
		goto out;
	}

	/*
	 * If the work is currently running, it must be blocked on
	 * the mutex, but we'll set scan_sdata = NULL and it'll
	 * simply exit once it acquires the mutex.
	 */
	cancel_delayed_work(&local->scan_work);
	/* and clean up */
	__ieee80211_scan_completed(&local->hw, true, false);
out:
	mutex_unlock(&local->mtx);
}

int ieee80211_request_sched_scan_start(struct ieee80211_sub_if_data *sdata,
				       struct cfg80211_sched_scan_request *req)
{
	struct ieee80211_local *local = sdata->local;
	int ret, i;

	mutex_lock(&sdata->local->mtx);

	if (local->sched_scanning) {
		ret = -EBUSY;
		goto out;
	}

	if (!local->ops->sched_scan_start) {
		ret = -ENOTSUPP;
		goto out;
	}

	for (i = 0; i < IEEE80211_NUM_BANDS; i++) {
#ifdef ROAM_OFFLOAD
		if (!local->hw.wiphy->bands[i])
			continue;
#endif /*ROAM_OFFLOAD*/
		local->sched_scan_ies.ie[i] = Sstar_kzalloc(2 +
						      IEEE80211_MAX_SSID_LEN +
						      local->scan_ies_len +
						      req->ie_len,
						      GFP_KERNEL);
		if (!local->sched_scan_ies.ie[i]) {
			ret = -ENOMEM;
			goto out_free;
		}

		local->sched_scan_ies.len[i] =
			ieee80211_build_preq_ies(local,
						 local->sched_scan_ies.ie[i],
						 req->ie, req->ie_len, i,
						 (u32) -1, 0);
	}

	ret = drv_sched_scan_start(local, sdata, req,
				   &local->sched_scan_ies);
	if (ret == 0) {
		local->sched_scanning = true;
		goto out;
	}

out_free:
	while (i > 0)
		Sstar_kfree(local->sched_scan_ies.ie[--i]);
out:
	mutex_unlock(&sdata->local->mtx);
	return ret;
}

int ieee80211_request_sched_scan_stop(struct ieee80211_sub_if_data *sdata)
{
	struct ieee80211_local *local = sdata->local;
	int ret = 0, i;

	mutex_lock(&sdata->local->mtx);

	if (!local->ops->sched_scan_stop) {
		ret = -ENOTSUPP;
		goto out;
	}

	if (local->sched_scanning) {
		for (i = 0; i < IEEE80211_NUM_BANDS; i++)
			Sstar_kfree(local->sched_scan_ies.ie[i]);

		drv_sched_scan_stop(local, sdata);
		local->sched_scanning = false;
	}
out:
	mutex_unlock(&sdata->local->mtx);

	return ret;
}

void ieee80211_sched_scan_results(struct ieee80211_hw *hw)
{
	struct ieee80211_local *local = hw_to_local(hw);
#ifdef ROAM_OFFLOAD
	if(local->sched_scanning)
	{
		local->sched_scanning = false;
		return;
	}
#endif /*ROAM_OFFLOAD*/

	trace_api_sched_scan_results(local);

	cfg80211_sched_scan_results(hw->wiphy);
}
//EXPORT_SYMBOL(ieee80211_sched_scan_results);

void ieee80211_sched_scan_stopped_work(struct work_struct *work)
{
	struct ieee80211_local *local =
		container_of(work, struct ieee80211_local,
			     sched_scan_stopped_work);
	int i;

	mutex_lock(&local->mtx);

	if (!local->sched_scanning) {
		mutex_unlock(&local->mtx);
		return;
	}

	for (i = 0; i < IEEE80211_NUM_BANDS; i++)
		Sstar_kfree(local->sched_scan_ies.ie[i]);

	local->sched_scanning = false;

	mutex_unlock(&local->mtx);

	cfg80211_sched_scan_stopped(local->hw.wiphy);
}

void ieee80211_sched_scan_stopped(struct ieee80211_hw *hw)
{
	struct ieee80211_local *local = hw_to_local(hw);

	trace_api_sched_scan_stopped(local);

	ieee80211_queue_work(&local->hw, &local->sched_scan_stopped_work);
}
//EXPORT_SYMBOL(ieee80211_sched_scan_stopped);
