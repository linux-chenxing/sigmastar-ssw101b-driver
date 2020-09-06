#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/kthread.h>
#include <linux/dcache.h>
#include <linux/netdevice.h>
#include <linux/rtnetlink.h>
#include "apollo.h"
#include "bh.h"
#include "hwio.h"
#include "wsm.h"
#include "sbus.h"
#include "debug.h"
#include "apollo_plat.h"
#include "sta.h"
#include "ap.h"
#include "scan.h"
#include "internal_cmd.h"
#include "svn_version.h"

#define SSTAR_WSM_ADAPTIVE		"set_adaptive"SSTAR_SPACE_STR
#define SSTAR_WSM_TXPWR_DCXO		"set_txpwr_and_dcxo"SSTAR_SPACE_STR
#define SSTAR_WSM_TXPWR			"set_rate_txpower_mode"SSTAR_SPACE_STR
#define SSTAR_WSM_SET_FREQ		"set_freq"SSTAR_SPACE_STR"%d"SSTAR_SPACE_STR"%d"
#define SSTAR_WSM_FIX_RATE		"lmac_rate"SSTAR_SPACE_STR"%d"
#define SSTAR_WSM_TOP_RATE		"lmac_max_rate"SSTAR_SPACE_STR"%d"
#define SSTAR_WSM_MIN_RATE		"lmac_min_rate"SSTAR_SPACE_STR"%d"
#define SSTAR_WSM_SET_RATE_POWER	"set_spec_rate_txpower_mode"SSTAR_SPACE_STR"%d"SSTAR_SPACE_STR"%d"

#define SSTAR_WSM_CMD_LEN		1680
const char *chip_6038  = "6038";
const char *chip_6032  = "6032";
const char *chip_6032i = "6032i";
const char *chip_101B  = "101B";

int ieee80211_set_channel(struct wiphy *wiphy,
				 struct net_device *netdev,
				 struct ieee80211_channel *chan,
				 enum nl80211_channel_type channel_type);

static void Sstar_internal_cmd_scan_dump(struct ieee80211_internal_scan_request *scan_req)
{
	int i;
	if(scan_req->n_ssids){
		for(i = 0;i<scan_req->n_ssids;i++){
			Sstar_printk_debug("%s: ssid[%s][%d]\n",__func__,scan_req->ssids[i].ssid,scan_req->ssids[i].ssid_len);
		}
	}	
	if(scan_req->n_channels){
		for(i = 0;i<scan_req->n_channels;i++){
			Sstar_printk_debug("%s: channel[%d]\n",__func__,scan_req->channels[i]);
		}
	}
	if(scan_req->n_macs){
		for(i = 0;i<scan_req->n_macs;i++){
			Sstar_printk_debug("%s: mac[%pM]\n",__func__,scan_req->macs[i].mac);
		}
	}
	Sstar_printk_debug("%s: ie_len[%d]\n",__func__,scan_req->ie_len);
}

bool  Sstar_internal_cmd_scan_build(struct ieee80211_local *local,struct ieee80211_internal_scan_request *req,
											   u8* channels,int n_channels,struct cfg80211_ssid *ssids,int n_ssids,
											   struct ieee80211_internal_mac *macs,int n_macs)
{
	u8* local_scan_ie;
	u8* scan_ie;
	int ie_len;
	/*
	*use default internal handle
	*/
	req->result_handle = NULL;
	req->priv = NULL;

	req->channels = channels;
	req->n_channels = n_channels;
	
	req->ssids =  ssids;
	req->n_ssids = n_ssids;

	req->macs = macs;
	req->n_macs = n_macs;

	req->no_cck = true;
	
	rcu_read_lock();
	local_scan_ie = rcu_dereference(local->internal_scan_ie);
	ie_len  = local->internal_scan_ie_len;

	if(local_scan_ie && ie_len){
		scan_ie = Sstar_kzalloc(ie_len,GFP_ATOMIC);

		if(scan_ie == NULL){
			Sstar_printk_err("%s:alloc ie err\n",__func__);
			rcu_read_unlock();
			return false;
		}
		memcpy(scan_ie,local_scan_ie,ie_len);
		req->ies = scan_ie;
		req->ie_len = ie_len;
	}else {
		req->ies = NULL;
		req->ie_len = 0;
	}
	rcu_read_unlock();

	return true;
}
bool Sstar_internal_cmd_scan_triger(struct ieee80211_sub_if_data *sdata,struct ieee80211_internal_scan_request *req)
{
	struct cfg80211_scan_request *scan_req = NULL;
	struct ieee80211_local *local  = sdata->local;
	u8 n_channels = 0;
	int i;
	struct wiphy *wiphy = local->hw.wiphy;
	u8 index;
	void *pos;
	void *pos_end;
	long status = 20*HZ;
	
	ASSERT_RTNL();
	ieee80211_scan_cancel(local);
	flush_workqueue(local->workqueue);
	
	mutex_lock(&local->mtx);

	if(!ieee80211_sdata_running(sdata)){
		Sstar_printk_scan("%s:%d\n",__func__,__LINE__);
		goto err;
	}
	
	if (local->scan_req)
	{
		Sstar_printk_scan("%s:%d\n",__func__,__LINE__);
		goto err;
	}

	if (!list_empty(&local->roc_list))
	{
		goto err;
	}

	if (!list_empty(&local->work_list)) {
		
		Sstar_printk_scan("%s(%s):work_list is not empty,pend scan\n",__func__,sdata->name);
		goto err;
	}
	
	if(Sstar_ieee80211_suspend(sdata->local)==true){
		
		Sstar_printk_err("ieee80211_scan drop:suspend\n");
		goto err;
	}
	
	if(req->n_channels == 0){
		for (i = 0; i < IEEE80211_NUM_BANDS; i++)
			if (wiphy->bands[i])
				n_channels += wiphy->bands[i]->n_channels;
	}else {
		n_channels = req->n_channels;
	}
	scan_req = Sstar_kzalloc(sizeof(*scan_req)
			+ sizeof(*scan_req->ssids) * req->n_ssids
			+ sizeof(*scan_req->channels) * n_channels
			+ req->ie_len + req->n_channels + sizeof(struct ieee80211_internal_mac)*req->n_macs, GFP_KERNEL);
	
	if(scan_req == NULL){
		Sstar_printk_scan("%s:Sstar_kzalloc scan_req err\n",__func__);
		goto err;
	}
	pos = (void *)&scan_req->channels[n_channels];
	pos_end = (void*)((u8*)pos+sizeof(*scan_req->ssids) * req->n_ssids+
			  req->ie_len + req->n_channels + sizeof(struct ieee80211_internal_mac)*req->n_macs);
	Sstar_printk_err("%s:scan_req(%p),pos(%p),pos_end(%p)\n",__func__,scan_req,pos,pos_end);
	/*
	*set channel
	*/
	if(req->n_channels){
		int freq;
		for (i = 0;i<req->n_channels;i++){
			
			if(req->channels[i] <= 14){
				freq = 2412+(req->channels[i] - 1)*5;
			}else {
				freq = 5000 + (5*req->channels[i]);
			}

			Sstar_printk_debug("%s:channel(%d),freq(%d)\n",__func__,req->channels[i],freq);

			scan_req->channels[i] = ieee80211_get_channel(wiphy,freq);

			if(scan_req->channels[i] == NULL){
				Sstar_printk_always("%s:ieee80211_get_channel err\n",__func__);
				goto err;
			}
		}
	}else {
		enum ieee80211_band band;
		i = 0;
		/* all channels */
		for (band = 0; band < IEEE80211_NUM_BANDS; band++) {
			int j;
			if (!wiphy->bands[band])
				continue;
			for (j = 0; j < wiphy->bands[band]->n_channels; j++) {
				scan_req->channels[i] =  &wiphy->bands[band]->channels[j];
				i++;
			}
		}
	}
	scan_req->n_channels = n_channels;
	/*
	*set ssid
	*/
	if( req->n_ssids){
		scan_req->ssids = (void *)pos;
		for(i=0;i<req->n_ssids;i++){			
			Sstar_printk_debug("%s:scan ssid(%s)(%d)\n",__func__,req->ssids[i].ssid,req->ssids[i].ssid_len);
			scan_req->ssids[i].ssid_len = req->ssids[i].ssid_len;
			memcpy(scan_req->ssids[i].ssid,req->ssids[i].ssid,req->ssids[i].ssid_len);
		}
		pos = scan_req->ssids+req->n_ssids;
	}
	scan_req->n_ssids = req->n_ssids;
	/*
	*set macs
	*/
	local->internal_scan.req.n_macs = req->n_macs;	
	if(req->n_macs){
		local->internal_scan.req.macs = pos;
		memcpy(local->internal_scan.req.macs, req->macs,sizeof(struct ieee80211_internal_mac)*req->n_macs);
		pos = local->internal_scan.req.macs + req->n_macs;
	}
	/*
	*set ie
	*/
	if (req->ie_len) {		
		scan_req->ie = (void *)pos;
		memcpy((void*)scan_req->ie,req->ies,req->ie_len);
		scan_req->ie_len = req->ie_len;
		pos = (u8*)scan_req->ie+req->ie_len;
	}

	/*
	*set channel
	*/
	if(req->channels){
		local->internal_scan.req.channels = pos;
		memcpy(local->internal_scan.req.channels,req->channels,req->n_channels);
	    pos = local->internal_scan.req.channels+req->n_channels;
	}
	Sstar_printk_err("%s:pos(%p),pos_end(%p)\n",__func__,pos,pos_end);
	WARN_ON(pos != pos_end);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 1, 0))
	for (i = 0; i < IEEE80211_NUM_BANDS; i++)
		if (wiphy->bands[i])
			scan_req->rates[i] =
				(1 << wiphy->bands[i]->n_bitrates) - 1;
		
	scan_req->no_cck = req->no_cck;
#endif
	
	scan_req->wiphy = wiphy;

	local->internal_scan.req.n_channels = req->n_channels;	
	local->internal_scan.req.ies = (u8*)scan_req->ie;
	local->internal_scan.req.ie_len = scan_req->ie_len;
	local->internal_scan.req.ssids = scan_req->ssids;
	local->internal_scan.req.n_ssids = scan_req->n_ssids;

	local->internal_scan.req.req_flags = req->req_flags;
	
	rcu_assign_pointer(local->internal_scan.req.result_handle,req->result_handle);
	rcu_assign_pointer(local->internal_scan.req.priv,req->priv);

	Sstar_common_hash_list_init(local->internal_scan.mac_hash_list,IEEE80211_INTERNAL_SCAN_HASHENTRIES);
	
	for(index = 0;index<local->internal_scan.req.n_macs;index++){
		int hash_index = Sstar_hash_index(local->internal_scan.req.macs[index].mac,6,IEEE80211_INTERNAL_SCAN_HASHBITS);
		struct hlist_head *hlist = &local->internal_scan.mac_hash_list[hash_index];
		hlist_add_head(&local->internal_scan.req.macs[index].hnode,hlist);
	}
	
	Sstar_internal_cmd_scan_dump(&local->internal_scan.req);
	
	if(ieee80211_internal_scan_triger(sdata,scan_req) == false){
		Sstar_printk_scan("%s scan triger err\n",__func__);
		
		for(index = 0;index<local->internal_scan.req.n_macs;index++){
			hlist_del(&local->internal_scan.req.macs[index].hnode);
		}
		rcu_assign_pointer(local->internal_scan.req.result_handle,NULL);
		rcu_assign_pointer(local->internal_scan.req.priv,NULL);
		memset(&local->internal_scan.req,0,sizeof(struct ieee80211_internal_scan_sta));
		goto err;
	}
	mutex_unlock(&local->mtx);

	status = wait_event_timeout(local->internal_scan_wq,atomic_read(&local->internal_scan_status) != IEEE80211_INTERNAL_SCAN_STATUS__WAIT,status);

	if(status == 0){
		Sstar_printk_err("%s: internal scan timeout\n",__func__);
		return false;
	}

	Sstar_printk_debug("%s: status(%ld)\n",__func__,status);

	if(atomic_read(&local->internal_scan_status) == IEEE80211_INTERNAL_SCAN_STATUS__ABORT)
		return false;
	
	return true;
err:
	if(scan_req)
		Sstar_kfree(scan_req);
	mutex_unlock(&local->mtx);

	return false;
}

bool Sstar_internal_cmd_stainfo(struct ieee80211_local *local,struct ieee80211_internal_sta_req *sta_req)
{
	struct ieee80211_internal_sta_info stainfo;
	struct sta_info *sta;
	u8 index = 0;
	struct hlist_head *hhead;
	struct hlist_node *node;
	struct ieee80211_internal_mac *mac_node;
	unsigned int hash_index = 0;
	bool (__rcu *sta_handle)(struct ieee80211_internal_sta_info *stainfo,void *priv);	
	struct hlist_head Sstar_sta_mac_hlist[SSTAR_COMMON_HASHENTRIES];

	
	memset(&stainfo,0,sizeof(struct ieee80211_internal_sta_info));	
	
	WARN_ON(sta_req->sta_handle == NULL);
	BUG_ON((sta_req->n_macs != 0)&&(sta_req->macs == NULL));
	
	Sstar_common_hash_list_init(Sstar_sta_mac_hlist,SSTAR_COMMON_HASHENTRIES);

	for(index = 0;index<sta_req->n_macs;index++){
		hash_index = Sstar_hash_index(sta_req->macs[index].mac,
								 6,SSTAR_COMMON_HASHBITS);

		hhead = &Sstar_sta_mac_hlist[hash_index];
		hlist_add_head(&sta_req->macs[index].hnode,&Sstar_sta_mac_hlist[hash_index]);
	}
	
	mutex_lock(&local->sta_mtx);
	sta_handle = rcu_dereference(sta_req->sta_handle);
	list_for_each_entry_rcu(sta, &local->sta_list, list) {
		struct ieee80211_channel_state *chan_state = ieee80211_get_channel_state(local, sta->sdata);

		if(sta->sdata->vif.type != sta_req->type){
			continue;
		}
		
		if(sta->uploaded == false){
			continue;
		}
		
		if(sta->dead == true){
			continue;
		}
		
		if(sta_req->n_macs){
			u8 sta_needed = false;
			
			hash_index = Sstar_hash_index(sta->sta.addr,6,SSTAR_COMMON_HASHBITS);
			hhead = &Sstar_sta_mac_hlist[hash_index];
			hlist_for_each(node,hhead){
				mac_node = hlist_entry(node,struct ieee80211_internal_mac,hnode);
				if (memcmp(mac_node->mac,sta->sta.addr,6) == 0){
					sta_needed = true;
					break;
				}
			}
			
			if(sta_needed == false){
				continue;
			}
		}
		stainfo.sdata = sta->sdata;
		
		if(sta_req->req_flag&IEEE80211_INTERNAL_STA_FLAGS_CHANNEL){
			stainfo.channel = channel_hw_value(chan_state->oper_channel);
			stainfo.channel_type = !!(test_sta_flag(sta,WLAN_STA_40M_CH)&&!test_sta_flag(sta,WLAN_STA_40M_CH_SEND_20M));
		}
		
		if(sta_req->req_flag&IEEE80211_INTERNAL_STA_FLAGS_SIGNAL){
			stainfo.signal = sta->last_signal2;
			stainfo.avg_signal = (s8) -Sstar_ewma_read(&sta->avg_signal2);
		}
		
		if(sta_req->req_flag&IEEE80211_INTERNAL_STA_FLAGS_TXRXBYTE){
			stainfo.rx_bytes = sta->rx_bytes;
			stainfo.tx_bytes = sta->tx_bytes;
		}

		if(sta_req->req_flag&IEEE80211_INTERNAL_STA_FLAGS_TOPRATE){			
			struct Sstar_common *hw_priv = (struct Sstar_common *)local->hw.priv;
			struct Sstar_vif *priv = (struct Sstar_vif *)sta->sdata->vif.drv_priv;
			if(sta->sdata->vif.type == NL80211_IFTYPE_STATION){				
				wsm_read_mib(hw_priv, WSM_MIB_ID_GET_RATE, &stainfo.top_rate, sizeof(unsigned int), priv->if_id);
			}else if(sta->sdata->vif.type == NL80211_IFTYPE_AP){
				struct Sstar_sta_priv *sta_priv = (struct Sstar_sta_priv *)&sta->sta.drv_priv;
				u8 sta_id = (u8)sta_priv->link_id;
				if(sta_id != 0){					
					wsm_write_mib(hw_priv, WSM_MIB_ID_GET_RATE, &sta_id, 1,priv->if_id);
					wsm_read_mib(hw_priv, WSM_MIB_ID_GET_RATE, &stainfo.top_rate, sizeof(unsigned int), priv->if_id);
				}
			}
			stainfo.top_rate = stainfo.top_rate/2;
		}

		if(sta_req->req_flag&IEEE80211_INTERNAL_STA_FLAGS_SSID){
			rcu_read_lock();
			
			stainfo.ssid_len = 0;
			memset(stainfo.ssid,0,IEEE80211_MAX_SSID_LEN);
			
			if(sta->sdata->vif.type == NL80211_IFTYPE_STATION){
				struct cfg80211_bss *cbss = sta->sdata->u.mgd.associated;
				
				if(cbss){
					const char *ssid = NULL;
                    ssid = ieee80211_bss_get_ie(cbss, SSTAR_WLAN_EID_SSID);
                    if(ssid){						
                        memcpy(stainfo.ssid, &ssid[2], ssid[1]);
                        stainfo.ssid_len = ssid[1];
                    }
				}				
			}else if(sta->sdata->vif.type == NL80211_IFTYPE_AP){
				struct ieee80211_bss_conf *bss_conf = &sta->sdata->vif.bss_conf;
				stainfo.ssid_len = bss_conf->ssid_len;
				if(stainfo.ssid_len)
					memcpy(stainfo.ssid,bss_conf->ssid,stainfo.ssid_len);
				
			}else {
				WARN_ON(1);
			}
			rcu_read_unlock();
		}
		memcpy(stainfo.mac,sta->sta.addr,6);
		stainfo.filled = sta_req->req_flag;
		if(sta_handle)
			sta_handle(&stainfo,sta_req->priv);
		
		memset(&stainfo,0,sizeof(struct ieee80211_internal_sta_info));
	}
	mutex_unlock(&local->sta_mtx);

	return true;
}
bool Sstar_internal_cmd_monitor_req(struct ieee80211_sub_if_data *sdata,struct ieee80211_internal_monitor_req *monitor_req)
{
	struct ieee80211_local *local  = sdata->local;
	struct Sstar_vif *priv = (struct Sstar_vif *)sdata->vif.drv_priv;
	bool res = false;
	unsigned int freq;
	struct ieee80211_sub_if_data *other_sdata;
	
	struct ieee80211_channel *chan;
	enum nl80211_iftype old_type = sdata->vif.type;
	
	if(!ieee80211_sdata_running(sdata)){
		Sstar_printk_err("%s: sdata[%s] not running\n",__func__,sdata->name);
		return false;
	}
	
	if(priv->join_status != SSTAR_APOLLO_JOIN_STATUS_PASSIVE){
		Sstar_printk_err("%s: sdata[%s] in other mode,please close it\n",__func__,sdata->name);
		return false;
	}

	list_for_each_entry(other_sdata, &local->interfaces, list){

		if(!ieee80211_sdata_running(other_sdata)){
			continue;
		}

		priv = (struct Sstar_vif *)other_sdata->vif.drv_priv;

		if(priv->join_status != SSTAR_APOLLO_JOIN_STATUS_PASSIVE){
			Sstar_printk_err("%s: other_sdata[%s] in other mode,please close it\n",__func__,other_sdata->name);
			return false;
		}
	}
	if(ieee8011_channel_valid(&local->hw,monitor_req->ch) == false){
		Sstar_printk_err("%s: ch %d err\n",__func__,monitor_req->ch);
		return false;
	}
	
	switch(monitor_req->chtype){
	case NL80211_CHAN_NO_HT:
	case NL80211_CHAN_HT20:
	case NL80211_CHAN_HT40MINUS:
	case NL80211_CHAN_HT40PLUS:
		break;
	default:
		Sstar_printk_err("error, %d\n", monitor_req->chtype);
		return false;
	}

	if(monitor_req->ch <= 14){
		freq = 2412+(monitor_req->ch - 1)*5;
	}else {
		freq = 5000 + (5*monitor_req->ch);
	}
	chan = ieee80211_get_channel(local->hw.wiphy, freq);

	if(chan == NULL){
		Sstar_printk_err("%s: freq(%d) get channel err\n",__func__,freq);
		return false;
	}
	
	local->internal_monitor.req.ch = monitor_req->ch;
	local->internal_monitor.req.chtype = monitor_req->chtype;
	
	rcu_assign_pointer(local->internal_monitor.req.monitor_rx,monitor_req->monitor_rx);
	rcu_assign_pointer(local->internal_monitor.req.priv,monitor_req->priv);
	
	Sstar_printk_debug("%s:[%s] channel %d\n",__func__,sdata->name,local->internal_monitor.req.ch);
	if(ieee80211_if_change_type(sdata, NL80211_IFTYPE_MONITOR)){
		Sstar_printk_err("%s:type change err\n",__func__);
		res  = false;
		goto err;
	}

	if(ieee80211_set_channel(local->hw.wiphy,sdata->dev,chan,monitor_req->chtype)){
		Sstar_printk_err("%s: set channel err\n",__func__);
		goto err;
	}

	return true;
err:
	ieee80211_if_change_type(sdata,old_type);
	rcu_assign_pointer(local->internal_monitor.req.monitor_rx,NULL);
	rcu_assign_pointer(local->internal_monitor.req.priv,NULL);
	local->internal_monitor.req.ch = 0;
	
	return res;
}

bool Sstar_internal_cmd_stop_monitor(struct ieee80211_sub_if_data *sdata)
{
	if(!ieee80211_sdata_running(sdata)){
		Sstar_printk_err("%s: sdata[%s] not running\n",__func__,sdata->name);
		return false;
	}

	if(sdata->vif.type != NL80211_IFTYPE_MONITOR){
		Sstar_printk_err("%s: sdata[%s] not in monitor mode\n",__func__,sdata->name);
		return false;
	}

	ieee80211_if_change_type(sdata,NL80211_IFTYPE_STATION);

	rcu_assign_pointer(sdata->local->internal_monitor.req.monitor_rx,NULL);
	rcu_assign_pointer(sdata->local->internal_monitor.req.priv,NULL);

	synchronize_rcu();
	sdata->local->internal_monitor.req.ch = 0;
	sdata->local->internal_monitor.req.chtype = 0;
	
	return true;
}
bool Sstar_internal_wsm_adaptive(struct Sstar_common *hw_priv,struct ieee80211_internal_wsm_adaptive *adaptive)
{
	char* cmd = NULL;
	int len;
	bool res = true;
	
	cmd = Sstar_kzalloc(SSTAR_WSM_CMD_LEN,GFP_KERNEL);

	if(cmd == NULL){
		Sstar_printk_err("%s:alloc cmd err\n",__func__);
		res = false;
		goto err;
	}
	
	len = snprintf(cmd,SSTAR_WSM_CMD_LEN,SSTAR_WSM_ADAPTIVE"%d",adaptive->enable);

	if(len<=0){
		Sstar_printk_err("%s:snprintf failed(%d). \n",__func__, len);
		res = false;
		goto err;
	}
	if(len+1>SSTAR_WSM_CMD_LEN){
		Sstar_printk_err("%s:len err (%d)\n",__func__,len);
		res = false;
		goto err;
	}
	Sstar_printk_debug("%s:wsm [%s][%d]\n",__func__,cmd,len);
	
	if( wsm_write_mib(hw_priv, WSM_MIB_ID_FW_CMD, cmd, len+1,0) < 0){
		Sstar_printk_err("%s: write mib failed \n",__func__);
		res = false;
	}
	
err:
	if(cmd)
		Sstar_kfree(cmd);
	return res;
}

bool Sstar_internal_wsm_txpwr_dcxo(struct Sstar_common *hw_priv,struct ieee80211_internal_wsm_txpwr_dcxo *txpwr_dcxo)
{
	int len;
	char* cmd = NULL;
	bool res = true;
	
	if(txpwr_dcxo->txpwr_L > 32 || txpwr_dcxo->txpwr_L < -32){
		Sstar_printk_err("error, txpwr_L %d\n", txpwr_dcxo->txpwr_L);
		res = false;
		goto err;
	}
	
	if(txpwr_dcxo->txpwr_M > 32 || txpwr_dcxo->txpwr_M < -32){
		Sstar_printk_err("error, txpwr_M %d\n", txpwr_dcxo->txpwr_M);
		res = false;
		goto err;
	}
	
	if(txpwr_dcxo->txpwr_H > 32 || txpwr_dcxo->txpwr_H < -32){
		Sstar_printk_err("error, txpwr_H %d\n", txpwr_dcxo->txpwr_H);
		res = false;
		goto err;
	}
	
	if(txpwr_dcxo->dcxo > 127 || txpwr_dcxo->dcxo < 0){
		Sstar_printk_err("error, dcxo %d\n", txpwr_dcxo->dcxo);
		res = false;
		goto err;
	}

	cmd = Sstar_kzalloc(SSTAR_WSM_CMD_LEN,GFP_KERNEL);

	if(cmd == NULL){
		Sstar_printk_err("%s:alloc cmd err\n",__func__);
		res = false;
		goto err;
	}

	len = snprintf(cmd, SSTAR_WSM_CMD_LEN,"set_txpwr_and_dcxo,%d,%d,%d,%d ",
		           txpwr_dcxo->txpwr_L,txpwr_dcxo->txpwr_M, txpwr_dcxo->txpwr_H, txpwr_dcxo->dcxo);

	if(len<=0){
		Sstar_printk_err("%s:len err (%d)\n",__func__,len);
		res = false;
		goto err;
	}
	if(len+1>SSTAR_WSM_CMD_LEN){
		Sstar_printk_err("%s:len err (%d)\n",__func__,len);
		res = false;
		goto err;
	}
	Sstar_printk_debug("%s:wsm [%s][%d]\n",__func__,cmd,len);
	if(wsm_write_mib(hw_priv, WSM_MIB_ID_FW_CMD, cmd, len+1, 0) < 0){
		Sstar_printk_err("%s: write mib failed \n",__func__);
		res = false;
	}
err:
	if(cmd)
		Sstar_kfree(cmd);
	return res;
}

bool Sstar_internal_wsm_txpwr(struct Sstar_common *hw_priv,struct ieee80211_internal_wsm_txpwr *txpwr)
{
	int len;
	char* cmd = NULL;
	bool res = true;

	if(txpwr->txpwr_indx != 0 && txpwr->txpwr_indx != 1){
		Sstar_printk_err("error, txpwr_indx %d\n", txpwr->txpwr_indx);
		res = false;
		goto err;
	}

	cmd = Sstar_kzalloc(SSTAR_WSM_CMD_LEN,GFP_KERNEL);

	if(cmd == NULL){
		Sstar_printk_err("%s:alloc cmd err\n",__func__);
		res = false;
		goto err;
	}

	len = snprintf(cmd, SSTAR_WSM_CMD_LEN, SSTAR_WSM_TXPWR"%d",txpwr->txpwr_indx);

	if(len<=0){
		Sstar_printk_err("%s:len err (%d)\n",__func__,len);
		res = false;
		goto err;
	}

	if(len+1>SSTAR_WSM_CMD_LEN){
		Sstar_printk_err("%s:len err (%d)\n",__func__,len);
		res = false;
		goto err;
	}
	Sstar_printk_debug("%s:wsm [%s][%d]\n",__func__,cmd,len);
	if(wsm_write_mib(hw_priv, WSM_MIB_ID_FW_CMD, cmd, len+1, 0) < 0){
		Sstar_printk_err("%s: write mib failed \n",__func__);
		res = false;
	}
err:
	if(cmd)
		Sstar_kfree(cmd);
	return res;
}
bool Sstar_internal_wsm_set_rate(struct Sstar_common *hw_priv,struct ieee80211_internal_rate_req *req)
{
	int len;
	char* cmd = NULL;
	bool res = true;

	cmd = Sstar_kzalloc(SSTAR_WSM_CMD_LEN,GFP_KERNEL);

	if(cmd == NULL){
		Sstar_printk_err("%s:alloc cmd err\n",__func__);
		res = false;
		goto err;
	}

	if(req->flags & IEEE80211_INTERNAL_RATE_FLAGS_CLEAR_TX_RATE){
		len = snprintf(cmd, SSTAR_WSM_CMD_LEN, SSTAR_WSM_FIX_RATE,0);

		if(len<=0){
			Sstar_printk_err("%s:len err (%d)\n",__func__,len);
			res = false;
			goto err;
		}

		if(wsm_write_mib(hw_priv, WSM_MIB_ID_FW_CMD, cmd, len+1, 0) < 0){
			Sstar_printk_err("%s: write mib failed \n",__func__);
			res = false;
			goto err;
		}

		memset(cmd,0,SSTAR_WSM_CMD_LEN);
	}

	if(req->flags & IEEE80211_INTERNAL_RATE_FLAGS_CLEAE_TOP_RATE){
		len = snprintf(cmd, SSTAR_WSM_CMD_LEN, SSTAR_WSM_TOP_RATE,0);

		if(len<=0){
			Sstar_printk_err("%s:len err (%d)\n",__func__,len);
			res = false;
			goto err;
		}

		if(wsm_write_mib(hw_priv, WSM_MIB_ID_FW_CMD, cmd, len+1, 0) < 0){
			Sstar_printk_err("%s: write mib failed \n",__func__);
			res = false;
			goto err;
		}

		memset(cmd,0,SSTAR_WSM_CMD_LEN);
	}

	if(req->flags & IEEE80211_INTERNAL_RATE_FLAGS_CLEAR_MIN_RATE){
		len = snprintf(cmd, SSTAR_WSM_CMD_LEN, SSTAR_WSM_MIN_RATE,0);

		if(len<=0){
			Sstar_printk_err("%s:len err (%d)\n",__func__,len);
			res = false;
			goto err;
		}

		if(wsm_write_mib(hw_priv, WSM_MIB_ID_FW_CMD, cmd, len+1, 0) < 0){
			Sstar_printk_err("%s: write mib failed \n",__func__);
			res = false;
			goto err;
		}

		memset(cmd,0,SSTAR_WSM_CMD_LEN);
	}
	
	if(req->flags & IEEE80211_INTERNAL_RATE_FLAGS_SET_TX_RATE){
		len = snprintf(cmd, SSTAR_WSM_CMD_LEN, SSTAR_WSM_FIX_RATE,req->rate);

		if(len<=0){
			Sstar_printk_err("%s:len err (%d)\n",__func__,len);
			res = false;
			goto err;
		}

		if(wsm_write_mib(hw_priv, WSM_MIB_ID_FW_CMD, cmd, len+1, 0) < 0){
			Sstar_printk_err("%s: write mib failed \n",__func__);
			res = false;
			goto err;
		}

		memset(cmd,0,SSTAR_WSM_CMD_LEN);
	}

	if(req->flags & IEEE80211_INTERNAL_RATE_FLAGS_SET_TOP_RATE){
		len = snprintf(cmd, SSTAR_WSM_CMD_LEN, SSTAR_WSM_TOP_RATE,req->rate);

		if(len<=0){
			Sstar_printk_err("%s:len err (%d)\n",__func__,len);
			res = false;
			goto err;
		}

		if(wsm_write_mib(hw_priv, WSM_MIB_ID_FW_CMD, cmd, len+1, 0) < 0){
			Sstar_printk_err("%s: write mib failed \n",__func__);
			res = false;
			goto err;
		}

		memset(cmd,0,SSTAR_WSM_CMD_LEN);
	}

	if(req->flags & IEEE80211_INTERNAL_RATE_FLAGS_SET_MIN_RATE){
		len = snprintf(cmd, SSTAR_WSM_CMD_LEN, SSTAR_WSM_MIN_RATE,req->rate);

		if(len<=0){
			Sstar_printk_err("%s:len err (%d)\n",__func__,len);
			res = false;
			goto err;
		}

		if(wsm_write_mib(hw_priv, WSM_MIB_ID_FW_CMD, cmd, len+1, 0) < 0){
			Sstar_printk_err("%s: write mib failed \n",__func__);
			res = false;
			goto err;
		}

		memset(cmd,0,SSTAR_WSM_CMD_LEN);
	}
err:
	if(cmd)
		Sstar_kfree(cmd);
	return res;
}

bool Sstar_internal_wsm_set_rate_power(struct Sstar_common *hw_priv,
												   struct ieee80211_internal_rate_power_req *req)
{
	#define MIN_RATE_INDEX	(0)
	#define MAX_RATE_INDEX	(10)
	#define MIN_POWER		(-16)
	#define MAX_POWER		(16)

	bool ret = true;
	char* cmd = NULL;
	int len = 0;
	
	if((req->rate_index < MIN_RATE_INDEX) ||(req->rate_index > MAX_RATE_INDEX)){
		Sstar_printk_err("%s:rate_index(%d) err\n",__func__,req->rate_index);
		ret = false;
		goto exit;
	}

	if((req->power < MIN_POWER) ||(req->power > MAX_POWER)){
		Sstar_printk_err("%s:rate_index(%d) err\n",__func__,req->rate_index);
		ret = false;
		goto exit;
	}
	
	cmd = Sstar_kzalloc(SSTAR_WSM_CMD_LEN,GFP_KERNEL);

	if(cmd == NULL){
		Sstar_printk_err("%s:alloc cmd err\n",__func__);
		ret = false;
		goto exit;
	}

	len = snprintf(cmd, SSTAR_WSM_CMD_LEN, SSTAR_WSM_SET_RATE_POWER,req->rate_index,req->power);

	if(len <= 0){
		Sstar_printk_err("%s:len(%d) err\n",__func__,len);
		ret = false;
		goto exit;
	}

	if(wsm_write_mib(hw_priv, WSM_MIB_ID_FW_CMD, cmd, len+1, 0) < 0){
		Sstar_printk_err("%s: write mib failed \n",__func__);
		ret = false;
		goto exit;
	}
	
exit:
	if(cmd)
		Sstar_kfree(cmd);

	return ret;

	#undef MIN_RATE_INDEX
	#undef MAX_RATE_INDEX
	#undef MIN_POWER
	#undef MAX_POWER
}
bool Sstar_internal_freq_set(struct ieee80211_hw *hw,struct ieee80211_internal_set_freq_req *req)
{
	struct ieee80211_local *local = hw_to_local(hw);
	struct Sstar_common *hw_priv = (struct Sstar_common *)hw->priv;
	struct ieee80211_channel *channel;
	char* cmd = NULL;
	int len;
	bool res = true;
	struct ieee80211_special_freq special_req;
	
	ASSERT_RTNL();

	channel = ieee8011_chnum_to_channel(hw,req->channel_num);

	if(channel == NULL){
		Sstar_printk_err("%s: channel is NULL(%d)\n",__func__,req->channel_num);
		res = false;
		goto out;
	}
	
	if(req->set == false){
		req->freq = channel_center_freq(channel);
	}
	
	if((req->freq < 2300) || (req->freq>2600)){
		Sstar_printk_err("%s: freq err(%zu)\n",__func__,req->freq);
		res = false;
		goto out;
	}
	
	mutex_lock(&local->mtx);
	__ieee80211_recalc_idle(local);
	mutex_unlock(&local->mtx);

	if((local->hw.conf.flags & IEEE80211_CONF_IDLE) == 0){
		Sstar_printk_err("%s: please down ap or sta\n",__func__);
		res = false;
		goto out;
	}

	cmd = Sstar_kzalloc(SSTAR_WSM_CMD_LEN,GFP_KERNEL);

	if(cmd == NULL){
		Sstar_printk_err("%s:alloc cmd err\n",__func__);
		res = false;
		goto out;
	}

	len = snprintf(cmd, SSTAR_WSM_CMD_LEN, SSTAR_WSM_SET_FREQ,req->channel_num,(int)req->freq);

	if(len <= 0){
		Sstar_printk_err("%s:len err (%d)\n",__func__,len);
		res = false;
		goto out;
	}

	if(len+1>SSTAR_WSM_CMD_LEN){
		Sstar_printk_err("%s:len err (%d)\n",__func__,len);
		res = false;
		goto out;
	}
	special_req.channel = channel;
	special_req.freq    = req->freq;
	
	if(channel_center_freq(channel) != req->freq){		
		if(ieee80211_special_freq_update(local,&special_req) == false){
			res = false;
			goto out;
		}
	}else {
		ieee80211_special_freq_clear(local,&special_req);
	}
    Sstar_printk_err("%s:%s:%d\n",__func__,cmd,len);	
	if(wsm_write_mib(hw_priv, WSM_MIB_ID_FW_CMD, cmd, len+1, 0) < 0){
		Sstar_printk_err("%s: write mib failed \n",__func__);
		ieee80211_special_freq_clear(local,&special_req);
		res = false;
		goto out;
	}
out:
	if(cmd)
		Sstar_kfree(cmd);

	return res;
}
bool Sstar_internal_channel_auto_select(struct ieee80211_sub_if_data *sdata,
													  struct ieee80211_internal_channel_auto_select_req *req)
{
	struct ieee80211_internal_scan_request scan_req;
	
	scan_req.req_flags = IEEE80211_INTERNAL_SCAN_FLAGS__CCA;
	/*
	*all off supported channel will be scanned
	*/
	scan_req.channels   = NULL;
	scan_req.n_channels = 0;
	scan_req.macs       = NULL;
	scan_req.n_macs     = 0;
	scan_req.ies		= NULL;
	scan_req.ie_len		= 0;
	scan_req.no_cck     = false;
	scan_req.priv		= NULL;
	scan_req.result_handle = NULL;
	scan_req.ssids      = NULL;
	scan_req.n_ssids    = 0;

	return Sstar_internal_cmd_scan_triger(sdata,&scan_req);
}

static bool Sstar_internal_channel_auto_select_results_handle(struct ieee80211_hw *hw,struct Sstar_internal_scan_results_req *req,struct ieee80211_internal_scan_sta *sta_info)
{
	struct ieee80211_internal_channel_auto_select_results *cca_results = (struct ieee80211_internal_channel_auto_select_results *)req->priv;
	s8 signal = (s8)sta_info->signal;
	u8 cur_channel = sta_info->channel;
	u8 index = 0;
	struct ieee80211_channel *channel;
	
	if(ieee8011_channel_valid(hw,cur_channel) == false){
		Sstar_printk_err("%s:channel(%d) err\n",__func__,cur_channel);
		return false;
	}

	if(sta_info->cca == false){
		Sstar_printk_err("%s:not in cca state\n",__func__);
		return false;
	}
	
	req->n_stas ++;
	cca_results->n_aps[cur_channel-1]++;
	
	if(cca_results->version == 1)
		cca_results->weight[cur_channel-1] += ieee80211_rssi_weight(signal);
	else 
		cca_results->weight[cur_channel-1]++;
	
	channel = ieee8011_chnum_to_channel(hw,cur_channel);

	if(channel_in_special(channel) == true){
		Sstar_printk_debug("%s:channel [%d] is in special freq\n",__func__,channel_hw_value(channel));
		return true;
	}
	/*
	*2.4G channel
	*/
	Sstar_printk_debug("ssid[%s],channel[%d],signal(%d)\n",sta_info->ssid,cur_channel,signal);
	/*
	*channel 1-13
	*weight[x] +=  val[x] + val[x-1] + val[x-2] + val[x-3] + val[x+1] + val[x+2] + val[x+3]
	*/
	if(cur_channel<=13){
		u8 low;
		u8 high;

		low = cur_channel>=4?cur_channel-3:1;
		high = cur_channel<= 10 ? cur_channel+3:13;
		
		for(index=cur_channel+1;index<=high;index++){
			channel = ieee8011_chnum_to_channel(hw,index);
			/*
			*skip special freq
			*/
			if(channel_in_special(channel) == true){
				Sstar_printk_debug("%s:skip special freq(%d)\n",__func__,channel_hw_value(channel));
				continue;
			}
			
			if(cca_results->version == 1)
				cca_results->weight[index-1] += ieee80211_rssi_weight(signal - 2*(index-cur_channel));
			else 
				cca_results->weight[index-1] ++;
		}

		for(index=cur_channel-1;index>=low;index--){
			channel = ieee8011_chnum_to_channel(hw,index);
			/*
			*skip special freq
			*/
			if(channel_in_special(channel) == true){
				Sstar_printk_debug("%s:skip special freq(%d)\n",__func__,channel_hw_value(channel));
				continue;
			}
			if(cca_results->version == 1)
				cca_results->weight[index-1] += ieee80211_rssi_weight(signal - 2*(cur_channel-index));
			else 
				cca_results->weight[index-1] ++;
		}
	}
	/*
	*channel 14
	*/
	else if(cur_channel == 14){
		
	}
	/*
	*5G channel
	*/
	else {
		
	}

	for(index = 0;index<IEEE80211_SSTAR_MAX_SCAN_CHANNEL_INDEX;index++){
		Sstar_printk_debug("weight[%d]=[%d]\n",index,cca_results->weight[index]);
	}
	return true;
}
bool Sstar_internal_channel_auto_select_results(struct ieee80211_sub_if_data *sdata,
												struct ieee80211_internal_channel_auto_select_results *results)
{
	#define SSTAR_BUSY_RATIO_MIN		100
	struct Sstar_internal_scan_results_req results_req;
	struct ieee80211_local *local = sdata->local;
	u8 *busy_ratio;
	u8 i;
	u32 min_ap_num = (u32)(-1);
	u8  min_busy_ratio = 128;
	u8  min_ap_num_ration = 128;
	u8 channel = 0;
	int band;
	u32 ignore_flags = IEEE80211_CHAN_DISABLED;
	struct ieee80211_supported_band *sband;
	u8 ignor_channel_mask[IEEE80211_SSTAR_MAX_SCAN_CHANNEL_INDEX];
	u8 channel_mask[IEEE80211_SSTAR_MAX_SCAN_CHANNEL_INDEX];

	results_req.n_stas = 0;
	results_req.flush = true;
	results_req.priv = results;
	results_req.result_handle = Sstar_internal_channel_auto_select_results_handle;
	busy_ratio = ieee80211_scan_cca_val_get(&local->hw);
	
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0))
	ignore_flags |= IEEE80211_CHAN_NO_OFDM;
#endif
	if(ieee80211_scan_internal_req_results(local,&results_req) == false){
		Sstar_printk_err("%s:get results\n",__func__);
		goto err;
	}
	
	for(i = 0;i<14;i++){
		Sstar_printk_debug("busy_ratio[%d]=[%d]\n",i,busy_ratio[i]);
	}
	
	memset(ignor_channel_mask,0,IEEE80211_SSTAR_MAX_SCAN_CHANNEL_INDEX);
	memset(channel_mask,1,IEEE80211_SSTAR_MAX_SCAN_CHANNEL_INDEX);

	for(i= 0;i<results->ignore_n_channels;i++){
		
		BUG_ON(results->ignore_channels == NULL);
		
		if(ieee8011_channel_valid(&local->hw,results->ignore_channels[i]) == false){
			Sstar_printk_err("%s:ignore channel %d err\n",__func__,results->ignore_channels[i]);
			goto err;
		}
		ignor_channel_mask[results->ignore_channels[i]-1] = 1;
		
		Sstar_printk_debug("%s channel %d ignored\n",__func__,results->ignore_channels[i]);
	}

	if(results->n_channels){
		memset(channel_mask,0,IEEE80211_SSTAR_MAX_SCAN_CHANNEL_INDEX);
		for(i = 0;i<results->n_channels;i++){
			BUG_ON(results->channels == NULL);
			if(ieee8011_channel_valid(&local->hw,results->channels[i]) == false){
				Sstar_printk_err("%s:ignore channel %d err\n",__func__,results->channels[i]);
				goto err;
			}

			channel_mask[results->channels[i]-1] = 1;
		}
	}
	for (band = 0; band < IEEE80211_NUM_BANDS; band++) {
		
		sband = local->hw.wiphy->bands[band];
		
		if (!sband)
			continue;
		/*
		*2.4G channel and 5G
		*/
		for(i = 0;i<sband->n_channels;i++){
			/*
			*0 means that the channel do not process cca
			*/
			if(busy_ratio[channel_hw_value(&sband->channels[i])-1] == 0){
				continue;
			}
			
			if(ignor_channel_mask[channel_hw_value(&sband->channels[i])-1] == 1){
				continue;
			}

			if(channel_mask[channel_hw_value(&sband->channels[i])-1] == 0){
				continue;
			}
			/*
			*special freq must be skiped
			*/
			if(channel_in_special(&sband->channels[i])){
				Sstar_printk_debug("%s:skip special freq(%d)\n",__func__,channel_hw_value(&sband->channels[i]));
				continue;
			}
			/*
			*some disabled channel must be skiped
			*/
			if(ignore_flags&sband->channels[i].flags){
				Sstar_printk_debug("%s: channel[%d] not support ofdm\n",__func__,channel_hw_value(&sband->channels[i]));
				continue;
			}
			
			if(busy_ratio[channel_hw_value(&sband->channels[i])-1]<SSTAR_BUSY_RATIO_MIN){

				if(results->weight[channel_hw_value(&sband->channels[i])-1]<=min_ap_num){
					if(results->weight[channel_hw_value(&sband->channels[i])-1]==min_ap_num){
						if(busy_ratio[channel_hw_value(&sband->channels[i])-1]<=min_ap_num_ration){
							min_ap_num = results->weight[channel_hw_value(&sband->channels[i])-1];
							channel = channel_hw_value(&sband->channels[i]);
							min_ap_num_ration = busy_ratio[channel_hw_value(&sband->channels[i])-1];
						}
					}else {
						min_ap_num = results->weight[channel_hw_value(&sband->channels[i])-1];
						channel = channel_hw_value(&sband->channels[i]);
						min_ap_num_ration = busy_ratio[channel_hw_value(&sband->channels[i])-1];
					}
				}
				
			}else if(min_ap_num == (u32)(-1)){
				if(busy_ratio[channel_hw_value(&sband->channels[i])-1]<min_busy_ratio){
					min_busy_ratio = busy_ratio[channel_hw_value(&sband->channels[i])-1];
					channel = channel_hw_value(&sband->channels[i]);
				}
			}
		}			
	}

	if(channel == 0){
		WARN_ON(channel == 0);
		goto err;
	}
	Sstar_printk_always("%s channel %d\n",__func__,channel);
	memcpy(results->busy_ratio,busy_ratio,IEEE80211_SSTAR_MAX_SCAN_CHANNEL_INDEX);
	results->susgest_channel = channel;
	ieee80211_scan_cca_val_put(&local->hw);
	return true;
err:
	ieee80211_scan_cca_val_put(&local->hw);
	return false;
}

bool Sstar_internal_request_chip_cap(struct ieee80211_hw *hw,struct ieee80211_internal_req_chip *req)
{
	struct Sstar_common *hw_priv = (struct Sstar_common *)hw->priv;

	if(req->flags & IEEE80211_INTERNAL_REQ_CHIP_FLAGS__CHIP_VER){
		if(hw_priv->wsm_caps.firmwareCap &CAPABILITIES_EFUSE8){
			req->chip_version = chip_6038;
		}else if(hw_priv->wsm_caps.firmwareCap &CAPABILITIES_EFUSEI){
			req->chip_version = chip_6032i;
		}else if(hw_priv->wsm_caps.firmwareCap &CAPABILITIES_EFUSEB){
			req->chip_version = chip_101B;
		}else {
			req->chip_version = chip_6032i;
		}
	}

	/*other code */

	return true;
}

bool Sstar_internal_update_ap_conf(struct ieee80211_sub_if_data *sdata,
									     struct ieee80211_internal_ap_conf *conf_req,bool clear)
{
	
	if(!ieee80211_sdata_running(sdata)){
		Sstar_printk_scan("%s:%d\n",__func__,__LINE__);
		goto err;
	}

	if(conf_req&&conf_req->channel){
		if(ieee8011_channel_valid(&sdata->local->hw,(int)conf_req->channel) == false){
			goto err;
		}
	}

	return ieee80211_update_ap_config(sdata,conf_req,clear);
err:
	return false;
}

