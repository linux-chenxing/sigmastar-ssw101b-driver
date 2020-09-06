/*
 * Scan implementation for sigmastar APOLLO mac80211 drivers
 *
 * Copyright (c) 2016, sigmastar
 *
 * Based on:
 * Copyright (c) 2010, ST-Ericsson
 * Author: Dmitry Tarnyagin <dmitry.tarnyagin@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/sched.h>
#include "apollo.h"
#include "scan.h"
#include "sta.h"
#include "pm.h"
#include "bh.h"
#include "Sstar_p2p.h"
#ifdef SSTAR_SUPPORT_SMARTCONFIG
extern int smartconfig_magic_scan_done(struct Sstar_common *hw_priv);
#endif
static void Sstar_scan_restart_delayed(struct Sstar_vif *priv);

//#ifdef CONFIG_WIRELESS_EXT
extern void etf_v2_scan_end(struct Sstar_common *hw_priv, struct ieee80211_vif *vif );
extern void etf_v2_scan_rx(struct Sstar_common *hw_priv,struct sk_buff *skb,u8 rssi );
//#endif
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
static int Sstar_advance_scan_start(struct Sstar_common *hw_priv)
{
	int tmo = 0;
	tmo += hw_priv->advanceScanElems.duration;
	#ifdef CONFIG_PM
	Sstar_pm_stay_awake(&hw_priv->pm_state, tmo * HZ / 1000);
	#endif
	/* Invoke Advance Scan Duration Timeout Handler */
	Sstar_hw_priv_queue_delayed_work(hw_priv,
		&hw_priv->advance_scan_timeout, tmo * HZ / 1000);
	return 0;
}
#endif

static void Sstar_remove_wps_p2p_ie(struct wsm_template_frame *frame)
{
	u8 *ies;
	int ies_len;
	int ie_len;
	u32 p2p_ie_len = 0;
	u32 wps_ie_len = 0;

	ies = &frame->skb->data[sizeof(struct ieee80211_hdr_3addr)];
	ies_len = frame->skb->len - sizeof(struct ieee80211_hdr_3addr);

	while (ies_len >= 6) {
		ie_len = ies[1] + 2;
		if ((ies[0] == SSTAR_WLAN_EID_VENDOR_SPECIFIC)
			&& (ies[2] == 0x00 && ies[3] == 0x50 && ies[4] == 0xf2 && ies[5] == 0x04)) {
			wps_ie_len = ie_len;
			memmove(ies, ies + ie_len, ies_len);
			ies_len -= ie_len;

		}
		else if ((ies[0] == SSTAR_WLAN_EID_VENDOR_SPECIFIC) &&
			(ies[2] == 0x50 && ies[3] == 0x6f && ies[4] == 0x9a && ies[5] == 0x09)) {
			p2p_ie_len = ie_len;
			memmove(ies, ies + ie_len, ies_len);
			ies_len -= ie_len;
		} else {
			ies += ie_len;
			ies_len -= ie_len;
		}
	}

	if (p2p_ie_len || wps_ie_len) {
		Sstar_skb_trim(frame->skb, frame->skb->len - (p2p_ie_len + wps_ie_len));
	}
}
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
static int Sstar_disable_filtering(struct Sstar_vif *priv)
{
	int ret = 0;
	bool bssid_filtering = 0;
	struct wsm_rx_filter rx_filter;
	struct wsm_beacon_filter_control bf_control;
	struct Sstar_common *hw_priv = ABwifi_vifpriv_to_hwpriv(priv);

	/* RX Filter Disable */
	rx_filter.promiscuous = 0;
	rx_filter.bssid = 0;
	rx_filter.fcs = 0;
	rx_filter.probeResponder = 0;
	rx_filter.keepalive = 0;
	ret = wsm_set_rx_filter(hw_priv, &rx_filter,
			priv->if_id);

	/* Beacon Filter Disable */
	bf_control.enabled = 0;
	bf_control.bcn_count = 1;
	if (!ret)
		ret = wsm_beacon_filter_control(hw_priv, &bf_control,
					priv->if_id);

	/* BSSID Filter Disable */
	if (!ret)
		ret = wsm_set_bssid_filtering(hw_priv, bssid_filtering,
					 priv->if_id);

	return ret;
}

#endif
static int Sstar_scan_start(struct Sstar_vif *priv, struct wsm_scan *scan)
{
	int ret, i;
	int tmo = 5000;
	struct Sstar_common *hw_priv = ABwifi_vifpriv_to_hwpriv(priv);

	for (i = 0; i < scan->numOfChannels; ++i)
		tmo += scan->ch[i].maxChannelTime + 10;

	atomic_set(&hw_priv->scan.in_progress, 1);
	atomic_set(&hw_priv->recent_scan, 1);
	#ifndef CONFIG_WAKELOCK
	#ifdef CONFIG_PM
	Sstar_pm_stay_awake(&hw_priv->pm_state, tmo * HZ / 1000);
	#endif
	#endif
	Sstar_hw_priv_queue_delayed_work(hw_priv, &hw_priv->scan.timeout,
		tmo * HZ / 1000);
	hw_priv->scan.wait_complete = 1;
#ifdef P2P_MULTIVIF
	ret = wsm_scan(hw_priv, scan, priv->if_id ? 1 : 0);
#else
	ret = wsm_scan(hw_priv, scan, priv->if_id);
#endif
	if (unlikely(ret)) {
		hw_priv->scan.wait_complete = 0;
		atomic_set(&hw_priv->scan.in_progress, 0);
		Sstar_cancle_delayed_work(&hw_priv->scan.timeout,true);
//		Sstar_scan_restart_delayed(priv);
	}
	return ret;
}

#ifdef ROAM_OFFLOAD
static int Sstar_sched_scan_start(struct Sstar_vif *priv, struct wsm_scan *scan)
{
	int ret;
	struct Sstar_common *hw_priv = ABwifi_vifpriv_to_hwpriv(priv);

	ret = wsm_scan(hw_priv, scan, priv->if_id);
	if (unlikely(ret)) {
		atomic_set(&hw_priv->scan.in_progress, 0);
	}
	return ret;
}
#endif /*ROAM_OFFLOAD*/

int Sstar_hw_scan(struct ieee80211_hw *hw,
		   struct ieee80211_vif *vif,
		   struct ieee80211_scan_req_wrap *req_wrap)
{
	struct Sstar_common *hw_priv = hw->priv;
	struct Sstar_vif *priv = ABwifi_get_vif_from_ieee80211(vif);
	struct wsm_template_frame frame = {
		.frame_type = WSM_FRAME_TYPE_PROBE_REQUEST,
	};
	int i;
	int ret = 0;
	int roc_if_id = 0;
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
	u16 advance_scan_req_channel;
#endif

	if(Sstar_bh_is_term(hw_priv)){
		return -EOPNOTSUPP;
	}

	if(atomic_read(&priv->enabled) == 0){
		Sstar_printk_err("%s:priv is not enable\n",__func__);
		return -EOPNOTSUPP;
	}
	/* Scan when P2P_GO corrupt firmware MiniAP mode */
	if (priv->join_status == SSTAR_APOLLO_JOIN_STATUS_AP)
	{
		return -EOPNOTSUPP;
	}
	mutex_lock(&hw_priv->conf_mutex);
	roc_if_id = hw_priv->roc_if_id;
	mutex_unlock(&hw_priv->conf_mutex);
	if (work_pending(&priv->offchannel_work) ||
			(roc_if_id != -1)) {
		Sstar_printk_err( "[SCAN] Offchannel work pending,ignoring scan work %d\n",hw_priv->roc_if_id);
		return -EBUSY;
	}
	if (req_wrap->req->n_ssids == 1 && !req_wrap->req->ssids[0].ssid_len)
		req_wrap->req->n_ssids = 0;

	wiphy_dbg(hw->wiphy, "[SCAN] Scan request for %d SSIDs.\n",
		req_wrap->req->n_ssids);

	if (req_wrap->req->n_ssids > hw->wiphy->max_scan_ssids)
	{
		Sstar_printk_err("%s:req->n_ssids > hw->wiphy->max_scan_ssids\n",__func__);
		return -EINVAL;
	}

	frame.skb = ieee80211_probereq_get(hw, vif, NULL, 0,
		req_wrap->req->ie, req_wrap->req->ie_len);
	if (!frame.skb)
		return -ENOMEM;
	
#ifdef SSTAR_P2P_CHANGE
	Sstar_parase_p2p_mgmt_frame(priv,frame.skb,true);
#endif
#ifdef ROAM_OFFLOAD
	if (priv->join_status != SSTAR_APOLLO_JOIN_STATUS_STA) {
		if (req_wrap->req->channels[0]->band == NL80211_BAND_2GHZ)
			hw_priv->num_scanchannels = 0;
		else
			hw_priv->num_scanchannels = hw_priv->num_2g_channels;
		
		for (i=0; i < req_wrap->req->n_channels; i++) {
			hw_priv->scan_channels[hw_priv->num_scanchannels + i].number = \
				channel_hw_value(req_wrap->req->channels[i]);
			#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0))
			if (req_wrap->req->channels[i]->flags & IEEE80211_CHAN_PASSIVE_SCAN) 
			#else
			if (req_wrap->req->channels[i]->flags &IEEE80211_CHAN_NO_IR)
			#endif
			{
				hw_priv->scan_channels[hw_priv->num_scanchannels + i].minChannelTime = 50;
				hw_priv->scan_channels[hw_priv->num_scanchannels + i].maxChannelTime = 110;
			}
			else {
				hw_priv->scan_channels[hw_priv->num_scanchannels + i].minChannelTime = 10;
				hw_priv->scan_channels[hw_priv->num_scanchannels + i].maxChannelTime = 40;
				hw_priv->scan_channels[hw_priv->num_scanchannels + i].number |= \
					SSTAR_APOLLO_SCAN_TYPE_ACTIVE;
			}
			hw_priv->scan_channels[hw_priv->num_scanchannels + i].txPowerLevel = \
				req_wrap->req->channels[i]->max_power;
			if (req_wrap->req->channels[0]->band == NL80211_BAND_5GHZ)
				hw_priv->scan_channels[hw_priv->num_scanchannels + i].number |= \
					SSTAR_APOLLO_SCAN_BAND_5G;
		}
		if (req_wrap->req->channels[0]->band == NL80211_BAND_2GHZ)
			hw_priv->num_2g_channels = req_wrap->req->n_channels;
		else
			hw_priv->num_5g_channels = req_wrap->req->n_channels;
	}
	hw_priv->num_scanchannels = hw_priv->num_2g_channels + hw_priv->num_5g_channels;
#endif /*ROAM_OFFLOAD*/

	/*
	*supplicant requres scan,we must stay awake.
	*/
	Sstar_hold_suspend(hw_priv);
	/* will be unlocked in Sstar_scan_work() */
	down(&hw_priv->scan.lock);
	mutex_lock(&hw_priv->conf_mutex);
	Sstar_printk_scan("%s:if_id(%d)\n",__func__,priv->if_id);
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
	/* Active Scan - Serving Channel Request Handling */
	advance_scan_req_channel = channel_hw_value(req_wrap->req->channels[0]);
	if (hw_priv->enable_advance_scan &&
		(hw_priv->advanceScanElems.scanMode ==
			SSTAR_APOLLO_SCAN_MEASUREMENT_ACTIVE) &&
		(priv->join_status == SSTAR_APOLLO_JOIN_STATUS_STA) &&
		(channel_hw_value(hw_priv->channel) == advance_scan_req_channel)) {
		BUG_ON(hw_priv->scan.req);
		/* wsm_lock_tx(hw_priv); */
		wsm_vif_lock_tx(priv);
		hw_priv->scan.if_id = priv->if_id;
		/* Disable Power Save */
		if (priv->powersave_mode.pmMode & WSM_PSM_PS) {
			struct wsm_set_pm pm = priv->powersave_mode;
			pm.pmMode = WSM_PSM_ACTIVE;
			wsm_set_pm(hw_priv, &pm, priv->if_id);
		}
		/* Disable Rx Beacon and Bssid filter */
		ret = Sstar_disable_filtering(priv);
		if (ret)
			wiphy_err(priv->hw->wiphy,
			"%s: Disable BSSID or Beacon filtering failed: %d.\n",
			__func__, ret);
		wsm_unlock_tx(hw_priv);
		mutex_unlock(&hw_priv->conf_mutex);
		/* Transmit Probe Request with Broadcast SSID */
		Sstar_tx(hw, frame.skb);
		/* Start Advance Scan Timer */
		Sstar_advance_scan_start(hw_priv);
	} else {
#endif
		if (frame.skb) {
			if (priv->if_id == 0)
				Sstar_remove_wps_p2p_ie(&frame);
#ifdef P2P_MULTIVIF

#ifdef SSTAR_SUPPORT_WIDTH_40M
//#ifdef P2P_MULTIVIF
			if(priv->if_id&&(priv->vif->p2p==true))
			{
				struct Sstar_ieee80211_mgmt *mgmt = (struct Sstar_ieee80211_mgmt *)frame.skb->data;
				Sstar_printk_debug( "%s:Sstar_clear_wpas_p2p_40M_ie,if_id(%d)\n",__func__,priv->if_id);
				Sstar_clear_wpas_p2p_40M_ie(mgmt,frame.skb->len);
			}
//#endif
#endif
			ret = wsm_set_template_frame(hw_priv, &frame,
					priv->if_id ? 1 : 0);
#else
			ret = wsm_set_template_frame(hw_priv, &frame,
					priv->if_id);
#endif
			if (ret) {
				mutex_unlock(&hw_priv->conf_mutex);
				up(&hw_priv->scan.lock);
				Sstar_dev_kfree_skb(frame.skb);
				Sstar_release_suspend(hw_priv);
				return ret;
			}
			priv->tmpframe_probereq_set = 1;
		}

//		wsm_vif_lock_tx(priv);
		/*
		*must make sure that there are no pkgs in the lmc.
		*/
		wsm_lock_tx_async(hw_priv);
		wsm_flush_tx(hw_priv);
		
		BUG_ON(hw_priv->scan.req);
		hw_priv->scan.req = req_wrap->req;
		hw_priv->scan.req_wrap = req_wrap;
		hw_priv->scan.n_ssids = 0;
		hw_priv->scan.status = 0;
		hw_priv->scan.begin = &req_wrap->req->channels[0];
		hw_priv->scan.curr = hw_priv->scan.begin;
		hw_priv->scan.end = &req_wrap->req->channels[req_wrap->req->n_channels];
		hw_priv->scan.output_power = hw_priv->output_power;
		hw_priv->scan.if_id = priv->if_id;
		hw_priv->scan.passive = !!(req_wrap->flags & IEEE80211_SCAN_REQ_PASSIVE_SCAN);
		hw_priv->scan.cca = !!(req_wrap->flags & IEEE80211_SCAN_REQ_CCA);
		#ifdef SSTAR_P2P_CHANGE
		/*
		*when p2p scan for p2p go , must make sure that we have 
		*receive the p2p go beacon or probe resp.
		*/
		if((priv->if_id==1)&&(atomic_read(&hw_priv->go_bssid_set))){
			hw_priv->p2p_scan_start_time = jiffies;
			atomic_set(&hw_priv->receive_go_resp,0);
		}
		#endif
		/* TODO:COMBO: Populate BIT4 in scanflags to decide on which MAC
		 * address the SCAN request will be sent */

		for (i = 0; i < req_wrap->req->n_ssids; ++i) {
			struct wsm_ssid *dst =
				&hw_priv->scan.ssids[hw_priv->scan.n_ssids];
			BUG_ON(req_wrap->req->ssids[i].ssid_len > sizeof(dst->ssid));
			memcpy(&dst->ssid[0], req_wrap->req->ssids[i].ssid,
				sizeof(dst->ssid));
			dst->length = req_wrap->req->ssids[i].ssid_len;
			++hw_priv->scan.n_ssids;
		}

		mutex_unlock(&hw_priv->conf_mutex);

		if (frame.skb)
			Sstar_dev_kfree_skb(frame.skb);
		Sstar_printk_scan("%s:scan, delay suspend\n",__func__);
		Sstar_hw_priv_queue_work(hw_priv, &hw_priv->scan.work);

#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
	}
#endif
	return 0;
}

#ifdef ROAM_OFFLOAD
int Sstar_hw_sched_scan_start(struct ieee80211_hw *hw,
		   struct ieee80211_vif *vif,
		   struct cfg80211_sched_scan_request *req,
		   struct ieee80211_sched_scan_ies *ies)
{
	struct Sstar_common *hw_priv = hw->priv;
	struct Sstar_vif *priv = ABwifi_get_vif_from_ieee80211(vif);
	struct wsm_template_frame frame = {
		.frame_type = WSM_FRAME_TYPE_PROBE_REQUEST,
	};
	int i;

	wiphy_warn(hw->wiphy, "[SCAN] Scheduled scan request-->.\n");

	if (!priv->vif)
		return -EINVAL;

	/* Scan when P2P_GO corrupt firmware MiniAP mode */
	if (priv->join_status == SSTAR_APOLLO_JOIN_STATUS_AP)
		return -EOPNOTSUPP;

	wiphy_warn(hw->wiphy, "[SCAN] Scheduled scan: n_ssids %d, ssid[0].len = %d\n", req->n_ssids, req->ssids[0].ssid_len);
	if (req->n_ssids == 1 && !req->ssids[0].ssid_len)
		req->n_ssids = 0;

	wiphy_dbg(hw->wiphy, "[SCAN] Scan request for %d SSIDs.\n",
		req->n_ssids);

	if (req->n_ssids > hw->wiphy->max_scan_ssids)
		return -EINVAL;

	frame.skb = ieee80211_probereq_get(hw, priv->vif, NULL, 0,
		ies->ie[0], ies->len[0]);
	if (!frame.skb)
		return -ENOMEM;

	/* will be unlocked in Sstar_scan_work() */
	down(&hw_priv->scan.lock);
	mutex_lock(&hw_priv->conf_mutex);
	if (frame.skb) {
		int ret;
		if (priv->if_id == 0)
			Sstar_remove_wps_p2p_ie(&frame);
		ret = wsm_set_template_frame(hw_priv, &frame, priv->if_id);
		if (0 == ret) {
			priv->tmpframe_probereq_set = 1;
			/* Host want to be the probe responder. */
			ret = wsm_set_probe_responder(priv, true);
		}
		if (ret) {
			mutex_unlock(&hw_priv->conf_mutex);
			up(&hw_priv->scan.lock);
			Sstar_dev_kfree_skb(frame.skb);
			return ret;
		}
	}

	wsm_lock_tx(hw_priv);

	BUG_ON(hw_priv->scan.req);
	hw_priv->scan.sched_req = req;
	hw_priv->scan.n_ssids = 0;
	hw_priv->scan.status = 0;
	hw_priv->scan.begin = &req->channels[0];
	hw_priv->scan.curr = hw_priv->scan.begin;
	hw_priv->scan.end = &req->channels[req->n_channels];
	hw_priv->scan.output_power = hw_priv->output_power;

	for (i = 0; i < req->n_ssids; ++i) {
		struct wsm_ssid *dst =
			&hw_priv->scan.ssids[hw_priv->scan.n_ssids];
		BUG_ON(req->ssids[i].ssid_len > sizeof(dst->ssid));
		memcpy(&dst->ssid[0], req->ssids[i].ssid,
			sizeof(dst->ssid));
		dst->length = req->ssids[i].ssid_len;
		++hw_priv->scan.n_ssids;
		{
			u8 j;
			wiphy_warn(hw->wiphy, "[SCAN] SSID %d\n",i);
			for(j=0; j<req->ssids[i].ssid_len; j++)
				wiphy_warn(priv->hw->wiphy, "[SCAN] 0x%x\n", req->ssids[i].ssid[j]);
		}
	}

	mutex_unlock(&hw_priv->conf_mutex);

	if (frame.skb)
		Sstar_dev_kfree_skb(frame.skb);
	Sstar_hw_priv_queue_work(hw_priv, &hw_priv->scan.swork);
	wiphy_warn(hw->wiphy, "<--[SCAN] Scheduled scan request.\n");
	return 0;
}
#endif /*ROAM_OFFLOAD*/

void Sstar_scan_work(struct work_struct *work)
{
	struct Sstar_common *hw_priv = container_of(work,
						struct Sstar_common,
						scan.work);
	struct Sstar_vif *priv, *vif;
	struct ieee80211_channel **it;
	struct wsm_scan scan = {
		.scanType = WSM_SCAN_TYPE_FOREGROUND,
		.scanFlags = 0, /* TODO:COMBO */
		//.scanFlags = WSM_SCAN_FLAG_SPLIT_METHOD, /* TODO:COMBO */
	};
	bool first_run;
	int i;
	u32 ProbeRequestTime  = 10;
	u32 ChannelRemainTime = 20;
	u32 maxChannelTime;
	int scan_status = 0;
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
	int ret = 0;
	u16 advance_scan_req_channel = channel_hw_value(hw_priv->scan.begin[0]);
#endif

	
	priv = __ABwifi_hwpriv_to_vifpriv(hw_priv, hw_priv->scan.if_id);

	/*TODO: COMBO: introduce locking so vif is not removed in meanwhile */

    if (!priv) {
		Sstar_printk_err("%s:[SCAN] interface removed\n",__func__);
		ieee80211_scan_completed(hw_priv->hw,
					 hw_priv->scan.status ? 1 : 0);
		Sstar_release_suspend(hw_priv);
        return;
    }

	if (priv->if_id)
		scan.scanFlags |= WSM_FLAG_MAC_INSTANCE_1;
	else
		scan.scanFlags &= ~WSM_FLAG_MAC_INSTANCE_1;

	Sstar_for_each_vif(hw_priv, vif, i) {
		if (!vif)
			continue;
	if (vif->bss_loss_status > SSTAR_APOLLO_BSS_LOSS_NONE)
		scan.scanFlags |= WSM_SCAN_FLAG_FORCE_BACKGROUND;
	}


	first_run = hw_priv->scan.begin == hw_priv->scan.curr &&
			hw_priv->scan.begin != hw_priv->scan.end;

	if (first_run) {
		/* Firmware gets crazy if scan request is sent
		 * when STA is joined but not yet associated.
		 * Force unjoin in this case. */
		if (Sstar_cancle_delayed_work(&priv->join_timeout,true) > 0)
			Sstar_join_timeout(&priv->join_timeout.work);
	}

	mutex_lock(&hw_priv->conf_mutex);

	if (first_run) {
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
		/* Passive Scan - Serving Channel Request Handling */
		if (hw_priv->enable_advance_scan &&
			(hw_priv->advanceScanElems.scanMode ==
				SSTAR_APOLLO_SCAN_MEASUREMENT_PASSIVE) &&
			(priv->join_status == SSTAR_APOLLO_JOIN_STATUS_STA) &&
			(channel_hw_value(hw_priv->channel) ==
				advance_scan_req_channel)) {
			/* If Advance Scan Request is for Serving Channel Device
			 * should be Active and Filtering Should be Disable */
			if (priv->powersave_mode.pmMode & WSM_PSM_PS) {
				struct wsm_set_pm pm = priv->powersave_mode;
				pm.pmMode = WSM_PSM_ACTIVE;
				wsm_set_pm(hw_priv, &pm, priv->if_id);
			}
			/* Disable Rx Beacon and Bssid filter */
			ret = Sstar_disable_filtering(priv);
			if (ret)
				wiphy_err(hw_priv->hw->wiphy,
				"%s: Disable BSSID or Beacon filtering failed: %d.\n",
				__func__, ret);
		} else if (hw_priv->enable_advance_scan &&
			(hw_priv->advanceScanElems.scanMode ==
				SSTAR_APOLLO_SCAN_MEASUREMENT_PASSIVE) &&
			(priv->join_status == SSTAR_APOLLO_JOIN_STATUS_STA)) {
				if (priv->join_status == SSTAR_APOLLO_JOIN_STATUS_STA &&
					!(priv->powersave_mode.pmMode & WSM_PSM_PS)) {
					struct wsm_set_pm pm = priv->powersave_mode;
					pm.pmMode = WSM_PSM_PS;
					Sstar_set_pm(priv, &pm);
				}
		} else {
#endif
			if (priv->join_status == SSTAR_APOLLO_JOIN_STATUS_MONITOR) {
				/* FW bug: driver has to restart p2p-dev mode
				 * after scan */
				Sstar_disable_listening(priv);
			}
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
		}
#endif
	}

	if (!hw_priv->scan.req || (hw_priv->scan.curr == hw_priv->scan.end)) {
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
		if (hw_priv->enable_advance_scan &&
			(hw_priv->advanceScanElems.scanMode ==
				SSTAR_APOLLO_SCAN_MEASUREMENT_PASSIVE) &&
			(priv->join_status == SSTAR_APOLLO_JOIN_STATUS_STA) &&
			(channel_hw_value(hw_priv->channel) ==
				advance_scan_req_channel)) {
			/* WSM Lock should be held here for WSM APIs */
			wsm_vif_lock_tx(priv);
			/* wsm_lock_tx(priv); */
			/* Once Duration is Over, enable filtering
			 * and Revert Back Power Save */
			if (priv->powersave_mode.pmMode & WSM_PSM_PS)
				wsm_set_pm(hw_priv, &priv->powersave_mode,
					priv->if_id);
			Sstar_update_filtering(priv);
		} else {
			if (!hw_priv->enable_advance_scan) {
#endif
				if (hw_priv->scan.output_power != hw_priv->output_power)
				/* TODO:COMBO: Change when mac80211 implementation
				 * is available for output power also */
#ifdef P2P_MULTIVIF
					WARN_ON(wsm_set_output_power(hw_priv,
						hw_priv->output_power * 10,
						priv->if_id ? 1 : 0));
#else
					WARN_ON(wsm_set_output_power(hw_priv,
						hw_priv->output_power * 10,
						priv->if_id));
#endif
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
			}
		}
#endif
		#ifndef SSTAR_P2P_CHANGE
		while(priv->scan_again)
		{
			int status;
			priv->scan_again = 0;
			scan.ch = Sstar_kzalloc(
			sizeof(struct wsm_scan_ch[3]),
			GFP_KERNEL);
			if (!scan.ch) {
				break;
			}
			scan.numOfSSIDs = hw_priv->scan.n_ssids;
			scan.numOfProbeRequests = 3;
			maxChannelTime = (scan.numOfSSIDs * scan.numOfProbeRequests *
			15) + 10;
			maxChannelTime = (maxChannelTime < 35) ? 35 : maxChannelTime;
			scan.ch[0].number = 1;
			scan.ch[0].maxChannelTime = maxChannelTime;
			scan.ch[0].minChannelTime = 35;

			scan.ch[1].number = 6;
			scan.ch[1].maxChannelTime = maxChannelTime;
			scan.ch[1].minChannelTime = 35;

			scan.ch[2].number = 11;
			scan.ch[2].maxChannelTime = maxChannelTime;
			scan.ch[2].minChannelTime = 35;
	
			scan.ssids = &hw_priv->scan.ssids[0];

			scan.probeDelay = 15;

			scan.numOfChannels = 3;
			scan.band = IEEE80211_BAND_2GHZ;


			status = Sstar_scan_start(priv, &scan);
			Sstar_kfree(scan.ch);

			if(status)
				break;

			mutex_unlock(&hw_priv->conf_mutex);

			return;
			
		}
		#else
		while((hw_priv->scan.if_id == 1)&&(atomic_read(&hw_priv->go_bssid_set) == 1)&&
			(atomic_read(&hw_priv->receive_go_resp) == 0))
		{
			int status;
			Sstar_printk_scan("%s:p2p scan again,channel(%d)\n",__func__,atomic_read(&hw_priv->p2p_oper_channel));
			if(atomic_read(&hw_priv->p2p_oper_channel) == 0){
				Sstar_printk_err("%s:p2p_oper_channel is zero\n",__func__);
				break;
			}

			if(!time_is_after_jiffies(hw_priv->p2p_scan_start_time+2*HZ)){
				Sstar_printk_err("%s:p2p has continued scaning a long time(2s),but .....\n",__func__);
				atomic_set(&hw_priv->go_bssid_set,0);
				atomic_set(&hw_priv->p2p_oper_channel,0);
				break;
			}

			scan.ch = Sstar_kzalloc(
			sizeof(struct wsm_scan_ch[5]),
			GFP_KERNEL);
			if (!scan.ch) {
				break;
			}

			scan.numOfSSIDs = hw_priv->scan.n_ssids;
			scan.numOfProbeRequests = 3;
			#if 0
			maxChannelTime = (scan.numOfSSIDs * scan.numOfProbeRequests *
			15) + 10;
			maxChannelTime = (maxChannelTime < 35) ? 35 : maxChannelTime;
			#else
			maxChannelTime = 110;
			#endif

			scan.ch[0].number = atomic_read(&hw_priv->p2p_oper_channel);
			scan.ch[0].maxChannelTime = maxChannelTime;
			scan.ch[0].minChannelTime = 35;

			scan.ch[1].number = 1;//atomic_read(&hw_priv->p2p_oper_channel);
			scan.ch[1].maxChannelTime = maxChannelTime;
			scan.ch[1].minChannelTime = 35;

			scan.ch[2].number = 6;//atomic_read(&hw_priv->p2p_oper_channel);
			scan.ch[2].maxChannelTime = maxChannelTime;
			scan.ch[2].minChannelTime = 35;

			scan.ch[3].number = 11;//atomic_read(&hw_priv->p2p_oper_channel);
			scan.ch[3].maxChannelTime = maxChannelTime;
			scan.ch[3].minChannelTime = 35;
			scan.maxTransmitRate = WSM_TRANSMIT_RATE_6;

			scan.ch[4].number = atomic_read(&hw_priv->p2p_oper_channel);
			scan.ch[4].maxChannelTime = maxChannelTime;
			scan.ch[4].minChannelTime = 35;
			
			scan.ssids = &hw_priv->scan.ssids[0];

			scan.probeDelay = 15;

			scan.numOfChannels = 5;
			scan.band = IEEE80211_BAND_2GHZ;

			status = Sstar_scan_start(priv, &scan);
			Sstar_kfree(scan.ch);

			if(status)
				break;

			mutex_unlock(&hw_priv->conf_mutex);

			return;
		}
		#endif 
		if (hw_priv->scan.status < 0)
			wiphy_dbg(priv->hw->wiphy,
					"[SCAN] Scan failed (%d).\n",
					hw_priv->scan.status);
		else if (hw_priv->scan.req)
			wiphy_dbg(priv->hw->wiphy,
					"[SCAN] Scan completed.\n");
		else
			wiphy_dbg(priv->hw->wiphy,
					"[SCAN] Scan canceled.\n");

		hw_priv->scan.req = NULL;
		hw_priv->scan.cca = 0;
		hw_priv->scan.req_wrap = NULL;
		Sstar_printk_scan("%s:end(%d)\n",__func__,hw_priv->scan.if_id);
		Sstar_scan_restart_delayed(priv);
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
		hw_priv->enable_advance_scan = false;
#endif /* CONFIG_SSTAR_APOLLO_TESTMODE */
		wsm_unlock_tx(hw_priv);
		mutex_unlock(&hw_priv->conf_mutex);
		ieee80211_scan_completed(hw_priv->hw,
					 hw_priv->scan.status ? 1 : 0);
		up(&hw_priv->scan.lock);
		Sstar_release_suspend(hw_priv);
		return;
	} else {
		struct ieee80211_channel *first = *hw_priv->scan.curr;
		for (it = hw_priv->scan.curr + 1, i = 1;
		     it != hw_priv->scan.end &&
				i < WSM_SCAN_MAX_NUM_OF_CHANNELS;
		     ++it, ++i) {
			 	
			
			if ((*it)->band != first->band)
				break;

#ifdef WIFI_ALLIANCE_CERTIF
			if (((*it)->flags ^ first->flags) &  
			#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0))
				    IEEE80211_CHAN_PASSIVE_SCAN
		       #else
				    IEEE80211_CHAN_NO_IR
		       #endif
			   )
				break;  

			if (!(first->flags & 
			#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0))
				IEEE80211_CHAN_PASSIVE_SCAN
			#else
				IEEE80211_CHAN_NO_IR
			#endif
				) &&
			    (*it)->max_power != first->max_power)
				break;
#endif //WIFI_ALLIANCE_CERTIF
		}
		scan.band = first->band;
		#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 1, 0))
		if (hw_priv->scan.req->no_cck)
			scan.maxTransmitRate = WSM_TRANSMIT_RATE_6;
		else
		#endif
			scan.maxTransmitRate = WSM_TRANSMIT_RATE_1;
		
		if (priv->if_id&&priv->vif->p2p){
			scan.maxTransmitRate = WSM_TRANSMIT_RATE_6;
		}

		
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
		if (hw_priv->enable_advance_scan) {
			if (hw_priv->advanceScanElems.scanMode ==
				SSTAR_APOLLO_SCAN_MEASUREMENT_PASSIVE)
				scan.numOfProbeRequests = 0;
			else
				scan.numOfProbeRequests = 1;
		} else {
#endif
			/* TODO: Is it optimal? */
#ifndef CONFIG_SSTAR_5G_PRETEND_2G
			scan.numOfProbeRequests =
				(first->flags &
				#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0))
				IEEE80211_CHAN_PASSIVE_SCAN
				#else
				IEEE80211_CHAN_NO_IR
				#endif
				)
				? 0 : 3;
#else
		scan.numOfProbeRequests = 3;
#endif
			if(hw_priv->scan.cca == 1)
				scan.numOfProbeRequests = 1;
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
		}
#endif /* CONFIG_SSTAR_APOLLO_TESTMODE */
		/*
		*passive scan
		*/
		if(hw_priv->scan.passive)
			scan.numOfProbeRequests = 0;
		scan.numOfSSIDs = hw_priv->scan.n_ssids;
		scan.ssids = &hw_priv->scan.ssids[0];
		scan.numOfChannels = it - hw_priv->scan.curr;
		/* TODO: Is it optimal? */
		if(scan.numOfChannels == 3)
		{
			ProbeRequestTime = 10;
			ChannelRemainTime = 10;
		}
		scan.probeDelay = ProbeRequestTime;
		/* It is not stated in WSM specification, however
		 * FW team says that driver may not use FG scan
		 * when joined. */
		if (priv->join_status == SSTAR_APOLLO_JOIN_STATUS_STA) {
			scan.scanType = WSM_SCAN_TYPE_BACKGROUND;
			scan.scanFlags = WSM_SCAN_FLAG_FORCE_BACKGROUND;
		}		
		if(hw_priv->scan.cca == 1){
			scan.scanFlags |= WSM_FLAG_BEST_CHANNEL_START;
		}
		scan.ch = Sstar_kzalloc(
			sizeof(struct wsm_scan_ch[it - hw_priv->scan.curr]),
			GFP_KERNEL);
		if (!scan.ch) {
			hw_priv->scan.status = -ENOMEM;
			goto fail;
		}
		maxChannelTime = (scan.numOfSSIDs * scan.numOfProbeRequests *
			ProbeRequestTime) + ChannelRemainTime;
		maxChannelTime = (maxChannelTime < 35) ? 35 : maxChannelTime;
		for (i = 0; i < scan.numOfChannels; ++i) {
			scan.ch[i].number = channel_hw_value(hw_priv->scan.curr[i]);
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
			if (hw_priv->enable_advance_scan) {
				scan.ch[i].minChannelTime =
					hw_priv->advanceScanElems.duration;
				scan.ch[i].maxChannelTime =
					hw_priv->advanceScanElems.duration;
			} else {
#endif
				#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0))
				if (hw_priv->scan.curr[i]->flags & IEEE80211_CHAN_PASSIVE_SCAN) 
				#else
				if (hw_priv->scan.curr[i]->flags & IEEE80211_CHAN_NO_IR)
				#endif
				{
					scan.ch[i].minChannelTime = 50;
					scan.ch[i].maxChannelTime = 110;
				}
				else {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 1, 0))
					if (hw_priv->scan.req->no_cck)						
						scan.ch[i].minChannelTime = 35;
					else
#endif //#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 1, 0))
						scan.ch[i].minChannelTime = 15;
					scan.ch[i].maxChannelTime = maxChannelTime;
				}
				if(hw_priv->scan.cca == 1){
					scan.ch[i].maxChannelTime = 500;
				}
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
			}
#endif
		}
		/*
		if((priv->if_id == 1)&&(priv->join_status<=SSTAR_APOLLO_JOIN_STATUS_MONITOR))
		if(i>=14)
		{
			scan.ch[i-1].number = scan.ch[0].number;
			scan.ch[i-1].maxChannelTime = scan.ch[0].maxChannelTime;
			scan.ch[i-1].minChannelTime = scan.ch[0].minChannelTime;
		}
		*/
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
		if (!hw_priv->enable_advance_scan) {
#endif
			if (
				#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0))
				!(first->flags & IEEE80211_CHAN_PASSIVE_SCAN)
				#else
				!(first->flags & IEEE80211_CHAN_NO_IR)
				#endif
				&&
			    hw_priv->scan.output_power != first->max_power) {
				hw_priv->scan.output_power = first->max_power;
				/* TODO:COMBO: Change after mac80211 implementation
			 	* complete */
#ifdef P2P_MULTIVIF
				WARN_ON(wsm_set_output_power(hw_priv,
						hw_priv->scan.output_power * 10,
						priv->if_id ? 1 : 0));
#else
				WARN_ON(wsm_set_output_power(hw_priv,
						hw_priv->scan.output_power * 10,
						priv->if_id));
#endif
			}
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
		}
#endif
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
		if (hw_priv->enable_advance_scan &&
			(hw_priv->advanceScanElems.scanMode ==
				SSTAR_APOLLO_SCAN_MEASUREMENT_PASSIVE) &&
			(priv->join_status == SSTAR_APOLLO_JOIN_STATUS_STA) &&
			(channel_hw_value(hw_priv->channel)== advance_scan_req_channel)) {
				/* Start Advance Scan Timer */
				hw_priv->scan.status = Sstar_advance_scan_start(hw_priv);
				wsm_unlock_tx(hw_priv);
		} else
#endif
		/*
		*sometimes,t920 take a long time to scan,and
		*the usb recive process run too slowly.if that
		*happens,can triger some errors.
		*/
		//hw_priv->scan.status = Sstar_scan_start(priv, &scan);
		Sstar_printk_scan("scan start band(%d),(%d)\n",scan.band,scan.numOfChannels);
		scan_status = Sstar_scan_start(priv, &scan);
		Sstar_kfree(scan.ch);
		if (WARN_ON(scan_status)){
			hw_priv->scan.status = scan_status;
			goto fail;
		}
		#ifndef SSTAR_P2P_CHANGE
		if((priv->if_id == 1)&&(priv->join_status<=SSTAR_APOLLO_JOIN_STATUS_MONITOR)&&(priv->scan_again == 0))
		{
			priv->scan_again = 1;
		}
		else
		{
			priv->scan_again = 0;
		}
		#endif
		hw_priv->scan.curr = it;
	}
	mutex_unlock(&hw_priv->conf_mutex);
	return;

fail:	
	hw_priv->scan.curr = hw_priv->scan.end;
	mutex_unlock(&hw_priv->conf_mutex);
	Sstar_hw_priv_queue_work(hw_priv, &hw_priv->scan.work);
	return;
}

#ifdef ROAM_OFFLOAD
void Sstar_sched_scan_work(struct work_struct *work)
{
	struct Sstar_common *hw_priv = container_of(work, struct Sstar_common,
		scan.swork);
	struct wsm_scan scan;
	struct wsm_ssid scan_ssid;
	int i;
	struct Sstar_vif *priv = ABwifi_hwpriv_to_vifpriv(hw_priv,
					hw_priv->scan.if_id);
	if (unlikely(!priv)) {
		WARN_ON(1);
		return;
	}

	Sstar_priv_vif_list_read_unlock(&priv->vif_lock);

	/* Firmware gets crazy if scan request is sent
	 * when STA is joined but not yet associated.
	 * Force unjoin in this case. */
	if (Sstar_cancle_delayed_work(&priv->join_timeout,true) > 0) {
		Sstar_join_timeout(&priv->join_timeout.work);
	}
	mutex_lock(&hw_priv->conf_mutex);
	hw_priv->auto_scanning = 1;

	scan.band = 0;

	if (priv->join_status == SSTAR_APOLLO_JOIN_STATUS_STA)
		scan.scanType = 3; /* auto background */
	else
		scan.scanType = 2; /* auto foreground */

	scan.scanFlags = 0x01; /* bit 0 set => forced background scan */
	scan.maxTransmitRate = WSM_TRANSMIT_RATE_6;
	scan.autoScanInterval = (0xba << 24)|(30 * 1024); /* 30 seconds, -70 rssi */
	scan.numOfProbeRequests = 1;
	//scan.numOfChannels = 11;
	scan.numOfChannels = hw_priv->num_scanchannels;
	scan.numOfSSIDs = 1;
	scan.probeDelay = 100;
	scan_ssid.length = priv->ssid_length;
	memcpy(scan_ssid.ssid, priv->ssid, priv->ssid_length);
	scan.ssids = &scan_ssid;

	scan.ch = Sstar_kzalloc(
		sizeof(struct wsm_scan_ch[scan.numOfChannels]),
		GFP_KERNEL);
	if (!scan.ch) {
		hw_priv->scan.status = -ENOMEM;
		goto fail;
	}

	for (i = 0; i < scan.numOfChannels; i++) {
		scan.ch[i].number = hw_priv->scan_channels[i].number;
		scan.ch[i].minChannelTime = hw_priv->scan_channels[i].minChannelTime;
		scan.ch[i].maxChannelTime = hw_priv->scan_channels[i].maxChannelTime;
		scan.ch[i].txPowerLevel = hw_priv->scan_channels[i].txPowerLevel;
	}

#if 0
	for (i = 1; i <= scan.numOfChannels; i++) {
		scan.ch[i-1].number = i;
		scan.ch[i-1].minChannelTime = 10;
		scan.ch[i-1].maxChannelTime = 40;
	}
#endif

	hw_priv->scan.status = Sstar_sched_scan_start(priv, &scan);
	Sstar_kfree(scan.ch);
	if (hw_priv->scan.status)
		goto fail;
	mutex_unlock(&hw_priv->conf_mutex);
	return;

fail:
	mutex_unlock(&hw_priv->conf_mutex);
	Sstar_hw_priv_queue_work(hw_priv, &hw_priv->scan.swork);
	return;
}

void Sstar_hw_sched_scan_stop(struct Sstar_common *hw_priv)
{
	struct Sstar_vif *priv = ABwifi_hwpriv_to_vifpriv(hw_priv,
					hw_priv->scan.if_id);
	if (unlikely(!priv))
		return;
	Sstar_priv_vif_list_read_unlock(&priv->vif_lock);

	wsm_stop_scan(hw_priv, priv->if_id);

	return;
}
#endif /*ROAM_OFFLOAD*/


static void Sstar_scan_restart_delayed(struct Sstar_vif *priv)
{
	struct Sstar_common *hw_priv = ABwifi_vifpriv_to_hwpriv(priv);
	struct Sstar_vif *priv_delay = NULL;
	u8 i = 0;
	Sstar_for_each_vif(hw_priv, priv_delay, i) {
		if (priv_delay == NULL)
			continue;

		if(!((priv_delay->delayed_link_loss) || (priv_delay->delayed_unjoin) ||
			(priv_delay->join_status == SSTAR_APOLLO_JOIN_STATUS_MONITOR)))
			continue;

		Sstar_printk_scan("%s:if_id(%d),scan_id(%d),join_status(%d),delayed_link_loss(%d),delayed_unjoin(%d)\n",
			__func__,priv_delay->if_id,hw_priv->scan.if_id,priv_delay->join_status,priv_delay->delayed_link_loss,priv_delay->delayed_unjoin);
		if (priv_delay->delayed_link_loss) {
			int tmo = priv_delay->cqm_beacon_loss_count;

			if (hw_priv->scan.direct_probe)
				tmo = 0;

			priv_delay->delayed_link_loss = 0;
			/* Restart beacon loss timer and requeue
			   BSS loss work. */
			wiphy_dbg(priv_delay->hw->wiphy,
					"[CQM] Requeue BSS loss in %d "
					"beacons.\n", tmo);
			spin_lock_bh(&priv_delay->bss_loss_lock);
			priv_delay->bss_loss_status = SSTAR_APOLLO_BSS_LOSS_NONE;
			spin_unlock_bh(&priv_delay->bss_loss_lock);
			Sstar_cancle_delayed_work(&priv_delay->bss_loss_work,false);
			Sstar_hw_priv_queue_delayed_work(hw_priv,
					&priv_delay->bss_loss_work,
					tmo * HZ / 10);
		}

		/* FW bug: driver has to restart p2p-dev mode after scan. */
		if (priv_delay->join_status == SSTAR_APOLLO_JOIN_STATUS_MONITOR) {
			/*Sstar_enable_listening(priv);*/
//			WARN_ON(1);
			Sstar_update_filtering(priv_delay);
		}

		if (priv_delay->delayed_unjoin) {
			priv_delay->delayed_unjoin = false;
			Sstar_printk_scan("%s:restart delayed_unjoin\n",__func__);
			#if 1
			wsm_lock_tx_async(hw_priv);
			#endif
			if (Sstar_hw_priv_queue_work(hw_priv, &priv_delay->unjoin_work) <= 0)
				wsm_unlock_tx(hw_priv);
		}
	}
}
void Sstar_scan_listenning_restart_delayed(struct Sstar_vif *priv)
{
	Sstar_scan_restart_delayed(priv);
}
static void Sstar_scan_complete(struct Sstar_common *hw_priv, int if_id)
{
	struct Sstar_vif *priv;
	atomic_xchg(&hw_priv->recent_scan, 0);



	if (hw_priv->scan.direct_probe) {
		mutex_lock(&hw_priv->conf_mutex);
		priv = __ABwifi_hwpriv_to_vifpriv(hw_priv, if_id);
		if (priv) {
			wiphy_dbg(priv->hw->wiphy, "[SCAN] Direct probe "
				  "complete.\n");
			Sstar_scan_restart_delayed(priv);
		} else {
			wiphy_dbg(priv->hw->wiphy, "[SCAN] Direct probe "
				  "complete without interface!\n");
		}
		mutex_unlock(&hw_priv->conf_mutex);
		hw_priv->scan.direct_probe = 0;
		up(&hw_priv->scan.lock);
		wsm_unlock_tx(hw_priv);
	} else {
		Sstar_scan_work(&hw_priv->scan.work);
	}

	
}

void Sstar_scan_complete_cb(struct Sstar_common *hw_priv,
				struct wsm_scan_complete *arg)
{
	int i;
	struct Sstar_vif *priv = ABwifi_hwpriv_to_vifpriv(hw_priv,
					hw_priv->scan.if_id);	

	
	if (unlikely(!priv))
	{
		return;
	}


#ifdef ROAM_OFFLOAD
	if (hw_priv->auto_scanning)
		Sstar_hw_priv_queue_delayed_work(hw_priv,
				&hw_priv->scan.timeout, 0);
#endif /*ROAM_OFFLOAD*/

	if (unlikely(priv->mode == NL80211_IFTYPE_UNSPECIFIED)) {
		/* STA is stopped. */
		Sstar_priv_vif_list_read_unlock(&priv->vif_lock);
		Sstar_printk_err("%s:priv->mode == NL80211_IFTYPE_UNSPECIFIED\n",__func__);
		return;
	}
	Sstar_priv_vif_list_read_unlock(&priv->vif_lock);

	//printk("hw_priv->bStartTx %d\n",hw_priv->bStartTx);
	if(hw_priv->bStartTx)
	{
		Sstar_hw_priv_queue_delayed_work(hw_priv,&hw_priv->scan.timeout, HZ/10);
		return;
	}

	
#ifdef SSTAR_SUPPORT_SMARTCONFIG
	priv->scan_no_connect=priv->scan_no_connect_back;
	if (hw_priv->scan.scan_smartconfig){
		//printk("**%s %d**\n", __FUNCTION__, hw_priv->scan.scan_smartconfig);
		smartconfig_magic_scan_done(hw_priv);
		return ;
	}
#endif
	Sstar_printk_scan("hw_priv->scan.status %d\n",hw_priv->scan.status);
	if(hw_priv->scan.status == -ETIMEDOUT)
		wiphy_warn(hw_priv->hw->wiphy,
			"Scan timeout already occured. Don't cancel work");
	if ((hw_priv->scan.status != -ETIMEDOUT) &&
		(Sstar_cancle_delayed_work(&hw_priv->scan.timeout,false/*can't set to true,because this function is call in bh, must not wait in bh */) > 0)) {
		hw_priv->scan.status = 1;
		if(hw_priv->scan.cca){
			struct ieee80211_internal_scan_notity notify;
			notify.cca.val = arg->busy_ratio;
			notify.cca.val_len = sizeof(arg->busy_ratio);
			notify.success = true;
			ieee80211_scan_cca_notify(hw_priv->hw,&notify);
		}
		else {
				struct ieee80211_local *local = hw_to_local(hw_priv->hw);
				for(i=0;(i<CHANNEL_NUM)&&(i<14);i++){
				local->noise_floor[i] = arg->busy_ratio[i];
			}
}

		Sstar_hw_priv_queue_delayed_work(hw_priv,
				&hw_priv->scan.timeout, 0);
	}
}
//#ifdef CONFIG_WIRELESS_EXT
extern int wsm_start_scan_etf(struct Sstar_common *hw_priv, struct ieee80211_vif *vif );

void etf_scan_end_work(struct work_struct *work)
{
	struct Sstar_common *hw_priv =
		container_of(work, struct Sstar_common, etf_tx_end_work);
	
	struct Sstar_vif *priv = __ABwifi_hwpriv_to_vifpriv(hw_priv,
					hw_priv->scan.if_id);
	
	etf_v2_scan_end(hw_priv,priv->vif);
}
//#endif  //CONFIG_WIRELESS_EXT
void Sstar_scan_timeout(struct work_struct *work)
{
	struct Sstar_common *hw_priv =
		container_of(work, struct Sstar_common, scan.timeout.work);
//#ifdef CONFIG_WIRELESS_EXT
	if(hw_priv->bStartTx)
	{
		struct Sstar_vif *priv = __ABwifi_hwpriv_to_vifpriv(hw_priv,
						hw_priv->scan.if_id);
		if(hw_priv->bStartTxWantCancel==0){
			
			wsm_start_scan_etf(hw_priv,priv->vif);
		}
		else {
			hw_priv->bStartTx = 0;
			hw_priv->bStartTxWantCancel  = 0;
			if(hw_priv->etf_test_v2){
				Sstar_hw_priv_queue_work(hw_priv, &hw_priv->etf_tx_end_work);
			}
			//stop etf test
			//up(&hw_priv->scan.lock);
			//printk("Sstar_scan_timeout bStartTx %d\n",hw_priv->bStartTx);
		}
		return;
	}
//#endif  //CONFIG_WIRELESS_EXT
	
	if (likely(atomic_xchg(&hw_priv->scan.in_progress, 0))) {
		if (hw_priv->scan.status > 0)
			hw_priv->scan.status = 0;
		else if (!hw_priv->scan.status) {
			wiphy_warn(hw_priv->hw->wiphy,
				"Timeout waiting for scan "
				"complete notification.\n");
			hw_priv->scan.status = -ETIMEDOUT;
			hw_priv->scan.curr = hw_priv->scan.end;
			if(WARN_ON(wsm_stop_scan(hw_priv,
						hw_priv->scan.if_id ? 1 : 0)))
			{
				hw_priv->scan.wait_complete= 0;
				//#ifndef OPER_CLOCK_USE_SEM
				//mutex_unlock(&hw_priv->wsm_oper_lock);
				//#else
				//up(&hw_priv->wsm_oper_lock);
				//#endif
				wsm_oper_unlock(hw_priv);
			}
		}
		Sstar_scan_complete(hw_priv, hw_priv->scan.if_id);
#ifdef ROAM_OFFLOAD
	} else if (hw_priv->auto_scanning) {
		hw_priv->auto_scanning = 0;
		ieee80211_sched_scan_results(hw_priv->hw);
#endif /*ROAM_OFFLOAD*/
	}

}

#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
void Sstar_advance_scan_timeout(struct work_struct *work)
{
	struct Sstar_common *hw_priv =
		container_of(work, struct Sstar_common, advance_scan_timeout.work);

	struct Sstar_vif *priv = ABwifi_hwpriv_to_vifpriv(hw_priv,
					hw_priv->scan.if_id);
	if (WARN_ON(!priv))
		return;
	Sstar_priv_vif_list_read_unlock(&priv->vif_lock);

	hw_priv->scan.status = 0;
	if (hw_priv->advanceScanElems.scanMode ==
		SSTAR_APOLLO_SCAN_MEASUREMENT_PASSIVE) {
		/* Passive Scan on Serving Channel
		 * Timer Expire */
		Sstar_scan_complete(hw_priv, hw_priv->scan.if_id);
	} else {
		/* Active Scan on Serving Channel
		 * Timer Expire */
		mutex_lock(&hw_priv->conf_mutex);
		//wsm_lock_tx(priv);
		wsm_vif_lock_tx(priv);
		/* Once Duration is Over, enable filtering
		 * and Revert Back Power Save */
		if ((priv->powersave_mode.pmMode & WSM_PSM_PS))
			wsm_set_pm(hw_priv, &priv->powersave_mode,
				priv->if_id);
		hw_priv->scan.req = NULL;
		Sstar_update_filtering(priv);
		hw_priv->enable_advance_scan = false;
		wsm_unlock_tx(hw_priv);
		mutex_unlock(&hw_priv->conf_mutex);
		ieee80211_scan_completed(hw_priv->hw,
			hw_priv->scan.status ? true : false);
		up(&hw_priv->scan.lock);
	}
}
#endif

void Sstar_probe_work(struct work_struct *work)
{
	struct Sstar_common *hw_priv =
		container_of(work, struct Sstar_common, scan.probe_work.work);
	struct Sstar_vif *priv, *vif;
	u8 queueId = Sstar_queue_get_queue_id(hw_priv->pending_frame_id);
	struct Sstar_queue *queue = &hw_priv->tx_queue[queueId];
	const struct Sstar_txpriv *txpriv;
	struct wsm_tx *wsm;
	struct wsm_template_frame frame = {
		.frame_type = WSM_FRAME_TYPE_PROBE_REQUEST,
	};
	struct wsm_ssid ssids[1] = {{
		.length = 0,
	} };
	struct wsm_scan_ch ch[1] = {{
		.minChannelTime = 0,
		.maxChannelTime = 10,
	} };
	struct wsm_scan scan = {
		.scanType = WSM_SCAN_TYPE_FOREGROUND,
		.numOfProbeRequests = 1,
		.probeDelay = 0,
		.numOfChannels = 1,
		.ssids = ssids,
		.ch = ch,
	};
	u8 *ies;
	size_t ies_len;
	int ret = 1;
	int i;
	Sstar_printk_scan("[SCAN] Direct probe work.\n");
	BUG_ON(queueId >= 4);
	BUG_ON(!hw_priv->channel);
	if(Sstar_bh_is_term(hw_priv))
	{
		#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
		BUG_ON(Sstar_queue_remove(hw_priv, queue,
				hw_priv->pending_frame_id));
		#else
		BUG_ON(Sstar_queue_remove(queue, hw_priv->pending_frame_id));
		#endif
		wsm_unlock_tx(hw_priv);
		return;
	}
	mutex_lock(&hw_priv->conf_mutex);
	if (unlikely(down_trylock(&hw_priv->scan.lock))) {
		/* Scan is already in progress. Requeue self. */
		if (unlikely(atomic_read(&hw_priv->scan.in_progress))){
			schedule();
			Sstar_hw_priv_queue_delayed_work(hw_priv,
						&hw_priv->scan.probe_work, HZ / 10);
			mutex_unlock(&hw_priv->conf_mutex);
			Sstar_printk_scan("%s:scanning so delay work\n",__func__);
		}
		else{
			#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
			BUG_ON(Sstar_queue_remove(hw_priv, queue,
					hw_priv->pending_frame_id));
			#else
			BUG_ON(Sstar_queue_remove(queue, hw_priv->pending_frame_id));
			#endif
			wsm_unlock_tx(hw_priv);
			mutex_unlock(&hw_priv->conf_mutex);
			Sstar_printk_scan("%s:listenning or other delete pendding frame\n",__func__);
		}
		return;
	}

	if (Sstar_queue_get_skb(queue,	hw_priv->pending_frame_id,
			&frame.skb, &txpriv)) {
		up(&hw_priv->scan.lock);
		mutex_unlock(&hw_priv->conf_mutex);
		wsm_unlock_tx(hw_priv);
		Sstar_printk_err("[SCAN] Direct probe work. return\n");
		return;
	}
	priv = __ABwifi_hwpriv_to_vifpriv(hw_priv, txpriv->if_id);
	if (!priv) {
		up(&hw_priv->scan.lock);
		mutex_unlock(&hw_priv->conf_mutex);
		Sstar_printk_err("[SCAN] Direct probe work. !priv\n");
		return;
	}
	wsm = (struct wsm_tx *)frame.skb->data;
	scan.maxTransmitRate = wsm->maxTxRate;
	scan.band = (hw_priv->channel->band == IEEE80211_BAND_5GHZ) ?
		WSM_PHY_BAND_5G : WSM_PHY_BAND_2_4G;
	if (priv->join_status == SSTAR_APOLLO_JOIN_STATUS_STA) {
		scan.scanType = WSM_SCAN_TYPE_BACKGROUND;
		scan.scanFlags = WSM_SCAN_FLAG_FORCE_BACKGROUND;
		if (priv->if_id)
			scan.scanFlags |= WSM_FLAG_MAC_INSTANCE_1;
		else
			scan.scanFlags &= ~WSM_FLAG_MAC_INSTANCE_1;
	}
	Sstar_for_each_vif(hw_priv, vif, i) {
		if (!vif)
			continue;
		if (vif->bss_loss_status > SSTAR_APOLLO_BSS_LOSS_NONE)
			scan.scanFlags |= WSM_SCAN_FLAG_FORCE_BACKGROUND;
	}
	ch[0].number = channel_hw_value(hw_priv->channel);

	Sstar_skb_pull(frame.skb, txpriv->offset);

	ies = &frame.skb->data[sizeof(struct ieee80211_hdr_3addr)];
	ies_len = frame.skb->len - sizeof(struct ieee80211_hdr_3addr);
	if (ies_len) {
		u8 *ssidie =
			(u8 *)cfg80211_find_ie(SSTAR_WLAN_EID_SSID, ies, ies_len);
		if (ssidie && ssidie[1] && ssidie[1] <= sizeof(ssids[0].ssid)) {
			u8 *nextie = &ssidie[2 + ssidie[1]];
			/* Remove SSID from the IE list. It has to be provided
			 * as a separate argument in Sstar_scan_start call */

			/* Store SSID localy */
			ssids[0].length = ssidie[1];
			memcpy(ssids[0].ssid, &ssidie[2], ssids[0].length);
			scan.numOfSSIDs = 1;

			/* Remove SSID from IE list */
			ssidie[1] = 0;
			memmove(&ssidie[2], nextie, &ies[ies_len] - nextie);
			Sstar_skb_trim(frame.skb, frame.skb->len - ssids[0].length);
		}
	}

	if (priv->if_id == 0)
		Sstar_remove_wps_p2p_ie(&frame);

	/* FW bug: driver has to restart p2p-dev mode after scan */
	if (priv->join_status == SSTAR_APOLLO_JOIN_STATUS_MONITOR) {
		WARN_ON(1);
		/*Sstar_disable_listening(priv);*/
	}
	ret = WARN_ON(wsm_set_template_frame(hw_priv, &frame,
				priv->if_id));

	hw_priv->scan.direct_probe = 1;
	hw_priv->scan.if_id = priv->if_id;
	hw_priv->scan.status = 0;
	priv->tmpframe_probereq_set = 1;
	if (!ret) {
		wsm_flush_tx(hw_priv);
		ret = WARN_ON(Sstar_scan_start(priv, &scan));
	}
	mutex_unlock(&hw_priv->conf_mutex);

	Sstar_skb_push(frame.skb, txpriv->offset);
	if (!ret)
		IEEE80211_SKB_CB(frame.skb)->flags |= IEEE80211_TX_STAT_ACK;
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
		(Sstar_queue_remove(hw_priv, queue,hw_priv->pending_frame_id));
#else
		(Sstar_queue_remove(queue, hw_priv->pending_frame_id));
#endif

	if (ret) {
		hw_priv->scan.direct_probe = 0;
		up(&hw_priv->scan.lock);
		wsm_unlock_tx(hw_priv);
	}

	return;
}


#if 0
#define SCAN_CHANNEL_NUM 14
extern 	void frame_hexdump(char *prefix, u8 *data, int len);
struct sk_buff *Sstar_ieee80211_probereq_get(struct Sstar_common *hw_priv)
{
	struct ieee80211_hdr_3addr *hdr;
	struct sk_buff *skb;
	size_t ie_ssid_len = 0;
	u8 *pos;
	u32 ratebit[2] = {0,0};
	u8 i = 0;
	size_t ielen = 0;
	struct wiphy *wiphy = hw_priv->hw->wiphy;

	skb = Sstar_dev_alloc_skb(1024);
	if (!skb)
		return NULL;

	//skb_reserve(skb, local->hw.extra_tx_headroom);

	hdr = (struct ieee80211_hdr_3addr *) skb_put(skb, sizeof(*hdr));
	memset(hdr, 0, sizeof(*hdr));
	hdr->frame_control = cpu_to_le16(IEEE80211_FTYPE_MGMT |
					 IEEE80211_STYPE_PROBE_REQ);
	memset(hdr->addr1, 0xff, ETH_ALEN);
	memcpy(hdr->addr2, hw_priv->addresses[0].addr, ETH_ALEN);
	memset(hdr->addr3, 0xff, ETH_ALEN);

	for (i = 0; i < IEEE80211_NUM_BANDS; i++)  
		if (wiphy->bands[i])   
			ratebit[i] =(1 << wiphy->bands[i]->n_bitrates) - 1;

#if (0)//test hidden ssid
	u8 ssid_len = 0;
	u8 *ssid = "sigmastar_test";
	ssid_len = strlen(ssid);

	ie_ssid_len = ssid_len + 2;
	pos = skb_put(skb, ie_ssid_len);
	
	*pos++ = SSTAR_WLAN_EID_SSID;
	*pos++ = ssid_len;
	
	memcpy(pos, ssid, ssid_len);
	pos += ssid_len;
#else
	ie_ssid_len = 2;//without ssid
 	pos = skb_put(skb, ie_ssid_len);
	*pos++ = SSTAR_WLAN_EID_SSID;
	*pos++ = 0;
#endif
	ielen = ieee80211_build_preq_ies(hw_to_local(hw_priv->hw),pos,
					 NULL, 0, IEEE80211_BAND_2GHZ,
					 ratebit[IEEE80211_BAND_2GHZ], 0);
	Sstar_skb_put(skb,ielen);
	frame_hexdump("probeReq",(u8*)skb->data,skb->len);
	return skb;
}

void Sstar_private_scan(struct Sstar_vif *priv, u16 channel)
{
	struct Sstar_vif *vif;
	struct Sstar_common *hw_priv = ABwifi_vifpriv_to_hwpriv(priv);
	struct wsm_template_frame frame = {
		.frame_type = WSM_FRAME_TYPE_PROBE_REQUEST,
	};
	struct wsm_ssid ssids[1] = {{
		.length = 0,
	} };
	struct wsm_scan_ch ch[SCAN_CHANNEL_NUM];
	
	struct wsm_scan scan = {
		.scanType = WSM_SCAN_TYPE_FOREGROUND,
		.numOfProbeRequests = 5,
		.probeDelay = 20,
		.numOfChannels = 1,
		.ssids = ssids,
		.ch = ch,
	};
	u8 *ies;
	size_t ies_len;
	int ret = 1;
	int i;
	Sstar_printk_scan( "[SCAN] Direct private work. %p\n", hw_priv);
	//printk(KERN_ERR "[SCAN] %s: 1\n",__func__);
	//printk(KERN_ERR "[SCAN] %s: 2\n",__func__);
	
	wsm_lock_tx(hw_priv);
	if(Sstar_bh_is_term(hw_priv))
	{
		wsm_unlock_tx(hw_priv);
		return;
	}
	down(&hw_priv->scan.lock);
	mutex_lock(&hw_priv->conf_mutex);
	
	frame.skb = Sstar_ieee80211_probereq_get(hw_priv);
	if(frame.skb==NULL){
		
		Sstar_printk_scan("[SCAN] %s:%d\n",__func__,__LINE__);
		return;
	}
	
	frame.skb = Sstar_mgmt_add_private_ie(frame.skb);
	
	scan.maxTransmitRate = 0;//wsm->maxTxRate;
	scan.band = (hw_priv->channel->band == IEEE80211_BAND_5GHZ) ?
		WSM_PHY_BAND_5G : WSM_PHY_BAND_2_4G;
	if (priv->join_status == SSTAR_APOLLO_JOIN_STATUS_STA) {
		scan.scanType = WSM_SCAN_TYPE_BACKGROUND;
		scan.scanFlags = WSM_SCAN_FLAG_FORCE_BACKGROUND;
		if (priv->if_id)
			scan.scanFlags |= WSM_FLAG_MAC_INSTANCE_1;
		else
			scan.scanFlags &= ~WSM_FLAG_MAC_INSTANCE_1;
	}
	Sstar_for_each_vif(hw_priv, vif, i) {
		if (!vif)
			continue;
		if (vif->bss_loss_status > SSTAR_APOLLO_BSS_LOSS_NONE)
			scan.scanFlags |= WSM_SCAN_FLAG_FORCE_BACKGROUND;
	}
	
	if(channel == 0){
		for(i=0; i<SCAN_CHANNEL_NUM; i++){
			ch[i].number = i+1;
			ch[i].maxChannelTime = 55;
			ch[i].minChannelTime = 35;
		}

		scan.numOfChannels = SCAN_CHANNEL_NUM;
	}else{
		ch[0].number = channel;
		ch[0].maxChannelTime = 55;
		ch[0].minChannelTime = 35;
	}

	ies = &frame.skb->data[sizeof(struct ieee80211_hdr_3addr)];
	ies_len = frame.skb->len - sizeof(struct ieee80211_hdr_3addr);
	if (ies_len) {
		u8 *ssidie =
			(u8 *)cfg80211_find_ie(SSTAR_WLAN_EID_SSID, ies, ies_len);
		if (ssidie && ssidie[1] && ssidie[1] <= sizeof(ssids[0].ssid)) {
			u8 *nextie = &ssidie[2 + ssidie[1]];
			/* Remove SSID from the IE list. It has to be provided
			 * as a separate argument in Sstar_scan_start call */

			/* Store SSID localy */
			ssids[0].length = ssidie[1];
			memcpy(ssids[0].ssid, &ssidie[2], ssids[0].length);
			scan.numOfSSIDs = 1;

			/* Remove SSID from IE list */
			ssidie[1] = 0;
			memmove(&ssidie[2], nextie, &ies[ies_len] - nextie);
			skb_trim(frame.skb, frame.skb->len - ssids[0].length);
		}
	}
	/* FW bug: driver has to restart p2p-dev mode after scan */
	if (priv->join_status == SSTAR_APOLLO_JOIN_STATUS_MONITOR) {
		WARN_ON(1);
		/*Sstar_disable_listening(priv);*/
	}
	ret = WARN_ON(wsm_set_template_frame(hw_priv, &frame,
				priv->if_id));

	hw_priv->scan.direct_probe = 1;
	hw_priv->scan.if_id = priv->if_id;
	hw_priv->scan.status = 0;
	//priv->tmpframe_probereq_set = 1;
	if (!ret) {
		ret = WARN_ON(Sstar_scan_start(priv, &scan));
	}
	mutex_unlock(&hw_priv->conf_mutex);
	if (ret) {
		hw_priv->scan.direct_probe = 0;
		up(&hw_priv->scan.lock);
		wsm_unlock_tx(hw_priv);
	}
	Sstar_dev_kfree_skb(frame.skb);
	return;
}

#endif
void Sstar_wait_scan_complete_sync(struct Sstar_common *hw_priv)
{
	down(&hw_priv->scan.lock);
	mutex_lock(&hw_priv->conf_mutex);
	/*
	*here wait scan completed
	*/
	mutex_unlock(&hw_priv->conf_mutex);
	up(&hw_priv->scan.lock);
	Sstar_printk_scan( "%s\n",__func__);
}
void Sstar_cancel_hw_scan(struct ieee80211_hw *hw,struct ieee80211_vif *vif)
{
	struct Sstar_common *hw_priv = hw->priv;
	struct Sstar_vif *priv = ABwifi_get_vif_from_ieee80211(vif);

	Sstar_printk_scan( "%s:[%d]\n",__func__,priv->if_id);

	Sstar_wait_scan_complete_sync(hw_priv);
}
