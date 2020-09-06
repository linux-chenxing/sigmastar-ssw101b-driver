
/*
 * Mac80211 STA API for sigmastar APOLLO drivers
 *
 * Copyright (c) 2016, sigmastar
 *
 * Based on:
 * Copyright (c) 2010, stericsson
 * Author: Dmitry Tarnyagin <dmitry.tarnyagin@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/sched.h>
#include <linux/firmware.h>
#include <linux/if_arp.h>
#include <linux/ipv6.h>
#include <linux/icmpv6.h>
#include <net/ndisc.h>

#include "apollo.h"
#include "sta.h"
#include "ap.h"
#include "fwio.h"
#include "bh.h"
#include "debug.h"
#include "wsm.h"
#include "hwio.h"
#ifdef ROAM_OFFLOAD
#include <net/netlink.h>
#endif /*ROAM_OFFLOAD*/
//#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
#include "Sstar_testmode.h"
#include <net/netlink.h>
//#endif /* CONFIG_SSTAR_APOLLO_TESTMODE */

#include "net/Sstar_mac80211.h"
#include "dbg_event.h"
#include "smartconfig.h"

#include "mac80211/ieee80211_i.h"
#include "mac80211/Sstar_common.h"
#include "internal_cmd.h"

#define SSTAR_DEV_IOCTL_DEBUG 1

#if SSTAR_DEV_IOCTL_DEBUG
#define dev_printk(...) Sstar_printk_always(__VA_ARGS__)
#else
#define dev_printk(...)
#endif

#define DEV_MAC2STR(a) (a)[0],(a)[1],(a)[2],(a)[3],(a)[4],(a)[5]
#define DEV_MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
enum Sstar_msg_type{
    SSTAR_DEV_IO_GET_STA_STATUS   = 0,
    SSTAR_DEV_IO_GET_STA_RSSI     = 1, //STA connected AP's RSSI
    SSTAR_DEV_IO_GET_AP_INFO      = 2,  //STA or AP
    SSTAR_DEV_IO_GET_STA_INFO     = 3,
    SSTAR_DEV_IO_SET_STA_SCAN     = 4,
    SSTAR_DEV_IO_SET_FREQ         = 5,
    SSTAR_DEV_IO_SET_SPECIAL_OUI  = 6, //use for Beacon and Probe package
    SSTAR_DEV_IO_SET_STA_DIS      = 7,
    SSTAR_DEV_IO_SET_MONITOR      = 8,
    SSTAR_DEV_IO_SET_ADAPTIVE     = 9,
    SSTAR_DEV_IO_SET_TXPWR_DCXO   = 10,
    SSTAR_DEV_IO_SET_TXPWR        = 11,
    SSTAR_DEV_IO_GET_WORK_CHANNEL   = 12,
    SSTAR_DEV_IO_SET_BEST_CHANNEL_SCAN   = 13,
    SSTAR_DEV_IO_GET_AP_LIST   = 14,
    SSTAR_DEV_IO_GET_TP_RATE   = 15,
    SSTAR_DEV_IO_ETF_TEST   = 16,
    SSTAR_DEV_IO_ETF_GET_RESULT  = 17,
    SSTAR_DEV_IO_ETF_START_TX	= 18,
    SSTAR_DEV_IO_ETF_STOP_TX		= 19,
    SSTAR_DEV_IO_ETF_START_RX	= 20,
    SSTAR_DEV_IO_ETF_STOP_RX		= 21,
    SSTAR_DEV_IO_FIX_TX_RATE		 = 22,
    SSTAR_DEV_IO_MAX_TX_RATE		 = 23,
    SSTAR_DEV_IO_TX_RATE_FREE	 = 24,
    SSTAR_DEV_IO_SET_EFUSE_MAC    = 25,
	SSTAR_DEV_IO_SET_EFUSE_DCXO   = 26,
	SSTAR_DEV_IO_SET_EFUSE_DELTAGAIN = 27,
	SSTAR_DEV_IO_MIN_TX_RATE		 = 28,
	SSTAR_DEV_IO_SET_RATE_POWER	 = 29,
};

#define WSM_MAX_NUM_LINK_AP 14//Lmac support station number is 4;
typedef struct _Sstar_wifi_ap_info_{
    int wext_rssi;
    unsigned long rx_packets;
    unsigned long tx_packets;
    unsigned long tx_retry_count;
    int last_rx_rate_idx;
    unsigned char  wext_mac[ETH_ALEN];
    unsigned char  sta_cnt;
}Sstar_wifi_ap_info;

typedef struct _Sstar_wifi_sta_info_{
    int rssi;
    unsigned long rx_packets;
    unsigned long tx_packets;
    unsigned long tx_retry_count;
    u8  bssid[ETH_ALEN];
    u8  ssid[IEEE80211_MAX_SSID_LEN];
    size_t ssid_len;
}Sstar_wifi_sta_info;


struct altm_wext_msg{
    int type;
    int value;
    char externData[256];
};

extern unsigned int Sstar_wifi_status_get(void);
extern int Sstar_change_iface_to_monitor(struct net_device *dev);
extern void ieee80211_connection_loss(struct ieee80211_vif *vif);
extern int ieee80211_set_channel(struct wiphy *wiphy,
                 struct net_device *netdev,
                 struct ieee80211_channel *chan,
                 enum nl80211_channel_type channel_type);
extern int Sstar_find_link_id(struct Sstar_vif *priv, const u8 *mac);
extern int str2mac(char *dst_mac, char *src_str);
extern void Sstar_set_freq(struct Sstar_common *hw_priv, SPECIAL_CH_FREQ *pdata);
extern void Sstar_set_tx_power(struct Sstar_common *hw_priv, int txpw);
extern u8 ETF_bStartTx;
extern u8 ETF_bStartRx;
extern char ch_and_type[20];

extern u8 ucWriteEfuseFlag;
extern u32 chipversion;
extern struct rxstatus_signed gRxs_s;
extern struct test_threshold gthreshold_param;
extern u32 MyRand(void);
extern void etf_param_init(struct Sstar_common *hw_priv);
extern int wsm_start_tx_v2(struct Sstar_common *hw_priv, struct ieee80211_vif *vif );
extern int wsm_start_tx(struct Sstar_common *hw_priv, struct ieee80211_vif *vif);
extern int wsm_stop_tx(struct Sstar_common *hw_priv);

static char spec_oui_buf[256];
static char *spec_oui = "NULL";
module_param(spec_oui,charp,0644);
MODULE_PARM_DESC(spec_oui,"special oui");
void Sstar_set_special_oui(struct Sstar_common *hw_priv, char *pdata, int len)
{
    memset(spec_oui_buf, 0, 256);
    memcpy(spec_oui_buf, pdata, len);
    spec_oui = spec_oui_buf;

    return;
}

int Sstar_dev_get_ap_info(struct net_device *dev, struct altm_wext_msg *msg)
{
    int i = 0;
    int ret = 0;
    struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
    struct ieee80211_local *local = sdata->local;
    struct Sstar_vif *priv =  (struct Sstar_vif *)sdata->vif.drv_priv;
    struct Sstar_common *hw_priv = priv->hw_priv;	
    struct sta_info *sta = NULL;
    Sstar_wifi_ap_info *p_info;
    
    char *pdata = NULL;

    dev_printk("%s\n", __func__);
    
    if(msg == NULL){
        Sstar_printk_err("%s, error, pdata NULL \n",__func__);
        return -1;
    }
    if(!(p_info = (Sstar_wifi_ap_info *)Sstar_kzalloc(WSM_MAX_NUM_LINK_AP*sizeof(Sstar_wifi_ap_info), GFP_KERNEL))){
        Sstar_printk_err("%s, error, memory failed. \n",__func__);
        return -ENOMEM;
    }

    dev_printk("@@@ sta cnt %d\n", hw_priv->connected_sta_cnt);
    
    rcu_read_lock();
        
    list_for_each_entry_rcu(sta, &local->sta_list, list) {

        if(sta != NULL){
            if (sta->sdata->vif.type == NL80211_IFTYPE_AP){
                p_info[i].wext_rssi = (s8) -Sstar_ewma_read(&sta->avg_signal);
                p_info[i].rx_packets = sta->rx_packets;
                p_info[i].tx_packets = sta->tx_packets;
                p_info[i].tx_retry_count = sta->tx_retry_count;
                p_info[i].last_rx_rate_idx = sta->last_rx_rate_idx;
                
                memcpy(p_info[i].wext_mac, sta->sta.addr, ETH_ALEN);
                p_info[i].sta_cnt = hw_priv->connected_sta_cnt;
            
                dev_printk("%d: MAC "DEV_MACSTR"\n", i, DEV_MAC2STR(p_info[i].wext_mac));
                dev_printk("    RSSI %d\n", p_info[i].wext_rssi);
                dev_printk("    RX Pkts %ld\n", p_info[i].rx_packets);
                dev_printk("    TX Pkts %ld\n", p_info[i].tx_packets);
                dev_printk("    TX Retry %ld\n", p_info[i].tx_retry_count);
                
                ++i;
            }else{
                msg->value = (s8) -Sstar_ewma_read(&sta->avg_signal);
                Sstar_printk_wext("# rssi %d\n", msg->value);
                break;
            }
        }

    }

    rcu_read_unlock();

    memcpy(&pdata, &msg->externData[0], 4);

    if(pdata != NULL){
        if (copy_to_user(pdata, p_info, WSM_MAX_NUM_LINK_AP*sizeof(Sstar_wifi_ap_info))){
            Sstar_printk_err("%s, error, copy_to_user\n", __func__);
            ret = -1;
        }
    }
    
    if(p_info != NULL)
        Sstar_kfree(p_info);
    
    return ret;
}

int Sstar_dev_get_sta_info(struct net_device *dev, struct altm_wext_msg *msg)
{
    int ret = 0;
    struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
    //struct ieee80211_bss_conf *p_bss_conf = &sdata->vif.bss_conf;
    struct ieee80211_local *local = sdata->local;
    struct Sstar_vif *priv =  (struct Sstar_vif *)sdata->vif.drv_priv;
    //struct Sstar_common *hw_priv = priv->hw_priv;	
    struct sta_info *sta = NULL;
    Sstar_wifi_sta_info *p_info;

    dev_printk("%s\n", __func__);

    if(msg == NULL){
        Sstar_printk_err("%s, error, pdata NULL \n",__func__);
        return -1;
    }

    if (priv->mode != NL80211_IFTYPE_STATION){
        Sstar_printk_err("%s, not station mode \n",__func__);
        return -1;
    }

    if(priv->join_status != SSTAR_APOLLO_JOIN_STATUS_STA){
        Sstar_printk_err("%s, not join to AP \n",__func__);
        return -1;
    }
    
    if(!(p_info = (Sstar_wifi_sta_info *)Sstar_kzalloc(sizeof(Sstar_wifi_sta_info), GFP_KERNEL))){
        Sstar_printk_err("%s, error, memory failed. \n",__func__);
        return -ENOMEM;
    }

    rcu_read_lock();
    list_for_each_entry_rcu(sta, &local->sta_list, list) {

        if(sta != NULL){
            if (sta->sdata->vif.type == NL80211_IFTYPE_STATION){	
                //packets num
                struct ieee80211_if_managed *ifmgd = &sdata->u.mgd;
                struct cfg80211_bss *cbss;

                cbss = ifmgd->associated;

                if(cbss){
                    const char *ssid = NULL;

                    ssid = ieee80211_bss_get_ie(cbss, SSTAR_WLAN_EID_SSID);

                    if(ssid){						
                        memcpy(p_info->ssid, &ssid[2], ssid[1]);
                        p_info->ssid_len = ssid[1];
                    }
                }
                memcpy(p_info->bssid, sta->sta.addr, ETH_ALEN);
                p_info->rssi = (s8) -Sstar_ewma_read(&sta->avg_signal);
                p_info->rx_packets = sta->rx_packets;
                p_info->tx_packets = sta->tx_packets;
                p_info->tx_retry_count = sta->tx_retry_count;
                break;
            }
        }

    }
    rcu_read_unlock();

    dev_printk("    SSID %s\n", p_info->ssid);
    dev_printk("    BSSID "DEV_MACSTR"\n", DEV_MAC2STR(p_info->bssid));
    dev_printk("    RSSI %d\n", p_info->rssi);
    dev_printk("    RX Pkts %ld\n", p_info->rx_packets);
    dev_printk("    TX Pkts %ld\n", p_info->tx_packets);
    dev_printk("    TX Retry %ld\n", p_info->tx_retry_count);

    memcpy((u8*)msg->externData, p_info, sizeof(Sstar_wifi_sta_info));

    if(p_info != NULL)
        Sstar_kfree(p_info);

    return ret;
}
static bool Sstar_dev_handle_scan_sta(struct ieee80211_hw *hw,struct Sstar_internal_scan_results_req *req,
											   struct ieee80211_internal_scan_sta *sta)
{
	Wifi_Recv_Info_t *info     = (Wifi_Recv_Info_t *)req->priv;
	Wifi_Recv_Info_t *pos_info = NULL;
	
	if(req->n_stas >= MAC_FILTER_NUM){
		Sstar_printk_err("%s: n_stas(%d)\n",__func__,req->n_stas);
		return false;
	}

	pos_info = info+req->n_stas;
	req->n_stas ++;
	pos_info->channel = sta->channel;
	pos_info->Rssi = sta->signal;
	memcpy(pos_info->Bssid,sta->bssid,6);
	if(sta->ssid_len && sta->ssid)
		memcpy(pos_info->Ssid,sta->ssid,sta->ssid_len);
	return true;
}

int Sstar_dev_set_sta_scan(struct net_device *dev, struct altm_wext_msg *msg)
{
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	char zero_mac[ETH_ALEN] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	unsigned short channel;
	int ret = 0;
	struct ieee80211_internal_scan_request internal_scan;
	struct Sstar_internal_scan_results_req req;
	u8 *user_pos = NULL;
	u8 *recv_info = NULL;
	u8 scan_ch = 0;
	
	memset(&internal_scan,0,sizeof(struct ieee80211_internal_scan_request));
	
	if(!ieee80211_sdata_running(sdata)){
		Sstar_printk_err("%s:[%s] not running\n",__func__,sdata->name);
		ret = -ENETDOWN;
		goto exit;
	}

	if(sdata->vif.type != NL80211_IFTYPE_STATION){
        Sstar_printk_err("%s:[%s] not support scan\n",__func__,sdata->name);
		ret = -EOPNOTSUPP;
		goto exit;  
    }
	recv_info = Sstar_kzalloc(sizeof(Wifi_Recv_Info_t)*MAC_FILTER_NUM, GFP_KERNEL);

	if(recv_info == NULL){
		ret =  -ENOMEM;
		goto exit;
	}
	
	memcpy(&channel, &msg->externData[0], sizeof(channel));
	scan_ch = (u8)channel;
	
	if(memcmp(&msg->externData[2], zero_mac, ETH_ALEN) != 0){
		u8* mac = &msg->externData[2];
		u8 index = 0;
	
        internal_scan.macs = Sstar_kzalloc(sizeof(struct ieee80211_internal_mac)*MAC_FILTER_NUM, GFP_KERNEL);
		
		if(internal_scan.macs == NULL){
			ret = -ENOMEM;
			goto exit;
		}

		for(index = 0;index<MAC_FILTER_NUM;index++){
			memcpy(internal_scan.macs[index].mac,&mac[6*index],6);
		}
    }

	internal_scan.channels = &scan_ch;
	internal_scan.n_channels = 1;

	if(Sstar_internal_cmd_scan_triger(sdata,&internal_scan) == false){
		ret =  -EOPNOTSUPP;
		goto exit;
    }

	memcpy(&user_pos, &msg->externData[50], sizeof(void*));

	req.flush = true;
	req.n_stas = 0;
	req.priv = recv_info;
	req.result_handle = Sstar_dev_handle_scan_sta;

	ieee80211_scan_internal_req_results(sdata->local,&req);

	if(copy_to_user(user_pos, recv_info, MAC_FILTER_NUM*sizeof(Wifi_Recv_Info_t)) != 0){
		ret = -EINVAL;
	}
exit:

	if(internal_scan.macs)
		Sstar_kfree(internal_scan.macs);
	if(recv_info){
		Sstar_kfree(recv_info);
	}
	return ret;
}
int Sstar_dev_set_freq(struct net_device *dev, struct altm_wext_msg *msg)
{
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_internal_set_freq_req req;
	int ret = 0;
	unsigned short channel = 0;
	int freq;
	
	if(!ieee80211_sdata_running(sdata)){
		Sstar_printk_err("%s:[%s] not running\n",__func__,sdata->name);
		ret = -ENETDOWN;
		goto exit;
	}

	memcpy(&channel, &msg->externData[0], sizeof(unsigned short));
	memcpy(&freq, &msg->externData[2], sizeof(int));

	req.channel_num = channel;
	req.freq = freq;
	req.set = true;

	if(Sstar_internal_freq_set(&sdata->local->hw,&req)==false){
		ret =  -EINVAL;
	}
exit:
	return ret;
}
int Sstar_dev_set_special_oui(struct net_device *dev, struct altm_wext_msg *msg)
{
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	struct ieee80211_sub_if_data *sdata_update;
	
	int ret = 0;
	char *special = NULL;
	int len = 0;
	
	if(!ieee80211_sdata_running(sdata)){
		Sstar_printk_err("%s:[%s] not running\n",__func__,sdata->name);
		ret = -ENETDOWN;
		goto exit;
	}

	len =strlen(msg->externData);

	if((len<0)||(len>255)){
		Sstar_printk_err("%s:[%s] ie is err (%d)\n",__func__,sdata->name,len);
		ret = -EINVAL;
		goto exit;
	}

	special = Sstar_kzalloc(len, GFP_KERNEL);

	if(special == NULL){
		Sstar_printk_err("%s:[%s] special is not alloc\n",__func__,sdata->name);
		ret = -EINVAL;
		goto exit;
	}
	memcpy(special,msg->externData, len);

	list_for_each_entry(sdata_update, &local->interfaces, list){
		bool res = true;
		
		if(!ieee80211_sdata_running(sdata_update)){
			continue;
		}

		if(sdata_update->vif.type == NL80211_IFTYPE_STATION){
			res = ieee80211_ap_update_special_probe_request(sdata_update,special,len);
		}else if((sdata_update->vif.type == NL80211_IFTYPE_AP)&&
		         (rtnl_dereference(sdata_update->u.ap.beacon))){
		    res = ieee80211_ap_update_special_beacon(sdata_update,special,len);
			if(res == true){
				res = ieee80211_ap_update_special_probe_response(sdata_update,special,len);
			}
		}
		if(res == false){
			ret = -EOPNOTSUPP;
			goto exit;
		}
	}
exit:
	if(special)
		Sstar_kfree(special);
	return ret;
}

int Sstar_dev_set_sta_dis(struct net_device *dev, struct altm_wext_msg *msg)
{
    struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
    struct Sstar_vif *priv = (struct Sstar_vif *)sdata->vif.drv_priv;
    
    dev_printk("%s\n", __func__);

    ieee80211_connection_loss(priv->vif);
    return 0;
}

int Sstar_dev_set_monitor(struct net_device *dev, struct altm_wext_msg *msg)
{
    struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
    struct ieee80211_local *local = sdata->local;

    char ch_type = 0;
    int freq = 0;
    struct ieee80211_channel *ch;
    
    dev_printk("%s\n", __func__);

    memcpy(&ch_type, &msg->externData[0], sizeof(char));
    dev_printk("channel type: %d\n", ch_type);
    /****
    enum	nl80211_channel_type { NL80211_CHAN_NO_HT, NL80211_CHAN_HT20, NL80211_CHAN_HT40MINUS, NL80211_CHAN_HT40PLUS }
    *****/
    switch(ch_type){
        case NL80211_CHAN_NO_HT:
        case NL80211_CHAN_HT20:
        case NL80211_CHAN_HT40MINUS:
        case NL80211_CHAN_HT40PLUS:
            //good~
            break;
        default:
            Sstar_printk_err("error, %d\n", ch_type);
            return -1;
    }
    
    memcpy(&freq, &msg->externData[1], sizeof(int));
    dev_printk("freq: %d\n", freq);
    
    ch = ieee80211_get_channel(local->hw.wiphy, freq);
    if(ch != NULL){
        ieee80211_set_channel(local->hw.wiphy, dev, ch, ch_type);
    }else{
        Sstar_printk_err("%s, error\n", __func__);
        return -1;
    }
    
    return Sstar_change_iface_to_monitor(dev);
}
int Sstar_dev_set_adaptive(struct net_device *dev, struct altm_wext_msg *msg)
{
    int ret = 0;
    struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
    struct Sstar_vif *priv = (struct Sstar_vif *)sdata->vif.drv_priv;
    struct Sstar_common *hw_priv = priv->hw_priv;

    int adaptive;
    char cmd[32];

    dev_printk("%s\n", __func__);

    /*
    0: disable
    1: enable
    */
    //adaptive = msg->value;
    memcpy(&adaptive, msg->externData, sizeof(int));

    memset(cmd, 0, sizeof(cmd));
    sprintf(cmd, "set_adaptive %d ", adaptive);
    
    dev_printk("Sstar: %s\n", cmd);
    ret = wsm_write_mib(hw_priv, WSM_MIB_ID_FW_CMD, cmd, strlen(cmd), priv->if_id);
    if(ret < 0){
        Sstar_printk_err("%s: write mib failed(%d). \n",__func__, ret);
    }

    return ret;

}
int Sstar_dev_set_txpwr_dcxo(struct net_device *dev, struct altm_wext_msg *msg)
{
    int ret = 0;
    struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
    struct Sstar_vif *priv = (struct Sstar_vif *)sdata->vif.drv_priv;
    struct Sstar_common *hw_priv = priv->hw_priv;

    int txpwr_L, txpwr_M, txpwr_H;
    int dcxo;
    char cmd[32];

    dev_printk("%s\n", __func__);
    
    memcpy(&txpwr_L, &msg->externData[0], 4);/*detla gain & txpwr*/
    if(txpwr_L > 32 || txpwr_L < -32){
        Sstar_printk_err("error, txpwr_L %d\n", txpwr_L);
        return -1;
    }
    
    memcpy(&txpwr_M, &msg->externData[4], 4);/*detla gain & txpwr*/
    if(txpwr_M > 32 || txpwr_M < -32){
        Sstar_printk_err("error, txpwr_M %d\n", txpwr_M);
        return -1;
    }	

    memcpy(&txpwr_H, &msg->externData[8], 4);/*detla gain & txpwr*/
    if(txpwr_H > 32 || txpwr_H < -32){
        Sstar_printk_err("error, txpwr_H %d\n", txpwr_H);
        return -1;
    }

    memcpy(&dcxo, &msg->externData[12], 4);
    if(dcxo > 127 || dcxo < 0){
        Sstar_printk_err("error, dcxo %d\n", dcxo);
        return -1;
    }
    
    memset(cmd, 0, sizeof(cmd));
    sprintf(cmd, "set_txpwr_and_dcxo,%d,%d,%d,%d ", txpwr_L, txpwr_M, txpwr_H, dcxo);
    
    dev_printk("Sstar: %s\n", cmd);
    ret = wsm_write_mib(hw_priv, WSM_MIB_ID_FW_CMD, cmd, strlen(cmd), priv->if_id);
    if(ret < 0){
        Sstar_printk_err("%s: write mib failed(%d). \n",__func__, ret);
    }

    return ret;

}

int Sstar_dev_set_txpwr(struct net_device *dev, struct altm_wext_msg *msg)
{
    int ret = 0;
    struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
    struct Sstar_vif *priv = (struct Sstar_vif *)sdata->vif.drv_priv;
    struct Sstar_common *hw_priv = priv->hw_priv;

    int txpwr_indx = 0;
    int txpwr = 0;
    char cmd[32];

    dev_printk("%s\n", __func__);
    
    memcpy(&txpwr_indx, &msg->externData[0], sizeof(int));
    if(txpwr_indx != 0 && txpwr_indx != 1){
        Sstar_printk_err("error, txpwr_indx %d\n", txpwr_indx);
        return -1;
    }

    /*
    *bit0 (0:20M Low, 1:20M High)
    *bit1 (0:40M Low, 1:40M High)
    */
    if(txpwr_indx == 1)
        txpwr |= BIT(0) | BIT(1);
    
    // -- # cat sys/module/Sstar_wifi/parameters/wifi_txpw
    Sstar_set_tx_power(hw_priv, txpwr);
    
    memset(cmd, 0, sizeof(cmd));

    //Convert to lmac value, 0xF is 20M and 40M high tx power.
    if(txpwr_indx == 1)
        txpwr_indx = 0x0F;
    
    sprintf(cmd, "set_rate_txpower_mode %d ", txpwr_indx);
    
    dev_printk("Sstar: %s\n", cmd);
    ret = wsm_write_mib(hw_priv, WSM_MIB_ID_FW_CMD, cmd, strlen(cmd), priv->if_id);
    if(ret < 0){
        Sstar_printk_err("%s: write mib failed(%d). \n",__func__, ret);
    }

    return ret;

}
int Sstar_dev_get_work_channel(struct net_device *dev, struct altm_wext_msg *msg)
{
    int ret = 0;
	unsigned short channel = 0;
    struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
    struct Sstar_vif *priv = (struct Sstar_vif *)sdata->vif.drv_priv;
    struct Sstar_common *hw_priv = priv->hw_priv;
    dev_printk("%s\n", __func__);

    //msg->value = hw_priv->channel->hw_value;
    channel = channel_hw_value(hw_priv->channel);
    memcpy(&msg->externData[0], &channel, sizeof(unsigned short));
    
    return ret;
}

int Sstar_dev_set_best_channel_scan(struct net_device *dev, struct altm_wext_msg *msg)
{
	int ret = 0;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_internal_channel_auto_select_req req;
	struct ieee80211_internal_channel_auto_select_results results;
	Best_Channel_Scan_Result scan_result;
	int i = 0;
	
	if(!ieee80211_sdata_running(sdata)){
		Sstar_printk_err("%s:[%s] not running\n",__func__,sdata->name);
		ret = -ENETDOWN;
		goto exit;
	}

	if(sdata->vif.type != NL80211_IFTYPE_STATION){
		Sstar_printk_err("%s:[%s] not support scan\n",__func__,sdata->name);
		ret = -EOPNOTSUPP;
		goto exit;
	}
	memset(&req,0,sizeof(struct ieee80211_internal_channel_auto_select_req));

	if(Sstar_internal_channel_auto_select(sdata,&req) == false){
		ret = -EOPNOTSUPP;
	}
	
	memset(&scan_result,0,sizeof(Best_Channel_Scan_Result));
	memset(&results,0,sizeof(struct ieee80211_internal_channel_auto_select_results));
	results.version = 0;//use version 0
	
	if(Sstar_internal_channel_auto_select_results(sdata,&results) == false){
		Sstar_printk_err("%s:channel results err\n",__func__);
		ret = -EINVAL;
		goto exit;
	}

	for(i = 0;i<CHANNEL_NUM;i++){
		scan_result.channel_ap_num[i] = results.n_aps[i];
	}
	for(i = 0;i<CHANNEL_NUM;i++){
		scan_result.busy_ratio[i] = results.busy_ratio[i];
	}
	scan_result.suggest_ch = results.susgest_channel;
	memcpy(msg->externData, &scan_result, sizeof(scan_result));
exit:
	return ret;
}
static bool Sstar_dev_handle_ap_list(struct ieee80211_hw *hw,struct Sstar_internal_scan_results_req *req,
											   struct ieee80211_internal_scan_sta *sta)
{
	BEST_CHANNEL_INFO *info     = (BEST_CHANNEL_INFO *)req->priv;
	BEST_CHANNEL_INFO *pos_info = NULL;
	int i = 0;
	
	if(req->n_stas >= CHANNEL_NUM*AP_SCAN_NUM_MAX){
		Sstar_printk_err("%s: n_stas(%d)\n",__func__,req->n_stas);
		return false;
	}

	if(sta->channel > 14){
		Sstar_printk_err("%s: channel not support(%d)\n",__func__,sta->channel);
		return false;
	}
	
	pos_info = info+(sta->channel -1)*AP_SCAN_NUM_MAX;

	for(i = 0;i<AP_SCAN_NUM_MAX;i++){
		
		if(pos_info->flag == 1){
			pos_info++;
			continue;
		}

		req->n_stas ++;
		pos_info->enc_type = (u8)sta->enc_type;
		pos_info->rssi = sta->signal;	
		memcpy(pos_info->mac_addr,sta->bssid,6);
		if(sta->ssid_len && sta->ssid)
			memcpy(pos_info->ssid,sta->ssid,sta->ssid_len);
		pos_info->flag = 1;

		break;
	}

	return true;
}

int Sstar_dev_get_ap_list(struct net_device *dev, struct altm_wext_msg *msg)
{
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	int ret = 0;
	struct ieee80211_internal_scan_request internal_scan;
	struct Sstar_internal_scan_results_req req;
	u8 *user_pos = NULL;
	u8 *recv_info = NULL;

	memset(&internal_scan,0,sizeof(struct ieee80211_internal_scan_request));
	
	if(!ieee80211_sdata_running(sdata)){
		Sstar_printk_err("%s:[%s] not running\n",__func__,sdata->name);
		ret = -ENETDOWN;
		goto exit;
	}

	if(sdata->vif.type != NL80211_IFTYPE_STATION){
        Sstar_printk_err("%s:[%s] not support scan\n",__func__,sdata->name);
		ret = -EOPNOTSUPP;
		goto exit;  
    }
	recv_info = Sstar_kzalloc(sizeof(BEST_CHANNEL_INFO)*CHANNEL_NUM*AP_SCAN_NUM_MAX, GFP_KERNEL);
	if(recv_info == NULL){
		ret =  -ENOMEM;
		goto exit;
	}

	if(Sstar_internal_cmd_scan_triger(sdata,&internal_scan) == false){
		ret =  -EOPNOTSUPP;
		goto exit;
    }

	memcpy(&user_pos, &msg->externData[0], sizeof(void*));

	req.flush = true;
	req.n_stas = 0;
	req.priv = recv_info;
	req.result_handle = Sstar_dev_handle_ap_list;

	ieee80211_scan_internal_req_results(sdata->local,&req);

	if(copy_to_user(user_pos, recv_info, sizeof(BEST_CHANNEL_INFO)*CHANNEL_NUM*AP_SCAN_NUM_MAX) != 0){
		ret = -EINVAL;
	}
exit:

	if(internal_scan.macs)
		Sstar_kfree(internal_scan.macs);
	if(recv_info){
		Sstar_kfree(recv_info);
	}
	return ret;
}
int Sstar_dev_get_tp_rate(struct net_device *dev, struct altm_wext_msg *msg)
{
    int ret = 0;
    struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
    struct Sstar_vif *priv = (struct Sstar_vif *)sdata->vif.drv_priv;
    struct Sstar_common *hw_priv = priv->hw_priv;
    char mac_addr[6];
    int sta_id = 0;
    unsigned int rate_val = 0;
    
    dev_printk("%s\n", __func__);

    //ap mode
    if(sdata->vif.type != NL80211_IFTYPE_STATION){
        //clear mac addr buffer
        memset(mac_addr, 0, 6);
        
        //convert mac string to mac hex format
        str2mac(mac_addr, msg->externData);
        
        //according to mac hex, find out sta link id
        sta_id = Sstar_find_link_id(priv, mac_addr);

        Sstar_printk_wext("sta_id %d\n", sta_id);
        wsm_write_mib(hw_priv, WSM_MIB_ID_GET_RATE, &sta_id, 1, priv->if_id);
    }
    
    wsm_read_mib(hw_priv, WSM_MIB_ID_GET_RATE, &rate_val, sizeof(unsigned int), priv->if_id);

    //convert bits/s
    rate_val = rate_val/2;
    Sstar_printk_wext("rate: %d bits/s\n", rate_val);

    memcpy(&msg->externData[0], &rate_val, sizeof(unsigned int));

    return ret;
}
int Sstar_dev_etf_test(struct net_device *dev, struct altm_wext_msg *msg)
{
    int i =0 ;
    struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
    struct Sstar_vif *priv = (struct Sstar_vif *)sdata->vif.drv_priv;
    struct Sstar_common *hw_priv = ABwifi_vifpriv_to_hwpriv(priv);

    etf_param_init(hw_priv);

	ucWriteEfuseFlag = msg->value;
	Sstar_printk_always("ucWriteEfuseFlag:%d\n", ucWriteEfuseFlag);

    gthreshold_param.freq_ppm = 7000;
    gthreshold_param.rxevm = 400;
    gthreshold_param.rssifilter = -100;
    gthreshold_param.txevm = 400;
    gthreshold_param.txevmthreshold = 400;
    gthreshold_param.rxevmthreshold = 400;
    gthreshold_param.cableloss = 30*4;
    gthreshold_param.featureid = MyRand();

	Sstar_printk_always("featureid:%d\n", gthreshold_param.featureid);
	Sstar_printk_always("Freq:%d,txEvm:%d,rxEvm:%d,txevmthreshold:%d,rxevmthreshold:%d,Txpwrmax:%d,Txpwrmin:%d,Rxpwrmax:%d,Rxpwrmin:%d,rssifilter:%d,cableloss:%d,default_dcxo:%d\n",
		gthreshold_param.freq_ppm,gthreshold_param.txevm,gthreshold_param.rxevm,gthreshold_param.txevmthreshold,gthreshold_param.rxevmthreshold,
		gthreshold_param.txpwrmax,gthreshold_param.txpwrmin,gthreshold_param.rxpwrmax,
		gthreshold_param.rxpwrmin,gthreshold_param.rssifilter,gthreshold_param.cableloss,gthreshold_param.default_dcxo);

    hw_priv->etf_channel = 7;
    hw_priv->etf_channel_type = 0;
    hw_priv->etf_rate = 21;
    hw_priv->etf_len = 1000; 
    hw_priv->etf_greedfiled = 0;

    Sstar_for_each_vif(hw_priv,priv,i){
        if((priv != NULL))
        {
            Sstar_printk_wext("device ioctl etf test\n");
            down(&hw_priv->scan.lock);
            mutex_lock(&hw_priv->conf_mutex);
            //ETF_bStartTx = 1;
            wsm_start_tx_v2(hw_priv, priv->vif);
            
            mutex_unlock(&hw_priv->conf_mutex);
            break;
        }
    }

    return 0;
}

int Sstar_dev_etf_get_result(struct net_device *dev, struct altm_wext_msg *msg)
{
    int ret = 0;
    u8 chipid = 0;
    char *buff = NULL;

    chipid = chipversion;

	 if(!(buff = (char *)Sstar_kzalloc(256, GFP_KERNEL))){
        Sstar_printk_always("%s, error, memory failed. \n",__func__);
        return -ENOMEM;
    }
    memset(buff, 0, 256);

    sprintf(buff, "cfo:%d,txevm:%d,rxevm:%d,dcxo:%d,txrssi:%d,rxrssi:%d,result:%d (0:OK; -1:FreqOffset Error; -2:efuse hard error;"
        " -3:efuse no written; -4:efuse anaysis failed; -5:efuse full; -6:efuse version change; -7:rx null)",
    gRxs_s.Cfo,
    gRxs_s.txevm,
    gRxs_s.evm,
    gRxs_s.dcxo,
    gRxs_s.TxRSSI,
    gRxs_s.RxRSSI,
    gRxs_s.result
    );

	if(&msg->externData[0] != NULL)
	{
		memcpy(&msg->externData[0], buff, strlen(buff));
	}
	else
	{
		Sstar_printk_always("error:msg->externData[0] is null\n");
	}
    

	if(buff)
		Sstar_kfree(buff);

    
    return ret;
}


static int Sstar_dev_start_tx(struct net_device *dev, struct altm_wext_msg *msg)
{
	int i = 0;
	int ret = 0;
	int len = 0;
	int channel = 0;
	u32 rate = 0;
	u32 is_40M = 0;
	int band_value = 0;
	int greedfiled = 0;
	u8 ucDbgPrintOpenFlag = 1;
	char *extra = NULL;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	struct Sstar_common *hw_priv=local->hw.priv;
	struct Sstar_vif *vif;

	if(ETF_bStartTx || ETF_bStartRx){
		Sstar_printk_err("Error! already start_tx/send_singleTone, please stop_tx first!\n");
		return 0;
	}

	if(strlen(msg->externData) == 0)
	{
		Sstar_printk_err("Invalid parameters\n");
		return 0;
	}
	if(!(extra = Sstar_kmalloc(strlen(msg->externData), GFP_KERNEL)))
		return -ENOMEM;

	memcpy(extra, msg->externData, strlen(msg->externData));
	Sstar_printk_err("%s\n", extra);

	sscanf(extra, "%d %d %d %d %d", &channel, &band_value, &len, &is_40M, &greedfiled);

	//check rate 
		switch(band_value){
			case 10: rate = WSM_TRANSMIT_RATE_1;
				break;
			case 20: rate = WSM_TRANSMIT_RATE_2;
				break;
			case 55: rate = WSM_TRANSMIT_RATE_5;
				break;
			case 110: rate = WSM_TRANSMIT_RATE_11;
				break;
			case 60: rate = WSM_TRANSMIT_RATE_6;
				break;
			case 90: rate = WSM_TRANSMIT_RATE_9;
				break;
			case 120: rate = WSM_TRANSMIT_RATE_12;
				break;
			case 180: rate = WSM_TRANSMIT_RATE_18;
				break;
			case 240: rate = WSM_TRANSMIT_RATE_24;
				break;
			case 360: rate = WSM_TRANSMIT_RATE_36;
				break;
			case 480: rate = WSM_TRANSMIT_RATE_48;
				break;
			case 540: rate = WSM_TRANSMIT_RATE_54;
				break;
			case 65: rate = WSM_TRANSMIT_RATE_HT_6;
				break;
			case 130: rate = WSM_TRANSMIT_RATE_HT_13;
				break;
			case 195: rate = WSM_TRANSMIT_RATE_HT_19;
				break;
			case 260: rate = WSM_TRANSMIT_RATE_HT_26;
				break;
			case 390: rate = WSM_TRANSMIT_RATE_HT_39;
				break;
			case 520: rate = WSM_TRANSMIT_RATE_HT_52;
				break;
			case 585: rate = WSM_TRANSMIT_RATE_HT_58;
				break;
			case 650: rate = WSM_TRANSMIT_RATE_HT_65;
				break;
			default:
				{
					Sstar_printk_err("invalid rate!\n");
					ret = -EINVAL;
					goto exit;
				}
				
		}

	Sstar_printk_err("rate:%d\n", rate);
	
	if(is_40M == 1){
		is_40M = NL80211_CHAN_HT40PLUS;//
		channel -= 2;
	}

	Sstar_printk_wext("NL80211_CHAN_HT40PLUS:%d\n", NL80211_CHAN_HT40PLUS);

	//printk("%d, %d, %d, %d\n", channel, rate, len, is_40M);
	hw_priv->etf_channel = channel;
	hw_priv->etf_channel_type = is_40M;
	hw_priv->etf_rate = rate;
	hw_priv->etf_len = len; 
	hw_priv->etf_greedfiled = greedfiled;
	
	Sstar_for_each_vif(hw_priv,vif,i){
		if((vif != NULL)){
			Sstar_printk_wext("*******\n");

			down(&hw_priv->scan.lock);
			WARN_ON(wsm_write_mib(hw_priv, WSM_MIB_ID_DBG_PRINT_TO_HOST,
				&ucDbgPrintOpenFlag, sizeof(ucDbgPrintOpenFlag), vif->if_id));
			mutex_lock(&hw_priv->conf_mutex);
			
			ETF_bStartTx = 1;
			if(wsm_start_tx(hw_priv, vif->vif) != 0)
			{
				up(&hw_priv->scan.lock);
				Sstar_printk_err("%s:%d,wsm_start_tx error\n", __func__, __LINE__);
			}
			
			mutex_unlock(&hw_priv->conf_mutex);
			break;
		}
	}
exit:
	if(extra)
		Sstar_kfree(extra);
	return ret;
}

static int Sstar_dev_stop_tx(struct net_device *dev, struct altm_wext_msg *msg)
{
	int i = 0;
	int ret = 0;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	struct Sstar_common *hw_priv=local->hw.priv;
	struct Sstar_vif *vif;

	if(0 == ETF_bStartTx){
		Sstar_printk_err("please start start_rx first,then stop_tx\n");
		return -EINVAL;
	}
	
	mutex_lock(&hw_priv->conf_mutex);
	ETF_bStartTx = 0;
	mutex_unlock(&hw_priv->conf_mutex);
	
	Sstar_for_each_vif(hw_priv,vif,i){
		if((vif != NULL)){
			
			wsm_oper_unlock(hw_priv);
			wsm_stop_tx(hw_priv);
			wsm_stop_scan(hw_priv,i);
			up(&hw_priv->scan.lock);
		}
	}

	return ret;
}

static int Sstar_dev_start_rx(struct net_device *dev, struct altm_wext_msg *msg)
{
	int i = 0;
	int ret = 0;
	char cmd[20] = "monitor 1 ";
	u8 ucDbgPrintOpenFlag = 1;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	struct Sstar_common *hw_priv=local->hw.priv;
	struct Sstar_vif *vif;


	if(ETF_bStartTx || ETF_bStartRx){
		Sstar_printk_err("Error! already ETF_bStartRx/ETF_bStartTx/send_singleTone, please stop first!\n");
		return 0;
	}

	if(strlen(msg->externData) == 0)
	{
		Sstar_printk_err("Invalid parameters\n");
		return 0;
	}

	//./iwpriv wlan0 fwdbg 1
	Sstar_for_each_vif(hw_priv,vif,i){
		if (vif != NULL)
		{
			WARN_ON(wsm_write_mib(hw_priv, WSM_MIB_ID_DBG_PRINT_TO_HOST,
				&ucDbgPrintOpenFlag, sizeof(ucDbgPrintOpenFlag), vif->if_id));
			break;
		}
	}

	memcpy(cmd+10, msg->externData, strlen(msg->externData)+1);
	memset(ch_and_type, 0, 20);
	memcpy(ch_and_type, msg->externData, strlen(msg->externData)+1);
	
	Sstar_printk_wext("CMD:%s\n", cmd);
	i = 0;
	Sstar_for_each_vif(hw_priv,vif,i){
		if (vif != NULL)
		{
			ETF_bStartRx = 1;
			WARN_ON(wsm_write_mib(hw_priv, WSM_MIB_ID_FW_CMD,
				cmd, strlen(cmd)+1, vif->if_id));
			break;
		}
	}	

	return ret;
}


static int Sstar_dev_stop_rx(struct net_device *dev, struct altm_wext_msg *msg)
{
	int i = 0;
	int ret = 0;
	char cmd[20] = "monitor 0 ";
	u8 ucDbgPrintOpenFlag = 0;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	struct Sstar_common *hw_priv=local->hw.priv;
	struct Sstar_vif *vif;

	if((0 == ETF_bStartRx) || (NULL == ch_and_type)){
		Sstar_printk_err("please start start_rx first,then stop_rx\n");
		return -EINVAL;
	}

	ETF_bStartRx = 0;
	
	memcpy(cmd+10, ch_and_type, strlen(ch_and_type));
	//printk("cmd %s\n", cmd);
	i = 0;
	Sstar_for_each_vif(hw_priv,vif,i){
		if (vif != NULL)
		{
			WARN_ON(wsm_write_mib(hw_priv, WSM_MIB_ID_FW_CMD,
				cmd, 13, vif->if_id));
			break;
		}
	}

	Sstar_for_each_vif(hw_priv,vif,i){
		if (vif != NULL)
		{
			WARN_ON(wsm_write_mib(hw_priv, WSM_MIB_ID_DBG_PRINT_TO_HOST,
				&ucDbgPrintOpenFlag, sizeof(ucDbgPrintOpenFlag), vif->if_id));
			break;
		}
	}
	return ret;
}

int Sstar_wext_cmd(struct net_device *dev, void *data, int len)
{
    int ret = 0;
    struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
    struct ieee80211_local *local = sdata->local;
    struct Sstar_vif *priv = (struct Sstar_vif *)sdata->vif.drv_priv;
    struct ieee80211_hw *hw = &local->hw;

    struct Sstar_common *hw_priv = priv->hw_priv;
    struct altm_wext_msg *msg = NULL;

    if(atomic_read(&priv->enabled)==0){
        Sstar_printk_err("Sstar_wext_cmd() priv is disabled\n");
        ret = -1;
        goto __Exit__;
    }

    if(hw_priv == NULL){
        Sstar_printk_err("error, netdev NULL\n");
        ret = -1;
        goto __Exit__;
    }

    if(hw->vendcmd_nl80211 == 0)
    {
        struct nlattr *data_p = nla_find(data, len, SSTAR_TM_MSG_DATA);
        if (!data_p){
            ret = -1;
            goto __Exit__;
        }
        msg = (struct altm_wext_msg *)nla_data(data_p);
    }
    else
        msg = (struct altm_wext_msg *)data;
        
    dev_printk("cmd type: %d\n", msg->type);
    
    switch (msg->type)
    {
        case SSTAR_DEV_IO_GET_STA_STATUS:
            memcpy(msg->externData, &sdata->vif.bss_conf.assoc, 4);
            break;
        case SSTAR_DEV_IO_GET_STA_RSSI:
            {
                struct sta_info *sta = NULL;
                int rssi = 0;
                rcu_read_lock();
                list_for_each_entry_rcu(sta, &local->sta_list, list) {					
                    if (sta->sdata->vif.type == NL80211_IFTYPE_STATION){	
                        rssi = (s8) -Sstar_ewma_read(&sta->avg_signal);
                        memcpy(msg->externData, &rssi, 4);
                        break;
                    }
                }
                rcu_read_unlock();
            }
            break;
        case SSTAR_DEV_IO_GET_AP_INFO:
            Sstar_printk_wext("%s: get sta info(%d)\n",__func__, hw_priv->connected_sta_cnt);
            Sstar_dev_get_ap_info(dev, msg);
            break;
        case SSTAR_DEV_IO_GET_STA_INFO:
            Sstar_dev_get_sta_info(dev, msg);
            break;
        case SSTAR_DEV_IO_SET_STA_SCAN:
            Sstar_dev_set_sta_scan(dev, msg);
            break;
        case SSTAR_DEV_IO_SET_FREQ:
            Sstar_dev_set_freq(dev, msg);
            break;
        case SSTAR_DEV_IO_SET_SPECIAL_OUI:
            Sstar_dev_set_special_oui(dev, msg);
            break;
        case SSTAR_DEV_IO_SET_STA_DIS:
            Sstar_dev_set_sta_dis(dev, msg);
            break;
        case SSTAR_DEV_IO_SET_MONITOR:
            Sstar_dev_set_monitor(dev, msg);
            break;
        case SSTAR_DEV_IO_SET_ADAPTIVE:
            Sstar_dev_set_adaptive(dev, msg);
            break;
        case SSTAR_DEV_IO_SET_TXPWR_DCXO:
            Sstar_dev_set_txpwr_dcxo(dev, msg);
            break;
        case SSTAR_DEV_IO_SET_TXPWR:
            Sstar_dev_set_txpwr(dev, msg);
            break;
        case SSTAR_DEV_IO_GET_WORK_CHANNEL:
            Sstar_dev_get_work_channel(dev, msg);
            break;
        case SSTAR_DEV_IO_SET_BEST_CHANNEL_SCAN:
            Sstar_dev_set_best_channel_scan(dev, msg);
            break;
        case SSTAR_DEV_IO_GET_AP_LIST:
            Sstar_dev_get_ap_list(dev, msg);
            break;
        case SSTAR_DEV_IO_GET_TP_RATE:
            Sstar_dev_get_tp_rate(dev, msg);
			break;
		case SSTAR_DEV_IO_ETF_TEST:
            Sstar_dev_etf_test(dev, msg);
            break;
        case SSTAR_DEV_IO_ETF_GET_RESULT:
            Sstar_dev_etf_get_result(dev, msg);
            break;
		case SSTAR_DEV_IO_ETF_START_TX:
			Sstar_dev_start_tx(dev, msg);
			break;
		case SSTAR_DEV_IO_ETF_STOP_TX:
			Sstar_dev_stop_tx(dev, msg);
			break;
		case SSTAR_DEV_IO_ETF_START_RX:
			Sstar_dev_start_rx(dev, msg);
			break;
   		case SSTAR_DEV_IO_ETF_STOP_RX:
			Sstar_dev_stop_rx(dev, msg);
			break;
		case SSTAR_DEV_IO_FIX_TX_RATE:
		case SSTAR_DEV_IO_MAX_TX_RATE:
		case SSTAR_DEV_IO_TX_RATE_FREE:
		case SSTAR_DEV_IO_MIN_TX_RATE:
		{
			struct ieee80211_internal_rate_req req;
			req.flags = 0;
			req.rate  = 0;

			if(msg->type == SSTAR_DEV_IO_FIX_TX_RATE){
				req.flags |= IEEE80211_INTERNAL_RATE_FLAGS_SET_TX_RATE;
				req.flags |= IEEE80211_INTERNAL_RATE_FLAGS_CLEAE_TOP_RATE;
			}

			if(msg->type == SSTAR_DEV_IO_MAX_TX_RATE){
				req.flags |= IEEE80211_INTERNAL_RATE_FLAGS_SET_TOP_RATE;
				req.flags |= IEEE80211_INTERNAL_RATE_FLAGS_CLEAR_TX_RATE;
			}

			if(msg->type == SSTAR_DEV_IO_TX_RATE_FREE){
				req.flags |= IEEE80211_INTERNAL_RATE_FLAGS_CLEAE_TOP_RATE;
				req.flags |= IEEE80211_INTERNAL_RATE_FLAGS_CLEAR_TX_RATE;
			}

			if(msg->type == SSTAR_DEV_IO_MIN_TX_RATE){
				req.flags |= IEEE80211_INTERNAL_RATE_FLAGS_SET_MIN_RATE;
			}
			req.rate = msg->value;

			if(Sstar_internal_wsm_set_rate(hw_priv,&req) == false){
				Sstar_printk_err("%s:set rate err\n",__func__);
				ret = -EINVAL;
			}else {
				ret = 0;
			}
			break;
		}
		case SSTAR_DEV_IO_SET_RATE_POWER:
		{
			struct ieee80211_internal_rate_power_req power_req;

			memset(&power_req,0,sizeof(struct ieee80211_internal_rate_power_req));

			power_req.rate_index = msg->externData[0];
			power_req.power		 = msg->externData[1];

			if(Sstar_internal_wsm_set_rate_power(hw_priv,&power_req) == false){
				ret = -EINVAL;
			}else {
				ret = 0;
			}

			break;
		}
		case SSTAR_DEV_IO_SET_EFUSE_MAC:
		{
			u8 prv_mac[ETH_ALEN];

			memcpy(prv_mac,hw_priv->efuse.mac,ETH_ALEN);
			memcpy(hw_priv->efuse.mac,msg->externData,ETH_ALEN);

			if(Sstar_save_efuse(hw_priv,&hw_priv->efuse) != 0){
				memcpy(hw_priv->efuse.mac,prv_mac,ETH_ALEN);
				ret = -EINVAL;
			}

			break;
		}

		case SSTAR_DEV_IO_SET_EFUSE_DCXO:
		{
			u8 prv_dcxo = hw_priv->efuse.dcxo_trim;

			hw_priv->efuse.dcxo_trim = msg->externData[0];
			
			if(Sstar_save_efuse(hw_priv,&hw_priv->efuse) != 0){
				hw_priv->efuse.dcxo_trim = prv_dcxo;
				ret = -EINVAL;
			}

			break;
		}

		case SSTAR_DEV_IO_SET_EFUSE_DELTAGAIN:
		{
			u8 prv_deltagain1 = hw_priv->efuse.delta_gain1;
			u8 prv_deltagain2 = hw_priv->efuse.delta_gain2;
			u8 prv_deltagain3 = hw_priv->efuse.delta_gain3;

			hw_priv->efuse.delta_gain1 = msg->externData[0];
			hw_priv->efuse.delta_gain2 = msg->externData[1];
			hw_priv->efuse.delta_gain3 = msg->externData[2];

			if(Sstar_save_efuse(hw_priv,&hw_priv->efuse) != 0){
				hw_priv->efuse.delta_gain1 = prv_deltagain1;
				hw_priv->efuse.delta_gain2 = prv_deltagain2;
				hw_priv->efuse.delta_gain3 = prv_deltagain3;
				ret = -EINVAL;
			}

			break;
		}
        default:
            Sstar_printk_err("%s: not found. %d\n",__func__, msg->type);
            break;
    }

__Exit__:
    return ret;
}
