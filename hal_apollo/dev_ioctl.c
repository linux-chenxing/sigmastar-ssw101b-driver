
/*
 * Mac80211 STA API for altobeam APOLLO drivers
 *
 * Copyright (c) 2016, altobeam
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
//#ifdef CONFIG_ATBM_APOLLO_TESTMODE
#include "atbm_testmode.h"
#include <net/netlink.h>
//#endif /* CONFIG_ATBM_APOLLO_TESTMODE */

#include "net/atbm_mac80211.h"
#include "dbg_event.h"
#include "smartconfig.h"

#include "mac80211/ieee80211_i.h"
#include "mac80211/atbm_common.h"

#define ATBM_DEV_IOCTL_DEBUG 1

#if ATBM_DEV_IOCTL_DEBUG
#define dev_printk(...) printk(__VA_ARGS__)
#else
#define dev_printk(...)
#endif

#define DEV_MAC2STR(a) (a)[0],(a)[1],(a)[2],(a)[3],(a)[4],(a)[5]
#define DEV_MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"

static int rssi_val = 0;
int atbm_set_rssi_val(int val)
{
    rssi_val = val;
    return 0;
}



enum atbm_msg_type{
    ATBM_DEV_IO_GET_STA_STATUS   = 0,
    ATBM_DEV_IO_GET_STA_RSSI     = 1, //STA connected AP's RSSI
    ATBM_DEV_IO_GET_AP_INFO      = 2,  //STA or AP
    ATBM_DEV_IO_GET_STA_INFO     = 3,
    ATBM_DEV_IO_SET_STA_SCAN     = 4,
    ATBM_DEV_IO_SET_FREQ         = 5,
    ATBM_DEV_IO_SET_SPECIAL_OUI  = 6, //use for Beacon and Probe package
    ATBM_DEV_IO_SET_STA_DIS      = 7,
    ATBM_DEV_IO_SET_MONITOR      = 8,
    ATBM_DEV_IO_SET_ADAPTIVE     = 9,
    ATBM_DEV_IO_SET_TXPWR_DCXO   = 10,
    ATBM_DEV_IO_SET_TXPWR        = 11,
    ATBM_DEV_IO_GET_WORK_CHANNEL   = 12,
    ATBM_DEV_IO_SET_BEST_CHANNEL_SCAN   = 13,
    ATBM_DEV_IO_GET_AP_LIST   = 14,
    ATBM_DEV_IO_GET_TP_RATE   = 15,
    ATBM_DEV_IO_ETF_TEST   = 16,
    ATBM_DEV_IO_ETF_GET_RESULT   = 17,
};

#define WSM_MAX_NUM_LINK_AP 14//Lmac support station number is 4;
typedef struct _atbm_wifi_ap_info_{
    int wext_rssi;
    unsigned long rx_packets;
    unsigned long tx_packets;
    unsigned long tx_retry_count;
    int last_rx_rate_idx;
    unsigned char  wext_mac[ETH_ALEN];
    unsigned char  sta_cnt;
}atbm_wifi_ap_info;

typedef struct _atbm_wifi_sta_info_{
    int rssi;
    unsigned long rx_packets;
    unsigned long tx_packets;
    unsigned long tx_retry_count;
    u8  bssid[ETH_ALEN];
    u8  ssid[IEEE80211_MAX_SSID_LEN];
    size_t ssid_len;
}atbm_wifi_sta_info;


struct altm_wext_msg{
    int type;
    int value;
    char externData[256];
};
#ifdef ATBM_PRIVATE_IE

extern ATBM_PRIVATE_DATA atbm_priv_data;
extern int not_filter;
extern unsigned short ch_filter;
extern unsigned char private_scan_sta; //1: processing 0:idle
extern unsigned char filter_mac[MAC_FILTER_NUM][ETH_ALEN];
extern int best_ch_scan_flag;
extern Best_Channel_Scan_Result scan_result;
extern unsigned int channel_ap_num[CHANNEL_NUM];
extern BEST_CHANNEL_INFO scan_ap_buff[CHANNEL_NUM][AP_SCAN_NUM_MAX];
extern SPECIAL_CH_FREQ spec_recd[CHANNEL_NUM];
extern Wifi_Ap_Info_t priv_recv_info[MAC_FILTER_NUM];

extern unsigned int atbm_wifi_status_get(void);
extern void atbm_private_scan(struct atbm_vif *priv, u16 channel);
extern int atbm_upload_beacon_private(struct atbm_vif *priv);
extern int atbm_change_iface_to_monitor(struct net_device *dev);
extern void ieee80211_connection_loss(struct ieee80211_vif *vif);
extern int ieee80211_set_channel(struct wiphy *wiphy,
                 struct net_device *netdev,
                 struct ieee80211_channel *chan,
                 enum nl80211_channel_type channel_type);
extern int atbm_ioctl_best_ch_start(struct net_device *dev, struct iw_request_info *info, void *wrqu, char *extra);
extern int atbm_ioctl_best_ch_end(struct net_device *dev, struct iw_request_info *info, void *wrqu, char *extra);
extern int atbm_ioctl_best_ch_result(struct net_device *dev, struct iw_request_info *info, void *wrqu, char *extra);
extern int atbm_find_link_id(struct atbm_vif *priv, const u8 *mac);
extern int str2mac(char *dst_mac, char *src_str);
extern void atbm_set_freq(struct atbm_common *hw_priv, SPECIAL_CH_FREQ *pdata);
extern void atbm_set_tx_power(struct atbm_common *hw_priv, int txpw);

static char spec_oui_buf[256];
static char *spec_oui = "NULL";
module_param(spec_oui,charp,0644);
MODULE_PARM_DESC(spec_oui,"special oui");
void atbm_set_special_oui(struct atbm_common *hw_priv, char *pdata, int len)
{
    memset(spec_oui_buf, 0, 256);
    memcpy(spec_oui_buf, pdata, len);
    spec_oui = spec_oui_buf;

    return;
}

int atbm_dev_get_ap_info(struct net_device *dev, struct altm_wext_msg *msg)
{
    int i = 0;
    int ret = 0;
    struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
    struct ieee80211_local *local = sdata->local;
    struct atbm_vif *priv =  (struct atbm_vif *)sdata->vif.drv_priv;
    struct atbm_common *hw_priv = priv->hw_priv;	
    struct sta_info *sta = NULL;
    atbm_wifi_ap_info *p_info;
    
    char *pdata = NULL;

    dev_printk("%s\n", __func__);
    
    if(msg == NULL){
        printk("%s, error, pdata NULL \n",__func__);
        return -1;
    }
    if(!(p_info = (atbm_wifi_ap_info *)atbm_kzalloc(WSM_MAX_NUM_LINK_AP*sizeof(atbm_wifi_ap_info), GFP_KERNEL))){
        printk("%s, error, memory failed. \n",__func__);
        return -ENOMEM;
    }

    dev_printk("@@@ sta cnt %d\n", hw_priv->connected_sta_cnt);
    
    rcu_read_lock();
        
    list_for_each_entry_rcu(sta, &local->sta_list, list) {

        if(sta != NULL){
            if (sta->sdata->vif.type == NL80211_IFTYPE_AP){
                p_info[i].wext_rssi = (s8) -atbm_ewma_read(&sta->avg_signal);
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
                msg->value = (s8) -atbm_ewma_read(&sta->avg_signal);
                printk("# rssi %d\n", msg->value);
                break;
            }
        }

    }

    rcu_read_unlock();

    memcpy(&pdata, &msg->externData[0], 4);

    if(pdata != NULL){
        if (copy_to_user(pdata, p_info, WSM_MAX_NUM_LINK_AP*sizeof(atbm_wifi_ap_info))){
            printk("%s, error, copy_to_user\n", __func__);
            ret = -1;
        }
    }
    
    if(p_info != NULL)
        atbm_kfree(p_info);
    
    return ret;
}

int atbm_dev_get_sta_info(struct net_device *dev, struct altm_wext_msg *msg)
{
    int ret = 0;
    struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
    //struct ieee80211_bss_conf *p_bss_conf = &sdata->vif.bss_conf;
    struct ieee80211_local *local = sdata->local;
    struct atbm_vif *priv =  (struct atbm_vif *)sdata->vif.drv_priv;
    //struct atbm_common *hw_priv = priv->hw_priv;	
    struct sta_info *sta = NULL;
    atbm_wifi_sta_info *p_info;

    dev_printk("%s\n", __func__);

    if(msg == NULL){
        printk("%s, error, pdata NULL \n",__func__);
        return -1;
    }

    if (priv->mode != NL80211_IFTYPE_STATION){
        printk("%s, not station mode \n",__func__);
        return -1;
    }

    if(priv->join_status != ATBM_APOLLO_JOIN_STATUS_STA){
        printk("%s, not join to AP \n",__func__);
        return -1;
    }
    
    if(!(p_info = (atbm_wifi_sta_info *)atbm_kzalloc(sizeof(atbm_wifi_sta_info), GFP_KERNEL))){
        printk("%s, error, memory failed. \n",__func__);
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

                    ssid = ieee80211_bss_get_ie(cbss, ATBM_WLAN_EID_SSID);

                    if(ssid){						
                        memcpy(p_info->ssid, &ssid[2], ssid[1]);
                        p_info->ssid_len = ssid[1];
                    }
                }
                memcpy(p_info->bssid, sta->sta.addr, ETH_ALEN);
                p_info->rssi = (s8) -atbm_ewma_read(&sta->avg_signal);
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

    memcpy((u8*)msg->externData, p_info, sizeof(atbm_wifi_sta_info));

    if(p_info != NULL)
        atbm_kfree(p_info);

    return ret;
}
int atbm_dev_set_sta_scan(struct net_device *dev, struct altm_wext_msg *msg)
{
    int i;
    int ret = 0;
    struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
    struct atbm_vif *priv = (struct atbm_vif *)sdata->vif.drv_priv;
    unsigned short channel;
    char zero_mac[ETH_ALEN] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    char *pdata = NULL;

    dev_printk("%s\n", __func__);

    if(sdata->vif.type != NL80211_IFTYPE_STATION){
        printk("%s: not sta mode\n", __func__);
        return -1;
    }
    
    memcpy(&channel, &msg->externData[0], sizeof(channel));
    if(channel < 0 || channel > 14){
        printk("%s: invalid ch %d\n",__func__, channel);
        return -1;
    }
    dev_printk("ch:%d\n", channel);

    //clear recv buffer
    memset(&priv_recv_info[0], 0, sizeof(Wifi_Ap_Info_t)*MAC_FILTER_NUM);
    
    private_scan_sta = 1;
    ch_filter = channel;
    
    if(memcmp(&msg->externData[2], zero_mac, ETH_ALEN) == 0){
        not_filter = 1;
    }else{
        memcpy(&filter_mac[0][0], &msg->externData[2], MAC_FILTER_NUM*ETH_ALEN);
        not_filter = 0;
    }
    
    atbm_private_scan(priv, channel);
    atbm_wait_scan_complete_sync(priv->hw_priv);	
    private_scan_sta = 0;
    
    memcpy(&pdata, &msg->externData[50], 4);
    if(pdata == NULL){
        printk("%s, pdata is null\n", __func__);
        return -1;
    }
    
    for(i=0; i<MAC_FILTER_NUM; i++){
        if(priv_recv_info[i].flag == 1){
            if (copy_to_user(pdata, &priv_recv_info[i].priv_recv_info, sizeof(Wifi_Recv_Info_t))){
                printk("%s,%d, error, copy_to_user\n", __func__, i);
                return -1;
            }

            pdata += sizeof(Wifi_Recv_Info_t);
        }
    }

    return ret;
}
int atbm_dev_set_freq(struct net_device *dev, struct altm_wext_msg *msg)
{
    int ret = 0;
    struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
    struct atbm_vif *priv = (struct atbm_vif *)sdata->vif.drv_priv;
    struct atbm_common *hw_priv = priv->hw_priv;

    unsigned short channel = 0;
    int freq_val = 0;
    char cmd[32];
    
    dev_printk("%s\n", __func__);
    
    memcpy(&channel, &msg->externData[0], sizeof(unsigned short));
    if(channel < 1 || freq_val > 14){
        printk("ch error: %d \n",channel);
        return -1;
    }
    
    memcpy(&freq_val, &msg->externData[2], sizeof(int));
    if(freq_val < 2300 && freq_val > 2600){
        printk("freq error: %d \n",freq_val);
        return -1;
    }
    
    memset(cmd, 0, sizeof(cmd));
    sprintf(cmd, "set_freq %d,%d ", channel, freq_val);
    
    dev_printk("atbm: %s\n", cmd);

    spec_recd[channel-1].special_freq = freq_val;
    spec_recd[channel-1].flag = 1;//valid freq

    printk("atbm: ch %d, freq %d\n", channel, spec_recd[channel-1].special_freq);

    // -- # cat sys/module/atbm_wifi/parameters/wifi_freq
    atbm_set_freq(hw_priv, spec_recd);
    
    ret = wsm_write_mib(hw_priv, WSM_MIB_ID_FW_CMD, cmd, strlen(cmd), priv->if_id);
    if(ret < 0){
        printk("%s: write mib failed(%d). \n",__func__, ret);
    }

    return ret;
}
int atbm_dev_set_special_oui(struct net_device *dev, struct altm_wext_msg *msg)
{
    int ret = 0;
    struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
    struct atbm_vif *priv = (struct atbm_vif *)sdata->vif.drv_priv;
    struct atbm_common *hw_priv = priv->hw_priv;
    int length = 0;

    dev_printk("%s\n", __func__);

    length = strlen(msg->externData);
    if(length < 0 || length > USER_DATE_LEN){
        printk("%s: length incorrect %d\n", __func__, length);
        return -1;
    }

    memset(atbm_priv_data.user_data, 0, USER_DATE_LEN+1);
    memcpy(atbm_priv_data.user_data, msg->externData, length);
    atbm_priv_data.data_len = length;
    atbm_priv_data.set_flag = 1;

    if(sdata->vif.type == NL80211_IFTYPE_AP){
        ret = atbm_upload_beacon_private(priv);
        if(ret < 0){
            printk("%s, upload beacon private failed %d\n", __func__, ret);
        }
    }

    // -- # cat sys/module/atbm_wifi/parameters/spec_oui
    atbm_set_special_oui(hw_priv, atbm_priv_data.user_data, atbm_priv_data.data_len);

    return ret;
}

int atbm_dev_set_sta_dis(struct net_device *dev, struct altm_wext_msg *msg)
{
    struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
    struct atbm_vif *priv = (struct atbm_vif *)sdata->vif.drv_priv;
    
    dev_printk("%s\n", __func__);

    ieee80211_connection_loss(priv->vif);
    return 0;
}

int atbm_dev_set_monitor(struct net_device *dev, struct altm_wext_msg *msg)
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
            printk("error, %d\n", ch_type);
            return -1;
    }
    
    memcpy(&freq, &msg->externData[1], sizeof(int));
    dev_printk("freq: %d\n", freq);
    
    ch = ieee80211_get_channel(local->hw.wiphy, freq);
    if(ch != NULL){
        ieee80211_set_channel(local->hw.wiphy, dev, ch, ch_type);
    }else{
        printk("%s, error\n", __func__);
        return -1;
    }
    
    return atbm_change_iface_to_monitor(dev);
}
int atbm_dev_set_adaptive(struct net_device *dev, struct altm_wext_msg *msg)
{
    int ret = 0;
    struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
    struct atbm_vif *priv = (struct atbm_vif *)sdata->vif.drv_priv;
    struct atbm_common *hw_priv = priv->hw_priv;

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
    
    dev_printk("atbm: %s\n", cmd);
    ret = wsm_write_mib(hw_priv, WSM_MIB_ID_FW_CMD, cmd, strlen(cmd), priv->if_id);
    if(ret < 0){
        printk("%s: write mib failed(%d). \n",__func__, ret);
    }

    return ret;

}
int atbm_dev_set_txpwr_dcxo(struct net_device *dev, struct altm_wext_msg *msg)
{
    int ret = 0;
    struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
    struct atbm_vif *priv = (struct atbm_vif *)sdata->vif.drv_priv;
    struct atbm_common *hw_priv = priv->hw_priv;

    int txpwr_L, txpwr_M, txpwr_H;
    int dcxo;
    char cmd[32];

    dev_printk("%s\n", __func__);
    
    memcpy(&txpwr_L, &msg->externData[0], 4);/*detla gain & txpwr*/
    if(txpwr_L > 32 || txpwr_L < -32){
        printk("error, txpwr_L %d\n", txpwr_L);
        return -1;
    }
    
    memcpy(&txpwr_M, &msg->externData[4], 4);/*detla gain & txpwr*/
    if(txpwr_M > 32 || txpwr_M < -32){
        printk("error, txpwr_M %d\n", txpwr_M);
        return -1;
    }	

    memcpy(&txpwr_H, &msg->externData[8], 4);/*detla gain & txpwr*/
    if(txpwr_H > 32 || txpwr_H < -32){
        printk("error, txpwr_H %d\n", txpwr_H);
        return -1;
    }

    memcpy(&dcxo, &msg->externData[12], 4);
    if(dcxo > 127 || dcxo < 0){
        printk("error, dcxo %d\n", dcxo);
        return -1;
    }
    
    memset(cmd, 0, sizeof(cmd));
    sprintf(cmd, "set_txpwr_and_dcxo,%d,%d,%d,%d ", txpwr_L, txpwr_M, txpwr_H, dcxo);
    
    dev_printk("atbm: %s\n", cmd);
    ret = wsm_write_mib(hw_priv, WSM_MIB_ID_FW_CMD, cmd, strlen(cmd), priv->if_id);
    if(ret < 0){
        printk("%s: write mib failed(%d). \n",__func__, ret);
    }

    return ret;

}

int atbm_dev_set_txpwr(struct net_device *dev, struct altm_wext_msg *msg)
{
    int ret = 0;
    struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
    struct atbm_vif *priv = (struct atbm_vif *)sdata->vif.drv_priv;
    struct atbm_common *hw_priv = priv->hw_priv;

    int txpwr_indx = 0;
    int txpwr = 0;
    char cmd[32];

    dev_printk("%s\n", __func__);
    
    memcpy(&txpwr_indx, &msg->externData[0], sizeof(int));
    if(txpwr_indx != 0 && txpwr_indx != 1){
        printk("error, txpwr_indx %d\n", txpwr_indx);
        return -1;
    }

    /*
    *bit0 (0:20M Low, 1:20M High)
    *bit1 (0:40M Low, 1:40M High)
    */
    if(txpwr_indx == 1)
        txpwr |= BIT(0) | BIT(1);
    
    // -- # cat sys/module/atbm_wifi/parameters/wifi_txpw
    atbm_set_tx_power(hw_priv, txpwr);
    
    memset(cmd, 0, sizeof(cmd));

    //Convert to lmac value, 0xF is 20M and 40M high tx power.
    if(txpwr_indx == 1)
        txpwr_indx = 0x0F;
    
    sprintf(cmd, "set_rate_txpower_mode %d ", txpwr_indx);
    
    dev_printk("atbm: %s\n", cmd);
    ret = wsm_write_mib(hw_priv, WSM_MIB_ID_FW_CMD, cmd, strlen(cmd), priv->if_id);
    if(ret < 0){
        printk("%s: write mib failed(%d). \n",__func__, ret);
    }

    return ret;

}
int atbm_dev_get_work_channel(struct net_device *dev, struct altm_wext_msg *msg)
{
    int ret = 0;
    struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
    struct atbm_vif *priv = (struct atbm_vif *)sdata->vif.drv_priv;
    struct atbm_common *hw_priv = priv->hw_priv;
    dev_printk("%s\n", __func__);

    //msg->value = hw_priv->channel->hw_value;
    memcpy(&msg->externData[0], &hw_priv->channel->hw_value, sizeof(unsigned short));
    
    return ret;
}

int atbm_dev_set_best_channel_scan(struct net_device *dev, struct altm_wext_msg *msg)
{
    int ret = 0;
    struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
    struct atbm_vif *priv = (struct atbm_vif *)sdata->vif.drv_priv;
    //struct atbm_common *hw_priv = priv->hw_priv;

    dev_printk("%s\n", __func__);
    
    if(sdata->vif.type != NL80211_IFTYPE_STATION){
        printk("%s: not sta mode\n", __func__);
        return -1;
    }
    
    atbm_ioctl_best_ch_start(dev, NULL, NULL, NULL);

    //scan start
    private_scan_sta = 1;

    //all channel sacn
    ch_filter = 0;

    //not do mac filter
    not_filter = 1;

    atbm_private_scan(priv, 0);
    atbm_wait_scan_complete_sync(priv->hw_priv);	

    //scan end
    private_scan_sta = 0;

    atbm_ioctl_best_ch_end(dev, NULL, NULL, NULL);

    memcpy(msg->externData, &scan_result, sizeof(scan_result));
    
    return ret;
}

int atbm_dev_get_ap_list(struct net_device *dev, struct altm_wext_msg *msg)
{
    int ret = 0;
    int ch, i;
    struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
    struct atbm_vif *priv = (struct atbm_vif *)sdata->vif.drv_priv;
    //struct atbm_common *hw_priv = priv->hw_priv;
    char *pdata = NULL;
    
    dev_printk("%s\n", __func__);

    if(best_ch_scan_flag != 0){
        printk("warning, scan is processing\n");
        return -1;
    }

    if(sdata->vif.type != NL80211_IFTYPE_STATION){
        printk("%s: not sta mode\n", __func__);
        return -1;
    }
    
    atbm_ioctl_best_ch_start(dev, NULL, NULL, NULL);

    //scan start
    private_scan_sta = 1;

    //all channel sacn
    ch_filter = 0;

    //not do mac filter
    not_filter = 1;

    atbm_private_scan(priv, 0);
    atbm_wait_scan_complete_sync(priv->hw_priv);	

    //scan end
    private_scan_sta = 0;

    atbm_ioctl_best_ch_end(dev, NULL, NULL, NULL);

    
    for(ch=0; ch<CHANNEL_NUM; ch++){
        printk("channel(%d) ap num: %d\n", ch+1, channel_ap_num[ch]);
        for(i=0; i<AP_SCAN_NUM_MAX; i++){
            if(scan_ap_buff[ch][i].flag != 0){
                printk("      ssid: %s\n", scan_ap_buff[ch][i].ssid);
                printk("      mac: "DEV_MACSTR"\n", DEV_MAC2STR(scan_ap_buff[ch][i].mac_addr));
                printk("      rssi: %d\n", scan_ap_buff[ch][i].rssi);
                printk("      channel: %d\n\n", ch+1);
            }
        }
    }
    
    memcpy(&pdata, &msg->externData[0], 4);
    if(pdata == NULL){
        printk("%s, pdata is null\n", __func__);
        return -1;
    }
    
    if (copy_to_user(pdata, &scan_ap_buff[0][0], sizeof(scan_ap_buff))){
        printk("%s, error, copy_to_user\n", __func__);
        return -1;
    }
    
    return ret;
}
int atbm_dev_get_tp_rate(struct net_device *dev, struct altm_wext_msg *msg)
{
    int ret = 0;
    struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
    struct atbm_vif *priv = (struct atbm_vif *)sdata->vif.drv_priv;
    struct atbm_common *hw_priv = priv->hw_priv;
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
        sta_id = atbm_find_link_id(priv, mac_addr);

        printk("sta_id %d\n", sta_id);
        wsm_write_mib(hw_priv, WSM_MIB_ID_GET_RATE, &sta_id, 1, priv->if_id);
    }
    
    wsm_read_mib(hw_priv, WSM_MIB_ID_GET_RATE, &rate_val, sizeof(unsigned int), priv->if_id);

    //convert bits/s
    rate_val = rate_val/2;
    printk("rate: %d bits/s\n", rate_val);

    memcpy(&msg->externData[0], &rate_val, sizeof(unsigned int));

    return ret;
}

int atbm_wext_cmd(struct net_device *dev, void *data, int len)
{
    int ret = 0;
    struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
    struct ieee80211_local *local = sdata->local;
    struct atbm_vif *priv = (struct atbm_vif *)sdata->vif.drv_priv;
    struct ieee80211_hw *hw = &local->hw;

    struct atbm_common *hw_priv = priv->hw_priv;
    struct altm_wext_msg *msg = NULL;

    if(atomic_read(&priv->enabled)==0){
        printk("atbm_wext_cmd() priv is disabled\n");
        ret = -1;
        goto __Exit__;
    }

    if(hw_priv == NULL){
        printk("error, netdev NULL\n");
        ret = -1;
        goto __Exit__;
    }

    if(hw->vendcmd_nl80211 == 0)
    {
        struct nlattr *data_p = nla_find(data, len, ATBM_TM_MSG_DATA);
        if (!data_p){
            ret = -1;
            goto __Exit__;
        }
        msg = (struct altm_wext_msg *)nla_data(data_p);
    }
    else
        msg = (struct altm_wext_msg *)data;
    
    #ifdef RESET_CHANGE
    if(atomic_read(&hw_priv->reset_flag))
    {
        printk(KERN_ERR "%s:hw_priv->reset_flag lock \n",__func__);
        ret = -1;
        goto __Exit__;
    }
    #endif
    
    dev_printk("cmd type: %d\n", msg->type);
    
    switch (msg->type)
    {
        case ATBM_DEV_IO_GET_STA_STATUS:
            memcpy(msg->externData, &sdata->vif.bss_conf.assoc, 4);
            break;
        case ATBM_DEV_IO_GET_STA_RSSI:
            {
                struct sta_info *sta = NULL;
                int rssi = 0;
                rcu_read_lock();
                list_for_each_entry_rcu(sta, &local->sta_list, list) {					
                    if (sta->sdata->vif.type == NL80211_IFTYPE_STATION){	
                        rssi = (s8) -atbm_ewma_read(&sta->avg_signal);
                        memcpy(msg->externData, &rssi, 4);
                        break;
                    }
                }
                rcu_read_unlock();
            }
            break;
        case ATBM_DEV_IO_GET_AP_INFO:
            printk("%s: get sta info(%d)\n",__func__, hw_priv->connected_sta_cnt);
            atbm_dev_get_ap_info(dev, msg);
            break;
        case ATBM_DEV_IO_GET_STA_INFO:
            atbm_dev_get_sta_info(dev, msg);
            break;
        case ATBM_DEV_IO_SET_STA_SCAN:
            atbm_dev_set_sta_scan(dev, msg);
            break;
        case ATBM_DEV_IO_SET_FREQ:
            atbm_dev_set_freq(dev, msg);
            break;
        case ATBM_DEV_IO_SET_SPECIAL_OUI:
            atbm_dev_set_special_oui(dev, msg);
            break;
        case ATBM_DEV_IO_SET_STA_DIS:
            atbm_dev_set_sta_dis(dev, msg);
            break;
        case ATBM_DEV_IO_SET_MONITOR:
            atbm_dev_set_monitor(dev, msg);
            break;
        case ATBM_DEV_IO_SET_ADAPTIVE:
            atbm_dev_set_adaptive(dev, msg);
            break;
        case ATBM_DEV_IO_SET_TXPWR_DCXO:
            atbm_dev_set_txpwr_dcxo(dev, msg);
            break;
        case ATBM_DEV_IO_SET_TXPWR:
            atbm_dev_set_txpwr(dev, msg);
            break;
        case ATBM_DEV_IO_GET_WORK_CHANNEL:
            atbm_dev_get_work_channel(dev, msg);
            break;
        case ATBM_DEV_IO_SET_BEST_CHANNEL_SCAN:
            atbm_dev_set_best_channel_scan(dev, msg);
            break;
        case ATBM_DEV_IO_GET_AP_LIST:
            atbm_dev_get_ap_list(dev, msg);
            break;
        case ATBM_DEV_IO_GET_TP_RATE:
            atbm_dev_get_tp_rate(dev, msg);
            break;
        default:
            printk("%s: not found. %d\n",__func__, msg->type);
            break;
    }

__Exit__:
    return ret;
}

#else
extern u32 chipversion;
extern struct rxstatus_signed gRxs_s;
extern struct test_threshold gthreshold_param;
extern void etf_param_init(struct atbm_common *hw_priv);
extern int wsm_start_tx_v2(struct atbm_common *hw_priv, struct ieee80211_vif *vif );

int atbm_dev_etf_test(struct net_device *dev, struct altm_wext_msg *msg)
{
    int i =0 ;
    struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
    struct atbm_vif *priv = (struct atbm_vif *)sdata->vif.drv_priv;
    struct atbm_common *hw_priv = ABwifi_vifpriv_to_hwpriv(priv);

    etf_param_init(hw_priv);

    gthreshold_param.freq_ppm = 7000;
    gthreshold_param.rxevm = 400;
    gthreshold_param.rssifilter = -100;
    gthreshold_param.txevm = 400;
    gthreshold_param.txevmthreshold = 400;
    gthreshold_param.rxevmthreshold = 400;
    gthreshold_param.cableloss = 30*4;
    gthreshold_param.featureid = 1234;

    hw_priv->etf_channel = 7;
    hw_priv->etf_channel_type = 0;
    hw_priv->etf_rate = 21;
    hw_priv->etf_len = 1000; 
    hw_priv->etf_greedfiled = 0;

    atbm_for_each_vif(hw_priv,priv,i){
        if((priv != NULL))
        {
            printk("device ioctl etf test\n");
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

int atbm_dev_etf_get_result(struct net_device *dev, struct altm_wext_msg *msg)
{
    int ret = 0;
    u8 chipid = 0;
    char buff[512];

    chipid = chipversion;
    memset(buff, 0, 512);

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

#if 0
    if (copy_to_user(&msg->externData[0], &buff[0], strlen(buff))){
		printk("%s, error, copy_to_user\n", __func__);
        return -1;
    }
#endif

    memcpy(&msg->externData[0], &buff[0], strlen(buff));

    
    return ret;
}


int atbm_wext_cmd(struct net_device *dev, void *data, int len)
{
    int ret = 0;
    struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
    struct ieee80211_local *local = sdata->local;
    struct atbm_vif *priv = (struct atbm_vif *)sdata->vif.drv_priv;
    struct ieee80211_hw *hw = &local->hw;

    struct atbm_common *hw_priv = priv->hw_priv;
    struct altm_wext_msg *msg = NULL;
    struct nlattr *data_p = NULL;

    if(atomic_read(&priv->enabled)==0){
        printk("atbm_wext_cmd() priv is disabled\n");
        ret = -1;
        goto __Exit__;
    }

    if(hw_priv == NULL){
        printk("error, netdev NULL\n");
        ret = -1;
        goto __Exit__;
    }

    if(hw->vendcmd_nl80211 == 0)
    {
        printk("++++++++++++++++++++++++\n");
        data_p = nla_find(data, len, ATBM_TM_MSG_DATA);
        if (!data_p){
            ret = -1;
            goto __Exit__;
        }
        msg = (struct altm_wext_msg *)nla_data(data_p);
        printk("hw->vendcmd_nl80211:%d\n", hw->vendcmd_nl80211);
    }
    else
    {
        printk("***********************\n");
        msg = (struct altm_wext_msg *)data;
    }
    
    dev_printk("cmd type: %d\n", msg->type);

    switch (msg->type)
    {
        case ATBM_DEV_IO_ETF_TEST:
            atbm_dev_etf_test(dev, msg);
            break;
        case ATBM_DEV_IO_ETF_GET_RESULT:
            atbm_dev_etf_get_result(dev, msg);
            break;
        default:
            break;
    }
__Exit__:

    printk("%s: ATBM_PRIVATE_IE is not defined.\n",__func__);
    return ret;
}
#endif
