/*
 * Copyright 2018-, altobeam.inc
 */

#include <net/atbm_mac80211.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/skbuff.h>
#include <linux/etherdevice.h>
#include <linux/if_arp.h>
#include <linux/rtnetlink.h>
#include <linux/bitmap.h>
#ifdef CONFIG_ATBM_PM_QOS
#include <linux/pm_qos.h>
#endif
#include <linux/inetdevice.h>
#include <net/net_namespace.h>
#include <net/cfg80211.h>
#ifdef CONFIG_WIRELESS_EXT
#include <net/iw_handler.h>
#endif
#ifdef IPV6_FILTERING
#include <net/if_inet6.h>
#include <net/addrconf.h>
#endif /*IPV6_FILTERING*/

#include "ieee80211_i.h"
#include "driver-ops.h"
#include "rate.h"
#include "mesh.h"
#include "wep.h"
#include "led.h"
#include "cfg.h"
#include "debugfs.h"
#include "../sbus.h"
#include "atbm_common.h"
#include "../apollo.h"
#include "../smartconfig.h"
#include "../wsm.h"
#include "../internal_cmd.h"


#define FREQ_CNT	(10)
#define DCXO_CODE_MINI		0//24//0
#define DCXO_CODE_MAX		127//38//63
#define TARGET_FREQOFFSET_HZ  (7000)

#define DCXO_TRIM_REG 0x1610100c //bit 5:0
#define CHIP_VERSION_REG 0x0acc017c //chip version reg address
#define HW_CHIP_VERSION_AthenaB (0x24)
#define HW_CHIP_VERSION_AresB (0x49)
#define N_BIT_TO_SIGNED_32BIT(v,n)	(s32)(((v) & BIT(n-1))?((v)|0xffffffff<<n):(v))

#define HW_CHIP_VERION_Athena_B 0x24
#define HW_CHIP_VERION_Ares_B 0x49


struct rxstatus{
	u32 GainImb;
	u32 PhaseImb;
	u32 Cfo;
	u32 evm;
	u32  RSSI;
	u32 probcnt;
};

u32 chipversion = 0;

u8 ETF_bStartTx = 0;
u8 ETF_bStartRx = 0;
char ch_and_type[20] = {0};


static u8 CodeStart = 0;
static u8 CodeEnd = 0;

u8 ucWriteEfuseFlag = 0;

int Atbm_Test_Success = 0;
int atbm_test_rx_cnt = 0;
int txevm_total = 0;

struct efuse_headr efuse_data_etf;
struct rxstatus_signed gRxs_s;
struct test_threshold gthreshold_param;

extern int wsm_start_tx_v2(struct atbm_common *hw_priv, struct ieee80211_vif *vif );
extern int wsm_send_result(struct atbm_common *hw_priv, struct ieee80211_vif *vif );


#ifdef CONFIG_WIRELESS_EXT

enum atbm_test{
		ALTM_RATE_SET_FIX = 1,
		ALTM_RATE_SET_FLAGS = 2,
		ALTM_RATE_SET_INDEX = 3,
		ALTM_RATE_GET_FIX = 4,
		ALTM_RATE_GET_FLAGS = 5,
		ALTM_RATE_GET_INDEX = 6,
};

struct altm_msg{
	unsigned int type;
	unsigned int value;
	unsigned int externData[30];
};

typedef struct bitrate{
	char *rate;
	int flags;
	int index;
	char *ratename;
}BITRATE;

BITRATE ratetab[] = {
	{"1000000", 0, 0, "1"},
	{"2000000", 0, 1, "2"},
	{"5500000", 0, 2, "5.5"},
	{"11000000", 0, 3, "11"},
	{"6000000", 0, 4, "6"},
	{"9000000", 0, 5, "9"},
	{"12000000", 0, 6, "12"},
	{"18000000", 0, 7, "18"},
	{"24000000", 0, 8, "24"},
	{"36000000", 0, 9, "36"},
	{"48000000", 0, 10, "48"},
	{"54000000", 0, 11, "54"},
	{"6500000", 8, 0, "6.5"},
	{"13000000", 8, 1, "13"},
	{"19500000", 8, 2, "19.5"},
	{"26000000", 8, 3, "26"},
	{"39000000", 8, 4, "39"},
	{"52000000", 8, 5, "52"},
	{"58500000", 8, 6, "58.5"},
	{"65000000", 8, 7, "65"},
	{"6000000",  8, 8,  "32"},
};


#define RATE_CONTROL_UNICAST 1

#define	arraysize(a)	sizeof(a)/sizeof(a[0])


extern int atbm_tool_rts_threshold;
extern int atbm_tool_shortGi;
extern struct tagAtbmRateControl g_atbm_rate_Ctl;

extern int atbm_tesmode_reply(struct wiphy *wiphy,const void *data, int len);
extern int rate_altm_control_test(struct wiphy *wiphy, void *data, int len);
extern void atbm_set_shortGI(u32 shortgi);
extern int wsm_start_tx(struct atbm_common *hw_priv, struct ieee80211_vif *vif);
extern int wsm_stop_tx(struct atbm_common *hw_priv);

int atbm_ioctl_etf_result_get(struct net_device *dev, struct iw_request_info *info, union iwreq_data  *wrqu, char *extra);
extern int atbm_upload_beacon_private(struct atbm_vif *priv);
extern int wsm_get_efuse_status(struct atbm_common *hw_priv, struct ieee80211_vif *vif );
u32 GetChipVersion(struct atbm_common *hw_priv);
u32 MyRand(void);
void get_test_threshold(char *param);
int Test_FreqOffset(struct atbm_common *hw_priv, u32 *dcxo, int *pfreqErrorHz, struct rxstatus_signed *rxs_s, int channel);

#ifdef ATBM_PRIVATE_IE
extern int atbm_upload_proberesp_private(struct atbm_vif *priv);
#endif

/*
static const u32 band_table[21] = {10, 20, 55, 110, 60, 90, 120,
								180, 240, 360, 480, 540, 65, 130,
								195, 260, 390, 520, 585, 650, 320};

*/

struct iw_handler_def atbm_handlers_def;
#define USELESS 0
static const struct iw_priv_args atbm_privtab[] = {
#if USELESS
		{SIOCIWFIRSTPRIV + 0x0, IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | 17, 0, "efuse_set_mac"},
		{SIOCIWFIRSTPRIV + 0x1, IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | 1, 0, "smartcfg_start"},	
#endif
		{SIOCIWFIRSTPRIV + 0x2, IW_PRIV_TYPE_CHAR | 1000, 0, "start_tx"},
		{SIOCIWFIRSTPRIV + 0x3, IW_PRIV_TYPE_CHAR | 10, 0, "stop_tx"},
		{SIOCIWFIRSTPRIV + 0x4, IW_PRIV_TYPE_CHAR | 50, 0, "start_rx"},
		{SIOCIWFIRSTPRIV + 0x5, IW_PRIV_TYPE_CHAR | 10, 0, "stop_rx"},
		{SIOCIWFIRSTPRIV + 0x6, IW_PRIV_TYPE_CHAR | 5, 0, "fwdbg"},
		{SIOCIWFIRSTPRIV + 0x7, IW_PRIV_TYPE_CHAR | 10, 0, "help"},
		{SIOCIWFIRSTPRIV + 0x8, IW_PRIV_TYPE_CHAR | 50, 0, "fwcmd"},
#if USELESS
		{SIOCIWFIRSTPRIV + 0x9, IW_PRIV_TYPE_CHAR | 10, 0, "rate"},
		{SIOCIWFIRSTPRIV + 0xa, IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | 1, 0, "smartcfg_stop"},
		{SIOCIWFIRSTPRIV + 0xb, IW_PRIV_TYPE_CHAR | 10, 0, "rts_threshold"},
		{SIOCIWFIRSTPRIV + 0xc, IW_PRIV_TYPE_CHAR | 10, 0, "set_gi"},
		{SIOCIWFIRSTPRIV + 0xd, IW_PRIV_TYPE_CHAR | 10, 0, "getmac"},
		{SIOCIWFIRSTPRIV + 0xe, IW_PRIV_TYPE_CHAR | 10, 0, "wolEn"},
		{SIOCIWFIRSTPRIV + 0xf, IW_PRIV_TYPE_CHAR | 16, 0, "get_rx_stats"},
#endif
		//#ifdef ATBM_PRIVATE_IE
		{SIOCIWFIRSTPRIV + 0x10, IW_PRIV_TYPE_CHAR | 500, 0, "common"},
		//#else
		//{SIOCIWFIRSTPRIV + 0x10, IW_PRIV_TYPE_CHAR | 500, 0, "common"},
		//#endif
#if USELESS
		{SIOCIWFIRSTPRIV + 0x11, IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | 1, 0, "smartonv2"},
#endif
		{SIOCIWFIRSTPRIV + 0x12, IW_PRIV_TYPE_CHAR | 16, 0, "etf_result_get"},
#if USELESS
		{SIOCIWFIRSTPRIV + 0x13, IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | 1, 0, "smartoffv2"},
#endif
		#ifdef ATBM_PRIVATE_IE
		/*Private IE for Scan*/
		{SIOCIWFIRSTPRIV + 0x14, IW_PRIV_TYPE_CHAR | 500, 0, "insert_data"},
		{SIOCIWFIRSTPRIV + 0x15, IW_PRIV_TYPE_CHAR | 500, 0, "get_data"},
		{SIOCIWFIRSTPRIV + 0x16, IW_PRIV_TYPE_CHAR | 500, 0, "send_msg"},
		{SIOCIWFIRSTPRIV + 0x17, IW_PRIV_TYPE_CHAR | 500, 0, "recv_msg"},
		{SIOCIWFIRSTPRIV + 0x18, IW_PRIV_TYPE_CHAR | 32, 0, "private_test"},
		#ifdef USE_HIDDEN_SSID
		{SIOCIWFIRSTPRIV + 0x19, IW_PRIV_TYPE_CHAR | 32, 0, "hide_ssid"},
		#else
		{SIOCIWFIRSTPRIV + 0x19, IW_PRIV_TYPE_CHAR | 32, 0, "get_state"},
		#endif
		{SIOCIWFIRSTPRIV + 0x1A, IW_PRIV_TYPE_CHAR | 32, 0, "set_freq"},
		{SIOCIWFIRSTPRIV + 0x1B, IW_PRIV_TYPE_CHAR | 32, 0, "set_txpower"},
		{SIOCIWFIRSTPRIV + 0x1C, IW_PRIV_TYPE_CHAR | 32, 0, "ipc_reset"},
		{SIOCIWFIRSTPRIV + 0x1D, IW_PRIV_TYPE_CHAR | 32, 0, "get_tp_rate"},
		{SIOCIWFIRSTPRIV + 0x1E, IW_PRIV_TYPE_CHAR | 32, 0, "best_ch_scan"},
		{SIOCIWFIRSTPRIV + 0x1F, IW_PRIV_TYPE_CHAR | 64, 0, "switch_ch"},

		#else
		#ifdef USE_HIDDEN_SSID
		{SIOCIWFIRSTPRIV + 0x14, IW_PRIV_TYPE_CHAR | 32, 0, "hide_ssid"},
		#else
		{SIOCIWFIRSTPRIV + 0x14, IW_PRIV_TYPE_CHAR | 32, 0, "get_state"},
		#endif
		{SIOCIWFIRSTPRIV + 0x15, IW_PRIV_TYPE_CHAR | 32, 0, "set_freq"},
		{SIOCIWFIRSTPRIV + 0x16, IW_PRIV_TYPE_CHAR | 32, 0, "set_txpower"},
		{SIOCIWFIRSTPRIV + 0x17, IW_PRIV_TYPE_CHAR | 32, 0, "get_tp_rate"},
		{SIOCIWFIRSTPRIV + 0x18, IW_PRIV_TYPE_CHAR | 32, 0, "best_ch_scan"},
#ifdef CONFIG_ATBM_STA_LISTEN
		{SIOCIWFIRSTPRIV + 0x19, IW_PRIV_TYPE_CHAR | 32, 0, "sta_channel"},
#endif
		#endif
};

static int atbm_ioctl_command_help(struct net_device * dev,struct iw_request_info * ifno,union iwreq_data * wrqu,char * ext)
{
	int ret = 0;
		atbm_printk_wext("usage: 		iwpriv wlan0 <cmd> [para]..,\n");
		atbm_printk_wext("fwdbg		<0|1> = gets/sets firmware debug message switch, when setting, 0, close, 1, open\n");
		atbm_printk_wext("fwcmd		<firmware cmd line> = letting firmware performance command,e.g:fwcmd para1,para2,parq3....\n");
		atbm_printk_wext("start_tx 	<channel,rate,len,is_40M,greedfiled> = start transmmit \n");
		atbm_printk_wext("		channel value:1~14 \n");
		atbm_printk_wext("		rate id: 1, 2, 5.5, 11, 6, 9, 12, 18, 24, 36, 48, 54, 6.5, 13, 19.5,26, 39, 52, 58.5, 65, 32\n");
		atbm_printk_wext("		len range:100~1024\n");
		atbm_printk_wext("		is_40M value: 1:40M,0:20M\n");
		atbm_printk_wext("		greedfiled value: 1:greedfiled,0:\n");
		atbm_printk_wext("stop_tx		NO prarameter = stop transmmit\n");
		atbm_printk_wext("start_rx 	<[1~14],[0|1]> = start receive,parameter:channel,channel type;1:40M,0:20M\n");
		atbm_printk_wext("stop_rx		NO parameter = stop receive\n");
		atbm_printk_wext("set_gi		<0|1> = 1,support shortGI; 0, not support shortGI\n");
		atbm_printk_wext("rts_threshold		<show|value> = show or set rts threshold\n");
		atbm_printk_wext("rate		<show|free|rate id> = show rate,or free a fixed rate,or forces a fixed rate support");
		atbm_printk_wext(" rate id: 1, 2, 5.5, 11, 6, 9, 12, 18, 24, 36, 48, 54, 6.5, 13, 19.5,26, 39, 52, 58.5, 65\n");
		atbm_printk_wext("getmac		NO Parameter = get mac address\n");
		atbm_printk_wext("wolEn		<0|1> = 1,enable wol; 0, disable wol\n");
		atbm_printk_wext("get_rx_stats	<1>\n");
		atbm_printk_wext("freqoffset		<0|1> 0:not write efuse; 1: write efuse \n");
		atbm_printk_wext("etf_result_get  get the test result \n");
#ifdef ATBM_PRIVATE_IE
		atbm_printk_wext("insert_data      insert private user data to (Probe Req or Beacon or Probe Resp) \n");
		atbm_printk_wext("get_data         obtain private user data from the broadcast packet \n");
		atbm_printk_wext("send_msg         start private scan according to the channel setting \n");
		atbm_printk_wext("recv_msg         recv broadcast device information of contain private IE \n");
		atbm_printk_wext("ipc_reset        reset ipc private scan function \n");
		atbm_printk_wext("channel_switch   switch channle to another \n");
#endif
		atbm_printk_wext("hide_ssid        Enable or Disble hidden ssid func, only for AP mode \n");
		atbm_printk_wext("set_freq		 param1: channel [1:14],param2: freq val(2380 or 2504) \n");
		atbm_printk_wext("set_txpower         1: high tx power, 0: normal tx power \n");
		atbm_printk_wext("get_tp_rate         obtain current throughput, AP mode need to add mac address of station \n");
		atbm_printk_wext("best_ch_start         start the best channel scan \n");
		atbm_printk_wext("best_ch_end		   end the best channel scan \n");
		atbm_printk_wext("best_ch_rslt		   get the best channel scan result \n");
		atbm_printk_wext("switch_ch		   switch channel \n");
		atbm_printk_wext("getSigmstarEfuse		   get sigmstar 256bits efuse \n");
		atbm_printk_wext("setSigmstarEfuse		   set sigmstar 256bits efuse \n");

		
	return ret;
}

/**************************************************************************
**
** NAME        CmdLine_GetToken
**
** PARAMETERS:    *pLine -    current line location to parse.
**
** RETURNS:        the token located. It never be NULL, but can be "\0"
**              *pLine - next line location to parse.
**
** DESCRIPTION    Locate the next token from a cli.
**
**************************************************************************/
char * CmdLine_GetToken(char ** pLine)
{
    char *    str;
    char *    line;
    char ch;

    line = *pLine;

    /* escape white space */
    ch = line[0];
    while(ch != 0)
    {
        /* CmdLine_GetLine() has already replaced '\n' '\r' with 0 */
        if ( (ch == ' ') || (ch == ',') || (ch == '\t') ||(ch == ':'))
        {
            line++;
            ch = line[0];
            continue;
        }
        break;
    }

    str = line;
    while(ch != 0)
    {
        if ( (ch == ' ') || (ch == ',') || (ch == '\t')||(ch == ':') )
        {
            line[0] = 0;
            /* CmdLine_GetLine() has replaced '\n' '\r' with 0, so we can do line++ */
            line++;
            break;
        }
        line++;
        ch = line[0];
    }

    *pLine = line;

    return str;
}
int CmdLine_GetHex(char **pLine, unsigned int  *pDword)
{
    char *  str;
    char *  str0;
    int     got_hex;
    unsigned int  d = 0;

    str = CmdLine_GetToken(pLine);
    if (str[0] == 0)
    {
        return 0;
    }

    str0 = str;
    got_hex = 0;
    for (;;)
    {
        char    ch;

        ch = str[0];
        if (ch == 0)
        {
            break;
        }
        if (ch >= '0' && ch <= '9')
        {
            d = (d<<4) | (ch - '0');
        }
        else if (ch >= 'a' && ch <= 'f')
        {
            d = (d<<4) | (ch - 'a' + 10);
        }
        else if (ch >= 'A' && ch <= 'F')
        {
            d = (d<<4) | (ch - 'A' + 10);
        }
        else
        {
            got_hex = 0;
            break;
        }
        got_hex = 1;
        str++;
    }
    if (got_hex)
    {
        *pDword = d;
    }
    else
    {
        atbm_printk_wext("Invalid hexdecimal: %s\n", str0);
    }

    return got_hex;
}
int CmdLine_GetSignInteger(char **pLine, int *pDword)
{
    char *  str;
    char *  str0;
    int     got_int;
	int negativeFlag = 0;
    int  d = 0;

    str = CmdLine_GetToken(pLine);
    if (str[0] == 0)
    {
        return 0;
    }

    str0 = str;
    got_int = 0;
    for (;;)
    {
        char    ch;

        ch = str[0];
        if (ch == 0)
        {
            break;
        }
		if((ch == '-') && (str0 == str))
		{
			negativeFlag = -1;
            str++;
		}else if (ch >= '0' && ch <= '9')
        {
            d = d*10 + (ch - '0');
            got_int = 1;
            str++;
        }
        else
        {
            got_int = 0;
            break;
        }
    }
    if (got_int)
    {
    	if (negativeFlag < 0)
        	*pDword = d * negativeFlag;
    	else
    		*pDword = d;	
    }
    else
    {
        atbm_printk_err("Invalid unsigned decimal: %s\n", str0);
    }

    return got_int;
}


/**************************************************************************
**
** NAME        CmdLine_GetInteger
**
** PARAMETERS:  *pLine - the current line location to parse.
**
** RETURNS:        TRUE if the next token is an unsigned decimal integer.
**              *pDword - the integer returned. Unchanged if return FALSE.
**              *pLine - next line location to parse.
**
** DESCRIPTION    Read an unsigned decimal integer from a cli.
**
**************************************************************************/
int CmdLine_GetInteger(char **pLine, unsigned int *pDword)
{
    char *  str;
    char *  str0;
    int     got_int;
    unsigned int  d = 0;

    str = CmdLine_GetToken(pLine);
    if (str[0] == 0)
    {
        return -1;
    }

    str0 = str;
    got_int = 0;
    for (;;)
    {
        char    ch;

        ch = str[0];
        if (ch == 0)
        {
            break;
        }
        if (ch >= '0' && ch <= '9')
        {
            d = d*10 + (ch - '0');
            got_int = 1;
            str++;
        }
        else
        {
            got_int = 0;
            break;
        }
    }
    if (got_int)
    {
        *pDword = d;
    }
    else
    {
        atbm_printk_wext("Invalid unsigned decimal: %s\n", str0);
    }

    return got_int;
}

#if USELESS
static int atbm_ioctl_smartconfig_start(struct net_device *dev, struct iw_request_info *info, void *wrqu, char *extra)
{
	#ifdef ATBM_SUPPORT_SMARTCONFIG
	u32 value;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	struct atbm_common *hw_priv=local->hw.priv;
	
	value = *extra - 0x30;
	
	atbm_smartconfig_start(hw_priv,value);
	#endif

	return 0;
}
#endif

#if USELESS
static int atbm_ioctl_smartconfig_stop(struct net_device *dev, struct iw_request_info *info, void *wrqu, char *extra)
{
	#ifdef ATBM_SUPPORT_SMARTCONFIG
	u32 value;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	struct atbm_common *hw_priv=local->hw.priv;

	value = *extra - 0x30;
	atbm_smartconfig_stop(hw_priv);
	#endif

	return 0;
}
#endif

#if USELESS
static int atbm_ioctl_smartconfig_start_v2(struct net_device *dev, struct iw_request_info *info, void *wrqu, char *extra)
{
	#ifdef ATBM_SUPPORT_SMARTCONFIG
	u32 value;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	struct atbm_common *hw_priv=local->hw.priv;
	struct atbm_vif *vif;
	int i = 0;	
	u32 enable = 1;
	
	value = *extra - 0x30;
	atbm_for_each_vif(hw_priv,vif,i){
				if (vif != NULL)
				{
					WARN_ON(wsm_write_mib(hw_priv, WSM_MIB_ID_SMARTCONFIG_START,
						&enable, sizeof(enable), vif->if_id));
					break;
				}
			}
	atbm_smartconfig_start(hw_priv,enable);
	#endif

	return 0;
}
#endif

#if USELESS
static int atbm_ioctl_smartconfig_stop_v2(struct net_device *dev, struct iw_request_info *info, void *wrqu, char *extra)
{
	#ifdef ATBM_SUPPORT_SMARTCONFIG
	u32 value;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	struct atbm_common *hw_priv=local->hw.priv;
	struct atbm_vif *vif;
	int i = 0;	
	u32 enable = 0;

	value = *extra - 0x30;
	atbm_for_each_vif(hw_priv,vif,i){
				if (vif != NULL)
				{
					WARN_ON(wsm_write_mib(hw_priv, WSM_MIB_ID_SMARTCONFIG_START,
						&enable, sizeof(enable), vif->if_id));
					break;
				}
			}
	atbm_smartconfig_start(hw_priv,enable);
	#endif

	return 0;
}
#endif

static int atbm_ioctl_fwdbg(struct net_device *dev, struct iw_request_info *ifno, union iwreq_data *wrqu, char *ext)
{
	int i = 0;	
	int ret = 0;
	u8 ucDbgPrintOpenFlag = 0;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	struct atbm_common *hw_priv=local->hw.priv;
	struct atbm_vif *vif;
	
	ucDbgPrintOpenFlag = *ext - 0x30;
	atbm_printk_wext("ALTM_GET_DBG_PRINT_TO_HOST\n");
	atbm_printk_wext("%s dbgflag:%d\n", __func__, ucDbgPrintOpenFlag);
			atbm_for_each_vif(hw_priv,vif,i){
				if (vif != NULL)
				{
					WARN_ON(wsm_write_mib(hw_priv, WSM_MIB_ID_DBG_PRINT_TO_HOST,
						&ucDbgPrintOpenFlag, sizeof(ucDbgPrintOpenFlag), vif->if_id));
					break;
				}
			}


	return ret;
}

static int atbm_ioctl_fwcmd(struct net_device *dev, struct iw_request_info *ifno, union iwreq_data *wrqu, char *ext)
{
	
	int i = 0;
	int ret = 0;
	char *extra = NULL;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	
	struct atbm_common *hw_priv=local->hw.priv;
	struct atbm_vif *vif;
	if(!(extra = atbm_kmalloc(wrqu->data.length+1,GFP_KERNEL)))
		return -ENOMEM;
	if(copy_from_user(extra, wrqu->data.pointer, wrqu->data.length)){
		atbm_kfree(extra);
		return -EINVAL;
	}

	if(wrqu->data.length <= 1){
		atbm_printk_err("invalid parameter!\n");
		atbm_printk_err("e.g:./iwpriv wlan0 fwcmd intr\n");
		atbm_kfree(extra);
		return -EINVAL;
	}

	for(i=0;i<wrqu->data.length;i++){
		if(extra[i] == ',')
			extra[i] =' ';
	}

	if(extra[0] == ' '){
		atbm_printk_err("invalid parameter!\n");
		atbm_kfree(extra);
		return -EINVAL;
	}
	//printk("exttra = %s  %d\n", extra, wrqu->data.length);
	i = 0;
	atbm_for_each_vif(hw_priv,vif,i){
		if (vif != NULL)
		{
			atbm_printk_wext("exttra = %s\n", extra);
			WARN_ON(wsm_write_mib(hw_priv, WSM_MIB_ID_FW_CMD,
				extra, wrqu->data.length, vif->if_id));
			break;
		}
	}
	atbm_kfree(extra);
	return ret;
	
}

static int atbm_ioctl_start_tx(struct net_device *dev, struct iw_request_info *info, union iwreq_data *wrqu, char *ext)
{
	int i = 0;
	int ret = 0;
	int len = 0;
	int num = 0;
	int flag = 0;
	int channel = 0;
	u32 rate = 0;
	u32 is_40M = 0;
	int band_value = 0;
	int greedfiled = 0;
	char *extra  = NULL;
	char *rate_p = NULL;
	char *len_p = NULL;
	char *is_40M_p = NULL;
	char *greedfiled_p = NULL;
	char threshold_param[100] = {0};
	char *delim = "cfg:";
	char *pthreshold = NULL;
	int etf_v2 = 0;
	u8 ucDbgPrintOpenFlag = 1;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	
	struct atbm_common *hw_priv=local->hw.priv;
	struct atbm_vif *vif;
	atbm_test_rx_cnt = 0;
	txevm_total = 0;

	memset(&gthreshold_param, 0, sizeof(struct test_threshold));
	memset(&gRxs_s, 0, sizeof(struct rxstatus_signed));

	chipversion = GetChipVersion(hw_priv);
	atbm_printk_wext("chipversion:0x%x\n", chipversion);

	if(ETF_bStartTx || ETF_bStartRx){
		atbm_printk_err("Error! already start_tx/send_singleTone, please stop_tx first!\n");
		return 0;
	}

	if(!(extra = atbm_kmalloc(wrqu->data.length + 1, GFP_KERNEL)))
		return -ENOMEM;
	if((ret = copy_from_user(extra, wrqu->data.pointer, wrqu->data.length)) != 0){
		atbm_kfree(extra);
		return -EINVAL;
	}

	atbm_printk_wext("atbm_ioctl_start_tx:%s\n",extra);
	if(wrqu->data.length < 10){
		atbm_printk_err("need to input parameters\n");
		atbm_printk_err("e.g: ./iwpriv wlan0 start_tx channel,rate,len,is_40M,greedfiled\n");
		atbm_printk_err("e.g: ./iwpriv wlan0 start_tx 1,1,300,1\n");
		return -EINVAL;
	}

	pthreshold = strstr(extra, delim);
	if(pthreshold)
	{
		memcpy(threshold_param, pthreshold, strlen(pthreshold));
		memset(pthreshold, 0, strlen(pthreshold));
		atbm_printk_wext("**extra:%s**\n", extra);
		atbm_printk_wext("**threshold_param:%s**\n", threshold_param);
	}
	get_test_threshold(threshold_param);

	for(i=0;extra[i] != ',';i++){
		if(extra[i] == ','){
			break;
		}
		channel = channel * 10 +(extra[i] - 0x30);
	}
	
	for(i=0;i<wrqu->data.length;i++){
		if((extra[i] == ',') && (num == 0)){
				num++;
				rate_p = extra + i + 1;
		}
		else if((extra[i] == ',') && (num == 1)){
			len_p = extra + i + 1;
			num++;
		}
		else if((extra[i] == ',') && (num == 2)){
			is_40M_p = extra + i + 1;
			num++;
		}
		else if((extra[i] == ',') && (num == 3)){
			greedfiled_p = extra + i + 1;
			break;
		}
	}

	if((NULL == rate_p) || (NULL == len_p) || (NULL == is_40M_p) || (NULL == greedfiled_p)){
		atbm_printk_err("need more parameter,please try again!\n");
		atbm_printk_err("e.g: ./iwpriv wlan0 start_tx channel,rate,len,is_40M,greedfiled\n");
		atbm_kfree(extra);
		return -EINVAL;
	}

	for(i=0;rate_p[i] != ',';i++){
		if(rate_p[i] == ','){
			break;
		}
		if(rate_p[i] == '.'){
			flag = 1;
			continue;
		}	
		band_value = band_value* 10 +(rate_p[i] - 0x30);
	}

	if(flag == 0)
		band_value = band_value * 10;

	for(i=0;len_p[i] != ',';i++){
		if(len_p[i] == ','){
			break;
		}
		len = len * 10 +(len_p[i] - 0x30);
	}

	
	if(is_40M_p[0] == ','){
		atbm_printk_err("invalid channnel type\n");
		atbm_printk_err("is_40M value: 1:40M,0:20M\n");
		atbm_kfree(extra);
		return -EINVAL;
	}

	if((greedfiled_p[0] == ',') || (greedfiled_p[0] == ' ') || (greedfiled_p[0]) == '\0'){
		atbm_printk_err("invalid channnel type\n");
		atbm_printk_err("ps_mode value: 1:greedfiled,0:\n");
		atbm_kfree(extra);
		return -EINVAL;
	}

	//printk("is_40M :%s greedfiled_p:%s\n", is_40M_p, greedfiled_p);
	

	for(i=0;is_40M_p[i] != ',';i++){
		if(1 == i){
			atbm_printk_err("invalid channel type!\n");
			atbm_printk_err("is_40M value: 1:40M,0:20M\n");
			atbm_kfree(extra);
			return 	-EINVAL;
		}
		is_40M = is_40M * 10 + (is_40M_p[i] - 0x30);
	}

	for(i=0;greedfiled_p[i] != '\0';i++){
		greedfiled = greedfiled * 10 + (greedfiled_p[i] - 0x30);
	}


	//printk("is_40M = %d\n", is_40M);

	if((is_40M != 0) && (is_40M != 1)){
			atbm_printk_err("invalid 40M or 20M %d\n",is_40M);
			atbm_kfree(extra);
			return -EINVAL;
	}

	if((greedfiled != 0) && (greedfiled != 1)){
			atbm_printk_err("invalid greedfiled %d\n",greedfiled);
			atbm_kfree(extra);
			return -EINVAL;
	}
	
	//check channel
	if(channel <= 0 || channel > 14){
		atbm_printk_err("invalid channel!\n");
		atbm_kfree(extra);
		return -EINVAL;
	}
	//printk("rate===%d\n",rate);

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
				atbm_printk_err("invalid rate!\n");
				return -EINVAL;
				
		}

	if((is_40M == 1 )&& (rate < WSM_TRANSMIT_RATE_HT_6)){
		atbm_printk_err("invalid 40M rate\n");
		atbm_kfree(extra);
		return -EINVAL;
	}	
	if((is_40M == 1 )&& ((channel < 3)||(channel > 11))){
		atbm_printk_err("invalid 40M rate,channel value range:3~11\n");
		atbm_kfree(extra);
		return -EINVAL;
	}
	
	if(len == 99999){
		ucWriteEfuseFlag = 1;
		etf_v2 = 1;	
		len = hw_priv->etf_len = 1000; 
	}else if(len == 99998)
	{
		ucWriteEfuseFlag = 0;
		etf_v2 = 1;	
		len = hw_priv->etf_len = 1000; 
	}
	//Prevent USB from being unplugged suddenly in product testing
	if(etf_v2)
	{
		hw_priv->bStartTx = 1;
		hw_priv->bStartTxWantCancel = 1;
		hw_priv->etf_test_v2 =1;
	}

	//check len
	if(len < 100 || len > 1024){
		atbm_printk_err("len:%d\n", len);
		atbm_printk_err("invalid len!\n");
		atbm_kfree(extra);
		return -EINVAL;
	}
	if(is_40M == 1){
		is_40M = NL80211_CHAN_HT40PLUS;//
		channel -= 2;
	}

	atbm_printk_wext("NL80211_CHAN_HT40PLUS:%d\n", NL80211_CHAN_HT40PLUS);

	//printk("%d, %d, %d, %d\n", channel, rate, len, is_40M);
	hw_priv->etf_channel = channel;
	hw_priv->etf_channel_type = is_40M;
	hw_priv->etf_rate = rate;
	hw_priv->etf_len = len; 
	hw_priv->etf_greedfiled = greedfiled;
	
	atbm_for_each_vif(hw_priv,vif,i){
		if((vif != NULL)){
			atbm_printk_wext("*******\n");

			down(&hw_priv->scan.lock);
			if(!etf_v2)
			{
				WARN_ON(wsm_write_mib(hw_priv, WSM_MIB_ID_DBG_PRINT_TO_HOST,
					&ucDbgPrintOpenFlag, sizeof(ucDbgPrintOpenFlag), vif->if_id));
			}
			mutex_lock(&hw_priv->conf_mutex);
			
			if(etf_v2){
				
				CodeStart = DCXO_CODE_MINI;
				CodeEnd = DCXO_CODE_MAX;
				if(wsm_start_tx_v2(hw_priv, vif->vif) != 0)
				{
					up(&hw_priv->scan.lock);
					atbm_printk_err("%s:%d,wsm_start_tx_v2 error\n", __func__, __LINE__);
				}
			}
			else
			{
				ETF_bStartTx = 1;
				if(wsm_start_tx(hw_priv, vif->vif) != 0)
				{
					up(&hw_priv->scan.lock);
					atbm_printk_err("%s:%d,wsm_start_tx error\n", __func__, __LINE__);
				}
			}
			mutex_unlock(&hw_priv->conf_mutex);
			break;
		}
	}
	
	atbm_kfree(extra);
	return ret;
}

static int atbm_ioctl_stop_tx(struct net_device *dev, struct iw_request_info *info, union iwreq_data *wrqu, char *ext)
{
	int i = 0;
	int ret = 0;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	struct atbm_common *hw_priv=local->hw.priv;

	struct atbm_vif *vif;

	if(0 == ETF_bStartTx){
		atbm_printk_err("please start start_rx first,then stop_rx\n");
		return -EINVAL;
	}

	if(wrqu->data.length > 1){
		atbm_printk_err("redundant parameters,please try again!\n");
		return -EINVAL;
	}
	
	mutex_lock(&hw_priv->conf_mutex);
	ETF_bStartTx = 0;
	mutex_unlock(&hw_priv->conf_mutex);
	//./iwpriv wlan0 fwdbg 0
	
	atbm_for_each_vif(hw_priv,vif,i){
		if((vif != NULL)){
			
			wsm_oper_unlock(hw_priv);
			wsm_stop_tx(hw_priv);
			wsm_stop_scan(hw_priv,i);
			up(&hw_priv->scan.lock);
		}
	}
	
	//printk("%s %d\n", __FUNCTION__, __LINE__);

	return ret;
}

static int atbm_ioctl_start_rx(struct net_device *dev, struct iw_request_info *info, union iwreq_data *wrqu, char *ext)
{
	int i = 0;
	int ret = 0;
	int is_40M = 0;
	int channel = 0;
	char *extra = NULL;
	char *is_40M_p = NULL;
	char cmd[20] = "monitor 1 ";
	u8 ucDbgPrintOpenFlag = 1;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	struct atbm_common *hw_priv=local->hw.priv;
	struct atbm_vif *vif;


	if(ETF_bStartTx || ETF_bStartRx){
		atbm_printk_err("Error! already ETF_bStartRx/ETF_bStartTx/send_singleTone, please stop first!\n");
		return 0;
	}


	//./iwpriv wlan0 fwdbg 1
	atbm_for_each_vif(hw_priv,vif,i){
				if (vif != NULL)
				{
					WARN_ON(wsm_write_mib(hw_priv, WSM_MIB_ID_DBG_PRINT_TO_HOST,
						&ucDbgPrintOpenFlag, sizeof(ucDbgPrintOpenFlag), vif->if_id));
					break;
				}
			}

	if(wrqu->data.length <= 3){
		atbm_printk_err("need to input parameters\n");
		atbm_printk_err("e.g: ./iwpriv wlan0 start_rx 1,0\n");
		return -EINVAL;
	}

	if(!(extra = atbm_kmalloc(wrqu->data.length+1, GFP_KERNEL)))
		return -EINVAL;

	if((ret = copy_from_user(extra, wrqu->data.pointer, wrqu->data.length)) != 0){
		atbm_kfree(extra);
		return -EINVAL;
	}

	for(i=0;extra[i] != ',';i++){
		if(extra[i] == ','){
			break;
		}
		channel = channel * 10 +(extra[i] - 0x30);
	}

	for(i=0;i<wrqu->data.length;i++){
			if(extra[i] == ','){
				is_40M_p = extra +i + 1;
				break;
			}	
		}

	if((NULL == is_40M_p) || (is_40M_p[0] == '\0')){
		atbm_printk_err("invalid channnel type\n");
		atbm_printk_err("is_40M value: 1:40M,0:20M\n");
		atbm_kfree(extra);
		return -EINVAL;
	}

	for(i=0;is_40M_p[i] != '\0';i++){
		if(1 == i){
			atbm_printk_err("invalid channel type!\n");
			atbm_printk_err("is_40M value: 1:40M,0:20M\n");
			atbm_kfree(extra);
			return 	-EINVAL;
		}
		is_40M = is_40M * 10 + (is_40M_p[i] - 0x30);
	}

	atbm_printk_wext("is_40M:%d\n", is_40M);
	if((is_40M != 0) && (is_40M != 1)){
		atbm_kfree(extra);
		atbm_printk_err("invalid 40M or 20M\n");
		return -EINVAL;
	}
	
	if(channel <= 0 || channel > 14){
			atbm_kfree(extra);
			atbm_printk_err("invalid channel!\n");
			return -EINVAL;
		}

	for(i=0;i<wrqu->data.length;i++){
		if(extra[i] == ',')
			extra[i] =' ';
	}


	if((is_40M == 1 )&& ((channel == 1)||(channel > 11))){
		atbm_kfree(extra);
		atbm_printk_err("invalid 40M rate\n");
		return -EINVAL;
	}
	memset(ch_and_type, 0, 20);
	memcpy(ch_and_type, extra, wrqu->data.length);
	memcpy(cmd+10, extra, wrqu->data.length);
	
	atbm_printk_wext("CMD:%s\n", cmd);
	i = 0;
	atbm_for_each_vif(hw_priv,vif,i){
		if (vif != NULL)
		{
			ETF_bStartRx = 1;
			atbm_printk_wext("extra = %s %d\n", extra, wrqu->data.length + 10);
			WARN_ON(wsm_write_mib(hw_priv, WSM_MIB_ID_FW_CMD,
				cmd, wrqu->data.length + 10, vif->if_id));
			break;
		}
	}	

	if(extra)
		atbm_kfree(extra);
	return ret;
}


static int atbm_ioctl_stop_rx(struct net_device *dev, struct iw_request_info *info, union iwreq_data *wrqu, char *ext)
{
	int i = 0;
	int ret = 0;
	char cmd[20] = "monitor 0 ";
	u8 ucDbgPrintOpenFlag = 0;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	struct atbm_common *hw_priv=local->hw.priv;
	struct atbm_vif *vif;

	if(wrqu->data.length > 1){
		atbm_printk_err("redundant parameters, please try again!\n");
		return -EINVAL;
	}

	if((0 == ETF_bStartRx) || (NULL == ch_and_type)){
		atbm_printk_err("please start start_rx first,then stop_rx\n");
		return -EINVAL;
	}

	ETF_bStartRx = 0;
	
	memcpy(cmd+10, ch_and_type, strlen(ch_and_type));
	//printk("cmd %s\n", cmd);
	i = 0;
	atbm_for_each_vif(hw_priv,vif,i){
		if (vif != NULL)
		{
			WARN_ON(wsm_write_mib(hw_priv, WSM_MIB_ID_FW_CMD,
				cmd, 13, vif->if_id));
			break;
		}
	}

	atbm_for_each_vif(hw_priv,vif,i){
				if (vif != NULL)
				{
					WARN_ON(wsm_write_mib(hw_priv, WSM_MIB_ID_DBG_PRINT_TO_HOST,
						&ucDbgPrintOpenFlag, sizeof(ucDbgPrintOpenFlag), vif->if_id));
					break;
				}
			}
	return ret;
}

#if USELESS
static int atbm_ioctl_get_rx_stats(struct net_device *dev, struct iw_request_info *info, union iwreq_data *wrqu, char *ext)
{
	int ret = 0;
	char *extra = NULL;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	struct atbm_common *hw_priv=local->hw.priv;
	struct atbm_vif *vif;
	int i = 0;
	
	if(!(extra = atbm_kmalloc(16, GFP_KERNEL)))
		return -ENOMEM;

	if((ret = copy_from_user(extra, wrqu->data.pointer, wrqu->data.length))){
		atbm_kfree(extra);
		return -EINVAL;
	}

	atbm_for_each_vif(hw_priv,vif,i){
				if (vif != NULL)
				{
					WARN_ON(wsm_read_mib(hw_priv, WSM_MIB_ID_GET_ETF_RX_STATS,
						extra, 16, vif->if_id));
					break;
				}
			}

	if((ret = copy_to_user(wrqu->data.pointer, extra, 16)) != 0){
		atbm_kfree(extra);
		return -EINVAL;
	}
	atbm_kfree(extra);
	return ret;
}
#endif

#if USELESS
static int atbm_ioctl_set_rate(struct net_device *dev, struct iw_request_info *info, union iwreq_data *wrqu, char *ext)
{
	int ret = 0;
	int value = 0;
	int arraylen = 0;
	int counter;
	char *extra = NULL;
	struct altm_msg msg;
	//struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	//struct ieee80211_local *local = sdata->local;
	
	if(!(extra = atbm_kmalloc(wrqu->data.length+1, GFP_KERNEL)))
		return -ENOMEM;
	if((ret = copy_from_user(extra, wrqu->data.pointer, wrqu->data.length))){
		atbm_kfree(extra);
		return -EINVAL;
	}

	if (wrqu->data.length == 1)
	{
		atbm_kfree(extra);
		atbm_printk_err("rate need argument\n");
		return ret;
	}

	if(!(memcmp(extra, "show", wrqu->data.length))){
		//atbm_tesmode_reply(wiphy, &g_atbm_rate_Ctl.rate_fix[RATE_CONTROL_UNICAST], sizeof(u32));
		atbm_printk_wext("rate fix=%d\n", g_atbm_rate_Ctl.rate_fix[RATE_CONTROL_UNICAST]);
		//atbm_tesmode_reply(wiphy, &g_atbm_rate_Ctl.my_flags[RATE_CONTROL_UNICAST], sizeof(u8));
		atbm_printk_wext("rate flags=%d\n", g_atbm_rate_Ctl.my_flags[RATE_CONTROL_UNICAST]);
		//atbm_tesmode_reply(wiphy, &g_atbm_rate_Ctl.my_index[RATE_CONTROL_UNICAST], sizeof(u8));
		atbm_printk_wext("rate index=%d\n", g_atbm_rate_Ctl.my_index[RATE_CONTROL_UNICAST]);
	}
	else if (!(memcmp(extra, "free", wrqu->data.length)))
	{
		//msg.type = ALTM_RATE_SET_FIX;
		msg.value = 0;
		g_atbm_rate_Ctl.rate_fix[RATE_CONTROL_UNICAST] = msg.value;
		//msg.type = ALTM_RATE_SET_FLAGS;
		msg.value = 0;
		g_atbm_rate_Ctl.my_flags[RATE_CONTROL_UNICAST] = msg.value;
		//msg.type = ALTM_RATE_SET_INDEX;
		msg.value = 0;
		g_atbm_rate_Ctl.my_index[RATE_CONTROL_UNICAST] = msg.value;
	}else{
		arraylen = arraysize(ratetab);
		for(counter = 0; counter < arraylen; counter++)
		{
			if (strcasecmp(extra, ratetab[counter].ratename) == 0)
			{
					//msg.type = ALTM_RATE_SET_FIX;
					msg.value = 1;
					g_atbm_rate_Ctl.rate_fix[RATE_CONTROL_UNICAST] = msg.value;
					//atbm_tesmode_reply(wiphy, &g_atbm_rate_Ctl.my_flags[RATE_CONTROL_UNICAST], sizeof(u8));
					//msg.type = ALTM_RATE_SET_FLAGS;
					value &= ~0x8;
					msg.value = ratetab[counter].flags | value;
					g_atbm_rate_Ctl.my_flags[RATE_CONTROL_UNICAST] = msg.value;
					//msg.type = ALTM_RATE_SET_INDEX;
					msg.value = ratetab[counter].index;
					g_atbm_rate_Ctl.my_index[RATE_CONTROL_UNICAST] = msg.value;
					break;
			}
		}
		if (counter == arraylen)
		{
			atbm_printk_err("Maybe do not support '%s' rate or invalid parameter!\n", extra);
			atbm_kfree(extra);
			return -EINVAL;
		}
	}

	atbm_kfree(extra);
	return ret;
}
#endif

#if USELESS
static int atbm_ioctl_set_rts(struct net_device *dev, struct iw_request_info *info, union iwreq_data *wrqu, char *ext)
{
	int i = 0;
	int ret = 0;
	int value = 0;
	char *extra = NULL;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	struct atbm_common *hw_priv=local->hw.priv;
	struct atbm_vif *vif;

	if(wrqu->data.length <= 1){
		atbm_printk_err("need parameter,please try again!\n");
		return -EINVAL;
	}

	if(!(extra = atbm_kmalloc(wrqu->data.length+1, GFP_KERNEL)))
		return -EINVAL;
	if((ret = copy_from_user(extra, wrqu->data.pointer, wrqu->data.length))){
		atbm_kfree(extra);
		return -EINVAL;
	}

	if(!(memcmp(extra, "show", wrqu->data.length))){
		atbm_printk_err("get_rts_threshold = %d\n", atbm_tool_rts_threshold);
		atbm_kfree(extra);
		return ret;
	}
	else{
		for(i=0;extra[i] != '\0';i++){
			value = value * 10 + (extra[i] - 0x30);
		}

		if((value <0 ) || (value >= 2048)){
			atbm_printk_err("invalid parameter!\n");
			atbm_kfree(extra);
			return -EINVAL;
		}
		
		atbm_printk_wext("set_rtsthr is %d\n", value);
		atbm_for_each_vif(hw_priv,vif,i){
			if(vif != NULL)
			{
				__le32 val32;
				if (value != (u32) -1)
					val32 = __cpu_to_le32(value);
				else
					val32 = 0; /* disabled */

				/* mutex_lock(&priv->conf_mutex); */
				ret = WARN_ON(wsm_write_mib(hw_priv, WSM_MIB_ID_DOT11_RTS_THRESHOLD,
					&val32, sizeof(val32), vif->if_id));
				atbm_tool_rts_threshold = val32;
				break;
			}
		}
	}
	atbm_kfree(extra);
	return ret;
}
#endif

#if USELESS
static int atbm_ioctl_ctl_gi(struct net_device *dev, struct iw_request_info *info, union iwreq_data *wrqu, char *ext)
{
	int ret = 0;
	int value = 0;
	char *extra = NULL;
	atbm_printk_wext("@@@@@@@ atbm_ioctl_ctl_gi()\n");

	if(!(extra = atbm_kmalloc(wrqu->data.length+1, GFP_KERNEL))){
		atbm_printk_err("atbm_kmalloc failed!\n");
		return -EINVAL;
	}
	if((ret = copy_from_user(extra, wrqu->data.pointer, wrqu->data.length))){
		atbm_kfree(extra);
		return -EINVAL;
	}

	if((wrqu->data.length <= 1) || (wrqu->data.length > 2)){
		atbm_kfree(extra);
		atbm_printk_err("invalid parameter,please try again!\n");
		return -EINVAL;
	}

	value = *extra - 0x30;
	if((value < 0 ) || (value > 1)){
		atbm_kfree(extra);
		atbm_printk_err("invalid parameter,parameter must be 1 or 0\n");
		return -EINVAL;
	}
	atbm_printk_wext("set_short_gi(%d)\n", value);
	atbm_set_shortGI(value);
	atbm_tool_shortGi = value;
	atbm_kfree(extra);
	return ret;
}
#endif

#if USELESS
static int atbm_ioctl_setmac(struct net_device *dev, struct iw_request_info *info, void *wrqu, char *extra)
{
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	int index;
	u8 macAddr[6] = {0};
	u8 extraBuff[18] = {0};
	int ret = -EINVAL;
#define   isHex(value)  	(((value > '9') && (value < '0')) && \
							 ((value > 'f') && (value < 'a')) && \
							 ((value > 'F') && (value < 'A')))
	memcpy(extraBuff, extra, 17);
	extraBuff[17] = ':';
	for (index  = 0 ; index < 17; index+= 3)
	{
		if (isHex(extraBuff[index]))
		{
					
			atbm_printk_err("mac addr format error\n");
			return -EINVAL;
		}
		if (isHex(extraBuff[index + 1]))
		{
					
			atbm_printk_err( "mac addr format error\n");
			return -EINVAL;
		}
		
		if (extraBuff[index + 2] != ':')
		{
			atbm_printk_err("mac addr format error\n");
			return -EINVAL;
		}
		
	}

#undef isHex

	sscanf(extraBuff, "%x:%x:%x:%x:%x:%x", (int *)&macAddr[0], (int *)&macAddr[1], (int *)&macAddr[2], (int *)&macAddr[3], (int *)&macAddr[4], (int *)&macAddr[5]);

	if (local->ops != NULL)
	{
		ret = local->ops->set_mac_addr2efuse(&local->hw, macAddr);
	}

	return ret;
}
#endif

#if USELESS
static int atbm_ioctl_getmac(struct net_device *dev, struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{
	//int index;
	u8 macAddr[6] = {0};
	//u8 extraBuff[18] = {0};
	int ret = -EINVAL;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	struct atbm_common *hw_priv=local->hw.priv;

	if(wrqu->data.length > 1){
		atbm_printk_err("command 'getmac' need not parameters\n");
		return ret;
	}

	if ((ret = wsm_get_mac_address(hw_priv, &macAddr[0])) == 0){
		atbm_printk_err("macAddr:%02x:%02x:%02x:%02x:%02x:%02x\n", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
		return ret;
	}
	else{
		atbm_printk_err("read mac address failed\n");
		return ret;
	}
	
	return ret;
}
#endif

#if USELESS
static int atbm_ioctl_start_wol(struct net_device *dev, struct iw_request_info *info, union iwreq_data *wrqu, char *ext)
{
	int i;
	int ret = 0;
	int value = 0;
	char *extra = NULL;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	struct atbm_common *hw_priv=local->hw.priv;
	struct atbm_vif *vif;

	if(!(extra = atbm_kmalloc(wrqu->data.length+1, GFP_KERNEL))){
		atbm_printk_err("atbm_kmalloc failed!\n");
		return -EINVAL;
	}
	if((ret = copy_from_user(extra, wrqu->data.pointer, wrqu->data.length))){
		atbm_kfree(extra);
		return -EINVAL;
	}

	if((wrqu->data.length <= 1) || (wrqu->data.length > 2)){
		atbm_printk_err("invalid parameter,please try again!\n");
		atbm_kfree(extra);
		return -EINVAL;
	}

	value = *extra - 0x30;
	if((value < 0 ) || (value > 1)){
		atbm_printk_err("invalid parameter,parameter must be 1 or 0\n");
		atbm_kfree(extra);
		return -EINVAL;
	}
	atbm_printk_wext("start wol func(%d)\n", value);
	atbm_for_each_vif(hw_priv,vif,i){
		if(vif != NULL)
		{
			__le32 val32;
			if (value != (u32) -1)
				val32 = __cpu_to_le32(value);
			else
				val32 = 0; /* disabled */
	
			ret = WARN_ON(wsm_write_mib(hw_priv, WSM_MIB_ID_START_WOL,
				&val32, sizeof(val32), vif->if_id));
			break;
		}
	}
	atbm_kfree(extra);
	return ret;

}
#endif

#define MAC2STR(a) (a)[0],(a)[1],(a)[2],(a)[3],(a)[4],(a)[5]
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#ifdef ATBM_PRIVATE_IE
/*
*1, SSTAR_INSERT_USERDATA_CMD    	
*	ioctl(global->ioctl_sock, SSTAR_INSERT_USERDATA_CMD, &user_data)
*	
*   update special ie to beacon,probe response and probe request ,if possible 
*/
static int atbm_ioctl_ie_insert_user_data(struct net_device *dev, struct iw_request_info *info, void *wrqu, char *extra)
{
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	struct ieee80211_sub_if_data *sdata_update;
	u8 atbm_oui[4]={0x41,0x54,0x42,0x4D};
	
	int ret = 0;
	char *special = NULL;
	int len = 0;
	union iwreq_data *wdata = (union iwreq_data *)wrqu;
	
	if(!ieee80211_sdata_running(sdata)){
		atbm_printk_err("%s:[%s] not running\n",__func__,sdata->name);
		ret = -ENETDOWN;
		goto exit;
	}

	len = wdata->data.length-1;

	if((len<0)||(len>255)){
		atbm_printk_err("%s:[%s] ie is err (%d)\n",__func__,sdata->name,len);
		ret = -EINVAL;
		goto exit;
	}

	special = atbm_kzalloc(len+4, GFP_KERNEL);

	if(special == NULL){
		atbm_printk_err("%s:[%s] special is not alloc\n",__func__,sdata->name);
		ret = -EINVAL;
		goto exit;
	}
	
	memcpy(special,atbm_oui,4);
	
	if(copy_from_user(special+4, wdata->data.pointer, len)){
		atbm_printk_err("[IE] copy_from_user fail\n");
		ret = -ENODATA;
		goto exit;
	}

	list_for_each_entry(sdata_update, &local->interfaces, list){
		bool res = true;
		
		if(!ieee80211_sdata_running(sdata_update)){
			continue;
		}

		if(sdata_update->vif.type == NL80211_IFTYPE_STATION){
			res = ieee80211_ap_update_special_probe_request(sdata_update,special,len+4);
		}else if((sdata_update->vif.type == NL80211_IFTYPE_AP)&&
		         (rtnl_dereference(sdata_update->u.ap.beacon))){
		    res = ieee80211_ap_update_special_beacon(sdata_update,special,len+4);
			if(res == true){
				res = ieee80211_ap_update_special_probe_response(sdata_update,special,len+4);
			}
		}
		if(res == false){
			ret = -EOPNOTSUPP;
			goto exit;
		}
	}
exit:
	if(special)
		atbm_kfree(special);
	return ret;
}

/*
*2,  SSTAR_GET_USERDATA_CMD    			
*	ioctl(global->ioctl_sock, SSTAR_INSERT_USERDATA_CMD, &user_data)
*	get special ie of the received beacon
*/
static bool atbm_handle_special_ie(struct ieee80211_hw *hw,struct atbm_internal_scan_results_req *req,struct ieee80211_internal_scan_sta *sta)
{
	u8 *special_ie = (u8*)req->priv;
	u8 *pos = NULL;
	u8 atbm_oui[4]={0x41,0x54,0x42,0x4D};//ATBM
	
	if(req->n_stas >= MAC_FILTER_NUM){
		atbm_printk_err("%s: n_ies(%d)\n",__func__,req->n_stas);
		return false;
	}

	pos = special_ie + (req->n_stas*USER_DATE_LEN);

	if(sta->ie&&sta->ie_len){
		if(memcmp(sta->ie,atbm_oui,4) == 0){
			if((sta->ie_len>4)){
				memcpy(pos,sta->ie+4,sta->ie_len-4);
				req->n_stas++;
			}
		}else {
			memcpy(pos,sta->ie,sta->ie_len);
			req->n_stas++;
		}
	}
	return true;
}
static int atbm_ioctl_ie_get_user_data(struct net_device *dev, struct iw_request_info *info, void *wrqu, char *extra)
{
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	int ret = 0;
	union iwreq_data *wdata = (union iwreq_data *)wrqu;
	struct atbm_internal_scan_results_req req;
	u8 *special_ie = NULL;
	
	if(!ieee80211_sdata_running(sdata)){
		atbm_printk_err("%s:[%s] not running\n",__func__,sdata->name);
		ret = -ENETDOWN;
		goto exit;
	}

	special_ie = atbm_kzalloc(MAC_FILTER_NUM*USER_DATE_LEN, GFP_KERNEL);

	if(special_ie == NULL){
		atbm_printk_err("%s:[%s] special is not alloc\n",__func__,sdata->name);
		ret = -EINVAL;
		goto exit;
	}

	req.flush = true;
	req.priv  = special_ie;
	req.result_handle  = atbm_handle_special_ie;
	req.n_stas = 0;
	ieee80211_scan_internal_req_results(local,&req);

	if(copy_to_user(wdata->data.pointer, special_ie, USER_DATE_LEN*MAC_FILTER_NUM) != 0){
		ret = -EINVAL;
		goto exit;
	}
	
exit:
	if(special_ie)
		atbm_kfree(special_ie);
	
	return ret;
}


/*
*3,  SSTAR_SEND_MSG_CMD       		     
*    ioctl(global->ioctl_sock, SSTAR_SEND_MSG_CMD, &Wifi_Send_Info_t)
*
*    triger scan
*/
static int atbm_ioctl_ie_send_msg(struct net_device *dev, struct iw_request_info *info, void *wrqu, char *extra)
{
	int ret = 0;
	int len = 0;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	Wifi_Send_Info_t *send_info = NULL;
	union iwreq_data *wdata = (union iwreq_data *)wrqu;
	struct ieee80211_internal_scan_request internal_scan;
	u8 channel = 0;
	
	memset(&internal_scan,0,sizeof(struct ieee80211_internal_scan_request));

	if(!ieee80211_sdata_running(sdata)){
		atbm_printk_err("%s:[%s] not running\n",__func__,sdata->name);
		ret = -ENETDOWN;
		goto exit;
	}

	if(sdata->vif.type != NL80211_IFTYPE_STATION){
		atbm_printk_err("%s:[%s] not support scan\n",__func__,sdata->name);
		ret = -EOPNOTSUPP;
		goto exit;
	}

	len = wdata->data.length-1;

	if(len>0){
		
		send_info = (Wifi_Send_Info_t *)atbm_kzalloc(wdata->data.length+1, GFP_KERNEL);
		if(send_info == NULL){
			ret =  -ENOMEM;
			goto exit;
		}
		if(copy_from_user((void *)send_info, wdata->data.pointer, wdata->data.length)){
			ret =  -EINVAL;
			goto exit;
		}

		if(send_info->mac_filter){
			u8 i = 0;
			internal_scan.macs = atbm_kzalloc(MAC_FILTER_NUM*sizeof(struct ieee80211_internal_mac), GFP_KERNEL);

			if(internal_scan.macs == NULL){
				ret =  -EINVAL;
				goto exit;
			}

			for(i = 0;i<MAC_FILTER_NUM;i++){
				memcpy(internal_scan.macs[i].mac,send_info->Bssid[i],6);
			}
			internal_scan.n_macs = MAC_FILTER_NUM;
		}
		channel = send_info->channel;
	}else {
	}

	if(channel != 0){
		internal_scan.channels = &channel;
		internal_scan.n_channels = 1;
	}
    if(atbm_internal_cmd_scan_triger(sdata,&internal_scan) == false){
		ret =  -ENOMEM;
		goto exit;
    }
exit:
	if(send_info)
		atbm_kfree(send_info);
	if(internal_scan.macs)
		atbm_kfree(internal_scan.macs);
	return ret;
}
static bool atbm_handle_scan_sta(struct ieee80211_hw *hw,struct atbm_internal_scan_results_req *req,struct ieee80211_internal_scan_sta *sta)
{
	Wifi_Recv_Info_t *info     = (Wifi_Recv_Info_t *)req->priv;
	Wifi_Recv_Info_t *pos_info = NULL;
	u8 atbm_oui[4]={0x41,0x54,0x42,0x4D};//ATBM
	
	if(req->n_stas >= MAC_FILTER_NUM){
		atbm_printk_err("%s: n_stas(%d)\n",__func__,req->n_stas);
		return false;
	}
#ifndef SIGMASTAR_FILTER_MACADDR_ONLY
	atbm_printk_debug("%s:ssid[%s] ie_len(%d)\n",__func__,sta->ssid,sta->ie_len);
	if((sta->ie == NULL) || (sta->ie_len == 0))
		return true;
#endif
	if(sta->ie && sta->ie_len){	
		atbm_printk_debug("%s:ie[%s] ie_len(%d)\n",__func__,sta->ie,sta->ie_len);
		if(memcmp(sta->ie,atbm_oui,4) || (sta->ie_len < 4))
			return true;
	}
	
	pos_info = info+req->n_stas;
	req->n_stas ++;
	pos_info->channel = sta->channel;
	pos_info->Rssi = sta->signal;
	memcpy(pos_info->Bssid,sta->bssid,6);
	if(sta->ssid_len && sta->ssid)
		memcpy(pos_info->Ssid,sta->ssid,sta->ssid_len);
	if(sta->ie && sta->ie_len)
		memcpy(pos_info->User_data,sta->ie+4,sta->ie_len-4);
	return true;
}

/*
*4,  SSTAR_RECV_MSG_CMD       
*	 ioctl(global->ioctl_sock, SSTAR_RECV_MSG_CMD, &Wifi_Recv_Info_t)
*	 get the received beacon and probe response
*/
static int atbm_ioctl_ie_recv_msg(struct net_device *dev, struct iw_request_info *info, void *wrqu, char *extra)
{
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	u8 *recv_info = NULL;
	union iwreq_data *wdata = (union iwreq_data *)wrqu;
	struct atbm_internal_scan_results_req req;
	int ret = 0;

	req.flush = false;
	req.n_stas = 0;
	req.priv   = NULL;
	req.result_handle = NULL;
	
	if(!ieee80211_sdata_running(sdata)){
		atbm_printk_err("%s:[%s] not running\n",__func__,sdata->name);
		ret = -ENETDOWN;
		goto exit;
	}

	recv_info = atbm_kzalloc(sizeof(Wifi_Recv_Info_t)*MAC_FILTER_NUM, GFP_KERNEL);

	if(recv_info == NULL){
		ret =  -ENOMEM;
		goto exit;
	}

	req.flush = true;
	req.n_stas = 0;
	req.priv = recv_info;
	req.result_handle = atbm_handle_scan_sta;

	ieee80211_scan_internal_req_results(sdata->local,&req);

	if(copy_to_user((void *)wdata->data.pointer, recv_info, MAC_FILTER_NUM*sizeof(Wifi_Recv_Info_t)) != 0){
		ret = -EINVAL;
	}
exit:
	if(recv_info)
		atbm_kfree(recv_info);
	return ret;
}
int my_isdigit(char val)
{
	if(('0'/*0x30*/ <= val)&& (val <= '9')/*0x39*/)
		return 1;
	else
		return 0;
}
int my_atoi(const char *str)
{
	int ret=0,sign=1;
	atbm_printk_wext("@@@atoi() 1\n");

	if(NULL == str)
		return -1;

	atbm_printk_wext("@@@atoi() 2\n");

	//skip tab and space
	for(; *str==' '||*str=='\t'; str++)
	
	if(*str == '-')
		sign = -1;
	
	if(*str == '-' || *str == '+')
		str++;
	atbm_printk_wext("@@@atoi() 3\n");
	while(my_isdigit(*str)){
		atbm_printk_wext("@@@atoi() 0x%x\n", *str);
		ret = ret*10 + *str - '0';
		str++;
	}
	return sign*ret;
}

unsigned short str2channel(char *ptr)
{
	switch(*ptr){
		case 0x30:
			return 0;
		case 0x31: 
			if(*(ptr+1) == 0)
				return 1;
			else if(*(ptr+1) == 0x30)
				return 10;
			else if(*(ptr+1) == 0x31)
				return 11;
			else if(*(ptr+1) == 0x32)
				return 12;
			else if(*(ptr+1) == 0x33)
				return 13;
			else if(*(ptr+1) == 0x34)
				return 14;
		case 0x32: return 2;
		case 0x33: return 3;
		case 0x34: return 4;
		case 0x35: return 5;
		case 0x36: return 6;
		case 0x37: return 7;
		case 0x38: return 8;
		case 0x39: return 9;

		default:
			return 3;

	}
}

static int atbm_ioctl_ie_ipc_reset(struct net_device *dev, struct iw_request_info *info, void *wrqu, char *extra)
{
	int ret = 0;
	struct ieee80211_sub_if_data *sdata = NULL;
	struct atbm_vif *priv = NULL;
	unsigned char *ptr = NULL;
	union iwreq_data *wdata = (union iwreq_data *)wrqu;	
	
	if(dev == NULL){
		atbm_printk_err("[IE] atbm_ioctl_ie_ipc_reset() dev NULL\n");
		return -1;
	}

	sdata = IEEE80211_DEV_TO_SUB_IF(dev);

	if(sdata == NULL){
		atbm_printk_err("[IE] atbm_ioctl_ie_ipc_reset() sdata NULL\n");
		return -1;
	}
	
	mutex_lock(&sdata->local->iflist_mtx);

	if(!(ptr = (char *)atbm_kzalloc(wdata->data.length+1, GFP_KERNEL)))
		return -ENOMEM;
	if((ret = copy_from_user(ptr, wdata->data.pointer, wdata->data.length))){
		atbm_kfree(ptr);
		return -EINVAL;
	}
	
	priv = (struct atbm_vif *)sdata->vif.drv_priv;
	
	if(atomic_read(&priv->enabled)==0){
		atbm_printk_err("[IE] atbm_ioctl_ie_ipc_reset() priv is disabled\n");
		atbm_kfree(ptr);
		mutex_unlock(&sdata->local->iflist_mtx);
		return -1;
	}
	
	atbm_printk_wext("\n[IE] atbm_ioctl_ie_ipc_reset()\n\n");
	
	if(memcmp(ptr, "data", 4) == 0){
		//clear private data
	}else if(memcmp(ptr, "mac", 3) == 0){
		//clear mac addr set
	}else if(memcmp(ptr, "recv", 4) == 0){
		//clear recv buffer
	}else if(memcmp(ptr, "all", 3) == 0){
		//clear private data
		//clear mac addr set
		//clear recv buffer
	}else{
		atbm_printk_err("[IE] atbm_ioctl_ie_ipc_reset() invalid param\n");
		atbm_kfree(ptr);
		mutex_unlock(&sdata->local->iflist_mtx);
		return -1;
	}
	
	if((memcmp(ptr, "data", 4) == 0) || (memcmp(ptr, "all", 3) == 0)){
		if(sdata->vif.type != NL80211_IFTYPE_STATION){
			ret = atbm_upload_beacon_private(priv);
			if(ret < 0){
				atbm_printk_err("[IE] reset: upload beacon private failed %d\n", ret);
			}
		
			//ret = atbm_upload_proberesp_private(priv);
			//if(ret < 0){
			//	printk("[IE] upload probe resp private failed %d\n", ret);
			//}
		}else{
			//not do anything
		}	
	}
	
	atbm_kfree(ptr);
	mutex_unlock(&sdata->local->iflist_mtx);

	return ret;
}
static int atbm_ioctl_ie_test(struct net_device *dev, struct iw_request_info *info, void *wrqu, char *extra)
{
	int ret = 0;

	union iwreq_data *wdata = (union iwreq_data *)wrqu;
	//struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	//struct atbm_vif *priv = ABwifi_hwpriv_to_vifpriv(hw_priv,0);
	Wifi_Send_Info_t send_info;
	char *ptr = NULL;
	unsigned short channel = 0;
	
	struct ieee80211_sub_if_data *sdata = NULL;
	struct atbm_vif *priv = NULL;
	
	if(dev == NULL){
		atbm_printk_err("[IE] atbm_ioctl_ie_test() dev NULL\n");
		return -1;
	}

	sdata = IEEE80211_DEV_TO_SUB_IF(dev);

	if(sdata == NULL){
		atbm_printk_err("[IE] atbm_ioctl_ie_test() sdata NULL\n");
		return -1;
	}
	

	priv = (struct atbm_vif *)sdata->vif.drv_priv;
	
	if(atomic_read(&priv->enabled)==0){
		atbm_printk_err("[IE] atbm_ioctl_ie_test() priv is disabled\n");
		return -1;
	}	

	atbm_printk_wext("\n[IE] atbm_ioctl_ie_test()\n\n");

	if(sdata->vif.type == NL80211_IFTYPE_STATION){
		atbm_printk_wext("[IE] STA Test Start\n");
		if(!(ptr = (char *)atbm_kzalloc(wdata->data.length+1, GFP_KERNEL))){
			return -ENOMEM;
		}
		if((ret = copy_from_user(ptr, wdata->data.pointer, wdata->data.length))){
			atbm_kfree(ptr);
			return -EINVAL;
		}

		frame_hexdump("Test", ptr, 10);
		
		//channel = my_atoi(ptr);
		//channel = 3;
		//channel = *(unsigned short *)ptr;
		//memcpy(&channel, ptr, sizeof(unsigned short));
		channel = str2channel(ptr);
		atbm_printk_wext("[IE] channel is %d\n", channel);
		
		memset(&send_info, 0, sizeof(Wifi_Send_Info_t));
		
		send_info.channel = channel;

		if(copy_to_user(wdata->data.pointer, (char *)&send_info, sizeof(Wifi_Send_Info_t)) != 0){
			atbm_kfree(ptr);
			return -EINVAL;
		}
		wdata->data.length = sizeof(Wifi_Send_Info_t) + 1;
		
		atbm_ioctl_ie_send_msg(dev, info, (void *)wrqu, extra);
		atbm_kfree(ptr);
	}else{
		atbm_printk_wext("[IE] AP Test Start\n");
	}

	return ret;
}
int SWICHN_FLAG=0;
int new_chan=0;
static int atbm_ioctl_channle_switch(struct net_device *dev, struct iw_request_info *info, void *wrqu, char *extra)
{
	int ret = 0;
	u8 new_ch_num = 0;
	int new_freq;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_channel *new_ch;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,30))		
	struct cfg80211_chan_def *chandef=NULL;
#endif
	struct ieee80211_channel_state *chan_state = ieee80211_get_channel_state(sdata->local, sdata);
	struct atbm_vif *priv = (struct atbm_vif *)sdata->vif.drv_priv;
	SWICHN_FLAG=1;
	new_ch_num=str2channel(extra);
	new_freq = ieee80211_channel_to_frequency(new_ch_num,IEEE80211_BAND_2GHZ);
	new_ch = ieee80211_get_channel(sdata->local->hw.wiphy, new_freq);
	new_chan=channel_hw_value(new_ch);
	chan_state->oper_channel = new_ch;
	//Add the channle switch Ie to Beacon
	if(sdata->vif.type != NL80211_IFTYPE_STATION){
		ret = atbm_upload_beacon_private(priv);
		if(ret < 0){
			atbm_printk_err("[IE] reset: upload beacon private failed %d\n", ret);
		}
	}
	//Add Timer to switch channel
	mod_timer(&priv->channel_timer,
		  jiffies +
		  msecs_to_jiffies(3*priv->beacon_int));//
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,30))
	chandef=(struct cfg80211_chan_def*)atbm_kmalloc(sizeof(struct cfg80211_chan_def),GFP_KERNEL);
	if(!chandef){
		atbm_printk_wext("chandef is Error\n");
	}
	//Notify hostapd that the channle is alread channged
	chandef->chan=new_ch;
	chandef->width=chan_state->_oper_channel_type;
	
	chandef->center_freq1 = new_freq;
	chandef->center_freq2 = 0;
	cfg80211_ch_switch_notify(dev,chandef);
#endif
	return ret;
}
static int atbm_ioctl_get_rssi(struct net_device *dev, struct iw_request_info *info, void *wrqu, char *extra)
{
	struct altm_wext_msg{
		int type;
		int value;
		char externData[256];
	}msg;
	
	struct _atbm_wifi_info_{
		int wext_rssi;
		u8	wext_mac[ETH_ALEN];
	}atbm_wifi_info[ATBMWIFI_MAX_STA_IN_AP_MODE];

	int i = 0;	
	int ret = 0;
	union iwreq_data *wdata = (union iwreq_data *)wrqu;
	struct ieee80211_sub_if_data *sdata = NULL;
	struct ieee80211_local *local;
	struct atbm_vif *priv = NULL;
	struct atbm_common *hw_priv;	
	struct sta_info *sta;


	if(dev == NULL){
		atbm_printk_err("atbm_ioctl_get_rssi() dev NULL\n");
		return -1;
	}

	sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	if(sdata == NULL){
		atbm_printk_err("atbm_ioctl_get_rssi() sdata NULL\n");
		return -1;
	}

	local = sdata->local;
	if(local == NULL){
		atbm_printk_err("atbm_ioctl_get_rssi() local NULL\n");
		return -1;
	}
	
	mutex_lock(&local->iflist_mtx);

	priv = (struct atbm_vif *)sdata->vif.drv_priv;
	if(priv == NULL){
		atbm_printk_err("atbm_ioctl_get_rssi() priv NULL\n");
		return -1;
	}

	if(atomic_read(&priv->enabled)==0){
		atbm_printk_err("atbm_ioctl_get_rssi() priv is disabled\n");
		mutex_unlock(&local->iflist_mtx);
		return -1;
	}
	
	hw_priv = priv->hw_priv;
	if(hw_priv == NULL){
		atbm_printk_err("atbm_ioctl_get_rssi() hw_priv NULL\n");
		mutex_unlock(&local->iflist_mtx);
		return -1;
	}

	atbm_printk_wext("atbm_ioctl_get_rssi()\n");

	memset(&msg, 0, sizeof(msg));
	memset(atbm_wifi_info,0,sizeof(struct _atbm_wifi_info_)*ATBMWIFI_MAX_STA_IN_AP_MODE);

	rcu_read_lock();
		
	list_for_each_entry_rcu(sta, &local->sta_list, list) {

		if(sta != NULL){
			if (sta->sdata->vif.type == NL80211_IFTYPE_AP){
				atbm_printk_wext( "@@@ sta cnt %d, %zu\n", hw_priv->connected_sta_cnt, sizeof(atbm_wifi_info));
				
				atbm_wifi_info[i].wext_rssi = sta->last_signal;
				memcpy(atbm_wifi_info[i].wext_mac, sta->sta.addr, ETH_ALEN);
				atbm_printk_wext( "%d get sta: rssi %d, "MACSTR"\n", i, atbm_wifi_info[i].wext_rssi, MAC2STR(atbm_wifi_info[i].wext_mac));
				
				++i;
			}else{
				msg.value = sta->last_signal;
				atbm_printk_wext( "atbm_ioctl_get_rssi() rssi %d\n", msg.value);
				break;
			}
		}

	}

	rcu_read_unlock();

	memcpy((u8*)msg.externData, (u8*)&atbm_wifi_info[0], sizeof(atbm_wifi_info));

	if(copy_to_user((u8 *)wdata->data.pointer, (u8*)(&msg), sizeof(msg)) != 0){
		mutex_unlock(&local->iflist_mtx);
		return -EINVAL;
	}
	
	atbm_printk_wext("atbm_ioctl_get_rssi() size %zu\n",sizeof(msg));
	mutex_unlock(&local->iflist_mtx);

	return ret;
}

#endif

extern unsigned int atbm_wifi_status_get(void);
static int atbm_ioctl_get_wifi_state(struct net_device *dev, struct iw_request_info *info, void *wrqu, char *extra)
{
	int ret = 0;
	unsigned char *ptr = NULL;
	unsigned int wifi_status = 0;
	union iwreq_data *wdata = (union iwreq_data *)wrqu;
	//struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	//struct ieee80211_local *local = sdata->local;
	//struct atbm_common *hw_priv=local->hw.priv;
	//struct atbm_vif *priv = (struct atbm_vif *)sdata->vif.drv_priv;
	struct ieee80211_sub_if_data *sdata = NULL;
	struct atbm_vif *priv = NULL;
	
	if(dev == NULL){
		atbm_printk_err("[IE] atbm_ioctl_get_wifi_state() dev NULL\n");
		return -1;
	}

	sdata = IEEE80211_DEV_TO_SUB_IF(dev);

	if(sdata == NULL){
		atbm_printk_err("[IE] atbm_ioctl_get_wifi_state() sdata NULL\n");
		return -1;
	}
	
	mutex_lock(&sdata->local->iflist_mtx);
	
	priv = (struct atbm_vif *)sdata->vif.drv_priv;
	
	if(atomic_read(&priv->enabled)==0){
		atbm_printk_err("[IE] atbm_ioctl_get_wifi_state() priv is disabled\n");
		mutex_unlock(&sdata->local->iflist_mtx);
		return -1;
	}
	
	ptr = wdata->data.pointer;

	//printk("atbm_ioctl_get_wifi_state()\n");

	//ap mode
	if(sdata->vif.type != NL80211_IFTYPE_STATION){
		ret = -1;
		goto Error;
	}

	wifi_status = atbm_wifi_status_get();
	
	atbm_printk_wext("%d\n", wifi_status);

	if(copy_to_user(ptr, (char *)&wifi_status, sizeof(unsigned int)) != 0){
		mutex_unlock(&sdata->local->iflist_mtx);
		ret = -EINVAL;
	}

Error:	
	mutex_unlock(&sdata->local->iflist_mtx);
	return ret;

}

static char wifi_freq_buf[300]={0};
static char *wifi_freq = "NULL";
module_param(wifi_freq,charp,0644);
MODULE_PARM_DESC(wifi_freq,"wifi freq");
void atbm_set_freq(struct atbm_common *hw_priv, SPECIAL_CH_FREQ *pdata)
{
	int i;
	
	memset(wifi_freq_buf, 0, sizeof(wifi_freq_buf));
	for(i=0; i<CHANNEL_NUM; i++){
		if(pdata[i].flag == 1){
			sprintf(wifi_freq_buf+strlen(wifi_freq_buf), "ch:%d, freq:%d \n", i+1, pdata[i].special_freq);
		}
	}
	
	wifi_freq = wifi_freq_buf;

	return;
}

static int atbm_ioctl_set_freq(struct net_device *dev, struct iw_request_info *info, void *wrqu, char *extra)
{
	union iwreq_data *wdata = (union iwreq_data *)wrqu;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	int ret = 0;
	char *freq_info = NULL;
	char *pos;
	int len = 0;
	unsigned int channel_num;
	unsigned int freq;
	struct ieee80211_internal_set_freq_req req;

	if(!ieee80211_sdata_running(sdata)){
		atbm_printk_err("%s:[%s] not running\n",__func__,sdata->name);
		ret = -ENETDOWN;
		goto exit;
	}

	len = wdata->data.length;

	if(len <= 0){
		atbm_printk_err("%s:len err\n",__func__);
		ret = -EINVAL;
		goto exit;
	}

	freq_info = atbm_kzalloc(wdata->data.length+1, GFP_KERNEL);

	if(freq_info == NULL){
		atbm_printk_err("%s:freq_info alloc err\n",__func__);
		ret = -EINVAL;
		goto exit;
	}

	if(copy_from_user(freq_info, wdata->data.pointer, wdata->data.length)){
		ret =  -EINVAL;
		goto exit;
	}

	pos = freq_info;
	
	CmdLine_GetInteger(&pos, &channel_num);
	CmdLine_GetInteger(&pos, &freq);
	if(freq == 0)
		req.set = false;
	else
		req.set = true;
	req.channel_num = (u16)channel_num;
	req.freq = freq;
	
	atbm_printk_wext("atbm: ch %d, freq %d\n", req.channel_num, req.freq);

	if(atbm_internal_freq_set(&sdata->local->hw,&req) == false){
		ret =  -EINVAL;
	}
	
exit:
	if(freq_info)
		atbm_kfree(freq_info);
	return ret;
}

static int wifi_tx_pw = 0;
static char wifi_txpw_buf[64]={0};
static char *wifi_txpw = "NULL";
module_param(wifi_txpw,charp,0644);
MODULE_PARM_DESC(wifi_txpw,"wifi tx power");

int atbm_get_tx_power(void)
{
	return wifi_tx_pw;
}

void atbm_set_tx_power(struct atbm_common *hw_priv, int txpw)
{
	char *p20, *p40, *pHT;
	
	wifi_tx_pw = txpw;

	if(wifi_tx_pw & BIT(0))
		p20 = "20M-High ";
	else
		p20 = "20M-Normal ";


	if(wifi_tx_pw & BIT(1))
		p40 = "40M-High ";
	else
		p40 = "40M-Normal ";

	if((hw_priv->channel_type == NL80211_CHAN_HT20)||(hw_priv->channel_type == NL80211_CHAN_NO_HT))
		pHT = "20M-Mode";
	else
		pHT = "40M-Mode";

	memset(wifi_txpw_buf, 0, sizeof(wifi_txpw_buf));
	sprintf(wifi_txpw_buf, "%s, %s, %s", p20, p40, pHT);
	wifi_txpw = wifi_txpw_buf;

	return;
}

static int atbm_ioctl_set_txpw(struct net_device *dev, struct iw_request_info *info, void *wrqu, char *extra)
{
	int ret = 0;
	int len = 0;
	char *ptr = NULL, *free_pp = NULL;
	char cmd[32];
	unsigned int tx_pw;

	union iwreq_data *wdata = (union iwreq_data *)wrqu;

	struct ieee80211_sub_if_data *sdata = NULL;
	struct ieee80211_local *local = NULL;
	struct atbm_vif *priv = NULL;
	struct atbm_common *hw_priv= NULL;
	
	if(dev == NULL){
		atbm_printk_err("[IE] atbm_ioctl_set_txpw() dev NULL\n");
		return -1;
	}

	sdata = IEEE80211_DEV_TO_SUB_IF(dev);

	if(sdata == NULL){
		atbm_printk_err("[IE] atbm_ioctl_set_txpw() sdata NULL\n");
		return -1;
	}
	
	mutex_lock(&sdata->local->iflist_mtx);
	local = sdata->local;
	hw_priv=local->hw.priv;
	
	priv = (struct atbm_vif *)sdata->vif.drv_priv;
	
	if(atomic_read(&priv->enabled)==0){
		atbm_printk_err("[IE] atbm_ioctl_set_txpw() priv is disabled\n");
		mutex_unlock(&sdata->local->iflist_mtx);
		return -1;
	}

	atbm_printk_wext("atbm_ioctl_set_txpw()\n\n");

	if(!(ptr = (char *)atbm_kzalloc(wdata->data.length+1, GFP_KERNEL))){
		mutex_unlock(&sdata->local->iflist_mtx);
		return -ENOMEM;
	}
	if((ret = copy_from_user(ptr, wdata->data.pointer, wdata->data.length))){
		atbm_kfree(ptr);
		mutex_unlock(&sdata->local->iflist_mtx);
		return -EINVAL;
	}
	
	free_pp = ptr;

	len = wdata->data.length-1;

	memset(cmd, 0, sizeof(cmd));
	sprintf(cmd, "set_txpower %s ", ptr);
	atbm_printk_wext("atbm: %s\n", cmd);
	ret = wsm_write_mib(hw_priv, WSM_MIB_ID_FW_CMD, cmd, strlen(cmd), priv->if_id);
	if(ret < 0)
		atbm_printk_err("atbm: txpw wsm write mib failed. \n");

	CmdLine_GetInteger(&ptr, &tx_pw);
	atbm_set_tx_power(hw_priv, (int)tx_pw);
	
	atbm_kfree(free_pp);
	mutex_unlock(&sdata->local->iflist_mtx);

	return ret;
}
unsigned char char2Hex(const char chart)
{
	unsigned char ret;
	
	switch(chart){
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9':
			ret = 0x0 + (chart - '0');
			break;

		case 'a':
		case 'b':
		case 'c':
		case 'd':
		case 'e':
		case 'f':
			ret = 0xA + (chart - 'a');
			break;
			
		case 'A':
		case 'B':
		case 'C':
		case 'D':
		case 'E':
		case 'F':
			ret = 0xA + (chart - 'A');
			break;

		default:
			atbm_printk_wext("char2Hex: error %c %d\n", chart, chart);
			ret = 0;
			
	}

	return ret;
}

/*
Func: str2mac
Param: 
	str->string format of MAC address
	i.e. 00:11:22:33:44:55
Return: 
	error -1
	OK 0
*/
int str2mac(char *dst_mac, char *src_str)
{
	int i;
	
	if(dst_mac == NULL || src_str == NULL)
		return -1;

	for(i=0; i<6; i++){
		dst_mac[i] = (char2Hex(src_str[i*3]) << 4) + (char2Hex(src_str[i*3 + 1]));
		atbm_printk_wext("str2mac: %x\n", dst_mac[i]);
	}

	return 0;	
}
extern int atbm_find_link_id(struct atbm_vif *priv, const u8 *mac);
static int atbm_ioctl_get_rate(struct net_device *dev, struct iw_request_info *info, void *wrqu, char *extra)
{
	int ret = 0;
	char mac_addr[6];

	int sta_id = 0;
	
	unsigned char *ptr = NULL;
	unsigned int rate_val = 0;
	
	union iwreq_data *wdata = (union iwreq_data *)wrqu;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	struct atbm_common *hw_priv=local->hw.priv;
	struct atbm_vif *priv = (struct atbm_vif *)sdata->vif.drv_priv;
	
	ptr = wdata->data.pointer;

	atbm_printk_wext("atbm_ioctl_get_rate()\n");

	//ap mode
	if(sdata->vif.type != NL80211_IFTYPE_STATION){
		//clear mac addr buffer
		memset(mac_addr, 0, 6);
		
		//convert mac string to mac hex format
		str2mac(mac_addr, ptr);
		
		//according to mac hex, find out sta link id
		sta_id = atbm_find_link_id(priv, mac_addr);

		atbm_printk_wext("atbm_ioctl_get_rate() sta_id %d\n", sta_id);
		wsm_write_mib(hw_priv, WSM_MIB_ID_GET_RATE, &sta_id, 1, priv->if_id);
	}

	wsm_read_mib(hw_priv, WSM_MIB_ID_GET_RATE, &rate_val, sizeof(unsigned int), priv->if_id);

	//convert bits/s
	rate_val = rate_val/2;
	atbm_printk_wext("rate: %d bits/s\n", rate_val);

	//memcpy(extra, (char *)&rate_val, sizeof(unsigned int));
	if(copy_to_user(ptr, (char *)&rate_val, sizeof(unsigned int)) != 0)
		return -EINVAL;

	return ret;
}
int atbm_ioctl_best_ch_start(struct net_device *dev, struct iw_request_info *info, void *wrqu, char *extra)
{
	int ret = 0;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_internal_channel_auto_select_req req;
	
	if(!ieee80211_sdata_running(sdata)){
		atbm_printk_err("%s:[%s] not running\n",__func__,sdata->name);
		ret = -ENETDOWN;
		goto exit;
	}

	if(sdata->vif.type != NL80211_IFTYPE_STATION){
		atbm_printk_err("%s:[%s] not support scan\n",__func__,sdata->name);
		ret = -EOPNOTSUPP;
		goto exit;
	}
	memset(&req,0,sizeof(struct ieee80211_internal_channel_auto_select_req));

	if(atbm_internal_channel_auto_select(sdata,&req) == false){
		ret = -EOPNOTSUPP;
	}
exit:
	return ret;
}
int atbm_ioctl_best_ch_scan_result(struct net_device *dev, struct iw_request_info *info, void *wrqu, char *extra)
{
	int ret = 0;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_internal_channel_auto_select_results results;
	Best_Channel_Scan_Result scan_result;
	int i = 0;
	union iwreq_data *wdata = (union iwreq_data *)wrqu;
	
	if(!ieee80211_sdata_running(sdata)){
		atbm_printk_err("%s:[%s] not running\n",__func__,sdata->name);
		ret = -ENETDOWN;
		goto exit;
	}

	if(sdata->vif.type != NL80211_IFTYPE_STATION){
		atbm_printk_err("%s:[%s] not support scan\n",__func__,sdata->name);
		ret = -EOPNOTSUPP;
		goto exit;
	}
	
	memset(&scan_result,0,sizeof(Best_Channel_Scan_Result));
	memset(&results,0,sizeof(struct ieee80211_internal_channel_auto_select_results));
	results.version = 0;//use version 0
	
	if(atbm_internal_channel_auto_select_results(sdata,&results) == false){
		atbm_printk_err("%s:channel results err\n",__func__);
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

	if(copy_to_user(wdata->data.pointer, &scan_result, sizeof(scan_result)) != 0)
		ret = -EINVAL;
exit:
	return ret;
}
static int atbm_ioctl_best_ch_scan(struct net_device *dev, struct iw_request_info *info, void *wrqu, char *extra)
{
	int ret = 0;

	unsigned char *ptr = NULL;
	union iwreq_data *wdata = (union iwreq_data *)wrqu;

	if(!(ptr = (char *)atbm_kzalloc(wdata->data.length+1, GFP_KERNEL)))
		return -ENOMEM;
	
	if((ret = copy_from_user(ptr, wdata->data.pointer, wdata->data.length))){
		atbm_kfree(ptr);
		return -EINVAL;
	}

	if(memcmp(ptr, "start", 5) == 0)
		ret = atbm_ioctl_best_ch_start(dev, info, wrqu, extra);
	else if(memcmp(ptr, "result", 6) == 0)
		ret = atbm_ioctl_best_ch_scan_result(dev, info, wrqu, extra);
	else
		ret = -1;

	if(ret < 0)
		atbm_printk_err("atbm_ioctl_best_ch_scan(), error %s\n", ptr);
	
	atbm_kfree(ptr);
	
	return ret;
}

#ifdef ATBM_PRIVATE_IE
static int atbm_ioctl_get_SIGMSTAR_256BITSEFUSE(struct net_device *dev, struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{
	u8 efuseBuff[32] = {0};
	//u8 extraBuff[18] = {0};
	int ret = -EINVAL;
	int i;
	union iwreq_data *wdata = (union iwreq_data *)wrqu;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	struct atbm_common *hw_priv=local->hw.priv;

	//if(wrqu->data.length > 1){
	//	printk("command 'getsigmstarefuse' need not parameters\n");
	//	return ret;
	//}

	if ((ret = wsm_get_SIGMSTAR_256BITSEFUSE(hw_priv, &efuseBuff[0], sizeof(efuseBuff))) == 0){
		
		atbm_printk_wext("Get sigmstar efuse data:\n");
		for(i = 0; i < sizeof(efuseBuff); i++)
		{
			atbm_printk_wext("%02x ", efuseBuff[i]);
		}
		atbm_printk_wext("\n");
	}
	else{
		atbm_printk_err("read sigmstarefuse failed\n");
	}

	if(copy_to_user(wdata->data.pointer, efuseBuff, 32) != 0){
		ret = -EINVAL;
		atbm_printk_wext("copy to user failed.\n");
	}
	
	return ret;	
}
static int atbm_ioctl_set_SIGMSTAR_256BITSEFUSE(struct net_device *dev, struct iw_request_info *info, union iwreq_data *wrqu, char *ext)
{
	int i;
	int ret = -EINVAL;
	char *extra = NULL;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	struct atbm_common *hw_priv=local->hw.priv;
	u8 efuseBuff[32 +1];
	int strheadLen, trueLen;
	atbm_printk_wext("####### efuse\n");
	if(!(extra = atbm_kmalloc(wrqu->data.length+1, GFP_KERNEL))){
		atbm_printk_err("atbm_kmalloc failed!\n");
		return -EINVAL;
	}
	if((ret = copy_from_user(extra, wrqu->data.pointer, wrqu->data.length))){
		atbm_kfree(extra);
		return -EINVAL;
	}
	atbm_printk_err("length=%d,data=%s\n", wrqu->data.length, extra);
	strheadLen=strlen("setSigmstarEfuse,");
	trueLen =((wrqu->data.length - strheadLen) > 32) ? 32:(wrqu->data.length - strheadLen);
	atbm_printk_err("trueLen=%d,strheadLen=%d\n", trueLen, strheadLen);

	memset(efuseBuff, 0, sizeof(efuseBuff));
	for(i =0; i < trueLen; i++)
	{
		efuseBuff[i] = extra[strheadLen + i];
	}
	
	if ((ret = wsm_set_SIGMSTAR_256BITSEFUSE(hw_priv, &efuseBuff[0], 32)) == 0)
	{
		atbm_printk_wext("Set sigmstar efuse data:\n");
		for(i = 0; i < sizeof(efuseBuff); i++)
		{
			atbm_printk_wext("%02hhx ", efuseBuff[i]);
		}
		atbm_printk_wext("\n");
	}
	else{
		atbm_printk_err("write sigmstarefuse failed\n");
	}

	return ret;
}

static int atbm_ioctl_channel_test_start(struct net_device *dev, struct iw_request_info *info, void *wrqu, char *extra)
{
	u8 chTestStart = 0;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	struct atbm_common *hw_priv=local->hw.priv;
	struct atbm_vif *priv = (struct atbm_vif *)sdata->vif.drv_priv;

	wsm_write_mib(hw_priv, WSM_MIB_ID_CHANNEL_TEST_START, &chTestStart, sizeof(u8), priv->if_id);
//	wsm_read_mib(hw_priv, WSM_MIB_ID_GET_CHANNEL_IDLE, NULL, sizeof(unsigned short), priv->if_id);

	return 0;
}

static int atbm_ioctl_get_channel_idle(struct net_device *dev, struct iw_request_info *info, void *wrqu, char *extra)
{
	int ret = 0;
	
	unsigned char *ptr = NULL;
	unsigned short idle = 0;

	union iwreq_data *wdata = (union iwreq_data *)wrqu;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	struct atbm_common *hw_priv=local->hw.priv;
	struct atbm_vif *priv = (struct atbm_vif *)sdata->vif.drv_priv;
	
	ptr = wdata->data.pointer;

	wsm_read_mib(hw_priv, WSM_MIB_ID_GET_CHANNEL_IDLE, &idle, sizeof(unsigned short), priv->if_id);

	atbm_printk_err("current_idle:%d\n", idle);
	//memcpy(extra, (char *)&idle, sizeof(unsigned short));
	if(copy_to_user(ptr, (char *)&idle, sizeof(unsigned short)) != 0)
		return -EINVAL;

	return ret;
}
#endif
static int atbm_save_efuse(struct atbm_common *hw_priv,struct efuse_headr *efuse_save)
{
	int ret = 0;
	int iResult=0;
	//struct atbm_vif *vif;
	struct efuse_headr efuse_bak;
	
	/*
	*LMC_STATUS_CODE__EFUSE_VERSION_CHANGE	failed because efuse version change  
	*LMC_STATUS_CODE__EFUSE_FIRST_WRITE, 		failed because efuse by first write   
	*LMC_STATUS_CODE__EFUSE_PARSE_FAILED,		failed because efuse data wrong, cannot be parase
	*LMC_STATUS_CODE__EFUSE_FULL,				failed because efuse have be writen full
	*/
	ret = wsm_efuse_change_data_cmd(hw_priv, efuse_save,0);
	if (ret == LMC_STATUS_CODE__EFUSE_FIRST_WRITE)
	{
		atbm_printk_err("first write\n");
		iResult = -3;
	}else if (ret == LMC_STATUS_CODE__EFUSE_PARSE_FAILED)
	{
		atbm_printk_err("parse failed\n");
		iResult = -4;
	}else if (ret == LMC_STATUS_CODE__EFUSE_FULL)
	{
		atbm_printk_err("efuse full\n");
		iResult = -5;
	}else if (ret == LMC_STATUS_CODE__EFUSE_VERSION_CHANGE)
	{
		atbm_printk_err("efuse version change\n");
		iResult = -6;
	}else
	{
		iResult = 0;
	}
	if (iResult == 0)
	{
		//frame_hexdump("efuse_d", efuse_save, sizeof(struct efuse_headr));
		memset(&efuse_bak,0,sizeof(struct efuse_headr));
		wsm_get_efuse_data(hw_priv,(void *)&efuse_bak, sizeof(struct efuse_headr));

		if(efuse_bak.specific != 0)
		{
			//sigmastar oid
			efuse_save->specific = efuse_bak.specific;
		}
		
		if(memcmp((void *)&efuse_bak, efuse_save, sizeof(struct efuse_headr)) !=0)
		{
			frame_hexdump("efuse_bak", (u8 *)&efuse_bak, sizeof(struct efuse_headr));
			iResult = -2;
		}else
		{
			iResult = 0;
		}
	}
	return iResult;
}

struct efuse_headr efuse_data_global;

static int atbm_ioctl_get_efuse(struct net_device *dev, struct iw_request_info *info, union iwreq_data *wrqu, char *ext)
{
	int ret = -EINVAL;
	union iwreq_data *wdata = (union iwreq_data *)wrqu;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	struct atbm_common *hw_priv=local->hw.priv;
	struct efuse_headr efuse_data;

	memset(&efuse_data_global,0, sizeof(struct efuse_headr));

	if ((ret = wsm_get_efuse_data(hw_priv, &efuse_data, sizeof(efuse_data))) == 0){	
		atbm_printk_init("Get efuse data is [%d,%d,%d,%d,%d,%d,%d,%d,%x:%x:%x:%x:%x:%x]\n",
				efuse_data.version,efuse_data.dcxo_trim,efuse_data.delta_gain1,efuse_data.delta_gain2,efuse_data.delta_gain3,
				efuse_data.Tj_room,efuse_data.topref_ctrl_bias_res_trim,efuse_data.PowerSupplySel,efuse_data.mac[0],efuse_data.mac[1],
				efuse_data.mac[2],efuse_data.mac[3],efuse_data.mac[4],efuse_data.mac[5]);
		memcpy(&efuse_data_global, &efuse_data, sizeof(struct efuse_headr));
	}
	else{
		atbm_printk_err("read efuse failed\n");
	}

	if(copy_to_user(wdata->data.pointer, &efuse_data, sizeof(efuse_data)) != 0){
		ret = -EINVAL;
		atbm_printk_wext("copy to user failed.\n");
	}
	return ret;
}

static int atbm_ioctl_set_efuse(struct net_device *dev, struct iw_request_info *info, union iwreq_data *wrqu, char *ext)
{
	int i = 0;
	int ret = -EINVAL;
	char *extra = NULL;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	struct atbm_common *hw_priv=local->hw.priv;
	int strheadLen = 0;
	char *pRxData;
	int rxData;
	char cmd[50] = "";
	
	atbm_printk_wext("####### efuse\n");
	if(!(extra = atbm_kmalloc(wrqu->data.length+1, GFP_KERNEL))){
		atbm_printk_err("atbm_kmalloc failed!\n");
		return -EINVAL;
	}
	if((ret = copy_from_user(extra, wrqu->data.pointer, wrqu->data.length))){
		atbm_kfree(extra);
		return -EINVAL;
	}
	extra[wrqu->data.length] = 0;

	for(i=0;i<wrqu->data.length;i++)
	{
		if(extra[i] == ',')
		{
			strheadLen=i;
			memcpy(cmd, extra, i);
			break;
		}
	}

	atbm_printk_err("cmd:%s\n", cmd);
	
	atbm_printk_err("length=%d,data=%s,strheadLen=%d\n", wrqu->data.length, extra, strheadLen);
	if (strheadLen >= wrqu->data.length){
		atbm_kfree(extra);
		return -EINVAL;
	}

	pRxData = &extra[strheadLen];

	if(memcmp(cmd, "setEfuse_dcxo", 13) == 0)
	{
		CmdLine_GetSignInteger(&pRxData, &rxData);
		efuse_data_global.dcxo_trim = rxData;
		atbm_printk_err("set efuse data is dcxo[%d]\n",efuse_data_global.dcxo_trim);
	}
	else if(memcmp(cmd, "setEfuse_deltagain", 18) == 0)
	{
		CmdLine_GetSignInteger(&pRxData, &rxData);
		efuse_data_global.delta_gain1 = rxData;
		//atbm_printk_err("%s %d\n", __func__, __LINE__);
		CmdLine_GetSignInteger(&pRxData, &rxData);
		efuse_data_global.delta_gain2 = rxData;
		//atbm_printk_err("%s %d\n", __func__, __LINE__);
		CmdLine_GetSignInteger(&pRxData, &rxData);
		efuse_data_global.delta_gain3 = rxData;
		
		atbm_printk_err("set efuse data is delta_gain[%d,%d,%d]\n",
			efuse_data_global.delta_gain1,efuse_data_global.delta_gain2,efuse_data_global.delta_gain3);
	}
	else if(memcmp(cmd, "setEfuse_mac", 12) == 0)
	{
		CmdLine_GetHex(&pRxData, &rxData);
		efuse_data_global.mac[0] = rxData;
		//atbm_printk_err("%s %d\n", __func__, __LINE__);
		CmdLine_GetHex(&pRxData, &rxData);
		efuse_data_global.mac[1] = rxData;
		//atbm_printk_err("%s %d\n", __func__, __LINE__);
		CmdLine_GetHex(&pRxData, &rxData);
		efuse_data_global.mac[2] = rxData;
		//atbm_printk_err("%s %d\n", __func__, __LINE__);
		CmdLine_GetHex(&pRxData, &rxData);
		efuse_data_global.mac[3] = rxData;
		//atbm_printk_err("%s %d\n", __func__, __LINE__);
		CmdLine_GetHex(&pRxData, &rxData);
		efuse_data_global.mac[4] = rxData;
		//atbm_printk_err("%s %d\n", __func__, __LINE__);
		CmdLine_GetHex(&pRxData, &rxData);
		efuse_data_global.mac[5] = rxData;
		
		atbm_printk_err("set efuse data is mac[%02x:%02x:%02x:%02x:%02x:%02x]\n",
					efuse_data_global.mac[0],efuse_data_global.mac[1],efuse_data_global.mac[2],
					efuse_data_global.mac[3],efuse_data_global.mac[4],efuse_data_global.mac[5]);
	}

	ret = atbm_save_efuse(hw_priv, &efuse_data_global);
	if (ret == 0)
	{
		atbm_printk_err("setEfuse success \n");
	}else
	{
		atbm_printk_err("setEfuse failed [%d]\n", ret);
	}
	atbm_kfree(extra);

	return ret;
}
static int atbm_ioctl_freqoffset(struct net_device *dev, struct iw_request_info *info, union iwreq_data  *wrqu, char *extra)
{
	int i = 0;	
	int ret = 0;
	int iResult=0;
	
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	struct atbm_common *hw_priv=local->hw.priv;
	//struct atbm_vif *vif;
	struct efuse_headr efuse_d,efuse_bak;
	u32 dcxo;
	int freqErrorHz;
	int ucWriteEfuseFlag = 0;
	int channel = 0;
	char *SavaEfuse_p = NULL;
	
	u8 buff[512];
	struct rxstatus_signed rxs_s;

	memset(&rxs_s,0,sizeof(struct rxstatus));
	memset(&efuse_d,0,sizeof(struct efuse_headr));
	memset(&efuse_bak,0,sizeof(struct efuse_headr));

	if(wrqu->data.length <= 3){
		atbm_printk_err("need to input parameters\n");
		atbm_printk_err("e.g: ./iwpriv wlan0 start_rx 1,0\n");
		return -EINVAL;
	}

	if(!(extra = atbm_kmalloc(wrqu->data.length+1, GFP_KERNEL)))
		return -EINVAL;

	if((ret = copy_from_user(extra, wrqu->data.pointer, wrqu->data.length)) != 0){
		atbm_kfree(extra);
		return -EINVAL;
	}

	for(i=0;extra[i] != ',';i++){
		if(extra[i] == ','){
			break;
		}
		channel = channel * 10 +(extra[i] - 0x30);
	}

	for(i=0;i<wrqu->data.length;i++){
			if(extra[i] == ','){
				SavaEfuse_p = extra +i + 1;
				break;
			}	
		}

	if((NULL == SavaEfuse_p) || (SavaEfuse_p[0] == '\0')){
		atbm_printk_err("invalid SavaEfuse_p\n");
		atbm_printk_err("ucWriteEfuseFlag value: 1:save efuse,0:not save efuse\n");
		atbm_kfree(extra);
		return -EINVAL;
	}

	for(i=0;SavaEfuse_p[i] != '\0';i++){
		if(1 == i){
		atbm_printk_err("invalid SavaEfuse_p1\n");
		atbm_printk_err("ucWriteEfuseFlag value: 1:save efuse,0:not save efuse\n");
		atbm_kfree(extra);
			return 	-EINVAL;
		}
		ucWriteEfuseFlag = ucWriteEfuseFlag * 10 + (SavaEfuse_p[i] - 0x30);
	}

	atbm_printk_wext("channel:%d ucWriteEfuseFlag:%d\n",channel, ucWriteEfuseFlag);

	
	if((ucWriteEfuseFlag != 0) && (ucWriteEfuseFlag != 1)){
		atbm_printk_err("invalid WriteEfuseFlag\n");
		atbm_kfree(extra);
		return -EINVAL;
	}
	
	if(channel <= 0 || channel > 14){
			atbm_printk_err("invalid channel!\n");
			atbm_kfree(extra);
			return -EINVAL;
		}

	for(i=0;i<wrqu->data.length;i++){
		if(extra[i] == ',')
			extra[i] =' ';
	}
		
	if(Test_FreqOffset(hw_priv,&dcxo, &freqErrorHz, &rxs_s, channel)){
		atbm_printk_err("Test_FreqOffset Error\n");
		iResult = -1;
		goto FEEQ_ERR;
	}
	//tmp = DCXOCodeRead(hw_priv);printk("tmp %d\n"tmp);	
	if(ucWriteEfuseFlag)
	{
		atbm_printk_wext("ucWriteEfuseFlag :%d\n",ucWriteEfuseFlag);
		wsm_get_efuse_data(hw_priv,(void *)&efuse_d,sizeof(struct efuse_headr));

		if(efuse_d.version == 0)
		{
			iResult = -3;
			goto FEEQ_ERR;
		}
		efuse_d.dcxo_trim = dcxo;
		/*
		*LMC_STATUS_CODE__EFUSE_VERSION_CHANGE	failed because efuse version change  
		*LMC_STATUS_CODE__EFUSE_FIRST_WRITE, 		failed because efuse by first write   
		*LMC_STATUS_CODE__EFUSE_PARSE_FAILED,		failed because efuse data wrong, cannot be parase
		*LMC_STATUS_CODE__EFUSE_FULL,				failed because efuse have be writen full
		*/
		ret = wsm_efuse_change_data_cmd(hw_priv, &efuse_d,0);
		if (ret == LMC_STATUS_CODE__EFUSE_FIRST_WRITE)
		{
			iResult = -3;
		}else if (ret == LMC_STATUS_CODE__EFUSE_PARSE_FAILED)
		{
			iResult = -4;
		}else if (ret == LMC_STATUS_CODE__EFUSE_FULL)
		{
			iResult = -5;
		}else if (ret == LMC_STATUS_CODE__EFUSE_VERSION_CHANGE)
		{
			iResult = -6;
		}else
		{
			iResult = 0;
		}

		frame_hexdump("efuse_d", (u8 *)&efuse_d, sizeof(struct efuse_headr));
		wsm_get_efuse_data(hw_priv,(void *)&efuse_bak, sizeof(struct efuse_headr));
		frame_hexdump("efuse_bak", (u8 *)&efuse_bak, sizeof(struct efuse_headr));
		
		if(memcmp((void *)&efuse_bak,(void *)&efuse_d, sizeof(struct efuse_headr)) !=0)
		{
			iResult = -2;
		}else
		{
			iResult = 0;
		}
		
	}

	
FEEQ_ERR:	
	
	sprintf(buff, "cfo:%d,evm:%d,gainImb:%d, phaseImb:%d,dcxo:%d,result:%d (0:OK; -1:FreqOffset Error; -2:efuse hard error;"
		" -3:efuse no written; -4:efuse anaysis failed; -5:efuse full; -6:efuse version change)",
	rxs_s.Cfo,
	rxs_s.evm,
	rxs_s.GainImb,
	rxs_s.PhaseImb,
	dcxo,
	iResult
	);

	if((ret = copy_to_user(wrqu->data.pointer, buff, strlen(buff))) != 0){
		atbm_kfree(extra);
		return -EINVAL;
	}
	atbm_kfree(extra);
	return ret;
}


static int atbm_ioctl_send_singleTone(struct net_device *dev, struct iw_request_info *info, union iwreq_data *wrqu, char *ext)
{
	int i = 0;
	int ii = 0;
	int ret = -EINVAL;
	int channel = 0;
	int ucDbgPrintOpenFlag = 1;
	char *extra = NULL;
	char *ptr = NULL;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	struct atbm_common *hw_priv=local->hw.priv;
	struct atbm_vif *vif = NULL;

	if(ETF_bStartTx || ETF_bStartRx){
		atbm_printk_err("Error! already start_tx/send_singleTone, please stop_tx first!\n");
		return 0;
	}
	
	if(!(extra = atbm_kmalloc(wrqu->data.length+1, GFP_KERNEL))){
		atbm_printk_err("atbm_kmalloc failed!\n");
		return -EINVAL;
	}
	if((ret = copy_from_user(extra, wrqu->data.pointer, wrqu->data.length))){
		atbm_kfree(extra);
		ret = -EINVAL;
		goto exit;
	}
	extra[wrqu->data.length] = 0;

	atbm_printk_always("atbm_ioctl_send_singleTone:%d,%d\n",strlen(extra), wrqu->data.length);

	atbm_printk_always("atbm_ioctl_send_singleTone:%s\n",extra);

	if(wrqu->data.length < strlen("singletone,7")){
		atbm_printk_err("e.g: ./iwpriv wlan0 common singletone,<channel>\n");
		atbm_printk_err("e.g: ./iwpriv wlan0 common singletone,7\n");
		ret = -EINVAL;
		goto exit;
	}


	for(ii = 0;ii<wrqu->data.length;ii++){
		if(extra[ii] == ',')
		{
			ptr = &extra[ii+1];

			break;
		}
	}

	for(ii = 0;ii<strlen(ptr);ii++)
	{
		channel = (channel * 10) + (ptr[ii] - 0x30);
	}

	if((channel <= 0) || (channel > 14))
	{
		atbm_printk_always("[ERROR]invalid channel:%d\n",channel);
		ret = -1;
		goto exit;
	}

	hw_priv->etf_channel = channel;
	hw_priv->etf_channel_type = 0;
	hw_priv->etf_rate = 4;
	hw_priv->etf_len = 1000; 
	hw_priv->etf_greedfiled = 0;

	atbm_printk_always("channel:%d\n",channel);
	
	atbm_for_each_vif(hw_priv,vif,i){
		if((vif != NULL)){			
			atbm_printk_wext("####### send singleTone #######\n");
			
			down(&hw_priv->scan.lock);
			
			WARN_ON(wsm_write_mib(hw_priv, WSM_MIB_ID_DBG_PRINT_TO_HOST,
				&ucDbgPrintOpenFlag, sizeof(ucDbgPrintOpenFlag), vif->if_id));
			
			mutex_lock(&hw_priv->conf_mutex);
			ETF_bStartTx = 1;
			wsm_start_tx(hw_priv, vif->vif);
			mutex_unlock(&hw_priv->conf_mutex);
			break;
		}
	}
exit:
	if(extra)
		atbm_kfree(extra);
	return ret;
}

static int atbm_ioctl_set_duty_ratio(struct net_device *dev, struct iw_request_info *info, union iwreq_data *wrqu, char *ext)
{
	int i = 0;
	int ii = 0;
	int ret = -EINVAL;
	int duty_ratio = 0;
	char *extra = NULL;
	char *ptr = NULL;
	int flag = 0;
	/*u8 writebuf[4] = {0};*/
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	struct ieee80211_local *local = sdata->local;
	struct atbm_common *hw_priv=local->hw.priv;
	struct atbm_vif *vif = NULL;


	if(!(extra = atbm_kmalloc(wrqu->data.length+1, GFP_KERNEL))){
		atbm_printk_err("atbm_kmalloc failed!\n");
		return -EINVAL;
	}
	if((ret = copy_from_user(extra, wrqu->data.pointer, wrqu->data.length))){
		atbm_kfree(extra);
		ret = -EINVAL;
		goto exit;
	}
	extra[wrqu->data.length] = 0;

	atbm_printk_always("atbm_ioctl_set_duty_ratio:%s\n",extra);

	if(wrqu->data.length < strlen("duty_ratio,")){
		atbm_printk_err("e.g: ./iwpriv wlan0 common duty_ratio,<duty_ratio_value>\n");
		atbm_printk_err("e.g: ./iwpriv wlan0 common duty_ratio,0.1\n");
		atbm_kfree(extra);
		ret = -EINVAL;
		goto exit;
	}


	for(ii = 0;ii<wrqu->data.length;ii++){
		if(extra[ii] == ',')
		{
			ptr = &extra[ii+1];

			break;
		}
	}
	atbm_printk_always(" ptr:%s\n",ptr);

	for(ii = 0;ii<strlen(ptr);ii++)
	{
		if(ptr[ii] == ','){
			break;
		}
		if(ptr[ii] == '.'){
			flag = 1;
			continue;
		}	
		duty_ratio = duty_ratio* 10 +(ptr[ii] - 0x30);
	}

	if(flag == 0)
		duty_ratio = duty_ratio * 10;

	if((duty_ratio < 0) || (duty_ratio > 10))
	{
		atbm_printk_always("[ERROR]invalid duty_ratio:%d\n",duty_ratio);
		ret = -1;
		goto exit;
	}

	atbm_printk_always("duty_ratio:%d\n",duty_ratio);
	
	atbm_for_each_vif(hw_priv,vif,i){
		if((vif != NULL)){			
			wsm_write_mib(hw_priv, WSM_MIB_ID_SET_DUTY_RATIO,
				&duty_ratio, sizeof(duty_ratio), vif->if_id);
			break;
		}
	}
exit:
	if(extra)
		atbm_kfree(extra);
	return ret;
}

static int atbm_ioctl_set_ap_conf(struct net_device *dev, struct iw_request_info *info, 
										   union iwreq_data *wrqu, char *ext)
{
	struct ieee80211_internal_ap_conf conf_req;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	char *extra = NULL;
	const char* pos;
	const char* pos_end;
	int ret = 0;
	int len = 0;
	int channel = 0;
	int i;
	if(!ieee80211_sdata_running(sdata)){
		ret = -ENETDOWN;
		goto exit;
	}
	
	extra = atbm_kmalloc(wrqu->data.length+1, GFP_KERNEL);

	if(extra == NULL){
		ret =  -ENOMEM;
		goto exit;
	}

	if((ret = copy_from_user(extra, wrqu->data.pointer, wrqu->data.length))){
		return -EINVAL;
		goto exit;
	}
	
	extra[wrqu->data.length] = 0;

	for(i = 0;i<wrqu->data.length;i++){
		if(extra[i] == ',')
			extra[i] = ATBM_SPACE;
	}
	
	atbm_printk_debug("%s:%s %d\n",__func__,extra,wrqu->data.length);
	pos = atbm_skip_space(extra,wrqu->data.length);

	if(pos == NULL){
		ret = -EINVAL;
		atbm_printk_err("%s: params err\n",__func__);
		goto exit;
	}

	pos = pos+strlen("ap_conf");

	pos = atbm_skip_space(pos,wrqu->data.length-(pos-extra));

	if(pos == NULL){
		atbm_printk_err("%s: params err\n",__func__);
		ret = -EINVAL;
		goto exit;
	}
	len = wrqu->data.length - (pos - extra);

	if(len <= 0){
		ret = -EINVAL;
		atbm_printk_err("%s: len err\n",__func__);
		goto exit;
	}
	
	pos_end = memchr(pos,ATBM_TAIL,len);
	if(pos_end != NULL)
		len = pos_end - pos;

	if(len>2){
		ret = -EINVAL;
		atbm_printk_err("%s: len too long\n",__func__);
		goto exit;
	}

	if(atbm_accsii_to_int(pos,len,&channel) == false){		
		atbm_printk_err("%s:channel err(%s),len(%d)\n",__func__,pos,len);
		ret = -EINVAL;
		goto exit;
	}

	if(channel && (ieee8011_channel_valid(&sdata->local->hw,channel) == false)){
		atbm_printk_err("%s:channel (%d)\n",__func__,channel);
		ret = -EINVAL;
		goto exit;
	}
	
	memset(&conf_req,0,sizeof(struct ieee80211_internal_ap_conf));
	conf_req.channel = (u8)channel;

	if(atbm_internal_update_ap_conf(sdata,&conf_req,conf_req.channel == 0?true:false) == false)
		ret = -EINVAL;
	
exit:
	if(extra)
		atbm_kfree(extra);

	return ret;
}
static int atbm_ioctl_common_cmd(struct net_device *dev, struct iw_request_info *info, void *wrqu, char *extra)
{
	int ret = 0;

	unsigned char *ptr = NULL;
	union iwreq_data *wdata = (union iwreq_data *)wrqu;
	atbm_printk_wext("atbm_ioctl_common_cmd(), length %d\n",wdata->data.length);

	if(!(ptr = (char *)atbm_kzalloc(wdata->data.length+1, GFP_KERNEL)))
		return -ENOMEM;
	if((ret = copy_from_user(ptr, wdata->data.pointer, wdata->data.length))){
		atbm_kfree(ptr);
		return -EINVAL;
	}
#ifdef ATBM_PRIVATE_IE
	if(memcmp(ptr, "get_rssi", 8) == 0)
		ret = atbm_ioctl_get_rssi(dev, info, wrqu, extra);
	else if(memcmp(ptr, "getSigmstarEfuse", 16) == 0)
		ret = atbm_ioctl_get_SIGMSTAR_256BITSEFUSE(dev, info, wrqu, extra);
	else if(memcmp(ptr, "setSigmstarEfuse", 16) == 0)
		ret = atbm_ioctl_set_SIGMSTAR_256BITSEFUSE(dev, info, wrqu, extra);
	else if(memcmp(ptr, "channel_test_start", 18) == 0)
		ret = atbm_ioctl_channel_test_start(dev, info, wrqu, extra);
	else if(memcmp(ptr, "get_channel_idle", 16) == 0)
		ret = atbm_ioctl_get_channel_idle(dev, info, wrqu, extra);
	else 
#endif
		if(memcmp(ptr, "setEfuse_dcxo", 13) == 0)
		ret = atbm_ioctl_set_efuse(dev, info, wrqu, extra);
	else if(memcmp(ptr, "setEfuse_deltagain", 18) == 0)
		ret = atbm_ioctl_set_efuse(dev, info, wrqu, extra);
	else if(memcmp(ptr, "setEfuse_mac", 12) == 0)
		ret = atbm_ioctl_set_efuse(dev, info, wrqu, extra);
	else if(memcmp(ptr, "getEfuse", 8) == 0)
		ret = atbm_ioctl_get_efuse(dev, info, wrqu, extra);
	else if(memcmp(ptr, "freqoffset", 10) == 0)
		ret = atbm_ioctl_freqoffset(dev, info, wrqu, extra);
	else if(memcmp(ptr, "singletone", 10) == 0)
		ret = atbm_ioctl_send_singleTone(dev, info, wrqu, extra);
	else if(memcmp(ptr, "duty_ratio", 10) == 0)
		ret = atbm_ioctl_set_duty_ratio(dev, info, wrqu, extra);
	else if(memcmp(ptr, "ap_conf", 7) == 0)
		ret = atbm_ioctl_set_ap_conf(dev, info, wrqu, extra);
	else
		ret = -1;

	if(ret < 0)
		atbm_printk_err("atbm_ioctl_common_cmd(), error %s\n", ptr);
	
	atbm_kfree(ptr);
	
	return ret;
}


#ifdef CONFIG_ATBM_STA_LISTEN
static int atbm_ioctl_set_sta_channel(struct net_device *dev, struct iw_request_info *info, void *wrqu, char *extra)
{
	unsigned char *ptr = NULL;
	const unsigned char *pos = NULL;
	const unsigned char *pos_end = NULL;
	int len = 0;
	union iwreq_data *wdata = (union iwreq_data *)wrqu;
	struct ieee80211_sub_if_data *sdata = IEEE80211_DEV_TO_SUB_IF(dev);
	int channel = 0;
	int ret = 0;
	
	if(wdata->data.length <= 0){
		ret =  -EINVAL;
		goto exit;
	}
	ptr = (char *)atbm_kzalloc(wdata->data.length+1, GFP_KERNEL);

	if(ptr == NULL){
		ret = -ENOMEM;
		goto exit;
	}

	if(copy_from_user(ptr, wdata->data.pointer, wdata->data.length) != 0){
		ret = -EINVAL;
		goto exit;
	}
	
	atbm_printk_wext("%s:len(%d)(%s)\n",__func__,wdata->data.length,ptr);
	/*
	*skip space
	*/

	pos = atbm_skip_space(ptr,wdata->data.length);

	if(pos == NULL){		
		atbm_printk_err("%s:pos == NULL\n",__func__);
		ret = -EINVAL;
		goto exit;
	}
	
	len = (int)(wdata->data.length - (pos-ptr));

	pos_end = memchr(pos,ATBM_TAIL,len);

	if(pos_end != NULL)
		len = pos_end - pos;
	
	if(len >2){
		atbm_printk_err("%s:len err (%d)\n",__func__,len);
		ret = -EINVAL;
		goto exit;
	}
	
	if(atbm_accsii_to_int(pos,len,&channel) == false){		
		atbm_printk_err("%s:channel err(%s),len(%d)\n",__func__,pos,len);
		ret = -EINVAL;
		goto exit;
	}
	ret = ieee80211_set_sta_channel(sdata,channel);
	
exit:
	if(ptr)
		atbm_kfree(ptr);
	return ret;
}

#endif
const iw_handler atbm_private_handler[]={
#if USELESS
	[0] = (iw_handler)atbm_ioctl_setmac,
	[1] = (iw_handler)atbm_ioctl_smartconfig_start,	
#endif
	[2] = (iw_handler)atbm_ioctl_start_tx,
	[3] = (iw_handler)atbm_ioctl_stop_tx,
	[4] = (iw_handler)atbm_ioctl_start_rx,
	[5] = (iw_handler)atbm_ioctl_stop_rx,
	[6] = (iw_handler)atbm_ioctl_fwdbg,
	[7] = (iw_handler)atbm_ioctl_command_help,
	[8] = (iw_handler)atbm_ioctl_fwcmd,
#if USELESS
	[9] = (iw_handler)atbm_ioctl_set_rate,
	[10] = (iw_handler)atbm_ioctl_smartconfig_stop,
	[11] = (iw_handler)atbm_ioctl_set_rts,
	[12] = (iw_handler)atbm_ioctl_ctl_gi,
	[13] = (iw_handler)atbm_ioctl_getmac,
	[14] = (iw_handler)atbm_ioctl_start_wol,
	[15] = (iw_handler)atbm_ioctl_get_rx_stats,
#endif
	//#ifdef ATBM_PRIVATE_IE
	[16] = (iw_handler)atbm_ioctl_common_cmd,
	//#else
	//[16] = (iw_handler)atbm_ioctl_common_cmd,
	//#endif
#if USELESS
	[17] = (iw_handler)atbm_ioctl_smartconfig_start_v2,
#endif
	[18] = (iw_handler)atbm_ioctl_etf_result_get,
#if USELESS
	[19] = (iw_handler)atbm_ioctl_smartconfig_stop_v2,
#endif
	#ifdef ATBM_PRIVATE_IE
	/*Private IE for Scan*/
	[20] = (iw_handler)atbm_ioctl_ie_insert_user_data,
	[21] = (iw_handler)atbm_ioctl_ie_get_user_data,
	[22] = (iw_handler)atbm_ioctl_ie_send_msg,
	[23] = (iw_handler)atbm_ioctl_ie_recv_msg,
	[24] = (iw_handler)atbm_ioctl_ie_test,
	[25] = (iw_handler)atbm_ioctl_get_wifi_state,
	[26] = (iw_handler)atbm_ioctl_set_freq,
	[27] = (iw_handler)atbm_ioctl_set_txpw,
	[28] = (iw_handler)atbm_ioctl_ie_ipc_reset,
	[29] = (iw_handler)atbm_ioctl_get_rate,
	[30] = (iw_handler)atbm_ioctl_best_ch_scan,
	[31] = (iw_handler)atbm_ioctl_channle_switch,
	#else
	[20] = (iw_handler)atbm_ioctl_get_wifi_state,
	[21] = (iw_handler)atbm_ioctl_set_freq,
	[22] = (iw_handler)atbm_ioctl_set_txpw,
	[23] = (iw_handler)atbm_ioctl_get_rate,
	[24] = (iw_handler)atbm_ioctl_best_ch_scan,
#ifdef CONFIG_ATBM_STA_LISTEN
	[25] = (iw_handler)atbm_ioctl_set_sta_channel,
#endif
	#endif	
};


void register_wext_common(struct ieee80211_local *local){

#ifdef CONFIG_WIRELESS_EXT
#ifdef CONFIG_CFG80211_WEXT
	if(local->hw.wiphy->wext)
	{
	        atbm_handlers_def.standard = local->hw.wiphy->wext->standard;
	        atbm_handlers_def.num_standard = local->hw.wiphy->wext->num_standard;
		#if WIRELESS_EXT >= 17
	        atbm_handlers_def.get_wireless_stats = local->hw.wiphy->wext->get_wireless_stats;
		#endif
#endif
		#ifdef CONFIG_WEXT_PRIV
			atbm_handlers_def.num_private = sizeof(atbm_private_handler)/sizeof(atbm_private_handler[0]) ;
			atbm_handlers_def.private = atbm_private_handler ;
			atbm_handlers_def.num_private_args = sizeof(atbm_privtab)/sizeof(atbm_privtab[0]);
			atbm_handlers_def.private_args = (struct iw_priv_args *)atbm_privtab;
		#endif
#ifdef CONFIG_CFG80211_WEXT
	}
#endif
#endif

}

int atbm_ioctl_etf_result_get(struct net_device *dev, struct iw_request_info *info, union iwreq_data  *wrqu, char *extra)
{
	int ret = 0;
	u8 chipid = 0;
	u8 buff[512];

	chipid = chipversion;
	memset(buff, 0, 512);

	sprintf(buff, "%d%dcfo:%d,txevm:%d,rxevm:%d,dcxo:%d,txrssi:%d,rxrssi:%d,result:%d (0:OK; -1:FreqOffset Error; -2:efuse hard error;"
		" -3:efuse no written; -4:efuse anaysis failed; -5:efuse full; -6:efuse version change; -7:rx null)",
	gRxs_s.valid,
	chipid,
	gRxs_s.Cfo,
	gRxs_s.txevm,
	gRxs_s.evm,
	gRxs_s.dcxo,
	gRxs_s.TxRSSI,
	gRxs_s.RxRSSI,
	gRxs_s.result
	);
#if 0
	printk( "%dcfo:%d,evm:%d,dcxo:%d,txrssi:%d,rxrssi:%d,result:%d (0:OK; -1:FreqOffset Error; -2:Write efuse Failed;-3:efuse not write;-4:rx null)",
	gRxs_s.valid,
	gRxs_s.Cfo,
	gRxs_s.evm,
	gRxs_s.dcxo,
	gRxs_s.TxRSSI,
	gRxs_s.RxRSSI,
	gRxs_s.result
	);
#endif
	if((ret = copy_to_user(wrqu->data.pointer, buff, strlen(buff))) != 0){
		return -EINVAL;
	}

	return ret;
}


#endif //CONFIG_WIRELESS_EXT

extern int atbm_direct_read_reg_32(struct atbm_common *hw_priv, u32 addr, u32 *val);
extern int atbm_direct_write_reg_32(struct atbm_common *hw_priv, u32 addr, u32 val);

//get chip version funciton
u32 GetChipVersion(struct atbm_common *hw_priv)
{	
#ifndef SPI_BUS
	u32 uiRegData;

	hw_priv->sbus_ops->sbus_read_sync(hw_priv->sbus_priv,CHIP_VERSION_REG,&uiRegData,4);	
	
	return uiRegData;
#else
	return 0;
#endif
}

u32 MyRand(void)
{
	u32 random_num = 0;
	u32 randseed = 0;
	struct timex txc;
	
	do_gettimeofday(&(txc.time));
	//randseed = jiffies;
	randseed = txc.time.tv_sec;
	random_num = randseed * 1103515245 + 12345;
	return ((random_num/65536)%32768);
}

void get_test_threshold(char *param)
{
	int Freq = 0;
	int txEvm = 0;
	int rxEvm = 0;
	int rxEvmthreshold = 0;
	int txEvmthreshold = 0;
	int Txpwrmax = 0;
	int Txpwrmin = 0;
	int Rxpwrmax = 0;
	int Rxpwrmin = 0;
	int rssifilter = 0;
	int cableloss = 0;
	int default_dcxo = 0;

	if(strlen(param) != 0)
	{
		sscanf(param, "cfg:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", 
			&Freq, &txEvm, &rxEvm, &txEvmthreshold,&rxEvmthreshold,&Txpwrmax, 
			&Txpwrmin, &Rxpwrmax, &Rxpwrmin, &rssifilter, &cableloss, &default_dcxo);
		gthreshold_param.freq_ppm = Freq;
		gthreshold_param.txevm = (txEvm?txEvm:65536); //txevm filter
		gthreshold_param.rxevm = (rxEvm?rxEvm:65536); //rxevm filter
		gthreshold_param.txevmthreshold = txEvmthreshold;
		gthreshold_param.rxevmthreshold = rxEvmthreshold;
		gthreshold_param.txpwrmax = Txpwrmax;
		gthreshold_param.txpwrmin = Txpwrmin;
		gthreshold_param.rxpwrmax = Rxpwrmax;
		gthreshold_param.rxpwrmin = Rxpwrmin;
		gthreshold_param.rssifilter = rssifilter;
		gthreshold_param.cableloss = (cableloss?cableloss:30)*4;	
		gthreshold_param.default_dcxo = default_dcxo;
	}
	else
	{
		gthreshold_param.freq_ppm = 7000;
		gthreshold_param.rxevm = (rxEvm?rxEvm:65536);
		gthreshold_param.rssifilter = -100;
		gthreshold_param.txevm = (txEvm?txEvm:65536);
		gthreshold_param.txevmthreshold = 400;
		gthreshold_param.rxevmthreshold = 400;
		gthreshold_param.cableloss = 30*4;
	}

	gthreshold_param.featureid = MyRand();
	atbm_printk_always("featureid:%d\n", gthreshold_param.featureid);
	atbm_printk_always("Freq:%d,txEvm:%d,rxEvm:%d,txevmthreshold:%d,rxevmthreshold:%d,Txpwrmax:%d,Txpwrmin:%d,Rxpwrmax:%d,Rxpwrmin:%d,rssifilter:%d,cableloss:%d,default_dcxo:%d\n",
		gthreshold_param.freq_ppm,gthreshold_param.txevm,gthreshold_param.rxevm,gthreshold_param.txevmthreshold,gthreshold_param.rxevmthreshold,
		gthreshold_param.txpwrmax,gthreshold_param.txpwrmin,gthreshold_param.rxpwrmax,
		gthreshold_param.rxpwrmin,gthreshold_param.rssifilter,gthreshold_param.cableloss,gthreshold_param.default_dcxo);
}

int DCXOCodeWrite(struct atbm_common *hw_priv,u8 data)
{
#ifndef SPI_BUS
	u32 uiRegData;
	atbm_direct_read_reg_32(hw_priv, DCXO_TRIM_REG, &uiRegData);
	//hw_priv->sbus_ops->sbus_read_sync(hw_priv->sbus_priv,DCXO_TRIM_REG,&uiRegData,4);
	uiRegData &= ~0x40003F;

	uiRegData |= (((data&0x40)<<16)|(data&0x3f));
	
	atbm_direct_write_reg_32(hw_priv, DCXO_TRIM_REG, uiRegData);
	//hw_priv->sbus_ops->sbus_write_sync(hw_priv->sbus_priv,DCXO_TRIM_REG,&uiRegData,4);
#endif
	return 0;
}

u8 DCXOCodeRead(struct atbm_common *hw_priv)
{	
#ifndef SPI_BUS

	u32 uiRegData;
	u8 dcxo;
	u8 dcxo_hi,dcxo_low;

	atbm_direct_read_reg_32(hw_priv, DCXO_TRIM_REG, &uiRegData);
	//hw_priv->sbus_ops->sbus_read_sync(hw_priv->sbus_priv,DCXO_TRIM_REG,&uiRegData,4);//
	dcxo_hi = (uiRegData>>22)&0x01;
	dcxo_low = uiRegData&0x3f;
	dcxo = (dcxo_hi << 6) + (dcxo_low&0x3f);
	
	return dcxo;
#else
	return 0;
#endif
}

int etf_rx_status_get(struct atbm_common *hw_priv)
{
	int ret = 0;
	int i = 0;
	struct rxstatus rxs; 
	char *extra = NULL;
	struct atbm_vif *vif;

	atbm_printk_always("[%s]:%d\n", __func__, __LINE__);
	if(!(extra = atbm_kmalloc(sizeof(struct rxstatus), GFP_KERNEL)))
	{
		atbm_printk_err("%s:malloc failed\n", __func__);
		return -ENOMEM;	
	}

	atbm_for_each_vif(hw_priv,vif,i){
		if (vif != NULL)
		{
			ret = wsm_read_mib(hw_priv, WSM_MIB_ID_GET_ETF_RX_STATS,
				extra, sizeof(struct rxstatus), vif->if_id);
			break;
		}
	}
	memcpy(&rxs, extra, sizeof(struct rxstatus));

	if(rxs.probcnt == 0)
	{
		atbm_printk_err("%s:rxs.probcnt == 0\n", __func__);
		goto out;
	}

	if(ret == 0)
	{
		gRxs_s.evm				= rxs.evm/rxs.probcnt;
		gRxs_s.RxRSSI			= (s16)N_BIT_TO_SIGNED_32BIT(rxs.RSSI, 8)*4;
		gRxs_s.RxRSSI += gthreshold_param.cableloss;	
	}
	else
	{
		atbm_printk_err("%s:get rx status failed\n", __func__);
	}
out:
	if(extra)
		atbm_kfree(extra);
	return ret;

}


int getFreqoffsetHz(struct atbm_common *hw_priv, struct rxstatus_signed *rxs_s)
{
	struct rxstatus rxs; 
	int FreqOffsetHz;
	char *extra = NULL;
	int i = 0;
	long lcfo = 0;
	struct atbm_vif *vif;
	if(!(extra = atbm_kmalloc(16, GFP_KERNEL)))
	return -ENOMEM;

	atbm_for_each_vif(hw_priv,vif,i){
				if (vif != NULL)
				{
					WARN_ON(wsm_read_mib(hw_priv, WSM_MIB_ID_GET_ETF_RX_STATS,
						extra, 16, vif->if_id));
					break;
				}
			}
	memcpy(&rxs, extra, sizeof(struct rxstatus));
	
	atbm_kfree(extra);
	
#if 0
	printk("Cfo:%d,RSSI:%d,evm:%d,GainImb:%d, PhaseImb:%d, FreqOffsetHz:%d\n",
	rxs.Cfo,
	rxs.RSSI,
	rxs.evm,
	rxs.GainImb,
	rxs.PhaseImb,
	FreqOffsetHz
	);
#endif
	
	rxs_s->GainImb		= (s16)N_BIT_TO_SIGNED_32BIT(rxs.GainImb, 10);
	rxs_s->PhaseImb		= (s16)N_BIT_TO_SIGNED_32BIT(rxs.PhaseImb, 10);
	rxs_s->Cfo			= (s16)N_BIT_TO_SIGNED_32BIT(rxs.Cfo, 16);
	rxs_s->evm			= rxs.evm;
	rxs_s->RxRSSI			= (s8)N_BIT_TO_SIGNED_32BIT(rxs.RSSI, 8);

	lcfo = rxs_s->Cfo;
	FreqOffsetHz = (int)(((lcfo*12207)/10)*(-1));

	atbm_printk_wext("Host Rx: Cfo:%d,RxRSSI:%d,evm:%d,GainImb:%d, PhaseImb:%d, FreqOffsetHz:%d\n",
	rxs_s->Cfo,
	rxs_s->RxRSSI,
	rxs_s->evm,
	rxs_s->GainImb,
	rxs_s->PhaseImb,
	FreqOffsetHz
	);
	

	return FreqOffsetHz;
}

int _getMaxRssiInd(struct rxstatus_signed rxs_arr[], int cnt)
{
	int i=0;
	struct rxstatus_signed *rxsMax;
	int cntMax = 0;

	rxsMax = &rxs_arr[0];

	atbm_printk_wext("_getMaxRssiInd()\n");
	
	for(i=1; i<cnt; i++)
	{
#if 0
		printk("#Cfo:%d,RxRSSI:%d,evm:%d,GainImb:%d, PhaseImb:%d\n",
		rxs_arr[i].Cfo,
		rxs_arr[i].RxRSSI,
		rxs_arr[i].evm,
		rxs_arr[i].GainImb,
		rxs_arr[i].PhaseImb
		);
#endif
		if(rxs_arr[i].RxRSSI >= rxsMax->RxRSSI)
		{
			if(rxs_arr[i].evm <= rxsMax->evm)
			{
				rxsMax = &rxs_arr[i];
				cntMax = i;
			}
		}
	}

	atbm_printk_wext("Host Rx: MaxRssi[%d]#Cfo:%d,RxRSSI:%d,evm:%d,GainImb:%d, PhaseImb:%d\n",
	cntMax,
	rxsMax->Cfo,
	rxsMax->RxRSSI,
	rxsMax->evm,
	rxsMax->GainImb,
	rxsMax->PhaseImb
	);

	return cntMax;
}

int Test_FreqOffset(struct atbm_common *hw_priv, u32 *dcxo, int *pfreqErrorHz, struct rxstatus_signed *rxs_s, int channel)
{
	u8 CodeValue,CodeValuebak;
	u8 CodeStart,CodeEnd;
	int b_fail =1;
	int freqErrorHz = 0;
	int targetFreqOffset = TARGET_FREQOFFSET_HZ;
	struct atbm_vif *vif;

	char cmd[64];
	int i = 0;
	int itmp=0;
	int index=0;
	u8 ucDbgPrintOpenFlag = 1;
	struct rxstatus_signed rxs_arr[FREQ_CNT];
	int freqErrorHz_arr[FREQ_CNT];
	
	CodeValue = DCXOCodeRead(hw_priv);	
	DCXOCodeWrite(hw_priv,CodeValue);	

	if(ETF_bStartTx || ETF_bStartRx){
		atbm_printk_err("Error! already start_tx, please stop_tx first!\n");
		return b_fail;
	}


	//./iwpriv wlan0 fwdbg 1
	ucDbgPrintOpenFlag = 1;
	atbm_for_each_vif(hw_priv,vif,i){
				if (vif != NULL)
				{
					WARN_ON(wsm_write_mib(hw_priv, WSM_MIB_ID_DBG_PRINT_TO_HOST,
						&ucDbgPrintOpenFlag, sizeof(ucDbgPrintOpenFlag), vif->if_id));
					break;
				}
			}


	//start DUT Rx

	sprintf(cmd,  "monitor 1,%d,0",channel);
	
	atbm_printk_wext("start DUT Rx CMD:%s\n", cmd);
	atbm_for_each_vif(hw_priv,vif,i){
		if (vif != NULL)
		{
			ETF_bStartRx = 1;
			WARN_ON(wsm_write_mib(hw_priv, WSM_MIB_ID_FW_CMD,
				cmd, strlen(cmd) + 10, vif->if_id));
			break;
		}
	}

	CodeStart = DCXO_CODE_MINI;
	CodeEnd = DCXO_CODE_MAX;

	msleep(50);
	atbm_printk_wext("CodeValue default:%d\n",CodeValue);
	while(1)
	{
		CodeValuebak = CodeValue;
		for (itmp=0;itmp<FREQ_CNT;itmp++)
		{
			msleep(10);
			freqErrorHz_arr[itmp] = getFreqoffsetHz(hw_priv, &rxs_arr[itmp]);
		}
		index = _getMaxRssiInd(rxs_arr,FREQ_CNT);
		freqErrorHz = freqErrorHz_arr[index];
		memcpy(rxs_s, &rxs_arr[index], sizeof(struct rxstatus_signed));

		if (freqErrorHz >= targetFreqOffset)
		{
			CodeStart = CodeValue;
			CodeValue += (CodeEnd - CodeStart)/2;
			CodeStart = CodeValuebak;

			atbm_printk_wext("freqErrorHz:%d >= targetFreqOffset%d,CodeValue%d CodeEnd[%d]. CodeStart[%d]\n",
				freqErrorHz,targetFreqOffset,	CodeValue,CodeEnd , CodeStart);
			
			DCXOCodeWrite(hw_priv,CodeValue);
			if (CodeValue >= 0xff)
			{
				break;
			}
		}
		else if ((int)freqErrorHz <= -targetFreqOffset)
		{
			CodeEnd = CodeValue;
			CodeValue -= (CodeEnd - CodeStart)/2;
			CodeEnd = CodeValuebak;
			
			atbm_printk_wext("freqErrorHz:%d <= targetFreqOffset%d,CodeValue%d CodeEnd[%d]. CodeStart[%d]\n",
				freqErrorHz,targetFreqOffset,	CodeValue,CodeEnd , CodeStart);
			
			DCXOCodeWrite(hw_priv,CodeValue);
			if (CodeValue < 0x01)
			{
				break;
			}
			if (0x01 == CodeEnd)
			{
				break;
			}
		}
		else
		{
			atbm_printk_wext("[PASS]freqErrorKHz[%d] CodeValue[%d]!\n",freqErrorHz/1000,CodeValue);
			b_fail = 0;
			*dcxo = CodeValue;
			*pfreqErrorHz = freqErrorHz;
			break;
		}

		if(CodeValue == CodeValuebak)
		{
			break;
		}

	}


	mutex_lock(&hw_priv->conf_mutex);
	memset(cmd,0,sizeof(cmd));
	memcpy(cmd, "monitor 0", sizeof(cmd));
	//stop DUT Rx
#if 1	
	atbm_printk_wext("stop DUT Rx CMD:%s\n", cmd);
	atbm_for_each_vif(hw_priv,vif,i){
		if (vif != NULL)
		{
			ETF_bStartRx = 0;
			WARN_ON(wsm_write_mib(hw_priv, WSM_MIB_ID_FW_CMD,
				cmd, strlen(cmd) + 10, vif->if_id));
			break;
		}
	}
	mutex_unlock(&hw_priv->conf_mutex);

	//./iwpriv wlan0 fwdbg 1
	ucDbgPrintOpenFlag = 0;
	atbm_for_each_vif(hw_priv,vif,i){
				if (vif != NULL)
				{
					WARN_ON(wsm_write_mib(hw_priv, WSM_MIB_ID_DBG_PRINT_TO_HOST,
						&ucDbgPrintOpenFlag, sizeof(ucDbgPrintOpenFlag), vif->if_id));
					break;
				}
			}

	
#endif
	return b_fail;
}


/*
return value:
	0: success.
	1:freq offset not ok yet,need next test
	2:dcxo cal max
	3.dcxo cal min
*/
int Test_FreqOffset_v2(struct atbm_common *hw_priv, u32 *dcxo, int *pfreqErrorHz)
{
	u8 CodeValue,CodeValuebak;
	int b_fail =1;
	int freqErrorHz = 0;
	int targetFreqOffset = TARGET_FREQOFFSET_HZ;
	static int first_cal = 0;

	if(gthreshold_param.freq_ppm != 0)
		targetFreqOffset = gthreshold_param.freq_ppm;

	if((gthreshold_param.default_dcxo != 0)
		&& (gthreshold_param.default_dcxo >= 0)
		&& (gthreshold_param.default_dcxo <= 127)
		&& (first_cal == 0))
	{
		first_cal = 1;
		atbm_printk_always("wirte default dcxo when calibration firstly\n");
		CodeValue = gthreshold_param.default_dcxo;
		DCXOCodeWrite(hw_priv,CodeValue);	
	}
	else
	{
		CodeValue = DCXOCodeRead(hw_priv);	
		DCXOCodeWrite(hw_priv,CodeValue);	
	}


	atbm_printk_always("CodeValue default:%d\n",CodeValue);

	
		CodeValuebak = CodeValue;

		freqErrorHz = gRxs_s.Cfo;

		if (freqErrorHz > targetFreqOffset)
		{
			CodeStart = CodeValue;
			CodeValue += (CodeEnd - CodeStart)/2;
			CodeStart = CodeValuebak;

			atbm_printk_always("freqErrorHz[%d] > targetFreqOffset[%d],CodeValue[%d] ,CodeStart[%d], CodeEnd[%d] . \n",
				freqErrorHz,targetFreqOffset,	CodeValue, CodeStart ,CodeEnd );
			
			DCXOCodeWrite(hw_priv,CodeValue);

			b_fail = 1;
			if (CodeValue >= 126)
			{
				b_fail = 2;
			}
			if (CodeValue >= 0xff)
			{
				b_fail = 2;
			}
		}
		else if ((int)freqErrorHz < -targetFreqOffset)
		{
			CodeEnd = CodeValue;
			CodeValue -= (CodeEnd - CodeStart)/2;
			CodeEnd = CodeValuebak;

			atbm_printk_always("freqErrorHz[%d] < targetFreqOffset[%d],CodeValue[%d] ,CodeStart[%d], CodeEnd[%d] . \n",
				freqErrorHz,targetFreqOffset,	CodeValue, CodeStart ,CodeEnd );
			DCXOCodeWrite(hw_priv,CodeValue);

			b_fail = 1;
			
			if (CodeValue <= 2)
			{
				b_fail = 3;
			}
			if (0x01 == CodeEnd)
			{
				b_fail = 3;
			}
		}
		else
		{
			first_cal = 0;
			atbm_printk_always("[dcxo PASS]freqErrorKHz[%d] CodeValue[%d]!\n",freqErrorHz/1000,CodeValue);
			b_fail = 0;
			*dcxo = CodeValue;
			*pfreqErrorHz = freqErrorHz;
		}

		if((CodeEnd == CodeStart) ||
			((CodeEnd - CodeStart) == 1) ||
			((CodeEnd - CodeStart) == -1))
		{
			atbm_printk_always("CodeValue[%d] ,CodeStart[%d], CodeEnd[%d] . \n",CodeValue, CodeStart ,CodeEnd);
			b_fail = 2;
		}

		
		return b_fail;

}


static int atbm_freqoffset_save_efuse(struct atbm_common *hw_priv,struct rxstatus_signed rxs_s,u32 dcxo)
{
	int ret = 0;
	int iResult=0;
	//struct atbm_vif *vif;
	struct efuse_headr efuse_d,efuse_bak;
	
	
	//u8 buff[512];

	memset(&efuse_d,0,sizeof(struct efuse_headr));
	memset(&efuse_bak,0,sizeof(struct efuse_headr));

	

	//tmp = DCXOCodeRead(hw_priv);printk("tmp %d\n"tmp);	
	if(ucWriteEfuseFlag)
	{
		atbm_printk_always("ucWriteEfuseFlag :%d\n",ucWriteEfuseFlag);
		wsm_get_efuse_data(hw_priv,(void *)&efuse_d,sizeof(struct efuse_headr));

		if(efuse_d.version == 0)
		{
			//The first time efuse is written,all the data should be written, 
			//The production test only modifies part of the value, so efuse cannot be written.
			iResult = -3;
			goto FEEQ_ERR;
		}

		if(efuse_d.dcxo_trim == dcxo) // old dcxo equal new dcxo, no need to write efuse.
		{
			atbm_printk_always(" old dcxo equal new dcxo, no need to write efuse.\n");
			iResult = 0;
			goto FEEQ_ERR;
		}
		efuse_d.dcxo_trim = dcxo;
		/*
		*LMC_STATUS_CODE__EFUSE_VERSION_CHANGE	failed because efuse version change  
		*LMC_STATUS_CODE__EFUSE_FIRST_WRITE, 		failed because efuse by first write   
		*LMC_STATUS_CODE__EFUSE_PARSE_FAILED,		failed because efuse data wrong, cannot be parase
		*LMC_STATUS_CODE__EFUSE_FULL,				failed because efuse have be writen full
		*/
		ret = wsm_efuse_change_data_cmd(hw_priv, &efuse_d,0);
		if (ret == LMC_STATUS_CODE__EFUSE_FIRST_WRITE)
		{
			iResult = -3;
		}else if (ret == LMC_STATUS_CODE__EFUSE_PARSE_FAILED)
		{
			iResult = -4;
		}else if (ret == LMC_STATUS_CODE__EFUSE_FULL)
		{
			iResult = -5;
		}else if (ret == LMC_STATUS_CODE__EFUSE_VERSION_CHANGE)
		{
			iResult = -6;
		}else
		{
			iResult = 0;
		}
		frame_hexdump("efuse_d", (u8 *)&efuse_d, sizeof(struct efuse_headr));
		wsm_get_efuse_data(hw_priv,(void *)&efuse_bak, sizeof(struct efuse_headr));
		frame_hexdump("efuse_bak", (u8 *)&efuse_bak, sizeof(struct efuse_headr));
		
		if(memcmp((void *)&efuse_bak,(void *)&efuse_d, sizeof(struct efuse_headr)) !=0)
		{
			iResult = -2;
		}else
		{
			iResult = 0;
		}
		
	}

	
FEEQ_ERR:	
	
	/*sprintf(buff, "cfo:%d,evm:%d,gainImb:%d, phaseImb:%d,dcxo:%d,result:%d (0:OK; -1:FreqOffset Error; -2:efuse hard error;"
		" -3:efuse no written; -4:efuse anaysis failed; -5:efuse full; -6:efuse version change)",
	rxs_s.Cfo,
	rxs_s.evm,
	rxs_s.GainImb,
	rxs_s.PhaseImb,
	dcxo,
	iResult
	);*/

	//if((ret = copy_to_user(wrqu->data.pointer, buff, strlen(buff))) != 0){
	//	return -EINVAL;
	//}

	return iResult;
}


/**************************************************************************
**
** NAME         LMC_FM_GetATBMIe
**
** PARAMETERS:  pElements  -> Pointer to the Ie list
**              Length     -> Size of the Ie List
**              
** RETURNS:     Pointer to element if found or 0 otherwise.
**
** DESCRIPTION  Searches for ATBM test element  from a given IE list.
** 
**************************************************************************/
u8* LMC_FM_GetATBMIe(u8 *pElements,u16 Length)
{
  u8     ATBMIeOui[3]   = ATBM_OUI	;
  
  struct ATBM_TEST_IE  *Atbm_Ie;
	//dump_mem(pElements,Length);

   if(Length > sizeof(struct ATBM_TEST_IE)){
		pElements += Length-sizeof(struct ATBM_TEST_IE);
		Atbm_Ie =(struct ATBM_TEST_IE  *) pElements;
		/*
		DBG_Printf("Atbm_Ie->oui_type %x,Atbm_Ie->oui %x %x,size %x\n",
			Atbm_Ie->oui_type,
			Atbm_Ie->oui[2],
			ATBMIeOui[2],
			sizeof(struct ATBM_TEST_IE));
		
		dump_mem(pElements,16);*/

		 if(pElements[0]== D11_WIFI_ELT_ID){
			 if((memcmp(Atbm_Ie->oui,ATBMIeOui,3)==0)&&
			 	(Atbm_Ie->oui_type== WIFI_ATBM_IE_OUI_TYPE) ){
				return pElements;
			}
		 }
   }

  return (u8 *)NULL  ;
}//end LMC_FM_GetP2PIe()

int etf_v2_compare_test_result(void)
{	
	if((gthreshold_param.txpwrmax == 0) && (gthreshold_param.txpwrmin == 0))
	{
		gthreshold_param.txpwrmax = 65536;
		if((efuse_data_etf.specific & 0x1))//outerPA(6038)
		{

			gthreshold_param.txpwrmin = -60+gthreshold_param.cableloss;	
		}
		else
		{
			gthreshold_param.txpwrmin = -84+gthreshold_param.cableloss;	
		}
		atbm_printk_wext("Use default Txrssimin threshold[-84+120]:%d\n", gthreshold_param.txpwrmin);
	}

	if((gthreshold_param.txevmthreshold != 0) && (gRxs_s.txevm > gthreshold_param.txevmthreshold))
	{
		atbm_printk_err("Test txevm:%d > threshold txevm:%d\n", gRxs_s.txevm, gthreshold_param.txevmthreshold);
		return 1;
	}

	if((gthreshold_param.rxevmthreshold != 0) && (gRxs_s.evm > gthreshold_param.rxevmthreshold))
	{
		atbm_printk_err("Test rxevm:%d > threshold rxevm:%d\n", gRxs_s.evm, gthreshold_param.rxevmthreshold);
		return 2;
	}
	if((gthreshold_param.txpwrmax != 0) && (gthreshold_param.txpwrmin != 0) &&
		((gRxs_s.TxRSSI > gthreshold_param.txpwrmax) ||
		(gRxs_s.TxRSSI < gthreshold_param.txpwrmin)))
	{
		atbm_printk_err("Test txpower:%d,txpowermax:%d, txpowermin:%d\n",
			gRxs_s.TxRSSI, gthreshold_param.txpwrmax, gthreshold_param.txpwrmin);
		return 3;
	}
	if((gthreshold_param.rxpwrmax != 0) && (gthreshold_param.rxpwrmin!= 0) &&
		((gRxs_s.RxRSSI > gthreshold_param.rxpwrmax) ||
		(gRxs_s.RxRSSI < gthreshold_param.rxpwrmin)))
	{
		atbm_printk_err("Test rxpower:%d,rxpowermax:%d, rxpowermin:%d\n",
			gRxs_s.RxRSSI, gthreshold_param.rxpwrmax, gthreshold_param.rxpwrmin);
		return 4;
	}

	return 0;
}

void etf_v2_scan_end(struct atbm_common *hw_priv, struct ieee80211_vif *vif )
{
	int ret = 0;
	int result = 0;//(0:OK; -1:FreqOffset Error; -2:Write efuse Failed;-3:efuse not write;-4:rx fail)
	u32 dcxo = 0;
	int freqErrorHz;
	int ErrCode = -1;

	if(hw_priv->etf_test_v2 == 0)
	{
		atbm_test_rx_cnt = 0;
	}
	else
	{
		ret = etf_rx_status_get(hw_priv);
		if(ret != 0)
		{
			atbm_test_rx_cnt = 0;
			atbm_printk_err("%s:etf_rx_status_get failed,ret:%d\n", __func__, ret);
		}
	}
	msleep(10);

	if(atbm_test_rx_cnt <= 5){
		memset(&gRxs_s, 0, sizeof(struct rxstatus_signed));
		up(&hw_priv->scan.lock);
		hw_priv->etf_test_v2 = 0;
		atbm_printk_always("etf rx data[%d] less than 5 packet\n",atbm_test_rx_cnt);
#ifdef CONFIG_ATBM_PRODUCT_TEST_USE_GOLDEN_LED
		if((Atbm_Test_Success == 1) || (Atbm_Test_Success == -1)){
			gRxs_s.valid = 1;	
			Atbm_Test_Success = 0;
			atbm_test_rx_cnt = 0;
			txevm_total = 0;
			//ETF_bStartTx = 0;
			return;
		}
#endif
		gRxs_s.result = -7;		

		gRxs_s.dcxo = dcxo;
		gRxs_s.valid = 1;	
		atbm_test_rx_cnt = 0;
		txevm_total = 0;
		//ETF_bStartTx = 0;
		return;
	}
	
	gRxs_s.TxRSSI += gthreshold_param.cableloss;
	gRxs_s.txevm = txevm_total/atbm_test_rx_cnt;
	
	atbm_printk_always("Average: Cfo:%d,TxRSSI:%d,RxRSSI:%d,txevm:%d,rxevm:%d\n",	
	gRxs_s.Cfo,
	gRxs_s.TxRSSI,
	gRxs_s.RxRSSI,
	gRxs_s.txevm,
	gRxs_s.evm
	);
	
#if 0//CONFIG_ATBM_PRODUCT_TEST_NO_UART
	int efuse_remainbit = 0;

	efuse_remainbit = wsm_get_efuse_status(hw_priv, vif);
	printk("efuse remain bit:%d\n", efuse_remainbit);

	if(efuse_remainbit < 8)
	{		
		printk("##efuse is full,do not calibrte FreqOffset\n##");
		dcxo = efuse_data_etf.dcxo_trim;
		if(gthreshold_param.freq_ppm != 0)
		{
			if((gRxs_s.Cfo > -gthreshold_param.freq_ppm) &&
				(gRxs_s.Cfo < gthreshold_param.freq_ppm))
			{
				printk("#1#cur cfo:%d, targetFreqOffset:%d\n",
					gRxs_s.Cfo, gthreshold_param.freq_ppm);
				goto success;
			}
			else
			{
				printk("#1#cur cfo:%d, targetFreqOffset:%d\n",
					gRxs_s.Cfo, gthreshold_param.freq_ppm);
				goto Error;
			}
		}
		else
		{
			if((gRxs_s.Cfo > -TARGET_FREQOFFSET_HZ) &&
				(gRxs_s.Cfo < TARGET_FREQOFFSET_HZ))
			{
				printk("#2#cur cfo:%d, targetFreqOffset:%d\n",
					gRxs_s.Cfo, TARGET_FREQOFFSET_HZ);
				goto success;
			}
			else
			{
				printk("#2#cur cfo:%d, targetFreqOffset:%d\n",
					gRxs_s.Cfo, TARGET_FREQOFFSET_HZ);
				goto Error;
			}
		}
	}
#endif
	if(gthreshold_param.freq_ppm != 0)
		result = Test_FreqOffset_v2(hw_priv,&dcxo,&freqErrorHz);
	else
	{
		dcxo = efuse_data_etf.dcxo_trim;
		atbm_printk_always("Not need to Calibrate FreqOffset\n");
		result = 0;
		goto success;
	}
	
	if(result == 1)
	{
		//start next scan
		atbm_printk_always("start next scan\n");

		//mutex_lock(&hw_priv->conf_mutex);
		//wsm_stop_tx(hw_priv);
		//mutex_unlock(&hw_priv->conf_mutex);

		msleep(100);
		txevm_total = 0;
		wsm_start_tx_v2(hw_priv,vif);
	}
	else  if(result == 0)  //etf dcxo success
	{
success:
		if((ErrCode = etf_v2_compare_test_result()) != 0)
			goto Error;
		atbm_printk_always("etf test success \n");
		gRxs_s.result = atbm_freqoffset_save_efuse(hw_priv,gRxs_s,dcxo);

		gRxs_s.dcxo = dcxo;
		gRxs_s.valid = 1;
		up(&hw_priv->scan.lock);
		hw_priv->etf_test_v2 = 0;
		//del_timer_sync(&hw_priv->etf_expire_timer);
#ifdef CONFIG_ATBM_PRODUCT_TEST_USE_GOLDEN_LED
		Atbm_Test_Success = 1;
		wsm_send_result(hw_priv,vif);
#endif
		
	}else
	{
		gRxs_s.result = -1;
Error:
		gRxs_s.result = ErrCode;
		gRxs_s.dcxo = dcxo;
		gRxs_s.valid = 1;
		atbm_printk_always("etf test Fail \n");
		up(&hw_priv->scan.lock);
		hw_priv->etf_test_v2 = 0;
		//del_timer_sync(&hw_priv->etf_expire_timer);
#ifdef CONFIG_ATBM_PRODUCT_TEST_USE_GOLDEN_LED
		Atbm_Test_Success = -1;
		wsm_send_result(hw_priv,vif);
#endif

	}

	atbm_test_rx_cnt = 0;
	//ETF_bStartTx = 0;
}

void etf_v2_scan_rx(struct atbm_common *hw_priv,struct sk_buff *skb,u8 rssi )
{

	s32 Cfo;
	s32  RSSI;
	s32 tmp;
	s16 txevm;
	struct ATBM_TEST_IE  *Atbm_Ie = NULL;
	u8 *data = (u8 *)skb->data + offsetof(struct atbm_ieee80211_mgmt, u.probe_resp.variable);
	int len = skb->len - offsetof(struct atbm_ieee80211_mgmt, u.probe_resp.variable);
	Atbm_Ie = (struct ATBM_TEST_IE  *)LMC_FM_GetATBMIe(data,len);
	
	if((Atbm_Ie)
#ifdef ATBM_PRODUCT_TEST_USE_FEATURE_ID
		&& (Atbm_Ie->featureid == gthreshold_param.featureid)
#endif
		)
	{
		tmp				= Atbm_Ie->result[1];
		tmp				= (s32)N_BIT_TO_SIGNED_32BIT(tmp, 16);
		Cfo = (s32)(((tmp*12207)/10));
		 
		txevm				= (s16)N_BIT_TO_SIGNED_32BIT(Atbm_Ie->result[2], 16);
		RSSI			= (s16)N_BIT_TO_SIGNED_32BIT(Atbm_Ie->result[3], 10);
		
		if( RSSI < gthreshold_param.rssifilter)
		{
			atbm_printk_always("[%d]: Cfo:%d,TxRSSI:%d, rx dump packet,throw......\n",
			atbm_test_rx_cnt,	
			Cfo,
			RSSI
			);
			return;
		}

		if(txevm < gthreshold_param.txevm)
		{
			if(atbm_test_rx_cnt == 0)
			{		
				gRxs_s.Cfo = Cfo;
				//gRxs_s.evm = evm;
				gRxs_s.TxRSSI = RSSI;
			}else
			{

				gRxs_s.Cfo = (gRxs_s.Cfo*3 + Cfo )/4;
				//gRxs_s.evm = evm;
				gRxs_s.TxRSSI = RSSI;
				//gRxs_s.TxRSSI = (gRxs_s.TxRSSI*3*10 + RSSI*10 +5)/40;

			}

			atbm_printk_always("[%d]: Cfo1:%d, Cfo:%d,TxRSSI:%d,txevm:%d\n",
			atbm_test_rx_cnt,
			tmp,
			Cfo,
			RSSI,txevm
			);

			//printk("etf_v2_scan_rx %d,cnt %d,[0x%x,0x%x,0x%x,0x%x,0x%x]\n",Atbm_Ie->test_type,atbm_test_rx_cnt,
			//	Atbm_Ie->result[0],Atbm_Ie->result[1],Atbm_Ie->result[2],Atbm_Ie->result[3],Atbm_Ie->result[3]);
			txevm_total += txevm;
			atbm_test_rx_cnt++;
		}
		
	}

}

void atbm_etf_test_expire_timer(unsigned long arg)
{
	//printk("++ETF Test time out!++\n");
	//Atbm_Test_Success = 3;
}






/*
start return  1
stop return 0
*/
int ETF_Test_is_Start(void){
	if(ETF_bStartTx || ETF_bStartRx)
		return 1;
	return 0;
}

void ETF_Test_Init(void){
	ETF_bStartTx = 0;
	ETF_bStartRx = 0;
}

void etf_param_init(struct atbm_common *hw_priv)
{
	atbm_test_rx_cnt = 0;
	txevm_total = 0;

	ucWriteEfuseFlag = 0;

	CodeStart = DCXO_CODE_MINI;
	CodeEnd = DCXO_CODE_MAX;

	memset(&gthreshold_param, 0, sizeof(struct test_threshold));
	memset(&gRxs_s, 0, sizeof(struct rxstatus_signed));

	chipversion = GetChipVersion(hw_priv);
	atbm_printk_wext("chipversion:0x%x\n", chipversion);
}

