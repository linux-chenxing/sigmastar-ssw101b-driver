#include <linux/module.h>
#include <linux/kobject.h>
#include <net/atbm_mac80211.h>
#include <linux/kthread.h>
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
#include "module_fs.h"
#include "svn_version.h"


#define ATBM_SHOW_MSG_MAX_SIZE		PAGE_SIZE

#define ATBM_CODE__EXAMPLE_CODE		0
#define ATBM_CODE__MAX				1
#ifdef MODULE
#define atbm_module_parent			(&THIS_MODULE->mkobj.kobj)
#else
#define atbm_module_parent			(NULL)
#endif
extern struct atbm_common 			*g_hw_priv;

#define ATBM_CODE_STR__ATTR_EXAMPLE	"attr_example"

#define atbm_module_show_put(_show,...)	\
	do{										\
		int ret = 0;						\
		ret = snprintf((_show)->show_buff+(_show)->show_count,(_show)->show_size-(_show)->show_count,__VA_ARGS__);		\
		if(ret>=0)	(_show)->show_count+=ret;				\
	}while(0)

struct atbm_module_show
{
	char *show_buff;
	int  show_count;
	int  show_size;
};

struct atbm_store_code {
	const char *label;
	int (*code_cmd)(struct atbm_common *hw_priv,const char *buf,int len);
};
static int atbm_module_attr_example(struct atbm_common *hw_priv,const char *buf,int len)
{
	printk(KERN_ERR "%s\n",__func__);
	return 1;
}

struct atbm_store_code atbm_store_code_buff[]={
	[ATBM_CODE__EXAMPLE_CODE] = {.label=ATBM_CODE_STR__ATTR_EXAMPLE,.code_cmd = atbm_module_attr_example},
};

static ssize_t atbm_module_decode_common_store(const char *buf, size_t n)
{
	char *p;
	int len;
	struct atbm_store_code *store_code;
	int code = 0;
	p = memchr(buf, '\n', n); 
	len = p ? p - buf : n;
	
	for(store_code = &atbm_store_code_buff[code];code<ATBM_CODE__MAX;code++){
		if(store_code->code_cmd&&strlen(store_code->label)
		   &&!strncmp(buf, store_code->label, strlen(store_code->label))){
		   printk(KERN_ERR "%s:Process\n",store_code->label);
		   code = store_code->code_cmd(atbm_hw_priv_dereference(),buf,len);
		   break;
		}
	}
	return code;
}
static ssize_t atbm_module_common_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	ssize_t mg_size = 0;
	atbm_module_muxlock();
	mg_size = atbm_module_decode_common_store(buf,n);
	atbm_module_muxunlock();
	return mg_size;
}
static ssize_t atbm_module_common_show(struct kobject *kobj,
			     struct kobj_attribute *attr, char *buf)
{
	#define ATTR_EXAMPLE	ATBM_CODE_STR__ATTR_EXAMPLE"\n"
	ssize_t len = 0;
	len = strlen(ATTR_EXAMPLE)+1;
	memcpy(buf,ATTR_EXAMPLE,len);
	#undef ATTR_EXAMPLE
	return len;
}
static void atbm_module_firmware_caps_show(struct atbm_module_show *show_buff,struct atbm_common *hw_priv)
{
	#define WSM_CAP(cap)  !!(hw_priv->wsm_caps.firmwareCap&cap)
	
	atbm_module_show_put(show_buff,LIGHT"Firmare Cap:"NORMAL ENTER);
	atbm_module_show_put(show_buff,"PRIVATE_IE  [%d]\n" ,WSM_CAP(CAPABILITIES_ATBM_PRIVATE_IE)); 
	atbm_module_show_put(show_buff,"NVR_IPC     [%d]\n" ,WSM_CAP(CAPABILITIES_NVR_IPC));  
	atbm_module_show_put(show_buff,"NO_CONFIRM  [%d]\n" ,WSM_CAP(CAPABILITIES_NO_CONFIRM));
	atbm_module_show_put(show_buff,"SDIO_PATCH  [%d]\n" ,WSM_CAP(CAPABILITIES_SDIO_PATCH));
	atbm_module_show_put(show_buff,"NO_BACKOFF  [%d]\n" ,WSM_CAP(CAPABILITIES_NO_BACKOFF));
	atbm_module_show_put(show_buff,"CFO         [%d]\n" ,WSM_CAP(CAPABILITIES_CFO));  
	atbm_module_show_put(show_buff,"AGC         [%d]\n" ,WSM_CAP(CAPABILITIES_AGC));  
	atbm_module_show_put(show_buff,"TXCAL       [%d]\n" ,WSM_CAP(CAPABILITIES_TXCAL));
	atbm_module_show_put(show_buff,"CTS_BUG     [%d]\n" ,WSM_CAP(CAPABILITIES_CTS_BUG));
	atbm_module_show_put(show_buff,"MONITOR     [%d]\n" ,WSM_CAP(CAPABILITIES_MONITOR));  
	atbm_module_show_put(show_buff,"CUSTOM      [%d]\n" ,WSM_CAP(CAPABILITIES_CUSTOM));
	atbm_module_show_put(show_buff,"SMARTCONFIG [%d]\n" ,WSM_CAP(CAPABILITIES_SMARTCONFIG));
	atbm_module_show_put(show_buff,"ETF         [%d]\n" ,WSM_CAP(CAPABILITIES_ETF));
	atbm_module_show_put(show_buff,"LMAC_RATECTL[%d]\n" ,WSM_CAP(CAPABILITIES_LMAC_RATECTL));  
	atbm_module_show_put(show_buff,"LMAC_TPC    [%d]\n" ,WSM_CAP(CAPABILITIES_LMAC_TPC));  
	atbm_module_show_put(show_buff,"LMAC_TEMPC  [%d]\n" ,WSM_CAP(CAPABILITIES_LMAC_TEMPC));  
	atbm_module_show_put(show_buff,"USE_IPC     [%d]\n" ,WSM_CAP(CAPABILITIES_USE_IPC));
	atbm_module_show_put(show_buff,"OUTER_PA    [%d]\n" ,WSM_CAP(CAPABILITIES_OUTER_PA));
	atbm_module_show_put(show_buff,"HW_CHECKSUM [%d]\n" ,WSM_CAP(CAPABILITIES_HW_CHECKSUM));
	atbm_module_show_put(show_buff,"MULTI_RX    [%d]\n" ,WSM_CAP(CAPABILITIES_SINGLE_CHANNEL_MULTI_RX));
	atbm_module_show_put(show_buff,"USB_RECOVERY_BUG      [%d]\n" ,WSM_CAP(CAPABILITIES_USB_RECOVERY_BUG)); 	
	atbm_module_show_put(show_buff,"POWER_CONSUMPTION     [%d]\n" ,WSM_CAP(CAPABILITIES_POWER_CONSUMPTION));
	atbm_module_show_put(show_buff,"RSSI_DECIDE_TXPOWER   [%d]\n" ,WSM_CAP(CAPABILITIES_RSSI_DECIDE_TXPOWER));
	atbm_module_show_put(show_buff,"RTS_LONG_DUR          [%d]\n" ,WSM_CAP(CAPABILITIES_RTS_LONG_DURATION));
	atbm_module_show_put(show_buff,"TX_CFO_PPM_CORRECTION [%d]\n" ,WSM_CAP(CAPABILITIES_TX_CFO_PPM_CORRECTION));
	atbm_module_show_put(show_buff,"NOISE_SET_DCXO        [%d]\n" ,WSM_CAP(CAPABILITIES_NOISE_SET_DCXO));
}

static void atbm_module_driver_caps_show(struct atbm_module_show *show_buff,struct atbm_common *hw_priv)
{
	atbm_module_show_put(show_buff,LIGHT"Driver Cap:"NORMAL ENTER);
#ifdef USB_BUS
	atbm_module_show_put(show_buff,"HIF_TYPE    [%s]\n","USB");
#endif
#ifdef SDIO_BUS
	atbm_module_show_put(show_buff,"HIF_TYPE    [%s]\n","SDIO");
#endif 
#ifdef SPI_BUS
	atbm_module_show_put(show_buff,"HIF_TYPE    [%s]\n","SPI");
#endif 
#ifdef ATBM_NOT_SUPPORT_40M_CHW
	atbm_module_show_put(show_buff,"HW_CHW      [%s]\n","40M");
#else
	atbm_module_show_put(show_buff,"HW_CHW      [%s]\n","20M");
#endif
#ifdef CONFIG_ATBM_5G_PRETEND_2G
	atbm_module_show_put(show_buff,"BAND_SUPPORT[%s]\n","5G and 2G");
#else
	atbm_module_show_put(show_buff,"BAND_SUPPORT[%s]\n","only 2G");
#endif
	
}
static ssize_t atbm_module_show_system_info(struct kobject *kobj,
			     struct kobj_attribute *attr, char *buf)
{
	struct atbm_module_show sys_show;
	struct atbm_common *hw_priv = NULL;
	
	sys_show.show_buff = buf;
	sys_show.show_size = ATBM_SHOW_MSG_MAX_SIZE;
	sys_show.show_count = 0;
	
	atbm_module_muxlock();
	if(atbm_hw_priv_dereference() == NULL){
		atbm_module_show_put(&sys_show,"system info not exit,please plug chip\n");
		goto exit;
	}
	hw_priv = atbm_hw_priv_dereference();
	atbm_module_show_put(&sys_show,LIGHT"DriverVer   [%d]"NORMAL ENTER ,DRIVER_VER);
	atbm_module_show_put(&sys_show,LIGHT"FirmwareVer [%d]"NORMAL ENTER ,hw_priv->wsm_caps.firmwareVersion);
	atbm_module_firmware_caps_show(&sys_show,hw_priv);
	atbm_module_driver_caps_show(&sys_show,hw_priv);
exit:
	atbm_module_muxunlock();

	return sys_show.show_count;
}
static struct kobject *atbm_module_kobj = NULL;

static struct kobj_attribute atbm_module_common_attr  = __ATTR(atbm_cmd, 0644,atbm_module_common_show,      atbm_module_common_store);
static struct kobj_attribute atbm_module_sysinfo_attr = __ATTR(atbm_sys,  0444,atbm_module_show_system_info, NULL);


static struct attribute *atbm_module_attribute_group[]= {
	&atbm_module_common_attr.attr,
	&atbm_module_sysinfo_attr.attr,
	NULL,
};
static struct attribute_group atbm_module_attr_group = {
	.attrs = atbm_module_attribute_group,
};

int atbm_module_attribute_init(void)
{
	int error;
	struct kobject *parent = atbm_module_parent;
	
	atbm_module_kobj = kobject_create_and_add("atbmfs",parent);
	if (!atbm_module_kobj){
		return -EINVAL;
	}
	error = sysfs_create_group(atbm_module_kobj, &atbm_module_attr_group);
	if (error)
		kobject_put(atbm_module_kobj);
	return error;
}
void atbm_module_attribute_exit(void)
{
	if(atbm_module_kobj == NULL)
		return;
	sysfs_remove_group(atbm_module_kobj, &atbm_module_attr_group);
	kobject_put(atbm_module_kobj);
}
