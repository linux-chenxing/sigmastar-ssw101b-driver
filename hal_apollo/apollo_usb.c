/*
 * Mac80211 USB driver for sigmastar APOLLO device
 * *
 * Copyright (c) 2016, sigmastar
 * Author:
 *
 * Based on apollo code Copyright (c) 2010, ST-Ericsson
 * Author: Dmitry Tarnyagin <dmitry.tarnyagin@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifdef CONFIG_USB_AGGR_URB_TX
#undef CONFIG_USE_DMA_ADDR_BUFFER
#define CONFIG_USE_DMA_ADDR_BUFFER
#endif //SSTAR_NEW_USB_AGGR_TX

 #define DEBUG 1
#include <linux/version.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <net/Sstar_mac80211.h>
#include <linux/kthread.h>
#include <linux/device.h>
#include <linux/usb.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/kref.h>
#include <linux/suspend.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/rtnetlink.h>

#include "mac80211/ieee80211_i.h"
#include "mac80211/driver-ops.h"
#include "hwio.h"
#include "apollo.h"
#include "sbus.h"
#include "apollo_plat.h"
#include "debug.h"
#include "bh.h"
#include "svn_version.h"
#include "module_fs.h"

#if defined(CONFIG_HAS_EARLYSUSPEND)
#ifdef SSTAR_PM_USE_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#endif

#ifdef SSTAR_PM_USE_EARLYSUSPEND
#pragma message("Suspend Remove Interface")
#endif

#ifdef CONFIG_TX_NO_CONFIRM
#pragma message("Tx No Confirm")
#endif

#ifdef CONFIG_USB_AGGR_URB_TX
#pragma message("Usb Aggr Tx")
#endif

#ifdef CONFIG_USE_DMA_ADDR_BUFFER
#pragma message("Usb Dma Buff")
#endif

#ifdef CONFIG_WSM_CMD_XMIT_DIRECTLY
#pragma message("Cmd Xmit Directly")
#endif

#ifdef CONFIG_USB_DATA_XMIT_DIRECTLY
#pragma message("Date Xmit Directly")
#endif

#define DBG_EVENT_LOG
#include "dbg_event.h"
MODULE_DESCRIPTION("mac80211 sigmastar apollo wifi USB driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("Sstar_wlan");
#define WSM_TX_SKB 1

#define SSTAR_USB_EP0_MAX_SIZE 64
#define SSTAR_USB_EP1_MAX_RX_SIZE 512
#define SSTAR_USB_EP2_MAX_TX_SIZE 512

#define SSTAR_USB_VENQT_WRITE  0x40
#define SSTAR_USB_VENQT_READ 0xc0

#define usb_printk(...)
/*usb vendor define type, EP0, bRequest*/
enum {
	VENDOR_HW_READ=0,
	VENDOR_HW_WRITE=1,
	VENDOR_HW_RESVER=2,
	VENDOR_SW_CPU_JUMP=3,/*cpu jump to real lmac code,after fw download*/
	VENDOR_SW_READ=4,
	VENDOR_SW_WRITE=5,	
#if (PROJ_TYPE<ARES_B)
	VENDOR_DBG_SWITCH=6,
#else
	VENDOR_HW_RESET	=6,
#endif
	VENDOR_EP0_CMD=7,
};

 enum Sstar_system_action{
 	SSTAR_SYSTEM_REBOOT,
	SSTAR_SYSTEM_RMMOD,
	SSTAR_SYSTEM_NORMAL,
	SSTAR_SYSTEM_MAX,
 };

extern void Sstar_wifi_run_status_set(int status);
 int Sstar_usb_pm(struct sbus_priv *self, bool  auto_suspend);
 
 int Sstar_usb_pm_async(struct sbus_priv *self, bool  auto_suspend);
 void Sstar_usb_urb_put(struct sbus_priv *self,unsigned long *bitmap,int id,int tx);
 int Sstar_usb_urb_get(struct sbus_priv *self,unsigned long *bitmap,int max_urb,int tx);
 static void Sstar_usb_lock(struct sbus_priv *self);
 static void Sstar_usb_unlock(struct sbus_priv *self);
 
extern void wsm_alloc_tx_buffer_NoLock(struct Sstar_common *hw_priv);
extern int wsm_release_tx_buffer_NoLock(struct Sstar_common *hw_priv, int count);
int wsm_release_vif_tx_buffer_Nolock(struct Sstar_common *hw_priv, int if_id,
				int count);
void Sstar_usb_release_tx_err_urb(struct sbus_priv *self,unsigned long *bitmap,int tx);
int Sstar_rx_bh_flush(struct Sstar_common *hw_priv);


#ifdef CONFIG_USE_DMA_ADDR_BUFFER
#define PER_PACKET_LEN 2048 //must pow 512
 char * Sstar_usb_get_txDMABuf(struct sbus_priv *self); 
 char * Sstar_usb_pick_txDMABuf(struct sbus_priv *self); 
 void Sstar_usb_free_txDMABuf(struct sbus_priv *self);
 void Sstar_usb_free_txDMABuf_all(struct sbus_priv *self,u8 * buffer,int cnt);
 

 #define SSTAR_USB_ALLOC_URB(iso)		usb_alloc_urb(iso, GFP_ATOMIC)
#define SSTAR_USB_SUBMIT_URB(pUrb)		usb_submit_urb(pUrb, GFP_ATOMIC)
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 5, 0))

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35))
#define Sstar_usb_buffer_alloc(dev, size, dma) usb_alloc_coherent((dev), (size), (in_interrupt() ? GFP_ATOMIC : GFP_KERNEL), (dma))
#define Sstar_usb_buffer_free(dev, size, addr, dma) usb_free_coherent((dev), (size), (addr), (dma))
#else //(LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35))
#define Sstar_usb_buffer_alloc(dev, size, dma) usb_buffer_alloc((dev), (size), (in_interrupt() ? GFP_ATOMIC : GFP_KERNEL), (dma))
#define Sstar_usb_buffer_free(dev, size, addr, dma) usb_buffer_free((dev), (size), (addr), (dma))
#endif

#else
#define Sstar_usb_buffer_alloc(_dev, _size, _dma)	Sstar_kmalloc(_size, GFP_ATOMIC)
#define Sstar_usb_buffer_free(_dev, _size, _addr, _dma)	Sstar_kfree(_addr) 
#endif //#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 5, 0)



#define Sstar_dma_addr_t					dma_addr_t
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 5, 0))
#define SSTAR_USB_FILL_HTTX_BULK_URB(pUrb,	\
				pUsb_Dev,	\
				uEndpointAddress,		\
				pTransferBuf,			\
				BufSize,				\
				Complete,	\
				pContext,				\
				TransferDma)				\
  				do{	\
					usb_fill_bulk_urb(pUrb, pUsb_Dev, usb_sndbulkpipe(pUsb_Dev, uEndpointAddress),	\
								pTransferBuf, BufSize, Complete, pContext);	\
					pUrb->transfer_dma	= TransferDma; \
					pUrb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;	\
				}while(0)
#else
#define SSTAR_USB_FILL_HTTX_BULK_URB(pUrb,	\
				pUsb_Dev,				\
				uEndpointAddress,		\
				pTransferBuf,			\
				BufSize,				\
				Complete,				\
			       pContext,	\
					TransferDma)	\
  				do{	\
					FILL_BULK_URB(pUrb, pUsb_Dev, usb_sndbulkpipe(pUsb_Dev, uEndpointAddress),	\
								pTransferBuf, BufSize, Complete, pContext);	\
				}while(0)
#endif	

#endif //#ifdef CONFIG_USE_DMA_ADDR_BUFFER
#ifdef CONFIG_NFRAC_40M
#define DPLL_CLOCK 40
#elif defined (CONFIG_NFRAC_26M)
#define DPLL_CLOCK 26
#endif
struct build_info{
	int ver;
	int dpll;
	char driver_info[64];
};
typedef int (*sbus_complete_handler)(struct urb *urb);
static int Sstar_usb_receive_data(struct sbus_priv *self,unsigned int addr,void *dst, int count,sbus_callback_handler hander);
static int Sstar_usb_xmit_data(struct sbus_priv *self,unsigned int addr,const void *pdata, int len,sbus_callback_handler hander);

#if 0
const char DRIVER_INFO[]={"[===USB-APOLLO=="__DATE__" "__TIME__"""=====]"};
#else
#if (PROJ_TYPE==ARES_A)
#define PROJ_TYPE_STR "[===USB-ARES_A=="
#elif (PROJ_TYPE==ARES_B)
#define PROJ_TYPE_STR "[===USB-ARES_B=="
#elif (PROJ_TYPE==ATHENA_B)
#define PROJ_TYPE_STR "[===USB-ATHENA_B=="
#else
#define PROJ_TYPE_STR "[===USB-OTHER=="
#endif  // (PROJ_TYPE==ATHENA_B)
#endif
#pragma	message(PROJ_TYPE_STR)
const char DRIVER_INFO[]={PROJ_TYPE_STR};
static int driver_build_info(void)
{
	struct build_info build;
	build.ver=DRIVER_VER;
	build.dpll=DPLL_CLOCK;
	memcpy(build.driver_info,(void*)DRIVER_INFO,sizeof(DRIVER_INFO));
	Sstar_printk_init("SVN_VER=%d,DPLL_CLOCK=%d,BUILD_TIME=%s\n",build.ver,build.dpll,build.driver_info);
	return 0;
}
int G_tx_urb=0;
int G_rx_urb;
int wifi_module_exit =0;
#define Sstar_wifi_get_status()			SSTAR_VOLATILE_GET(&wifi_module_exit)
#define Sstar_wifi_set_status(status)	SSTAR_VOLATILE_SET(&wifi_module_exit,status)
//in Sstar_usb_probe set 1, exit Sstar_usb_probe set 0, 
//when  wifi_usb_probe_doing =1 ,can't not wifi_module_exit, must wait until wifi_usb_probe_doing==0
static int wifi_usb_probe_doing =0;
static int wifi_tx_urb_pending =0;
#define TEST_URB_NUM 5
#ifdef CONFIG_USB_AGGR_URB_TX
#define URB_AGGR_NUM 16
#define PER_BUFF_AGGR_NUM 8
#define TX_URB_NUM 4
#define RX_URB_NUM 4
#define BUFF_ALLOC_LEN (PER_BUFF_AGGR_NUM*PER_PACKET_LEN)
#else
#define TX_URB_NUM 32
#define RX_URB_NUM 16
#endif
struct sbus_urb {
	struct sbus_priv* obj;
	struct urb *test_urb;
	struct sk_buff *test_skb;
	void *data;
	sbus_callback_handler	callback_handler;
	int urb_id;
	int test_pending;
	int test_seq;
	int test_hwChanId;
#ifdef CONFIG_USE_DMA_ADDR_BUFFER
	Sstar_dma_addr_t dma_transfer_addr;	/* (in) dma addr for transfer_buffer */
	
	int pallocated_buf_len;
	int frame_cnt;
	int dma_buff_alloced;
#endif
	u8 *pallocated_buf;
};
struct dvobj_priv{
	struct usb_device *pusbdev;
	struct usb_interface *pusbintf;
	//struct sk_buff *rx_skb;
#ifdef CONFIG_USE_DMA_ADDR_BUFFER
	Sstar_dma_addr_t tx_dma_addr;
	char * tx_dma_addr_buffer;
	unsigned long tx_dma_addr_buffer_len;
	struct sbus_urb *tx_save_urb;
	int tx_save_urb_data_len;
#ifdef CONFIG_USB_AGGR_URB_TX
	char * NextAllocPost;
	char * NextFreePost;
	char * tx_dma_addr_buffer_end;
	bool tx_dma_addr_buffer_full;
	int  free_dma_buffer_cnt;
	int  total_dma_buffer_cnt;
#endif  //CONFIG_USB_AGGR_URB_TX
#endif //CONFIG_USE_DMA_ADDR_BUFFER
	struct sbus_urb rx_urb[RX_URB_NUM];
	struct sbus_urb tx_urb[TX_URB_NUM];
#ifdef CONFIG_WSM_CMD_XMIT_DIRECTLY
	struct sbus_urb wsm_urb;
	struct usb_anchor wsm_submitted;
	struct urb *wait_urb;
#endif
	struct usb_anchor tx_submitted;
	struct usb_anchor rx_submitted;
#ifdef CONDIF_SSTAR_CTRL_REQUEST_ASYNC
	struct usb_anchor ctrl_submitted;
	atomic_t		  ctrl_err;
#endif
	unsigned long	 rx_urb_map[BITS_TO_LONGS(RX_URB_NUM)];
	unsigned long	 tx_urb_map[BITS_TO_LONGS(TX_URB_NUM)];
	unsigned long	 tx_err_urb_map[BITS_TO_LONGS(TX_URB_NUM)];	
#ifdef CONFIG_TX_NO_CONFIRM
	unsigned long	 txpending_urb_map[BITS_TO_LONGS(TX_URB_NUM)];
#endif  //CONFIG_TX_NO_CONFIRM
	int tx_test_seq_need; //just fot test
	int tx_test_hwChanId_need;//just fot test
	struct urb *cmd_urb;
	struct sbus_priv *self;
	struct net_device *netdev;
	u8	usb_speed; // 1.1, 2.0 or 3.0
	u8	nr_endpoint;
	int ep_in;
	int ep_in_size;
	int ep_out;
	int ep_out_size;
	int	ep_num[6]; //endpoint number
	struct sk_buff *suspend_skb;
	unsigned long suspend_skb_len;
#ifdef CONFIG_USE_DMA_ADDR_BUFFER
#endif
};
struct sbus_priv {
	struct dvobj_priv *drvobj;
	struct Sstar_common	*core;
	struct Sstar_platform_data *pdata;
	//struct sk_buff *rx_skb;
	//struct sk_buff *tx_skb;
	
#ifdef USB_USE_TASTLET_TXRX
	struct tasklet_struct tx_cmp_tasklet;
	struct tasklet_struct rx_cmp_tasklet;
#else
	struct workqueue_struct 	*tx_workqueue;
	struct workqueue_struct 	*rx_workqueue;
	struct work_struct rx_complete_work;
	struct work_struct tx_complete_work;
#endif
	bool            rx_running;
	void 			*tx_data;
	int   			tx_vif_selected;
	unsigned int   	tx_hwChanId;
	unsigned int   	rx_seqnum;
	atomic_t 			rx_lock;
	atomic_t 			tx_lock;
	//sbus_callback_handler	tx_callback_handler;
	//sbus_callback_handler	rx_callback_handler;
	spinlock_t		lock;
	struct mutex 	sbus_mutex;
	int 			auto_suspend;
	int 			suspend;
	u8 *usb_data;
	u8 *usb_req_data;
	#if defined(CONFIG_HAS_EARLYSUSPEND)
	#ifdef SSTAR_PM_USE_EARLYSUSPEND
	struct early_suspend Sstar_early_suspend;
	struct semaphore early_suspend_lock;
	atomic_t early_suspend_state;
	#endif
	#endif /* CONFIG_HAS_EARLYSUSPEND && DHD_USE_EARLYSUSPEND */
	struct sbus_wtd         * wtd;
};
struct sbus_wtd {
	int 	wtd_init;
	struct task_struct		*wtd_thread;
	wait_queue_head_t		wtd_evt_wq;
	atomic_t				wtd_term;
	atomic_t				wtd_run;
	atomic_t				wtd_probe;
};

static const struct usb_device_id Sstar_usb_ids[] = {
	/* Asus */
	{USB_DEVICE(0x1b20, 0x8888)},
	{ /* end: all zeroes */}
};
struct Sstar_usb_driver_ref {
	struct kref ref;
	struct usb_driver *Sstar_driver;
};

struct Sstar_usb_driver_ref Sstar_usb_kref;

MODULE_DEVICE_TABLE(usb, Sstar_usb_ids);
#ifdef CONFIG_TX_NO_CONFIRM
int Sstar_usb_free_tx_wsm(struct sbus_priv *self,struct sbus_urb *tx_urb);
#endif //#ifdef CONFIG_TX_NO_CONFIRM
static int  Sstar_usb_init(void);
static void  Sstar_usb_exit(void);
static struct sbus_wtd         g_wtd={
	.wtd_init  = 0,
	.wtd_thread = NULL,
};
struct mutex Sstar_usb_mod_lock;	
struct dvobj_priv *g_dvobj= NULL;
#define Sstar_usb_module_lock_int()		mutex_init(&Sstar_usb_mod_lock)
#define Sstar_usb_module_lock()			mutex_lock(&Sstar_usb_mod_lock)
#define Sstar_usb_module_unlock()		mutex_unlock(&Sstar_usb_mod_lock)
#define Sstar_usb_module_trylock() 			mutex_trylock(&Sstar_usb_mod_lock)
#define Sstar_usb_module_lock_release()	mutex_destroy(&Sstar_usb_mod_lock)
#define Sstar_usb_module_lock_check()	lockdep_assert_held(&Sstar_usb_mod_lock)
#define Sstar_usb_dvobj_assign_pointer(p)	SSTAR_VOLATILE_SET(&g_dvobj,p)
#define Sstar_usb_dvobj_dereference()		SSTAR_VOLATILE_GET(&g_dvobj)
static void  Sstar_usb_fw_sync(struct Sstar_common *hw_priv,struct dvobj_priv *dvobj);
extern struct Sstar_common *g_hw_priv;

extern void Sstar_tx_tasklet(unsigned long priv);
extern void Sstar_rx_tasklet(unsigned long priv);

#ifndef USB_USE_TASTLET_TXRX

void Sstar_tx_complete_work(struct work_struct *work)
{
	struct sbus_priv *self =
			container_of(work, struct sbus_priv , tx_complete_work);
	Sstar_tx_tasklet((unsigned long)self->core);
}

void Sstar_rx_complete_work(struct work_struct *work)
{
	struct sbus_priv *self =
			container_of(work, struct struct sbus_priv , rx_complete_work);
	Sstar_rx_tasklet((unsigned long)self->core);
}
#endif

static int Sstar_usb_xmit_init(struct sbus_priv *self)
{
	
#ifdef USB_USE_TASTLET_TXRX
	Sstar_printk_init("Sstarwifi USB_USE_TASTLET_TXRX enable (%p)\n",self->core);
	tasklet_init(&self->tx_cmp_tasklet, Sstar_tx_tasklet, (unsigned long)self->core);
#else
	struct Sstar_common *hw_priv = self->core;
	Sstar_printk_init("Sstarwifi INIT_WORK enable\n");
	INIT_WORK(&self->tx_complete_work, Sstar_tx_complete_work);
	self->tx_workqueue= create_singlethread_workqueue(ieee80211_alloc_name(hw_priv->hw,"tx_workqueue")/*"tx_workqueue"*/);

	if(self->tx_workqueue == NULL)
		return -1;
#endif
	return 0;
}

static int Sstar_usb_xmit_deinit(struct sbus_priv *self)
{
	
#ifdef USB_USE_TASTLET_TXRX
	tasklet_kill(&self->tx_cmp_tasklet);
#else
	flush_workqueue(self->tx_workqueue);
	destroy_workqueue(self->tx_workqueue);
	self->tx_workqueue = NULL;	
#endif
	return 0;
}
static int Sstar_usb_rev_init(struct sbus_priv *self)
{
#ifdef USB_USE_TASTLET_TXRX
	Sstar_printk_init("Sstarwifi USB_USE_TASTLET_TXRX enable (%p)\n",self->core);
	tasklet_init(&self->rx_cmp_tasklet, Sstar_rx_tasklet, (unsigned long)self->core);
#else
	struct Sstar_common *hw_priv = self->core;
	Sstar_printk_init("Sstarwifi INIT_WORK enable\n");
	INIT_WORK(&self->rx_complete_work, Sstar_rx_complete_work);
	self->rx_workqueue= create_singlethread_workqueue(ieee80211_alloc_name(hw_priv->hw,"rx_workqueue")/*"rx_workqueue"*/);
	if(self->rx_workqueue == NULL)
		return -1;
#endif
    return 0;
}

static int Sstar_usb_rev_deinit(struct sbus_priv *self)
{
	
#ifdef USB_USE_TASTLET_TXRX
	tasklet_kill(&self->rx_cmp_tasklet); 
#else
	flush_workqueue(self->rx_workqueue);
	destroy_workqueue(self->rx_workqueue);
	self->rx_workqueue = NULL;	
#endif
	return 0;
}

static int Sstar_usb_xmit_schedule(struct sbus_priv *self)
{
	struct Sstar_common *hw_priv = self->core;

	if(atomic_read(&hw_priv->bh_term)|| hw_priv->bh_error || (hw_priv->bh_thread == NULL))
		return -1;
#ifdef USB_USE_TASTLET_TXRX
	
	tasklet_schedule(&self->tx_cmp_tasklet);
#else
	if(self->tx_workqueue==NULL)
	{
		Sstar_printk_err("Sstar_bh_schedule_tx term ERROR\n");
		return -1;
	}
	queue_work(self->tx_workqueue,&self->tx_complete_work);
#endif
	return 0;

}
static int Sstar_usb_rev_schedule(struct sbus_priv *self)
{
	struct Sstar_common *hw_priv = self->core;
	
	if(atomic_read(&hw_priv->bh_term)|| hw_priv->bh_error || (hw_priv->bh_thread == NULL))
		return -1;
#ifdef USB_USE_TASTLET_TXRX
	tasklet_schedule(&self->rx_cmp_tasklet);
#else
	if((self->rx_workqueue==NULL))
	{
		return -1;
	}
	queue_work(self->rx_workqueue,&self->rx_complete_work);
#endif
	return 0;
}

/*
*lock for probe dan disconnect
*/
void Sstar_usb_module_muxlock(void)
{
	Sstar_usb_module_lock();
}

void Sstar_usb_module_muxunlock(void)
{
	Sstar_usb_module_unlock();
}

/* sbus_ops implemetation */
static int Sstar_usbctrl_vendorreq_sync(struct sbus_priv *self, u8 request,u8 b_write,
					u16 value, u16 index, void *pdata,u16 len)
{
	unsigned int pipe;
	int status;
	u8 reqtype;
	int vendorreq_times = 0;
	struct usb_device *udev = self->drvobj->pusbdev;
	static int count;
	u8 * reqdata=self->usb_req_data;


	//printk("Sstar_usbctrl_vendorreq_sync++ reqtype=%d\n",reqtype);
	if (!reqdata){
		Sstar_printk_err("regdata is Null\n");
	}
	if(len > SSTAR_USB_EP0_MAX_SIZE){
		Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"usbctrl_vendorreq request 0x%x, b_write %d! len:%d >%d too long \n",
		       request, b_write, len,SSTAR_USB_EP0_MAX_SIZE);
		return -1;
	}
	if(b_write){
		pipe = usb_sndctrlpipe(udev, 0); /* write_out */
		reqtype =  SSTAR_USB_VENQT_WRITE;//host to device
		// reqdata must used dma data
		memcpy(reqdata,pdata,len);
	}
	else {
		pipe = usb_rcvctrlpipe(udev, 0); /* read_in */
		reqtype =  SSTAR_USB_VENQT_READ;//device to host
	}
	do {
		status = usb_control_msg(udev, pipe, request, reqtype, value,
						 index, reqdata, len, 500); /*500 ms. timeout*/
		if (status < 0) {
			Sstar_printk_err("%s:err(%d)addr[%x] len[%d],b_write %d request %d\n",__func__,status,value|(index<<16),len,b_write, request);
		} else if(status != len) {
			Sstar_printk_err("%s:len err(%d)\n",__func__,status);
		}
		else{
			break;
		}
	} while (++vendorreq_times < 3);

	if((b_write==0) && (status>0)){
		memcpy(pdata,reqdata,len);
	}
	if (status < 0 && count++ < 4)
		Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"reg 0x%x, usbctrl_vendorreq TimeOut! status:0x%x value=0x%x\n",
		       value, status, *(u32 *)pdata);
	return status;
}
#ifdef HW_DOWN_FW
static int Sstar_usb_hw_read_port(struct sbus_priv *self, u32 addr, void *pdata,int len)
{
	int ret = 0;
	u8 request = VENDOR_HW_READ; //HW
	u16 wvalue = (u16)(addr & 0x0000ffff);
	u16 index = addr >> 16;

	//printk("ERR,read addr %x,len %x\n",addr,len);

	//hardware just support len=4
	WARN_ON((len != 4) && (request== VENDOR_HW_READ));
	ret = Sstar_usbctrl_vendorreq_sync(self,request,0,wvalue, index, pdata,len);
	if (ret < 0)
	{
		Sstar_printk_err("ERR read addr %x,len %x\n",addr,len);
	}
	return ret;
}
#ifndef CONDIF_SSTAR_CTRL_REQUEST_ASYNC
static int Sstar_usb_hw_write_port(struct sbus_priv *self, u32 addr, const void *pdata,int len)
{

	u8 request = VENDOR_HW_WRITE; //HW
	u16 wvalue = (u16)(addr & 0x0000ffff);
	u16 index = addr >> 16;
	int ret =0;

	Sstar_usb_pm(self,0);

	//printk(KERN_ERR "%s:addr(%x)\n",__func__,addr);
	//hardware just support len=4
	//WARN_ON((len != 4) && (request== VENDOR_HW_WRITE));
	ret =  Sstar_usbctrl_vendorreq_sync(self,request,1,wvalue, index, (void *)pdata,len);
	if (ret < 0)
	{
		Sstar_printk_err("ERR write addr %x,len %x\n",addr,len);
	}
	Sstar_usb_pm(self,1);
	return ret;
}
#else
struct Sstar_usb_ctrlrequest{
		struct sbus_priv *self;
		struct usb_ctrlrequest ctrl;
		u8 mem[0] __attribute__((__aligned__(64)));
};
static void Sstar_usb_ctrlwrite_async_cb(struct urb *urb)
{
	struct Sstar_usb_ctrlrequest *ctrl = (struct Sstar_usb_ctrlrequest *)urb->context;
	struct sbus_priv *self = ctrl->self;
	
	BUG_ON(self == NULL);
	
	if(urb->status){
		Sstar_printk_err("%s: urb err[%d]\n",__func__,urb->status);
		atomic_set(&self->drvobj->ctrl_err,1);
	}
	Sstar_kfree(urb->context);
}

static int Sstar_usb_hw_write_port(struct sbus_priv *self, u32 addr, const void *pdata,int len)
{
	struct Sstar_usb_ctrlrequest *ctrl = NULL;
	
	struct urb *urb = NULL;	
	u16 wvalue = (u16)(addr & 0x0000ffff);
	u16 index = addr >> 16;
	int ret =0;
	u8* buff = NULL;
	int resubmitted = 0;
	
	Sstar_usb_pm(self,0);

	if(atomic_read(&self->drvobj->ctrl_err)){
		ret = -ENODEV;		
		Sstar_printk_err("%s: ctrl_err\n",__func__);
		goto exit;
	}
	
	if((!!pdata)^(!!len)){
		Sstar_printk_err("%s: len(%d),pdata(%p)\n",__func__,len,pdata);
		ret = -EINVAL;
		goto exit;
	}
	
	if(len > SSTAR_USB_EP0_MAX_SIZE){
		Sstar_printk_err("%s: len(%d)\n",__func__,len);
		ret = -EINVAL;
		goto exit;
	}
	
	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb) {
		Sstar_printk_err("%s: alloc urb err\n",__func__);
		ret = -ENOMEM;
		goto exit;
	}

	ctrl = Sstar_kzalloc(sizeof(*ctrl)+len,GFP_KERNEL);
	
	if(ctrl == NULL){		
		Sstar_printk_err("%s: alloc ctrl err\n",__func__);
		ret = -ENOMEM;
		goto exit;
	}

	buff = ctrl->mem;
	ctrl->self = self;
	
	if(pdata)
		memcpy(buff,pdata,len);

	ctrl->ctrl.bRequestType = SSTAR_USB_VENQT_WRITE;
	ctrl->ctrl.bRequest = VENDOR_HW_WRITE;
	ctrl->ctrl.wValue = cpu_to_le16(wvalue);
	ctrl->ctrl.wIndex = cpu_to_le16(index);
	ctrl->ctrl.wLength = cpu_to_le16(len);

	usb_fill_control_urb(urb, self->drvobj->pusbdev, usb_sndctrlpipe(self->drvobj->pusbdev, 0),
			     (unsigned char *)(&ctrl->ctrl), buff, len,
			     Sstar_usb_ctrlwrite_async_cb, ctrl);
xmit:
	usb_anchor_urb(urb, &self->drvobj->ctrl_submitted);
	ret = usb_submit_urb(urb, GFP_KERNEL);
	if (ret < 0) {		
		Sstar_printk_err("%s: submit_urb err (%d)\n",__func__,ret);		
		usb_unanchor_urb(urb);
		if(resubmitted == 0){
			usb_wait_anchor_empty_timeout(&self->drvobj->ctrl_submitted,40);
			resubmitted = 1;
			goto xmit;
		}
		Sstar_kfree(ctrl);
	}
exit:
	Sstar_usb_pm(self,1);
	if(urb)
		usb_free_urb(urb);
	return ret;
}
#endif
int Sstar_usb_sw_read_port(struct sbus_priv *self, u32 addr, void *pdata,int len)
{
	u8 request = VENDOR_SW_READ; //SW
	u16 wvalue = (u16)(addr & 0x0000ffff);
	u16 index = addr >> 16;

	WARN_ON(len > SSTAR_USB_EP0_MAX_SIZE);
	return Sstar_usbctrl_vendorreq_sync(self,request,0,wvalue, index, pdata,len);
}

#else
static int Sstar_usb_sw_read_port(struct sbus_priv *self, u32 addr, void *pdata,int len)
{
	u8 request = VENDOR_SW_READ; //SW
	u16 wvalue = (u16)(addr & 0x0000ffff);
	u16 index = addr >> 16;

	WARN_ON(len > SSTAR_USB_EP0_MAX_SIZE);
	return Sstar_usbctrl_vendorreq_sync(self,request,0,wvalue, index, pdata,len);
}
//#ifndef HW_DOWN_FW
static int Sstar_usb_sw_write_port(struct sbus_priv *self, u32 addr,const void *pdata,int len)
{
	u8 request = VENDOR_SW_WRITE; //SW
	u16 wvalue = (u16)(addr & 0x0000ffff);
	u16 index = addr >> 16;

	WARN_ON(len > SSTAR_USB_EP0_MAX_SIZE);
	return Sstar_usbctrl_vendorreq_sync(self,request,1,wvalue, index, (void *)pdata,len);
}
#endif
int Sstar_lmac_start(struct sbus_priv *self)
{
	u8 request = VENDOR_SW_CPU_JUMP;
	static int tmpdata =0;
	return Sstar_usbctrl_vendorreq_sync(self,request,1,0, 0, &tmpdata,0);
}
int Sstar_usb_ep0_cmd(struct sbus_priv *self)
{
	u8 request = VENDOR_EP0_CMD; //SW
	
	static int tmpdata =0;
	return Sstar_usbctrl_vendorreq_sync(self,request,1,0, 0, &tmpdata,0);
}
static int Sstar_usb_device_reset(struct sbus_priv *self)
{
	int ret = 0;
	/*
	*reset usb
	*/
	ret = usb_lock_device_for_reset(self->drvobj->pusbdev, self->drvobj->pusbintf);
	if (ret == 0) {
		ret = usb_reset_device(self->drvobj->pusbdev);
		usb_unlock_device(self->drvobj->pusbdev);
	}
	
	if(ret != 0){
		Sstar_printk_err( "%s:usb lock err\n",__func__);
		goto error;
	}
	
	if(self->drvobj->pusbdev->state != USB_STATE_CONFIGURED){
		Sstar_printk_err( "%s:state(%d) err\n",__func__,self->drvobj->pusbdev->state);
		goto error;
	}
	
	return 0;
error:
	return -1;
}
static void Sstar_usb_block_urbs(struct sbus_priv *self)
{
	int urb_index = 0;
	unsigned long flags = 0;
	struct sbus_urb *urb_unlink;
	
	spin_lock_irqsave(&self->lock, flags);
	urb_unlink = self->drvobj->rx_urb;
	for(urb_index=0;urb_index<RX_URB_NUM;urb_index++){
		#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
		usb_block_urb(urb_unlink[urb_index].test_urb);
		#else
		atomic_inc(&urb_unlink[urb_index].test_urb->reject);
		#endif
	}
	urb_unlink = self->drvobj->tx_urb;
	for(urb_index=0;urb_index<TX_URB_NUM;urb_index++){
		#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
		usb_block_urb(urb_unlink[urb_index].test_urb);
		#else
		atomic_inc(&urb_unlink[urb_index].test_urb->reject);
		#endif
	}
	spin_unlock_irqrestore(&self->lock, flags);
}

static void Sstar_usb_unblock_urbs(struct sbus_priv *self)
{
	int urb_index = 0;
	unsigned long flags = 0;
	struct sbus_urb *urb_unlink;
	
	spin_lock_irqsave(&self->lock, flags);
	urb_unlink = self->drvobj->rx_urb;
	for(urb_index=0;urb_index<RX_URB_NUM;urb_index++){
		#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
		usb_unblock_urb(urb_unlink[urb_index].test_urb);
		#else
		usb_unpoison_urb(urb_unlink[urb_index].test_urb);
		#endif
	}
	urb_unlink = self->drvobj->tx_urb;
	for(urb_index=0;urb_index<TX_URB_NUM;urb_index++){
		#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
		usb_unblock_urb(urb_unlink[urb_index].test_urb);
		#else
		usb_unpoison_urb(urb_unlink[urb_index].test_urb);
		#endif
	}
	spin_unlock_irqrestore(&self->lock, flags);
}

static int Sstar_usb_wait_anchor_empty_timeout(struct sbus_priv *self,int timeout)
{
	int urb_index = 0;
	struct Sstar_common *hw_priv = self->core;
	struct sbus_urb *urb_unlink;
	int ret = 0;

	Sstar_printk_bus( "Unlink Rx Urb\n");
	urb_unlink = self->drvobj->rx_urb;
	for(urb_index=0;urb_index<RX_URB_NUM;urb_index++){
		ret = usb_unlink_urb(urb_unlink[urb_index].test_urb);
		Sstar_printk_bus( "usb_unlink_urb[%d][%d]\n",urb_index,ret);
	}
	ret = usb_wait_anchor_empty_timeout(&hw_priv->sbus_priv->drvobj->rx_submitted,timeout);
	if(ret == 0){
		Sstar_printk_err("%s:rx_submitted cancle timeout (%d)\n",__func__,usb_anchor_empty(&hw_priv->sbus_priv->drvobj->rx_submitted));
		goto exit;
	}
	Sstar_printk_bus("Unlink Rx Urb (%d)\n",ret);
	Sstar_printk_bus( "Unlink Tx Urb\n");
	urb_unlink = self->drvobj->tx_urb;
	for(urb_index=0;urb_index<TX_URB_NUM;urb_index++){
		ret = usb_unlink_urb(urb_unlink[urb_index].test_urb);
		Sstar_printk_err("usb_unlink_urb[%d][%d]\n",urb_index,ret);
	}
	ret = usb_wait_anchor_empty_timeout(&hw_priv->sbus_priv->drvobj->tx_submitted,timeout);
	if(ret == 0){
		Sstar_printk_err("%s:tx_submitted cancle timeout(%d) \n",__func__,usb_anchor_empty(&hw_priv->sbus_priv->drvobj->tx_submitted));
	}
	Sstar_printk_bus("Unlink Tx Urb(%d)\n",ret);
exit:
	return ret;
}
static void Sstar_destroy_wsm_cmd(struct Sstar_common *hw_priv)
{
	/*
	*flush work
	*/
	flush_workqueue(hw_priv->workqueue);
	Sstar_printk_exit("Flush hw_priv->workqueue\n");
	/*
	*try to release wsm_oper_unlock
	*/
#ifdef OPER_CLOCK_USE_SEM
	del_timer_sync(&hw_priv->wsm_pm_timer);
	spin_lock_bh(&hw_priv->wsm_pm_spin_lock);
	if(atomic_read(&hw_priv->wsm_pm_running) == 1){
		atomic_set(&hw_priv->wsm_pm_running, 0);
		wsm_oper_unlock(hw_priv);
		Sstar_release_suspend(hw_priv);
		Sstar_printk_exit("%s,up pm lock\n",__func__);
	}
	spin_unlock_bh(&hw_priv->wsm_pm_spin_lock);
#endif
	/*
	*release scan 
	*/
	if(atomic_read(&hw_priv->scan.in_progress)){
		/*
		*maybe scan work is runing,here flush it.
		*/
		Sstar_printk_exit("scan running,maybe need cancle\n");
		flush_workqueue(hw_priv->workqueue);
		if(atomic_read(&hw_priv->scan.in_progress)&&(hw_priv->scan.status != -ETIMEDOUT) &&
		   (Sstar_cancle_delayed_work(&hw_priv->scan.timeout,true) > 0)){
		    Sstar_printk_exit("scan running,try to cancle\n");
			Sstar_scan_timeout(&hw_priv->scan.timeout.work);
		}
	}
	Sstar_printk_exit("Flush pm and scan\n");
	flush_workqueue(hw_to_local(hw_priv->hw)->workqueue);
	flush_workqueue(hw_priv->workqueue);
	
	synchronize_net();
}
static int Sstar_reinit_firmware(struct Sstar_common *hw_priv)
{
	struct wsm_operational_mode mode = {
		.power_mode = wsm_power_mode_quiescent,
		.disableMoreFlagUsage = true,
	};
	int ret = 0;
	u8 if_id = 0;

	/*
	*load firmware
	*/
	hw_priv->wsm_caps.firmwareReady = 0;
	ret = Sstar_load_firmware(hw_priv);
	if (ret){
		Sstar_printk_err( "Sstar_load_firmware ERROR!\n");
		goto error_reload;
	}
	Sstar_printk_init("mdelay wait wsm_startup_done  !!\n");
	if (wait_event_interruptible_timeout(hw_priv->wsm_startup_done,
			hw_priv->wsm_caps.firmwareReady,3*HZ)<=0){
		Sstar_printk_err("%s: reload fw err\n",__func__);
		goto error_reload;
	}
	atomic_xchg(&hw_priv->bh_halt,0);
	Sstar_firmware_init_check(hw_priv);
	for (if_id = 0; if_id < ABwifi_get_nr_hw_ifaces(hw_priv); if_id++) {
		/* Set low-power mode. */
		ret = wsm_set_operational_mode(hw_priv, &mode, if_id);
		if (ret) {
			WARN_ON(1);
			goto error_reload;
		}
		/* Enable multi-TX confirmation */
		ret = wsm_use_multi_tx_conf(hw_priv, true, if_id);
		if (ret) {
#ifndef CONFIG_TX_NO_CONFIRM
			WARN_ON(1);
			goto error_reload;
#else //CONFIG_TX_NO_CONFIRM
			ret = 0;
#endif
		}
	}
	
error_reload:
	return ret;
}
static int _Sstar_lmac_restart(struct sbus_priv *self)
{
	int ret = 0;
	struct Sstar_common *hw_priv = self->core;
	int i;
	/*
	*kill all tx and rx urb and release tx pkg and cmd
	*/
	Sstar_printk_init("%s\n",__func__);
	wsm_lock_tx_async(hw_priv);
	Sstar_wifi_set_status(2);
	/*
	*from now ,rx_urb and tx_urb can not be submitted again until
	*urb unblock.
	*/
	Sstar_usb_block_urbs(self);
	synchronize_rcu();
	ret = Sstar_usb_device_reset(self);
	if(ret){
		Sstar_printk_err("Reset Usb Err\n");
		goto error_prepare;
	}
	Sstar_printk_init("%s:(%d)\n",__func__,atomic_add_return(0, &hw_priv->tx_lock));	
	Sstar_printk_init("Release All Tx Urb\n");
	ret = Sstar_usb_wait_anchor_empty_timeout(self,1000);
	if(ret == 0){
		ret = Sstar_usb_device_reset(self);
		if(ret){
			Sstar_printk_err("Reset Usb Err\n");
			goto error_prepare;
		}
		ret = Sstar_usb_wait_anchor_empty_timeout(self,1000);

		if(ret == 0){
			Sstar_printk_err("Cancle Usb Err\n");
			goto error_prepare;
		}
	}	
	Sstar_rx_bh_flush(hw_priv);	
	Sstar_usb_release_tx_err_urb(self,self->drvobj->tx_urb_map,1);
	synchronize_rcu();
	ret = Sstar_usb_device_reset(self);
	if(ret){
		Sstar_printk_err("Reset Usb Err\n");
		goto error_prepare;
	}
	Sstar_printk_init("Flush txrx urb\n");
	/*
	*waitting all wsm cmd destory
	*/
	Sstar_destroy_wsm_cmd(hw_priv);
	
	hw_priv->bh_error = 0;
	smp_mb();
	ieee80211_pre_restart_hw_sync(hw_priv->hw);
	Sstar_printk_init("Flush iee80211 hw\n");
	Sstar_tx_queues_lock(hw_priv);
	/*
	*hold rtnl_lock,make sure that when down load fw,network layer cant not 
	*send pkg and cmd
	*/
	rtnl_lock();
	/*
	*release hw buff
	*/
	hw_priv->wsm_tx_seq = 0;
	hw_priv->buf_id_tx = 0;
	hw_priv->wsm_rx_seq = 0;
	hw_priv->hw_bufs_used = 0;
	hw_priv->save_buf = NULL;
	hw_priv->save_buf_len = 0;
	hw_priv->save_buf_vif_selected = -1;
	hw_priv->buf_id_rx = 0;
	for (i = 0; i < SSTAR_WIFI_MAX_VIFS; i++)
		hw_priv->hw_bufs_used_vif[i] = 0;
	
	atomic_set(&hw_priv->Sstar_pluged,1);
	Sstar_usb_unblock_urbs(self);
	Sstar_wifi_set_status(0);
	/*
	*load firmware
	*/
	ret = Sstar_reinit_firmware(hw_priv);
	
	if(ret){
		Sstar_printk_init("%s:reload fw err\n",__func__);
		goto error_reload;
	}
	/*
	*restart ap and sta
	*/
	ret = ieee80211_restart_hw_sync(hw_priv->hw);
	
	Sstar_printk_init("%s:(%d)\n",__func__,atomic_add_return(0, &hw_priv->tx_lock));
	rtnl_unlock();
	Sstar_tx_queues_unlock(hw_priv);
	wsm_unlock_tx(hw_priv);
	return ret;
error_reload:
	rtnl_unlock();
	Sstar_tx_queues_unlock(hw_priv);
error_prepare:
	wsm_unlock_tx_async(hw_priv);
	return -1;
}
int Sstar_lmac_restart(struct sbus_priv *self)
{
	int ret  = -1;
	/*
	*hold Sstar_usb_module_lock,means that during reload firmware
	*disconnect and probe will be locked
	*/
	if(Sstar_usb_module_trylock()){ 
		if(Sstar_hw_priv_dereference()){
			ret = _Sstar_lmac_restart(self);
		}
		Sstar_usb_module_unlock();
	}else {
		Sstar_printk_err("%s:Skip Restart\n",__func__);
	}
	return ret;
}

#if (PROJ_TYPE>=ARES_B)

#define HW_RESET_REG_CPU   				BIT(16)
#define HW_RESET_REG_HIF   				BIT(17)
#define HW_RESET_REG_SYS   				BIT(18)
#define HW_RESRT_REG_CHIP  				BIT(19)
#define HW_RESET_REG_NEED_IRQ_TO_LMAC	BIT(20)
int Sstar_usb_ep0_hw_reset_cmd(struct sbus_priv *self,enum HW_RESET_TYPE type,bool irq_lmac)
{
	u8 request = VENDOR_HW_RESET; //SW
	u16 wvalue ;
	u16 index ;
	
	static int tmpdata =0;
	if(type==HW_RESET_HIF){
		tmpdata = HW_RESET_REG_HIF;
	}
	else if(type==HW_RESET_HIF_SYSTEM){
		tmpdata = HW_RESET_REG_HIF|HW_RESET_REG_SYS;
	}
	else if(type==HW_RESET_HIF_SYSTEM_USB){
		tmpdata = HW_RESRT_REG_CHIP;
	}
	else if(type==HW_HOLD_CPU){
		tmpdata = HW_RESET_REG_CPU;
	}
	else if(type==HW_RUN_CPU){
		tmpdata = 0;
	}else if (type == HW_RESET_HIF_SYSTEM_CPU)
	{
		tmpdata = HW_RESET_REG_CPU|HW_RESET_REG_HIF|HW_RESET_REG_SYS;
	}
	if(irq_lmac){
		tmpdata |= HW_RESET_REG_NEED_IRQ_TO_LMAC;
	}
	//tmpdata |= 0x40;
	//tmpdata |= VENDOR_HW_RESET<<8;
	wvalue = (tmpdata>>16)&0xff;
	wvalue |= ((request + 0x40 + ((tmpdata>>16)&0xff))<<8)&0xff00;
	index = wvalue;
	Sstar_printk_bus("ep0_hw_reset request %d wvalue %x\n",request,wvalue);
	return Sstar_usbctrl_vendorreq_sync(self,request,1,wvalue, index, &tmpdata,0);
}
#endif  //(PROJ_TYPE>=ARES_B)
/*
wvalue=1 : open uart debug;
 wvalue=0 : close uart debug;
 */
int Sstar_usb_debug_config(struct sbus_priv *self,u16 wvalue)
{
#if (PROJ_TYPE<ARES_B)
	u8 request = VENDOR_DBG_SWITCH;
	u16 index = 0;

	usb_printk( "Sstar_usb_debug_config\n");

	return Sstar_usbctrl_vendorreq_sync(self,request,1,wvalue, index, &wvalue,0);
#else //#if (PROJ_TYPE>=ARES_B)
	return 0;
#endif  //#if (PROJ_TYPE<ARES_B)
}

static void Sstar_usb_xmit_data_complete(struct urb *urb)
{
	struct sbus_urb *tx_urb=(struct sbus_urb*)urb->context;
	struct sbus_priv *self = tx_urb->obj;
	struct Sstar_common	*hw_priv	= self->core;
	//unsigned long flags=0;

	if(hw_priv == NULL)
	{
		Sstar_printk_err( "<WARNING> q. hw_priv =0 drop\n");
		return;
	}

	switch(urb->status){
		case 0:
			break;
		case -ENOENT:
		case -ECONNRESET:
		case -ENODEV:
		case -ESHUTDOWN:
			Sstar_printk_bus("WARNING>%s %d status %d\n",__func__,__LINE__,urb->status);
			goto __free;
		default:
			Sstar_printk_bus( "WARNING> %s %d status %d\n",__func__,__LINE__,urb->status);
			goto __free;
	}
//resubmit:
	if(!self->core->init_done){
		Sstar_printk_err("[BH] irq. init_done =0 drop\n");
		return ;
	}
	if (/* WARN_ON */(self->core->bh_error)){
		Sstar_printk_err( "[BH] irq. bh_error =0 drop\n");
		return ;
	}

#ifdef CONFIG_USB_AGGR_URB_TX
	Sstar_usb_free_txDMABuf_all(self,tx_urb->pallocated_buf,tx_urb->dma_buff_alloced);
#endif //CONFIG_USB_AGGR_URB_TX


	Sstar_usb_urb_put(self,self->drvobj->tx_urb_map,tx_urb->urb_id,1);

	tx_urb->test_pending = 0;


	Sstar_usb_xmit_schedule(self);
	
	
	return ;

__free:
	Sstar_printk_err("<Warning> usb drop 1 frame txend:_urb_put %d\n",tx_urb->urb_id);
//	spin_lock_bh(&hw_priv->tx_com_lock);
//	spin_lock_irqsave(&hw_priv->tx_com_lock, flags);
	wsm_release_tx_buffer_NoLock(hw_priv, 1);
	
#ifdef CONFIG_USB_AGGR_URB_TX
	Sstar_usb_free_txDMABuf_all(self,tx_urb->pallocated_buf,tx_urb->dma_buff_alloced);
#endif //CONFIG_USB_AGGR_URB_TX
	Sstar_usb_urb_put(self,self->drvobj->tx_urb_map,tx_urb->urb_id,1);
	tx_urb->test_pending = 0;
//	spin_unlock_bh(&hw_priv->tx_com_lock);
//	spin_unlock_irqrestore(&hw_priv->tx_com_lock, flags);

	Sstar_usb_xmit_schedule(self);

	return ;
}

void Sstar_usb_free_err_cmd(struct sbus_priv *self)
{
	struct Sstar_common	*hw_priv = self->core;

	spin_lock_bh(&hw_priv->wsm_cmd.lock);
	hw_priv->wsm_cmd.ret = -1;
	hw_priv->wsm_cmd.done = 1;
	hw_priv->wsm_cmd.cmd = 0xFFFF;
	spin_unlock_bh(&hw_priv->wsm_cmd.lock);
	printk_once(KERN_ERR "%s:release wsm_cmd.lock\n",__func__);
	wake_up(&hw_priv->wsm_cmd_wq);		
}
void Sstar_usb_free_err_data(struct sbus_priv *self,struct sbus_urb *tx_urb)
{
	struct Sstar_common	*hw_priv = self->core;
	struct wsm_tx *wsm = (struct wsm_tx *)tx_urb->data;	
	struct Sstar_queue *queue;
	u8 queue_id;
	struct sk_buff *skb;
	const struct Sstar_txpriv *txpriv;
	
	printk_once(KERN_ERR "%s:release tx pakage\n",__func__);
	BUG_ON(wsm == NULL);
	queue_id = Sstar_queue_get_queue_id(wsm->packetID);

	BUG_ON(queue_id >= 4);
	queue = &hw_priv->tx_queue[queue_id];
	BUG_ON(queue == NULL);

	if(!WARN_ON(Sstar_queue_get_skb(queue, wsm->packetID, &skb, &txpriv))) {

		struct ieee80211_tx_info *tx = IEEE80211_SKB_CB(skb);
		//int tx_count = 0;
		int i;
		wsm_release_vif_tx_buffer_Nolock(hw_priv,txpriv->if_id,1);
		tx->flags |= IEEE80211_TX_STAT_ACK;
		tx->status.rates[0].count = 1;
		for (i = 1; i < IEEE80211_TX_MAX_RATES; ++i) {
			tx->status.rates[i].count = 0;
			tx->status.rates[i].idx = -1;
		}
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
		Sstar_queue_remove(hw_priv, queue, wsm->packetID);
#else
		Sstar_queue_remove(queue, wsm->packetID);
#endif
	}else {
		wsm_release_vif_tx_buffer_Nolock(hw_priv,Sstar_queue_get_if_id(wsm->packetID),1);
	}
}

#ifdef CONFIG_USB_AGGR_URB_TX
static int Sstar_usb_xmit_data(struct sbus_priv *self,
				   unsigned int addr,
				   const void *pdata, int len,sbus_callback_handler hander)
{
	//unsigned int pipe;
	int status=0;
	int tx_burst=0;
	int vif_selected;
	struct wsm_hdr_tx *wsm;
	struct Sstar_common *hw_priv=self->core;
	void *txdata =NULL;
	char * txdmabuff = NULL;
	char * usb_aggr_buff = NULL;
	u8 *data =NULL;
	int tx_len=0;
	int actual_len = 0;
	int ret = 0;
	int urb_id =-1;
	struct sbus_urb *tx_urb = NULL;
	usb_printk( "Sstar_usb_xmit_data++\n");
	
	spin_lock_bh(&hw_priv->tx_com_lock);
		
	urb_id = Sstar_usb_urb_get(self,self->drvobj->tx_urb_map,TX_URB_NUM,1);
	if(urb_id<0){
		usb_printk( "Sstar_usb_xmit_data:urb_id<0\n");
		status=-4;
		goto error;
	}
	usb_printk( "Sstar_usb_xmit_data++ %d\n",__LINE__);

	/*if (atomic_read(&self->tx_lock)==0)*/
	if (hw_priv->device_can_sleep) {
		hw_priv->device_can_sleep = false;
	}
	
	tx_urb = &self->drvobj->tx_urb[urb_id];
	tx_urb->pallocated_buf_len = 0;
	tx_urb->frame_cnt=0;
	tx_urb->dma_buff_alloced = 0;
	tx_urb->data=NULL;
	usb_aggr_buff = NULL;
	tx_urb->pallocated_buf = Sstar_usb_pick_txDMABuf(self);
	
	if(tx_urb->pallocated_buf==NULL)
	{
		//usb_printk( "Sstar_usb_pick_txDMABuf:err\n");
		Sstar_usb_urb_put(self,self->drvobj->tx_urb_map,urb_id,1);
		status=-7;
		goto error;
	}
	
	do {
		wsm_alloc_tx_buffer_NoLock(hw_priv);
		ret = wsm_get_tx(hw_priv, &data, &actual_len, &tx_burst,&vif_selected);
		if (ret <= 0) {
			  wsm_release_tx_buffer_NoLock(hw_priv, 1);
			  //Sstar_usb_urb_put(self,self->drvobj->tx_urb_map,urb_id,1);
		  	  status=-3;
			  break;
		} else {

			if(usb_aggr_buff == NULL){
				usb_aggr_buff= Sstar_usb_get_txDMABuf(self);
				BUG_ON(usb_aggr_buff == NULL);
				tx_urb->dma_buff_alloced ++;
			}
			txdmabuff = usb_aggr_buff; 
			wsm = (struct wsm_hdr_tx *)data;
			tx_len=wsm->usb_len;
#if (PROJ_TYPE<ARES_A)	
			//athenaB must set usb_len = 2048, ares not need
			wsm->usb_len = PER_PACKET_LEN;
#endif //#if (PROJ_TYPE!=ARES_A)	
			BUG_ON(tx_len < sizeof(*wsm));
			BUG_ON(actual_len < sizeof(*wsm));
			tx_urb->frame_cnt++;

			self->tx_vif_selected =vif_selected;
			tx_urb->data = data;
			//WSM_FIRMWARE_CHECK_ID have no confirm
			if(wsm->id == WSM_FIRMWARE_CHECK_ID){
				 wsm_release_tx_buffer_NoLock(hw_priv, 1);
				 tx_urb->data = NULL;
				 Sstar_printk_init("WSM_FIRMWARE_CHECK_ID\n");
			}

			wsm->flag = __cpu_to_le16(0xe569)<<16;
			wsm->flag |= BIT(6);
			wsm->flag |= (self->tx_hwChanId & 0x1f);
			wsm->id &= __cpu_to_le32(~WSM_TX_SEQ(WSM_TX_SEQ_MAX));
			wsm->id |= cpu_to_le32(WSM_TX_SEQ(hw_priv->wsm_tx_seq));
			txdata =wsm;
			usb_printk( "%s flag=%x, tx_len %d seq %d len %d\n",__func__,wsm->flag,tx_len,hw_priv->wsm_tx_seq,wsm->len);

			tx_urb->callback_handler = hander;
			tx_urb->test_pending = 1;
			tx_urb->test_seq= hw_priv->wsm_tx_seq;
			tx_urb->test_hwChanId= self->tx_hwChanId;
			
			self->tx_hwChanId++;
			usb_printk( "tx_seq %d\n",self->tx_hwChanId);
			hw_priv->wsm_tx_seq = (hw_priv->wsm_tx_seq + 1) & WSM_TX_SEQ_MAX;
		
			if (vif_selected != -1) {
				hw_priv->hw_bufs_used_vif[vif_selected]++;
			}
			//memcpy(txdmabuff,txdata,actual_len);
			Sstar_xmit_linearize(hw_priv,(struct wsm_tx *)wsm,txdmabuff,actual_len);
			tx_urb->pallocated_buf_len += PER_PACKET_LEN;
			/*
			*if the data is cmd ,keep it last in the aggr buff
			*/
			if(wsm_txed(hw_priv, data)==0){
				hw_priv->wsm_txframe_num++;
			}else {
				tx_urb->data = NULL;
			}
#ifdef CONFIG_TX_NO_CONFIRM
			//cmd or need confirm frame not agg
			if(Sstar_usb_free_tx_wsm(self,tx_urb)==0)
				break;
#endif  //CONFIG_TX_NO_CONFIRMCONFIG_TX_NO_CONFIRM
			//the last dma buffer
			usb_aggr_buff += PER_PACKET_LEN;
			if(tx_urb->frame_cnt>=(PER_BUFF_AGGR_NUM*tx_urb->dma_buff_alloced)){
				if(usb_aggr_buff >= self->drvobj->tx_dma_addr_buffer_end)
					break;
				usb_aggr_buff = Sstar_usb_pick_txDMABuf(self);
				if(usb_aggr_buff == NULL)
					break;
				usb_aggr_buff = NULL;
			}
		}
	}while(1);

	if(tx_urb->pallocated_buf_len == 0){
		Sstar_usb_urb_put(self,self->drvobj->tx_urb_map,urb_id,1);
		status= -6;
		goto error;
	}

	if(tx_urb->frame_cnt ==0){
		WARN_ON(1);
	}
	usb_anchor_urb(tx_urb->test_urb, &self->drvobj->tx_submitted);
	if(!Sstar_wifi_get_status()){
		atomic_add(1, &hw_priv->bh_tx);
		tx_urb->dma_transfer_addr = self->drvobj->tx_dma_addr +((Sstar_dma_addr_t)tx_urb->pallocated_buf - (Sstar_dma_addr_t)self->drvobj->tx_dma_addr_buffer);
		SSTAR_USB_FILL_HTTX_BULK_URB(tx_urb->test_urb,
			self->drvobj->pusbdev, self->drvobj->ep_out,tx_urb->pallocated_buf,tx_urb->pallocated_buf_len,
			Sstar_usb_xmit_data_complete,tx_urb,tx_urb->dma_transfer_addr);
		//usb_anchor_urb(self->drvobj->tx_urb,
		status = usb_submit_urb(tx_urb->test_urb, GFP_ATOMIC);
	}else {
		status = -1;
	}
	
	if (status&&tx_urb->frame_cnt) {
		u8 i = 0;
		int wsm_id;		
		struct wsm_tx *wsm_txd = NULL;		
		status = 1;
		usb_unanchor_urb(tx_urb->test_urb);
		tx_urb->test_urb->status = 0;
		for(i = 0;i<tx_urb->frame_cnt;i++){			
			wsm_txd = (struct wsm_tx *)(tx_urb->pallocated_buf+i*PER_PACKET_LEN);			
			wsm_id = __le16_to_cpu(wsm_txd->hdr.id) & 0x3F;			
			Sstar_printk_err("%s:wsm_id(%x)\n",__func__,wsm_id);			
			self->tx_hwChanId--;			
			hw_priv->wsm_tx_seq = (hw_priv->wsm_tx_seq - 1) & WSM_TX_SEQ_MAX;			
			if(wsm_id == WSM_FIRMWARE_CHECK_ID){
				continue;			
			}			
			if(wsm_id == WSM_TRANSMIT_REQ_MSG_ID){				
				#ifdef CONFIG_TX_NO_CONFIRM				
				if(!(wsm_txd->htTxParameters&__cpu_to_le32(WSM_NEED_TX_CONFIRM))){
					continue;				
				}				
				#endif				
				tx_urb->data = wsm_txd;				
				Sstar_usb_free_err_data(self,tx_urb);			
			}else {				
				Sstar_usb_free_err_cmd(self);			
			}						
			wsm_release_tx_buffer_NoLock(hw_priv, 1);		
		}		
		Sstar_usb_urb_put(self,self->drvobj->tx_urb_map,urb_id,1);		
		tx_urb->data = NULL;		
		status = 1;		
		Sstar_printk_err("release all data finished\n");	
		Sstar_usb_free_txDMABuf_all(self,tx_urb->pallocated_buf,tx_urb->dma_buff_alloced);
		goto error;	
	}
		
	if(status==0){
		spin_unlock_bh(&hw_priv->tx_com_lock);
		return 1;
	}
error:
	spin_unlock_bh(&hw_priv->tx_com_lock);
	return status;
}
#else
static int Sstar_usb_xmit_data(struct sbus_priv *self,
				   unsigned int addr,
				   const void *pdata, int len,sbus_callback_handler hander)
{
	int status=0;
	int tx_burst=0;
	int vif_selected;
	struct wsm_hdr_tx *wsm;
	struct Sstar_common *hw_priv=self->core;
	void *txdata =NULL;
	u8 *data =NULL;
	int tx_len=0;
	int actual_len;
	int ret = 0;
	int urb_id =-1;
	struct sbus_urb *tx_urb = NULL;
	spin_lock_bh(&hw_priv->tx_com_lock);
	urb_id = Sstar_usb_urb_get(self,self->drvobj->tx_urb_map,TX_URB_NUM,1);
	if(urb_id<0){
		usb_printk( "Sstar_usb_xmit_data:urb_id<0\n");
		status=-4;
		goto error;
	}
	/*if (atomic_read(&self->tx_lock)==0)*/{
		if (hw_priv->device_can_sleep) {
				hw_priv->device_can_sleep = false;
		}

		ret = wsm_get_tx(hw_priv, &data, &actual_len, &tx_burst,&vif_selected);
		if (ret <= 0) {
			  Sstar_usb_urb_put(self,self->drvobj->tx_urb_map,urb_id,1);
			 // printk("tx:Sstar_usb_urb_put NULL %d\n",urb_id);
			  status=-3;
			  goto error;
		} else {
			
			wsm_alloc_tx_buffer_NoLock(hw_priv);
			tx_urb = &self->drvobj->tx_urb[urb_id];
			wsm = (struct wsm_hdr_tx *)data;
			tx_len=wsm->usb_len;
			BUG_ON(tx_len < sizeof(*wsm));
			BUG_ON(actual_len < sizeof(*wsm));
			BUG_ON(__le32_to_cpu(wsm->usb_len) != tx_len);

			atomic_add(1, &hw_priv->bh_tx);
			//self->tx_data = (void *)data;
			self->tx_vif_selected =vif_selected;
			tx_urb->data = data;
			//WSM_FIRMWARE_CHECK_ID have no confirm
			if(wsm->id == WSM_FIRMWARE_CHECK_ID){
				wsm_release_tx_buffer_NoLock(hw_priv, 1);
				 tx_urb->data = NULL;
				Sstar_printk_bus("WSM_FIRMWARE_CHECK_ID\n");
			}
			wsm->flag = __cpu_to_le16(0xe569)<<16;
			wsm->flag |= BIT(6);
			wsm->flag |= (self->tx_hwChanId & 0x1f);
			wsm->id &= __cpu_to_le32(~WSM_TX_SEQ(WSM_TX_SEQ_MAX));
			wsm->id |= cpu_to_le32(WSM_TX_SEQ(hw_priv->wsm_tx_seq));
			txdata =wsm;
			//usb_printk( "tx hw_bufs_used_vif[%d]=%d\n",vif_selected,hw_priv->hw_bufs_used_vif[vif_selected]);
			//tx_len =ALIGN(tx_len,USB_BLOCK_SIZE);
			//printk("%s wsm->id=%x, tx_len %d seq %d len %d\n",__func__,wsm->id ,tx_len,hw_priv->wsm_tx_seq,wsm->len);

			//atomic_xchg(&self->tx_lock, 1);
			tx_urb->callback_handler = hander;
			tx_urb->test_pending = 1;
			tx_urb->test_seq= hw_priv->wsm_tx_seq;
			tx_urb->test_hwChanId= self->tx_hwChanId;
			//printk( "txstart:urb_get %d seq %d\n",urb_id,hw_priv->wsm_tx_seq);

			if(wsm_txed(hw_priv, data)==0){
				hw_priv->wsm_txframe_num++;
			}else {
				tx_urb->data = NULL;
			}
			
			if (vif_selected != -1) {
				hw_priv->hw_bufs_used_vif[vif_selected]++;
			}			
			usb_anchor_urb(tx_urb->test_urb, &self->drvobj->tx_submitted);
			if(!Sstar_wifi_get_status()){
#ifdef CONFIG_USE_DMA_ADDR_BUFFER
				///self->drvobj->tx_urb->transfer_flags |= URB_ZERO_PACKET;
				//memcpy(tx_urb->pallocated_buf,txdata,tx_len);
				Sstar_xmit_linearize(hw_priv,(struct wsm_tx*)txdata,tx_urb->pallocated_buf,tx_len);
				SSTAR_USB_FILL_HTTX_BULK_URB(tx_urb->test_urb,
					self->drvobj->pusbdev, self->drvobj->ep_out,tx_urb->pallocated_buf,tx_len,
					Sstar_usb_xmit_data_complete,tx_urb,tx_urb->dma_transfer_addr);
				//usb_anchor_urb(self->drvobj->tx_urb,
				status = usb_submit_urb(tx_urb->test_urb, GFP_ATOMIC);
#else //CONFIG_USE_DMA_ADDR_BUFFER
				//tx_urb->data = NULL;
				unsigned int pipe;
				u8 *submit_buff = tx_urb->pallocated_buf;
				
				BUG_ON(submit_buff == NULL);
				//memcpy(submit_buff,txdata,actual_len);				
				Sstar_xmit_linearize(hw_priv,(struct wsm_tx*)txdata,submit_buff,actual_len);
				pipe = usb_sndbulkpipe(self->drvobj->pusbdev, self->drvobj->ep_out);
				//printk("%s:tx_urb->data[%p]\n",__func__,tx_urb->data);
				///self->drvobj->tx_urb->transfer_flags |= URB_ZERO_PACKET;
				usb_fill_bulk_urb(tx_urb->test_urb,
					self->drvobj->pusbdev, pipe,submit_buff,tx_len,
					Sstar_usb_xmit_data_complete,tx_urb);
				//usb_anchor_urb(self->drvobj->tx_urb,
				status = usb_submit_urb(tx_urb->test_urb, GFP_ATOMIC);
#endif //CONFIG_USE_DMA_ADDR_BUFFER
			}else{
				printk_once(KERN_ERR "%s:module exit,do not submit urb\n",__func__);
				status = -1;
			}
			if (status) {
				usb_unanchor_urb(tx_urb->test_urb);
				tx_urb->test_urb->status = 0;
				if(__le16_to_cpu(wsm->id) != WSM_FIRMWARE_CHECK_ID){
					if (vif_selected != -1) {
						Sstar_usb_free_err_data(self,tx_urb);
					}else {
						Sstar_usb_free_err_cmd(self);		
					}								
					//atomic_xchg(&self->tx_lock, 0);
					printk_once(KERN_ERR"<WARNING>%s %d tx_len %d\n",__func__,__LINE__,tx_len);
					wsm_release_tx_buffer_NoLock(hw_priv, 1);
				}
				Sstar_usb_urb_put(self,self->drvobj->tx_urb_map,urb_id,1);
				tx_urb->data = NULL;
				status = 1;
				printk_once(KERN_ERR"tx:Sstar_usb_urb_put %d\n",urb_id);
				//msleep(1000);
				goto error;
			}
	
			//self->tx_data = NULL;
			self->tx_hwChanId++;
			usb_printk( "tx_seq %d\n",self->tx_hwChanId);
			hw_priv->wsm_tx_seq = (hw_priv->wsm_tx_seq + 1) & WSM_TX_SEQ_MAX;
		
		}
	}
	if(status==0){
		spin_unlock_bh(&hw_priv->tx_com_lock);
		return 1;
	}
error:
	spin_unlock_bh(&hw_priv->tx_com_lock);
	return status;
}
#endif //CONFIG_USB_AGGR_URB_TX

void Sstar_usb_receive_data_cancel(struct sbus_priv *self)
{
	Sstar_printk_err("&&&fuc=%s\n",__func__);
	//usb_kill_urb(self->drvobj->rx_urb);
	Sstar_usb_pm(self,1);
}
void Sstar_usb_kill_all_txurb(struct sbus_priv *self)
{
	int i=0;
	for(i=0;i<TX_URB_NUM;i++){
		usb_kill_urb(self->drvobj->tx_urb[i].test_urb);
	}

}
void Sstar_usb_kill_all_rxurb(struct sbus_priv *self)
{
	int i=0;
	for(i=0;i<RX_URB_NUM;i++){
		usb_kill_urb(self->drvobj->rx_urb[i].test_urb);
	}

}

void Sstar_usb_urb_free(struct sbus_priv *self,struct sbus_urb * pUrb,int max_num)
{
	int i=0;
#ifdef CONFIG_USE_DMA_ADDR_BUFFER
		if(self->drvobj->tx_dma_addr){
			Sstar_usb_buffer_free(self->drvobj->pusbdev,self->drvobj->tx_dma_addr_buffer_len,self->drvobj->tx_dma_addr_buffer, self->drvobj->tx_dma_addr);
			self->drvobj->tx_dma_addr = 0;
		}
#endif //CONFIG_USE_DMA_ADDR_BUFFER
	for(i=0;i<max_num;i++){
		usb_kill_urb(pUrb[i].test_urb);
		usb_free_urb(pUrb[i].test_urb);
		if(pUrb[i].test_skb)
			Sstar_dev_kfree_skb(pUrb[i].test_skb);
		pUrb[i].test_skb =NULL;
		pUrb[i].test_urb =NULL;
		#ifndef CONFIG_USE_DMA_ADDR_BUFFER
		if(pUrb[i].pallocated_buf){
			Sstar_kfree(pUrb[i].pallocated_buf);
			pUrb[i].pallocated_buf = NULL;
		}
		#endif
	}
}


void Sstar_usb_urb_map_show(struct sbus_priv *self)
{
	Sstar_printk_err("tx_urb_map %lx\n",self->drvobj->tx_urb_map[0]);

}
#ifdef CONFIG_USE_DMA_ADDR_BUFFER


#ifndef ALIGN
#define ALIGN(a,b)			(((a) + ((b) - 1)) & (~((b)-1)))
#endif
#ifdef CONFIG_USB_AGGR_URB_TX
char * Sstar_usb_pick_txDMABuf(struct sbus_priv *self)
{
	unsigned long flags=0;
	char * buf;
	spin_lock_irqsave(&self->lock, flags);
	if(self->drvobj->NextAllocPost == self->drvobj->NextFreePost){
		if(self->drvobj->tx_dma_addr_buffer_full){
			usb_printk( "Sstar_usb_pick_txDMABuf:tx_dma_addr_buffer_full %d \n",self->drvobj->tx_dma_addr_buffer_full);
			spin_unlock_irqrestore(&self->lock, flags);
			return NULL;
		}
	}
	buf = self->drvobj->NextAllocPost;
	spin_unlock_irqrestore(&self->lock, flags);
	return buf;
}

char * Sstar_usb_get_txDMABuf(struct sbus_priv *self)
{
	char * buf;
	unsigned long flags=0;
	spin_lock_irqsave(&self->lock, flags);
	if(self->drvobj->NextAllocPost == self->drvobj->NextFreePost){
		if(self->drvobj->tx_dma_addr_buffer_full){
			spin_unlock_irqrestore(&self->lock, flags);
			return NULL;
		}
	}
	else {
		
	}
	
	self->drvobj->free_dma_buffer_cnt--;
	if(self->drvobj->free_dma_buffer_cnt <0){
		self->drvobj->free_dma_buffer_cnt=0;
		Sstar_printk_err("free_dma_buffer_cnt ERR,NextAllocPost %p drvobj->NextFreePost %p\n",self->drvobj->NextAllocPost, self->drvobj->NextFreePost);
		
		BUG_ON(1);
	}
	
	buf = self->drvobj->NextAllocPost;
	self->drvobj->NextAllocPost += BUFF_ALLOC_LEN;
	if(self->drvobj->NextAllocPost >= self->drvobj->tx_dma_addr_buffer_end){
		self->drvobj->NextAllocPost = self->drvobj->tx_dma_addr_buffer;
	}
	if(self->drvobj->NextAllocPost == self->drvobj->NextFreePost){
		self->drvobj->tx_dma_addr_buffer_full =1;
	}
	spin_unlock_irqrestore(&self->lock, flags);

	return buf;
}


void Sstar_usb_free_txDMABuf(struct sbus_priv *self)
{
	if(self->drvobj->NextAllocPost == self->drvobj->NextFreePost){
		if(self->drvobj->tx_dma_addr_buffer_full==0){
			Sstar_printk_err("self->drvobj->free_dma_buffer_cnt %d\n",self->drvobj->free_dma_buffer_cnt);
			WARN_ON(self->drvobj->tx_dma_addr_buffer_full==0);
			return;
		}
		self->drvobj->tx_dma_addr_buffer_full = 0;
	}
	else {
		
	}
	self->drvobj->free_dma_buffer_cnt++;
	if(self->drvobj->total_dma_buffer_cnt < self->drvobj->free_dma_buffer_cnt){
		Sstar_printk_err("<WARNING> Sstar_usb_free_txDMABuf (buffer(%p)  NextFreePost(%p))free_dma_buffer_cnt %d \n",
			self->drvobj->NextAllocPost, 
			self->drvobj->NextFreePost,
			self->drvobj->free_dma_buffer_cnt);
		
		BUG_ON(1);
		
	}
	self->drvobj->NextFreePost += BUFF_ALLOC_LEN;
	if(self->drvobj->NextFreePost == self->drvobj->tx_dma_addr_buffer_end){
		self->drvobj->NextFreePost = self->drvobj->tx_dma_addr_buffer;
	}
}
void Sstar_usb_free_txDMABuf_all(struct sbus_priv *self,u8 * buffer,int cnt)
{
	unsigned long flags=0;
	spin_lock_irqsave(&self->lock, flags);
	WARN_ON(cnt==0);
	if((char *)buffer != self->drvobj->NextFreePost){			
		Sstar_printk_err("<WARNING> Sstar_usb_free_txDMABuf_all (buffer(%p) != NextFreePost(%p))free_dma_buffer_cnt %d cnt %d\n",buffer, self->drvobj->NextFreePost,self->drvobj->free_dma_buffer_cnt,cnt);
		BUG_ON(1);
		self->drvobj->NextFreePost = buffer;
	}
	while(cnt--){
		Sstar_usb_free_txDMABuf(self);
	}
	spin_unlock_irqrestore(&self->lock, flags);
}
#endif
#endif //#ifdef CONFIG_USE_DMA_ADDR_BUFFER
int Sstar_usb_urb_malloc(struct sbus_priv *self,struct sbus_urb * pUrb,int max_num,int len,int b_skb,int b_dma_buffer)
{
	int i=0;
#ifdef CONFIG_USE_DMA_ADDR_BUFFER
	len = ALIGN(len,PER_PACKET_LEN);
	self->drvobj->tx_dma_addr = 0;
	if(b_dma_buffer){
#ifdef CONFIG_USB_AGGR_URB_TX
		self->drvobj->tx_dma_addr_buffer_len=len*max_num*URB_AGGR_NUM;//ALIGN(len*max_num,4096);
#else
	    self->drvobj->tx_dma_addr_buffer_len=len*max_num;//ALIGN(len*max_num,4096);
#endif  //CONFIG_USB_AGGR_URB_TX
		Sstar_printk_init("Sstar_usb_urb_malloc CONFIG_USE_DMA_ADDR_BUFFER max_num %d, total %d\n",max_num,(int)self->drvobj->tx_dma_addr_buffer_len);
		self->drvobj->tx_dma_addr_buffer = Sstar_usb_buffer_alloc(self->drvobj->pusbdev,self->drvobj->tx_dma_addr_buffer_len, &self->drvobj->tx_dma_addr);
		if(self->drvobj->tx_dma_addr_buffer==NULL){
			Sstar_dbg(SSTAR_APOLLO_DBG_ERROR, "Can't allocate tx_dma_addr_buffer.");
			goto __free_urb;
		}
		usb_printk( "tx_dma_addr_buffer %x dma %x\n",self->drvobj->tx_dma_addr_buffer ,self->drvobj->tx_dma_addr);
	}
	
#ifdef CONFIG_USB_AGGR_URB_TX
	self->drvobj->NextAllocPost = self->drvobj->tx_dma_addr_buffer;
	self->drvobj->NextFreePost = self->drvobj->tx_dma_addr_buffer;
	self->drvobj->tx_dma_addr_buffer_end = self->drvobj->tx_dma_addr_buffer+self->drvobj->tx_dma_addr_buffer_len;
	self->drvobj->tx_dma_addr_buffer_full = 0;
	self->drvobj->free_dma_buffer_cnt= self->drvobj->tx_dma_addr_buffer_len/BUFF_ALLOC_LEN;
	self->drvobj->total_dma_buffer_cnt= self->drvobj->tx_dma_addr_buffer_len/BUFF_ALLOC_LEN;
	Sstar_printk_init("CONFIG_USB_AGGR_URB_TX enable cnt tx_dma_addr_buffer_end(%p)tx_dma_addr_buffer(%p),%d\n",
		self->drvobj->tx_dma_addr_buffer,self->drvobj->tx_dma_addr_buffer_end,(int)self->drvobj->tx_dma_addr_buffer_len/BUFF_ALLOC_LEN);
#endif  //CONFIG_USB_AGGR_URB_TX
#endif //#ifdef CONFIG_USE_DMA_ADDR_BUFFER
	
	for(i=0;i<max_num;i++){
		pUrb[i].test_urb=usb_alloc_urb(0,GFP_KERNEL);
		if (!pUrb[i].test_urb){
			Sstar_dbg(SSTAR_APOLLO_DBG_ERROR, "Can't allocate test_urb.");
			goto __free_urb;
		}
#ifdef CONFIG_USE_DMA_ADDR_BUFFER
		
		if(b_dma_buffer){		
#ifdef CONFIG_USB_AGGR_URB_TX
			pUrb[i].pallocated_buf =NULL;
			pUrb[i].test_skb = NULL;
			pUrb[i].pallocated_buf_len = 0;
			pUrb[i].dma_transfer_addr = 0;
#else
			pUrb[i].pallocated_buf =self->drvobj->tx_dma_addr_buffer+i*len;
			pUrb[i].test_skb = NULL;
			pUrb[i].pallocated_buf_len = len;
			pUrb[i].dma_transfer_addr = self->drvobj->tx_dma_addr+i*len;
#endif //USB_ATHENAB_AGGR
		}
		else 
#endif //#ifdef CONFIG_USE_DMA_ADDR_BUFFER
		if(b_skb){
			pUrb[i].test_skb = __Sstar_dev_alloc_skb(len,GFP_KERNEL);
			if (!pUrb[i].test_skb){
				Sstar_dbg(SSTAR_APOLLO_DBG_ERROR, "Can't allocate test_skb.");
				goto __free_skb;
			}
			pUrb[i].pallocated_buf = NULL;
		}else {
			#ifndef CONFIG_USE_DMA_ADDR_BUFFER
			pUrb[i].pallocated_buf = Sstar_kzalloc(len,GFP_KERNEL);
			if(pUrb[i].pallocated_buf == NULL){
				Sstar_dbg(SSTAR_APOLLO_DBG_ERROR, "Can't allocate pallocated_buf.");
				goto __free_skb;
			}
			#endif 
			pUrb[i].test_skb = NULL;
		}
		pUrb[i].test_pending =0;
		pUrb[i].urb_id = i;
		pUrb[i].obj = self;
		pUrb[i].data = NULL;
	}
	return 0;
__free_skb:
#ifdef CONFIG_USE_DMA_ADDR_BUFFER
	if(self->drvobj->tx_dma_addr){
		Sstar_usb_buffer_free(self->drvobj->pusbdev,self->drvobj->tx_dma_addr_buffer_len,self->drvobj->tx_dma_addr_buffer, self->drvobj->tx_dma_addr);
		self->drvobj->tx_dma_addr = 0;
	}
#endif //CONFIG_USE_DMA_ADDR_BUFFER
	#ifndef CONFIG_USE_DMA_ADDR_BUFFER
	for( ;i>=0;--i){
		if(pUrb[i].pallocated_buf){
			Sstar_kfree(pUrb[i].pallocated_buf);
			pUrb[i].pallocated_buf = NULL;
		}
	}
	#endif
	for( ;i>=0;--i){
		#ifndef CONFIG_USE_DMA_ADDR_BUFFER
		if(pUrb[i].pallocated_buf){
			Sstar_kfree(pUrb[i].pallocated_buf);
			pUrb[i].pallocated_buf = NULL;
		}
		#endif
		Sstar_dev_kfree_skb(pUrb[i].test_skb);
	}
	i = max_num;
__free_urb:
	for( ;i>=0;--i){
		usb_free_urb(pUrb[i].test_urb);
	}

	return -ENOMEM;
}
#ifdef CONFIG_USB_DATA_XMIT_DIRECTLY
static int Sstar_usb_data_send(struct sbus_priv *self)
{
	int status;
	struct Sstar_common *hw_priv = self->core;
	hw_priv->sbus_ops->lock(hw_priv->sbus_priv);
	do{
		status = hw_priv->sbus_ops->sbus_memcpy_toio(hw_priv->sbus_priv,0x1,NULL,TX_BUFFER_SIZE);
	}while(status > 0);
	hw_priv->sbus_ops->unlock(hw_priv->sbus_priv);
	
	return 0;
}
#endif
#ifdef CONFIG_WSM_CMD_XMIT_DIRECTLY
static void Sstar_usb_wsm_xmit_complete(struct urb *urb)
{
	struct sbus_urb *tx_urb=(struct sbus_urb*)urb->context;
	struct sbus_priv *self = tx_urb->obj;
	
	if(self == NULL)
	{
		Sstar_printk_err("<WARNING> q. hw_priv =0 drop\n");
		return;
	}

	usb_unanchor_urb(self->drvobj->wait_urb);
}
void Sstar_usb_wsm_urb_wait_timeout(struct sbus_priv *self,int timeout)
{
	struct usb_anchor *wsm_anchor = &self->drvobj->wsm_submitted;
	int ret = 0;
	struct Sstar_common *hw_priv = self->core;
	
	ret = usb_wait_anchor_empty_timeout(wsm_anchor,timeout);

	if(ret == 0){
		Sstar_printk_err("%s:wsm cmd send timout\n",__func__);
		if(hw_priv != NULL){
			Sstar_usb_free_err_cmd(self);
		}
	}
}
int Sstar_usb_wsm_urb_int(struct sbus_priv *self)
{
	int ret = 0;
	struct sbus_urb *wsm_urb = &self->drvobj->wsm_urb;
	struct usb_anchor *wsm_anchor = &self->drvobj->wsm_submitted;
	
	init_usb_anchor(wsm_anchor);
	wsm_urb->test_urb=usb_alloc_urb(0,GFP_KERNEL);
	
	if(wsm_urb->test_urb == NULL){
		ret = -1;
		goto err;
	}
	self->drvobj->wait_urb = usb_alloc_urb(0,GFP_KERNEL);

	if(self->drvobj->wait_urb == NULL){
		ret = -1;
		goto err;
	}
	
	wsm_urb->obj = self;
	wsm_urb->pallocated_buf = Sstar_kzalloc(TX_BUFFER_SIZE,GFP_KERNEL);

	if(wsm_urb->pallocated_buf == NULL){
		ret = -1;
		goto err;
	}
	wsm_urb->obj = self;
	return 0;
err:
	if(self->drvobj->wait_urb){
		usb_free_urb(self->drvobj->wait_urb);
		self->drvobj->wait_urb = NULL;
	}

	if(wsm_urb->test_urb){
		usb_free_urb(wsm_urb->test_urb);
		wsm_urb->test_urb = NULL;
	}

	if(wsm_urb->pallocated_buf){
		Sstar_kfree(wsm_urb->pallocated_buf);
		wsm_urb->pallocated_buf = NULL;
	}
	return ret;
}

int Sstar_usb_wsm_urb_deint(struct sbus_priv *self)
{
	struct sbus_urb *wsm_urb = &self->drvobj->wsm_urb;
	
	Sstar_usb_wsm_urb_wait_timeout(self,1000);

	if(wsm_urb->test_urb){
		usb_kill_urb(wsm_urb->test_urb);
		usb_free_urb(wsm_urb->test_urb);
		wsm_urb->test_urb = NULL;
	}
	if(self->drvobj->wait_urb){
		usb_free_urb(self->drvobj->wait_urb);
		self->drvobj->wait_urb = NULL;
	}
	if(wsm_urb->pallocated_buf){
		Sstar_kfree(wsm_urb->pallocated_buf);
		wsm_urb->pallocated_buf = NULL;
	}

	return 0;
}
static int __Sstar_usb_wsm_send(struct sbus_priv *self)
{
	struct Sstar_common *hw_priv = self->core;
	struct sbus_urb *wsm_urb = NULL;
	struct usb_anchor *wsm_anchor = NULL;
	int status;
	u8 *txdata = NULL;
	u32 tx_len = 0;
	u32 actual_len = 0;
	struct urb *wait_urb = self->drvobj->wait_urb;		
	struct wsm_hdr_tx *wsm = NULL;

	spin_lock_bh(&hw_priv->tx_com_lock);
	wsm_urb = &self->drvobj->wsm_urb;
	wsm_anchor = &self->drvobj->wsm_submitted;
	if(usb_anchor_empty(wsm_anchor) != 1){
		WARN_ON(1);
		usb_unanchor_urb(wsm_urb->test_urb);
		usb_unanchor_urb(wait_urb);
		if(usb_anchor_empty(wsm_anchor) != 1){
			Sstar_printk_err("%s:wsm anchor is not empty\n",__func__);
			BUG_ON(1);
		}
	}
	wsm_alloc_tx_buffer_NoLock(hw_priv);
	spin_lock_bh(&hw_priv->wsm_cmd.lock);
	if(hw_priv->wsm_cmd.ptr){
		
		wsm = (struct wsm_hdr_tx *)hw_priv->wsm_cmd.ptr;
		actual_len = hw_priv->wsm_cmd.len;
		tx_len = wsm->usb_len;
		txdata= hw_priv->wsm_cmd.ptr;
		
		self->tx_vif_selected = -1;

		wsm->flag = __cpu_to_le16(0xe569)<<16;
		wsm->flag |= BIT(6);
		wsm->flag |= (self->tx_hwChanId & 0x1f);
		wsm->id &= __cpu_to_le32(~WSM_TX_SEQ(WSM_TX_SEQ_MAX));
		wsm->id |= cpu_to_le32(WSM_TX_SEQ(hw_priv->wsm_tx_seq));

		wsm_urb->callback_handler = NULL;
		wsm_urb->test_pending = 1;
		wsm_urb->test_seq= hw_priv->wsm_tx_seq;
		wsm_urb->test_hwChanId= self->tx_hwChanId;
		hw_priv->wsm_cmd.last_send_cmd = hw_priv->wsm_cmd.cmd;
		hw_priv->wsm_cmd.ptr = NULL;

	}
	spin_unlock_bh(&hw_priv->wsm_cmd.lock);
	
	usb_anchor_urb(wsm_urb->test_urb, wsm_anchor);
	usb_anchor_urb(wait_urb,wsm_anchor);
	if(!Sstar_wifi_get_status()&&(!Sstar_bh_is_term(hw_priv))&&txdata&&wsm_urb->pallocated_buf){
		unsigned int pipe;
		u8 *submit_buff = wsm_urb->pallocated_buf;
		
		BUG_ON(submit_buff == NULL);
		memcpy(submit_buff,txdata,actual_len);
		pipe = usb_sndbulkpipe(self->drvobj->pusbdev, self->drvobj->ep_out);
		usb_fill_bulk_urb(wsm_urb->test_urb,
			self->drvobj->pusbdev, pipe,submit_buff,tx_len,
			Sstar_usb_wsm_xmit_complete,wsm_urb);
		status = usb_submit_urb(wsm_urb->test_urb, GFP_ATOMIC);
	}else{
		printk_once(KERN_ERR "%s:module exit,do not submit urb(%p)\n",__func__,wsm_urb->pallocated_buf);
		status = -1;
	}
	if(status == 0){
		self->tx_hwChanId++;
		hw_priv->wsm_tx_seq = (hw_priv->wsm_tx_seq + 1) & WSM_TX_SEQ_MAX;
	}else {
		usb_unanchor_urb(wsm_urb->test_urb);
		usb_unanchor_urb(wait_urb);
		wsm_release_tx_buffer_NoLock(hw_priv,1);
		if(txdata)
			Sstar_usb_free_err_cmd(self);
	}
	spin_unlock_bh(&hw_priv->tx_com_lock);

	return status;
}
int Sstar_usb_wsm_send(struct sbus_priv *self)
{
	struct Sstar_common *hw_priv = self->core;
	struct sbus_urb *wsm_urb = NULL;
	struct usb_anchor *wsm_anchor = NULL;
	int status;
	u8 cmd_recovery = 0;
	
	if((hw_priv == NULL) || (self->drvobj == NULL)){
		WARN_ON(1);
		return -1;
	}
	
	wsm_urb = &self->drvobj->wsm_urb;
	wsm_anchor = &self->drvobj->wsm_submitted;

	hw_priv->sbus_ops->lock(hw_priv->sbus_priv);
cmd_retry:
	status = __Sstar_usb_wsm_send(self);
	if(status == 0){

		/*
		*wait urb send
		*/
		status = usb_wait_anchor_empty_timeout(wsm_anchor,20000);

		if(status != 0){
			/*
			*urb has been send but status is not sucess,maybe usb has been
			*disconnect,so releas the wsm cmd
			*/
			if(wsm_urb->test_urb->status != 0){
				Sstar_printk_err("%s:wsm cmd send err\n",__func__);
				Sstar_usb_free_err_cmd(self);
			}
		}else {
			/*
			*kill wsm urb,usb kernel will call Sstar_usb_wsm_xmit_complete
			*/
			usb_kill_urb(wsm_urb->test_urb);
			wsm_release_tx_buffer_NoLock(hw_priv,1);
			if(usb_wait_anchor_empty_timeout(wsm_anchor,20000) == 0){
				WARN_ON(1);
				usb_unanchor_urb(wsm_urb->test_urb);
				usb_unanchor_urb(self->drvobj->wait_urb);
			}
			if(cmd_recovery == 0){
				int recovery = 0;
				Sstar_printk_err("%s:wsm urb not send,try to recovery\n",__func__);
				recovery = wsm_recovery(hw_priv);
				if((recovery != RECOVERY_ERR)&&(recovery != RECOVERY_BH_HALT)){
					spin_lock_bh(&hw_priv->wsm_cmd.lock);
					hw_priv->wsm_cmd.ptr = hw_priv->wsm_cmd_buf.begin;
					hw_priv->wsm_cmd.len = hw_priv->wsm_cmd_buf.data - hw_priv->wsm_cmd_buf.begin;
					cmd_recovery = 1;
					spin_unlock_bh(&hw_priv->wsm_cmd.lock);
					goto cmd_retry;
				}
			}
			/*
			*recovery err ,so release cmd
			*/
			Sstar_usb_free_err_cmd(self);
		}
	}
	hw_priv->sbus_ops->unlock(hw_priv->sbus_priv);
	return 0;
}
#endif
static int Sstar_usb_wait_submitted_xmited(struct sbus_priv *self)
{
	int status = 0;

	status = usb_wait_anchor_empty_timeout(&self->drvobj->tx_submitted,10000);

	if(status == 0)
		return -1;

#ifdef CONFIG_WSM_CMD_XMIT_DIRECTLY
	status = usb_wait_anchor_empty_timeout(&self->drvobj->wsm_submitted,10000);

	if(status == 0)
		return -1;
#endif

#ifdef CONDIF_SSTAR_CTRL_REQUEST_ASYNC
	status = usb_wait_anchor_empty_timeout(&self->drvobj->ctrl_submitted,10000);	
	if(status == 0)
		return -1;
	status = atomic_read(&self->drvobj->ctrl_err);
#endif
	return status;
}

int Sstar_usb_free_tx_wsm(struct sbus_priv *self,struct sbus_urb *tx_urb)
{
	struct wsm_tx *wsm = NULL;


	
	wsm = tx_urb->data; 
	if((wsm) && (!(wsm->htTxParameters&__cpu_to_le32(WSM_NEED_TX_CONFIRM)))){
		
		struct Sstar_queue *queue;
		u8 queue_id;
		struct Sstar_common	*hw_priv = self->core;
		struct sk_buff *skb;
		const struct Sstar_txpriv *txpriv;

		queue_id = Sstar_queue_get_queue_id(wsm->packetID);

		BUG_ON(queue_id >= 4);

		queue = &hw_priv->tx_queue[queue_id];
		BUG_ON(queue == NULL);

		if(!WARN_ON(Sstar_queue_get_skb(queue, wsm->packetID, &skb, &txpriv))) {

			struct ieee80211_tx_info *tx = IEEE80211_SKB_CB(skb);
			//int tx_count = 0;
			int i;

			wsm_release_vif_tx_buffer_Nolock(hw_priv,txpriv->if_id,1);
			wsm_release_tx_buffer_NoLock(hw_priv, 1);
			
			tx->flags |= IEEE80211_TX_STAT_ACK;
			tx->status.rates[0].count = 1;
			for (i = 1; i < IEEE80211_TX_MAX_RATES; ++i) {
				tx->status.rates[i].count = 0;
				tx->status.rates[i].idx = -1;
			}

#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
			Sstar_queue_remove(hw_priv, queue, wsm->packetID);
#else
			Sstar_queue_remove(queue, wsm->packetID);
#endif
		}else {
			wsm_release_vif_tx_buffer_Nolock(hw_priv,Sstar_queue_get_if_id(wsm->packetID),1);
			wsm_release_tx_buffer_NoLock(hw_priv, 1);
		}
		tx_urb->data = NULL;
		return 1;
	}
	return 0;
}
void Sstar_usb_release_tx_err_urb(struct sbus_priv *self,unsigned long *bitmap,int tx)
{
	unsigned long flags=0;
	int clear_id;
	struct sbus_urb *tx_urb = NULL;	
	struct Sstar_common	*hw_priv = self->core;
	
	if(tx == 0)
		return;
	
	spin_lock_irqsave(&self->lock, flags);
	clear_id = find_first_bit(self->drvobj->tx_err_urb_map,TX_URB_NUM);
	for(;(clear_id>=0)&&(clear_id<TX_URB_NUM);clear_id = find_first_bit(self->drvobj->tx_err_urb_map,TX_URB_NUM))
	{
		__clear_bit(clear_id,self->drvobj->tx_err_urb_map);
		__clear_bit(clear_id,bitmap);
		spin_unlock_irqrestore(&self->lock, flags);

		tx_urb = &self->drvobj->tx_urb[clear_id];
		
		if(tx_urb == NULL){
			Sstar_printk_err( "tx_urb == NULL\n");
			spin_lock_irqsave(&self->lock, flags);
			continue;
		}
		self->tx_hwChanId--;
		hw_priv->wsm_tx_seq = (hw_priv->wsm_tx_seq - 1) & WSM_TX_SEQ_MAX;
		if(tx_urb->data == NULL){
			Sstar_usb_free_err_cmd(self);
		}else{			
			Sstar_usb_free_err_data(self,tx_urb);
		}
		tx_urb->data = NULL;
		spin_lock_irqsave(&self->lock, flags);
	}
	spin_unlock_irqrestore(&self->lock, flags);
}
int Sstar_usb_urb_get(struct sbus_priv *self,unsigned long *bitmap,int max_urb,int tx)
{
	int id = 0;
	unsigned long flags=0;

	Sstar_usb_release_tx_err_urb(self,bitmap,tx);
	spin_lock_irqsave(&self->lock, flags);
#ifndef CONFIG_USB_AGGR_URB_TX
#ifdef CONFIG_TX_NO_CONFIRM
	if(tx){
		int clear_id;
		struct sbus_urb *tx_urb = NULL;	
		clear_id = find_first_bit(self->drvobj->txpending_urb_map,TX_URB_NUM);
		for(;(clear_id>=0)&&(clear_id<TX_URB_NUM);clear_id = find_first_bit(self->drvobj->txpending_urb_map,TX_URB_NUM))
		{
			__clear_bit(clear_id,self->drvobj->txpending_urb_map);
			__clear_bit(clear_id,bitmap);
			spin_unlock_irqrestore(&self->lock, flags);

			tx_urb = &self->drvobj->tx_urb[clear_id];
			
			if(tx_urb == NULL){
				Sstar_printk_err("tx_urb == NULL\n");
				spin_lock_irqsave(&self->lock, flags);
				continue;
			}
			if(tx_urb->data == NULL){
				spin_lock_irqsave(&self->lock, flags);
				continue;
			}
			
			Sstar_usb_free_tx_wsm(self,tx_urb);
			tx_urb->data = NULL;
			spin_lock_irqsave(&self->lock, flags);
		}
	}
#endif  //CONFIG_TX_NO_CONFIRM
#endif //CONFIG_USB_AGGR_URB_TX
	id= find_first_zero_bit(bitmap,max_urb);
	if((id>=max_urb)||(id<0)){
		spin_unlock_irqrestore(&self->lock, flags);
		return -1;
	}
	__set_bit(id,bitmap);
	if(tx){
		wifi_tx_urb_pending++;
		WARN_ON(wifi_tx_urb_pending>TX_URB_NUM);
	}
	spin_unlock_irqrestore(&self->lock, flags);

	return id;
}
#ifdef CONFIG_TX_NO_CONFIRM
int Sstar_usb_set_pending_urb(struct sbus_priv *self,int id,int tx)
{
	int release = 1;
	//unsigned long flags=0;

	while(tx){
		struct sbus_urb *tx_urb = &self->drvobj->tx_urb[id];	
		struct wsm_tx *wsm = NULL;

		if(tx_urb == NULL)
			break;
		
		if(tx_urb->data == NULL)
			break;

		wsm = tx_urb->data;
		if(!(wsm->htTxParameters&__cpu_to_le32(WSM_NEED_TX_CONFIRM))){
			release = 0;
			__set_bit(id,self->drvobj->txpending_urb_map);
		}else {
			tx_urb->data = NULL;
		}
		break;
	}

	return release;
}
#endif //#ifdef CONFIG_TX_NO_CONFIRM
int Sstar_usb_set_err_urb(struct sbus_priv *self,int id,int tx)
{
	int release = 1;
	//unsigned long flags=0;

	while(tx){
		struct sbus_urb *tx_urb = &self->drvobj->tx_urb[id];	
		struct urb *urb = NULL;
		if(tx_urb == NULL)
			break;

		if( tx_urb->test_urb == NULL)
			break;

		urb = tx_urb->test_urb;
		
		if(urb->status == 0)
			break;
		Sstar_printk_err("%s: tx err urb\n",__func__);
		release = 0;
		__set_bit(id,self->drvobj->tx_err_urb_map);
		urb->status = 0;
		break;
	}

	return release;
}

void Sstar_usb_urb_put(struct sbus_priv *self,unsigned long *bitmap,int id,int tx)
{
	unsigned long flags=0;
	int release = 1;
	spin_lock_irqsave(&self->lock, flags);
// ARESB we not need set err urb, we will recovery
#if (PROJ_TYPE<ARES_B)
	release = Sstar_usb_set_err_urb(self,id,tx);
#endif  //ARES_B

#ifndef CONFIG_USB_AGGR_URB_TX
#ifdef CONFIG_TX_NO_CONFIRM
	if(release)
		release = Sstar_usb_set_pending_urb(self,id,tx);
#endif //#ifdef CONFIG_TX_NO_CONFIRM
#endif //#ifndef CONFIG_USB_AGGR_URB_TX

	if(tx){
		wifi_tx_urb_pending--;
		WARN_ON(wifi_tx_urb_pending<0);
	}
	//WARN_ON((*bitmap & BIT(id))==0);
	
	if(release)
		__clear_bit(id,bitmap);
	spin_unlock_irqrestore(&self->lock, flags);
}
extern bool Sstar_rx_directly(struct Sstar_common *hw_priv,struct sk_buff *skb,
			int (*rx_func)(struct Sstar_common *hw_priv,struct sk_buff *skb));
static void Sstar_usb_submit_rev_skb(struct sbus_priv *self,struct sk_buff *skb,int urb_id)
{
	struct Sstar_common *hw_priv = self->core;
	unsigned long flags;
	bool rx_running;

	spin_lock_irqsave(&hw_priv->rx_frame_queue.lock, flags);
	__Sstar_skb_queue_tail(&hw_priv->rx_frame_queue, skb);
	rx_running = hw_priv->bh_running;
	spin_unlock_irqrestore(&hw_priv->rx_frame_queue.lock, flags);

	if(urb_id != -1)
		Sstar_usb_urb_put(self,self->drvobj->rx_urb_map,urb_id,0);
	
	if(rx_running == false)
		Sstar_usb_rev_schedule(self);
}
static int Sstar_usb_rev_skb_cb(struct Sstar_common *hw_priv,struct sk_buff *skb)
{
	Sstar_usb_submit_rev_skb(hw_priv->sbus_priv,skb,-1);
	return 0;
}
static void Sstar_usb_receive_data_complete(struct urb *urb)
{
	struct sbus_urb *rx_urb=(struct sbus_urb*)urb->context;
	struct sbus_priv *self = NULL;
	struct sk_buff *skb=NULL;
	struct Sstar_common *hw_priv=NULL;
	int RecvLength=urb->actual_length;
	struct wsm_hdr *wsm;
	static u8 run_inirq = 1;
	
	usb_printk( "rxend  Len %d\n",RecvLength);
	
	if(rx_urb != NULL){
		self = rx_urb->obj;
		skb = rx_urb->test_skb;
	}
	if(self != NULL){
		hw_priv = self->core;
	}
	
	if(!hw_priv)
		goto __free;
	if (!skb){
		WARN_ON(1);
		goto __free;
	}	

	switch(urb->status){
		case 0:
			break;
		case -ENOENT:
		case -ECONNRESET:
		case -ENODEV:
		case -ESHUTDOWN:
			Sstar_printk_bus("Sstar_usb_rx_complete1 error status=%d len %d\n",urb->status,RecvLength);
			goto __free;
		case -EPROTO:
			Sstar_printk_bus( "Sstar_usb_rx_complete3 error status=%d len %d\n",urb->status,RecvLength);
			if(RecvLength !=0){
				break;
			}
		case -EOVERFLOW:
			Sstar_printk_err("<ERROR>:EOVERFLOW,len=%d\n",RecvLength);
			if(RecvLength)
				break;
			Sstar_printk_bus("<ERROR>:Sstar_usb_rx_complete status=%d len %d\n",urb->status,RecvLength);
			goto __free;
		default:
			Sstar_printk_bus("Sstar_usb_rx_complete2 error status=%d len %d\n",urb->status,RecvLength);
			goto resubmit;
	}

	if((self->drvobj->suspend_skb != NULL)&&(self->drvobj->suspend_skb_len != 0))
	{
		if(Sstar_skb_tailroom(self->drvobj->suspend_skb)<self->drvobj->suspend_skb_len+RecvLength)
		{
			struct sk_buff * long_suspend_skb = NULL;
			BUG_ON(self->drvobj->suspend_skb_len+RecvLength>RX_BUFFER_SIZE);
			Sstar_printk_err("suspend skb len is not enough(%d),(%ld)\n",
				Sstar_skb_tailroom(self->drvobj->suspend_skb),self->drvobj->suspend_skb_len+RecvLength);

			long_suspend_skb = Sstar_dev_alloc_skb(RX_BUFFER_SIZE+64);
			BUG_ON(!long_suspend_skb);
			Sstar_skb_reserve(long_suspend_skb, 64);
			memcpy((u8 *)long_suspend_skb->data,self->drvobj->suspend_skb->data,self->drvobj->suspend_skb_len);
			Sstar_dev_kfree_skb(self->drvobj->suspend_skb);
			self->drvobj->suspend_skb = long_suspend_skb;
		}
		memcpy((u8 *)self->drvobj->suspend_skb->data+self->drvobj->suspend_skb_len,skb->data,RecvLength);
		RecvLength += self->drvobj->suspend_skb_len;
		memcpy(skb->data,(u8 *)self->drvobj->suspend_skb->data,RecvLength);
		self->drvobj->suspend_skb_len = 0;					
		Sstar_printk_bus("suspend rebuild\n");
	}
	
	wsm = (struct wsm_hdr *)skb->data;
	if (wsm->len != RecvLength){	
		//add because usb not reset when rmmod driver, just drop error frame  
		if(hw_priv->wsm_caps.firmwareReady==0){
			//BUG_ON(1);
			goto resubmit;
		}
		if(((wsm->len  % 512)==0) && ((wsm->len+1) == RecvLength)){
			//this correct , lmac output len = (wsm len +1 ) ,inorder to let hmac usb callback
		}
		else {
			if((urb->actual_length > RX_BUFFER_SIZE) || (wsm->len < urb->actual_length)){
			
				int actual_length = urb->actual_length;
				u8 *data = skb->data;
				Sstar_printk_err("actual_length(%d),max_len(%d),transfer_buffer_length(%d)\n",
								urb->actual_length,RX_BUFFER_SIZE,urb->transfer_buffer_length);
					
				while(actual_length > 0){
					wsm = (struct wsm_hdr *)data;
					Sstar_printk_err("rx id (%x),len(%d)\n",wsm->id,wsm->len);
					data += wsm->len;
					actual_length -= wsm->len;
				}
			}
			wsm = (struct wsm_hdr *)skb->data;
			Sstar_printk_err("rx rebulid usbsuspend  id %d wsm->len %d,RecvLength %d\n",wsm->id,wsm->len,RecvLength);

			if(self->drvobj->suspend_skb_len== 0){ 
				if(wsm->len > RX_BUFFER_SIZE){
					Sstar_printk_err(" %s %d id %d wsm->len %d,RecvLength %d\n",__func__,__LINE__,wsm->id,wsm->len,RecvLength);
					goto resubmit;
				}
				Sstar_printk_err("rx rebulid usbsuspend0	\n");
				/*
				* alloc 4K buff for suspend_skb is save
				*/
				BUG_ON(self->drvobj->suspend_skb == NULL);
				memcpy((u8 *)self->drvobj->suspend_skb->data,skb->data,RecvLength);
				self->drvobj->suspend_skb_len = RecvLength;
				goto resubmit;
			}
		}
	}
	
	WARN_ON(self->drvobj->suspend_skb_len != 0);
	
	if (WARN_ON(4 > RecvLength)){
		Sstar_printk_err("%s %d id %d wsm->len %d,RecvLength %d\n",__func__,__LINE__,wsm->id,wsm->len,RecvLength);
		frame_hexdump("Sstar_usb_receive_data_complete",(u8 *)wsm,32);
		goto resubmit;
		//goto __free;
	}

	BUG_ON(RecvLength > RX_BUFFER_SIZE);
	
	skb->pkt_type = SSTAR_RX_RAW_FRAME;
		
	if(rx_urb->callback_handler){
		rx_urb->callback_handler(hw_priv,wsm);
		return;
	}
	
	self->rx_seqnum++;
	printk_once(KERN_ERR"%s:in_irq(%d)\n",__func__,!!in_irq());

	if(!hw_priv->init_done){
		Sstar_printk_err( "[BH] irq. init_done =0 drop\n");
		goto __free;
	}
	if (/* WARN_ON */(hw_priv->bh_error))
		goto __free;
	
#ifdef CONFIG_USB_URB_RX_SUBMIT_DIRECTLY
	if(0){
	}else {
		extern int Sstar_rx_single_channel_bh_cb(struct Sstar_common *hw_priv,struct sk_buff *skb);
		int (*cb_handle)(struct Sstar_common *hw_priv,struct sk_buff *skb);
#if 0
		static u32 inirq_rev = 0;

		inirq_rev++;
		if((inirq_rev%2) == 0){
			rx_func = Sstar_rx_single_channel_bh_cb;
			skb->pkt_type = SSTAR_RX_DERICTLY_DATA_FRAME;
		}else {
			rx_func = Sstar_usb_rev_skb_cb;
			skb->pkt_type = SSTAR_RX_WSM_DATA_FRAME;
		}
#else		
		if(!in_irq()){
			cb_handle = Sstar_rx_single_channel_bh_cb;
			skb->pkt_type = SSTAR_RX_DERICTLY_DATA_FRAME;
			if(run_inirq)
				printk_once("[Sstar_log]%s:urb in softirq\n",__func__);
			run_inirq = 0;
		}else {
			cb_handle = Sstar_usb_rev_skb_cb;
			skb->pkt_type = SSTAR_RX_WSM_DATA_FRAME;

			if(run_inirq == 0)
				printk_once("[Sstar_log]%s:urb in hardirq\n",__func__);
			run_inirq = 1;
		}
#endif		
		if(Sstar_rx_directly(hw_priv,skb,cb_handle) == true){
			int status=0;		
			rx_urb->test_skb=skb;
			Sstar_skb_trim(rx_urb->test_skb,0);

			rx_urb->callback_handler = NULL;
			rx_urb->test_pending = 1;

			usb_fill_bulk_urb(rx_urb->test_urb, 
							  self->drvobj->pusbdev, 
							  usb_rcvbulkpipe(self->drvobj->pusbdev, self->drvobj->ep_in),
							  skb->data,RX_BUFFER_SIZE,Sstar_usb_receive_data_complete,rx_urb);
			usb_anchor_urb(rx_urb->test_urb, &self->drvobj->rx_submitted);
			status = usb_submit_urb(rx_urb->test_urb, GFP_ATOMIC);
			if (status) {
				usb_unanchor_urb(rx_urb->test_urb);
				Sstar_printk_err("receive_data usb_submit_urb ++ ERR %d\n",status);
				goto __free;
			}
			return;
		}
	}
#endif
	rx_urb->test_skb = NULL;
	Sstar_usb_submit_rev_skb(self,skb,rx_urb->urb_id);
	return;
resubmit:
	if(!hw_priv->init_done){
		Sstar_printk_err( "[BH] irq. init_done =0 drop\n");
		goto __free;
	}
	if (/* WARN_ON */(hw_priv->bh_error))
		goto __free;

	Sstar_usb_urb_put(self,self->drvobj->rx_urb_map,rx_urb->urb_id,0);
	return;
	
__free:
	if(self->drvobj->suspend_skb_len != 0){
		Sstar_printk_err("rx rebulid usbsuspend3	rx drop\n");
	}
	Sstar_usb_urb_put(self,self->drvobj->rx_urb_map,rx_urb->urb_id,0);
	Sstar_printk_err("[WARNING] Sstar_usb_receive_data drop\n");
	return;

}




static int Sstar_usb_receive_data(struct sbus_priv *self,unsigned int addr,void *dst, int count,sbus_callback_handler hander)
{
	unsigned int pipe;
	int status=0;
	struct sk_buff *skb;
	struct sbus_urb *rx_urb;
	int urb_id;

	if(self->suspend == 1){
		Sstar_printk_err("%s:usb suspend\n",__func__);
		status =  2;
		goto system_err;
	}
	if(Sstar_wifi_get_status()>=2){
		Sstar_printk_err("Sstar_usb_receive_data drop urb req because rmmod driver\n");
		status = 3;
		goto system_err;
	}

	urb_id = Sstar_usb_urb_get(self,self->drvobj->rx_urb_map,RX_URB_NUM,0);
	if(urb_id<0){
		status=-4;
		goto system_err;
	}
	G_rx_urb=urb_id;
	rx_urb = &self->drvobj->rx_urb[urb_id];
	//if not rxdata complete
	//initial new rxdata
	if(rx_urb->test_skb == NULL){
		if(dst ){
			rx_urb->test_skb=dst;
			Sstar_skb_trim(rx_urb->test_skb,0);

		}
		else {
			skb=Sstar_dev_alloc_skb(count+64);
			if (!skb){
				status=-1;
				Sstar_printk_err("Sstar_usb_receive_data++ Sstar_dev_alloc_skb %p ERROR\n",skb);
				goto __err_skb;
			}
			Sstar_skb_reserve(skb, 64);
			rx_urb->test_skb=skb;
		}
	}
	else {
		if(dst ){	
			Sstar_dev_kfree_skb(dst);		
		}

	}
	
	skb = rx_urb->test_skb;
	rx_urb->callback_handler = hander;
	rx_urb->test_pending = 1;
	//atomic_xchg(&self->rx_lock, 1);

	pipe = usb_rcvbulkpipe(self->drvobj->pusbdev, self->drvobj->ep_in);
	usb_fill_bulk_urb(rx_urb->test_urb, self->drvobj->pusbdev, pipe,skb->data,count,Sstar_usb_receive_data_complete,rx_urb);
	usb_anchor_urb(rx_urb->test_urb, &self->drvobj->rx_submitted);
	status = usb_submit_urb(rx_urb->test_urb, GFP_ATOMIC);
	usb_printk( "usb_rx urb_id %d\n",urb_id);
	if (status) {
		usb_unanchor_urb(rx_urb->test_urb);
		status = -2;
		//atomic_xchg(&self->rx_lock, 0);
		Sstar_printk_err("receive_data usb_submit_urb ++ ERR %d\n",status);
		goto __err_skb;
	}

__err_skb:
	if(status < 0){
		Sstar_usb_urb_put(self,self->drvobj->rx_urb_map,urb_id,0);
	}
	return status;
system_err:
	if(dst)
		Sstar_dev_kfree_skb(dst);
	return status;
}


static int Sstar_usb_memcpy_fromio(struct sbus_priv *self,
				     unsigned int addr,
				     void *dst, int count)
{
	int i=0;
	if(atomic_add_return(0, &self->rx_lock)){
		return -1;
	}
	for(i=0;i<RX_URB_NUM;i++){
	 	Sstar_usb_receive_data(self,addr,dst,count,NULL);
	}
	return 0;
}
void Sstar_usb_rxlock(struct sbus_priv *self)
{
	atomic_add(1, &self->rx_lock);
}
void Sstar_usb_rxunlock(struct sbus_priv *self)
{
	atomic_sub(1, &self->rx_lock);
}

static int Sstar_usb_memcpy_toio(struct sbus_priv *self,
				   unsigned int addr,
				   const void *src, int count)
{
	return Sstar_usb_xmit_data(self, addr,(void *)src,count,NULL);
}

static int Sstar_usb_memcpy_fromio_async(struct sbus_priv *self,
				     unsigned int addr,
				     void *dst, int count,sbus_callback_handler func)
{
	 int ret = 0;

	//  self->rx_callback_handler = func;

	// if(atomic_add_return(0, &self->rx_lock)){
	//	 return -1;
	 //}

	 Sstar_usb_receive_data(self,addr,dst,count,func);

	 return ret;
}

static int Sstar_usb_memcpy_toio_async(struct sbus_priv *self,
				   unsigned int addr,
				   const void *src, int count ,sbus_callback_handler func)
{
	 int ret =  0;
	 Sstar_usb_xmit_data(self, addr,(void *)src, count,func);

	 return ret;
}

static void Sstar_usb_lock(struct sbus_priv *self)
{
	//mutex_lock(&self->sbus_mutex);
	Sstar_usb_pm_async(self,0);

}

static void Sstar_usb_unlock(struct sbus_priv *self)
{
	//mutex_unlock(&self->sbus_mutex);
	Sstar_usb_pm_async(self,1);
}

static int Sstar_usb_off(const struct Sstar_platform_data *pdata)
{
	int ret = 0;

	//if (pdata->insert_ctrl)
	//	ret = pdata->insert_ctrl(pdata, false);
	return ret;
}

static int Sstar_usb_on(const struct Sstar_platform_data *pdata)
{
	int ret = 0;

   // if (pdata->insert_ctrl)
	//	ret = pdata->insert_ctrl(pdata, true);

	return ret;
}


static int Sstar_usb_reset(struct sbus_priv *self)
{
//	u32 regdata = 1;
	Sstar_printk_bus(" %s\n",__func__);
	usb_reset_device(interface_to_usbdev(self->drvobj->pusbintf));
	//#ifdef HW_DOWN_FW
	//Sstar_usb_hw_write_port(self,0x16100074,&regdata,4);
	//#else
	//Sstar_usb_sw_write_port(self,0x16100074,&regdata,4);
	//#endif
	return 0;
}
static int Sstar_usb_lock_reset(struct sbus_priv *self){
	int result;
	result=usb_lock_device_for_reset(interface_to_usbdev(self->drvobj->pusbintf),self->drvobj->pusbintf);
	if(result<0){
		Sstar_printk_err("unable to lock device for reset :%d\n",result);
	}else{
		result=usb_reset_device(interface_to_usbdev(self->drvobj->pusbintf));
		usb_unlock_device(interface_to_usbdev(self->drvobj->pusbintf));
	}
	return result;
}
static u32 Sstar_usb_align_size(struct sbus_priv *self, u32 size)
{
	size_t aligned = size;
	return aligned;
}

int Sstar_usb_set_block_size(struct sbus_priv *self, u32 size)
{
	return 0;
}
#ifdef CONFIG_PM
int Sstar_usb_pm(struct sbus_priv *self, bool  auto_suspend)
{
	int ret = 0;

	//printk("Sstar_usb_pm %d -> %d\n",self->auto_suspend ,auto_suspend);
	//if(self->auto_suspend == auto_suspend){
		//printk("***********func=%s,usage_count=%d\n",__func__,self->drvobj->pusbdev->dev.power.usage_count);
	//	return;
	//}
	self->auto_suspend  = auto_suspend;
#if USB_AUTO_WAKEUP
	if(auto_suspend){
#if (LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,33))
			usb_autopm_put_interface(self->drvobj->pusbintf);
#elif (LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,20))
			usb_autopm_enable(self->drvobj->pusbintf);
#else
			usb_autosuspend_device(self->drvobj->pusbdev, 1);
#endif //(LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,33))
	}
	else { //if(auto_suspend)
#if (LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,33))
		usb_autopm_get_interface(self->drvobj->pusbintf);
#elif (LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,20))
			usb_autopm_disable(self->drvobj->pusbintf);
#else //(LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,33))
			usb_autoresume_device(self->drvobj->pusbdev, 1);
#endif //(LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,33))
	}
#endif  //#if USB_AUTO_WAKEUP
	//printk("***********func=%s,usage_count=%d\n",__func__,self->drvobj->pusbdev->dev.power.usage_count);
	return ret;
}
int Sstar_usb_pm_async(struct sbus_priv *self, bool  auto_suspend)
{
	int ret = 0;

	self->auto_suspend  = auto_suspend;
	
#if USB_AUTO_WAKEUP
	if(auto_suspend){
#if (LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,33))
		usb_autopm_put_interface_async(self->drvobj->pusbintf);
#else
#endif
	}
	else {
#if (LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,33))
		usb_autopm_get_interface_async(self->drvobj->pusbintf);
#else
#endif
	}
#endif  //#if USB_AUTO_WAKEUP

	return ret;
}
#else
int Sstar_usb_pm(struct sbus_priv *self, bool  auto_suspend)
{
	return 0;
}
int Sstar_usb_pm_async(struct sbus_priv *self, bool  auto_suspend)
{

	return 0;
}
#endif
static int Sstar_usb_irq_subscribe(struct sbus_priv *self, sbus_irq_handler handler,void *priv)
{
	int ret = 0;
	return ret;
}
static int Sstar_usb_irq_unsubscribe(struct sbus_priv *self)
{
	int ret = 0;
	return ret;
}

void Sstar_wtd_wakeup( struct sbus_priv *self)
{
#ifdef CONFIG_SSTARWIFI_WDT
	if(atomic_read(&self->wtd->wtd_term))
		return;
	atomic_set(&g_wtd.wtd_run, 1);
	Sstar_printk_init( "[Sstar_wtd] wakeup.\n");
	wake_up(&self->wtd->wtd_evt_wq);
#endif //CONFIG_SSTARWIFI_WDT
}
static int Sstar_wtd_process(void *arg)
{
#ifdef CONFIG_SSTARWIFI_WDT
	int status=0;
	int term=0;
	int wtd_run=0;
	int waittime = 20;
	int wtd_probe=0;
	while(1){
		status = wait_event_interruptible(g_wtd.wtd_evt_wq, ({
				term = atomic_read(&g_wtd.wtd_term);
				wtd_run = atomic_read(&g_wtd.wtd_run);
				(term || wtd_run);}));
		if (status < 0 || term ){
			Sstar_printk_exit("[Sstar_wtd]:1 thread break %d %d\n",status,term);
			goto __stop;
		}
		atomic_set(&g_wtd.wtd_run, 0);
		while(waittime-- >0){
			msleep(1);
			term = atomic_read(&g_wtd.wtd_term);
			if(term) {
				Sstar_printk_exit("[Sstar_wtd]:2 thread break %d %d\n",status,term);
				goto __stop;
			}
			wtd_probe = atomic_read(&g_wtd.wtd_probe);
			if(wtd_probe != 0){
				Sstar_printk_exit("[Sstar_wtd]:wtd_probe(%d) have done\n",wtd_probe);
				break;
			}
		}
		waittime = 10;
		//check if sdio init ok?
		wtd_probe = atomic_read(&g_wtd.wtd_probe);
		//if sdio init have probem, need call wtd again
		if(wtd_probe != 1){
			atomic_set(&g_wtd.wtd_run, 1);
			Sstar_printk_exit("[Sstar_wtd]:wtd_run again\n");
		}
	}
__stop:
	while(term){
		
		Sstar_printk_exit("[Sstar_wtd]:kthread_should_stop\n");
		if(kthread_should_stop()){
			break;
		}
		schedule_timeout_uninterruptible(msecs_to_jiffies(100));
	}
#endif //CONFIG_SSTARWIFI_WDT
	return 0;
}
static void Sstar_wtd_init(void)
{
#ifdef CONFIG_SSTARWIFI_WDT
	int err = 0;
	struct sched_param param = { .sched_priority = 1 };
	if(g_wtd.wtd_init)
		return;
	Sstar_printk_init( "[wtd] register.\n");
	init_waitqueue_head(&g_wtd.wtd_evt_wq);
	atomic_set(&g_wtd.wtd_term, 0);
	g_wtd.wtd_thread = kthread_create(&Sstar_wtd_process, &g_wtd, "Sstar_wtd");
	if (IS_ERR(g_wtd.wtd_thread)) {
		err = PTR_ERR(g_wtd.wtd_thread);
		g_wtd.wtd_thread = NULL;
	} else {
	sched_setscheduler(g_wtd.wtd_thread,
			SCHED_FIFO, &param);
#ifdef HAS_PUT_TASK_STRUCT
		get_task_struct(g_wtd.wtd_thread);
#endif
		wake_up_process(g_wtd.wtd_thread);
	}
	g_wtd.wtd_init = 1;
#endif //CONFIG_SSTARWIFI_WDT
}
static void Sstar_wtd_exit(void)
{
#ifdef CONFIG_SSTARWIFI_WDT
	struct task_struct *thread = g_wtd.wtd_thread;
	if (WARN_ON(!thread))
		return;
	if(atomic_read(&g_wtd.wtd_term)==0)
		return;
	g_wtd.wtd_thread = NULL;
	Sstar_printk_exit( "[wtd] unregister.\n");
	atomic_add(1, &g_wtd.wtd_term);
	wake_up(&g_wtd.wtd_evt_wq);
	kthread_stop(thread);
#ifdef HAS_PUT_TASK_STRUCT
	put_task_struct(thread);
#endif
	g_wtd.wtd_init = 0;
#endif //CONFIG_SSTARWIFI_WDT
}
static struct sbus_ops Sstar_usb_sbus_ops = {
	.sbus_memcpy_fromio	= Sstar_usb_memcpy_fromio,
	.sbus_memcpy_toio	= Sstar_usb_memcpy_toio,
#ifdef HW_DOWN_FW
	.sbus_read_sync		= Sstar_usb_hw_read_port,
	.sbus_write_sync	= Sstar_usb_hw_write_port,
#else //SW
	.sbus_read_sync		= Sstar_usb_sw_read_port,
	.sbus_write_sync	= Sstar_usb_sw_write_port,
#endif
	.sbus_read_async	= Sstar_usb_memcpy_fromio_async,
	.sbus_write_async	= Sstar_usb_memcpy_toio_async,
	.lock				= Sstar_usb_lock,
	.unlock				= Sstar_usb_unlock,
	.reset				= Sstar_usb_reset,
	.usb_lock_reset     = Sstar_usb_lock_reset,
	.align_size			= Sstar_usb_align_size,
	.power_mgmt			= Sstar_usb_pm,
	.set_block_size		= Sstar_usb_set_block_size,
	.wtd_wakeup			= Sstar_wtd_wakeup,
	#ifdef SSTAR_USB_RESET
	.usb_reset			= Sstar_usb_reset,
	#endif
	.bootloader_debug_config = Sstar_usb_debug_config,
	.lmac_start			=Sstar_lmac_start,
#ifdef USB_CMD_UES_EP0	
	.ep0_cmd			=Sstar_usb_ep0_cmd,
#endif
	.irq_unsubscribe	= Sstar_usb_irq_unsubscribe,
	.irq_subscribe	= Sstar_usb_irq_subscribe,
	.lmac_restart   = Sstar_lmac_restart,
#ifdef CONFIG_WSM_CMD_XMIT_DIRECTLY
	.sbus_wsm_write = Sstar_usb_wsm_send,
#endif
#ifdef CONFIG_USB_DATA_XMIT_DIRECTLY
	.sbus_data_write = Sstar_usb_data_send,
#endif
	.sbus_wait_data_xmited = Sstar_usb_wait_submitted_xmited,
	.sbus_xmit_func_init   = Sstar_usb_xmit_init,
	.sbus_xmit_func_deinit  = Sstar_usb_xmit_deinit,
	.sbus_rev_func_init    = Sstar_usb_rev_init,
	.sbus_rev_func_deinit  = Sstar_usb_rev_deinit,
	.sbus_xmit_schedule    = Sstar_usb_xmit_schedule,
	.sbus_rev_schedule     = Sstar_usb_rev_schedule,
};

static struct dvobj_priv *usb_dvobj_init(struct usb_interface *usb_intf)
{
	int	i;
	struct dvobj_priv *pdvobjpriv=NULL;
	struct usb_device_descriptor 	*pdev_desc;
	struct usb_host_config			*phost_conf;
	struct usb_config_descriptor		*pconf_desc;
	struct usb_host_interface		*phost_iface;
	struct usb_interface_descriptor	*piface_desc;
	struct usb_host_endpoint		*phost_endp;
	struct usb_endpoint_descriptor	*pendp_desc;
	struct usb_device				*pusbd;
	
	pdvobjpriv = Sstar_kzalloc(sizeof(*pdvobjpriv), GFP_KERNEL);
	if (!pdvobjpriv){
		Sstar_dbg(SSTAR_APOLLO_DBG_ERROR, "Can't allocate USB dvobj.");
		goto exit;
	}
	pdvobjpriv->pusbintf = usb_intf ;
	pusbd = pdvobjpriv->pusbdev = interface_to_usbdev(usb_intf);
	usb_set_intfdata(usb_intf, pdvobjpriv);
	#ifdef CONFIG_PM
	#ifdef SSTAR_SUSPEND_REMOVE_INTERFACE
	pusbd->reset_resume = 1;
	#endif
	#endif
	//pdvobjpriv->RtNumInPipes = 0;
	//pdvobjpriv->RtNumOutPipes = 0;
	pdev_desc = &pusbd->descriptor;
#if 1
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"\nSstar_usb_device_descriptor:\n");
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"bLength=%x\n", pdev_desc->bLength);
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"bDescriptorType=%x\n", pdev_desc->bDescriptorType);
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"bcdUSB=%x\n", pdev_desc->bcdUSB);
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"bDeviceClass=%x\n", pdev_desc->bDeviceClass);
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"bDeviceSubClass=%x\n", pdev_desc->bDeviceSubClass);
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"bDeviceProtocol=%x\n", pdev_desc->bDeviceProtocol);
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"bMaxPacketSize0=%x\n", pdev_desc->bMaxPacketSize0);
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"idVendor=%x\n", pdev_desc->idVendor);
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"idProduct=%x\n", pdev_desc->idProduct);
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"bcdDevice=%x\n", pdev_desc->bcdDevice);
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"iManufacturer=%x\n", pdev_desc->iManufacturer);
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"iProduct=%x\n", pdev_desc->iProduct);
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"iSerialNumber=%x\n", pdev_desc->iSerialNumber);
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"bNumConfigurations=%x\n", pdev_desc->bNumConfigurations);
#endif

	phost_conf = pusbd->actconfig;
	pconf_desc = &phost_conf->desc;
#if 1
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"\nSstar_usb_configuration_descriptor:\n");
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"bLength=%x\n", pconf_desc->bLength);
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"bDescriptorType=%x\n", pconf_desc->bDescriptorType);
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"wTotalLength=%x\n", pconf_desc->wTotalLength);
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"bNumInterfaces=%x\n", pconf_desc->bNumInterfaces);
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"bConfigurationValue=%x\n", pconf_desc->bConfigurationValue);
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"iConfiguration=%x\n", pconf_desc->iConfiguration);
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"bmAttributes=%x\n", pconf_desc->bmAttributes);
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"bMaxPower=%x\n", pconf_desc->bMaxPower);
#endif

	phost_iface = &usb_intf->altsetting[0];
	piface_desc = &phost_iface->desc;
#if 1
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"\nSstar_usb_interface_descriptor:\n");
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"bLength=%x\n", piface_desc->bLength);
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"bDescriptorType=%x\n", piface_desc->bDescriptorType);
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"bInterfaceNumber=%x\n", piface_desc->bInterfaceNumber);
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"bAlternateSetting=%x\n", piface_desc->bAlternateSetting);
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"bNumEndpoints=%x\n", piface_desc->bNumEndpoints);
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"bInterfaceClass=%x\n", piface_desc->bInterfaceClass);
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"bInterfaceSubClass=%x\n", piface_desc->bInterfaceSubClass);
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"bInterfaceProtocol=%x\n", piface_desc->bInterfaceProtocol);
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"iInterface=%x\n", piface_desc->iInterface);
#endif

	//pdvobjpriv->NumInterfaces = pconf_desc->bNumInterfaces;
	//pdvobjpriv->InterfaceNumber = piface_desc->bInterfaceNumber;
	pdvobjpriv->nr_endpoint = piface_desc->bNumEndpoints;


	for (i = 0; i < pdvobjpriv->nr_endpoint; i++)
	{
		phost_endp = phost_iface->endpoint + i;
		if (phost_endp)
		{
			pendp_desc = &phost_endp->desc;

			Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"\nusb_endpoint_descriptor(%d):\n", i);
			Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"bLength=%x\n",pendp_desc->bLength);
			Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"bDescriptorType=%x\n",pendp_desc->bDescriptorType);
			Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"bEndpointAddress=%x\n",pendp_desc->bEndpointAddress);
			Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"wMaxPacketSize=%d\n",le16_to_cpu(pendp_desc->wMaxPacketSize));
			Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"bInterval=%x\n",pendp_desc->bInterval);

			if (usb_endpoint_is_bulk_in(pendp_desc))
			{
				Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"usb_endpoint_is_bulk_in = %x\n",usb_endpoint_num(pendp_desc));
				pdvobjpriv->ep_in_size=le16_to_cpu(pendp_desc->wMaxPacketSize);
				pdvobjpriv->ep_in=usb_endpoint_num(pendp_desc);
			}
			else if (usb_endpoint_is_bulk_out(pendp_desc))
			{
				Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"usb_endpoint_is_bulk_out = %x\n",usb_endpoint_num(pendp_desc));
				pdvobjpriv->ep_out_size=le16_to_cpu(pendp_desc->wMaxPacketSize);
				pdvobjpriv->ep_out=usb_endpoint_num(pendp_desc);
			}
			pdvobjpriv->ep_num[i] = usb_endpoint_num(pendp_desc);
		}
	}
	usb_get_dev(pusbd);
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"nr_endpoint=%d, in_num=%d, out_num=%d\n\n", pdvobjpriv->nr_endpoint, pdvobjpriv->ep_in, pdvobjpriv->ep_out);
exit:
	return pdvobjpriv;
}
static int g_rebootSystem = 0;
/*
*you can modify this function,but NOTIFY THAT:
*in this funciton you can not SLEEP!!!
*/
static void Sstar_usb_sta_deauthen_iterate_atomic_cb(void *data, u8 *mac,
			 struct ieee80211_vif *vif)
{
	struct Sstar_common *hw_priv = (struct Sstar_common *)data;
	struct Sstar_vif *priv = ABwifi_get_vif_from_ieee80211(vif);

	if(hw_priv != priv->hw_priv){
		Sstar_printk_err("%s:hw_priv != priv->hw_priv\n",__func__);
		return;
	}

	if(priv->join_status != SSTAR_APOLLO_JOIN_STATUS_STA){
		Sstar_printk_err("%s:priv not sta mode\n",__func__);
		return;
	}

	Sstar_printk_exit( "[%pM] force deauthen\n",mac);
	ieee80211_connection_loss(vif);
}
static void Sstar_process_system_action(struct Sstar_common *hw_priv,
														enum Sstar_system_action action)
{
	struct ieee80211_local *local = hw_to_local(hw_priv->hw);

	switch(action){
		
		case SSTAR_SYSTEM_REBOOT:
		case SSTAR_SYSTEM_RMMOD:
			atomic_set(&local->resume_timer_start,1);
			mod_timer(&local->resume_timer, round_jiffies(jiffies + 5*HZ));
			break;
		case SSTAR_SYSTEM_NORMAL:
		case SSTAR_SYSTEM_MAX:
		default:
			break;
	}

	
}
static void Sstar_usb_station_diconnect_sync(struct Sstar_common *hw_priv,
														 enum Sstar_system_action action)
{	
	struct ieee80211_hw *hw = NULL;
	struct ieee80211_local *local = NULL;
	struct ieee80211_sub_if_data *sdata;
	
	if(hw_priv == NULL){
		Sstar_printk_err( "%s:hw_priv == NULL\n",__func__);
		goto sync_end;
	}
	hw = hw_priv->hw;

	if(hw_priv != hw->priv){
		Sstar_printk_err("%s:hw_priv != hw->priv\n",__func__);
		goto sync_end;
	}
	/*
	*according to system action,do some job:
	*reboot or rmmod:disable supplicant authen
	*/
	Sstar_process_system_action(hw_priv,action);
	/*
	*send deauthen to all connected ap
	*/
	local = hw_to_local(hw);
	ieee80211_iterate_active_interfaces_atomic(hw,Sstar_usb_sta_deauthen_iterate_atomic_cb,
											   (void*)hw_priv);
	/*
	*make sure beacon_connection_loss_work has been processed
	*/
	flush_workqueue(local->workqueue);
	/*
	*make sure deauthen has been send;
	*/
	
	mutex_lock(&local->mtx);
	mutex_lock(&local->iflist_mtx);
	list_for_each_entry(sdata, &local->interfaces, list)
		drv_flush(local, sdata, false);
	mutex_unlock(&local->iflist_mtx);
	mutex_unlock(&local->mtx);
	/*
	*make sure unjoin work complete
	*/
	flush_workqueue(hw_priv->workqueue);	
sync_end:
	return;
}
static void Sstar_usb_reboot_sync(void)
{
	Sstar_usb_module_lock();
	Sstar_usb_station_diconnect_sync(Sstar_hw_priv_dereference(),SSTAR_SYSTEM_REBOOT);	
	Sstar_usb_fw_sync(Sstar_hw_priv_dereference(),Sstar_usb_dvobj_dereference());
	Sstar_usb_module_unlock();
}
static int Sstar_reboot_notifier(struct notifier_block *nb,
				unsigned long action, void *unused)
{
	Sstar_printk_exit("Sstar_reboot_notifier(%ld)\n",action);
	g_rebootSystem =0;
	Sstar_usb_reboot_sync();
	if(action == SYS_POWER_OFF){
		return NOTIFY_DONE;
	}
	atomic_set(&g_wtd.wtd_term, 1);
	atomic_set(&g_wtd.wtd_run, 0);
	Sstar_usb_exit();
	return NOTIFY_DONE;
}

/* Probe Function to be called by USB stack when device is discovered */
static struct notifier_block Sstar_reboot_nb = {
	.notifier_call = Sstar_reboot_notifier,
	.priority=1,
};

#if defined(CONFIG_HAS_EARLYSUSPEND)
#ifdef SSTAR_PM_USE_EARLYSUSPEND
static void Sstar_early_suspend(struct early_suspend *h)
{
	struct sbus_priv *self = container_of(h, struct sbus_priv, Sstar_early_suspend);
	
	if (unlikely(!down_trylock(&self->early_suspend_lock))) {
		Sstar_printk_bus("zezer:early suspend--------------------\n");
		atomic_set(&self->early_suspend_state, 1);	
		wiphy_rfkill_set_hw_state(self->core->hw->wiphy,true);
		up(&self->early_suspend_lock);
	}
}
static void Sstar_late_resume(struct early_suspend *h)
{
	struct sbus_priv *self = container_of(h, struct sbus_priv, Sstar_early_suspend);
	down(&self->early_suspend_lock);
	if(atomic_read(&self->early_suspend_state)){
		atomic_set(&self->early_suspend_state, 0);
		wiphy_rfkill_set_hw_state(self->core->hw->wiphy,false);
	}
	up(&self->early_suspend_lock);
}

static void Sstar_enable_early_suspend(struct sbus_priv *self)
{
	sema_init(&self->early_suspend_lock, 1);
	
	down(&self->early_suspend_lock);
	atomic_set(&self->early_suspend_state, 0);	
	self->Sstar_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 100;
	self->Sstar_early_suspend.suspend = Sstar_early_suspend;
	self->Sstar_early_suspend.resume = Sstar_late_resume;
	register_early_suspend(&self->Sstar_early_suspend);
	up(&self->early_suspend_lock);
}

static void Sstar_disable_early_suspend(struct sbus_priv *self)
{
	unregister_early_suspend(&self->Sstar_early_suspend);
}
#endif
#endif

static int __Sstar_usb_probe(struct usb_interface *intf,
				   const struct usb_device_id *id)
{
	struct sbus_priv *self;
	struct dvobj_priv *dvobj;
	int status;
	
	if(Sstar_wifi_get_status()){
		Sstar_dbg(SSTAR_APOLLO_DBG_ERROR, "[Sstar]Can't probe USB. when wifi_module_exit");
		wifi_usb_probe_doing= 0;
		return -ETIMEDOUT;
	}		
	wifi_usb_probe_doing= 1;
	
	Sstar_dbg(SSTAR_APOLLO_DBG_INIT, "Probe called %d v1\n",intf->minor);
	atomic_set(&g_wtd.wtd_probe, 0);

	self = Sstar_kzalloc(sizeof(*self), GFP_KERNEL);
	if (!self) {
		Sstar_dbg(SSTAR_APOLLO_DBG_ERROR, "Can't allocate USB sbus_priv.");
		wifi_usb_probe_doing= 0;
		return -ENOMEM;
	}
	mutex_init(&self->sbus_mutex);

	spin_lock_init(&self->lock);
	self->pdata = Sstar_get_platform_data();
	/* 1--- Initialize dvobj_priv */
	dvobj = usb_dvobj_init(intf);
	if (!dvobj){
		Sstar_dbg(SSTAR_APOLLO_DBG_ERROR, "Can't allocate USB dvobj.");
		Sstar_usb_dvobj_assign_pointer(NULL);
		wifi_usb_probe_doing= 0;
		return -ENOMEM;
	}
	dvobj->self =self;
	self->drvobj=dvobj;
	/*2---alloc rx_urb*/
	dvobj->suspend_skb = __Sstar_dev_alloc_skb(RX_BUFFER_SIZE+64,GFP_KERNEL);//dev_alloc_skb(RX_BUFFER_SIZE+64);
	BUG_ON(dvobj->suspend_skb == NULL);
	Sstar_skb_reserve(self->drvobj->suspend_skb, 64);
	dvobj->suspend_skb_len = 0;
#ifdef CONFIG_USE_DMA_ADDR_BUFFER	
	Sstar_printk_init("CONFIG_USE_DMA_ADDR_BUFFER TX_BUFFER_SIZE %x\n",TX_BUFFER_SIZE);
	dvobj->tx_save_urb=NULL;
#else
	Sstar_printk_init("not CONFIG_USE_DMA_ADDR_BUFFER\n");
#endif //CONFIG_USE_DMA_ADDR_BUFFER
	status = Sstar_usb_urb_malloc(self,dvobj->rx_urb,RX_URB_NUM,RX_BUFFER_SIZE, 1,0);
	if (status != 0){
		Sstar_dbg(SSTAR_APOLLO_DBG_ERROR, "Can't allocate rx_urb.");
		wifi_usb_probe_doing= 0;
		return status;
	}
	memset(dvobj->rx_urb_map,0,sizeof(dvobj->rx_urb_map));
	/*3---alloc tx_urb*/
	status = Sstar_usb_urb_malloc(self,dvobj->tx_urb,TX_URB_NUM,TX_BUFFER_SIZE, 0,1);

	if (!dvobj->tx_urb){
		Sstar_dbg(SSTAR_APOLLO_DBG_ERROR, "Can't allocate tx_urb.");
		wifi_usb_probe_doing= 0;
		return -ENOMEM;
	}	
	memset(dvobj->tx_urb_map,0,sizeof(dvobj->tx_urb_map));
	memset(dvobj->tx_err_urb_map,0,sizeof(dvobj->tx_err_urb_map));
#ifdef CONFIG_TX_NO_CONFIRM
	Sstar_printk_init("CONFIG_TX_NO_CONFIRM\n");
	memset(dvobj->txpending_urb_map,0,sizeof(dvobj->txpending_urb_map));
#else
	Sstar_printk_init("not CONFIG_TX_NO_CONFIRM\n");
#endif //#ifdef CONFIG_TX_NO_CONFIRM

#ifdef CONFIG_WSM_CMD_XMIT_DIRECTLY
	status = Sstar_usb_wsm_urb_int(self);
	if(status){
		wifi_usb_probe_doing= 0;
		return -ENOMEM;
	}
#endif
	/*4---alloc cmd_urb*/
	dvobj->cmd_urb=usb_alloc_urb(0,GFP_KERNEL);
	if (!dvobj->cmd_urb){
		Sstar_dbg(SSTAR_APOLLO_DBG_ERROR, "Can't allocate cmd_urb.");
		wifi_usb_probe_doing= 0;
		return -ENOMEM;
	}
	/*5---alloc rx data buffer*/
	self->usb_data = Sstar_kzalloc(SSTAR_USB_EP0_MAX_SIZE+16,GFP_KERNEL);
	if (!self->usb_data){
		wifi_usb_probe_doing= 0;
		return -ENOMEM;
	}
	//self->rx_skb = NULL;
//#define ALIGN(a,b)  (((a)+((b)-1))&(~((b)-1)))
	self->usb_req_data = (u8 *)ALIGN((unsigned long)self->usb_data,16);

	init_usb_anchor(&dvobj->tx_submitted);
	init_usb_anchor(&dvobj->rx_submitted);
#ifdef CONDIF_SSTAR_CTRL_REQUEST_ASYNC
	init_usb_anchor(&dvobj->ctrl_submitted);
#endif
	atomic_xchg(&self->tx_lock, 0);
	atomic_xchg(&self->rx_lock, 0);
	self->tx_hwChanId = 0;
	self->rx_seqnum = 0;
	self->drvobj->tx_test_seq_need =0;
	self->drvobj->tx_test_hwChanId_need =0;
/*
	Sstar_usb_hw_read_port(self,0XB000540,&self->tx_hwChanId,4);
	self->tx_hwChanId++;
	self->tx_hwChanId &=  0x1F;
	self->drvobj->tx_test_hwChanId_need = self->tx_hwChanId;
	*/
	Sstar_printk_init("self->tx_hwChanId %d\n",self->tx_hwChanId);
	
	//self->tx_callback_handler = NULL;
	//self->rx_callback_handler = NULL;
	wifi_tx_urb_pending = 0;

	self->wtd = &g_wtd;
	//
	//usb auto-suspend init
	self->suspend=0;	
	self->auto_suspend=0;	

#ifdef CONFIG_PM
#if USB_AUTO_WAKEUP

#if (LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,35))
	usb_enable_autosuspend(self->drvobj->pusbdev);
#else
	self->drvobj->pusbdev->autosuspend_disabled = 0;//autosuspend disabled by the user
#endif
#ifdef CONFIG_PM
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
	{
		dvobj->pusbdev->do_remote_wakeup=1;
		dvobj->pusbintf->needs_remote_wakeup = 1;
		device_init_wakeup(&dvobj->pusbintf->dev,1);
		pm_runtime_set_autosuspend_delay(&dvobj->pusbdev->dev,15000);
	}
#endif //(LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
#endif //CONFIG_PM

#else //USB_AUTO_WAKEUP
#if (LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,35))
	usb_disable_autosuspend(self->drvobj->pusbdev);
#else //(LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,35))
	self->drvobj->pusbdev->autosuspend_disabled = 1;//autosuspend disabled by the user
#endif //(LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,35))
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18))
	dvobj->pusbdev->do_remote_wakeup=0;
	dvobj->pusbintf->needs_remote_wakeup = 0;
	device_init_wakeup(&dvobj->pusbintf->dev,1);
#endif //(LINUX_VERSION_CODE>=KERNEL_VERSION(2,6,35))

#endif //#if USB_AUTO_WAKEUP
#endif //CONFIG_PM


	//initial wifi mac
	status = Sstar_core_probe(&Sstar_usb_sbus_ops,
			      self, &intf->dev, &self->core);

	if (status) {
		Sstar_printk_err("<ERROR> %s %d reset usb\n",__func__,__LINE__);
		usb_free_urb(dvobj->cmd_urb);
		Sstar_usb_urb_free(self,self->drvobj->rx_urb,RX_URB_NUM);
		Sstar_usb_urb_free(self,self->drvobj->tx_urb,TX_URB_NUM);
#ifdef CONFIG_WSM_CMD_XMIT_DIRECTLY
		Sstar_usb_wsm_urb_deint(self);
#endif
#ifdef CONDIF_SSTAR_CTRL_REQUEST_ASYNC
		usb_kill_anchored_urbs(&dvobj->ctrl_submitted);
#endif
		if(self->drvobj->suspend_skb)
		{
			Sstar_dev_kfree_skb(self->drvobj->suspend_skb);
			self->drvobj->suspend_skb_len = 0;
			self->drvobj->suspend_skb = NULL;
		}
		if(dvobj->pusbdev)
			usb_put_dev(dvobj->pusbdev);
		usb_set_intfdata(intf, NULL);
		Sstar_kfree(self->usb_data);
		Sstar_kfree(dvobj);
		Sstar_kfree(self);
		atomic_set(&g_wtd.wtd_probe, -1);
		Sstar_usb_dvobj_assign_pointer(NULL);
		Sstar_hw_priv_assign_pointer(NULL);
		//reset usb
		usb_reset_device(interface_to_usbdev(intf));
		#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 6, 0))
		status = -EPROBE_DEFER;
		#endif
	}
	else {
		atomic_set(&g_wtd.wtd_probe, 1);
		Sstar_usb_dvobj_assign_pointer(dvobj);
		#if defined(CONFIG_HAS_EARLYSUSPEND)
		#ifdef SSTAR_PM_USE_EARLYSUSPEND
		Sstar_enable_early_suspend(self);
		#endif
		#endif
		Sstar_printk_err("[Sstar_wtd]:set wtd_probe = 1\n");
	}
	wifi_usb_probe_doing=0;
	return status;
}

static int Sstar_usb_probe(struct usb_interface *intf,
				   const struct usb_device_id *id)
{
	int ret = 0;
	
	Sstar_usb_module_lock();
	ret = __Sstar_usb_probe(intf,id);
	Sstar_usb_module_unlock();

	return ret;
}
/* Disconnect Function to be called by USB stack when
 * device is disconnected */
static void __Sstar_usb_disconnect(struct usb_interface *intf)
{
	struct dvobj_priv *dvobj = usb_get_intfdata(intf);
	struct sbus_priv *self  = NULL;
	struct usb_device *pdev=NULL;
#ifdef ANDROID
	mdelay(2000);
#endif
	if (dvobj) {
		self  = dvobj->self;
		pdev = dvobj->pusbdev;
		if (self->core) {
			Sstar_printk_exit(" %s %d core %p\n",__func__,__LINE__,self->core);
			Sstar_core_release(self->core);
			self->core = NULL;
		}
		#if defined(CONFIG_HAS_EARLYSUSPEND)
		#ifdef SSTAR_PM_USE_EARLYSUSPEND
		Sstar_disable_early_suspend(self);
		#endif
		#endif
#ifdef ANDROID
		mdelay(2000);
#endif
#if (EXIT_MODULE_RESET_USB==0)
		if(g_rebootSystem)
#endif /*EXIT_MODULE_RESET_USB*/
		if (interface_to_usbdev(intf)->state != USB_STATE_NOTATTACHED) 
		{
			Sstar_printk_exit("usb_reset_device:---------------------------------------------------\n");
			usb_reset_device(interface_to_usbdev(intf));
		}	
		self->suspend = 1;
		Sstar_usb_urb_free(self,self->drvobj->rx_urb,RX_URB_NUM);
		Sstar_usb_urb_free(self,self->drvobj->tx_urb,TX_URB_NUM);
#ifdef CONFIG_WSM_CMD_XMIT_DIRECTLY
		Sstar_usb_wsm_urb_deint(self);
#endif
#ifdef CONDIF_SSTAR_CTRL_REQUEST_ASYNC
		usb_kill_anchored_urbs(&dvobj->ctrl_submitted);
#endif
		if(self->drvobj->suspend_skb)
		{
			Sstar_dev_kfree_skb(self->drvobj->suspend_skb);
			self->drvobj->suspend_skb = NULL;
			self->drvobj->suspend_skb_len = 0;
		}
		usb_kill_urb(self->drvobj->cmd_urb);
		usb_free_urb(self->drvobj->cmd_urb);
		usb_set_intfdata(intf, NULL);
		mutex_destroy(&self->sbus_mutex);
		Sstar_kfree(self->usb_data);
		Sstar_kfree(self);
		Sstar_kfree(dvobj);
		Sstar_usb_dvobj_assign_pointer(NULL);
		Sstar_hw_priv_assign_pointer(NULL);
		if(pdev)
		{
			BUG_ON((pdev!=interface_to_usbdev(intf)));
			Sstar_printk_exit("we have get dev,so put it in the end\n");
			usb_put_dev(pdev);
		}
		Sstar_printk_exit("Sstar_usb_disconnect---->oK\n");
	}
}
static void Sstar_usb_disconnect_flush(struct Sstar_common *hw_priv)
{
	struct sbus_priv *self  = hw_priv->sbus_priv;
	/*
	*flush txrx queue
	*/
	synchronize_net();
	
	#ifndef USB_USE_TASTLET_TXRX
	{
		flush_workqueue(self->tx_workqueue);
		flush_workqueue(self->rx_workqueue);		
	}
	#else
	{
		tasklet_disable(&self->tx_cmp_tasklet);
		tasklet_disable(&self->rx_cmp_tasklet);
		Sstar_tx_tasklet((unsigned long)hw_priv);
		Sstar_rx_tasklet((unsigned long)hw_priv);
		tasklet_enable(&self->rx_cmp_tasklet);
		tasklet_enable(&self->tx_cmp_tasklet);
	}
	#endif
	/*
	*cancle cmd
	*/
	spin_lock_bh(&hw_priv->tx_com_lock);
	spin_lock_bh(&hw_priv->wsm_cmd.lock);
	if(hw_priv->wsm_cmd.cmd != 0xFFFF){
		hw_priv->wsm_cmd.ret = -1;
		hw_priv->wsm_cmd.done = 1;
		hw_priv->wsm_cmd.cmd = 0xFFFF;
		if(hw_priv->wsm_cmd.ptr){
			hw_priv->wsm_cmd.ptr = NULL;
			Sstar_printk_err("%s:cancle new cmd\n",__func__);
		}else {
			Sstar_printk_err("%s:cancle old cmd\n",__func__);
		}
		wake_up(&hw_priv->wsm_cmd_wq);
	}
	spin_unlock_bh(&hw_priv->wsm_cmd.lock);
	spin_unlock_bh(&hw_priv->tx_com_lock);
}
static void Sstar_usb_prepare_disconnect(struct Sstar_common *hw_priv)
{
	int i = 0;
	
	if(hw_priv == NULL)
		return;

	Sstar_printk_err("%s\n",__func__);
	/*
	*before clear queue ,lock tx
	*/
	wsm_lock_tx_async(hw_priv);
	/*
	*flush out all packets
	*/
	Sstar_usb_disconnect_flush(hw_priv);
	/*
	*flush all pkg in the hmac,because we can not receive pkg confirm
	*/
	for (i = 0; i < 4; i++)
		Sstar_queue_clear(&hw_priv->tx_queue[i], SSTAR_WIFI_ALL_IFS);
	/*
	*Sstar_wait_scan_complete_sync must be called before wsm_wait_pm_sync.
	*wait scan complete,maybe wait scan cmd confirm,or scan complicaton
	*/

	//end the Sstar_iw test and release the lock when the USB device is removed
	if(hw_priv->etf_test_v2)
	{
		hw_priv->bStartTx = 0;
		hw_priv->bStartTxWantCancel  = 0;
		hw_priv->etf_test_v2 = 0;
		Sstar_printk_always("[%s]:usb device removed,stop etf test @_@ @_@\n",__func__);
		//call etf_v2_scan_end function when the USB device is removed
		Sstar_hw_priv_queue_work(hw_priv, &hw_priv->etf_tx_end_work);
		//release the lock in wsm_scan function when the USB device is removed
		wsm_oper_unlock(hw_priv);
	}
	
	Sstar_wait_scan_complete_sync(hw_priv);
	/*
	*wait pm complete.
	*/
	wsm_wait_pm_sync(hw_priv);
	/*
	*unlock tx
	*/
	wsm_unlock_tx(hw_priv);

}
static void Sstar_usb_disconnect(struct usb_interface *intf)
{
	struct dvobj_priv *dvobj = usb_get_intfdata(intf);
	
	if(dvobj){
		/*
		*prepare for disconnect: 
		*1.lock tx , flush and clear all queue
		*2.clear pending cmd
		*3.wait scan complete
		*4.wait pm unlock
		*/
		Sstar_usb_prepare_disconnect(dvobj->self->core);
	}
	/*
	*call Sstar_usb_module_lock to protect other process
	*from getting the hw_priv of NULL;
	*/
	Sstar_usb_module_lock();
	__Sstar_usb_disconnect(intf);	
	Sstar_usb_module_unlock();

	//debug fs, set error flag.
	Sstar_wifi_run_status_set(1);
}

int __Sstar_usb_suspend(struct sbus_priv *self)
{
	Sstar_printk_pm("***********func=%s,line=%d\n",__func__,__LINE__);
	self->suspend=1;
	return 0;

}

int __Sstar_usb_resume(struct sbus_priv *self)
{
	Sstar_printk_pm("===----===start resume state\n");
	self->suspend=0;
	Sstar_usb_memcpy_fromio(self,0,NULL,RX_BUFFER_SIZE);
	return 0;
	
}
#ifdef CONFIG_PM
#ifdef SSTAR_SUSPEND_REMOVE_INTERFACE
static int __Sstar_usb_hw_suspend(struct sbus_priv *self)
{
	int ret = 0;
	struct Sstar_common *hw_priv = self->core;
	struct Sstar_ep0_cmd{
			u32 cmd_id;
			u32 lmac_seq;
			u32 hmac_seq;
			u32 data[32];
	};
	u32 buf[DOWNLOAD_BLOCK_SIZE/4];
	int tx_size=12;
	struct Sstar_ep0_cmd *cmd = (struct Sstar_ep0_cmd *)buf;
	u32 addr = hw_priv->wsm_caps.HiHwCnfBufaddr;
	
	cmd->cmd_id=0x17690125;
	cmd->lmac_seq=11;
	cmd->hmac_seq=12;

	ret = Sstar_ep0_write(hw_priv,addr,buf, tx_size);
	if (ret < 0) {
		Sstar_printk_pm("%s:err\n",__func__);
		goto error;
	}
	ret = Sstar_usb_ep0_cmd(hw_priv->sbus_priv);
	if(ret < 0){
		Sstar_printk_pm("%s:err\n",__func__);
		goto error;
	}

	ret = Sstar_direct_write_reg_32(hw_priv,0x0b000548,1);

	if(ret < 0){
		Sstar_printk_pm("%s:0x0b000548 err\n",__func__);
		goto error;
	}
	ret  = 0;
error:
	return ret;
}
static int __Sstar_usb_suspend_comm(struct usb_interface *intf)
{
	struct dvobj_priv *dvobj = usb_get_intfdata(intf);
	struct sbus_priv *self =  dvobj->self;
	struct Sstar_common *hw_priv = self->core;
	struct ieee80211_local *local = hw_to_local(hw_priv->hw);
	int ret = 0;
	
	Sstar_usb_module_lock_check();

	if(local->wowlan == true){
		return 0;
	}
	wsm_lock_tx_async(hw_priv);
	Sstar_wifi_set_status(2);
	/*
	*from now ,rx_urb and tx_urb can not be submitted again until
	*urb unblock.
	*unblock urbs after resume 
	*/
	Sstar_usb_block_urbs(self);
	synchronize_rcu();
	ret = Sstar_usb_wait_anchor_empty_timeout(self,1000);
	if(ret == 0){
		/*
		*try again
		*/
		ret = Sstar_usb_wait_anchor_empty_timeout(self,10000);
		if(ret == 0){
			ret = -EBUSY;
			Sstar_printk_pm("%s:wait urb timeout\n",__func__);
			goto urbs_sync_err;
		}
	}	
	Sstar_rx_bh_flush(hw_priv);	
	Sstar_usb_release_tx_err_urb(self,self->drvobj->tx_urb_map,1);

	ret = __Sstar_usb_hw_suspend(self);

	if(ret != 0){
		Sstar_printk_pm("%s:fw prepare suspend err\n",__func__);
		goto urbs_sync_err;
	}
	synchronize_rcu();
	/*
	*waitting all wsm cmd destory
	*/
	Sstar_destroy_wsm_cmd(hw_priv);
	highlight_debug("[Sstar usb wifi]:suspend done");
	return ret;
urbs_sync_err:
	highlight_debug("[Sstar usb wifi]:suspend fail,watting retry");
	Sstar_usb_unblock_urbs(self);
	Sstar_wifi_set_status(0);
	wsm_unlock_tx(hw_priv);
	Sstar_usb_memcpy_fromio(dvobj->self,0,NULL,RX_BUFFER_SIZE);
	return ret;
}
static int __Sstar_usb_resume_comm(struct usb_interface *intf)
{
	struct dvobj_priv *dvobj = usb_get_intfdata(intf);
	struct sbus_priv *self =  dvobj->self;
	struct Sstar_common *hw_priv = self->core;
	struct ieee80211_local *local = hw_to_local(hw_priv->hw);
	int ret = -1;
	int i = 0;
	
	Sstar_usb_module_lock_check();
	/*
	*hold rtnl_lock,make sure that when down load fw,network layer cant not 
	*send pkg and cmd
	*/
	rtnl_lock();
	/*
	*suspend fail ,can not to do resume.waiting disconnect...
	*/
	if(dvobj->self->suspend==0){
		ret = -1;
		goto wow_resume;
	}
	dvobj->self->suspend = 0;
	smp_mb();
	if(local->wowlan == true){
		Sstar_printk_pm("wowlan support\n");
		Sstar_usb_memcpy_fromio(dvobj->self,0,NULL,RX_BUFFER_SIZE);
		ret = 0;
		goto wow_resume;
	}
	/*
	*release hw buff
	*/
	hw_priv->wsm_tx_seq = 0;
	hw_priv->buf_id_tx = 0;
	hw_priv->wsm_rx_seq = 0;
	hw_priv->hw_bufs_used = 0;
	hw_priv->save_buf = NULL;
	hw_priv->save_buf_len = 0;
	hw_priv->save_buf_vif_selected = -1;
	hw_priv->buf_id_rx = 0;
	for (i = 0; i < SSTAR_WIFI_MAX_VIFS; i++)
		hw_priv->hw_bufs_used_vif[i] = 0;
	
	atomic_set(&hw_priv->Sstar_pluged,1);
	/*
	*block usbs at suspend.
	*here unblock urbs for later using
	*/
	Sstar_usb_unblock_urbs(self);
	Sstar_wifi_set_status(0);
	/*
	*reload firmware and reinit lmac
	*/
	ret = Sstar_reinit_firmware(hw_priv);
	
	wsm_unlock_tx(hw_priv);
wow_resume:
	if(ret != 0){
		/*
		*usb resume fail,so disable tx task
		*/
		atomic_set(&hw_priv->Sstar_pluged,0);
		/*
		*notify usb kernel we need to reprobe.
		*/
		intf->needs_binding = 1;
		highlight_debug("[Sstar usb wifi] resume fail,waitting disconnect.....");
	}else {
		highlight_debug("[Sstar usb wifi] resume done");
	}
	rtnl_unlock();	
	return ret;
}
static int Sstar_usb_reset_resume(struct usb_interface *intf)
{
	int ret = 0;
	highlight_debug("[Sstar usb wifi]:try to reset resume");
	Sstar_usb_module_lock();
	if(Sstar_hw_priv_dereference()){
		ret = __Sstar_usb_resume_comm(intf);
	}
	Sstar_usb_module_unlock();
	return ret;
}
#endif
static int Sstar_usb_suspend(struct usb_interface *intf,pm_message_t message)
{
	#ifndef SSTAR_SUSPEND_REMOVE_INTERFACE
	struct dvobj_priv *dvobj = usb_get_intfdata(intf);
	Sstar_printk_pm("***********func=%s,line=%d\n",__func__,__LINE__);
	dvobj->self->suspend=1;
	//Sstar_usb_suspend_start(dvobj->self);
	//msleep(20);
	//Sstar_usb_urb_kill(dvobj->self,dvobj->rx_urb,RX_URB_NUM);
	return 0;
	#else
	struct dvobj_priv *dvobj = usb_get_intfdata(intf);
	int ret = 0;
	highlight_debug("[Sstar usb wifi]:try to suspend");
	Sstar_usb_module_lock();
	if(Sstar_hw_priv_dereference()){
		ret = __Sstar_usb_suspend_comm(intf);
	}	
	dvobj->self->suspend = (ret==0);
	Sstar_usb_module_unlock();
	return ret;
	#endif

}
static int Sstar_usb_resume(struct usb_interface *intf)
{
	#ifndef SSTAR_SUSPEND_REMOVE_INTERFACE
	struct dvobj_priv *dvobj = usb_get_intfdata(intf);
	Sstar_printk_pm("===----===start resume state\n");
	dvobj->self->suspend=0;
	Sstar_usb_memcpy_fromio(dvobj->self,0,NULL,RX_BUFFER_SIZE);
	return 0;
	#else
	int ret = 0;
	highlight_debug("[Sstar usb wifi]:try to resume");
	Sstar_usb_module_lock();
	if(Sstar_hw_priv_dereference()){
		ret = __Sstar_usb_resume_comm(intf);
	}
	Sstar_usb_module_unlock();
	return ret;
	#endif
	
}
#endif

#if (PROJ_TYPE<ARES_B)
static int Sstar_usb_pre_reset(struct usb_interface *intf)
{
	struct dvobj_priv *dvobj = usb_get_intfdata(intf);
	struct Sstar_common *hw_priv = NULL;
	int ret = 0;//(atomic_read(&hw_priv->bh_halt) == 0);
	/*
	*if bh_halt ,we try to reset firmware,other reset chip
	*/
	if(dvobj == NULL)
		return 1;
	if(dvobj->self == NULL)
		return 1;
	if(dvobj->self->core == NULL)
		return 1;
	hw_priv = dvobj->self->core;
	ret = (atomic_read(&hw_priv->bh_halt) == 0);
	if(atomic_read(&hw_priv->bh_halt)){
		struct Sstar_ep0_cmd{
			u32 cmd_id;
			u32 lmac_seq;
			u32 hmac_seq;
			u32 data[32];
		};
		u32 buf[DOWNLOAD_BLOCK_SIZE/4];
		int tx_size=12;
		struct Sstar_ep0_cmd *cmd = (struct Sstar_ep0_cmd *)buf;
		u32 addr = hw_priv->wsm_caps.HiHwCnfBufaddr;
		
		cmd->cmd_id=0x17690124;
		cmd->lmac_seq=11;
		cmd->hmac_seq=12;

		ret = Sstar_ep0_write(hw_priv,addr,buf, tx_size);
		if (ret < 0) {
			Sstar_printk_init("%s:err\n",__func__);
			goto error;
		}
		ret = Sstar_usb_ep0_cmd(hw_priv->sbus_priv);
		if(ret < 0){
			Sstar_printk_err("%s:err\n",__func__);
			goto error;
		}

		ret = Sstar_direct_write_reg_32(hw_priv,0x0b000548,1);

		if(ret < 0){
			Sstar_printk_init("%s:0x0b000548 err\n",__func__);
			goto error;
		}
		ret  = 0;
		mdelay(100);
	}
	Sstar_printk_init("%s:%d\n",__func__,interface_to_usbdev(intf)->state);
error:
	return ret;
}
static int Sstar_usb_post_reset(struct usb_interface *intf)
{
	struct dvobj_priv *dvobj = usb_get_intfdata(intf);
	struct Sstar_common *hw_priv = NULL;
	Sstar_printk_init("%s:%d\n",__func__,interface_to_usbdev(intf)->state);
	if(dvobj == NULL)
		return 1;
	if(dvobj->self == NULL)
		return 1;
	if(dvobj->self->core == NULL)
		return 1;
	hw_priv = dvobj->self->core;
	return atomic_read(&hw_priv->bh_halt) == 0;
}
#else
static int Sstar_usb_pre_reset(struct usb_interface *intf)
{
	struct dvobj_priv *dvobj = usb_get_intfdata(intf);
	if(dvobj == NULL)
		return 1;
	if(dvobj->self == NULL)
		return 1;
	if(dvobj->self->core == NULL)
		return 1;
	Sstar_printk_init("%s:%d\n",__func__,interface_to_usbdev(intf)->state);
	return 0;
}
static int Sstar_usb_post_reset(struct usb_interface *intf)
{
	struct dvobj_priv *dvobj = usb_get_intfdata(intf);
	Sstar_printk_init("%s:%d\n",__func__,interface_to_usbdev(intf)->state);
	if(dvobj == NULL)
		return 1;
	if(dvobj->self == NULL)
		return 1;
	if(dvobj->self->core == NULL)
		return 1;
	return 0;
}
#endif

static struct usb_driver apollod_driver = {
	.name		= "Sstar_wlan",
	.id_table	= Sstar_usb_ids,
	.probe		= Sstar_usb_probe,
	.disconnect	= Sstar_usb_disconnect,
#ifdef CONFIG_PM
	.suspend	= Sstar_usb_suspend,
	.resume		= Sstar_usb_resume,
#ifdef SSTAR_SUSPEND_REMOVE_INTERFACE
	.reset_resume = Sstar_usb_reset_resume,
#endif
#endif
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0))
	.disable_hub_initiated_lpm = 1,
#endif  //(LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0))
#if USB_AUTO_WAKEUP
	.supports_autosuspend = 1,
#else //USB_AUTO_WAKEUP
	.supports_autosuspend = 0,
	
#endif  //USB_AUTO_WAKEUP
	.pre_reset = Sstar_usb_pre_reset,
	.post_reset = Sstar_usb_post_reset,
};

static int Sstar_usb_register(void)
{
	struct Sstar_usb_driver_ref *driver_ref = &Sstar_usb_kref;
	int ret = 0;
	
	kref_init(&driver_ref->ref);
	
	ret = usb_register(&apollod_driver);

	driver_ref->Sstar_driver = ret?NULL:&apollod_driver;

	return ret;
}
static void Sstar_usb_driver_release(struct kref *kref)
{
	struct Sstar_usb_driver_ref *driver_ref = container_of(kref, struct Sstar_usb_driver_ref, ref);

	if(driver_ref->Sstar_driver== NULL){
		WARN_ON(1);
		return;
	}
	
	Sstar_printk_exit("%s\n",__func__);
	usb_deregister(driver_ref->Sstar_driver);
	
	driver_ref->Sstar_driver = NULL;
}
/*
*return 1 ,driver removed success
*return 0 ,others
*/
static int Sstar_usb_deregister(void)
{
	struct Sstar_usb_driver_ref *driver_ref = &Sstar_usb_kref;	
	return kref_put(&driver_ref->ref, Sstar_usb_driver_release);
}
/* Init Module function -> Called by insmod */
static int  Sstar_usb_init(void)
{
	const struct Sstar_platform_data *pdata;
	int ret;
	pdata = Sstar_get_platform_data();

	ret=driver_build_info();
	if (pdata->clk_ctrl) {
		ret = pdata->clk_ctrl(pdata, true);
		if (ret)
			goto err_clk;
	}
	if (pdata->power_ctrl) {
		ret = pdata->power_ctrl(pdata, true);
		if (ret)
			goto err_power;
	}
	ret = Sstar_usb_register();
	if (ret)
		goto err_reg;

	ret = Sstar_usb_on(pdata);
	if (ret)
		goto err_on;

	Sstar_wtd_init();
	return 0;

err_on:
	if (pdata->power_ctrl)
		pdata->power_ctrl(pdata, false);
err_power:
	if (pdata->clk_ctrl)
		pdata->clk_ctrl(pdata, false);
err_clk:
	Sstar_usb_deregister();
err_reg:
	return ret;
}

/* Called at Driver Unloading */
static void  Sstar_usb_exit(void)
{
	struct Sstar_platform_data *pdata;
	Sstar_printk_exit("Sstar_usb_exit+++++++\n");
	pdata = Sstar_get_platform_data();
	if(Sstar_usb_deregister())
		Sstar_wtd_exit();
	Sstar_printk_exit("Sstar_usb_exit:usb_deregister\n");
	Sstar_usb_off(pdata);
	if (pdata->power_ctrl)
		pdata->power_ctrl(pdata, false);
	if (pdata->clk_ctrl)
		pdata->clk_ctrl(pdata, false);
	Sstar_printk_exit("Sstar_usb_exit---------\n");
}

static struct platform_device *usb_platform_dev = NULL;
#ifdef CONFIG_PM_SLEEP
static int Sstar_usb_pm_notifier(struct notifier_block *b, unsigned long pm_event, void *d)
{
	switch (pm_event) {
	case PM_HIBERNATION_PREPARE:
	case PM_SUSPEND_PREPARE:
		if(usb_platform_dev)
			Sstar_cache_fw_before_suspend(&usb_platform_dev->dev);
		break;

	case PM_POST_RESTORE:
		/* Restore from hibernation failed. We need to clean
		 * up in exactly the same way, so fall through. */
	case PM_POST_HIBERNATION:
	case PM_POST_SUSPEND:
		
		break;

	case PM_RESTORE_PREPARE:
	default:
		break;
	}

	return NOTIFY_DONE;
}
static struct notifier_block Sstar_usb_pm_nb = {
	.notifier_call = Sstar_usb_pm_notifier,
	.priority = 0,
};
#endif
static int Sstar_usb_platform_probe(struct platform_device *pdev)
{
#ifdef CONFIG_PM_SLEEP
	Sstar_printk_init("%s\n",__func__);
	register_pm_notifier(&Sstar_usb_pm_nb);
#endif
	return 0;
}
static int Sstar_usb_platform_remove(struct platform_device *pdev)
{
#ifdef CONFIG_PM_SLEEP
	Sstar_printk_exit("%s\n",__func__);
	unregister_pm_notifier(&Sstar_usb_pm_nb);
#endif
	return 0;
}

static struct platform_driver Sstar_usb_platform_driver = {
	.probe = Sstar_usb_platform_probe,
	.remove = Sstar_usb_platform_remove,
	.driver = {
		.name = "Sstarusbwifi",
	},
};

static void Sstar_usb_platform_init(void)
{
	int ret;
	usb_platform_dev = platform_device_alloc("Sstarusbwifi",0);

	if(!usb_platform_dev){
		Sstar_printk_init( "alloc platform device err\n");
		goto unreg_plate;
	}
	ret = platform_device_add(usb_platform_dev);
	if (ret){
		Sstar_printk_init("platform_device_add err\n");
		goto put_dev;
	}
	ret = platform_driver_register(&Sstar_usb_platform_driver);
	if (ret)
		goto put_dev;
	return;
put_dev:
	platform_device_put(usb_platform_dev);
unreg_plate:
	usb_platform_dev = NULL;
	return;
}
static void Sstar_usb_platform_exit(void)
{
	if(usb_platform_dev){
		platform_driver_unregister(&Sstar_usb_platform_driver);
		platform_device_unregister(usb_platform_dev);
		usb_platform_dev = NULL;
	}
}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0))
static int Sstar_process_each_dev(struct usb_device *usb_dev, void *data)
{
	struct usb_interface *intf = NULL;
	int probe_wait = 0;
	bool found = false;
	
	printk(KERN_ERR "%s--usb_dev:product(%s),manufacturer(%s),vendor(%x),product(%x)\n",
		__func__,usb_dev->product,usb_dev->manufacturer,usb_dev->descriptor.idVendor,usb_dev->descriptor.idProduct);
	if((usb_dev->descriptor.idVendor != 0x7a) || (usb_dev->descriptor.idProduct != 0x8888)){
		goto exit;
	}
retry:
	
	intf = usb_find_interface(&apollod_driver,-1);

	if(intf&&(interface_to_usbdev(intf) == usb_dev)){
		struct net_device *dev;

		rtnl_lock();
		for_each_netdev(&init_net, dev){
			struct ieee80211_sub_if_data *sdata;
			
			if (!dev->ieee80211_ptr || !dev->ieee80211_ptr->wiphy)
				continue;
		
			if (dev->ieee80211_ptr->wiphy->privid != mac80211_wiphy_privid)
				continue;

			sdata = IEEE80211_DEV_TO_SUB_IF(dev);

			if(wiphy_dev(sdata->local->hw.wiphy) != &intf->dev){
				continue;
			}

			printk(KERN_ERR "%s:found (%s) dev\n",__func__,sdata->name);
			found = true;
			break;
		}	
		rtnl_unlock();
		
	} 
	
	if(found == false){
		probe_wait ++;		
		if(probe_wait >= 10){
			goto exit;
		}
		printk(KERN_ERR "%s:not find interface(%d)",__func__,probe_wait);
		schedule_timeout_interruptible(msecs_to_jiffies(300));
		goto retry;
	}
exit:
	return 0;
}

static void abtm_usb_enum_each_interface(void)
{
	usb_for_each_dev(NULL,Sstar_process_each_dev);
}
#else
static void abtm_usb_enum_each_interface(void)
{
	
}
#endif
static int __init Sstar_usb_module_init(void)
{
	Sstar_printk_init("Sstar_usb_module_init %x\n",wifi_module_exit);
	ieee80211_Sstar_mem_int();
	ieee80211_Sstar_skb_int();
	Sstar_init_firmware();
	Sstar_usb_module_lock_int();
	Sstar_wifi_set_status(0);
	Sstar_usb_module_lock();
	Sstar_usb_platform_init();
	Sstar_usb_dvobj_assign_pointer(NULL);
	Sstar_module_attribute_init();
	Sstar_usb_module_unlock();
	Sstar_ieee80211_init();
	Sstar_usb_init();
	register_reboot_notifier(&Sstar_reboot_nb);
	abtm_usb_enum_each_interface();
	return 0;
}
//wifi_usb_probe_doing
//inorder to wait usb probe  function done ,
//when we running in usb_probe function, can't Sstar_usb_module_exit
static void Sstar_wait_usb_probe_end(void)
{	
	int loop=0;
	while(wifi_usb_probe_doing){
		Sstar_printk_exit("Sstar_wait_usb_probe_end\n");	
		mdelay(10);
		loop++;
		if(loop>200)
			break;
	}
}
static void Sstar_usb_wait_tx_pending_complete(void)
{
	int loop =0;
	Sstar_printk_exit("%s:wifi_tx_urb_pending(%d)\n",__func__,wifi_tx_urb_pending);
	while(wifi_tx_urb_pending !=0){
		mdelay(10);
		if(loop++ > 100){
			break;
		}
	}	
	Sstar_printk_exit("%s:wifi_tx_urb_pending(%d)\n",__func__,wifi_tx_urb_pending);
}

static void  Sstar_usb_fw_sync(struct Sstar_common *hw_priv,struct dvobj_priv *dvobj)
{
	int loop =0;
#if 1//(PROJ_TYPE<ARES_B)
	u32 regdata = 0;
#endif
	Sstar_usb_module_lock_check();
	Sstar_wifi_set_status(1);
	Sstar_usb_wait_tx_pending_complete();
	Sstar_printk_exit("%s:(%p)(%p)\n",__func__,hw_priv,dvobj);
	if((dvobj == NULL) || (hw_priv == NULL)){
		Sstar_printk_exit("%s(%p),(%p) err1\n",__func__,dvobj,hw_priv);
		goto exit;
	}

	if(dvobj->self->core != hw_priv){		
		Sstar_printk_exit("%s(%p),(%p) err2\n",__func__,dvobj->self->core,hw_priv);
		goto exit;
	}
	Sstar_wait_scan_complete_sync(hw_priv);
	wsm_wait_pm_sync(hw_priv);
	/*
	*must make sure that there are no pkgs in the lmc.
	*/
	wsm_lock_tx_async(hw_priv);
	wsm_flush_tx(hw_priv);

	#if 0//(PROJ_TYPE>=ARES_B)
	//reboot system not call reset cmd here, will call usb_reset cmd later
	if(g_rebootSystem==0){
		Sstar_printk_exit("Sstar_usb_ep0_hw_reset_cmd +++\n");
		Sstar_usb_ep0_hw_reset_cmd(dvobj->self,HW_RESET_HIF_SYSTEM_CPU,0);
	}
	#else
	Sstar_printk_exit("HiHwCnfBufaddr  %x\n",hw_priv->wsm_caps.HiHwCnfBufaddr);
	if(atomic_read(&g_wtd.wtd_probe)&&((hw_priv->wsm_caps.HiHwCnfBufaddr  & 0xFFF00000) == 0x9000000)){
		Sstar_printk_exit("Sstar_usb_hw_write_port  0x87690121\n");
	
		regdata = 0x87690121;
		Sstar_usb_hw_write_port(dvobj->self,hw_priv->wsm_caps.HiHwCnfBufaddr,&regdata,4);
		loop = 0;
		Sstar_usb_ep0_cmd(dvobj->self);
		while(regdata != 0x87690122) {						
			mdelay(10);						
			Sstar_usb_hw_read_port(dvobj->self,hw_priv->wsm_caps.HiHwCnfBufaddr+4,&regdata,4);
			if(regdata == 0x87690122){
				Sstar_printk_exit("wait usb lmac rmmod ok !!!!!!!!\n");
				break;
			}
			if(loop++ > 100){
				Sstar_printk_exit("wait usb lmac rmmod fail !!!!!!!!\n");
				break;
			}
		}
	}
	#endif
	wsm_unlock_tx(hw_priv);
exit:
	Sstar_wifi_set_status(2);
	Sstar_printk_exit("Sstar_usb_rmmod_sync loop  %d!!!!!!!!\n",loop);
}

static void Sstar_usb_rmmod_sync(void)
{
	Sstar_usb_module_lock();
	Sstar_usb_station_diconnect_sync(Sstar_hw_priv_dereference(),SSTAR_SYSTEM_RMMOD);	
	Sstar_usb_fw_sync(Sstar_hw_priv_dereference(),Sstar_usb_dvobj_dereference());
	Sstar_usb_module_unlock();
}
static void  Sstar_usb_module_exit(void)
{
	Sstar_printk_exit("Sstar_usb_module_exit g_dvobj %p wifi_usb_probe_doing %d\n",g_dvobj ,wifi_usb_probe_doing);
	Sstar_wait_usb_probe_end();
	Sstar_usb_rmmod_sync();
	Sstar_printk_exit("Sstar_usb_exit!!!\n");
	atomic_set(&g_wtd.wtd_term, 1);
	atomic_set(&g_wtd.wtd_run, 0);
	Sstar_usb_exit();
	unregister_reboot_notifier(&Sstar_reboot_nb);
	Sstar_ieee80211_exit();
	Sstar_release_firmware();
	Sstar_usb_module_lock();
	Sstar_usb_dvobj_assign_pointer(NULL);
	Sstar_usb_platform_exit();
	Sstar_usb_module_unlock();
	Sstar_module_attribute_exit();
	Sstar_wifi_set_status(0);
	Sstar_usb_module_lock_release();
	ieee80211_Sstar_mem_exit();
	ieee80211_Sstar_skb_exit();
	Sstar_printk_exit("Sstar_usb_module_exit--%d\n",wifi_module_exit);
	return ;
}


module_init(Sstar_usb_module_init);
module_exit(Sstar_usb_module_exit);
