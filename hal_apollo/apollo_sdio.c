/*
 * Mac80211 SDIO driver for altobeam APOLLO device
 * *
 * Copyright (c) 2016, altobeam
 * Author:
 *
 * Based on apollo code Copyright (c) 2010, ST-Ericsson
 * Author: Dmitry Tarnyagin <dmitry.tarnyagin@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
 #define DEBUG 1
//#undef CONFIG_ATBM_APOLLO_USE_GPIO_IRQ 
#include <linux/version.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio.h>
#include <linux/spinlock.h>
#include <net/atbm_mac80211.h>
#include <linux/kthread.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/platform_device.h>


#include "apollo.h"
#include "sbus.h"
#include "apollo_plat.h"
#include "debug.h"
#include "hwio.h"
#include "svn_version.h"
MODULE_DESCRIPTION("mac80211 altobeam apollo wifi SDIO driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("atbm_wlan");
struct build_info{
	int ver;
	int dpll;
	char driver_info[64];
};
#ifdef CONFIG_ATBM_SUPPORT_SG
#pragma message("Support Network SG")
#endif

#ifdef CONFIG_TX_NO_CONFIRM
#pragma message("Tx No Confirm")
#endif

//const char DRIVER_INFO[]={"[====="__DATE__" "__TIME__"""=====]"};
const char DRIVER_INFO[]={"[====="" """"=====]"};
static int driver_build_info(void)
{
	struct build_info build;
	build.ver=DRIVER_VER;
	if (DPLL_CLOCK==1)
		build.dpll=40;
	else if(DPLL_CLOCK==2)
		build.dpll=24;
	else
		build.dpll=26;
	memcpy(build.driver_info,(void*)DRIVER_INFO,sizeof(DRIVER_INFO));
	atbm_printk_init("SVN_VER=%d,DPLL_CLOCK=%d,BUILD_TIME=%s\n",build.ver,build.dpll,build.driver_info);

#if (OLD_RATE_POLICY==0)
	atbm_printk_init("----drvier RATEPOLCIY=NEW\n");
#else
	atbm_printk_init("----drvier RATEPOLCIY=OLD\n");
#endif

#if (PROJ_TYPE==APOLLO_1601)
	atbm_printk_init("----drvier support chip APOLLOB 1601\n");
#elif (PROJ_TYPE==APOLLO_1606)
	atbm_printk_init("----drvier support chip APOLLOB 1606\n");
#elif (PROJ_TYPE==APOLLO_C)
	atbm_printk_init("----drvier support chip APOLLOC \n");
#elif (PROJ_TYPE==ATHENA_B)
	atbm_printk_init("----drvier support chip ATHENA_B \n");
#endif

	return 0;
}
#define SDIO_RX_WORKQUEUE
struct sbus_priv {
	struct sdio_func	*func;
	struct atbm_common	*core;
	
	struct workqueue_struct 	*tx_workqueue;
	struct work_struct tx_complete_work;
#ifdef SDIO_RX_WORKQUEUE
	struct work_struct rx_complete_work;
	struct workqueue_struct 	*rx_workqueue;
#else
	struct tasklet_struct rx_cmp_tasklet;
#endif	
	
	
	const struct atbm_platform_data *pdata;
	spinlock_t		lock;
	sbus_irq_handler	irq_handler;
	int 			atbm_bgf_irq;
	int 			oob_irq_enabled;
	void			*irq_priv;
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
static const struct sdio_device_id atbm_sdio_ids[] = {
	{ SDIO_DEVICE(SDIO_ANY_ID, SDIO_ANY_ID) },
	{ /* end: all zeroes */			},
};

static int  atbm_sdio_init(void);
static void  atbm_sdio_exit(void);
extern 	int atbm_plat_request_gpio_irq(const struct atbm_platform_data *pdata,struct sbus_priv *self,int * atbm_bgf_irq);
extern 	void atbm_plat_free_gpio_irq(const struct atbm_platform_data *pdata,struct sbus_priv *self,int atbm_bgf_irq);
static int atbm_sdio_reset_chip(struct sbus_priv *self);
extern void atbm_sdio_rx_bh(struct atbm_common *hw_priv);
extern void atbm_sdio_tx_bh(struct atbm_common *hw_priv);

static struct sbus_wtd         g_wtd={
	.wtd_init  = 0,
	.wtd_thread = NULL,
};
#ifdef SDIO_RX_WORKQUEUE
static void atbm_rx_complete_work(struct work_struct *work)
{
	struct sbus_priv *self =
			container_of(work, struct sbus_priv , rx_complete_work);

	atbm_sdio_rx_bh(self->core);
}
#endif
static void atbm_tx_complete_work(struct work_struct *work)
{
	struct sbus_priv *self =
				container_of(work, struct sbus_priv , tx_complete_work);
	atbm_sdio_tx_bh(self->core);
}
static int atbm_sdio_xmit_init(struct sbus_priv *self)
{
	struct atbm_common *hw_priv = self->core;
	
	atbm_printk_init("atbmwifi INIT_WORK enable\n");
	INIT_WORK(&self->tx_complete_work, atbm_tx_complete_work);
	self->tx_workqueue= create_singlethread_workqueue(ieee80211_alloc_name(hw_priv->hw,"tx_workqueue")/*"tx_workqueue"*/);

	if(self->tx_workqueue == NULL)
		return -1;

	hw_priv->xmit_buff = atbm_kzalloc(SDIO_TX_MAXLEN, GFP_KERNEL);

	if(hw_priv->xmit_buff == NULL){
		return -1;
	}
	
	return 0;
}

static int atbm_sdio_xmit_deinit(struct sbus_priv *self)
{
	flush_workqueue(self->tx_workqueue);
	destroy_workqueue(self->tx_workqueue);
	self->tx_workqueue = NULL;	
	if(self->core->xmit_buff){
		atbm_kfree(self->core->xmit_buff);
		self->core->xmit_buff = NULL;
	}	
	return 0;
}
static atbm_sdio_rev_init(struct sbus_priv *self)
{
	struct atbm_common *hw_priv = self->core;
	
	atbm_printk_init("atbmwifi INIT_WORK enable\n");
	hw_priv->bh_running = false;
#ifdef SDIO_RX_WORKQUEUE
	INIT_WORK(&self->rx_complete_work, atbm_rx_complete_work);
	self->rx_workqueue= create_singlethread_workqueue(ieee80211_alloc_name(hw_priv->hw,"rx_workqueue")/*"rx_workqueue"*/);
	if(self->rx_workqueue == NULL)
		return -1;
#else
	tasklet_init(&self->rx_cmp_tasklet, atbm_sdio_rx_bh, hw_priv);
#endif
    return 0;
}

static int atbm_sdio_rev_deinit(struct sbus_priv *self)
{
#ifdef SDIO_RX_WORKQUEUE	
	flush_workqueue(self->rx_workqueue);
	destroy_workqueue(self->rx_workqueue);
	self->rx_workqueue = NULL;
#else
	tasklet_kill(&self->rx_cmp_tasklet); 
#endif
	return 0;
}

static int atbm_sdio_xmit_schedule(struct sbus_priv *self)
{
	struct atbm_common *hw_priv = self->core;

	if(atomic_read(&hw_priv->bh_term)|| hw_priv->bh_error || (hw_priv->bh_thread == NULL))
		return -1;
	if(self->tx_workqueue==NULL)
	{
		atbm_printk_err("atbm_bh_schedule_tx term ERROR\n");
		return -1;
	}
	queue_work(self->tx_workqueue,&self->tx_complete_work);
	return 0;

}
static int atbm_sdio_rev_schedule(struct sbus_priv *self)
{
	struct atbm_common *hw_priv = self->core;
	
	if(atomic_read(&hw_priv->bh_term)|| hw_priv->bh_error || (hw_priv->bh_thread == NULL))
		return -1;
#ifdef SDIO_RX_WORKQUEUE
	if((self->rx_workqueue==NULL))
	{
		return -1;
	}
	queue_work(self->rx_workqueue,&self->rx_complete_work);
#else
	tasklet_schedule(&self->rx_cmp_tasklet);
#endif
	return 0;
}
static int atbm_sdio_rev_giveback(struct sbus_priv *self,void *giveback)
{
	struct atbm_common *hw_priv = self->core;
	struct wsm_rx *rx = (struct wsm_rx *)giveback;
	u32 hw_xmited = rx->channel_type;
	int hw_free;
	
	spin_lock_bh(&hw_priv->tx_com_lock);
	BUG_ON((int)hw_xmited > (int)hw_priv->n_xmits);
	if(hw_priv->n_xmits - hw_xmited <= hw_priv->wsm_caps.numInpChBufs){
		hw_free =  (hw_priv->wsm_caps.numInpChBufs-hw_priv->hw_bufs_used) - (hw_priv->n_xmits-hw_xmited);
		if(hw_priv->hw_bufs_free < hw_free)
			hw_priv->hw_bufs_free = hw_free;
	}
	spin_unlock_bh(&hw_priv->tx_com_lock);

	return 0;
}

/* sbus_ops implemetation */

static int atbm_sdio_memcpy_fromio(struct sbus_priv *self,
				     unsigned int addr,
				     void *dst, int count)
{
	return sdio_memcpy_fromio(self->func, dst, addr, count);
}

static int atbm_sdio_memcpy_toio(struct sbus_priv *self,
				   unsigned int addr,
				   const void *src, int count)
{
	return sdio_memcpy_toio(self->func, addr, (void *)src, count);
}


int atbm_readb_func0(struct sbus_priv *self,
						 unsigned int addr,int *ret_err)
{
	u8 data;
	sdio_claim_host(self->func);
	data = sdio_f0_readb(self->func,addr,ret_err);
	sdio_release_host(self->func);
	return data;
}

int atbm_writeb_func0(struct sbus_priv *self,
						 unsigned int addr,u8 data)
{
	int ret_err;
	sdio_claim_host(self->func);
	//data = sdio_f0_writeb(self->func,addr,ret_err);
	sdio_f0_writeb(self->func, data, addr, &ret_err);
	sdio_release_host(self->func);
	return ret_err;
}

static void atbm_sdio_lock(struct sbus_priv *self)
{
	sdio_claim_host(self->func);
}

static void atbm_sdio_unlock(struct sbus_priv *self)
{
	sdio_release_host(self->func);
}
#ifndef CONFIG_ATBM_APOLLO_USE_GPIO_IRQ
static void atbm_sdio_irq_handler(struct sdio_func *func)
{
	struct sbus_priv *self = sdio_get_drvdata(func);

	BUG_ON(!self);

	if (self->irq_handler)
		self->irq_handler(self->irq_priv);
}
#endif
#ifdef CONFIG_ATBM_APOLLO_USE_GPIO_IRQ

irqreturn_t atbm_gpio_hardirq(int irq, void *dev_id)
{
	return IRQ_WAKE_THREAD;
}
void atbm_oob_intr_set(struct sbus_priv *self, bool enable)
{
	unsigned long flags;

	if (!self)
		return;

	spin_lock_irqsave(&self->lock, flags);
	if (self->oob_irq_enabled != enable) {
		if (enable)
			enable_irq(self->atbm_bgf_irq);
		else
			disable_irq_nosync(self->atbm_bgf_irq);
		self->oob_irq_enabled = enable;
	}
	spin_unlock_irqrestore(&self->lock, flags);
}

irqreturn_t atbm_gpio_irq(int irq, void *dev_id)
{
	struct sbus_priv *self = dev_id;

	if (self) {
		atbm_oob_intr_set(self, 0);
		self->irq_handler(self->irq_priv);
		if(!in_interrupt())
			atbm_oob_intr_set(self, 1);
		return IRQ_HANDLED;
	} else {
		return IRQ_NONE;
	}
}

static int atbm_request_irq(struct sbus_priv *self)
{
	int ret = 0;
	int func_num;
	u8 cccr;
//	int bgf_irq;
	
	/* Hack to access Fuction-0 */
	func_num = self->func->num;
	self->func->num = 0;

	cccr = sdio_readb(self->func, SDIO_CCCR_IENx, &ret);
	if (WARN_ON(ret))
		goto err;

	/* Master interrupt enable ... */
	cccr |= BIT(0);

	/* ... for our function */
	cccr |= BIT(func_num);

	sdio_writeb(self->func, cccr, SDIO_CCCR_IENx, &ret);
	if (WARN_ON(ret))
		goto err;

	/* back to	Fuction-1 */
	self->func->num = func_num;

	ret = atbm_plat_request_gpio_irq(self->pdata,self,&self->atbm_bgf_irq);
	//printk("========================bgf_irq=%d\n",bgf_irq);

	if (WARN_ON(ret))
		goto err;
	self->oob_irq_enabled = 1;

	return 0;

err:
	atbm_plat_free_gpio_irq(self->pdata,self,self->atbm_bgf_irq);
	atbm_printk_bus("[%s]  fail exiting sw_gpio_irq_request..   :%d\n",__func__, ret);
	return ret;
}
#endif
static int atbm_sdio_irq_subscribe(struct sbus_priv *self,
				     sbus_irq_handler handler,
				     void *priv)
{
	int ret;
	unsigned long flags;

	if (!handler)
		return -EINVAL;

	spin_lock_irqsave(&self->lock, flags);
	self->irq_priv = priv;
	self->irq_handler = handler;
	spin_unlock_irqrestore(&self->lock, flags);

	atbm_printk_bus("[ATBM_WIFI]SW IRQ subscribe\n");
	sdio_claim_host(self->func);
#ifndef CONFIG_ATBM_APOLLO_USE_GPIO_IRQ
	#pragma message("atbm wifi SDIO_IRQ")
	atbm_printk_bus("[ATBM_WIFI] used SDIO Irq \n");
	ret = sdio_claim_irq(self->func, atbm_sdio_irq_handler);
	if (ret)
		atbm_printk_err("Failed to claim sdio Irq :%d\n",ret);
#else
	#pragma message("atbm wifi GPIO_IRQ")
	atbm_printk_bus("[ATBM_WIFI] used GPIO Irq \n");
	ret = atbm_request_irq(self);
#endif
	sdio_release_host(self->func);
	return ret;
}

static int atbm_sdio_irq_unsubscribe(struct sbus_priv *self)
{
	int ret = 0;
	unsigned long flags;
	//const struct resource *irq = self->pdata->irq;

	WARN_ON(!self->irq_handler);
	if (!self->irq_handler)
		return 0;

	atbm_printk_bus("[ATBM_WIFI]:SW IRQ unsubscribe\n");

#ifndef CONFIG_ATBM_APOLLO_USE_GPIO_IRQ
	sdio_claim_host(self->func);
	ret = sdio_release_irq(self->func);
	sdio_release_host(self->func);
#else
    atbm_plat_free_gpio_irq(self->pdata,self,self->atbm_bgf_irq);
	//free_irq(self->atbm_bgf_irq,self);
	//gpio_free(self->pdata->irq_gpio);
#endif  //CONFIG_ATBM_APOLLO_USE_GPIO_IRQ

	spin_lock_irqsave(&self->lock, flags);
	self->irq_priv = NULL;
	self->irq_handler = NULL;
	spin_unlock_irqrestore(&self->lock, flags);

	return ret;
}


#if ((ATBM_WIFI_PLATFORM != 10) && (ATBM_WIFI_PLATFORM != PLATFORM_AMLOGIC_S805) \
	&& (ATBM_WIFI_PLATFORM != PLATFORM_AMLOGIC_905))
static int atbm_detect_card(const struct atbm_platform_data *pdata)
{
	/* HACK!!!
	 * Rely on mmc->class_dev.class set in mmc_alloc_host
	 * Tricky part: a new mmc hook is being (temporary) created
	 * to discover mmc_host class.
	 * Do you know more elegant way how to enumerate mmc_hosts?
	 */

	struct mmc_host *mmc = NULL;
	struct class_dev_iter iter;
	struct device *dev;
	static struct platform_device *sdio_platform_dev = NULL;
	int status = 0;
	
	sdio_platform_dev = platform_device_alloc("atbmsdiowifi",0);
	if(sdio_platform_dev == NULL){
		status = -ENOMEM;
		goto platform_dev_err;
	}

	if(platform_device_add(sdio_platform_dev) != 0){
		status = -ENOMEM;
		goto platform_dev_err;
	}
	
	mmc = mmc_alloc_host(0, &sdio_platform_dev->dev);
	
	if (!mmc){
		status = -ENOMEM;
		goto exit;
	}

	BUG_ON(!mmc->class_dev.class);
	class_dev_iter_init(&iter, mmc->class_dev.class, NULL, NULL);
	for (;;) {
		dev = class_dev_iter_next(&iter);
		if (!dev) {
			atbm_printk_err( "atbm: %s is not found.\n",
				pdata->mmc_id);
			break;
		} else {
			struct mmc_host *host = container_of(dev,
				struct mmc_host, class_dev);

			atbm_printk_bus("apollo:  found. %s\n",
				dev_name(&host->class_dev));

			if (dev_name(&host->class_dev) &&
				strcmp(dev_name(&host->class_dev),
					pdata->mmc_id))
				continue;

			if(host->card == NULL)
				mmc_detect_change(host, 10);
			else
				atbm_printk_err("%s:%s has been attached\n",__func__,pdata->mmc_id);
			break;
		}
	}
	mmc_free_host(mmc);
exit:
	if(sdio_platform_dev)
		platform_device_unregister(sdio_platform_dev);
	return status;
platform_dev_err:
	if(sdio_platform_dev)
		platform_device_put(sdio_platform_dev);
	return status;
}
#endif //PLATFORM_AMLOGIC_S805

static int atbm_sdio_off(const struct atbm_platform_data *pdata)
{
	int ret = 0;

	if (pdata->insert_ctrl)
		ret = pdata->insert_ctrl(pdata, false);
	return ret;
}
#if ((ATBM_WIFI_PLATFORM != 10) && (ATBM_WIFI_PLATFORM != PLATFORM_AMLOGIC_S805) \
	&& (ATBM_WIFI_PLATFORM != PLATFORM_AMLOGIC_905))

static int atbm_sdio_on(const struct atbm_platform_data *pdata)
{
	int ret = 0;
    if (pdata->insert_ctrl)
		ret = pdata->insert_ctrl(pdata, true);
	msleep(200);
	atbm_detect_card(pdata);
	return ret;
}
#endif //#if ((ATBM_WIFI_PLATFORM != 10) && (ATBM_WIFI_PLATFORM != PLATFORM_AMLOGIC_S805))

static int atbm_cmd52_abort(struct sbus_priv *self)
{
	int ret;
	int regdata;
	sdio_claim_host(self->func);

	/* SDIO Simplified Specification V2.0, 4.4 Reset for SDIO */
	regdata = sdio_f0_readb(self->func, SDIO_CCCR_ABORT, &ret);
	atbm_printk_err("%s,%d ret %d\n",__func__,__LINE__,ret);
	if (ret)
		regdata = 0x08;
	else
		regdata |= 0x01;
	sdio_f0_writeb(self->func, regdata, SDIO_CCCR_ABORT, &ret);
//	msleep(1500);
	atbm_printk_err("%s,%d ret %d\n",__func__,__LINE__,ret);
	sdio_release_host(self->func);
	return ret;
}

static int atbm_sdio_reset(struct sbus_priv *self)
{
	int ret;
	int regdata;
	int func_num;

	return 0;
	atbm_printk_bus("atbm_sdio_reset++\n");
	sdio_claim_host(self->func);
	/* Hack to access Fuction-0 */
	func_num = self->func->num;

	self->func->num = 0;

	/**********************/
	atbm_printk_bus("SDIO_RESET++\n");
	/* SDIO Simplified Specification V2.0, 4.4 Reset for SDIO */
	regdata = sdio_readb(self->func, SDIO_CCCR_ABORT, &ret);
	if (ret)
		regdata = 0x08;
	else
		regdata |= 0x08;
	sdio_writeb(self->func, regdata, SDIO_CCCR_ABORT, &ret);
	if (WARN_ON(ret))
		goto set_func0_err;
	msleep(1500);
	regdata = sdio_readb(self->func, SDIO_CCCR_ABORT, &ret);
	atbm_printk_bus("SDIO_RESET-- 0x%x\n",regdata);

	/**********************/
	atbm_printk_bus("SDIO_SPEED_EHS++\n");
	regdata = sdio_readb(self->func, SDIO_CCCR_SPEED, &ret);
	if (WARN_ON(ret))
		goto set_func0_err;

	regdata |= SDIO_SPEED_EHS;
	sdio_writeb(self->func, regdata, SDIO_CCCR_SPEED, &ret);
	if (WARN_ON(ret))
		goto set_func0_err;

	regdata = sdio_readb(self->func, SDIO_CCCR_SPEED, &ret);
	atbm_printk_bus("SDIO_SPEED_EHS -- 0x%x:0x%x\n",regdata,SDIO_SPEED_EHS);

	/**********************/
	atbm_printk_bus("SDIO_BUS_WIDTH_4BIT++\n");
	regdata = sdio_readb(self->func, SDIO_CCCR_IF, &ret);
	if (WARN_ON(ret))
		goto set_func0_err;

	//regdata |= SDIO_BUS_WIDTH_4BIT;
	regdata = 0xff;
	sdio_writeb(self->func, regdata, SDIO_CCCR_IF, &ret);
	if (WARN_ON(ret))
		goto set_func0_err;
	regdata = sdio_readb(self->func, SDIO_CCCR_IF, &ret);
	atbm_printk_bus("SDIO_BUS_WIDTH_4BIT -- 0x%x:0x%x\n",regdata,SDIO_BUS_WIDTH_4BIT);
	/**********************/
	atbm_printk_bus("SDIO_BUS_ENABLE_FUNC++\n");
	regdata = sdio_readb(self->func, SDIO_CCCR_IOEx, &ret);
	if (WARN_ON(ret))
		goto set_func0_err;
	regdata |= BIT(func_num);
	atbm_printk_bus("SDIO_BUS_ENABLE_FUNC regdata %x\n",regdata);
	sdio_writeb(self->func, regdata, SDIO_CCCR_IOEx, &ret);
	if (WARN_ON(ret))
		goto set_func0_err;
	regdata = sdio_readb(self->func, SDIO_CCCR_IOEx, &ret);
	atbm_printk_bus("SDIO_BUS_ENABLE_FUNC -- 0x%x\n",regdata);
	/**********************/

set_func0_err:
	self->func->num = func_num;
	sdio_set_block_size(self->func,512);
	/* Restore the WLAN function number */
	sdio_release_host(self->func);

	return 0;
}

static u32 atbm_sdio_align_size(struct sbus_priv *self, u32 size)
{
	u32 aligned = sdio_align_size(self->func, size);
	return aligned;
}

int atbm_sdio_set_block_size(struct sbus_priv *self, u32 size)
{
	return sdio_set_block_size(self->func, size);
}

static int atbm_sdio_pm(struct sbus_priv *self, bool  suspend)
{
	int ret = 0;
	return ret;
}
int atbm_wtd_term(struct atbm_common *hw_priv)
{
	return atomic_read(&hw_priv->sbus_priv->wtd->wtd_term);
}
void atbm_wtd_wakeup( struct sbus_priv *self)
{
#ifdef CONFIG_ATBMWIFI_WDT
	if(atomic_read(&self->wtd->wtd_term))
		return;
	atomic_set(&g_wtd.wtd_run, 1);
	atbm_printk_err("[atbm_wtd] wakeup.\n");
	wake_up(&self->wtd->wtd_evt_wq);
#endif //CONFIG_ATBMWIFI_WDT
}
static int atbm_wtd_process(void *arg)
{
#ifdef CONFIG_ATBMWIFI_WDT
	int status=0;
	int term=0;
	int wtd_run=0;
	while(1){
		status = wait_event_interruptible(g_wtd.wtd_evt_wq, ({
				term = atomic_read(&g_wtd.wtd_term);
				wtd_run = atomic_read(&g_wtd.wtd_run);
				(term || wtd_run);}));
		if (status < 0 || term ){
			atbm_printk_exit("[atbm_wtd]:1 thread break %d %d\n",status,term);
			goto __stop;
		}
		atomic_set(&g_wtd.wtd_run, 0);
		
	}
__stop:
	while(term){
		
		atbm_printk_exit("[atbm_wtd]:kthread_should_stop\n");
		if(kthread_should_stop()){
			break;
		}
		schedule_timeout_uninterruptible(msecs_to_jiffies(100));
	}
#endif //CONFIG_ATBMWIFI_WDT
	return 0;
}
static void atbm_wtd_init(void)
{
#ifdef CONFIG_ATBMWIFI_WDT
	int err = 0;
	struct sched_param param = { .sched_priority = 1 };
	if(g_wtd.wtd_init)
		return;
	atbm_printk_exit( "[wtd] register.\n");
	init_waitqueue_head(&g_wtd.wtd_evt_wq);
	atomic_set(&g_wtd.wtd_term, 0);
	g_wtd.wtd_thread = kthread_create(&atbm_wtd_process, &g_wtd, "atbm_wtd");
	if (IS_ERR(g_wtd.wtd_thread)) {
		err = PTR_ERR(g_wtd.wtd_thread);
		g_wtd.wtd_thread = NULL;
	} else {
		WARN_ON(sched_setscheduler(g_wtd.wtd_thread,
			SCHED_FIFO, &param));
#ifdef HAS_PUT_TASK_STRUCT
		get_task_struct(g_wtd.wtd_thread);
#endif
		wake_up_process(g_wtd.wtd_thread);
	}
	g_wtd.wtd_init = 1;
#endif //CONFIG_ATBMWIFI_WDT
}
static void atbm_wtd_exit(void)
{
#ifdef CONFIG_ATBMWIFI_WDT
	struct task_struct *thread = g_wtd.wtd_thread;
	if (WARN_ON(!thread))
		return;
	if(atomic_read(&g_wtd.wtd_term)==0)
		return;
	g_wtd.wtd_thread = NULL;
	atbm_printk_exit( "[wtd] unregister.\n");
	atomic_add(1, &g_wtd.wtd_term);
	wake_up(&g_wtd.wtd_evt_wq);
	kthread_stop(thread);
#ifdef HAS_PUT_TASK_STRUCT
	put_task_struct(thread);
#endif
	g_wtd.wtd_init = 0;
#endif //CONFIG_ATBMWIFI_WDT
}
static struct sbus_ops atbm_sdio_sbus_ops = {
	.sbus_memcpy_fromio	= atbm_sdio_memcpy_fromio,
	.sbus_memcpy_toio	= atbm_sdio_memcpy_toio,
	.sbus_read_sync 	= atbm_sdio_memcpy_fromio,
	.sbus_write_sync	= atbm_sdio_memcpy_toio,
	.lock				= atbm_sdio_lock,
	.unlock				= atbm_sdio_unlock,
	.irq_subscribe		= atbm_sdio_irq_subscribe,
	.irq_unsubscribe	= atbm_sdio_irq_unsubscribe,
	.reset				= atbm_sdio_reset,
	.align_size			= atbm_sdio_align_size,
	.power_mgmt			= atbm_sdio_pm,
	.set_block_size		= atbm_sdio_set_block_size,
	.wtd_wakeup			= atbm_wtd_wakeup,
	.sbus_reset_chip    = atbm_sdio_reset_chip,
	.abort				= atbm_cmd52_abort,
	//.sbus_cmd52_fromio =atbm_cmd52_fromio,
	//.sbus_cmd52_toio =atbm_cmd52_toio,
	.sbus_xmit_func_init   = atbm_sdio_xmit_init,
	.sbus_xmit_func_deinit  = atbm_sdio_xmit_deinit,
	.sbus_rev_func_init    = atbm_sdio_rev_init,
	.sbus_rev_func_deinit  = atbm_sdio_rev_deinit,
	.sbus_xmit_schedule    = atbm_sdio_xmit_schedule,
	.sbus_rev_schedule     = atbm_sdio_rev_schedule,
	.sbus_rev_giveback	   = atbm_sdio_rev_giveback,
};

/* Probe Function to be called by SDIO stack when device is discovered */
static int atbm_sdio_probe(struct sdio_func *func,
			      const struct sdio_device_id *id)
{
	struct sbus_priv *self;
	int status;

	atbm_dbg(ATBM_APOLLO_DBG_INIT, "Probe called\n");
	
	atomic_set(&g_wtd.wtd_probe, 0);
	func->card->quirks|=MMC_QUIRK_LENIENT_FN0;
	func->card->quirks |= MMC_QUIRK_BLKSZ_FOR_BYTE_MODE;

	self = atbm_kzalloc(sizeof(*self), GFP_KERNEL);
	if (!self) {
		atbm_dbg(ATBM_APOLLO_DBG_ERROR, "Can't allocate SDIO sbus_priv.");
		return -ENOMEM;
	}

	spin_lock_init(&self->lock);
	self->pdata = atbm_get_platform_data();
	self->func = func;
	self->wtd = &g_wtd;
	sdio_set_drvdata(func, self);
	sdio_claim_host(func);
	sdio_enable_func(func);
	sdio_release_host(func);

	//reset test start
	//atbm_sdio_reset(self);
	//reset test end

	atbm_printk_init("%s:v12\n",__func__);
	status = atbm_core_probe(&atbm_sdio_sbus_ops,
			      self, &func->dev, &self->core);
	if (status) {
		sdio_claim_host(func);
		sdio_disable_func(func);
		sdio_release_host(func);
		sdio_set_drvdata(func, NULL);
		atbm_kfree(self);
		atomic_set(&g_wtd.wtd_probe, -1);
		//printk("[atbm_wtd]:set wtd_probe = -1\n");
	}
	else {
		atomic_set(&g_wtd.wtd_probe, 1);
		atbm_printk_exit("[atbm_wtd]:set wtd_probe = 1\n");
	}
	return status;
}

/* Disconnect Function to be called by SDIO stack when
 * device is disconnected */
static int atbm_sdio_reset_chip(struct sbus_priv *self)
{
	atbm_printk_bus("%s\n",__func__);
	atbm_reset_lmc_cpu(self->core);
	return 0;
}
static void atbm_sdio_disconnect(struct sdio_func *func)
{
	struct sbus_priv *self = sdio_get_drvdata(func);
	atbm_printk_exit("atbm_sdio_disconnect");
	if (self) {
		atomic_set(&g_wtd.wtd_probe, 0);
		if (self->core) {
#ifdef RESET_CHIP
			atbm_reset_chip((struct atbm_common *)self->core->hw->priv);
#else
			/*
			*should not rest cpu here,we will do it at function atbm_unregister_common
			*/
//			atbm_reset_lmc_cpu((struct atbm_common *)self->core->hw->priv);
#endif
			atbm_core_release(self->core);
			self->core = NULL;
		}
		sdio_claim_host(func);
		/*
		*	reset sdio
		*/
		{
			int ret;
			int regdata;
			/**********************/
			atbm_printk_exit("[%s]:SDIO_RESET++\n",dev_name(&func->card->host->class_dev));
			/* SDIO Simplified Specification V2.0, 4.4 Reset for SDIO */
			regdata = sdio_f0_readb(func, SDIO_CCCR_ABORT, &ret);
			if (ret)
				regdata = 0x08;
			else
				regdata |= 0x08;
			sdio_f0_writeb(func, regdata, SDIO_CCCR_ABORT, &ret);
			WARN_ON(ret);
			msleep(50);
			regdata = sdio_f0_readb(func, SDIO_CCCR_ABORT, &ret);
			atbm_printk_exit("[%s]:SDIO_RESET-- 0x%x\n",dev_name(&func->card->host->class_dev),regdata);

			/**********************/
		}
		sdio_disable_func(func);
		sdio_release_host(func);
		sdio_set_drvdata(func, NULL);
		atbm_kfree(self);
	}
}

static int atbm_suspend(struct device *dev)
{
	int ret;
	struct sdio_func *func = dev_to_sdio_func(dev);

	/* Notify SDIO that CW1200 will remain powered during suspend */
	mmc_pm_flag_t flags=sdio_get_host_pm_caps(func);
	//printk("mmc_pm_flag=%x\n",flags);
	if(!(flags&MMC_PM_KEEP_POWER)){
		atbm_dbg(ATBM_APOLLO_DBG_ERROR,
				"cant remain alive while host is suspended\n");
		return -ENOSYS;
	}
	ret = sdio_set_host_pm_flags(func, MMC_PM_KEEP_POWER);
	if (ret)
		atbm_dbg(ATBM_APOLLO_DBG_ERROR,
			   "set sdio keep pwr flag failed:%d\n", ret);
	/*sdio irq wakes up host*/
	if (flags&MMC_PM_WAKE_SDIO_IRQ){
		ret = sdio_set_host_pm_flags(func, MMC_PM_WAKE_SDIO_IRQ);
	}
	if (ret)
		atbm_dbg(ATBM_APOLLO_DBG_ERROR,
			   "set sdio wake up irq flag failed:%d\n", ret);
	return ret;
}

static int atbm_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops atbm_pm_ops = {
	.suspend = atbm_suspend,
	.resume = atbm_resume,
};

static struct sdio_driver sdio_driver = {
	.name		= "atbm_wlan",
	.id_table	= atbm_sdio_ids,
	.probe		= atbm_sdio_probe,
	.remove		= atbm_sdio_disconnect,
	.drv = {
		.pm = &atbm_pm_ops,
	}
};
static int atbm_reboot_notifier(struct notifier_block *nb,
				unsigned long action, void *unused)
{
	atbm_printk_exit("atbm_reboot_notifier\n");
	atomic_set(&g_wtd.wtd_term, 1);
	atomic_set(&g_wtd.wtd_run, 0);
	atbm_sdio_exit();
	atbm_ieee80211_exit();
	atbm_release_firmware();
	return NOTIFY_DONE;
}

/* Probe Function to be called by USB stack when device is discovered */
static struct notifier_block atbm_reboot_nb = {
	.notifier_call = atbm_reboot_notifier,
	.priority=1,
};


/* Init Module function -> Called by insmod */
static int  atbm_sdio_init(void)
{
	const struct atbm_platform_data *pdata;
	int ret;
	pdata = atbm_get_platform_data();

	ret=driver_build_info();
	if (pdata->clk_ctrl) {
		ret = pdata->clk_ctrl(pdata, true);
		if (ret)
			goto err_clk;
	}
/*
* modify for rockchip platform
*/
#if (ATBM_WIFI_PLATFORM == 10)
	if (pdata->insert_ctrl&&pdata->power_ctrl)
	{
		ret = pdata->insert_ctrl(pdata, false);
		ret = pdata->power_ctrl(pdata, false);
		if (ret)
			goto err_power;
		ret = pdata->power_ctrl(pdata, true);
		if (ret)
			goto err_power;
		ret = pdata->insert_ctrl(pdata, true);
	}
	else
	{
		goto err_power;
	}
#else
	if (pdata->power_ctrl) {
		ret = pdata->power_ctrl(pdata, true);
		if (ret)
			goto err_power;
	}
#endif
	ret = sdio_register_driver(&sdio_driver);
	if (ret)
		goto err_reg;
#if ((ATBM_WIFI_PLATFORM != 10) && (ATBM_WIFI_PLATFORM != PLATFORM_AMLOGIC_S805)\
	&& (ATBM_WIFI_PLATFORM != PLATFORM_AMLOGIC_905))

	ret = atbm_sdio_on(pdata);
	if (ret)
		goto err_on;
#endif
	atbm_wtd_init();
	return 0;

#if ((ATBM_WIFI_PLATFORM != 10) && (ATBM_WIFI_PLATFORM != PLATFORM_AMLOGIC_S805)\
	&& (ATBM_WIFI_PLATFORM != PLATFORM_AMLOGIC_905))

err_on:
	if (pdata->power_ctrl)
		pdata->power_ctrl(pdata, false);
#endif
err_power:
	if (pdata->clk_ctrl)
		pdata->clk_ctrl(pdata, false);
err_clk:
	sdio_unregister_driver(&sdio_driver);
err_reg:
	return ret;
}

/* Called at Driver Unloading */
static void  atbm_sdio_exit(void)
{
	const struct atbm_platform_data *pdata;
	pdata = atbm_get_platform_data();
	atbm_wtd_exit();
	sdio_unregister_driver(&sdio_driver);
	atbm_sdio_off(pdata);
	if (pdata->power_ctrl)
		pdata->power_ctrl(pdata, false);
	if (pdata->clk_ctrl)
		pdata->clk_ctrl(pdata, false);
}


static int __init apollo_sdio_module_init(void)
{
	ieee80211_atbm_mem_int();
	ieee80211_atbm_skb_int();
	register_reboot_notifier(&atbm_reboot_nb);
	atbm_init_firmware();
	atbm_ieee80211_init();
	return atbm_sdio_init();
}
static void  apollo_sdio_module_exit(void)
{	
	atomic_set(&g_wtd.wtd_term, 1);
	atomic_set(&g_wtd.wtd_run, 0);
	atbm_sdio_exit();
	atbm_ieee80211_exit();
	atbm_release_firmware();
	unregister_reboot_notifier(&atbm_reboot_nb);
	ieee80211_atbm_mem_exit();
	ieee80211_atbm_skb_exit();
}


module_init(apollo_sdio_module_init);
module_exit(apollo_sdio_module_exit);
