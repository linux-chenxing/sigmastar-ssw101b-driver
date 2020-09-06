/*
 * Mac80211 spi driver for sigmastar APOLLO device
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
 #define DEBUG 1

#include <linux/version.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <net/Sstar_mac80211.h>
#include <linux/kthread.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/spi/spi.h>
#include "apollo.h"
#include "sbus.h"
#include "apollo_plat.h"
#include "debug.h"
#include "hwio_spi.h"
#include "svn_version.h"

MODULE_DESCRIPTION("mac80211 sigmastar apollo wifi spi driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("Sstar_wlan");
struct build_info{
	int ver;
	int dpll;
	char driver_info[64];
};

const char DRIVER_INFO[]={"[====="" """"=====]"};
static int driver_build_info(void)
{
	struct build_info build;
	build.ver=DRIVER_VER;
	if (DPLL_CLOCK==1)
		build.dpll=40;
	else
		build.dpll=26;
	memcpy(build.driver_info,(void*)DRIVER_INFO,sizeof(DRIVER_INFO));
	Sstar_printk_init("SVN_VER=%d,DPLL_CLOCK=%d,BUILD_TIME=%s\n",build.ver,build.dpll,build.driver_info);

#if (OLD_RATE_POLICY==0)
	Sstar_printk_init("----drvier RATEPOLCIY=NEW\n");
#else
	Sstar_printk_init("----drvier RATEPOLCIY=OLD\n");
#endif

#if (PROJ_TYPE==APOLLO_1601)
	Sstar_printk_init("----drvier support chip APOLLOB 1601\n");
#elif (PROJ_TYPE==APOLLO_1606)
	Sstar_printk_init("----drvier support chip APOLLOB 1606\n");
#elif (PROJ_TYPE==APOLLO_C)
	Sstar_printk_init("----drvier support chip APOLLOC \n");
#endif

	return 0;
}

#define SELF_BUFLEN (512)
struct sbus_priv {
	struct spi_device	*spi;
	struct Sstar_common	*core;
	const struct Sstar_platform_data *pdata;
	spinlock_t		lock;
	
	spinlock_t		spi_rwlock;
	struct sbus_wtd         * wtd;
	
	int old_channelFlag;
};
struct sbus_wtd {
	int 	wtd_init;
	struct task_struct		*wtd_thread;
	wait_queue_head_t		wtd_evt_wq;
	atomic_t				wtd_term;
	atomic_t				wtd_run;
	atomic_t				wtd_probe;
	atomic_t				wtd_spi_read_ready;
	struct Sstar_common		*core;
};

static int  Sstar_spi_init(void);
static void  Sstar_spi_exit(void);
static struct sbus_wtd         g_wtd={
	.wtd_init  = 0,
	.wtd_thread = NULL,
};

static void spidev_complete(void *arg)
{
	complete(arg);
}

static ssize_t
spidev_sync(struct sbus_priv *self, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	message->complete = spidev_complete;
	message->context = &done;

	spin_lock_irq(&self->lock);
	if (self->spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(self->spi, message);
	spin_unlock_irq(&self->lock);

	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
		//if (status == 0)
		//	status = message->actual_length;
	}
	return status;
}

static ssize_t
spidev_sync_write_then_write(struct sbus_priv *self, u8 *cmd, size_t cmd_len,const u8 *tx, size_t tx_len)
{
	struct spi_transfer	t[2];
	struct spi_message	m;

	memset(t, 0, sizeof(t));
	t[0].tx_buf = cmd;
	t[0].len 	= cmd_len;
	t[1].tx_buf	= tx;
	t[1].len 	= tx_len;

	spi_message_init(&m);
	spi_message_add_tail(&t[0], &m);
	spi_message_add_tail(&t[1], &m);
	return spidev_sync(self, &m);
}

static ssize_t
spidev_sync_write_then_read(struct sbus_priv *self, u8 *cmd, size_t cmd_len, u8 *rx, size_t rx_len)
{
	struct spi_transfer	t[2];
	struct spi_message	m;

	memset(t, 0, sizeof(t));
	t[0].tx_buf = cmd;
	t[0].len 	= cmd_len;
	t[1].rx_buf	= rx;
	t[1].len 	= rx_len;

	spi_message_init(&m);
	spi_message_add_tail(&t[0], &m);
	spi_message_add_tail(&t[1], &m);
	return spidev_sync(self, &m);
}

/* Read-only message with current device setup */
static int	Sstar_spi_read_data(struct sbus_priv *self, void *rx, size_t rx_len)
{
	ssize_t			status = 0;
	u8 cmdbuff[2] = {0x0b, 0};

	status = spidev_sync_write_then_read(self, cmdbuff, sizeof(cmdbuff), rx, rx_len);
	
	return status;
}

/* Write-only message with current device setup */
static int	Sstar_spi_write_data(struct sbus_priv *self,const void *tx, size_t tx_len)
{
	ssize_t			status = 0;
	u8 cmdbuff[2] = {0x51, 0};

	status = spidev_sync_write_then_write(self, cmdbuff, sizeof(cmdbuff), tx, tx_len);

	return status;
}

static int Sstar_spi_read_status(struct sbus_priv *self, u32 *status, size_t tx_len)
{
	ssize_t			ret = 0;
	u8 cmdbuff[2] = {0x05, 0};

	ret = spidev_sync_write_then_read(self, cmdbuff, sizeof(cmdbuff), (u8 *)status, tx_len);
	
	return ret;	
}
static int Sstar_spi_read_channelflag(struct sbus_priv *self, u32 *channelflag)
{
	ssize_t			ret = 0;
	u32 status = 0;
	unsigned long flags;

	ret = Sstar_spi_read_status(self, &status, sizeof(status));
	if (ret == 0)
	{
	
		spin_lock_irqsave(&self->lock, flags);
		if ((status & SPI_CHANNEL_FLAG) == self->old_channelFlag)
		{
			*channelflag = 0;
		}else
		{
			*channelflag = 1;
		}
	
		spin_unlock_irqrestore(&self->lock, flags);
	}
	
	return ret;	
}
static int  Sstar_spi_update_channelflag(struct sbus_priv *self)
{
	ssize_t			ret = 0;
	u32 status = 0;
	unsigned long flags;

	ret = Sstar_spi_read_status(self, &status, sizeof(status));
	if (ret == 0)
	{
		spin_lock_irqsave(&self->lock, flags);
		self->old_channelFlag = status & SPI_CHANNEL_FLAG;
		spin_unlock_irqrestore(&self->lock, flags);
	}
	return ret;	
}
static int Sstar_spi_read_ready(struct sbus_priv *self, u32 *ready)
{
	ssize_t			ret = 0;
	u32 status = 0;

	ret = Sstar_spi_read_status(self, &status, sizeof(status));
	if (ret == 0)
	{
		if (status & SPI_STATUS_READY)
		{
			*ready = 1;
		}else
		{
			*ready = 0;
		}
	}
	
	return ret;	
}
static int Sstar_spi_reset_cpu(struct sbus_priv *self)
{
	ssize_t			ret = 0;
	u8 cmdbuff[4] = {0xaa, 0, 0, 0xaa};

	ret = spidev_sync_write_then_write(self, cmdbuff, 2, &cmdbuff[2], 2);
	
	return ret;	
}
static int Sstar_spi_shutdown_wlan(struct sbus_priv *self)
{
	ssize_t			ret = 0;
	u8 cmdbuff[4] = {0xaa, 0, 2, 0xac};

	ret = spidev_sync_write_then_write(self, cmdbuff, 2, &cmdbuff[2], 2);
	
	return ret;	
}
static int Sstar_spi_reset_chip(struct sbus_priv *self)
{
	ssize_t			ret = 0;
	u8 cmdbuff[4] = {0xaa, 0, 1, 0xab};

	ret = spidev_sync_write_then_write(self, cmdbuff, 2, &cmdbuff[2], 2);
	
	return ret;	
}


static int Sstar_spi_write_firmware(struct sbus_priv *self,unsigned int addr, const void *src, int count)
{
	ssize_t			ret = 0;
	u32 status = 0;
	
	if (addr < DOWNLOAD_DTCM_ADDR)
	{
		u8	iccm_cmdbuf[2] = {0x51,0x00};		
		ret = spidev_sync_write_then_write(self, iccm_cmdbuf, sizeof(iccm_cmdbuf), src, count);
	}
	else
	{
		//dccm		
		u8	dccm_cmdbuf[2] = {0x56,0x00};
		ret = spidev_sync_write_then_write(self, dccm_cmdbuf, sizeof(dccm_cmdbuf), src, count);
	}
	if (ret)
	{
		return -1;
	}

	ret = Sstar_spi_read_status(self, &status, sizeof(status));
	if ((ret) || (status & SPI_OVERRUN))//overrun
	{
		Sstar_dbg(SSTAR_APOLLO_DBG_ERROR,"[Sstar_load_fw]:SPI_STATUS_OVERRUN\n");
		ret = -2;
	}
	
	return ret;
}

static u32 Sstar_spi_align_size(struct sbus_priv *self, u32 size)
{
	u32 aligned = size;
	return aligned;
}

static int Sstar_spi_reset(struct sbus_priv *self)
{
	return 0;
}

static int Sstar_spi_set_block_size(struct sbus_priv *self, u32 size)
{
	return 0;
}

static void Sstar_spi_lock(struct sbus_priv *self)
{
	

}

static void Sstar_spi_unlock(struct sbus_priv *self)
{
	
}

static int Sstar_spi_irq_subscribe(struct sbus_priv *self, sbus_irq_handler handler,void *priv)
{
	int ret = 0;
	return ret;
}
static int Sstar_spi_irq_unsubscribe(struct sbus_priv *self)
{
	int ret = 0;
	return ret;
}
static int Sstar_spi_pm(struct sbus_priv *self, bool  suspend)
{
	int ret = 0;
	return ret;
}
void Sstar_spi_read_rdy_start(void)
{
#ifdef CONFIG_SSTARWIFI_WDT
		if(g_wtd.wtd_init == 0)
			return;

		if(atomic_read(&g_wtd.wtd_term))
			return;
		atomic_set(&g_wtd.wtd_spi_read_ready, 1);
		wake_up(&g_wtd.wtd_evt_wq);
		Sstar_printk_bus("[Sstar_wtd] wakeup.\n");
#endif //CONFIG_SSTARWIFI_WDT
}

void Sstar_spi_read_rdy_end(void)
{
#ifdef CONFIG_SSTARWIFI_WDT
		if(g_wtd.wtd_init == 0)
			return;

		atomic_set(&g_wtd.wtd_spi_read_ready, 0);
#endif //CONFIG_SSTARWIFI_WDT
}

void Sstar_spi_status_rx_ready(struct Sstar_common *priv)
{
	int ret = 0;
	u32 ready =0 ;
	BUG_ON(!priv->sbus_ops);

	ret = Sstar_spi_read_ready(priv->sbus_priv, &ready);
	Sstar_printk_bus("[wtd] ready %d\n", ready);
	if ((ready == 1))
	{
		if (atomic_add_return(1, &priv->bh_rx) == 1){
			atomic_set(&g_wtd.wtd_spi_read_ready, 0);
			wake_up(&priv->bh_wq);
		}
	}
	return;
}


void Sstar_wtd_wakeup( struct sbus_priv *self)
{
#ifdef CONFIG_SSTARWIFI_WDT
	if(atomic_read(&self->wtd->wtd_term))
		return;
	atomic_set(&g_wtd.wtd_run, 1);
	Sstar_printk_bus( "[Sstar_wtd] wakeup.\n");
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
	int spi_read_ready = 0;
	Sstar_printk_bus("[Sstar_wtd]:Sstar_wtd_process start++\n");

	while(1){
		status = wait_event_interruptible(g_wtd.wtd_evt_wq, ({
				term = atomic_read(&g_wtd.wtd_term);
				wtd_run = atomic_read(&g_wtd.wtd_run);
				spi_read_ready = atomic_read(&g_wtd.wtd_spi_read_ready);
				(term || wtd_run || spi_read_ready);}));
		if (status < 0 || term ){
			Sstar_printk_exit("[Sstar_wtd]:1 thread break %d %d\n",status,term);
			goto __stop;
		}
		
		if (spi_read_ready)
		{		
			do
			{
				Sstar_spi_status_rx_ready(g_wtd.core);
				if (atomic_read(&g_wtd.wtd_term))
				{
					atomic_set(&g_wtd.wtd_spi_read_ready, 0);
					break;
				}
				msleep(1000);
			}while(atomic_read(&g_wtd.wtd_spi_read_ready));
			
			Sstar_printk_exit("[Sstar_wtd]:Sstar_spi_status_rx_ready end++\n");
			continue;
		}
		
		atomic_set(&g_wtd.wtd_run, 0);
		do
		{
			/*
			*must make sure that g_hw_priv->bh_error is 0 when hmac is
			*in reset state,but........
			*/
			Sstar_hw_priv_dereference()->bh_error = 0;
			err = Sstar_reset_driver(Sstar_hw_priv_dereference());
			mdelay(5);
		}
	while(err == -1);
	}
__stop:
	while(term){
		
		Sstar_printk_bus("[Sstar_wtd]:kthread_should_stop\n");
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
		WARN_ON(sched_setscheduler(g_wtd.wtd_thread,
			SCHED_FIFO, &param));
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
	Sstar_printk_exit("[wtd] unregister.\n");
	atomic_add(1, &g_wtd.wtd_term);
	wake_up(&g_wtd.wtd_evt_wq);
	kthread_stop(thread);
#ifdef HAS_PUT_TASK_STRUCT
	put_task_struct(thread);
#endif
	g_wtd.wtd_init = 0;
#endif //CONFIG_SSTARWIFI_WDT
}
static struct sbus_ops Sstar_spi_sbus_ops = {
	.sbus_read_data = Sstar_spi_read_data,
	.sbus_write_data = Sstar_spi_write_data,
	.sbus_read_status = Sstar_spi_read_status,
	.sbus_read_ready = Sstar_spi_read_ready,
	.sbus_update_channelflag = Sstar_spi_update_channelflag,
	.sbus_read_channelflag = Sstar_spi_read_channelflag,
	.sbus_reset_cpu = Sstar_spi_reset_cpu,
	.sbus_reset_chip = Sstar_spi_reset_chip,
	.sbus_shutdown_wlan = Sstar_spi_shutdown_wlan,
	.sbus_write_firmware = Sstar_spi_write_firmware,
	.align_size			= Sstar_spi_align_size,
	.reset				= Sstar_spi_reset,
	.set_block_size		= Sstar_spi_set_block_size,
	.lock 				= Sstar_spi_lock,
	.unlock 				= Sstar_spi_unlock,
	.irq_unsubscribe	= Sstar_spi_irq_unsubscribe,
	.irq_subscribe	= Sstar_spi_irq_subscribe,
	.power_mgmt			= Sstar_spi_pm,
};

/* Probe Function to be called by spi stack when device is discovered */
static int  Sstar_spi_probe(struct spi_device *spi)
{
	struct sbus_priv *self;
	int status;

	Sstar_dbg(SSTAR_APOLLO_DBG_INIT, "Probe called\n");
	
	atomic_set(&g_wtd.wtd_probe, 0);
	
	atomic_set(&g_wtd.wtd_spi_read_ready, 0);

	self = Sstar_kzalloc(sizeof(*self), GFP_KERNEL);
	if (!self) {
		Sstar_dbg(SSTAR_APOLLO_DBG_ERROR, "Can't allocate spi sbus_priv.");
		return -ENOMEM;
	}

	spin_lock_init(&self->lock);
	self->wtd = &g_wtd;
	self->spi = spi;
	self->old_channelFlag = 0;

	status = Sstar_core_probe(&Sstar_spi_sbus_ops,
			      self, &spi->dev, &g_wtd.core);
	if (status) {
		goto err_status;
		//printk("[Sstar_wtd]:set wtd_probe = -1\n");
	}
	self->core = g_wtd.core;

	atomic_set(&g_wtd.wtd_probe, 1);
	Sstar_printk_init("[Sstar_wtd]:set wtd_probe = 1\n");	
	spi_set_drvdata(spi, self);
	return 0;
	
err_status:
	Sstar_kfree(self);
	atomic_set(&g_wtd.wtd_probe, -1);
	return status;
}

/* Disconnect Function to be called by spi stack when
 * device is disconnected */
static int Sstar_spi_disconnect(struct spi_device *spi)
{
	struct sbus_priv *self = spi_get_drvdata(spi);

	if (self) {
		
		spin_lock_irq(&self->lock);
		self->spi = NULL;
		spi_set_drvdata(spi, NULL);
		spin_unlock_irq(&self->lock);
		atomic_set(&g_wtd.wtd_probe, 0);
		if (self->core) {
			Sstar_core_release(self->core);
			self->core = NULL;
		}
		Sstar_kfree(self);
	}
	return 0;
}

static struct spi_driver spi_driver = {
	.driver = {
		.name  = "SSTARWIFI",
		.owner = THIS_MODULE,
	},
	.probe =	Sstar_spi_probe,
	.remove = (Sstar_spi_disconnect),

	/* NOTE:  suspend/resume methods are not necessary here.
	 * We don't do anything except pass the requests to/from
	 * the underlying controller.  The refrigerator handles
	 * most issues; the controller driver handles the rest.
	 */
};

static int Sstar_reboot_notifier(struct notifier_block *nb,
				unsigned long action, void *unused)
{
	Sstar_printk_exit("Sstar_reboot_notifier\n");
	atomic_set(&g_wtd.wtd_term, 1);
	atomic_set(&g_wtd.wtd_run, 0);
	Sstar_spi_exit();
	Sstar_ieee80211_exit();
	Sstar_release_firmware();
	return NOTIFY_DONE;
}

/* Probe Function to be called by USB stack when device is discovered */
static struct notifier_block Sstar_reboot_nb = {
	.notifier_call = Sstar_reboot_notifier,
	.priority=1,
};

/* Init Module function -> Called by insmod */
static int  Sstar_spi_init(void)
{
	int ret;

	ret=driver_build_info();
	
	Sstar_wtd_init();
	ret=spi_register_driver(&spi_driver);
	if (ret)
		goto err_reg;

	return 0;

err_reg:
	return ret;
}

/* Called at Driver Unloading */
static void  Sstar_spi_exit(void)
{
	Sstar_wtd_exit();
	spi_unregister_driver(&spi_driver);
}


static int __init apollo_spi_module_init(void)
{
	ieee80211_Sstar_mem_int();
	ieee80211_Sstar_skb_int();
	register_reboot_notifier(&Sstar_reboot_nb);
	Sstar_init_firmware();
	Sstar_ieee80211_init();

	return Sstar_spi_init();
}
static void  apollo_spi_module_exit(void)
{	
	atomic_set(&g_wtd.wtd_term, 1);
	atomic_set(&g_wtd.wtd_run, 0);
	Sstar_spi_exit();
	Sstar_ieee80211_exit();
	Sstar_release_firmware();
	unregister_reboot_notifier(&Sstar_reboot_nb);
	ieee80211_Sstar_mem_exit();
	ieee80211_Sstar_skb_exit();
}


module_init(apollo_spi_module_init);
module_exit(apollo_spi_module_exit);
