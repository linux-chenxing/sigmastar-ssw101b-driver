/*
 * Device handling thread implementation for mac80211 sigmastar APOLLO drivers
 *
 * Copyright (c) 2016, sigmastar
 * Author:
 *
 * Based on:
 * sstar UMAC CW1200 driver, which is
 * Copyright (c) 2010, ST-Ericsson
 * Author: Ajitpal Singh <ajitpal.singh@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
//#undef CONFIG_SSTAR_APOLLO_USE_GPIO_IRQ
#include <net/Sstar_mac80211.h>
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
#if defined(CONFIG_SSTAR_APOLLO_BH_DEBUG)
#define bh_printk  Sstar_printk_always
#else
#define bh_printk(...)
#endif
#define IS_SMALL_MSG(wsm_id) (wsm_id & 0x1000)
static int Sstar_bh(void *arg);
extern void Sstar_monitor_pc(struct Sstar_common *hw_priv);

static int Sstar_bh_read_ctrl_reg(struct Sstar_common *hw_priv,
					  u16 *ctrl_reg);
int Sstar_bh_read_ctrl_reg_unlock(struct Sstar_common *hw_priv,
					  u16 *ctrl_reg);

static struct sk_buff *Sstar_get_skb(struct Sstar_common *hw_priv, u32 len);
int Sstar_rx_tasklet(struct Sstar_common *hw_priv, int id,
		  struct wsm_hdr *wsm, struct sk_buff **skb_p);

typedef int (*reed_ctrl_handler_t)(struct Sstar_common *hw_priv,u16 *ctrl_reg);
typedef int (*reed_data_handler_t)(struct Sstar_common *hw_priv, void *buf, u32 buf_len);

/* TODO: Verify these numbers with WSM specification. */
#define DOWNLOAD_BLOCK_SIZE_WR	(0x2000 - 4)
/* an SPI message cannot be bigger than (2"12-1)*2 bytes
 * "*2" to cvt to bytes */
#define MAX_SZ_RD_WR_BUFFERS	(DOWNLOAD_BLOCK_SIZE_WR*2)
#define PIGGYBACK_CTRL_REG	(2)
#define EFFECTIVE_BUF_SIZE	(MAX_SZ_RD_WR_BUFFERS - PIGGYBACK_CTRL_REG)

typedef int (*Sstar_wsm_handler)(struct Sstar_common *hw_priv,
	u8 *data, size_t size);

#ifdef MCAST_FWDING
int wsm_release_buffer_to_fw(struct Sstar_vif *priv, int count);
#endif
/*Os wake lock*/
#define SSTAR_OS_WAKE_LOCK(WAKELOCK)			Sstar_os_wake_lock(WAKELOCK)
#define SSTAR_OS_WAKE_UNLOCK(WAKELOCK)		Sstar_os_wake_unlock(WAKELOCK)
/*BH wake lock*/
#define SSTAR_BH_WAKE_LOCK(WAKELOCK)         Sstar_bh_wake_lock(WAKELOCK)
#define SSTAR_BH_WAKE_UNLOCK(WAKELOCK)         Sstar_bh_wake_unlock(WAKELOCK)
int Sstar_os_check_wakelock(struct Sstar_common *hw_priv)
{
	if (!hw_priv)
		return 0;
#ifdef CONFIG_HAS_WAKELOCK
	/* Indicate to the SD Host to avoid going to suspend if internal locks are up */
	if (wake_lock_active(&hw_priv->hw_wake) ||
		(wake_lock_active(&hw_priv->bh_wake)))
		return 1;
#endif
	return 0;
}

int Sstar_os_wake_lock(struct Sstar_common *hw_priv)
{
	unsigned long flags;
	int ret = 0;
	ret=Sstar_os_check_wakelock(hw_priv);
	if(ret){
		return 0;
	}
	if (hw_priv) {
		spin_lock_irqsave(&hw_priv->wakelock_spinlock, flags);
		if (hw_priv->wakelock_hw_counter == 0) {
#ifdef CONFIG_HAS_WAKELOCK
			wake_lock(&hw_priv->hw_wake);
#elif defined(SDIO_BUS) && (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 36))
			//pm_stay_awake(hw_priv);
#endif
		}
		hw_priv->wakelock_hw_counter++;
		ret = hw_priv->wakelock_hw_counter;
		spin_unlock_irqrestore(&hw_priv->wakelock_spinlock, flags);
	}
	return ret;
}

int Sstar_os_wake_unlock(struct Sstar_common *hw_priv)
{
	unsigned long flags;
	int ret = 0;

	if (hw_priv) {
		spin_lock_irqsave(&hw_priv->wakelock_spinlock, flags);
		if (hw_priv->wakelock_hw_counter > 0) {
			hw_priv->wakelock_hw_counter--;
			if (hw_priv->wakelock_hw_counter == 0) {
#ifdef CONFIG_HAS_WAKELOCK
				wake_unlock(&hw_priv->hw_wake);
#elif defined(SDIO_BUS) && (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 36))
				//pm_relax(hw_priv);
#endif
			}
			ret = hw_priv->wakelock_hw_counter;
		}
		spin_unlock_irqrestore(&hw_priv->wakelock_spinlock, flags);
	}
	return ret;
}
int Sstar_bh_wake_lock(struct Sstar_common *hw_priv)
{
	unsigned long flags;
	int ret;
	ret=Sstar_os_check_wakelock(hw_priv);
	if(ret){
		return 0;
	}
	if (hw_priv) {
		spin_lock_irqsave(&hw_priv->wakelock_spinlock, flags);
		if (hw_priv->wakelock_bh_counter == 0) {
#ifdef CONFIG_HAS_WAKELOCK
			wake_lock(&hw_priv->bh_wake);
#elif defined(SDIO_BUS) && (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 36))
			//pm_stay_awake(hw_priv);
#endif		
			hw_priv->wakelock_bh_counter++;
		}
		spin_unlock_irqrestore(&hw_priv->wakelock_spinlock, flags);
	}
	return 0;
}

void Sstar_bh_wake_unlock(struct Sstar_common *hw_priv)
{
	unsigned long flags;
	if (hw_priv) {
		spin_lock_irqsave(&hw_priv->wakelock_spinlock, flags);
		if (hw_priv->wakelock_bh_counter > 0) {
			hw_priv->wakelock_bh_counter = 0;
#ifdef CONFIG_HAS_WAKELOCK
		wake_unlock(&hw_priv->bh_wake);
#elif defined(SDIO_BUS) && (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 36))
		//pm_relax(hw_priv);
#endif
		}
		spin_unlock_irqrestore(&hw_priv->wakelock_spinlock, flags);
	}

}
void Sstar_sdio_bh_halt(struct Sstar_common *hw_priv)
{
	Sstar_printk_err("%s:bh halt\n",__func__);
	if (atomic_add_return(1, &hw_priv->bh_halt) == 1){
		atomic_set(&hw_priv->Sstar_pluged,0);
		wake_up(&hw_priv->bh_wq);
	}
}

int Sstar_register_bh(struct Sstar_common *hw_priv)
{
	int err = 0;
	struct sched_param param = { .sched_priority = 1 };
	Sstar_printk_init("[BH] register.\n");
	BUG_ON(hw_priv->bh_thread);
	atomic_set(&hw_priv->bh_rx, 0);
	atomic_set(&hw_priv->bh_tx, 0);
	atomic_set(&hw_priv->bh_term, 0);
	atomic_set(&hw_priv->bh_suspend, SSTAR_APOLLO_BH_RESUMED);
	hw_priv->buf_id_tx = 0;
	hw_priv->buf_id_rx = 0;
	hw_priv->syncChanl_done=1;
	init_waitqueue_head(&hw_priv->bh_wq);
	init_waitqueue_head(&hw_priv->bh_evt_wq);
	hw_priv->bh_thread = kthread_create(&Sstar_bh, hw_priv, ieee80211_alloc_name(hw_priv->hw,"Sstar_bh"));
	if (IS_ERR(hw_priv->bh_thread)) {
		err = PTR_ERR(hw_priv->bh_thread);
		hw_priv->bh_thread = NULL;
	} else {
		WARN_ON(sched_setscheduler(hw_priv->bh_thread,
			SCHED_FIFO, &param));
#ifdef HAS_PUT_TASK_STRUCT
		get_task_struct(hw_priv->bh_thread);
#endif
		wake_up_process(hw_priv->bh_thread);
	}
	return err;
}
static void Sstar_hw_buff_reset(struct Sstar_common *hw_priv)
{
	int i;
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
}
int Sstar_reset_lmc_cpu(struct Sstar_common *hw_priv)
{
	u32 val32;
	int ret=0;
	int retry=0;
	if(hw_priv == NULL)
	{
		return -1;
	}
	while (retry <= MAX_RETRY) {
		ret = Sstar_reg_read_32(hw_priv, SSTAR_HIFREG_CONFIG_REG_ID, &val32);
		if(!ret){
			retry=0;
			break;
		}else{
			/*reset sdio internel reg by send cmd52 to abort*/
			WARN_ON(hw_priv->sbus_ops->abort(hw_priv->sbus_priv));
			retry++;
			mdelay(1);
			Sstar_printk_err(
				"%s:%d: enable_irq: can't read " \
				"config register.\n", __func__,__LINE__);
		}
	}
	val32 |= SSTAR_HIFREG_CONFIG_CPU_RESET_BIT_2;
	val32 |= SSTAR_HIFREG_CONFIG_CPU_RESET_BIT;	
	
	while (retry <= MAX_RETRY) {
		ret = Sstar_reg_write_32(hw_priv, SSTAR_HIFREG_CONFIG_REG_ID,val32);
		if(!ret){
		    retry=0;
			break;
		}else{
			/*reset sdio internel reg by send cmd52 to abort*/
			WARN_ON(hw_priv->sbus_ops->abort(hw_priv->sbus_priv));
			retry++;
			mdelay(1);
			Sstar_printk_err(
				"%s:%d: enable_irq: can't write " \
				"config register.\n", __func__,__LINE__);
		}
	}
	while (retry <= MAX_RETRY) {
		ret = Sstar_reg_read_32(hw_priv, SSTAR_HIFREG_CONFIG_REG_ID, &val32);
		if(!ret){
			retry=0;
			break;
		}else{
			/*reset sdio internel reg by send cmd52 to abort*/
			WARN_ON(hw_priv->sbus_ops->abort(hw_priv->sbus_priv));
			retry++;
			mdelay(1);
			Sstar_printk_err(
				"%s:%d: enable_irq: can't read " \
				"config register.\n", __func__,__LINE__);
		}
	}
	val32 &= ~SSTAR_HIFREG_CONFIG_CPU_RESET_BIT_2;

	while (retry <= MAX_RETRY) {
		ret = Sstar_reg_write_32(hw_priv, SSTAR_HIFREG_CONFIG_REG_ID,val32);
		if(!ret){
			retry=0;
			break;
		}else{
			/*reset sdio internel reg by send cmd52 to abort*/
			WARN_ON(hw_priv->sbus_ops->abort(hw_priv->sbus_priv));
			retry++;
			mdelay(1);
			Sstar_printk_err(
				"%s:%d: enable_irq: can't write " \
				"config register.\n", __func__,__LINE__);
		}
	}

	while (retry <= MAX_RETRY) {
		ret = Sstar_reg_read_32(hw_priv, SSTAR_HIFREG_CONFIG_REG_ID, &val32);
		if(!ret){
			retry=0;
			break;
		}else{
			/*reset sdio internel reg by send cmd52 to abort*/
			WARN_ON(hw_priv->sbus_ops->abort(hw_priv->sbus_priv));
			retry++;
			mdelay(1);
			Sstar_printk_err( "%s:%d: set_mode: can't read config register.\n",__func__,__LINE__);
		}
	}
	val32 |= SSTAR_HIFREG_CONFIG_ACCESS_MODE_BIT;

	while (retry <= MAX_RETRY) {
		ret = Sstar_reg_write_32(hw_priv, SSTAR_HIFREG_CONFIG_REG_ID,val32);
		if(!ret){
			retry=0;
			break;
		}else{
			/*reset sdio internel reg by send cmd52 to abort*/
			WARN_ON(hw_priv->sbus_ops->abort(hw_priv->sbus_priv));
			retry++;
			mdelay(1);
			Sstar_printk_err("%s:%d: set_mode: can't write config register.\n",__func__,__LINE__);
		}
	}
	return ret;
}

void Sstar_unregister_bh(struct Sstar_common *hw_priv)
{
	struct task_struct *thread = hw_priv->bh_thread;
	if (WARN_ON(!thread))
		return;

	hw_priv->bh_thread = NULL;
	bh_printk( "[BH] unregister.\n");
	atomic_add(1, &hw_priv->bh_term);
	wake_up(&hw_priv->bh_wq);
	kthread_stop(thread);
#ifdef HAS_PUT_TASK_STRUCT
	put_task_struct(thread);
#endif
}
static int Sstar_rx_directly(struct Sstar_common *hw_priv,struct sk_buff **skb_rx)
{
	struct sk_buff *skb = *skb_rx;
	u8 *data;
	struct wsm_hdr *wsm;
	u32 wsm_len;
	int wsm_id;
	u8 wsm_seq;
	static int rx_resync = 1;
	int status = 0;;
	
	data = skb->data;
	wsm = (struct wsm_hdr *)data;
	
	wsm_len = __le32_to_cpu(wsm->len);
	wsm_id  = __le32_to_cpu(wsm->id) & 0xFFF;
	wsm_seq = (__le32_to_cpu(wsm->id) >> 13) & 7;
	
	Sstar_skb_trim(skb, wsm_len);
	if (unlikely(wsm_id == 0x0800)) {
		wsm_handle_exception(hw_priv,
			 &data[sizeof(*wsm)],
			wsm_len - sizeof(*wsm));
			status = -1;
			goto exit;
	} else if (unlikely(!rx_resync)) {
		if (WARN_ON(wsm_seq != hw_priv->wsm_rx_seq)) {
			status = -2;
			goto exit;
		}
	}
	hw_priv->wsm_rx_seq = (wsm_seq + 1) & 7;
	rx_resync = 0;

	if (wsm_id & 0x0400) {
		int rc = wsm_release_tx_buffer(hw_priv, 1);
		if (WARN_ON(rc < 0)){
			status = -3;
			goto exit;
		}else if (rc > 0){
			Sstar_bh_wakeup(hw_priv);
		}
	}
	/* Sstar_wsm_rx takes care on SKB livetime */
	if (WARN_ON(Sstar_rx_tasklet(hw_priv, wsm_id, wsm,
				  skb_rx))){
		status = -4;
		goto exit;
	}
exit:
	if (*skb_rx) {
		Sstar_dev_kfree_skb(*skb_rx);
		*skb_rx = NULL;
	}
	return status;
}
static void Sstar_sdio_submit_skb(struct Sstar_common *hw_priv,struct sk_buff *skb)
{
	bool bh_running = false;
	
	skb->pkt_type = SSTAR_RX_RAW_FRAME;
	spin_lock_bh(&hw_priv->rx_frame_queue.lock);
	__Sstar_skb_queue_tail(&hw_priv->rx_frame_queue, skb);
	bh_running = hw_priv->bh_running;
	spin_unlock_bh(&hw_priv->rx_frame_queue.lock);

	if (bh_running == true)
		;
	else if (hw_priv->sbus_ops->sbus_rev_schedule)
		hw_priv->sbus_ops->sbus_rev_schedule(hw_priv->sbus_priv);
	else if (atomic_add_return(1, &hw_priv->bh_rx) == 1){
		wake_up(&hw_priv->bh_wq);
	}		
}
static int Sstar_sdio_process_read_data(struct Sstar_common *hw_priv,reed_ctrl_handler_t read_ctrl_func,
	reed_data_handler_t read_data_func ,bool directly)
{
	#define LMAC_MAX_RX_BUFF	(24)
	u32 read_len_lsb = 0;
	u32 read_len_msb = 0;
	u32 read_len;
	struct sk_buff *skb_rx = NULL;
	u16 ctrl_reg = 0;
	u32 alloc_len;
	u8 *data;
	u8 rx_continue_cnt = 0;
	
rx_check:
	if (WARN_ON(read_ctrl_func(
			hw_priv, &ctrl_reg))){
			Sstar_printk_err("%s:read_ctrl_func err\n",__func__);
			goto err;
	}
rx_continue:
	read_len_lsb = (ctrl_reg & SSTAR_HIFREG_CONT_NEXT_LEN_LSB_MASK)*2;
	read_len_msb = (ctrl_reg & SSTAR_HIFREG_CONT_NEXT_LEN_MSB_MASK)*2;
	read_len=((read_len_msb>>2)+read_len_lsb);
	if (!read_len) {
		return 0;
	}

	if (WARN_ON((read_len < sizeof(struct wsm_hdr)) ||
			(read_len > EFFECTIVE_BUF_SIZE))) {
			Sstar_printk_err("Invalid read len: %d,read_cnt(%d)\n",
				read_len,rx_continue_cnt);
		
		Sstar_sdio_bh_halt(hw_priv);
		goto err;
	}
	/* Add SIZE of PIGGYBACK reg (CONTROL Reg)
	 * to the NEXT Message length + 2 Bytes for SKB */
	read_len = read_len + 2;
	alloc_len = read_len;
	if (alloc_len % SDIO_BLOCK_SIZE ) {
		alloc_len -= (alloc_len % SDIO_BLOCK_SIZE );
		alloc_len += SDIO_BLOCK_SIZE;
	}
	/* Check if not exceeding CW1200 capabilities */
	if (WARN_ON_ONCE(alloc_len > EFFECTIVE_BUF_SIZE)) {
		Sstar_printk_err("Read aligned len: %d\n",
			alloc_len);
	}
	skb_rx = Sstar_get_skb(hw_priv, alloc_len);
	if (WARN_ON(!skb_rx)){
		Sstar_printk_err("%s:can not get skb,rx_continue_cnt(%d)\n",__func__,rx_continue_cnt);
		goto err;
	}

	Sstar_skb_trim(skb_rx, 0);
	Sstar_skb_put(skb_rx, read_len);
	data = skb_rx->data;
	if (WARN_ON(!data)){
		goto err;
	}
	if (WARN_ON(read_data_func(hw_priv, data, alloc_len))){
		Sstar_printk_err("%s:read_data_func err,rx_continue_cnt(%d)\n",__func__,rx_continue_cnt);
		Sstar_sdio_bh_halt(hw_priv);
		goto err;
	}
	/* Piggyback */
	ctrl_reg = __le16_to_cpu(
		((__le16 *)data)[alloc_len / 2 - 1]);
	
	if(Sstar_bh_is_term(hw_priv) || atomic_read(&hw_priv->bh_term)){
		goto err;
	}
	
	hw_priv->irq_timestamp = jiffies;
	
	if(directly == true){
		skb_rx->pkt_type = SSTAR_RX_DERICTLY_DATA_FRAME;
		if(unlikely(Sstar_rx_directly(hw_priv,&skb_rx) != 0)){
			Sstar_sdio_bh_halt(hw_priv);
			goto err;
		}
	}else {
		Sstar_sdio_submit_skb(hw_priv, skb_rx);
	}
	
	read_len_lsb = (ctrl_reg & SSTAR_HIFREG_CONT_NEXT_LEN_LSB_MASK)*2;
	read_len_msb = (ctrl_reg & SSTAR_HIFREG_CONT_NEXT_LEN_MSB_MASK)*2;
	read_len=((read_len_msb>>2)+read_len_lsb);
	rx_continue_cnt++;
	
	if(read_len)
		goto rx_continue;
	goto rx_check;
	
	return 0;
err:
	if(skb_rx)
		Sstar_dev_kfree_skb(skb_rx);
	return -1;
}

static bool Sstar_sdio_wait_enough_space(struct Sstar_common	*hw_priv,u32 n_needs)
{
#define MAX_LOOP_POLL_CNT  (2*3000)
	u32 hw_xmited = 0;
	bool enough = false;
	int ret = 0;
	int loop = 0;
	u32 print = 0;

	spin_lock_bh(&hw_priv->tx_com_lock);
	enough = hw_priv->hw_bufs_free >= n_needs ? true : false;
	spin_unlock_bh(&hw_priv->tx_com_lock);
	
	while(enough == false){
		
		hw_priv->sbus_ops->lock(hw_priv->sbus_priv);
		ret = Sstar_direct_read_unlock(hw_priv,hw_priv->wsm_caps.NumOfHwXmitedAddr,&hw_xmited);
		hw_priv->sbus_ops->unlock(hw_priv->sbus_priv);

		if(ret){
			enough = false;
			break;
		}
		
		spin_lock_bh(&hw_priv->tx_com_lock);
		BUG_ON((int)hw_priv->n_xmits < (int)hw_xmited);
		BUG_ON(hw_priv->n_xmits - hw_xmited > hw_priv->wsm_caps.numInpChBufs);
		hw_priv->hw_bufs_free =  (hw_priv->wsm_caps.numInpChBufs-hw_priv->hw_bufs_used) - 
								 (hw_priv->n_xmits-hw_xmited);
		enough = hw_priv->hw_bufs_free >= n_needs ? true : false;
		spin_unlock_bh(&hw_priv->tx_com_lock);

		
		if(enough == false){
			loop ++;
			if(loop>=MAX_LOOP_POLL_CNT)
				break;
			if((loop >= 3)&&(print == 0)){			
				Sstar_printk_always("%s:n_xmits(%d),hw_xmited(%d),need(%d)\n",__func__,
					hw_priv->n_xmits,hw_xmited,n_needs);
				print = 1;
			}
			schedule_timeout_interruptible(msecs_to_jiffies(2));
		}
	}

	return enough;
}

static bool Sstar_sdio_have_enough_space(struct Sstar_common	*hw_priv,u32 n_needs)
{
	u32 hw_xmited = 0;
	bool enough = false;
	int ret = 0;
	
	spin_lock_bh(&hw_priv->tx_com_lock);
	enough = hw_priv->hw_bufs_free >= n_needs ? true : false;
	spin_unlock_bh(&hw_priv->tx_com_lock);

	if(enough == false){
		hw_priv->sbus_ops->lock(hw_priv->sbus_priv);
		ret = Sstar_direct_read_unlock(hw_priv,hw_priv->wsm_caps.NumOfHwXmitedAddr,&hw_xmited);
		hw_priv->sbus_ops->unlock(hw_priv->sbus_priv);

		if(ret){
			enough = false;
		}else {
			spin_lock_bh(&hw_priv->tx_com_lock);
			BUG_ON((int)hw_priv->n_xmits < (int)hw_xmited);
			BUG_ON(hw_priv->n_xmits - hw_xmited > hw_priv->wsm_caps.numInpChBufs);
			hw_priv->hw_bufs_free =  (hw_priv->wsm_caps.numInpChBufs-hw_priv->hw_bufs_used) - 
									 (hw_priv->n_xmits-hw_xmited);
			enough = hw_priv->hw_bufs_free >= n_needs ? true : false;
			spin_unlock_bh(&hw_priv->tx_com_lock);
		}
	}

	return enough;
}
static void Sstar_sdio_release_err_data(struct Sstar_common	*hw_priv,struct wsm_tx *wsm)
{
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
	
	wsm_release_tx_buffer(hw_priv, 1);
	if(!WARN_ON(Sstar_queue_get_skb(queue, wsm->packetID, &skb, &txpriv))) {

		struct ieee80211_tx_info *tx = IEEE80211_SKB_CB(skb);
		//int tx_count = 0;
		int i;
		wsm_release_vif_tx_buffer(hw_priv,txpriv->if_id,1);
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
		wsm_release_vif_tx_buffer(hw_priv,Sstar_queue_get_if_id(wsm->packetID),1);
	}
}
static void Sstar_sdio_release_err_cmd(struct Sstar_common	*hw_priv)
{
	spin_lock_bh(&hw_priv->wsm_cmd.lock);
	hw_priv->wsm_cmd.ret = -1;
	hw_priv->wsm_cmd.done = 1;
	hw_priv->wsm_cmd.cmd = 0xFFFF;
	spin_unlock_bh(&hw_priv->wsm_cmd.lock);
	printk_once(KERN_ERR "%s:release wsm_cmd.lock\n",__func__);
	wake_up(&hw_priv->wsm_cmd_wq);		
}
static int Sstar_sdio_free_tx_wsm(struct Sstar_common	*hw_priv,struct wsm_tx *wsm)
{
	if((wsm) && (!(wsm->htTxParameters&__cpu_to_le32(WSM_NEED_TX_CONFIRM)))){
		
		struct Sstar_queue *queue;
		u8 queue_id;
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

			wsm_release_vif_tx_buffer(hw_priv,txpriv->if_id,1);
			wsm_release_tx_buffer(hw_priv, 1);
			
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
			wsm_release_vif_tx_buffer(hw_priv,Sstar_queue_get_if_id(wsm->packetID),1);
			wsm_release_tx_buffer(hw_priv, 1);
		}
		return 1;
	}
	return 0;
}

void Sstar_sdio_tx_bh(struct Sstar_common *hw_priv)
{
#ifdef  SSTAR_WSM_SDIO_TX_MULT
	#define WSM_SDIO_TX_MULT_BLOCK_SIZE	(6*SDIO_BLOCK_SIZE)
#else
	#define WSM_SDIO_TX_MULT_BLOCK_SIZE	(SDIO_BLOCK_SIZE)
#endif
	int tx_burst;
	struct wsm_hdr_tx *wsm_tx;
	int vif_selected;
	u32 tx_len=0;
	u32 putLen=0;
	u8 *data;
	int ret=0;
	int txMutiFrameCount=0;
#if (PROJ_TYPE>=ARES_A)
	u32 wsm_flag_u32 = 0;
	u16 wsm_len_u16[2];
	u16 wsm_len_sum;
#endif	//	(PROJ_TYPE==ARES_A)	
	bool enough = false;
	
xmit_continue:

	txMutiFrameCount = 0;
	putLen = 0;
	enough = false;
	
	do {
		
		enough = Sstar_sdio_have_enough_space(hw_priv,1);
		
		if(enough == false){
			if(txMutiFrameCount > 0)
				break;
			else
				goto xmit_wait;
		}
		
		ret = wsm_get_tx(hw_priv, &data, &tx_len, &tx_burst,&vif_selected);
		
		if (ret <= 0) {
			if(txMutiFrameCount > 0)
				break;
			else
				goto xmit_finished;
		}
		
		txMutiFrameCount++;
		wsm_tx = (struct wsm_hdr_tx *)data;
		BUG_ON(tx_len < sizeof(*wsm_tx));
		BUG_ON(__le32_to_cpu(wsm_tx->len) != tx_len);
		
#if (PROJ_TYPE>=ARES_A)
		wsm_flag_u32 = (tx_len) & 0xffff;
		wsm_len_u16[0] = wsm_flag_u32 & 0xff;
		wsm_len_u16[1] = (wsm_flag_u32 >> 8)& 0xff;
		wsm_len_sum = wsm_len_u16[0] + wsm_len_u16[1];
		if (wsm_len_sum & BIT(8)){
			wsm_flag_u32 |= ((wsm_len_sum + 1) & 0xff) << 24;
		}else {
			wsm_flag_u32 |= (wsm_len_sum & 0xff) << 24;
		}
		wsm_tx->flag=__cpu_to_le32(wsm_flag_u32);
#endif //(PROJ_TYPE==ARES_A)

		if (tx_len <= 8)
			tx_len = 16;

		if (tx_len % (WSM_SDIO_TX_MULT_BLOCK_SIZE) ) {
			tx_len -= (tx_len % (WSM_SDIO_TX_MULT_BLOCK_SIZE) );
			tx_len += WSM_SDIO_TX_MULT_BLOCK_SIZE;
		}

		/* Check if not exceeding Sstar
		capabilities */
		if (WARN_ON_ONCE(tx_len > EFFECTIVE_BUF_SIZE)) {
			Sstar_printk_err("Write aligned len:"
			" %d\n", tx_len);
		}
		
#if (PROJ_TYPE<ARES_A)					
		wsm_tx->flag=(((tx_len/SDIO_BLOCK_SIZE)&0xff)-1);
#endif //(PROJ_TYPE==ARES_A)
		wsm_tx->id &= __cpu_to_le32(~WSM_TX_SEQ(WSM_TX_SEQ_MAX));
		wsm_tx->id |= cpu_to_le32(WSM_TX_SEQ(hw_priv->wsm_tx_seq));
#ifdef  SSTAR_WSM_SDIO_TX_MULT
		wsm_tx->tx_len = tx_len;
		wsm_tx->tx_id = wsm_tx->id;
#endif
		wsm_alloc_tx_buffer(hw_priv);

		spin_lock_bh(&hw_priv->tx_com_lock);
		hw_priv->n_xmits ++;
		hw_priv->hw_bufs_free --;
		BUG_ON(hw_priv->hw_bufs_free < 0);
		spin_unlock_bh(&hw_priv->tx_com_lock);
		
		Sstar_xmit_linearize(hw_priv,(struct wsm_tx *)data,&hw_priv->xmit_buff[putLen],wsm_tx->len);
		
		putLen += tx_len;
		hw_priv->wsm_tx_seq = (hw_priv->wsm_tx_seq + 1) & WSM_TX_SEQ_MAX;

		if (vif_selected != -1) {
			hw_priv->hw_bufs_used_vif[vif_selected]++;
		}
		
		if(wsm_txed(hw_priv, data)){
			break;
		}else {
			hw_priv->wsm_txframe_num++;
			if(Sstar_sdio_free_tx_wsm(hw_priv,(struct wsm_tx *)data) == 0){
				break;
			}
		}
		
		if (putLen+hw_priv->wsm_caps.sizeInpChBuf>SDIO_TX_MAXLEN){
			break;
		}
	}while(1);
	
	hw_priv->buf_id_offset = txMutiFrameCount;
	atomic_add(1, &hw_priv->bh_tx);
		
	if (WARN_ON(Sstar_data_write(hw_priv,hw_priv->xmit_buff, putLen))) {		
		Sstar_printk_err("%s: xmit data err\n",__func__);
		goto xmit_err;
	}
	
xmit_wait:	
	if((enough == false)&&(Sstar_sdio_wait_enough_space(hw_priv,1) == false)){
		Sstar_printk_err("%s: wait space timeout\n",__func__);
		goto xmit_err;
	}
	
	goto xmit_continue;
	
xmit_finished:	
	return;
xmit_err:	
	Sstar_sdio_bh_halt(hw_priv);
	return;
}
void Sstar_sdio_rx_bh(struct Sstar_common *hw_priv)
{
	static bool hard_irq = true;
	
	if(hw_priv->hard_irq == false){
		/*
		*irq bh has read the lmac packages
		*/
		struct sk_buff *skb;
		struct sk_buff_head local_list;
		hard_irq = false;
		__Sstar_skb_queue_head_init(&local_list);

		spin_lock_bh(&hw_priv->rx_frame_queue.lock);
		hw_priv->bh_running = true;
	restart:
		Sstar_skb_queue_splice_tail_init(&hw_priv->rx_frame_queue, &local_list);
		spin_unlock_bh(&hw_priv->rx_frame_queue.lock);

		while ((skb = __Sstar_skb_dequeue(&local_list)) != NULL){
			Sstar_rx_directly(hw_priv,&skb);
		}

		spin_lock_bh(&hw_priv->rx_frame_queue.lock);
		if(!Sstar_skb_queue_empty(&hw_priv->rx_frame_queue))
			goto restart;
		hw_priv->bh_running = false;
		spin_unlock_bh(&hw_priv->rx_frame_queue.lock);
	}else {
		BUG_ON(hard_irq == false);
		hw_priv->sbus_ops->lock(hw_priv->sbus_priv);
		Sstar_sdio_process_read_data(hw_priv,Sstar_bh_read_ctrl_reg_unlock,Sstar_data_read_unlock,true);
		hw_priv->sbus_ops->unlock(hw_priv->sbus_priv);
#ifdef CONFIG_SSTAR_APOLLO_USE_GPIO_IRQ
		Sstar_oob_intr_set(hw_priv->sbus_priv,true);
#endif
	}
}

void Sstar_irq_handler(struct Sstar_common *hw_priv)
{
	/* To force the device to be always-on, the host sets WLAN_UP to 1 */
	if(!hw_priv->init_done){
		Sstar_printk_err("[BH] irq. init_done =0 drop\n");
		return;
	}

	if (Sstar_bh_is_term(hw_priv))
		return;

	if(!in_interrupt())	{
		/*
		*not in interrput ,read data directly
		*/
		hw_priv->hard_irq = false;
#ifdef CONFIG_SSTAR_APOLLO_USE_GPIO_IRQ
		hw_priv->sbus_ops->lock(hw_priv->sbus_priv);
#endif
		Sstar_sdio_process_read_data(hw_priv,Sstar_bh_read_ctrl_reg_unlock,Sstar_data_read_unlock,false);
		if (Sstar_bh_is_term(hw_priv) || atomic_read(&hw_priv->bh_term)){
			hw_priv->bh_error = 1;
			wake_up(&hw_priv->bh_wq);
		}
		
#ifdef CONFIG_SSTAR_APOLLO_USE_GPIO_IRQ
		hw_priv->sbus_ops->unlock(hw_priv->sbus_priv);		
#endif

		return;
	}else {
		/*
		*handle irq in interupt
		*/
		hw_priv->hard_irq = true;
	}
	
	if(hw_priv->sbus_ops->sbus_rev_schedule)
		hw_priv->sbus_ops->sbus_rev_schedule(hw_priv->sbus_priv);
	else if (atomic_add_return(1, &hw_priv->bh_rx) == 1){
		wake_up(&hw_priv->bh_wq);
	}
}

int Sstar_bh_suspend(struct Sstar_common *hw_priv)
{
	int i =0;
	struct Sstar_vif *priv = NULL;
	Sstar_printk_pm("[BH] try suspend.\n");
	if (hw_priv->bh_error) {
		wiphy_warn(hw_priv->hw->wiphy, "BH error -- can't suspend\n");
		return -EINVAL;
	}
#ifdef MCAST_FWDING

 	Sstar_for_each_vif(hw_priv, priv, i) {
		if (!priv)
			continue;
		if ( (priv->multicast_filter.enable)
			&& (priv->join_status == SSTAR_APOLLO_JOIN_STATUS_AP) ) {
			wsm_release_buffer_to_fw(priv,
				(hw_priv->wsm_caps.numInpChBufs - 1));
			break;
		}
	}
#endif
	atomic_set(&hw_priv->bh_suspend, SSTAR_APOLLO_BH_SUSPEND);
	wake_up(&hw_priv->bh_wq);
	return Sstar_wait_event_timeout_stay_awake(hw_priv,hw_priv->bh_evt_wq, hw_priv->bh_error ||
		(SSTAR_APOLLO_BH_SUSPENDED == atomic_read(&hw_priv->bh_suspend)),
		 60 * HZ,false) ? 0 : -ETIMEDOUT;
}

int Sstar_bh_resume(struct Sstar_common *hw_priv)
{
int i =0;
#ifdef MCAST_FWDING
	int ret;
	struct Sstar_vif *priv =NULL;
#endif

	Sstar_printk_pm("[BH] try resume.\n");
	if (hw_priv->bh_error) {
		wiphy_warn(hw_priv->hw->wiphy, "BH error -- can't resume\n");
		return -EINVAL;
	}

	atomic_set(&hw_priv->bh_suspend, SSTAR_APOLLO_BH_RESUME);
	wake_up(&hw_priv->bh_wq);
    Sstar_printk_pm("wakeup bh,wait evt_wq\n");
#ifdef MCAST_FWDING
	ret = Sstar_wait_event_timeout_stay_awake(hw_priv,hw_priv->bh_evt_wq, hw_priv->bh_error ||
				(SSTAR_APOLLO_BH_RESUMED == atomic_read(&hw_priv->bh_suspend)),
				1 * HZ,false) ? 0 : -ETIMEDOUT;

	Sstar_for_each_vif(hw_priv, priv, i) {
		if (!priv)
			continue;
		if ((priv->join_status == SSTAR_APOLLO_JOIN_STATUS_AP)
				&& (priv->multicast_filter.enable)) {
			u8 count = 0;
			WARN_ON(wsm_request_buffer_request(priv, &count));
			bh_printk(
				"[BH] BH resume. Reclaim Buff %d \n",count);
			break;
		}
	}

	return ret;
#else
	return Sstar_wait_event_timeout_stay_awake(hw_priv,hw_priv->bh_evt_wq,hw_priv->bh_error ||
		(SSTAR_APOLLO_BH_RESUMED == atomic_read(&hw_priv->bh_suspend)),
		100 * HZ,false) ? 0 : -ETIMEDOUT;
#endif

}

void wsm_alloc_tx_buffer(struct Sstar_common *hw_priv)
{
	spin_lock_bh(&hw_priv->tx_com_lock);
	++hw_priv->hw_bufs_used;
	spin_unlock_bh(&hw_priv->tx_com_lock);
}

int wsm_release_tx_buffer(struct Sstar_common *hw_priv, int count)
{
	int ret = 0;
	int hw_bufs_used;
	spin_lock_bh(&hw_priv->tx_com_lock);
	hw_bufs_used = hw_priv->hw_bufs_used;
	hw_priv->hw_bufs_used -= count;

	if (WARN_ON(hw_priv->hw_bufs_used < 0))
		//ret = -1;
		hw_priv->hw_bufs_used=0;
	/* Tx data patch stops when all but one hw buffers are used.
	   So, re-start tx path in case we find hw_bufs_used equals
	   numInputChBufs - 1.
	 */
	else if (hw_bufs_used >= (hw_priv->wsm_caps.numInpChBufs - 1))
		ret = 1;
	spin_unlock_bh(&hw_priv->tx_com_lock);
	if (!(hw_priv->hw_bufs_used))
		wake_up(&hw_priv->bh_evt_wq);
	return ret;
}

#ifdef MCAST_FWDING
#ifndef USB_BUS

//just for sdio
int wsm_release_buffer_to_fw(struct Sstar_vif *priv, int count)
{
	int i;
	u8 flags;
	struct wsm_buf *buf;
	u32 buf_len;
	struct wsm_hdr_tx *wsm;
	struct Sstar_common *hw_priv = priv->hw_priv;


	if (priv->join_status != SSTAR_APOLLO_JOIN_STATUS_AP) {
		return 0;
	}

	bh_printk( "Rel buffer to FW %d, %d\n", count, hw_priv->hw_bufs_used);

	for (i = 0; i < count; i++) {
		if ((hw_priv->hw_bufs_used + 1) < hw_priv->wsm_caps.numInpChBufs) {
			flags = i ? 0: 0x1;

			wsm_alloc_tx_buffer(hw_priv);

			buf = &hw_priv->wsm_release_buf[i];
			buf_len = buf->data - buf->begin;

			/* Add sequence number */
			wsm = (struct wsm_hdr_tx *)buf->begin;
			BUG_ON(buf_len < sizeof(*wsm));

			wsm->id &= __cpu_to_le32(
				~WSM_TX_SEQ(WSM_TX_SEQ_MAX));
			wsm->id |= cpu_to_le32(
				WSM_TX_SEQ(hw_priv->wsm_tx_seq));

			Sstar_printk_bh("REL %d\n", hw_priv->wsm_tx_seq);

			if (WARN_ON(Sstar_data_write(hw_priv,
				buf->begin, buf_len))) {
				break;
			}


			hw_priv->buf_released = 1;
			hw_priv->wsm_tx_seq = (hw_priv->wsm_tx_seq + 1) & WSM_TX_SEQ_MAX;
		} else
			break;
	}

	if (i == count) {
		return 0;
	}

	/* Should not be here */
	Sstar_printk_err("[BH] Less HW buf %d,%d.\n", hw_priv->hw_bufs_used,
			hw_priv->wsm_caps.numInpChBufs);
	WARN_ON(1);

	return -1;
}
#endif //USB_BUS
#endif
static struct sk_buff *Sstar_get_skb(struct Sstar_common *hw_priv, u32 len)
{
	struct sk_buff *skb;
	u32 alloc_len = (len > SDIO_BLOCK_SIZE) ? len : SDIO_BLOCK_SIZE;

	if (len > SDIO_BLOCK_SIZE || !hw_priv->skb_cache) {
		skb = __Sstar_dev_alloc_skb(alloc_len
				+ WSM_TX_EXTRA_HEADROOM
				+ 8  /* TKIP IV */
				+ 12 /* TKIP ICV + MIC */
				- 2  /* Piggyback */,GFP_KERNEL);
		BUG_ON(skb==NULL);
		/* In AP mode RXed SKB can be looped back as a broadcast.
		 * Here we reserve enough space for headers. */
		Sstar_skb_reserve(skb, WSM_TX_EXTRA_HEADROOM
				+ 8 /* TKIP IV */
				- WSM_RX_EXTRA_HEADROOM);
	} else {
		skb = hw_priv->skb_cache;
		hw_priv->skb_cache = NULL;
	}
	return skb;
}


void Sstar_put_skb(struct Sstar_common *hw_priv, struct sk_buff *skb)
{
	if (hw_priv->skb_cache){
		//printk("%s Sstar_kfree_skb skb=%p\n",__func__,skb);
		Sstar_dev_kfree_skb(skb);
	}
	else
		hw_priv->skb_cache = skb;
}

int Sstar_bh_read_ctrl_reg(struct Sstar_common *hw_priv,
					  u16 *ctrl_reg)
{
	int ret=0,retry=0;
	while (retry <= MAX_RETRY) {
		ret = Sstar_reg_read_16(hw_priv,
				SSTAR_HIFREG_CONTROL_REG_ID, ctrl_reg);
		if(!ret){
				break;
		}else{
			/*reset sdio internel reg by send cmd52 to abort*/
			WARN_ON(hw_priv->sbus_ops->abort(hw_priv->sbus_priv));
			retry++;
			mdelay(retry);
			Sstar_printk_err(
				"[BH] Failed to read control register.ret=%x\n",ret);
		}
	}
	return ret;
}
int Sstar_bh_read_ctrl_reg_unlock(struct Sstar_common *hw_priv,
					  u16 *ctrl_reg)
{
	int ret=0,retry=0;
	while (retry <= MAX_RETRY) {
		ret = Sstar_reg_read_16_unlock(hw_priv,
				SSTAR_HIFREG_CONTROL_REG_ID, ctrl_reg);
		if(!ret){
				break;
		}else{
			/*reset sdio internel reg by send cmd52 to abort*/
			WARN_ON(hw_priv->sbus_ops->abort(hw_priv->sbus_priv));
			retry++;
			mdelay(retry);
			Sstar_printk_err(
				"[BH] Failed to read control register.ret=%x\n",ret);
		}
	}
	return ret;
}

//just ARESB have this function
//used this function to clear sdio rtl bug register
// if not do this sdio direct mode (wr/read reigster) will not work
// this function is the same to Sstar_data_force_write (used queue mode clear bit to clear)
// 
int Sstar_powerave_sdio_sync(struct Sstar_common *hw_priv)
{
	int ret=0;
	//int retry=0;
	u32 config_reg;
	ret = Sstar_reg_read_unlock_32(hw_priv, SSTAR_HIFREG_CONFIG_REG_ID, &config_reg);
	if (ret < 0) {
		Sstar_printk_err("%s: enable_irq: can't read config register.\n", __func__);
	}

	if(config_reg & SSTAR_HIFREG_PS_SYNC_SDIO_FLAG)
	{
		Sstar_printk_err("%s:%d\n",__func__,__LINE__);
		//Sstar_hw_buff_reset(hw_priv);
		config_reg |= SSTAR_HIFREG_PS_SYNC_SDIO_CLEAN;
		Sstar_reg_write_unlock_32(hw_priv,SSTAR_HIFREG_CONFIG_REG_ID,config_reg);
	}
	return ret;
}
int Sstar_device_wakeup(struct Sstar_common *hw_priv)
{
	u16 ctrl_reg;
	int ret=0;
	int loop = 1;

#ifdef PS_SETUP

	/* To force the device to be always-on, the host sets WLAN_UP to 1 */
	ret = Sstar_reg_write_16(hw_priv, SSTAR_HIFREG_CONTROL_REG_ID,
			SSTAR_HIFREG_CONT_WUP_BIT);
	if (WARN_ON(ret))
		return ret;
#endif
	while(1){
		mdelay(5);
		ret = Sstar_bh_read_ctrl_reg(hw_priv, &ctrl_reg);
		if (WARN_ON(ret)){
		}
		/* If the device returns WLAN_RDY as 1, the device is active and will
		 * remain active. */
		Sstar_printk_bh("Rdy =%x\n",ctrl_reg);
		if (ctrl_reg & SSTAR_HIFREG_CONT_RDY_BIT) {
			Sstar_printk_bh("[BH] Device awake.<%d>\n",loop);
			ret= 1;
			break;
		}
	}
	return ret;
}
int rxMutiCnt[17]={0};
int rxCnt=0;
int rxMutiCnt_Num;
unsigned long g_printf_count = 0;
int Sstar_rx_tasklet(struct Sstar_common *hw_priv, int id,
		  struct wsm_hdr *wsm, struct sk_buff **skb_p)
{
	struct sk_buff *skb = *skb_p;
	struct sk_buff *Sstar_skb_copy;
	//struct wsm_hdr *wsm;
	u32 wsm_len;
	int wsm_id;
	int data_len;
	int ret=0;
#define RX_ALLOC_BUFF_OFFLOAD (  (40+16)/*RX_DESC_OVERHEAD*/ -16 /*WSM_HI_RX_IND*/)

		wsm_len = __le32_to_cpu(wsm->len);
		wsm_id	= __le32_to_cpu(wsm->id) & 0xFFF;
		if((wsm_id == WSM_MULTI_RECEIVE_INDICATION_ID)||
			(WSM_SINGLE_CHANNEL_MULTI_RECEIVE_INDICATION_ID == wsm_id)){
			struct wsm_multi_rx *  multi_rx = (struct wsm_multi_rx *)skb->data;			
			int RxFrameNum = multi_rx->RxFrameNum;
			
			data_len = wsm_len ;
			data_len -= sizeof(struct wsm_multi_rx);
			
			rxMutiCnt[ALIGN(wsm_len,1024)/1024]++;
			rxMutiCnt_Num+=RxFrameNum;
			
			wsm = (struct wsm_hdr *)(multi_rx+1);
			wsm_len = __le32_to_cpu(wsm->len);
			wsm_id	= __le32_to_cpu(wsm->id) & 0xFFF;
			
			//frame_hexdump("dump sdio wsm rx ->",wsm,32);
			do {

				if(data_len < wsm_len){
					Sstar_printk_err("skb->len %x,wsm_len %x data_len %x\n",skb->len,wsm_len,data_len);
					//frame_hexdump("dump sdio wsm rx ->",skb->data,64);
					break;
				}
				WARN_ON((wsm_id  & (~WSM_TX_LINK_ID(WSM_TX_LINK_ID_MAX))) !=  WSM_RECEIVE_INDICATION_ID);
				Sstar_skb_copy = __Sstar_dev_alloc_skb(wsm_len + 16,GFP_KERNEL);
				BUG_ON(Sstar_skb_copy == NULL);
				Sstar_skb_reserve(Sstar_skb_copy,  (8 - (((unsigned long)Sstar_skb_copy->data)&7))/*ALIGN 8*/);
				Sstar_skb_copy->pkt_type = skb->pkt_type;
				memmove(Sstar_skb_copy->data, wsm, wsm_len);
				Sstar_skb_put(Sstar_skb_copy,wsm_len);
				ret=wsm_handle_rx(hw_priv,wsm_id,wsm,&Sstar_skb_copy);
				if(ret){
					rxMutiCnt_Num=0;
					memset(rxMutiCnt,0,sizeof(rxMutiCnt));
					if(Sstar_skb_copy != NULL){
						Sstar_dev_kfree_skb(Sstar_skb_copy);
					}
					return ret;
				}
				data_len -= ALIGN(wsm_len + RX_ALLOC_BUFF_OFFLOAD,4);
				RxFrameNum--;

				wsm = (struct wsm_hdr *)((u8 *)wsm +ALIGN(( wsm_len + RX_ALLOC_BUFF_OFFLOAD),4));
				wsm_len = __le32_to_cpu(wsm->len);
				wsm_id	= __le32_to_cpu(wsm->id) & 0xFFF;
				
				if(Sstar_skb_copy != NULL){
					Sstar_dev_kfree_skb(Sstar_skb_copy);
				}
			}while((RxFrameNum>0) && (data_len > 32));
			BUG_ON(RxFrameNum != 0);
			
		}
		else {
			//rxMutiCnt[ALIGN(wsm_len,1024)/1024]++;
			rxCnt++;
			ret=wsm_handle_rx(hw_priv,id,wsm,skb_p);
		}
		return ret;

}
//#define DEBUG_SDIO
#ifdef SSTAR_SDIO_PATCH
#define CHECKSUM_LEN 4
static u32 GloId_array[64]={0};
u16 Sstar_CalCheckSum(const u8 *data,u16 len)
{
  u16 t;
  const u8 *dataptr;
  const u8 *last_byte;
  u16 sum = 0;
  dataptr = data;
  last_byte = data + len - 1;
  while(dataptr < last_byte) {	/* At least two more bytes */
    t = (dataptr[0] << 8) + dataptr[1];
    sum += t;
    if(sum < t) {
      sum++;		/* carry */
    }
    dataptr += 2;
  }
  
  if(dataptr == last_byte) {
    t = (dataptr[0] << 8) + 0;
    sum += t;
    if(sum < t) {
      sum++;		/* carry */
    }
  }
  sum=(~sum)&0xffff;

  /* Return sum in host byte order. */
  return sum;
}
void Sstar_packetId_to_seq(struct Sstar_common *hw_priv,u32 packetId)
{
	GloId_array[hw_priv->SdioSeq]=packetId;
}
int Sstar_seq_to_packetId(struct Sstar_common *hw_priv,u32 seq)
{
	u32 packetId;
	packetId=GloId_array[seq];
	GloId_array[seq]=0;
	return packetId;
}
#endif //SSTAR_SDIO_PATCH
static int Sstar_bh(void *arg)
{
	struct Sstar_common *hw_priv = arg;
	struct Sstar_vif *priv = NULL;
	int rx, tx=0, term, suspend;
	u16 ctrl_reg = 0;
	int pending_tx = 0;
	long status;
	bool powersave_enabled;
	int i;
	int ret_flush;				

	
#define __ALL_HW_BUFS_USED (hw_priv->hw_bufs_used)
	while (1) {
		powersave_enabled = 1;
		Sstar_hw_vif_read_lock(&hw_priv->vif_list_lock);
		Sstar_for_each_vif_safe(hw_priv, priv, i) 
		{
			if (!priv)
				continue;
			powersave_enabled &= !!priv->powersave_enabled;
		}
		Sstar_hw_vif_read_unlock(&hw_priv->vif_list_lock);
		if (!__ALL_HW_BUFS_USED
				&& powersave_enabled
				&& !hw_priv->device_can_sleep
				&& !atomic_read(&hw_priv->recent_scan)) {
			status = HZ/8;
			bh_printk( "[BH] No Device wakedown.\n");
#ifdef PS_SETUP
			WARN_ON(Sstar_reg_write_16(hw_priv,
						SSTAR_HIFREG_CONTROL_REG_ID, 0));
			hw_priv->device_can_sleep = true;
#endif
		} else if (__ALL_HW_BUFS_USED)
			/* Interrupt loss detection */
			status = HZ;
		else
			status = HZ/4;
		/* If a packet has already been txed to the device then read the
		   control register for a probable interrupt miss before going
		   further to wait for interrupt; if the read length is non-zero
		   then it means there is some data to be received */
		status = wait_event_interruptible_timeout(hw_priv->bh_wq, ({
				rx = atomic_xchg(&hw_priv->bh_rx, 0);
				tx = atomic_xchg(&hw_priv->bh_tx, 0);
				term = atomic_xchg(&hw_priv->bh_term, 0);
				suspend = pending_tx ?
					0 : atomic_read(&hw_priv->bh_suspend);
				(rx || tx || term || suspend || hw_priv->bh_error || atomic_read(&hw_priv->bh_halt));
			}), status);
		
		if (status < 0 || term || hw_priv->bh_error){
			Sstar_bh_read_ctrl_reg(hw_priv, &ctrl_reg);
			//printk(" ++ctrl_reg= %x,\n",ctrl_reg);
			Sstar_printk_err("%s BH thread break %ld %d %d ctrl_reg=%x\n",__func__,status,term,hw_priv->bh_error,ctrl_reg);
			break;
		}

		if(atomic_read(&hw_priv->bh_halt)){
			atomic_set(&hw_priv->Sstar_pluged,0);
			Sstar_printk_err("%s:bh_halt\n",__func__);
			if(hw_priv->sbus_ops->lmac_restart(hw_priv->sbus_priv) != 0){
				atomic_xchg(&hw_priv->bh_halt,0);
				hw_priv->bh_error = 1;
				break;
			}
		}
		
		if (!status && __ALL_HW_BUFS_USED)
		{
			unsigned long timestamp = jiffies;
			long timeout;
			bool pending = false;
			int i;
			
			if(time_is_after_jiffies(hw_priv->irq_timestamp+msecs_to_jiffies(500))){
				continue;
			}

			Sstar_bh_read_ctrl_reg(hw_priv, &ctrl_reg);
			
			if(ctrl_reg & SSTAR_HIFREG_CONT_NEXT_LEN_MASK)
			{
				int ret;
				ret=Sstar_direct_write_reg_32(hw_priv,0xac8029c,0x7ff);
				if(ret){
					Sstar_printk_err("0xac8029c Error\n");
				}
				Sstar_printk_err("MISS 1\n");

				
				hw_priv->sbus_ops->lock(hw_priv->sbus_priv);
				Sstar_sdio_process_read_data(hw_priv,Sstar_bh_read_ctrl_reg_unlock,Sstar_data_read_unlock,hw_priv->hard_irq == true);
				hw_priv->sbus_ops->unlock(hw_priv->sbus_priv);

				continue;
			}
			wiphy_warn(hw_priv->hw->wiphy, "Missed interrupt Status =%d, buffused=%d\n",(int)status,(int)__ALL_HW_BUFS_USED);
			rx = 1;
			Sstar_printk_err("[bh] next wsm_rx_seq %d wsm_tx_seq %d\n",hw_priv->wsm_rx_seq,hw_priv->wsm_tx_seq);
			Sstar_printk_err("[bh] wsm_hiftx_cmd_num %d wsm_hif_cmd_conf_num %d\n",hw_priv->wsm_hiftx_num,hw_priv->wsm_hifconfirm_num);
			Sstar_printk_err("[bh] wsm_txframe_num %d wsm_txconfirm_num %d\n",hw_priv->wsm_txframe_num,hw_priv->wsm_txconfirm_num);
			Sstar_printk_err("[bh] num_pending[0]=%d num_pending[1]=%d pending[2]=%d pending[3]=%d\n",
															hw_priv->tx_queue[0].num_pending,
															hw_priv->tx_queue[1].num_pending,
															hw_priv->tx_queue[2].num_pending,
															hw_priv->tx_queue[3].num_pending);
			//Sstar_monitor_pc(hw_priv);

			Sstar_bh_read_ctrl_reg(hw_priv, &ctrl_reg);
			Sstar_printk_err(" ++ctrl_reg= %x,\n",ctrl_reg);

			/* Get a timestamp of "oldest" frame */
			for (i = 0; i < 4; ++i)
				pending |= Sstar_queue_get_xmit_timestamp(
						&hw_priv->tx_queue[i],
						&timestamp, -1,
						hw_priv->pending_frame_id);

			/* Check if frame transmission is timed out.
			 * Add an extra second with respect to possible
			 * interrupt loss. */
			timeout = timestamp +
					WSM_CMD_LAST_CHANCE_TIMEOUT +
					1 * HZ  -
					jiffies;

			/* And terminate BH tread if the frame is "stuck" */
			if (pending && timeout < 0) {
				ret_flush=wsm_sync_channle_process(hw_priv,IN_BH);
				if(ret_flush==RECOVERY_ERR){
					Sstar_printk_err("RESET CHANN ERR %d\n",__LINE__);
					break;
				}else{
					ctrl_reg=0;
					continue;
				}
			}
		} else if (!status) {
			if (!hw_priv->device_can_sleep
					&& !atomic_read(&hw_priv->recent_scan)) {
			        bh_printk(KERN_ERR "[BH] Device wakedown. Timeout.\n");
#ifdef PS_SETUP
				WARN_ON(Sstar_reg_write_16(hw_priv,
						SSTAR_HIFREG_CONTROL_REG_ID, 0));
				hw_priv->device_can_sleep = true;
#endif//#ifdef PS_SETUP
			}
			continue;
		} else if (suspend) {
			Sstar_printk_err("[BH] Device suspend.\n");
			powersave_enabled = 1;
			//if in recovery clear reg
			if(hw_priv->syncChanl_done==0){
			     ctrl_reg=0;
			}
			Sstar_hw_vif_read_lock(&hw_priv->vif_list_lock);
			Sstar_for_each_vif_safe(hw_priv, priv, i) {
				if (!priv)
					continue;
				powersave_enabled &= !!priv->powersave_enabled;
			}
			Sstar_hw_vif_read_unlock(&hw_priv->vif_list_lock);
			if (powersave_enabled) {
				bh_printk( "[BH] No Device wakedown. Suspend.\n");
			}

			atomic_set(&hw_priv->bh_suspend, SSTAR_APOLLO_BH_SUSPENDED);
			wake_up(&hw_priv->bh_evt_wq);
			Sstar_printk_err("[BH] wait resume..\n");
			status = wait_event_interruptible(hw_priv->bh_wq,
					SSTAR_APOLLO_BH_RESUME == atomic_read(
						&hw_priv->bh_suspend));
			if (status < 0) {
				wiphy_err(hw_priv->hw->wiphy,
					"%s: Failed to wait for resume: %ld.\n",
					__func__, status);
				break;
			}
			Sstar_printk_err("[BH] Device resume.\n");
			atomic_set(&hw_priv->bh_suspend, SSTAR_APOLLO_BH_RESUMED);
			wake_up(&hw_priv->bh_evt_wq);
			atomic_add(1, &hw_priv->bh_rx);
			continue;
		}
	}

	if (!term) {
		int loop = 3;
		Sstar_dbg(SSTAR_APOLLO_DBG_ERROR, "[BH] Fatal error, exitting.\n");
		hw_priv->bh_error = 1;
		while(loop-->0){
			Sstar_monitor_pc(hw_priv);
			msleep(10);
		}

		hw_priv->sbus_ops->wtd_wakeup(hw_priv->sbus_priv);
		Sstar_hw_vif_read_lock(&hw_priv->vif_list_lock);
		Sstar_for_each_vif_safe(hw_priv, priv, i) {
			if (!priv)
				continue;
//			ieee80211_driver_hang_notify(priv->vif, GFP_KERNEL);
		}
		Sstar_hw_vif_read_unlock(&hw_priv->vif_list_lock);
		#ifdef CONFIG_PM
		Sstar_pm_stay_awake(&hw_priv->pm_state, 3*HZ);
		#endif
		/* TODO: schedule_work(recovery) */
#ifndef HAS_PUT_TASK_STRUCT
		/* The only reason of having this stupid code here is
		 * that __put_task_struct is not exported by kernel. */
		for (;;) {
			int status = wait_event_interruptible(hw_priv->bh_wq, ({
				term = atomic_xchg(&hw_priv->bh_term, 0);
				(term);
				}));

			if (status || term)
				break;
		}
#endif
	}
	Sstar_printk_exit("Sstar_wifi_BH_thread stop ++\n");
	/*
	add this code just because 'linux kernel' need kthread not exit ,
	before kthread_stop func call,
	*/
	if(term)
	{
		//clear pendding cmd
		if(!mutex_trylock(&hw_priv->wsm_cmd_mux))
		{
			spin_lock_bh(&hw_priv->wsm_cmd.lock);
			if(hw_priv->wsm_cmd.cmd != 0xFFFF)
			{
				hw_priv->wsm_cmd.ret = -1;
				hw_priv->wsm_cmd.done = 1;
				spin_unlock_bh(&hw_priv->wsm_cmd.lock);
				Sstar_printk_exit("cancle current pendding cmd,release wsm_cmd.lock\n");
				wake_up(&hw_priv->wsm_cmd_wq);
				msleep(2);
				spin_lock_bh(&hw_priv->wsm_cmd.lock);
			}
			spin_unlock_bh(&hw_priv->wsm_cmd.lock);
		}
		else
		{
			mutex_unlock(&hw_priv->wsm_cmd_mux);
		}

		/*
		*cancle the current scanning process
		*/
		mutex_lock(&hw_priv->conf_mutex);
		if(atomic_read(&hw_priv->scan.in_progress))
		{
			struct Sstar_vif *scan_priv = ABwifi_hwpriv_to_vifpriv(hw_priv,
						hw_priv->scan.if_id);
			bool scanto_running = false;
			Sstar_priv_vif_list_read_unlock(&scan_priv->vif_lock);
			mutex_unlock(&hw_priv->conf_mutex);
			scanto_running = Sstar_cancle_delayed_work(&hw_priv->scan.timeout,true);
			mutex_lock(&hw_priv->conf_mutex);
			if(scanto_running>0)
			{
				hw_priv->scan.curr = hw_priv->scan.end;
				mutex_unlock(&hw_priv->conf_mutex);
				Sstar_scan_timeout(&hw_priv->scan.timeout.work);
				mutex_lock(&hw_priv->conf_mutex);
			}
		}
		mutex_unlock(&hw_priv->conf_mutex);
		{
			u8 i = 0;
			//cancel pendding work
			#define SSTAR_CANCEL_PENDDING_WORK(work,work_func)			\
				do{														\
					if(Sstar_cancle_queue_work(work,true)==true)			\
					{													\
						work_func(work);								\
					}													\
				}														\
				while(0)
					
			if(Sstar_cancle_delayed_work(&hw_priv->scan.probe_work,true))
				Sstar_probe_work(&hw_priv->scan.probe_work.work);
			if(Sstar_cancle_delayed_work(&hw_priv->rem_chan_timeout,true))
				Sstar_rem_chan_timeout(&hw_priv->rem_chan_timeout.work);
			SSTAR_CANCEL_PENDDING_WORK(&hw_priv->scan.work,Sstar_scan_work);
			SSTAR_CANCEL_PENDDING_WORK(&hw_priv->event_handler,Sstar_event_handler);
#ifdef CONFIG_SSTAR_BA_STATUS
			SSTAR_CANCEL_PENDDING_WORK(&hw_priv->ba_work,Sstar_ba_work);
#endif
			#ifdef SSTAR_SUPPORT_WIDTH_40M
			SSTAR_CANCEL_PENDDING_WORK(&hw_priv->get_cca_work,Sstar_get_cca_work);
			del_timer_sync(&hw_priv->chantype_timer);
			#endif
			#ifdef SSTAR_SUPPORT_SMARTCONFIG
			SSTAR_CANCEL_PENDDING_WORK(&hw_priv->scan.smartwork,Sstar_smart_scan_work);
			SSTAR_CANCEL_PENDDING_WORK(&hw_priv->scan.smartsetChanwork,Sstar_smart_setchan_work);
			SSTAR_CANCEL_PENDDING_WORK(&hw_priv->scan.smartstopwork,Sstar_smart_stop_work);
			del_timer_sync(&hw_priv->smartconfig_expire_timer);
			#endif
#ifdef CONFIG_SSTAR_BA_STATUS
			del_timer_sync(&hw_priv->ba_timer);
#endif
			SSTAR_CANCEL_PENDDING_WORK(&hw_priv->tx_policy_upload_work,tx_policy_upload_work);
			Sstar_for_each_vif(hw_priv, priv, i) {
				if(priv == NULL)
					continue;
				SSTAR_CANCEL_PENDDING_WORK(&priv->wep_key_work,Sstar_wep_key_work);
				SSTAR_CANCEL_PENDDING_WORK(&priv->join_work,Sstar_join_work);
				SSTAR_CANCEL_PENDDING_WORK(&priv->unjoin_work,Sstar_unjoin_work);
				SSTAR_CANCEL_PENDDING_WORK(&priv->offchannel_work,Sstar_offchannel_work);
				SSTAR_CANCEL_PENDDING_WORK(&priv->link_id_work,Sstar_link_id_work);
				SSTAR_CANCEL_PENDDING_WORK(&priv->tx_failure_work,Sstar_tx_failure_work);
				SSTAR_CANCEL_PENDDING_WORK(&priv->set_tim_work, Sstar_set_tim_work);
				SSTAR_CANCEL_PENDDING_WORK(&priv->multicast_start_work,Sstar_multicast_start_work);
				SSTAR_CANCEL_PENDDING_WORK(&priv->multicast_stop_work, Sstar_multicast_stop_work);
				SSTAR_CANCEL_PENDDING_WORK(&priv->linkid_reset_work, Sstar_link_id_reset);
				SSTAR_CANCEL_PENDDING_WORK(&priv->update_filtering_work, Sstar_update_filtering_work);
				SSTAR_CANCEL_PENDDING_WORK(&priv->set_beacon_wakeup_period_work,
											Sstar_set_beacon_wakeup_period_work);
				SSTAR_CANCEL_PENDDING_WORK(&priv->ht_info_update_work, Sstar_ht_info_update_work);
				#ifdef SSTAR_SUPPORT_WIDTH_40M
				//SSTAR_CANCEL_PENDDING_WORK(&priv->chantype_change_work, Sstar_channel_type_change_work);
				if(Sstar_cancle_delayed_work(&priv->chantype_change_work,true))
					Sstar_channel_type_change_work(&priv->chantype_change_work.work);
				#endif
				
				Sstar_cancle_delayed_work(&priv->dhcp_retry_work,true);					
				if(Sstar_cancle_delayed_work(&priv->bss_loss_work,true))
					Sstar_bss_loss_work(&priv->bss_loss_work.work);
				if(Sstar_cancle_delayed_work(&priv->connection_loss_work,true))
					Sstar_connection_loss_work(&priv->connection_loss_work.work);
				if(Sstar_cancle_delayed_work(&priv->set_cts_work,true))
					Sstar_set_cts_work(&priv->set_cts_work.work);
				if(Sstar_cancle_delayed_work(&priv->link_id_gc_work,true))
					Sstar_link_id_gc_work(&priv->link_id_gc_work.work);
				if(Sstar_cancle_delayed_work(&priv->pending_offchanneltx_work,true))
					Sstar_pending_offchanneltx_work(&priv->pending_offchanneltx_work.work);
				if(Sstar_cancle_delayed_work(&priv->join_timeout,true))
					Sstar_join_timeout(&priv->join_timeout.work);	
				del_timer_sync(&priv->mcast_timeout);
				#ifdef SSTAR_PRIVATE_IE
				del_timer_sync(&priv->channel_timer);
				SSTAR_CANCEL_PENDDING_WORK(&priv->set_channel_work,Sstar_set_channel_work);
				#endif
			}
		}
	}
	while(term){
		if(kthread_should_stop()){
			break;
		}
		schedule_timeout_uninterruptible(msecs_to_jiffies(100));
	}
	Sstar_printk_exit("Sstar_wifi_BH_thread stop --\n");
	return 0;
}
