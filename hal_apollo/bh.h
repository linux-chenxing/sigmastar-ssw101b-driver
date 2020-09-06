/*
 * Device handling thread interface for mac80211 sigmastar APOLLO drivers
 *
 * Copyright (c) 2016, sigmastar
 * Author:
 *
 * Copyright (c) 2010, ST-Ericsson
 * Author: Dmitry Tarnyagin <dmitry.tarnyagin@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef SSTAR_APOLLO_BH_H
#define SSTAR_APOLLO_BH_H

/* extern */ struct Sstar_common;

#define SDIO_BLOCK_SIZE 256

/* Suspend state privates */
enum Sstar_bh_pm_state {
        SSTAR_APOLLO_BH_RESUMED = 0,
        SSTAR_APOLLO_BH_SUSPEND,
        SSTAR_APOLLO_BH_SUSPENDED,
        SSTAR_APOLLO_BH_RESUME,
};
enum Sstar_rx_frame_type{
	SSTAR_RX_RAW_FRAME = 1,
	SSTAR_RX_DERICTLY_DATA_FRAME,
	SSTAR_RX_SLOW_MGMT_FRAME,
	SSTAR_RX_WSM_CMD_FRAME,
	SSTAR_RX_WSM_DATA_FRAME,
};
#include "bh_usb.h"

int Sstar_register_bh(struct Sstar_common *hw_priv);
void Sstar_unregister_bh(struct Sstar_common *hw_priv);
void Sstar_irq_handler(struct Sstar_common *hw_priv);
void Sstar_bh_wakeup(struct Sstar_common *hw_priv);
int Sstar_bh_suspend(struct Sstar_common *hw_priv);
int Sstar_bh_resume(struct Sstar_common *hw_priv);
/* Must be called from BH thread. */
void Sstar_enable_powersave(struct Sstar_vif *priv,
			     bool enable);
int wsm_release_tx_buffer(struct Sstar_common *hw_priv, int count);
void wsm_alloc_tx_buffer(struct Sstar_common *hw_priv);
int wsm_release_vif_tx_buffer(struct Sstar_common *hw_priv, int if_id,
				int count);
void Sstar_put_skb(struct Sstar_common *hw_priv, struct sk_buff *skb);

int Sstar_powerave_sdio_sync(struct Sstar_common *hw_priv);
int Sstar_device_wakeup(struct Sstar_common *hw_priv);
void Sstar_get_cca_work(struct work_struct *work);
#ifdef SSTAR_SDIO_PATCH
u16 Sstar_CalCheckSum(const u8 *data,u16 len);
void Sstar_packetId_to_seq(struct Sstar_common *hw_priv,u32 packetId);
int Sstar_seq_to_packetId(struct Sstar_common *hw_priv,u32 seq);
#endif
#ifdef SSTAR_PRIVATE_IE
void Sstar_set_channel_work(struct work_struct *work);
void Sstar_channel_timer(unsigned long arg);
#endif
static inline int Sstar_bh_is_term(struct Sstar_common *hw_priv){
	if((hw_priv->bh_thread==NULL) || (hw_priv->bh_error==1)||(atomic_read(&hw_priv->Sstar_pluged)==0)){
		return 1;
	}
	else {
		return 0;
	}
}
#define can_not_queue_work(hw_priv) 					\
	(((hw_priv)->workqueue==NULL))
#define Sstar_hw_priv_queue_work(hw_priv,work)		\
	(can_not_queue_work(hw_priv) ? -1:queue_work((hw_priv)->workqueue,work))
#define Sstar_hw_priv_queue_delayed_work(hw_priv,dwork,delay)	\
	(can_not_queue_work(hw_priv) ? -1:queue_delayed_work((hw_priv)->workqueue,dwork,delay))
static inline bool Sstar_cancle_queue_work(struct work_struct *work,bool sync)
{
	bool retval = false;
	if((sync == true) || work_pending(work))
	{
		retval = cancel_work_sync(work);
	}

	return retval;
}

static inline bool Sstar_cancle_delayed_work(struct delayed_work *dwork,bool sync)
{
	bool retval = false;
	if(sync == true)
	{
		retval = cancel_delayed_work_sync(dwork);
	}
	else
	{
		retval = cancel_delayed_work(dwork);
	}

	return retval;
}
#ifdef CONFIG_PM
#define Sstar_hold_suspend(__hw_priv)		Sstar_pm_stay_awake_lock(&((__hw_priv)->pm_state))
#define Sstar_release_suspend(__hw_priv)		Sstar_pm_stay_awake_unlock(&((__hw_priv)->pm_state))
#else
#define Sstar_hold_suspend(__hw_priv)		BUG_ON(__hw_priv==NULL)
#define Sstar_release_suspend(__hw_priv)		BUG_ON(__hw_priv==NULL)
#endif

#define Sstar_wait_event_timeout_stay_awake(_hw_priv,_wait_q,_cond,_timeout,_awake)	\
({																					\
	long ret_timeout = _timeout;													\
	if(_awake == true)	Sstar_hold_suspend(_hw_priv);								\
	ret_timeout = wait_event_timeout(_wait_q,_cond,ret_timeout);					\
	if(_awake == true)	Sstar_release_suspend(_hw_priv);								\
	ret_timeout;																	\
})
static inline void Sstar_ieee80211_rx(struct ieee80211_hw	*hw,struct sk_buff *skb)
{
#ifdef IEEE80211_TASKLET
	ieee80211_rx_irqsafe(hw,skb);	
#else	
	if(skb->pkt_type == SSTAR_RX_DERICTLY_DATA_FRAME){
		ieee80211_rx_irqsafe(hw,skb);
	}else if(softirq_count() == 0){
		skb->pkt_type = 0;
		ieee80211_rx_ni(hw,skb);
	}else  {
		skb->pkt_type = 0;
		ieee80211_rx(hw,skb);
	}
#endif
}

static inline void Sstar_ieee80211_tx_status(struct ieee80211_hw	*hw,struct sk_buff *skb)
{
	ieee80211_tx_status_ni(hw,skb);
}
#endif /* SSTAR_APOLLO_BH_H */
