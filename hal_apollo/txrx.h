/*
 * Datapath interface for sigmastar APOLLO mac80211 drivers
 * *
 * Copyright (c) 2016, sigmastar
 * Author:
 *
 *Based on apollo code
 * Copyright (c) 2010, ST-Ericsson
 * Author: Dmitry Tarnyagin <dmitry.tarnyagin@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef SSTAR_APOLLO_TXRX_H
#define SSTAR_APOLLO_TXRX_H

#include <linux/list.h>

/* extern */ struct ieee80211_hw;
/* extern */ struct sk_buff;
/* extern */ struct wsm_tx;
/* extern */ struct wsm_rx;
/* extern */ struct wsm_tx_confirm;
/* extern */ struct Sstar_txpriv;
/* extern */ struct Sstar_vif;

struct tx_policy {
	union {
		__le32 tbl[3];
		u8 raw[12];
	};
	u8  defined;		/* TODO: u32 or u8, profile and select best */
	u8  usage_count;	/* --// -- */
	u8  retry_count;	/* --// -- */
	u8  uploaded;
};

struct tx_policy_cache_entry {
	struct tx_policy policy;
	struct list_head link;
};

#define TX_POLICY_CACHE_SIZE	(8)
struct tx_policy_cache {
	struct tx_policy_cache_entry cache[TX_POLICY_CACHE_SIZE];
	struct list_head used;
	struct list_head free;
	spinlock_t lock;
};

/* ******************************************************************** */
/* TX policy cache							*/
/* Intention of TX policy cache is an overcomplicated WSM API.
 * Device does not accept per-PDU tx retry sequence.
 * It uses "tx retry policy id" instead, so driver code has to sync
 * linux tx retry sequences with a retry policy table in the device.
 */
void tx_policy_init(struct Sstar_common *hw_priv);
void tx_policy_upload_work(struct work_struct *work);

/* ******************************************************************** */
/* TX implementation							*/

u32 Sstar_rate_mask_to_wsm(struct Sstar_common *hw_priv,
			       u32 rates);
void Sstar_tx(struct ieee80211_hw *dev, struct sk_buff *skb);
void Sstar_skb_dtor(struct Sstar_common *hw_priv,
		     struct sk_buff *skb,
		     const struct Sstar_txpriv *txpriv);

/* ******************************************************************** */
/* WSM callbacks							*/

void Sstar_tx_confirm_cb(struct Sstar_common *hw_priv,
			  struct wsm_tx_confirm *arg);
void Sstar_rx_cb(struct Sstar_vif *priv,
		  struct wsm_rx *arg,
		  struct sk_buff **skb_p);

/* ******************************************************************** */
/* Timeout								*/

void Sstar_tx_timeout(struct work_struct *work);

/* ******************************************************************** */
/* Security								*/
int Sstar_alloc_key(struct Sstar_common *hw_priv);
void Sstar_free_key(struct Sstar_common *hw_priv, int idx);
void Sstar_free_keys(struct Sstar_common *hw_priv);
int Sstar_upload_keys(struct Sstar_vif *priv);

/* ******************************************************************** */
/* Workaround for WFD test case 6.1.10					*/
void Sstar_link_id_reset(struct work_struct *work);

#endif /* SSTAR_APOLLO_TXRX_H */
