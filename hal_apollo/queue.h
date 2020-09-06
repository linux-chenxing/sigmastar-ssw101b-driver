/*
 * O(1) TX queue with built-in allocator for sigmastar APOLLO drivers
 * *
 * Copyright (c) 2016, sigmastar
 * Author:
 *
 * Based on apollo code
 * Copyright (c) 2010, ST-Ericsson
 * Author: Dmitry Tarnyagin <dmitry.tarnyagin@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef SSTAR_APOLLO_QUEUE_H_INCLUDED
#define SSTAR_APOLLO_QUEUE_H_INCLUDED
#include "mac80211/ieee80211_i.h"

/* private */ struct Sstar_queue_item;

/* extern */ struct sk_buff;
/* extern */ struct wsm_tx;
/* extern */ struct Sstar_common;
/* extern */ struct Sstar_vif;
/* extern */ struct ieee80211_tx_queue_stats;
/* extern */ struct Sstar_txpriv;

/* forward */ struct Sstar_queue_stats;

typedef void (*Sstar_queue_skb_dtor_t)(struct Sstar_common *priv,
					struct sk_buff *skb,
					const struct Sstar_txpriv *txpriv);

struct Sstar_queue {
	struct Sstar_queue_stats *stats;
	u32				capacity;
	u32			num_queued;
	u32			num_queued_vif[SSTAR_WIFI_MAX_VIFS];
	u32			num_pending;
	u32			num_pending_vif[SSTAR_WIFI_MAX_VIFS];
	u32			num_sent;
	u32			num_put;
	struct Sstar_queue_item *pool;
	struct list_head	queue;
	struct list_head	free_pool;
	struct list_head	pending;
#ifndef	SSTAR_WIFI_QUEUE_LOCK_BUG
	int			tx_locked_cnt;
#else
	int			tx_locked_cnt[SSTAR_WIFI_MAX_VIFS];
#endif
	int			*link_map_cache[SSTAR_WIFI_MAX_VIFS];
#ifndef	SSTAR_WIFI_QUEUE_LOCK_BUG
	bool			overfull;
#else
	bool			overfull[SSTAR_WIFI_MAX_VIFS];
#endif
	spinlock_t		lock;
	u8			queue_id;
	u8			generation;
#ifndef	SSTAR_WIFI_QUEUE_LOCK_BUG
	struct timer_list	gc;
#else
	struct timer_list	gc[SSTAR_WIFI_MAX_VIFS];
	u8 timer_to_if_id[SSTAR_WIFI_MAX_VIFS];
#endif
	unsigned long		ttl;
	struct sk_buff *skb_last;
};

struct Sstar_queue_stats {
	spinlock_t		lock;
	int			*link_map_cache[SSTAR_WIFI_MAX_VIFS];
	int			num_queued[SSTAR_WIFI_MAX_VIFS];
	int			map_capacity;
	wait_queue_head_t	wait_link_id_empty;
	Sstar_queue_skb_dtor_t	skb_dtor;
	struct Sstar_common	*hw_priv;
};

struct Sstar_txpriv {
	u8 link_id;
	u8 raw_link_id;
	u8 tid;
	u8 rate_id;
	u8 offset;
	u8 if_id;
#ifndef P2P_MULTIVIF
	u8 offchannel_if_id;
#else
	u8 raw_if_id;
#endif
};

int Sstar_queue_stats_init(struct Sstar_queue_stats *stats,
			    size_t map_capacity,
			    Sstar_queue_skb_dtor_t skb_dtor,
			    struct Sstar_common *priv);
int Sstar_queue_init(struct Sstar_queue *queue,
		      struct Sstar_queue_stats *stats,
		      u8 queue_id,
		      size_t capacity,
		      unsigned long ttl);
int Sstar_queue_clear(struct Sstar_queue *queue, int if_id);
void Sstar_queue_stats_deinit(struct Sstar_queue_stats *stats);
void Sstar_queue_deinit(struct Sstar_queue *queue);

size_t Sstar_queue_get_num_queued(struct Sstar_vif *priv,
				   struct Sstar_queue *queue,
				   u32 link_id_map);
int Sstar_queue_put(struct Sstar_queue *queue,
		     struct sk_buff *skb,
		     struct Sstar_txpriv *txpriv);
int Sstar_queue_get(struct Sstar_queue *queue,
			int if_id,
		     u32 link_id_map,
		     struct wsm_tx **tx,
		     struct ieee80211_tx_info **tx_info,
		     struct Sstar_txpriv **txpriv);
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
int Sstar_queue_requeue(struct Sstar_common *hw_priv,
			struct Sstar_queue *queue,
			u32 packetID, bool check);
#else
int Sstar_queue_requeue(struct Sstar_queue *queue, u32 packetID, bool check);
#endif
int Sstar_queue_requeue_all(struct Sstar_queue *queue);
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
int Sstar_queue_remove(struct Sstar_common *hw_priv,
			struct Sstar_queue *queue,
			u32 packetID);
#else
int Sstar_queue_remove(struct Sstar_queue *queue,
			u32 packetID);
#endif /*CONFIG_SSTAR_APOLLO_TESTMODE*/
int Sstar_queue_get_skb(struct Sstar_queue *queue, u32 packetID,
			 struct sk_buff **skb,
			 const struct Sstar_txpriv **txpriv);
#ifndef SSTAR_WIFI_QUEUE_LOCK_BUG
void Sstar_queue_lock(struct Sstar_queue *queue);
void Sstar_queue_unlock(struct Sstar_queue *queue);
#else
void Sstar_queue_lock(struct Sstar_queue *queue,int if_id);
void Sstar_queue_unlock(struct Sstar_queue *queue,int if_id);
#endif
bool Sstar_queue_get_xmit_timestamp(struct Sstar_queue *queue,
				     unsigned long *timestamp, int if_id,
				     u32 pending_frameID);


bool Sstar_queue_stats_is_empty(struct Sstar_queue_stats *stats,
				 u32 link_id_map, int if_id);

static inline u8 Sstar_queue_get_queue_id(u32 packetID)
{
	return (packetID >> 16) & 0xF;
}

static inline u8 Sstar_queue_get_if_id(u32 packetID)
{
	return (packetID >> 20) & 0xF;
}

static inline u8 Sstar_queue_get_link_id(u32 packetID)
{
	return (packetID >> 24) & 0xF;
}

static inline u8 Sstar_queue_get_generation(u32 packetID)
{
	return (packetID >>  8) & 0xFF;
}

#endif /* SSTAR_APOLLO_QUEUE_H_INCLUDED */
