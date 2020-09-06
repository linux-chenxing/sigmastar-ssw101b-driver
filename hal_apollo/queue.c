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

#include <net/Sstar_mac80211.h>
#include <linux/sched.h>
#include "apollo.h"
#include "queue.h"
#include "debug.h"
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
#include <linux/time.h>
#endif /*CONFIG_SSTAR_APOLLO_TESTMODE*/

/* private */ struct Sstar_queue_item
{
	struct list_head	head;
	struct sk_buff		*skb;
	u32			packetID;
	unsigned long		queue_timestamp;
	unsigned long		xmit_timestamp;
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
	unsigned long		mdelay_timestamp;
	unsigned long		qdelay_timestamp;
#endif /*CONFIG_SSTAR_APOLLO_TESTMODE*/
	struct Sstar_txpriv	txpriv;
	u8			generation;
};


#ifndef SSTAR_WIFI_QUEUE_LOCK_BUG
static inline void __Sstar_queue_lock(struct Sstar_queue *queue)
{
	struct Sstar_queue_stats *stats = queue->stats;
	if (queue->tx_locked_cnt++ == 0) {
		ieee80211_stop_queue(stats->hw_priv->hw, queue->queue_id);
	}
}

static inline void __Sstar_queue_unlock(struct Sstar_queue *queue)
{
	struct Sstar_queue_stats *stats = queue->stats;
	BUG_ON(!queue->tx_locked_cnt);
	if (--queue->tx_locked_cnt == 0) {
		ieee80211_wake_queue(stats->hw_priv->hw, queue->queue_id);
	}
}
#else
static inline void __Sstar_queue_lock(struct Sstar_queue *queue,u8 if_id)
{
	struct Sstar_queue_stats *stats = queue->stats;
	if (queue->tx_locked_cnt[if_id]++ == 0){
		ieee80211_stop_queue(stats->hw_priv->hw, queue->queue_id+4*if_id);
	}else{
		BUG_ON(ieee80211_queue_stopped(stats->hw_priv->hw,queue->queue_id+4*if_id) == 0);
	}
}

static inline void __Sstar_queue_unlock(struct Sstar_queue *queue,u8 if_id)
{
	struct Sstar_queue_stats *stats = queue->stats;
	BUG_ON(!queue->tx_locked_cnt[if_id]);
	if (--queue->tx_locked_cnt[if_id] == 0){
		ieee80211_wake_queue(stats->hw_priv->hw, queue->queue_id+4*if_id);
	}
}
#endif
static inline void Sstar_queue_parse_id(u32 packetID, u8 *queue_generation,
						u8 *queue_id,
						u8 *item_generation,
						u8 *item_id,
						u8 *if_id,
						u8 *link_id)
{
	*item_id		= (packetID >>  0) & 0xFF;
	*item_generation	= (packetID >>  8) & 0xFF;
	*queue_id		= (packetID >> 16) & 0xF;
	*if_id			= (packetID >> 20) & 0xF;
	*link_id		= (packetID >> 24) & 0xF;
	*queue_generation	= (packetID >> 28) & 0xF;
}

static inline u32 Sstar_queue_make_packet_id(u8 queue_generation, u8 queue_id,
						u8 item_generation, u8 item_id,
						u8 if_id, u8 link_id)
{
	/*TODO:COMBO: Add interfaceID to the packetID */
	return ((u32)item_id << 0) |
		((u32)item_generation << 8) |
		((u32)queue_id << 16) |
		((u32)if_id << 20) |
		((u32)link_id << 24) |
		((u32)queue_generation << 28);
}

static void Sstar_queue_post_gc(struct Sstar_queue_stats *stats,
				 struct list_head *gc_list)
{
	struct Sstar_queue_item *item;

	while (!list_empty(gc_list)) {
		item = list_first_entry(
			gc_list, struct Sstar_queue_item, head);
		list_del(&item->head);
		stats->skb_dtor(stats->hw_priv, item->skb, &item->txpriv);
		Sstar_kfree(item);
	}
}

static void Sstar_queue_register_post_gc(struct list_head *gc_list,
				     struct Sstar_queue_item *item)
{
	struct Sstar_queue_item *gc_item;
	gc_item = Sstar_kmalloc(sizeof(struct Sstar_queue_item),
			GFP_ATOMIC);
	BUG_ON(!gc_item);
	memcpy(gc_item, item, sizeof(struct Sstar_queue_item));
	list_add_tail(&gc_item->head, gc_list);
}
#ifndef	SSTAR_WIFI_QUEUE_LOCK_BUG

static void __Sstar_queue_gc(struct Sstar_queue *queue,
			      struct list_head *head,
			      bool unlock)
{
	struct Sstar_queue_stats *stats = queue->stats;
	struct Sstar_queue_item *item = NULL;
	struct Sstar_vif *priv;
	int if_id;
	bool wakeup_stats = false;

	while (!list_empty(&queue->queue)) {
		struct Sstar_txpriv *txpriv;
		item = list_first_entry(
			&queue->queue, struct Sstar_queue_item, head);
		if (jiffies - item->queue_timestamp < queue->ttl)
			break;

		txpriv = &item->txpriv;
		if_id = txpriv->if_id;
		--queue->num_queued;
		--queue->num_queued_vif[if_id];
		--queue->link_map_cache[if_id][txpriv->link_id];
		spin_lock_bh(&stats->lock);
		--stats->num_queued[if_id];
		if (!--stats->link_map_cache[if_id][txpriv->link_id])
			wakeup_stats = true;
		spin_unlock_bh(&stats->lock);
		priv = ABwifi_hwpriv_to_vifpriv(stats->hw_priv, if_id);
		if (priv) {
			Sstar_debug_tx_ttl(priv);
			Sstar_priv_vif_list_read_unlock(&priv->vif_lock);
		}
		Sstar_queue_register_post_gc(head, item);
		item->skb = NULL;
		list_move_tail(&item->head, &queue->free_pool);
	}

	if (wakeup_stats)
		wake_up(&stats->wait_link_id_empty);

	if (queue->overfull) {
		if (queue->num_queued <= (queue->capacity/2)) {
			queue->overfull = false;
			if (unlock)
				__Sstar_queue_unlock(queue);
		} else if (item) {
			unsigned long tmo = item->queue_timestamp + queue->ttl;
			mod_timer(&queue->gc, tmo);
			#ifdef CONFIG_PM
			Sstar_pm_stay_awake(&stats->hw_priv->pm_state,
					tmo - jiffies);
			#endif
		}
	}
}

static void Sstar_queue_gc(unsigned long arg)
{
	LIST_HEAD(list);
	struct Sstar_queue *queue =
		(struct Sstar_queue *)arg;

	spin_lock_bh(&queue->lock);
	__Sstar_queue_gc(queue, &list, true);
	spin_unlock_bh(&queue->lock);
	Sstar_queue_post_gc(queue->stats, &list);
}
#else
static void __Sstar_queue_gc(struct Sstar_queue *queue,
			      struct list_head *head,
			      bool unlock,u8 if_id_clear)
{
	struct Sstar_queue_stats *stats = queue->stats;
	struct Sstar_queue_item *item = NULL;
	struct list_head *pos, *nx;
	struct Sstar_vif *priv;
	int if_id;
	bool wakeup_stats = false;
	struct Sstar_txpriv *txpriv;
	int loop = 0;
	
	//list_for_each_entry_safe(item, tmp_item, &queue->queue, head) {
	list_for_each_safe(pos,nx,&queue->queue){
	
		item = list_entry(pos,struct Sstar_queue_item, head);
		txpriv = &item->txpriv;
		if_id = txpriv->if_id;
		BUG_ON(++loop > 300);
		if(if_id != if_id_clear){
			item = NULL;
			continue;
		}

		if (jiffies - item->queue_timestamp < queue->ttl)
			break;

		--queue->num_queued;
		--queue->num_queued_vif[if_id];
		--queue->link_map_cache[if_id][txpriv->link_id];
		spin_lock_bh(&stats->lock);
		--stats->num_queued[if_id];
		if (!--stats->link_map_cache[if_id][txpriv->link_id])
			wakeup_stats = true;
		spin_unlock_bh(&stats->lock);
		priv = ABwifi_hwpriv_to_vifpriv(stats->hw_priv, if_id);
		if (priv) {
			Sstar_debug_tx_ttl(priv);
			Sstar_priv_vif_list_read_unlock(&priv->vif_lock);
		}
		Sstar_queue_register_post_gc(head, item);
		item->skb = NULL;
		list_move_tail(&item->head, &queue->free_pool);
		item = NULL;
	}
	
	if (wakeup_stats)
		wake_up(&stats->wait_link_id_empty);

	if (queue->overfull[if_id_clear]) {
		u32 capacity = 0;
		priv = __ABwifi_hwpriv_to_vifpriv(stats->hw_priv,if_id_clear);
		if(priv)
			capacity = priv->queue_cap*2/3;
		else
			capacity = queue->capacity/4;
		
		if (queue->num_queued_vif[if_id_clear] <= capacity) {
			queue->overfull[if_id_clear] = false;
			if (unlock){
				__Sstar_queue_unlock(queue,if_id_clear);
			}
		} else if (item) {
			if(item->txpriv.if_id == if_id_clear){
				unsigned long tmo = item->queue_timestamp + queue->ttl;
				mod_timer(&queue->gc[if_id_clear], tmo);
				#ifdef CONFIG_PM
				Sstar_pm_stay_awake(&stats->hw_priv->pm_state,
						tmo - jiffies);
				#endif
			}else {				
				Sstar_printk_err("%s:if_id[%d],id_clear[%d],queued[%d]\n",__func__,item->txpriv.if_id,if_id_clear,queue->num_queued_vif[if_id_clear]);
			}
		}
	}
}

static void Sstar_queue_gc(unsigned long arg)
{
	LIST_HEAD(list);
	u8 *pif_id = (u8*)arg;
	u8 if_id = *pif_id;
	u8 *timer_to_if_id = pif_id - if_id;
	struct Sstar_queue *queue = container_of(timer_to_if_id, struct Sstar_queue,
						 timer_to_if_id[0]);
	spin_lock_bh(&queue->lock);
	__Sstar_queue_gc(queue, &list, true,if_id);
	spin_unlock_bh(&queue->lock);
	Sstar_queue_post_gc(queue->stats, &list);
}
#endif
int Sstar_queue_stats_init(struct Sstar_queue_stats *stats,
			    size_t map_capacity,
			    Sstar_queue_skb_dtor_t skb_dtor,
			    struct Sstar_common *hw_priv)
{
	int i;

	memset(stats, 0, sizeof(*stats));
	stats->map_capacity = map_capacity;
	stats->skb_dtor = skb_dtor;
	stats->hw_priv = hw_priv;
	spin_lock_init(&stats->lock);
	init_waitqueue_head(&stats->wait_link_id_empty);
	for (i = 0; i < SSTAR_WIFI_MAX_VIFS; i++) {
		stats->link_map_cache[i] = Sstar_kzalloc(sizeof(int[map_capacity]),
			GFP_KERNEL);
		if (!stats->link_map_cache[i]) {
			for (; i >= 0; i--)
				Sstar_kfree(stats->link_map_cache[i]);
			return -ENOMEM;
		}
	}

	return 0;
}

int Sstar_queue_init(struct Sstar_queue *queue,
		      struct Sstar_queue_stats *stats,
		      u8 queue_id,
		      size_t capacity,
		      unsigned long ttl)
{
	int i;

	memset(queue, 0, sizeof(*queue));
	queue->stats = stats;
	queue->capacity = capacity;
	queue->queue_id = queue_id;
	queue->ttl = ttl;
	INIT_LIST_HEAD(&queue->queue);
	INIT_LIST_HEAD(&queue->pending);
	INIT_LIST_HEAD(&queue->free_pool);
	spin_lock_init(&queue->lock);
	#ifndef	SSTAR_WIFI_QUEUE_LOCK_BUG
	init_timer(&queue->gc);
	queue->gc.data = (unsigned long)queue;
	queue->gc.function = Sstar_queue_gc;
	#else
	for(i = 0;i<SSTAR_WIFI_MAX_VIFS;i++){
		init_timer(&queue->gc[i]);
		queue->timer_to_if_id[i]=i;
		queue->gc[i].data = (unsigned long)(&queue->timer_to_if_id[i]);
		queue->gc[i].function = Sstar_queue_gc;
	}
	#endif
	queue->pool = Sstar_kzalloc(sizeof(struct Sstar_queue_item) * capacity,
			GFP_KERNEL);
	if (!queue->pool)
		return -ENOMEM;

	for (i = 0; i < SSTAR_WIFI_MAX_VIFS; i++) {
		queue->link_map_cache[i] =
				Sstar_kzalloc(sizeof(int[stats->map_capacity]),
			GFP_KERNEL);
		if (!queue->link_map_cache[i]) {
			for (; i >= 0; i--)
				Sstar_kfree(queue->link_map_cache[i]);
			Sstar_kfree(queue->pool);
			queue->pool = NULL;
			return -ENOMEM;
		}
	}

	for (i = 0; i < capacity; ++i)
		list_add_tail(&queue->pool[i].head, &queue->free_pool);

	return 0;
}

/* TODO:COMBO: Flush only a particular interface specific parts */
int Sstar_queue_clear(struct Sstar_queue *queue, int if_id)
{
	int i, cnt, iter;
	struct Sstar_queue_stats *stats = queue->stats;
	struct Sstar_queue_item *item = NULL,*pitem=NULL;
	LIST_HEAD(gc_list);

	cnt = 0;
	spin_lock_bh(&queue->lock);
	queue->generation++;
	queue->generation &= 0xf;
	
	list_for_each_entry_safe(item, pitem,&queue->pending, head){
		if(SSTAR_WIFI_ALL_IFS == if_id || item->txpriv.if_id == if_id){
			Sstar_queue_register_post_gc(&gc_list, item);
			item->skb = NULL;
			list_move_tail(&item->head, &queue->free_pool);
			cnt++;
		}
	}
	WARN_ON(cnt > queue->num_pending);
	queue->num_pending -= cnt;

	list_for_each_entry_safe(item, pitem,&queue->queue, head){
		if(SSTAR_WIFI_ALL_IFS == if_id || item->txpriv.if_id == if_id){
			Sstar_queue_register_post_gc(&gc_list, item);
			item->skb = NULL;
			list_move_tail(&item->head, &queue->free_pool);
			cnt++;
		}
	}
	BUG_ON(cnt > queue->num_queued);
	queue->num_queued -= cnt;
	
	if (SSTAR_WIFI_ALL_IFS != if_id) {
		queue->num_queued_vif[if_id] = 0;
		queue->num_pending_vif[if_id] = 0;
	} else {
		for (iter = 0; iter < SSTAR_WIFI_MAX_VIFS; iter++) {
			queue->num_queued_vif[iter] = 0;
			queue->num_pending_vif[iter] = 0;
		}
	}
	spin_lock_bh(&stats->lock);
	if (SSTAR_WIFI_ALL_IFS != if_id) {
		for (i = 0; i < stats->map_capacity; ++i) {
			stats->num_queued[if_id] -=
				queue->link_map_cache[if_id][i];
			stats->link_map_cache[if_id][i] -=
				queue->link_map_cache[if_id][i];
			queue->link_map_cache[if_id][i] = 0;
		}
	} else {
		for (iter = 0; iter < SSTAR_WIFI_MAX_VIFS; iter++) {
			for (i = 0; i < stats->map_capacity; ++i) {
				stats->num_queued[iter] -=
					queue->link_map_cache[iter][i];
				stats->link_map_cache[iter][i] -=
					queue->link_map_cache[iter][i];
				queue->link_map_cache[iter][i] = 0;
			}
		}
	}
	spin_unlock_bh(&stats->lock);
	#ifndef	SSTAR_WIFI_QUEUE_LOCK_BUG
	if (unlikely(queue->overfull)) {
		queue->overfull = false;
		__Sstar_queue_unlock(queue);
	}
	#else
	if(SSTAR_WIFI_ALL_IFS == if_id){
		for(i = 0;i<SSTAR_WIFI_MAX_VIFS;i++)
			if(unlikely(queue->overfull[i])){
				queue->overfull[i] = false;
				__Sstar_queue_unlock(queue,i);
			}
	}else if(unlikely(queue->overfull[if_id])){
		queue->overfull[if_id] = false;
		__Sstar_queue_unlock(queue,if_id);
	}
	#endif
	spin_unlock_bh(&queue->lock);
	wake_up(&stats->wait_link_id_empty);
	Sstar_queue_post_gc(stats, &gc_list);
	return 0;
}

void Sstar_queue_stats_deinit(struct Sstar_queue_stats *stats)
{
	int i;

	for (i = 0; i < SSTAR_WIFI_MAX_VIFS ; i++) {
		Sstar_kfree(stats->link_map_cache[i]);
		stats->link_map_cache[i] = NULL;
	}
}

void Sstar_queue_deinit(struct Sstar_queue *queue)
{
	int i;

	Sstar_queue_clear(queue, SSTAR_WIFI_ALL_IFS);
	#ifndef	SSTAR_WIFI_QUEUE_LOCK_BUG
	del_timer_sync(&queue->gc);
	#else
	for (i = 0; i < SSTAR_WIFI_MAX_VIFS; i++) {
		del_timer_sync(&queue->gc[i]);
	}
	#endif
	INIT_LIST_HEAD(&queue->free_pool);
	Sstar_kfree(queue->pool);
	for (i = 0; i < SSTAR_WIFI_MAX_VIFS; i++) {
		Sstar_kfree(queue->link_map_cache[i]);
		queue->link_map_cache[i] = NULL;
	}
	queue->pool = NULL;
	queue->capacity = 0;
}

size_t Sstar_queue_get_num_queued(struct Sstar_vif *priv,
				   struct Sstar_queue *queue,
				   u32 link_id_map)
{
	size_t ret;
	int i, bit;
	size_t map_capacity = queue->stats->map_capacity;

	if (!link_id_map)
		return 0;

	spin_lock_bh(&queue->lock);
	if (likely(link_id_map == (u32) -1)) {
		ret = queue->num_queued_vif[priv->if_id] -
			queue->num_pending_vif[priv->if_id];
	} else {
		ret = 0;
		for (i = 0, bit = 1; i < map_capacity; ++i, bit <<= 1) {
			if (link_id_map & bit)
				ret +=
				queue->link_map_cache[priv->if_id][i];
		}
	}
	spin_unlock_bh(&queue->lock);
	return ret;
}

#include <linux/ip.h>
#include <linux/tcp.h>
int Sstar_tcp_ack_offload(struct Sstar_queue *queue,struct Sstar_txpriv *txpriv,struct sk_buff *skb_new)
{
	struct sk_buff *skb_last = queue->skb_last;
	spin_lock_bh(&queue->lock);
	if((queue->link_map_cache[txpriv->if_id][txpriv->link_id] > 0)&&(skb_last != NULL)){
		if ((skb_new->protocol == htons(ETH_P_IP))
			&&(skb_new->len == skb_last->len))
			{
			struct iphdr *ip_new = ip_hdr(skb_new);
			struct iphdr *ip_last = ip_hdr(skb_last);
			if(((ip_new->ihl == 5)&&(ip_last->ihl == 5))
			 &&((ip_new->protocol == IPPROTO_TCP) && (ip_last->protocol == IPPROTO_TCP))
			 &&((htons(ip_new->tot_len) == 40)&&(htons(ip_last->tot_len) == 40))
			 &&((ip_new->saddr == ip_last->saddr)&&(ip_new->daddr == ip_last->daddr))
			 &&(ip_new->tos == ip_last->tos)){
				struct tcphdr * tcphdr_new =	((struct tcphdr *)skb_transport_header(skb_new));
				struct tcphdr * tcphdr_last =	((struct tcphdr *)skb_transport_header(skb_last));
				if((tcphdr_new->source == tcphdr_last->source)
					&&(tcphdr_new->dest == tcphdr_last->dest)
					&&(tcphdr_new->seq == tcphdr_last->seq)
					&&(tcphdr_new->window == tcphdr_last->window)
					&&(tcphdr_new->doff == tcphdr_last->doff)
					&&(tcphdr_new->urg_ptr == tcphdr_last->urg_ptr)){
					u16 tcphdr_new_dff_u16 = ((u16*)(tcphdr_new))[6];
					u16 tcphdr_last_dff_u16 = ((u16*)(tcphdr_last))[6];
					if(tcphdr_last_dff_u16 == tcphdr_new_dff_u16){
						memcpy(ip_last,ip_new,40);
						spin_unlock_bh(&queue->lock);
						return	1;
					}
				}
			 }
		}
	}
	spin_unlock_bh(&queue->lock);
	return 0;
}
int Sstar_queue_put(struct Sstar_queue *queue,
		     struct sk_buff *skb,
		     struct Sstar_txpriv *txpriv)
{
	int ret = 0;
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
	struct timeval tmval;
#endif /*CONFIG_SSTAR_APOLLO_TESTMODE*/
	LIST_HEAD(gc_list);
	struct Sstar_queue_stats *stats = queue->stats;
	/* TODO:COMBO: Add interface ID info to queue item */

	if (txpriv->link_id >= queue->stats->map_capacity)
		return -EINVAL;

	spin_lock_bh(&queue->lock);
	if (!WARN_ON(list_empty(&queue->free_pool))) {
		struct Sstar_queue_item *item = list_first_entry(
			&queue->free_pool, struct Sstar_queue_item, head);
		BUG_ON(item->skb);
		queue->skb_last = skb;

		list_move_tail(&item->head, &queue->queue);
		item->skb = skb;
		item->txpriv = *txpriv;
		item->generation = 0;
		item->packetID = Sstar_queue_make_packet_id(
			queue->generation, queue->queue_id,
			item->generation, item - queue->pool,
			txpriv->if_id, txpriv->raw_link_id);
		item->queue_timestamp = jiffies;
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
		do_gettimeofday(&tmval);
		item->qdelay_timestamp = tmval.tv_usec;
#endif /*CONFIG_SSTAR_APOLLO_TESTMODE*/

		++queue->num_queued;
		++queue->num_queued_vif[txpriv->if_id];
		++queue->link_map_cache[txpriv->if_id][txpriv->link_id];

		spin_lock_bh(&stats->lock);
		++stats->num_queued[txpriv->if_id];
		++stats->link_map_cache[txpriv->if_id][txpriv->link_id];
		spin_unlock_bh(&stats->lock);
		/*
		 * TX may happen in parallel sometimes.
		 * Leave extra queue slots so we don't overflow.
		 */
		#ifndef SSTAR_WIFI_QUEUE_LOCK_BUG
		if (queue->overfull == false &&
				queue->num_queued >= (queue->capacity - (num_present_cpus()))) {
			queue->overfull = true;
			__Sstar_queue_lock(queue);
			mod_timer(&queue->gc, jiffies);
		}
		#else
		{
			#define xIF_QUEUE_OVERFLOW(__queue,_if_id,_if_cap) \
				(__queue->num_queued_vif[_if_id] >= (_if_cap - (num_present_cpus())))
			#define ALLIF_QUEUE_OVERFLOW(__queue) \
				(__queue->num_queued >= (__queue->capacity - (num_present_cpus())))
			
			if(queue->overfull[txpriv->if_id]==false){
			    struct Sstar_vif *priv;
				u32 capacity = 0;
				
				priv = __ABwifi_hwpriv_to_vifpriv(stats->hw_priv,txpriv->if_id);
				if(priv)
					capacity = priv->queue_cap;
				else
					capacity = queue->capacity/2;
				if(xIF_QUEUE_OVERFLOW(queue,txpriv->if_id,capacity) || ALLIF_QUEUE_OVERFLOW(queue)){
					queue->overfull[txpriv->if_id] = true;
					__Sstar_queue_lock(queue,txpriv->if_id);
					mod_timer(&queue->gc[txpriv->if_id], jiffies);

				}
			}else {
				struct Sstar_common	*hw_priv = queue->stats->hw_priv;
				Sstar_printk_err( "%s queue_locked:if_id[%d],num_queued_vif[%d],queue[%d],all queued(%d),reason[%ld],lockcnt(%d)\n",
				__func__,txpriv->if_id,queue->num_queued_vif[txpriv->if_id],queue->queue_id,queue->num_queued,
				hw_to_local(hw_priv->hw)->queue_stop_reasons[queue->queue_id + 4*txpriv->if_id],queue->tx_locked_cnt[txpriv->if_id]);
			}
		}
		#endif
	} else {
		#ifdef SSTAR_WIFI_QUEUE_LOCK_BUG
		struct Sstar_common	*hw_priv = queue->stats->hw_priv;
		Sstar_printk_err( "%s free_pool:if_id[%d],num_queued_vif[%d],queue[%d],all queued(%d),reason[%ld],lockcnt(%d)\n",
			__func__,txpriv->if_id,queue->num_queued_vif[txpriv->if_id],queue->queue_id,queue->num_queued,
			hw_to_local(hw_priv->hw)->queue_stop_reasons[queue->queue_id + 4*txpriv->if_id],queue->tx_locked_cnt[txpriv->if_id]);
		#endif
		ret = -ENOENT;
	}
#if 0
	printk(KERN_ERR "queue_put queue %d, %d, %d\n",
		queue->num_queued,
		queue->link_map_cache[txpriv->if_id][txpriv->link_id],
		queue->num_pending);
	printk(KERN_ERR "queue_put stats %d, %d\n", stats->num_queued,
		stats->link_map_cache[txpriv->if_id][txpriv->link_id]);
#endif
	spin_unlock_bh(&queue->lock);
	return ret;
}

int Sstar_queue_get(struct Sstar_queue *queue,
			int if_id,
		     u32 link_id_map,
		     struct wsm_tx **tx,
		     struct ieee80211_tx_info **tx_info,
		     struct Sstar_txpriv **txpriv)
{
	int ret = -ENOENT;
	struct Sstar_queue_item *item;
	struct Sstar_queue_stats *stats = queue->stats;
	bool wakeup_stats = false;
	u8 queue_generation = 0;
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
	struct timeval tmval;
#endif /*CONFIG_SSTAR_APOLLO_TESTMODE*/

	spin_lock_bh(&queue->lock);
	list_for_each_entry(item, &queue->queue, head) {
		if ((item->txpriv.if_id == if_id) &&
			(link_id_map & BIT(item->txpriv.link_id))) {
			ret = 0;
			break;
		}
	}

	if (!WARN_ON(ret)) {
		if(queue->skb_last == item->skb){
			queue->skb_last = NULL;
		}
		*tx = (struct wsm_tx *)item->skb->data;
		*tx_info = IEEE80211_SKB_CB(item->skb);
		*txpriv = &item->txpriv;
		queue_generation = (item->packetID >> 28) & 0xF;
		/*
		*queue_generation must be equal to queue->generation
		*/
		if(queue_generation != (queue->generation&0xF)){
			Sstar_printk_err( "%s:update generation(%d)->(%d)\n",__func__,queue_generation,queue->generation);
			item->packetID &= 0x0fffffff;//BIT(28):BIT(31)->0
			item->packetID |= ((u32)(queue->generation&0xF))<<28;
			BUG_ON(((item->packetID >> 28) & 0xF) != queue->generation);
		}
		(*tx)->packetID = __cpu_to_le32(item->packetID);
		list_move_tail(&item->head, &queue->pending);
		++queue->num_pending;
		++queue->num_pending_vif[item->txpriv.if_id];
		--queue->link_map_cache[item->txpriv.if_id]
				[item->txpriv.link_id];
		++queue->num_put;
		item->xmit_timestamp = jiffies;
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
		do_gettimeofday(&tmval);
		item->mdelay_timestamp = tmval.tv_usec;
#endif /*CONFIG_SSTAR_APOLLO_TESTMODE*/

		spin_lock_bh(&stats->lock);
		--stats->num_queued[item->txpriv.if_id];
		if (!--stats->link_map_cache[item->txpriv.if_id]
					[item->txpriv.link_id])
			wakeup_stats = true;

		spin_unlock_bh(&stats->lock);
#if 0
		printk(KERN_ERR "queue_get queue %d, %d, %d\n",
		queue->num_queued,
		queue->link_map_cache[item->txpriv.if_id][item->txpriv.link_id],
		queue->num_pending);
		printk(KERN_ERR "queue_get stats %d, %d\n", stats->num_queued,
		stats->link_map_cache[item->txpriv.if_id]
		[item->txpriv.link_id]);
#endif
	}
	spin_unlock_bh(&queue->lock);
	if (wakeup_stats)
		wake_up(&stats->wait_link_id_empty);
	return ret;
}

#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
int Sstar_queue_requeue(struct Sstar_common *hw_priv,
	struct Sstar_queue *queue, u32 packetID, bool check)
#else
int Sstar_queue_requeue(struct Sstar_queue *queue, u32 packetID, bool check)
#endif
{
	int ret = 0;
	u8 queue_generation, queue_id, item_generation, item_id, if_id, link_id;
	struct Sstar_queue_item *item;
	struct Sstar_queue_stats *stats = queue->stats;

	Sstar_queue_parse_id(packetID, &queue_generation, &queue_id,
				&item_generation, &item_id, &if_id, &link_id);

	item = &queue->pool[item_id];
#ifdef P2P_MULTIVIF
	if (check && item->txpriv.if_id == SSTAR_WIFI_GENERIC_IF_ID) {
#else
	if (check && item->txpriv.offchannel_if_id == SSTAR_WIFI_GENERIC_IF_ID) {
#endif
		Sstar_printk_err("Requeued frame dropped for "
						"generic interface id.\n");
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
		Sstar_queue_remove(hw_priv, queue, packetID);
#else
		Sstar_queue_remove(queue, packetID);
#endif
		return 0;
	}

#ifndef P2P_MULTIVIF
	if (!check)
		item->txpriv.offchannel_if_id = SSTAR_WIFI_GENERIC_IF_ID;
#endif

	/*if_id = item->txpriv.if_id;*/

	spin_lock_bh(&queue->lock);
	BUG_ON(queue_id != queue->queue_id);
	if (unlikely(queue_generation != queue->generation)) {
		ret = -ENOENT;
	} else if (unlikely(item_id >= (unsigned) queue->capacity)) {
		WARN_ON(1);
		ret = -EINVAL;
	} else if (unlikely(item->generation != item_generation)) {
		WARN_ON(1);
		ret = -ENOENT;
	} else {
		--queue->num_pending;
		--queue->num_pending_vif[if_id];
		++queue->link_map_cache[if_id][item->txpriv.link_id];

		spin_lock_bh(&stats->lock);
		++stats->num_queued[item->txpriv.if_id];
		++stats->link_map_cache[if_id][item->txpriv.link_id];
		spin_unlock_bh(&stats->lock);

		item->generation = ++item_generation;
		item->packetID = Sstar_queue_make_packet_id(
			queue_generation, queue_id, item_generation, item_id,
			if_id, link_id);
		list_move(&item->head, &queue->queue);
#if 0
		printk(KERN_ERR "queue_requeue queue %d, %d, %d\n",
		queue->num_queued,
		queue->link_map_cache[if_id][item->txpriv.link_id],
		queue->num_pending);
		printk(KERN_ERR "queue_requeue stats %d, %d\n",
		stats->num_queued,
		stats->link_map_cache[if_id][item->txpriv.link_id]);
#endif
	}
	spin_unlock_bh(&queue->lock);
	return ret;
}

int Sstar_queue_requeue_all(struct Sstar_queue *queue)
{
	struct Sstar_queue_stats *stats = queue->stats;
	spin_lock_bh(&queue->lock);
	while (!list_empty(&queue->pending)) {
		struct Sstar_queue_item *item = list_entry(
			queue->pending.prev, struct Sstar_queue_item, head);

		--queue->num_pending;
		--queue->num_pending_vif[item->txpriv.if_id];
		++queue->link_map_cache[item->txpriv.if_id]
				[item->txpriv.link_id];

		spin_lock_bh(&stats->lock);
		++stats->num_queued[item->txpriv.if_id];
		++stats->link_map_cache[item->txpriv.if_id]
				[item->txpriv.link_id];
		spin_unlock_bh(&stats->lock);

		++item->generation;
		item->packetID = Sstar_queue_make_packet_id(
			queue->generation, queue->queue_id,
			item->generation, item - queue->pool,
			item->txpriv.if_id, item->txpriv.raw_link_id);
		list_move(&item->head, &queue->queue);
	}
	spin_unlock_bh(&queue->lock);

	return 0;
}
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
int Sstar_queue_remove(struct Sstar_common *hw_priv,
				struct Sstar_queue *queue, u32 packetID)
#else
int Sstar_queue_remove(struct Sstar_queue *queue, u32 packetID)
#endif /*CONFIG_SSTAR_APOLLO_TESTMODE*/
{
	int ret = 0;
	u8 queue_generation, queue_id, item_generation, item_id, if_id, link_id;
	struct Sstar_queue_item *item;
	struct Sstar_queue_stats *stats = queue->stats;
	struct sk_buff *gc_skb = NULL;
	struct Sstar_txpriv gc_txpriv;

	Sstar_queue_parse_id(packetID, &queue_generation, &queue_id,
				&item_generation, &item_id, &if_id, &link_id);

	item = &queue->pool[item_id];

	spin_lock_bh(&queue->lock);
	BUG_ON(queue_id != queue->queue_id);
	/*TODO:COMBO:Add check for interface ID also */
	if (unlikely(queue_generation != queue->generation)) {
		ret = -ENOENT;
	} else if (unlikely(item_id >= (unsigned) queue->capacity)) {
		WARN_ON(1);
		ret = -EINVAL;
	} else if (unlikely(item->generation != item_generation)) {
		WARN_ON(1);
		ret = -ENOENT;
	} else {
		gc_txpriv = item->txpriv;
		gc_skb = item->skb;
		item->skb = NULL;
		--queue->num_pending;
		--queue->num_pending_vif[if_id];
		--queue->num_queued;
		--queue->num_queued_vif[if_id];
		++queue->num_sent;
		++item->generation;
#ifdef CONFIG_SSTAR_APOLLO_TESTMODE
		spin_lock_bh(&hw_priv->tsm_lock);
		if (hw_priv->start_stop_tsm.start) {
			if (queue_id == hw_priv->tsm_info.ac) {
				struct timeval tmval;
				unsigned long queue_delay;
				unsigned long media_delay;
				do_gettimeofday(&tmval);

				if (tmval.tv_usec > item->qdelay_timestamp)
					queue_delay = tmval.tv_usec -
						item->qdelay_timestamp;
				else
					queue_delay = tmval.tv_usec +
					1000000 - item->qdelay_timestamp;

				if (tmval.tv_usec > item->mdelay_timestamp)
					media_delay = tmval.tv_usec -
						item->mdelay_timestamp;
				else
					media_delay = tmval.tv_usec +
					1000000 - item->mdelay_timestamp;
				hw_priv->tsm_info.sum_media_delay +=
							media_delay;
				hw_priv->tsm_info.sum_pkt_q_delay += queue_delay;
				hw_priv->tsm_stats.txed_msdu_count++;
				if (queue_delay <= 500)
					hw_priv->tsm_stats.bin00++;
				else if (queue_delay <= 1000)
					hw_priv->tsm_stats.bin01++;
				else if (queue_delay <= 2000)
					hw_priv->tsm_stats.bin02++;
				else if (queue_delay <= 4000)
					hw_priv->tsm_stats.bin03++;
				else if (queue_delay <= 6000)
					hw_priv->tsm_stats.bin04++;
				else if (queue_delay <= 8000)
					hw_priv->tsm_stats.bin05++;
				else if (queue_delay <= 10000)
					hw_priv->tsm_stats.bin0++;
				else if (queue_delay <= 20000)
					hw_priv->tsm_stats.bin1++;
				else if (queue_delay <= 40000)
					hw_priv->tsm_stats.bin2++;
				else
					hw_priv->tsm_stats.bin3++;
			}
		}
		spin_unlock_bh(&hw_priv->tsm_lock);
#endif /*CONFIG_SSTAR_APOLLO_TESTMODE*/
		/* Do not use list_move_tail here, but list_move:
		 * try to utilize cache row.
		 */
		list_move(&item->head, &queue->free_pool);

		#ifndef SSTAR_WIFI_QUEUE_LOCK_BUG
		if (unlikely(queue->overfull) &&
		    (queue->num_queued <= (queue->capacity / 2))) {
			queue->overfull = false;
			__Sstar_queue_unlock(queue);
		}
		#else
		{
			if (unlikely(queue->overfull[if_id])){
				
				u32 capacity = 0;
				struct Sstar_vif *priv;
				priv = __ABwifi_hwpriv_to_vifpriv(stats->hw_priv,if_id);
				if(priv)
					capacity = priv->queue_cap*2/3;
				else
					capacity = queue->capacity/4;
				
				if(queue->num_queued_vif[if_id] <= capacity) {
					queue->overfull[if_id] = false;
					__Sstar_queue_unlock(queue,if_id);
				}
			}
		}
		#endif
	}
	spin_unlock_bh(&queue->lock);

#if 0
	printk(KERN_ERR "queue_drop queue %d, %d, %d\n",
		queue->num_queued, queue->link_map_cache[if_id][0],
		queue->num_pending);
	printk(KERN_ERR "queue_drop stats %d, %d\n", stats->num_queued,
		stats->link_map_cache[if_id][0]);
#endif
	if (gc_skb)
		stats->skb_dtor(stats->hw_priv, gc_skb, &gc_txpriv);

	return ret;
}

int Sstar_queue_get_skb(struct Sstar_queue *queue, u32 packetID,
			 struct sk_buff **skb,
			 const struct Sstar_txpriv **txpriv)
{
	int ret = 0;
	u8 queue_generation, queue_id, item_generation, item_id, if_id, link_id;
	struct Sstar_queue_item *item;

	Sstar_queue_parse_id(packetID, &queue_generation, &queue_id,
				&item_generation, &item_id, &if_id, &link_id);

	item = &queue->pool[item_id];

	spin_lock_bh(&queue->lock);
	BUG_ON(queue_id != queue->queue_id);
	/* TODO:COMBO: Add check for interface ID here */
	if (unlikely(queue_generation != queue->generation)) {
		ret = -ENOENT;
	} else if (unlikely(item_id >= (unsigned) queue->capacity)) {
		WARN_ON(1);
		ret = -EINVAL;
	} else if (unlikely(item->generation != item_generation)) {
		WARN_ON(1);
		ret = -ENOENT;
	} else {
		*skb = item->skb;
		*txpriv = &item->txpriv;
	}
	spin_unlock_bh(&queue->lock);
	if(ret<0){
		Sstar_printk_err( "%s:packetID(%x),queue_generation(%d),queue_id(%d),item_generation(%d),item_id(%d),if_id(%d)\n",
			__func__,packetID,queue_generation,queue_id,item_generation,item_id,if_id);
		Sstar_printk_err( "%s:queue->generation(%d),queue->queue_id(%d),item->generation(%d),queue->capacity(%d)\n",__func__,
			queue->generation,queue->queue_id,item->generation,queue->capacity);
	}
	return ret;
}
#ifndef SSTAR_WIFI_QUEUE_LOCK_BUG
void Sstar_queue_lock(struct Sstar_queue *queue)
{
	spin_lock_bh(&queue->lock);
	__Sstar_queue_lock(queue);
	spin_unlock_bh(&queue->lock);
}

void Sstar_queue_unlock(struct Sstar_queue *queue)
{
	spin_lock_bh(&queue->lock);
	__Sstar_queue_unlock(queue);
	spin_unlock_bh(&queue->lock);
}
#else
void Sstar_queue_lock(struct Sstar_queue *queue,int if_id)
{
	spin_lock_bh(&queue->lock);
	__Sstar_queue_lock(queue,if_id);
	spin_unlock_bh(&queue->lock);
}

void Sstar_queue_unlock(struct Sstar_queue *queue,int if_id)
{
	spin_lock_bh(&queue->lock);
	__Sstar_queue_unlock(queue,if_id);
	spin_unlock_bh(&queue->lock);
}
#endif
bool Sstar_queue_get_xmit_timestamp(struct Sstar_queue *queue,
				     unsigned long *timestamp, int if_id,
				     u32 pending_frameID)
{
	struct Sstar_queue_item *item;
	bool ret;

	spin_lock_bh(&queue->lock);
	ret = !list_empty(&queue->pending);
	if (ret) {
		list_for_each_entry(item, &queue->pending, head) {
			if (((if_id == SSTAR_WIFI_GENERIC_IF_ID) ||
				(if_id == SSTAR_WIFI_ALL_IFS) ||
					(item->txpriv.if_id == if_id)) &&
					(item->packetID != pending_frameID)) {
				if (time_before(item->xmit_timestamp,
							*timestamp))
					*timestamp = item->xmit_timestamp;
			}
		}
	}
	spin_unlock_bh(&queue->lock);
	return ret;
}

bool Sstar_queue_stats_is_empty(struct Sstar_queue_stats *stats,
				 u32 link_id_map, int if_id)
{
	bool empty = true;

	spin_lock_bh(&stats->lock);
	if (link_id_map == (u32)-1)
		empty = stats->num_queued[if_id] == 0;
	else {
		int i, if_id;
		for (if_id = 0; if_id < SSTAR_WIFI_MAX_VIFS; if_id++) {
			for (i = 0; i < stats->map_capacity; ++i) {
				if (link_id_map & BIT(i)) {
					if (stats->link_map_cache[if_id][i]) {
						empty = false;
						break;
					}
				}
			}
		}
	}
	spin_unlock_bh(&stats->lock);

	return empty;
}


unsigned long  Sstar_queue_ttl(struct Sstar_queue *queue){
	struct Sstar_queue_item *item = list_first_entry(
				&queue->queue, struct Sstar_queue_item, head);
	return (jiffies - item->queue_timestamp)*(1000/HZ);
}
