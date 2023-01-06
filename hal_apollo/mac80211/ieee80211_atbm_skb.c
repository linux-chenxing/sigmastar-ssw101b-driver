/*
 * Datapath implementation for altobeam APOLLO mac80211 drivers
 * *
 * Copyright (c) 2016, altobeam
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

#include <net/atbm_mac80211.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <net/ip.h>
#include <linux/kobject.h>

#include "ieee80211_i.h"

#if (ATBM_ALLOC_SKB_DEBUG == 1)
//#define ATBM_SKB_DESTRUCTOR
#define ATBM_SKB_SHOW_BUFF_MAX_SIZE		PAGE_SIZE
#define ATBM_SKB_SHOW_PUT(_show,...)	\
	do{										\
		int ret = 0;						\
		ret = snprintf((_show)->show_buff+(_show)->show_count,(_show)->show_size-(_show)->show_count,__VA_ARGS__);		\
		if(ret>=0)	(_show)->show_count+=ret;				\
	}while(0)

struct ieee80211_atbm_skb_show
{
	char *show_buff;
	int  show_count;
	int  show_size;
};

#define IEEE80211_ATBM_SKB_HEAD_SIZE sizeof(struct ieee80211_atbm_skb_hdr)
#define IEEE80211_ATBM_CHECK_SKB(_skb)	WARN_ON((_skb->data-_skb->head<IEEE80211_ATBM_SKB_HEAD_SIZE)	\
	||(((struct ieee80211_atbm_skb_hdr*)_skb->head)->masker != ATBM_SKB_MASKER))

static struct list_head ieee80211_atbm_skb_list;
static struct list_head ieee80211_atbm_skb_destructor_list;
static spinlock_t ieee80211_atbm_skb_spin_lock;
#define ATBM_SKB_MASKER 0xAAFFFF55
static struct kobject *atbm_skb_kobj = NULL;
static void ieee80211_atbm_skb_destructor(struct sk_buff *skb);
static ssize_t atbm_skb_show(struct kobject *kobj,struct kobj_attribute *attr, char *buf);

static struct kobj_attribute atbm_skb_attr = __ATTR(atbmskb, 0444, atbm_skb_show, NULL);

static struct attribute *atbm_skb_attribute_group[]= {
	&atbm_skb_attr.attr,
	NULL,
};

static struct attribute_group atbm_skb_attr_group = {
	.attrs = atbm_skb_attribute_group,
};
static u32 skb_generation = 0;
static u32 skb_add_generation = 0;
static u32 skb_del_generation = 0;

static ssize_t atbm_skb_show(struct kobject *kobj,
			     struct kobj_attribute *attr, char *buf)
{
	struct ieee80211_atbm_skb_show skb_show;
	struct ieee80211_atbm_skb_hdr *atbm_hdr = NULL;
	unsigned long flags;
	u32 inlist_counter = 0;
	u32 bytes_in_use = 0;

	skb_show.show_buff = buf;
	skb_show.show_count = 0;
	skb_show.show_size = ATBM_SKB_SHOW_BUFF_MAX_SIZE;

	spin_lock_irqsave(&ieee80211_atbm_skb_spin_lock, flags);
	ATBM_SKB_SHOW_PUT(&skb_show,"skb_lis:add[%d],del[%d],left[%d]\n",
		skb_add_generation,skb_del_generation,skb_add_generation - skb_del_generation);
	list_for_each_entry(atbm_hdr, &ieee80211_atbm_skb_list, head) {
		ATBM_SKB_SHOW_PUT(&skb_show,"<skb_list>[%s][%p]\n", atbm_hdr->call_addr,atbm_hdr);
		inlist_counter++;
		bytes_in_use += atbm_hdr->truesize;
	}
	list_for_each_entry(atbm_hdr, &ieee80211_atbm_skb_destructor_list, head) {
		ATBM_SKB_SHOW_PUT(&skb_show,"<des_list>[%s][%p]\n", atbm_hdr->call_addr,atbm_hdr);
		inlist_counter++;
		bytes_in_use += atbm_hdr->truesize;
	}
	ATBM_SKB_SHOW_PUT(&skb_show,"skb in use[%d],total bytes[%d],true bytes[%d]\n",
		inlist_counter,bytes_in_use,bytes_in_use - (u32)(inlist_counter*IEEE80211_ATBM_SKB_HEAD_SIZE));
	WARN_ON(inlist_counter != (skb_add_generation - skb_del_generation));
	spin_unlock_irqrestore(&ieee80211_atbm_skb_spin_lock, flags);

	return skb_show.show_count;
}
static int ieee80211_atbm_skb_object_int(void)
{
	int error;

	atbm_skb_kobj = kobject_create_and_add("atbm_skb",
					    NULL);
	if (!atbm_skb_kobj)
		return -EINVAL;

	error = sysfs_create_group(atbm_skb_kobj, &atbm_skb_attr_group);
	if (error)
		kobject_put(atbm_skb_kobj);
	return error;
}
static void ieee80211_atbm_skb_object_exit(void)
{
	if(atbm_skb_kobj == NULL)
		return;
	sysfs_remove_group(atbm_skb_kobj, &atbm_skb_attr_group);
	kobject_put(atbm_skb_kobj);
}

void ieee80211_atbm_skb_exit(void)
{
	struct ieee80211_atbm_skb_hdr *atbm_hdr = NULL;
	unsigned long flags;
	printk(KERN_ERR"ieee80211_atbm_skb_exit\n");
	spin_lock_irqsave(&ieee80211_atbm_skb_spin_lock, flags);
	while (!list_empty(&ieee80211_atbm_skb_list)) {
		atbm_hdr = list_first_entry(
			&ieee80211_atbm_skb_list, struct ieee80211_atbm_skb_hdr, head);

		printk(KERN_ERR "%s[skb_list]:skb addr(%s) flags %x\n",__func__,atbm_hdr->call_addr,atbm_hdr->flags);
		list_del(&atbm_hdr->head);
	}

	while (!list_empty(&ieee80211_atbm_skb_destructor_list)) {
		atbm_hdr = list_first_entry(
			&ieee80211_atbm_skb_destructor_list, struct ieee80211_atbm_skb_hdr, head);

		printk(KERN_ERR "%s[destructor_list]:skb addr(%s) flags %x\n",__func__,atbm_hdr->call_addr,atbm_hdr->flags);
		list_del(&atbm_hdr->head);
	}
	spin_unlock_irqrestore(&ieee80211_atbm_skb_spin_lock, flags);
	ieee80211_atbm_skb_object_exit();
}

void ieee80211_atbm_skb_int(void)
{
	printk(KERN_ERR "%s:%d\n",__func__,(u32)IEEE80211_ATBM_SKB_HEAD_SIZE);
	spin_lock_init(&ieee80211_atbm_skb_spin_lock);
	INIT_LIST_HEAD(&ieee80211_atbm_skb_list);
	INIT_LIST_HEAD(&ieee80211_atbm_skb_destructor_list);
	ieee80211_atbm_skb_object_int();
}

static void ieee80211_atbm_add_skb_hdr_to_destructor_list(struct sk_buff *skb,
			struct ieee80211_atbm_skb_hdr *atbm_skb_hdr,const char *func)
{
	unsigned long flags;
	
	BUG_ON(skb->destructor != ieee80211_atbm_skb_destructor);
	spin_lock_irqsave(&ieee80211_atbm_skb_spin_lock, flags);
	memset(atbm_skb_hdr,0,sizeof(struct ieee80211_atbm_skb_hdr));
	atbm_skb_hdr->truesize = skb->truesize;
	atbm_skb_hdr->call_addr = func;
	atbm_skb_hdr->masker = ATBM_SKB_MASKER;
	atbm_skb_hdr->generation = ++skb_generation;
	skb_add_generation++;
	list_add_tail(&atbm_skb_hdr->head, &ieee80211_atbm_skb_destructor_list);
	spin_unlock_irqrestore(&ieee80211_atbm_skb_spin_lock, flags);
}

static void ieee80211_atbm_add_skb_hdr_to_normal_list(struct sk_buff *skb,
			struct ieee80211_atbm_skb_hdr *atbm_skb_hdr,const char *func)
{
	unsigned long flags;	
	BUG_ON(skb->destructor == ieee80211_atbm_skb_destructor);
	spin_lock_irqsave(&ieee80211_atbm_skb_spin_lock, flags);
	memset(atbm_skb_hdr,0,sizeof(struct ieee80211_atbm_skb_hdr));
	atbm_skb_hdr->truesize = skb->truesize;
	atbm_skb_hdr->call_addr = func;
	atbm_skb_hdr->masker = ATBM_SKB_MASKER;
	atbm_skb_hdr->generation = ++skb_generation;
	skb_add_generation++;
	list_add_tail(&atbm_skb_hdr->head, &ieee80211_atbm_skb_list);
	spin_unlock_irqrestore(&ieee80211_atbm_skb_spin_lock, flags);
}
static void ieee80211_atbm_add_skb_hdr_to_list(struct sk_buff *skb,struct ieee80211_atbm_skb_hdr *atbm_skb_hdr,const char *func)
{
	BUG_ON(atbm_skb_hdr==NULL);
	#ifdef ATBM_SKB_DESTRUCTOR
	if(skb->destructor == NULL){
		skb->destructor = ieee80211_atbm_skb_destructor;
	}
	#endif
	if(skb->destructor != ieee80211_atbm_skb_destructor)
		ieee80211_atbm_add_skb_hdr_to_normal_list(skb,atbm_skb_hdr,func);
	else		
		ieee80211_atbm_add_skb_hdr_to_destructor_list(skb,atbm_skb_hdr,func);
	
}

static void ieee80211_atbm_remove_skb_hdr_from_list(struct sk_buff *skb,struct ieee80211_atbm_skb_hdr *atbm_skb_hdr)
{
	unsigned long flags;
	
	if((atbm_skb_hdr->masker != ATBM_SKB_MASKER)||(skb_headroom(skb)<IEEE80211_ATBM_SKB_HEAD_SIZE)){
		return;
	}

	spin_lock_irqsave(&ieee80211_atbm_skb_spin_lock, flags);
	list_del(&atbm_skb_hdr->head);
	atbm_skb_hdr->masker = 0;
	atbm_skb_hdr->call_addr = NULL;
	atbm_skb_hdr->flags = 0;
	atbm_skb_hdr->generation = 0;
	atbm_skb_hdr->truesize = 0;
	skb_del_generation++;
	spin_unlock_irqrestore(&ieee80211_atbm_skb_spin_lock, flags);	
}
static void ieee80211_atbm_skb_destructor(struct sk_buff *skb)
{
	struct ieee80211_atbm_skb_hdr *atbm_hdr = (struct ieee80211_atbm_skb_hdr *)(skb->head);
	if((skb_headroom(skb)>=IEEE80211_ATBM_SKB_HEAD_SIZE)&&(atbm_hdr->masker == ATBM_SKB_MASKER))
		ieee80211_atbm_remove_skb_hdr_from_list(skb,atbm_hdr);
	else {
		printk(KERN_ERR "%s:no atbm header\n",__func__);
	}	
}

void ieee80211_atbm_add_skb_to_debug_list(struct sk_buff *skb,const char *func)
{
	struct ieee80211_atbm_skb_hdr *atbm_hdr = (struct ieee80211_atbm_skb_hdr *)(skb->head);
	if(skb_headroom(skb) < IEEE80211_ATBM_SKB_HEAD_SIZE){
		printk(KERN_ERR "%s: headroom(%d)\n",__func__,skb_headroom(skb));
		return;
	}
	if(atbm_hdr->masker == ATBM_SKB_MASKER){
		struct ieee80211_atbm_skb_hdr *temp_atbm_hdr = NULL;
		unsigned long flags;
		spin_lock_irqsave(&ieee80211_atbm_skb_spin_lock, flags);
		if(skb->destructor != ieee80211_atbm_skb_destructor){
			list_for_each_entry(temp_atbm_hdr, &ieee80211_atbm_skb_list, head) {
				if((temp_atbm_hdr == atbm_hdr)&&
				   (temp_atbm_hdr->call_addr == atbm_hdr->call_addr)&&
				   (temp_atbm_hdr->generation == atbm_hdr->generation))
					break;
			}
		}else {
			list_for_each_entry(temp_atbm_hdr, &ieee80211_atbm_skb_destructor_list, head) {
				if((temp_atbm_hdr == atbm_hdr)&&
				   (temp_atbm_hdr->call_addr == atbm_hdr->call_addr)&&
				   (temp_atbm_hdr->generation == atbm_hdr->generation))
					break;
			}
		}
		spin_unlock_irqrestore(&ieee80211_atbm_skb_spin_lock, flags);
		WARN_ON(temp_atbm_hdr != atbm_hdr);
		return;
	}

	ieee80211_atbm_add_skb_hdr_to_list(skb,atbm_hdr,func);
}

void ieee80211_atbm_rx_debug_setflag(struct sk_buff *skb,u32 flags)
{
	struct ieee80211_atbm_skb_hdr *atbm_hdr;
	
	if(!skb){
		return;
	}

	atbm_hdr = (struct ieee80211_atbm_skb_hdr *)(skb->head);
	
	if((skb_headroom(skb)>=IEEE80211_ATBM_SKB_HEAD_SIZE)&&(atbm_hdr->masker == ATBM_SKB_MASKER)){
		atbm_hdr->flags |= flags;
	}
}

void ieee80211_atbm_rx_debug_setflag2(struct sk_buff *skb,u16 fc)
{
	struct ieee80211_atbm_skb_hdr *atbm_hdr;
	
	if(!skb){
		return;
	}

	atbm_hdr = (struct ieee80211_atbm_skb_hdr *)(skb->head);
	
	if((skb_headroom(skb)>=IEEE80211_ATBM_SKB_HEAD_SIZE)&&(atbm_hdr->masker == ATBM_SKB_MASKER)){
		atbm_hdr->flags |= (fc<<16);
	}
}


struct sk_buff *__ieee80211_atbm_dev_alloc_skb(unsigned int length,gfp_t gfp_mask,const char *func)
{
	struct sk_buff * atbm_skb = NULL;
	
	atbm_skb = __dev_alloc_skb(length+IEEE80211_ATBM_SKB_HEAD_SIZE,gfp_mask);

	if(atbm_skb == NULL){
		return atbm_skb;
	}

	skb_reserve(atbm_skb, IEEE80211_ATBM_SKB_HEAD_SIZE);

	ieee80211_atbm_add_skb_hdr_to_list(atbm_skb,(struct ieee80211_atbm_skb_hdr *)(atbm_skb->head),func);
	return atbm_skb;
}

struct sk_buff *ieee80211_atbm_dev_alloc_skb(unsigned int length,const char *func)
{
	struct sk_buff * atbm_skb = NULL;
	
	atbm_skb = dev_alloc_skb(length+IEEE80211_ATBM_SKB_HEAD_SIZE);

	if(atbm_skb == NULL){
		return atbm_skb;
	}

	skb_reserve(atbm_skb, IEEE80211_ATBM_SKB_HEAD_SIZE);

	ieee80211_atbm_add_skb_hdr_to_list(atbm_skb,(struct ieee80211_atbm_skb_hdr *)(atbm_skb->head),func);
	return atbm_skb;
}
struct sk_buff *ieee80211_atbm_alloc_skb(unsigned int size,gfp_t priority,const char *func)
{
	struct sk_buff * atbm_skb = NULL;
	atbm_skb = alloc_skb(size+IEEE80211_ATBM_SKB_HEAD_SIZE,priority);

	if(atbm_skb){
		skb_reserve(atbm_skb, IEEE80211_ATBM_SKB_HEAD_SIZE);
		ieee80211_atbm_add_skb_hdr_to_list(atbm_skb,(struct ieee80211_atbm_skb_hdr *)(atbm_skb->head),func);
	}

	return atbm_skb;
}
void ieee80211_atbm_dev_kfree_skb_any(struct sk_buff *skb)
{
	struct ieee80211_atbm_skb_hdr *atbm_hdr;
	
	if(!skb){
		return;
	}

	atbm_hdr = (struct ieee80211_atbm_skb_hdr *)(skb->head);
	if(skb->destructor != ieee80211_atbm_skb_destructor){
		if((skb_headroom(skb)>=IEEE80211_ATBM_SKB_HEAD_SIZE)&&(atbm_hdr->masker == ATBM_SKB_MASKER))
			ieee80211_atbm_remove_skb_hdr_from_list(skb,atbm_hdr);
		else {
			if(skb->destructor != NULL)
				printk(KERN_ERR "%s:no atbm header\n",__func__);
		}
	}
	dev_kfree_skb_any(skb);
}

void ieee80211_atbm_dev_kfree_skb(struct sk_buff *skb)
{
	struct ieee80211_atbm_skb_hdr *atbm_hdr;
	
	if(!skb){
		return;
	}

	atbm_hdr = (struct ieee80211_atbm_skb_hdr *)(skb->head);
	if(skb->destructor != ieee80211_atbm_skb_destructor){
		if((skb_headroom(skb)>=IEEE80211_ATBM_SKB_HEAD_SIZE)&&(atbm_hdr->masker == ATBM_SKB_MASKER))
			ieee80211_atbm_remove_skb_hdr_from_list(skb,atbm_hdr);
		else {
			if(skb->destructor != NULL)
				printk(KERN_ERR "%s:no atbm header\n",__func__);
		}
	}
	dev_kfree_skb(skb);
}

void ieee80211_atbm_kfree_skb(struct sk_buff *skb)
{
	struct ieee80211_atbm_skb_hdr *atbm_hdr;
	
	if(!skb){
		return;
	}

	atbm_hdr = (struct ieee80211_atbm_skb_hdr *)(skb->head);
	if(skb->destructor != ieee80211_atbm_skb_destructor){
		if((skb_headroom(skb)>=IEEE80211_ATBM_SKB_HEAD_SIZE)&&(atbm_hdr->masker == ATBM_SKB_MASKER))
			ieee80211_atbm_remove_skb_hdr_from_list(skb,atbm_hdr);
		else {
			if(skb->destructor != NULL)
				printk(KERN_ERR "%s:no atbm header\n",__func__);
		}
	}
	kfree_skb(skb);
}
void ieee80211_atbm_dev_kfree_skb_irq(struct sk_buff *skb)
{
	struct ieee80211_atbm_skb_hdr *atbm_hdr;
	
	if(!skb){
		return;
	}

	atbm_hdr = (struct ieee80211_atbm_skb_hdr *)(skb->head);
	if(skb->destructor != ieee80211_atbm_skb_destructor){
		if((skb_headroom(skb)>=IEEE80211_ATBM_SKB_HEAD_SIZE)&&(atbm_hdr->masker == ATBM_SKB_MASKER))
			ieee80211_atbm_remove_skb_hdr_from_list(skb,atbm_hdr);
		else {
			if(skb->destructor != NULL)
				printk(KERN_ERR "%s:no atbm header\n",__func__);
		}
	}
	dev_kfree_skb_irq(skb);
}

void ieee80211_atbm_skb_reserve(struct sk_buff *skb, int len)
{
	skb_reserve(skb,len);
}

void ieee80211_atbm_skb_trim(struct sk_buff *skb, unsigned int len)
{
	skb_trim(skb,len);
}

unsigned char *ieee80211_atbm_skb_put(struct sk_buff *skb, unsigned int len)
{
	return skb_put(skb,len);
}

void ieee80211_atbm_skb_queue_tail(struct sk_buff_head *list, struct sk_buff *newsk)
{
	skb_queue_tail(list,newsk);
}
struct sk_buff *__ieee80211_atbm_skb_dequeue(struct sk_buff_head *list)
{
	struct sk_buff * skb = __skb_dequeue(list);

	if(skb) IEEE80211_ATBM_CHECK_SKB(skb);

	return skb;
}

struct sk_buff *ieee80211_atbm_skb_dequeue(struct sk_buff_head *list)
{
	struct sk_buff * skb = skb_dequeue(list);
	
	if(skb) IEEE80211_ATBM_CHECK_SKB(skb);

	return skb;
}

void __ieee80211_atbm_skb_queue_purge(struct sk_buff_head *list)
{
	struct sk_buff *skb;
	while ((skb = __ieee80211_atbm_skb_dequeue(list)) != NULL)
		ieee80211_atbm_kfree_skb(skb);
}

void ieee80211_atbm_skb_queue_purge(struct sk_buff_head *list)
{
	struct sk_buff *skb;
	while ((skb = ieee80211_atbm_skb_dequeue(list)) != NULL){
		ieee80211_atbm_kfree_skb(skb);
	}
}

int ieee80211_atbm_skb_tailroom(const struct sk_buff *skb)
{
	return skb_tailroom(skb);
}

int ieee80211_atbm_skb_queue_empty(const struct sk_buff_head *list)
{
	return skb_queue_empty(list);
}

void ieee80211_atbm_skb_queue_splice_tail_init(struct sk_buff_head *list,
					      struct sk_buff_head *head)
{
	skb_queue_splice_tail_init(list,head);
}

void ieee80211_atbm_skb_queue_head_init(struct sk_buff_head *list)
{
	skb_queue_head_init(list);
}

void __ieee80211_atbm_skb_queue_tail(struct sk_buff_head *list,struct sk_buff *newsk,const char *func)
{
//	IEEE80211_ATBM_CHECK_SKB(newsk);
	struct ieee80211_atbm_skb_hdr *atbm_hdr = (struct ieee80211_atbm_skb_hdr *)(newsk->head);

	if((skb_headroom(newsk)<IEEE80211_ATBM_SKB_HEAD_SIZE)||(atbm_hdr->masker != ATBM_SKB_MASKER)){
		printk(KERN_ERR"%s:[%d][%lx][%s]\n",__func__,skb_headroom(newsk),atbm_hdr->masker,func);
		WARN_ON(1);
	}
	__skb_queue_tail(list,newsk);
}

__u32 ieee80211_atbm_skb_queue_len(const struct sk_buff_head *list_)
{
	return skb_queue_len(list_);
}

void __ieee80211_atbm_skb_queue_head_init(struct sk_buff_head *list)
{
	__skb_queue_head_init(list);
}
void __ieee80211_atbm_skb_queue_head(struct sk_buff_head *list,
				    struct sk_buff *newsk)
{
	IEEE80211_ATBM_CHECK_SKB(newsk);
	__skb_queue_head(list,newsk);
}
struct sk_buff *ieee80211_atbm_skb_copy(const struct sk_buff *skb, gfp_t gfp_mask,const char *func)
{
	struct sk_buff *new_skb = NULL;
	
	new_skb = skb_copy(skb,gfp_mask);

	if(new_skb)
		ieee80211_atbm_add_skb_hdr_to_list(new_skb,(struct ieee80211_atbm_skb_hdr *)(new_skb->head),func);

	return new_skb;
	
}
unsigned char *ieee80211_atbm_skb_pull(struct sk_buff *skb, unsigned int len)
{
	return skb_pull(skb,len);
}

unsigned char *ieee80211_atbm_skb_push(struct sk_buff *skb, unsigned int len)
{
	char *p;
	p = skb_push(skb,len);
	return p;
}
int ieee80211_atbm_dev_queue_xmit(struct sk_buff *skb)
{
	IEEE80211_ATBM_CHECK_SKB(skb);
	if(skb->destructor != ieee80211_atbm_skb_destructor)
		ieee80211_atbm_remove_skb_hdr_from_list(skb,(struct ieee80211_atbm_skb_hdr *)(skb->head));
	else
		skb_orphan(skb);
		
	return dev_queue_xmit(skb);
}

int ieee80211_atbm_netif_rx_ni(struct sk_buff *skb)
{
	IEEE80211_ATBM_CHECK_SKB(skb);
	if(skb->destructor != ieee80211_atbm_skb_destructor)
		ieee80211_atbm_remove_skb_hdr_from_list(skb,(struct ieee80211_atbm_skb_hdr *)(skb->head));
	else 
		skb_orphan(skb);
	return netif_rx_ni(skb);
}
void ieee80211_atbm_skb_set_queue_mapping(struct sk_buff *skb, u16 queue_mapping)
{
	skb_set_queue_mapping(skb,queue_mapping);
}

void ieee80211_atbm_skb_set_tail_pointer(struct sk_buff *skb, const int offset)
{
	skb_set_tail_pointer(skb,offset);
}
void ieee80211_atbm_skb_reset_tail_pointer(struct sk_buff *skb)
{
	skb_reset_tail_pointer(skb);
}
void __ieee80211_atbm_skb_unlink(struct sk_buff *skb, struct sk_buff_head *list)
{
	IEEE80211_ATBM_CHECK_SKB(skb);
	__skb_unlink(skb,list);
}

void ieee80211_atbm_skb_set_mac_header(struct sk_buff *skb, const int offset)
{
	skb_set_mac_header(skb,offset);
}
void ieee80211_atbm_skb_set_network_header(struct sk_buff *skb, int offset)
{
	skb_set_network_header(skb,offset);
}

void ieee80211_atbm_skb_set_transport_header(struct sk_buff *skb, int offset)
{
	skb_set_transport_header(skb,offset);
}

void ieee80211_atbm_skb_queue_splice(const struct sk_buff_head *list,
				    struct sk_buff_head *head)
{
	skb_queue_splice(list,head);
}

void ieee80211_atbm_skb_queue_splice_init(struct sk_buff_head *list,
					 struct sk_buff_head *head)
{
	skb_queue_splice_init(list,head);
}

int __ieee80211_atbm_pskb_trim(struct sk_buff *skb, unsigned int len)
{
	return __pskb_trim(skb,len);
}

int ieee80211_atbm_pskb_may_pull(struct sk_buff *skb, unsigned int len)
{
	return pskb_may_pull(skb,len);
}
unsigned int ieee80211_atbm_skb_headroom(const struct sk_buff *skb)
{
	if(skb_headroom(skb)<IEEE80211_ATBM_SKB_HEAD_SIZE){
//		WARN_ON(1);
		return 0;
	}
	return skb_headroom(skb)-IEEE80211_ATBM_SKB_HEAD_SIZE;
}
int ieee80211_atbm_pskb_expand_head(struct sk_buff *skb, int nhead, int ntail,
		     gfp_t gfp_mask,const char *func)
{
	int ret = 0;
	if(skb->destructor != ieee80211_atbm_skb_destructor)
		ieee80211_atbm_remove_skb_hdr_from_list(skb,(struct ieee80211_atbm_skb_hdr *)(skb->head));
	else
		skb_orphan(skb);
	ret = pskb_expand_head(skb,nhead+IEEE80211_ATBM_SKB_HEAD_SIZE,ntail,gfp_mask);

	if(ret == 0){
		ieee80211_atbm_add_skb_hdr_to_list(skb,(struct ieee80211_atbm_skb_hdr *)(skb->head),func);
	}

	return ret;
}

struct sk_buff *ieee80211_atbm_skb_copy_expand(const struct sk_buff *skb,
				int newheadroom, int newtailroom,
				gfp_t gfp_mask,const char *func)
{
	struct sk_buff *skb_copy = NULL;
	
	skb_copy = skb_copy_expand(skb,newheadroom+IEEE80211_ATBM_SKB_HEAD_SIZE,newtailroom,gfp_mask);

	if(skb_copy){
		ieee80211_atbm_add_skb_hdr_to_list(skb_copy,(struct ieee80211_atbm_skb_hdr *)(skb_copy->head),func);
	}

	return skb_copy;
	
}
int ieee80211_atbm_skb_cloned(const struct sk_buff *skb)
{
	return skb_cloned(skb);
}
struct sk_buff *ieee80211_atbm_skb_clone(struct sk_buff *skb, gfp_t gfp_mask,const char *func)
{
	struct sk_buff *cloned_skb = NULL;

	cloned_skb = skb_clone(skb,gfp_mask);

	if(cloned_skb){		
		ieee80211_atbm_add_skb_hdr_to_list(cloned_skb,(struct ieee80211_atbm_skb_hdr *)(cloned_skb->head),func);
	}

	return cloned_skb;
}
int ieee80211_atbm_netif_rx(struct sk_buff *skb)
{	
	if(skb->destructor != ieee80211_atbm_skb_destructor)
		ieee80211_atbm_remove_skb_hdr_from_list(skb,(struct ieee80211_atbm_skb_hdr *)(skb->head));
	else
		skb_orphan(skb);
	return netif_rx(skb);
}
int ieee80211_atbm_netif_receive_skb(struct sk_buff *skb)
{	
	if(skb->destructor != ieee80211_atbm_skb_destructor)
		ieee80211_atbm_remove_skb_hdr_from_list(skb,(struct ieee80211_atbm_skb_hdr *)(skb->head));
	else
		skb_orphan(skb);
	return netif_receive_skb(skb);
}
int ieee80211_atbm_skb_linearize(struct sk_buff *skb)
{
	return skb_linearize(skb);
}

struct sk_buff *ieee80211_atbm_skb_peek(struct sk_buff_head *list_)
{
	return skb_peek(list_);
}

int ieee80211_atbm_skb_shared(const struct sk_buff *skb)
{
	return skb_shared(skb);
}

int ieee80211_atbm_skb_padto(struct sk_buff *skb, unsigned int len)
{
	return skb_padto(skb,len);
}

struct sk_buff *ieee80211_atbm_skb_get(struct sk_buff *skb)
{
	return skb_get(skb);
}
void ieee80211_atbm_skb_orphan(struct sk_buff *skb,const char *func)
{
#ifdef ATBM_SKB_DESTRUCTOR
	bool our_skb = skb->destructor == ieee80211_atbm_skb_destructor ? true:false;
	struct ieee80211_atbm_skb_hdr *atbm_hdr;
	skb_orphan(skb);

	if((our_skb == false)||(skb_headroom(skb)<IEEE80211_ATBM_SKB_HEAD_SIZE)){
		pskb_expand_head(skb,IEEE80211_ATBM_SKB_HEAD_SIZE,0,GFP_ATOMIC);
		WARN_ON(our_skb == true);
	}
	
	atbm_hdr = (struct ieee80211_atbm_skb_hdr *)skb->head;

	ieee80211_atbm_add_skb_hdr_to_normal_list(skb,atbm_hdr,func);
#else
	func = func;
	skb_orphan(skb);
#endif
}
void ieee80211_atbm_amsdu_to_8023s(struct sk_buff *skb, struct sk_buff_head *list,
			      const u8 *addr, enum nl80211_iftype iftype,
			      const unsigned int extra_headroom,
			      bool has_80211_header)
{
	struct sk_buff *frame = NULL;
	u16 ethertype;
	u8 *payload;
	const struct ethhdr *eth;
	int remaining, err;
	u8 dst[ETH_ALEN], src[ETH_ALEN];

	if (has_80211_header) {
		err = ieee80211_data_to_8023(skb, addr, iftype);
		if (err)
			goto out;

		/* skip the wrapping header */
		eth = (struct ethhdr *) ieee80211_atbm_skb_pull(skb, sizeof(struct ethhdr));
		if (!eth)
			goto out;
	} else {
		eth = (struct ethhdr *) skb->data;
	}

	while (skb != frame) {
		u8 padding;
		__be16 len = eth->h_proto;
		unsigned int subframe_len = sizeof(struct ethhdr) + ntohs(len);

		remaining = skb->len;
		memcpy(dst, eth->h_dest, ETH_ALEN);
		memcpy(src, eth->h_source, ETH_ALEN);

		padding = (4 - subframe_len) & 0x3;
		/* the last MSDU has no padding */
		if (subframe_len > remaining)
			goto purge;

		ieee80211_atbm_skb_pull(skb, sizeof(struct ethhdr));
		/* reuse skb for the last subframe */
		if (remaining <= subframe_len + padding)
			frame = skb;
		else {
			unsigned int hlen = ALIGN(extra_headroom, 4);
			/*
			 * Allocate and reserve two bytes more for payload
			 * alignment since sizeof(struct ethhdr) is 14.
			 */
			frame = ieee80211_atbm_dev_alloc_skb(hlen + subframe_len + 2,__func__);
			if (!frame)
				goto purge;

			ieee80211_atbm_skb_reserve(frame, hlen + sizeof(struct ethhdr) + 2);
			memcpy(ieee80211_atbm_skb_put(frame, ntohs(len)), skb->data,
				ntohs(len));

			eth = (struct ethhdr *)ieee80211_atbm_skb_pull(skb, ntohs(len) +
							padding);
			if (!eth) {
				ieee80211_atbm_dev_kfree_skb(frame);
				goto purge;
			}
		}

		skb_reset_network_header(frame);
		frame->dev = skb->dev;
		frame->priority = skb->priority;

		payload = frame->data;
		ethertype = (payload[6] << 8) | payload[7];

		if (likely((ether_addr_equal(payload, rfc1042_header) &&
			    ethertype != ETH_P_AARP && ethertype != ETH_P_IPX) ||
			   ether_addr_equal(payload, bridge_tunnel_header))) {
			/* remove RFC1042 or Bridge-Tunnel
			 * encapsulation and replace EtherType */
			ieee80211_atbm_skb_pull(frame, 6);
			memcpy(ieee80211_atbm_skb_push(frame, ETH_ALEN), src, ETH_ALEN);
			memcpy(ieee80211_atbm_skb_push(frame, ETH_ALEN), dst, ETH_ALEN);
		} else {
			memcpy(ieee80211_atbm_skb_push(frame, sizeof(__be16)), &len,
				sizeof(__be16));
			memcpy(ieee80211_atbm_skb_push(frame, ETH_ALEN), src, ETH_ALEN);
			memcpy(ieee80211_atbm_skb_push(frame, ETH_ALEN), dst, ETH_ALEN);
		}
		__ieee80211_atbm_skb_queue_tail(list, frame,__func__);
	}

	return;

 purge:
	__ieee80211_atbm_skb_queue_purge(list);
 out:
	ieee80211_atbm_dev_kfree_skb(skb);
}

#endif
