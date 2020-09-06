/*
 * Datapath implementation for sigmastar APOLLO mac80211 drivers
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

#include <net/Sstar_mac80211.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <net/ip.h>
#include <linux/kobject.h>

#include "ieee80211_i.h"

#if defined (SSTAR_ALLOC_MEM_DEBUG)
#pragma message("Mem Debug Enable")
#define SSTAR_MEM_SHOW_BUFF_MAX_SIZE		PAGE_SIZE
#define SSTAR_MEM_SHOW_PUT(_show,...)	\
	do{										\
		int ret = 0;						\
		ret = scnprintf((_show)->show_buff+(_show)->show_count,(_show)->show_size-(_show)->show_count,__VA_ARGS__);		\
		if(ret>=0)	(_show)->show_count+=ret;				\
	}while(0)
#define SSTAR_MEM_RESERV			(64-((sizeof(char *)+sizeof(struct list_head))%64))
struct ieee80211_Sstar_mem
{
	struct list_head head;
	const char *call_addr;
	u32 mem_len;
	u8 mem[0] __attribute__((__aligned__(64)));
};

struct ieee80211_Sstar_mem_show
{
	char *show_buff;
	int  show_count;
	int  show_size;
};
static struct list_head ieee80211_Sstar_mem_list;
static spinlock_t ieee80211_Sstar_mem_spin_lock;
static struct kobject *Sstar_mem_kobj = NULL;
static u32 mem_add_generation = 0;
static u32 mem_del_generation = 0;

static ssize_t Sstar_mem_show(struct kobject *kobj,struct kobj_attribute *attr, char *buf);

static struct kobj_attribute Sstar_mem_attr = __ATTR(Sstarmem, 0444, Sstar_mem_show, NULL);

static struct attribute *Sstar_mem_attribute_group[]= {
	&Sstar_mem_attr.attr,
	NULL,
};

static struct attribute_group Sstar_mem_attr_group = {
	.attrs = Sstar_mem_attribute_group,
};
static void *__ieee80211_Sstar_kmalloc(size_t s, gfp_t gfp,const char *call_addr)
{
	struct ieee80211_Sstar_mem *Sstar_mem = NULL;
	void *p = NULL;
	unsigned long flags;
	
	Sstar_mem = kmalloc(s+sizeof(struct ieee80211_Sstar_mem),gfp);
	Sstar_printk_debug( "alloc Sstar_mem(%p)\n",Sstar_mem);
	if(Sstar_mem){
		p = (void*)Sstar_mem->mem;
		Sstar_mem->call_addr = call_addr;
		Sstar_mem->mem_len = s+sizeof(struct ieee80211_Sstar_mem);
		spin_lock_irqsave(&ieee80211_Sstar_mem_spin_lock, flags);
		list_add_tail(&Sstar_mem->head, &ieee80211_Sstar_mem_list);
		mem_add_generation++;
		spin_unlock_irqrestore(&ieee80211_Sstar_mem_spin_lock, flags);
	}

	return p;
}
void *ieee80211_Sstar_kmalloc(size_t s, gfp_t gfp,const char *func)
{
	void *p = __ieee80211_Sstar_kmalloc(s, gfp,func);
	return p;
}

//void *ieee80211_Sstar_kzalloc(size_t s, gfp_t gfp,void* call_addr)
void *ieee80211_Sstar_kzalloc(size_t s, gfp_t gfp,const char *func)
{
	void *p = __ieee80211_Sstar_kmalloc(s, gfp,func);
	if(p)
		memset(p, 0, s);
	return p;
}
void *ieee80211_Sstar_kcalloc(size_t n, size_t size, gfp_t gfp,const char *func)
{
	if((n == 0) || (size==0))
		return NULL;
	return ieee80211_Sstar_kzalloc(n*size, gfp | __GFP_ZERO,func);
}

void *ieee80211_Sstar_krealloc(void *p, size_t new_size, gfp_t gfp,const char *func)
{
	struct ieee80211_Sstar_mem *Sstar_mem = NULL;
	struct ieee80211_Sstar_mem *Sstar_mem_new = NULL;
	void *p_new = NULL;
	unsigned long flags;
	
	if((p == NULL)||(new_size == 0))
		return NULL;

	Sstar_mem = container_of(p, struct ieee80211_Sstar_mem, mem);

	
	spin_lock_irqsave(&ieee80211_Sstar_mem_spin_lock, flags);	
	list_del(&Sstar_mem->head);
	mem_del_generation++;
	spin_unlock_irqrestore(&ieee80211_Sstar_mem_spin_lock, flags);

	Sstar_mem_new = krealloc(Sstar_mem,new_size+sizeof(struct ieee80211_Sstar_mem),gfp);

	if(Sstar_mem_new){
		p_new = (void*)Sstar_mem_new->mem;
		Sstar_mem_new->call_addr = func;
		Sstar_mem_new->mem_len = new_size+sizeof(struct ieee80211_Sstar_mem);
		spin_lock_irqsave(&ieee80211_Sstar_mem_spin_lock, flags);		
		list_add_tail(&Sstar_mem_new->head, &ieee80211_Sstar_mem_list);
		mem_add_generation++;
		spin_unlock_irqrestore(&ieee80211_Sstar_mem_spin_lock, flags);
	}

	return p_new;
}

void ieee80211_Sstar_kfree(void *p)
{
	struct ieee80211_Sstar_mem *Sstar_mem = NULL;
	unsigned long flags;
	
	if(p == NULL)
		return;
	Sstar_mem = container_of(p, struct ieee80211_Sstar_mem, mem);

	spin_lock_irqsave(&ieee80211_Sstar_mem_spin_lock, flags);	
	list_del(&Sstar_mem->head);
	mem_del_generation++;
	spin_unlock_irqrestore(&ieee80211_Sstar_mem_spin_lock, flags);

	kfree(Sstar_mem);
}
static ssize_t Sstar_mem_show(struct kobject *kobj,
			     struct kobj_attribute *attr, char *buf)
{
	struct ieee80211_Sstar_mem_show mem_show;
	unsigned long flags;
	struct ieee80211_Sstar_mem *Sstar_mem = NULL;
	u32 mem_in_list = 0;
	u32 mem_total_bytes = 0;
	
	mem_show.show_buff = buf;
	mem_show.show_count = 0;
	mem_show.show_size = SSTAR_MEM_SHOW_BUFF_MAX_SIZE;

	spin_lock_irqsave(&ieee80211_Sstar_mem_spin_lock, flags);
	SSTAR_MEM_SHOW_PUT(&mem_show,"mem_lis:add[%d],del[%d],left[%d]\n",
		mem_add_generation,mem_del_generation,mem_add_generation - mem_del_generation);
	list_for_each_entry(Sstar_mem, &ieee80211_Sstar_mem_list, head) {
		SSTAR_MEM_SHOW_PUT(&mem_show,"[%s][%p]\n", Sstar_mem->call_addr,Sstar_mem);
		mem_in_list++;
		mem_total_bytes += Sstar_mem->mem_len;
	}
	SSTAR_MEM_SHOW_PUT(&mem_show,"mem in use[%d],total bytes[%d],true byes[%d]\n",
		mem_in_list,mem_total_bytes,(u32)(mem_total_bytes-sizeof(struct ieee80211_Sstar_mem)*mem_in_list));
	spin_unlock_irqrestore(&ieee80211_Sstar_mem_spin_lock, flags);

	return mem_show.show_count;
}
static int ieee80211_Sstar_mem_object_int(void)
{
	int error;

	Sstar_mem_kobj = kobject_create_and_add("Sstar_mem",
					    Sstar_module_parent);
	if (!Sstar_mem_kobj)
		return -EINVAL;

	error = sysfs_create_group(Sstar_mem_kobj, &Sstar_mem_attr_group);
	if (error)
		kobject_put(Sstar_mem_kobj);
	return error;
}
static void ieee80211_Sstar_mem_object_exit(void)
{
	if(Sstar_mem_kobj == NULL)
		return;
	sysfs_remove_group(Sstar_mem_kobj, &Sstar_mem_attr_group);
	kobject_put(Sstar_mem_kobj);
}
void ieee80211_Sstar_mem_exit(void)
{
	struct ieee80211_Sstar_mem *Sstar_mem = NULL;
	unsigned long flags;
	Sstar_printk_exit("ieee80211_Sstar_mem_exit\n");
	spin_lock_irqsave(&ieee80211_Sstar_mem_spin_lock, flags);
	while (!list_empty(&ieee80211_Sstar_mem_list)) {
		Sstar_mem = list_first_entry(
			&ieee80211_Sstar_mem_list, struct ieee80211_Sstar_mem, head);

		Sstar_printk_always("%s:malloc addr(%s)\n",__func__,Sstar_mem->call_addr);
		list_del(&Sstar_mem->head);
		kfree(Sstar_mem);
	}
	spin_unlock_irqrestore(&ieee80211_Sstar_mem_spin_lock, flags);
	ieee80211_Sstar_mem_object_exit();
}

void ieee80211_Sstar_mem_int(void)
{
	Sstar_printk_init("%s:%d\n",__func__,(u32)sizeof(struct ieee80211_Sstar_mem));
	spin_lock_init(&ieee80211_Sstar_mem_spin_lock);
	INIT_LIST_HEAD(&ieee80211_Sstar_mem_list);
	ieee80211_Sstar_mem_object_int();
}
#endif
