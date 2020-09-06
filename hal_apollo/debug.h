/*
 * DebugFS code for sigmastar APOLLO mac80211 driver
 * *
 * Copyright (c) 2016, sigmastar
 * Author:
 *
 * Based on apollo code
 * Copyright (c) 2011, ST-Ericsson
 * Author: Dmitry Tarnyagin <dmitry.tarnyagin@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef SSTAR_APOLLO_DEBUG_H_INCLUDED
#define SSTAR_APOLLO_DEBUG_H_INCLUDED


struct cw200_common;


#define SSTAR_APOLLO_DBG_MSG		0x00000001
#define SSTAR_APOLLO_DBG_NIY		0x00000002
#define SSTAR_APOLLO_DBG_SBUS		0x00000004
#define SSTAR_APOLLO_DBG_INIT		0x00000008
#define SSTAR_APOLLO_DBG_ERROR	0x00000010
#define SSTAR_APOLLO_DBG_DCXO_DPLL  0x00000020
#define SSTAR_APOLLO_DBG_LEVEL	(SSTAR_APOLLO_DBG_INIT|SSTAR_APOLLO_DBG_NIY|SSTAR_APOLLO_DBG_ERROR)


#define FUNC_ENTER() //printk("%s %d++\n",__func__,__LINE__)
#define FUNC_EXIT() //printk("%s %d--\n",__func__,__LINE__)


#define Sstar_dbg(level, ...)				\
 if ((level) & SSTAR_APOLLO_DBG_LEVEL)		\
		Sstar_printk_init(__VA_ARGS__);	\


 /* TODO It should be removed before official delivery */
 static __inline void frame_hexdump(char *prefix, u8 *data, int len)
 {
	 int i;

	Sstar_printk_always("%s hexdump:\n", prefix);
	 for (i = 0; i < len; i++) {
	 	if((i % 16)==0)
			Sstar_printk_always("\n");
		Sstar_printk_always("%02x ", data[i]);

	 }
	Sstar_printk_always("\n");
 }
void SSTARWIFI_DBG_PRINT2(const char * func,const int line,unsigned int data);
void SSTARWIFI_DBG_PRINT(const char * func,const int line);

struct Sstar_debug_param{
	void *private;
	char *buff;
	int size;
	int cout;
};

#ifdef CONFIG_SSTAR_APOLLO_DEBUGFS
typedef struct seq_file *P_VDEBUG_SEQFILE;
#define VDEBUG_SEQFILE         struct seq_file 
#define VDEBUG_PRINTF(...)  	seq_printf(__VA_ARGS__)
#define VDEBUG_PUTS(a,b)		seq_puts(a,b)
#define VDEBUG_PRIV(seq)		((seq)->private)
#else

typedef struct Sstar_debug_param *P_VDEBUG_SEQFILE;
#define VDEBUG_SEQFILE         struct Sstar_debug_param
#define VDEBUG_PRINTF(a,...)  	a->cout += snprintf(a->buff + a->cout, a->size - a->cout, __VA_ARGS__)
#define VDEBUG_PUTS(a,b)		a->cout += snprintf(a->buff + a->cout, a->size - a->cout, b)
#define VDEBUG_PRIV(seq)		((seq)->private)
#endif
#ifdef CONFIG_SSTAR_APOLLO_DEBUG
int Sstar_debug_init_common(struct Sstar_common *hw_priv);
int Sstar_debug_init_priv(struct Sstar_common *hw_priv,
			   struct Sstar_vif *priv);
void Sstar_debug_release_common(struct Sstar_common *hw_priv);
void Sstar_debug_release_priv(struct Sstar_vif *priv);

static inline void Sstar_debug_txed(struct Sstar_vif *priv)
{
	++priv->debug.tx;
}

static inline void Sstar_debug_txed_agg(struct Sstar_vif *priv)
{
	++priv->debug.tx_agg;
}

static inline void Sstar_debug_txed_multi(struct Sstar_vif *priv,
					   int count)
{
	++priv->debug.tx_multi;
	priv->debug.tx_multi_frames += count;
}

static inline void Sstar_debug_rxed(struct Sstar_vif *priv)
{
	++priv->debug.rx;
}

static inline void Sstar_debug_rxed_agg(struct Sstar_vif *priv)
{
	++priv->debug.rx_agg;
}

static inline void Sstar_debug_tx_cache_miss(struct Sstar_common *common)
{
	++common->debug->tx_cache_miss;
}

static inline void Sstar_debug_tx_align(struct Sstar_vif *priv)
{
	++priv->debug.tx_align;
}

static inline void Sstar_debug_tx_ttl(struct Sstar_vif *priv)
{
	++priv->debug.tx_ttl;
}

static inline void Sstar_debug_tx_burst(struct Sstar_common *hw_priv)
{
	++hw_priv->debug->tx_burst;
}

static inline void Sstar_debug_rx_burst(struct Sstar_common *hw_priv)
{
	++hw_priv->debug->rx_burst;
}

static inline void Sstar_debug_ba(struct Sstar_common *hw_priv,
				   int ba_cnt, int ba_acc, int ba_cnt_rx,
				   int ba_acc_rx)
{
	hw_priv->debug->ba_cnt = ba_cnt;
	hw_priv->debug->ba_acc = ba_acc;
	hw_priv->debug->ba_cnt_rx = ba_cnt_rx;
	hw_priv->debug->ba_acc_rx = ba_acc_rx;
}

int Sstar_print_fw_version(struct Sstar_common *hw_priv, u8* buf, size_t len);
int Sstar_status_show_priv(VDEBUG_SEQFILE * seq, void *v);
int Sstar_ht_show_info(VDEBUG_SEQFILE * seq, void *v);
int Sstar_wifi_show_status(VDEBUG_SEQFILE * seq, void *v);
int Sstar_status_show_common(VDEBUG_SEQFILE * seq, void *v);
int Sstar_counters_show(VDEBUG_SEQFILE * seq, void *v);
int Sstar_statistics_show(P_VDEBUG_SEQFILE seq, void *v);
int Sstar_pkt_show(P_VDEBUG_SEQFILE seq, void *v);


#else /* CONFIG_SSTAR_APOLLO_DEBUGFS */

static inline int Sstar_debug_init_common(struct Sstar_common *hw_priv)
{
	return 0;
}

static inline int Sstar_debug_init_priv(struct Sstar_common *hw_priv,
			   struct Sstar_vif *priv)
{
	return 0;
}

static inline void Sstar_debug_release_common(struct Sstar_common *hw_priv)
{
}

static inline void Sstar_debug_release_priv(struct Sstar_vif *priv)
{
}

static inline void Sstar_debug_txed(struct Sstar_vif *priv)
{
}

static inline void Sstar_debug_txed_agg(struct Sstar_vif *priv)
{
}

static inline void Sstar_debug_txed_multi(struct Sstar_vif *priv,
					   int count)
{
}

static inline void Sstar_debug_rxed(struct Sstar_vif *priv)
{
}

static inline void Sstar_debug_rxed_agg(struct Sstar_vif *priv)
{
}

static inline void Sstar_debug_tx_cache_miss(struct Sstar_common *priv)
{
}

static inline void Sstar_debug_tx_align(struct Sstar_vif *priv)
{
}

static inline void Sstar_debug_tx_ttl(struct Sstar_vif *priv)
{
}

static inline void Sstar_debug_tx_burst(struct Sstar_common *priv)
{
}

static inline void Sstar_debug_rx_burst(struct Sstar_common *priv)
{
}

static inline void Sstar_debug_ba(struct Sstar_common *hw_priv,
				   int ba_cnt, int ba_acc, int ba_cnt_rx,
				   int ba_acc_rx)
{
}


static inline int Sstar_status_show_priv(VDEBUG_SEQFILE * seq, void *v)
{
	return 0;
}
static inline int Sstar_ht_show_info(VDEBUG_SEQFILE * seq, void *v)
{
	return 0;
}
static inline int Sstar_wifi_show_status(VDEBUG_SEQFILE * seq, void *v)
{
	return 0;
}

static inline int Sstar_status_show_common(VDEBUG_SEQFILE * seq, void *v)
{
	return 0;
}
static inline int Sstar_counters_show(VDEBUG_SEQFILE * seq, void *v)
{
	return 0;
}
static inline int Sstar_statistics_show(P_VDEBUG_SEQFILE seq, void *v)
{
	return 0;
}
static inline int Sstar_pkt_show(P_VDEBUG_SEQFILE seq, void *v)
{
	return 0;
}

//int Sstar_print_fw_version(struct Sstar_vif *priv, u8* buf, size_t len)
//{
//}

#endif /* CONFIG_SSTAR_APOLLO_DEBUGFS */

#endif /* SSTAR_APOLLO_DEBUG_H_INCLUDED */
