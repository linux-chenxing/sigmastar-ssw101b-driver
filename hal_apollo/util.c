
#include <net/atbm_mac80211.h>
#include <linux/kthread.h>
#include <linux/dma-mapping.h>
#include <linux/skbuff.h>

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
/* Must be called from BH thraed. */
void atbm_enable_powersave(struct atbm_vif *priv,
			     bool enable)
{
	atbm_dbg(ATBM_APOLLO_DBG_MSG, "[BH] Powerave is %s.\n",
			enable ? "enabled" : "disabled");
	priv->powersave_enabled = enable;
}

void atbm_bh_wakeup(struct atbm_common *hw_priv)
{
	atbm_dbg(ATBM_APOLLO_DBG_MSG, "[BH] wakeup.\n");
	if(hw_priv->sbus_ops->sbus_xmit_schedule)
		hw_priv->sbus_ops->sbus_xmit_schedule(hw_priv->sbus_priv);
	else if (atomic_add_return(1, &hw_priv->bh_tx) == 1){
		wake_up(&hw_priv->bh_wq);
	}
}

int wsm_release_vif_tx_buffer(struct atbm_common *hw_priv, int if_id,
				int count)
{
	int ret = 0;

	spin_lock_bh(&hw_priv->tx_com_lock);
	hw_priv->hw_bufs_used_vif[if_id] -= count;

	if (WARN_ON(hw_priv->hw_bufs_used_vif[if_id] < 0)){
		atbm_printk_err( "%s:[%d][%d]\n",__func__,if_id,hw_priv->hw_bufs_used_vif[if_id]);
		hw_priv->hw_bufs_used_vif[if_id] = 0;
		//BUG_ON(1);
		//ret = -1;
	}

	spin_unlock_bh(&hw_priv->tx_com_lock);

	if (!hw_priv->hw_bufs_used_vif[if_id])
		wake_up(&hw_priv->bh_evt_wq);

	return ret;
}


int wsm_release_vif_tx_buffer_Nolock(struct atbm_common *hw_priv, int if_id,
				int count)
{
	int ret = 0;

	hw_priv->hw_bufs_used_vif[if_id] -= count;

	if (!hw_priv->hw_bufs_used_vif[if_id])
		wake_up(&hw_priv->bh_evt_wq);

	if (WARN_ON(hw_priv->hw_bufs_used_vif[if_id] < 0)){
		atbm_printk_err( "%s:[%d][%d]\n",__func__,if_id,hw_priv->hw_bufs_used_vif[if_id]);
		hw_priv->hw_bufs_used_vif[if_id] =0;
		//BUG_ON(1);
		//ret = -1;
	}

	return ret;
}
void atbm_monitor_pc(struct atbm_common *hw_priv)
{
	u32 testreg1[10] = {0};
	u32 testreg_pc = 0;
	u32 testreg_ipc = 0;
	u32 val28;
	u32 val20;
	int i = 0;

	atbm_direct_write_reg_32(hw_priv,0x16100050,1);
	//atbm_direct_read_reg_32(hw_priv,0x18e00014,&testreg1);
	atbm_direct_read_reg_32(hw_priv,0x16100054,&testreg_pc);
	atbm_direct_read_reg_32(hw_priv,0x16100058,&testreg_ipc);
	atbm_direct_read_reg_32(hw_priv,0x16101028,&val28);
	atbm_direct_read_reg_32(hw_priv,0x16101020,&val20);
	//atbm_direct_read_reg_32(hw_priv,0x16400000,&testreg_uart);

	for(i=0;i<10;i++){
		atbm_direct_read_reg_32(hw_priv,hw_priv->wsm_caps.exceptionaddr+4*i+4,&testreg1[i]);
	}
	atbm_direct_write_reg_32(hw_priv,0x16100050,0);

	atbm_printk_err("ERROR !! pc:[%x],ipc[%x] \n",testreg_pc,testreg_ipc);
	atbm_printk_err("ERROR !! reg0:[%x],reg1[%x],reg2[%x],reg3[%x],reg4[%x],reg5[%x],reg6[%x] \n",
														testreg1[0],
														testreg1[1],
														testreg1[2],
														testreg1[3],
														testreg1[4],
														testreg1[5],
														testreg1[6]);

	atbm_printk_err( "%s:0x16101028(%x)\n",__func__,val28);	
	atbm_printk_err( "%s:0x16101020(%x)\n",__func__,val20);
	
}


#ifdef	ATBM_WIFI_QUEUE_LOCK_BUG

void atbm_set_priv_queue_cap(struct atbm_vif *priv)
{
	struct atbm_common	*hw_priv = priv->hw_priv;
	struct atbm_vif *other_priv;
	u8 i = 0;

	priv->queue_cap = ATBM_QUEUE_SINGLE_CAP;
	atbm_for_each_vif(hw_priv, other_priv, i) {
		if(other_priv == NULL)
			continue;
		if(other_priv == priv)
			continue;
		if(other_priv->join_status <=  ATBM_APOLLO_JOIN_STATUS_MONITOR)
			continue;
		other_priv->queue_cap = ATBM_QUEUE_COMB_CAP;
		priv->queue_cap = ATBM_QUEUE_COMB_CAP;
		atbm_printk_err( "%s:[%d],queue_cap[%d]\n",__func__,other_priv->if_id,other_priv->queue_cap);
	}

	atbm_printk_err( "%s:[%d],queue_cap[%d]\n",__func__,priv->if_id,priv->queue_cap);
}

void atbm_clear_priv_queue_cap(struct atbm_vif *priv)
{
	struct atbm_common	*hw_priv = priv->hw_priv;
	struct atbm_vif *other_priv;
	struct atbm_vif *prev_priv = NULL;
	u8 i = 0;

	priv->queue_cap = ATBM_QUEUE_DEFAULT_CAP;

	atbm_for_each_vif(hw_priv, other_priv, i) {

		if(other_priv == NULL)
			continue;
		if(other_priv == priv)
			continue;
		if(other_priv->join_status <=  ATBM_APOLLO_JOIN_STATUS_MONITOR)
			continue;
		
		other_priv->queue_cap = ATBM_QUEUE_SINGLE_CAP;

		if(prev_priv == NULL){
			prev_priv = other_priv;
			continue;
		}

		prev_priv->queue_cap = ATBM_QUEUE_COMB_CAP;
		other_priv->queue_cap = ATBM_QUEUE_COMB_CAP;
		prev_priv = other_priv;
		
		atbm_printk_err( "%s:[%d],queue_cap[%d]\n",__func__,prev_priv->if_id,prev_priv->queue_cap);
	}
}
#endif

void atbm_xmit_linearize(struct atbm_common	*hw_priv,
	 struct wsm_tx *wsm,char *xmit,int xmit_len)
{
	int wsm_id = __le16_to_cpu(wsm->hdr.id) & 0x3F;	
	
	while(wsm_id == WSM_TRANSMIT_REQ_MSG_ID){
		
		u8 queueId = wsm_queue_id_to_linux(wsm->queueId & 0x03);
		struct atbm_queue *queue = &hw_priv->tx_queue[queueId];
		const struct atbm_txpriv *txpriv = NULL;
		struct sk_buff *skb = NULL;
		int sg_len = 0;
		int sg = 0;
		struct ieee80211_tx_info *tx_info;
		
		if(atbm_queue_get_skb(queue,wsm->packetID,
				&skb, &txpriv) != 0){
			WARN_ON(1);
			break;
		}

		BUG_ON((void *)skb->data != (void *)wsm);

		if(!skb_is_nonlinear(skb)){
			break;
		}

		tx_info = IEEE80211_SKB_CB(skb);
		
		printk_once(KERN_ERR "%s:sg process\n",__func__);

		memcpy(xmit,skb->data,skb_headlen(skb));

		sg_len += skb_headlen(skb);
		xmit += sg_len;
		
		for (sg = 0; sg < skb_shinfo(skb)->nr_frags; sg++){
			
			skb_frag_t *frag = &skb_shinfo(skb)->frags[sg];
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 2, 0))			
			memcpy(xmit,page_address(frag->page) + frag->page_offset,frag->size);
#else
			memcpy(xmit,page_address(frag->page.p) + frag->page_offset,frag->size);
#endif
			xmit += frag->size;
			sg_len += frag->size;
		}
		
		if(tx_info->sg_tailneed){
			printk_once(KERN_ERR "%s sg_tailneed(%d)\n",__func__,tx_info->sg_tailneed);
			memset(xmit,0,tx_info->sg_tailneed);
			if(sg_len + tx_info->sg_tailneed != xmit_len){
				atbm_printk_err("%s:sg_len(%d),xmit_len(%d),sg_tailneed(%d)\n",__func__,sg_len,xmit_len,tx_info->sg_tailneed);
				WARN_ON_ONCE(1);
			}

			sg_len += tx_info->sg_tailneed;
		}
		WARN_ON_ONCE(sg_len != xmit_len);
		return;		
	}

	memcpy(xmit,wsm,xmit_len);
}
