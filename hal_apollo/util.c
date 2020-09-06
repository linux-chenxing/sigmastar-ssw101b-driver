
#include <net/Sstar_mac80211.h>
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
void Sstar_enable_powersave(struct Sstar_vif *priv,
			     bool enable)
{
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG, "[BH] Powerave is %s.\n",
			enable ? "enabled" : "disabled");
	priv->powersave_enabled = enable;
}

void Sstar_bh_wakeup(struct Sstar_common *hw_priv)
{
	Sstar_dbg(SSTAR_APOLLO_DBG_MSG, "[BH] wakeup.\n");
	if(hw_priv->sbus_ops->sbus_xmit_schedule)
		hw_priv->sbus_ops->sbus_xmit_schedule(hw_priv->sbus_priv);
	else if (atomic_add_return(1, &hw_priv->bh_tx) == 1){
		wake_up(&hw_priv->bh_wq);
	}
}

int wsm_release_vif_tx_buffer(struct Sstar_common *hw_priv, int if_id,
				int count)
{
	int ret = 0;

	spin_lock_bh(&hw_priv->tx_com_lock);
	hw_priv->hw_bufs_used_vif[if_id] -= count;

	if (WARN_ON(hw_priv->hw_bufs_used_vif[if_id] < 0)){
		Sstar_printk_err( "%s:[%d][%d]\n",__func__,if_id,hw_priv->hw_bufs_used_vif[if_id]);
		hw_priv->hw_bufs_used_vif[if_id] = 0;
		//BUG_ON(1);
		//ret = -1;
	}

	spin_unlock_bh(&hw_priv->tx_com_lock);

	if (!hw_priv->hw_bufs_used_vif[if_id])
		wake_up(&hw_priv->bh_evt_wq);

	return ret;
}


int wsm_release_vif_tx_buffer_Nolock(struct Sstar_common *hw_priv, int if_id,
				int count)
{
	int ret = 0;

	hw_priv->hw_bufs_used_vif[if_id] -= count;

	if (!hw_priv->hw_bufs_used_vif[if_id])
		wake_up(&hw_priv->bh_evt_wq);

	if (WARN_ON(hw_priv->hw_bufs_used_vif[if_id] < 0)){
		Sstar_printk_err( "%s:[%d][%d]\n",__func__,if_id,hw_priv->hw_bufs_used_vif[if_id]);
		hw_priv->hw_bufs_used_vif[if_id] =0;
		//BUG_ON(1);
		//ret = -1;
	}

	return ret;
}
void Sstar_monitor_pc(struct Sstar_common *hw_priv)
{
	u32 testreg1[10] = {0};
	u32 testreg_pc = 0;
	u32 testreg_ipc = 0;
	u32 val28;
	u32 val20;
	int i = 0;

	Sstar_direct_write_reg_32(hw_priv,0x16100050,1);
	//Sstar_direct_read_reg_32(hw_priv,0x18e00014,&testreg1);
	Sstar_direct_read_reg_32(hw_priv,0x16100054,&testreg_pc);
	Sstar_direct_read_reg_32(hw_priv,0x16100058,&testreg_ipc);
	Sstar_direct_read_reg_32(hw_priv,0x16101028,&val28);
	Sstar_direct_read_reg_32(hw_priv,0x16101020,&val20);
	//Sstar_direct_read_reg_32(hw_priv,0x16400000,&testreg_uart);

	for(i=0;i<10;i++){
		Sstar_direct_read_reg_32(hw_priv,hw_priv->wsm_caps.exceptionaddr+4*i+4,&testreg1[i]);
	}
	Sstar_direct_write_reg_32(hw_priv,0x16100050,0);

	Sstar_printk_err("ERROR !! pc:[%x],ipc[%x] \n",testreg_pc,testreg_ipc);
	Sstar_printk_err("ERROR !! reg0:[%x],reg1[%x],reg2[%x],reg3[%x],reg4[%x],reg5[%x],reg6[%x] \n",
														testreg1[0],
														testreg1[1],
														testreg1[2],
														testreg1[3],
														testreg1[4],
														testreg1[5],
														testreg1[6]);

	Sstar_printk_err( "%s:0x16101028(%x)\n",__func__,val28);	
	Sstar_printk_err( "%s:0x16101020(%x)\n",__func__,val20);
	
}


#ifdef	SSTAR_WIFI_QUEUE_LOCK_BUG

void Sstar_set_priv_queue_cap(struct Sstar_vif *priv)
{
	struct Sstar_common	*hw_priv = priv->hw_priv;
	struct Sstar_vif *other_priv;
	u8 i = 0;

	priv->queue_cap = SSTAR_QUEUE_SINGLE_CAP;
	Sstar_for_each_vif(hw_priv, other_priv, i) {
		if(other_priv == NULL)
			continue;
		if(other_priv == priv)
			continue;
		if(other_priv->join_status <=  SSTAR_APOLLO_JOIN_STATUS_MONITOR)
			continue;
		other_priv->queue_cap = SSTAR_QUEUE_COMB_CAP;
		priv->queue_cap = SSTAR_QUEUE_COMB_CAP;
		Sstar_printk_err( "%s:[%d],queue_cap[%d]\n",__func__,other_priv->if_id,other_priv->queue_cap);
	}

	Sstar_printk_err( "%s:[%d],queue_cap[%d]\n",__func__,priv->if_id,priv->queue_cap);
}

void Sstar_clear_priv_queue_cap(struct Sstar_vif *priv)
{
	struct Sstar_common	*hw_priv = priv->hw_priv;
	struct Sstar_vif *other_priv;
	struct Sstar_vif *prev_priv = NULL;
	u8 i = 0;

	priv->queue_cap = SSTAR_QUEUE_DEFAULT_CAP;

	Sstar_for_each_vif(hw_priv, other_priv, i) {

		if(other_priv == NULL)
			continue;
		if(other_priv == priv)
			continue;
		if(other_priv->join_status <=  SSTAR_APOLLO_JOIN_STATUS_MONITOR)
			continue;
		
		other_priv->queue_cap = SSTAR_QUEUE_SINGLE_CAP;

		if(prev_priv == NULL){
			prev_priv = other_priv;
			continue;
		}

		prev_priv->queue_cap = SSTAR_QUEUE_COMB_CAP;
		other_priv->queue_cap = SSTAR_QUEUE_COMB_CAP;
		prev_priv = other_priv;
		
		Sstar_printk_err( "%s:[%d],queue_cap[%d]\n",__func__,prev_priv->if_id,prev_priv->queue_cap);
	}
}
#endif

void Sstar_xmit_linearize(struct Sstar_common	*hw_priv,
	 struct wsm_tx *wsm,char *xmit,int xmit_len)
{
	int wsm_id = __le16_to_cpu(wsm->hdr.id) & 0x3F;	
	
	while(wsm_id == WSM_TRANSMIT_REQ_MSG_ID){
		
		u8 queueId = wsm_queue_id_to_linux(wsm->queueId & 0x03);
		struct Sstar_queue *queue = &hw_priv->tx_queue[queueId];
		const struct Sstar_txpriv *txpriv = NULL;
		struct sk_buff *skb = NULL;
		int sg_len = 0;
		int sg = 0;
		struct ieee80211_tx_info *tx_info;
		
		if(Sstar_queue_get_skb(queue,wsm->packetID,
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
				Sstar_printk_err("%s:sg_len(%d),xmit_len(%d),sg_tailneed(%d)\n",__func__,sg_len,xmit_len,tx_info->sg_tailneed);
				WARN_ON_ONCE(1);
			}

			sg_len += tx_info->sg_tailneed;
		}
		WARN_ON_ONCE(sg_len != xmit_len);
		return;		
	}

	memcpy(xmit,wsm,xmit_len);
}

int Sstar_save_efuse(struct Sstar_common *hw_priv,struct efuse_headr *efuse_save)
{
	int ret = 0;
	int iResult=0;
	//struct Sstar_vif *vif;
	struct efuse_headr efuse_bak;
	
	/*
	*LMC_STATUS_CODE__EFUSE_VERSION_CHANGE	failed because efuse version change  
	*LMC_STATUS_CODE__EFUSE_FIRST_WRITE, 		failed because efuse by first write   
	*LMC_STATUS_CODE__EFUSE_PARSE_FAILED,		failed because efuse data wrong, cannot be parase
	*LMC_STATUS_CODE__EFUSE_FULL,				failed because efuse have be writen full
	*/
	ret = wsm_efuse_change_data_cmd(hw_priv, efuse_save,0);
	if (ret == LMC_STATUS_CODE__EFUSE_FIRST_WRITE)
	{
		Sstar_printk_err("first write\n");
		iResult = -3;
	}else if (ret == LMC_STATUS_CODE__EFUSE_PARSE_FAILED)
	{
		Sstar_printk_err("parse failed\n");
		iResult = -4;
	}else if (ret == LMC_STATUS_CODE__EFUSE_FULL)
	{
		Sstar_printk_err("efuse full\n");
		iResult = -5;
	}else if (ret == LMC_STATUS_CODE__EFUSE_VERSION_CHANGE)
	{
		Sstar_printk_err("efuse version change\n");
		iResult = -6;
	}else
	{
		iResult = 0;
	}
	if (iResult == 0)
	{
		//frame_hexdump("efuse_d", efuse_save, sizeof(struct efuse_headr));
		memset(&efuse_bak,0,sizeof(struct efuse_headr));
		wsm_get_efuse_data(hw_priv,(void *)&efuse_bak, sizeof(struct efuse_headr));

		if(efuse_bak.specific != 0)
		{
			//sigmastar oid
			efuse_save->specific = efuse_bak.specific;
		}
		
		if(memcmp((void *)&efuse_bak, efuse_save, sizeof(struct efuse_headr)) !=0)
		{
			frame_hexdump("efuse_bak", (u8 *)&efuse_bak, sizeof(struct efuse_headr));
			iResult = -2;
		}else
		{
			iResult = 0;
		}
	}
	return iResult;
}

