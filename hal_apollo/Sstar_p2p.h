#ifndef _SSTAR_P2P_H_
#define _SSTAR_P2P_H_
int TxRxPublicActionFrame(u8 *pframe ,u32 len,int bTx);

#ifdef SSTAR_P2P_CHANGE
void Sstar_parase_p2p_scan_resp(struct Sstar_vif *priv,struct sk_buff *skb);
bool Sstar_parase_p2p_action_frame(struct Sstar_vif *priv,struct sk_buff *skb,bool tx);
bool Sstar_parase_p2p_mgmt_frame(struct Sstar_vif *priv,struct sk_buff *skb,bool tx);
#endif

#endif  //_SSTAR_P2P_H_

