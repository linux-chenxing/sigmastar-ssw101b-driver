#include <linux/types.h>
#include <net/Sstar_mac80211.h>
#include "apollo.h"
#include "hwio_usb.h"
#include "bh_usb.h"
#include "sbus.h"



int Sstar_ep0_read(struct Sstar_common *hw_priv, u32 addr,
				void *buf, u32 buf_len)
{

	BUG_ON(!hw_priv->sbus_ops);
	return hw_priv->sbus_ops->sbus_read_sync(hw_priv->sbus_priv,
						  addr,
						  buf, buf_len);
}

int Sstar_ep0_write(struct Sstar_common *hw_priv, u32 addr,
				const void *buf, u32 buf_len)
{
	//printk("__Sstar_reg_write sdio_reg_addr_17bit 0x%x,addr_sdio 0x%x,len %d,buf_id %d\n",sdio_reg_addr_17bit,addr_sdio,buf_len,buf_id);

	BUG_ON(!hw_priv->sbus_ops);
	return hw_priv->sbus_ops->sbus_write_sync(hw_priv->sbus_priv,
						addr,buf, buf_len);
}
#ifdef USB_CMD_UES_EP0
#define USB_EP0_FLAG 0x1234
int Sstar_ep0_write_cmd(struct Sstar_common *hw_priv, struct wsm_hdr_tx * wsm_h)
{

	int ret=0;
#ifdef USB_CMD_UES_EP0

	hw_priv->sbus_ops->power_mgmt(hw_priv->sbus_priv, false);

	u32 put = 0;
	u8 *buf = NULL;
	u32 addr = hw_priv->wsm_caps.HiHwCnfBufaddr;
	u32 size = wsm_h->len;
	u8 *data =(u8 *) wsm_h;
	
	wsm_h->flag = USB_EP0_FLAG;
	wsm_h->id &= __cpu_to_le32(~WSM_TX_SEQ(WSM_TX_SEQ_MAX));
	wsm_h->id |= cpu_to_le32(WSM_TX_SEQ(hw_priv->wsm_tx_seq_ep0));
	
	//printk("wsm_cmd_send:wsm->len=%d,id:0x%x,seq:0x%x\n",__le32_to_cpu(wsm_h->len),wsm_h->id,hw_priv->wsm_tx_seq_ep0);
	//frame_hexdump(__func__, (u8 *)wsm_h,wsm_h->len);
	//printk("\n");


	buf = Sstar_kmalloc(DOWNLOAD_BLOCK_SIZE*2, GFP_KERNEL | GFP_DMA);
	if (!buf) {
		Sstar_dbg(SSTAR_APOLLO_DBG_ERROR,
			"%s: can't allocate bootloader buffer.\n", __func__);
		ret = -ENOMEM;
		goto error;
	}

	if(size > DOWNLOAD_BLOCK_SIZE)
	{
		/*  downloading loop */
		for (put = DOWNLOAD_BLOCK_SIZE; put < size ;put += DOWNLOAD_BLOCK_SIZE) {
			u32 tx_size;

			/* calculate the block size */
			tx_size  = min((size - put),(u32)DOWNLOAD_BLOCK_SIZE);
			memcpy(buf, &data[put], tx_size);

			/* send the block to sram */
			ret = Sstar_ep0_write(hw_priv,put+addr,buf, tx_size);
			if (ret < 0) {
				Sstar_dbg(SSTAR_APOLLO_DBG_ERROR,
					"%s: can't write block at line %d.\n",
					__func__, __LINE__);
				Sstar_printk_err("%s:put = 0x%x\n",__func__,put);
				goto error;
			}
		} /* End of bootloader download loop */
		
		memcpy(buf, &data[0], DOWNLOAD_BLOCK_SIZE);
		/* send the block to sram */
		ret = Sstar_ep0_write(hw_priv,addr,buf, DOWNLOAD_BLOCK_SIZE); // last step write first block
		if (ret < 0) {
			Sstar_dbg(SSTAR_APOLLO_DBG_ERROR,
				"%s: can't write block at line %d.\n",
				__func__, __LINE__);
			Sstar_printk_err("%s\n",__func__);
			goto error;
		}
	}else
	{
		memcpy(buf, &data[0], size);
		/* send the block to sram */
		ret = Sstar_ep0_write(hw_priv,addr,buf, size);
		if (ret < 0) {
			Sstar_dbg(SSTAR_APOLLO_DBG_ERROR,
				"%s: can't write block at line %d.\n",
				__func__, __LINE__);
			Sstar_printk_err("%s\n",__func__);
			goto error;
		}
	}


	hw_priv->sbus_ops->ep0_cmd(hw_priv->sbus_priv);
	hw_priv->wsm_tx_seq_ep0 = (hw_priv->wsm_tx_seq_ep0 + 1)& WSM_TX_SEQ_MAX;
	spin_lock_bh(&hw_priv->tx_com_lock);
	++hw_priv->hw_bufs_used;
	hw_priv->wsm_cmd.ptr = NULL;
	spin_unlock_bh(&hw_priv->tx_com_lock);

error:
	hw_priv->sbus_ops->power_mgmt(hw_priv->sbus_priv, true);
	Sstar_kfree(buf);
#endif
	return ret;


}

#endif
int Sstar_fw_write(struct Sstar_common *priv, u32 addr, const void *buf,
                        u32 buf_len)
{
	return Sstar_ep0_write(priv,  addr, buf, buf_len);
}

int Sstar_direct_read_reg_32(struct Sstar_common *hw_priv, u32 addr, u32 *val)
{
    int ret;

	ret= Sstar_ep0_read(hw_priv, addr, val, sizeof(int));

	if (ret <= 0) {
		*val = 0xff;
		Sstar_dbg(SSTAR_APOLLO_DBG_ERROR,
			"%s:  can't write " \
			"config register.\n", __func__);
		goto out;
	}

out:
	return ret;
}

int Sstar_direct_write_reg_32(struct Sstar_common *hw_priv, u32 addr, u32 val)
{
	int ret = Sstar_ep0_write(hw_priv, addr, &val, sizeof(val));
	if (ret < 0) {
		Sstar_dbg(SSTAR_APOLLO_DBG_ERROR,
			"%s:  can't write " \
			"config register.\n", __func__);
		goto out;
	}

out:
	return ret;
}
#define MEMENDADDR (0x9000000+80*1024-4)
#define MEMVAL	   (0xaaffaaff)
#define SSTAR_READ_REG_TEST 			(0)
#define SSTAR_DEBUG_BUS_TEST			(0)

#if SSTAR_READ_REG_TEST
struct Sstar_reg_bit_s
{
	u32 addr;
	u32 end_bit;
	u32 start_bit;
	u32 val;
};
static struct Sstar_reg_bit_s Sstar_debug_bus_reg_bit[]=
{	
	{0x161000A0,	31,0, 0x13071307}, 
	{0x161000A4,	31,0,	0x13071307}, 
	{0x161000A8,	31,0,	0x13071307}, 
	{0x161000AC,	31,0, 0x13071307}, 
	{0x161000B0,	31,0,	0x13071307}, 
	{0x161000B4,	31,0,	0x13071307}, 
	{0x161000B8,	31,0,	0x13071307}, 
	{0x161000BC,	31,0,	0x13071307}, 
	{0x161000C0,	31,0,	0x13071307}, 
	{0x161000C4,	31,0,	0x13071307}, 
	{0x161000C8,	31,0,	0x13071307}, 
	{0x161000CC,	31,0,	0x13001307}, 
	{0x161000D0,	31,0,	0x13071300}, 
	{0x161000D4,	31,0,	0x13071307}, 
	{0x161000D8,	31,0, 0x13071307}, 
	{0x161000DC,	31,0,	0x13071307}, 
	{0x161000E0,	31,0, 0x13071307}, 
	{0x161000E4,	31,0, 0x13071307}, 
	{0x16100174,	31,0, 0x13071307}, 
	{0xffffffff,	31, 0 , 0}		 	
};

int Sstar_usb_write_bit(struct Sstar_common *hw_priv,u32 addr,u8 endBit,
	u8 startBit,u32 data )
{                                                              
	u32	uiRegValue=0;                                        
	u32 regmask=0;
	int ret = 0;
	ret=Sstar_direct_read_reg_32(hw_priv,addr,&uiRegValue); 
	if(ret<0){
		Sstar_printk_err("%s:read addr(%x) err\n",__func__,addr);
		goto rw_end;
	}                             
	regmask = ~((1<<startBit) -1);                               
	regmask &= ((1<<endBit) -1)|(1<<endBit);                     
	uiRegValue &= ~regmask;                                      
	uiRegValue |= (data <<startBit)&regmask;                     
	ret = Sstar_direct_write_reg_32(hw_priv,addr,uiRegValue);
	if(ret<0)
	{
		Sstar_printk_err("%s:write addr(%x) err\n",__func__,addr);
		goto rw_end;
	}

rw_end:
	return ret;
}                                                              

int Sstar_usb_write_bit_table(struct Sstar_common *hw_priv,struct Sstar_reg_bit_s *reg_table)
{
	int retval = 0;
	struct Sstar_reg_bit_s *preg_table =NULL;
	if(reg_table == NULL){
		retval = -1;
		goto exit;
	}
	preg_table = reg_table;

	while(preg_table->addr != 0xffffffff){
		
		retval = Sstar_usb_write_bit(hw_priv,preg_table->addr,
			preg_table->end_bit,preg_table->start_bit,preg_table->val);

		if(retval<0){
			Sstar_printk_err("%s:addr(%x),startb(%d),endb(%d),val(%x)\n",__func__,
				preg_table->addr,preg_table->start_bit,preg_table->end_bit,preg_table->val);
			break;
		}

		preg_table++;
		
	}

exit:
	return retval;
}
#endif

struct Sstar_reg_val_s{
	u32 addr;
	u32 val;
};

#if SSTAR_DEBUG_BUS_TEST
static struct Sstar_reg_val_s Sstar_debugbus_reg[]=
{	
	{0x1610009c,0x1a3b},	
	{0x161000A0,0x13071300},
	{0x161000AC,0x13071307},
	{0x161000B8,0x13071307},
	{0x161000C0,0x0b070b07},
	{0x161000C4,0x0b070b07},
	{0x161000C8,0x0b070b07},
	{0x161000CC,0x0b001307},
	{0x161000D0,0x00001300},
	{0x16100174,0x00000000},
	{0xffffffff, 0}
};
#endif
#ifdef HW_DOWN_FW

static struct Sstar_reg_val_s Sstar_usb_only_abort_en[]=
{
	{0x16100008,0xffffffff},
	{0x0b000130,0xf},
	{0x0b000134,0xf},
	{0xffffffff, 0}
};

static struct Sstar_reg_val_s Sstar_usb_all_irq_en[]=
{
	{0x16100008,0},
	{0x0b000130,0},
	{0x0b000134,0},
	{0xffffffff,0}
};

#endif
#if SSTAR_READ_REG_TEST
static struct Sstar_reg_val_s Sstar_usb_bp_table[]=
{
	{0x1610101c,0x03030010},
	{0x16101018,0x00002007},
	{0xffffffff,0}
};

static struct Sstar_reg_val_s Sstar_usb_bx_table[]=
{
	{0x1610101c,0x03030006},
	{0x16101018,0x00002006},
	{0xffffffff,0}
};
#endif


int Sstar_write_reg_table(struct Sstar_common *hw_priv,struct Sstar_reg_val_s *reg_table)
{
	int retval = 0;
	struct Sstar_reg_val_s *preg_table =NULL;
	if(reg_table == NULL){
		retval = -1;
		goto exit;
	}
	preg_table = reg_table;
	//printk(KERN_ERR "Sstar_write_reg_table++++++++\n");
	while(preg_table->addr != 0xffffffff){
		
		retval = Sstar_direct_write_reg_32(hw_priv,preg_table->addr,preg_table->val);

		if(retval<0){
			Sstar_printk_err("%s:addr(%x),val(%x)\n",__func__,
				preg_table->addr,preg_table->val);
			break;
		}

		preg_table++;
		
	}
exit:
	//printk(KERN_ERR "Sstar_write_reg_table-----------\n");

	return retval;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
int Sstar_before_load_firmware(struct Sstar_common *hw_priv)
{
	
	#define SSTAR_VOL_L					(11)
	
	u32 val;
	int ret = 0;
	#if SSTAR_READ_REG_TEST
	u32 index;
	u8 buffer[64];
	#endif
#if (PROJ_TYPE>=ARES_A)
	ret = Sstar_direct_write_reg_32(hw_priv,0x0ae0000c,0x0);
        if(ret<0)
               Sstar_printk_err("write 0x0ae0000c err\n");
#endif
	//1.2v
	#if (SSTAR_VOL_L == 12)
	#pragma message ("1.2v")
	Sstar_printk_init("+++++++++++++++++1.2v+++++++++++++++++++\n");
	ret = Sstar_direct_write_reg_32(hw_priv,0xacc0178,0x5400071);
	if(ret<0)
		Sstar_printk_err("write 0xacc0178 err\n");
	#elif(SSTAR_VOL_L == 10)
	//1.0v
	#pragma message ("1.0v")
	Sstar_printk_init("+++++++++++++++++1.0v+++++++++++++++++++\n");
	ret = Sstar_direct_write_reg_32(hw_priv,0xacc0178,0x3400071);
	if(ret<0)
		Sstar_printk_err("write 0xacc0178 err\n");
	#else
	//1.1v
	Sstar_printk_init("+++++++++++++++++1.1v++++++++++++++++++\n");
	Sstar_printk_init("===================~_~====================\n");
	#pragma message ("1.1v")
	#endif
#if 0
	ret = Sstar_write_reg_table(hw_priv,Sstar_usb_bp_table);
	if(ret<0)
	{
		printk(KERN_ERR "Sstar_write_reg_table err \n");
	}
	printk(KERN_ERR "Sstar_usb_bx_tablexxxxxxxxxxxxxxxxxxxxx\n");
#endif
	#if SSTAR_DEBUG_BUS_TEST
	/*
	*debug bus reg
	*/
	#pragma message ("debug bus en")
	Sstar_printk_init("++++++++++++debug bus en++++++++++++++++++\n");
	ret = Sstar_write_reg_table(hw_priv,Sstar_debugbus_reg);
	if(ret<0)
	{
		Sstar_printk_err("Sstar_write_reg_table err \n");
	}
	#endif
	
	#ifdef HW_DOWN_FW
	/*
	*disable irq
	*/
	ret = Sstar_write_reg_table(hw_priv,Sstar_usb_only_abort_en);
	if(ret<0)
		Sstar_printk_err("0x16100008 write err\n");
	#endif
	val = 1;
	ret = 0;
	
	#if SSTAR_READ_REG_TEST
	/*
	*read reg test
	*/
	#pragma message ("++++++++++++Sstar read reg test++++++++++++\n");
	mdelay(1000);
	ret = Sstar_direct_write_reg_32(hw_priv,MEMENDADDR,MEMVAL);
	if(ret<0)
		Sstar_printk_err("write mem err\n");
	 
    Sstar_printk_init("write mem MEMENDADDR\n");
	
	for(index=0;index<100000;index++)
	{
		ret = Sstar_direct_read_reg_32(hw_priv,MEMENDADDR,&val);
		//ret= Sstar_ep0_read(hw_priv, 0x9000000, buffer, 64);
		if(ret<0)
		{
			Sstar_printk_err("0x16400000 read err,index(%d)\n",index);
			break;
		}
		else
		{
			if(val != MEMVAL){
				Sstar_printk_err("0x16400000 read err (%x),index(%d)\n",val,index);
				break;
			}

			val = 0;
			udelay(1);
		}
	}
	Sstar_printk_init(" cxcxc read MEMENDADDR end(%d),index(%d)\n",ret,index);
	for(index=0;index<100000;index++)
	{
		ret=Sstar_direct_read_reg_32(hw_priv,0x0acc017c,&val);

		if(ret<=0)
		{
			Sstar_printk_err("0x0acc017c read err(%d),index(%d)\n",ret,index);
			break;
		}
		else
		{
			val&=0x3f;

			if(val != 0x24)
			{
				Sstar_printk_err("0x0acc017c read err(%x),index(%d)\n",val,index);
				break;
			}

			val = 0;
			udelay(1);
		}
	}
	Sstar_printk_init("read 0x0acc017c end(%d),index(%d)\n",ret,index);
	#if 1
	while(ret<=0)
	{
		mdelay(1000);
		ret=Sstar_direct_read_reg_32(hw_priv,0x0acc017c,&val);
		if(ret<=0)
		{
			Sstar_printk_err("0x0acc017c read err(%d)\n",ret);
		}else{
			Sstar_printk_err( "0x0acc017c read good(%d)\n",ret);
		}
	}
	#endif
	#endif
#ifdef  HW_DOWN_FW
#ifdef USB_HOLD_CPU_FUNC
	Sstar_direct_read_reg_32(hw_priv,0x161010cc,&val);
	val |= BIT(15);
	Sstar_printk_init("0x161010cc %d\n",val);
	Sstar_direct_write_reg_32(hw_priv,0x161010cc,val);
	Sstar_usb_ep0_hw_reset_cmd(hw_priv->sbus_priv,HW_RESET_HIF_SYSTEM_CPU,0);
	Sstar_printk_init"%s %d\n",__func__,__LINE__);
#else
	Sstar_direct_read_reg_32(hw_priv,0x16101000,&val);
	val |= BIT(8);
	Sstar_direct_write_reg_32(hw_priv,0x16101000,val);
	Sstar_direct_read_reg_32(hw_priv,0x1610007c,&val);
	val |= BIT(1);
	Sstar_direct_write_reg_32(hw_priv,0x1610007c,val);
#endif
#endif  //HARDWARE_USB_JUMP
	return 0;
}


int Sstar_after_load_firmware(struct Sstar_common *hw_priv)
{
//	u32 val32;
#if (PROJ_TYPE>=ARES_A)


	int ret;
	u32 val32;

	Sstar_write_reg_table(hw_priv,Sstar_usb_all_irq_en);

	Sstar_write_reg_table(hw_priv,Sstar_usb_all_irq_en);
//	printk("=Sstar_direct_read_reg_32(hw_priv,0x1610102c,&val32)\n");
	ret=Sstar_direct_read_reg_32(hw_priv,0x1610102c,&val32);
	if(ret<0){
		Sstar_dbg(SSTAR_APOLLO_DBG_ERROR,
			"%s: 0x1610102c: can't read register.\n", __func__);
	}
	Sstar_printk_init("%s:0x1610102c=0x%x\n",__func__, val32);
	
	val32 &= ~(0xffff0000);
	val32 |= BIT(0) | BIT(1) | (0x1 << 16);
	ret=Sstar_direct_write_reg_32(hw_priv,0x1610102c,val32);
	if(ret<0){
		Sstar_dbg(SSTAR_APOLLO_DBG_ERROR,
			"%s: 0x1610102c: can't write register.\n", __func__);
	}
	
	ret=Sstar_direct_read_reg_32(hw_priv,0x1610102c,&val32);
	if(ret<0){
		Sstar_dbg(SSTAR_APOLLO_DBG_ERROR,
			"%s: 0x1610102c: can't read register.\n", __func__);
	}	
	Sstar_printk_init("%s:0x1610102c=0x%x\n",__func__, val32);

#ifdef USB_HOLD_CPU_FUNC
	Sstar_usb_ep0_hw_reset_cmd(hw_priv->sbus_priv,HW_RUN_CPU,0);
#else
	ret=Sstar_direct_read_reg_32(hw_priv,0x16101000,&val32);
	if(ret<0){
		Sstar_dbg(SSTAR_APOLLO_DBG_ERROR,
			"%s: 0x1610102c: can't read register.\n", __func__);
	}
	
	val32 &= ~( BIT(8));
	
	Sstar_printk_init("%s:0x16101000=0x%x\n",__func__, val32);
	ret=Sstar_direct_write_reg_32(hw_priv,0x16101000,val32);
	if(ret<0){
		Sstar_dbg(SSTAR_APOLLO_DBG_ERROR,
			"%s: 0x1610102c: can't write register.\n", __func__);
	}
#endif
#elif (PROJ_TYPE==ATHENA_LITE_ECO)
        //
        u32 val32;
        u32 regdata ;
        // PC  jump to address ICCM
#ifndef HW_DOWN_FW
        hw_priv->sbus_ops->lmac_start(hw_priv->sbus_priv);
#else
        Sstar_write_reg_table(hw_priv,Sstar_usb_all_irq_en);
        val32 = 0x100;
        regdata = 0x100;
        /*reset cpu*/
        Sstar_direct_write_reg_32(hw_priv,0x16101000,val32);
        /*default_ivb_reg_enable|ilm_boot_reg &default_ivb_reg */
        Sstar_direct_write_reg_32(hw_priv,0x1610102c,0x10003);
        /*release cpu*/
        Sstar_direct_write_reg_32(hw_priv,0x16101000,0x0);

#endif  //#if HARDWARE_USB_JUMP	
#elif (PROJ_TYPE==ATHENA_LITE)
	//
	u32 val32;
	u32 regdata ;	
	// PC  jump to address ICCM
#ifndef HW_DOWN_FW
	hw_priv->sbus_ops->lmac_start(hw_priv->sbus_priv);
#else
	Sstar_write_reg_table(hw_priv,Sstar_usb_all_irq_en);	
	val32 = 0x100;
	regdata = 0x100;
	/*reset cpu*/
	Sstar_direct_write_reg_32(hw_priv,0x16101000,val32);
	/*default_ivb_reg_enable|ilm_boot_reg &default_ivb_reg */
	Sstar_direct_write_reg_32(hw_priv,0x1610102c,0x3);
	/*release cpu*/
	Sstar_direct_write_reg_32(hw_priv,0x16101000,0x0);
	
#endif  //#if HARDWARE_USB_JUMP
#elif (PROJ_TYPE==ATHENA_B)
	u32 regdata ;	
#ifdef HW_DOWN_FW
	Sstar_write_reg_table(hw_priv,Sstar_usb_all_irq_en);
#endif
	//
	// PC  jump to address ICCM
	//hw_priv->sbus_ops->lmac_start(hw_priv->sbus_priv);
	Sstar_direct_read_reg_32(hw_priv,0x1610007c,&regdata);
	regdata |= BIT(1);
	Sstar_direct_write_reg_32(hw_priv,0x1610007c,regdata);	
	Sstar_direct_read_reg_32(hw_priv,0x16101000,&regdata);
	regdata |= BIT(8);
	Sstar_direct_write_reg_32(hw_priv,0x16101000,regdata);
	regdata &= ~BIT(8);
	Sstar_direct_write_reg_32(hw_priv,0x16101000,regdata);

#endif //#if (PROJ_TYPE==ARES_A)
	//PC jump ok
	hw_priv->init_done = 1;
	 /*Sstar receive packet form the device*/
	hw_priv->sbus_ops->lock(hw_priv->sbus_priv);
	hw_priv->sbus_ops->sbus_memcpy_fromio(hw_priv->sbus_priv,0x2,NULL,RX_BUFFER_SIZE);
	hw_priv->sbus_ops->unlock(hw_priv->sbus_priv);
	
	return hw_priv->sbus_ops->sbus_wait_data_xmited ? 
		   hw_priv->sbus_ops->sbus_wait_data_xmited(hw_priv->sbus_priv) : 
		   0;
}

void Sstar_firmware_init_check(struct Sstar_common *hw_priv)
{
	int status =0;	
	struct sk_buff *skb = Sstar_dev_alloc_skb(1600);
	struct wsm_hdr_tx * hdr ;
	
	//printk("Sstar_firmware_init_check send data %p\n",	hw_priv->save_buf );
	
	//spin_lock_bh(&hw_priv->wsm_cmd.lock);
	mutex_lock(&hw_priv->wsm_cmd_mux);
	hw_priv->save_buf =  skb->data;
	hdr =  (struct wsm_hdr_tx *)skb->data;
	hdr->len = 1538;
	hdr->id = WSM_FIRMWARE_CHECK_ID;
	hdr->usb_len= 1538;
	hdr->usb_id = WSM_FIRMWARE_CHECK_ID;
	hw_priv->save_buf_len = 1538;
	hw_priv->save_buf_vif_selected= -1;
	//hw_priv->wsm_tx_seq = 0;
	hw_priv->sbus_ops->lock(hw_priv->sbus_priv);
	status = hw_priv->sbus_ops->sbus_memcpy_toio(hw_priv->sbus_priv,0x1,NULL,TX_BUFFER_SIZE);
	hw_priv->sbus_ops->unlock(hw_priv->sbus_priv);
	hw_priv->save_buf = NULL;
	//wait usb tx complete
	//mdelay(50);
	//spin_unlock_bh(&hw_priv->wsm_cmd.lock);
	mutex_unlock(&hw_priv->wsm_cmd_mux);

	if(hw_priv->sbus_ops->sbus_wait_data_xmited)
		hw_priv->sbus_ops->sbus_wait_data_xmited(hw_priv->sbus_priv);
	else 
		mdelay(100);//delay 100ms may be not safely,but have no idea
	Sstar_dev_kfree_skb(skb);
}

