/*
 * Low-level API for mac80211 sigmastar apollo wifi drivers
 *
 * Copyright (c) 2016, sigmastar
 *
 * Based on:
 * Copyright (c) 2010, stericsson
 * Author: Dmitry Tarnyagin <dmitry.tarnyagin@stericsson.com>
 *
 * Based on:
 * ST-Ericsson UMAC CW1200 driver which is
 * Copyright (c) 2010, ST-Ericsson
 * Author: Ajitpal Singh <ajitpal.singh@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef SSTAR_APOLLO_HWIO_SDIO_H_INCLUDED
#define SSTAR_APOLLO_HWIO_SDIO_H_INCLUDED

#include "apollo_plat.h"

/* DPLL initial values */
#define DPLL_INIT_VAL_9000		(0x00000191)
#define DPLL_INIT_VAL_CW1200		(0x0EC4F121)
#define MAX_RETRY		10
#define SDIO_TX_MAXLEN 4096*8

#define SIGMASTAR_WIFI_HDR_FLAG  (0x34353677)
#if (PROJ_TYPE>=ARES_A)
#define DOWNLOAD_ITCM_ADDR		(0x00010000)
#else
#define DOWNLOAD_ITCM_ADDR		(0x00000000)
#endif 
#define DOWNLOAD_ITCM_SIZE		(160*1024)
#define DOWNLOAD_DTCM_ADDR		(0x00800000)
#define DOWNLOAD_DTCM_SIZE		(48*1024)


#define DOWNLOAD_BLOCK_SIZE		(128)




#define SYS_BASE_ADDR_SILICON		(0)
#define PAC_BASE_ADDRESS_SILICON	(SYS_BASE_ADDR_SILICON + 0x09000000)
#define PAC_SHARED_MEMORY_SILICON	(PAC_BASE_ADDRESS_SILICON)

#define CW12000_APB(addr)		(PAC_SHARED_MEMORY_SILICON + (addr))

/* ***************************************************************
*Device register definitions
*************************************************************** */
/* WBF - SPI Register Addresses */
#define SSTAR_HIFREG_ADDR_ID_BASE		(0x0000)
/* 16/32 bits */
#define SSTAR_HIFREG_CONFIG_REG_ID		(0x0000)
/* 16/32 bits */
#define SSTAR_HIFREG_CONTROL_REG_ID		(0x0001)
/* 16 bits, Q mode W/R */
#define SSTAR_HIFREG_IN_OUT_QUEUE_REG_ID	(0x0002)
/* 32 bits, AHB bus R/W */
#define SSTAR_HIFREG_AHB_DPORT_REG_ID	(0x0003)
/* 16/32 bits */
#define SSTAR_HIFREG_SRAM_BASE_ADDR_REG_ID   (0x0004)
/* 32 bits, APB bus R/W */
#define SSTAR_HIFREG_SRAM_DPORT_REG_ID	(0x0005)
/* 32 bits, t_settle/general */
#define SSTAR_HIFREG_TSET_GEN_R_W_REG_ID	(0x0006)
/* 16 bits, Q mode read, no length */
#define SSTAR_HIFREG_FRAME_OUT_REG_ID	(0x0007)
#define SSTAR_HIFREG_ADDR_ID_MAX		(SSTAR_HIFREG_FRAME_OUT_REG_ID)

/* WBF - Control register bit set */
/* next o/p length, bit 11 to 0 */
#define SSTAR_HIFREG_CONT_NEXT_LEN_MASK	(0xCFFF)
#define SSTAR_HIFREG_CONT_NEXT_LEN_LSB_MASK	(0x0FFF)
#define SSTAR_HIFREG_CONT_NEXT_LEN_MSB_MASK	(0xC000)
#define SSTAR_HIFREG_CONT_WUP_BIT		(BIT(12))
#define SSTAR_HIFREG_CONT_RDY_BIT		(BIT(13))
#define SSTAR_HIFREG_CONT_IRQ_ENABLE		(BIT(14))
#define SSTAR_HIFREG_CONT_RDY_ENABLE		(BIT(15))
#define SSTAR_HIFREG_CONT_IRQ_RDY_ENABLE	(BIT(14)|BIT(15))


#define SSTAR_HIFREG_PS_SYNC_SDIO_FLAG	(BIT(23))
#define SSTAR_HIFREG_PS_SYNC_SDIO_CLEAN	(BIT(24))

/* SPI Config register bit set */
#define SSTAR_HIFREG_CONFIG_FRAME_BIT	(BIT(2))
#define SSTAR_HIFREG_CONFIG_WORD_MODE_BITS	(BIT(3)|BIT(4))
#define SSTAR_HIFREG_CONFIG_WORD_MODE_1	(BIT(3))
#define SSTAR_HIFREG_CONFIG_WORD_MODE_2	(BIT(4))
#define SSTAR_HIFREG_CONFIG_ERROR_0_BIT	(BIT(5))
#define SSTAR_HIFREG_CONFIG_ERROR_1_BIT	(BIT(6))
#define SSTAR_HIFREG_CONFIG_ERROR_2_BIT	(BIT(7))

/* TBD: Sure??? */
#define SSTAR_HIFREG_CONFIG_CSN_FRAME_BIT	(BIT(7))
#define SSTAR_HIFREG_CONFIG_ERROR_3_BIT	(BIT(8))
#define SSTAR_HIFREG_CONFIG_ERROR_4_BIT	(BIT(9))
/* QueueM */
#define SSTAR_HIFREG_CONFIG_ACCESS_MODE_BIT	(BIT(10))
/* AHB bus */
#define SSTAR_HIFREG_CONFIG_AHB_PFETCH_BIT	(BIT(11))
#define SSTAR_HIFREG_CONFIG_CPU_CLK_DIS_BIT	(BIT(12))
/* APB bus */
#define SSTAR_HIFREG_CONFIG_PFETCH_BIT	(BIT(13))
/* cpu reset */
#define SSTAR_HIFREG_CONFIG_CPU_RESET_BIT	(BIT(14))
#define SSTAR_HIFREG_CONFIG_CLEAR_INT_BIT	(BIT(15))

/* For CW1200 the IRQ Enable and Ready Bits are in CONFIG register */
#define SSTAR_HIFREG_CONF_IRQ_RDY_ENABLE	(BIT(16)|BIT(17))
#define SSTAR_HIFREG_CONF_RDY_IRQ_ENABLE	(BIT(17))
#define SSTAR_HIFREG_CONF_DATA_IRQ_ENABLE	(BIT(16))

#define SSTAR_HIFREG_CONFIG_CPU_RESET_BIT_2	(BIT(22))
int Sstar_data_read_unlock(struct Sstar_common *hw_priv, void *buf, u32 buf_len);
int Sstar_data_write_unlock(struct Sstar_common *hw_priv, const void *buf,
			size_t buf_len);
int Sstar_data_read(struct Sstar_common *hw_priv,
		     void *buf, u32 buf_len);

int Sstar_data_write(struct Sstar_common *hw_priv, const void *buf,
			size_t buf_len);

int Sstar_reg_read_dpll(struct Sstar_common *hw_priv, u16 addr,
		    void *buf, u32 buf_len);
int Sstar_reg_write_dpll(struct Sstar_common *hw_priv, u16 addr,
		     const void *buf, u32 buf_len);
int Sstar_reg_read(struct Sstar_common *hw_priv, u16 addr,
		    void *buf, u32 buf_len);
int Sstar_reg_write(struct Sstar_common *hw_priv, u16 addr,
		     const void *buf, u32 buf_len);
int Sstar_reg_read_unlock(struct Sstar_common *hw_priv, u16 addr,
		    void *buf, u32 buf_len);
int Sstar_reg_write_unlock(struct Sstar_common *hw_priv, u16 addr,
		     const void *buf, u32 buf_len);

static inline int Sstar_reg_read_8(struct Sstar_common *hw_priv,
				     u16 addr, u8 *val)
{
	u32 bigVal;
	int ret;
	ret = Sstar_reg_read_dpll(hw_priv, addr, &bigVal, sizeof(bigVal));
//	Sstar_dbg(SSTAR_APOLLO_DBG_MSG, "%x,bigVal,func=%s\n",bigVal,__func__);
	*val = (u8)bigVal;
	return ret;
}
static inline int Sstar_reg_write_8(struct Sstar_common *hw_priv,
				      u16 addr, u8 val)
{
	u32 bigVal = (u32)val;
//	Sstar_dbg(SSTAR_APOLLO_DBG_MSG, "%x,val,func=%s\n",val,__func__);
	return Sstar_reg_write_dpll(hw_priv, addr, &bigVal, sizeof(bigVal));
}

static inline int Sstar_reg_read_16(struct Sstar_common *hw_priv,
				     u16 addr, u16 *val)
{
	u32 bigVal;
	int ret;
	ret = Sstar_reg_read(hw_priv, addr, &bigVal, sizeof(bigVal));
	*val = (u16)bigVal;
	return ret;
}
static inline int Sstar_reg_read_16_unlock(struct Sstar_common *hw_priv,
				     u16 addr, u16 *val)
{
	u32 bigVal;
	int ret;
	ret = Sstar_reg_read_unlock(hw_priv, addr, &bigVal, sizeof(bigVal));
	*val = (u16)bigVal;
	return ret;
}

static inline int Sstar_reg_write_16(struct Sstar_common *hw_priv,
				      u16 addr, u16 val)
{
	u32 bigVal = (u32)val;
	return Sstar_reg_write(hw_priv, addr, &bigVal, sizeof(bigVal));
}

static inline int Sstar_reg_read_32(struct Sstar_common *hw_priv,
				     u16 addr, u32 *val)
{
	return Sstar_reg_read(hw_priv, addr, val, sizeof(int));
}

static inline int Sstar_reg_write_32(struct Sstar_common *hw_priv,
				      u16 addr, u32 val)
{
	return Sstar_reg_write(hw_priv, addr, &val, sizeof(int));
}
static inline int Sstar_reg_read_unlock_32(struct Sstar_common *hw_priv,
				     u16 addr, u32 *val)
{
	return Sstar_reg_read_unlock(hw_priv, addr, val, sizeof(int));
}
static inline int Sstar_reg_write_unlock_32(struct Sstar_common *hw_priv,
				      u16 addr, u32 val)
{
	return Sstar_reg_write_unlock(hw_priv, addr, &val, sizeof(int));
}
int Sstar_indirect_read_unlock(struct Sstar_common *hw_priv, u32 addr, void *buf,
			 u32 buf_len, u32 prefetch, u16 port_addr);

int Sstar_indirect_read(struct Sstar_common *hw_priv, u32 addr, void *buf,
			 u32 buf_len, u32 prefetch, u16 port_addr);
int Sstar_apb_write(struct Sstar_common *hw_priv, u32 addr, const void *buf,
		     u32 buf_len);
int Sstar_ahb_write(struct Sstar_common *priv, u32 addr, const void *buf,
                     u32 buf_len);
int Sstar_ahb_write_unlock(struct Sstar_common *priv, u32 addr, const void *buf,
                     u32 buf_len);
int Sstar_direct_read_reg_32(struct Sstar_common *hw_priv, u32 addr, u32 *val);
int Sstar_direct_write_reg_32(struct Sstar_common *hw_priv, u32 addr, u32 val);
int __Sstar_irq_enable(struct Sstar_common *priv, int enable);
void __Sstar_irq_dbgPrint(struct Sstar_common *priv);
int Sstar_direct_read_unlock(struct Sstar_common *hw_priv, u32 addr, u32 *val);
int Sstar_direct_write_unlock(struct Sstar_common *hw_priv, u32 addr, u32 val);

static inline int Sstar_apb_read(struct Sstar_common *hw_priv, u32 addr,
				  void *buf, u32 buf_len)
{
	return Sstar_indirect_read(hw_priv, addr, buf, buf_len,
		SSTAR_HIFREG_CONFIG_PFETCH_BIT, SSTAR_HIFREG_SRAM_DPORT_REG_ID);
}

static inline int Sstar_ahb_read(struct Sstar_common *hw_priv, u32 addr,
				  void *buf, size_t buf_len)
{
	return Sstar_indirect_read(hw_priv, addr, buf, buf_len,
		SSTAR_HIFREG_CONFIG_AHB_PFETCH_BIT, SSTAR_HIFREG_AHB_DPORT_REG_ID);
}
static inline int Sstar_ahb_read_unlock(struct Sstar_common *hw_priv, u32 addr,
				  void *buf, size_t buf_len)
{
	return Sstar_indirect_read_unlock(hw_priv, addr, buf, buf_len,
		SSTAR_HIFREG_CONFIG_AHB_PFETCH_BIT, SSTAR_HIFREG_AHB_DPORT_REG_ID);
}

static inline int Sstar_apb_read_32(struct Sstar_common *hw_priv,
				     u32 addr, u32 *val)
{
	return Sstar_apb_read(hw_priv, addr, val, sizeof(int));
}

static inline int Sstar_apb_write_32(struct Sstar_common *hw_priv,
				      u32 addr, u32 val)
{
	return Sstar_apb_write(hw_priv, addr, &val, sizeof(val));
}

static inline int Sstar_ahb_read_32(struct Sstar_common *hw_priv,
				     u32 addr, u32 *val)
{
	return Sstar_ahb_read(hw_priv, addr, val, sizeof(int));
}
#define WRITE_32K_ADDR_MSK	(0xfffff000)

#define WRITE_32K_ADDR	(0x16101000)
static inline int Sstar_ahb_write_32(struct Sstar_common *hw_priv,
				      u32 addr, u32 val)
{
	int ret = Sstar_ahb_write(hw_priv, addr, &val, sizeof(val));
	if((addr&WRITE_32K_ADDR_MSK)==WRITE_32K_ADDR)
	{
		mdelay(10);
	}
	return ret;
}
static inline int Sstar_ahb_read_unlock_32(struct Sstar_common *hw_priv,
				     u32 addr, u32 *val)
{
	return Sstar_ahb_read_unlock(hw_priv, addr, val, sizeof(int));
}
static inline int Sstar_ahb_write_unlock_32(struct Sstar_common *hw_priv,
				      u32 addr, u32 val)
{
	int ret;
	int retry = 0;
	while(retry<=3){
		ret = Sstar_ahb_write_unlock(hw_priv, addr, &val, sizeof(val));
		if(ret){
			Sstar_printk_err("%s\n",__func__);
		}else{
			break;
		}
	}
	if((addr&WRITE_32K_ADDR_MSK)==WRITE_32K_ADDR)
	{
		mdelay(10);
	}
	return ret;
}

#endif /* SSTAR_APOLLO_HWIO_SDIO_H_INCLUDED */
