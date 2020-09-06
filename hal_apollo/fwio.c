/*
 * Firmware I/O code for mac80211 sigmastar APOLLO drivers
 * *
 * Copyright (c) 2016, sigmastar
 * Author:
 *
 * Based on apollo code
 * Copyright (c) 2010, ST-Ericsson
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

#include <linux/init.h>
#include <linux/vmalloc.h>
#include <linux/sched.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/debugfs.h>

#include "apollo.h"
#include "fwio.h"
#include "hwio.h"
#include "sbus.h"
#include "debug.h"
#include "bh.h"
#include "dcxo_dpll.h"

#ifdef SSTAR_USE_SAVED_FW
#pragma message("Suspend Save Firmware")
#endif
#ifdef CONFIG_USE_FW_H
#pragma message("Use Firmware.h")
#endif
static char *fw = FIRMWARE_DEFAULT_PATH;
module_param(fw, charp, 0644);
MODULE_PARM_DESC(fw, "Override platform_data firmware file");
#pragma message(FIRMWARE_DEFAULT_PATH)


struct firmware_headr {
	u32 flags; /*0x34353677*/
	u32 version;
	u32 iccm_len;
	u32 dccm_len;
	u32 reserve[3];
	u16 reserve2;
	u16 checksum;
};

struct firmware_sigmastar {
	struct firmware_headr hdr;
	u8 *fw_iccm;
	u8 *fw_dccm;
};
static struct firmware_sigmastar Sstar_fw;

void Sstar_release_firmware(void)
{
	Sstar_printk_exit("Sstar_release_firmware\n");
	if(Sstar_fw.fw_dccm)
	{
		vfree(Sstar_fw.fw_dccm);
		Sstar_fw.fw_dccm = NULL;
	}
	if(Sstar_fw.fw_iccm)
	{
		vfree(Sstar_fw.fw_iccm);
		Sstar_fw.fw_iccm = NULL;
	}
}
int Sstar_init_firmware(void)
{
	Sstar_printk_init("Sstar_init_firmware\n");
	memset(&Sstar_fw,0,sizeof(struct firmware_sigmastar));
	return 0;
}

int Sstar_set_firmare(struct firmware_sigmastar *fw)
{
#ifdef SSTAR_USE_SAVED_FW
	if(!fw || (!fw->fw_dccm&&!fw->fw_iccm))
	{
		Sstar_printk_err(KERN_ERR "fw is err\n");
		return -1;
	}

	if(Sstar_fw.fw_dccm || Sstar_fw.fw_iccm)
	{
		Sstar_printk_err("Sstar_fw has been set\n");
		return -1;
	}
	memcpy(&Sstar_fw.hdr,&fw->hdr,sizeof(struct firmware_headr));
	
	if(Sstar_fw.hdr.iccm_len)
	{
		Sstar_fw.fw_iccm = vmalloc(Sstar_fw.hdr.iccm_len);
		Sstar_printk_err("%s:fw_iccm(%p)\n",__func__,Sstar_fw.fw_iccm);
		if(!Sstar_fw.fw_iccm)
		{
			Sstar_printk_err("alloc Sstar_fw.fw_iccm err\n");
			goto err;
		}
		memcpy(Sstar_fw.fw_iccm,fw->fw_iccm,Sstar_fw.hdr.iccm_len);
	}

	if(Sstar_fw.hdr.dccm_len)
	{
		Sstar_fw.fw_dccm= vmalloc(Sstar_fw.hdr.dccm_len);
		
		Sstar_printk_err("%s:fw_dccm(%p)\n",__func__,Sstar_fw.fw_dccm);
		if(!Sstar_fw.fw_dccm)
		{
			Sstar_printk_err("alloc Sstar_fw.fw_dccm err\n");
			goto err;
		}
		memcpy(Sstar_fw.fw_dccm,fw->fw_dccm,Sstar_fw.hdr.dccm_len);
	}
	return 0;
err:
	if(Sstar_fw.fw_iccm)
	{
		vfree(Sstar_fw.fw_iccm);
		Sstar_fw.fw_iccm = NULL;
	}

	if(Sstar_fw.fw_dccm)
	{
		vfree(Sstar_fw.fw_dccm);
		Sstar_fw.fw_dccm = NULL;
	}
#endif //#ifndef USB_BUS
	return -1;
}

#define FW_IS_READY	((Sstar_fw.fw_dccm != NULL) || (Sstar_fw.fw_iccm != NULL))
int Sstar_get_fw(struct firmware_sigmastar *fw)
{
	if(!FW_IS_READY)
	{
		return -1;
	}

	memcpy(&fw->hdr,&Sstar_fw.hdr,sizeof(struct firmware_headr));
	fw->fw_iccm = Sstar_fw.fw_iccm;
	fw->fw_dccm = Sstar_fw.fw_dccm;
	Sstar_printk_always("%s:get fw\n",__func__);
	return 0;
}


int Sstar_get_hw_type(u32 config_reg_val, int *major_revision)
{
#if 0
	int hw_type = -1;
	u32 config_value = config_reg_val;
	//u32 silicon_type = (config_reg_val >> 24) & 0x3;
	u32 silicon_vers = (config_reg_val >> 31) & 0x1;

	/* Check if we have CW1200 or STLC9000 */

	hw_type = HIF_1601_CHIP;
#endif
	return HIF_1601_CHIP;
}

static int Sstar_load_firmware_generic(struct Sstar_common *priv, u8 *data,u32 size,u32 addr)
{

	int ret=0;
	u32 put = 0;
	u8 *buf = NULL;


	buf = Sstar_kmalloc(DOWNLOAD_BLOCK_SIZE*2, GFP_KERNEL | GFP_DMA);
	if (!buf) {
		Sstar_dbg(SSTAR_APOLLO_DBG_ERROR,
			"%s: can't allocate bootloader buffer.\n", __func__);
		ret = -ENOMEM;
		goto error;
	}
	
#ifndef HW_DOWN_FW
	if(priv->sbus_ops->bootloader_debug_config)
		priv->sbus_ops->bootloader_debug_config(priv->sbus_priv,0);
#endif //#ifndef HW_DOWN_FW

	/*  downloading loop */
	Sstar_printk_init( "%s: addr %x: len %x\n",__func__,addr,size);
	for (put = 0; put < size ;put += DOWNLOAD_BLOCK_SIZE) {
		u32 tx_size;


		/* calculate the block size */
		tx_size  = min((size - put),(u32)DOWNLOAD_BLOCK_SIZE);

		memcpy(buf, &data[put], tx_size);

		/* send the block to sram */
		ret = Sstar_fw_write(priv,put+addr,buf, tx_size);
		if (ret < 0) {
			Sstar_dbg(SSTAR_APOLLO_DBG_ERROR,
				"%s: can't write block at line %d.\n",
				__func__, __LINE__);
			Sstar_printk_err("%s:put = 0x%x\n",__func__,put);
			goto error;
		}
	} /* End of bootloader download loop */

error:
	Sstar_kfree(buf);
	return ret;


}

char * Sstar_HwGetChipFw(struct Sstar_common *priv)
{
	u32 chipver = 0;
	char * strHwChipFw;

	if(fw)
	{
		Sstar_printk_always("%s, use module_param fw [%s]\n",__func__, fw );
	 	return fw;
	}

	Sstar_direct_read_reg_32(priv,0x0acc017c,&chipver);
	chipver&=0x3f;
	switch(chipver)
	{
		case 0x0:	
			strHwChipFw = ("ApolloC0.bin");		
			break;
		case 0x1:	
			strHwChipFw = ("ApolloC0_TC.bin");	
			break;
		case 0x3:	
			strHwChipFw = ("ApolloC1_TC.bin");	
			break;
		case 0xc:	
			strHwChipFw = ("ApolloD.bin");		
			break;
		case 0xd:	
			strHwChipFw = ("ApolloD_TC.bin");	
			break;
		case 0x10:	
			strHwChipFw = ("ApolloE.bin");		
			break;
		case 0x20:	
			strHwChipFw = ("AthenaA.bin");		
			break;
		case 0x14:	
			strHwChipFw = ("ApolloF.bin");		
			break;
		case 0x15:	
			strHwChipFw = ("ApolloF_TC.bin");	
			break;
		case 0x24:	
			strHwChipFw = ("AthenaB.bin");		
			break;
		case 0x25:	
			strHwChipFw = ("AthenaBX.bin");		
			break;
		case 0x18:	
			strHwChipFw = ("Apollo_FM.bin");		
			break;
		default:
			strHwChipFw = FIRMWARE_DEFAULT_PATH;		
		break;
	}

	Sstar_printk_always("%s, chipver=0x%x, use fw [%s]\n",__func__, chipver,strHwChipFw );

	return strHwChipFw;
}

//#define TEST_DCXO_CONFIG move to makefile
#ifndef CONFIG_USE_FW_H
#define USED_FW_FILE
#endif
#ifdef USED_FW_FILE
/*check if fw headr ok*/
static int Sstar_fw_checksum(struct firmware_headr * hdr)
{
	return 1;
}
#else
#ifdef USB_BUS
#include "firmware_usb.h"
#endif
#ifdef SDIO_BUS
#include "firmware_sdio.h"
#endif
#ifdef SPI_BUS
#include "firmware_spi.h"
#endif
#endif
#ifdef CONFIG_PM_SLEEP
#pragma message("CONFIG_PM_SLEEP")
int Sstar_cache_fw_before_suspend(struct device	 *pdev)
{
#if defined (USED_FW_FILE) && defined(SSTAR_USE_SAVED_FW)
	int ret = 0;
	const char *fw_path= fw;
	const struct firmware *firmware = NULL;
	struct firmware_sigmastar fw_sigmastar;

	memset(&fw_sigmastar,0,sizeof(struct firmware_sigmastar));
	if(fw_path == NULL){
		goto error2;
	}
	if(FW_IS_READY){
		Sstar_printk_err("Sstar_fw ready\n");
		goto error2;
	}
	
	ret = request_firmware(&firmware, fw_path, pdev);
	if(ret){
		Sstar_printk_err("%s:request_firmware err\n",__func__);
		goto error2;
	}
	if(*(int *)firmware->data == SIGMASTAR_WIFI_HDR_FLAG){
		memcpy(&fw_sigmastar.hdr,firmware->data,sizeof(struct firmware_headr));
		if(Sstar_fw_checksum(&fw_sigmastar.hdr)==0){
			ret = -1;
			 Sstar_dbg(SSTAR_APOLLO_DBG_ERROR,"%s: Sstar_fw_checksum fail 11\n", __func__);
			 goto error1;
		}
		fw_sigmastar.fw_iccm = (u8 *)firmware->data + sizeof(struct firmware_headr);
		fw_sigmastar.fw_dccm = fw_sigmastar.fw_iccm + fw_sigmastar.hdr.iccm_len;
		Sstar_dbg(SSTAR_APOLLO_DBG_ERROR,"%s: have header,lmac version(%d) iccm_len(%d) dccm_len(%d)\n", __func__,
			fw_sigmastar.hdr.version,fw_sigmastar.hdr.iccm_len,fw_sigmastar.hdr.dccm_len);
	}
	else {
		fw_sigmastar.hdr.version =  0x001;
		if(firmware->size > DOWNLOAD_ITCM_SIZE){
			fw_sigmastar.hdr.iccm_len =  DOWNLOAD_ITCM_SIZE;
			fw_sigmastar.hdr.dccm_len =  firmware->size - fw_sigmastar.hdr.iccm_len;
			if(fw_sigmastar.hdr.dccm_len > DOWNLOAD_DTCM_SIZE) {
				ret = -1;
			 	Sstar_dbg(SSTAR_APOLLO_DBG_ERROR,"%s: Sstar_fw_checksum fail 22\n", __func__);
			 	goto error1;
			}
			fw_sigmastar.fw_iccm = (u8 *)firmware->data;
			fw_sigmastar.fw_dccm = fw_sigmastar.fw_iccm+fw_sigmastar.hdr.iccm_len;
		}
		else {
			fw_sigmastar.hdr.iccm_len = firmware->size;
			fw_sigmastar.hdr.dccm_len = 0;
			fw_sigmastar.fw_iccm = (u8 *)firmware->data;
			
		}

	}
	Sstar_release_firmware();
	
	memcpy(&Sstar_fw.hdr,&fw_sigmastar.hdr,sizeof(struct firmware_headr));
	if(Sstar_fw.hdr.iccm_len)
	{
		Sstar_fw.fw_iccm = vmalloc(Sstar_fw.hdr.iccm_len);
		
		if(!Sstar_fw.fw_iccm)
		{
			Sstar_printk_err( "alloc Sstar_fw.fw_iccm err\n");
			goto error1;
		}
		memcpy(Sstar_fw.fw_iccm,fw_sigmastar.fw_iccm,Sstar_fw.hdr.iccm_len);
	}

	if(Sstar_fw.hdr.dccm_len)
	{
		Sstar_fw.fw_dccm= vmalloc(Sstar_fw.hdr.dccm_len);

		if(!Sstar_fw.fw_dccm)
		{
			Sstar_printk_err("alloc Sstar_fw.fw_dccm err\n");
			goto error1;
		}
		memcpy(Sstar_fw.fw_dccm,fw_sigmastar.fw_dccm,Sstar_fw.hdr.dccm_len);
	}
	Sstar_printk_always("%s:cached fw\n",__func__);
	release_firmware(firmware);
	return 0;
error1:
	
	Sstar_printk_err("%s:error1\n",__func__);
	release_firmware(firmware);
	if(Sstar_fw.fw_iccm)
	{
		vfree(Sstar_fw.fw_iccm);
		Sstar_fw.fw_iccm = NULL;
	}

	if(Sstar_fw.fw_dccm)
	{
		vfree(Sstar_fw.fw_dccm);
		Sstar_fw.fw_dccm = NULL;
	}
error2:
	Sstar_printk_err("%s:error2\n",__func__);
	return ret;
#else
	return 0;
#endif//
}
#endif
static int Sstar_start_load_firmware(struct Sstar_common *priv)
{

	int ret;
#ifdef USED_FW_FILE
	const char *fw_path= Sstar_HwGetChipFw(priv);
#endif//
	const struct firmware *firmware = NULL;
	struct firmware_sigmastar fw_sigmastar;
loadfw:
	//u32 testreg_uart;
#ifdef START_DCXO_CONFIG
	Sstar_ahb_write_32(priv,0x18e00014,0x200);
	Sstar_ahb_read_32(priv,0x18e00014,&val32_1);
	//Sstar_ahb_read_32(priv,0x16400000,&testreg_uart);
	Sstar_printk_always("0x18e000e4-->%08x %08x\n",val32_1);
#endif//TEST_DCXO_CONFIG
	if(!FW_IS_READY)
	{
#ifdef USED_FW_FILE
	    Sstar_dbg(SSTAR_APOLLO_DBG_MSG,"%s:FW FILE = %s\n",__func__,fw_path);
		ret = request_firmware(&firmware, fw_path, priv->pdev);
		if (ret) {
			Sstar_dbg(SSTAR_APOLLO_DBG_ERROR,
				"%s: can't load firmware file %s.\n",
				__func__, fw_path);
			goto error;
		}
		BUG_ON(!firmware->data);
		if(*(int *)firmware->data == SIGMASTAR_WIFI_HDR_FLAG){
			memcpy(&fw_sigmastar.hdr,firmware->data,sizeof(struct firmware_headr));
			if(Sstar_fw_checksum(&fw_sigmastar.hdr)==0){
				ret = -1;
				 Sstar_dbg(SSTAR_APOLLO_DBG_ERROR,"%s: Sstar_fw_checksum fail 11\n", __func__);
				 goto error;
			}
			fw_sigmastar.fw_iccm = (u8 *)firmware->data + sizeof(struct firmware_headr);
			fw_sigmastar.fw_dccm = fw_sigmastar.fw_iccm + fw_sigmastar.hdr.iccm_len;
			Sstar_dbg(SSTAR_APOLLO_DBG_ERROR,"%s: have header,lmac version(%d) iccm_len(%d) dccm_len(%d),fwsize(%zu),hdrsize(%zu)\n", __func__,
				fw_sigmastar.hdr.version,fw_sigmastar.hdr.iccm_len,fw_sigmastar.hdr.dccm_len,firmware->size,sizeof(struct firmware_headr));

			//frame_hexdump("fw_iccm ",fw_sigmastar.fw_iccm,64);
			//frame_hexdump("fw_dccm ",fw_sigmastar.fw_dccm,64);
		}
		else {
			fw_sigmastar.hdr.version =  0x001;
			if(firmware->size > DOWNLOAD_ITCM_SIZE){
				fw_sigmastar.hdr.iccm_len =  DOWNLOAD_ITCM_SIZE;
				fw_sigmastar.hdr.dccm_len =  firmware->size - fw_sigmastar.hdr.iccm_len;
				if(fw_sigmastar.hdr.dccm_len > DOWNLOAD_DTCM_SIZE) {
					ret = -1;
				 	Sstar_dbg(SSTAR_APOLLO_DBG_ERROR,"%s: Sstar_fw_checksum fail 22\n", __func__);
				 	goto error;
				}
				fw_sigmastar.fw_iccm = (u8 *)firmware->data;
				fw_sigmastar.fw_dccm = fw_sigmastar.fw_iccm+fw_sigmastar.hdr.iccm_len;
			}
			else {
				fw_sigmastar.hdr.iccm_len = firmware->size;
				fw_sigmastar.hdr.dccm_len = 0;
				fw_sigmastar.fw_iccm = (u8 *)firmware->data;
				
			}

		}
#else //USED_FW_FILE
		{
		Sstar_dbg(SSTAR_APOLLO_DBG_ERROR,"%s: used firmware.h=\n", __func__);
		fw_sigmastar.hdr.iccm_len = sizeof(fw_code);
		fw_sigmastar.hdr.dccm_len = sizeof(fw_data);
		
		fw_sigmastar.fw_iccm = &fw_code[0];
		fw_sigmastar.fw_dccm = &fw_data[0];
		}
#endif //USED_FW_FILE
		Sstar_set_firmare(&fw_sigmastar);
	}
	else
	{
		if((ret = Sstar_get_fw(&fw_sigmastar))<0)
		{
			goto error;
		}
	}
	Sstar_dbg(SSTAR_APOLLO_DBG_ERROR,"%s: START DOWNLOAD ICCM=========\n", __func__);

	ret = Sstar_load_firmware_generic(priv,fw_sigmastar.fw_iccm,fw_sigmastar.hdr.iccm_len,DOWNLOAD_ITCM_ADDR);
	if(ret<0)
		goto error;
	#ifdef USB_BUS
	fw_sigmastar.hdr.dccm_len = 0x8000;
	#else
	if(fw_sigmastar.hdr.dccm_len > 0x9000)
	fw_sigmastar.hdr.dccm_len = 0x9000;
	#endif
	Sstar_dbg(SSTAR_APOLLO_DBG_ERROR,"%s: START DOWNLOAD DCCM=========\n", __func__);
	ret = Sstar_load_firmware_generic(priv,fw_sigmastar.fw_dccm,fw_sigmastar.hdr.dccm_len,DOWNLOAD_DTCM_ADDR);
	if(ret<0)
		goto error;

	Sstar_dbg(SSTAR_APOLLO_DBG_MSG, "%s: FIRMWARE DOWNLOAD SUCCESS\n",__func__);

error:
	if (ret<0){
		if(Sstar_reset_lmc_cpu(priv) == 0)
			goto loadfw;
	}
	if (firmware)
		release_firmware(firmware);
	return ret;


}


int Sstar_load_firmware(struct Sstar_common *hw_priv)
{
	int ret;

	Sstar_printk_init("Sstar_before_load_firmware++\n");
	ret = Sstar_before_load_firmware(hw_priv);
	if(ret <0)
		goto out;
	Sstar_printk_init("Sstar_start_load_firmware++\n");
	ret = Sstar_start_load_firmware(hw_priv);
	if(ret <0)
		goto out;
	Sstar_printk_init("Sstar_after_load_firmware++\n");
	ret = Sstar_after_load_firmware(hw_priv);
	if(ret <0){
		goto out;
	}
	ret =0;
out:
	return ret;

}

