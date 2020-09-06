#define USB_AUTO_WAKEUP 0
#define HW_DOWN_FW
#define DOWNLOAD_BLOCK_SIZE		(64)


#ifndef SSTAR_APOLLO_HWIO_USB_H_INCLUDED
#define SSTAR_APOLLO_HWIO_USB_H_INCLUDED
#include "apollo_plat.h"

#define SIGMASTAR_WIFI_HDR_FLAG  (0x34353677)
#if ((PROJ_TYPE>=ARES_A) || (PROJ_TYPE==ATHENA_LITE_ECO))
#define DOWNLOAD_ITCM_ADDR		(0x00010000)
#else
#define DOWNLOAD_ITCM_ADDR		(0x00000000)
#endif 
#define DOWNLOAD_ITCM_SIZE		(128*1024)
#define DOWNLOAD_DTCM_ADDR		(0x00800000)
#define DOWNLOAD_DTCM_SIZE		(48*1024)



#define SYS_BASE_ADDR_SILICON		(0)
#define PAC_BASE_ADDRESS_SILICON	(SYS_BASE_ADDR_SILICON + 0x09000000)
#define PAC_SHARED_MEMORY_SILICON	(PAC_BASE_ADDRESS_SILICON)


int Sstar_direct_write_reg_32(struct Sstar_common *hw_priv, u32 addr, u32 val);
int Sstar_direct_read_reg_32(struct Sstar_common *hw_priv, u32 addr, u32 *val);
int Sstar_fw_write(struct Sstar_common *priv, u32 addr, const void *buf,
                        u32 buf_len);
int Sstar_ep0_read(struct Sstar_common *hw_priv, u32 addr,
				void *buf, u32 buf_len);
int Sstar_ep0_write(struct Sstar_common *hw_priv, u32 addr,
				const void *buf, u32 buf_len);
#ifdef USB_CMD_UES_EP0
int Sstar_ep0_write_cmd(struct Sstar_common *hw_priv, struct wsm_hdr_tx * wsm_h);
#endif
/*
*lock for probe dan disconnect
*/
extern void Sstar_usb_module_muxlock(void);
extern void Sstar_usb_module_muxunlock(void);

#define Sstar_module_muxlock()			Sstar_usb_module_muxlock()
#define Sstar_module_muxunlock()			Sstar_usb_module_muxunlock()		
	
#endif //SSTAR_APOLLO_HWIO_USB_H_INCLUDED
