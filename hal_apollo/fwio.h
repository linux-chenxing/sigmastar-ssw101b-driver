/*
 * Firmware API for mac80211 sigmastar APOLLO drivers
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

#ifndef FWIO_H_INCLUDED
#define FWIO_H_INCLUDED

#ifndef CONFIG_FW_NAME
#if ((SSTAR_WIFI_PLATFORM == 13/*PLATFORM_AMLOGIC_S805*/) || (SSTAR_WIFI_PLATFORM == 8))
#define FIRMWARE_DEFAULT_PATH	"../wifi/Sstar/fw.bin"
#else // PLATFORM_AMLOGIC_S805
#define FIRMWARE_DEFAULT_PATH	"fw.bin"
#endif //PLATFORM_AMLOGIC_S805
#else 
#define FIRMWARE_DEFAULT_PATH CONFIG_FW_NAME 
#endif
#define SDD_FILE_DEFAULT_PATH	("sdd.bin")

#define SSTAR_APOLLO_REV_1601	(1601)

struct Sstar_common;


int Sstar_get_hw_type(u32 config_reg_val, int *major_revision);

int Sstar_load_firmware(struct Sstar_common *hw_priv);
void Sstar_release_firmware(void);
int Sstar_init_firmware(void);
#ifdef CONFIG_PM_SLEEP
int Sstar_cache_fw_before_suspend(struct device	 *pdev);
#endif
#endif
