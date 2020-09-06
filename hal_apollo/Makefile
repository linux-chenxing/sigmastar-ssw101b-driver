#CONFIG_TX_NO_CONFIRM
#CONFIG_USB_AGGR_URB_TX
#CONFIG_TX_NO_CONFIRM
#
# 可以用来选择不同是模式，CONFIG_USB_AGGR_URB_TX 定义上时一定需要定义CONFIG_USE_DMA_ADDR_BUFFER
# 1> CONFIG_USB_AGGR_URB_TX + CONFIG_TX_NO_CONFIRM，每个urb 发送多个txframe 并且不需要confirm，需要打开lmac的NO_NEED_CONFIRM 宏，并且ratecontrol在lmac 实现
# 2> CONFIG_USB_AGGR_URB_TX 每个urb 发送多个txframe 并且需要confirm，需要关闭lmac的NO_NEED_CONFIRM 宏
# 3> 所有宏都不定义，每个urb 发送一个txframe 并且需要confirm，并且urb直接使用skbbuffer，需要关闭lmac的NO_NEED_CONFIRM 宏
# 4> 定义CONFIG_USE_DMA_ADDR_BUFFER，每个urb 发送一个txframe 并且需要confirm，并且urb直接DMAbuffer，需要把数据从skb copy到DMAbuffer，需要关闭lmac的NO_NEED_CONFIRM 宏，
# 5> 定义CONFIG_USE_DMA_ADDR_BUFFER+ CONFIG_TX_NO_CONFIRM，每个urb 发送一个txframe 并且不需要confirm，并且urb直接DMAbuffer，需要把数据从skb copy到DMAbuffer，需要开lmac的NO_NEED_CONFIRM 宏，并且ratecontrol在lmac 实现
# 6> 定义CONFIG_TX_NO_CONFIRM，每个urb 发送一个txframe 并且不需要confirm，并且urb直接使用skbbuffer，需要打开lmac的NO_NEED_CONFIRM 宏，并且ratecontrol在lmac 实现
#
##对于调度慢+cpu强的板子使用 模式1 或者 模式2
##对于调度快+cpu慢的板子使用 模式3 或者 模式6
## 默认使用模式 3
#####################################################################################################################################################################################################################
#ccflags-y += -DCONFIG_SSTAR_APOLLO_5GHZ_SUPPORT
#ccflags-y += -DCONFIG_SSTAR_APOLLO_WAPI_SUPPORT
#ccflags-y  += -I$(shell pwd)/../include/ \
        -include $(shell pwd)/../include/linux/compat-2.6.h \
        -DCOMPAT_STATIC
##指定配置文件路径，当配置文件存在时使用配置文件中的dcxo值不使用efuse中的dcxo，配置文件中的delta_gain为修正值，如果文件不存在则直接使用efuse中的数据
SSTAR_CONFIG_FILE="/Sstar_txpwer_dcxo_cfg.txt"
ifneq ($(IF1NAME),)
ccflags-y += -DCONFIG_IF1NAME=\"$(IF1NAME)\"
endif
ifneq ($(IF2NAME),)
ccflags-y += -DCONFIG_IF2NAME=\"$(IF2NAME)\"
endif		
SSTAR_MODULES_NAME ?= $(MODULES_NAME)
ifneq ($(FW),)
ccflags-y += -DCONFIG_FW_NAME=\"$(FW)\"
endif
ifneq ($(SDIO_HOST),)
ccflags-y += -DCONFIG_SSTAR_SDIO_MMC_ID=\"$(SDIO_HOST)\"
endif
ccflags-y += $(SSTAR_WIFI__EXT_CCFLAGS)
ccflags-y += -DCONFIG_SSTAR_APOLLO_DISABLE_BEACON_HINTS
ccflags-y += -DOLD_RATE_POLICY=1

ccflags-y += -DMINSTREL_RSSI_USED
ccflags-y += -DSSTAR_P2P_ADDR_USE_LOCAL_BIT
ccflags-y += -DSSTAR_WIFI_QUEUE_LOCK_BUG
ccflags-y += -DSSTAR_CHANGE_AP_TIMESTAMP_TO_BOOTTIME
ccflags-y += -DSSTAR_USE_SAVED_FW
#ccflags-y += -DSSTAR_AP_SME
#ccflags-y += -DP2P_MULTIVIF
ccflags-y += -DSSTAR_P2P_CHANGE
ccflags-y += -DSSTAR_WIFI_NO_P2P_PS
ccflags-y += -DSSTAR_SPECIAL_PKG_DOWN_RATE
#ccflags-y += -DMCAST_FWDING
ccflags-y += -DSSTAR_APOLLO_SUSPEND_RESUME_FILTER_ENABLE
ccflags-y += -DCONFIG_SSTAR_IOCTRL
ccflags-y += -DOPER_CLOCK_USE_SEM
ccflags-y += -DEXIT_MODULE_RESET_USB=0
ccflags-y += -DSSTAR_VIF_LIST_USE_RCU_LOCK
#ccflags-y += -DSSTAR_SUPPORT_SMARTCONFIG 
ccflags-y += -DCONFIG_SSTAR_MONITOR_HDR_PRISM
#ccflags-y += -DIEEE80211_TASKLET
ccflags-y += -DCONFIG_RATE_HW_CONTROL
ccflags-y += -DCONFIG_SSTAR_SUPPORT_SG
#add AP+STA mode , sta channel change send event to hostapd
ccflags-y += -DCONFIG_SSTAR_AP_CHANNEL_CHANGE_EVENT

ifeq ($(platform),PLATFORM_AMLOGIC_S805)
ccflags-y += -DCONFIG_SSTAR_APOLLO_USE_GPIO_IRQ
endif
ifeq ($(platform),PLATFORM_AMLOGIC)
#ccflags-y += -DCONFIG_SSTAR_APOLLO_USE_GPIO_IRQ
endif
#ccflags-y += -DCONFIG_SSTAR_APOLLO_USE_GPIO_IRQ
#ccflags-y += -DSSTAR_PKG_REORDER
ccflags-y += -DSSTAR_SUPPORT_WIDTH_40M
#ccflags-y += -DIPC_AP_USED_11G_NO_RTS
#p2p mode always in client mode, used in 4.4.2 android ,because monitor interface is not open
#(ANDRIOD_P2P in wpa_supplicant is open will cause this bug )
#ccflags-y += -DP2P_ALWAYS_CLIENT
#ccflags-y += -DSSTAR_11W_TEST

#wifi alliance test ,
#some function will cause pass wifi alliance certificate,but not good for some AP
#ccflags-y += -DWIFI_ALLIANCE_CERTIF
#ccflags-y += -DSSTAR_11W_TEST
#if used sdio download firmware
ccflags-y += -DWIFI_FW_DOWNLOAD
# Extra IE for probe response from upper layer is needed in P2P GO
# For offloading probe response to FW, the extra IE must be included
# in the probe response template
ccflags-y += -DSSTAR_PROBE_RESP_EXTRA_IE
ccflags-y += -DCONFIG_SSTAR_APOLLO_DEBUG
#ccflags-y += -DCONFIG_SSTAR_STA_LISTEN
#loader firmwre fast
ccflags-$(DRVLOADERFAST) += -DCONDIF_SSTAR_CTRL_REQUEST_ASYNC
###################################################################
#SSTAR_SUSPEND_REMOVE_INTERFACE
#before enter suspend state ,remove all interface
###################################################################
ccflags-$(PMRELODDFW) += -DSSTAR_SUSPEND_REMOVE_INTERFACE
###################################################################
#SSTAR_PM_USE_EARLYSUSPEND
#early suspend enbale
###################################################################
ccflags-$(EARLYSUSPEND) += -DSSTAR_PM_USE_EARLYSUSPEND
###################################################################
#SSTAR_ALLOC_MEM_DEBUG
#debug mem leakage
###################################################################
ccflags-$(MEM_DEBUG) += -DSSTAR_ALLOC_MEM_DEBUG
###################################################################
#SSTAR_ALLOC_SKB_DEBUG
#debug skb leakage
###################################################################
ccflags-$(SKB_DEBUG) += -DSSTAR_ALLOC_SKB_DEBUG
####################################################################
#CONFIG_USE_FW_H
#use firmware.h as firmware
####################################################################
ccflags-$(LOAD_FW_H) += -DCONFIG_USE_FW_H
####################################################################
#SSTAR_NOT_SUPPORT_40M_CHW
#only support 20m channel
####################################################################
ccflags-$(CONFIG_NOT_SUPPORT_40M_CHW) += -DSSTAR_NOT_SUPPORT_40M_CHW
####################################################################
#SSTAR_SUPPORT_PKG_MONITOR
#monitor funciton
####################################################################
ccflags-$(MONITOR) += -DSSTAR_SUPPORT_PKG_MONITOR
####################################################################
#CONFIG_MAC80211_BRIDGE
#enable bridge
####################################################################
ccflags-$(BRIDGE) += -DCONFIG_MAC80211_BRIDGE
####################################################################
#CONFIG_TX_NO_CONFIRM      
#enable tx noconfirm
####################################################################
ccflags-$(NOTXCONFIRM) += -DCONFIG_TX_NO_CONFIRM
####################################################################
#CONFIG_USB_AGGR_URB_TX
#enable usb aggr tx 
####################################################################
ccflags-$(USBAGGTX) += -DCONFIG_USB_AGGR_URB_TX
####################################################################
#CONFIG_USE_DMA_ADDR_BUFFER
#urb use dma buff
####################################################################
ccflags-$(USBDMABUFF) += -DCONFIG_USE_DMA_ADDR_BUFFER
####################################################################
#CONFIG_WSM_CMD_XMIT_DIRECTLY
#cmd transmit directly
####################################################################
ccflags-$(USBCMDENHANCE) += -DCONFIG_WSM_CMD_XMIT_DIRECTLY
####################################################################
#CONFIG_USB_DATA_XMIT_DIRECTLY
#usb data xmit directly
####################################################################
ccflags-$(USBDATAENHANCE) += -DCONFIG_USB_DATA_XMIT_DIRECTLY
####################################################################
#CONFIG_SSTAR_5G_PRETEND_2G
#wifi use 5G channel to pretend 2g special channel
####################################################################
ccflags-$(CH5G) += -DCONFIG_SSTAR_5G_PRETEND_2G

####################################################################
#CONFIG_SSTAR_PRODUCT_TEST_USE_GOLDEN_LED
#wifi product  test use Golden LED
####################################################################
ccflags-$(PRI_IE) += -DSSTAR_PRIVATE_IE
ifneq ($(PRI_IE),y)
ccflags-y += -DCONFIG_SSTAR_STA_LISTEN
endif
#ccflags-y += -DCONFIG_SSTAR_PRODUCT_TEST_USE_GOLDEN_LED
####################################################################
#SSTAR_PRODUCT_TEST_USE_FEATURE_ID
#wifi product test use feature ID
####################################################################
ccflags-y += -DSSTAR_PRODUCT_TEST_USE_FEATURE_ID

#ccflags-y += -DIPV6_FILTERING
#ccflags-y += -DCONFIG_SSTAR_APOLLO_BH_DEBUG
#ccflags-y += -DCONFIG_SSTAR_APOLLO_WSM_DEBUG
#ccflags-y += -DCONFIG_SSTAR_APOLLO_WSM_DUMPS
#ccflags-y += -DCONFIG_SSTAR_APOLLO_WSM_DUMPS_SHORT
#ccflags-y += -DCONFIG_SSTAR_APOLLO_TXRX_DEBUG
#ccflags-y += -DCONFIG_SSTAR_APOLLO_TX_POLICY_DEBUG
#ccflags-y += -DCONFIG_SSTAR_APOLLO_STA_DEBUG
#ccflags-y += -DCONFIG_SSTAR_APOLLO_DUMP_ON_ERROR
#ccflags-y += -DCONFIG_SSTAR_APOLLO_ITP
ccflags-y += -DCONFIG_SSTAR_APOLLO_TESTMODE
# use the mac addr in file :"/data/.mac.info"
ccflags-y += -DCUSTOM_FEATURE_MAC
#ccflags-y += -DTEST_RF_POWER
#ccflags-y += -DUSB_CMD_UES_EP0
#ccflags-y += -DRESET_CHANGE
ccflags-y += -DCONFIG_SSTARWIFI_WDT
# use the txpower and dcxo value in file :"$(SSTAR_CONFIG_FILE)",when file exist
ccflags-y += -DCONFIG_TXPOWER_DCXO_VALUE=\"$(SSTAR_CONFIG_FILE)\"
#PLATFORM_XUNWEI
#PLATFORM_SUN6I
#PLATFORM_FRIENDLY
#PLATFORM_SUN6I_64
#ccflags-y += -DSSTAR_WIFI_PLATFORM=PLATFORM_HIS
ifeq ($(CONFIG_ARES),y)
ccflags-y += -DPROJ_TYPE=ARES_A
endif
ifeq ($(CONFIG_ARESB),y)
ccflags-y += -DPROJ_TYPE=ARES_B
ccflags-$(CHECKSUM) += -DCHKSUM_HW_SUPPORT
ccflags-y += -DDPLL_CLOCK=DPLL_CLOCK_24M
endif

ifeq ($(CONFIG_HERA),y)
ccflags-y += -DPROJ_TYPE=HERA
ccflags-$(CHECKSUM) += -DCHKSUM_HW_SUPPORT
ccflags-y += -DDPLL_CLOCK=DPLL_CLOCK_24M
endif

ifeq ($(CONFIG_ATHENAB),y)
ccflags-y += -DPROJ_TYPE=ATHENA_B
ifeq ($(SDIO_BUS),y)
ccflags-y += -DSSTAR_SDIO_PATCH
ccflags-y += -DCONFIG_TX_NO_CONFIRM
endif
endif
ifeq ($(CONFIG_FPGA),y)
#ccflags-y += -DDCXO_USE_SMU_REG
ccflags-y += -DSDIO_CHANGE=1
#ccflags-y += -DRESET_CHANGE
#ccflags-y += -DTEST_DCXO_CONFIG=1
endif
#if TEST_DCXO_CONFIG is not config,DCXO_TYPE is not used
#if TEST_DPLL_CONFIG is not config,DPLL_CLOCK is not used
ifeq ($(CONFIG_1601),y)
ccflags-y += -DDCXO_USE_SMU_REG
ccflags-y += -DSDIO_CHANGE=0
ccflags-y += -DTEST_DCXO_CONFIG
ccflags-y += -DTEST_DPLL_CONFIG
ccflags-y += -DCHIP_V1601=1
ccflags-y += -DPROJ_TYPE=APOLLO_1601
ccflags-y += -DDCXO_TYPE=CRYSTAL_MODE
ccflags-y += -DDPLL_CLOCK=DPLL_CLOCK_40M
endif
ifeq ($(CONFIG_1606),y)
ccflags-y += -DDCXO_USE_SMU_REG
ccflags-y += -DSDIO_CHANGE=0
ccflags-y += -DTEST_DCXO_CONFIG
ccflags-y += -DTEST_DPLL_CONFIG
ccflags-y += -DCHIP_V1601=0
ccflags-y += -DPROJ_TYPE=APOLLO_1606
ccflags-y += -DDCXO_TYPE=CRYSTAL_MODE
ccflags-y += -DDPLL_CLOCK=DPLL_CLOCK_40M
endif
ifeq ($(CONFIG_APOLLOC),y)
#ccflags-y += -DDCXO_USE_SMU_REG
#ccflags-y += -DTEST_DCXO_CONFIG
#ccflags-y += -DTEST_DPLL_CONFIG
ccflags-y += -DCHIP_V1601=0
ccflags-y += -DSDIO_CHANGE=1
ccflags-y += -DPROJ_TYPE=APOLLO_C
ccflags-y += -DDCXO_TYPE=CRYSTAL_MODE
ccflags-y += -DDPLL_CLOCK=DPLL_CLOCK_40M
#ccflags-y += -DCONFIG_SSTAR_APOLLO_SUPPORT_SGI
endif

ifeq ($(CONFIG_APOLLOD),y)
ccflags-y += -DDCXO_USE_SMU_REG
ccflags-y += -DTEST_DCXO_CONFIG
ccflags-y += -DTEST_DPLL_CONFIG
ccflags-y += -DCHIP_V1601=0
ccflags-y += -DSDIO_CHANGE=1
ccflags-y += -DPROJ_TYPE=APOLLO_C
ccflags-y += -DDCXO_TYPE=CRYSTAL_MODE
ccflags-y += -DDPLL_CLOCK=DPLL_CLOCK_40M
#ccflags-y += -DCONFIG_SSTAR_APOLLO_SUPPORT_SGI
endif
ifeq ($(CONFIG_APOLLOE),y)
ccflags-y += -DCHIP_V1601=0
ccflags-y += -DSDIO_CHANGE=1
ccflags-y += -DPROJ_TYPE=APOLLO_E
ccflags-y += -DCONFIG_SSTAR_APOLLO_SUPPORT_SGI
endif
ifeq ($(CONFIG_ATHENAB_24M),y)
ccflags-y += -DDCXO_USE_SMU_REG
ccflags-y += -DTEST_DCXO_CONFIG
ccflags-y += -DTEST_DPLL_CONFIG
ccflags-y += -DPROJ_TYPE=ATHENA_B
ccflags-y += -DDCXO_TYPE=CRYSTAL_MODE
ccflags-y += -DDPLL_CLOCK=DPLL_CLOCK_24M
ccflags-y += -DSSTAR_SDIO_PATCH
ccflags-y += -DCONFIG_TX_NO_CONFIRM

endif
ifeq ($(CONFIG_ATHENALITE),y)
ccflags-y += -DCHKSUM_HW_SUPPORT
ccflags-y += -DCONFIG_USB_AGGR_URB_TX
ccflags-y += -DCONFIG_USE_DMA_ADDR_BUFFER
ccflags-y += -DPROJ_TYPE=ATHENA_LITE
endif
ifeq ($(CONFIG_ATHENALITE_ECO),y)
ccflags-y += -DCHKSUM_HW_SUPPORT
ccflags-y += -DCONFIG_USB_AGGR_URB_TX
ccflags-y += -DCONFIG_USE_DMA_ADDR_BUFFER
ccflags-y += -DPROJ_TYPE=ATHENA_LITE_ECO
endif
ifeq ($(CONFIG_ARES_24M),y)
#ccflags-y += -DDCXO_USE_SMU_REG
ccflags-y += -DTEST_DCXO_CONFIG
ccflags-y += -DTEST_DPLL_CONFIG
ccflags-y += -DPROJ_TYPE=ARES_A
ccflags-y += -DDCXO_TYPE=CRYSTAL_MODE
ccflags-y += -DDPLL_CLOCK=DPLL_CLOCK_24M
endif

ccflags-y += -DCONFIG_SSTAR_APOLLO_SUPPORT_SGI
ifeq ($(USB_BUS),y)
ccflags-y += -DUSB_BUS
ccflags-y += -DUSB_USE_TASTLET_TXRX
ccflags-y += -DUSB_BUS_BUG
ccflags-y += -DCONFIG_USB_URB_RX_SUBMIT_DIRECTLY
#ccflags-y += -DCONFIG_WSM_CMD_XMIT_DIRECTLY
#ccflags-y += -DCONFIG_USB_DATA_XMIT_DIRECTLY
#ccflags-y += -DCONFIG_USE_DMA_ADDR_BUFFER
#ccflags-y += -DCONFIG_USB_AGGR_URB_TX
#ccflags-y += -DCONFIG_TX_NO_CONFIRM
#ccflags-y += -DSSTAR_IPC_ASSOC
#ccflags-y += -DSSTAR_PRIVATE_IE
#ccflags-y += -DSIGMASTAR_FILTER_MACADDR_ONLY
#ccflags-y += -DCONFIG_SSTAR_5G_PRETEND_2G
endif
ifeq ($(TX_NO_CONFIRM),y)
#ccflags-y += -DCONFIG_USE_DMA_ADDR_BUFFER
ccflags-y += -DCONFIG_USB_AGGR_URB_TX
ccflags-y += -DCONFIG_TX_NO_CONFIRM
endif
ifeq ($(SDIO_BUS),y)
ccflags-y += -DMCAST_FWDING
ccflags-y += -DRESET_CHANGE
ccflags-y += -DSDIO_BUS
#ccflags-y += -DSSTAR_SDIO_TXRX_ENHANCE
#ccflags-y += -DSSTAR_WSM_SDIO_TX_MULT
#ccflags-y += -DRESET_CHIP
endif
ifeq ($(SPI_BUS),y)
ccflags-y += -DSPI_BUS
endif
#ccflags-y += -DPS_SETUP
MODFLAGS =-DMODULE -fno-pic 
CFLAGS_MODULE =$(MODFLAGS)

ifeq ($(CONFIG_SSTAR_APOLLO_DEBUGFS),y)
ccflags-y  += -DCONFIG_SSTAR_APOLLO_DEBUGFS
endif
#ccflags-y  += -DCONFIG_SSTAR_APOLLO_DEBUG
ccflags-y += -DCONFIG_MAC80211_SSTAR_RC_DEFAULT_MINSTREL
#ccflags-y += -DMINSTREL_SAMPLE_SKIP_LOW_RATE_HT
#ccflags-y += -DMINSTREL_SAMPLE_SKIP_LOW_RATE

#ccflags-y+= -DCONFIG_MAC80211_LEDS
# ccflags-y += -DCONFIG_MAC80211_NOINLINE
# ccflags-y += -DCONFIG_MAC80211_VERBOSE_DEBUG
# ccflags-y += -DCONFIG_MAC80211_HT_DEBUG
# ccflags-y += -DCONFIG_MAC80211_TKIP_DEBUG
# ccflags-y += -DCONFIG_MAC80211_IBSS_DEBUG
# ccflags-y += -DCONFIG_MAC80211_VERBOSE_PS_DEBUG
# ccflags-y += -DCONFIG_MAC80211_VERBOSE_MPL_DEBUG
# ccflags-y += -DCONFIG_MAC80211_VERBOSE_MHWMP_DEBUG
# ccflags-y += -DCONFIG_MAC80211_DEBUG_COUNTERS
ccflags-y  += -DPROBE_RESP_EXTRA_IE
#ccflags-y  += -DIPV6_FILTERING
#ccflags-y += -DCONFIG_SSTAR_APOLLO_TESTMODE
#ccflags-y += -DCONFIG_NL80211_TESTMODE
ccflags-y += -DCONFIG_MAC80211_SSTAR_ROAMING_CHANGES
ifeq ($(CONFIG_MAC80211_SSTAR_DEBUGFS),y)
ccflags-y  += -DCONFIG_MAC80211_SSTAR_DEBUGFS
endif
ccflags-$(CONFIG_MAC80211_SSTAR_RC_MINSTREL) += -DCONFIG_MAC80211_SSTAR_RC_MINSTREL
ccflags-$(CONFIG_MAC80211_SSTAR_RC_MINSTREL_HT) += -DCONFIG_MAC80211_SSTAR_RC_MINSTREL_HT
$(SSTAR_MODULES_NAME)-y := \
	mac80211/bridge.o \
        mac80211/main.o \
        mac80211/status.o \
        mac80211/sta_info.o \
        mac80211/wep.o \
        mac80211/wpa.o \
        mac80211/wapi.o \
        mac80211/scan.o mac80211/offchannel.o \
        mac80211/ht.o mac80211/agg-tx.o mac80211/agg-rx.o \
        mac80211/ibss.o \
        mac80211/mlme.o mac80211/work.o \
        mac80211/iface.o \
        mac80211/rate.o \
        mac80211/michael.o \
        mac80211/tkip.o \
        mac80211/aes_ccm.o \
        mac80211/aes_cmac.o \
        mac80211/cfg.o \
        mac80211/rx.o \
        mac80211/spectmgmt.o \
        mac80211/tx.o \
        mac80211/key.o \
        mac80211/util.o \
        mac80211/wme.o \
        mac80211/event.o \
        mac80211/chan.o \
	mac80211/wext_cfg.o \
	mac80211/ap_sme.o \
	mac80211/ieee80211_Sstar_mem.o \
	mac80211/ieee80211_Sstar_skb.o

$(SSTAR_MODULES_NAME)-$(CONFIG_MAC80211_SSTAR_LEDS) += mac80211/led.o
$(SSTAR_MODULES_NAME)-$(CONFIG_MAC80211_SSTAR_DEBUGFS) += \
        mac80211/debugfs.o \
        mac80211/debugfs_sta.o \
        mac80211/debugfs_netdev.o \
        mac80211/debugfs_key.o

$(SSTAR_MODULES_NAME)-$(CONFIG_MAC80211_SSTAR_MESH) += \
        mac80211/mesh.o \
        mac80211/mesh_pathtbl.o \
        mac80211/mesh_plink.o \
        mac80211/mesh_hwmp.o

$(SSTAR_MODULES_NAME)-$(CONFIG_PM) += mac80211/pm.o

$(SSTAR_MODULES_NAME)-$(CONFIG_MAC80211_SSTAR_DRIVER_API_TRACER) += mac80211/driver-trace.o
CFLAGS_driver-trace.o := -I$(src)

# objects for PID algorithm
rc80211_pid-$(CONFIG_MAC80211_SSTAR_DEBUGFS) := mac80211/rc80211_pid_algo.o
rc80211_pid-$(CONFIG_MAC80211_SSTAR_DEBUGFS) += mac80211/rc80211_pid_debugfs.o

rc80211_minstrel-y := mac80211/rc80211_minstrel.o
rc80211_minstrel-$(CONFIG_MAC80211_SSTAR_DEBUGFS) += mac80211/rc80211_minstrel_debugfs.o

rc80211_minstrel_ht-y := mac80211/rc80211_minstrel_ht.o
rc80211_minstrel_ht-$(CONFIG_MAC80211_SSTAR_DEBUGFS) += mac80211/rc80211_minstrel_ht_debugfs.o

$(SSTAR_MODULES_NAME)-$(CONFIG_MAC80211_SSTAR_RC_PID) += $(rc80211_pid-y)
$(SSTAR_MODULES_NAME)-$(CONFIG_MAC80211_SSTAR_RC_MINSTREL) += $(rc80211_minstrel-y)
$(SSTAR_MODULES_NAME)-$(CONFIG_MAC80211_SSTAR_RC_MINSTREL_HT) += $(rc80211_minstrel_ht-y)

$(SSTAR_MODULES_NAME)-y += \
		fwio.o \
		txrx.o \
		queue.o \
		wsm.o \
		sta.o \
		ap.o \
		scan.o \
		main.o \
		util.o \
		dbg_event.o \
		Sstar_smartconfig.o \
		smartconfig.o  \
		dev_ioctl.o  \
		Sstar_p2p.o   \
		module_fs.o  \
		internal_cmd.o
ifeq ($(USB_BUS),y)
$(SSTAR_MODULES_NAME)-y += apollo_usb.o Sstar_platform.o hwio_usb.o bh_usb.o
else ifeq ($(SDIO_BUS),y)
$(SSTAR_MODULES_NAME)-y += apollo_sdio.o Sstar_platform.o hwio_sdio.o bh_sdio.o dcxo_dpll.o
else ifeq ($(SPI_BUS),y)
$(SSTAR_MODULES_NAME)-y += apollo_spi.o Sstar_platform.o hwio_spi.o bh_spi.o
endif
$(SSTAR_MODULES_NAME)-y	+= debug.o
$(SSTAR_MODULES_NAME)-$(CONFIG_PM)		+= pm.o


obj-$(CONFIG_SSTAR_APOLLO) += $(SSTAR_MODULES_NAME).o

ccflags-y += -D__CHECK_ENDIAN__
MODFLAGS =-DMODULE -fno-pic
CFLAGS_MODULE =$(MODFLAGS)

