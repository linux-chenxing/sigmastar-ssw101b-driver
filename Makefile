#
#makefile for build atbm_wifi.ko
#
###############################################################################
#
# when release to customer, the CUSTOMER_SUPPORT_USED must set to y
#
###############################################################################
CUSTOMER_SUPPORT_USED=n
###############################################################################

#PLATFORM_XUNWEI					1
#PLATFORM_SUN6I						2
#PLATFORM_FRIENDLY					3
#PLATFORM_SUN6I_64					4
#PLATFORM_HI3798M					5
#PLATFORM_HI3518E					6
#PLATFORM_X86PC						7
#PLATFORM_AMLOGIC					8
#PLATFORM_AMLOGIC_905X				9
#PLATFORM_ROCKCHIP					10
#PLATFORM_MSTAR						11
#PLATFORM_CDLINUX					12
#PLATFORM_AMLOGIC_S805				13
#PLATFORM_HIS_LINUX_3_4				14
#PLATFORM_ROCKCHIP_3229				15
#PLATFORM_ROCKCHIP_3229_ANDROID8	16
#PLATFORM_HS_IPC					17
#PLATFORM_SIGMASTAR					18
#PLATFORM_HI3516EV200				19
#PLATFORM_XUNWEI_2G					20
#PLATFORM_NVT98517					21
#PLATFORM_INGENIC					22
#PLATFORM_SUN8I						23

export
platform ?= PLATFORM_SUN8I
sys ?= Linux
arch ?= arm
ATBM_WIFI__EXT_CCFLAGS = -DATBM_WIFI_PLATFORM=23

export
KERDIR := /home/cronyx/firmware/output/build/linux-4.9.118
CROSS_COMPILE := /home/cronyx/firmware/output/host/bin/arm-openipc-linux-musleabihf-

ifeq ($(CUSTOMER_SUPPORT_USED),y)
MAKEFILE_SUB ?= Makefile.build.customer
else
MAKEFILE_SUB ?= Makefile.build
endif

ifeq ($(KERNELRELEASE),)

all: install strip

install:
	@echo "make PLATFORM_CROSS=$(platform)"
	$(MAKE) all -f $(MAKEFILE_SUB) ARCH=$(arch) CROSS_COMPILE=$(CROSS_COMPILE) KDIR=$(KERDIR) SYS=$(sys) PLAT=$(platform)
clean:
	$(MAKE) -f $(MAKEFILE_SUB) KDIR=$(KERDIR) ARCH=$(arch) clean
strip:
	$(MAKE) -f $(MAKEFILE_SUB) KDIR=$(KERDIR) ARCH=$(arch) SYS=$(sys) PLAT=$(platform) strip
else
export
include $(src)/Makefile.build.kernel
endif
