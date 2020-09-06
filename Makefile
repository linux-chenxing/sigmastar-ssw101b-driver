#
#makefile for build Sstar_wifi.ko
#
###############################################################################
#
# when release to customer ,the CUSTOMER_SUPPORT_USED must set to y!!!!!!!!!!!!!
#
###############################################################################
CUSTOMER_SUPPORT_USED=y
###############################################################################
#PLATFORM_SIGMASTAR                           18
export
platform ?= PLATFORM_SIGMASTAR
#Android
#Linux
sys ?=Linux
#arch:arm or arm64
arch ?= arm
#export 
#SSTAR_WIFI__EXT_CCFLAGS = -DSSTAR_WIFI_PLATFORM=$(platform)

MDIR:=
export MDIR

ifeq ($(CUSTOMER_SUPPORT_USED),y)
MAKEFILE_SUB ?= Makefile.build.customer
else
MAKEFILE_SUB ?= Makefile.build
endif

ifeq ($(KERNELRELEASE),)
ifeq ($(platform),PLATFORM_SIGMASTAR)
ifeq ($(sys),Linux)

CROSS_COMPILE = arm-buildroot-linux-uclibcgnueabihf-
LINUX_SRC = /home/ljk/mijia/mijia_camera_common/buildroot-2017.08/output/build/linux-custom
MDIR =
else

endif
export
arch = arm
SSTAR_WIFI__EXT_CCFLAGS = -DSSTAR_WIFI_PLATFORM=18
endif

ifeq ($(platform),PLATFORM_PCX86)
all:install

install:
	@echo "make PLATFORM_PCX86"
	$(MAKE) all -f Makefile.build.local KDIR=$(LINUX_SRC)
clean:
	$(MAKE) -f Makefile.build.local KDIR=$(LINUX_SRC) clean
else
all:install

install:

	$(warning, "install operation")
	#@echo "make PLATFORM_CROSS=$(platform)"
	$(MAKE) all -f $(MAKEFILE_SUB) ARCH=$(arch)  CROSS_COMPILE=$(CROSS_COMPILE) KDIR=$(LINUX_SRC) SYS=$(sys) PLAT=$(platform) -j8
clean:
	$(MAKE) -f $(MAKEFILE_SUB) KDIR=$(LINUX_SRC) ARCH=$(arch) clean
strip:	
	$(MAKE) -f $(MAKEFILE_SUB) KDIR=$(LINUX_SRC) ARCH=$(arch) SYS=$(sys) PLAT=$(platform) strip
endif
else
export 
include $(src)/Makefile.build.kernel
endif

