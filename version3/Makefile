##########################################
# Makefile for RTIMD-I2C Device Drivers.
##########################################
ccflags-y += -Idrivers/misc/rtimd-i2c
#ccflags-y += -v

obj-$(CONFIG_RTIMD_I2C) := rtimd-i2c.o
obj-$(CONFIG_RTIMD_I2C) += rtimd_dev_config.o

ccflags-$(CONFIG_RTIMD_I2C_DEBUG) += -DDEBUG

ifeq ($(CONFIG_MDC_RDC200A), y)
  ccflags-y += -DCFG_MDC_RDC200A
endif

ifeq ($(CONFIG_MDC_RDC200), y)
  ccflags-y += -DCFG_MDC_RDC200
endif

ifeq ($(CONFIG_PANEL_RDP370F), y)
  ccflags-y += -DCFG_PANEL_RDP370F
endif

ifeq ($(CONFIG_PANEL_RDP502H), y)
  ccflags-y += -DCFG_PANEL_RDP502H
endif

ifeq ($(CONFIG_PANEL_RDP551F), y)
  ccflags-y += -DCFG_PANEL_RDP551F
endif


