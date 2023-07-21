# SPDX-License-Identifier: GPL-2.0

#
# Makefile for misc devices that really don't fit anywhere else.
#
ccflags-y += -I$(MTK_CUSTOM_PATH)/imgsensor/inc \

obj-y		+= regulator/
obj-y		+= gpio/
#obj-y		+= mt6306/
obj-y		+= mclk/
obj-y		+= imgsensor_cfg_table.o

ifeq ($(CONFIG_ARCH_MTK_PROJECT), "k69v1_64_g85_basic_ref")
ccflags-y += -DG88_PROJECT
endif
