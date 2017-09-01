#
# Makefile for lge touch core driver
#

# Each configuration option enables a list of files.

ifdef CONFIG_LGE_TOUCH_CORE_MTK
	include $(srctree)/drivers/misc/mediatek/Makefile.custom
# ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/synaptics_3320/-
# ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
endif

obj-$(CONFIG_LGE_TOUCH_CORE_BASE)	+= touch_core.o
obj-$(CONFIG_LGE_TOUCH_CORE_BASE)	+= touch_core_sysfs.o
obj-$(CONFIG_LGE_TOUCH_CORE_BASE)	+= touch_i2c.o
obj-$(CONFIG_LGE_TOUCH_CORE_BASE)	+= touch_spi.o
obj-$(CONFIG_LGE_TOUCH_CORE_BASE)	+= touch_model.o
obj-$(CONFIG_LGE_TOUCH_CORE_BASE)	+= touch_notify.o

obj-$(CONFIG_LGE_TOUCH_CORE)		+= touch_hwif.o
obj-$(CONFIG_LGE_TOUCH_CORE_MTK)	+= touch_hwif_mtk.o

obj-$(CONFIG_LGE_TOUCH_LG4945)		+= touch_lg4945.o
obj-$(CONFIG_LGE_TOUCH_LG4945)		+= touch_lg4945_watch.o
obj-$(CONFIG_LGE_TOUCH_LG4945)		+= touch_lg4945_prd.o
obj-$(CONFIG_LGE_TOUCH_LG4945)		+= touch_lg4945_abt.o
obj-$(CONFIG_LGE_TOUCH_SYNAPTICS)	+= touch_synaptics.o

