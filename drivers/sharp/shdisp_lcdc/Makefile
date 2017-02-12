ifneq (, $(filter F%, $(SH_BUILD_ID)))
EXTRA_CFLAGS += -DSHDISP_FACTORY_MODE_ENABLE
endif

obj-$(CONFIG_USES_SHLCDC) += shdisp_kerl.o

obj-$(CONFIG_USES_SHLCDC) += shdisp_system.o

obj-$(CONFIG_USES_SHLCDC) += shdisp_bl71y6.o

obj-$(CONFIG_USES_SHLCDC) += shdisp_pm.o
obj-$(CONFIG_USES_SHLCDC) += shdisp_panel.o

ifeq ($(CONFIG_SHDISP_PANEL_ANDY),y)
obj-$(CONFIG_USES_SHLCDC) += shdisp_andy.o
endif

obj-$(CONFIG_USES_SHLCDC) += shdisp_clmr.o
obj-$(CONFIG_USES_SHLCDC) += shdisp_clmr_usr.o

obj-$(CONFIG_USES_SHLCDC) += shdisp_dbg.o
EXTRA_CFLAGS    += -DSH_BUILD_ID=\"$(SH_BUILD_ID)\"
EXTRA_CFLAGS    += -DUSE_LINUX
