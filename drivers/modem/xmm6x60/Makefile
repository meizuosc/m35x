# Makefile of modem_if

EXTRA_CFLAGS += -Idrivers/modem/xmm6x60

obj-y += meizu_modem.o modem_io_device.o
obj-y += modem_utils.o

obj-$(CONFIG_UMTS_MODEM_XMM6260) += modemctl_xmm6260.o
obj-$(CONFIG_UMTS_MODEM_XMM6262) += modemctl_xmm6262.o
obj-$(CONFIG_LINK_DEVICE_HSIC) += modem_hsic.o
