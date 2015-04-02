# arch/arm/mach-exynos/Makefile
#
# Copyright (c) 2010-2011 Samsung Electronics Co., Ltd.
#		http://www.samsung.com/
#
# Licensed under GPLv2

obj-y				:=
obj-m				:=
obj-n				:=
obj-				:=

# Core

obj-$(CONFIG_ARCH_EXYNOS)	+= common.o rtc-tick-wakeup.o
obj-$(CONFIG_ARCH_EXYNOS4)	+= clock-exynos4.o asv.o asv-4x12.o
obj-$(CONFIG_ARM_TRUSTZONE)	+= irq-sgi.o
obj-$(CONFIG_CPU_EXYNOS4210)	+= clock-exynos4210.o
obj-$(CONFIG_SOC_EXYNOS4212)	+= clock-exynos4212.o
obj-$(CONFIG_SOC_EXYNOS5250)	+= clock-exynos5250.o
obj-$(CONFIG_SOC_EXYNOS5410)	+= clock-exynos5410.o asv-exynos.o asv-exynos5410.o cci.o
obj-$(CONFIG_EXYNOS5410_POWER_MODE)	+= exynos-power-mode.o

obj-$(CONFIG_PM)		+= pm.o
obj-$(CONFIG_PM_GENERIC_DOMAINS) += pm_domains.o

ifeq ($(CONFIG_SOC_EXYNOS5250),y)
obj-$(CONFIG_CPU_IDLE)         += cpuidle-exynos5250.o
else
obj-$(CONFIG_CPU_IDLE)         += cpuidle.o
endif

obj-$(CONFIG_SOC_EXYNOS5250)	+= ori-asv-exynos.o ori-abb-exynos.o ori-asv-exynos5250.o

obj-$(CONFIG_ARCH_EXYNOS)	+= pmu.o

obj-$(CONFIG_SMP)		+= platsmp.o headsmp.o

obj-$(CONFIG_EXYNOS4_MCT)	+= mct.o

obj-$(CONFIG_HOTPLUG_CPU)	+= hotplug.o

obj-$(CONFIG_ARCH_EXYNOS)	+= clock-audss.o

obj-$(CONFIG_EXYNOS_FIQ_DEBUGGER)	+= exynos_fiq_debugger.o

obj-$(CONFIG_EXYNOS_BUSFREQ_OPP)	+= ppmu.o busfreq_opp_exynos4.o busfreq_opp_4x12.o

obj-$(CONFIG_EXYNOS5_CORESIGHT)		+= coresight-exynos5.o
obj-$(CONFIG_EXYNOS_PERSISTENT_CLOCK)	+= persistent_clock.o

obj-$(CONFIG_EXYNOS5410_DEBUG)		+= debug_exynos5410.o

obj-$(CONFIG_ARM_TRUSTZONE)		+= smc.o
plus_sec := $(call as-instr,.arch_extension sec,+sec)
AFLAGS_smc.o :=-Wa,-march=armv7-a$(plus_sec)

# machine support

obj-$(CONFIG_MACH_SMDKC210)		+= mach-smdkv310.o
obj-$(CONFIG_MACH_SMDKV310)		+= mach-smdkv310.o
obj-$(CONFIG_MACH_ARMLEX4210)		+= mach-armlex4210.o
obj-$(CONFIG_MACH_UNIVERSAL_C210)	+= mach-universal_c210.o
obj-$(CONFIG_MACH_NURI)			+= mach-nuri.o
obj-$(CONFIG_MACH_ORIGEN)		+= mach-origen.o

obj-$(CONFIG_MACH_SMDK4212)		+= mach-smdk4x12.o
obj-$(CONFIG_MACH_SMDK4412)		+= mach-smdk4x12.o
obj-$(CONFIG_MACH_SMDK4412)		+= board-smdk4x12-mmc.o
obj-$(CONFIG_MACH_SMDK4412)		+= board-smdk4x12-audio.o
obj-$(CONFIG_MACH_SMDK4412)		+= board-smdk4x12-display.o
obj-$(CONFIG_MACH_SMDK4412)		+= board-smdk4x12-usb.o
obj-$(CONFIG_MACH_SMDK4412)		+= board-smdk4x12-media.o
obj-$(CONFIG_MACH_SMDK4412)		+= board-smdk4x12-power.o

obj-$(CONFIG_MACH_SMDK5410)		+= mach-smdk5410.o
obj-$(CONFIG_MACH_SMDK5410)		+= board-smdk5410-mmc.o
obj-$(CONFIG_MACH_SMDK5410)		+= board-smdk5410-power.o
obj-$(CONFIG_MACH_SMDK5410)		+= board-smdk5410-usb.o
obj-$(CONFIG_MACH_SMDK5410)		+= board-smdk5410-audio.o
obj-$(CONFIG_MACH_SMDK5410)		+= board-smdk5410-input.o
obj-$(CONFIG_MACH_SMDK5410)		+= board-smdk5410-clock.o
obj-$(CONFIG_MACH_SMDK5410)		+= board-smdk5410-media.o
obj-$(CONFIG_MACH_SMDK5410)		+= board-smdk5410-display.o
obj-$(CONFIG_MACH_TF4)			+= mach-tf4.o
obj-$(CONFIG_MACH_TF4)			+= board-tf4-display.o
obj-$(CONFIG_MACH_TF4)			+= board-tf4-mmc.o
obj-$(CONFIG_MACH_TF4)			+= board-tf4-power.o
obj-$(CONFIG_MACH_TF4)			+= board-tf4-usb.o
obj-$(CONFIG_MACH_TF4)			+= board-tf4-audio.o
obj-$(CONFIG_MACH_TF4)			+= board-tf4-media.o
obj-$(CONFIG_MACH_TF4)			+= board-tf4-input.o
#obj-$(CONFIG_MACH_TF4)			+= board-tf4-modemif.o
obj-$(CONFIG_MACH_TF4)			+= board-tf4-clock.o

obj-$(CONFIG_MACH_M6X)			+= mach-m6x.o hardware.o meizu-sys.o
obj-$(CONFIG_MACH_M6X)			+= devs-gpio-i2c-m6x.o
obj-$(CONFIG_MACH_M6X)			+= board-m6x-reboot.o
obj-$(CONFIG_MACH_M6X)			+= board-m6x-mmc.o
obj-$(CONFIG_MACH_M6X)			+= board-m6x-power.o
obj-$(CONFIG_MACH_M6X)			+= board-m6x-usb.o

ifndef CONFIG_RECOVERY_KERNEL
obj-$(CONFIG_MACH_M6X)	+= board-m6x-audio.o
endif

obj-$(CONFIG_MACH_M6X)			+= board-m6x-input.o
obj-$(CONFIG_MACH_M6X)			+= board-m6x-clock.o
obj-$(CONFIG_MACH_M6X)			+= board-m6x-media.o
obj-$(CONFIG_MACH_M6X)			+= board-m6x-gpio.o
obj-$(CONFIG_MACH_M6X)			+= board-m6x-gpio-leds.o
ifndef CONFIG_RECOVERY_KERNEL
obj-$(CONFIG_MACH_M6X)			+= board-m6x-camera.o
endif
obj-$(CONFIG_MACH_M6X)			+= rsa.o sha.o meizu_security.o

obj-$(CONFIG_MEIZU_MODEM)		+= board-m6x-modems.o
obj-$(CONFIG_MEIZU_MODEM_WD)		+= dev-xmm6260.o
obj-$(CONFIG_FB_S3C)			+= board-m6x-display.o
obj-$(CONFIG_MEIZU_M6X_SENSORS)		+= board-m6x-sensors.o
obj-$(CONFIG_BCMDHD)			+= board-m6x-wlan.o
obj-$(CONFIG_MEIZU_M6X_JACK)		+= dev-m6x-jack.o
obj-$(CONFIG_REGULATOR_FIXED_VOLTAGE)	+= board-m6x-fixed-regulator.o

ifndef CONFIG_RECOVERY_KERNEL
obj-$(CONFIG_MACH_M6X)			+= board-m6x-gps.o
obj-$(CONFIG_MACH_M6X)			+= board-m6x-nfc.o
obj-$(CONFIG_MACH_M6X)			+= board-m6x-bluetooth.o
obj-$(CONFIG_MACH_M6X)			+= board-m6x-factory-test.o
endif

obj-$(CONFIG_MACH_EXYNOS4_DT)		+= mach-exynos4-dt.o
obj-$(CONFIG_MACH_EXYNOS5_DT)		+= mach-exynos5-dt.o

obj-$(CONFIG_MACH_SMDK5250)		+= mach-smdk5250.o

# device support

obj-y					+= dev-uart.o
obj-$(CONFIG_ARCH_EXYNOS)		+= dev-audio.o
obj-$(CONFIG_EXYNOS4_DEV_AHCI)		+= dev-ahci.o
obj-$(CONFIG_EXYNOS_DEV_DWMCI)		+= dev-dwmci.o
obj-$(CONFIG_EXYNOS4_DEV_FIMC_IS)	+= dev-fimc-is.o
obj-$(CONFIG_EXYNOS5_DEV_FIMC_IS)	+= dev-fimc-is.o
obj-$(CONFIG_EXYNOS4_DEV_FIMC_LITE)	+= dev-fimc-lite.o
obj-$(CONFIG_EXYNOS5_DEV_GSC)		+= dev-gsc.o setup-gsc.o
obj-$(CONFIG_EXYNOS5_DEV_SCALER)	+= dev-scaler.o
obj-$(CONFIG_EXYNOS_DEV_ROTATOR)	+= dev-rotator.o
obj-$(CONFIG_EXYNOS_DEV_SYSMMU)		+= dev-sysmmu.o
obj-$(CONFIG_EXYNOS_DEV_DMA)		+= dma.o
obj-$(CONFIG_EXYNOS_DEV_USB_SWITCH)	+= dev-usb-switch.o
obj-$(CONFIG_EXYNOS4_DEV_USB_OHCI)	+= dev-ohci.o
obj-$(CONFIG_EXYNOS5_DEV_USB3_DRD)	+= dev-usb3-drd.o
obj-$(CONFIG_EXYNOS5_DEV_JPEG)		+= dev-jpeg.o
obj-$(CONFIG_EXYNOS5_DEV_JPEG_HX)	+= dev-jpeg-hx.o
obj-$(CONFIG_EXYNOS_DEV_TMU)		+= dev-tmu.o
obj-$(CONFIG_EXYNOS5_DEV_BTS)		+= dev-bts.o
obj-$(CONFIG_EXYNOS5_DEV_HSI2C0)	+= dev-hs-i2c0.o
obj-$(CONFIG_EXYNOS5_DEV_HSI2C1)	+= dev-hs-i2c1.o
obj-$(CONFIG_EXYNOS5_DEV_HSI2C2)	+= dev-hs-i2c2.o
obj-$(CONFIG_EXYNOS5_DEV_HSI2C3)	+= dev-hs-i2c3.o

obj-$(CONFIG_ARCH_EXYNOS)		+= setup-i2c0.o
obj-$(CONFIG_EXYNOS4_SETUP_FIMC)	+= setup-fimc.o
obj-$(CONFIG_EXYNOS4_SETUP_CSIS)	+= setup-csis.o
obj-$(CONFIG_EXYNOS_SETUP_FIMD0)	+= setup-fimd0.o
obj-$(CONFIG_EXYNOS_SETUP_FIMD1)	+= setup-fimd1.o
obj-$(CONFIG_EXYNOS_SETUP_DP)		+= setup-dp.o
obj-$(CONFIG_FB_MIPI_DSIM)		+= setup-mipidsim.o setup-dp.o
obj-$(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION) += secmem.o
obj-$(CONFIG_EXYNOS4_SETUP_FIMC_IS)	+= setup-fimc-is.o
obj-$(CONFIG_EXYNOS5_SETUP_FIMC_IS)	+= setup-fimc-is.o
obj-$(CONFIG_VISION_MODE)		+= setup-fimc-is-sensor.o
obj-$(CONFIG_EXYNOS4_SETUP_I2C1)	+= setup-i2c1.o
obj-$(CONFIG_EXYNOS4_SETUP_I2C2)	+= setup-i2c2.o
obj-$(CONFIG_EXYNOS4_SETUP_I2C3)	+= setup-i2c3.o
obj-$(CONFIG_EXYNOS4_SETUP_I2C4)	+= setup-i2c4.o
obj-$(CONFIG_EXYNOS4_SETUP_I2C5)	+= setup-i2c5.o
obj-$(CONFIG_EXYNOS4_SETUP_I2C6)	+= setup-i2c6.o
obj-$(CONFIG_EXYNOS4_SETUP_I2C7)	+= setup-i2c7.o
obj-$(CONFIG_EXYNOS5_SETUP_HSI2C0)	+= setup-hs-i2c0.o
obj-$(CONFIG_EXYNOS5_SETUP_HSI2C1)	+= setup-hs-i2c1.o
obj-$(CONFIG_EXYNOS5_SETUP_HSI2C2)	+= setup-hs-i2c2.o
obj-$(CONFIG_EXYNOS5_SETUP_HSI2C3)	+= setup-hs-i2c3.o
obj-$(CONFIG_EXYNOS4_SETUP_KEYPAD)	+= setup-keypad.o
obj-$(CONFIG_EXYNOS4_SETUP_MFC)		+= setup-mfc.o
obj-$(CONFIG_EXYNOS4_SETUP_SDHCI_GPIO)	+= setup-sdhci-gpio.o
obj-$(CONFIG_EXYNOS4_SETUP_USB_PHY)	+= setup-usb-phy.o
obj-$(CONFIG_EXYNOS_SETUP_SPI)		+= setup-spi.o
obj-$(CONFIG_EXYNOS5_SETUP_TVOUT)	+= setup-tvout.o
obj-$(CONFIG_EXYNOS_SETUP_ADC)		+= setup-adc.o
obj-$(CONFIG_ION_EXYNOS)		+= dev-ion.o
obj-$(CONFIG_CMA)			+= reserve-mem.o
obj-$(CONFIG_EXYNOS5_SETUP_JPEG)	+= setup-jpeg.o
obj-$(CONFIG_EXYNOS5_SETUP_JPEG_HX)	+= setup-jpeg-hx.o
obj-$(CONFIG_RESET_REASON)		+= resetreason.o
obj-$(CONFIG_EXYNOS_TMU)		+= tmu-exynos.o

obj-$(CONFIG_EXYNOS5410_BTS)		+= bts-exynos5410.o
obj-$(CONFIG_BL_SWITCHER)		+= bL_control.o bL_setup.o
obj-$(CONFIG_TOUCH_BOOSTER)		+= touch_booster.o
