/*
 * Copyright (c) 2010-2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS - Power management unit definition
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_REGS_PMU_H
#define __ASM_ARCH_REGS_PMU_H __FILE__

#include <mach/map.h>

#define EXYNOS_PMUREG(x)			(S5P_VA_PMU + (x))

#define EXYNOS_CENTRAL_SEQ_CONFIGURATION	EXYNOS_PMUREG(0x0200)
#define EXYNOS_CENTRAL_LOWPWR_CFG		(1 << 16)

#define EXYNOS_CENTRAL_SEQ_OPTION		EXYNOS_PMUREG(0x0208)

#define EXYNOS_SEQ_TRANSITION(_nr)		EXYNOS_PMUREG(0x0220 + (_nr) * 4)
#define EXYNOS_SEQ_COREBLK_TRANSITION(_nr)	EXYNOS_PMUREG(0x0260 + (_nr) * 4)

#define S5P_SWRESET				EXYNOS_PMUREG(0x0400)
#define EXYNOS_SWRESET				EXYNOS_PMUREG(0x0400)

#define EXYNOS_RST_STAT				EXYNOS_PMUREG(0x0404)
#define EXYNOS_AUTOMATIC_WDT_RESET_DISABLE	EXYNOS_PMUREG(0x0408)
#define EXYNOS_MASK_WDT_RESET_REQUEST		EXYNOS_PMUREG(0x040C)
#define EXYNOS_SYS_WDTRESET			(1 << 20)

#define EXYNOS_WAKEUP_STAT			EXYNOS_PMUREG(0x0600)
#define EXYNOS_EINT_WAKEUP_MASK			EXYNOS_PMUREG(0x0604)
#define EXYNOS_WAKEUP_MASK			EXYNOS_PMUREG(0x0608)

#define EXYNOS_HDMI_PHY_CONTROL			EXYNOS_PMUREG(0x0700)
#define EXYNOS_HDMI_PHY_ENABLE			(1 << 0)

#define S5P_MIPI_DPHY_CONTROL(n)		(soc_is_exynos5410() ? \
						EXYNOS_PMUREG(0x0714 + (n) * 4) : \
						EXYNOS_PMUREG(0x0710 + (n) * 4))
#define S5P_MIPI_DPHY_ENABLE			(1 << 0)
#define S5P_MIPI_DPHY_SRESETN			(1 << 1)
#define S5P_MIPI_DPHY_MRESETN			(1 << 2)

#define ABB_MODE_060V				0
#define ABB_MODE_065V				1
#define ABB_MODE_070V				2
#define ABB_MODE_075V				3
#define ABB_MODE_080V				4
#define ABB_MODE_085V				5
#define ABB_MODE_090V				6
#define ABB_MODE_095V				7
#define ABB_MODE_100V				8
#define ABB_MODE_105V				9
#define ABB_MODE_110V				10
#define ABB_MODE_115V				11
#define ABB_MODE_120V				12
#define ABB_MODE_125V				13
#define ABB_MODE_130V				14
#define ABB_MODE_135V				15
#define ABB_MODE_140V				16
#define ABB_MODE_145V				17
#define ABB_MODE_150V				18
#define ABB_MODE_155V				19
#define ABB_MODE_160V				20
#define ABB_MODE_BYPASS				255
#define EXYNOS_ABB_INIT				(0x80000080)
#define EXYNOS_ABB_INIT_BYPASS			(0x80000000)

#define EXYNOS_INFORM0				EXYNOS_PMUREG(0x0800)
#define EXYNOS_INFORM1				EXYNOS_PMUREG(0x0804)
#define EXYNOS_INFORM2				EXYNOS_PMUREG(0x0808)
#define EXYNOS_INFORM3				EXYNOS_PMUREG(0x080C)
#define EXYNOS_INFORM4				EXYNOS_PMUREG(0x0810)
#define EXYNOS_INFORM5				EXYNOS_PMUREG(0x0814)
#define EXYNOS_INFORM6				EXYNOS_PMUREG(0x0818)
#define EXYNOS_INFORM7				EXYNOS_PMUREG(0x081C)
#define EXYNOS_PMU_SPARE0			EXYNOS_PMUREG(0x0900)
#define EXYNOS_PMU_SPARE1			EXYNOS_PMUREG(0x0904)
#define EXYNOS_PMU_SPARE2			EXYNOS_PMUREG(0x0908)
#define EXYNOS_IROM_DATA0			EXYNOS_PMUREG(0x0980)
#define EXYNOS_IROM_DATA1			EXYNOS_PMUREG(0x0984)
#define EXYNOS_IROM_DATA2			EXYNOS_PMUREG(0x0988)
#define EXYNOS_IROM_DATA3			EXYNOS_PMUREG(0x098C)
#define EXYNOS_CHECK_SLEEP			0x00000BAD
#define EXYNOS_CHECK_DIDLE			0xBAD00000
#define EXYNOS_CHECK_LPA			0xABAD0000

#define EXYNOS_PMU_DEBUG			EXYNOS_PMUREG(0x0A00)

#define EXYNOS_ARM_CORE0_CONFIGURATION		EXYNOS_PMUREG(0x2000)
#define EXYNOS_ARM_CORE0_STATUS			EXYNOS_PMUREG(0x2004)
#define EXYNOS_ARM_CORE0_OPTION			EXYNOS_PMUREG(0x2008)
#define EXYNOS_ARM_CORE1_CONFIGURATION		EXYNOS_PMUREG(0x2080)
#define EXYNOS_ARM_CORE1_STATUS			EXYNOS_PMUREG(0x2084)
#define EXYNOS_ARM_CORE1_OPTION			EXYNOS_PMUREG(0x2088)
#define EXYNOS_ARM_CORE_OPTION(_nr)		(EXYNOS_ARM_CORE0_OPTION \
						+ ((_nr) * 0x80))
#define EXYNOS_USE_DELAYED_RESET_ASSERTION	BIT(12)
#define EXYNOS_ARM_CORE_STATUS(_nr)		(EXYNOS_ARM_CORE0_STATUS \
						+ ((_nr) * 0x80))
#define EXYNOS_ARM_CORE_CONFIGURATION(_nr)	\
			(EXYNOS_ARM_CORE0_CONFIGURATION + ((_nr) * 0x80))
#define EXYNOS_CORE_LOCAL_PWR_EN		0x3

#define EXYNOS_ARM_COMMON_CONFIGURATION		EXYNOS_PMUREG(0x2500)
#define EXYNOS_ARM_COMMON_STATUS		EXYNOS_PMUREG(0x2504)
#define EXYNOS_COMMON_CONFIGURATION(_nr)	\
			(EXYNOS_ARM_COMMON_CONFIGURATION + ((_nr) * 0x80))
#define EXYNOS_COMMON_STATUS(_nr)		\
			(EXYNOS_COMMON_CONFIGURATION(_nr) + 0x4)
#define EXYNOS_COMMON_OPTION(_nr)		\
			(EXYNOS_COMMON_CONFIGURATION(_nr) + 0x8)

#define EXYNOS_ARM_L2_CONFIGURATION		EXYNOS_PMUREG(0x2600)
#define EXYNOS_L2_CONFIGURATION(_nr)	\
			(EXYNOS_ARM_L2_CONFIGURATION + ((_nr) * 0x80))
#define EXYNOS_L2_STATUS(_nr)		\
			(EXYNOS_L2_CONFIGURATION(_nr) + 0x4)
#define EXYNOS_L2_OPTION(_nr)		\
			(EXYNOS_L2_CONFIGURATION(_nr) + 0x8)

#define EXYNOS_L2_COMMON_PWR_EN			0x3

/* KFC Start */
#define EXYNOS_KFC_CORE0_CONFIGURATION		EXYNOS_PMUREG(0x2200)
#define EXYNOS_KFC_CORE0_STATUS			EXYNOS_PMUREG(0x2204)
#define EXYNOS_KFC_CORE0_OPTION			EXYNOS_PMUREG(0x2208)
#define EXYNOS_KFC_CORE_OPTION(_nr)		(EXYNOS_KFC_CORE0_OPTION \
						+ ((_nr) * 0x80))
#define EXYNOS_KFC_CORE_STATUS(_nr)		(EXYNOS_KFC_CORE0_STATUS \
						+ ((_nr) * 0x80))
#define EXYNOS_KFC_CORE_CONFIGURATION(_nr)	\
			(EXYNOS_KFC_CORE0_CONFIGURATION + ((_nr) * 0x80))
/* KFC End */

#define EXYNOS5410_ISP_ARM_CONFIGURATION			EXYNOS_PMUREG(0x2480)
#define EXYNOS5410_ISP_ARM_STATUS				EXYNOS_PMUREG(0x2484)

#define EXYNOS_PAD_RET_DRAM_OPTION		EXYNOS_PMUREG(0x3008)
#define EXYNOS_PAD_RET_MAUDIO_OPTION		EXYNOS_PMUREG(0x3028)
#define EXYNOS_PAD_RET_JTAG_OPTION		EXYNOS_PMUREG(0x3048)
#define EXYNOS_PAD_RET_GPIO_OPTION		EXYNOS_PMUREG(0x3108)
#define EXYNOS_PAD_RET_UART_OPTION		EXYNOS_PMUREG(0x3128)
#define EXYNOS_PAD_RET_MMCA_OPTION		EXYNOS_PMUREG(0x3148)
#define EXYNOS_PAD_RET_MMCB_OPTION		EXYNOS_PMUREG(0x3168)
#define EXYNOS_PAD_RET_EBIA_OPTION		EXYNOS_PMUREG(0x3188)
#define EXYNOS_PAD_RET_EBIB_OPTION		EXYNOS_PMUREG(0x31A8)

#define EXYNOS_PS_HOLD_CONTROL			EXYNOS_PMUREG(0x330C)

/* For XXX_IP_CONFIGURATION(Power Domain) */
#define EXYNOS_INT_LOCAL_PWR_EN			0x7

/* For SYS_PWR_REG */
#define EXYNOS_SYS_PWR_CFG			(1 << 0)

/* Only for EXYNOS4XXX */
#define EXYNOS4_ABB_INT				EXYNOS_PMUREG(0x0780)
#define EXYNOS4_ABB_MIF				EXYNOS_PMUREG(0x0784)
#define EXYNOS4_ABB_G3D				EXYNOS_PMUREG(0x0788)
#define EXYNOS4_ABB_ARM				EXYNOS_PMUREG(0x078C)
#define EXYNOS4_ABB_MEMBER(_nr)			(EXYNOS4_ABB_INT + (_nr * 0x4))

#define EXYNOS4_ARM_CORE0_LOWPWR		EXYNOS_PMUREG(0x1000)
#define EXYNOS4_DIS_IRQ_CORE0			EXYNOS_PMUREG(0x1004)
#define EXYNOS4_DIS_IRQ_CENTRAL0		EXYNOS_PMUREG(0x1008)
#define EXYNOS4_ARM_CORE1_LOWPWR		EXYNOS_PMUREG(0x1010)
#define EXYNOS4_DIS_IRQ_CORE1			EXYNOS_PMUREG(0x1014)
#define EXYNOS4_DIS_IRQ_CENTRAL1		EXYNOS_PMUREG(0x1018)
#define EXYNOS4_ARM_CORE2_LOWPWR		EXYNOS_PMUREG(0x1020)
#define EXYNOS4_DIS_IRQ_CORE2			EXYNOS_PMUREG(0x1024)
#define EXYNOS4_DIS_IRQ_CENTRAL2		EXYNOS_PMUREG(0x1028)
#define EXYNOS4_ARM_CORE3_LOWPWR		EXYNOS_PMUREG(0x1030)
#define EXYNOS4_DIS_IRQ_CORE3			EXYNOS_PMUREG(0x1034)
#define EXYNOS4_DIS_IRQ_CENTRAL3		EXYNOS_PMUREG(0x1038)
#define EXYNOS4_ARM_COMMON_LOWPWR		EXYNOS_PMUREG(0x1080)
#define EXYNOS4_L2_0_LOWPWR			EXYNOS_PMUREG(0x10C0)
#define EXYNOS4_L2_1_LOWPWR			EXYNOS_PMUREG(0x10C4)
#define EXYNOS4_CMU_ACLKSTOP_LOWPWR		EXYNOS_PMUREG(0x1100)
#define EXYNOS4_CMU_SCLKSTOP_LOWPWR		EXYNOS_PMUREG(0x1104)
#define EXYNOS4_CMU_RESET_LOWPWR		EXYNOS_PMUREG(0x110C)
#define EXYNOS4_APLL_SYSCLK_LOWPWR		EXYNOS_PMUREG(0x1120)
#define EXYNOS4_MPLL_SYSCLK_LOWPWR		EXYNOS_PMUREG(0x1124)
#define EXYNOS4_VPLL_SYSCLK_LOWPWR		EXYNOS_PMUREG(0x1128)
#define EXYNOS4_EPLL_SYSCLK_LOWPWR		EXYNOS_PMUREG(0x112C)
#define EXYNOS4_CMU_CLKSTOP_GPS_ALIVE_LOWPWR	EXYNOS_PMUREG(0x1138)
#define EXYNOS4_CMU_RESET_GPSALIVE_LOWPWR	EXYNOS_PMUREG(0x113C)
#define EXYNOS4_CMU_CLKSTOP_CAM_LOWPWR		EXYNOS_PMUREG(0x1140)
#define EXYNOS4_CMU_CLKSTOP_TV_LOWPWR		EXYNOS_PMUREG(0x1144)
#define EXYNOS4_CMU_CLKSTOP_MFC_LOWPWR		EXYNOS_PMUREG(0x1148)
#define EXYNOS4_CMU_CLKSTOP_G3D_LOWPWR		EXYNOS_PMUREG(0x114C)
#define EXYNOS4_CMU_CLKSTOP_LCD0_LOWPWR		EXYNOS_PMUREG(0x1150)
#define EXYNOS4_CMU_CLKSTOP_MAUDIO_LOWPWR	EXYNOS_PMUREG(0x1158)
#define EXYNOS4_CMU_CLKSTOP_GPS_LOWPWR		EXYNOS_PMUREG(0x115C)
#define EXYNOS4_CMU_RESET_CAM_LOWPWR		EXYNOS_PMUREG(0x1160)
#define EXYNOS4_CMU_RESET_TV_LOWPWR		EXYNOS_PMUREG(0x1164)
#define EXYNOS4_CMU_RESET_MFC_LOWPWR		EXYNOS_PMUREG(0x1168)
#define EXYNOS4_CMU_RESET_G3D_LOWPWR		EXYNOS_PMUREG(0x116C)
#define EXYNOS4_CMU_RESET_LCD0_LOWPWR		EXYNOS_PMUREG(0x1170)
#define EXYNOS4_CMU_RESET_MAUDIO_LOWPWR		EXYNOS_PMUREG(0x1178)
#define EXYNOS4_CMU_RESET_GPS_LOWPWR		EXYNOS_PMUREG(0x117C)
#define EXYNOS4_TOP_BUS_LOWPWR			EXYNOS_PMUREG(0x1180)
#define EXYNOS4_TOP_RETENTION_LOWPWR		EXYNOS_PMUREG(0x1184)
#define EXYNOS4_TOP_PWR_LOWPWR			EXYNOS_PMUREG(0x1188)
#define EXYNOS4_LOGIC_RESET_LOWPWR		EXYNOS_PMUREG(0x11A0)
#define EXYNOS4_ONENAND_MEM_LOWPWR		EXYNOS_PMUREG(0x11C0)
#define EXYNOS4_G2D_ACP_MEM_LOWPWR		EXYNOS_PMUREG(0x11C8)
#define EXYNOS4_USBOTG_MEM_LOWPWR		EXYNOS_PMUREG(0x11CC)
#define EXYNOS4_HSMMC_MEM_LOWPWR		EXYNOS_PMUREG(0x11D0)
#define EXYNOS4_CSSYS_MEM_LOWPWR		EXYNOS_PMUREG(0x11D4)
#define EXYNOS4_SECSS_MEM_LOWPWR		EXYNOS_PMUREG(0x11D8)
#define EXYNOS4_PAD_RETENTION_DRAM_LOWPWR	EXYNOS_PMUREG(0x1200)
#define EXYNOS4_PAD_RETENTION_MAUDIO_LOWPWR	EXYNOS_PMUREG(0x1204)
#define EXYNOS4_PAD_RETENTION_GPIO_LOWPWR	EXYNOS_PMUREG(0x1220)
#define EXYNOS4_PAD_RETENTION_UART_LOWPWR	EXYNOS_PMUREG(0x1224)
#define EXYNOS4_PAD_RETENTION_MMCA_LOWPWR	EXYNOS_PMUREG(0x1228)
#define EXYNOS4_PAD_RETENTION_MMCB_LOWPWR	EXYNOS_PMUREG(0x122C)
#define EXYNOS4_PAD_RETENTION_EBIA_LOWPWR	EXYNOS_PMUREG(0x1230)
#define EXYNOS4_PAD_RETENTION_EBIB_LOWPWR	EXYNOS_PMUREG(0x1234)
#define EXYNOS4_PAD_RETENTION_ISOLATION_LOWPWR	EXYNOS_PMUREG(0x1240)
#define EXYNOS4_PAD_RETENTION_ALV_SEL_LOWPWR	EXYNOS_PMUREG(0x1260)
#define EXYNOS4_XUSBXTI_LOWPWR			EXYNOS_PMUREG(0x1280)
#define EXYNOS4_XXTI_LOWPWR			EXYNOS_PMUREG(0x1284)
#define EXYNOS4_EXT_REGULATOR_LOWPWR		EXYNOS_PMUREG(0x12C0)
#define EXYNOS4_GPIO_MODE_LOWPWR		EXYNOS_PMUREG(0x1300)
#define EXYNOS4_GPIO_MODE_MAUDIO_LOWPWR		EXYNOS_PMUREG(0x1340)
#define EXYNOS4_CAM_LOWPWR			EXYNOS_PMUREG(0x1380)
#define EXYNOS4_TV_LOWPWR			EXYNOS_PMUREG(0x1384)
#define EXYNOS4_MFC_LOWPWR			EXYNOS_PMUREG(0x1388)
#define EXYNOS4_G3D_LOWPWR			EXYNOS_PMUREG(0x138C)
#define EXYNOS4_LCD0_LOWPWR			EXYNOS_PMUREG(0x1390)
#define EXYNOS4_MAUDIO_LOWPWR			EXYNOS_PMUREG(0x1398)
#define EXYNOS4_GPS_LOWPWR			EXYNOS_PMUREG(0x139C)
#define EXYNOS4_GPS_ALIVE_LOWPWR		EXYNOS_PMUREG(0x13A0)

#define EXYNOS4_ARM_COMMON_OPTION		EXYNOS_PMUREG(0x2408)
#define EXYNOS4_TOP_PWR_OPTION			EXYNOS_PMUREG(0x2C48)
#define EXYNOS4_CAM_OPTION			EXYNOS_PMUREG(0x3C08)
#define EXYNOS4_TV_OPTION			EXYNOS_PMUREG(0x3C28)
#define EXYNOS4_MFC_OPTION			EXYNOS_PMUREG(0x3C48)
#define EXYNOS4_G3D_OPTION			EXYNOS_PMUREG(0x3C68)
#define EXYNOS4_LCD0_OPTION			EXYNOS_PMUREG(0x3C88)
#define EXYNOS4_MAUDIO_OPTION			EXYNOS_PMUREG(0x3CC8)
#define EXYNOS4_GPS_OPTION			EXYNOS_PMUREG(0x3CE8)
#define EXYNOS4_GPS_ALIVE_OPTION		EXYNOS_PMUREG(0x3D08)

#define EXYNOS4_CAM_CONFIGURATION		EXYNOS_PMUREG(0x3C00)
#define EXYNOS4_TV_CONFIGURATION		EXYNOS_PMUREG(0x3C20)
#define EXYNOS4_MFC_CONFIGURATION		EXYNOS_PMUREG(0x3C40)
#define EXYNOS4_G3D_CONFIGURATION		EXYNOS_PMUREG(0x3C60)
#define EXYNOS4_LCD0_CONFIGURATION		EXYNOS_PMUREG(0x3C80)
#define EXYNOS4_GPS_CONFIGURATION		EXYNOS_PMUREG(0x3CE0)

/* For EXYNOS_CENTRAL_SEQ_OPTION */
#define EXYNOS4_USE_STANDBY_WFI0		(1 << 16)
#define EXYNOS4_USE_STANDBY_WFI1		(1 << 17)
#define EXYNOS4_USE_STANDBYWFI_ISP_ARM		(1 << 18)
#define EXYNOS4_USE_STANDBY_WFE0		(1 << 24)
#define EXYNOS4_USE_STANDBY_WFE1		(1 << 25)
#define EXYNOS4_USE_STANDBYWFE_ISP_ARM		(1 << 26)

/* Only for EXYNOS4210 */
#define EXYNOS4210_USBDEV_PHY_CONTROL		EXYNOS_PMUREG(0x0704)
#define EXYNOS4210_USBHOST_PHY_CONTROL		EXYNOS_PMUREG(0x0708)
#define EXYNOS4210_DAC_PHY_CONTROL		EXYNOS_PMUREG(0x070C)
#define EXYNOS4210_DAC_PHY_ENABLE		(1 << 0)

#define EXYNOS4210_PMU_SATA_PHY_CONTROL		EXYNOS_PMUREG(0x0720)
#define EXYNOS4210_PMU_SATA_PHY_CONTROL_EN	(1 << 0)

#define EXYNOS4210_CMU_CLKSTOP_LCD1_LOWPWR	EXYNOS_PMUREG(0x1154)
#define EXYNOS4210_CMU_RESET_LCD1_LOWPWR	EXYNOS_PMUREG(0x1174)
#define EXYNOS4210_MODIMIF_MEM_LOWPWR		EXYNOS_PMUREG(0x11C4)
#define EXYNOS4210_PCIE_MEM_LOWPWR		EXYNOS_PMUREG(0x11E0)
#define EXYNOS4210_SATA_MEM_LOWPWR		EXYNOS_PMUREG(0x11E4)
#define EXYNOS4210_LCD1_LOWPWR			EXYNOS_PMUREG(0x1394)

#define EXYNOS4210_LCD1_CONFIGURATION		EXYNOS_PMUREG(0x3CA0)
#define EXYNOS4210_LCD1_OPTION			EXYNOS_PMUREG(0x3CA8)

/* Only for EXYNOS4212 & EXYNOS4412 */
#define EXYNOS4x12_USB_PHY_CONTROL			EXYNOS_PMUREG(0x0704)
#define EXYNOS4x12_HSIC0_PHY_CONTROL			EXYNOS_PMUREG(0x0708)
#define EXYNOS4x12_HSIC1_PHY_CONTROL			EXYNOS_PMUREG(0x070C)
#define EXYNOS4X12_ISP_ARM_LOWPWR			EXYNOS_PMUREG(0x1050)
#define EXYNOS4X12_DIS_IRQ_ISP_ARM_LOCAL_LOWPWR		EXYNOS_PMUREG(0x1054)
#define EXYNOS4X12_DIS_IRQ_ISP_ARM_CENTRAL_LOWPWR	EXYNOS_PMUREG(0x1058)
#define EXYNOS4X12_CMU_ACLKSTOP_COREBLK_LOWPWR		EXYNOS_PMUREG(0x1110)
#define EXYNOS4X12_CMU_SCLKSTOP_COREBLK_LOWPWR		EXYNOS_PMUREG(0x1114)
#define EXYNOS4X12_CMU_RESET_COREBLK_LOWPWR		EXYNOS_PMUREG(0x111C)
#define EXYNOS4X12_MPLLUSER_SYSCLK_LOWPWR		EXYNOS_PMUREG(0x1130)
#define EXYNOS4X12_CMU_CLKSTOP_ISP_LOWPWR		EXYNOS_PMUREG(0x1154)
#define EXYNOS4X12_CMU_RESET_ISP_LOWPWR			EXYNOS_PMUREG(0x1174)
#define EXYNOS4X12_TOP_BUS_COREBLK_LOWPWR		EXYNOS_PMUREG(0x1190)
#define EXYNOS4X12_TOP_RETENTION_COREBLK_LOWPWR		EXYNOS_PMUREG(0x1194)
#define EXYNOS4X12_TOP_PWR_COREBLK_LOWPWR		EXYNOS_PMUREG(0x1198)
#define EXYNOS4X12_OSCCLK_GATE_LOWPWR			EXYNOS_PMUREG(0x11A4)
#define EXYNOS4X12_LOGIC_RESET_COREBLK_LOWPWR		EXYNOS_PMUREG(0x11B0)
#define EXYNOS4X12_OSCCLK_GATE_COREBLK_LOWPWR		EXYNOS_PMUREG(0x11B4)
#define EXYNOS4X12_HSI_MEM_LOWPWR			EXYNOS_PMUREG(0x11C4)
#define EXYNOS4X12_ROTATOR_MEM_LOWPWR			EXYNOS_PMUREG(0x11DC)
#define EXYNOS4X12_PAD_RETENTION_GPIO_COREBLK_LOWPWR	EXYNOS_PMUREG(0x123C)
#define EXYNOS4X12_PAD_ISOLATION_COREBLK_LOWPWR		EXYNOS_PMUREG(0x1250)
#define EXYNOS4X12_GPIO_MODE_COREBLK_LOWPWR		EXYNOS_PMUREG(0x1320)
#define EXYNOS4X12_TOP_ASB_RESET_LOWPWR			EXYNOS_PMUREG(0x1344)
#define EXYNOS4X12_TOP_ASB_ISOLATION_LOWPWR		EXYNOS_PMUREG(0x1348)
#define EXYNOS4X12_ISP_LOWPWR				EXYNOS_PMUREG(0x1394)
#define EXYNOS4X12_DRAM_FREQ_DOWN_LOWPWR		EXYNOS_PMUREG(0x13B0)
#define EXYNOS4X12_DDRPHY_DLLOFF_LOWPWR			EXYNOS_PMUREG(0x13B4)
#define EXYNOS4X12_CMU_SYSCLK_ISP_LOWPWR		EXYNOS_PMUREG(0x13B8)
#define EXYNOS4X12_CMU_SYSCLK_GPS_LOWPWR		EXYNOS_PMUREG(0x13BC)
#define EXYNOS4X12_LPDDR_PHY_DLL_LOCK_LOWPWR		EXYNOS_PMUREG(0x13C0)

#define EXYNOS4X12_ARM_L2_0_OPTION			EXYNOS_PMUREG(0x2608)
#define EXYNOS4X12_ARM_L2_1_OPTION			EXYNOS_PMUREG(0x2628)
#define EXYNOS4X12_ONENAND_MEM_OPTION			EXYNOS_PMUREG(0x2E08)
#define EXYNOS4X12_HSI_MEM_OPTION			EXYNOS_PMUREG(0x2E28)
#define EXYNOS4X12_G2D_ACP_MEM_OPTION			EXYNOS_PMUREG(0x2E48)
#define EXYNOS4X12_USBOTG_MEM_OPTION			EXYNOS_PMUREG(0x2E68)
#define EXYNOS4X12_HSMMC_MEM_OPTION			EXYNOS_PMUREG(0x2E88)
#define EXYNOS4X12_CSSYS_MEM_OPTION			EXYNOS_PMUREG(0x2EA8)
#define EXYNOS4X12_SECSS_MEM_OPTION			EXYNOS_PMUREG(0x2EC8)
#define EXYNOS4X12_ROTATOR_MEM_OPTION			EXYNOS_PMUREG(0x2F48)

#define EXYNOS4x12_ISP_CONFIGURATION		EXYNOS_PMUREG(0x3CA0)

/* Only for EXYNOS5250 */
/* for EXYNOS_WAKEUP_MASK */
#define EXYNOS5_MASK_RTC_ALARM				BIT(1)
#define EXYNOS5_MASK_RTC_TICK				BIT(2)
#define EXYNOS5_MASK_KEY				BIT(5)
#define EXYNOS5_MASK_HSI				BIT(8)
#define EXYNOS5_MASK_MMC0				BIT(9)
#define EXYNOS5_MASK_MMC1				BIT(10)
#define EXYNOS5_MASK_MMC2				BIT(11)
#define EXYNOS5_MASK_MMC3				BIT(12)
#define EXYNOS5_MASK_I2S				BIT(13)
#define EXYNOS5_MASK_TIMER				BIT(14)
#define EXYNOS5_MASK_CEC				BIT(15)
#define EXYNOS5_MASK_EXT_GIC0_IRQ			BIT(16)
#define EXYNOS5_MASK_EXT_GIC0_FIQ			BIT(17)
#define EXYNOS5_MASK_EXT_GIC1_IRQ			BIT(18)
#define EXYNOS5_MASK_EXT_GIC1_FIQ			BIT(19)
#define EXYNOS5_MASK_C2C_RESET_REQ			BIT(20)
#define EXYNOS5_DEFAULT_WAKEUP_MASK			(EXYNOS5_MASK_EXT_GIC0_IRQ |\
							EXYNOS5_MASK_EXT_GIC0_FIQ |\
							EXYNOS5_MASK_EXT_GIC1_IRQ |\
							EXYNOS5_MASK_EXT_GIC1_FIQ)

/* Only for EXYNOS5410 */
/* for EXYNOS_MASK_WDT_RESET_REQUEST */
#define EXYNOS5410_SYS_WDTRESET				(1 << 0)

#define EXYNOS5410_LPI_BUS_MASK0			EXYNOS_PMUREG(0x159C)
#define EXYNOS5410_LPI_BUS_MASK0_ISP0			(1 << 7)
#define EXYNOS5410_LPI_BUS_MASK0_ISP1			(1 << 8)

#define EXYNOS5410_LPI_BUS_MASK1			EXYNOS_PMUREG(0x15A0)
#define EXYNOS5410_LPI_BUS_MASK1_P_ISP			(1 << 16)

#define EXYNOS5410_ISP_ARM_OPTION			EXYNOS_PMUREG(0x2488)

#define EXYNOS5410_MFC_CONFIGURATION			EXYNOS_PMUREG(0x4060)
#define EXYNOS5410_MFC_STATUS				EXYNOS_PMUREG(0x4064)
#define EXYNOS5410_MFC_OPTION				EXYNOS_PMUREG(0x4068)
#define EXYNOS5410_G3D_CONFIGURATION			EXYNOS_PMUREG(0x4080)
#define EXYNOS5410_G3D_STATUS				EXYNOS_PMUREG(0x4084)
#define EXYNOS5410_G3D_OPTION				EXYNOS_PMUREG(0x4088)
#define EXYNOS5410_DISP0_CONFIGURATION			EXYNOS_PMUREG(0x40A0)
#define EXYNOS5410_DISP0_STATUS				EXYNOS_PMUREG(0x40A4)
#define EXYNOS5410_DISP0_OPTION				EXYNOS_PMUREG(0x40A8)
#define EXYNOS5410_DISP1_CONFIGURATION			EXYNOS_PMUREG(0x40C0)
#define EXYNOS5410_DISP1_STATUS				EXYNOS_PMUREG(0x40C4)
#define EXYNOS5410_DISP1_OPTION				EXYNOS_PMUREG(0x40C8)
#define EXYNOS5410_MAU_CONFIGURATION			EXYNOS_PMUREG(0x40E0)
#define EXYNOS5410_MAU_STATUS				EXYNOS_PMUREG(0x40E4)
#define EXYNOS5410_MAU_OPTION				EXYNOS_PMUREG(0x40E8)
#define EXYNOS5250_ADC_PHY_CONTROL			EXYNOS_PMUREG(0x0718)
#define EXYNOS5410_ADC_PHY_CONTROL			EXYNOS_PMUREG(0x0720)
#define EXYNOS5_ADC_PHY_ENABLE				(1 << 0)

#define EXYNOS5_USBDEV_PHY_CONTROL			EXYNOS_PMUREG(0x0704)
#define EXYNOS5_USBDEV_PHY_ENABLE			(1 << 0)
#define EXYNOS5_USBDEV1_PHY_CONTROL			EXYNOS_PMUREG(0x0708)
#define EXYNOS5_USBDEV1_PHY_ENABLE			(1 << 0)

#define EXYNOS5_USBHOST_PHY_CONTROL			(soc_is_exynos5250() ? \
							EXYNOS_PMUREG(0x0708) : \
							EXYNOS_PMUREG(0x070C))
#define EXYNOS5_USBHOST_PHY_ENABLE			(1 << 0)

#define EXYNOS5_ADC_PHY_CONTROL				EXYNOS_PMUREG(0x0718)
#define EXYNOS5_ADC_PHY_ENABLE				(1 << 0)

#define EXYNOS5410_DPTX_PHY_CONTROL			EXYNOS_PMUREG(0x0728)
#define EXYNOS5410_DPTX_PHY_ENABLE			(1 << 0)

#define EXYNOS5_ABB_INT					EXYNOS_PMUREG(0x0780)
#define EXYNOS5_ABB_ARM					EXYNOS_PMUREG(0x0784)
#define EXYNOS5_ABB_G3D					EXYNOS_PMUREG(0x0788)
#define EXYNOS5_ABB_MIF					EXYNOS_PMUREG(0x078C)
#define EXYNOS5_ABB_MEMBER(_member)			EXYNOS_##_member

#define EXYNOS5410_BB_CON0				EXYNOS_PMUREG(0x0780)
#define EXYNOS5410_BB_CON1				EXYNOS_PMUREG(0x0784)
#define EXYNOS5410_BB_SEL_EN				(1 << 31)
#define EXYNOS5410_BB_PMOS_EN				(1 << 7)


#define EXYNOS5_ARM_CORE0_SYS_PWR_REG			EXYNOS_PMUREG(0x1000)
#define EXYNOS5_DIS_IRQ_ARM_CORE0_LOCAL_SYS_PWR_REG	EXYNOS_PMUREG(0x1004)
#define EXYNOS5_DIS_IRQ_ARM_CORE0_CENTRAL_SYS_PWR_REG	EXYNOS_PMUREG(0x1008)
#define EXYNOS5_ARM_CORE1_SYS_PWR_REG			EXYNOS_PMUREG(0x1010)
#define EXYNOS5_DIS_IRQ_ARM_CORE1_LOCAL_SYS_PWR_REG	EXYNOS_PMUREG(0x1014)
#define EXYNOS5_DIS_IRQ_ARM_CORE1_CENTRAL_SYS_PWR_REG	EXYNOS_PMUREG(0x1018)
#define EXYNOS5_FSYS_ARM_SYS_PWR_REG			EXYNOS_PMUREG(0x1040)
#define EXYNOS5_DIS_IRQ_FSYS_ARM_CENTRAL_SYS_PWR_REG	EXYNOS_PMUREG(0x1048)
#define EXYNOS5_ISP_ARM_SYS_PWR_REG			EXYNOS_PMUREG(0x1050)
#define EXYNOS5_DIS_IRQ_ISP_ARM_LOCAL_SYS_PWR_REG	EXYNOS_PMUREG(0x1054)
#define EXYNOS5_DIS_IRQ_ISP_ARM_CENTRAL_SYS_PWR_REG	EXYNOS_PMUREG(0x1058)
#define EXYNOS5_ARM_COMMON_SYS_PWR_REG			EXYNOS_PMUREG(0x1080)
#define EXYNOS5_ARM_L2_SYS_PWR_REG			EXYNOS_PMUREG(0x10C0)
#define EXYNOS5_CMU_ACLKSTOP_SYS_PWR_REG		EXYNOS_PMUREG(0x1100)
#define EXYNOS5_CMU_SCLKSTOP_SYS_PWR_REG		EXYNOS_PMUREG(0x1104)
#define EXYNOS5_CMU_RESET_SYS_PWR_REG			EXYNOS_PMUREG(0x110C)
#define EXYNOS5_CMU_ACLKSTOP_SYSMEM_SYS_PWR_REG		EXYNOS_PMUREG(0x1120)
#define EXYNOS5_CMU_SCLKSTOP_SYSMEM_SYS_PWR_REG		EXYNOS_PMUREG(0x1124)
#define EXYNOS5_CMU_RESET_SYSMEM_SYS_PWR_REG		EXYNOS_PMUREG(0x112C)
#define EXYNOS5_DRAM_FREQ_DOWN_SYS_PWR_REG		EXYNOS_PMUREG(0x1130)
#define EXYNOS5_DDRPHY_DLLOFF_SYS_PWR_REG		EXYNOS_PMUREG(0x1134)
#define EXYNOS5_DDRPHY_DLLLOCK_SYS_PWR_REG		EXYNOS_PMUREG(0x1138)
#define EXYNOS5_APLL_SYSCLK_SYS_PWR_REG			EXYNOS_PMUREG(0x1140)
#define EXYNOS5_MPLL_SYSCLK_SYS_PWR_REG			EXYNOS_PMUREG(0x1144)
#define EXYNOS5_VPLL_SYSCLK_SYS_PWR_REG			EXYNOS_PMUREG(0x1148)
#define EXYNOS5_EPLL_SYSCLK_SYS_PWR_REG			EXYNOS_PMUREG(0x114C)
#define EXYNOS5_BPLL_SYSCLK_SYS_PWR_REG			EXYNOS_PMUREG(0x1150)
#define EXYNOS5_CPLL_SYSCLK_SYS_PWR_REG			EXYNOS_PMUREG(0x1154)
#define EXYNOS5_GPLL_SYSCLK_SYS_PWR_REG			EXYNOS_PMUREG(0x1158)
#define EXYNOS5_MPLLUSER_SYSCLK_SYS_PWR_REG		EXYNOS_PMUREG(0x1164)
#define EXYNOS5_BPLLUSER_SYSCLK_SYS_PWR_REG		EXYNOS_PMUREG(0x1170)
#define EXYNOS5_TOP_BUS_SYS_PWR_REG			EXYNOS_PMUREG(0x1180)
#define EXYNOS5_TOP_RETENTION_SYS_PWR_REG		EXYNOS_PMUREG(0x1184)
#define EXYNOS5_TOP_PWR_SYS_PWR_REG			EXYNOS_PMUREG(0x1188)
#define EXYNOS5_TOP_BUS_SYSMEM_SYS_PWR_REG		EXYNOS_PMUREG(0x1190)
#define EXYNOS5_TOP_RETENTION_SYSMEM_SYS_PWR_REG	EXYNOS_PMUREG(0x1194)
#define EXYNOS5_TOP_PWR_SYSMEM_SYS_PWR_REG		EXYNOS_PMUREG(0x1198)
#define EXYNOS5_LOGIC_RESET_SYS_PWR_REG			EXYNOS_PMUREG(0x11A0)
#define EXYNOS5_OSCCLK_GATE_SYS_PWR_REG			EXYNOS_PMUREG(0x11A4)
#define EXYNOS5_LOGIC_RESET_SYSMEM_SYS_PWR_REG		EXYNOS_PMUREG(0x11B0)
#define EXYNOS5_OSCCLK_GATE_SYSMEM_SYS_PWR_REG		EXYNOS_PMUREG(0x11B4)
#define EXYNOS5_USBOTG_MEM_SYS_PWR_REG			EXYNOS_PMUREG(0x11C0)
#define EXYNOS5_G2D_MEM_SYS_PWR_REG			EXYNOS_PMUREG(0x11C8)
#define EXYNOS5_USBDRD_MEM_SYS_PWR_REG			EXYNOS_PMUREG(0x11CC)
#define EXYNOS5_SDMMC_MEM_SYS_PWR_REG			EXYNOS_PMUREG(0x11D0)
#define EXYNOS5_CSSYS_MEM_SYS_PWR_REG			EXYNOS_PMUREG(0x11D4)
#define EXYNOS5_SECSS_MEM_SYS_PWR_REG			EXYNOS_PMUREG(0x11D8)
#define EXYNOS5_ROTATOR_MEM_SYS_PWR_REG			EXYNOS_PMUREG(0x11DC)
#define EXYNOS5_INTRAM_MEM_SYS_PWR_REG			EXYNOS_PMUREG(0x11E0)
#define EXYNOS5_INTROM_MEM_SYS_PWR_REG			EXYNOS_PMUREG(0x11E4)
#define EXYNOS5_JPEG_MEM_SYS_PWR_REG			EXYNOS_PMUREG(0x11E8)
#define EXYNOS5_HSI_MEM_SYS_PWR_REG			EXYNOS_PMUREG(0x11EC)
#define EXYNOS5_MCUIOP_MEM_SYS_PWR_REG			EXYNOS_PMUREG(0x11F4)
#define EXYNOS5_SATA_MEM_SYS_PWR_REG			EXYNOS_PMUREG(0x11FC)
#define EXYNOS5_PAD_RETENTION_DRAM_SYS_PWR_REG		EXYNOS_PMUREG(0x1200)
#define EXYNOS5_PAD_RETENTION_MAU_SYS_PWR_REG		EXYNOS_PMUREG(0x1204)
#define EXYNOS5_PAD_RETENTION_EFNAND_SYS_PWR_REG	EXYNOS_PMUREG(0x1208)
#define EXYNOS5_PAD_RETENTION_GPIO_SYS_PWR_REG		EXYNOS_PMUREG(0x1220)
#define EXYNOS5_PAD_RETENTION_UART_SYS_PWR_REG		EXYNOS_PMUREG(0x1224)
#define EXYNOS5_PAD_RETENTION_MMCA_SYS_PWR_REG		EXYNOS_PMUREG(0x1228)
#define EXYNOS5_PAD_RETENTION_MMCB_SYS_PWR_REG		EXYNOS_PMUREG(0x122C)
#define EXYNOS5_PAD_RETENTION_EBIA_SYS_PWR_REG		EXYNOS_PMUREG(0x1230)
#define EXYNOS5_PAD_RETENTION_EBIB_SYS_PWR_REG		EXYNOS_PMUREG(0x1234)
#define EXYNOS5_PAD_RETENTION_SPI_SYS_PWR_REG		EXYNOS_PMUREG(0x1238)
#define EXYNOS5_PAD_RETENTION_GPIO_SYSMEM_SYS_PWR_REG	EXYNOS_PMUREG(0x123C)
#define EXYNOS5_PAD_ISOLATION_SYS_PWR_REG		EXYNOS_PMUREG(0x1240)
#define EXYNOS5_PAD_ISOLATION_SYSMEM_SYS_PWR_REG	EXYNOS_PMUREG(0x1250)
#define EXYNOS5_PAD_ALV_SEL_SYS_PWR_REG			EXYNOS_PMUREG(0x1260)
#define EXYNOS5_XUSBXTI_SYS_PWR_REG			EXYNOS_PMUREG(0x1280)
#define EXYNOS5_XXTI_SYS_PWR_REG			EXYNOS_PMUREG(0x1284)
#define EXYNOS5_EXT_REGULATOR_SYS_PWR_REG		EXYNOS_PMUREG(0x12C0)
#define EXYNOS5_GPIO_MODE_SYS_PWR_REG			EXYNOS_PMUREG(0x1300)
#define EXYNOS5_GPIO_MODE_SYSMEM_SYS_PWR_REG		EXYNOS_PMUREG(0x1320)
#define EXYNOS5_GPIO_MODE_MAU_SYS_PWR_REG		EXYNOS_PMUREG(0x1340)
#define EXYNOS5_TOP_ASB_RESET_SYS_PWR_REG		EXYNOS_PMUREG(0x1344)
#define EXYNOS5_TOP_ASB_ISOLATION_SYS_PWR_REG		EXYNOS_PMUREG(0x1348)
#define EXYNOS5_GSCL_SYS_PWR_REG			EXYNOS_PMUREG(0x1400)
#define EXYNOS5_ISP_SYS_PWR_REG				EXYNOS_PMUREG(0x1404)
#define EXYNOS5_MFC_SYS_PWR_REG				EXYNOS_PMUREG(0x1408)
#define EXYNOS5_G3D_SYS_PWR_REG				EXYNOS_PMUREG(0x140C)
#define EXYNOS5_DISP1_SYS_PWR_REG			EXYNOS_PMUREG(0x1414)
#define EXYNOS5_MAU_SYS_PWR_REG				EXYNOS_PMUREG(0x1418)
#define EXYNOS5_CMU_CLKSTOP_GSCL_SYS_PWR_REG		EXYNOS_PMUREG(0x1480)
#define EXYNOS5_CMU_CLKSTOP_ISP_SYS_PWR_REG		EXYNOS_PMUREG(0x1484)
#define EXYNOS5_CMU_CLKSTOP_MFC_SYS_PWR_REG		EXYNOS_PMUREG(0x1488)
#define EXYNOS5_CMU_CLKSTOP_G3D_SYS_PWR_REG		EXYNOS_PMUREG(0x148C)
#define EXYNOS5_CMU_CLKSTOP_DISP1_SYS_PWR_REG		EXYNOS_PMUREG(0x1494)
#define EXYNOS5_CMU_CLKSTOP_MAU_SYS_PWR_REG		EXYNOS_PMUREG(0x1498)
#define EXYNOS5_CMU_SYSCLK_GSCL_SYS_PWR_REG		EXYNOS_PMUREG(0x14C0)
#define EXYNOS5_CMU_SYSCLK_ISP_SYS_PWR_REG		EXYNOS_PMUREG(0x14C4)
#define EXYNOS5_CMU_SYSCLK_MFC_SYS_PWR_REG		EXYNOS_PMUREG(0x14C8)
#define EXYNOS5_CMU_SYSCLK_G3D_SYS_PWR_REG		EXYNOS_PMUREG(0x14CC)
#define EXYNOS5_CMU_SYSCLK_DISP1_SYS_PWR_REG		EXYNOS_PMUREG(0x14D4)
#define EXYNOS5_CMU_SYSCLK_MAU_SYS_PWR_REG		EXYNOS_PMUREG(0x14D8)
#define EXYNOS5_CMU_RESET_GSCL_SYS_PWR_REG		EXYNOS_PMUREG(0x1580)
#define EXYNOS5_CMU_RESET_ISP_SYS_PWR_REG		EXYNOS_PMUREG(0x1584)
#define EXYNOS5_CMU_RESET_MFC_SYS_PWR_REG		EXYNOS_PMUREG(0x1588)
#define EXYNOS5_CMU_RESET_G3D_SYS_PWR_REG		EXYNOS_PMUREG(0x158C)
#define EXYNOS5_CMU_RESET_DISP1_SYS_PWR_REG		EXYNOS_PMUREG(0x1594)
#define EXYNOS5_CMU_RESET_MAU_SYS_PWR_REG		EXYNOS_PMUREG(0x1598)

#define EXYNOS5_ARM_CORE0_OPTION			EXYNOS_PMUREG(0x2008)
#define EXYNOS5_ARM_CORE1_OPTION			EXYNOS_PMUREG(0x2088)
#define EXYNOS5_FSYS_ARM_OPTION				EXYNOS_PMUREG(0x2208)
#define EXYNOS5_ISP_ARM_CONFIGURATION			EXYNOS_PMUREG(0x2280)
#define EXYNOS5_ISP_ARM_STATUS				EXYNOS_PMUREG(0x2284)
#define EXYNOS5_ISP_ARM_OPTION				EXYNOS_PMUREG(0x2288)
#define EXYNOS5_ARM_COMMON_OPTION			EXYNOS_PMUREG(0x2408)
#define EXYNOS5_ARM_L2_OPTION				EXYNOS_PMUREG(0x2608)
#define EXYNOS5_USE_RETENTION				BIT(4)
#define EXYNOS5_TOP_PWR_OPTION				EXYNOS_PMUREG(0x2C48)
#define EXYNOS5_TOP_PWR_SYSMEM_OPTION			EXYNOS_PMUREG(0x2CC8)
#define EXYNOS5_LOGIC_RESET_DURATION3			EXYNOS_PMUREG(0x2D1C)
#define EXYNOS5_DUR_WAIT_RESET_MASK			0xFFFFF
#define EXYNOS5_DUR_WAIT_RESET_MIN			0xF
#define EXYNOS5_JPEG_MEM_OPTION				EXYNOS_PMUREG(0x2F48)
#define EXYNOS5_PAD_RETENTION_SPI_OPTION		EXYNOS_PMUREG(0x31C8)
#define EXYNOS5_PAD_RETENTION_GPIO_SYSMEM_OPTION	EXYNOS_PMUREG(0x31E8)
#define EXYNOS5_XXTI_DURATION3				EXYNOS_PMUREG(0x343C)
#define EXYNOS5_EXT_REGULATOR_DURATION3			EXYNOS_PMUREG(0x361C)
#define EXYNOS5_GSCL_CONFIGURATION			EXYNOS_PMUREG(0x4000)
#define EXYNOS5_GSCL_STATUS				EXYNOS_PMUREG(0x4004)
#define EXYNOS5_ISP_CONFIGURATION			EXYNOS_PMUREG(0x4020)
#define EXYNOS5_ISP_STATUS				EXYNOS_PMUREG(0x4024)
#define EXYNOS5_GSCL_OPTION				EXYNOS_PMUREG(0x4008)
#define EXYNOS5_ISP_OPTION				EXYNOS_PMUREG(0x4028)
#define EXYNOS5_MFC_CONFIGURATION			EXYNOS_PMUREG(0x4040)
#define EXYNOS5_MFC_OPTION				EXYNOS_PMUREG(0x4048)
#define EXYNOS5_G3D_CONFIGURATION			EXYNOS_PMUREG(0x4060)
#define EXYNOS5_G3D_STATUS				EXYNOS_PMUREG(0x4064)
#define EXYNOS5_G3D_OPTION				EXYNOS_PMUREG(0x4068)
#define EXYNOS5_DISP1_CONFIGURATION			EXYNOS_PMUREG(0x40A0)
#define EXYNOS5_DISP1_STATUS				EXYNOS_PMUREG(0x40A4)
#define EXYNOS5_DISP1_OPTION				EXYNOS_PMUREG(0x40A8)
#define EXYNOS5_MAU_CONFIGURATION			EXYNOS_PMUREG(0x40C0)
#define EXYNOS5_MAU_OPTION				EXYNOS_PMUREG(0x40C8)
#define EXYNOS5_LOCAL_POWER_STATE_SHIFT			(16)
#define EXYNOS5_LOCAL_POWER_STATE_MASK			(0x3f)
#define EXYNOS5_USE_SC_COUNTER				(1 << 0)
#define EXYNOS5_USE_SC_FEEDBACK				(1 << 1)

#define EXYNOS5_MANUAL_L2RSTDISABLE_CONTROL		(1 << 2)
#define EXYNOS5_SKIP_DEACTIVATE_ACEACP_IN_PWDN		(1 << 7)

#define EXYNOS5_OPTION_USE_STANDBYWFI			(1 << 16)
#define EXYNOS5_OPTION_USE_STANDBYWFE			(1 << 24)

#define EXYNOS5_OPTION_USE_RETENTION			(1 << 4)

/* Only for EXYNOS5410 */
#define EXYNOS5410_LPI_MASK0					EXYNOS_PMUREG(0x0004)
#define EXYNOS5410_LPI_MASK1					EXYNOS_PMUREG(0x0008)
#define EXYNOS5410_LPI_MASK2					EXYNOS_PMUREG(0x000C)
#define ATB_ISP_ARM						(1 << 12)
#define ATB_KFC							(1 << 13)
#define ATB_NOC							(1 << 14)

#define EXYNOS5410_ARM_INTR_SPREAD_ENABLE			EXYNOS_PMUREG(0x0100)
#define EXYNOS5410_ARM_INTR_SPREAD_USE_STANDBYWFI		EXYNOS_PMUREG(0x0104)
#define EXYNOS5410_UP_SCHEDULER					EXYNOS_PMUREG(0x0120)
#define SPREAD_ENABLE						(0xF)
#define SPREAD_USE_STANDWFI					(0xF)

#define EXYNOS5410_BB_CON1					EXYNOS_PMUREG(0x0784)
#define EXYNOS5410_BB_SEL_EN					(1 << 31)
#define EXYNOS5410_BB_PMOS_EN					(1 << 7)
#define EXYNOS5410_BB_1300X					(0X0F)

#define EXYNOS5410_ARM_CORE2_SYS_PWR_REG			EXYNOS_PMUREG(0x1020)
#define EXYNOS5410_DIS_IRQ_ARM_CORE2_LOCAL_SYS_PWR_REG		EXYNOS_PMUREG(0x1024)
#define EXYNOS5410_DIS_IRQ_ARM_CORE2_CENTRAL_SYS_PWR_REG	EXYNOS_PMUREG(0x1028)
#define EXYNOS5410_ARM_CORE3_SYS_PWR_REG			EXYNOS_PMUREG(0x1030)
#define EXYNOS5410_DIS_IRQ_ARM_CORE3_LOCAL_SYS_PWR_REG		EXYNOS_PMUREG(0x1034)
#define EXYNOS5410_DIS_IRQ_ARM_CORE3_CENTRAL_SYS_PWR_REG	EXYNOS_PMUREG(0x1038)
#define EXYNOS5410_KFC_CORE0_SYS_PWR_REG			EXYNOS_PMUREG(0x1040)
#define EXYNOS5410_DIS_IRQ_KFC_CORE0_LOCAL_SYS_PWR_REG		EXYNOS_PMUREG(0x1044)
#define EXYNOS5410_DIS_IRQ_KFC_CORE0_CENTRAL_SYS_PWR_REG	EXYNOS_PMUREG(0x1048)
#define EXYNOS5410_KFC_CORE1_SYS_PWR_REG			EXYNOS_PMUREG(0x1050)
#define EXYNOS5410_DIS_IRQ_KFC_CORE1_LOCAL_SYS_PWR_REG		EXYNOS_PMUREG(0x1054)
#define EXYNOS5410_DIS_IRQ_KFC_CORE1_CENTRAL_SYS_PWR_REG	EXYNOS_PMUREG(0x1058)
#define EXYNOS5410_KFC_CORE2_SYS_PWR_REG			EXYNOS_PMUREG(0x1060)
#define EXYNOS5410_DIS_IRQ_KFC_CORE2_LOCAL_SYS_PWR_REG		EXYNOS_PMUREG(0x1064)
#define EXYNOS5410_DIS_IRQ_KFC_CORE2_CENTRAL_SYS_PWR_REG	EXYNOS_PMUREG(0x1068)
#define EXYNOS5410_KFC_CORE3_SYS_PWR_REG			EXYNOS_PMUREG(0x1070)
#define EXYNOS5410_DIS_IRQ_KFC_CORE3_LOCAL_SYS_PWR_REG		EXYNOS_PMUREG(0x1074)
#define EXYNOS5410_DIS_IRQ_KFC_CORE3_CENTRAL_SYS_PWR_REG	EXYNOS_PMUREG(0x1078)
#define EXYNOS5410_ISP_ARM_SYS_PWR_REG				EXYNOS_PMUREG(0x1090)
#define EXYNOS5410_DIS_IRQ_ISP_ARM_LOCAL_SYS_PWR_REG		EXYNOS_PMUREG(0x1094)
#define EXYNOS5410_DIS_IRQ_ISP_ARM_CENTRAL_SYS_PWR_REG		EXYNOS_PMUREG(0x1098)
#define EXYNOS5410_ARM_COMMON_SYS_PWR_REG			EXYNOS_PMUREG(0x10A0)
#define EXYNOS5410_KFC_COMMON_SYS_PWR_REG			EXYNOS_PMUREG(0x10B0)
#define EXYNOS5410_KFC_L2_SYS_PWR_REG				EXYNOS_PMUREG(0x10D0)
#define EXYNOS5410_DPLL_SYSCLK_SYS_PWR_REG			EXYNOS_PMUREG(0x1158)
#define EXYNOS5410_IPLL_SYSCLK_SYS_PWR_REG			EXYNOS_PMUREG(0x115C)
#define EXYNOS5410_KPLL_SYSCLK_SYS_PWR_REG			EXYNOS_PMUREG(0x1160)
#define EXYNOS5410_ONENANDXL_MEM_SYS_PWR			EXYNOS_PMUREG(0x11C0)
#define EXYNOS5410_USBDEV_MEM_SYS_PWR				EXYNOS_PMUREG(0x11CC)
#define EXYNOS5410_USBDEV1_MEM_SYS_PWR				EXYNOS_PMUREG(0x11D0)
#define EXYNOS5410_SDMMC_MEM_SYS_PWR				EXYNOS_PMUREG(0x11D4)
#define EXYNOS5410_CSSYS_MEM_SYS_PWR				EXYNOS_PMUREG(0x11D8)
#define EXYNOS5410_SECSS_MEM_SYS_PWR				EXYNOS_PMUREG(0x11DC)
#define EXYNOS5410_ROTATOR_MEM_SYS_PWR				EXYNOS_PMUREG(0x11E0)
#define EXYNOS5410_INTRAM_MEM_SYS_PWR				EXYNOS_PMUREG(0x11E4)
#define EXYNOS5410_INTROM_MEM_SYS_PWR				EXYNOS_PMUREG(0x11E8)
#define EXYNOS5410_JPEG_MEM_SYS_PWR				EXYNOS_PMUREG(0x11EC)
#define EXYNOS5410_SFMC0_MEM_SYS_PWR				EXYNOS_PMUREG(0x11F0)
#define EXYNOS5410_SFMC1_MEM_SYS_PWR				EXYNOS_PMUREG(0x11F4)
#define EXYNOS5410_HSI_MEM_SYS_PWR				EXYNOS_PMUREG(0x11F8)
#define EXYNOS5410_MCUIOP_MEM_SYS_PWR				EXYNOS_PMUREG(0x11FC)
#define EXYNOS5410_PAD_RETENTION_GPIO_SYS_PWR			EXYNOS_PMUREG(0x1210)
#define EXYNOS5410_PAD_RETENTION_UART_SYS_PWR			EXYNOS_PMUREG(0x1214)
#define EXYNOS5410_PAD_RETENTION_MMC0_SYS_PWR			EXYNOS_PMUREG(0x1218)
#define EXYNOS5410_PAD_RETENTION_MMC1_SYS_PWR			EXYNOS_PMUREG(0x121C)
#define EXYNOS5410_PAD_RETENTION_MMC2_SYS_PWR			EXYNOS_PMUREG(0x1220)
#define EXYNOS5410_PAD_RETENTION_HSI_SYS_PWR			EXYNOS_PMUREG(0x1224)
#define EXYNOS5410_PAD_RETENTION_EBIA_SYS_PWR			EXYNOS_PMUREG(0x1228)
#define EXYNOS5410_PAD_RETENTION_EBIB_SYS_PWR			EXYNOS_PMUREG(0x122C)
#define EXYNOS5410_PAD_RETENTION_SPI_SYS_PWR			EXYNOS_PMUREG(0x1230)
#define EXYNOS5410_PAD_RETENTION_GPIO_COREBLK_SYS_PWR		EXYNOS_PMUREG(0x1234)
#define EXYNOS5410_DISP0_SYS_PWR_REG				EXYNOS_PMUREG(0x1410)
#define EXYNOS5410_CMU_CLKSTOP_DISP0_SYS_PWR_REG		EXYNOS_PMUREG(0x1490)
#define EXYNOS5410_CMU_SYSCLK_DISP0_SYS_PWR_REG			EXYNOS_PMUREG(0x14D0)
#define EXYNOS5410_CMU_RESET_DISP0_SYS_PWR			EXYNOS_PMUREG(0x1590)
#define EXYNOS5410_LPI_NOC_MASK0				EXYNOS_PMUREG(0x159C)
#define EXYNOS5410_LPI_NOC_MASK1				EXYNOS_PMUREG(0x15A0)
#define EXYNOS5410_LPI_NOC_MASK2				EXYNOS_PMUREG(0x15A4)
#define EXYNOS_ARM_CORE2_CONFIGURATION				EXYNOS_PMUREG(0x2100)
#define EXYNOS5410_ARM_CORE2_OPTION				EXYNOS_PMUREG(0x2108)
#define EXYNOS_ARM_CORE3_CONFIGURATION				EXYNOS_PMUREG(0x2180)
#define EXYNOS5410_ARM_CORE3_OPTION				EXYNOS_PMUREG(0x2188)
#define EXYNOS5410_ARM_COMMON_STATUS				EXYNOS_PMUREG(0x2504)
#define EXYNOS5410_ARM_COMMON_OPTION				EXYNOS_PMUREG(0x2508)
#define EXYNOS5410_KFC_COMMON_STATUS				EXYNOS_PMUREG(0x2584)
#define EXYNOS5410_KFC_COMMON_OPTION				EXYNOS_PMUREG(0x2588)
#define EXYNOS5410_LOGIC_RESET_DURATION3			EXYNOS_PMUREG(0x2D1C)

#define EXYNOS5410_PAD_RET_GPIO_OPTION			EXYNOS_PMUREG(0x30C8)
#define EXYNOS5410_PAD_RET_UART_OPTION			EXYNOS_PMUREG(0x30E8)
#define EXYNOS5410_PAD_RET_MMCA_OPTION			EXYNOS_PMUREG(0x3108)
#define EXYNOS5410_PAD_RET_MMCB_OPTION			EXYNOS_PMUREG(0x3128)
#define EXYNOS5410_PAD_RET_MMCC_OPTION			EXYNOS_PMUREG(0x3148)
#define EXYNOS5410_PAD_RET_HSI_OPTION			EXYNOS_PMUREG(0x3168)
#define EXYNOS5410_PAD_RET_SPI_OPTION			EXYNOS_PMUREG(0x31C8)

#define EXYNOS5410_MFC_CONFIGURATION			EXYNOS_PMUREG(0x4060)
#define EXYNOS5410_MFC_STATUS				EXYNOS_PMUREG(0x4064)
#define EXYNOS5410_MFC_OPTION				EXYNOS_PMUREG(0x4068)
#define EXYNOS5410_G3D_CONFIGURATION			EXYNOS_PMUREG(0x4080)
#define EXYNOS5410_G3D_STATUS				EXYNOS_PMUREG(0x4084)
#define EXYNOS5410_G3D_OPTION				EXYNOS_PMUREG(0x4088)
#define EXYNOS5410_DISP0_CONFIGURATION			EXYNOS_PMUREG(0x40A0)
#define EXYNOS5410_DISP0_STATUS				EXYNOS_PMUREG(0x40A4)
#define EXYNOS5410_DISP0_OPTION				EXYNOS_PMUREG(0x40A8)
#define EXYNOS5410_DISP1_CONFIGURATION			EXYNOS_PMUREG(0x40C0)
#define EXYNOS5410_DISP1_STATUS				EXYNOS_PMUREG(0x40C4)
#define EXYNOS5410_DISP1_OPTION				EXYNOS_PMUREG(0x40C8)
#define EXYNOS5410_MAU_CONFIGURATION			EXYNOS_PMUREG(0x40E0)
#define EXYNOS5410_MAU_STATUS				EXYNOS_PMUREG(0x40E4)
#define EXYNOS5410_MAU_OPTION				EXYNOS_PMUREG(0x40E8)
#define EXYNOS5410_I2S_DEBUG				EXYNOS_PMUREG(0x4D04)

/* For EXYNOS_CENTRAL_SEQ_OPTION */
#define EXYNOS5_USE_STANDBYWFI_ARM_CORE0		(1 << 16)
#define EXYNOS5_USE_STANDBYWFI_ARM_CORE1		(1 << 17)
#define EXYNOS5_USE_STANDBYWFE_ARM_CORE0		(1 << 24)
#define EXYNOS5_USE_STANDBYWFE_ARM_CORE1		(1 << 25)

#define EXYNOS5410_ARM_USE_STANDBY_WFI0		(1 << 4)
#define EXYNOS5410_ARM_USE_STANDBY_WFI1		(1 << 5)
#define EXYNOS5410_ARM_USE_STANDBY_WFI2		(1 << 6)
#define EXYNOS5410_ARM_USE_STANDBY_WFI3		(1 << 7)
#define EXYNOS5410_KFC_USE_STANDBY_WFI0		(1 << 8)
#define EXYNOS5410_KFC_USE_STANDBY_WFI1		(1 << 9)
#define EXYNOS5410_KFC_USE_STANDBY_WFI2		(1 << 10)
#define EXYNOS5410_KFC_USE_STANDBY_WFI3		(1 << 11)
#define EXYNOS5410_ARM_USE_STANDBY_WFE0		(1 << 16)
#define EXYNOS5410_ARM_USE_STANDBY_WFE1		(1 << 17)
#define EXYNOS5410_ARM_USE_STANDBY_WFE2		(1 << 18)
#define EXYNOS5410_ARM_USE_STANDBY_WFE3		(1 << 19)
#define EXYNOS5410_KFC_USE_STANDBY_WFE0		(1 << 20)
#define EXYNOS5410_KFC_USE_STANDBY_WFE1		(1 << 21)
#define EXYNOS5410_KFC_USE_STANDBY_WFE2		(1 << 22)
#define EXYNOS5410_KFC_USE_STANDBY_WFE3		(1 << 23)

#define EXYNOS5410_USE_STANDBY_WFI_ALL		(EXYNOS5410_ARM_USE_STANDBY_WFI0  \
						 | EXYNOS5410_ARM_USE_STANDBY_WFI1  \
						 | EXYNOS5410_ARM_USE_STANDBY_WFI2  \
						 | EXYNOS5410_ARM_USE_STANDBY_WFI3  \
						 | EXYNOS5410_KFC_USE_STANDBY_WFI0  \
						 | EXYNOS5410_KFC_USE_STANDBY_WFI1  \
						 | EXYNOS5410_KFC_USE_STANDBY_WFI2  \
						 | EXYNOS5410_KFC_USE_STANDBY_WFI3)

#define DUR_WAIT_RESET				(0xF)
#endif /* __ASM_ARCH_REGS_PMU_H */
