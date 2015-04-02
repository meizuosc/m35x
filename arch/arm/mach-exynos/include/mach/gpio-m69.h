/* linux/arch/arm/mach-exynos/include/mach/gpio-m69.h
 *
 * Copyright (C) 2013 Meizu Technology Co.Ltd, Zhuhai, China
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc.,
 *
 * Revision History
 *
 * Author: wangbo@meizu.com
 * Date:    2013-8-28
 *
 */

#ifndef _MEIZU_M69_GPIO_H_
#define _MEIZU_M69_GPIO_H_

#if defined(CONFIG_MEIZU_M69_V1)
#define M69_PMIC_EINT			IRQ_EINT(21)

static const unsigned short m69_gpio_v1_tbl[] = {
	[MEIZU_PMIC_IRQ_INDEX] =              EXYNOS5410_GPX2(5),
	[MEIZU_TOUCH_IRQ_INDEX] =             EXYNOS5410_GPX0(6),
	[MEIZU_KEYOFF_IRQ_INDEX] =            EXYNOS5410_GPX2(4),
	[MEIZU_IR_IRQ_INDEX] =                EXYNOS5410_GPX0(3),
	[MEIZU_VBUS_DET_IRQ_INDEX] =          EXYNOS5410_GPX0(5),
	[MEIZU_MCU_IRQ_INDEX] =               EXYNOS5410_GPX0(2),
	[MEIZU_PMIC_IRQ2_INDEX] =             EXYNOS5410_GPX1(2),
	[MEIZU_USB_ID_IRQ_INDEX] =            EXYNOS5410_GPX0(1),
	[MEIZU_VOLUP_IRQ_INDEX] =             EXYNOS5410_GPX3(5),
	[MEIZU_WIFI_IRQ_INDEX] =              EXYNOS5410_GPX2(0),
	[MEIZU_FUELGAGE_IRQ_INDEX] =          EXYNOS5410_GPX2(2),
	[MEIZU_USB_IRQ_INDEX] =               EXYNOS5410_GPX0(0),
	[MEIZU_HOME_IRQ_INDEX] =              EXYNOS5410_GPX2(1),
	[MEIZU_MIC_IRQ_INDEX] =               EXYNOS5410_GPX3(0),
	[MEIZU_PMIC_RST_IRQ_INDEX] =          EXYNOS5410_GPX3(1),
	[MEIZU_BT_HOSTWAKE_IRQ_INDEX] =       EXYNOS5410_GPX3(2),
	[MEIZU_NFC_IRQ_INDEX] =               EXYNOS5410_GPX3(4),
	[MEIZU_VOLDOWN_IRQ_INDEX] =           EXYNOS5410_GPX1(7),
	[MEIZU_HDETEC_IRQ_INDEX] =            EXYNOS5410_GPX3(6),
	[MEIZU_CODEC_IRQ_INDEX] =             EXYNOS5410_GPX3(7),
	[MEIZU_LED_ID1_INDEX] =               0,
	[MEIZU_LED_ID2_INDEX] =               0,
	[MEIZU_LED_ID3_INDEX] =               0,
	[MEIZU_LED_ID4_INDEX] =               0,
	[MEIZU_LED_ID5_INDEX] =               0,
	[MEIZU_LED_ID6_INDEX] =               0,
	[MEIZU_BT_RXD_INDEX] =                EXYNOS5410_GPA0(0),
	[MEIZU_BT_TXD_INDEX] =                EXYNOS5410_GPA0(1),
	[MEIZU_BT_CTS_INDEX] =                EXYNOS5410_GPA0(2),
	[MEIZU_BT_RTS_INDEX] =                EXYNOS5410_GPA0(3),
	[MEIZU_BT_REGON_INDEX] =              EXYNOS5410_GPF1(0),
	[MEIZU_BT_HOST_WAKE_INDEX] =          EXYNOS5410_GPX3(2),
	[MEIZU_BT_WAKE_INDEX] =               EXYNOS5410_GPG1(3),
	[MEIZU_WL_REGON_INDEX] =              EXYNOS5410_GPE0(5),
	[MEIZU_CORE_SEL_INDEX] =              0,
	[MEIZU_CHG_SDA_INDEX] =               EXYNOS5410_GPY7(5),
	[MEIZU_CHG_SCL_INDEX] =               EXYNOS5410_GPY7(6),
	[MEIZU_IR_PWEN_INDEX] =               EXYNOS5410_GPY6(3),
	[MEIZU_IR_SDA_INDEX] =                EXYNOS5410_GPG0(4),
	[MEIZU_IR_SCL_INDEX] =                EXYNOS5410_GPG1(2),
	[MEIZU_AKM_SDA_INDEX] =               0,
	[MEIZU_AKM_SCL_INDEX] =               0,
	[MEIZU_MCU_SDA_INDEX] =               EXYNOS5410_GPA1(2),
	[MEIZU_MCU_SCL_INDEX] =               EXYNOS5410_GPA1(3),
	[MEIZU_MCU_SDA_O_INDEX] =             EXYNOS5410_GPA1(2),
	[MEIZU_MCU_SCL_O_INDEX] =             EXYNOS5410_GPA1(3),
	[MEIZU_MCU_SLEEP_INDEX] =             EXYNOS5410_GPY5(3),
	[MEIZU_MCU_RST_INDEX] =               EXYNOS5410_GPY2(4),
	[MEIZU_MCU_BUSY_INDEX] =              EXYNOS5410_GPY1(2),
	[MEIZU_VER_PIN0_INDEX] =              EXYNOS5410_GPJ2(0),
	[MEIZU_VER_PIN1_INDEX] =              EXYNOS5410_GPJ1(1),
	[MEIZU_VER_PIN2_INDEX] =              EXYNOS5410_GPJ1(4),
	[MEIZU_NFC_WAKE_INDEX] =              EXYNOS5410_GPE0(7),
	[MEIZU_NFC_REG_PU_INDEX] =            EXYNOS5410_GPE0(0),
	[MEIZU_NFC_VSIM_REQ_INDEX] =          EXYNOS5410_GPX3(3),
	[MEIZU_NFC_REQ_INDEX] =               EXYNOS5410_GPX3(4),
	[MEIZU_NFC_SCL_INDEX] =               EXYNOS5410_GPF1(7),
	[MEIZU_NFC_SDA_INDEX] =               EXYNOS5410_GPG0(1),
	[MEIZU_GPS_PWR_ON_INDEX] =            EXYNOS5410_GPG1(1),
	[MEIZU_GPS_CLK_REQ_INDEX] =           EXYNOS5410_GPF1(6),
	[MEIZU_GPS_RXD_INDEX] =               EXYNOS5410_GPA0(4),
	[MEIZU_GPS_TXD_INDEX] =               EXYNOS5410_GPA0(5),
	[MEIZU_GPS_CTS_INDEX] =               EXYNOS5410_GPA0(6),
	[MEIZU_GPS_RTS_INDEX] =               EXYNOS5410_GPA0(7),
	[MEIZU_FUEL_SDA_INDEX] =              EXYNOS5410_GPY7(7),
	[MEIZU_FUEL_SCL_INDEX] =              EXYNOS5410_GPY7(1),
	[MEIZU_GY_DEN_INDEX] =                0,
	[MEIZU_G_INT1_INDEX] =                0,
	[MEIZU_G_INT2_INDEX] =                0,
	[MEIZU_A_INT1_INDEX] =                0,
	[MEIZU_A_INT2_INDEX] =                0,
	[MEIZU_MOTOR_PWM_INDEX] =             EXYNOS5410_GPB2(0),
	[MEIZU_TOUCH_RST_INDEX] =             EXYNOS5410_GPY1(0),
	[MEIZU_TOUCH_SDA_INDEX] =             EXYNOS5410_GPB3(2),
	[MEIZU_TOUCH_SCL_INDEX] =             EXYNOS5410_GPB3(3),
	[MEIZU_ISP_VDD12_8MSEN_INDEX] =      0 /*EXYNOS5410_GPX2(5)*/,
	[MEIZU_ISP_RST_INDEX] =               EXYNOS5410_GPH1(4),
	[MEIZU_ISP_XCLK_IN_INDEX] =           EXYNOS5410_GPM7(5),
	[MEIZU_ISP_IRQ_INDEX] =               EXYNOS5410_GPH0(0),
	[MEIZU_ISP_I2CSDA_INDEX] =            EXYNOS5410_GPB3(0),
	[MEIZU_ISP_I2CSCL_INDEX] =            EXYNOS5410_GPB3(1),
	[MEIZU_ISP_SPI_CLK_INDEX] =           EXYNOS5410_GPA2(0),
	[MEIZU_ISP_SPI_MOSI_INDEX] =          EXYNOS5410_GPA2(3),
	[MEIZU_ISP_SPI_CS_INDEX] =            EXYNOS5410_GPA2(1),
	[MEIZU_CODEC_AUEN_INDEX] =            EXYNOS5410_GPE0(6),
	[MEIZU_CODEC_RST_INDEX] =             EXYNOS5410_GPF1(4),
	[MEIZU_CODEC_SDA_INDEX] =             EXYNOS5410_GPE0(1),
	[MEIZU_CODEC_SCL_INDEX] =             EXYNOS5410_GPF0(2),
	[MEIZU_NOISE_CANCELLER_WAKE_INDEX] =  EXYNOS5410_GPF0(0),
	[MEIZU_NOISE_CANCELLER_RST_INDEX] =   EXYNOS5410_GPG0(7),
	[MEIZU_PA_SDA_INDEX] =                EXYNOS5410_GPF0(1),
	[MEIZU_PA_SCL_INDEX] =                EXYNOS5410_GPF1(1),
	[MEIZU_ES305B_SDA_INDEX] =            EXYNOS5410_GPE0(1),
	[MEIZU_ES305B_SCL_INDEX] =            EXYNOS5410_GPF0(2),
	[MEIZU_LCD_5VEN_INDEX] =              EXYNOS5410_GPJ2(2),
	[MEIZU_LCD_N5VEN_INDEX] =             EXYNOS5410_GPJ2(0),
	[MEIZU_LCD_BIAS_SDA_INDEX] =          EXYNOS5410_GPJ0(3),
	[MEIZU_LCD_BIAS_SCL_INDEX] =          EXYNOS5410_GPJ1(5),
	[MEIZU_LCD_BL_SDA_INDEX] =            EXYNOS5410_GPJ2(5),
	[MEIZU_LCD_BL_SCL_INDEX] =            EXYNOS5410_GPJ1(1),
	[MEIZU_LCD_BL_EN_INDEX] =             EXYNOS5410_GPJ3(4),
	[MEIZU_LCD_ID_INDEX] =                EXYNOS5410_GPJ1(4),
	[MEIZU_LCD_RST_INDEX] =               EXYNOS5410_GPJ1(2),
	[MEIZU_LCD_TE_INDEX] =                EXYNOS5410_GPJ1(0),
	[MEIZU_LCD_HW_TE_INDEX] =             EXYNOS5410_GPJ4(0),
	[MEIZU_TF_CARD_EN_INDEX] =            EXYNOS5410_GPY3(7),
	[MEIZU_EEPROM_WP_INDEX] =             0,
	[MEIZU_EEPROM_SCL_INDEX] =            0,
	[MEIZU_EEPROM_SDA_INDEX] =            0,
	[MEIZU_SENSOR_SCL_INDEX] =            EXYNOS5410_GPA1(3),
	[MEIZU_SENSOR_SDA_INDEX] =            EXYNOS5410_GPA1(2),
	[MEIZU_GPIO_FACTORY_MODE_INDEX] =     EXYNOS5410_GPJ1(7),
	[MEIZU_GPIO_TEST_LED_INDEX] =         EXYNOS5410_GPJ0(2),
	[MEIZU_GPIO_MODEM_POWER_ON_INDEX] =   EXYNOS5410_GPY6(0),
	[MEIZU_GPIO_MODEM_RDY_INDEX] =        EXYNOS5410_GPX0(4),
	[MEIZU_GPIO_AP_RDY_INDEX] =           EXYNOS5410_GPY5(7),
	[MEIZU_GPIO_MODEM_RTS_INDEX] =        EXYNOS5410_GPX1(4),
	[MEIZU_GPIO_AP_RTS_INDEX] =           EXYNOS5410_GPY0(2),
	[MEIZU_GPIO_MODEM_RESEND_INDEX] =     EXYNOS5410_GPY0(0),
	[MEIZU_GPIO_AP_RESEND_INDEX] =        EXYNOS5410_GPY0(3),
	[MEIZU_GPIO_MODEM_ALIVE_INDEX] =      EXYNOS5410_GPX0(7),
	[MEIZU_GPIO_MODEM_TO_AP1_INDEX] =     EXYNOS5410_GPY6(6),
	[MEIZU_GPIO_MODEM_TO_AP2_INDEX] =     EXYNOS5410_GPX1(1),
	[MEIZU_GPIO_AP_TO_MODEM1_INDEX] =     EXYNOS5410_GPY2(2),
	[MEIZU_GPIO_AP_TO_MODEM2_INDEX] =     EXYNOS5410_GPY6(7),
	[MEIZU_BOARD_ID1_INDEX] =             EXYNOS5410_GPJ0(1),
	[MEIZU_BOARD_ID2_INDEX] =             EXYNOS5410_GPJ0(0),
	[MEIZU_BOARD_ID3_INDEX] =             EXYNOS5410_GPK0(0),

};
#else
#error "Please Configure The M69 Version."
#endif /* CONFIG_MEIZU_M69_V1 */

#endif /* _MEIZU_M69_GPIO_H_ */
