/*
 *  max17047.h
 *
 *  Copyright (C) 2012 Maxim Integrated Product
 *  Gyungoh Yoo <jack.yoo@maxim-ic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MAX17047_BATTERY_H_
#define __MAX17047_BATTERY_H_

#define MAX17047_REG_STATUS		0x00
#define MAX17047_REG_VALRT_TH		0x01
#define MAX17047_REG_TALRT_TH		0x02
#define MAX17047_REG_SOCALRT_TH		0x03
#define MAX17047_REG_ATRATE		0x04
#define MAX17047_REG_REMCAPREP		0x05
#define MAX17047_REG_SOCREP		0x06
#define MAX17047_REG_AGE		0x07
#define MAX17047_REG_TEMPERATURE	0x08
#define MAX17047_REG_VCELL		0x09
#define MAX17047_REG_CURRENT		0x0A
#define MAX17047_REG_AVERAGECURRENT	0x0B
#define MAX17047_REG_SOCMIX		0x0D
#define MAX17047_REG_SOCAV		0x0E
#define MAX17047_REG_REMCAPMIX		0x0F
#define MAX17047_REG_FULLCAP		0x10
#define MAX17047_REG_TTE		0x11
#define MAX17047_REG_VEMPTY		0x12
#define MAX17047_REG_FULLSOCTHR		0x13
#define MAX17047_REG_RCELL		0x14
#define MAX17047_REG_AERAGETEMPERATURE	0x16
#define MAX17047_REG_CYCLES		0x17
#define MAX17047_REG_DESIGNCAP		0x18
#define MAX17047_REG_AVERAGEVCELL	0x19
#define MAX17047_REG_MAXMINTEMPERATURE	0x1A
#define MAX17047_REG_MAXMINVCELL	0x1B
#define MAX17047_REG_MAXMINCURRENT	0x1C
#define MAX17047_REG_CONFIG		0x1D
#define MAX17047_REG_ICHGTERM		0x1E
#define MAX17047_REG_REMCAPAV		0x1F
#define MAX17047_REG_VERSION		0x21
#define MAX17047_REG_QRESIDUAL10	0x22
#define MAX17047_REG_FULLCAPNOM		0x23
#define MAX17047_REG_TEMPNOM		0x24
#define MAX17047_REG_TEMPLIM		0x25
#define MAX17047_REG_AIN		0x27
#define MAX17047_REG_LEARNCFG		0x28
#define MAX17047_REG_FILTERCFG		0x29
#define MAX17047_REG_RELAXCFG		0x2A
#define MAX17047_REG_MISCCFG		0x2B
#define MAX17047_REG_TGAIN		0x2C
#define MAX17047_REG_TOFF		0x2D
#define MAX17047_REG_CGAIN		0x2E
#define MAX17047_REG_COFF		0x2F
#define MAX17047_REG_QRESIDUAL20	0x32
#define MAX17047_REG_IAVG_EMPTY		0x36
#define MAX17047_REG_FCTC		0x37
#define MAX17047_REG_RCOMP0		0x38
#define MAX17047_REG_TEMPCO		0x39
#define MAX17047_REG_V_EMPTY		0x3A
#define MAX17047_REG_FSTAT		0x3D
#define MAX17047_REG_TIMER		0x3E
#define MAX17047_REG_SHDNTIMER		0x3F
#define MAX17047_REG_QRESIDUAL30	0x42
#define MAX17047_REG_DQACC		0x45
#define MAX17047_REG_DPACC		0x46
#define MAX17047_REG_QH			0x4D
#define MAX17047_REG_CHAR_TBL		0x80
#define MAX17047_REG_VFOCV		0xFB
#define MAX17047_REG_SOCVF		0xFF

/* MAX17047_REG_STATUS */
#define MAX17047_R_POR			0x0002
#define MAX17047_R_BST			0x0008
#define MAX17047_R_VMN			0x0100
#define MAX17047_R_TMN			0x0200
#define MAX17047_R_SMN			0x0400
#define MAX17047_R_BI			0x0800
#define MAX17047_R_VMX			0x1000
#define MAX17047_R_TMX			0x2000
#define MAX17047_R_SMX			0x4000
#define MAX17047_R_BR			0x8000

/* MAX17047_REG_CONFIG */
#define MAX17047_R_BER			0x0001
#define MAX17047_R_BEI			0x0002
#define MAX17047_R_AEN			0x0004
#define MAX17047_R_FTHRM		0x0008
#define MAX17047_R_ETHRM		0x0010
#define MAX17047_R_ALSH			0x0020
#define MAX17047_R_I2CSH		0x0040
#define MAX17047_R_SHDN			0x0080
#define MAX17047_R_TEX			0x0100
#define MAX17047_R_TEN			0x0200
#define MAX17047_R_AINSH		0x0400
#define MAX17047_R_ALRTP		0x0800
#define MAX17047_R_VS			0x1000
#define MAX17047_R_TS			0x2000
#define MAX17047_R_SS			0x4000

#define MAX17047_NAME		"fuelgauge"
#define RETRY_CNT	5
#undef DEBUG

extern int write_battery_model(int len, void *buf);
extern int read_battery_model(int len, void *buf);

struct max17047_reg_data {
	u8 reg;
};

struct max17047_platform_data {
	int r_sns;		/* sensing resistor, in mOhm */
	bool current_sensing;
};

#endif
