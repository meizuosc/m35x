#ifndef __LINUX_TPS65132__
#define __LINUX_TPS65132__

#define TPS65132_NAME "tps-vol"

struct tps65132_pd_data {
	int vpos_val;
	int vneg_val;
	int app_val;
	int disp_val;
	int disn_val;
	int stored_en;
};

#define TPS65132_VPOS_REG  		0x00
#define TPS65132_VNEG_REG  		0x01
#define TPS65132_APP_DIS_REG 	0x02
#define TPS65132_CTRL_REG		0xff

#define TPS65132_APP_SMARTPHONE 0
#define TPS65132_APP_TABLET     1

#define TPS65132_DISP_NOCHG		0
#define TPS65132_DISP_ACTIVE	1
#define TPS65132_DISN_NOCHG		0
#define TPS65132_DISN_ACTIVE	1

#define TPS65132_VPOS_5V		0x0a
#define TPS65132_VPOS_5V5		0x0f
#define TPS65132_VNEG_5V		0x0a
#define TPS65132_VNEG_5V5		0x0f

#endif
