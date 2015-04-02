/*
 * Definitions for compass of mx sensor hub.
 */
#ifndef _MX_HUB_COMPASS_H_
#define _MX_HUB_COMPASS_H_

#include <linux/ioctl.h>

/* To avoid device dependency, convert to general name */
#define AKM_I2C_NAME			"akm8963"
#define AKM_MISCDEV_NAME		"akm8963_dev"
#define AKM_SYSCLS_NAME			"compass"
#define AKM_SYSDEV_NAME			"akm8963"


#define AKM_WIA_VALE				0x48
#define AKM_MEASURE_TIME_US		10000
#define AKM_DRDY_IS_HIGH(x)		((x) & 0x01)


#define ACC_DATA_FLAG		0
#define MAG_DATA_FLAG		1
#define FUSION_DATA_FLAG		2
#define AKM_NUM_SENSORS		3

#define ACC_DATA_READY		(1<<(ACC_DATA_FLAG))
#define MAG_DATA_READY		(1<<(MAG_DATA_FLAG))
#define FUSION_DATA_READY	(1<<(FUSION_DATA_FLAG))

#define AKMIO				0xA1


struct akm8963_platform_data {
	char layout;
	int gpio_DRDY;
	int gpio_RSTN;
};

#endif //_MX_HUB_COMPASS_H_

