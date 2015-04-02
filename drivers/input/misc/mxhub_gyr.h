
/*
 * Definitions for Gyroscope of mx sensor hub.
 */
#ifndef _MX_HUB_GYR_H_
#define _MX_HUB_GYR_H_



#define LSM330DLC_GYR_DEV_NAME		"lsm330dlc_gyr"

#define GYR_SYSCLS_NAME			"gyroscope"
#define GYR_SYSDEV_NAME			"gyr"

#define LSM330DLC_GYR_ENABLED	1
#define LSM330DLC_GYR_DISABLED	0
#define LSM330DLC_GYR_MIN_POLL_PERIOD_MS	2

#define FS_MAX		32768


#define FUZZ			0
#define FLAT			0
#define I2C_AUTO_INCREMENT	(0x80)


#endif //_MX_HUB_GYR_H_

