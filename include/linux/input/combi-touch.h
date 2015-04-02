//[*]--------------------------------------------------------------------------------------------------[*]
//
//
// 
//  I2C Touchscreen driver (platform data struct)
//  2012.01.17
//  Copyright @ CRUCIALTEC. All Rights Reserved.
//
// 
//
//[*]--------------------------------------------------------------------------------------------------[*]
#ifndef __COMBI_TOUCH_H__
#define __COMBI_TOUCH_H__

//[*]--------------------------------------------------------------------------------------------------[*]
#ifdef CONFIG_HAS_EARLYSUSPEND
	#include <linux/earlysuspend.h>
#endif

//[*]--------------------------------------------------------------------------------------------------[*]
//
// System control register address
//
//[*]--------------------------------------------------------------------------------------------------[*]
#define	REG_CMD_RESET			0x0800		// 0x00000001 = reset
#define	REG_CMD_STOP_RUN		0x1000		// 0x00000001 = stop, 0x00000000 = run
#define	REG_CMD_CHARGER_ON_OFF		0x1800		// 0x00000001 = on, 0x00000000 = off
#define	REG_CMD_SLEEP_WAKEUP		0x2000		// 0x00000001 = sleep, 0x00000000 = wakeup
#define	REG_CMD_TOUCH_DETECT_ON_OFF	0x2400		// 0x00000001 = on, 0x00000000 = off
#define	REG_CMD_SLOW_INTERVAL_ON_OFF	0x2600		// 0x00000001 = on. 0x00000000 = off
#define	REG_CMD_RECALIBRATION		0x2800		// 0x00000001 = recalibration

//[*]--------------------------------------------------------------------------------------------------[*]
//
// Flash command register address
//
//[*]--------------------------------------------------------------------------------------------------[*]
#define	REG_CMD_FW_UPDATE_MODE		0x7000
#define	REG_CMD_FW_ERASE		0x6000
#define	REG_CMD_FW_WRITE		0x5000
#define	REG_CMD_FW_READ			0x4000

//[*]--------------------------------------------------------------------------------------------------[*]
//
// Data register address
//
//[*]--------------------------------------------------------------------------------------------------[*]
#define REGISTER_BASE			0xC000
	
//[*]--------------------------------------------------------------------------------------------------[*]
#define	REG_VID				(REGISTER_BASE + 0x0000)
#define	REG_PID				(REGISTER_BASE + 0x0004)
#define	REG_VERSION			(REGISTER_BASE + 0x0008)
#define	REG_RESOLUTION			(REGISTER_BASE + 0x0010)

//[*]--------------------------------------------------------------------------------------------------[*]
#define	TOUCH_USER_MODE			0x00
#define	TOUCH_BOOT_MODE			0x01

#define	AIM902_MAX_FW_SIZE		8192	      // 8 Kbytes
#define	AIM902_SECTOR_SIZE		1024	

#define	FW_1ST_END_ADDR			(0x400 -1)
#define	FW_2ND_END_ADDR			(0x800 -1)
#define	REG_SIZE_BYTES			4

//[*]--------------------------------------------------------------------------------------------------[*]
//[*]--------------------------------------------------------------------------------------------------[*]
//
// Touch status/data/extention register address
//
//[*]--------------------------------------------------------------------------------------------------[*]
#define	REG_TS_STATUS			0xFF00

typedef	struct	status_reg__t	{
	unsigned int	ts_cnt		:5;		// lsb
	unsigned int	reserved1	:3;
	unsigned int	button		:8;
	unsigned int	reserved2	:16;	        // msb
}	__attribute__ ((packed))	status_reg_t;

typedef	union	status_reg__u	{
	unsigned int		        uint;
	status_reg_t		        bits;
}	__attribute__ ((packed))	status_reg_u;

//[*]--------------------------------------------------------------------------------------------------[*]
#define	REG_TS_DATA_BASE		0xFF04
#define	REG_TS_DATA(x)			((x << 2) + REG_TS_DATA_BASE)

typedef	struct	data_reg__t	{
	unsigned int	x		:11;
	unsigned int	y		:11;
	unsigned int	id		:5;
	unsigned int	type		:2;		// 1 = press, 2 = move, 3 = release, 0 = unknown
	unsigned int	reserved	:3;
}	__attribute__ ((packed))	data_reg_t;

typedef	union	data_reg__u	{
	unsigned int	uint;
	data_reg_t		bits;
}	__attribute__ ((packed))	data_reg_u;

//[*]--------------------------------------------------------------------------------------------------[*]
#define	REG_TS_EXT_DATA_BASE	0xFF40
#define	REG_TS_EXT_DATA(x)		((x << 2) + REG_TS_EXT_DATA_BASE)

typedef	struct	ext_data_reg__t	{
	unsigned int	pressure	:8;
	unsigned int	area		:3;
	unsigned int	reserved	:5;
	unsigned int	delta_x		:8;
	unsigned int	delta_y		:8;
}	__attribute__ ((packed))	ext_data_reg_t;

typedef	union	ext_data_reg__u	{
	unsigned int		uint;
	ext_data_reg_t		bits;
}	__attribute__ ((packed))	ext_data_reg_u;

//[*]--------------------------------------------------------------------------------------------------[*]
//
// COMBI Control Function define
//
//[*]--------------------------------------------------------------------------------------------------[*]
//[*]--------------------------------------------------------------------------------------------------[*]
// Firmware update Control function
//[*]--------------------------------------------------------------------------------------------------[*]
extern			int	combi_input_open	(struct input_dev *input);
extern			int	combi_mode_change	(struct touch *ts, unsigned char mode);
extern			int	combi_flash_erase	(struct touch *ts);
extern			int	combi_flash_write	(struct touch *ts, unsigned char *data);
extern			int	combi_flash_verify	(struct touch *ts, unsigned char *data);
extern			int 	combi_flash_firmware	(struct device *dev, const char *fw_name);

//[*]--------------------------------------------------------------------------------------------------[*]
// I2C Control function
//[*]--------------------------------------------------------------------------------------------------[*]
extern			int 	combi_i2c_read		(struct i2c_client *client, unsigned char *cmd, unsigned int cmd_len, unsigned char *data, unsigned int len);
//extern	int 	combi_i2c_write		(struct i2c_client *client, unsigned char *cmd, unsigned int cmd_len, unsigned char *data, unsigned int len);
extern			int 	combi_i2c_write	        (struct touch *ts, unsigned char *data);


//[*]--------------------------------------------------------------------------------------------------[*]
// calibration function
//[*]--------------------------------------------------------------------------------------------------[*]
extern			int 	combi_calibration	(struct touch *ts);

//[*]--------------------------------------------------------------------------------------------------[*]
// Touch data processing function
//[*]--------------------------------------------------------------------------------------------------[*]
extern			void	combi_work		(struct touch *ts);
extern			void	combi_disable		(struct touch *ts);
extern			void	combi_enable		(struct touch *ts);
extern			int	combi_early_probe	(struct touch *ts);
extern			int	combi_probe		(struct touch *ts);
extern			int	combi_gpio_init		(struct touch *ts);


#ifdef	CONFIG_HAS_EARLYSUSPEND
extern			void	combi_suspend		(struct early_suspend *h);
extern			void	combi_resume		(struct early_suspend *h);
#endif
//[*]--------------------------------------------------------------------------------------------------[*]
#endif /* __COMBI_TOUCH_H__ */
//[*]--------------------------------------------------------------------------------------------------[*]
//[*]--------------------------------------------------------------------------------------------------[*]
