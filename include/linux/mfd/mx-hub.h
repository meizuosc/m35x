/*
 * mx-sensor-hub.h - Driver for the sensor hub
 *
 * Copyright (C) 2013 Meizu Technology Co.Ltd, Zhuhai, China
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __LINUX_MFD_MX_SENSOR_HUB_H
#define __LINUX_MFD_MX_SENSOR_HUB_H

/*****************************************************************/

#include <linux/wakelock.h>
#include <linux/earlysuspend.h>

#define FW_VERSION     				0x01
#define FLASH_ADDR_FW_VERSION   	(1024*124 - 4)
#define MX_HUB_DEVICE_ID			'Z'

#define TWI_CMD_UPD			'U'
#define TWI_CMD_EXTBOOT		'X'
#define TWI_CMD_CRCCAL		'K'// TWI Command to get CRC
#define TWI_CMD_CRCCHECK	'C'// TWI Command to get CRC
#define TWI_CMD_BINFO		'I'// Get the bootloader information
#define TWI_CMD_STATUS		'S'// Get the bootloader status
#define TWI_CMD_EEPOM		'W'// read/write EEPOM


#define MCU_RUN_APP		'A'// mcu run into app
#define MCU_RUN_BTL		'B'// mcu run into bootloader



/* MX Sensor Hub  registers */
enum mx_hub_reg {
	MX_HUB_REG_DEVICE_ID		= 0x00,
	MX_HUB_REG_VERSION_A		= 0x01,
	MX_HUB_REG_VERSION_B		= 0x02,
	MX_HUB_REG_UPGRADE			= 0x03,
	MX_HUB_REG_CRC_LOW			= 0x04,
	MX_HUB_REG_CRC_HIGH		= 0x05,
	MX_HUB_REG_REV				= 0x06,
	MX_HUB_REG_INFO_B_A		= 0x07,

	MX_HUB_REG_STATUS			= 0x08,
	MX_HUB_REG_SOFTRESET 		= 0x09,
	
	MX_HUB_REG_IRQ_TYPE			= 0x0E,
	MX_HUB_REG_IRQ_MASK			= 0x0F,
	MX_HUB_REG_IRQ 				= 0x10,
	
	MX_HUB_REG_LED_BRN 			= 0x11,
	MX_HUB_REG_LED_SLP 			= 0x12,
	MX_HUB_REG_LED_FADE 		= 0x13,
	MX_HUB_REG_LED_SLPPRD 		= 0x14,// period L
	MX_HUB_REG_LED_SLPPRDH 		= 0x15,// period H
	MX_HUB_REG_LED_SLPTOP 		= 0x16,
	MX_HUB_REG_LED_FUNC_EN		= 0x17,
	
	MX_HUB_REG_CMP_INFO 			= 0x1A,
	MX_HUB_REG_ACC_INFO 			= 0x1B,
	MX_HUB_REG_GYR_INFO 			= 0x1C,	
	
	/* reg  for touch key */
	MX_HUB_REG_KEY_BASE 		= 0x20,
	MX_HUB_REG_WAKEUP_TYPE    	= (MX_HUB_REG_KEY_BASE + 0x00),
	MX_HUB_REG_WAKEUP_CNT    	= (MX_HUB_REG_KEY_BASE + 0x01),//0x21,
	MX_HUB_REG_WAKEUP_KEY    	= (MX_HUB_REG_KEY_BASE + 0x02),
	MX_HUB_REG_GESTURE_X_UNMASK	= (MX_HUB_REG_KEY_BASE + 0x03),
	MX_HUB_REG_GESTURE_Y_UNMASK	= (MX_HUB_REG_KEY_BASE + 0x04),
	MX_HUB_REG_GESTURE_TAP		= (MX_HUB_REG_KEY_BASE + 0x05),	// 12.5ms
	MX_HUB_REG_DOUBLE_TAP		= (MX_HUB_REG_KEY_BASE + 0x06),	// 12.5ms
	MX_HUB_REG_SINGLE_TAP		= (MX_HUB_REG_KEY_BASE + 0x07),	// 12.5ms
	MX_HUB_REG_LONGPRESS_TAP	= (MX_HUB_REG_KEY_BASE + 0x08),	// 12.5ms
	
	/* reg  for compass */
	MX_HUB_REG_COMPASS_BASE 	= 0x2F,	
	MX_HUB_REG_COMPASS_BASE_S 	= MX_HUB_REG_COMPASS_BASE,
	MX_HUB_REG_COMPASS_EN		= (MX_HUB_REG_COMPASS_BASE + 0x00),
	MX_HUB_REG_COMPASS_DLY0	= (MX_HUB_REG_COMPASS_BASE + 0x01),
	MX_HUB_REG_COMPASS_DLY1	= (MX_HUB_REG_COMPASS_BASE + 0x09),
	MX_HUB_REG_COMPASS_DLY2	= (MX_HUB_REG_COMPASS_BASE + 0x11),
	
	MX_HUB_REG_TWI_CMD_BINFO	= (MX_HUB_REG_COMPASS_BASE + 0x1A), // Only read
	
	MX_HUB_REG_COMPASS_ACC0	= (MX_HUB_REG_COMPASS_BASE + 0x1B),	
	MX_HUB_REG_COMPASS_ACC1	= (MX_HUB_REG_COMPASS_BASE + 0x1D),
	MX_HUB_REG_COMPASS_ACC2	= (MX_HUB_REG_COMPASS_BASE + 0x1F),
	MX_HUB_REG_COMPASS_DRDY	= (MX_HUB_REG_COMPASS_BASE + 0x21),
	MX_HUB_REG_COMPASS_RY		= (MX_HUB_REG_COMPASS_BASE + 0x23),
	MX_HUB_REG_COMPASS_RZ		= (MX_HUB_REG_COMPASS_BASE + 0x27),
	MX_HUB_REG_COMPASS_THROTTLE		= (MX_HUB_REG_COMPASS_BASE + 0x2B),
	MX_HUB_REG_COMPASS_RUDDER			= (MX_HUB_REG_COMPASS_BASE + 0x2F),
	
	/* reg for acc */
	MX_HUB_REG_ACC_BASE 				= 0x60,
	/*	CONTROL REGISTERS	*/
	MX_HUB_REG_ACC_WHO_AM_I 			= (MX_HUB_REG_ACC_BASE + 0x0F),	/*	WhoAmI register		*/
	MX_HUB_REG_ACC_TEMP_CFG_REG		= (MX_HUB_REG_ACC_BASE +0x1F),	/*	temper sens control reg	*/
	/* ctrl 1: ODR3 ODR2 ODR ODR0 LPen Zenable Yenable Zenable */
	MX_HUB_REG_ACC_CTRL_REG1 			= (MX_HUB_REG_ACC_BASE + 0x20),	/*	control reg 1		*/
	MX_HUB_REG_ACC_CTRL_REG2 			= (MX_HUB_REG_ACC_BASE + 0x21),	/*	control reg 2		*/
	MX_HUB_REG_ACC_CTRL_REG3 			= (MX_HUB_REG_ACC_BASE + 0x22),	/*	control reg 3		*/
	MX_HUB_REG_ACC_CTRL_REG4 			= (MX_HUB_REG_ACC_BASE + 0x23),	/*	control reg 4		*/
	MX_HUB_REG_ACC_CTRL_REG5 			= (MX_HUB_REG_ACC_BASE + 0x24),	/*	control reg 5		*/
	MX_HUB_REG_ACC_CTRL_REG6 			= (MX_HUB_REG_ACC_BASE + 0x25),	/*	control reg 6		*/
	MX_HUB_REG_ACC_STATUS_REG 			= (MX_HUB_REG_ACC_BASE + 0x27),
	MX_HUB_REG_ACC_AXISDATA 			= (MX_HUB_REG_ACC_BASE + 0x28),


	MX_HUB_REG_ACC_FIFO_CTRL_REG 		= (MX_HUB_REG_ACC_BASE + 0x2E),	/*	FiFo control reg	*/

	MX_HUB_REG_ACC_INT_CFG1 			= (MX_HUB_REG_ACC_BASE + 0x30),	/*	interrupt 1 config	*/
	MX_HUB_REG_ACC_INT_SRC1 			= (MX_HUB_REG_ACC_BASE + 0x31),	/*	interrupt 1 source	*/
	MX_HUB_REG_ACC_INT_THS1 			= (MX_HUB_REG_ACC_BASE + 0x32),	/*	interrupt 1 threshold	*/
	MX_HUB_REG_ACC_INT_DUR1 			= (MX_HUB_REG_ACC_BASE + 0x33),	/*	interrupt 1 duration	*/


	MX_HUB_REG_ACC_TT_CFG 				= (MX_HUB_REG_ACC_BASE + 0x38),	/*	tap config		*/
	MX_HUB_REG_ACC_TT_SRC 				= (MX_HUB_REG_ACC_BASE + 0x39),	/*	tap source		*/
	MX_HUB_REG_ACC_TT_THS 				= (MX_HUB_REG_ACC_BASE + 0x3A),	/*	tap threshold		*/
	MX_HUB_REG_ACC_TT_LIM 				= (MX_HUB_REG_ACC_BASE + 0x3B),	/*	tap time limit		*/
	MX_HUB_REG_ACC_TT_TLAT 				= (MX_HUB_REG_ACC_BASE + 0x3C),	/*	tap time latency	*/
	MX_HUB_REG_ACC_TT_TW 				= (MX_HUB_REG_ACC_BASE + 0x3D),	/*	tap time window		*/
	/*	end CONTROL REGISTRES	*/

	/* reg for gyr */
	MX_HUB_REG_GYR_BASE 				= 0xA0,
	/* lsm330dlc gyroscope registers */
	MX_HUB_REG_GYR_WHO_AM_I 			= (MX_HUB_REG_GYR_BASE+0x0F),

	MX_HUB_REG_GYR_CTRL_REG1 			= (MX_HUB_REG_GYR_BASE+0x20) ,   /* CTRL REG1 */
	MX_HUB_REG_GYR_CTRL_REG2 			= (MX_HUB_REG_GYR_BASE+0x21) ,   /* CTRL REG2 */
	MX_HUB_REG_GYR_CTRL_REG3 			= (MX_HUB_REG_GYR_BASE+0x22) ,   /* CTRL_REG3 */
	MX_HUB_REG_GYR_CTRL_REG4 			= (MX_HUB_REG_GYR_BASE+0x23) ,   /* CTRL_REG4 */
	MX_HUB_REG_GYR_CTRL_REG5 			= (MX_HUB_REG_GYR_BASE+0x24) ,   /* CTRL_REG5 */
	MX_HUB_REG_GYR_REFERENCE 			= (MX_HUB_REG_GYR_BASE+0x25) ,   /* REFERENCE REG */
	MX_HUB_REG_GYR_STATUS_REG 			= (MX_HUB_REG_GYR_BASE+0x27),
	MX_HUB_REG_GYR_FIFO_CTRL_REG 		= (MX_HUB_REG_GYR_BASE+0x2E) ,   /* FIFO CONTROL REGISTER */
	MX_HUB_REG_GYR_FIFO_SRC_REG 		= (MX_HUB_REG_GYR_BASE+0x2F) ,   /* FIFO SOURCE REGISTER */
	MX_HUB_REG_GYR_OUT_X_L	 			= (MX_HUB_REG_GYR_BASE+0x28) ,   /* 1st AXIS OUT REG of 6 */
	
	MX_HUB_REG_WRITE_DATA_RDY 			= 0xF0,
	MX_HUB_REG_READ_DATA_RDY 			= 0xF2,
	
	MX_HUB_REG_MAX,
};

/* IRQ definitions */
enum {
	MX_HUB_IRQ_KEY = 0,
	MX_HUB_IRQ_COMPASS,
	MX_HUB_IRQ_ACC,
	MX_HUB_IRQ_GYR,

	MX_HUB_IRQ_MAX,
};


/* key type definitions */
enum {
	MX_KEY_GESTURE = 0,
	MX_KEY_DOUBLE,
	MX_KEY_LONGPRESS,
	MX_KEY_SINGLE,
	MX_KEY_GESTURE2,
	
	MX_KEY_NONE,

	MX_KEY_TYPE_MAX,
};

/* LED function definitions */
enum {
	LED_FUNC_BRN_FADE_IN = (1<<0),
	LED_FUNC_BRN_FADE_OUT = (1<<1),
	
	LED_FUNC_SLOPE_EN = (1<<6),
	LED_FUNC_FADE_EN = (1<<7),
};


#define MX_HUB_IRQ_KEY_MASK		(1 << 0)
#define MX_HUB_IRQ_COMPASS_MASK		(1 << 1)
#define MX_HUB_IRQ_ACC_MASK			(1 << 2)
#define MX_HUB_IRQ_GYR_MASK			(1 << 3)

/*****************************************************************/
typedef union mx_hub_boot_intfo
{
    unsigned int int_32[2];
    unsigned char byte[8];
    struct
    {   
        unsigned char ID;  
        unsigned char AVer;
        unsigned char BVer;
        unsigned char bForceUpdate;    
        unsigned short CRC;
        unsigned char unused1;  
        unsigned char run_in_B_A;  
    };
}s_boot_intfo;

typedef struct _device_info
{
	unsigned char ver;
	unsigned char id;
	unsigned short crc;
}s_device_info;

struct mx_hub_dev {
	struct device *dev;
	struct i2c_client *client;
	struct regulator *regulator;
	struct mutex iolock;
	struct wake_lock wake_lock;
	struct mutex irqlock;
	struct class *hubclass;
	union mx_hub_boot_intfo binfo;
	s_device_info img_info;
	s_device_info dev_info;

	int irq_base;
	int irq;
	u8 irq_masks_cur;
	u8 irq_masks_cache;
	int type;
	bool wakeup;
	
	volatile int is_update;
	volatile bool hub_sleep;
	int key_wakeup_type;
	bool debug;

	unsigned int gpio_busy;
	unsigned int gpio_wake;
	unsigned int gpio_reset;
	unsigned int gpio_irq;
	
	unsigned int AVer;
	unsigned int BVer;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};


/* to set gpios numb connected to gyro interrupt pins,
 * the unused ones have to be set to -EINVAL
 */
#define LSM330DLC_GYR_DEFAULT_INT1_GPIO	(-EINVAL)
#define LSM330DLC_GYR_DEFAULT_INT2_GPIO	(-EINVAL)

/* Gyroscope Sensor Full Scale */
#define LSM330DLC_GYR_FS_250DPS		(0x00)
#define LSM330DLC_GYR_FS_500DPS		(0x10)
#define LSM330DLC_GYR_FS_2000DPS		(0x30)


/* acc section */
#define LSM330DLC_ACC_I2C_SADROOT		(0x0C)
/* I2C address if acc SA0 pin to GND */
#define LSM330DLC_ACC_I2C_SAD_L		((LSM330DLC_ACC_I2C_SADROOT<<1)| \
							LSM330DLC_SAD0L)
/* I2C address if acc SA0 pin to Vdd */
#define LSM330DLC_ACC_I2C_SAD_H		((LSM330DLC_ACC_I2C_SADROOT<<1)| \
							LSM330DLC_SAD0H)

/* to set gpios numb connected to gyro interrupt pins,
 * the unused ones havew to be set to -EINVAL
 */
#define LSM330DLC_ACC_DEFAULT_INT1_GPIO	(-EINVAL)
#define LSM330DLC_ACC_DEFAULT_INT2_GPIO	(-EINVAL)

/* Accelerometer Sensor Full Scale */
#define	LSM330DLC_ACC_FS_MASK		(0x30)
#define LSM330DLC_ACC_G_2G		(0x00)
#define LSM330DLC_ACC_G_4G		(0x10)
#define LSM330DLC_ACC_G_8G		(0x20)
#define LSM330DLC_ACC_G_16G		(0x30)


struct acc_platform_data {
	unsigned int poll_interval;
	unsigned int min_interval;

	u8 fs_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

	/* set gpio_int[1,2] either to the choosen gpio pin number or to -EINVAL
	 * if leaved unconnected
	 */
	int gpio_int1;
	int gpio_int2;
};

struct gyr_platform_data {
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

	unsigned int poll_interval;
	unsigned int min_interval;

	u8 fs_range;

	/* fifo related */
	u8 watermark;
	u8 fifomode;

	/* gpio ports for interrupt pads */
	int gpio_int1;
	int gpio_int2;		/* int for fifo */

	/* axis mapping */
	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;
};


struct mx_sensor_hub_platform_data {
	/* IRQ */
	int irq_base;
	bool wakeup;
	char *name;
	
	unsigned int gpio_busy;
	unsigned int gpio_wake;
	unsigned int gpio_reset;
	unsigned int gpio_irq;
	
	int nbuttons;
	int * keymap;
	
	char compass_layout;

	struct acc_platform_data *pacc;
	struct gyr_platform_data *pgyr;
};

/*****************************************************************/

int mx_hub_irq_init(struct mx_hub_dev *mx_hub);
void mx_hub_irq_exit(struct mx_hub_dev *mx_hub);
int mx_hub_irq_resume(struct mx_hub_dev *mx_hub);

extern int mx_hub_readbyte(struct i2c_client *client, u8 reg);
extern int mx_hub_writebyte(struct i2c_client *client, u8 reg, u8 data);
extern int mx_hub_readdata(struct i2c_client *client, u8 reg,int bytes,void *dest);
extern int mx_hub_writedata(struct i2c_client *client, u8 reg, int bytes, const void *src);

extern int mx_hub_readdata_rdy(struct i2c_client *client, u8 reg,int bytes,void *dest);
extern int mx_hub_writedata_rdy(struct i2c_client *client, u8 reg, int bytes, const void *src);
extern int mx_hub_waitforidle(struct mx_hub_dev *mx,int mtimeout);

extern int mx_hub_setkeytype(struct mx_hub_dev *hub,u8 type);



/*****************************************************************/

#endif	/* __LINUX_MFD_MX_SENSOR_HUB_H */
