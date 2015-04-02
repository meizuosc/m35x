#ifndef __LINUX_ES305B_SOC_H_
#define __LINUX_ES305B_SOC_H_
/*-----------------API ---------------------*/

/* AFTER POWER UP OR SYSYTEM RESET*/
#define ES305B_MSG_BOOT					0x0001
#define ES305B_MSG_BOOT_ACK				0x0101

/* ack */
#define ES305B_NORMAL_ACK				0x80

/*wake up */
#define A200_msg_Wakeup
#define WAKEUP_WAITTIME_AFTER			30 // ms

/* sync */
#define A200_msg_Sync					0x8000
#define A200_msg_Sync_Polling			0x0000
#define A200_msg_Sync_Int_level_low		0x0001
#define A200_msg_Sync_Int_level_high	0x0002
#define A200_msg_Sync_Int_falling_edge	0x0003
#define A200_msg_Sync_Int_raising_edge	0x0004
#define SYNC_WAIT_TIME_BEFORE			10 // ms

/* S/W reset */
#define A200_msg_Reset					0x8002
#define RESET_IMMEDIATE					0x0000
#define RESET_DELAYED					0x0001

/* RE-DOWNLOAD FIRMWARE  */
#define A200_msg_BootloadInitiate		0x80030000

/* get/set device id and parameter
* set :
1. es305_msg(SetDeviceParmID, Parameter ID)
   Parameter ID : 8 bit device + 8 bit device parameter
2. read 4 byte ack.
3. es305_msg(SetDeviceParm, value)
4. read 4 byte ack.
* get:
1. es305_msg(GetDeviceParm, Parameter ID)
   Parameter ID : 8 bit device + 8 bit device parameter
*/
#define A200_msg_GetDeviceParm			0x800B
#define A200_msg_SetDeviceParmID		0x800C
#define A200_msg_SetDeviceParm			0x800D

/* power state */
#define A200_msg_SetPowerState_Sleep	0x80100001
#define STOP_CLOCK_WAITTIME_AFTER		120 // ms

/* get/set mic sample rate */
#define A200_msg_GetMicSampleRate		0x80500000
#define A200_msg_SetMicSampleRate		0x8051
#define MIC_8K_SAMPLE_RATE				0x0008
#define MIC_16K_SAMPLE_RATE				0x000A
#define MIC_48K_SAMPLE_RATE				0x0030

/* nb/wb switching */
#define A200_msg_GetAlgoSampleRate		0x804B0000
#define A200_msg_SetAlgoSampleRate		0x804C
#define ALGO_SAMPLE_RATE_NB_8K			0x0000
#define ALGO_SAMPLE_RATE_WB_16K			0x0001
#define A200_msg_GetChangeStatus		0x804F0000
#define A200_msg_GetSmoothRate			0x804D0000
#define A200_msg_SetSmoothRate			0x804E
#define SmoothRate_2000_DB_PER_SECOND	0x07D0
#define SmoothRate_2500_DB_PER_SECOND	0x09C4 // default
#define SmoothRate_3200_DB_PER_SECOND	0x0C80 // max

/* digital pass through */
#define A200_msg_DigitalPassThrough		0x8052

/* write data block */
#define A200_msg_WriteDataBlock			0x802F

/* get/set algorithm Parameter and Parameter ID */
#define A200_msg_GetAlgorithmParm		0x8016
#define A200_msg_SetAlgorithmParmID		0x8017
#define A200_msg_SetAlgorithmParm		0x8018

/* get algorithm statistic and statistic ID */
#define A200_msg_GetAlgorithmStatistics	0x8018

/* set presets */
#define A200_msg_SetPreset				0x8031

/* get/set digital input gain */
#define A200_msg_GetDigitalInputGain	0x801A
#define A200_msg_SetDigitalInputGain	0x801B

/* get/set digital output gain */
#define A200_msg_GetDigitalOutputGain	0x801D
#define A200_msg_SetDigitalOutputGain	0x8015

/* enable/disable voice processing */
#define A200_msg_VoiceProcessing		0x801C
#define A200_msg_GetVoiceProcessingMode	0x8043
#define VOICE_PROCESSING_ON				0x0001 // default
#define VOICE_PROCESSING_OFF			0x0000

/* get/set audio routing */
#define A200_msg_GetAudioRouting		0x8027
#define A200_msg_SetAudioRouting		0x8026

/* diagnostic api commands */
#define A200_msg_GetSigRMS				0x8013
#define A200_msg_GetSigPeek				0x8014
#define A200_msg_OutputKnownSig			0x801E


/* ------------- general definitions -------------*/
#define ES305B_I2C_NAME					"audience_es305b"
#define ES305B_I2S_SLAVE_ADDRESS 		(0x3E)
#define POLLING_TIMEOUT					20 // ms
#define RESET_TIMEOUT					50 // ms

// unconfirmed
#define ES305B_MAX_FW_SIZE				(32 * 4096)

#define TIMEOUT							20 /* ms */
#define RETRY_CNT						5
#define POLLING_RETRY_CNT				10
#define ES305B_ERROR_CODE				0xffff
#define ES305B_ACTIVE					1
#define ES305B_CMD_FIFO_DEPTH			128 /* 128 / 4 = 32 */
#define ERROR							0xffffffff

/* ---------------------Stucture -------------------*/

struct es305b_platform_data {
	uint32_t gpio_es305b_reset;
	uint32_t gpio_es305b_clk;
	uint32_t gpio_es305b_wake;

};

struct es305b_config_data {
	unsigned int data_len;
	unsigned int *cmd_data[];  /* [mode][cmd_len][cmds..] */
};

enum es305b_config_mode {
	ES305B_CONFIG_FULL,
	ES305B_CONFIG_VP
};

enum ES305B_NS_STATES {
	ES305B_NS_STATE_AUTO,
	ES305B_NS_STATE_OFF,
	ES305B_NS_STATE_CT,
	ES305B_NS_STATE_FT,
	ES305B_NS_NUM_STATES
};

enum ES305B_MODE {
	ES305B_SUSPEND,
	ES305B_INCALL_CT_NB,
	ES305B_INCALL_WHS_NB,
	ES305B_INCALL_DV_NB,
	ES305B_INCALL_BT,
	ES305B_VOIP_CT_NB,
	ES305B_VOIP_WHS_NB,
	ES305B_VOIP_DV_NB,
	ES305B_VOIP_BT,
	ES305B_BT_RING,
	ES305B_BYPASS_A2C,
	ES305B_BYPASS_C2A,
	ES305B_LASTMODE,
	ES305B_INVALID,
	ES305B_PATH_MAX
};

enum es305b_status {
	ES305B_NORMAL,
	ES305B_SLEEP,
	ES305B_RST,
};

struct es305bimg {
	unsigned char *buf;
	unsigned img_size;
};

enum es305b_reset {
	ES305B_SOFTWARE_RST,
	ES305B_COLD_RST,
	ES305B_HARDWARE_RST,
};

struct es305b_soc {
	struct device *dev;
	struct i2c_client *client;
	struct mutex es305b_mutex;
	int gpio_es305b_wake;
	int gpio_es305b_reset;
	enum ES305B_MODE mode;
	enum es305b_status  status;
	int nr_bt;

	struct regulator *crystal_regulator;
};

struct es305b_img {
	unsigned char *buf;
	unsigned img_size;
};

#define ES305B_IOCTL_MAGIC ';'

#define ES305B_BOOTUP_INIT _IOW(ES305B_IOCTL_MAGIC, 1, struct es305b_img *)
#define ES305B_SET_CONFIG _IOW(ES305B_IOCTL_MAGIC, 2, unsigned int *)
#define ES305B_SET_PARAM	   _IOW(ES305B_IOCTL_MAGIC, 4, struct es305b_config_data *)
#define ES305B_SYNC_CMD _IO(ES305B_IOCTL_MAGIC, 9)
#define ES305B_SLEEP_CMD _IO(ES305B_IOCTL_MAGIC, 11)
#define ES305B_HWRESET_CMD _IO(ES305B_IOCTL_MAGIC, 12)
// #define ES305B_COLDRESET_CMD _IO(ES305B_IOCTL_MAGIC, 13)
// #define ES305B_SWRESET_CMD _IO(ES305B_IOCTL_MAGIC, 14)
#define ES305B_WAKEUP_CMD _IO(ES305B_IOCTL_MAGIC, 13)
#define ES305B_MDELAY _IOW(ES305B_IOCTL_MAGIC, 14, unsigned int)
#define ES305B_READ_FAIL_COUNT _IOR(ES305B_IOCTL_MAGIC, 15, unsigned int *)
#define ES305B_READ_SYNC_DONE _IOR(ES305B_IOCTL_MAGIC, 16, bool *)
#define ES305B_WRITE_MSG		_IOW(ES305B_IOCTL_MAGIC, 17, unsigned)
#define ES305B_READ_DATA		_IOR(ES305B_IOCTL_MAGIC, 18, unsigned)

/* For Diag */
#define ES305B_SET_MIC_ONOFF	_IOW(ES305B_IOCTL_MAGIC, 0x50, unsigned)
#define ES305B_SET_MICSEL_ONOFF	_IOW(ES305B_IOCTL_MAGIC, 0x51, unsigned)
// #define ES305B_READ_DATA		_IOR(ES305B_IOCTL_MAGIC, 0x52, unsigned)
// #define ES305B_WRITE_MSG		_IOW(ES305B_IOCTL_MAGIC, 0x53, unsigned)
#define ES305B_SET_CMD_FILE	_IOW(ES305B_IOCTL_MAGIC, 0x54, unsigned)

// extern struct es305b_soc *es305b;
extern int es305b_setmode(int mode);

#endif	/* __LINUX_ES305B_SOC_H_ */
