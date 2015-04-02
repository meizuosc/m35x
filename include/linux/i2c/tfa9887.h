#ifndef __LINUX_TFA9887_H_
#define __LINUX_TFA9887_H_

typedef unsigned char subaddress_t;
#define TFA98XX_REVISION       (subaddress_t)0x03
#define TFA98XX_I2S_CONTROL    (subaddress_t)0x04
#define TFA98XX_AUDIO_CONTROL  (subaddress_t)0x06
#define TFA98XX_SYSTEM_CONTROL (subaddress_t)0x09
#define TFA98XX_I2S_SEL        (subaddress_t)0x0A

enum tfa9887_status {
	TFA9887_NORMAL,
	TFA9887_SLEEP,
	TFA9887_RST,
};

enum tfa9887_mode {
	TFA9887_SUSPEND,
	TFA9887_INCALL_CT_NB,
	TFA9887_INCALL_WHS_NB,
	TFA9887_INCALL_DV_NB,
	TFA9887_INCALL_BT,
	TFA9887_VOIP_CT_NB,
	TFA9887_VOIP_WHS_NB,
	TFA9887_VOIP_DV_NB,
	TFA9887_VOIP_BT,
	TFA9887_BT_RING,
	TFA9887_BYPASS_A2C,
	TFA9887_LASTMODE,
	TFA9887_INVALID,
	TFA9887_PATH_MAX
};

struct tfa9887_pdata {
	struct device *dev;
	struct i2c_client *client;
	struct mutex tfa9887_mutex;
	enum tfa9887_mode mode;
	enum tfa9887_status  status;
};

struct tfa9887_write_read_img {
	unsigned char *write;
	unsigned char *read;
	size_t write_size;
	size_t read_size;
};

#define TFA9887_IOCTL_MAGIC '.'
#define TFA9887_BOOTUP_INIT _IOW(TFA9887_IOCTL_MAGIC, 1, struct es305b_img *)
#define TFA9887_SET_CONFIG _IOW(TFA9887_IOCTL_MAGIC, 2, unsigned int *)
#define TFA9887_I2C_WRITE_READ _IOW(TFA9887_IOCTL_MAGIC, 4, struct tfa9887_write_read_img*)
#define TFA9887_SYNC_CMD _IO(TFA9887_IOCTL_MAGIC, 9)
#define TFA9887_ID _IO(TFA9887_IOCTL_MAGIC, 11)
#define TFA9887_HWRESET_CMD _IO(TFA9887_IOCTL_MAGIC, 12)
#define TFA9887_COLDRESET_CMD _IO(TFA9887_IOCTL_MAGIC, 13)
#define TFA9887_SWRESET_CMD _IO(TFA9887_IOCTL_MAGIC, 14)
#define TFA9887_WAKEUP_CMD _IO(TFA9887_IOCTL_MAGIC, 15)
#define TFA9887_MDELAY _IOW(TFA9887_IOCTL_MAGIC, 16, unsigned int)
#define TFA9887_READ_FAIL_COUNT _IOR(TFA9887_IOCTL_MAGIC, 17, unsigned int *)
#define TFA9887_READ_SYNC_DONE _IOR(TFA9887_IOCTL_MAGIC, 18, bool *)


#endif	/* __LINUX_TFA9887_H_ */
