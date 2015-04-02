#include <linux/module.h>
#include <linux/i2c.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/ms2r.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#ifdef CONFIG_VIDEO_SAMSUNG_V4L2
#include <linux/videodev2_samsung.h>
#endif
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h> 

#include <linux/delay.h>

#include "ms2r.h"
#include "ms2r_regs.h"
#include "ms2r_ctrl.h"

#include <linux/firmware.h>

#define DEFAULT_DEBUG 0

#define DEFAULT_CAPTURE_WIDTH 3264
#define DEFAULT_CAPTURE_HEIGHT 2448

/* preview size regs */
static struct ms2r_reg ms2r_prev_vga[] = {
	{I2C_8BIT, MON_SIZE_REG, MON_VGA},
};
static struct ms2r_reg ms2r_prev_854x640[] = { 
	{I2C_8BIT, MON_SIZE_REG, MON_854X640},
};
static struct ms2r_reg ms2r_prev_960x720[] = {
	{I2C_8BIT, MON_SIZE_REG, MON_960X720},
};
static struct ms2r_reg ms2r_prev_720p[] = {
	{I2C_8BIT, MON_SIZE_REG, MON_720P},
};
static struct ms2r_reg ms2r_prev_1216x912[] = {
	{I2C_8BIT, MON_SIZE_REG, MON_1216X912},
};
static struct ms2r_reg ms2r_prev_1080p[] = {
	{I2C_8BIT, MON_SIZE_REG, MON_1080P},
};
static struct ms2r_reg ms2r_prev_3264x2448[] = {
	{I2C_8BIT, MON_SIZE_REG, MON_3264X2448},
};
static struct ms2r_reg ms2r_pre_1600x1200[] = {
	{I2C_8BIT, MON_SIZE_REG, MON_1600X1200},
};
static struct ms2r_reg ms2r_pre_1280x960[] = {
	{I2C_8BIT, MON_SIZE_REG, MON_1280X960},
};

/* capture format regs */
static struct ms2r_reg ms2r_fmt_yuv422[] = {
	{I2C_8BIT, YUVOUT_MAIN_REG, MAIN_OUTPUT_YUV422},
};
static struct ms2r_reg ms2r_fmt_jpeg[] = {
	{I2C_8BIT, YUVOUT_MAIN_REG, MAIN_OUTPUT_JPEG_422},
};
static struct ms2r_reg ms2r_fmt_raw10pack[] = {
	{I2C_8BIT, YUVOUT_MAIN_REG, MAIN_RAW10_PACK},
};

/* capture size regs */
static struct ms2r_reg ms2r_cap_vga[] = {
	{I2C_8BIT, MAIN_IMAGE_SIZE_REG, MAIN_SIZE_640_480},
};
static struct ms2r_reg ms2r_cap_720p[] = {
	{I2C_8BIT, MAIN_IMAGE_SIZE_REG, MAIN_SIZE_720p},
};
static struct ms2r_reg ms2r_cap_1m[] = {
	{I2C_8BIT, MAIN_IMAGE_SIZE_REG, MAIN_SIZE_1280_960},
};
static struct ms2r_reg ms2r_cap_2m[] = {
	{I2C_8BIT, MAIN_IMAGE_SIZE_REG, MAIN_SIZE_1600_1200},
};
static struct ms2r_reg ms2r_cap_3m[] = {
	{I2C_8BIT, MAIN_IMAGE_SIZE_REG, MAIN_SIZE_2048_1536},
};
static struct ms2r_reg ms2r_cap_5m[] = {
	{I2C_8BIT, MAIN_IMAGE_SIZE_REG, MAIN_SIZE_2560_1920},
};
static struct ms2r_reg ms2r_cap_8m[] = {
	{I2C_8BIT, MAIN_IMAGE_SIZE_REG, MAIN_SIZE_3264_2448},
};

/* preview size structs */
static struct ms2r_size_struct ms2r_prev_sizes[] = {
	{
		.width = 3264,
		.height = 2448,
		.regs = ms2r_prev_3264x2448,
		.size = ARRAY_SIZE(ms2r_prev_3264x2448),
	},
	{  /* 1080P */
		.width		= 1920,
		.height		= 1080,
		.regs 		= ms2r_prev_1080p,
		.size			= ARRAY_SIZE(ms2r_prev_1080p),
	},
	{
		.width = 1600,
		.height = 1200,
		.regs = ms2r_pre_1600x1200,
		.size = ARRAY_SIZE(ms2r_pre_1600x1200),
	},
	{
		.width = 1280,
		.height = 960,
		.regs = ms2r_pre_1280x960,
		.size = ARRAY_SIZE(ms2r_pre_1280x960),
	},
	{  /* 1216 * 912 */
		.width		= 1216,
		.height		= 912,
		.regs 		= ms2r_prev_1216x912,
		.size			= ARRAY_SIZE(ms2r_prev_1216x912),
	},
	{  /* 720P */
		.width		= 1280,
		.height		= 720,
		.regs 		= ms2r_prev_720p,
		.size			= ARRAY_SIZE(ms2r_prev_720p),
	},
	{  /* 960 * 720*/
		.width		= 960,
		.height		= 720,
		.regs 		= ms2r_prev_960x720,
		.size			= ARRAY_SIZE(ms2r_prev_960x720),
	},
	{ /* 854 * 640 */
		.width		= 854,
		.height		= 640,
		.regs 		= ms2r_prev_854x640,
		.size			= ARRAY_SIZE(ms2r_prev_854x640),
	},	
	{ /* VGA */
		.width		= 640,
		.height		= 480,
		.regs 		= ms2r_prev_vga,
		.size			= ARRAY_SIZE(ms2r_prev_vga),
	},	
};

/* capture format structs */
static struct ms2r_format_struct ms2r_cap_formats[] = {
	{
		.desc = "YUYV 4:2:2",
		.pixelformat = V4L2_PIX_FMT_YUYV,
		.mbus_code = V4L2_MBUS_FMT_VYUY8_2X8,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.bpp = 2,
		.regs = ms2r_fmt_yuv422,
		.size = ARRAY_SIZE(ms2r_fmt_yuv422),
	},
	{
		.desc = "JPEG encoded data",
		.pixelformat = V4L2_PIX_FMT_JPEG,
		.mbus_code = V4L2_MBUS_FMT_JPEG_1X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.bpp = 0,
		.regs = ms2r_fmt_jpeg,
		.size = ARRAY_SIZE(ms2r_fmt_jpeg),
	},
	{
		.desc = "raw10 pack",
		.pixelformat = V4L2_PIX_FMT_SGRBG10,
		.mbus_code = V4L2_MBUS_FMT_SGRBG10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.bpp = 0,
		.regs = ms2r_fmt_raw10pack,
		.size = ARRAY_SIZE(ms2r_fmt_raw10pack),
	},
};

/* capture size structs */
static struct ms2r_size_struct ms2r_cap_sizes[] = {
	{  /* 8M */
		.width		= 3264,
		.height		= 2448,
		.regs 		= ms2r_cap_8m,
		.size			= ARRAY_SIZE(ms2r_cap_8m),
	},	
	{  /* 5M */
		.width		= 2560,
		.height		= 1920,
		.regs 		= ms2r_cap_5m,
		.size			= ARRAY_SIZE(ms2r_cap_5m),
	},
	{  /* 3M */
		.width		= 2048,
		.height		= 1536,
		.regs 		= ms2r_cap_3m,
		.size			= ARRAY_SIZE(ms2r_cap_3m),
	},	
	{  /* 2M */
		.width		= 1600,
		.height		= 1200,
		.regs 		= ms2r_cap_2m,
		.size			= ARRAY_SIZE(ms2r_cap_2m),
	},
	{  /* 1M */
		.width		= 1280,
		.height		= 960,
		.regs 		= ms2r_cap_1m,
		.size			= ARRAY_SIZE(ms2r_cap_1m),
	},
	{  /* 720P */
		.width		= 1280,
		.height		= 720,
		.regs 		= ms2r_cap_720p,
		.size			= ARRAY_SIZE(ms2r_cap_720p),
	},
	{  /* VGA */
		.width		= 640,
		.height		= 480,
		.regs 		= ms2r_cap_vga,
		.size			= ARRAY_SIZE(ms2r_cap_vga),
	},
};

static u32 ms2r_swap_byte(u8 *data, enum ms2r_i2c_size size)
{
	if (size == I2C_8BIT)
		return *data;
	else if (size == I2C_16BIT)
		return be16_to_cpu(*((u16 *)data));
	else 
		return be32_to_cpu(*((u32 *)data));
}

/*
  * I2C read reg function
  * size: read size, 8 bit
  * addr: 16bit address, higher byte is category, lower byte cmd
  * val: store for read value
*/
int ms2r_read_reg(struct v4l2_subdev *sd,
		u16 addr, u32 *val,
		enum ms2r_i2c_size size)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ms2r_state *state = to_state(sd);
	struct i2c_msg msg[2];
	u8 wbuf[5], rbuf[I2C_MAX + 1];
	u8 category = (addr >> 8) & 0xff;
	u8 cmd = addr & 0xff;
	int ret;

	if (!client->adapter)
		return -ENODEV;

	if (size != I2C_8BIT && size != I2C_16BIT && size != I2C_32BIT)
		return -EINVAL;

	/* 1st I2C operation for writing category & command. */
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 5;		/* 1(cmd size per bytes) + 4 */
	msg[0].buf = wbuf;
	wbuf[0] = 5;		/* same right above this */
	wbuf[1] = CMD_READ_PARAMETER;
	wbuf[2] = category;
	wbuf[3] = cmd;
	wbuf[4] = size;

	/* 2nd I2C operation for reading data. */
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = size + 1;
	msg[1].buf = rbuf;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s(), failed READ[%d] at "
				"cat[%02x] cmd[%02x]\n", __func__,
				size, category, cmd);
		return ret;
	}

	*val = ms2r_swap_byte(&rbuf[1], size);

	if (state->debug) 
		pr_info("read: 0x%04x = 0x%x\n", addr, *val);

	return 0;
}

/*
  * I2C write reg function
  * size: write size, 8 bit
  * addr: 16bit address, higher byte is category, lower byte cmd
  * val: write value
*/
int ms2r_write_reg(struct v4l2_subdev *sd,
		u16 addr, u32 val,
		enum ms2r_i2c_size size)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ms2r_state *state = to_state(sd);
	struct device *cdev = &client->dev;
	struct i2c_msg msg[1];
	u8 wbuf[I2C_MAX + 4];
	u32 *buf = (u32 *)&wbuf[4];
	u8 category = (addr >> 8) & 0xff;
	u8 cmd = addr & 0xff;
	int ret;

	if (!client->adapter)
		return -ENODEV;

	if (size != I2C_8BIT && size != I2C_16BIT && size != I2C_32BIT) {
		dev_err(cdev, "Wrong data size\n");
		return -EINVAL;
	}

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = size + 4;
	msg->buf = wbuf;
	wbuf[0] = size + 4;
	wbuf[1] = CMD_WRITE_PARAMETER;
	wbuf[2] = category;
	wbuf[3] = cmd;

	*buf = ms2r_swap_byte((u8 *)&val, size);

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s(), failed WRITE[%d] at "
				"cat[%02x] cmd[%02x], ret %d\n",
				__func__,
				size, msg->buf[2], msg->buf[3], ret);
		return ret;
	}

	if (state->debug)
		pr_info("write: 0x%04x = 0x%x\n", addr, val);

	return 0;
}

/*
  * I2C read memory function
  * data: write data
  * addr: 32bit address
  * size: write size, max 16 bit
*/
int ms2r_write_memory(struct v4l2_subdev *sd,
		u32 addr, const char *data,
		int size)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ms2r_state *state = to_state(sd);
	struct i2c_msg msg[1];
	int ret;

	/*firmware buffer should be alloc memory before download*/
	if (!state->fw_buffer) {
		pr_err("%s: firmware buffer is NULL\n", __func__);
		return -EINVAL;
	}

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = size + 8;
	msg->buf = state->fw_buffer;
	msg->buf[0] = 0;
	msg->buf[1] = CMD_WRITE_8BIT_MEMORY;
	msg->buf[2] = (addr >> 24) & 0xff;
	msg->buf[3] = (addr >> 16) & 0xff;
	msg->buf[4] = (addr >> 8) & 0xff;
	msg->buf[5] = addr & 0xff;
	msg->buf[6] = (size >> 8) & 0xff;
	msg->buf[7] = size & 0xff;
	memcpy(&msg->buf[8], data, size);

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s(), failed WRITE[%d] at "
				"cat[%02x] cmd[%02x], ret %d\n", __func__,
				size, msg->buf[2], msg->buf[3], ret);
		return ret;
	}

	return 0;
}


/*
  * I2C read memory function
  * data: buffer holding data read form slave.
  * addr: 32bit address
  * size: write size, max 16 bit
*/
int ms2r_read_memory(struct v4l2_subdev *sd,
		u32 addr, char *data, int size)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ms2r_state *state = to_state(sd);
	struct i2c_msg msg[2];
	int ret;
	int count;

	/*firmware buffer should be alloc memory before download*/
	if (!state->fw_buffer) {
		pr_err("%s: firmware buffer is NULL\n", __func__);
		return -EINVAL;
	}

	if (!client->adapter)
		return -ENODEV;
	/*
	* First write
	*/
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 8;
	msg[0].buf = state->fw_buffer;
	msg[0].buf[0] = 0;
	msg[0].buf[1] = CMD_READ_8BIT_MEMORY;
	msg[0].buf[2] = (addr >> 24) & 0xff;
	msg[0].buf[3] = (addr >> 16) & 0xff;
	msg[0].buf[4] = (addr >> 8) & 0xff;
	msg[0].buf[5] = addr & 0xff;
	msg[0].buf[6] = (size >> 8) & 0xff;
	msg[0].buf[7] = size & 0xff;

	/*
	* Then read
	*/
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = size + 3;
	msg[1].buf = state->fw_buffer;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s(), failed WRITE[%d] at "
				"cat[%02x] cmd[%02x], ret %d\n", __func__,
				size, msg->buf[2], msg->buf[3], ret);
		return ret;
	}

	count = (msg[1].buf[1] << 8) + msg[1].buf[2];
	if(count == size) {
		memcpy(data, &msg[1].buf[3], size);
	} else {
		pr_err("%s(), read count 0x%x is NOT equal to 0x%x",
			__func__, count, size);
		return -EINVAL;
	}

	return 0;
}

int ms2r_write_regs(struct v4l2_subdev *sd, struct ms2r_reg *regs, int size)
{
	int ret = 0, i;
	struct ms2r_reg reg;

	for (i = 0; i < size; i++) {
		reg = regs[i];
		ret = ms2r_write_reg(sd, reg.addr, reg.val, reg.size);
		CHECK_ERR(ret);
	}

	return 0;
}

/*****************************************/
/**********       sys interface    **************/
/*****************************************/

/*
  * show firmware status
*/
static ssize_t show_firmware_status(struct device *d,
		struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *sd = dev_get_drvdata(d);
	struct ms2r_state *state = to_state(sd);	

	return sprintf(buf, "%d\n", state->fw_status);
}

static ssize_t store_register(struct device *d,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct v4l2_subdev *sd = dev_get_drvdata(d);
	struct ms2r_state *state = to_state(sd);
	int ret, type;
	u32 addr, val;

	if (!ms2r_is_isppower(state)) {
		pr_err("Please power on first\n");
		return -EINVAL;
	}
	
	if (sscanf(buf, "w%d %x=%x", &type, &addr, &val) == 3) {
		switch (type) {
		case 8:
			ret = ms2r_w8(sd, (u16)addr, val);
			break;
		case 16:
			ret = ms2r_w16(sd, (u16)addr, val);
			break;			
		case 32:
			ret = ms2r_w32(sd, (u16)addr, val);
			break;			
		default:
			ret = -EINVAL;
			break;			
		}
		CHECK_ERR(ret);
		pr_info("write: 0x%04x = 0x%x\n", addr, val); 
	} else if (sscanf(buf, "r%d %x", &type, &addr) == 2) {
		switch (type) {
		case 8:
			ret = ms2r_r8(sd, (u16)addr, &val);
			break;
		case 16:
			ret = ms2r_r16(sd, (u16)addr, &val);
			break;			
		case 32:
			ret = ms2r_r32(sd, (u16)addr, &val);		
		default:
			ret = -EINVAL;
			break;			
		}
		CHECK_ERR(ret);
		pr_info("read: 0x%04x = 0x%x\n", addr, val); 
	} else {
		ret = -EINVAL;
		pr_err("Invalid format. write format: echo > \"w8/16/32 addr=value\" > register; \
			read format: echo > \"r8/16/32 addr\n");
	}
	
	return count;
}

static ssize_t store_debug(struct device *d,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int val;
	struct v4l2_subdev *sd = dev_get_drvdata(d);
	struct ms2r_state *state = to_state(sd);

	if (sscanf(buf, "%d", &val) == 1)
		state->debug = !!val;

	return count;
}

static ssize_t show_module_type(struct device *d,
		struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *sd = dev_get_drvdata(d);
	struct ms2r_state *state = to_state(sd);
	char *module_type = NULL;

	switch (state->module_type) {
	case MODULE_REAR_LITEON:
		module_type = "liteon";
		break;
	case MODULE_REAR_SHARP:
		module_type = "sharp";
		break;
	default:
		module_type = "unkown";
		break;
	}

	return sprintf(buf, "module type:%s\n", module_type);
}

static ssize_t store_module_type(struct device *d,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int val;
	struct v4l2_subdev *sd = dev_get_drvdata(d);
	struct ms2r_state *state = to_state(sd);

	if (sscanf(buf, "%d", &val) != 1) {
		pr_err("%s(), input err\n", __func__);
		return -EINVAL;
	}

	#if 0
	if (!!val) {
		ret = ms2r_request_module_fw(sd);
		if (ret) {
			pr_err("%s(), request module retrievint fw failed\n", __func__);
			return ret;
		}
		ms2r_retrieve_module(sd, false);
	}
	#else
	if (MODULE_REAR_LITEON == val) {
		pr_err("%s(), force using liteon\n", __func__);
		state->module_type = MODULE_REAR_LITEON;
	} else if (MODULE_REAR_SHARP == val) {
		pr_err("%s(), force using sharp\n", __func__);
		state->module_type = MODULE_REAR_SHARP;
	} else {
		pr_err("%s(), err! invalid input: %d!\n", __func__, val);
		return -EINVAL;
	}

	ms2r_save_file(sd);
	#endif

	return count;
}

static ssize_t show_pre_flash_current(struct device *d,
		struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *sd = dev_get_drvdata(d);
	struct ms2r_state *state = to_state(sd);	

	return sprintf(buf, "%d\n", state->pre_flash_current / 1000);
}

static ssize_t store_pre_flash_current(struct device *d,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int val;
	struct v4l2_subdev *sd = dev_get_drvdata(d);
	struct ms2r_state *state = to_state(sd);

	if (sscanf(buf, "%d", &val) == 1) {
		val = val * 1000;
		if (val > MAX_FLASH_CURRENT) {
			pr_err("%s:flash current is too large!!\n", __func__);
			return -EINVAL;
		}
		state->pre_flash_current = val;
	} else 
		return -EINVAL;

	pr_info("%s:pre flash current = %d uA\n", __func__, state->pre_flash_current);
	
	return count;
}

static ssize_t show_full_flash_current(struct device *d,
		struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *sd = dev_get_drvdata(d);
	struct ms2r_state *state = to_state(sd);	

	return sprintf(buf, "%d\n", state->full_flash_current / 1000);
}

static ssize_t store_full_flash_current(struct device *d,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int val;
	struct v4l2_subdev *sd = dev_get_drvdata(d);
	struct ms2r_state *state = to_state(sd);

	if (sscanf(buf, "%d", &val) == 1) {
		val = val * 1000;
		if (val > MAX_FLASH_CURRENT) {
			pr_err("%s:flash current is too large!!\n", __func__);
			return -EINVAL;
		}
		state->full_flash_current = val;
	} else 
		return -EINVAL;

	pr_info("%s:full flash current = %d uA\n", __func__, state->full_flash_current);
	
	return count;
}

static ssize_t show_version(struct device *d,
		struct device_attribute *attr, char *buf)
{
	int val, ret;
	struct v4l2_subdev *sd = dev_get_drvdata(d);
	struct ms2r_state *state = to_state(sd);

	if (!ms2r_is_isppower(state)) {
		pr_err("Please power on first\n");
		return -EINVAL;
	}

	ret = ms2r_r16(sd, VER_FIRMWARE_REG, &val);
	CHECK_ERR(ret);
	pr_info("Firmware Version: 0x%04x\n", val);

	ret = ms2r_r16(sd, VER_HARDWARE_REG, &val);
	CHECK_ERR(ret);
	pr_info("Hardware Version: 0x%04x\n", val);

	ret = ms2r_r16(sd, VER_PARAMETER_REG, &val);
	CHECK_ERR(ret);
	pr_info("Parameter Version: 0x%04x\n", val);

	ret = ms2r_r16(sd, VER_AWB_REG, &val);
	CHECK_ERR(ret);
	pr_info("AWB Version: 0x%04x\n", val);

	return 0;
}

static DEVICE_ATTR(firmware_status, 0444, show_firmware_status, NULL);
static DEVICE_ATTR(register, 0220, NULL, store_register);
static DEVICE_ATTR(debug, 0660, NULL, store_debug);
static DEVICE_ATTR(module_type, 0660,
	show_module_type, store_module_type);
static DEVICE_ATTR(pre_flash_current, 0660, 
	show_pre_flash_current, store_pre_flash_current);
static DEVICE_ATTR(full_flash_current, 0660, 
	show_full_flash_current, store_full_flash_current);
static DEVICE_ATTR(version, 0660,
	show_version, NULL);

static struct attribute *ms2r_attributes[] = {
	&dev_attr_firmware_status.attr,
	&dev_attr_register.attr,
	&dev_attr_debug.attr,
	&dev_attr_module_type.attr,
	&dev_attr_pre_flash_current.attr,
	&dev_attr_full_flash_current.attr,
	&dev_attr_version.attr,
	NULL
};

static const struct attribute_group ms2r_group = {
	.attrs = ms2r_attributes,
};

/*
  * clear completion counter function
  * before waiting completion, we should do this
*/
void ms2r_prepare_wait(struct v4l2_subdev *sd)
{
	struct ms2r_state *state = to_state(sd);

	init_completion(&state->completion);
}

/*
  * wait interrupt event function without checking
*/
int ms2r_wait_irq(struct v4l2_subdev *sd, const unsigned int timeout)
{
	struct ms2r_state *state = to_state(sd);
	int ret;

	ret = wait_for_completion_timeout(&state->completion,
		msecs_to_jiffies(timeout));
	if (ret <= 0) {
		pr_err("%s(), err: %d\n", __func__, ret);
		return -ETIME;
	}
	return 0;
}

/*
  * wait interrupt event function with timeout
  * if proper event comes, return 0
*/
int ms2r_wait_irq_and_check(struct v4l2_subdev *sd, u8 mask,
	const unsigned int timeout)
{
	struct ms2r_state *state = to_state(sd);
	int ret;

	ret = wait_for_completion_timeout(&state->completion,
		msecs_to_jiffies(timeout));
	if (ret <= 0) {
		pr_info("%s: timeout in %u ms when wating for %d. ERR ret: %d\n",
			__func__, timeout, mask, ret);
		return -ETIME;
	}

	mutex_lock(&state->mutex);

	if ((state->irq_status & mask) == mask)
		ret = 0;
	else 
		ret = -EINVAL;
	mutex_unlock(&state->mutex);	
	
	return ret;
}

#if 0
int ms2r_enable_root_irq(struct v4l2_subdev *sd)
{
	return ms2r_w8(sd, INT_ROOR_ENABLE_REG, 0x01);
}
#else
inline int ms2r_enable_root_irq(struct v4l2_subdev *sd)
{
	pr_debug("%s(), Do nothing when using MS2R\n", __func__);
	return 0;
}
#endif

int ms2r_enable_irq(struct v4l2_subdev *sd)
{
	int ret;

	ret = ms2r_w8(sd, INT_ENABLE_REG, 0xff);
	CHECK_ERR(ret);
	
	return ms2r_enable_root_irq(sd);
}

/*
  * set ISP operating mode function
  * parameter, monitor or capture
*/
int ms2r_set_sys_mode(struct v4l2_subdev *sd, enum isp_mode mode)
{
	int ret;
	enum sys_mode smode;
	struct ms2r_state *state = to_state(sd);

	if (state->mode == mode) return 0;

	pr_info("%s(), ISP mode will be set to %d\n", __func__, mode);

	switch (mode) {
	case PARAMETER_MODE:
		smode = SYS_PARAMETER_MODE;
		break;
	case MONITOR_MODE:
		smode = SYS_MONITOR_MODE;
		break;
	case CAPTURE_MODE:
		smode = SYS_CAPTURE_MODE;
		break;
	default:
		return -EINVAL;
	}

	if (SYS_PARAMETER_MODE != smode) {
		ms2r_prepare_wait(sd);
		ret = ms2r_enable_irq(sd);
		if (ret) return ret;
		ret = ms2r_w8(sd,  SYS_MODE_REG, smode);
		CHECK_ERR(ret);
		ret = ms2r_wait_irq_and_check(sd, INT_MASK_MODE, WAIT_TIMEOUT);
		if (ret) return ret;
	} else {
		/*
		* There is NO interrupt when transferring to parameter mode.
		*/
		ret = ms2r_w8(sd,  SYS_MODE_REG, smode);
		CHECK_ERR(ret);
	}
	state->mode = mode;

	return 0;
}

/*
  * set isp parameter mode function
*/
static int ms2r_set_parameter_mode(struct v4l2_subdev *sd)
{
	struct ms2r_state *state = to_state(sd);
	int ret;

	if (state->mode == PARAMETER_MODE) return 0;

	/*
	* There is NO interrupt when transferring to parameter mode.
	*/
	ret = ms2r_w8(sd,  SYS_MODE_REG, SYS_PARAMETER_MODE);	
	CHECK_ERR(ret);

	state->mode = PARAMETER_MODE;

	return 0;
}

/*
  * set isp normal monitor(preview) mode function
*/
static int ms2r_set_monitor_mode(struct v4l2_subdev *sd)
{
	int ret, i, retry = 3;
	struct ms2r_state *state = to_state(sd);

	if (state->mode == MONITOR_MODE) return 0;

	/* ensure special panorama off
	* Needless in MS2R
	*/

	ms2r_prepare_wait(sd);

	ret = ms2r_w8(sd,  SYS_MODE_REG, SYS_MONITOR_MODE);	
	CHECK_ERR(ret);
	
	for (i = 0; i < retry; i++) {
		ret = wait_for_completion_timeout(&state->completion,
			msecs_to_jiffies(WAIT_TIMEOUT));
		if (ret <= 0) {
			pr_err("%s: timeout in %u ms, ret is %d\n",
				__func__, WAIT_TIMEOUT, ret);
			return -ETIME;
		}

		if (state->irq_status & INT_MASK_MODE)
			break;
	}

	if (i == retry) return -EINVAL;

#if 0
	/*
	* set capture normal mode
	* This is needless in MS2R
	*/
	ret = ms2r_w8(sd, CAP_MODE_REG, CAP_MODE_NORMAL);
	CHECK_ERR(ret);
#endif

	state->mode = MONITOR_MODE;

	return 0;
}

/*
  * set isp capture mode function
*/
static int ms2r_set_capture_mode(struct v4l2_subdev *sd)
{
	int ret, i, retry = 5;
	struct ms2r_state *state = to_state(sd);

	pr_info("%s:wdr = %d\n", __func__, state->userset.wdr);

	if (state->mode == CAPTURE_MODE) return 0;

	ms2r_prepare_wait(sd);

	ret = ms2r_w8(sd,  SYS_MODE_REG, SYS_CAPTURE_MODE);	
	CHECK_ERR(ret);
	
	for (i = 0; i < retry; i++) {
		ret = wait_for_completion_timeout(&state->completion,
			msecs_to_jiffies(5000));
		if (ret <= 0) {
			pr_err("%s: timeout in %u ms, ret is: %d\n", __func__, 5000, ret);
			return -ETIME;
		}

		mutex_lock(&state->mutex);

		/* change flash to full current */
		if (state->irq_status & INT_MASK_FRAMESYNC)
			if (state->userset.flash_mode != MS2R_FLASH_OFF)
				ms2r_set_flash_current(state, state->full_flash_current);

		if (state->irq_status & INT_MASK_CAPTURE) {
			mutex_unlock(&state->mutex);
			break;
		}
		mutex_unlock(&state->mutex);	
	}

	if (i == retry) return -EINVAL;

	state->mode = CAPTURE_MODE;

	return 0;
}

int ms2r_set_mode(struct v4l2_subdev *sd, enum isp_mode mode)
{
	int ret;

	if (to_state(sd)->mode == mode) {
		pr_warn("%s(), ISP mode has already been set to %d\n",
			__func__, mode);
		return 0;
	} else {
		pr_info("%s(), set ISP mode to %d\n", __func__, mode);
	}

	switch (mode) {	
	case PARAMETER_MODE:
		ret = ms2r_set_parameter_mode(sd);
		break;
	case MONITOR_MODE:
		ret = ms2r_set_monitor_mode(sd);
		break;
	case CAPTURE_MODE:
		ret = ms2r_set_capture_mode(sd);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

int ms2r_enable_ae_lock(struct v4l2_subdev *sd, bool enable)
{
	int ret = 0;

	if (enable) {
		pr_info("%s(), enable AE lock\n", __func__);
		ret = ms2r_w8(sd, AE_LOCK_REG, 0x01);
		CHECK_ERR(ret);
	} else {
		pr_info("%s(), disable AE lock\n", __func__);
		ret = ms2r_w8(sd, AE_LOCK_REG, 0x00);
		CHECK_ERR(ret);
	}

	return ret;
}

int ms2r_enable_awb_lock(struct v4l2_subdev *sd, bool enable)
{
	int ret = 0;

	if (enable) {
		pr_info("%s(), enable AWB lock\n", __func__);
		ret = ms2r_w8(sd, AWB_LOCK_REG, 0x01);
		CHECK_ERR(ret);
	} else {
		pr_info("%s(), disable AWB lock\n", __func__);
		ret = ms2r_w8(sd, AWB_LOCK_REG, 0x00);
		CHECK_ERR(ret);
	}

	return ret;
}

/*
  * set isp panorama preview mode
*/
static int ms2r_set_panorama_mode(struct v4l2_subdev *sd)
{
	int ret;

	/* shoud set parameter mode first */
	ret = ms2r_set_mode(sd, PARAMETER_MODE);
	if (ret) return ret;

	/* enable special panorama first */
	ret = ms2r_w8(sd, SPECIAL_MON_REG, SPECIAL_PANORAMA);
	CHECK_ERR(ret);

	/* enter monitor mode */
	ret = ms2r_set_sys_mode(sd, MONITOR_MODE);
	if (ret) return ret;
	
	/* disable AE and AWB Lock */
	ret = ms2r_enable_ae_lock(sd, false);
	ret = ms2r_enable_awb_lock(sd, false);
	if (!ret) 
		pr_info("%s():change camera to panorama monitor mode.\n", __func__);

	return ret;
}

static void set_prev_special_resolution_val(struct ms2r_state *state,
	enum v4l2_camera_mode mode)
{
#if 0
	/* normal preview and record */
	if (mode != V4L2_CAMERA_PANORAMA) {
		ms2r_prev_1216x912[0].val = 0x36;
	} else {
		/* panorama preview mode */
		ms2r_prev_1216x912[0].val = 0x24;
	}

	/*
	* For Front camera record mode's frame rate
	*/
	if (FRONT_CAMERA == state->cam_id  &&
		V4L2_CAMERA_RECORD == mode) {
		ms2r_prev_720p[0].val = 0x28;		
	} else {
		ms2r_prev_720p[0].val = 0x21;
	}
#endif
	/*
	* Rear camera's 1080p is diffrent from front's.
	*/
	if (BACK_CAMERA == state->cam_id) {
		ms2r_prev_1080p[0].val = 0x28;
	} else {
		ms2r_prev_1080p[0].val = 0x25;
	}

	/*
	* Rear camera's VGA supports 120FPS at record mode
	*/
	if (BACK_CAMERA == state->cam_id &&
		V4L2_CAMERA_RECORD == mode) {
		//ms2r_prev_vga[0].val = MON_VGA_120FPS;
		ms2r_prev_vga[0].val = MON_VGA;
	} else {
		ms2r_prev_vga[0].val = MON_VGA;
	}
}

static int ms2r_set_preview_size(struct v4l2_subdev *sd, 
	struct v4l2_mbus_framefmt *fmt, enum v4l2_camera_mode mode)
{
	struct ms2r_state *state = to_state(sd);
	int i, ret;
	bool update_isp = false;
	static unsigned int last_reg_val = 0;
	struct ms2r_size_struct *sizes = ms2r_prev_sizes;
	int len = ARRAY_SIZE(ms2r_prev_sizes);
		
	/* look down to find the smaller preview size */
	for (i = len - 1; i >= 0; i--) 
		if ((fmt->width <= sizes[i].width) &&
			(fmt->height <= sizes[i].height))
			break;

	if (i < 0) {
		pr_err("%s: can not mach preview size(%d, %d)\n", 
			__func__, fmt->width, fmt->height);
		return -EINVAL;
	}

	fmt->width = sizes[i].width;
	fmt->height = sizes[i].height;

	set_prev_special_resolution_val(state, mode);

	if (last_reg_val != sizes[i].regs->val ||
		state->prev_size.width != fmt->width ||
		state->prev_size.height != fmt->height ||
		state->cam_id == FRONT_CAMERA) {
		update_isp = true;
	}

	state->prev_size = sizes[i];

	if (update_isp) {
		/* should be set parameter mode first */
		ret = ms2r_set_mode(sd, PARAMETER_MODE);
		if (ret) return ret;

		if (state->need_sleep) {
			pr_info("%s(), sleep for 100 ms\n", __func__);
			msleep(100);
		} else {
			state->need_sleep = true;
		}

		pr_info("%s(), write ISP reg 0x%04x to value 0x%x\n"
			"Last reg value is 0x%x", __func__,
			state->prev_size.regs->addr, state->prev_size.regs->val, last_reg_val);
		ret = ms2r_write_regs(sd, state->prev_size.regs, state->prev_size.size);
		if (ret) return ret;

		last_reg_val = state->prev_size.regs->val;

		#if 1
		if (state->cam_id == BACK_CAMERA &&
			fmt->height == 1080) {
			pr_info("%s(), write for 30FPS of rear 1080p\n", __func__);
			ms2r_w8(sd, EVP_MODE_MON_REG, EVP_MODE_30FPS);
		} else {
			ms2r_w8(sd, EVP_MODE_MON_REG, EVP_MODE_AUTO);
		}
		#endif
	}else {
		pr_info("%s(), no Need to update ISP register\n", __func__);
	}
	
	return 0;
}

int ms2r_set_capture_format(struct v4l2_subdev *sd,
	struct v4l2_mbus_framefmt *fmt)
{
	struct ms2r_state *state = to_state(sd);
	struct ms2r_format_struct *formats = ms2r_cap_formats;
	int i, ret, len = ARRAY_SIZE(ms2r_cap_formats);
		
	for (i = 0; i < len; i++)
		if (fmt->code == formats[i].mbus_code)
			break;

	if (i == len) {
		pr_err("%s: can not mach format(%d)\n", __func__, fmt->code);
		return -EINVAL;
	}

	if (state->cap_fmt.mbus_code == fmt->code)
		return 0;

	state->cap_fmt = formats[i];
	
	ret = ms2r_write_regs(sd, state->cap_fmt.regs, state->cap_fmt.size);
	if (ret) return ret;

	return 0;
}

int ms2r_set_capture_size(struct v4l2_subdev *sd,
	struct v4l2_mbus_framefmt *fmt)
{
	struct ms2r_state *state = to_state(sd);
	struct ms2r_size_struct *sizes = ms2r_cap_sizes;
	int i, ret, len = ARRAY_SIZE(ms2r_cap_sizes);

	pr_info("%s(), user wants %d * %d capture size\n",
		__func__, fmt->width, fmt->height);
	for (i = 0; i < len; i++)
		if ((fmt->width == sizes[i].width) &&
			(fmt->height == sizes[i].height))
			break;

	if (i == len) {
		pr_err("%s: can not mach capture size(%d, %d)\n", 
			__func__, fmt->width, fmt->height);
		return -EINVAL;
	}

	fmt->width = sizes[i].width;
	fmt->height = sizes[i].height;

	if (state->cap_size.width == fmt->width &&
		state->cap_size.height == fmt->height)
		return 0;

	state->cap_size = sizes[i];

	ret = ms2r_write_regs(sd, state->cap_size.regs, state->cap_size.size);
	if (ret) return ret;

	return 0;
}

static int 
ms2r_get_camera_mode_type(struct ms2r_state *state, enum v4l2_camera_mode mode)
{
	if (mode < V4L2_CAMERA_SINGLE_CAPTURE)
		return PREVIEW_MODE_TYPE;
	else if (mode <= V4L2_CAMERA_PANORAMA_CAPTURE)
		return CAPTURE_MODE_TYPE;
	else 
		return -EINVAL;
}

static int ms2r_s_fmt(struct v4l2_subdev *sd, 
	struct v4l2_mbus_framefmt *fmt)
{
	int ret;
	int mode = fmt->reserved[0];
	struct ms2r_state *state = to_state(sd);
	enum camera_mode_type type;

	pr_info("%s: >> (w, h) = (%d, %d), fmt = 0x%x, mode = %d\n",
		__func__, fmt->width, fmt->height, fmt->code, mode);

	fmt->field = V4L2_FIELD_NONE;

	type = ms2r_get_camera_mode_type(state, mode);
	if (type < 0) return -EINVAL;
	
	if (type == PREVIEW_MODE_TYPE) {
		ret = ms2r_set_preview_size(sd, fmt, mode);
		if (ret) return ret;
	} else {  /* CAPTURE_MODE_TYPE */
		/* do nothing */
	}

	state->camera_mode = mode;

	pr_info("%s: << (w, h) = (%d, %d), fmt = 0x%x, mode =%d\n",
		__func__, fmt->width, fmt->height, fmt->code, mode);
	
	return 0;
}

static int ms2r_enum_framesizes(struct v4l2_subdev *sd, 
					struct v4l2_frmsizeenum *fsize)
{
	struct ms2r_state *state = to_state(sd);
	enum camera_mode_type type;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	
	type = ms2r_get_camera_mode_type(state, state->camera_mode);
	if (type < 0) return -EINVAL;
	
	if (type == PREVIEW_MODE_TYPE) {
		fsize->discrete.width = state->prev_size.width;
		fsize->discrete.height = state->prev_size.height;	
	} else {
		fsize->discrete.width = state->cap_size.width;
		fsize->discrete.height = state->cap_size.height;	
	}

	return 0;
}

static int ms2r_stream_on(struct v4l2_subdev *sd)
{
	struct ms2r_state *state = to_state(sd);	
	int ret = 0;

	pr_info("%s: stream on, camera_mode is %d\n",
		__func__, state->camera_mode);

	/* reset this flag in case of wrong wake up */
//	fimc_reset_wakeup_flag();
	
	switch (state->camera_mode) {
	case  V4L2_CAMERA_PREVIEW:
	case  V4L2_CAMERA_RECORD:
		ret = ms2r_set_mode(sd, MONITOR_MODE);
		break;
	case V4L2_CAMERA_PANORAMA:
		ret = ms2r_set_panorama_mode(sd);
		break;
	default:
		break;
	}

	if (!ret) state->stream_on = true;

	return ret;
}

static int ms2r_stream_off(struct v4l2_subdev *sd)
{
	struct ms2r_state *state = to_state(sd);	

	pr_info("%s(), +++++++++++\n", __func__);
	
	if (state->stream_on) state->stream_on = false;
	
	return 0;
}

static int ms2r_s_stream(struct v4l2_subdev *sd, int enable)
{
	if(enable)
		return ms2r_stream_on(sd);
	else
		return ms2r_stream_off(sd);
}

/*Remove ms2r_fps_to_sensor_step() as we don't use it any more*/

static int ms2r_s_parm(struct v4l2_subdev *sd, 
	struct v4l2_streamparm *param)
{
	/* do nothing, we don't set framerate registers any more */
	
	return 0;
}

static int ms2r_reset(struct v4l2_subdev *sd, u32 val)
{
	struct ms2r_state *state = to_state(sd);

	state->pdata->reset();
	
	return 0;
}

static int ms2r_set_sensor_power(struct ms2r_state *state, bool enable)
{
	int ret;
	
	if (state->sensor_power == enable) {
		pr_err("%s(), sensor power has already been set to %d",
			__func__, enable);
		return -EBUSY;
	}

	ret = state->pdata->set_sensor_power(state->cam_id, enable);
	if (!ret) state->sensor_power = enable;

	return ret;
}

/*
* This only configuration power and clock need by ISP.
* Sensors' power is NOT configure here.
*/
int ms2r_set_power_clock(struct ms2r_state *state, bool enable)
{
	struct ms2r_platform_data *pdata = state->pdata;
	struct v4l2_subdev *sd = &state->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	
	if (state->isp_power == enable) {
		pr_err("%s(), ISP Power has already been set to %d!\n",
			__func__, enable);
		return -EBUSY;
	}
	
	if (enable) {
		ret = pdata->set_isp_power(true);
		if (ret) {
			pr_err("%s(), set ISP powr failed!\n", __func__);
			return ret;
		}

		ret = pdata->clock_enable(&client->dev, true);
		if (ret) return ret;
		state->isp_power = enable;	
	} else {
		state->isp_power = enable;
		pdata->clock_enable(&client->dev, false);
		pdata->set_isp_power(false);
		if (state->sensor_power)   
			ms2r_set_sensor_power(state, false);
	}

	pr_debug("%s(), set power to %d successed!\n", __func__, enable);

	return ret;
}

/*
* Configure sensor, ISP and clock together.
*/
int ms2r_sensor_isp_s_power(struct v4l2_subdev *sd, int on)
{
	struct ms2r_state *state = to_state(sd);
	bool enable = !!on;
	int ret;

	pr_debug("%s(), set sensor && ISP power to %d\n",
		__func__, enable);

	/* set sensor power */
	ret = ms2r_set_sensor_power(state, enable);
	if (ret) {
		pr_err("%s(), set sensor powr failed!\n", __func__);
		return ret;
	}

	/*
	* set ISP power && clocks needed.
	*/
	ret = ms2r_set_power_clock(state, enable);
	if (ret) {
		pr_err("%s(), set ISP powr failed!\n", __func__);
		return ret;
	}

	return 0;
}

static int ms2r_init(struct v4l2_subdev *sd, u32 cam_id)
{
	int ret = 0;
	struct ms2r_state *state = to_state(sd);	
	int retry_times = 1, tried_times = 0;

	if (!state->prepared) {
		pr_err("%s(), The module retrieving thread has NOT been finished yet\n", __func__);
		return -EBUSY;
	}

	if (cam_id != BACK_CAMERA && cam_id != FRONT_CAMERA) {
		pr_err("unsupport camera id %d\n", cam_id);
		return -EINVAL;
	}

	state->cam_id = cam_id;

	pr_debug("%s(), Power on and Initialize camera %d\n", __func__, state->cam_id);
	ret = ms2r_sensor_isp_s_power(sd, 1);
	if (ret) {
		pr_info("%s(), power on failed\n", __func__);
		return ret;
	}
	if (!state->sensor_power || !state->isp_power) {
		pr_err("%s(), Power on sensor && ISP  first!!!\n", __func__);
		return -EINVAL;
	}

	ret = ms2r_request_fw(sd, state->cam_id);
	if (ret) {
		pr_err("%s(), request fw failed!!!\n", __func__);
		goto exit;
	}

	ret = ms2r_load_firmware(sd);
	while (ret && tried_times++ < retry_times) {
		pr_info("%s(), load failed, retry %d times\n",
			__func__, tried_times);
		ms2r_sensor_isp_s_power(sd, 0);
		ms2r_sensor_isp_s_power(sd, 1);
		ret = ms2r_load_firmware(sd);
	}
	if (ret) {
		pr_err("%s(), load fw failed!!!\n", __func__);
		goto exit;
	}

	/* enable all irq */
	ret = ms2r_enable_irq(sd);
	if (ret) goto exit;
#if 0
	/*
	* Needless in MS2R
	*/
	if (state->cam_id == BACK_CAMERA)
		ret = ms2r_w8(sd, 0x013f, 0x00);
	else
		ret = ms2r_w8(sd, 0x013f, 0x01);
	if (ret) goto exit;
#endif
	
	state->fps = MS2R_FR_AUTO;
	state->mode = PARAMETER_MODE;
	state->stream_on = false;
	memset(&state->prev_size, 0, sizeof(struct ms2r_size_struct));

	state->need_sleep = false;
	state->slow_shut_delay = 0;

	state->cap_size.width = DEFAULT_CAPTURE_WIDTH;
	state->cap_size.height = DEFAULT_CAPTURE_HEIGHT;
	state->cap_fmt.mbus_code = V4L2_MBUS_FMT_JPEG_1X8;

	/* init default userset parameter, this is the reset value of ISP */
	state->userset.manual_wb = MS2R_WB_AUTO;
	state->userset.scene = MS2R_SCENE_AUTO;	
	state->userset.zoom_level = MS2R_ZL_1;
	state->userset.wdr = MS2R_WDR_OFF;
	state->userset.iso = MS2R_ISO_AUTO;
	state->userset.flash_mode = MS2R_FLASH_OFF;
	state->userset.rotation = MS2R_ROTATE_0;
	state->userset.reverse = MS2R_NO_REVERSE;
	state->userset.af_mode = MS2R_FOCUS_AUTO;

	/*
	* Select MIPI I/F
	*/
	ret = ms2r_w8(sd, OUT_SELECT_REG, MIPI_IF);
	if (ret) goto exit;
	if (BACK_CAMERA == state->cam_id) {
		/*
		* Add by qudao, for mipi 4 lanes out
		*/
		pr_debug("%s(), For MS2R's 4 lanes output\n", __func__);
		ret = ms2r_w8(sd, MS2R_CSI_OUT_4LANE, 0x04);
		if (ret) goto exit;
	} else {
		pr_debug("%s(), Front Camera uses 4 lanes output\n", __func__);
		ret = ms2r_w8(sd, MS2R_CSI_OUT_4LANE, 0x04);
		if (ret) goto exit;

		pr_debug("%s(), Horizontal mirror\n", __func__);
		state->userset.mirror = MS2R_MIRROR;
		ms2r_w8(sd, MON_MIRROR_ISP_REG, MS2R_MIRROR);
	}
	
exit:
	/*
	* power off will be done at close procedure.
	*/
	//ms2r_set_sensor_power(state, false);
	if (ret) {
		pr_err("%s(), camera initialization failed!!\n", __func__);
	} else {
		pr_info("%s(), camera initialization finished\n", __func__);
	}
	return ret;
}

static void ms2r_handle_normal_cap(struct ms2r_state *state, int irq_status)
{
	pr_info("%s: irq status = 0x%02x\n", __func__, irq_status);
	complete(&state->completion);  /* just to wake up any waiters */
}

static void ms2r_handle_work(struct work_struct *work)
{
	struct ms2r_state *state = container_of(work, struct ms2r_state, work);
	int ret;
	u32 irq_status;

	pr_debug("%s() entered, isp power:%d\n",
		__func__, ms2r_is_isppower(state));

	if (!ms2r_is_isppower(state)) return;

	/* read interrupt status */
	ret = ms2r_r8(&state->sd, INT_FACTOR_REG, &irq_status);
	if (ret) return;	

	/* save the irq status */
	mutex_lock(&state->mutex);
	state->irq_status = irq_status;
	mutex_unlock(&state->mutex);

	switch (state->cap_mode) {
	case CAP_NORMAL_MODE:
		ms2r_handle_normal_cap(state, irq_status);
		break;
	case CAP_PANORAMA_MODE:
		ms2r_handle_panorama_cap(state, irq_status);
		break;
	case CAP_MULTI_CAP_MODE:
		ms2r_handle_multi_cap(state, irq_status);
		break;
	case CAP_SMILE_CAP_MODE:
		ms2r_handle_smile_cap(state, irq_status);
		break;
	case CAP_AUTO_BRACKET_MODE:
		ms2r_handle_auto_bracket_cap(state, irq_status);
		break;
	default:
		break;
	}
}

/*
  * ISP interrupt handle function
  * we can operate i2c because it is in irq thread
*/
static irqreturn_t ms2r_irq_handler(int irq, void *dev_id)
{
	struct ms2r_state *state = (struct ms2r_state *)dev_id;

	queue_work(state->wq, &state->work);
	
	return IRQ_HANDLED;
}

#if defined(CONFIG_MEDIA_CONTROLLER) && defined(CONFIG_ARCH_EXYNOS5)
static inline int ms2r_pad_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_fh *fh,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	pr_info("%s() entered\n", __func__);
	return 0;
}

static inline int ms2r_pad_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			  struct v4l2_subdev_format *fmt)
{
	pr_debug("%s() entered\n", __func__);
	return 0;
}

static inline int ms2r_pad_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			  struct v4l2_subdev_format *fmt)
{
	pr_info("%s() ++++. Via s_fmt on /dev/v4l2-subdev*\n"
		"which: 0x%x, pad: %d, reserved(0: preview, 3: capture): %d\n"
		"%d X %d, code: 0x%x\n", __func__,
		fmt->which, fmt->pad, fmt->format.reserved[0],
		fmt->format.width, fmt->format.height, fmt->format.code);
#if 0	
	pr_info("%s(), Invoking ms2r_s_fmt()\n", __func__);
	ms2r_s_fmt(sd, &fmt->format);
#endif
	return 0;
}

static struct v4l2_subdev_pad_ops ms2r_pad_ops = {
	.enum_mbus_code	= ms2r_pad_enum_mbus_code,
	.get_fmt	= ms2r_pad_get_fmt,
	.set_fmt	= ms2r_pad_set_fmt,
};
#endif

/*
  * ***************** v4l2 subdev functions  *****************
*/
static const struct v4l2_subdev_core_ops ms2r_core_ops = {
	.init = ms2r_init,
	.load_fw = ms2r_load_firmware,
	.reset = ms2r_reset,
	.s_power = ms2r_sensor_isp_s_power,
	.g_ctrl = ms2r_g_ctrl,
	.s_ctrl = ms2r_s_ctrl,
	.g_ext_ctrls = ms2r_g_ext_ctrls,
};

static const struct v4l2_subdev_video_ops ms2r_video_ops = {
	.enum_framesizes = ms2r_enum_framesizes,
	.s_mbus_fmt = ms2r_s_fmt,
	.s_stream = ms2r_s_stream,
	.s_parm = ms2r_s_parm,
};

static const struct v4l2_subdev_ops ms2r_subdev_ops = {
	.core = &ms2r_core_ops,
#if defined(CONFIG_MEDIA_CONTROLLER) && defined(CONFIG_ARCH_EXYNOS5)
	.pad	= &ms2r_pad_ops,
#endif
	.video = &ms2r_video_ops,
};


static inline int ms2r_link_setup(struct media_entity *entity,
			    const struct media_pad *local,
			    const struct media_pad *remote, u32 flags)
{
	pr_debug("%s() +++++\n", __func__);
	return 0;
}
static const struct media_entity_operations ms2r_media_ops = {
	.link_setup = ms2r_link_setup,
};

static int ms2r_check_pdata(struct ms2r_platform_data *pdata)
{
	if (pdata == NULL || !pdata->init_gpio || !pdata->init_clock || 
		!pdata->set_isp_power || !pdata->set_sensor_power || !pdata->clock_enable) {
		pr_err("platform data is uncorrect.\n");
		return -EINVAL;
	}

	return 0;
}

static int ms2r_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct ms2r_state *state;
	struct v4l2_subdev *sd = NULL;	
	struct ms2r_platform_data *pdata = client->dev.platform_data;

	dev_info(&client->dev, "%s() entered\n", __func__);

	/* check platform data */
	ret = ms2r_check_pdata(pdata);
	if (ret) {
		dev_err(&client->dev, "platform data is incorrect\n");
		return ret;
	}
	
	/* alloc ms2r private data */
	state = devm_kzalloc(&client->dev,
			sizeof(struct ms2r_state), GFP_KERNEL);
	if (!state) {
		dev_err(&client->dev, "can not alloc memory.\n");
		return -ENOMEM;
	}

	/* init members */
	state->pdata = pdata;
	state->isp_power = false;
	state->sensor_power = false;
	state->debug = DEFAULT_DEBUG;
	state->fled_regulator = NULL;
	state->cap_mode = CAP_NORMAL_MODE;
	state->pre_flash_current = PRE_FLASH_CURRENT;
	state->full_flash_current = FULL_FLASH_CURRENT;
	/*
	* Init module type to max number of available modules
	*/
	state->module_type = MODULE_NUMBER;

	mutex_init(&state->mutex);
	init_completion(&state->completion);
	wake_lock_init(&state->wake_lock, WAKE_LOCK_SUSPEND, "ms2r");

	/* create a workqueue */
	state->wq = create_singlethread_workqueue("ms2r_wq");
	if(!state->wq){
		printk("Failed to setup workqueue - ms2r_wq \n");
		ret = -EINVAL;
		goto err_create_workqueue;
	}
	INIT_WORK(&state->work, ms2r_handle_work);

	/* register interrupt */
	ret = gpio_request(client->irq, "ms2r");
	if (ret) {
		dev_err(&client->dev, "can not request gpio (%d).\n", client->irq);
		ret = -EINVAL;
		goto err_gpio_request;
	}
	state->irq = gpio_to_irq(client->irq);
	if (state->irq < 0) {
		ret = -EINVAL;
		goto err_gpio_to_irq;
	}

	pr_debug("irq = %d, gpio=%d.\n", state->irq,  client->irq);

	ret = request_irq(state->irq, ms2r_irq_handler, IRQF_TRIGGER_RISING,
	    		client->name, state);
	if (ret) {
		dev_err(&client->dev, "request irq(%d) fail.\n", state->irq);
		goto err_gpio_to_irq;
	}

	/* register subdev */	
	sd = &state->sd;
	v4l2_i2c_subdev_init(sd, client, &ms2r_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#if defined(CONFIG_MEDIA_CONTROLLER)
	/*
	* Media framework support
	*/
	state->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_init(&sd->entity, 1, &state->pad, 0);
	if (ret<0) {
		dev_err(&client->dev, "%s(): can not init media entity!\n", __func__);
	}
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	sd->entity.ops = &ms2r_media_ops;
#endif

	/* init gpios */
	ret = pdata->init_gpio(&client->dev);
	if (ret) {
		dev_err(&client->dev, "platform init fail.\n");
		goto err_pdata_init;
	}

	/* init clocks */
	ret = pdata->init_clock(&client->dev);
	if (ret) {
		dev_err(&client->dev, "platform init fail.\n");
		goto err_pdata_init;
	}

	/* create sys interface */
	ret = sysfs_create_group(&client->dev.kobj, &ms2r_group);
	if (ret) {
		dev_err(&client->dev, "failed to create sysfs files\n");
		goto err_pdata_init;
	}

	pr_info("%s(), will allocate mem\n", __func__);
	ret = ms2r_allocate_fw_mem(sd);
	if (ret) {
		pr_err("%s(), allocate fw mem failed!!!\n", __func__);
		goto err_pdata_init;
	}

	state->prepared = true;
	ms2r_verdict_module_type(sd);

	dev_info(&client->dev, "ms2r has been probed\n");
	
	return 0;

err_pdata_init:
	free_irq(state->irq, state);
	v4l2_device_unregister_subdev(sd);
err_gpio_to_irq:
	gpio_free(client->irq);
err_gpio_request:
	destroy_workqueue(state->wq);
err_create_workqueue:
	devm_kfree(&client->dev, state);
	
	return ret;
}

static int ms2r_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ms2r_state *state = to_state(sd);

	sysfs_remove_group(&client->dev.kobj, &ms2r_group);
	v4l2_device_unregister_subdev(sd);
	free_irq(state->irq, state);
	gpio_free(client->irq);
	destroy_workqueue(state->wq);
	devm_kfree(&client->dev, state);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	ms2r_release_fw_mem(sd);
	dev_info(&client->dev, "ms2r has been removed\n");	
	
	return 0;
}

static const struct i2c_device_id ms2r_id[] = {
	{MS2R_DRIVER_NAME, 0},
	{},
};

static struct i2c_driver ms2r_i2c_driver = {
	.driver = {
		.name = MS2R_DRIVER_NAME,
	}, 
	.probe = ms2r_probe,
	.remove = ms2r_remove,
	.id_table = ms2r_id,
};

static int __init ms2r_module_init(void)
{
	return i2c_add_driver(&ms2r_i2c_driver);
}

static void __exit ms2r_module_exit(void)
{
	i2c_del_driver(&ms2r_i2c_driver);
}

module_init(ms2r_module_init);
module_exit(ms2r_module_exit);

MODULE_DESCRIPTION("Meizu ms2r ISP driver");
MODULE_AUTHOR("QuDao <qudaoman@126.com>");
MODULE_LICENSE("GPL");
