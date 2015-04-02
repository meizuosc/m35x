#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <linux/workqueue.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#ifdef CONFIG_VIDEO_SAMSUNG_V4L2
#include <linux/videodev2_samsung.h>
#endif
#include <linux/completion.h>
#include <linux/wakelock.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <mach/gpio-common.h>
#include <linux/vmalloc.h>

#include "ms2r.h"
#include "ms2r_regs.h"

extern int manipulate_camera_module(int len, void *buf, bool write);

#define MODULE_KEY "module="
/* firmware file */
#define MS2R_SIOLOADER_FW_VIA_I2C "MS2_SFW.bin"
#define BACK_CAMERA_FW_LITEON "RS_MS2_PF.bin"
#define BACK_CAMERA_FW_SHARP "RS_MS2_PF_SHARP.bin"
#define FRONT_CAMERA_FW "RS_MS2_PF_IMX132.bin"
#define MS2R_VERDICT_MODULE_FW_NAME "MS2_AP_BOOT.bin"
#define MS2R_CAMERA_FW_SIZE_VIA_SIO (0x78800) /*482KB*/
#define MS2R_SIOLOADER_FW_SIZE_VIA_I2C (0x1a6c)
#define MS2R_VERDICT_MODULE_FW_SIZE \
		(MS2R_SIOLOADER_FW_SIZE_VIA_I2C + MS2R_CAMERA_FW_SIZE_VIA_SIO)
#define MS2R_SECTION64_FRAME_SIZE (64 * 1024)  /* 64KB */
/*
* These value should be adjusted accordingly.
*/
#define DIV_RED_REG_VALUE 0x0247036d
/*
* This is destination mem addr when transfering fw via SIO
*/
#define SIO_TRANS_DEST_ADDR 0x01100000
#define SIO_TRANS_SIZE 0x00010000 /*multiples of 16*/
#define CAM_FW_START_ADDR 0x01100020
/*
* This is destination mem add of SIO loader.
*/
#define SIO_LOADER_ADDR 0x01000100
/*
* This is destination mem addr when transfering fw via I2C
*/
#define MS2R_SRAM_TOP_ADDR 0x01100000
#if 0
#define MS2R_FIRMWARE_FILE_NAME "isp_firmware.bin"
#define MS2R_FIRMWARE_FILE_SIZE (2016 * 1024)  /*abount 2 M*/
/* this size for 64KB download */
#define MS2R_SECTION64_WRITE_SIZE (1984 * 1024)
/* this size for 8KB download */
#define MS2R_SECTION8_WRITE_SIZE \
	MS2R_FIRMWARE_FILE_SIZE - MS2R_SECTION64_WRITE_SIZE

#define MS2R_SECTION32_FRAME_SIZE 		(32 * 1024)  /* 32KB */
#define MS2R_SECTION8_FRAME_SIZE 		(8 * 1024) /* 8KB */
#define MS2R_SECTION64_FLASH_ADDRESS 	0x10000000
#define MS2R_SECTION8_FLASH_ADDRESS 	0x101f0000
#define MS2R_INTERNAL_RAM_ADDRESS 		0x68000000

/* firmware address stored in file*/
#define NEW_FIRMWARE_HIGH_ADDRESS 		0x0016fffc
#define NEW_FIRMWARE_LOW_ADDRESS 		0x0016fffd

/****************************************/
/***********   factory test functions    **********/
/****************************************/

static bool ms2r_is_factory_test_mode(void)
{
//	return !!mx_is_factory_test_mode(MX_FACTORY_TEST_CAMERA);
	return false;
}

static void ms2r_factory_test_init(void)
{
	pr_debug("%s\n", __func__);
	
//	mx_set_factory_test_led(1);
}

static void ms2r_factory_test_success(void)
{
	int onoff = 0;

	pr_info("%s()\n", __func__);

	/*we don't return in factory test mode*/
	while (1) {
//		mx_set_factory_test_led(onoff);
		msleep(200);
		onoff = !onoff;
	}
}

static void ms2r_factory_test_fail(void)
{
	pr_err("%s()\n", __func__);
	
//	mx_set_factory_test_led(0);
}

/******************************************/

/*
  * set firmware status function
  * NONE, REQUESTING, LOADED_OK or LOADED_FAIL
  * If in factory test mode, should set factory test status
*/
static void ms2r_set_firmware_status(struct v4l2_subdev *sd, 
	enum firmware_status status)
{
	struct ms2r_state *state = to_state(sd);
	bool test_mode = ms2r_is_factory_test_mode();

	switch (status) {
	case FIRMWARE_NONE:
		break;
	case FIRMWARE_REQUESTING:
		if (test_mode) {
			state->fw_updated = false;
			ms2r_factory_test_init();
		}
		break;
	case FIRMWARE_CHECKED:
		if (test_mode) {
			wake_lock(&state->wake_lock);   /* don't sleep and return */
			if (state->fw_updated)
				ms2r_factory_test_success();
			else 
				ms2r_factory_test_fail();
		}
		break;
	case FIRMWARE_UPDATE_FAIL:
		if (test_mode) {
			wake_lock(&state->wake_lock);
			ms2r_factory_test_fail();
		}
		break;
	default:
		return;
	}

	pr_info("%s:status = %d\n", __func__, status);
	
	state->fw_status = status;
}

/*
  * download section data to ISP flash
*/
static int ms2r_download_section_firmware(struct v4l2_subdev *sd, 
	int flashaddress, 
	const char *data, 
	int size, 
	int interval)
{
	u32 val;
	int offset, i, ret, retry_count, count = 0;
	
	for(offset = 0; offset < size; offset += interval) {	
		/* set erase address */
		pr_info("set flash erase address:0x%x\n", flashaddress + offset);	

		ret = ms2r_w32(sd, FLASH_ADDRESS_REG, flashaddress + offset);
		CHECK_ERR(ret);
		
		/* send erase command */
		ret = ms2r_w8(sd, FLASH_ERASE_CMD_REG, 0x01);
		CHECK_ERR(ret);
		
		/* wait for erase finished */
		retry_count = 20;
		while (retry_count--) {  /* abount 300ms */
			ret = ms2r_r8(sd, FLASH_ERASE_CMD_REG, &val);
			CHECK_ERR(ret);	
			
			if(!val)
				break;
			msleep(50);
		}

		if (retry_count <= 0) {
			printk("%s: get FLASH_ERASE_CMD fail\n", __func__);	
			return -EINVAL;
		}

		/* set program bytes */
		ret = ms2r_w16(sd, FLASH_SIZE_REG, 
			(interval == MS2R_SECTION64_FRAME_SIZE)?0:interval);
		CHECK_ERR(ret);
		
		/* clear RAM */
		ret = ms2r_w8(sd, RAM_CLEAN_CMD_REG, 0x01);
		CHECK_ERR(ret);			
		
		/* wait for clear finished */
		retry_count = 100;
		while(retry_count--) {
			ret = ms2r_r8(sd, RAM_CLEAN_CMD_REG, &val);

			if(!val)
				break;
			msleep(10);
		}
		if (retry_count <= 0) {
			pr_err("%s: get FLASH_ERASE_CMD fail\n", __func__);	
			return -EINVAL;
		}

		pr_debug("begin write block data, block size=0x%x\n", interval);
		
		for(i = 0; i < interval; i += MS2R_SECTION8_FRAME_SIZE) {
			ret = ms2r_write_memory(sd, 
				MS2R_INTERNAL_RAM_ADDRESS + i, 
				data + offset + i, 
				MS2R_SECTION8_FRAME_SIZE);
			CHECK_ERR(ret);	
		}
		pr_debug("end write block data, block size=0x%x\n", interval);
		
		/* send flash write command */
		ret = ms2r_w8(sd, FLASH_WRITE_CMD_REG, 0x01);
		CHECK_ERR(ret);		
		
		/* wait for writing flash finished */
		retry_count = 20;
		while(retry_count--) {
			ret = ms2r_r8(sd, FLASH_WRITE_CMD_REG, &val);
			CHECK_ERR(ret);	
			
			if(!val)
				break;
			msleep(50);
		}

		if (retry_count <= 0) {
			pr_err("%s: get FLASH_ERASE_CMD fail\n", __func__);	
			return -EINVAL;
		}
		
		count++;
		pr_info("write count %d##########\n", count);	
	}

	return 0;
}

static int ms2r_download_firmware(struct v4l2_subdev *sd, 
	const char *data, int size)
{
	int ret;
	struct ms2r_state *state = to_state(sd);

	state->fw_buffer = vmalloc(MS2R_SECTION64_FRAME_SIZE + 8);
	if (!state->fw_buffer) return -ENOMEM;
	
	ret = ms2r_w8(sd, CAMERA_FLASH_TYPE_REG, 0x00);
	if (ret) {
		pr_err("%s: set CAMERA_FLASH_TYPE fail\n", __func__);	
		goto exit_free_memory;
	}

	/* download 64KB section */
	ret = ms2r_download_section_firmware(sd, 
		MS2R_SECTION64_FLASH_ADDRESS,
		data, 
		MS2R_SECTION64_WRITE_SIZE, 
		MS2R_SECTION64_FRAME_SIZE);
	if (ret) {
		pr_err("%s: download section 1 fail\n", __func__);	
		goto exit_free_memory;
	}

	/* download 8KB section */
	ret = ms2r_download_section_firmware(sd, 
		MS2R_SECTION8_FLASH_ADDRESS,
		data + MS2R_SECTION64_WRITE_SIZE, 
		MS2R_SECTION8_WRITE_SIZE, 
		MS2R_SECTION8_FRAME_SIZE);
	if (ret)
		pr_err("%s: download section 2 fail\n", __func__);	

exit_free_memory:
	vfree(state->fw_buffer);
	
	return ret;
}


/*
  * check the integrity of the firmware, should do after power on and finishing download firmware  
  * if checksum is ok, checksum value must be zero
*/
static int ms2r_get_checksum(struct v4l2_subdev *sd)
{
	u32 val, chsum;
	int	ret, acc, i, loop = 5;
	u32 chk_addr, chk_size, set_size;
	unsigned short ret_sum = 0;

	chk_addr = MS2R_SECTION64_FLASH_ADDRESS;
	chk_size = MS2R_FIRMWARE_FILE_SIZE;
	acc = 0x02;	/* 16bit unit */

	while (chk_size > 0) {
		if (chk_size >= MS2R_SECTION8_FRAME_SIZE)
			set_size = MS2R_SECTION8_FRAME_SIZE;
		else
			set_size = chk_size;
		
		/* set the start address */
		ret = ms2r_w32(sd, FLASH_ADDRESS_REG, chk_addr);
		CHECK_ERR(ret);
		
		/* set the size for checksum */
		ret = ms2r_w16(sd, FLASH_SIZE_REG, set_size);
		CHECK_ERR(ret);

		/* start to get the checksum */
		ret = ms2r_w8(sd, FLASH_CHKSUM_CMD_REG, acc);
		CHECK_ERR(ret);

		/* wait for getting the checksum */
		for(i = 0; i < loop; i++) {
			msleep(10);
			
			ret = ms2r_r8(sd, FLASH_CHKSUM_CMD_REG, &val);
			CHECK_ERR(ret);

			if (!val) { 
				ret = ms2r_r16(sd, FLASH_CHKSUM_RESULT_REG, &chsum);
				CHECK_ERR(ret);

				ret_sum += chsum;
				break;
			}
		}

		if (i == loop) {
			pr_err("%s(), get checksum failed at %d retries\n",
				__func__, loop);
			return -EINVAL;
		}
		
		/* next iteration */
		chk_addr += set_size;
		chk_size -= set_size;
	}
	
	return ret_sum;
}

/*
  * get current firmware version from ISP
*/
static int ms2r_get_firmware_version(struct v4l2_subdev *sd)
{
	int ret;
	u32 val;

	ret = ms2r_r16(sd, FIRMWARE_MINOR_VER_REG, &val);
	CHECK_ERR(ret);

	return val;
}

/*
  * get firmware version from file
*/
static int ms2r_get_new_firmware_version(const struct firmware *fw)
{
	
	return (fw->data[NEW_FIRMWARE_HIGH_ADDRESS] << 8) 
		| fw->data[NEW_FIRMWARE_LOW_ADDRESS];
}

/*
  * download firmware data to ISP flash
*/
static int ms2r_update_firmware(struct v4l2_subdev *sd, 
	const struct firmware *fw)
{
	int ret;
	struct ms2r_state *state = to_state(sd);

	pr_info("begin to download firmware\n");

	wake_lock(&state->wake_lock);

	ret = ms2r_set_power_clock(state, true);
	if (ret) goto exit_update;

	ret = ms2r_download_firmware(sd, fw->data, fw->size);
	if (ret) goto exit_update;
	
	if((ret = ms2r_get_checksum(sd))) {
		pr_err("%s: get checksum fail, checksum = %d\n",
			__func__, ret);
		ret = -EINVAL;
		goto exit_update;
	}

	if (ms2r_is_factory_test_mode()) state->fw_updated = true;
	
	pr_info("finish downloading firmware !\n");	

exit_update:
	wake_unlock(&state->wake_lock);
	if (state->isp_power) ms2r_set_power_clock(state, false);

	return ret;
}

/*
  * erase ISP firmware function
*/
int ms2r_erase_firmware(struct v4l2_subdev *sd)
{
	int ret;
	u32 val;
	struct ms2r_state *state = to_state(sd);

	/* if has power, return */
	if (state->isp_power) return -EINVAL;

	ret = ms2r_set_power_clock(state, true);
	if (ret) {
		pr_err("%s():power fail\n", __func__);
		return ret;
	}

	/* set flash type */
	ret = ms2r_w8(sd, CAMERA_FLASH_TYPE_REG, 0x00);
	if(ret) goto exit;
	
	/* set erase address */	
	ret =ms2r_w32(sd, FLASH_ADDRESS_REG, MS2R_SECTION64_FLASH_ADDRESS);
	if(ret) goto exit;
	
	/* send chip erase command */
	ret =ms2r_w8(sd, FLASH_ERASE_CMD_REG, 0x02);
	if(ret) goto exit;
	
	/* wait for erase finished */
	
	pr_info("%s: erase wait...\n", __func__);	
	
	while(1) {
		ret = ms2r_r8(sd, FLASH_ERASE_CMD_REG, &val);
		if(ret) goto exit;
		
		if(val == 0x00)
			break;
		msleep(50);
	}
	
	pr_info("%s: chip erase OK!\n", __func__);

exit:
	ms2r_set_power_clock(state, false);
	return ret;
}

/*
  * make a decision for update firmware, return true for update
  * get update decision sequence
  * (1) get new firmware version number from file
  * (2) if in factory test mode, return true, force to update 
  * (2) power on ISP
  * (3) do a checksum, if checksum fail, return true
  * (4) run firmware in order to get current current ISP firmware version
  * (5) compare old and new firmware version number, if not be equal, return true
  * (6) power off
*/
static bool ms2r_get_update_decision(struct v4l2_subdev *sd, const struct firmware *fw)
{
	int old_version, ret;
	struct ms2r_state *state = to_state(sd);
	bool decision = false;

	/* get new firmware version */
	state->fw_version = ms2r_get_new_firmware_version(fw);

	/* if in factory test mode, force to update */
	if (ms2r_is_factory_test_mode()) return true;

	/* power on before get update decision */
	ret = ms2r_set_power_clock(state, true);
	if (ret) goto exit_decision;
	
	msleep(100);  /* delay is necessary for the system boot on ? */

	/* if checksum fail means the firmware is not integrity */
	ret = ms2r_get_checksum(sd);
	if (ret) {
		pr_err("%s:do checksum error, checksum is %d\n",
			__func__, ret);
		decision = true;
		goto exit_decision;
	}

	/* after checksum, check the version num between old and new firmware */
	ret = ms2r_run_firmware(sd);
	if (ret) goto exit_decision;

#ifdef CONFIG_SKIP_CAMERA_UPDATE   /* used for eng */
	decision = false;
	goto exit_decision;
#else
	old_version = ms2r_get_firmware_version(sd);

	pr_info("firmware old version = 0x%x, new version = 0x%x\n", 
		old_version, state->fw_version);

	/* if old version is not equal to new version , we should update firmware */
	if (old_version != state->fw_version) {
		decision = true;
		goto exit_decision;
	}
#endif

exit_decision:
	if (state->isp_power) ms2r_set_power_clock(state, false);
	return decision;
}

static void ms2r_fw_request_complete_handler(const struct firmware *fw,
						  void *context)
{
	int ret = 0, retry_count = 3;
	struct v4l2_subdev *sd = (struct v4l2_subdev *)context;
	enum firmware_status fw_status = FIRMWARE_CHECKED;

	if (!fw) {
		pr_err("load ms2r firmware fail\n");
		goto exit_firmware;
	}

	/* check firmware size */
	if (fw->size != MS2R_FIRMWARE_FILE_SIZE) {
		pr_err("ms2r: firmware incorrect size(%d)\n", fw->size);
		goto exit_firmware;
	}	

	/* get update decision, retry 3 times */
	if (ms2r_get_update_decision(sd, fw)) {
retry:
		ret = ms2r_update_firmware(sd, fw);
		if (ret) {
			if (retry_count--) {
				goto retry;
			} else {
				fw_status = FIRMWARE_UPDATE_FAIL;
				goto exit_firmware;
			}
		}
	}

exit_firmware:
	ms2r_set_firmware_status(sd, fw_status);
}

int ms2r_load_firmware(struct v4l2_subdev *sd)
{
	int ret = 0;	
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	ms2r_set_firmware_status(sd, FIRMWARE_REQUESTING);

	ret = request_firmware_nowait(THIS_MODULE,
				      FW_ACTION_HOTPLUG,
				      MS2R_FIRMWARE_FILE_NAME,
				      &client->dev,
				      GFP_KERNEL,
				      sd,
				      ms2r_fw_request_complete_handler);
	if (ret) {
		dev_err(&client->dev, "could not load firmware (err=%d)\n", ret);
		ms2r_set_firmware_status(sd, FIRMWARE_CHECKED);
	}

	return ret;
}

/*
  * load firmware from sys interface: download_firmware
*/
int ms2r_load_firmware_sys(struct device *dev, struct v4l2_subdev *sd)
{
	int ret;
	struct ms2r_state *state = to_state(sd);
	const struct firmware *fw = NULL;
	enum firmware_status fw_status = state->fw_status;

	if (state->isp_power) {
		pr_err("camera has power on, please power off first\n");
		return -EINVAL;
	}

	ret = ms2r_set_power_clock(state, true);
	if (ret) {
		pr_err("%s():power fail", __func__);
		return -EINVAL;
	}

	wake_lock(&state->wake_lock);

	ms2r_set_firmware_status(sd, FIRMWARE_REQUESTING);
	
	ret = request_firmware(&fw, MS2R_FIRMWARE_FILE_NAME, dev);
	if (ret) {
		pr_err("%s() Upload failed. (file %s not found?)\n", __func__, MS2R_FIRMWARE_FILE_NAME);
		goto exit;
	}
	
	pr_info("%s() firmware read %d bytes.\n", __func__, fw->size);

	if (fw->size != MS2R_FIRMWARE_FILE_SIZE) {
		pr_err("ms2r: firmware incorrect size %d\n", fw->size);
		ret = -EINVAL;
		goto exit;
	}	

	state->fw_version = ms2r_get_new_firmware_version(fw);
	
	ret = ms2r_download_firmware(sd, fw->data, fw->size);
	if (ret) {
		pr_err("ms2r: download firmware fail\n");
		fw_status = FIRMWARE_UPDATE_FAIL;
		goto exit;
	}		

	fw_status = FIRMWARE_CHECKED;

exit:
	wake_unlock(&state->wake_lock);
	if (state->isp_power) ms2r_set_power_clock(state, false);
	ms2r_set_firmware_status(sd, fw_status);
	
	return ret;
}
#endif

static int ms2r_run_firmware(struct v4l2_subdev *sd)
{
	int ret;

	ms2r_prepare_wait(sd);

	ret = ms2r_w8(sd, CAMERA_START_CMD_REG, 0x02);
	CHECK_ERR(ret);

	ret = ms2r_wait_irq_and_check(sd, INT_MASK_MODE, 2000);
	if (ret) {
		pr_err("%s: wait timeout in %u miliseconds\n",
			__func__, 2000);
		return ret;
	}

	return 0;
}


/*
* @addr: the start address of ms2r memory to be validated.
* @data: the data to be validated.
* @size: the length of the data.
*/
static int ms2r_check_memory_content(struct v4l2_subdev *sd,
	u32 addr, const void *data, int size)
{
	u8 *read_buffer = NULL;
	int readed_size, remain_size, op_size;
	int ret = 0;
	int i;
	/*
	* Using I2C read mem
	*/
	pr_info("Read mem by I2C for check");
	read_buffer = kzalloc(0x2000, GFP_KERNEL);
	readed_size= 0;
	remain_size = size;
	while(remain_size > 0) {
		if (remain_size > 0x2000)
			op_size = 0x2000;
		else
			op_size = remain_size;

		ret = ms2r_read_memory(sd,
			addr + readed_size,
			read_buffer,
			op_size);
		if (ret) {
			pr_info("%s(), read memory failed\n", __func__);
			break;
		}

		pr_info("%s(), read 0x%x bytes via I2C, offset: 0x%x\n",
			__func__, op_size, readed_size);

		if (memcmp(read_buffer, data + readed_size, op_size)) {
			pr_err("%s(), Content is NOT equal\n", __func__);
			for (i = 0; i < op_size; i++) {
				pr_info("read_buffer is 0x%x, fw is 0x%x\n",
					read_buffer[i],
					*((u8 *)(data) + readed_size + i));
			}
			ret = -EINVAL;
			break;
		} else {
			pr_info("%s(), the same\n", __func__);
		}

		readed_size += op_size;
		remain_size -= op_size;
	}
	kfree(read_buffer);
	return ret;
}


/*
* SIO Loader must running when using SIO.
*/
static int ms2r_send_camera_firmware_sio(struct v4l2_subdev *sd,
	const void *data, int size)
{
	int ret = 0;
	const void *spi_buf = data;
	struct ms2r_state *state = to_state(sd);

	if (!ms2r_is_isppower(state)) {
		pr_err("%s(), Need to Power on ISP first!!!\n", __func__);
		return -EINVAL;
	}

	if (!size || !data) {
		pr_err("%s(), Err: size:%d, data: %p!!!\n",
			__func__, size, data);
		return -EINVAL;
	}

	msleep(5);

	/*
	* Set LSI TYPE
	*/
	ret = ms2r_w8(sd, LSI_MODE_REG, 0x01);
	CHECK_ERR(ret);
	/*
	* Set PLL value
	*/
	ret = ms2r_w32(sd, PLL1DIV_VALUE_REG, DIV_RED_REG_VALUE);
	CHECK_ERR(ret);
	/*
	* SIO mode start
	*/
	ret = ms2r_w8(sd, SIO_TRANS_SWITCH_REG, ACTIVE_SIO);
	CHECK_ERR(ret);
	msleep(6);
	/*
	* Set destination address for SIO transmmission
	*/
	ret = ms2r_w32(sd, DATA_RAM_ADDR_REG, SIO_TRANS_DEST_ADDR);
	CHECK_ERR(ret);
	/*
	* Set SIO transmmission size.
	*/
	ret = ms2r_w32(sd, DATA_TRANS_SIZE_REG, size);
	CHECK_ERR(ret);

	ret = ms2r_spi_write(spi_buf, size);
	if (ret < 0) {
		pr_err("%s(), SPI transfer failed: %d\n", __func__, ret);
		return ret;
	} else {
		pr_info("%s(), SPI transfer OK!\n", __func__);
	}

	/*
	* Set start address of camera firmware.
	*/
	ret = ms2r_w32(sd, CAMERA_START_ADDRESS_REG, CAM_FW_START_ADDR);
	CHECK_ERR(ret);
	/*
	* Start camera firmware
	*/
	//ret = ms2r_w8(sd, CAMERA_START_CMD_REG, 0x02);
	//CHECK_ERR(ret);
	ret = ms2r_run_firmware(sd);
	if (ret < 0) {
		pr_err("%s(), run firmware failed!!! ret: %d\n",
			__func__, ret);
	}

	return ret;
}

static int ms2r_send_sioloader(struct v4l2_subdev *sd, const void *data, int size)
{
	int ret;
	int remain_size, op_size, written_size;
	/*
	* each I2C operation is 8KB
	*/
	int per_i2c_operation_size = 0x2000;
	struct ms2r_state *state = to_state(sd);

	if (!ms2r_is_isppower(state)) {
		pr_err("%s(), Need to Power on ISP first!!!\n", __func__);
		return -EINVAL;
	}

	if (!size || !data) {
		pr_err("%s(), Err: size:%d, data: %p!!!\n",
			__func__, size, data);
		return -EINVAL;
	}

	written_size = 0;
	remain_size = size;
	pr_info("%s(), start SIO loader sending\n", __func__);
	while(remain_size > 0) {
		if (remain_size > per_i2c_operation_size)
			op_size = per_i2c_operation_size;
		else
			op_size = remain_size;

		ret = ms2r_write_memory(sd,
			SIO_LOADER_ADDR + written_size,
			data + written_size,
			op_size);
		CHECK_ERR(ret);
		pr_info("%s(), write 0x%x bytes via I2C, offset: 0x%x\n",
			__func__, op_size, written_size);

		written_size += op_size;
		remain_size -= op_size;
	}

	/*
	* Set start adress of SIO Loader program
	*/
	ret = ms2r_w32(sd, CAMERA_START_ADDRESS_REG, SIO_LOADER_ADDR);
	CHECK_ERR(ret);
	/*
	* Start SIO Loader program
	*/
	ret = ms2r_w8(sd, CAMERA_START_CMD_REG, 0x02);
	CHECK_ERR(ret);

	return 0;
}

static int ms2r_send_camera_firmware_i2c(struct v4l2_subdev *sd,
	const void *data, int size)
{
	int ret;
	int remain_size, op_size, written_size;
	/*
	* each I2C operation is 8KB
	*/
	int per_i2c_operation_size = 0x2000;
	struct ms2r_state *state = to_state(sd);

	if (!ms2r_is_isppower(state)) {
		pr_err("%s(), Need to Power on ISP first!!!\n", __func__);
		return -EINVAL;
	}

	if (!size || !data) {
		pr_err("%s(), Err: size:%d, data: %p!!!\n",
			__func__, size, data);
		return -EINVAL;
	}

	/*
	* Set PLL value
	*/
	ret = ms2r_w32(sd, PLL1DIV_VALUE_REG, DIV_RED_REG_VALUE);
	CHECK_ERR(ret);
	/*
	* Active I2C Write to RAM.
	*/
	ret = ms2r_w8(sd, SIO_TRANS_SWITCH_REG, ACTIVE_I2C);
	CHECK_ERR(ret);

	/*
	* Since I find failure may occur randomly so
	* I add it here, however the doc don't mention it.
	*/
	msleep(3);

	written_size = 0;
	remain_size = size;
	while(remain_size > 0) {
		if (remain_size > per_i2c_operation_size)
			op_size = per_i2c_operation_size;
		else
			op_size = remain_size;

		ret = ms2r_write_memory(sd,
			MS2R_SRAM_TOP_ADDR + written_size,
			data + written_size,
			op_size);
		CHECK_ERR(ret);
		pr_info("%s(), write 0x%x bytes via I2C, offset: 0x%x\n",
			__func__, op_size, written_size);

		written_size += op_size;
		remain_size -= op_size;
	}

	/*
	* Set start adress of camera firmware
	*/
	ret = ms2r_w32(sd, CAMERA_START_ADDRESS_REG, CAM_FW_START_ADDR);
	CHECK_ERR(ret);
	/*
	* Start Camera firmware
	*/
	ret = ms2r_w8(sd, CAMERA_START_CMD_REG, 0x02);
	CHECK_ERR(ret);

	return 0;
}

int ms2r_allocate_fw_mem(struct v4l2_subdev *sd)
{
	int ret = 0;
	struct ms2r_state *state = to_state(sd);

	state->fw_buf_i2c_size = MS2R_SIOLOADER_FW_SIZE_VIA_I2C;
	state->fw_buf_spi_size = MS2R_CAMERA_FW_SIZE_VIA_SIO;

	state->fw_buffer = vmalloc(MS2R_SECTION64_FRAME_SIZE + 8);
	if (!state->fw_buffer) {
		ret = -ENOMEM;
		pr_err("%s(), allocate fw_buffer failed\n", __func__);
		return ret;
	}

	state->fw_buf_i2c = kzalloc(state->fw_buf_i2c_size , GFP_KERNEL);
	if (!state->fw_buf_i2c) {
		ret = -ENOMEM;
		pr_err("%s(), allocate fw_buf_i2c failed\n", __func__);
		goto flag1;
	}

	state->fw_buf_spi = kzalloc(state->fw_buf_spi_size, GFP_KERNEL);
	if (!state->fw_buf_spi) {
		ret = -ENOMEM;
		pr_err("%s(), allocate fw_buf_spi failed\n", __func__);
		goto flag2;
	}

	pr_info("%s(), ok!\n", __func__);
	return 0;

flag2:
	kfree(state->fw_buf_i2c);
	state->fw_buf_i2c = NULL;
flag1:
	vfree(state->fw_buffer);
	state->fw_buffer = NULL;

	return ret;
}

void ms2r_release_fw_mem(struct v4l2_subdev *sd)
{
	struct ms2r_state *state = to_state(sd);

	if (state->fw_buffer) {
		vfree(state->fw_buffer);
		state->fw_buffer = NULL;
	}
	if (state->fw_buf_i2c) {
		kfree(state->fw_buf_i2c);
		state->fw_buf_i2c = NULL;
	}
	if (state->fw_buf_spi) {
		kfree(state->fw_buf_spi);
		state->fw_buf_spi = NULL;
	}
}

int ms2r_request_fw(struct v4l2_subdev *sd, int camera_id)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ms2r_state *state = to_state(sd);
	const struct firmware *fw_sioloader = NULL;
	const struct firmware *fw = NULL;
	const char *camera_fw_name = NULL;

	if (!state->fw_buf_i2c || !state->fw_buf_spi) {
		pr_err("%s(), Err: !!!Compulsory memory can NOT be found !!!\n",
			__func__);
		return -ENOMEM;
	}

	if (BACK_CAMERA == camera_id) {
		switch (state->module_type) {
		case MODULE_REAR_LITEON:
			pr_info("%s(), Utilize liteon module\n", __func__);
			camera_fw_name = BACK_CAMERA_FW_LITEON;
			break;
		case MODULE_REAR_SHARP:
			pr_info("%s(), Utilize sharp module\n", __func__);
			camera_fw_name = BACK_CAMERA_FW_SHARP;
			break;
		default:
			#if 0
			ret = -EINVAL;
			pr_err("%s(), Illegal module type %d!!!\n",
				__func__, state->module_type);
			return ret;
			#else
			pr_warn("\n%s(), !!!unknown module type: %d !!!\n"
			"Utilize liteon as default \n", __func__, state->module_type);
			camera_fw_name = BACK_CAMERA_FW_LITEON;
			break;
			#endif
		}
	} else {
		camera_fw_name = FRONT_CAMERA_FW;
	}

	pr_info("%s(), request %s for camera %d\n", __func__,
		camera_fw_name, camera_id);

	if (state->fw_buf_i2c_size != MS2R_SIOLOADER_FW_SIZE_VIA_I2C) {
		ret = request_firmware(&fw_sioloader, MS2R_SIOLOADER_FW_VIA_I2C,
			&client->dev);
		if (ret) {
			pr_err("%s() Upload failed. (file %s not found?)\n", __func__,
				MS2R_SIOLOADER_FW_VIA_I2C);
			return ret;
		}

		pr_info("%s(), update SIO Loader\n", __func__);
		memcpy(state->fw_buf_i2c, fw_sioloader->data, fw_sioloader->size);
		state->fw_buf_i2c_size = fw_sioloader->size;
		release_firmware(fw_sioloader);
	}

	ret = request_firmware(&fw, camera_fw_name, &client->dev);
	if (ret) {
		pr_err("%s() Upload failed. (file %s not found?)\n", __func__,
			camera_fw_name);
		goto flag1;
	}
	memcpy(state->fw_buf_spi, fw->data, fw->size);
	state->fw_buf_spi_size = fw->size;
	release_firmware(fw);
	pr_debug("%s(), OK!\n", __func__);
flag1:
	return ret;
}

int ms2r_load_firmware(struct v4l2_subdev *sd)
{
	int ret = 0;
	struct ms2r_state *state = to_state(sd);

	if ((!state->fw_buf_i2c) || (!state->fw_buf_spi)
		|| (!state->fw_buffer)) {
		pr_err("%s(), !!!Firmware is NOT present!!!\n", __func__);
		return -EINVAL;
	}

#ifdef USING_I2C_TRANS_CAM_FW
	/*
	* Currently using I2C, will convert to SIO.
	*/
	ret = ms2r_send_camera_firmware_i2c(sd, state->fw_buf_i2c,
		state->fw_buf_i2c_size);
	if (ret) {
		pr_err("%s() Send camera firmware via I2C failed!\n", __func__);
		return ret;
	}
#else
	ret = ms2r_send_sioloader(sd, state->fw_buf_i2c,
		state->fw_buf_i2c_size);
	if (ret) {
		pr_err("%s() Send SIO Loader via I2C failed!\n", __func__);
		return ret;
	}

	ret = ms2r_send_camera_firmware_sio(sd, state->fw_buf_spi,
		state->fw_buf_spi_size);
	if (ret) {
		pr_err("%s() Send camera firmware via SPI failed!\n", __func__);
		return ret;
	}
#endif

	pr_info("%s(), Download fw OK!", __func__);
	return 0;
}

static int ms2r_parse_file(struct v4l2_subdev *sd)
{
	int ret = 0;
	struct ms2r_state *state = to_state(sd);
	char file_content[strlen(MODULE_KEY) + 2];
	int read_size = sizeof(file_content) - 1;
	int module_type = -1;

	memset(file_content, 0, sizeof(file_content));
	ret = manipulate_camera_module(read_size, file_content, false);
	if (ret) {
		pr_err("%s(), read module from file failed\n", __func__);
		return ret;
	}

	if (!strstr(file_content, MODULE_KEY)) {
		pr_err("%s(), err!parse file failed, the file_content is %s\n",
			__func__, file_content);
		return -EINVAL;
	}

	ret = sscanf(file_content, MODULE_KEY"%d", &module_type);
	if (ret != 1) {
		pr_err("%s(), err!parse content: %s.\n", __func__, file_content);
		return -EIO;
	}

	if (module_type != MODULE_REAR_LITEON
		&& module_type != MODULE_REAR_SHARP) {
		pr_err("%s(), unkown module type(%d) err\n", __func__, module_type);
		return -EINVAL;
	} else {
		pr_info("%s(), module %d found sucess\n", __func__, module_type);
		state->module_type = module_type;
	}

	return 0;
}

int ms2r_save_file(struct v4l2_subdev *sd)
{
	int ret = 0;
	struct ms2r_state *state = to_state(sd);
	char file_content[strlen(MODULE_KEY) + 2];

	if (state->module_type != MODULE_REAR_LITEON
		&& state->module_type != MODULE_REAR_SHARP) {
		pr_err("%s(), Err! Can NOT save unkown module type %d to file\n",
		__func__, state->module_type);
		return -EINVAL;
	}

	memset(file_content, 0, sizeof(file_content));
	snprintf(file_content, sizeof(file_content), MODULE_KEY"%d", state->module_type);

	pr_info("%s(), write %s to file\n", __func__, file_content);
	ret = manipulate_camera_module(sizeof(file_content), file_content, true);
	if (ret) {
		pr_err("%s(), write module to file failed\n", __func__);
		return ret;
	}

	return 0;
}

#if 1
int ms2r_retrieve_module(struct v4l2_subdev *sd, bool mani_file)
{
	int ret = 0;
	u32 val;
	struct ms2r_state *state = to_state(sd);

	if (/*mani_file*/0) {
		if (!ms2r_parse_file(sd)) {
			pr_info("%s(), retrieve module type %d from file success\n",
				__func__, state->module_type);
			return 0;
		} else {
			pr_info("%s(), retrieve module type from file failed, try ISP now\n",
				__func__);
		}
	}

	/*
	* If file system does not contain the module id information
	* then retreive it from ISP.
	*/
	state->cam_id = BACK_CAMERA;
	ret = ms2r_sensor_isp_s_power(sd, 1);
	if (ret) {
		pr_err("%s(), power on failed\n", __func__);
		return ret;
	}

	/*
	* load firmware
	*/
	ret = ms2r_load_firmware(sd);
	if (ret) {
		pr_err("%s(), load fw failed!!!\n", __func__);
		goto flag1;
	}

	/*
	* Read module ID and save it.
	*/
	ret = ms2r_r8(sd, MODULE_ID_REG, &val);
	if (ret) {
		pr_err("%s(), read module ID failed!!!\n", __func__);
		goto flag1;
	}

#if 0
	if (MODULE_ID_REAR_LITEON == val) {
		pr_info("%s(), liteon module(0x%x) found for rear camera\n",
			__func__, val);
		state->module_type = MODULE_REAR_LITEON;
	} else {
		pr_info("%s(), sharp module(0x%x) found for rear camera\n",
			__func__, val);
		state->module_type = MODULE_REAR_SHARP;
	}
#else
	if (MODULE_ID_REAR_SHARP == val) {
		pr_info("%s(), sharp module(0x%x) found for rear camera\n",
			__func__, val);
		state->module_type = MODULE_REAR_SHARP;
	} else {
		pr_info("%s(), liteon module(0x%x) found for rear camera\n",
			__func__, val);
		state->module_type = MODULE_REAR_LITEON;
	}
#endif

	if (mani_file) {
		if (ms2r_save_file(sd)) {
			pr_err("%s(), save module type to file failed\n", __func__);
		}
	}

flag1:
	/*
	* Disable ISP power && clock
	*/
	ret = ms2r_sensor_isp_s_power(sd, 0);
	return ret;
}
#else
/*
* There is no sharp module currently so no verdict is needed.
*/
int ms2r_retrieve_module(struct v4l2_subdev *sd, bool mani_file)
{
	int ret;
	struct ms2r_state *state = to_state(sd);
	char file_content[strlen(MODULE_KEY) + 2];

	pr_info("%s(), always utilize liteon module since no sharp module is shipping\n",
		__func__);
	state->module_type = MODULE_REAR_LITEON;

	pr_info("%s(), clear the camera file\n", __func__);
	memset(file_content, 0, sizeof(file_content));
	ret = manipulate_camera_module(sizeof(file_content), file_content, true);
	if (ret) {
		pr_err("%s(), clear camera file failed\n", __func__);
		return ret;
	}
	return 0;
}
#endif


static void ms2r_verdict_module_type_complete_handler(const struct firmware *fw,
						  void *context)
{
	struct v4l2_subdev *sd = (struct v4l2_subdev *)context;
	struct ms2r_state *state = to_state(sd);

	if (!fw) {
		pr_err("%s(), load verdict module fw fail\n", __func__);
		return;
	}
	state->prepared = false;
	/* check firmware size */
	if (fw->size != MS2R_VERDICT_MODULE_FW_SIZE) {
		pr_err("%s(), firmware incorrect size(%d)\n", __func__, fw->size);
		goto flag1;
	}

	memcpy(state->fw_buf_i2c, fw->data, state->fw_buf_i2c_size);
	memcpy(state->fw_buf_spi, fw->data + state->fw_buf_i2c_size,
		state->fw_buf_spi_size);

	pr_info("%s(), will retrieve module\n", __func__);
	ms2r_retrieve_module(sd, true);

flag1:
	release_firmware(fw);
	state->fw_buf_i2c_size = 0;
	state->prepared = true;
	return;
}

int ms2r_verdict_module_type(struct v4l2_subdev *sd)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	ret = request_firmware_nowait(THIS_MODULE,
				      FW_ACTION_HOTPLUG,
				      MS2R_VERDICT_MODULE_FW_NAME,
				      &client->dev,
				      GFP_KERNEL,
				      sd,
				      ms2r_verdict_module_type_complete_handler);
	if (ret) {
		dev_err(&client->dev, "could not verdict module type (err=%d)\n", ret);
	}

	return ret;
}

int ms2r_request_module_fw(struct v4l2_subdev *sd)
{
	int ret = 0;
	struct ms2r_state *state = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	const struct firmware *fw = NULL;

	ret = request_firmware(&fw, MS2R_VERDICT_MODULE_FW_NAME, &client->dev);
	if (ret) {
		pr_err("%s() Upload failed. (file %s not found?)\n", __func__,
			MS2R_VERDICT_MODULE_FW_NAME);
		return ret;
	}

	if (fw->size != MS2R_VERDICT_MODULE_FW_SIZE) {
		pr_err("%s(), firmware incorrect size(%d)\n", __func__, fw->size);
		ret = -EINVAL;
		goto exit;
	}

	state->fw_buf_i2c_size = MS2R_SIOLOADER_FW_SIZE_VIA_I2C;
	state->fw_buf_spi_size = MS2R_CAMERA_FW_SIZE_VIA_SIO;

	memcpy(state->fw_buf_i2c, fw->data, state->fw_buf_i2c_size);
	memcpy(state->fw_buf_spi, fw->data + state->fw_buf_i2c_size,
		state->fw_buf_spi_size);

exit:
	release_firmware(fw);
	return ret;
}
