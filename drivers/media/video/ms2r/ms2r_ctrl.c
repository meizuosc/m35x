#include <linux/i2c.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/ms2r.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/delay.h>
#ifdef CONFIG_VIDEO_SAMSUNG_V4L2
#include <linux/videodev2_samsung.h>
#endif
/*
* The following 2 are added by qudao
* to replace linux/videodev2_samsung.h
*/
#include <linux/videodev2_exynos_camera.h> /*Resemble videodev2_samsung.h*/
#include <linux/videodev2_exynos_media.h>
#include <linux/completion.h>
#include <linux/wakelock.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <asm/mach-types.h>

#include "ms2r.h"
#include "ms2r_regs.h"
#include "ms2r_ctrl.h"

#define MIN_ZOOM_POS 		0x01
#define MIN_ZOOM_STEP	0x02

#define CHECK_USERSET(v) do {\
	if (state->userset.v == ctrl->value) \
		return 0;\
	} while (0);

#define SET_USERSET(v) do {\
	state->userset.v = ctrl->value;\
	} while (0);

#define CHECK_CTRL_VAL(array) do {\
	if (ctrl->value < 0 || ctrl->value >= ARRAY_SIZE(array)) \
		return -EINVAL;\
	} while (0);
	
#define MAX_JPEG_SIZE (5 * 1024 * 1024)   /* 5 M */
#define MAX_SMILE_PERCENT 100

static u8 ms2r_wb_regs[MS2R_WB_MAX] = {
	[MS2R_WB_INCANDESCENT] = AWB_INCANDESCENT,
	[MS2R_WB_FLUORESCENT_HIGH] = AWB_FLUORESCENT_HIGH,
	[MS2R_WB_FLUORESCENT_LOW] = AWB_FLUORESCENT_LOW,
	[MS2R_WB_SUNNY] = AWB_DAYLIGHT,
	[MS2R_WB_CLOUDY] = AWB_CLOUDY,
	[MS2R_WB_SHADE] = AWB_SHADE,
	[MS2R_WB_HORIZON] = AWB_HORIZON,
};

static u8 ms2r_brightness_regs[MS2R_EV_MAX] = {
	[MS2R_EV_MINUS_2] = EV_M2,
	[MS2R_EV_MINUS_1_5] = EV_M1_5,
	[MS2R_EV_MINUS_1] = EV_M1,
	[MS2R_EV_MINUS_0_5] = EV_M0_5,
	[MS2R_EV_DEFAULT] = EV_00,
	[MS2R_EV_PLUS_0_5] = EV_P0_5,
	[MS2R_EV_PLUS_1] = EV_P1,
	[MS2R_EV_PLUS_1_5] = EV_P1_5,
	[MS2R_EV_PLUS_2] = EV_P2,
};

static u8 ms2r_scene_regs[MS2R_SCENE_MAX] = {
	[MS2R_SCENE_NONE] = SCENE_OFF,
	[MS2R_SCENE_AUTO] = SCENE_AUTO,
	[MS2R_SCENE_PORTRAIT] = SCENE_PORTRAIT,
	[MS2R_SCENE_LANDSCAPE] = SCENE_LANDSCAPE,
	[MS2R_SCENE_SPORTS] = SCENE_SPORT,
	[MS2R_SCENE_NIGHTSHOT] = SCENE_NIGHT,
	[MS2R_SCENE_SUNSET] = SCENE_SUNSET,
	[MS2R_SCENE_MICRO] = SCENE_MARCO,
	[MS2R_SCENE_CHARACTER] = SCENE_CHARACTER,
};

/*
* This is diffrent from M6MO
*/
static u8 ms2r_af_window_regs[MS2R_FOCUS_MAX] = {
	[MS2R_FOCUS_AUTO] = NORMAL_AF,
	[MS2R_FOCUS_MACRO] = NORMAL_AF,
	[MS2R_FOCUS_MACRO_CAF] = NORMAL_AF,
	[MS2R_FOCUS_FD] = NORMAL_AF,
	[MS2R_FOCUS_FD_CAF] = NORMAL_AF,
	[MS2R_FOCUS_TOUCH] = TOUCH_AF,
	[MS2R_FOCUS_TOUCH_CAF] = TOUCH_AF,
	[MS2R_FOCUS_AUTO_CAF] = NORMAL_AF,
};

/*
* MS2R do NOT mention this.
*/
static u8 ms2r_af_scan_mode_regs[MS2R_FOCUS_MAX] = {
	[MS2R_FOCUS_AUTO] = AF_FAST_SCAN,
	[MS2R_FOCUS_MACRO] = AF_FAST_SCAN,
	[MS2R_FOCUS_MACRO_CAF] = AF_CONTINUOUS_FOCUS,
	[MS2R_FOCUS_FD] = AF_FAST_SCAN,
	[MS2R_FOCUS_FD_CAF] = AF_CONTINUOUS_FOCUS,
	[MS2R_FOCUS_TOUCH] = AF_FAST_SCAN,
	[MS2R_FOCUS_TOUCH_CAF] = AF_CONTINUOUS_FOCUS,
	[MS2R_FOCUS_AUTO_CAF] = AF_CONTINUOUS_FOCUS,
};

static u8 ms2r_auto_focus_regs[AUTO_FOCUS_MAX] = {
	[AUTO_FOCUS_OFF] = AF_STOP,
	[AUTO_FOCUS_ON] = AF_START,
};

static u8 ms2r_iso_regs[MS2R_ISO_MAX] = {
	[MS2R_ISO_AUTO] = ISO_SEL_AUTO,
	[MS2R_ISO_50] = ISO_SEL_50,
	[MS2R_ISO_100] = ISO_SEL_100,
	[MS2R_ISO_200] = ISO_SEL_200,
	[MS2R_ISO_400] = ISO_SEL_400,
	[MS2R_ISO_800] = ISO_SEL_800,
	[MS2R_ISO_1600] = ISO_SEL_1600,
	[MS2R_ISO_3200] = ISO_SEL_3200,
};

static u8 ms2r_wdr_regs[MS2R_WDR_MAX] = {
	[MS2R_WDR_LOW] = PART_WDR_LOW,
	[MS2R_WDR_MIDDLE] = PART_WDR_MIDDLE,
	[MS2R_WDR_HIGH] = PART_WDR_HIGH,
};

static u8 ms2r_flash_regs[MS2R_FLASH_MAX] = {
	[MS2R_FLASH_OFF] = LED_FLASH_OFF,
	[MS2R_FLASH_AUTO] = LED_FLASH_AUTO,
	[MS2R_FLASH_ON] = LED_FLASH_ON,
};

/* MAIN_MIRROR, MAIN_REVERSE, MAIN_ROTATION, PREVIEW_ROTATION, THUMB_ROTATION */
static u8 ms2r_rotation_regs[MS2R_ROTATE_MAX][5] = {
	[MS2R_ROTATE_0] = {0x00, 0x00, 0x00, 0x00, 0x00},
	[MS2R_ROTATE_90] = {0x00, 0x00, 0X01, 0X01, 0X01},
	[MS2R_ROTATE_180] = {0x01, 0x01, 0x00, 0x00, 0x00},
	[MS2R_ROTATE_270] = {0x00, 0x00, 0x02, 0x02, 0x02},
};

/*
* unit is ms
*/
static int slow_shut_delay[] = {
	[M_SLOW_SHUT_AUTO] = 0,
	[M_SLOW_SHUT_1_8] = 0,
	[M_SLOW_SHUT_1_4] = 0,
	[M_SLOW_SHUT_1_2] = 0,
	[M_SLOW_SHUT_1] = 0,
	[M_SLOW_SHUT_1D5] = 0,
};

/*************************************************/
/***********  panorama capture functions  *************/
/*************************************************/
/*
  * set panorama capture start and stop function
  * panorama capture sequence
  * (1) lock ae and awb
  * (1) set panorama mode on
  * (2) enable all interrupt
  * (3) set panorama capture start
  * (4) wait every picture finishing interrupt
  * (5) wait all picture finishing interrupt
  * (6) wait all picture stitch interrupt
*/
int ms2r_set_panorama_capture(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ms2r_state *state = to_state(sd);
	int i, ret;

	if (ctrl->value) {
		if (state->cap_mode == CAP_PANORAMA_MODE) return 0;

		state->cap_mode = CAP_PANORAMA_MODE;
		state->pano.counter = 0;
		state->pano.stitch_status = PANORAMA_STITCH_INIT;
		/* initialize all panorama picture info */
		for (i = 0; i < PANORAMA_MAX_PICTURE; i++) {
			state->pano.pictures[i].status = PANORAMA_UNKNOWN_ERR;
			state->pano.pictures[i].extra = 0;
		}
		ms2r_prepare_wait(sd);

		/* set multi-cap regs */
		do {
			struct ms2r_reg regs[] = {
				{I2C_8BIT, AE_LOCK_REG, 0x01},
				{I2C_8BIT, AWB_LOCK_REG, 0x01},
				{I2C_8BIT, CAP_MODE_REG, CAP_MODE_PARORAMA},
				{I2C_8BIT, PANO_CTRL_REG, 0x00},
				{I2C_8BIT, INT_ENABLE_REG, 0xff},
				{I2C_8BIT, INT_ROOR_ENABLE_REG, 0x01},
				{I2C_8BIT, PANO_CAP_READY_REG, PANO_CAP_READY_START},  /* start capture */
			};
			
			ret = ms2r_write_regs(sd, regs, ARRAY_SIZE(regs));
			if (ret) {
				state->cap_mode = CAP_NORMAL_MODE; /* recovery normal mode */
				return ret;
			}
		} while (0);

		pr_info("%s():start panorama capture.\n", __func__);
	} else {
		pr_info("%s():stop panorama capture.\n", __func__);
		ret = ms2r_w8(sd, CAP_MODE_REG, CAP_MODE_NORMAL);
		CHECK_ERR(ret);
		
		if (state->cap_mode == CAP_PANORAMA_MODE)
			state->cap_mode = CAP_NORMAL_MODE;
	}
	
	return 0;
}

static void set_pano_picture_status(struct ms2r_state *state, int index,
	enum panorama_picture_status status)
{
	struct v4l2_subdev *sd = &state->sd;
	int ret;

	/*lock first*/
	mutex_lock(&state->mutex);
	
	switch (status) {
	case PANORAMA_SUCCESS:
	case PANORAMA_FATAL_ERR:
	case PANORAMA_UNKNOWN_ERR:
	case PANORAMA_COMPLETE:
		break;
	case PANORAMA_RETRY_ERR:
		do {
			int x, y;
			ret = ms2r_r16(sd, PANO_OFFSETX_H_REG, &x);
			if (ret) goto exit_unlock_mutex;
			ret = ms2r_r16(sd, PANO_OFFSETY_H_REG, &y);
			if (ret) goto exit_unlock_mutex;
			state->pano.pictures[index].extra = ((x & 0xffff) << 16) | (y & 0xffff);
		} while (0);
		break;
	case PANORAMA_BIG_ERR:
		do {
			int val;
			ret = ms2r_r8(sd, PANO_ERROR_NO_REG, &val);
			if (ret) goto exit_unlock_mutex;
			pr_info("%s(), big error found, err: %d\n", __func__, (signed char)val);
			state->pano.pictures[index].extra = val;
		} while (0);
		break;
	default:
		return;
	}

	state->pano.pictures[index].status = status;

exit_unlock_mutex:
	mutex_unlock(&state->mutex);
}

static void set_pano_stitch_status(struct ms2r_state *state)
{
	int ret;
	u32 val;
	
	ret = ms2r_r8(&state->sd, PANO_ERROR_NO_REG,  &val);
	if(ret) return;

	mutex_lock(&state->mutex);
	
	if (!val) {
		state->pano.stitch_status = PANORAMA_STITCH_OK;
		printk("stitching all images success\n");
	} else {
		state->pano.stitch_status = PANORAMA_STITCH_FAIL;
		printk("stitching all images fail, 0x%02x reg is 0x%02x.\n", PANO_ERROR_NO_REG, val);
	}
	
	mutex_unlock(&state->mutex);
}

void ms2r_handle_panorama_cap(struct ms2r_state *state, int irq_status)
{
	if (irq_status & INT_MASK_ZOOM) {   /* stitch interrupt */
		set_pano_stitch_status(state);
		complete(&state->completion);
	} else if (irq_status & INT_MASK_MODE) {  /* finish all capture interrupt */
		set_pano_picture_status(state, state->pano.counter++, PANORAMA_COMPLETE);
		pr_info("Finish panorama capture!\n");		
	} else if (irq_status & INT_MASK_CAPTURE) {   /* finish one capture interrupt */
		if (state->pano.counter >= PANORAMA_MAX_PICTURE) {
			set_pano_picture_status(state, state->pano.counter, PANORAMA_FATAL_ERR);
			pr_err("***panorama picture counter is overflow***\n");
		} else {
			set_pano_picture_status(state, state->pano.counter++, PANORAMA_SUCCESS);
			pr_info("Valid %d-th image captured.\n", state->pano.counter);
		}
	} else if (irq_status & INT_MASK_FRAMESYNC) {  /* mirror error */
		set_pano_picture_status(state, state->pano.counter, PANORAMA_RETRY_ERR);
	} else if (irq_status & INT_MASK_FD) {   /* big error */
		set_pano_picture_status(state, state->pano.counter, PANORAMA_BIG_ERR);
	} else if (irq_status & INT_MASK_SOUND) {  /* fatal error */
		set_pano_picture_status(state, state->pano.counter, PANORAMA_FATAL_ERR);
		pr_err("Fatal error found in captured image!\n");
	} else {   /* unknown error */
		set_pano_picture_status(state, state->pano.counter, PANORAMA_UNKNOWN_ERR);
		pr_err("%s():other interrupt status in panorama capture process!0x%02x\n", 
			__func__, irq_status);
	}
}

static int ms2r_wait_panorama_stitch(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ms2r_state *state = to_state(sd);

	mutex_lock(&state->mutex);

	if (state->pano.stitch_status == PANORAMA_STITCH_INIT) {
		int ret;
		mutex_unlock(&state->mutex);
		ret = ms2r_wait_irq(sd, 8000);
		if (ret) return ret;
		mutex_lock(&state->mutex);
	}

	ctrl->value = state->pano.stitch_status;
	
	mutex_unlock(&state->mutex);
	
	return 0;
}

/*
  * terminate panorama capture manual
*/
static int ms2r_terminate_panorama(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	return ms2r_w8(sd, PANO_CAP_READY_REG, PANO_CAP_READY_STOP);
}

/*
  * get current panorama picture information, should be modified in future
*/
static int ms2rs_set_cur_panorama_info(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ms2r_state *state = to_state(sd);
	int index = ctrl->value;
	int ret;

	if (index < 1 || index > PANORAMA_MAX_PICTURE) {
		pr_err("wrong pan num: %d!!!\n", index);
		return -EINVAL;
	}

	index--;
	mutex_lock(&state->mutex);
	ret = state->pano.pictures[index].status;
	ctrl->value = state->pano.pictures[index].extra;
	mutex_unlock(&state->mutex);
	
	return ret;
}

static int ms2r_get_pan_direction(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int err, value;

	err = ms2r_r8(sd, PANO_CAP_DIRECTION_REG, &value);
	if (!err) ctrl->value = value;
	return err;
}

/*************************************************/
/***********  multi capture functions  ****************/
/*************************************************/
/*
  * set multi capture start or stop function
  * multi capture seqence
  * (1) set multi cap number 
  * (2) set auto multi cap mode
  * (3) enable interrupt
  * (4) start capture
  * (5) wait every multi cap picture interrupt until finish capture
  * (6) get every multi cap pictures
*/
static int ms2r_set_multi_capture(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{

	struct ms2r_state *state = to_state(sd);
	int ret = 0;

	if (ctrl->value) {
		int i;
		
		pr_info("multi capture start, number is %d\n", ctrl->value);

		/* check ctrl value */
		if (ctrl->value > MULTI_CAP_MAX_PICTURE) return -EINVAL;
		
		if (state->cap_mode == CAP_MULTI_CAP_MODE) return 0;

		state->multi_cap.numbers = ctrl->value;  /* set multi-cap picture number */
		state->multi_cap.counter = 0;  /* reset counter */
		state->multi_cap.ready = MULTI_CAP_READY_INIT;
		/* initialize all multi-picture information */
		for (i = 0; i < state->multi_cap.numbers; i++) 
			state->multi_cap.pictures[i].status = MULTI_CAP_INIT;
		ms2r_prepare_wait(sd);

		/* set multi-cap regs */
		ret = ms2r_w8(sd, CAP_FRM_COUNT_REG, state->multi_cap.numbers);
		CHECK_ERR(ret);
		ret = ms2r_w8(sd, CAP_MODE_REG, CAP_MODE_AUTO_MULTICAP);
		CHECK_ERR(ret);
		ret = ms2r_set_sys_mode(sd, CAPTURE_MODE);
		if (ret) return ret;
		state->cap_mode = CAP_MULTI_CAP_MODE;
	} else {
		pr_info("%s(), switch back to normal cap mode\n", __func__);
		ret = ms2r_w8(sd, CAP_MODE_REG, CAP_MODE_NORMAL);
		CHECK_ERR(ret);

		if (state->cap_mode == CAP_MULTI_CAP_MODE)
			state->cap_mode = CAP_NORMAL_MODE;  /* recover capture normal mode */
	}

	return 0;
}

/*
  * handle multi capture interrupt function
*/
void ms2r_handle_multi_cap(struct ms2r_state *state, int irq_status)
{
	struct v4l2_subdev *sd = &state->sd;
	
	pr_info("%s: irq status = 0x%02x\n", __func__, irq_status);

	/* has wait all multi capture pictures, wait ready flag */
	if (state->multi_cap.counter >= state->multi_cap.numbers) {
		mutex_lock(&state->mutex);
		if (irq_status & INT_MASK_CAPTURE)
			state->multi_cap.ready = MULTI_CAP_READY_SUCCESS;
		else 
			state->multi_cap.ready = MULTI_CAP_READY_FAIL;
		mutex_unlock(&state->mutex);
		complete(&state->completion);
	} else if ((irq_status & INT_MASK_SOUND) && 
				(irq_status & INT_MASK_FRAMESYNC)) {  /* wait the i-th picture success */
		mutex_lock(&state->mutex);
		state->multi_cap.pictures[state->multi_cap.counter++].status 
			= MULTI_CAP_SUCCESS;
		mutex_unlock(&state->mutex);

		pr_info("wait %d-th multi capture picture success.\n", state->multi_cap.counter);
		
		ms2r_enable_root_irq(sd);
		complete(&state->completion);
	} else if (irq_status & INT_MASK_MODE) {
		pr_info("change to multi-cap mode success !");
	}
}

/*
  * wait finishing the i-th multi-cap
  * if it has been finished, it's status is not equal to MULTI_CAP_INIT
*/
static int ms2r_wait_multi_capture_picture(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int i, ret;
	struct ms2r_state *state = to_state(sd);
	
	if (ctrl->value  < 1 || ctrl->value > state->multi_cap.numbers) {
		pr_err("%s():ctrl->value %d is out of range[1-%d].", 
			__func__, ctrl->value, state->multi_cap.numbers);
		return -EINVAL;
	}
	
	i = ctrl->value - 1;  /* get the multi cap index */

	mutex_lock(&state->mutex);

	/* 
	  * if this picture has not come, try to wait
	  * else the picture has come
	*/
	if (state->multi_cap.pictures[i].status == MULTI_CAP_INIT) {
		mutex_unlock(&state->mutex);  /* free the lock first */
		ret = ms2r_wait_irq(sd, WAIT_TIMEOUT);
		if (ret) return ret;
		mutex_lock(&state->mutex);
	}

	/* set the picture status */
	ctrl->value = state->multi_cap.pictures[i].status;

	pr_info("%s: status = %d\n", __func__, state->multi_cap.pictures[i].status);

	mutex_unlock(&state->mutex);

	return 0;
}

static int ms2r_wait_multi_capture_ready(struct v4l2_subdev *sd, 
	struct v4l2_control *ctrl)
{
	int ret;
	struct ms2r_state *state = to_state(sd);

	mutex_lock(&state->mutex);

	/* this multi-cap picture is not ready */
	if (state->multi_cap.ready == MULTI_CAP_READY_INIT) {
		mutex_unlock(&state->mutex);
		ret = ms2r_wait_irq(sd, WAIT_TIMEOUT);
		if (ret) return ret;
		mutex_lock(&state->mutex);
	}

	/* get the status */
	ctrl->value = state->multi_cap.ready;

	pr_info("%s: ready = %d\n", __func__, state->multi_cap.ready);

	mutex_unlock(&state->mutex);
	
	return 0;
}

/*
  * get the i-th mul-cap picture functions
  * sequence
  * (1) enable root irq
  * (2) select frame image
  * (3) wait irq 
  * (4) enable root irq
  * (5) set transfer start
  * (6) wait irq
*/
static int ms2r_get_multi_capture_picture(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret;
	struct ms2r_state *state = to_state(sd);

	pr_info("%s(), get %d picture of multi-cap\n", __func__, ctrl->value);

	/* check ctrl value */
	if (ctrl->value < 1 || ctrl->value > state->multi_cap.numbers) {
		pr_err("%s():ctrl->value %d is out of range[1-%d]", 
			__func__, ctrl->value, state->multi_cap.numbers);
		return -EINVAL;
	}

	ms2r_prepare_wait(sd);
	ret = ms2r_enable_root_irq(sd);
	CHECK_ERR(ret);
	ret = ms2r_w8(sd, CAP_SEL_FRAME_MAIN_REG, ctrl->value);
	CHECK_ERR(ret);
	ret = ms2r_wait_irq_and_check(sd, INT_MASK_CAPTURE, WAIT_TIMEOUT);
	if (ret) return ret;
	
	ret = ms2r_enable_root_irq(sd);
	CHECK_ERR(ret);
	ret = ms2r_w8(sd, CAP_TRANSFER_START_REG, CAP_TRANSFER_START);
	CHECK_ERR(ret);
	ret = ms2r_wait_irq_and_check(sd, INT_MASK_CAPTURE, WAIT_TIMEOUT);
	if (ret) return ret;

	/*set ctrl value to 0 means get picture success*/
	ctrl->value = 0;
	
	return 0;
}

/*****************************************/
/********  face detection capture ************/
/*****************************************/
/*
  * enable smile face detection function
  * smile face capture sequence
  * (1) set face detection on
  * (2) set smile level [0~100]
  * (3) wait smile detection interrupt
*/
static int ms2r_set_smile_face_detection(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret;
	struct ms2r_state *state = to_state(sd);

	if (ctrl->value < 0 || ctrl->value > 100) {
		pr_err("wrong smile level value!please set[0-100]\n");
		return -EINVAL;
	}

	if (ctrl->value) {
		pr_info("%s():smile face detection enable.\n", __func__);
		state->smile_cap.detection = SMILE_NO_DETECTION;
		state->cap_mode = CAP_SMILE_CAP_MODE;
		ret = ms2r_w8(sd, FACE_DETECT_MAX_REG, 0x0b);
		CHECK_ERR(ret);
		ret = ms2r_w8(sd, FACE_DETECT_CTL_REG, SMILE_FACE_DETECT_ON);
		CHECK_ERR(ret);
		ret = ms2r_w8(sd, FD_SMILE_LEVEL_THRS_REG, ctrl->value);
		CHECK_ERR(ret);
	} else {
		pr_info("%s():smile face detection disable.\n", __func__);
		ret = ms2r_w8(sd, FD_SMILE_LEVEL_THRS_REG, 0x00);
		CHECK_ERR(ret);
		ret = ms2r_w8(sd, FACE_DETECT_CTL_REG, SMILE_FACE_DETECT_OFF);
		CHECK_ERR(ret);
		state->cap_mode = CAP_NORMAL_MODE;
	}
	
	return 0;
}

/*
  * handle smile interrupt function
*/
void ms2r_handle_smile_cap(struct ms2r_state *state, int irq_status)
{
	if (irq_status & INT_MASK_FRAMESYNC) {
		pr_info("*********%s():smile detection********\n", __func__);
		state->smile_cap.detection = SMILE_DETECTION;
		complete(&state->completion);
	}
}

static int ms2r_get_smile_face_detection(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ms2r_state *state = to_state(sd);
	
	ctrl->value = state->smile_cap.detection;
	
	return 0;
}

static int ms2r_get_face_detected_num(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int val, ret;

	ret = ms2r_r8(sd, FACE_DETECT_NUM_REG, &val);
	CHECK_ERR(ret);

	ctrl->value = val;
	
	return ret;
}

static int ms2r_set_face_selection(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int val, ret;
	int retry_count = 100;
	
	ret =  ms2r_w8(sd, FACE_DETECT_READ_SEL_REG, ctrl->value);
	CHECK_ERR(ret);

	while (retry_count--) {
		ret = ms2r_r8(sd, FACE_DETECT_READ_SEL_REG, &val);
		if (!ret && val == 0xff) 
			return 0;
		msleep(5);
	}

	return -EINVAL;
}

static int ms2r_get_selected_face_location(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret, x, y;

	ret = ms2r_r16(sd, FACE_DETECT_X_LOCATION_REG, &x);
	CHECK_ERR(ret);
	ret = ms2r_r16(sd, FACE_DETECT_Y_LOCATION_REG, &y);
	CHECK_ERR(ret);
	
	ctrl->value = ((x & 0xffff) << 16) | (y & 0xffff);
	
	return 0;
}

static int ms2r_get_selected_face_size(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret, w, h;

	ret = ms2r_r16(sd, FACE_DETECT_FRAME_WIDTH_REG, &w);
	CHECK_ERR(ret);
	ret = ms2r_r16(sd, FACE_DETECT_FRAME_HEIGH_REG, &h);
	CHECK_ERR(ret);
	
	ctrl->value = ((w & 0xffff) << 16) | (h & 0xffff);
	
	return 0;
}

/*
  * set face detection direction, 0, 90, 180 or 270 angle
*/
static int ms2r_set_face_detection_direction(struct v4l2_subdev *sd, 
	struct v4l2_control *ctrl)
{
	return ms2r_w8(sd, FACE_DETECT_DIRECTION_REG, ctrl->value);
}

/*
* Auto bracket capture
*/
static int ms2r_set_auto_bracket_capture(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret;
	struct ms2r_state *state = to_state(sd);
	pr_info("%s(), ctrl->value is %d\n", __func__, ctrl->value);

	if (ctrl->value >= AUTO_BRACKET_EV_MIN &&
		ctrl->value <= AUTO_BRACKET_EV_MAX) {
		if (state->cap_mode == CAP_AUTO_BRACKET_MODE) {
			pr_warn("%s(), cap mode has already been set to auto bracket\n", __func__);
			return 0;
		}

		ret = ms2r_w8(sd, AUTO_BRACKET_EV_REG, ctrl->value);

		ret = ms2r_w8(sd, CAP_MODE_REG, CAP_MODE_AUTO_BRACKET);
		CHECK_ERR(ret);

		state->cap_mode = CAP_AUTO_BRACKET_MODE;
	} else {
		pr_info("%s(), switch back to normal cap mode\n", __func__);
		ret = ms2r_w8(sd, CAP_MODE_REG, CAP_MODE_NORMAL);
		CHECK_ERR(ret);
		state->cap_mode = CAP_NORMAL_MODE;
	}

	return 0;
}

/*
* handle auto bracket capture interrupt
*/
void ms2r_handle_auto_bracket_cap(struct ms2r_state *state, int irq_status)
{
	pr_info("%s: irq status = 0x%02x\n", __func__, irq_status);
	complete(&state->completion);  /* just to wake up any waiters */
}

static int ms2r_get_jpeg_mem_size(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	ctrl->value = MAX_JPEG_SIZE;
	
	return 0;
}

static int ms2r_set_touch_af_ae_en(struct v4l2_subdev *sd, int val)
{
	int ret;

	if (val != TOUCH_AF_AE_DISABLE &&
		val != TOUCH_AF_AE_ENABLE) {
		pr_err("%s(), ctrl->value :%d is invalid!!!\n", __func__, val);
		return -EINVAL;
	}

	ret = ms2r_w8(sd, TOUCH_AF_AE_EN_REG, val);
	CHECK_ERR(ret);

	return 0;
}

#if 0
static void ms2r_get_af_touch_coordinate(int val, int *x, int *y)
{
	int row, col;

	row = val / AF_TOUCH_ROW;
	col = val - row * AF_TOUCH_ROW;
	
	if(row > AF_TOUCH_ROW - 1)
		row = AF_TOUCH_ROW - 1;
	if(row < 0)
		row = 0;
	if(col > AF_TOUCH_COL - 1)
		col = AF_TOUCH_COL - 1;
	if(col < 0)
		col = 0;

	pr_info("%s(): row %d, col %d.\n", __func__, row, col);
	
	/*x stand for col, y stand for row*/
	*x = col * AF_TOUCH_WIDTH;
	*y = row * AF_TOUCH_HEIGHT;
}

static int ms2r_set_focus_position(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	struct ms2r_state *state = to_state(sd);
	int x, y;

	CHECK_USERSET(focus_position);

	ms2r_get_af_touch_coordinate(ctrl->value, &x, &y);
	
	ret = ms2r_w16(sd, AF_TOUCH_WIN_W_REG, AF_TOUCH_WIDTH);
	CHECK_ERR(ret);
	ret = ms2r_w16(sd, AF_TOUCH_WIN_H_REG, AF_TOUCH_HEIGHT);	
	CHECK_ERR(ret);
	
	ret = ms2r_w16(sd, AF_TOUCH_WIN_X_REG, x);
	CHECK_ERR(ret);
	ret = ms2r_w16(sd, AF_TOUCH_WIN_Y_REG, y);
	CHECK_ERR(ret);

	SET_USERSET(focus_position);
	
	return 0;
}
#else
static int ms2r_set_touch_position(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	int val;
	//struct ms2r_state *state = to_state(sd);
	struct touch_area data;

	if (copy_from_user(&data, (void __user *)ctrl->value, sizeof(data))) {
		pr_err("%s(), retrieve touch area failed!!!\n", __func__);
		return -EFAULT;
	}

	pr_info("%s(), x: 0x%x, y: 0x%x, width: 0x%x, height: 0x%x\n",
		__func__, data.x, data.y, data.width, data.height);

	/*
	* Stop Touch AF/AE
	*/
	ms2r_set_touch_af_ae_en(sd, TOUCH_AF_AE_DISABLE);

	ret = ms2r_w16(sd, TOUCH_AF_AE_X_LOCATION_REG, data.x);
	CHECK_ERR(ret);
	ret = ms2r_w16(sd, TOUCH_AF_AE_Y_LOCATION_REG, data.y);
	CHECK_ERR(ret);

	ret = ms2r_w16(sd, TOUCH_AF_AE_FRAME_WIDTH_REG, data.width);
	CHECK_ERR(ret);
	ret = ms2r_w16(sd, TOUCH_AF_AE_FRAME_HEIGHT_REG, data.height);
	CHECK_ERR(ret);

	/*
	* Start Touch AF/AE
	*/
	ms2r_set_touch_af_ae_en(sd, TOUCH_AF_AE_ENABLE);

	/*
	* Check whether Touch AF/AE is starting
	*/
	ret = ms2r_r8(sd, TOUCH_AF_AE_EN_REG, &val);
	CHECK_ERR(ret);
	if (TOUCH_AF_AE_DISABLE == val) {
		pr_err("%s(), Touch AF/AE failed!!!(Touch area invalid???)\n", __func__);
		return -EINVAL;
	} else {
		pr_err("%s(), Touch AF/AE starts OK\n", __func__);
	}

	return 0;
}
#endif

static int ms2r_set_wb_preset(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ms2r_state *state = to_state(sd);
	int ret = 0;

	CHECK_CTRL_VAL(ms2r_wb_regs);
	CHECK_USERSET(manual_wb);

	/* set manual wb first */
	if(ctrl->value == MS2R_WB_AUTO) {
		ret = ms2r_w8(sd, AWB_MODE_REG, AWB_AUTO);
	} else {
		ret = ms2r_w8(sd, AWB_MODE_REG, AWB_MANUAL);	
		CHECK_ERR(ret);
		ret = ms2r_w8(sd, AWB_MANUAL_REG, ms2r_wb_regs[ctrl->value]);
		CHECK_ERR(ret);
	}
	
	SET_USERSET(manual_wb);
	
	return 0;
}

static int ms2r_set_image_brightness(struct v4l2_subdev *sd, 
	struct v4l2_control *ctrl)
{
	struct ms2r_state *state = to_state(sd);
	int ret = 0;

	CHECK_CTRL_VAL(ms2r_brightness_regs);
	CHECK_USERSET(brightness);

	pr_info("%s(), wirte 0x%x to register 0x%x\n", __func__,
		ms2r_brightness_regs[ctrl->value], EV_BIAS_REG);

	ret = ms2r_w8(sd, EV_BIAS_REG, ms2r_brightness_regs[ctrl->value]);
	CHECK_ERR(ret);

	SET_USERSET(brightness);
	
	return 0;
}

static int ms2r_set_scenemode(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;

#if 0
	CHECK_CTRL_VAL(ms2r_scene_regs);
	CHECK_USERSET(scene);

	ret = ms2r_w8(sd, SCENE_MODE_REG, ms2r_scene_regs[ctrl->value]);
	CHECK_ERR(ret);

	SET_USERSET(scene);
#else
	pr_info("%s(), ctrl->value is %d\n", __func__, ctrl->value);
	if (ctrl->value != TEXT_MODE_AUTO && ctrl->value != TEXT_MODE_TEXT) {
		pr_err("%s(), sorry, scene mode is NOT supported yet\n", __func__);
		return -EINVAL;
	}
	ret = ms2r_w8(sd, TEXT_MODE_REG, ctrl->value);
	CHECK_ERR(ret);
	return 0;
#endif
 
	return 0;
}

/*
* Validated
* This starts or stops focus.
*/
static int ms2r_set_focus(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;

	CHECK_CTRL_VAL(ms2r_auto_focus_regs);

	pr_info("%s(), write reg 0x%x to %d\n", __func__, AF_START_REG,
		ms2r_auto_focus_regs[ctrl->value]);
	ret = ms2r_w8(sd, AF_START_REG, ms2r_auto_focus_regs[ctrl->value]);
	CHECK_ERR(ret);

	return 0;
}

#if 0
static int ms2r_set_focus_mode(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	struct ms2r_state *state = to_state(sd);

	pr_info("%s(), ctrl->value is %d\n", __func__, ctrl->value);
	CHECK_USERSET(af_mode);

	if (ctrl->value < MS2R_FOCUS_AUTO || ctrl->value >= MS2R_FOCUS_MAX)
		return -EINVAL;
	/*
	* Should be trimmed.
	*/
	//ret = ms2r_w8(sd, AF_MODE_REG, AF_NORMAL);

	ret = ms2r_w8(sd, AF_WINDOW_REG, ms2r_af_window_regs[ctrl->value]);
	pr_info("%s(), ctrl->value %d, write %d to reg 0x%x\n", __func__,
		ctrl->value, ms2r_af_window_regs[ctrl->value], AF_WINDOW_REG);
	CHECK_ERR(ret);
	/*
	* MS2R does NOT mention this.
	*/
	ret = ms2r_w8(sd, AF_SCAN_MODE_REG, ms2r_af_scan_mode_regs[ctrl->value]);
	CHECK_ERR(ret);

	SET_USERSET(af_mode);

	return 0;
}
#else
static int ms2r_set_focus_mode(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;

	pr_info("%s(), ctrl->value is %d\n", __func__, ctrl->value);

	if (ctrl->value < MS2R_FOCUS_AUTO || ctrl->value >= MS2R_FOCUS_MAX)
		return -EINVAL;

#if 0
	/*
	* Set AF Window to use touch window frame
	*/
	if (MS2R_FOCUS_TOUCH == ctrl->value ||
		MS2R_FOCUS_TOUCH_CAF == ctrl->value) {
		pr_info("%s(), ctrl->value %d, write %d to reg 0x%x\n", __func__,
			ctrl->value, ms2r_af_window_regs[ctrl->value], AF_WINDOW_REG);
		ret = ms2r_w8(sd, AF_WINDOW_REG, ms2r_af_window_regs[ctrl->value]);
		CHECK_ERR(ret);
	} else {
		pr_info("%s(), Currently do nothing for focus mode: %d\n", __func__, ctrl->value);
	}
#else
	if (ctrl->value == MS2R_FOCUS_MACRO_CAF || ctrl->value == MS2R_FOCUS_FD_CAF ||
		ctrl->value == MS2R_FOCUS_TOUCH_CAF || ctrl->value == MS2R_FOCUS_AUTO_CAF) {
		pr_info("%s(), CAF mode\n", __func__);
		ret = ms2r_w8(sd, AF_MODE_REG, AF_CAF);
	} else {
		pr_info("%s(), non CAF mode\n", __func__);
		ret = ms2r_w8(sd, AF_MODE_REG, AF_NORMAL);
	}
	pr_info("%s(), ctrl->value %d, write %d to reg 0x%04x\n", __func__,
		ctrl->value, ms2r_af_window_regs[ctrl->value], AF_WINDOW_REG);
	ret = ms2r_w8(sd, AF_WINDOW_REG, ms2r_af_window_regs[ctrl->value]);
	CHECK_ERR(ret);
#endif

	return 0;
}

#endif

static int ms2r_set_af_range_mode(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret;
	if (ctrl->value < AF_NORMAL_RANGE ||
		ctrl->value > AF_FULL_RANGE) {
		pr_err("%s(), invalid input value: %d !!!\n", __func__, ctrl->value);
		return -EINVAL;
	}

	pr_info("%s(), set af range to %d\n", __func__, ctrl->value);
	ret = ms2r_w8(sd, AF_RANGE_MODE_REG, ctrl->value);
	CHECK_ERR(ret);
	return 0;
}


static int ms2r_set_touch_ae_strength(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret;

	if (ctrl->value < TOUCH_AE_STREN_VALUE_OFF ||
		ctrl->value > TOUCH_AE_STREN_VALUE_MAX) {
		pr_err("%s(), invalid input %d !!!\n", __func__, ctrl->value);
		return -EINVAL;
	}

	pr_info("%s(), set touch ae strength to %d\n", __func__, ctrl->value);
	ret = ms2r_w8(sd, TOUCH_AE_STRENGTH_VALUE_REG, ctrl->value);
	CHECK_ERR(ret);

	return 0;
}

static int ms2r_set_zoom_level(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	struct ms2r_state *state = to_state(sd);

	CHECK_USERSET(zoom_level);

	if (ctrl->value < MS2R_ZL_1)
		ctrl->value = MS2R_ZL_1;
	if(ctrl->value > MS2R_ZL_70)
		ctrl->value = MS2R_ZL_70;
	
	ret = ms2r_w8(sd, ZOOM_POSITOIN_REG, ctrl->value);
	CHECK_ERR(ret);

	SET_USERSET(zoom_level);
	
	return 0;
}

static int ms2r_set_iso(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	struct ms2r_state *state = to_state(sd);

	CHECK_CTRL_VAL(ms2r_iso_regs);
	CHECK_USERSET(iso);

	ret = ms2r_w8(sd, ISO_SEL_REG, ms2r_iso_regs[ctrl->value]);
	CHECK_ERR(ret);

	SET_USERSET(iso);
	
	return 0;
}

static int ms2r_set_wdr(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	struct ms2r_state *state = to_state(sd);

	CHECK_CTRL_VAL(ms2r_wdr_regs);
	CHECK_USERSET(wdr);

	pr_info("%s(), set New WDR to %d, the last is %d", __func__,
		ctrl->value, state->userset.wdr);

	if (ctrl->value == MS2R_WDR_OFF)
		ret = ms2r_w8(sd, PART_WDR_EN_REG, PART_WDR_OFF);
	else if (ctrl->value == MS2R_WDR_AUTO)
		ret = ms2r_w8(sd, PART_WDR_EN_REG, PART_WDR_AUTO);
	else {
		ret = ms2r_w8(sd, PART_WDR_EN_REG, PART_WDR_ON);
		CHECK_ERR(ret);
		ret = ms2r_w8(sd, PART_WDR_LVL_REG, ms2r_wdr_regs[ctrl->value]);
	}
	CHECK_ERR(ret);

	SET_USERSET(wdr);
	
	return 0;
}

int ms2r_set_flash_current(struct ms2r_state *state, int cur)
{
	if ((!state->fled_regulator) || (cur > MAX_FLASH_CURRENT)) 
		return -EINVAL;

	pr_info("%s(), set current to %d\n", __func__, cur);
	return regulator_set_current_limit(state->fled_regulator, cur, MAX_FLASH_CURRENT);
}

static int ms2r_enable_flash_led(struct ms2r_state *state, struct v4l2_control *ctrl) 
{
	int ret = 0;
	
	if (ctrl->value != MS2R_FLASH_OFF) {
		/* if the first time to open flash led */
		if (!state->fled_regulator) {
			pr_info("%s(), turn on flash led!\n", __func__);

			/*
			* select flash led category is needless in ms2r
			*/

			/* open the flash voltage */
			state->fled_regulator = regulator_get(NULL, FLASH_LED_NAME);
			if (IS_ERR(state->fled_regulator)) {
				pr_err("%s()->%d:regulator get fail !!\n", __FUNCTION__, __LINE__);
				state->fled_regulator = NULL;
				return -ENODEV;
			}
		
			ret = ms2r_set_flash_current(state, state->pre_flash_current);
			if (ret) goto err_exit;
			ret = regulator_enable(state->fled_regulator);
			if (ret) goto err_exit;
		}
	} else {
		if (state->fled_regulator) {
			pr_info("%s(), turn off flash led!\n", __func__);
			
			ret = regulator_disable(state->fled_regulator);	
			if (ret) goto err_exit;
			regulator_put(state->fled_regulator);
			state->fled_regulator = NULL;
		}
	}
	
	return 0;
	
err_exit:
	regulator_put(state->fled_regulator);
	state->fled_regulator = NULL;
	return ret;
}

static int ms2r_set_flash_mode(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	struct ms2r_state *state = to_state(sd);

	pr_info("%s(), ctrl->value is %d\n", __func__, ctrl->value);
	CHECK_CTRL_VAL(ms2r_flash_regs);
	CHECK_USERSET(flash_mode);

	pr_info("%s(), write %d to reg 0x%x\n", __func__,
		ms2r_flash_regs[ctrl->value], LED_FLASH_CONTROL_REG);
	ret = ms2r_w8(sd, LED_FLASH_CONTROL_REG, ms2r_flash_regs[ctrl->value]);
	CHECK_ERR(ret);
	
	ret = ms2r_enable_flash_led(state, ctrl);
	if (ret) return ret;

	SET_USERSET(flash_mode);
	
	return 0;
}

static int ms2r_set_rotation(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ms2r_state *state = to_state(sd);
	int ret = 0;

	struct ms2r_reg regs[] = {
		{I2C_8BIT, MAIN_MIRROR_REG, ms2r_rotation_regs[ctrl->value][0]},
		{I2C_8BIT, MAIN_REVERSE_REG, ms2r_rotation_regs[ctrl->value][1]},
		{I2C_8BIT, MAIN_ROTATION_REG, ms2r_rotation_regs[ctrl->value][2]},
		{I2C_8BIT, PREVIEW_ROTATION_REG, ms2r_rotation_regs[ctrl->value][3]},
		{I2C_8BIT, THUMB_ROTATION_REG, ms2r_rotation_regs[ctrl->value][4]},
	};

	CHECK_CTRL_VAL(ms2r_rotation_regs);
	CHECK_USERSET(rotation);

	ret = ms2r_write_regs(sd, regs, ARRAY_SIZE(regs));
	CHECK_ERR(ret);

	SET_USERSET(rotation);
	
	return 0;
}

static int ms2r_get_scene_ev(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	return ms2r_r16(sd, SCENE_EV_REG, &ctrl->value);
}

/*
* get auto focus result (0-operating, 1-success, 2-fail, 3-stopped at edge)
* Validated.
*/
static int ms2r_get_auto_focus_result(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret;
	ret = ms2r_r8(sd, AF_RESULT_REG, &ctrl->value);
	CHECK_ERR(ret);
	pr_info("%s(), read reg 0x%x is 0x%x\n", __func__, AF_RESULT_REG, ctrl->value);
	return ret;
}

static int ms2r_transfer_capture_data(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	struct ms2r_state *state = to_state(sd);
	pr_info("%s(), ctr->value is %d\n", __func__, ctrl->value);

	switch (ctrl->value) {
	case 1:
		/*
		* MS2R does NOT need to wait interrupt.
		*/
		ret = ms2r_w8(sd, CAP_TRANSFER_START_REG, CAP_TRANSFER_START);
		CHECK_ERR(ret);
		break;
	case 2:
		ret = ms2r_w8(sd, CAP_TRANSFER_START_REG, CAP_TRANSFER_STOP);
		CHECK_ERR(ret);
		/* recovery flash to pre current */
		if (state->userset.flash_mode != MS2R_FLASH_OFF)
			ms2r_set_flash_current(state, state->pre_flash_current);

		if (state->slow_shut_delay) {
			pr_info("%s(), sleep for %d ms\n", __func__, state->slow_shut_delay);
			msleep(state->slow_shut_delay);
			state->slow_shut_delay = 0;
		}
		break;
	default:
		return -EINVAL;
	}
	
	return ret;
}

static int ms2r_start_capture(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	return ms2r_set_mode(sd, CAPTURE_MODE);
}

static int ms2r_wakeup_preview(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	/* to let poll exit in hal layer preview thread */
//	fimc_wakeup_preview(); 

	return 0;
}

static int ms2r_get_jpeg_main_size(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	
	ret = ms2r_r32(sd, JPEG_IMAGE_SIZE_REG,  &ctrl->value);
	CHECK_ERR(ret);
	
	pr_info("%s():jpeg size is %d.\n", __func__, ctrl->value);
	WARN(0 == ctrl->value, "JPEG Image size is 0!\n");
	
	return 0;
}

static int ms2r_set_reverse(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ms2r_state *state = to_state(sd);
	int ret;

	CHECK_USERSET(reverse);
	
	ret = ms2r_w8(sd, MON_REVERSE_ISP_REG, !!ctrl->value);
	CHECK_ERR(ret);

	SET_USERSET(reverse);

	return 0;
}

static int ms2r_set_mirror(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ms2r_state *state = to_state(sd);
	int ret;

	CHECK_USERSET(mirror);
	
	ret = ms2r_w8(sd, MON_MIRROR_ISP_REG, !!ctrl->value);
	CHECK_ERR(ret);
	
	SET_USERSET(mirror);

	return 0;
}

static int ms2r_set_colorbar(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	return ms2r_w8(sd, COLOR_BAR_REG, ENABLE_COLOR_BAR);
}

static int ms2r_s_cap_format(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct v4l2_mbus_framefmt fmt;

	switch (ctrl->value) {
	case V4L2_PIX_FMT_YUYV:
		pr_info("%s(), set cap format to YUYV\n", __func__);
		fmt.code = V4L2_MBUS_FMT_VYUY8_2X8;
		break;
	case V4L2_PIX_FMT_JPEG:
		pr_info("%s(), set cap format to JPEG\n", __func__);
		fmt.code = V4L2_MBUS_FMT_JPEG_1X8;
		break;
	case V4L2_PIX_FMT_SGRBG10:
		pr_info("%s(), set cap format to RAW10\n", __func__);
		fmt.code = V4L2_MBUS_FMT_SGRBG10_1X10;
		break;
	default:
		pr_err("%s(), !!!ERR: unkown cap format\n!!!", __func__);
		return -EINVAL;
	}

	return ms2r_set_capture_format(sd, &fmt);
}

static int ms2r_s_capture_size(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct v4l2_mbus_framefmt fmt;
	
	fmt.width = (ctrl->value >> 16) & 0xffff;
	fmt.height = ctrl->value & 0xffff;

	return ms2r_set_capture_size(sd, &fmt);
}

static int ms2r_set_preflash(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ms2r_state *state = to_state(sd);
	int ret;

	/*
	* Set preflash current
	*/
	pr_info("%s(), will set preflash current to %d\n",
		__func__, state->pre_flash_current);
	if (state->userset.flash_mode != MS2R_FLASH_OFF) {
		ms2r_set_flash_current(state, state->pre_flash_current);
	} else {
		pr_err("%s(), Err, flash_mode is OFF!!!!\n", __func__);
		return -EINVAL;
	}

	pr_info("%s(), ctrl->value is %d, Not wait int\n", __func__, ctrl->value);
	ret = ms2r_w8(sd, PREFLASH_START_REG, ctrl->value);
	CHECK_ERR(ret);

	return 0;
}

static int ms2r_set_mainflash(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ms2r_state *state = to_state(sd);
	int ret;

	pr_info("%s(), ctrl->value is %d\n", __func__, ctrl->value);

	if (ctrl->value) {
		/*
		* Set mainflash current
		*/
		pr_info("%s(), will set mainflash current to %d\n",
			__func__, state->full_flash_current);
		if (state->userset.flash_mode != MS2R_FLASH_OFF) {
			ms2r_set_flash_current(state, state->full_flash_current);
		} else {
			pr_err("%s(), Err, flash_mode is OFF!!!!\n", __func__);
			return -EINVAL;
		}

		pr_info("%s(), write mainflash reg, NOT wait int\n", __func__);
		ret = ms2r_w8(sd, MAINFLASH_START_REG, ctrl->value);
		CHECK_ERR(ret);
	}else {
		ret = ms2r_w8(sd, MAINFLASH_START_REG, ctrl->value);
		CHECK_ERR(ret);
	}

	return 0;
}

/*
* For light info, from which we can know whether light on is needed.
*/
static int ms2r_get_light_info(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	ms2r_r8(sd, INFO_LIGHT_REG, &ctrl->value);
	pr_info("%s(), reg value is %d\n", __func__, ctrl->value);
	return 0;
}

static int ms2r_get_now_bv(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	ms2r_r16(sd, NOW_BV_REG, &ctrl->value);
	pr_info("%s(), reg value is 0x%x\n", __func__, ctrl->value);
	return 0;
}

static int ms2r_get_now_gain(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	ms2r_r16(sd, NOW_GAIN_REG, &ctrl->value);
	pr_info("%s(), reg value is 0x%x\n", __func__, ctrl->value);
	return 0;
}

static int ms2r_get_now_exposure(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	ms2r_r16(sd, NOW_EXPOSURE_REG, &ctrl->value);
	pr_info("%s(), reg value is 0x%x\n", __func__, ctrl->value);
	return 0;
}

static int ms2r_get_preflash_result(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	ms2r_r8(sd, PREFLASH_START_REG, &ctrl->value);
	pr_info("%s(), reg value is 0x%x\n", __func__, ctrl->value);
	return 0;
}

static int ms2r_get_mainflash_result(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	ms2r_r8(sd, MAINFLASH_START_REG, &ctrl->value);
	pr_info("%s(), reg value is 0x%x\n", __func__, ctrl->value);
	return 0;
}

static int ms2r_get_focus_position(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	ms2r_r8(sd, FOCUS_POSITION_REG, &ctrl->value);
	pr_info("%s(), reg value is 0x%x\n", __func__, ctrl->value);
	return 0;
}

static int ms2r_set_ae_lock(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	pr_info("%s(), ctrl->value is %d\n", __func__, ctrl->value);
	ret = ms2r_enable_ae_lock(sd, !!ctrl->value);
	return ret;
}

static int ms2r_set_awb_lock(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	pr_info("%s(), ctrl->value is %d\n", __func__, ctrl->value);
	ret = ms2r_enable_awb_lock(sd, !!ctrl->value);
	return ret;
}

static int ms2r_set_fps(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	pr_info("%s(), ctrl->value is %d\n", __func__, ctrl->value);
	if (ctrl->value < EVP_MODE_AUTO || ctrl->value > EVP_MODE_30FPS) {
		pr_err("%s(), unsupported FPS\n", __func__);
		return -EINVAL;
	}
	ret = ms2r_w8(sd, EVP_MODE_MON_REG, ctrl->value);
	CHECK_ERR(ret);
	return ret;
}

static int ms2r_set_manual_focus(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	pr_info("%s(), ctrl->value is %d\n", __func__, ctrl->value);
	if (ctrl->value < MF_DISABLE ||ctrl->value > MF_ENABLE) {
		pr_err("%s(), unsupported manual focus setting\n", __func__);
		return -EINVAL;
	}
	ret = ms2r_w8(sd, MANUAL_FOCUS_ENABLE_REG, ctrl->value);
	CHECK_ERR(ret);
	return ret;
}

static int ms2r_set_manual_focus_percentage(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	pr_info("%s(), ctrl->value is %d\n", __func__, ctrl->value);
	if (ctrl->value < MF_PERCEN_MIN ||ctrl->value > MF_PERCEN_MAX) {
		pr_err("%s(), unsupported manual focus position percentage\n", __func__);
		return -EINVAL;
	}
	ret = ms2r_w8(sd, MANUAL_FOCUS_PERCEN_REG, ctrl->value);
	CHECK_ERR(ret);
	return ret;
}

static int ms2r_set_flicker_mode(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	pr_info("%s(), ctrl->value is %d\n", __func__, ctrl->value);
	if (ctrl->value < FLICKER_AUTO_DETECT_COMPEN ||
		ctrl->value > FLICKER_AUTO_REDETECT_RECOMPEN) {
		pr_err("%s(), unsupported flicker mode\n", __func__);
		return -EINVAL;
	}
	ret = ms2r_w8(sd, FLICKER_REG, ctrl->value);
	CHECK_ERR(ret);
	return ret;
}

static int ms2r_set_slow_shut(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	struct ms2r_state *state = to_state(sd);
	pr_info("%s(), ctrl->value is %d\n", __func__, ctrl->value);
	if (ctrl->value < M_SLOW_SHUT_AUTO ||
		ctrl->value > M_SLOW_SHUT_1D5) {
		pr_err("%s(), unsupported slow shut mode\n", __func__);
		return -EINVAL;
	}
	ret = ms2r_w8(sd, M_SLOW_SHUT_REG, ctrl->value);
	CHECK_ERR(ret);
	state->slow_shut_delay = slow_shut_delay[ctrl->value];
	return ret;
}

static int ms2r_set_sharpness(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	pr_info("%s(), ctrl->value is %d\n", __func__, ctrl->value);
	if (ctrl->value < SHARP_LVL_MIN ||
		ctrl->value > SHARP_LVL_MAX) {
		pr_err("%s(), unsupported shapness level\n", __func__);
		return -EINVAL;
	}

	ret = ms2r_w8(sd, SHARP_EN_REG, 1);
	CHECK_ERR(ret);

	ret = ms2r_w8(sd, SHARP_NO_REG, ctrl->value);
	CHECK_ERR(ret);
	return ret;
}

static int ms2r_set_chroma(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	pr_info("%s(), ctrl->value is %d\n", __func__, ctrl->value);
	if (ctrl->value < CHROMA_LVL_MIN ||
		ctrl->value > CHROMA_LVL_MAX) {
		pr_err("%s(), unsupported chroma level\n", __func__);
		return -EINVAL;
	}

	ret = ms2r_w8(sd, CHROMA_EN_REG, 1);
	CHECK_ERR(ret);

	ret = ms2r_w8(sd, CHROMA_LVL_REG, ctrl->value);
	CHECK_ERR(ret);
	return ret;
}

static int ms2r_set_caf_scenario(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	pr_info("%s(), ctrl->value is %d\n", __func__, ctrl->value);
	if (ctrl->value < SCE_VIDEO_CAF ||
		ctrl->value > SCE_STILL_CAP_CAF) {
		pr_err("%s(), unsupported caf scenario\n", __func__);
		return -EINVAL;
	}

	ret = ms2r_w8(sd, CAF_SCENARIO_REG, ctrl->value);
	CHECK_ERR(ret);

	return ret;
}

/*************************************************/
/***********  exif information functions  ****************/
/*************************************************/

static int ms2r_get_exif_exptime_n(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	return ms2r_r32(sd, INFO_EXPTIME_NUMERATOR_REG, &ctrl->value);
}

static int ms2r_get_exif_exptime_d(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	return ms2r_r32(sd, INFO_EXPTIME_DENUMINATOR_REG, &ctrl->value);
}

static int ms2r_get_exif_tv_n(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	return ms2r_r32(sd, INFO_TV_NUMERATOR_REG, &ctrl->value);
}

static int ms2r_get_exif_tv_d(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	return ms2r_r32(sd, INFO_TV_DENUMINATOR_REG, &ctrl->value);
}

static int ms2r_get_exif_av_n(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	return ms2r_r32(sd, INFO_AV_NUMERATOR_REG, &ctrl->value);
}

static int ms2r_get_exif_av_d(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	return ms2r_r32(sd, INFO_AV_DENUMINATOR_REG, &ctrl->value);
}

static int ms2r_get_exif_bv_n(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	return ms2r_r32(sd, INFO_BV_NUMERATOR_REG, &ctrl->value);
}

static int ms2r_get_exif_bv_d(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	return ms2r_r32(sd, INFO_BV_DENUMINATOR_REG, &ctrl->value);
}

static int ms2r_get_exif_ebv_n(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	return ms2r_r32(sd, INFO_EBV_NUMERATOR_REG, &ctrl->value);
}

static int ms2r_get_exif_ebv_d(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	return ms2r_r32(sd, INFO_EBV_DENUMINATOR_REG, &ctrl->value);
}

static int ms2r_get_exif_iso_ext(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	return ms2r_r16(sd, INFO_ISO_REG, &ctrl->value);
}

static int ms2r_get_exif_iso(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	return ms2r_r16(sd, INFO_ISO_REG, &ctrl->value);
}

static int ms2r_get_exif_flash(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	return ms2r_r16(sd, INFO_FLASH_REG, &ctrl->value);
}

static int ms2r_get_exif_sdr(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	return ms2r_r16(sd, INFO_SDR_REG, &ctrl->value);
}

static int ms2r_get_exif_qval(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	return ms2r_r16(sd, INFO_QVAL_REG, &ctrl->value);
}

/*********************************************/

static int ms2r_g_ext_ctrl(struct v4l2_subdev *sd, struct v4l2_ext_control *ctrl)
{
	int ret = 0;
	
	switch (ctrl->id) {
	case V4L2_CTRL_CLASS_CAMERA_REGISTER:
		ret = ms2r_read_reg(sd, (u16)ctrl->value, &ctrl->value, ctrl->size);
		break;
	case V4L2_CTRL_CLASS_CAMERA_EXPTIME_N:
		ret = ms2r_get_exif_exptime_n(sd, ctrl);
		break;
	case V4L2_CTRL_CLASS_CAMERA_EXPTIME_D:
		ret = ms2r_get_exif_exptime_d(sd, ctrl);
		break;
	case V4L2_CTRL_CLASS_CAMERA_TV_N:
		ret = ms2r_get_exif_tv_n(sd, ctrl);
		break;
	case V4L2_CTRL_CLASS_CAMERA_TV_D:
		ret = ms2r_get_exif_tv_d(sd, ctrl);
		break;
	case V4L2_CTRL_CLASS_CAMERA_AV_N:
		ret = ms2r_get_exif_av_n(sd, ctrl);
		break;
	case V4L2_CTRL_CLASS_CAMERA_AV_D:
		ret = ms2r_get_exif_av_d(sd, ctrl);
		break;
	case V4L2_CTRL_CLASS_CAMERA_BV_N:
		ret = ms2r_get_exif_bv_n(sd, ctrl);
		break;
	case V4L2_CTRL_CLASS_CAMERA_BV_D:
		ret = ms2r_get_exif_bv_d(sd, ctrl);
		break;
	case V4L2_CTRL_CLASS_CAMERA_EBV_N:
		ret = ms2r_get_exif_ebv_n(sd, ctrl);
		break;
	case V4L2_CTRL_CLASS_CAMERA_EBV_D:
		ret = ms2r_get_exif_ebv_d(sd, ctrl);
		break;
	case V4L2_CTRL_CLASS_CAMERA_ISO:
		ret = ms2r_get_exif_iso_ext(sd, ctrl);
		break;
	case V4L2_CTRL_CLASS_CAMERA_FLASH:
		ret = ms2r_get_exif_flash(sd, ctrl);
		break;
	case V4L2_CTRL_CLASS_CAMERA_SDR:
		ret = ms2r_get_exif_sdr(sd, ctrl);
		break;
	case V4L2_CTRL_CLASS_CAMERA_QV:
		ret = ms2r_get_exif_qval(sd, ctrl);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

int ms2r_g_ext_ctrls(struct v4l2_subdev *sd, struct v4l2_ext_controls *ctrls)
{
	struct v4l2_ext_control *ctrl = ctrls->controls;
	int i, ret = 0;

	for (i = 0; i < ctrls->count; i++, ctrl++) {
		ret = ms2r_g_ext_ctrl(sd, ctrl);
		if (ret) {
			ctrls->error_idx = i;
			break;
		}
	}
	return ret;
}

/*exif informaion*/
static int ms2r_get_exif_exptime(struct v4l2_subdev *sd, struct v4l2_control *ctrl, int flag)
{
	if (flag) 
		return ms2r_r32(sd, INFO_EXPTIME_DENUMINATOR_REG, &ctrl->value);
	else 
		return ms2r_r32(sd, INFO_EXPTIME_NUMERATOR_REG, &ctrl->value);
}

int ms2r_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	struct ms2r_state *state = to_state(sd);
	
	switch (ctrl->id) {
	case V4L2_CID_CAMERA_WHITE_BALANCE:		
		ctrl->value = state->userset.manual_wb;
		break;
	case V4L2_CID_CAMERA_BRIGHTNESS:
		ctrl->value = state->userset.brightness;
		break;
	case V4L2_CID_ZOOM_ABSOLUTE:
		ctrl->value = state->userset.zoom_level;
		break;
	case V4L2_CID_CAMERA_FOCUS_WINDOW:
		ctrl->value = state->userset.focus_position;
		break;
	case V4L2_CID_CAM_JPEG_MAIN_OFFSET:
	case V4L2_CID_CAM_JPEG_THUMB_OFFSET:
	case V4L2_CID_CAM_JPEG_POSTVIEW_OFFSET:		
		ctrl->value =  0;
		break;
	case V4L2_CID_CAM_FW_MINOR_VER:
		ctrl->value = state->fw_version;
		break;
	case V4L2_CID_CAM_JPEG_MEMSIZE:
		ret = ms2r_get_jpeg_mem_size(sd, ctrl);
		break;	
	case V4L2_CID_CAM_JPEG_MAIN_SIZE:
		ret = ms2r_get_jpeg_main_size(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_SCENE_EV:
		ret = ms2r_get_scene_ev(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_AUTO_FOCUS_RESULT:
		ret = ms2r_get_auto_focus_result(sd, ctrl); 
		break;

	case V4L2_CID_CAMERA_PANO_PICTURE_NUM:
		ctrl->value = state->pano.counter + 1;
		break;
	case V4L2_CID_CAMERA_PANO_READY:
		ret = ms2r_wait_panorama_stitch(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_PANO_DIRECTION:
		ret = ms2r_get_pan_direction(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_MULTI_CAPTURE_READY:
		ret = ms2r_wait_multi_capture_ready(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_SMILE_FACE_DETECTION:
		ret = ms2r_get_smile_face_detection(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_FACE_DET_NUM:
		ret = ms2r_get_face_detected_num(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_SELECTED_FACE_LOCATION:
		ret = ms2r_get_selected_face_location(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_SELECTED_FACE_SIZE:
		ret = ms2r_get_selected_face_size(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_EXIF_EXPTIME_NUMERATOR:
		pr_info("LEGACY %s(), ctrl->id is 0x%x\n", __func__, ctrl->id);
		ret = ms2r_get_exif_exptime(sd, ctrl, 0);
		break;

	case V4L2_CID_CAMERA_EXIF_EXPTIME_DENUMINATOR:
		pr_info("LEGACY %s(), ctrl->id is 0x%x\n", __func__, ctrl->id);		
		ret = ms2r_get_exif_exptime(sd, ctrl, 1);
		break;

	case V4L2_CID_CAMERA_EXIF_ISOV:
		pr_info("LEGACY %s(), ctrl->id is 0x%x\n", __func__, ctrl->id);
		ret = ms2r_get_exif_iso(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_LIGHT_INFO:
		ret = ms2r_get_light_info(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_NOW_BV:
		ret = ms2r_get_now_bv(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_NOW_GAIN:
		ret = ms2r_get_now_gain(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_NOW_EXPOSURE:
		ret = ms2r_get_now_exposure(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_PREFLASH:
		ret = ms2r_get_preflash_result(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_MAINFLASH:
		ret = ms2r_get_mainflash_result(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_FOCUS_POSITION:
		ret = ms2r_get_focus_position(sd, ctrl);
		break;
	default:
		pr_err("%s: no such control, ctrl->id=0x%x, ctrl->value = %d\n", __func__,ctrl->id,ctrl->value);
		/*
		* The upper layer may NOT treat this as an error.
		*/
		return -ENOIOCTLCMD;
	}
	
	return ret;
}

int ms2r_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	
	switch (ctrl->id) {
	case V4L2_CID_CAMERA_CAPTURE:
		ret = ms2r_transfer_capture_data(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_WHITE_BALANCE:
		ret = ms2r_set_wb_preset(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_BRIGHTNESS:	
		ret = ms2r_set_image_brightness(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_SCENE_MODE:
		ret = ms2r_set_scenemode(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_FOCUS_MODE:
		ret = ms2r_set_focus_mode(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_SET_AUTO_FOCUS:
		ret = ms2r_set_focus(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_FOCUS_WINDOW:
		ret = ms2r_set_touch_position(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_SET_TOUCH_AF_AE:
		ret = ms2r_set_touch_af_ae_en(sd, ctrl->value);
		break;
	case V4L2_CID_CAMERA_SET_AF_RANGE_MODE:
		ret = ms2r_set_af_range_mode(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_SET_TOUCH_AE_STRENGTH:
		ret = ms2r_set_touch_ae_strength(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_ZOOM:
		ret = ms2r_set_zoom_level(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_ISO:
		ret = ms2r_set_iso(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_WDR:
		pr_info("%s(), set WDR to %d\n", __func__, ctrl->value);
		ret = ms2r_set_wdr(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_FLASH_MODE:
		ret = ms2r_set_flash_mode(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_ROTATION:
		ret = ms2r_set_rotation(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_ISP_REVERSE:
		ret = ms2r_set_reverse(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_ISP_MIRROR:
		ret = ms2r_set_mirror(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_COLORBAR:
		ret = ms2r_set_colorbar(sd, ctrl);
		break;
		
	case V4L2_CID_CAMERA_CAPTURE_FORMAT:
		ret = ms2r_s_cap_format(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_CAPTURE_SIZE:
		ret = ms2r_s_capture_size(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_QUICK_CAPTURE:
		ret = ms2r_start_capture(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_WAKEUP_PREVIEW:
		ret = ms2r_wakeup_preview(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_PANO_CAPTURE:
		ret = ms2r_set_panorama_capture(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_TERMINATE_PANO:
		ret = ms2r_terminate_panorama(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_PANO_STATUS:
		ret = ms2rs_set_cur_panorama_info(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_MULTI_CAPTURE:
		ret = ms2r_set_multi_capture(sd, ctrl);	
		break;
	case V4L2_CID_CAMERA_WAIT_MULTI_CAPTURE:
		ret = ms2r_wait_multi_capture_picture(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_MULTI_CAPTURE_PICTURE:
		ret = ms2r_get_multi_capture_picture(sd, ctrl);	
		break;

	case V4L2_CID_CAMERA_SMILE_FACE_DETECTION:
		ret = ms2r_set_smile_face_detection(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_FACE_DET_SELECT:
		ret = ms2r_set_face_selection(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_FACE_DETECTION_DIRECTION:
		ret = ms2r_set_face_detection_direction(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_AUTO_BRACKET_CAPTURE:
		ret = ms2r_set_auto_bracket_capture(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_PREFLASH:
		ret = ms2r_set_preflash(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_MAINFLASH:
		ret = ms2r_set_mainflash(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_AE_LOCK:
		ret = ms2r_set_ae_lock(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_AWB_LOCK:
		ret = ms2r_set_awb_lock(sd, ctrl);
		break;

	case V4L2_CID_CAMERA_FPS:
		ret = ms2r_set_fps(sd, ctrl);
		break;

	case V4L2_CID_MANUAL_FOCUS_SETTING:
		ret = ms2r_set_manual_focus(sd, ctrl);
		break;
	case V4L2_CID_MANUAL_FOCUS_PERCENTAGE:
		ret = ms2r_set_manual_focus_percentage(sd, ctrl);
		break;

	case V4L2_CID_FLICKER_MODE:
		ret = ms2r_set_flicker_mode(sd, ctrl);
		break;

	case V4L2_CID_M_SLOW_SHUT:
		ret = ms2r_set_slow_shut(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_SHAPNESS:
		ret = ms2r_set_sharpness(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_CHROMA:
		ret = ms2r_set_chroma(sd, ctrl);
		break;
	case V4L2_CID_CAMERA_CAF_SCENARIO:
		ret = ms2r_set_caf_scenario(sd, ctrl);
		break;

	default:
		pr_err("%s: no such control, ctrl->id=0x%x\n", __func__,ctrl->id);
		/*
		* The upper layer may NOT treat this as an error.
		*/
		return -ENOIOCTLCMD;
	}
	
	return ret;
}
