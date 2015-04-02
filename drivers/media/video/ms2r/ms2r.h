#ifndef __MS2R_CORE_H__ 
#define __MS2R_CORE_H__ 

#define MS2R_DRIVER_NAME "ms2r"
//#define USING_I2C_TRANS_CAM_FW

/* the time for wait completion */
#define WAIT_TIMEOUT 5000  /*in milisecond*/

#define MS2R_DEBUG 1
#define ms2r_dbg(fmt, ...) pr_info("%s:"fmt, __func__, ##__VA_ARGS__)
#define ms2r_trace() pr_info("%s:line = %d\n", __func__, __LINE__)

#define CHECK_ERR(x) if ((x) < 0) { \
				pr_err("%s():line = %d, i2c failed, ret = %d\n", \
				__func__, __LINE__, x); \
				return x; \
			}

#define BACK_CAMERA 0
#define FRONT_CAMERA 1  

/* flash */
#define FLASH_LED_NAME "flash_led"
/* unit is uA */
#define PRE_FLASH_CURRENT 100000
#define FULL_FLASH_CURRENT 400000
#define MAX_FLASH_CURRENT 1000000

enum ms2r_i2c_size {
	I2C_8BIT = 1,
	I2C_16BIT = 2,
	I2C_32BIT = 4,
	I2C_MAX = 4,
};

struct ms2r_reg {
	enum ms2r_i2c_size size;
	unsigned int addr;
	unsigned int val;
};

/* 
  * firmware status: indicate ISP firmware checking or updating status
  * FIRMWARE_NONE: ISP firmware has not been checked
  * FIRMWARE_REQUESTING: in the firmware checking or downloading progress
  * FIRMWARE_CHECKED: has been finished checking or downloading firmware
  * FIRMWARE_UPDATE_FAIL:fail to download data to ISP
*/
enum firmware_status {
	FIRMWARE_NONE,
	FIRMWARE_REQUESTING,
	FIRMWARE_CHECKED,
	FIRMWARE_UPDATE_FAIL,
};

/*
  * ISP read or write mode, for register or memory
 */
enum cmd_type {
	CMD_READ_PARAMETER = 1,
	CMD_WRITE_PARAMETER,
	CMD_READ_8BIT_MEMORY,
	CMD_WRITE_8BIT_MEMORY,
	CMD_READ_16BIT_MEMORY,
	CMD_WRITE_16BIT_MEMORY,
	CMD_READ_32BIT_MEMORY,
	CMD_WRITE_32BIT_MEMORY,
};

enum isp_mode {
	INITIALIZE_MODE,
	PARAMETER_MODE,
	MONITOR_MODE,
	CAPTURE_MODE,
};

/*
  * camera preview, record, and capture mode
*/
enum v4l2_camera_mode {
	V4L2_CAMERA_PREVIEW,
	V4L2_CAMERA_RECORD,
	V4L2_CAMERA_PANORAMA,
	V4L2_CAMERA_SINGLE_CAPTURE,
	V4L2_CAMERA_MULTI_CAPTURE,
	V4L2_CAMERA_PANORAMA_CAPTURE,
};

enum camera_mode_type {
	PREVIEW_MODE_TYPE,
	CAPTURE_MODE_TYPE,
};

/*
 * Store information about the video data format.  The color matrix
 * is deeply tied into the format, so keep the relevant values here.
 */
struct ms2r_format_struct {
	__u8 *desc;
	__u32 pixelformat;
	enum v4l2_mbus_pixelcode mbus_code;
	enum v4l2_colorspace colorspace;	
	int bpp;
	struct ms2r_reg *regs;
	int size;
};

struct ms2r_size_struct {
	int width;
	int height;
	struct ms2r_reg *regs;
	int size;
};

/* user configure setting, it should be initialized in init function */
struct ms2r_userset {
	unsigned int manual_wb;
	unsigned int brightness;
	unsigned int scene;
	unsigned int zoom_level;
	unsigned int wdr;	
	unsigned int iso;
	unsigned int flash_mode;
	unsigned int rotation;
	unsigned int mirror;
	unsigned int reverse;
	unsigned int af_mode;
	unsigned int focus_position;
};

enum cap_mode {
	CAP_NORMAL_MODE,
	CAP_PANORAMA_MODE,
	CAP_MULTI_CAP_MODE,
	CAP_SMILE_CAP_MODE,
	CAP_AUTO_BRACKET_MODE,
};

#define PANORAMA_MAX_PICTURE 40

/* panorama status */
enum panorama_picture_status {
	PANORAMA_SUCCESS,
	PANORAMA_RETRY_ERR,
	PANORAMA_BIG_ERR,
	PANORAMA_FATAL_ERR,	
	PANORAMA_UNKNOWN_ERR,
	PANORAMA_COMPLETE,  /* means for complete panorama capture */
};

/* panorama stitch status */
enum panorama_stitch_status {
	PANORAMA_STITCH_OK,
	PANORAMA_STITCH_FAIL,
	PANORAMA_STITCH_INIT,
};

/* panorama picture information */
struct panorama_picture {
	enum panorama_picture_status status;  /* status */
	unsigned int extra;  /* extra information */
};

/* panorama data struct */
struct panorama_struct {
	int counter;  /* picture counter */
	struct panorama_picture pictures[PANORAMA_MAX_PICTURE];
	enum panorama_stitch_status stitch_status;
};

#define MULTI_CAP_MAX_PICTURE 9

/* multi capture ready status */
enum multi_cap_picture_status {
	MULTI_CAP_SUCCESS,
	MULTI_CAP_FAIL,
	MULTI_CAP_INIT,
};

enum multi_cap_ready_status {
	MULTI_CAP_READY_SUCCESS,
	MULTI_CAP_READY_FAIL,
	MULTI_CAP_READY_INIT,
};

struct multi_cap_picture {
	enum multi_cap_picture_status status;
};

struct multi_cap_struct {
	int numbers;
	int counter;
	struct multi_cap_picture pictures[MULTI_CAP_MAX_PICTURE];
	enum multi_cap_ready_status ready;
};

enum smile_detection {
	SMILE_NO_DETECTION,
	SMILE_DETECTION,
};

/* smile cap data struct */
struct smile_cap_struct {
	enum smile_detection detection;
};

enum module_type {
	MODULE_REAR_LITEON,
	MODULE_REAR_SHARP,
	MODULE_NUMBER, //number of modules
};

struct ms2r_state {
	struct ms2r_platform_data *pdata;
	struct i2c_client *client;
	struct v4l2_subdev sd;
#if defined(CONFIG_MEDIA_CONTROLLER)
	struct media_pad pad;
#endif
	struct v4l2_mbus_framefmt fmt; /* current format and size */
	struct ms2r_size_struct prev_size;  /* last preview size*/
	struct ms2r_size_struct cap_size;   /* last capture size */
	struct ms2r_format_struct cap_fmt;  /* last capture format */

	int irq;  /* irq number */
	int irq_status;

	int fps;
	enum isp_mode mode;   /* isp operating mode:parameter, monitor or capture */
	enum v4l2_camera_mode camera_mode;   /* user modes: preview, capture or record */
	bool stream_on;
	struct ms2r_userset userset;   /* user setting */
	bool isp_power;
	bool sensor_power;
	bool debug;
	int cam_id;   /* used distinguishing between front and back camera */
	struct regulator *fled_regulator;
	int pre_flash_current;
	int full_flash_current;
	
	struct completion completion;
	struct mutex mutex;

	int fw_version;
	enum firmware_status fw_status;
	u8 *fw_buffer;
	/*
	* Must be DMA safe.
	*/
	u8 *fw_buf_spi;
	size_t fw_buf_spi_size;
	u8 *fw_buf_i2c;
	size_t fw_buf_i2c_size;
	enum module_type module_type;
	bool fw_updated;  /* used by factory test */
	struct wake_lock wake_lock;

	struct work_struct work;  /* work for panorama cap, multi cap and smile cap */
	struct workqueue_struct *wq;
	enum cap_mode cap_mode;
	
	struct panorama_struct pano;  /* panorama capture data */
	struct multi_cap_struct	multi_cap;   /* multi-capture data */
	struct smile_cap_struct smile_cap;  /* smile capture data */
	bool need_sleep;
	bool prepared;
	int slow_shut_delay;
};

struct touch_area {
	int x;
	int y;
	int width;
	int height;
};

/*
  * I2C operation functions, implement in ms2r_drv.c
*/
int ms2r_read_reg(struct v4l2_subdev *sd, u16 addr, u32 *val, enum ms2r_i2c_size size);
int ms2r_write_reg(struct v4l2_subdev *sd, u16 addr, u32 val, enum ms2r_i2c_size size);
int ms2r_write_memory(struct v4l2_subdev *sd,
		u32 addr, const char *data,
		int size);
int ms2r_read_memory(struct v4l2_subdev *sd,
		u32 addr, char *data, int size);

int ms2r_write_regs(struct v4l2_subdev *sd, struct ms2r_reg *regs, int size);

#define ms2r_r8(sd, addr, val) ms2r_read_reg(sd, addr, val, I2C_8BIT)
#define ms2r_r16(sd, addr, val) ms2r_read_reg(sd, addr, val, I2C_16BIT)
#define ms2r_r32(sd, addr, val) ms2r_read_reg(sd, addr, val, I2C_32BIT)

#define ms2r_w8(sd, addr, val) ms2r_write_reg(sd, addr, val, I2C_8BIT)
#define ms2r_w16(sd, addr, val) ms2r_write_reg(sd, addr, val, I2C_16BIT)
#define ms2r_w32(sd, addr, val) ms2r_write_reg(sd, addr, val, I2C_32BIT)

int ms2r_spi_write(const void *buf, size_t size);
static inline bool ms2r_is_isppower(struct ms2r_state *state)
{
	return state->isp_power;
}

int ms2r_set_sys_mode(struct v4l2_subdev *sd, enum isp_mode mode);
int ms2r_set_power_clock(struct ms2r_state *state, bool enable);
int ms2r_sensor_isp_s_power(struct v4l2_subdev *sd, int on);
void ms2r_prepare_wait(struct v4l2_subdev *sd);
int ms2r_enable_root_irq(struct v4l2_subdev *sd);
int ms2r_enable_irq(struct v4l2_subdev *sd);
int ms2r_wait_irq(struct v4l2_subdev *sd, const unsigned int timeout);
int ms2r_wait_irq_and_check(struct v4l2_subdev *sd, u8 mask,
	const unsigned int timeout);
int ms2r_set_capture_format(struct v4l2_subdev *sd,
	struct v4l2_mbus_framefmt *fmt);
int ms2r_set_capture_size(struct v4l2_subdev *sd,
	struct v4l2_mbus_framefmt *fmt);
int ms2r_enable_ae_lock(struct v4l2_subdev *sd, bool enable);
int ms2r_enable_awb_lock(struct v4l2_subdev *sd, bool enable);
int ms2r_set_mode(struct v4l2_subdev *sd, enum isp_mode mode);

/* ms2r ctrl functions */
int ms2r_g_ext_ctrls(struct v4l2_subdev *sd, struct v4l2_ext_controls *ctrls);
int ms2r_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl);
int ms2r_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl);
int ms2r_set_flash_current(struct ms2r_state *state, 
	int cur);
void ms2r_handle_multi_cap(struct ms2r_state *state, int irq_status);
void ms2r_handle_panorama_cap(struct ms2r_state *state, int irq_status);
void ms2r_handle_smile_cap(struct ms2r_state *state, int irq_status);
void ms2r_handle_auto_bracket_cap(struct ms2r_state *state, int irq_status);

/* ms2r firmware functions */
int ms2r_load_firmware(struct v4l2_subdev *sd);
int ms2r_request_fw(struct v4l2_subdev *sd, int camera_id);
void ms2r_release_fw_mem(struct v4l2_subdev *sd);
int ms2r_allocate_fw_mem(struct v4l2_subdev *sd);
int ms2r_erase_firmware(struct v4l2_subdev *sd);
int ms2r_load_firmware_sys(struct device *dev, struct v4l2_subdev *sd);
int ms2r_verdict_module_type(struct v4l2_subdev *sd);
int ms2r_retrieve_module(struct v4l2_subdev *sd, bool mani_file);
int ms2r_request_module_fw(struct v4l2_subdev *sd);
int ms2r_save_file(struct v4l2_subdev *sd);

static inline struct ms2r_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ms2r_state, sd);
}

extern void fimc_wakeup_preview(void);
extern void fimc_reset_wakeup_flag(void);

#endif
