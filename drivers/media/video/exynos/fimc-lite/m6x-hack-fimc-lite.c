/*
* This file is for FIMC LITE manipulating ISP of MS2R.
* This file will not be compiled if CONFIG_VIDEO_MS2R is not defined.
*/
#include "fimc-lite-core.h"
#include <linux/i2c.h>
#define M6X_MAX_CAM_SENSORS 2

void flite_set_cam_clock(struct flite_dev *flite, bool on)
{
	pr_debug("%s(), ON/OFF(%d): Do nothing since we configure it in "
		"s_power\n", __func__, on);
}

int __subdev_set_power(struct v4l2_subdev *sd, int on)
{
	int *use_count;
	int ret;

	if (sd == NULL)
		return -ENXIO;

	use_count = &sd->entity.use_count;
	if (on && (*use_count)++ > 0)
		return 0;
	else if (!on && (*use_count == 0 || --(*use_count) > 0))
		return 0;

	if (on && !strcmp(sd->name, ISP_ENTITY_NAME)) {
		pr_debug("%s(), we do NOT power on %s here\n",
			__func__, ISP_ENTITY_NAME);
		ret = 0;
	} else {
		ret = v4l2_subdev_call(sd, core, s_power, on);
	}

	return ret != -ENOIOCTLCMD ? ret : 0;
}

int flite_enum_input(struct file *file, void *priv,
			       struct v4l2_input *i)
{
	struct flite_dev *flite = video_drvdata(file);
	struct exynos_platform_flite *pdata = flite->pdata;
	struct exynos_isp_info *isp_info;

	if (i->index >= M6X_MAX_CAM_SENSORS) {
		dev_err(&flite->pdev->dev, "%s(), index(%d) should NOT be greater than %d",
			__func__, i->index, M6X_MAX_CAM_SENSORS);
		return -EINVAL;
	}

	/*
	* Front and read sensor are all on one extern ISP.
	*/
	isp_info = pdata->isp_info[0];
	if (isp_info == NULL)
		return -EINVAL;

	i->type = V4L2_INPUT_TYPE_CAMERA;

	strncpy(i->name, isp_info->board_info->type, 32);

	return 0;

}

/*
* We should utilize this to distinguish m6x's front and rear sensors.
*/
int flite_s_input(struct file *file, void *priv, unsigned int i)
{
	int ret;
	struct flite_dev *flite = video_drvdata(file);
	dev_dbg(&flite->pdev->dev, "%s(), camera ID is %d\n",
		__func__, i);
	/*
	* We only have one to represent external ISP.
	* i represents ID of sensor: 0 is rear and 1 is front.
	*
	* Utilize v4l2_subdev_call() to manipulate external ISP.
	* We should power on MS2R subdev here since from the
	* index we can know which sensor should be powered.
	*/
	ret = v4l2_subdev_call(flite->pipeline.sensor, core, init, i);
	if (ret) {
		dev_err(&flite->pdev->dev, "%s(), subdev %s init failed: %d\n",
			__func__, flite->pipeline.sensor->name, ret);
		return ret;
	}

	return 0;
}

#if defined(CONFIG_VIDEO_MS2R) && defined(USING_VIDIOC_S_CTRL)
int flite_s_ioctrl(struct file *file, void *fh, struct v4l2_control *c)
{

	struct v4l2_subdev *sd;
	struct flite_dev *flite = video_drvdata(file);
	struct flite_frame *frame = &flite->d_frame;
	int ret = 0;

	sd = flite->pipeline.sensor;

	dev_dbg(&flite->pdev->dev, "%s(), id: 0x%x, value: 0x%x\n",
		__func__, c->id, c->value);

	switch (c->id) {
	case V4L2_CID_CACHEABLE:
		frame->cacheable = !!c->value;
		break;
	default:
		ret = v4l2_subdev_call(sd, core, s_ctrl, c);
		if (ret < 0 && ret != -ENOIOCTLCMD) {
			dev_err(&flite->pdev->dev, "%s(), s_ctrl on subdev failed",
				__func__);
			return ret;
		}
		break;
	}

	return 0;
}

int flite_g_ioctrl(struct file *file, void *fh, struct v4l2_control *c)
{

	struct v4l2_subdev *sd;
	struct flite_dev *flite = video_drvdata(file);
	struct flite_frame *frame = &flite->d_frame;
	int ret = 0;

	sd = flite->pipeline.sensor;

	dev_info(&flite->pdev->dev, "%s(), id: 0x%x\n",
		__func__, c->id);

	switch (c->id) {
	case V4L2_CID_CACHEABLE:
		c->value = frame->cacheable;
		break;
	default:
		ret = v4l2_subdev_call(sd, core, g_ctrl, c);
		if (ret < 0 && ret != -ENOIOCTLCMD) {
			dev_err(&flite->pdev->dev, "%s(), s_ctrl on subdev failed",
				__func__);
			return ret;
		}
		break;
	}

	return 0;
}
#endif
