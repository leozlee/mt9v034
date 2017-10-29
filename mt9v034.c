#include <linux/videodev2.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/log2.h>

#include <media/v4l2-subdev.h>
#include <media/v4l2-chip-ident.h>
#include <media/soc_camera.h>
#include "mt9v034.h"
#include "mt9v034_dev.h"

#define MT9V034_MODULE_NAME					"mt9v034"

#define MT9V034_PIXEL_ARRAY_HEIGHT			499
#define MT9V034_PIXEL_ARRAY_WIDTH			809

#define MT9V034_TOTAL_ROW_TIME_MIN			690

#define MT9V034_SYSCLK_FREQ_DEF				26600000

#define MT9V034_CHIP_VERSION				0x00
#define		MT9V034_CHIP_ID_REV1			0x1324
#define MT9V034_COLUMN_START				0x01
#define		MT9V034_COLUMN_START_MIN		1
#define		MT9V034_COLUMN_START_DEF		1
#define		MT9V034_COLUMN_START_MAX		752
#define MT9V034_ROW_START				0x02
#define		MT9V034_ROW_START_MIN			4
#define		MT9V034_ROW_START_DEF			4
#define		MT9V034_ROW_START_MAX			482
#define MT9V034_WINDOW_HEIGHT				0x03
#define		MT9V034_WINDOW_HEIGHT_MIN		1
#define		MT9V034_WINDOW_HEIGHT_DEF		480
#define		MT9V034_WINDOW_HEIGHT_MAX		480
#define MT9V034_WINDOW_WIDTH				0x04
#define		MT9V034_WINDOW_WIDTH_MIN		1
#define		MT9V034_WINDOW_WIDTH_DEF		752
#define		MT9V034_WINDOW_WIDTH_MAX		752
#define MT9V034_HORIZONTAL_BLANKING			0x05
#define		MT9V034_HORIZONTAL_BLANKING_MIN		61
#define		MT9V034_HORIZONTAL_BLANKING_DEF		94
#define		MT9V034_HORIZONTAL_BLANKING_MAX		1023
#define MT9V034_VERTICAL_BLANKING			0x06
#define		MT9V034_VERTICAL_BLANKING_MIN		2
#define		MT9V034_VERTICAL_BLANKING_DEF		45
#define		MT9V034_VERTICAL_BLANKING_MAX		32288
#define MT9V034_CHIP_CONTROL				0x07
#define		MT9V034_CHIP_CONTROL_DEFAULT	0x188
#define		MT9V034_CHIP_CONTROL_MASTER_MODE	(1 << 3)
#define		MT9V034_CHIP_CONTROL_SNAPSHOT_MODE	(1 << 4)
#define		MT9V034_CHIP_CONTROL_DOUT_ENABLE	(1 << 7)
#define		MT9V034_CHIP_CONTROL_SEQUENTIAL		(1 << 8)
#define MT9V034_SHUTTER_WIDTH1				0x08
#define MT9V034_SHUTTER_WIDTH2				0x09
#define MT9V034_SHUTTER_WIDTH_CONTROL			0x0a
#define MT9V034_TOTAL_SHUTTER_WIDTH			0x0b
#define		MT9V034_TOTAL_SHUTTER_WIDTH_MIN		0
#define		MT9V034_TOTAL_SHUTTER_WIDTH_DEF		480
#define		MT9V034_TOTAL_SHUTTER_WIDTH_MAX		32765
#define	MT9V034_MAX_TOTAL_SHUTTER_WIDTH		0xad
#define		MT9V034_MAX_TOTAL_SHUTTER_WIDTH_DEF	480
#define MT9V034_RESET						0x0c
#define MT9V034_READ_MODE					0x0d
#define		MT9V034_READ_MODE_DEFAULT_VALUE		0x300
#define		MT9V034_READ_MODE_ROW_BIN_MASK		(3 << 0)
#define		MT9V034_READ_MODE_ROW_BIN_SHIFT		0
#define		MT9V034_READ_MODE_COLUMN_BIN_MASK	(3 << 2)
#define		MT9V034_READ_MODE_COLUMN_BIN_SHIFT	2
#define		MT9V034_READ_MODE_ROW_FLIP			(1 << 4)
#define		MT9V034_READ_MODE_COLUMN_FLIP		(1 << 5)
#define		MT9V034_READ_MODE_DARK_COLUMNS		(1 << 6)
#define		MT9V034_READ_MODE_DARK_ROWS			(1 << 7)
#define MT9V034_SENSOR_TYPE_CONTROL			0x0f
#define		MT9V034_SENSOR_TYPE_MODE_COLOR		(1 << 1)
#define MT9V034_ANALOG_GAIN				0x35
#define		MT9V034_ANALOG_GAIN_MIN			16
#define		MT9V034_ANALOG_GAIN_DEF			16
#define		MT9V034_ANALOG_GAIN_MAX			64
#define MT9V034_BLACK_LEVEL_CALIBRATION_CONTROL		0x47
#define		MT9V034_BLACK_LEVEL_MANUAL_OVERRIDE	1
#define		MT9V034_BLACK_LEVEL_FRAMES_SHIFT	5
#define		MT9V034_BLACK_LEVEL_FRAMES_MASK		(7 << 5)
#define MT9V034_BLACK_LEVEL_CALIBRATION_VALUE	0x48
#define MT9V034_ROW_NOISE_CORR_CONTROL			0x70
#define	MT9V034_PIXEL_CLOCK					0x72
#define		MT9V034_PIXEL_CLOCK_INV_LINE		(1 << 0)
#define		MT9V034_PIXEL_CLOCK_INV_FRAME		(1 << 1)
#define		MT9V034_PIXEL_CLOCK_XOR_LINE		(1 << 2)
#define		MT9V034_PIXEL_CLOCK_CONT_LINE		(1 << 3)
#define		MT9V034_PIXEL_CLOCK_INV_PXL_CLK		(1 << 4)
#define MT9V034_TEST_PATTERN				0x7f
#define		MT9V034_TEST_PATTERN_DATA_MASK		(1023 << 0)
#define		MT9V034_TEST_PATTERN_DATA_SHIFT		0
#define		MT9V034_TEST_PATTERN_USE_DATA		(1 << 10)
#define		MT9V034_TEST_PATTERN_GRAY_MASK		(3 << 11)
#define		MT9V034_TEST_PATTERN_GRAY_NONE		(0 << 11)
#define		MT9V034_TEST_PATTERN_GRAY_VERTICAL	(1 << 11)
#define		MT9V034_TEST_PATTERN_GRAY_HORIZONTAL	(2 << 11)
#define		MT9V034_TEST_PATTERN_GRAY_DIAGONAL	(3 << 11)
#define		MT9V034_TEST_PATTERN_ENABLE		(1 << 13)
#define		MT9V034_TEST_PATTERN_FLIP		(1 << 14)
#define MT9V034_AEC_AGC_ENABLE				0xaf
#define		MT9V034_AEC_ENABLE			(1 << 0)
#define		MT9V034_AGC_ENABLE			(1 << 1)


#define V4L2_CTRL_CLASS_JPEG 0x009d0000		/* JPEG-compression controls */
#define V4L2_CTRL_CLASS_IMAGE_SOURCE 0x009e0000	/* Image source controls */
#define V4L2_CTRL_CLASS_IMAGE_PROC 0x009f0000	/* Image processing controls */


/* Image source controls */
#define V4L2_CID_IMAGE_SOURCE_CLASS_BASE	(V4L2_CTRL_CLASS_IMAGE_SOURCE | 0x900)
#define V4L2_CID_IMAGE_SOURCE_CLASS		(V4L2_CTRL_CLASS_IMAGE_SOURCE | 1)

#define V4L2_CID_VBLANK				(V4L2_CID_IMAGE_SOURCE_CLASS_BASE + 1)
#define V4L2_CID_HBLANK				(V4L2_CID_IMAGE_SOURCE_CLASS_BASE + 2)

// static char *sensor_type;
// module_param(sensor_type, charp, S_IRUGO);
// MODULE_PARM_DESC(sensor_type, "Sensor type: \"colour\" or \"monochrome\"");


static const struct soc_camera_data_format mt9v034_colour_formats[] = {
	/* Order important: first natively supported,
	 * second supported with a GPIO extender */
	{
		.name		= "Bayer (sRGB) 10 bit",
		.depth		= 10,
		.fourcc		= V4L2_PIX_FMT_SBGGR16,
		.colorspace	= V4L2_COLORSPACE_SRGB,
	}, {
		.name		= "Bayer (sRGB) 8 bit",
		.depth		= 8,
		.fourcc		= V4L2_PIX_FMT_SBGGR8,
		.colorspace	= V4L2_COLORSPACE_SRGB,
	}
};


struct mt9v034 {
	struct v4l2_subdev subdev;
	struct v4l2_rect rect;	/* Sensor window */
	__u32 fourcc;

	//struct mt9v034_platform_data *pdata;

	u32 sysclk;					//clock ,but here is fixed??
	u16 aec_agc;				//auto exposure control and auto 

	int model;	/* V4L2_IDENT_MT9V022* codes from v4l2-chip-ident.h */
	u16 chip_control;
};

static int reg_read(struct i2c_client *client, const u8 reg)
{
	s32 data = i2c_smbus_read_word_data(client, reg);
	return data < 0 ? data : swab16(data);
}

static int reg_write(struct i2c_client *client, const u8 reg, const u16 data)
{
	return i2c_smbus_write_word_data(client, reg, swab16(data));
}

static int reg_set(struct i2c_client *client, const u8 reg, const u16 data)
{
	int ret;

	ret = reg_read(client, reg);
	if (ret < 0)
		return ret;
	return reg_write(client, reg, ret | data);
}

static int reg_clear(struct i2c_client *client, const u8 reg, const u16 data)
{
	int ret;

	ret = reg_read(client, reg);
	if (ret < 0)
		return ret;
	return reg_write(client, reg, ret & ~data);
}


static struct mt9v034 *to_mt9v034(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mt9v034, subdev);
}

static struct mt9v034 *to_mt9v034_form_i2c_client(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct mt9v034, subdev);
}



// static int mt9v034_power_on(struct mt9v034 *mt9v034)
// {
// 	//step 1:soft reset
// 	//step 2:to read reset return
// 	//step 3:write 0 to chip_controls
// 	return 0;
// }

// static void mt9v034_power_off(struct mt9v034 *mt9v034)
// {
// 	//TODO:just set clock to zero
// }

// static int mt9v034_open(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
// {
// 	return 0;

// }

// static int mt9v034_close(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
// {
// 	return 0;
	
// }

static int mt9v034_set_bus_param(struct soc_camera_device *icd,
				 unsigned long flags)
{
	// struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
	// struct mt9v034 *mt9v034 = to_mt9v034_form_i2c_client(client);
	// struct soc_camera_link *icl = to_soc_camera_link(icd);
	// unsigned int width_flag = flags & SOCAM_DATAWIDTH_MASK;
	// int ret;
	// u16 pixclk = 0;

	// /* Only one width bit may be set */
	// if (!is_power_of_2(width_flag))
	// 	return -EINVAL;

	// if (icl->set_bus_param) {
	// 	ret = icl->set_bus_param(icl, width_flag);
	// 	if (ret)
	// 		return ret;
	// } else {
	// 	/*
	// 	 * Without board specific bus width settings we only support the
	// 	 * sensors native bus width
	// 	 */
	// 	if (width_flag != SOCAM_DATAWIDTH_10)
	// 		return -EINVAL;
	// }

	// flags = soc_camera_apply_sensor_flags(icl, flags);

	// if (flags & SOCAM_PCLK_SAMPLE_RISING)
	// 	pixclk |= 0x10;

	// if (!(flags & SOCAM_HSYNC_ACTIVE_HIGH))
	// 	pixclk |= 0x1;

	// if (!(flags & SOCAM_VSYNC_ACTIVE_HIGH))
	// 	pixclk |= 0x2;

	// ret = reg_write(client, MT9V034_PIXEL_CLOCK, pixclk);
	// if (ret < 0)
	// 	return ret;

	// if (!(flags & SOCAM_MASTER))
	// 	mt9v034->chip_control &= ~0x8;

	// ret = reg_write(client, MT9V034_CHIP_CONTROL, mt9v034->chip_control);
	// if (ret < 0)
	// 	return ret;

	// dev_dbg(&client->dev, "Calculated pixclk 0x%x, chip control 0x%x\n",
	// 	pixclk, mt9v034->chip_control);

	return 0;
}



static unsigned long mt9v034_query_bus_param(struct soc_camera_device *icd)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	unsigned int width_flag;

	if (icl->query_bus_param)
		width_flag = icl->query_bus_param(icl) &
			SOCAM_DATAWIDTH_MASK;
	else
		width_flag = SOCAM_DATAWIDTH_10;

	return SOCAM_PCLK_SAMPLE_RISING | SOCAM_PCLK_SAMPLE_FALLING |
		SOCAM_HSYNC_ACTIVE_HIGH | SOCAM_HSYNC_ACTIVE_LOW |
		SOCAM_VSYNC_ACTIVE_HIGH | SOCAM_VSYNC_ACTIVE_LOW |
		SOCAM_DATA_ACTIVE_HIGH | SOCAM_MASTER | SOCAM_SLAVE |
		width_flag;
}



static const struct v4l2_queryctrl mt9v034_controls[] = {
	{
		.id		= V4L2_CID_VBLANK,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Vertical Blanking",
		.minimum	= MT9V034_VERTICAL_BLANKING_MIN,
		.maximum	= MT9V034_VERTICAL_BLANKING_MAX,
		.step		= 1,
		.default_value	= MT9V034_VERTICAL_BLANKING_DEF,
	}, {
		.id		= V4L2_CID_HBLANK,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Horizontal Blanking",
		.minimum	= MT9V034_HORIZONTAL_BLANKING_MIN,
		.maximum	= MT9V034_HORIZONTAL_BLANKING_MAX,
		.step		= 1,
		.default_value	= MT9V034_HORIZONTAL_BLANKING_DEF,
	}, {
		.id		= V4L2_CID_GAIN,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Analog Gain",
		.minimum	= MT9V034_ANALOG_GAIN_MIN,
		.maximum	= MT9V034_ANALOG_GAIN_MAX,
		.step		= 1,
		.default_value	= MT9V034_ANALOG_GAIN_DEF,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id		= V4L2_CID_EXPOSURE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Exposure",
		.minimum	= MT9V034_TOTAL_SHUTTER_WIDTH_MIN,
		.maximum	= MT9V034_TOTAL_SHUTTER_WIDTH_MAX,
		.step		= 1,
		.default_value	= MT9V034_TOTAL_SHUTTER_WIDTH_DEF,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id		= V4L2_CID_AUTOGAIN,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Automatic Gain",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 1,
	}, {
		.id		= V4L2_CID_EXPOSURE_AUTO,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Automatic Exposure",
		.minimum	= V4L2_EXPOSURE_AUTO,	//0
		.maximum	= V4L2_EXPOSURE_MANUAL,	//1
		.step		= V4L2_EXPOSURE_MANUAL,	//1
		.default_value	= V4L2_EXPOSURE_MANUAL,//1
	}
};



static struct soc_camera_ops mt9v034_ops = {
	.set_bus_param		= mt9v034_set_bus_param,
	.query_bus_param	= mt9v034_query_bus_param,
	.controls			= mt9v034_controls,
	.num_controls		= ARRAY_SIZE(mt9v034_controls),
};



static int mt9v034_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = sd->priv;
	const struct v4l2_queryctrl *qctrl;
	unsigned long range;
	int data;

	printk("mt9v034_s_ctrl id=0x%x(%d) value=0x%x(%d)\n", ctrl->id, ctrl->id, ctrl->value, ctrl->value);
	qctrl = soc_camera_find_qctrl(&mt9v034_ops, ctrl->id);

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		data = reg_read(client, MT9V034_READ_MODE);
		if (data < 0)
			return -EIO;
		ctrl->value = !!(data & 0x10);
		break;
	case V4L2_CID_HBLANK:
		data = reg_read(client, MT9V034_READ_MODE);
		if (data < 0)
			return -EIO;
		ctrl->value = !!(data & 0x20);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		data = reg_read(client, MT9V034_AEC_AGC_ENABLE);
		if (data < 0)
			return -EIO;
		ctrl->value = !!(data & 0x1);
		break;
	case V4L2_CID_AUTOGAIN:
		data = reg_read(client, MT9V034_AEC_AGC_ENABLE);
		if (data < 0)
			return -EIO;
		ctrl->value = !!(data & 0x2);
		break;
	case V4L2_CID_GAIN:
		data = reg_read(client, MT9V034_ANALOG_GAIN);
		if (data < 0)
			return -EIO;

		range = qctrl->maximum - qctrl->minimum;
		ctrl->value = ((data - 16) * range + 24) / 48 + qctrl->minimum;

		break;
	case V4L2_CID_EXPOSURE:
		data = reg_read(client, MT9V034_TOTAL_SHUTTER_WIDTH);
		if (data < 0)
			return -EIO;

		range = qctrl->maximum - qctrl->minimum;
		ctrl->value = ((data - 1) * range + 239) / 479 + qctrl->minimum;

		break;
	}
	return 0;
}

static int mt9v034_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int data;
	struct mt9v034 *mt9v034 = container_of(sd, struct mt9v034, subdev);
	struct i2c_client *client = sd->priv;
	const struct v4l2_queryctrl *qctrl;

	printk("mt9v034_s_ctrl id=0x%x(%d) value=0x%x(%d)\n", ctrl->id, ctrl->id, ctrl->value, ctrl->value);

	qctrl = soc_camera_find_qctrl(&mt9v034_ops, ctrl->id);
	if (!qctrl)
		return -EINVAL;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		return reg_write(client, MT9V034_VERTICAL_BLANKING,ctrl->value); 
	case V4L2_CID_HBLANK:
		return reg_write(client, MT9V034_HORIZONTAL_BLANKING,
			     max_t(s32, ctrl->value,
			     MT9V034_TOTAL_ROW_TIME_MIN - mt9v034->rect.width));
	case V4L2_CID_GAIN:
			return reg_write(client, MT9V034_ANALOG_GAIN, ctrl->value);
	case V4L2_CID_EXPOSURE:
			return reg_write(client, MT9V034_TOTAL_SHUTTER_WIDTH,ctrl->value);
	case V4L2_CID_AUTOGAIN:
		if (ctrl->value)
			data = reg_set(client, MT9V034_AEC_AGC_ENABLE, 0x2);
		else
			data = reg_clear(client, MT9V034_AEC_AGC_ENABLE, 0x2);
		if (data < 0)
			return -EIO;
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		if (ctrl->value)
			data = reg_set(client, MT9V034_AEC_AGC_ENABLE, 0x1);
		else
			data = reg_clear(client, MT9V034_AEC_AGC_ENABLE, 0x1);
		if (data < 0)
			return -EIO;
		break;
	}
	return 0;
}


static int mt9v034_g_chip_ident(struct v4l2_subdev *sd,
				struct v4l2_dbg_chip_ident *id)
{
	struct i2c_client *client = sd->priv;
	struct mt9v034 *mt9v034 = to_mt9v034(sd);

	if (id->match.type != V4L2_CHIP_MATCH_I2C_ADDR)
		return -EINVAL;

	if (id->match.addr != client->addr)
		return -ENODEV;

	id->ident	= mt9v034->model;
	id->revision	= 0;

	return 0;
}



#ifdef CONFIG_VIDEO_ADV_DEBUG
static int mt9v034_g_register(struct v4l2_subdev *sd,
			      struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = sd->priv;

	if (reg->match.type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
		return -EINVAL;

	if (reg->match.addr != client->addr)
		return -ENODEV;

	reg->size = 2;
	reg->val = reg_read(client, reg->reg);

	if (reg->val > 0xffff)
		return -EIO;

	return 0;
}

static int mt9v034_s_register(struct v4l2_subdev *sd,
			      struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = sd->priv;

	if (reg->match.type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
		return -EINVAL;

	if (reg->match.addr != client->addr)
		return -ENODEV;

	if (reg_write(client, reg->reg, reg->val) < 0)
		return -EIO;

	return 0;
}
#endif



static int mt9v034_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = sd->priv;
	struct mt9v034 *mt9v034 = to_mt9v034(sd);

	if (enable)
		/* Switch to master "normal" mode */
		mt9v034->chip_control &= ~0x10;
	else
		/* Switch to snapshot mode */
		mt9v034->chip_control |= 0x10;

	if (reg_write(client, MT9V034_CHIP_CONTROL, mt9v034->chip_control) < 0)
		return -EIO;
	return 0;
}


static int mt9v034_s_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	struct i2c_client *client = sd->priv;
	struct mt9v034 *mt9v034 = to_mt9v034(sd);
	struct v4l2_rect rect = a->c;
	struct soc_camera_device *icd = client->dev.platform_data;
	int ret;

	/* Bayer format - even size lengths */
	if (mt9v034->fourcc == V4L2_PIX_FMT_SBGGR8 ||
	    mt9v034->fourcc == V4L2_PIX_FMT_SBGGR16) {
		rect.width	= ALIGN(rect.width, 2);
		rect.height	= ALIGN(rect.height, 2);
		/* Let the user play with the starting pixel */
	}

	soc_camera_limit_side(&rect.left, &rect.width,
		     MT9V034_COLUMN_START_MIN, MT9V034_WINDOW_WIDTH_MIN, MT9V034_WINDOW_WIDTH_MAX);

	soc_camera_limit_side(&rect.top, &rect.height,
		     MT9V034_ROW_START_MIN, MT9V034_WINDOW_HEIGHT_MIN, MT9V034_WINDOW_HEIGHT_MAX);

	/* Like in example app. Contradicts the datasheet though */
	ret = reg_read(client, MT9V034_AEC_AGC_ENABLE);
	if (ret >= 0) {
		if (ret & 1) /* Autoexposure */
			ret = reg_write(client, MT9V034_MAX_TOTAL_SHUTTER_WIDTH,
					rect.height + icd->y_skip_top + 43);
		else
			ret = reg_write(client, MT9V034_TOTAL_SHUTTER_WIDTH,
					rect.height + icd->y_skip_top + 43);
	}
	/* Setup frame format: defaults apart from width and height */
	if (!ret)
		ret = reg_write(client, MT9V034_COLUMN_START, rect.left);
	if (!ret)
		ret = reg_write(client, MT9V034_ROW_START, rect.top);
	if (!ret)
		/* Default 94, Phytec driver says:
		 * "width + horizontal blank >= 660" */
		ret = reg_write(client, MT9V034_HORIZONTAL_BLANKING,
				rect.width > 660 - 43 ? 43 :
				660 - rect.width);
	if (!ret)
		ret = reg_write(client, MT9V034_VERTICAL_BLANKING, 45);
	if (!ret)
		ret = reg_write(client, MT9V034_WINDOW_WIDTH, rect.width);
	if (!ret)
		ret = reg_write(client, MT9V034_WINDOW_HEIGHT,
				rect.height + icd->y_skip_top);

	if (ret < 0)
		return ret;

	dev_dbg(&client->dev, "Frame %ux%u pixel\n", rect.width, rect.height);

	mt9v034->rect = rect;

	return 0;
}

static int mt9v034_s_fmt(struct v4l2_subdev *sd, struct v4l2_format *f)
{
	struct mt9v034 *mt9v034 = to_mt9v034(sd);
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_crop a = {
		.c = {
			.left	= mt9v034->rect.left,
			.top	= mt9v034->rect.top,
			.width	= pix->width,
			.height	= pix->height,
		},
	};
	int ret;

	/* The caller provides a supported format, as verified per call to
	 * icd->try_fmt(), datawidth is from our supported format list */
	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_GREY:
	case V4L2_PIX_FMT_Y16:
		if (mt9v034->model != V4L2_IDENT_MT9V022IX7ATM)
			return -EINVAL;
		break;
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SBGGR16:
		if (mt9v034->model != V4L2_IDENT_MT9V022IX7ATC)
			return -EINVAL;
		break;
	case 0:
		/* No format change, only geometry */
		break;
	default:
		return -EINVAL;
	}

	/* No support for scaling on this camera, just crop. */
	ret = mt9v034_s_crop(sd, &a);
	if (!ret) {
		pix->width = mt9v034->rect.width;
		pix->height = mt9v034->rect.height;
		mt9v034->fourcc = pix->pixelformat;
	}

	return ret;
}

static int mt9v034_g_fmt(struct v4l2_subdev *sd, struct v4l2_format *f)
{
	struct mt9v034 *mt9v034 = to_mt9v034(sd);
	struct v4l2_pix_format *pix = &f->fmt.pix;

	pix->width			= mt9v034->rect.width;
	pix->height			= mt9v034->rect.height;
	pix->pixelformat	= mt9v034->fourcc;
	pix->field			= V4L2_FIELD_NONE;
	pix->colorspace		= V4L2_COLORSPACE_SRGB;

	return 0;
}


static int mt9v034_try_fmt(struct v4l2_subdev *sd, struct v4l2_format *f)
{
	struct i2c_client *client = sd->priv;
	struct soc_camera_device *icd = client->dev.platform_data;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int align = pix->pixelformat == V4L2_PIX_FMT_SBGGR8 ||
		pix->pixelformat == V4L2_PIX_FMT_SBGGR16;

	v4l_bound_align_image(&pix->width, MT9V034_WINDOW_WIDTH_MIN,
		MT9V034_WINDOW_WIDTH_MAX, align,
		&pix->height, MT9V034_WINDOW_HEIGHT_MIN + icd->y_skip_top,
		MT9V034_WINDOW_HEIGHT_MAX + icd->y_skip_top, align, 0);

	return 0;
}




//check
static int mt9v034_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	struct mt9v034 *mt9v034 = to_mt9v034(sd);

	a->c	= mt9v034->rect;
	a->type	= V4L2_BUF_TYPE_VIDEO_CAPTURE;

	return 0;
}


//check
static int mt9v034_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	a->bounds.left			= MT9V034_COLUMN_START_DEF;
	a->bounds.top			= MT9V034_ROW_START_DEF;
	a->bounds.width			= MT9V034_WINDOW_WIDTH_DEF;
	a->bounds.height		= MT9V034_WINDOW_HEIGHT_DEF;
	a->defrect				= a->bounds;
	a->type					= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator	= 1;
	a->pixelaspect.denominator	= 1;

	return 0;
}

//uncheck
static int mt9v034_init(struct i2c_client *client)
{
	struct mt9v034 *mt9v034 = to_mt9v034_form_i2c_client(client);
	int ret;

	/* Almost the default mode: master, parallel, simultaneous, and an
	 * undocumented bit 0x200, which is present in table 7, but not in 8,
	 * plus snapshot mode to disable scan for now */
	mt9v034->chip_control |= MT9V034_CHIP_CONTROL_SNAPSHOT_MODE;
	ret = reg_write(client, MT9V034_CHIP_CONTROL, mt9v034->chip_control);
	if (!ret)
		ret = reg_write(client, MT9V034_READ_MODE, MT9V034_READ_MODE_DEFAULT_VALUE);	//datasheet

	/* All defaults */
	if (!ret)
		/* AEC, AGC on */
		ret = reg_set(client, MT9V034_AEC_AGC_ENABLE, MT9V034_AEC_ENABLE | MT9V034_AGC_ENABLE);
	if (!ret)
		ret = reg_write(client, MT9V034_ANALOG_GAIN, MT9V034_ANALOG_GAIN_DEF);
	if (!ret)
		ret = reg_write(client, MT9V034_TOTAL_SHUTTER_WIDTH, MT9V034_TOTAL_SHUTTER_WIDTH);
	if (!ret)
		ret = reg_write(client, MT9V034_MAX_TOTAL_SHUTTER_WIDTH, MT9V034_MAX_TOTAL_SHUTTER_WIDTH_DEF);
	if (!ret)
		/* default - auto */
		ret = reg_clear(client, MT9V034_BLACK_LEVEL_CALIBRATION_CONTROL, MT9V034_BLACK_LEVEL_MANUAL_OVERRIDE);
	if (!ret)
		ret = reg_write(client, MT9V034_TEST_PATTERN, MT9V034_TEST_PATTERN_DATA_SHIFT);

	return ret;
}


static int mt9v034_video_probe(struct soc_camera_device *icd,
			       struct i2c_client *client)
{
	struct mt9v034 *mt9v034 = to_mt9v034_form_i2c_client(client);
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	s32 data;
	int ret;
	unsigned long flags;

	printk("I just want to chcek to client ->adapter->id:%d\r\n",client->adapter->id);
	dev_info(&client->dev, "Probing MT9V034 at address 0x%02x\n",client->addr);

	//FIXME:check power on ??

	//
	// if (!icd->dev.parent || to_soc_camera_host(icd->dev.parent)->nr != icd->iface)
	// 	return -ENODEV;

	/* Read out the chip version register */
	data = reg_read(client, MT9V034_CHIP_VERSION);

	printk("What I read from mt9v034 is %x\r\n",data);

	/* must be 0x1324 */
	if (data != MT9V034_CHIP_ID_REV1) {
		ret = -ENODEV;
		dev_info(&client->dev, "No MT9V034 found, ID register 0x%x\n",data);
		goto ei2c;
	}

	/* Soft reset */
	ret = reg_write(client, MT9V034_RESET, 1);
	if (ret < 0)
		goto ei2c;
	/* 15 clock cycles */
	udelay(200);
	if (reg_read(client, MT9V034_RESET)) {
		dev_err(&client->dev, "Resetting MT9V034 failed!\n");
		if (ret > 0)
			ret = -EIO;
		goto ei2c;
	}

	/* Set monochrome or colour sensor type */
	// if (sensor_type && (!strcmp("colour", sensor_type) || !strcmp("color", sensor_type))) {
	// 	ret = reg_write(client, MT9V034_SENSOR_TYPE_CONTROL, 4 | 0x11);
	// 	mt9v022->model = V4L2_IDENT_MT9V022IX7ATC;
	// 	icd->formats = mt9v022_colour_formats;
	// } 
	// else {
	// 	ret = reg_write(client, MT9V034_SENSOR_TYPE_CONTROL, 0x11);
	// 	mt9v022->model = V4L2_IDENT_MT9V022IX7ATM;
	// 	icd->formats = mt9v022_monochrome_formats;
	// }

	// if (ret < 0)
	// 	goto ei2c;

	//defaule value is 0x0011 
	//FIXME:V4L2_IDENT_MT9V022IX7ATC 是022的芯片ID，暂时找不到034芯片的ID
	ret = reg_write(client, MT9V034_SENSOR_TYPE_CONTROL, MT9V034_SENSOR_TYPE_MODE_COLOR | 0x11);
	mt9v034->model = V4L2_IDENT_MT9V022IX7ATC;
	icd->formats = mt9v034_colour_formats;

	icd->num_formats = 0;

	/*
	 * This is a 10bit sensor, so by default we only allow 10bit.
	 * The platform may support different bus widths due to
	 * different routing of the data lines.
	 */
	if (icl->query_bus_param)
		flags = icl->query_bus_param(icl);
	else
		flags = SOCAM_DATAWIDTH_10;

	if (flags & SOCAM_DATAWIDTH_10)
		icd->num_formats++;
	else
		icd->formats++;

	if (flags & SOCAM_DATAWIDTH_8)
		icd->num_formats++;

	mt9v034->fourcc = icd->formats->fourcc;

	dev_info(&client->dev, "Detected a MT9V034 chip ID %x, %s sensor\n",
		 data, mt9v034->model == V4L2_IDENT_MT9V022IX7ATM ?
		 "monochrome" : "colour");

	ret = mt9v034_init(client);
	if (ret < 0)
		dev_err(&client->dev, "Failed to initialise the camera\n");

ei2c:
	return ret;
}


static void mt9v034_video_remove(struct soc_camera_device *icd)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);

	dev_dbg(&icd->dev, "Video removed: %p, %p\n",icd->dev.parent, icd->vdev);
	if (icl->free_bus)
		icl->free_bus(icl);
}


//check
static struct v4l2_subdev_core_ops mt9v034_subdev_core_ops = {
	.g_ctrl		= mt9v034_g_ctrl,
	.s_ctrl		= mt9v034_s_ctrl,
	.g_chip_ident	= mt9v034_g_chip_ident,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= mt9v034_g_register,
	.s_register	= mt9v034_s_register,
#endif
};


//check
static struct v4l2_subdev_video_ops mt9v034_subdev_video_ops = {
	.s_stream	= mt9v034_s_stream,
	.s_fmt		= mt9v034_s_fmt,
	.g_fmt		= mt9v034_g_fmt,
	.try_fmt	= mt9v034_try_fmt,
	.s_crop		= mt9v034_s_crop,
	.g_crop		= mt9v034_g_crop,
	.cropcap	= mt9v034_cropcap,
};

//check
static struct v4l2_subdev_ops mt9v034_subdev_ops = {
	.core	= &mt9v034_subdev_core_ops,
	.video	= &mt9v034_subdev_video_ops,
};



static int mt9v034_probe(struct i2c_client *client,const struct i2c_device_id *did)
{
	struct mt9v034 *mt9v034;
	struct soc_camera_device *icd = client->dev.platform_data;
	//struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	//struct i2c_adapter *adapter = i2c_get_adapter(2);
	struct soc_camera_link *icl;

	int ret;

	printk(KERN_INFO "mt9v034 probe\r\n");

	if (!icd) {
		dev_err(&client->dev, "MT9V034: missing soc-camera data!\n");
		return -EINVAL;
	}

	//to get icd from soc_camera_device->dev->platform_data;
	icl = to_soc_camera_link(icd);

	if (!icl) {
		dev_err(&client->dev, "MT9V034 driver needs platform data\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter,I2C_FUNC_SMBUS_WORD_DATA)) {
		printk("I2C-Adapter doesn't support I2C_FUNC_SMBUS_WORD\n");
		return -EIO;
	}

	mt9v034 = kzalloc(sizeof(struct mt9v034), GFP_KERNEL);
	if (!mt9v034)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&mt9v034->subdev, client, &mt9v034_subdev_ops);

	//here we set 0x07(chip controls) is Progressive scan
	//and Master mode and parallel output and Simultaneous mode.
	mt9v034->chip_control = MT9V034_CHIP_CONTROL_DEFAULT;

	icd->ops		= &mt9v034_ops;

	icd->y_skip_top		= 1;

	mt9v034->rect.left		= MT9V034_COLUMN_START_DEF;
	mt9v034->rect.top		= MT9V034_ROW_START_DEF;
	mt9v034->rect.width		= MT9V034_WINDOW_WIDTH_DEF;
	mt9v034->rect.height	= MT9V034_WINDOW_HEIGHT_DEF;

	//need to fix 
	ret = mt9v034_video_probe(icd, client);
	if (ret) {
		icd->ops = NULL;
		i2c_set_clientdata(client, NULL);
		kfree(mt9v034);
	}
	return ret;
}





static int mt9v034_remove(struct i2c_client *client)
{
	struct mt9v034 *mt9v034 = i2c_get_clientdata(client);
	struct soc_camera_device *icd = client->dev.platform_data;

	
	icd->ops = NULL;
	//need to fix 
	mt9v034_video_remove(icd);
	i2c_set_clientdata(client, NULL);
	client->driver = NULL;
	kfree(mt9v034);

	return 0;
}


static const struct i2c_device_id mt9v034_id[] = {
	{ MT9V034_MODULE_NAME, NULL },
	{ }
};

MODULE_DEVICE_TABLE(i2c, mt9v034_id);

static struct i2c_driver mt9v034_driver = {
	.driver = {
		.name = MT9V034_MODULE_NAME,
		.owner = THIS_MODULE,
	},
	.probe		= mt9v034_probe,
	.remove		= mt9v034_remove,
	.id_table	= mt9v034_id,
};


static int __init mt9v034_mod_init(void)
{
	printk("mt9v034_mode init\r\n");
	return i2c_add_driver(&mt9v034_driver);
}

static void __exit mt9v034_mod_exit(void)
{
	printk("mt9v034 remove\r\n");
	i2c_del_driver(&mt9v034_driver);
}


module_init(mt9v034_mod_init);
module_exit(mt9v034_mod_exit);

MODULE_DESCRIPTION("Aptina MT9V034 Camera driver");
MODULE_AUTHOR("CareDrive");
MODULE_LICENSE("GPL");
