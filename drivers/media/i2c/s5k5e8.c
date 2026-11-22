// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2026 Andrii Cherniavskyi <chernyav.a@gmail.com>
// Copyright (c) 2026 Tuan Kiet Huynh <huynhtuankiet33@gmail.com>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define S5K5E8_LINK_FREQ_420MHZ		(420ULL * HZ_PER_MHZ)
#define S5K5E8_MCLK_FREQ_24MHZ		(24 * HZ_PER_MHZ)
#define S5K5E8_DATA_LANES		2
#define S5K5E8_NATIVE_WIDTH		2608
#define S5K5E8_NATIVE_HEIGHT		1960

#define S5K5E8_REG_CHIP_ID		CCI_REG16(0x0000)
#define S5K5E8_CHIP_ID			0x5e80

#define S5K5E8_REG_CTRL_MODE		CCI_REG8(0x0100)
#define S5K5E8_MODE_STREAMING		BIT(0)

#define S5K5E8_REG_ORIENTATION		CCI_REG8(0x0101)
#define S5K5E8_VFLIP			BIT(1)
#define S5K5E8_HFLIP			BIT(0)

#define S5K5E8_REG_EXPOSURE		CCI_REG16(0x0202)
#define S5K5E8_EXPOSURE_MIN		8
#define S5K5E8_EXPOSURE_STEP		1
#define S5K5E8_EXPOSURE_MARGIN		4

#define S5K5E8_REG_AGAIN		CCI_REG16(0x0204)
#define S5K5E8_AGAIN_MIN		0x020
#define S5K5E8_AGAIN_MAX		0x200
#define S5K5E8_AGAIN_STEP		1
#define S5K5E8_AGAIN_DEFAULT		0x020

#define S5K5E8_REG_VTS			CCI_REG16(0x0340)
#define S5K5E8_VTS_MAX			0xffff
#define S5K5E8_REG_HTS			CCI_REG16(0x0342)
#define S5K5E8_REG_X_ADDR_START		CCI_REG16(0x0344)
#define S5K5E8_REG_Y_ADDR_START		CCI_REG16(0x0346)
#define S5K5E8_REG_X_ADDR_END		CCI_REG16(0x0348)
#define S5K5E8_REG_Y_ADDR_END		CCI_REG16(0x034a)
#define S5K5E8_REG_X_OUTPUT_SIZE	CCI_REG16(0x034c)
#define S5K5E8_REG_Y_OUTPUT_SIZE	CCI_REG16(0x034e)

#define S5K5E8_REG_TEST_PATTERN		CCI_REG8(0x0601)

#define to_s5k5e8(_sd)			container_of(_sd, struct s5k5e8, sd)

static const s64 s5k5e8_link_freq_menu[] = {
	S5K5E8_LINK_FREQ_420MHZ,
};

/* List of supported formats to cover horizontal and vertical flip controls */
static const u32 s5k5e8_mbus_formats[] = {
	MEDIA_BUS_FMT_SGRBG10_1X10,	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,	MEDIA_BUS_FMT_SGBRG10_1X10,
};

struct s5k5e8_reg_list {
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

struct s5k5e8_mode {
	u32 width;				/* Frame width in pixels */
	u32 height;				/* Frame height in pixels */
	u32 hts;				/* Horizontal timing size */
	u32 vts;				/* Default vertical timing size */
	u32 exposure;				/* Default exposure value */
	struct v4l2_rect crop;			/* Sensor array crop rect */
	struct s5k5e8_reg_list reg_list;	/* Sensor register setting */
};

static const char *const s5k5e8_test_pattern_menu[] = {
	"Disabled",
	"Solid colour",
	"Colour bars",
	"Fade to grey colour bars",
};

static const char *const s5k5e8_supply_names[] = {
	"vddio",	/* Digital I/O power */
	"vdda",		/* Analog power */
	"vddd",		/* Digital core power */
};

#define S5K5E8_NUM_SUPPLIES ARRAY_SIZE(s5k5e8_supply_names)

struct s5k5e8 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *powerdown_gpio;
	struct regulator_bulk_data supplies[S5K5E8_NUM_SUPPLIES];

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct {
		/* hflip/vflip cluster */
		struct v4l2_ctrl *hflip;
		struct v4l2_ctrl *vflip;
	};
};

/*
 * Vendor analog and timing initialization sequence (including repeated
 * internal analog/timing writes required by the vendor power-up flow).
 */
static const struct cci_reg_sequence init_array_setting[] = {
	{ CCI_REG8(0x0100), 0x00 },
	{ CCI_REG8(0x3906), 0x7e },
	{ CCI_REG8(0x3c01), 0x0f },
	{ CCI_REG8(0x3c14), 0x00 },
	{ CCI_REG8(0x3235), 0x08 },
	{ CCI_REG8(0x3063), 0x2e },
	{ CCI_REG8(0x307a), 0x10 },
	{ CCI_REG8(0x307b), 0x0e },
	{ CCI_REG8(0x3079), 0x20 },
	{ CCI_REG8(0x3070), 0x05 },
	{ CCI_REG8(0x3067), 0x06 },
	{ CCI_REG8(0x3071), 0x62 },
	{ CCI_REG8(0x3072), 0x16 },
	{ CCI_REG8(0x3203), 0x43 },
	{ CCI_REG8(0x3205), 0x43 },
	{ CCI_REG8(0x320b), 0x42 },
	{ CCI_REG8(0x3007), 0x00 },
	{ CCI_REG8(0x3008), 0x14 },
	{ CCI_REG8(0x3020), 0x58 },
	{ CCI_REG8(0x300d), 0x34 },
	{ CCI_REG8(0x300e), 0x17 },
	{ CCI_REG8(0x3021), 0x02 },
	{ CCI_REG8(0x3010), 0x59 },
	{ CCI_REG8(0x3002), 0x01 },
	{ CCI_REG8(0x3005), 0x01 },
	{ CCI_REG8(0x3008), 0x04 },
	{ CCI_REG8(0x300f), 0x70 },
	{ CCI_REG8(0x3010), 0x69 },
	{ CCI_REG8(0x3017), 0x10 },
	{ CCI_REG8(0x3019), 0x19 },
	{ CCI_REG8(0x300c), 0x62 },
	{ CCI_REG8(0x3064), 0x10 },
	{ CCI_REG8(0x3c08), 0x0e },
	{ CCI_REG8(0x3c09), 0x10 },
	{ CCI_REG8(0x3c31), 0x0d },
	{ CCI_REG8(0x3c32), 0xac },
};

static const struct cci_reg_sequence s5k5e8_2592x1944_30fps_mode[] = {
	{ CCI_REG8(0x0136), 0x18 },
	{ CCI_REG8(0x0137), 0x00 },
	{ CCI_REG8(0x0305), 0x06 },
	{ CCI_REG8(0x0306), 0x18 },
	{ CCI_REG8(0x0307), 0xa8 },
	{ CCI_REG8(0x0308), 0x34 },
	{ CCI_REG8(0x0309), 0x42 },
	{ CCI_REG8(0x3c1f), 0x00 },
	{ CCI_REG8(0x3c17), 0x00 },
	{ CCI_REG8(0x3c0b), 0x04 },
	{ CCI_REG8(0x3c1c), 0x47 },
	{ CCI_REG8(0x3c1d), 0x15 },
	{ CCI_REG8(0x3c14), 0x04 },
	{ CCI_REG8(0x3c16), 0x00 },
	{ CCI_REG8(0x0820), 0x03 },
	{ CCI_REG8(0x0821), 0x44 },
	{ CCI_REG8(0x0114), 0x01 },
	{ S5K5E8_REG_X_ADDR_START,  0x0008 },
	{ S5K5E8_REG_Y_ADDR_START,  0x0008 },
	{ S5K5E8_REG_X_ADDR_END,    0x0a27 },
	{ S5K5E8_REG_Y_ADDR_END,    0x079f },
	{ S5K5E8_REG_X_OUTPUT_SIZE, 0x0a20 },
	{ S5K5E8_REG_Y_OUTPUT_SIZE, 0x0798 },
	{ CCI_REG8(0x0900), 0x00 },
	{ CCI_REG8(0x0901), 0x00 },
	{ CCI_REG8(0x0381), 0x01 },
	{ CCI_REG8(0x0383), 0x01 },
	{ CCI_REG8(0x0385), 0x01 },
	{ CCI_REG8(0x0387), 0x01 },
	{ S5K5E8_REG_VTS, 0x07b0 },
	{ S5K5E8_REG_HTS, 0x0b28 },
	{ CCI_REG8(0x0200), 0x00 },
	{ CCI_REG8(0x0201), 0x00 },
	{ S5K5E8_REG_EXPOSURE, 0x03de },
	{ CCI_REG8(0x3303), 0x02 },
	{ CCI_REG8(0x3400), 0x00 },
	{ CCI_REG8(0x323b), 0x02 },
	{ CCI_REG8(0x3301), 0x00 },
	{ CCI_REG8(0x3321), 0x04 },
	{ CCI_REG8(0x3306), 0x00 },
	{ CCI_REG8(0x3307), 0x08 },
	{ CCI_REG8(0x3308), 0x0a },
	{ CCI_REG8(0x3309), 0x27 },
	{ CCI_REG8(0x330a), 0x01 },
	{ CCI_REG8(0x330b), 0x01 },
	{ CCI_REG8(0x330e), 0x00 },
	{ CCI_REG8(0x330f), 0x08 },
	{ CCI_REG8(0x3310), 0x07 },
	{ CCI_REG8(0x3311), 0x9f },
	{ CCI_REG8(0x3312), 0x01 },
	{ CCI_REG8(0x3313), 0x01 },
};

static const struct cci_reg_sequence s5k5e8_1304x980_60fps_mode[] = {
	{ CCI_REG8(0x0136), 0x18 },
	{ CCI_REG8(0x0137), 0x00 },
	{ CCI_REG8(0x0305), 0x06 },
	{ CCI_REG8(0x0306), 0x18 },
	{ CCI_REG8(0x0307), 0xa8 },
	{ CCI_REG8(0x0308), 0x34 },
	{ CCI_REG8(0x0309), 0x42 },
	{ CCI_REG8(0x3c1f), 0x00 },
	{ CCI_REG8(0x3c17), 0x00 },
	{ CCI_REG8(0x3c0b), 0x04 },
	{ CCI_REG8(0x3c1c), 0x47 },
	{ CCI_REG8(0x3c1d), 0x15 },
	{ CCI_REG8(0x3c14), 0x04 },
	{ CCI_REG8(0x3c16), 0x00 },
	{ CCI_REG8(0x0820), 0x03 },
	{ CCI_REG8(0x0821), 0x44 },
	{ CCI_REG8(0x0114), 0x01 },
	{ S5K5E8_REG_X_ADDR_START,  0x0000 },
	{ S5K5E8_REG_Y_ADDR_START,  0x0000 },
	{ S5K5E8_REG_X_ADDR_END,    0x0a2f },
	{ S5K5E8_REG_Y_ADDR_END,    0x07a7 },
	{ S5K5E8_REG_X_OUTPUT_SIZE, 0x0518 },
	{ S5K5E8_REG_Y_OUTPUT_SIZE, 0x03d4 },
	{ CCI_REG8(0x0900), 0x01 },
	{ CCI_REG8(0x0901), 0x22 },
	{ CCI_REG8(0x0381), 0x01 },
	{ CCI_REG8(0x0383), 0x01 },
	{ CCI_REG8(0x0385), 0x01 },
	{ CCI_REG8(0x0387), 0x03 },
	{ S5K5E8_REG_VTS, 0x03f1 },
	{ S5K5E8_REG_HTS, 0x0ae0 },
	{ CCI_REG8(0x0200), 0x00 },
	{ CCI_REG8(0x0201), 0x00 },
	{ S5K5E8_REG_EXPOSURE, 0x03de },
	{ CCI_REG8(0x3303), 0x02 },
	{ CCI_REG8(0x3400), 0x00 },
	{ CCI_REG8(0x323b), 0x02 },
	{ CCI_REG8(0x3301), 0x00 },
	{ CCI_REG8(0x3321), 0x04 },
	{ CCI_REG8(0x3306), 0x00 },
	{ CCI_REG8(0x3307), 0x00 },
	{ CCI_REG8(0x3308), 0x0a },
	{ CCI_REG8(0x3309), 0x2f },
	{ CCI_REG8(0x330a), 0x01 },
	{ CCI_REG8(0x330b), 0x01 },
	{ CCI_REG8(0x330e), 0x00 },
	{ CCI_REG8(0x330f), 0x00 },
	{ CCI_REG8(0x3310), 0x07 },
	{ CCI_REG8(0x3311), 0xa7 },
	{ CCI_REG8(0x3312), 0x03 },
	{ CCI_REG8(0x3313), 0x01 },
};

/*
 * Both modes share the same PLL setup (the binned mode uses the full-mode
 * link setup matching Samsung OEM binned preview): 168 Mpix/s VT pixel array
 * rate (2856 * 1968 @ 29.89 fps in full mode, 2784 * 1009 @ 59.81 fps in 2x2
 * binned mode) and 2-lane CSI-2 output (~836 Mbps/lane via 0x0820/0x0821,
 * ~418 MHz actual / 420 MHz nominal link frequency).
 */
static const struct s5k5e8_mode s5k5e8_supported_modes[] = {
	{
		.width = 2592,
		.height = 1944,
		.vts = 1968,
		.hts = 2856,
		.exposure = 990,
		.crop = {
			.left = 8,
			.top = 8,
			.width = 2592,
			.height = 1944,
		},
		.reg_list = {
			.regs = s5k5e8_2592x1944_30fps_mode,
			.num_regs = ARRAY_SIZE(s5k5e8_2592x1944_30fps_mode),
		},
	},
	{
		.width = 1304,
		.height = 980,
		.vts = 1009,
		.hts = 2784,
		.exposure = 990,
		.crop = {
			.left = 0,
			.top = 0,
			.width = 2608,
			.height = 1960,
		},
		.reg_list = {
			.regs = s5k5e8_1304x980_60fps_mode,
			.num_regs = ARRAY_SIZE(s5k5e8_1304x980_60fps_mode),
		},
	},
};

static u32 s5k5e8_get_format_code(bool hflip, bool vflip)
{
	return s5k5e8_mbus_formats[(vflip ? 2 : 0) | (hflip ? 1 : 0)];
}

static const struct s5k5e8_mode *
s5k5e8_get_mode_from_state(struct v4l2_subdev_state *state)
{
	const struct v4l2_mbus_framefmt *fmt;

	fmt = v4l2_subdev_state_get_format(state, 0);

	return v4l2_find_nearest_size(s5k5e8_supported_modes,
				      ARRAY_SIZE(s5k5e8_supported_modes),
				      width, height, fmt->width, fmt->height);
}

static int s5k5e8_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k5e8 *s5k5e8 =
		container_of(ctrl->handler, struct s5k5e8, ctrl_handler);
	struct v4l2_subdev_state *state;
	const struct s5k5e8_mode *mode;
	s64 exposure_max, exposure_def;
	int ret;

	state = v4l2_subdev_get_locked_active_state(&s5k5e8->sd);
	if (WARN_ON_ONCE(!state))
		return -EINVAL;

	mode = s5k5e8_get_mode_from_state(state);

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		exposure_max =
			mode->height + ctrl->val - S5K5E8_EXPOSURE_MARGIN;
		exposure_def = min_t(s64, mode->exposure, exposure_max);
		ret = __v4l2_ctrl_modify_range(s5k5e8->exposure,
					       s5k5e8->exposure->minimum,
					       exposure_max,
					       s5k5e8->exposure->step,
					       exposure_def);
		if (ret)
			return ret;
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		v4l2_subdev_state_get_format(state, 0)->code =
			s5k5e8_get_format_code(s5k5e8->hflip->val,
					       s5k5e8->vflip->val);
		break;
	}

	/* V4L2 controls are applied when sensor is powered up for streaming */
	if (pm_runtime_get_if_in_use(s5k5e8->dev) == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(s5k5e8->regmap, S5K5E8_REG_AGAIN, ctrl->val,
				NULL);
		break;
	case V4L2_CID_EXPOSURE:
		ret = cci_write(s5k5e8->regmap, S5K5E8_REG_EXPOSURE,
				ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(s5k5e8->regmap, S5K5E8_REG_VTS,
				ctrl->val + mode->height, NULL);
		break;
	case V4L2_CID_VFLIP:
	case V4L2_CID_HFLIP:
		ret = cci_write(s5k5e8->regmap, S5K5E8_REG_ORIENTATION,
				(s5k5e8->vflip->val ? S5K5E8_VFLIP : 0) |
				(s5k5e8->hflip->val ? S5K5E8_HFLIP : 0),
				NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_write(s5k5e8->regmap, S5K5E8_REG_TEST_PATTERN,
				ctrl->val, NULL);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(s5k5e8->dev);

	return ret;
}

static const struct v4l2_ctrl_ops s5k5e8_ctrl_ops = {
	.s_ctrl = s5k5e8_set_ctrl,
};

static inline u64 s5k5e8_freq_to_pixel_rate(const u64 freq)
{
	return div_u64(freq * 2 * S5K5E8_DATA_LANES, 10);
}

static int s5k5e8_init_controls(struct s5k5e8 *s5k5e8)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &s5k5e8->ctrl_handler;
	const struct s5k5e8_mode *mode = &s5k5e8_supported_modes[0];
	s64 pixel_rate, hblank, vblank, exposure_max;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	int ret;

	v4l2_ctrl_handler_init(ctrl_hdlr, 11);

	ctrl = v4l2_ctrl_new_int_menu(ctrl_hdlr, NULL, V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(s5k5e8_link_freq_menu) - 1,
				      0, s5k5e8_link_freq_menu);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	pixel_rate = s5k5e8_freq_to_pixel_rate(s5k5e8_link_freq_menu[0]);
	v4l2_ctrl_new_std(ctrl_hdlr, NULL, V4L2_CID_PIXEL_RATE,
			  pixel_rate, pixel_rate, 1, pixel_rate);

	hblank = mode->hts - mode->width;
	s5k5e8->hblank = v4l2_ctrl_new_std(ctrl_hdlr, NULL, V4L2_CID_HBLANK,
					   hblank, hblank, 1, hblank);
	if (s5k5e8->hblank)
		s5k5e8->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank = mode->vts - mode->height;
	s5k5e8->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5k5e8_ctrl_ops,
					   V4L2_CID_VBLANK, vblank,
					   S5K5E8_VTS_MAX - mode->height, 1,
					   vblank);

	v4l2_ctrl_new_std(ctrl_hdlr, &s5k5e8_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  S5K5E8_AGAIN_MIN, S5K5E8_AGAIN_MAX,
			  S5K5E8_AGAIN_STEP, S5K5E8_AGAIN_DEFAULT);

	exposure_max = mode->vts - S5K5E8_EXPOSURE_MARGIN;
	s5k5e8->exposure =
		v4l2_ctrl_new_std(ctrl_hdlr, &s5k5e8_ctrl_ops,
				  V4L2_CID_EXPOSURE, S5K5E8_EXPOSURE_MIN,
				  exposure_max, S5K5E8_EXPOSURE_STEP,
				  mode->exposure);

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &s5k5e8_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(s5k5e8_test_pattern_menu) - 1,
				     0, 0, s5k5e8_test_pattern_menu);

	s5k5e8->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &s5k5e8_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (s5k5e8->hflip)
		s5k5e8->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	s5k5e8->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &s5k5e8_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (s5k5e8->vflip)
		s5k5e8->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		goto error_free_hdlr;
	}

	v4l2_ctrl_cluster(2, &s5k5e8->hflip);

	ret = v4l2_fwnode_device_parse(s5k5e8->dev, &props);
	if (ret)
		goto error_free_hdlr;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &s5k5e8_ctrl_ops,
					      &props);
	if (ret)
		goto error_free_hdlr;

	s5k5e8->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error_free_hdlr:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

static int s5k5e8_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct s5k5e8 *s5k5e8 = to_s5k5e8(sd);
	const struct s5k5e8_mode *mode = s5k5e8_get_mode_from_state(state);
	const struct s5k5e8_reg_list *reg_list = &mode->reg_list;
	int ret;

	ret = pm_runtime_resume_and_get(s5k5e8->dev);
	if (ret)
		return ret;

	cci_multi_reg_write(s5k5e8->regmap, init_array_setting,
			    ARRAY_SIZE(init_array_setting), &ret);
	cci_multi_reg_write(s5k5e8->regmap, reg_list->regs,
			    reg_list->num_regs, &ret);
	if (ret)
		goto error;

	ret = __v4l2_ctrl_handler_setup(s5k5e8->sd.ctrl_handler);
	if (ret)
		goto error;

	ret = cci_write(s5k5e8->regmap, S5K5E8_REG_CTRL_MODE,
			S5K5E8_MODE_STREAMING, NULL);
	if (ret)
		goto error;

	__v4l2_ctrl_grab(s5k5e8->hflip, true);
	__v4l2_ctrl_grab(s5k5e8->vflip, true);

	return 0;

error:
	dev_err(s5k5e8->dev, "failed to start streaming: %d\n", ret);
	pm_runtime_put_autosuspend(s5k5e8->dev);

	return ret;
}

static int s5k5e8_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct s5k5e8 *s5k5e8 = to_s5k5e8(sd);
	int ret;

	ret = cci_write(s5k5e8->regmap, S5K5E8_REG_CTRL_MODE, 0, NULL);
	if (ret)
		dev_err(s5k5e8->dev, "failed to stop streaming: %d\n", ret);

	__v4l2_ctrl_grab(s5k5e8->hflip, false);
	__v4l2_ctrl_grab(s5k5e8->vflip, false);

	pm_runtime_put_autosuspend(s5k5e8->dev);

	return ret;
}

static void s5k5e8_update_pad_format(struct s5k5e8 *s5k5e8,
				     const struct s5k5e8_mode *mode,
				     struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = s5k5e8_get_format_code(s5k5e8->hflip->cur.val,
					   s5k5e8->vflip->cur.val);
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int s5k5e8_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct s5k5e8 *s5k5e8 = to_s5k5e8(sd);
	const struct s5k5e8_mode *mode;
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *crop;
	s64 hblank, vblank, exposure_max;
	bool mode_changed;
	int ret;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE &&
	    v4l2_subdev_is_streaming(sd))
		return -EBUSY;

	mode = v4l2_find_nearest_size(s5k5e8_supported_modes,
				      ARRAY_SIZE(s5k5e8_supported_modes),
				      width, height, fmt->format.width,
				      fmt->format.height);

	s5k5e8_update_pad_format(s5k5e8, mode, &fmt->format);

	format = v4l2_subdev_state_get_format(state, 0);
	crop = v4l2_subdev_state_get_crop(state, 0);
	mode_changed = format->width != mode->width ||
		       format->height != mode->height;

	*format = fmt->format;
	*crop = mode->crop;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE && mode_changed) {
		hblank = mode->hts - mode->width;
		ret = __v4l2_ctrl_modify_range(s5k5e8->hblank, hblank, hblank,
					       1, hblank);
		if (ret)
			return ret;

		vblank = mode->vts - mode->height;
		ret = __v4l2_ctrl_modify_range(s5k5e8->vblank, vblank,
					       S5K5E8_VTS_MAX - mode->height,
					       1, vblank);
		if (ret)
			return ret;

		ret = __v4l2_ctrl_s_ctrl(s5k5e8->vblank, vblank);
		if (ret)
			return ret;

		exposure_max = mode->vts - S5K5E8_EXPOSURE_MARGIN;
		ret = __v4l2_ctrl_modify_range(s5k5e8->exposure,
					       S5K5E8_EXPOSURE_MIN,
					       exposure_max,
					       S5K5E8_EXPOSURE_STEP,
					       mode->exposure);
		if (ret)
			return ret;

		ret = __v4l2_ctrl_s_ctrl(s5k5e8->exposure, mode->exposure);
		if (ret)
			return ret;
	}

	return 0;
}

static int s5k5e8_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct s5k5e8 *s5k5e8 = to_s5k5e8(sd);

	/* Media bus code index is constant, but code formats are not */
	if (code->index > 0)
		return -EINVAL;

	code->code = s5k5e8_get_format_code(s5k5e8->hflip->cur.val,
					    s5k5e8->vflip->cur.val);

	return 0;
}

static int s5k5e8_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct s5k5e8 *s5k5e8 = to_s5k5e8(sd);

	if (fse->index >= ARRAY_SIZE(s5k5e8_supported_modes))
		return -EINVAL;

	if (fse->code != s5k5e8_get_format_code(s5k5e8->hflip->cur.val,
						s5k5e8->vflip->cur.val))
		return -EINVAL;

	fse->min_width = s5k5e8_supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = s5k5e8_supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int s5k5e8_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r = (struct v4l2_rect){
			.left = 0,
			.top = 0,
			.width = S5K5E8_NATIVE_WIDTH,
			.height = S5K5E8_NATIVE_HEIGHT,
		};
		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r = s5k5e8_supported_modes[0].crop;
		return 0;
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, 0);
		return 0;
	default:
		return -EINVAL;
	}
}

static int s5k5e8_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct s5k5e8 *s5k5e8 = to_s5k5e8(sd);
	const struct s5k5e8_mode *mode = &s5k5e8_supported_modes[0];

	s5k5e8_update_pad_format(s5k5e8, mode,
				 v4l2_subdev_state_get_format(state, 0));
	*v4l2_subdev_state_get_crop(state, 0) = mode->crop;

	return 0;
}

static const struct v4l2_subdev_video_ops s5k5e8_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops s5k5e8_pad_ops = {
	.set_fmt = s5k5e8_set_pad_format,
	.get_fmt = v4l2_subdev_get_fmt,
	.get_selection = s5k5e8_get_selection,
	.enum_mbus_code = s5k5e8_enum_mbus_code,
	.enum_frame_size = s5k5e8_enum_frame_size,
	.enable_streams = s5k5e8_enable_streams,
	.disable_streams = s5k5e8_disable_streams,
};

static const struct v4l2_subdev_ops s5k5e8_subdev_ops = {
	.video = &s5k5e8_video_ops,
	.pad = &s5k5e8_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5k5e8_internal_ops = {
	.init_state = s5k5e8_init_state,
};

static const struct media_entity_operations s5k5e8_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int s5k5e8_identify_sensor(struct s5k5e8 *s5k5e8)
{
	u64 val;
	int ret;

	ret = cci_read(s5k5e8->regmap, S5K5E8_REG_CHIP_ID, &val, NULL);
	if (ret) {
		dev_err(s5k5e8->dev, "failed to read chip id: %d\n", ret);
		return ret;
	}

	if (val != S5K5E8_CHIP_ID) {
		dev_err(s5k5e8->dev,
			"chip id mismatch: expected 0x%04x, got 0x%04llx\n",
			S5K5E8_CHIP_ID, val);
		return -ENODEV;
	}

	dev_dbg(s5k5e8->dev, "identified Samsung S5K5E8 sensor (chip id 0x%04llx)\n",
		val);

	return 0;
}

static int s5k5e8_check_hwcfg(struct s5k5e8 *s5k5e8)
{
	struct fwnode_handle *fwnode = dev_fwnode(s5k5e8->dev), *ep;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	unsigned long freq_bitmap;
	int ret;

	if (!fwnode)
		return -ENODEV;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return -EINVAL;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != S5K5E8_DATA_LANES) {
		dev_err(s5k5e8->dev, "Invalid number of data lanes: %u\n",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto endpoint_free;
	}

	ret = v4l2_link_freq_to_bitmap(s5k5e8->dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       s5k5e8_link_freq_menu,
				       ARRAY_SIZE(s5k5e8_link_freq_menu),
				       &freq_bitmap);

endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int s5k5e8_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k5e8 *s5k5e8 = to_s5k5e8(sd);
	unsigned int i;
	int ret;

	/* Enable supplies sequentially: vddio -> vdda -> vddd */
	for (i = 0; i < S5K5E8_NUM_SUPPLIES; i++) {
		ret = regulator_enable(s5k5e8->supplies[i].consumer);
		if (ret)
			goto disable_regulators;

		usleep_range(1000, 1500);
	}

	ret = clk_prepare_enable(s5k5e8->mclk);
	if (ret)
		goto disable_regulators;

	usleep_range(1000, 2000);

	gpiod_set_value_cansleep(s5k5e8->powerdown_gpio, 0);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(s5k5e8->reset_gpio, 0);

	usleep_range(10000, 11000);

	return 0;

disable_regulators:
	while (i > 0)
		regulator_disable(s5k5e8->supplies[--i].consumer);

	return ret;
}

static int s5k5e8_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k5e8 *s5k5e8 = to_s5k5e8(sd);
	unsigned int i;

	gpiod_set_value_cansleep(s5k5e8->reset_gpio, 1);
	gpiod_set_value_cansleep(s5k5e8->powerdown_gpio, 1);

	clk_disable_unprepare(s5k5e8->mclk);

	for (i = S5K5E8_NUM_SUPPLIES; i > 0; i--)
		regulator_disable(s5k5e8->supplies[i - 1].consumer);

	return 0;
}

static int s5k5e8_probe(struct i2c_client *client)
{
	struct s5k5e8 *s5k5e8;
	unsigned long freq;
	unsigned int i;
	int ret;

	s5k5e8 = devm_kzalloc(&client->dev, sizeof(*s5k5e8), GFP_KERNEL);
	if (!s5k5e8)
		return -ENOMEM;

	s5k5e8->dev = &client->dev;
	v4l2_i2c_subdev_init(&s5k5e8->sd, client, &s5k5e8_subdev_ops);

	s5k5e8->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(s5k5e8->regmap))
		return dev_err_probe(s5k5e8->dev, PTR_ERR(s5k5e8->regmap),
				     "failed to init CCI\n");

	s5k5e8->mclk = devm_v4l2_sensor_clk_get(s5k5e8->dev, NULL);
	if (IS_ERR(s5k5e8->mclk))
		return dev_err_probe(s5k5e8->dev, PTR_ERR(s5k5e8->mclk),
				     "failed to get MCLK clock\n");

	freq = clk_get_rate(s5k5e8->mclk);
	if (freq != S5K5E8_MCLK_FREQ_24MHZ)
		return dev_err_probe(s5k5e8->dev, -EINVAL,
				     "MCLK clock frequency %lu is not supported\n",
				     freq);

	ret = s5k5e8_check_hwcfg(s5k5e8);
	if (ret)
		return dev_err_probe(s5k5e8->dev, ret,
				     "failed to check HW configuration\n");

	s5k5e8->reset_gpio =
		devm_gpiod_get_optional(s5k5e8->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(s5k5e8->reset_gpio))
		return dev_err_probe(s5k5e8->dev,
				     PTR_ERR(s5k5e8->reset_gpio),
				     "failed to get reset GPIO\n");

	s5k5e8->powerdown_gpio =
		devm_gpiod_get_optional(s5k5e8->dev, "powerdown",
					GPIOD_OUT_HIGH);
	if (IS_ERR(s5k5e8->powerdown_gpio))
		return dev_err_probe(s5k5e8->dev,
				     PTR_ERR(s5k5e8->powerdown_gpio),
				     "failed to get powerdown GPIO\n");

	for (i = 0; i < S5K5E8_NUM_SUPPLIES; i++)
		s5k5e8->supplies[i].supply = s5k5e8_supply_names[i];

	ret = devm_regulator_bulk_get(s5k5e8->dev, S5K5E8_NUM_SUPPLIES,
				      s5k5e8->supplies);
	if (ret)
		return dev_err_probe(s5k5e8->dev, ret,
				     "failed to get supply regulators\n");

	ret = s5k5e8_power_on(s5k5e8->dev);
	if (ret)
		return dev_err_probe(s5k5e8->dev, ret,
				     "failed to power on sensor\n");

	ret = s5k5e8_identify_sensor(s5k5e8);
	if (ret)
		goto power_off;

	ret = s5k5e8_init_controls(s5k5e8);
	if (ret) {
		ret = dev_err_probe(s5k5e8->dev, ret,
				    "failed to init controls\n");
		goto power_off;
	}

	s5k5e8->sd.state_lock = s5k5e8->ctrl_handler.lock;
	s5k5e8->sd.internal_ops = &s5k5e8_internal_ops;
	s5k5e8->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	s5k5e8->sd.entity.ops = &s5k5e8_subdev_entity_ops;
	s5k5e8->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	s5k5e8->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&s5k5e8->sd.entity, 1, &s5k5e8->pad);
	if (ret) {
		ret = dev_err_probe(s5k5e8->dev, ret,
				    "failed to init media entity pads\n");
		goto v4l2_ctrl_handler_free;
	}

	ret = v4l2_subdev_init_finalize(&s5k5e8->sd);
	if (ret < 0) {
		ret = dev_err_probe(s5k5e8->dev, ret,
				    "failed to finalize v4l2 subdev\n");
		goto media_entity_cleanup;
	}

	pm_runtime_set_active(s5k5e8->dev);
	pm_runtime_enable(s5k5e8->dev);

	ret = v4l2_async_register_subdev_sensor(&s5k5e8->sd);
	if (ret < 0) {
		ret = dev_err_probe(s5k5e8->dev, ret,
				    "failed to register V4L2 subdev\n");
		goto subdev_cleanup;
	}

	pm_runtime_set_autosuspend_delay(s5k5e8->dev, 1000);
	pm_runtime_use_autosuspend(s5k5e8->dev);
	pm_runtime_idle(s5k5e8->dev);

	return 0;

subdev_cleanup:
	pm_runtime_disable(s5k5e8->dev);
	pm_runtime_set_suspended(s5k5e8->dev);
	v4l2_subdev_cleanup(&s5k5e8->sd);

media_entity_cleanup:
	media_entity_cleanup(&s5k5e8->sd.entity);

v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(s5k5e8->sd.ctrl_handler);

power_off:
	s5k5e8_power_off(s5k5e8->dev);

	return ret;
}

static void s5k5e8_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k5e8 *s5k5e8 = to_s5k5e8(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	pm_runtime_dont_use_autosuspend(s5k5e8->dev);
	pm_runtime_disable(s5k5e8->dev);

	if (!pm_runtime_status_suspended(s5k5e8->dev)) {
		s5k5e8_power_off(s5k5e8->dev);
		pm_runtime_set_suspended(s5k5e8->dev);
	}
}

static const struct dev_pm_ops s5k5e8_pm_ops = {
	RUNTIME_PM_OPS(s5k5e8_power_off, s5k5e8_power_on, NULL)
};

static const struct of_device_id s5k5e8_of_match[] = {
	{ .compatible = "samsung,s5k5e8" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s5k5e8_of_match);

static struct i2c_driver s5k5e8_i2c_driver = {
	.driver = {
		.name = "s5k5e8",
		.pm = pm_ptr(&s5k5e8_pm_ops),
		.of_match_table = s5k5e8_of_match,
	},
	.probe = s5k5e8_probe,
	.remove = s5k5e8_remove,
};

module_i2c_driver(s5k5e8_i2c_driver);

MODULE_AUTHOR("Andrii Cherniavskyi <chernyav.a@gmail.com>");
MODULE_AUTHOR("Tuan Kiet Huynh <huynhtuankiet33@gmail.com>");
MODULE_DESCRIPTION("Samsung S5K5E8 image sensor driver");
MODULE_LICENSE("GPL");
