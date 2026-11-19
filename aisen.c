// SPDX-License-Identifier: GPL-2.0
/*
 * AISEN MIPI CSI-2 single-channel source
 *
 * For the Raspberry Pi downstream RP1 CFE driver ("raspberrypi,rp1-cfe"),
 * which uses the classic per-pad-link model (NO V4L2 streams/routing API,
 * so it works on the stock RPi OS kernel where the streams API is disabled).
 *
 * Presents ONE source pad, linked by the CFE to csi2 sink pad 0 / channel 0:
 *   pad0 -> csi2 sink0 -> csi2_ch0 : VC0 640x480 UYVY
 *
 *
 * 2 or 4 D-PHY data lanes, CONTINUOUS clock, link frequency from DT
 * (default 200 MHz = 400 Mbps/lane).
 *
 * Lane count = endpoint "data-lanes" (2 or 4). The overlay sets it on BOTH
 * endpoints (sensor and CSI) via the "2lane" parameter.
 *
 * The camera free-runs a continuous clock, so it must be started/stopped over
 * I2C on every stream (reg 0x30 @ 0x33). The CFE opens its receiver BEFORE
 * calling s_stream(1), so the LP->HS clock-lane transition caused by the I2C
 * start is seen by the D-PHY.
 *
 * Per-channel VC/DT is read by csi2_get_vc_dt() from this driver's
 * pad::get_frame_desc (exactly ONE CSI-2 entry per source pad). The source
 * is started/stopped via the video .s_stream op.
 *
 * Copyright (c) 2026
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

/* Single source pad (MEDIA_PAD_FL_SOURCE) -> CFE csi2 ch0. */
#define CANLAB_PAD_VC0		0
#define CANLAB_NUM_PADS		1

#define CANLAB_DEFAULT_LINK_FREQ_HZ	200000000ULL	/* clock lane = 200 MHz */

/* Camera start/stop control over I2C (i2c-11, addr 0x33, reg 0x30). */
#define CANLAB_REG_CAMERA	0x30
#define CANLAB_CAMERA_ON	0x01
#define CANLAB_CAMERA_OFF	0x00

#define CANLAB_MBUS_CODE	MEDIA_BUS_FMT_UYVY8_1X16	/* UYVY, CSI-2 DT 0x1e */
#define CANLAB_BPP		16

/* Fixed camera output geometry. */
#define CANLAB_WIDTH		640u
#define CANLAB_HEIGHT		480u
#define CANLAB_VC		0
#define CANLAB_DT		MIPI_CSI2_DT_YUV422_8B

#define CANLAB_RESTART_DELAY_MS	20

#define CANLAB_MIN_WIDTH	64u
#define CANLAB_MIN_HEIGHT	64u
#define CANLAB_MAX_WIDTH	CANLAB_WIDTH
#define CANLAB_MAX_HEIGHT	CANLAB_HEIGHT

struct canlab_pad_cfg {
	u32 width;
	u32 height;
	u8 vc;
	u8 dt;
};

static const struct canlab_pad_cfg canlab_pad_cfg[CANLAB_NUM_PADS] = {
	[CANLAB_PAD_VC0] = { CANLAB_WIDTH, CANLAB_HEIGHT, CANLAB_VC, CANLAB_DT },
};

struct canlab {
	struct i2c_client *client;
	struct v4l2_subdev sd;
	struct media_pad pads[CANLAB_NUM_PADS];

	struct v4l2_ctrl_handler ctrls;

	struct clk *xclk;			/* optional */
	struct gpio_desc *reset_gpio;		/* optional, unused on this board */
	struct regulator *supply;		/* optional */

	unsigned int num_data_lanes;
	unsigned int bus_flags;			/* V4L2_MBUS_CSI2_* */
	s64 link_freq;				/* Hz */
	s64 link_freq_menu[1];

	struct mutex lock;			/* protects streaming state + HW */
	bool streaming;
};

static inline struct canlab *to_canlab(struct v4l2_subdev *sd)
{
	return container_of(sd, struct canlab, sd);
}

static void canlab_fill_default_fmt(struct v4l2_mbus_framefmt *fmt,
				    const struct canlab_pad_cfg *cfg)
{
	fmt->code = CANLAB_MBUS_CODE;
	fmt->width = cfg->width;
	fmt->height = cfg->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_SMPTE170M;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_LIM_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_709;
}

/* -------------------------------------------------------------------------
 * Device-specific register programming
 * ------------------------------------------------------------------------- */

static int canlab_start(struct canlab *c)
{
	struct device *dev = &c->client->dev;
	int ret;
	
	ret = i2c_smbus_write_byte_data(c->client, CANLAB_REG_CAMERA, CANLAB_CAMERA_OFF);
	if (ret)
		dev_warn(dev, "camera pre-stop failed: %d\n", ret);

	msleep(CANLAB_RESTART_DELAY_MS);
	dev_dbg(dev, "start: %u lanes, %lld Hz, %s clock\n",
		c->num_data_lanes, c->link_freq,
		(c->bus_flags & V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK) ?
			"non-continuous" : "continuous");

	ret = i2c_smbus_write_byte_data(c->client, CANLAB_REG_CAMERA, CANLAB_CAMERA_ON);
	if (ret)
		dev_err(dev, "camera start failed: %d\n", ret);

	return ret;
}

static void canlab_stop(struct canlab *c)
{
	int ret;

	ret = i2c_smbus_write_byte_data(c->client, CANLAB_REG_CAMERA,
					CANLAB_CAMERA_OFF);
	if (ret)
		dev_warn(&c->client->dev, "camera stop failed: %d\n", ret);
}

/* -------------------------------------------------------------------------
 * Power management
 * ------------------------------------------------------------------------- */

static int canlab_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct canlab *c = to_canlab(sd);
	int ret;

	/*
	 * Steady power only. Clock / supply / reset-gpio are optional; the calls
	 * below are no-ops for whatever the DT does not provide.
	 */
	ret = clk_prepare_enable(c->xclk);
	if (ret) {
		dev_err(dev, "failed to enable clock: %d\n", ret);
		return ret;
	}

	if (c->supply) {
		ret = regulator_enable(c->supply);
		if (ret) {
			dev_err(dev, "failed to enable supply: %d\n", ret);
			clk_disable_unprepare(c->xclk);
			return ret;
		}
	}
	gpiod_set_value_cansleep(c->reset_gpio, 0);

	return 0;
}

static int canlab_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct canlab *c = to_canlab(sd);

	gpiod_set_value_cansleep(c->reset_gpio, 1);
	if (c->supply)
		regulator_disable(c->supply);
	clk_disable_unprepare(c->xclk);

	return 0;
}

/* -------------------------------------------------------------------------
 * V4L2 subdev operations
 * ------------------------------------------------------------------------- */

static int canlab_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	unsigned int pad;

	for (pad = 0; pad < CANLAB_NUM_PADS; pad++) {
		struct v4l2_mbus_framefmt *fmt;

		fmt = v4l2_subdev_state_get_format(state, pad);
		if (fmt)
			canlab_fill_default_fmt(fmt, &canlab_pad_cfg[pad]);
	}

	return 0;
}

static int canlab_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt;

	/* Only UYVY is generated; clamp geometry to even, in-range values. */
	format->format.code = CANLAB_MBUS_CODE;
	format->format.width = clamp(format->format.width,
				     CANLAB_MIN_WIDTH, CANLAB_MAX_WIDTH) & ~1u;
	format->format.height = clamp(format->format.height,
				      CANLAB_MIN_HEIGHT, CANLAB_MAX_HEIGHT) & ~1u;
	format->format.field = V4L2_FIELD_NONE;
	format->format.colorspace = V4L2_COLORSPACE_SMPTE170M;
	format->format.ycbcr_enc = V4L2_YCBCR_ENC_601;
	format->format.quantization = V4L2_QUANTIZATION_LIM_RANGE;
	format->format.xfer_func = V4L2_XFER_FUNC_709;

	fmt = v4l2_subdev_state_get_format(state, format->pad);
	if (!fmt)
		return -EINVAL;
	*fmt = format->format;

	return 0;
}

static int canlab_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				 struct v4l2_mbus_frame_desc *fd)
{
	struct v4l2_subdev_state *state;
	const struct v4l2_mbus_framefmt *fmt;

	if (pad >= CANLAB_NUM_PADS)
		return -EINVAL;

	memset(fd, 0, sizeof(*fd));
	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 1;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	fmt = v4l2_subdev_state_get_format(state, pad);

	fd->entry[0].stream = 0;
	fd->entry[0].flags = 0;
	fd->entry[0].pixelcode = fmt ? fmt->code : CANLAB_MBUS_CODE;
	fd->entry[0].bus.csi2.vc = canlab_pad_cfg[pad].vc;
	fd->entry[0].bus.csi2.dt = canlab_pad_cfg[pad].dt;

	v4l2_subdev_unlock_state(state);

	return 0;
}

static int canlab_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_config *cfg)
{
	struct canlab *c = to_canlab(sd);

	cfg->type = V4L2_MBUS_CSI2_DPHY;
	cfg->bus.mipi_csi2.num_data_lanes = c->num_data_lanes;
	cfg->bus.mipi_csi2.flags = c->bus_flags;

	return 0;
}

static int canlab_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct canlab *c = to_canlab(sd);
	struct device *dev = &c->client->dev;
	int ret = 0;

	mutex_lock(&c->lock);

	if (enable) {
		if (c->streaming)
			goto out;

		ret = pm_runtime_resume_and_get(dev);
		if (ret < 0)
			goto out;

		ret = canlab_start(c);
		if (ret) {
			pm_runtime_put(dev);
			goto out;
		}
		c->streaming = true;
	} else {
		if (!c->streaming)
			goto out;

		canlab_stop(c);
		c->streaming = false;
		pm_runtime_put_sync(dev);
	}

out:
	mutex_unlock(&c->lock);
	return ret;
}

static const struct v4l2_subdev_pad_ops canlab_pad_ops = {
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = canlab_set_fmt,
	.get_frame_desc = canlab_get_frame_desc,
	.get_mbus_config = canlab_get_mbus_config,
};

static const struct v4l2_subdev_video_ops canlab_video_ops = {
	.s_stream = canlab_s_stream,
};

static const struct v4l2_subdev_ops canlab_subdev_ops = {
	.video = &canlab_video_ops,
	.pad = &canlab_pad_ops,
};

static const struct v4l2_subdev_internal_ops canlab_internal_ops = {
	.init_state = canlab_init_state,
};

static const struct media_entity_operations canlab_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/* -------------------------------------------------------------------------
 * Controls
 * ------------------------------------------------------------------------- */

static int canlab_s_ctrl(struct v4l2_ctrl *ctrl)
{
	return 0;	/* all controls are read-only */
}

static const struct v4l2_ctrl_ops canlab_ctrl_ops = {
	.s_ctrl = canlab_s_ctrl,
};

static int canlab_init_controls(struct canlab *c)
{
	struct v4l2_ctrl_handler *hdl = &c->ctrls;
	struct v4l2_ctrl *ctrl;
	s64 pixel_rate;
	int ret;

	ret = v4l2_ctrl_handler_init(hdl, 2);
	if (ret)
		return ret;

	c->link_freq_menu[0] = c->link_freq;
	ctrl = v4l2_ctrl_new_int_menu(hdl, &canlab_ctrl_ops, V4L2_CID_LINK_FREQ,
				      0, 0, c->link_freq_menu);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	pixel_rate = div_s64(c->link_freq * 2 * c->num_data_lanes, CANLAB_BPP);
	ctrl = v4l2_ctrl_new_std(hdl, &canlab_ctrl_ops, V4L2_CID_PIXEL_RATE,
				 pixel_rate, pixel_rate, 1, pixel_rate);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	c->sd.ctrl_handler = hdl;
	return 0;
}

/* -------------------------------------------------------------------------
 * Probe / remove
 * ------------------------------------------------------------------------- */

static int canlab_parse_dt(struct canlab *c)
{
	struct device *dev = &c->client->dev;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep;
	int ret;

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!ep)
		return dev_err_probe(dev, -EINVAL, "no endpoint found\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(dev, ret, "failed to parse endpoint\n");

	/* Lane count = endpoint data-lanes (2 or 4), set by the overlay. */
	c->num_data_lanes = bus_cfg.bus.mipi_csi2.num_data_lanes;
	c->bus_flags = bus_cfg.bus.mipi_csi2.flags;

	if (c->num_data_lanes != 2 && c->num_data_lanes != 4) {
		ret = dev_err_probe(dev, -EINVAL,
				    "data-lanes must be 2 or 4 (got %u)\n",
				    c->num_data_lanes);
		goto out_free;
	}

	if (bus_cfg.nr_of_link_frequencies)
		c->link_freq = bus_cfg.link_frequencies[0];
	else
		c->link_freq = CANLAB_DEFAULT_LINK_FREQ_HZ;

	ret = 0;
out_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	return ret;
}

static int canlab_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct canlab *c;
	unsigned int i;
	int ret;

	c = devm_kzalloc(dev, sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;

	c->client = client;
	mutex_init(&c->lock);

	ret = canlab_parse_dt(c);
	if (ret)
		goto err_mutex;

	c->xclk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(c->xclk)) {
		ret = dev_err_probe(dev, PTR_ERR(c->xclk), "failed to get clock\n");
		goto err_mutex;
	}

	c->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(c->reset_gpio)) {
		ret = dev_err_probe(dev, PTR_ERR(c->reset_gpio),
				    "failed to get reset gpio\n");
		goto err_mutex;
	}

	/* Optional "vdd" supply; not wired in the stock overlay for this board. */
	c->supply = devm_regulator_get_optional(dev, "vdd");
	if (IS_ERR(c->supply)) {
		ret = PTR_ERR(c->supply);
		if (ret != -ENODEV) {
			dev_err_probe(dev, ret, "failed to get vdd supply\n");
			goto err_mutex;
		}
		c->supply = NULL;
	}

	v4l2_i2c_subdev_init(&c->sd, client, &canlab_subdev_ops);
	c->sd.internal_ops = &canlab_internal_ops;
	c->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;	/* no FL_STREAMS */
	c->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	c->sd.entity.ops = &canlab_entity_ops;

	for (i = 0; i < CANLAB_NUM_PADS; i++)
		c->pads[i].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&c->sd.entity, CANLAB_NUM_PADS, c->pads);
	if (ret)
		goto err_mutex;

	ret = canlab_init_controls(c);
	if (ret)
		goto err_entity;

	ret = v4l2_subdev_init_finalize(&c->sd);
	if (ret)
		goto err_ctrls;

	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	ret = v4l2_async_register_subdev(&c->sd);
	if (ret) {
		dev_err(dev, "failed to register async subdev: %d\n", ret);
		goto err_pm;
	}

	dev_info(dev,
		 "aisen: VC%u %ux%u (pad0/ch0) UYVY, %u lanes, %lld Hz %s\n",
		 canlab_pad_cfg[CANLAB_PAD_VC0].vc,
		 canlab_pad_cfg[CANLAB_PAD_VC0].width,
		 canlab_pad_cfg[CANLAB_PAD_VC0].height,
		 c->num_data_lanes, c->link_freq,
		 (c->bus_flags & V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK) ?
			"non-continuous" : "continuous");

	return 0;

err_pm:
	pm_runtime_disable(dev);
	v4l2_subdev_cleanup(&c->sd);
err_ctrls:
	v4l2_ctrl_handler_free(&c->ctrls);
err_entity:
	media_entity_cleanup(&c->sd.entity);
err_mutex:
	mutex_destroy(&c->lock);
	return ret;
}

static void canlab_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct canlab *c = to_canlab(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&c->ctrls);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		canlab_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	mutex_destroy(&c->lock);
}

static const struct dev_pm_ops canlab_pm_ops = {
	SET_RUNTIME_PM_OPS(canlab_power_off, canlab_power_on, NULL)
};

static const struct of_device_id canlab_of_match[] = {
	{ .compatible = "canlab,aisen" },
	{ }
};
MODULE_DEVICE_TABLE(of, canlab_of_match);

static const struct i2c_device_id canlab_id[] = {
	{ "canlab-aisen" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, canlab_id);

static struct i2c_driver canlab_i2c_driver = {
	.driver = {
		.name = "aisen",
		.of_match_table = canlab_of_match,
		.pm = &canlab_pm_ops,
	},
	.probe = canlab_probe,
	.remove = canlab_remove,
	.id_table = canlab_id,
};
module_i2c_driver(canlab_i2c_driver);

MODULE_DESCRIPTION("AISEN MIPI CSI-2 single-channel 640x480 UYVY source (2/4 lanes, downstream RP1 CFE)");
MODULE_AUTHOR("Camera Driver Development");
MODULE_LICENSE("GPL");
