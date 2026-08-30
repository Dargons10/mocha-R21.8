/*
 * ov5693.c - OmniVision OV5693 sensor V4L2 subdev driver
 *
 * Copyright (c) 2026, Dargons10
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * OmniVision OV5693 is a 5MP CMOS image sensor with BGGR Bayer pattern.
 * Used in Xiaomi Mi Pad (mocha) front camera on CSI-E 1-lane.
 *
 * Compatible with Linux kernel 3.10
 */

#define pr_fmt(fmt) "ov5693: " fmt

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>

#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include "../tegra_vi.h"
#include <media/v4l2-ioctl.h>
#include <media/v4l2-common.h>
#include <media/v4l2-of.h>
#include <mach/io_dpd.h>

/* OV5693 registers */
#define OV5693_REG_CHIP_ID          0x300A
#define OV5693_CHIP_ID_VALUE        0x5693

#define OV5693_REG_MODE_SELECT      0x0100
#define OV5693_MODE_STANDBY         0x00
#define OV5693_MODE_STREAMING       0x01

#define OV5693_REG_EXPOSURE_HIGH    0x3500
#define OV5693_REG_EXPOSURE_MID     0x3501
#define OV5693_REG_EXPOSURE_LOW     0x3502
#define OV5693_REG_GAIN             0x350B

/* Default mode configuration */
#define OV5693_DEFAULT_WIDTH        2592
#define OV5693_DEFAULT_HEIGHT       1944
#define OV5693_DEFAULT_FPS          30
#define OV5693_PIXEL_RATE           160000000ULL
#define OV5693_LINE_LENGTH          2688
#define OV5693_FRAME_LENGTH         1960

/* Supported media bus formats */
static const u32 ov5693_mbus_formats[] = {
    V4L2_MBUS_FMT_SBGGR10_1X10,
    V4L2_MBUS_FMT_SBGGR8_1X8,
};

/* Frame sizes supported */
struct ov5693_framesize {
    u32 width;
    u32 height;
    u32 max_fps;
};

static const struct ov5693_framesize ov5693_framesizes[] = {
    { 2592, 1944, 15 },  /* 5MP @ 15fps */
    { 1920, 1080, 30 },  /* 1080p @ 30fps */
    { 1280, 720,  60 },  /* 720p @ 60fps */
    { 640,  480,  90 },  /* VGA @ 90fps */
};

#define OV5693_NUM_FRAMESIZES ARRAY_SIZE(ov5693_framesizes)

/* Sensor initialization registers */
static const u8 ov5693_init_regs[][3] = {
    { 0x01, 0x00, 0x00 },
    { 0x01, 0x03, 0x01 },
    { 0x30, 0x01, 0x0a },
    { 0x30, 0x02, 0x00 },
    { 0x30, 0x10, 0x00 },
    { 0x30, 0x11, 0x00 },
    { 0x30, 0x12, 0x00 },
    { 0x30, 0x16, 0x00 },
    { 0x30, 0x18, 0x00 },
    { 0x30, 0x20, 0x00 },
    { 0x30, 0x21, 0x00 },
    { 0x30, 0x80, 0x02 },
    { 0x30, 0x83, 0x00 },
    { 0x30, 0x84, 0x00 },
    { 0x30, 0x85, 0x00 },
    { 0x30, 0x86, 0x00 },
    { 0x30, 0x87, 0x00 },
    { 0x33, 0x50, 0x00 },
    { 0x33, 0x51, 0x00 },
    { 0x33, 0x52, 0x00 },
    { 0x50, 0x00, 0x0f },
    { 0x50, 0x01, 0x00 },
    { 0x50, 0x02, 0x00 },
    { 0x50, 0x03, 0x00 },
    { 0x01, 0x00, 0x01 },
};

#define OV5693_NUM_INIT_REGS ARRAY_SIZE(ov5693_init_regs)

/* OV5693 device structure */
struct ov5693 {
    struct i2c_client *client;
    struct v4l2_subdev sd;
    struct v4l2_ctrl_handler ctrl_handler;
    struct media_pad pad;

    struct regulator *avdd;    /* analog from DT */
    struct regulator *dovdd;   /* digital I/O from DT */
    struct regulator *dvdd;    /* digital core from DT */
    struct regulator *reg_af;  /* ov5693_afvdd = imx179_reg1 (palmas_ldo7 2.7V) */
    struct regulator *reg_1v2; /* vdd_cam_1v2 fixed 1.2V */
    struct regulator *reg_1v8; /* vdd_cam_1v8 fixed 1.8V */
    struct clk *mclk;

    int reset_gpio;  /* CAM2_RSTN = PCC1 */
    int pwdn_gpio;   /* CAM2_PWDN = PBB6 */
    int af_gpio;     /* CAM_AF_PWDN = PBB7 */

    bool powered;
    bool streaming;
    u32 csi_port;
    u32 num_lanes;

    struct v4l2_mbus_framefmt format;
    const struct ov5693_framesize *current_framesize;

    /* Controls */
    struct v4l2_ctrl *exposure;
    struct v4l2_ctrl *gain;
    struct v4l2_ctrl *pixel_rate;
};

static inline struct ov5693 *to_ov5693(struct v4l2_subdev *sd)
{
    return container_of(sd, struct ov5693, sd);
}

/* I2C register access */
static int ov5693_write_reg(struct ov5693 *ov5693, u16 reg, u8 val)
{
    struct i2c_client *client = ov5693->client;
    u8 buf[3];

    buf[0] = (reg >> 8) & 0xff;
    buf[1] = reg & 0xff;
    buf[2] = val;

    if (i2c_master_send(client, buf, 3) != 3)
        return -EIO;

    return 0;
}

static int ov5693_read_reg(struct ov5693 *ov5693, u16 reg, u8 *val)
{
    struct i2c_client *client = ov5693->client;
    u8 wbuf[2];
    struct i2c_msg msg[2];
    int ret;

    wbuf[0] = (reg >> 8) & 0xff;
    wbuf[1] = reg & 0xff;

    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].len = 2;
    msg[0].buf = wbuf;

    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = 1;
    msg[1].buf = val;

    ret = i2c_transfer(client->adapter, msg, 2);
    if (ret < 0) {
        pr_err("DEBUG: i2c_transfer error: %d\n", ret);
        return ret;
    }
    if (ret != 2) {
        pr_err("DEBUG: i2c_transfer partial: %d/2\n", ret);
        return -EIO;
    }

    return 0;
}

static int ov5693_write_init_regs(struct ov5693 *ov5693)
{
    int i, ret;

    for (i = 0; i < OV5693_NUM_INIT_REGS; i++) {
        u16 reg = (ov5693_init_regs[i][0] << 8) | ov5693_init_regs[i][1];
        ret = ov5693_write_reg(ov5693, reg, ov5693_init_regs[i][2]);
        if (ret) {
            pr_err("Failed to write reg 0x%04x: %d\n", reg, ret);
            return ret;
        }
    }

    return 0;
}

static int ov5693_check_chip_id(struct ov5693 *ov5693)
{
    u8 id_high = 0, id_low = 0;
    u16 chip_id;
    int ret;

    ret = ov5693_read_reg(ov5693, OV5693_REG_CHIP_ID, &id_high);
    if (ret) {
        pr_warn("chip_id HIGH read failed: %d (no fatal - continuando)\n", ret);
        return 0;
    }

    ret = ov5693_read_reg(ov5693, OV5693_REG_CHIP_ID + 1, &id_low);
    if (ret) {
        pr_warn("chip_id LOW read failed: %d (no fatal - continuando)\n", ret);
        return 0;
    }

    chip_id = (id_high << 8) | id_low;
    pr_info("Chip ID: 0x%04x (expected 0x%04x)\n", chip_id, OV5693_CHIP_ID_VALUE);

    if (chip_id != OV5693_CHIP_ID_VALUE)
        pr_warn("Chip ID mismatch: 0x%04x != 0x%04x (continuando de todos modos)\n",
                chip_id, OV5693_CHIP_ID_VALUE);

    return 0;
}

/* Power management - matching board-ardbeg-sensors.c:ardbeg_ov5693_power_on EXACTAMENTE */
static int ov5693_set_power(struct ov5693 *ov5693, bool on)
{
    int ret;
    struct tegra_io_dpd csie_io = {
        .name          = "CSIE",
        .io_dpd_reg_index = 1,
        .io_dpd_bit    = 12,
    };

    if (on && !ov5693->powered) {
        pr_info("Powering on OV5693\n");

        /* [stock NVC] MCLK first: clk_set_rate + prepare_enable (stock lo hace ANTES de power_on) */
        if (clk_get_rate(ov5693->mclk) != 24000000)
            clk_set_rate(ov5693->mclk, 24000000);
        ret = clk_prepare_enable(ov5693->mclk);
        if (ret) { pr_err("FAILED mclk: %d\n", ret); return ret; }
        pr_info("MCLK at %lu Hz\n", clk_get_rate(ov5693->mclk));

        /* [stock] Disable CSI IO DPD */
        tegra_io_dpd_disable(&csie_io);

        /* [stock] GPIOs iniciales: PWDN=0, RSTN=0, AF=0
         * IMPORTANTE: PWDN=0 → NPN transistor OFF → sensor PDN=HIGH (powerdown seguro)
         * El sensor recibe poder mientras está en powerdown, luego PWDN→1 lo saca. */
        if (gpio_is_valid(ov5693->pwdn_gpio))
            gpio_set_value(ov5693->pwdn_gpio, 0);
        if (gpio_is_valid(ov5693->reset_gpio))
            gpio_set_value(ov5693->reset_gpio, 0);
        if (gpio_is_valid(ov5693->af_gpio))
            gpio_set_value(ov5693->af_gpio, 0);
        usleep_range(10, 20);

        /* [stock] Reguladores: afvdd → avdd → 1v8 (stock NO usa dovdd ni dvdd explícitamente) */
        ret = regulator_enable(ov5693->reg_af);
        if (ret) { pr_err("FAILED afvdd: %d\n", ret); goto err_afvdd; }
        ret = regulator_enable(ov5693->avdd);
        if (ret) { pr_err("FAILED avdd: %d\n", ret); goto err_avdd; }
        ret = regulator_enable(ov5693->reg_1v8);
        if (ret) { pr_err("FAILED 1v8: %d\n", ret); goto err_1v8; }

        /* [stock] PWDN=1 DESPUÉS de reguladores → sensor sale de powerdown con poder estable */
        if (gpio_is_valid(ov5693->pwdn_gpio))
            gpio_set_value(ov5693->pwdn_gpio, 1);

        ret = regulator_enable(ov5693->reg_1v2);
        if (ret) { pr_err("FAILED 1v2: %d\n", ret); goto err_1v2; }

        udelay(2);

        /* [stock] Deassert reset */
        if (gpio_is_valid(ov5693->reset_gpio))
            gpio_set_value(ov5693->reset_gpio, 1);

        /* [stock] usleep(300-310) - stock no verifica chip ID */
        usleep_range(300, 310);

        /* check_chip_id NO FATAL - stock no lo hace, solo para debug */
        ov5693_check_chip_id(ov5693);

        pr_info("OV5693 power on complete\n");
        ov5693->powered = true;
        return 0;

err_1v2:
        /* PWDN=0 para volver a powerdown */
        if (gpio_is_valid(ov5693->pwdn_gpio))
            gpio_set_value(ov5693->pwdn_gpio, 0);
        regulator_disable(ov5693->reg_1v8);
err_1v8:
        regulator_disable(ov5693->avdd);
err_avdd:
        regulator_disable(ov5693->reg_af);
err_afvdd:
        if (gpio_is_valid(ov5693->af_gpio))
            gpio_set_value(ov5693->af_gpio, 0);
        if (gpio_is_valid(ov5693->reset_gpio))
            gpio_set_value(ov5693->reset_gpio, 0);
        tegra_io_dpd_enable(&csie_io);
        clk_disable_unprepare(ov5693->mclk);
        return ret;
    } else if (!on && ov5693->powered) {
        pr_info("Powering off OV5693\n");

        /* [stock] RSTN=0 primero, luego disable regs, luego PWDN=0 */
        if (gpio_is_valid(ov5693->reset_gpio))
            gpio_set_value(ov5693->reset_gpio, 0);
        udelay(2);

        regulator_disable(ov5693->reg_1v2);
        regulator_disable(ov5693->reg_1v8);

        /* [stock] PWDN=0 DESPUÉS de 1v8 off */
        if (gpio_is_valid(ov5693->pwdn_gpio))
            gpio_set_value(ov5693->pwdn_gpio, 0);
        if (gpio_is_valid(ov5693->af_gpio))
            gpio_set_value(ov5693->af_gpio, 0);

        regulator_disable(ov5693->avdd);
        regulator_disable(ov5693->reg_af);

        tegra_io_dpd_enable(&csie_io);
        clk_disable_unprepare(ov5693->mclk);

        ov5693->powered = false;
        ov5693->streaming = false;
        pr_info("Sensor powered off\n");
    }

    return 0;
}

/* V4L2 subdev core ops */
static int ov5693_s_power(struct v4l2_subdev *sd, int on)
{
    struct ov5693 *ov5693 = to_ov5693(sd);
    return ov5693_set_power(ov5693, on);
}

static int ov5693_init(struct v4l2_subdev *sd, u32 val)
{
    struct ov5693 *ov5693 = to_ov5693(sd);
    int ret;

    ret = ov5693_check_chip_id(ov5693);
    if (ret) {
        pr_err("Chip ID check failed\n");
        return ret;
    }

    ret = ov5693_write_init_regs(ov5693);
    if (ret) {
        pr_err("Failed to initialize sensor: %d\n", ret);
        return ret;
    }

    pr_info("Sensor initialized\n");
    return 0;
}

static const struct v4l2_subdev_core_ops ov5693_core_ops = {
    .s_power = ov5693_s_power,
    .init = ov5693_init,
};

/* V4L2 subdev video ops */
static int ov5693_enum_mbus_code(struct v4l2_subdev *sd,
                                  struct v4l2_subdev_fh *fh,
                                  struct v4l2_subdev_mbus_code_enum *code)
{
    if (code->index >= ARRAY_SIZE(ov5693_mbus_formats))
        return -EINVAL;

    code->code = ov5693_mbus_formats[code->index];
    return 0;
}

static int ov5693_enum_frame_size(struct v4l2_subdev *sd,
                                   struct v4l2_subdev_fh *fh,
                                   struct v4l2_subdev_frame_size_enum *fse)
{
    if (fse->index >= OV5693_NUM_FRAMESIZES)
        return -EINVAL;

    if (fse->code != ov5693_mbus_formats[0] &&
        fse->code != ov5693_mbus_formats[1])
        return -EINVAL;

    fse->min_width = ov5693_framesizes[fse->index].width;
    fse->max_width = ov5693_framesizes[fse->index].width;
    fse->min_height = ov5693_framesizes[fse->index].height;
    fse->max_height = ov5693_framesizes[fse->index].height;

    return 0;
}

static struct v4l2_mbus_framefmt *
ov5693_get_format(struct ov5693 *ov5693, struct v4l2_subdev_fh *fh,
                  unsigned int pad, enum v4l2_subdev_format_whence which)
{
    if (which == V4L2_SUBDEV_FORMAT_TRY)
        return v4l2_subdev_get_try_format(fh, pad);

    return &ov5693->format;
}

static int ov5693_get_fmt(struct v4l2_subdev *sd,
                           struct v4l2_subdev_fh *fh,
                           struct v4l2_subdev_format *fmt)
{
    struct ov5693 *ov5693 = to_ov5693(sd);
    struct v4l2_mbus_framefmt *format;

    format = ov5693_get_format(ov5693, fh, fmt->pad, fmt->which);
    fmt->format = *format;

    return 0;
}

static int ov5693_set_fmt(struct v4l2_subdev *sd,
                           struct v4l2_subdev_fh *fh,
                           struct v4l2_subdev_format *fmt)
{
    struct ov5693 *ov5693 = to_ov5693(sd);
    struct v4l2_mbus_framefmt *format;
    int i;

    format = ov5693_get_format(ov5693, fh, fmt->pad, fmt->which);

    for (i = 0; i < ARRAY_SIZE(ov5693_mbus_formats); i++) {
        if (ov5693_mbus_formats[i] == fmt->format.code)
            break;
    }
    if (i == ARRAY_SIZE(ov5693_mbus_formats))
        fmt->format.code = ov5693_mbus_formats[0];

    for (i = 0; i < OV5693_NUM_FRAMESIZES; i++) {
        if (ov5693_framesizes[i].width == fmt->format.width &&
            ov5693_framesizes[i].height == fmt->format.height)
            break;
    }
    if (i == OV5693_NUM_FRAMESIZES) {
        fmt->format.width = OV5693_DEFAULT_WIDTH;
        fmt->format.height = OV5693_DEFAULT_HEIGHT;
    }

    fmt->format.field = V4L2_FIELD_NONE;
    fmt->format.colorspace = V4L2_COLORSPACE_SRGB;

    *format = fmt->format;

    if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
        ov5693->current_framesize = &ov5693_framesizes[i];
    }

    return 0;
}

static int ov5693_s_stream(struct v4l2_subdev *sd, int enable)
{
    struct ov5693 *ov5693 = to_ov5693(sd);
    int ret;

    if (enable == ov5693->streaming)
        return 0;

    if (enable) {
        ret = ov5693_write_init_regs(ov5693);
        if (ret)
            return ret;

        ret = ov5693_write_reg(ov5693, OV5693_REG_MODE_SELECT,
                                OV5693_MODE_STREAMING);
        if (ret)
            return ret;

        ov5693->streaming = true;
        pr_info("Streaming started\n");
    } else {
        ret = ov5693_write_reg(ov5693, OV5693_REG_MODE_SELECT,
                                OV5693_MODE_STANDBY);
        if (ret)
            return ret;

        ov5693->streaming = false;
        pr_info("Streaming stopped\n");
    }

    return 0;
}

static const struct v4l2_subdev_video_ops ov5693_video_ops = {
    .s_stream = ov5693_s_stream,
};

/* V4L2 subdev pad ops */
static const struct v4l2_subdev_pad_ops ov5693_pad_ops = {
    .enum_mbus_code = ov5693_enum_mbus_code,
    .enum_frame_size = ov5693_enum_frame_size,
    .get_fmt = ov5693_get_fmt,
    .set_fmt = ov5693_set_fmt,
};

/* V4L2 subdev ops */
static const struct v4l2_subdev_ops ov5693_subdev_ops = {
    .core = &ov5693_core_ops,
    .video = &ov5693_video_ops,
    .pad = &ov5693_pad_ops,
};

/* V4L2 control ops */
static int ov5693_s_ctrl(struct v4l2_ctrl *ctrl)
{
    struct ov5693 *ov5693 = container_of(ctrl->handler, struct ov5693, ctrl_handler);
    int ret = 0;

    switch (ctrl->id) {
    case V4L2_CID_EXPOSURE:
        ret = ov5693_write_reg(ov5693, OV5693_REG_EXPOSURE_HIGH, (ctrl->val >> 16) & 0xff);
        if (!ret)
            ret = ov5693_write_reg(ov5693, OV5693_REG_EXPOSURE_MID, (ctrl->val >> 8) & 0xff);
        if (!ret)
            ret = ov5693_write_reg(ov5693, OV5693_REG_EXPOSURE_LOW, ctrl->val & 0xff);
        break;

    case V4L2_CID_GAIN:
        ret = ov5693_write_reg(ov5693, OV5693_REG_GAIN, ctrl->val & 0xff);
        break;

    default:
        return -EINVAL;
    }

    return ret;
}

static const struct v4l2_ctrl_ops ov5693_ctrl_ops = {
    .s_ctrl = ov5693_s_ctrl,
};

/* Device tree parsing */
static int ov5693_parse_dt(struct ov5693 *ov5693)
{
    struct device *dev = &ov5693->client->dev;
    struct device_node *np = dev->of_node;
    int ret;

    /* CAM2_RSTN = PCC1 — INIT_HIGH para mantener sensor des-asserted */
    ov5693->reset_gpio = of_get_named_gpio(np, "reset-gpios", 0);
    if (gpio_is_valid(ov5693->reset_gpio)) {
        ret = devm_gpio_request_one(dev, ov5693->reset_gpio,
                                     GPIOF_OUT_INIT_HIGH, "ov5693_reset");
        if (ret)
            ov5693->reset_gpio = -EINVAL;
    }

    /* CAM2_PWDN = PBB6 — INIT_HIGH para mantener sensor des-asserted */
    ov5693->pwdn_gpio = of_get_named_gpio(np, "pwdn-gpios", 0);
    if (gpio_is_valid(ov5693->pwdn_gpio)) {
        ret = devm_gpio_request_one(dev, ov5693->pwdn_gpio,
                                     GPIOF_OUT_INIT_HIGH, "ov5693_pwdn");
        if (ret)
            ov5693->pwdn_gpio = -EINVAL;
    }

    /* CAM_AF_PWDN = PBB7 */
    ov5693->af_gpio = of_get_named_gpio(np, "af-gpios", 0);
    if (gpio_is_valid(ov5693->af_gpio)) {
        ret = devm_gpio_request_one(dev, ov5693->af_gpio,
                                     GPIOF_OUT_INIT_LOW, "ov5693_af_pwdn");
        if (ret)
            ov5693->af_gpio = -EINVAL;
    }

    /* DT supply-name mapped regulators */
    ov5693->avdd = devm_regulator_get(dev, "avdd");
    if (IS_ERR(ov5693->avdd))
        return PTR_ERR(ov5693->avdd);

    ov5693->dovdd = devm_regulator_get(dev, "dovdd");
    if (IS_ERR(ov5693->dovdd))
        return PTR_ERR(ov5693->dovdd);

    ov5693->dvdd = devm_regulator_get(dev, "dvdd");
    if (IS_ERR(ov5693->dvdd))
        return PTR_ERR(ov5693->dvdd);

    /* Global regulators (NOT mapped via DT supply-name - use NULL for global lookup) */
    ov5693->reg_af = regulator_get(NULL, "imx179_reg1");
    if (IS_ERR(ov5693->reg_af)) {
        pr_err("Cannot get regulator imx179_reg1: %ld\n", PTR_ERR(ov5693->reg_af));
        return PTR_ERR(ov5693->reg_af);
    }

    ov5693->reg_1v2 = regulator_get(NULL, "vdd_cam_1v2");
    if (IS_ERR(ov5693->reg_1v2)) {
        pr_err("Cannot get regulator vdd_cam_1v2: %ld\n", PTR_ERR(ov5693->reg_1v2));
        return PTR_ERR(ov5693->reg_1v2);
    }

    ov5693->reg_1v8 = regulator_get(NULL, "vdd_cam_1v8");
    if (IS_ERR(ov5693->reg_1v8)) {
        pr_err("Cannot get regulator vdd_cam_1v8: %ld\n", PTR_ERR(ov5693->reg_1v8));
        return PTR_ERR(ov5693->reg_1v8);
    }

    ov5693->mclk = devm_clk_get(dev, "mclk");
    if (IS_ERR(ov5693->mclk))
        return PTR_ERR(ov5693->mclk);

    ret = of_property_read_u32(np, "csi-port", &ov5693->csi_port);
    if (ret)
        ov5693->csi_port = 1;

    ret = of_property_read_u32(np, "num-lanes", &ov5693->num_lanes);
    if (ret)
        ov5693->num_lanes = 1;

    return 0;
}

/* I2C probe */
static int ov5693_probe(struct i2c_client *client,
                          const struct i2c_device_id *id)
{
    struct ov5693 *ov5693;
    int ret;

    ov5693 = devm_kzalloc(&client->dev, sizeof(*ov5693), GFP_KERNEL);
    if (!ov5693)
        return -ENOMEM;

    ov5693->client = client;
    ov5693->powered = false;
    ov5693->streaming = false;

    ret = ov5693_parse_dt(ov5693);
    if (ret) {
        dev_err(&client->dev, "Failed to parse device tree: %d\n", ret);
        return ret;
    }

    /* Standalone I2C probe: check if sensor responds on bus WITHOUT power sequence */
    {
        struct i2c_msg msg[2];
        u8 wbuf = 0x30;  /* register 0x300A high byte */
        u8 rbuf;
        msg[0].addr = client->addr;
        msg[0].flags = 0;
        msg[0].len = 1;
        msg[0].buf = &wbuf;
        msg[1].addr = client->addr;
        msg[1].flags = I2C_M_RD;
        msg[1].len = 1;
        msg[1].buf = &rbuf;
        ret = i2c_transfer(client->adapter, msg, 2);
        if (ret == 2)
            dev_info(&client->dev, "I2C standalone probe: sensor RESPONDS at 0x%02x (reg=0x%02x)\n",
                     client->addr, rbuf);
        else
            dev_warn(&client->dev, "I2C standalone probe: NO SENSOR at 0x%02x (%d) - not populated?\n",
                     client->addr, ret);
    }

    /* Initialize V4L2 subdev */
    v4l2_i2c_subdev_init(&ov5693->sd, client, &ov5693_subdev_ops);
    ov5693->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
    ov5693->sd.owner = THIS_MODULE;

    /* Initialize media pad */
    ov5693->pad.flags = MEDIA_PAD_FL_SOURCE;
    ret = media_entity_init(&ov5693->sd.entity, 1, &ov5693->pad, 0);
    if (ret) {
        dev_err(&client->dev, "Failed to init media entity: %d\n", ret);
        return ret;
    }

    /* Initialize controls */
    v4l2_ctrl_handler_init(&ov5693->ctrl_handler, 3);

    ov5693->exposure = v4l2_ctrl_new_std(&ov5693->ctrl_handler, &ov5693_ctrl_ops,
                                          V4L2_CID_EXPOSURE, 0, 0xFFFFFF, 1, 0x1000);
    ov5693->gain = v4l2_ctrl_new_std(&ov5693->ctrl_handler, &ov5693_ctrl_ops,
                                      V4L2_CID_GAIN, 0, 0xFF, 1, 0x10);
    ov5693->pixel_rate = v4l2_ctrl_new_std(&ov5693->ctrl_handler, &ov5693_ctrl_ops,
                                            V4L2_CID_PIXEL_RATE, 0, OV5693_PIXEL_RATE, 1, OV5693_PIXEL_RATE);

    if (ov5693->ctrl_handler.error) {
        ret = ov5693->ctrl_handler.error;
        dev_err(&client->dev, "Failed to init controls: %d\n", ret);
        goto err_media;
    }

    ov5693->sd.ctrl_handler = &ov5693->ctrl_handler;

    /* Set default format */
    ov5693->format.code = V4L2_MBUS_FMT_SBGGR10_1X10;
    ov5693->format.width = OV5693_DEFAULT_WIDTH;
    ov5693->format.height = OV5693_DEFAULT_HEIGHT;
    ov5693->format.field = V4L2_FIELD_NONE;
    ov5693->format.colorspace = V4L2_COLORSPACE_SRGB;
    ov5693->current_framesize = &ov5693_framesizes[0];

    i2c_set_clientdata(client, ov5693);

    /* Notificar al VI driver que este sensor está listo */
    ret = tegra_vi_register_sensor(&ov5693->sd, "ovt,ov5693");
    if (ret)
        dev_warn(&client->dev, "tegra_vi_register_sensor failed: %d\n", ret);

    /* [stock] NO power-on ni chip_id en probe — el capture thread lo hará al arrancar */
    pr_info("OV5693 subdev initialized\n");

    dev_info(&client->dev, "OV5693 sensor probed (CSI port %d, %d lanes)\n",
             ov5693->csi_port, ov5693->num_lanes);

    return 0;

err_media:
    v4l2_ctrl_handler_free(&ov5693->ctrl_handler);
    media_entity_cleanup(&ov5693->sd.entity);
    return ret;
}

static int ov5693_remove(struct i2c_client *client)
{
    struct ov5693 *ov5693 = i2c_get_clientdata(client);

    if (ov5693->powered)
        ov5693_set_power(ov5693, false);

    regulator_put(ov5693->reg_1v8);
    regulator_put(ov5693->reg_1v2);
    regulator_put(ov5693->reg_af);

    v4l2_ctrl_handler_free(&ov5693->ctrl_handler);
    media_entity_cleanup(&ov5693->sd.entity);

    return 0;
}

static const struct i2c_device_id ov5693_id[] = {
    { "ov5693", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, ov5693_id);

static const struct of_device_id ov5693_of_match[] = {
    { .compatible = "ovt,ov5693" },
    { .compatible = "nvidia,ov5693-mocha" },
    { }
};
MODULE_DEVICE_TABLE(of, ov5693_of_match);

static struct i2c_driver ov5693_driver = {
    .probe  = ov5693_probe,
    .remove = ov5693_remove,
    .id_table = ov5693_id,
    .driver = {
        .name           = "ov5693",
        .of_match_table = ov5693_of_match,
    },
};
module_i2c_driver(ov5693_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Dargons10 <dargons10@users.noreply.github.com>");
MODULE_DESCRIPTION("OmniVision OV5693 sensor driver for Mocha (Tegra K1)");
