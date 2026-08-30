/*
 * imx179.c - Sony IMX179 sensor V4L2 subdev driver
 *
 * Copyright (c) 2026, Dargons10
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * Sony IMX179 is an 8MP CMOS image sensor with RGGB Bayer pattern.
 * Used in Xiaomi Mi Pad (mocha) rear camera on CSI-A 4-lane.
 *
 * Compatible with Linux kernel 3.10
 */

#define pr_fmt(fmt) "imx179: " fmt

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
#include "../tegra_vi.h"
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-common.h>
#include <media/v4l2-of.h>

/* IMX179 registers */
/* El chip ID real del IMX179 está en 0x0002 (2 bytes, valor 0x8179 según kernel stock) */
#define IMX179_REG_CHIP_ID          0x0002

#define IMX179_CHIP_ID_VALUE        0x8179
#define IMX179_REG_MODE_SELECT      0x0100
#define IMX179_MODE_STANDBY         0x00
#define IMX179_MODE_STREAMING       0x01

#define IMX179_REG_COARSE_INTEGRATION_TIME  0x0202
#define IMX179_REG_ANALOGUE_GAIN            0x0205

/* Default mode configuration */
#define IMX179_DEFAULT_WIDTH        3264
#define IMX179_DEFAULT_HEIGHT       2448
#define IMX179_DEFAULT_FPS          30
#define IMX179_PIXEL_RATE           200000000ULL
#define IMX179_LINE_LENGTH          3440
#define IMX179_FRAME_LENGTH         2500

/* Supported media bus formats */
static const u32 imx179_mbus_formats[] = {
    V4L2_MBUS_FMT_SRGGB10_1X10,
    V4L2_MBUS_FMT_SRGGB8_1X8,
};

/* Frame sizes supported */
struct imx179_framesize {
    u32 width;
    u32 height;
    u32 max_fps;
};

static const struct imx179_framesize imx179_framesizes[] = {
    { 3264, 2448, 15 },  /* 8MP @ 15fps */
    { 2592, 1944, 20 },  /* 5MP @ 20fps */
    { 1920, 1080, 30 },  /* 1080p @ 30fps */
    { 1280, 720,  60 },  /* 720p @ 60fps */
    { 640,  480,  90 },  /* VGA @ 90fps */
};

#define IMX179_NUM_FRAMESIZES ARRAY_SIZE(imx179_framesizes)

/* Sensor initialization registers */
static const u8 imx179_init_regs[][3] = {
    { 0x01, 0x00, 0x00 },
    { 0x01, 0x01, 0x00 },
    { 0x03, 0x01, 0x05 },
    { 0x03, 0x03, 0x01 },
    { 0x03, 0x05, 0x06 },
    { 0x03, 0x09, 0x05 },
    { 0x03, 0x0B, 0x01 },
    { 0x03, 0x0C, 0x00 },
    { 0x03, 0x0D, 0xA2 },
    { 0x03, 0x40, 0x09 },
    { 0x03, 0x41, 0xCE },
    { 0x03, 0x42, 0x0D },
    { 0x03, 0x43, 0x70 },
    { 0x03, 0x44, 0x00 },
    { 0x03, 0x45, 0x00 },
    { 0x03, 0x46, 0x00 },
    { 0x03, 0x47, 0x02 },
    { 0x03, 0x48, 0x0C },
    { 0x03, 0x49, 0xCF },
    { 0x03, 0x4A, 0x09 },
    { 0x03, 0x4B, 0x9D },
    { 0x03, 0x4C, 0x0C },
    { 0x03, 0x4D, 0xD0 },
    { 0x03, 0x4E, 0x09 },
    { 0x03, 0x4F, 0x9C },
    { 0x03, 0x83, 0x01 },
    { 0x03, 0x87, 0x01 },
    { 0x03, 0x90, 0x00 },
    { 0x04, 0x01, 0x00 },
    { 0x04, 0x05, 0x10 },
    { 0x30, 0x20, 0x10 },
    { 0x30, 0x41, 0x15 },
    { 0x30, 0x42, 0x87 },
    { 0x30, 0x89, 0x4F },
    { 0x33, 0x02, 0x01 },
    { 0x33, 0x09, 0x9A },
    { 0x33, 0x44, 0x57 },
    { 0x33, 0x45, 0x1F },
    { 0x33, 0x62, 0x0A },
    { 0x33, 0x63, 0x0A },
    { 0x33, 0x64, 0x00 },
    { 0x33, 0x68, 0x18 },
    { 0x33, 0x69, 0x00 },
    { 0x33, 0x70, 0x77 },
    { 0x33, 0x71, 0x2F },
    { 0x33, 0x72, 0x4F },
    { 0x33, 0x73, 0x2F },
    { 0x33, 0x74, 0x2F },
    { 0x33, 0x75, 0x37 },
    { 0x33, 0x76, 0x9F },
    { 0x33, 0x77, 0x37 },
    { 0x33, 0xC8, 0x00 },
    { 0x33, 0xD4, 0x0C },
    { 0x33, 0xD5, 0xD0 },
    { 0x33, 0xD6, 0x09 },
    { 0x33, 0xD7, 0x9C },
    { 0x41, 0x00, 0x0E },
    { 0x41, 0x08, 0x01 },
    { 0x41, 0x09, 0x7C },
};

#define IMX179_NUM_INIT_REGS ARRAY_SIZE(imx179_init_regs)

/* IMX179 device structure */
struct imx179 {
    struct i2c_client *client;
    struct v4l2_subdev sd;
    struct v4l2_ctrl_handler ctrl_handler;
    struct media_pad pad;

    struct regulator *vana;    /* avdd - palmas_ldo4 2.7V */
    struct regulator *vdig;    /* digital core */
    struct regulator *vif;     /* iovdd - palmas_ldo6 1.8V */
    struct regulator *reg1;    /* imx179_reg1 - palmas_ldo7 2.7V */
    struct regulator *reg_1v2; /* vdd_cam_1v2 fixed 1.2V */
    struct regulator *reg_1v8; /* vdd_cam_1v8 fixed 1.8V */
    struct clk *mclk;

    int reset_gpio;
    int pwdn_gpio;  /* PBB5 - not used in hardware, keep for DT compat */
    int af_gpio;    /* CAM_AF_PWDN = PBB7 */

    bool powered;
    bool streaming;
    u32 csi_port;
    u32 num_lanes;

    struct v4l2_mbus_framefmt format;
    const struct imx179_framesize *current_framesize;

    /* Controls */
    struct v4l2_ctrl *exposure;
    struct v4l2_ctrl *gain;
    struct v4l2_ctrl *pixel_rate;
};

static inline struct imx179 *to_imx179(struct v4l2_subdev *sd)
{
    return container_of(sd, struct imx179, sd);
}

/* I2C register access */
static int imx179_write_reg(struct imx179 *imx179, u16 reg, u8 val)
{
    struct i2c_client *client = imx179->client;
    u8 buf[3];
    int ret;
    int retries = 5;
    int delay_us[] = {0, 1000, 5000, 10000, 20000};  /* 0, 1, 5, 10, 20 ms */

    buf[0] = (reg >> 8) & 0xff;
    buf[1] = reg & 0xff;
    buf[2] = val;

    /* Retry with exponential backoff on I2C failure */
    while (retries > 0) {
        if (delay_us[3 - retries] > 0)
            usleep_range(delay_us[3 - retries], delay_us[3 - retries] * 2);

        ret = i2c_master_send(client, buf, 3);
        if (ret == 3)
            return 0;

        dev_warn(&client->dev, "i2c_write reg(0x%04x)=0x%02x failed (ret=%d), retrying...\n",
                 reg, val, ret);
        retries--;
    }

    dev_err(&client->dev, "i2c_write reg(0x%04x)=0x%02x failed after retries (ret=%d)\n",
            reg, val, ret);
    return -EIO;
}

static int imx179_read_reg(struct imx179 *imx179, u16 reg, u8 *val)
{
    struct i2c_client *client = imx179->client;
    struct i2c_msg msg[2];
    u8 buf[2];
    int ret;
    int retries = 5;

    buf[0] = reg >> 8;
    buf[1] = reg & 0xFF;

    /* Transacción combinada (repeated START) */
    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].len = 2;
    msg[0].buf = buf;

    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = 1;
    msg[1].buf = val;

    do {
        ret = i2c_transfer(client->adapter, msg, 2);
        if (ret == 2)
            return 0;
        usleep_range(2000, 4000);
        retries--;
    } while (retries > 0);

    if (ret != 2) {
        dev_err(&client->dev, "i2c_comb reg(0x%04x): ret=%d (write=%s read=%s)\n",
                reg, ret,
                (ret >= 1) ? "OK" : "FAIL",
                (ret >= 2) ? "OK" : "FAIL");
        return ret < 0 ? ret : -EIO;
    }

    return 0;
}

/* Forward declaration */
static int imx179_write_init_regs(struct imx179 *imx179);

/* Soft reset: put sensor in standby, wait, then resume */
static int imx179_soft_reset(struct imx179 *imx179)
{
    struct i2c_client *client = imx179->client;
    int ret;

    dev_warn(&client->dev, "Performing soft reset of IMX179\n");

    /* Put sensor in standby */
    ret = imx179_write_reg(imx179, IMX179_REG_MODE_SELECT, IMX179_MODE_STANDBY);
    if (ret) {
        dev_err(&client->dev, "Soft reset: standby write failed\n");
        return ret;
    }

    /* Wait for standby to take effect */
    msleep(10);

    /* Re-apply init registers to restore sensor to known state */
    ret = imx179_write_init_regs(imx179);
    if (ret) {
        dev_err(&client->dev, "Soft reset: init regs failed\n");
        return ret;
    }

    dev_info(&client->dev, "Soft reset complete\n");
    return 0;
}

static int imx179_write_init_regs(struct imx179 *imx179)
{
    int i, ret;

    for (i = 0; i < IMX179_NUM_INIT_REGS; i++) {
        u16 reg = (imx179_init_regs[i][0] << 8) | imx179_init_regs[i][1];
        ret = imx179_write_reg(imx179, reg, imx179_init_regs[i][2]);
        if (ret) {
            pr_err("Failed to write reg 0x%04x: %d\n", reg, ret);
            return ret;
        }
    }

    return 0;
}

static int imx179_check_chip_id(struct imx179 *imx179)
{
    u8 id_high, id_low;
    u16 chip_id;
    int ret;

    /* IMX179: chip ID de 16 bits en registro 0x0002 (igual que NVC framework stock) */
    ret = imx179_read_reg(imx179, IMX179_REG_CHIP_ID, &id_high);
    if (ret)
        return ret;

    ret = imx179_read_reg(imx179, IMX179_REG_CHIP_ID + 1, &id_low);
    if (ret)
        return ret;

    chip_id = (id_high << 8) | id_low;
    pr_info("Chip ID: 0x%04x (expected 0x%04x or 0x%04x)\n",
            chip_id, IMX179_CHIP_ID_VALUE, IMX179_CHIP_ID_VALUE & 0x7FFF);

    /* Acepta exacto (0x8179) o con flag de validez en bit 15 (0x0179) */
    if (chip_id == IMX179_CHIP_ID_VALUE || (chip_id & 0x7FFF) == (IMX179_CHIP_ID_VALUE & 0x7FFF)) {
        pr_info("Chip ID verified (0x%04x) - sensor ready\n", chip_id);
        return 0;
    }

    return -ENODEV;
}

/* Power management - matching board-ardbeg-sensors.c:ardbeg_imx179_power_on */
static int imx179_set_power(struct imx179 *imx179, bool on)
{
    int ret;

    if (on && !imx179->powered) {
        pr_info("Powering on IMX179\n");

        /* [NVC core order] MCLK FIRST (set_rate + enable) - sensor held in reset later */
        clk_prepare_enable(imx179->mclk);        /* refcnt ≥ 1 */
        clk_disable_unprepare(imx179->mclk);      /* refcnt → 0, MCLK stops */
        if (clk_get_rate(imx179->mclk) != 24000000)
            clk_set_rate(imx179->mclk, 24000000); /* Cambia divisor sin glitch */
        ret = clk_prepare_enable(imx179->mclk);   /* MCLK a 24 MHz */
        if (ret) {
            pr_err("Failed to enable mclk: %d\n", ret);
            return ret;
        }

        /* [NVC driver order] GPIOs first, sensor in reset */
        if (gpio_is_valid(imx179->reset_gpio))
            gpio_set_value(imx179->reset_gpio, 0);   /* RSTN LOW (assert reset) */
        if (gpio_is_valid(imx179->af_gpio))
            gpio_set_value(imx179->af_gpio, 1);       /* CAM_AF_PWDN HIGH */
        usleep_range(10, 20);

        /* Luego reguladores (avdd + iovdd, igual que stock imx179_power_on) */
        ret = regulator_enable(imx179->reg1);
        if (ret) { pr_err("Failed reg1: %d\n", ret); return ret; }
        ret = regulator_enable(imx179->reg_1v2);
        if (ret) { pr_err("Failed 1v2: %d\n", ret); goto err_1v2; }
        ret = regulator_enable(imx179->reg_1v8);
        if (ret) { pr_err("Failed 1v8: %d\n", ret); goto err_1v8; }
        ret = regulator_enable(imx179->vana);
        if (ret) { pr_err("Failed vana: %d\n", ret); goto err_vana; }
        ret = regulator_enable(imx179->vdig);
        if (ret) { pr_err("Failed vdig: %d\n", ret); goto err_vdig; }
        ret = regulator_enable(imx179->vif);
        if (ret) { pr_err("Failed vif: %d\n", ret); goto err_vif; }
        usleep_range(1, 2);

        /* Salir de reset */
        if (gpio_is_valid(imx179->reset_gpio))
            gpio_set_value(imx179->reset_gpio, 1);   /* RSTN HIGH (deassert reset) */
        usleep_range(300, 310);

        ret = imx179_check_chip_id(imx179);
        if (ret) {
            pr_err("Chip ID check failed: %d\n", ret);
            if (gpio_is_valid(imx179->reset_gpio))
                gpio_set_value(imx179->reset_gpio, 0);
            goto err_chip_id;
        }

        imx179->powered = true;

        /* Aplicar controles V4L2 (gain, exposure) a los registros hardware */
        v4l2_ctrl_handler_setup(&imx179->ctrl_handler);

        pr_info("IMX179 ready (gain=0x%02x, exposure=%d)\n",
                imx179->gain ? (u32)imx179->gain->val : 0,
                imx179->exposure ? (u32)imx179->exposure->val : 0);
        return 0;

err_chip_id:
        regulator_disable(imx179->vif);
err_vif:
        regulator_disable(imx179->vdig);
err_vdig:
        regulator_disable(imx179->vana);
err_vana:
        regulator_disable(imx179->reg_1v8);
err_1v8:
        regulator_disable(imx179->reg_1v2);
err_1v2:
        regulator_disable(imx179->reg1);
        clk_disable_unprepare(imx179->mclk);
        return ret;
    } else if (!on && imx179->powered) {
        pr_info("Powering off IMX179\n");

        if (gpio_is_valid(imx179->reset_gpio))
            gpio_set_value(imx179->reset_gpio, 0);
        if (gpio_is_valid(imx179->af_gpio))
            gpio_set_value(imx179->af_gpio, 0);
        usleep_range(1, 2);

        clk_disable_unprepare(imx179->mclk);
        regulator_disable(imx179->vif);
        regulator_disable(imx179->vdig);
        regulator_disable(imx179->vana);
        regulator_disable(imx179->reg_1v8);
        regulator_disable(imx179->reg_1v2);
        regulator_disable(imx179->reg1);

        imx179->powered = false;
        imx179->streaming = false;
        pr_info("Sensor powered off\n");
    }

    return 0;
}

/* V4L2 subdev core ops */
static int imx179_s_power(struct v4l2_subdev *sd, int on)
{
    struct imx179 *imx179 = to_imx179(sd);
    return imx179_set_power(imx179, on);
}

static int imx179_init(struct v4l2_subdev *sd, u32 val)
{
    struct imx179 *imx179 = to_imx179(sd);
    int ret;

    ret = imx179_check_chip_id(imx179);
    if (ret) {
        pr_err("Chip ID check failed\n");
        return ret;
    }

    ret = imx179_write_init_regs(imx179);
    if (ret) {
        pr_err("Failed to initialize sensor: %d\n", ret);
        return ret;
    }

    pr_info("Sensor initialized\n");
    return 0;
}

static const struct v4l2_subdev_core_ops imx179_core_ops = {
    .s_power = imx179_s_power,
    .init = imx179_init,
};

/* V4L2 subdev video ops */
static int imx179_enum_mbus_code(struct v4l2_subdev *sd,
                                  struct v4l2_subdev_fh *fh,
                                  struct v4l2_subdev_mbus_code_enum *code)
{
    if (code->index >= ARRAY_SIZE(imx179_mbus_formats))
        return -EINVAL;

    code->code = imx179_mbus_formats[code->index];
    return 0;
}

static int imx179_enum_frame_size(struct v4l2_subdev *sd,
                                   struct v4l2_subdev_fh *fh,
                                   struct v4l2_subdev_frame_size_enum *fse)
{
    if (fse->index >= IMX179_NUM_FRAMESIZES)
        return -EINVAL;

    if (fse->code != imx179_mbus_formats[0] &&
        fse->code != imx179_mbus_formats[1])
        return -EINVAL;

    fse->min_width = imx179_framesizes[fse->index].width;
    fse->max_width = imx179_framesizes[fse->index].width;
    fse->min_height = imx179_framesizes[fse->index].height;
    fse->max_height = imx179_framesizes[fse->index].height;

    return 0;
}

static struct v4l2_mbus_framefmt *
imx179_get_format(struct imx179 *imx179, struct v4l2_subdev_fh *fh,
                  unsigned int pad, enum v4l2_subdev_format_whence which)
{
    if (which == V4L2_SUBDEV_FORMAT_TRY)
        return v4l2_subdev_get_try_format(fh, pad);

    return &imx179->format;
}

static int imx179_get_fmt(struct v4l2_subdev *sd,
                           struct v4l2_subdev_fh *fh,
                           struct v4l2_subdev_format *fmt)
{
    struct imx179 *imx179 = to_imx179(sd);
    struct v4l2_mbus_framefmt *format;

    format = imx179_get_format(imx179, fh, fmt->pad, fmt->which);
    fmt->format = *format;

    return 0;
}

static int imx179_set_fmt(struct v4l2_subdev *sd,
                           struct v4l2_subdev_fh *fh,
                           struct v4l2_subdev_format *fmt)
{
    struct imx179 *imx179 = to_imx179(sd);
    struct v4l2_mbus_framefmt *format;
    int i;

    format = imx179_get_format(imx179, fh, fmt->pad, fmt->which);

    /* Find closest format */
    for (i = 0; i < ARRAY_SIZE(imx179_mbus_formats); i++) {
        if (imx179_mbus_formats[i] == fmt->format.code)
            break;
    }
    if (i == ARRAY_SIZE(imx179_mbus_formats))
        fmt->format.code = imx179_mbus_formats[0];

    /* Find closest frame size */
    for (i = 0; i < IMX179_NUM_FRAMESIZES; i++) {
        if (imx179_framesizes[i].width == fmt->format.width &&
            imx179_framesizes[i].height == fmt->format.height)
            break;
    }
    if (i == IMX179_NUM_FRAMESIZES) {
        fmt->format.width = IMX179_DEFAULT_WIDTH;
        fmt->format.height = IMX179_DEFAULT_HEIGHT;
        /* Find the fallback entry index */
        for (i = 0; i < IMX179_NUM_FRAMESIZES; i++) {
            if (imx179_framesizes[i].width == IMX179_DEFAULT_WIDTH &&
                imx179_framesizes[i].height == IMX179_DEFAULT_HEIGHT)
                break;
        }
    }

    fmt->format.field = V4L2_FIELD_NONE;
    fmt->format.colorspace = V4L2_COLORSPACE_SRGB;

    *format = fmt->format;

    if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
        imx179->current_framesize = &imx179_framesizes[i];
    }

    return 0;
}

static int imx179_s_stream(struct v4l2_subdev *sd, int enable)
{
    struct imx179 *imx179 = to_imx179(sd);
    struct i2c_client *client = imx179->client;
    int ret;
    int attempt;
    u8 chip_id_hi, chip_id_lo;

    if (enable == imx179->streaming)
        return 0;

    if (enable) {
        /* Retry the whole stream-on sequence up to 2 times if I2C errors occur.
         * This handles the case where the sensor was left in a bad state by
         * a previous HAL close/open cycle (STREAMOFF followed quickly by STREAMON).
         */
        for (attempt = 0; attempt < 2; attempt++) {
            /* Verify sensor is responsive before writing registers.
             * If not, perform soft reset to recover.
             */
            if (imx179_read_reg(imx179, 0x0013, &chip_id_hi) != 0 ||
                imx179_read_reg(imx179, 0x0014, &chip_id_lo) != 0) {
                dev_warn(&client->dev, "s_stream: chip ID read failed, attempting soft reset (attempt %d)\n", attempt);
                if (imx179_soft_reset(imx179) != 0) {
                    dev_err(&client->dev, "s_stream: soft reset failed\n");
                    continue;
                }
                /* After soft reset, retry the chip ID read */
                if (imx179_read_reg(imx179, 0x0013, &chip_id_hi) != 0) {
                    dev_err(&client->dev, "s_stream: chip ID read still failing after reset\n");
                    continue;
                }
            }

            ret = imx179_write_init_regs(imx179);
            if (ret) {
                dev_warn(&client->dev, "s_stream: init regs failed (attempt %d)\n", attempt);
                imx179_soft_reset(imx179);
                continue;
            }

            /* Program output window and binning based on resolution */
            {
                u16 out_width = imx179->format.width;
                u16 out_height = imx179->format.height;
                u16 sensor_w = 3264, sensor_h = 2448;
                u16 start_x, start_y, end_x, end_y;
                u16 binned_w, binned_h;
                bool use_binning;
                int reg_err = 0;

                /* Use 2x binning only when the 2x window fits in the sensor */
                use_binning = (out_width * 2 <= sensor_w && out_height * 2 <= sensor_h);

                if (use_binning) {
                    binned_w = out_width * 2;
                    binned_h = out_height * 2;
                    reg_err |= imx179_write_reg(imx179, 0x0301, 0x05);
                    reg_err |= imx179_write_reg(imx179, 0x0383, 0x01);
                    reg_err |= imx179_write_reg(imx179, 0x0385, 0x01);
                    reg_err |= imx179_write_reg(imx179, 0x0387, 0x01);
                    reg_err |= imx179_write_reg(imx179, 0x0389, 0x01);
                } else {
                    binned_w = out_width;
                    binned_h = out_height;
                    reg_err |= imx179_write_reg(imx179, 0x0301, 0x00);
                    reg_err |= imx179_write_reg(imx179, 0x0383, 0x00);
                    reg_err |= imx179_write_reg(imx179, 0x0385, 0x00);
                    reg_err |= imx179_write_reg(imx179, 0x0387, 0x00);
                    reg_err |= imx179_write_reg(imx179, 0x0389, 0x00);
                }

                /* Center the window */
                start_x = (sensor_w - binned_w) / 2;
                start_y = (sensor_h - binned_h) / 2;
                end_x = start_x + binned_w - 1;
                end_y = start_y + binned_h - 1;

                reg_err |= imx179_write_reg(imx179, 0x0344, (start_x >> 8) & 0xFF);
                reg_err |= imx179_write_reg(imx179, 0x0345, start_x & 0xFF);
                reg_err |= imx179_write_reg(imx179, 0x0346, (start_y >> 8) & 0xFF);
                reg_err |= imx179_write_reg(imx179, 0x0347, start_y & 0xFF);
                reg_err |= imx179_write_reg(imx179, 0x0348, (end_x >> 8) & 0xFF);
                reg_err |= imx179_write_reg(imx179, 0x0349, end_x & 0xFF);
                reg_err |= imx179_write_reg(imx179, 0x034A, (end_y >> 8) & 0xFF);
                reg_err |= imx179_write_reg(imx179, 0x034B, end_y & 0xFF);
                reg_err |= imx179_write_reg(imx179, 0x034C, (out_width >> 8) & 0xFF);
                reg_err |= imx179_write_reg(imx179, 0x034D, out_width & 0xFF);
                reg_err |= imx179_write_reg(imx179, 0x034E, (out_height >> 8) & 0xFF);
                reg_err |= imx179_write_reg(imx179, 0x034F, out_height & 0xFF);
                reg_err |= imx179_write_reg(imx179, 0x0340, 0x09);
                reg_err |= imx179_write_reg(imx179, 0x0341, 0xCE);
                reg_err |= imx179_write_reg(imx179, 0x0342, 0x0D);
                reg_err |= imx179_write_reg(imx179, 0x0343, 0x70);
                reg_err |= imx179_write_reg(imx179, 0x0202, 0x09);
                reg_err |= imx179_write_reg(imx179, 0x0203, 0x60);
                reg_err |= imx179_write_reg(imx179, 0x0205, 0x40);

                if (reg_err) {
                    dev_warn(&client->dev, "s_stream: register write failed, attempting soft reset (attempt %d)\n", attempt);
                    imx179_soft_reset(imx179);
                    continue;
                }

                pr_info("Output %dx%d window [%d,%d]-[%d,%d]%s\n",
                        out_width, out_height,
                        start_x, start_y, end_x, end_y,
                        use_binning ? " binning=2x2" : "");
            }

            ret = imx179_write_reg(imx179, IMX179_REG_MODE_SELECT,
                                    IMX179_MODE_STREAMING);
            if (ret) {
                dev_warn(&client->dev, "s_stream: mode select write failed (attempt %d)\n", attempt);
                imx179_soft_reset(imx179);
                continue;
            }

            /* Success! */
            imx179->streaming = true;
            pr_info("Streaming started\n");
            return 0;
        }

        dev_err(&client->dev, "s_stream: all attempts failed, sensor may be in bad state\n");
        return -EIO;
    } else {
        ret = imx179_write_reg(imx179, IMX179_REG_MODE_SELECT,
                                IMX179_MODE_STANDBY);
        if (ret)
            dev_warn(&client->dev, "s_stream: standby write failed, continuing anyway\n");

        imx179->streaming = false;
        pr_info("Streaming stopped\n");
    }

    return 0;
}

static const struct v4l2_subdev_video_ops imx179_video_ops = {
    .s_stream = imx179_s_stream,
};

/* V4L2 subdev pad ops */
static const struct v4l2_subdev_pad_ops imx179_pad_ops = {
    .enum_mbus_code = imx179_enum_mbus_code,
    .enum_frame_size = imx179_enum_frame_size,
    .get_fmt = imx179_get_fmt,
    .set_fmt = imx179_set_fmt,
};

/* V4L2 subdev ops */
static const struct v4l2_subdev_ops imx179_subdev_ops = {
    .core = &imx179_core_ops,
    .video = &imx179_video_ops,
    .pad = &imx179_pad_ops,
};

/* V4L2 control ops */
static int imx179_s_ctrl(struct v4l2_ctrl *ctrl)
{
    struct imx179 *imx179 = container_of(ctrl->handler, struct imx179, ctrl_handler);
    int ret = 0;

    switch (ctrl->id) {
    case V4L2_CID_EXPOSURE:
        /* IMX179: coarse_time[15:8] → reg 0x0202 (MSB), [7:0] → reg 0x0203 (LSB) */
        ret = imx179_write_reg(imx179, IMX179_REG_COARSE_INTEGRATION_TIME,
                                (ctrl->val >> 8) & 0xff);
        if (!ret)
            ret = imx179_write_reg(imx179, IMX179_REG_COARSE_INTEGRATION_TIME + 1,
                                    ctrl->val & 0xff);
        break;

    case V4L2_CID_GAIN:
        ret = imx179_write_reg(imx179, IMX179_REG_ANALOGUE_GAIN, ctrl->val & 0xff);
        break;



    default:
        return -EINVAL;
    }

    return ret;
}

static const struct v4l2_ctrl_ops imx179_ctrl_ops = {
    .s_ctrl = imx179_s_ctrl,
};

/* Device tree parsing */
static int imx179_parse_dt(struct imx179 *imx179)
{
    struct device *dev = &imx179->client->dev;
    struct device_node *np = dev->of_node;
    int ret;

    /* CAM_RSTN = PBB3 — INIT_HIGH para mantener sensor fuera de reset */
    imx179->reset_gpio = of_get_named_gpio(np, "reset-gpios", 0);
    if (gpio_is_valid(imx179->reset_gpio)) {
        ret = devm_gpio_request_one(dev, imx179->reset_gpio,
                                     GPIOF_OUT_INIT_HIGH, "imx179_reset");
        if (ret)
            imx179->reset_gpio = -EINVAL;
    }

    /* PBB5 - defined in DT as pwdn but NOT connected in hardware */
    imx179->pwdn_gpio = of_get_named_gpio(np, "pwdn-gpios", 0);
    if (gpio_is_valid(imx179->pwdn_gpio)) {
        ret = devm_gpio_request_one(dev, imx179->pwdn_gpio,
                                     GPIOF_OUT_INIT_HIGH, "imx179_pwdn");
        if (ret)
            imx179->pwdn_gpio = -EINVAL;
    }

    /* CAM_AF_PWDN = PBB7 — INIT_LOW (evita interferencia en I2C) */
    imx179->af_gpio = of_get_named_gpio(np, "af-gpios", 0);
    if (gpio_is_valid(imx179->af_gpio)) {
        ret = devm_gpio_request_one(dev, imx179->af_gpio,
                                     GPIOF_OUT_INIT_LOW, "imx179_af_pwdn");
        if (ret)
            imx179->af_gpio = -EINVAL;
    }

    /* DT supply-name mapped regulators */
    imx179->vana = devm_regulator_get(dev, "vana");
    if (IS_ERR(imx179->vana))
        return PTR_ERR(imx179->vana);

    imx179->vdig = devm_regulator_get(dev, "vdig");
    if (IS_ERR(imx179->vdig))
        return PTR_ERR(imx179->vdig);

    imx179->vif = devm_regulator_get(dev, "vif");
    if (IS_ERR(imx179->vif))
        return PTR_ERR(imx179->vif);

    /* Global regulators (NOT mapped via DT supply-name - use NULL for global lookup) */
    imx179->reg1 = regulator_get(NULL, "imx179_reg1");
    if (IS_ERR(imx179->reg1)) {
        pr_err("Cannot get regulator imx179_reg1: %ld\n", PTR_ERR(imx179->reg1));
        return PTR_ERR(imx179->reg1);
    }

    imx179->reg_1v2 = regulator_get(NULL, "vdd_cam_1v2");
    if (IS_ERR(imx179->reg_1v2)) {
        pr_err("Cannot get regulator vdd_cam_1v2: %ld\n", PTR_ERR(imx179->reg_1v2));
        return PTR_ERR(imx179->reg_1v2);
    }

    imx179->reg_1v8 = regulator_get(NULL, "vdd_cam_1v8");
    if (IS_ERR(imx179->reg_1v8)) {
        pr_err("Cannot get regulator vdd_cam_1v8: %ld\n", PTR_ERR(imx179->reg_1v8));
        return PTR_ERR(imx179->reg_1v8);
    }

    imx179->mclk = devm_clk_get(dev, "mclk");
    if (IS_ERR(imx179->mclk))
        return PTR_ERR(imx179->mclk);

    ret = of_property_read_u32(np, "csi-port", &imx179->csi_port);
    if (ret)
        imx179->csi_port = 0;

    ret = of_property_read_u32(np, "num-lanes", &imx179->num_lanes);
    if (ret)
        imx179->num_lanes = 4;

    return 0;
}

/* I2C probe */
static int imx179_probe(struct i2c_client *client,
                          const struct i2c_device_id *id)
{
    struct imx179 *imx179;
    int ret;

    imx179 = devm_kzalloc(&client->dev, sizeof(*imx179), GFP_KERNEL);
    if (!imx179)
        return -ENOMEM;

    imx179->client = client;
    imx179->powered = false;
    imx179->streaming = false;

    ret = imx179_parse_dt(imx179);
    if (ret) {
        dev_err(&client->dev, "Failed to parse device tree: %d\n", ret);
        return ret;
    }

    /* Initialize V4L2 subdev */
    v4l2_i2c_subdev_init(&imx179->sd, client, &imx179_subdev_ops);
    imx179->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
    imx179->sd.owner = THIS_MODULE;

    /* Initialize media pad */
    imx179->pad.flags = MEDIA_PAD_FL_SOURCE;
    ret = media_entity_init(&imx179->sd.entity, 1, &imx179->pad, 0);
    if (ret) {
        dev_err(&client->dev, "Failed to init media entity: %d\n", ret);
        return ret;
    }

    /* Initialize controls */
    v4l2_ctrl_handler_init(&imx179->ctrl_handler, 4);

    imx179->exposure = v4l2_ctrl_new_std(&imx179->ctrl_handler, &imx179_ctrl_ops,
                                          V4L2_CID_EXPOSURE, 1, 2500, 1, 2400);
    imx179->gain = v4l2_ctrl_new_std(&imx179->ctrl_handler, &imx179_ctrl_ops,
                                        V4L2_CID_GAIN, 0, 0xFF, 1, 0x40);
    imx179->pixel_rate = v4l2_ctrl_new_std(&imx179->ctrl_handler, &imx179_ctrl_ops,
                                            V4L2_CID_PIXEL_RATE, 0, IMX179_PIXEL_RATE, 1, IMX179_PIXEL_RATE);

    if (imx179->ctrl_handler.error) {
        ret = imx179->ctrl_handler.error;
        dev_err(&client->dev, "Failed to init controls: %d\n", ret);
        goto err_media;
    }

    imx179->sd.ctrl_handler = &imx179->ctrl_handler;

    /* Set default format */
    imx179->format.code = V4L2_MBUS_FMT_SRGGB10_1X10;
    imx179->format.width = IMX179_DEFAULT_WIDTH;
    imx179->format.height = IMX179_DEFAULT_HEIGHT;
    imx179->format.field = V4L2_FIELD_NONE;
    imx179->format.colorspace = V4L2_COLORSPACE_SRGB;
    imx179->current_framesize = &imx179_framesizes[0];

    i2c_set_clientdata(client, imx179);

    /* Notificar al VI driver que este sensor está listo */
    ret = tegra_vi_register_sensor(&imx179->sd, "sony,imx179");
    if (ret)
        dev_warn(&client->dev, "tegra_vi_register_sensor failed: %d\n", ret);

    /* Power cycle: bootloader dejó el sensor en estado desconocido */
    pr_info("Power cycling IMX179 for clean init\n");
    ret = imx179_set_power(imx179, true);
    if (ret) {
        dev_err(&client->dev, "Power-on during IMX179 probe failed: %d\n", ret);
    } else {
        /* Power off de forma limpia; el capture thread hará power_on cuando toque */
        imx179_set_power(imx179, false);
        pr_info("IMX179 power-cycled successfully in probe\n");
    }

    pr_info("IMX179 subdev initialized\n");

    dev_info(&client->dev, "IMX179 sensor probed (CSI port %d, %d lanes)\n",
             imx179->csi_port, imx179->num_lanes);

    return 0;

err_media:
    v4l2_ctrl_handler_free(&imx179->ctrl_handler);
    media_entity_cleanup(&imx179->sd.entity);
    return ret;
}

static int imx179_remove(struct i2c_client *client)
{
    struct imx179 *imx179 = i2c_get_clientdata(client);

    if (imx179->powered)
        imx179_set_power(imx179, false);

    regulator_put(imx179->reg_1v8);
    regulator_put(imx179->reg_1v2);
    regulator_put(imx179->reg1);

    v4l2_ctrl_handler_free(&imx179->ctrl_handler);
    media_entity_cleanup(&imx179->sd.entity);

    return 0;
}

static const struct i2c_device_id imx179_id[] = {
    { "imx179", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, imx179_id);

static const struct of_device_id imx179_of_match[] = {
    { .compatible = "sony,imx179" },
    { .compatible = "nvidia,imx179-mocha" },
    { }
};
MODULE_DEVICE_TABLE(of, imx179_of_match);

static struct i2c_driver imx179_driver = {
    .probe  = imx179_probe,
    .remove = imx179_remove,
    .id_table = imx179_id,
    .driver = {
        .name           = "imx179",
        .of_match_table = imx179_of_match,
    },
};
module_i2c_driver(imx179_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Dargons10 <dargons10@users.noreply.github.com>");
MODULE_DESCRIPTION("Sony IMX179 sensor driver for Mocha (Tegra K1)");
