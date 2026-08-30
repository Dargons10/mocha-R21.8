/*
 * tegra_tpg.c - Test Pattern Generator for Tegra VI
 *
 * Copyright (c) 2026, Dargons10
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * Generates test pattern frames for debugging when real sensors fail.
 * Supports multiple patterns: gradient, checkerboard, color bars, solid color.
 *
 * Compatible with Linux kernel 3.10
 */

#define pr_fmt(fmt) "tegra-tpg: " fmt

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <linux/time.h>
#include <linux/slab.h>

#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-of.h>
#include <media/media-device.h>
#include <media/media-entity.h>

/* TPG patterns */
#define TPG_PATTERN_GRADIENT    0
#define TPG_PATTERN_CHECKER   1
#define TPG_PATTERN_COLORBARS 2
#define TPG_PATTERN_SOLID     3
#define TPG_PATTERN_NOISE     4

/* Default configuration */
#define TPG_DEFAULT_WIDTH     3264
#define TPG_DEFAULT_HEIGHT    2448
#define TPG_DEFAULT_FPS       30
#define TPG_DEFAULT_PATTERN   TPG_PATTERN_CHECKER

/* Supported media bus formats */
static const u32 tpg_mbus_formats[] = {
    V4L2_MBUS_FMT_SRGGB10_1X10,
    V4L2_MBUS_FMT_SBGGR10_1X10,
    V4L2_MBUS_FMT_SGBRG10_1X10,
    V4L2_MBUS_FMT_SGRBG10_1X10,
    V4L2_MBUS_FMT_SRGGB8_1X8,
    V4L2_MBUS_FMT_SBGGR8_1X8,
};

#define TPG_NUM_FORMATS ARRAY_SIZE(tpg_mbus_formats)

struct tegra_tpg {
    struct v4l2_subdev sd;
    struct media_pad pad;
    struct v4l2_ctrl_handler ctrls;

    /* Configuration */
    u32 width;
    u32 height;
    u32 pattern;
    u32 fps;
    u32 mbus_code;
    u32 bits_per_sample;

    /* Streaming state */
    bool streaming;
    atomic_t frame_count;

    /* Synchronization */
    struct mutex lock;

    /* Reference to parent v4l2_device */
    struct v4l2_device *v4l2_dev;
};

static inline struct tegra_tpg *to_tegra_tpg(struct v4l2_subdev *sd)
{
    return container_of(sd, struct tegra_tpg, sd);
}

/* Utility: clamp value */
static inline u32 clamp_u32(u32 val, u32 min, u32 max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

/* Generate gradient pattern (10-bit Bayer) */
static void tpg_generate_gradient_10bit(u8 *buf, u32 width, u32 height, u32 frame)
{
    u32 x, y;
    u16 *ptr = (u16 *)buf;

    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            u16 val;
            /* Diagonal gradient with frame offset */
            val = ((x + y + frame * 10) % 1024);
            ptr[y * width + x] = val;
        }
    }
}

/* Generate gradient pattern (8-bit Bayer) */
static void tpg_generate_gradient_8bit(u8 *buf, u32 width, u32 height, u32 frame)
{
    u32 x, y;

    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            /* Diagonal gradient with frame offset */
            buf[y * width + x] = (x + y + frame * 2) & 0xFF;
        }
    }
}

/* Generate checkerboard pattern (10-bit) */
static void tpg_generate_checkerboard_10bit(u8 *buf, u32 width, u32 height, u32 frame)
{
    u32 x, y;
    u16 *ptr = (u16 *)buf;
    u32 block_size = 32;

    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            u32 bx = x / block_size;
            u32 by = y / block_size;
            u16 val;

            if ((bx + by + frame) & 1)
                val = 0x3FF;  /* White */
            else
                val = 0x000;  /* Black */

            ptr[y * width + x] = val;
        }
    }
}

/* Generate checkerboard pattern (8-bit) */
static void tpg_generate_checkerboard_8bit(u8 *buf, u32 width, u32 height, u32 frame)
{
    u32 x, y;
    u32 block_size = 32;

    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            u32 bx = x / block_size;
            u32 by = y / block_size;

            if ((bx + by + frame) & 1)
                buf[y * width + x] = 0xFF;
            else
                buf[y * width + x] = 0x00;
        }
    }
}

/* Generate color bars pattern (10-bit Bayer simulation) */
static void tpg_generate_colorbars_10bit(u8 *buf, u32 width, u32 height, u32 frame)
{
    u32 x, y;
    u16 *ptr = (u16 *)buf;
    u32 bar_width = width / 8;
    u16 bar_values[] = {0, 170, 341, 512, 683, 853, 1000, 1023};
    (void)frame;

    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            u32 bar = x / bar_width;
            bar = clamp_u32(bar, 0, 7);
            ptr[y * width + x] = bar_values[bar];
        }
    }
}

/* Generate color bars pattern (8-bit) */
static void tpg_generate_colorbars_8bit(u8 *buf, u32 width, u32 height, u32 frame)
{
    u32 x, y;
    u32 bar_width = width / 8;
    u8 bar_values[] = {0, 36, 73, 109, 146, 182, 219, 255};
    (void)frame;

    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            u32 bar = x / bar_width;
            bar = clamp_u32(bar, 0, 7);
            buf[y * width + x] = bar_values[bar];
        }
    }
}

/* Generate noise pattern (10-bit) */
static void tpg_generate_noise_10bit(u8 *buf, u32 width, u32 height, u32 frame)
{
    u32 i;
    u16 *ptr = (u16 *)buf;
    u32 total = width * height;
    u32 seed = frame * 2654435761U;

    for (i = 0; i < total; i++) {
        seed = (seed * 1103515245 + 12345) & 0x7FFFFFFF;
        ptr[i] = (seed >> 16) & 0x3FF;
    }
}

/* Generate noise pattern (8-bit) */
static void tpg_generate_noise_8bit(u8 *buf, u32 width, u32 height, u32 frame)
{
    u32 i;
    u32 total = width * height;
    u32 seed = frame * 2654435761U;

    for (i = 0; i < total; i++) {
        seed = (seed * 1103515245 + 12345) & 0x7FFFFFFF;
        buf[i] = (seed >> 16) & 0xFF;
    }
}

/* Generate solid color (gray) */
static void tpg_generate_solid(u8 *buf, u32 width, u32 height, u32 frame)
{
    u32 total = width * height;
    u8 gray = 0x80;
    (void)frame;

    memset(buf, gray, total);
}

/* Main frame generation function */
static void tpg_generate_frame(struct tegra_tpg *tpg, u8 *buf, u32 frame)
{
    switch (tpg->pattern) {
    case TPG_PATTERN_GRADIENT:
        if (tpg->bits_per_sample == 10)
            tpg_generate_gradient_10bit(buf, tpg->width, tpg->height, frame);
        else
            tpg_generate_gradient_8bit(buf, tpg->width, tpg->height, frame);
        break;

    case TPG_PATTERN_CHECKER:
        if (tpg->bits_per_sample == 10)
            tpg_generate_checkerboard_10bit(buf, tpg->width, tpg->height, frame);
        else
            tpg_generate_checkerboard_8bit(buf, tpg->width, tpg->height, frame);
        break;

    case TPG_PATTERN_COLORBARS:
        if (tpg->bits_per_sample == 10)
            tpg_generate_colorbars_10bit(buf, tpg->width, tpg->height, frame);
        else
            tpg_generate_colorbars_8bit(buf, tpg->width, tpg->height, frame);
        break;

    case TPG_PATTERN_NOISE:
        if (tpg->bits_per_sample == 10)
            tpg_generate_noise_10bit(buf, tpg->width, tpg->height, frame);
        else
            tpg_generate_noise_8bit(buf, tpg->width, tpg->height, frame);
        break;

    case TPG_PATTERN_SOLID:
    default:
        tpg_generate_solid(buf, tpg->width, tpg->height, frame);
        break;
    }
}

/* v4l2_subdev core ops */
static int tegra_tpg_s_power(struct v4l2_subdev *sd, int on)
{
    pr_debug("TPG power: %d\n", on);
    return 0;
}

static const struct v4l2_subdev_core_ops tegra_tpg_core_ops = {
    .s_power = tegra_tpg_s_power,
};

/* v4l2_subdev video ops */
static int tegra_tpg_enum_mbus_code(struct v4l2_subdev *sd, unsigned int index,
                                    enum v4l2_mbus_pixelcode *code)
{
    if (index >= TPG_NUM_FORMATS)
        return -EINVAL;

    *code = tpg_mbus_formats[index];
    return 0;
}

static int tegra_tpg_get_fmt(struct v4l2_subdev *sd,
                             struct v4l2_subdev_fh *fh,
                             struct v4l2_subdev_format *fmt)
{
    struct tegra_tpg *tpg = to_tegra_tpg(sd);

    fmt->format.width = tpg->width;
    fmt->format.height = tpg->height;
    fmt->format.code = tpg->mbus_code;
    fmt->format.field = V4L2_FIELD_NONE;
    fmt->format.colorspace = V4L2_COLORSPACE_SRGB;

    return 0;
}

static int tegra_tpg_set_fmt(struct v4l2_subdev *sd,
                             struct v4l2_subdev_fh *fh,
                             struct v4l2_subdev_format *fmt)
{
    struct tegra_tpg *tpg = to_tegra_tpg(sd);
    /* Validate format */
    fmt->format.width = clamp_u32(fmt->format.width, 64, 4096);
    fmt->format.height = clamp_u32(fmt->format.height, 64, 4096);
    fmt->format.field = V4L2_FIELD_NONE;

    /* Find matching mbus code or use default */
    if (fmt->format.code == 0)
        fmt->format.code = V4L2_MBUS_FMT_SRGGB10_1X10;

    /* Update configuration */
    tpg->width = fmt->format.width;
    tpg->height = fmt->format.height;
    tpg->mbus_code = fmt->format.code;

    /* Determine bits per sample */
    switch (fmt->format.code) {
    case V4L2_MBUS_FMT_SRGGB10_1X10:
    case V4L2_MBUS_FMT_SBGGR10_1X10:
    case V4L2_MBUS_FMT_SGBRG10_1X10:
    case V4L2_MBUS_FMT_SGRBG10_1X10:
        tpg->bits_per_sample = 10;
        break;
    default:
        tpg->bits_per_sample = 8;
        break;
    }

    fmt->format.code = tpg->mbus_code;

    return 0;
}

static int tegra_tpg_s_stream(struct v4l2_subdev *sd, int enable)
{
    struct tegra_tpg *tpg = to_tegra_tpg(sd);

    pr_info("TPG stream: %s\n", enable ? "ON" : "OFF");

    if (enable) {
        if (!tpg->streaming) {
            tpg->streaming = true;
            atomic_set(&tpg->frame_count, 0);
        }
    } else {
        tpg->streaming = false;
    }

    return 0;
}

static const struct v4l2_subdev_video_ops tegra_tpg_video_ops = {
    .enum_mbus_fmt = tegra_tpg_enum_mbus_code,
    .s_stream      = tegra_tpg_s_stream,
};

/* v4l2_subdev pad ops */
static int tegra_tpg_get_selection(struct v4l2_subdev *sd,
                                   struct v4l2_subdev_fh *fh,
                                   struct v4l2_subdev_selection *sel)
{
    struct tegra_tpg *tpg = to_tegra_tpg(sd);

    if (sel->target != V4L2_SEL_TGT_CROP)
        return -EINVAL;

    sel->r.left = 0;
    sel->r.top = 0;
    sel->r.width = tpg->width;
    sel->r.height = tpg->height;

    return 0;
}

static const struct v4l2_subdev_pad_ops tegra_tpg_pad_ops = {
    .get_selection = tegra_tpg_get_selection,
    .get_fmt       = tegra_tpg_get_fmt,
    .set_fmt       = tegra_tpg_set_fmt,
};

/* Main subdev operations */
static const struct v4l2_subdev_ops tegra_tpg_ops = {
    .core  = &tegra_tpg_core_ops,
    .video = &tegra_tpg_video_ops,
    .pad   = &tegra_tpg_pad_ops,
};

/* Initialize TPG subdev */
static int tegra_tpg_init(struct tegra_tpg *tpg)
{
    int ret;

    /* Initialize v4l2_subdev */
    v4l2_subdev_init(&tpg->sd, &tegra_tpg_ops);
    strlcpy(tpg->sd.name, "tegra-tpg", sizeof(tpg->sd.name));
    v4l2_set_subdevdata(&tpg->sd, tpg);

    /* Initialize media pad */
    tpg->pad.flags = MEDIA_PAD_FL_SOURCE;
    ret = media_entity_init(&tpg->sd.entity, 1, &tpg->pad, 0);
    if (ret) {
        pr_err("Failed to init media entity: %d\n", ret);
        return ret;
    }

    tpg->sd.entity.type = MEDIA_ENT_TYPE_V4L_SUBDEV;
    tpg->sd.entity.flags = 0;

    /* Initialize controls */
    v4l2_ctrl_handler_init(&tpg->ctrls, 4);
    v4l2_ctrl_new_std(&tpg->ctrls, NULL, V4L2_CID_HFLIP, 0, 1, 1, 0);
    v4l2_ctrl_new_std(&tpg->ctrls, NULL, V4L2_CID_VFLIP, 0, 1, 1, 0);
    tpg->sd.ctrl_handler = &tpg->ctrls;

    if (tpg->ctrls.error) {
        pr_err("TPG control error: %d\n", tpg->ctrls.error);
        v4l2_ctrl_handler_free(&tpg->ctrls);
    }

    /* Default configuration */
    tpg->width = TPG_DEFAULT_WIDTH;
    tpg->height = TPG_DEFAULT_HEIGHT;
    tpg->pattern = TPG_DEFAULT_PATTERN;
    tpg->fps = TPG_DEFAULT_FPS;
    tpg->mbus_code = V4L2_MBUS_FMT_SRGGB10_1X10;
    tpg->bits_per_sample = 10;
    tpg->streaming = false;

    mutex_init(&tpg->lock);
    atomic_set(&tpg->frame_count, 0);

    pr_info("TPG initialized: %dx%d, pattern=%d, fps=%d\n",
            tpg->width, tpg->height, tpg->pattern, tpg->fps);

    return 0;
}

/* Register TPG with v4l2_device */
int tegra_tpg_register(struct v4l2_device *v4l2_dev, struct tegra_tpg **tpg_out)
{
    struct tegra_tpg *tpg;
    int ret;

    tpg = kzalloc(sizeof(*tpg), GFP_KERNEL);
    if (!tpg)
        return -ENOMEM;

    tpg->v4l2_dev = v4l2_dev;

    ret = tegra_tpg_init(tpg);
    if (ret) {
        kfree(tpg);
        return ret;
    }

    /* Register subdev */
    ret = v4l2_device_register_subdev(v4l2_dev, &tpg->sd);
    if (ret) {
        pr_err("Failed to register TPG subdev: %d\n", ret);
        kfree(tpg);
        return ret;
    }

    /* Register entity with media device */
    if (v4l2_dev->mdev) {
        ret = media_device_register_entity(v4l2_dev->mdev, &tpg->sd.entity);
        if (ret)
            pr_warn("Failed to register TPG entity: %d\n", ret);
    }

    *tpg_out = tpg;
    pr_info("TPG registered successfully\n");
    return 0;
}

/* Unregister TPG */
void tegra_tpg_unregister(struct tegra_tpg *tpg)
{
    if (!tpg)
        return;

    /* Unregister subdev */
    if (tpg->sd.v4l2_dev)
        v4l2_device_unregister_subdev(&tpg->sd);

    /* Cleanup media entity */
    media_entity_cleanup(&tpg->sd.entity);

    /* Free resources */
    v4l2_ctrl_handler_free(&tpg->ctrls);
    kfree(tpg);

    pr_info("TPG unregistered\n");
}

/* Fill user-provided buffer with TPG pattern data (for VI fallback) */
int tegra_tpg_fill_buffer(struct tegra_tpg *tpg, u8 *buf,
                          u32 width, u32 height)
{
    u32 orig_w;
    u32 orig_h;

    if (!tpg || !buf || width == 0 || height == 0)
        return -EINVAL;

    /* Set TPG to requested dimensions temporarily */
    mutex_lock(&tpg->lock);
    orig_w = tpg->width;
    orig_h = tpg->height;
    tpg->width = width;
    tpg->height = height;
    mutex_unlock(&tpg->lock);

    tpg_generate_frame(tpg, buf, atomic_read(&tpg->frame_count));

    mutex_lock(&tpg->lock);
    tpg->width = orig_w;
    tpg->height = orig_h;
    mutex_unlock(&tpg->lock);

    atomic_inc(&tpg->frame_count);
    return 0;
}
EXPORT_SYMBOL_GPL(tegra_tpg_fill_buffer);

/* Module parameters */
static int tpg_pattern = TPG_DEFAULT_PATTERN;
module_param(tpg_pattern, int, 0644);
MODULE_PARM_DESC(tpg_pattern, "Test pattern: 0=gradient, 1=checker, 2=colorbars, 3=solid, 4=noise");

static int tpg_width = TPG_DEFAULT_WIDTH;
module_param(tpg_width, int, 0644);
MODULE_PARM_DESC(tpg_width, "TPG width");

static int tpg_height = TPG_DEFAULT_HEIGHT;
module_param(tpg_height, int, 0644);
MODULE_PARM_DESC(tpg_height, "TPG height");

static int tpg_fps = TPG_DEFAULT_FPS;
module_param(tpg_fps, int, 0644);
MODULE_PARM_DESC(tpg_fps, "TPG frames per second");

EXPORT_SYMBOL_GPL(tegra_tpg_register);
EXPORT_SYMBOL_GPL(tegra_tpg_unregister);
