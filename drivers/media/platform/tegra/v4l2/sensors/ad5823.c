/*
 * ad5823.c - AD5823 VCM focuser driver for Tegra V4L2
 *
 * Copyright (c) 2026, Dargons10
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * AD5823 is a Voice Coil Motor (VCM) driver for autofocus.
 * Used with IMX179 rear camera in Xiaomi Mi Pad (mocha).
 *
 * Compatible with Linux kernel 3.10
 */

#define pr_fmt(fmt) "ad5823: " fmt

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include "../tegra_vi.h"
#include <media/v4l2-of.h>

/* AD5823 does not respond on I2C — no AD5823 chip on this hardware.
 * Focus position is stored but not written to HW.
 * GPIO 223 (CAM_AF_PWDN) is managed by IMX179 driver.
 * Regulators are shared with IMX179.
 */

#define AD5823_FOCUS_MIN        0
#define AD5823_FOCUS_MAX        1023
#define AD5823_FOCUS_STEP       1
#define AD5823_FOCUS_DEFAULT    0

/* Focuser state — I2C writes disabled; AD5823 chip not present on HW */
struct ad5823 {
    struct v4l2_subdev sd;
    struct media_pad pad;
    struct v4l2_ctrl_handler ctrl_handler;

    struct regulator *vdd;
    struct regulator *vdd_i2c;

    bool powered;
    u16 current_focus;
};

static inline struct ad5823 *to_ad5823(struct v4l2_subdev *sd)
{
    return container_of(sd, struct ad5823, sd);
}

static int ad5823_set_focus(struct ad5823 *ad5823, u16 position)
{
    ad5823->current_focus = position;
    return 0;
}

static int ad5823_s_power(struct v4l2_subdev *sd, int on)
{
    struct ad5823 *ad5823 = to_ad5823(sd);
    int ret;

    if (on) {
        if (ad5823->powered)
            return 0;

        /* Enable regulators (IMX179 manages GPIO PWDN) */
        if (!IS_ERR(ad5823->vdd)) {
            ret = regulator_enable(ad5823->vdd);
            if (ret)
                return ret;
        }
        if (!IS_ERR_OR_NULL(ad5823->vdd_i2c)) {
            ret = regulator_enable(ad5823->vdd_i2c);
            if (ret) {
                if (!IS_ERR(ad5823->vdd))
                    regulator_disable(ad5823->vdd);
                return ret;
            }
        }

        ad5823->powered = true;
        ad5823_set_focus(ad5823, ad5823->current_focus);
    } else {
        if (!ad5823->powered)
            return 0;

        if (!IS_ERR_OR_NULL(ad5823->vdd_i2c))
            regulator_disable(ad5823->vdd_i2c);
        if (!IS_ERR(ad5823->vdd))
            regulator_disable(ad5823->vdd);

        ad5823->powered = false;
    }

    return 0;
}

static int ad5823_s_ctrl(struct v4l2_ctrl *ctrl)
{
    struct ad5823 *ad5823 = container_of(ctrl->handler,
                                          struct ad5823, ctrl_handler);

    switch (ctrl->id) {
    case V4L2_CID_FOCUS_ABSOLUTE:
        return ad5823_set_focus(ad5823, ctrl->val);
    default:
        return -EINVAL;
    }
}

static const struct v4l2_ctrl_ops ad5823_ctrl_ops = {
    .s_ctrl = ad5823_s_ctrl,
};

static const struct v4l2_subdev_core_ops ad5823_core_ops = {
    .s_power = ad5823_s_power,
};

static const struct v4l2_subdev_ops ad5823_subdev_ops = {
    .core = &ad5823_core_ops,
};

static int ad5823_parse_dt(struct device *dev, struct ad5823 *ad5823)
{

    ad5823->vdd = devm_regulator_get(dev, "vdd");
    if (IS_ERR(ad5823->vdd))
        return PTR_ERR(ad5823->vdd);

    ad5823->vdd_i2c = devm_regulator_get(dev, "vdd-i2c");
    if (IS_ERR(ad5823->vdd_i2c))
        ad5823->vdd_i2c = NULL;

    ad5823->current_focus = 0;

    return 0;
}

static int ad5823_probe(struct i2c_client *client,
                          const struct i2c_device_id *id)
{
    struct ad5823 *ad5823;
    int ret;

    ad5823 = devm_kzalloc(&client->dev, sizeof(*ad5823), GFP_KERNEL);
    if (!ad5823)
        return -ENOMEM;

    ad5823->powered = false;
    ad5823->current_focus = 0;

    ret = ad5823_parse_dt(&client->dev, ad5823);
    if (ret) {
        dev_err(&client->dev, "Failed to parse device tree: %d\n", ret);
        return ret;
    }

    v4l2_i2c_subdev_init(&ad5823->sd, client, &ad5823_subdev_ops);
    ad5823->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
    ad5823->sd.owner = THIS_MODULE;

    ad5823->pad.flags = MEDIA_PAD_FL_SOURCE;
    ret = media_entity_init(&ad5823->sd.entity, 1, &ad5823->pad, 0);
    if (ret)
        goto err_entity;

    ret = v4l2_ctrl_handler_init(&ad5823->ctrl_handler, 1);
    if (ret)
        goto err_entity;

    v4l2_ctrl_new_std(&ad5823->ctrl_handler, &ad5823_ctrl_ops,
                       V4L2_CID_FOCUS_ABSOLUTE,
                       AD5823_FOCUS_MIN, AD5823_FOCUS_MAX,
                       AD5823_FOCUS_STEP, AD5823_FOCUS_DEFAULT);

    if (ad5823->ctrl_handler.error) {
        ret = ad5823->ctrl_handler.error;
        goto err_ctrl_free;
    }

    ad5823->sd.ctrl_handler = &ad5823->ctrl_handler;

    i2c_set_clientdata(client, ad5823);

    /* Notificar al VI driver que el focuser está listo */
    ret = tegra_vi_register_focuser(&ad5823->sd, "adi,ad5823");
    if (ret)
        dev_warn(&client->dev, "tegra_vi_register_focuser failed: %d\n", ret);

    pr_info("AD5823 subdev initialized\n");

    dev_info(&client->dev, "AD5823 focuser probed as v4l2_subdev\n");
    return 0;

err_ctrl_free:
    v4l2_ctrl_handler_free(&ad5823->ctrl_handler);
    media_entity_cleanup(&ad5823->sd.entity);
err_entity:
    return ret;
}

static int ad5823_remove(struct i2c_client *client)
{
    struct ad5823 *ad5823 = i2c_get_clientdata(client);

    v4l2_ctrl_handler_free(&ad5823->ctrl_handler);
    media_entity_cleanup(&ad5823->sd.entity);

    return 0;
}

static const struct i2c_device_id ad5823_id[] = {
    { "ad5823", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, ad5823_id);

static const struct of_device_id ad5823_of_match[] = {
    { .compatible = "adi,ad5823" },
    { .compatible = "nvidia,ad5823-mocha" },
    { }
};
MODULE_DEVICE_TABLE(of, ad5823_of_match);

static struct i2c_driver ad5823_driver = {
    .probe  = ad5823_probe,
    .remove = ad5823_remove,
    .id_table = ad5823_id,
    .driver = {
        .name           = "ad5823",
        .of_match_table = ad5823_of_match,
    },
};
module_i2c_driver(ad5823_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Dargons10 <dargons10@users.noreply.github.com>");
MODULE_DESCRIPTION("ADI AD5823 focuser driver for Mocha (Tegra K1)");
