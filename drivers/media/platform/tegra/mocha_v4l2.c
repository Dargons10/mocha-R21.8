/*
 * mocha_v4l2.c - Mocha V4L2 camera platform stub
 *
 * Copyright (c) 2026, Dargons10
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * Stub driver - actual camera functionality is provided by
 * the V4L2 subdev drivers in v4l2/sensors/
 */

#include <linux/module.h>
#include <linux/platform_device.h>

static int mocha_v4l2_probe(struct platform_device *pdev)
{
    dev_info(&pdev->dev, "Mocha V4L2 camera platform stub loaded\n");
    return 0;
}

static int mocha_v4l2_remove(struct platform_device *pdev)
{
    return 0;
}

static const struct of_device_id mocha_v4l2_of_match[] = {
    { .compatible = "nvidia,mocha-v4l2" },
    { }
};
MODULE_DEVICE_TABLE(of, mocha_v4l2_of_match);

static struct platform_driver mocha_v4l2_driver = {
    .probe  = mocha_v4l2_probe,
    .remove = mocha_v4l2_remove,
    .driver = {
        .name           = "mocha-v4l2",
        .of_match_table = mocha_v4l2_of_match,
    },
};
module_platform_driver(mocha_v4l2_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dargons10");
MODULE_DESCRIPTION("Mocha V4L2 camera platform stub");
