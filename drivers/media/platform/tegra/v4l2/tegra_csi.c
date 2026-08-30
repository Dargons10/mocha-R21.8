/*
 * tegra_csi.c - Tegra CSI receiver driver
 *
 * Copyright (c) 2026, Dargons10
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * Compatible with Linux kernel 3.10
 */

#define pr_fmt(fmt) "tegra-csi: " fmt

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/delay.h>

#include "tegra_csi.h"
#include <media/media-device.h>
#include <media/media-entity.h>

/* CSI is managed internally by tegra_vi.c, not as a standalone device */

/* Initialize Media Controller entities for CSI ports */
static int tegra_csi_media_init(struct tegra_csi *csi)
{
    int i;
    int ret;

    if (!csi->mdev) {
        pr_warn("No media device, skipping CSI media init\n");
        return 0;
    }

    pr_info("Initializing CSI Media Controller\n");

    /* Initialize media entity for each CSI port */
    for (i = 0; i < csi->num_ports; i++) {
        struct tegra_csi_port *port = &csi->ports[i];

        port->entity_name = kasprintf(GFP_KERNEL, "tegra-csi.%d", port->port);
        if (!port->entity_name) {
            pr_err("Failed to allocate entity name for port %d\n", i);
            goto err_rollback;
        }
        port->entity.name = port->entity_name;

        port->entity.type = MEDIA_ENT_TYPE_V4L_SUBDEV;
        port->entity.flags = 0;  /* CSI is passthrough (sink + source pads) */

        /* Pad 0: SINK (input from sensor) */
        port->pads[0].index = 0;
        port->pads[0].flags = MEDIA_PAD_FL_SINK;

        /* Pad 1: SOURCE (output to VI) */
        port->pads[1].index = 1;
        port->pads[1].flags = MEDIA_PAD_FL_SOURCE;

        ret = media_entity_init(&port->entity, 2, port->pads, 0);
        if (ret) {
            pr_err("Failed to init media entity for port %d: %d\n", i, ret);
            kfree(port->entity_name);
            port->entity_name = NULL;
            goto err_rollback;
        }

        ret = media_device_register_entity(csi->mdev, &port->entity);
        if (ret) {
            pr_err("Failed to register entity for port %d: %d\n", i, ret);
            media_entity_cleanup(&port->entity);
            kfree(port->entity_name);
            port->entity_name = NULL;
            goto err_rollback;
        }

        pr_info("CSI port %d: entity registered (pad0=sink, pad1=source)\n",
                 port->port);
    }

    pr_info("CSI Media Controller initialized\n");
    return 0;

err_rollback:
    for (--i; i >= 0; i--) {
        struct tegra_csi_port *p = &csi->ports[i];
        if (p->entity_name) {
            media_entity_cleanup(&p->entity);
            kfree(p->entity_name);
            p->entity_name = NULL;
        }
    }
    return ret;
}

int tegra_csi_set_media_device(struct tegra_csi *csi, struct media_device *mdev)
{
    if (!csi)
        return -EINVAL;

    if (csi->mdev) {
        pr_warn("tegra_csi_set_media_device already called\n");
        return 0;
    }

    csi->mdev = mdev;

    /* Now initialize media entities with the mdev */
    return tegra_csi_media_init(csi);
}

int tegra_csi_init(struct tegra_csi *csi)
{
    int i;
    u32 val;

    if (!csi || !csi->base) {
        pr_err("Invalid CSI device\n");
        return -EINVAL;
    }

    /* Pequeño delay para estabilizar power/clocks antes de tocar registros CSI */
    usleep_range(50000, 100000);

    /* Initialize CSI ports configuration */
    csi->num_ports = 0;

    /* Add CSI ports based on DT */
    /* Port 0: CSI-A (used by IMX179) */
    csi->ports[csi->num_ports].port = 0;
    csi->ports[csi->num_ports].num_lanes = 4;
    csi->ports[csi->num_ports].active = true;
    csi->num_ports++;

    /* Port 4: CSI-E (used by OV5693) */
    csi->ports[csi->num_ports].port = 4;
    csi->ports[csi->num_ports].num_lanes = 1;
    csi->ports[csi->num_ports].active = true;
    csi->num_ports++;

    pr_info("CSI configured with %d ports\n", csi->num_ports);

    /* Enable CSI clocks via CLKEN_OVERRIDE (absolute offset 0xAF4) */
    {
        u32 clock_val = readl(csi->base + CSI_CLKEN_OVERRIDE);
        for (i = 0; i < csi->num_ports; i++)
            clock_val |= BIT(csi->ports[i].port);
        writel(clock_val, csi->base + CSI_CLKEN_OVERRIDE);

        /* Reset CSI PHY for all active ports */
        for (i = 0; i < csi->num_ports; i++) {
            u32 offset = tegra_csi_cil_base(csi->ports[i].port);

            if (!offset)
                continue;

            val = readl(csi->base + offset);
            val &= ~0x1;
            writel(val, csi->base + offset);
        }
        usleep_range(1000, 2000);

        /* Configure PHY for MIPI CSI-2 */
        for (i = 0; i < csi->num_ports; i++) {
            u32 offset = tegra_csi_cil_base(csi->ports[i].port);

            if (!offset)
                continue;

            val = readl(csi->base + offset);
            val |= 0x2;
            writel(val, csi->base + offset);
        }

        pr_info("CSI initialized (%d ports, clock override=0x%x)\n",
                 csi->num_ports, clock_val);
    }
    return 0;
}

/* CSI platform probe is unused - CSI is managed by tegra_vi.c internally */

void tegra_csi_media_cleanup(struct tegra_csi *csi)
{
    int i;

    if (!csi)
        return;

    for (i = 0; i < csi->num_ports; i++) {
        struct tegra_csi_port *p = &csi->ports[i];
        if (p->entity_name) {
            if (csi->mdev)
                media_device_unregister_entity(&p->entity);
            media_entity_cleanup(&p->entity);
            kfree(p->entity_name);
            p->entity_name = NULL;
        }
    }
}
EXPORT_SYMBOL_GPL(tegra_csi_media_cleanup);

EXPORT_SYMBOL_GPL(tegra_csi_init);
EXPORT_SYMBOL_GPL(tegra_csi_set_media_device);
