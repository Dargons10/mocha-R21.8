/*
 * tegra_csi.h - Tegra CSI receiver header
 *
 * Copyright (c) 2026, Dargons10
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * Compatible with Linux kernel 3.10
 *
 * Register map notes (Tegra124 / T124):
 *   VI and CSI share the same physical base at 0x54080000.
 *   VI uses offsets 0x000-0x1FF (config + per-channel capture regs).
 *   CSI uses offsets 0x808+ (capabilities, pixel parsers, CIL PHY blocks).
 *   CIL-A through CIL-E start at offsets 0x92C, 0x960, 0x994, 0x9C8, 0xA08.
 *   CSI-F does NOT exist on T124.
 */

#ifndef __TEGRA_CSI_H__
#define __TEGRA_CSI_H__

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-subdev.h>

#define TEGRA_CSI_DRIVER_NAME  "tegra-csi"
#define TEGRA_CSI_MAX_PORTS    5
#define TEGRA_CSI_MAX_LANES    4

/* CSI base address is shared with VI on T124 */
#define TEGRA_CSI_BASE         0x54080000
#define TEGRA_CSI_SIZE         0x20000

/* ========== CSI CIL base addresses (absolute offsets from 0x54080000) ========== */
/* Each CIL PHY block controls one MIPI CSI-2 port */
#define CSI_CILA_BASE                0x92C
#define CSI_CILB_BASE                0x960
#define CSI_CILC_BASE                0x994
#define CSI_CILD_BASE                0x9C8
#define CSI_CILE_BASE                0xA08

/* Per-CIL pad control registers (same as CIL base on T124) */
#define CSI_CILA_PAD_CONTROL_0       0x92C
#define CSI_CILB_PAD_CONTROL_0       0x960
#define CSI_CILC_PAD_CONTROL_0       0x994
#define CSI_CILD_PAD_CONTROL_0       0x9C8
#define CSI_CILE_PAD_CONTROL_0       0xA08

/* ========== Registers within each CIL block (offset from CIL base) ========== */
#define CSI_CIL_PHY_CONTROL          0x00   /* PHY configuration / pad config */
#define CSI_CIL_INTERRUPT_MASK       0x04   /* Interrupt mask */
#define CSI_CIL_STATUS_REG           0x08   /* Status register */
#define CSI_CIL_STATUS_ENABLE        0x0C   /* Status enable */
#define CSI_CIL_CONTROL              0x10   /* Clock / pixel control */
#define CSI_CIL_PORT_CONTROL         0x00   /* Alias for PHY_CONTROL (bit0=enable) */

/* ========== Shared CSI registers (absolute offsets from 0x54080000) ========== */
#define CSI_CLKEN_OVERRIDE           0xAF4  /* CSI clock enable override */

/* CIL base lookup: port number -> absolute offset */
static inline u32 tegra_csi_cil_base(int port)
{
    static const u32 cil_bases[] = {
        CSI_CILA_BASE,  /* port 0 = CSI-A */
        CSI_CILB_BASE,  /* port 1 = CSI-B */
        CSI_CILC_BASE,  /* port 2 = CSI-C */
        CSI_CILD_BASE,  /* port 3 = CSI-D */
        CSI_CILE_BASE,  /* port 4 = CSI-E */
    };
    if (port < 0 || port >= ARRAY_SIZE(cil_bases))
        return 0;
    return cil_bases[port];
}

/* CSI port configuration - with Media Controller support */
struct tegra_csi_port {
    int port;
    int num_lanes;
    bool active;

    /* Media Controller */
    struct media_entity entity;
    struct media_pad pads[2];  /* 0=sink (from sensor), 1=source (to VI) */
    char *entity_name;         /* kasprintf'd name, freed on cleanup */
};

struct tegra_csi {
    void __iomem *base;

    struct tegra_csi_port ports[TEGRA_CSI_MAX_PORTS];
    int num_ports;

    /* Media Controller reference */
    struct media_device *mdev;
};

/* Function prototypes */
int tegra_csi_init(struct tegra_csi *csi);
int tegra_csi_set_media_device(struct tegra_csi *csi, struct media_device *mdev);
void tegra_csi_media_cleanup(struct tegra_csi *csi);

#endif /* __TEGRA_CSI_H__ */
