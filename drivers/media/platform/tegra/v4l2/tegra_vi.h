/*
 * tegra_vi.h - Tegra Video Input (VI) V4L2 driver header
 *
 * Copyright (c) 2026, Dargons10
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#ifndef __TEGRA_VI_H__
#define __TEGRA_VI_H__

#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-vmalloc.h>
#include <media/v4l2-device.h>
#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-subdev.h>

#define TEGRA_VI_DRIVER_NAME    "tegra-vi"
#define TEGRA_VI_MAX_CHANNELS   5
#define TEGRA_VI_MAX_BUFFERS    4

/* VI + CSI share the same physical base on T124 */
#define TEGRA_VI_BASE         0x54080000
#define TEGRA_VI_SIZE         0x10000

/* ========== VI CFG registers (0x000-0x0F8) ========== */
#define TEGRA_VI_CFG_VI_INCR_SYNCPT            0x000
#define TEGRA_VI_CFG_VI_INCR_SYNCPT_CNTRL      0x004
#define TEGRA_VI_CFG_VI_INCR_SYNCPT_ERROR      0x008
#define TEGRA_VI_CFG_CTXSW                     0x020
#define TEGRA_VI_CFG_INTSTATUS                 0x024
#define TEGRA_VI_CFG_PWM_CONTROL               0x038
#define TEGRA_VI_CFG_PWM_HIGH_PULSE            0x03c
#define TEGRA_VI_CFG_PWM_LOW_PULSE             0x040
#define TEGRA_VI_CFG_PWM_SELECT_PULSE_A        0x044
#define TEGRA_VI_CFG_PWM_SELECT_PULSE_B        0x048
#define TEGRA_VI_CFG_PWM_SELECT_PULSE_C        0x04c
#define TEGRA_VI_CFG_PWM_SELECT_PULSE_D        0x050
#define TEGRA_VI_CFG_VGP1                      0x064
#define TEGRA_VI_CFG_VGP2                      0x068
#define TEGRA_VI_CFG_VGP3                      0x06c
#define TEGRA_VI_CFG_VGP4                      0x070
#define TEGRA_VI_CFG_VGP5                      0x074
#define TEGRA_VI_CFG_VGP6                      0x078
#define TEGRA_VI_CFG_INTERRUPT_MASK            0x08c
#define TEGRA_VI_CFG_INTERRUPT_TYPE_SELECT     0x090
#define TEGRA_VI_CFG_INTERRUPT_POLARITY_SELECT 0x094
#define TEGRA_VI_CFG_INTERRUPT_STATUS          0x098
#define TEGRA_VI_CFG_VGP_SYNCPT_CONFIG         0x0ac
#define TEGRA_VI_CFG_VI_SW_RESET               0x0b4
#define TEGRA_VI_CFG_CG_CTRL                   0x0b8
#define TEGRA_VI_CFG_VI_MCCIF_FIFOCTRL         0x0e4
#define TEGRA_VI_CFG_TIMEOUT_WCOAL_VI          0x0e8
#define TEGRA_VI_CFG_DVFS                      0x0f0
#define TEGRA_VI_CFG_RESERVE                   0x0f4
#define TEGRA_VI_CFG_RESERVE_1                 0x0f8

/* ========== VI CSI capture registers (per-channel, base 0x100 + ch*0x100) ========== */
#define VI_CSI_CH_BASE(ch)                  (0x100 + (ch) * 0x100)
#define TEGRA_VI_CSI_CH_SW_RESET(ch)        (VI_CSI_CH_BASE(ch) + 0x00)
#define TEGRA_VI_CSI_CH_SINGLE_SHOT(ch)     (VI_CSI_CH_BASE(ch) + 0x04)
#define TEGRA_VI_CSI_CH_SINGLE_SHOT_STATE_UPDATE(ch) (VI_CSI_CH_BASE(ch) + 0x08)
#define TEGRA_VI_CSI_CH_IMAGE_DEF(ch)       (VI_CSI_CH_BASE(ch) + 0x0c)
#define TEGRA_VI_CSI_CH_RGB2Y_CTRL(ch)      (VI_CSI_CH_BASE(ch) + 0x10)
#define TEGRA_VI_CSI_CH_MEM_TILING(ch)      (VI_CSI_CH_BASE(ch) + 0x14)
#define TEGRA_VI_CSI_CH_CSI_IMAGE_SIZE(ch)  (VI_CSI_CH_BASE(ch) + 0x18)
#define TEGRA_VI_CSI_CH_CSI_IMAGE_SIZE_WC(ch) (VI_CSI_CH_BASE(ch) + 0x1c)
#define TEGRA_VI_CSI_CH_CSI_IMAGE_DT(ch)    (VI_CSI_CH_BASE(ch) + 0x20)
#define TEGRA_VI_CSI_CH_SURFACE0_OFFSET_MSB(ch)  (VI_CSI_CH_BASE(ch) + 0x24)
#define TEGRA_VI_CSI_CH_SURFACE0_OFFSET_LSB(ch)  (VI_CSI_CH_BASE(ch) + 0x28)
#define TEGRA_VI_CSI_CH_SURFACE1_OFFSET_MSB(ch)  (VI_CSI_CH_BASE(ch) + 0x2c)
#define TEGRA_VI_CSI_CH_SURFACE1_OFFSET_LSB(ch)  (VI_CSI_CH_BASE(ch) + 0x30)
#define TEGRA_VI_CSI_CH_SURFACE2_OFFSET_MSB(ch)  (VI_CSI_CH_BASE(ch) + 0x34)
#define TEGRA_VI_CSI_CH_SURFACE2_OFFSET_LSB(ch)  (VI_CSI_CH_BASE(ch) + 0x38)
#define TEGRA_VI_CSI_CH_SURFACE0_BF_OFFSET_MSB(ch) (VI_CSI_CH_BASE(ch) + 0x3c)
#define TEGRA_VI_CSI_CH_SURFACE0_BF_OFFSET_LSB(ch) (VI_CSI_CH_BASE(ch) + 0x40)
#define TEGRA_VI_CSI_CH_SURFACE1_BF_OFFSET_MSB(ch) (VI_CSI_CH_BASE(ch) + 0x44)
#define TEGRA_VI_CSI_CH_SURFACE1_BF_OFFSET_LSB(ch) (VI_CSI_CH_BASE(ch) + 0x48)
#define TEGRA_VI_CSI_CH_SURFACE2_BF_OFFSET_MSB(ch) (VI_CSI_CH_BASE(ch) + 0x4c)
#define TEGRA_VI_CSI_CH_SURFACE2_BF_OFFSET_LSB(ch) (VI_CSI_CH_BASE(ch) + 0x50)
#define TEGRA_VI_CSI_CH_SURFACE0_STRIDE(ch)       (VI_CSI_CH_BASE(ch) + 0x54)
#define TEGRA_VI_CSI_CH_SURFACE1_STRIDE(ch)       (VI_CSI_CH_BASE(ch) + 0x58)
#define TEGRA_VI_CSI_CH_SURFACE2_STRIDE(ch)       (VI_CSI_CH_BASE(ch) + 0x5c)
#define TEGRA_VI_CSI_CH_SURFACE_HEIGHT0(ch)       (VI_CSI_CH_BASE(ch) + 0x60)
#define TEGRA_VI_CSI_CH_ISPINTF_CONFIG(ch)        (VI_CSI_CH_BASE(ch) + 0x64)
#define TEGRA_VI_CSI_CH_ERROR_STATUS(ch)          (VI_CSI_CH_BASE(ch) + 0x84)
#define TEGRA_VI_CSI_CH_ERROR_INT_MASK(ch)        (VI_CSI_CH_BASE(ch) + 0x88)
#define TEGRA_VI_CSI_CH_WD_CTRL(ch)               (VI_CSI_CH_BASE(ch) + 0x8c)
#define TEGRA_VI_CSI_CH_WD_PERIOD(ch)             (VI_CSI_CH_BASE(ch) + 0x90)

/* Convenience aliases for channel 0 and 1 (as used by vi2.c) */
#define TEGRA_VI_CSI_0_SW_RESET            0x100
#define TEGRA_VI_CSI_0_SINGLE_SHOT         0x104
#define TEGRA_VI_CSI_0_SINGLE_SHOT_STATE_UPDATE   0x108
#define TEGRA_VI_CSI_0_IMAGE_DEF           0x10c
#define TEGRA_VI_CSI_0_RGB2Y_CTRL          0x110
#define TEGRA_VI_CSI_0_MEM_TILING          0x114
#define TEGRA_VI_CSI_0_CSI_IMAGE_SIZE      0x118
#define TEGRA_VI_CSI_0_CSI_IMAGE_SIZE_WC   0x11c
#define TEGRA_VI_CSI_0_CSI_IMAGE_DT        0x120
#define TEGRA_VI_CSI_0_SURFACE0_OFFSET_MSB 0x124
#define TEGRA_VI_CSI_0_SURFACE0_OFFSET_LSB 0x128
#define TEGRA_VI_CSI_0_SURFACE1_OFFSET_MSB 0x12c
#define TEGRA_VI_CSI_0_SURFACE1_OFFSET_LSB 0x130
#define TEGRA_VI_CSI_0_SURFACE2_OFFSET_MSB 0x134
#define TEGRA_VI_CSI_0_SURFACE2_OFFSET_LSB 0x138
#define TEGRA_VI_CSI_0_SURFACE0_BF_OFFSET_MSB 0x13c
#define TEGRA_VI_CSI_0_SURFACE0_BF_OFFSET_LSB 0x140
#define TEGRA_VI_CSI_0_SURFACE1_BF_OFFSET_MSB 0x144
#define TEGRA_VI_CSI_0_SURFACE1_BF_OFFSET_LSB 0x148
#define TEGRA_VI_CSI_0_SURFACE2_BF_OFFSET_MSB 0x14c
#define TEGRA_VI_CSI_0_SURFACE2_BF_OFFSET_LSB 0x150
#define TEGRA_VI_CSI_0_SURFACE0_STRIDE     0x154
#define TEGRA_VI_CSI_0_SURFACE1_STRIDE     0x158
#define TEGRA_VI_CSI_0_SURFACE2_STRIDE     0x15c
#define TEGRA_VI_CSI_0_SURFACE_HEIGHT0     0x160
#define TEGRA_VI_CSI_0_ISPINTF_CONFIG      0x164
#define TEGRA_VI_CSI_0_ERROR_STATUS        0x184
#define TEGRA_VI_CSI_0_ERROR_INT_MASK      0x188
#define TEGRA_VI_CSI_0_WD_CTRL             0x18c
#define TEGRA_VI_CSI_0_WD_PERIOD           0x190

#define TEGRA_VI_CSI_1_SW_RESET            0x200
#define TEGRA_VI_CSI_1_SINGLE_SHOT         0x204
#define TEGRA_VI_CSI_1_SINGLE_SHOT_STATE_UPDATE   0x208
#define TEGRA_VI_CSI_1_IMAGE_DEF           0x20c
#define TEGRA_VI_CSI_1_RGB2Y_CTRL          0x210
#define TEGRA_VI_CSI_1_MEM_TILING          0x214
#define TEGRA_VI_CSI_1_CSI_IMAGE_SIZE      0x218
#define TEGRA_VI_CSI_1_CSI_IMAGE_SIZE_WC   0x21c
#define TEGRA_VI_CSI_1_CSI_IMAGE_DT        0x220
#define TEGRA_VI_CSI_1_SURFACE0_OFFSET_MSB 0x224
#define TEGRA_VI_CSI_1_SURFACE0_OFFSET_LSB 0x228
#define TEGRA_VI_CSI_1_SURFACE1_OFFSET_MSB 0x22c
#define TEGRA_VI_CSI_1_SURFACE1_OFFSET_LSB 0x230
#define TEGRA_VI_CSI_1_SURFACE2_OFFSET_MSB 0x234
#define TEGRA_VI_CSI_1_SURFACE2_OFFSET_LSB 0x238
#define TEGRA_VI_CSI_1_SURFACE0_BF_OFFSET_MSB 0x23c
#define TEGRA_VI_CSI_1_SURFACE0_BF_OFFSET_LSB 0x240
#define TEGRA_VI_CSI_1_SURFACE1_BF_OFFSET_MSB 0x244
#define TEGRA_VI_CSI_1_SURFACE1_BF_OFFSET_LSB 0x248
#define TEGRA_VI_CSI_1_SURFACE2_BF_OFFSET_MSB 0x24c
#define TEGRA_VI_CSI_1_SURFACE2_BF_OFFSET_LSB 0x250
#define TEGRA_VI_CSI_1_SURFACE0_STRIDE     0x254
#define TEGRA_VI_CSI_1_SURFACE1_STRIDE     0x258
#define TEGRA_VI_CSI_1_SURFACE2_STRIDE     0x25c
#define TEGRA_VI_CSI_1_SURFACE_HEIGHT0     0x260
#define TEGRA_VI_CSI_1_ISPINTF_CONFIG      0x264
#define TEGRA_VI_CSI_1_ERROR_STATUS        0x284
#define TEGRA_VI_CSI_1_ERROR_INT_MASK      0x288
#define TEGRA_VI_CSI_1_WD_CTRL             0x28c
#define TEGRA_VI_CSI_1_WD_PERIOD           0x290

/* ========== CSI pixel parser / stream registers ========== */
#define TEGRA_CSI_INPUT_STREAM_A_CONTROL         0x838
#define TEGRA_CSI_PIXEL_STREAM_A_CONTROL0        0x83c
#define TEGRA_CSI_PIXEL_STREAM_A_CONTROL1        0x840
#define TEGRA_CSI_PIXEL_STREAM_A_GAP             0x844
#define TEGRA_CSI_PIXEL_STREAM_PPA_COMMAND       0x848
#define TEGRA_CSI_PIXEL_STREAM_A_EXPECTED_FRAME  0x84c
#define TEGRA_CSI_CSI_PIXEL_PARSER_A_INTERRUPT_MASK 0x850
#define TEGRA_CSI_CSI_PIXEL_PARSER_A_STATUS      0x854
#define TEGRA_CSI_CSI_SW_SENSOR_A_RESET          0x858

#define TEGRA_CSI_INPUT_STREAM_B_CONTROL         0x86c
#define TEGRA_CSI_PIXEL_STREAM_B_CONTROL0        0x870
#define TEGRA_CSI_PIXEL_STREAM_B_CONTROL1        0x874
#define TEGRA_CSI_PIXEL_STREAM_B_GAP             0x878
#define TEGRA_CSI_PIXEL_STREAM_PPB_COMMAND       0x87c
#define TEGRA_CSI_PIXEL_STREAM_B_EXPECTED_FRAME  0x880
#define TEGRA_CSI_CSI_PIXEL_PARSER_B_INTERRUPT_MASK 0x884
#define TEGRA_CSI_CSI_PIXEL_PARSER_B_STATUS      0x888
#define TEGRA_CSI_CSI_SW_SENSOR_B_RESET          0x88c

/* ========== CSI PHY / CIL shared registers ========== */
#define TEGRA_CSI_PHY_CIL_COMMAND                0x908
#define TEGRA_CSI_CIL_PAD_CONFIG0                0x90c

/* ========== CIL-A registers (port 0, CSI-A, IMX179 uses lanes from this) ========== */
#define TEGRA_CSI_CILA_PAD_CONFIG0               0x92c
#define TEGRA_CSI_CILA_PAD_CONFIG1               0x930
#define TEGRA_CSI_PHY_CILA_CONTROL0              0x934
#define TEGRA_CSI_CSI_CIL_A_INTERRUPT_MASK       0x938
#define TEGRA_CSI_CSI_CIL_A_STATUS               0x93c
#define TEGRA_CSI_CSI_CILA_STATUS                0x940
#define TEGRA_CSI_CIL_A_ESCAPE_MODE_COMMAND      0x944
#define TEGRA_CSI_CIL_A_ESCAPE_MODE_DATA         0x948
#define TEGRA_CSI_CSICIL_SW_SENSOR_A_RESET       0x94c

/* ========== CIL-B registers ========== */
#define TEGRA_CSI_CILB_PAD_CONFIG0               0x960
#define TEGRA_CSI_CILB_PAD_CONFIG1               0x964
#define TEGRA_CSI_PHY_CILB_CONTROL0              0x968
#define TEGRA_CSI_CSI_CIL_B_INTERRUPT_MASK       0x96c
#define TEGRA_CSI_CSI_CIL_B_STATUS               0x970
#define TEGRA_CSI_CSI_CILB_STATUS                0x974
#define TEGRA_CSI_CIL_B_ESCAPE_MODE_COMMAND      0x978
#define TEGRA_CSI_CIL_B_ESCAPE_MODE_DATA         0x97c
#define TEGRA_CSI_CSICIL_SW_SENSOR_B_RESET       0x980

/* ========== CIL-C registers ========== */
#define TEGRA_CSI_CILC_PAD_CONFIG0               0x994
#define TEGRA_CSI_CILC_PAD_CONFIG1               0x998
#define TEGRA_CSI_PHY_CILC_CONTROL0              0x99c
#define TEGRA_CSI_CSI_CIL_C_INTERRUPT_MASK       0x9a0
#define TEGRA_CSI_CSI_CIL_C_STATUS               0x9a4
#define TEGRA_CSI_CSI_CILC_STATUS                0x9a8
#define TEGRA_CSI_CIL_C_ESCAPE_MODE_COMMAND      0x9ac
#define TEGRA_CSI_CIL_C_ESCAPE_MODE_DATA         0x9b0
#define TEGRA_CSI_CSICIL_SW_SENSOR_C_RESET       0x9b4

/* ========== CIL-D registers ========== */
#define TEGRA_CSI_CILD_PAD_CONFIG0               0x9c8
#define TEGRA_CSI_CILD_PAD_CONFIG1               0x9cc
#define TEGRA_CSI_PHY_CILD_CONTROL0              0x9d0
#define TEGRA_CSI_CSI_CIL_D_INTERRUPT_MASK       0x9d4
#define TEGRA_CSI_CSI_CIL_D_STATUS               0x9d8
#define TEGRA_CSI_CSI_CILD_STATUS                0x9dc
#define TEGRA_CSI_CIL_D_ESCAPE_MODE_COMMAND      0x9ec
#define TEGRA_CSI_CIL_D_ESCAPE_MODE_DATA         0x9f0
#define TEGRA_CSI_CSICIL_SW_SENSOR_D_RESET       0x9f4

/* ========== CIL-E registers (port 4, CSI-E, OV5693 uses this) ========== */
#define TEGRA_CSI_CILE_PAD_CONFIG0               0xa08
#define TEGRA_CSI_CILE_PAD_CONFIG1               0xa0c
#define TEGRA_CSI_PHY_CILE_CONTROL0              0xa10
#define TEGRA_CSI_CSI_CIL_E_INTERRUPT_MASK       0xa14
#define TEGRA_CSI_CSI_CIL_E_STATUS               0xa18
#define TEGRA_CSI_CIL_E_ESCAPE_MODE_COMMAND      0xa1c
#define TEGRA_CSI_CIL_E_ESCAPE_MODE_DATA         0xa20
#define TEGRA_CSI_CSICIL_SW_SENSOR_E_RESET       0xa24

/* ========== CSI TPG / pattern generator registers ========== */
#define TEGRA_CSI_PATTERN_GENERATOR_CTRL_A       0xa68
#define TEGRA_CSI_PG_BLANK_A                     0xa6c
#define TEGRA_CSI_PG_PHASE_A                     0xa70
#define TEGRA_CSI_PG_RED_FREQ_A                  0xa74
#define TEGRA_CSI_PG_RED_FREQ_RATE_A             0xa78
#define TEGRA_CSI_PG_GREEN_FREQ_A                0xa7c
#define TEGRA_CSI_PG_GREEN_FREQ_RATE_A           0xa80
#define TEGRA_CSI_PG_BLUE_FREQ_A                 0xa84
#define TEGRA_CSI_PG_BLUE_FREQ_RATE_A            0xa88

#define TEGRA_CSI_PATTERN_GENERATOR_CTRL_B       0xa9c
#define TEGRA_CSI_PG_BLANK_B                     0xaa0
#define TEGRA_CSI_PG_PHASE_B                     0xaa4
#define TEGRA_CSI_PG_RED_FREQ_B                  0xaa8
#define TEGRA_CSI_PG_RED_FREQ_RATE_B             0xaac
#define TEGRA_CSI_PG_GREEN_FREQ_B                0xab0
#define TEGRA_CSI_PG_GREEN_FREQ_RATE_B           0xab4
#define TEGRA_CSI_PG_BLUE_FREQ_B                 0xab8
#define TEGRA_CSI_PG_BLUE_FREQ_RATE_B            0xabc

#define TEGRA_CSI_DPCM_CTRL_A                    0xad0
#define TEGRA_CSI_DPCM_CTRL_B                    0xad4
#define TEGRA_CSI_STALL_COUNTER                  0xae8
#define TEGRA_CSI_CSI_READONLY_STATUS            0xaec
#define TEGRA_CSI_CSI_SW_STATUS_RESET            0xaf0
#define TEGRA_CSI_CLKEN_OVERRIDE                 0xaf4
#define TEGRA_CSI_DEBUG_CONTROL                  0xaf8
#define TEGRA_CSI_DEBUG_COUNTER_0                0xafc
#define TEGRA_CSI_DEBUG_COUNTER_1                0xb00
#define TEGRA_CSI_DEBUG_COUNTER_2                0xb04

/* ========== TEGRA_IMAGE_FORMAT_T_* (IMAGE_DEF[23:16]) ========== */
#define TEGRA_IMAGE_FORMAT_T_L8                  16
#define TEGRA_IMAGE_FORMAT_T_R16_I               32
#define TEGRA_IMAGE_FORMAT_T_B5G6R5              33
#define TEGRA_IMAGE_FORMAT_T_R5G6B5              34
#define TEGRA_IMAGE_FORMAT_T_A1B5G5R5            35
#define TEGRA_IMAGE_FORMAT_T_A1R5G5B5            36
#define TEGRA_IMAGE_FORMAT_T_B5G5R5A1            37
#define TEGRA_IMAGE_FORMAT_T_R5G5B5A1            38
#define TEGRA_IMAGE_FORMAT_T_A4B4G4R4            39
#define TEGRA_IMAGE_FORMAT_T_A4R4G4B4            40
#define TEGRA_IMAGE_FORMAT_T_B4G4R4A4            41
#define TEGRA_IMAGE_FORMAT_T_R4G4B4A4            42
#define TEGRA_IMAGE_FORMAT_T_A8B8G8R8            64
#define TEGRA_IMAGE_FORMAT_T_A8R8G8B8            65
#define TEGRA_IMAGE_FORMAT_T_B8G8R8A8            66
#define TEGRA_IMAGE_FORMAT_T_R8G8B8A8            67
#define TEGRA_IMAGE_FORMAT_T_A2B10G10R10         68
#define TEGRA_IMAGE_FORMAT_T_A2R10G10B10         69
#define TEGRA_IMAGE_FORMAT_T_B10G10R10A2         70
#define TEGRA_IMAGE_FORMAT_T_R10G10B10A2         71
#define TEGRA_IMAGE_FORMAT_T_A8Y8U8V8            193
#define TEGRA_IMAGE_FORMAT_T_V8U8Y8A8            194
#define TEGRA_IMAGE_FORMAT_T_A2Y10U10V10         197
#define TEGRA_IMAGE_FORMAT_T_V10U10Y10A2         198
#define TEGRA_IMAGE_FORMAT_T_Y8_U8__Y8_V8        200
#define TEGRA_IMAGE_FORMAT_T_Y8_V8__Y8_U8        201
#define TEGRA_IMAGE_FORMAT_T_U8_Y8__V8_Y8        202
#define TEGRA_IMAGE_FORMAT_T_V8_Y8__U8_Y8        203
#define TEGRA_IMAGE_FORMAT_T_Y8__U8__V8_N444     224
#define TEGRA_IMAGE_FORMAT_T_Y8__U8V8_N444       225
#define TEGRA_IMAGE_FORMAT_T_Y8__V8U8_N444       226
#define TEGRA_IMAGE_FORMAT_T_Y8__U8__V8_N422     227
#define TEGRA_IMAGE_FORMAT_T_Y8__U8V8_N422       228
#define TEGRA_IMAGE_FORMAT_T_Y8__V8U8_N422       229
#define TEGRA_IMAGE_FORMAT_T_Y8__U8__V8_N420     230
#define TEGRA_IMAGE_FORMAT_T_Y8__U8V8_N420       231
#define TEGRA_IMAGE_FORMAT_T_Y8__V8U8_N420       232

/* ========== TEGRA_IMAGE_DT_* (CSI_IMAGE_DT[7:0]) ========== */
#define TEGRA_IMAGE_DT_YUV420_8                 24
#define TEGRA_IMAGE_DT_YUV420_10                25
#define TEGRA_IMAGE_DT_YUV420CSPS_8             28
#define TEGRA_IMAGE_DT_YUV420CSPS_10            29
#define TEGRA_IMAGE_DT_YUV422_8                 30
#define TEGRA_IMAGE_DT_YUV422_10                31
#define TEGRA_IMAGE_DT_RGB444                   32
#define TEGRA_IMAGE_DT_RGB555                   33
#define TEGRA_IMAGE_DT_RGB565                   34
#define TEGRA_IMAGE_DT_RGB666                   35
#define TEGRA_IMAGE_DT_RGB888                   36
#define TEGRA_IMAGE_DT_RAW6                     40
#define TEGRA_IMAGE_DT_RAW7                     41
#define TEGRA_IMAGE_DT_RAW8                     42
#define TEGRA_IMAGE_DT_RAW10                    43
#define TEGRA_IMAGE_DT_RAW12                    44
#define TEGRA_IMAGE_DT_RAW14                    45

/* ========== MIPI Calibration registers (at 0x700E3000, not in VI/CSI space) ========== */
/* Tomados de vi2.c del kernel stock — necesarios para que el pixel parser funcione */
#define MIPI_CAL_CTRL               0x00
#define     STARTCAL                (1 << 0)
#define     CLKEN_OVR               (1 << 4)
#define MIPI_CAL_AUTOCAL_CTRL0      0x04
#define CIL_MIPI_CAL_STATUS         0x08
#define     CAL_DONE                (1 << 16)
#define CILA_MIPI_CAL_CONFIG        0x14
#define     SELA                    (1 << 21)
#define CILB_MIPI_CAL_CONFIG        0x18
#define     SELB                    (1 << 21)
#define CILC_MIPI_CAL_CONFIG        0x1c
#define     SELC                    (1 << 21)
#define CILD_MIPI_CAL_CONFIG        0x20
#define     SELD                    (1 << 21)
#define CILE_MIPI_CAL_CONFIG        0x24
#define     SELE                    (1 << 21)
#define DSIA_MIPI_CAL_CONFIG        0x38
#define     SELDSIA                 (1 << 21)
#define DSIB_MIPI_CAL_CONFIG        0x3c
#define     SELDSIB                 (1 << 21)
#define MIPI_BIAS_PAD_CFG0          0x58
#define     E_VCLAMP_REF            (1 << 0)
#define MIPI_BIAS_PAD_CFG2          0x60
#define     PDVREG                  (1 << 1)
#define DSIA_MIPI_CAL_CONFIG_2      0x64
#define     CLKSELDSIA              (1 << 21)
#define DSIB_MIPI_CAL_CONFIG_2      0x68
#define     CLKSELDSIB              (1 << 21)
#define CILC_MIPI_CAL_CONFIG_2      0x6c
#define     CLKSELC                 (1 << 21)
#define CILD_MIPI_CAL_CONFIG_2      0x70
#define     CLKSELD                 (1 << 21)
#define CSIE_MIPI_CAL_CONFIG_2      0x74
#define     CLKSELE                 (1 << 21)

#define MIPI_CAL_BASE               0x700E3000

/* Frame start/end flags for interrupt */
#define VI_CSI_FRAME_START            (1 << 0)
#define VI_CSI_FRAME_END              (1 << 1)
#define VI_CSI_EOF                    (1 << 2)

enum tegra_vi_buffer_state {
    TEGRA_VI_BUF_STATE_UNUSED,
    TEGRA_VI_BUF_STATE_QUEUED,
    TEGRA_VI_BUF_STATE_ACTIVE,
    TEGRA_VI_BUF_STATE_DONE,
    TEGRA_VI_BUF_STATE_ERROR,
};

struct tegra_vi_buffer {
    struct vb2_buffer vb;
    struct list_head list;
    enum tegra_vi_buffer_state state;
    u32 sequence;
};

struct tegra_csi;
struct tegra_tpg;
struct v4l2_subdev;

struct tegra_vi;

struct tegra_vi_channel {
    struct video_device *vdev;
    struct v4l2_subdev *sensor_sd;
    u32 width;
    u32 height;
    u32 format;

    /* CSI/port config */
    int csi_port;
    int num_lanes;

    /* DMA bounce buffer for VI HW capture */
    void *bounce_buf_cpu;
    dma_addr_t bounce_buf_dma;
    size_t bounce_size;

    /* nvhost syncpt for VI HW capture */
    u32 syncpt_id;
    u32 syncpt_thresh;
    bool syncpt_initialized;

    /* Sensor streaming state */
    bool sensor_powered;
    bool sensor_streaming;
    bool csi_configured;
    bool cal_done;     /* MIPI calibration already performed */
    bool sensor_dead;  /* Sensor has permanently failed - skip to TPG */

    /* Focuser (VCM) for autofocus */
    struct v4l2_subdev *focuser_sd;
};

/* Async sensor tracking - kernel 3.10 no tiene v4l2_async_notifier */
struct tegra_vi_sensor_entry {
    struct list_head list;
    char compatible[64];
    struct device_node *node;
    struct v4l2_subdev *sd;
    int channel;
    bool found;
    bool is_focuser;
};

struct tegra_vi {
    struct device *dev;
    void __iomem *base;

    struct v4l2_device v4l2_dev;

    /* Media Controller - NUEVO */
    struct media_device mdev;
    struct media_entity entity;
    struct media_pad pads[TEGRA_VI_MAX_CHANNELS];
    bool media_registered;
    bool entity_registered;

    struct vb2_queue queue;
    struct mutex lock;
    spinlock_t slock;

    struct list_head buf_queue;

    int num_channels;

    struct tegra_csi *csi;

    /* Clock pointers for pipeline (stored for disable) */
    struct clk *clk_vi;
    struct clk *clk_csi;
    struct clk *clk_csus;
    struct clk *clk_cilab;
    struct clk *clk_cile;

    /* Per-channel */
    struct tegra_vi_channel channels[TEGRA_VI_MAX_CHANNELS];

    /* Test Pattern Generator - fallback */
    struct tegra_tpg *tpg;

    bool streaming;
    int sequence;
    int active_channel;

    /* Capture thread */
    struct task_struct *capture_thread;
    wait_queue_head_t capture_wait;
    bool thread_should_stop;
    int frame_count;
    int frame_errors;

    /* Async sensor discovery (custom, kernel 3.10 sin v4l2_async) */
    struct list_head sensor_pending_list;
    int num_expected_sensors;
    int num_found_sensors;
    struct delayed_work sensor_scan_work;
    int sensor_scan_retries;
    bool sensor_scan_done;
};

/* Funciones para registro async de sensores */
int tegra_vi_register_sensor(struct v4l2_subdev *sd, const char *compatible);
int tegra_vi_register_focuser(struct v4l2_subdev *sd, const char *compatible);

/* VI_INCR_SYNCPT event conditions for nvhost syncpts */
#define VI_CSI_PPA_FRAME_START          (9 << 8)
#define VI_CSI_PPB_FRAME_START          (10 << 8)
#define VI_MWA_ACK_DONE                 (6 << 8)
#define VI_MWB_ACK_DONE                 (7 << 8)

#endif /* __TEGRA_VI_H__ */
