/*
 * tegra_vi.c - Tegra Video Input (VI) V4L2 capture driver
 *
 * Copyright (c) 2026, Dargons10
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This driver provides a V4L2 interface for capturing video from MIPI CSI-2
 * sensors connected to the Tegra K1 Video Input hardware.
 *
 * Compatible with Linux kernel 3.10
 */

#define pr_fmt(fmt) "tegra-vi: " fmt

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>

#include <linux/of_i2c.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/tegra-powergate.h>
#include <linux/vmalloc.h>
#include <linux/nvhost.h>
#include <nvhost_syncpt.h>
#include <host1x.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-of.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-vmalloc.h>
#include <media/media-device.h>
#include <media/media-entity.h>

#include "tegra_vi.h"
#include "tegra_csi.h"

/* TPG functions */
#include "tegra_tpg.h"

/* Video formats supported */
static const struct tegra_vi_format {
    u32 fourcc;
    const char *description;
    int depth;
} tegra_vi_formats[] = {
    { V4L2_PIX_FMT_SBGGR8,  "Bayer 8-bit BGBG..GRGR..", 8 },
    { V4L2_PIX_FMT_SGBRG8,  "Bayer 8-bit GBGB..RGRG..", 8 },
    { V4L2_PIX_FMT_SGRBG8,  "Bayer 8-bit GRGR..BGBG..", 8 },
    { V4L2_PIX_FMT_SRGGB8,  "Bayer 8-bit RGRG..GBGB..", 8 },
    { V4L2_PIX_FMT_SBGGR10, "Bayer 10-bit BGBG..GRGR..", 16 },
    { V4L2_PIX_FMT_SGBRG10, "Bayer 10-bit GBGB..RGRG..", 16 },
    { V4L2_PIX_FMT_SGRBG10, "Bayer 10-bit GRGR..BGBG..", 16 },
    { V4L2_PIX_FMT_SRGGB10, "Bayer 10-bit RGRG..GBGB..", 16 },
};

#define NUM_FORMATS ARRAY_SIZE(tegra_vi_formats)

/* Depth lookup: bytes per pixel for a given fourcc */
static int tegra_vi_format_depth(u32 fourcc)
{
    int i;
    for (i = 0; i < NUM_FORMATS; i++) {
        if (tegra_vi_formats[i].fourcc == fourcc)
            return tegra_vi_formats[i].depth / 8;
    }
    return 2; /* default: 16-bit = 2 bytes */
}

static struct tegra_vi *g_vi;

/* Per video-device data (maps video_device to channel) */
struct tegra_vi_vdev_data {
    struct tegra_vi *vi;
    int channel;
};

/* Hardware register access */
static inline void tegra_vi_write(struct tegra_vi *vi, u32 reg, u32 val)
{
    iowrite32(val, vi->base + reg);
}

static inline u32 tegra_vi_read(struct tegra_vi *vi, u32 reg)
{
    return ioread32(vi->base + reg);
}

/* IRQ Handler - uses TEGRA_VI_CFG_* register map from vi2.c */
static irqreturn_t tegra_vi_irq_handler(int irq, void *data)
{
    struct tegra_vi *vi = data;
    u32 status, mask;

    status = tegra_vi_read(vi, TEGRA_VI_CFG_INTERRUPT_STATUS);
    mask = tegra_vi_read(vi, TEGRA_VI_CFG_INTERRUPT_MASK);

    /* Clear all pending interrupts by writing back status */
    tegra_vi_write(vi, TEGRA_VI_CFG_INTERRUPT_STATUS, status);

    /* Wake capture thread on any interrupt */
    if (status & mask)
        wake_up_interruptible(&vi->capture_wait);

    return (status & mask) ? IRQ_HANDLED : IRQ_NONE;
}

/* Forward declarations */
static int tegra_vi_start_streaming(struct vb2_queue *vq, unsigned int count);
static int tegra_vi_stop_streaming(struct vb2_queue *vq);
static int tegra_vi_queue_setup(struct vb2_queue *vq,
                                    const struct v4l2_format *fmt,
                                    unsigned int *num_buffers,
                                    unsigned int *num_planes,
                                    unsigned int sizes[],
                                    void *alloc_ctxs[]);
static void tegra_vi_buf_queue(struct vb2_buffer *vb);


/* Async sensor discovery */
static void tegra_vi_sensor_scan_work(struct work_struct *work);
static void tegra_vi_build_sensor_list(struct tegra_vi *vi);
static int tegra_vi_async_complete(struct tegra_vi *vi);

static struct vb2_ops tegra_vi_vb2_ops = {
    .queue_setup    = tegra_vi_queue_setup,
    .buf_queue      = tegra_vi_buf_queue,
    .start_streaming = tegra_vi_start_streaming,
    .stop_streaming = tegra_vi_stop_streaming,
    .wait_prepare   = vb2_ops_wait_prepare,
    .wait_finish    = vb2_ops_wait_finish,
};

/* Media Controller functions - NUEVO */
static int tegra_vi_media_init(struct tegra_vi *vi);
static int tegra_vi_create_links(struct tegra_vi *vi);
static int tegra_vi_find_sensors(struct tegra_vi *vi);

/* Lista de compatibles conocidos de sensores */
static const char *tegra_vi_known_sensors[] = {
    "sony,imx179",
    "ovt,ov5693",
    "adi,ad5823",
    NULL
};

/* Find sensors via device tree - busca por compatibles conocidos */
static int tegra_vi_find_sensors(struct tegra_vi *vi)
{
    const char **compat;
    int found = 0;

    pr_info("Looking for sensors in device tree\n");

    for (compat = tegra_vi_known_sensors; *compat; compat++) {
        struct device_node *np;
        struct i2c_client *client;
        struct v4l2_subdev *sd;
        u32 csi_port_val;
        int channel;

        for_each_compatible_node(np, NULL, *compat) {
            if (!of_device_is_available(np))
                continue;

            client = of_find_i2c_device_by_node(np);
            if (!client) {
                pr_debug("Sensor %s: I2C device not ready yet\n", *compat);
                continue;
            }

            sd = i2c_get_clientdata(client);
            if (!sd) {
                pr_debug("Sensor %s: subdev not ready yet\n", *compat);
                put_device(&client->dev);
                continue;
            }

            /* Determinar canal por csi-port */
            channel = found;
            if (of_property_read_u32(np, "csi-port", &csi_port_val) == 0)
                channel = csi_port_val;

            /* Validar csi-port contra num_channels */
            if (channel < 0 || channel >= vi->num_channels) {
                pr_warn("Sensor %s: csi-port %d invalid (num_channels=%d), clamping\n",
                        sd->name, csi_port_val, vi->num_channels);
                channel = found;
            }

            if (!sd->v4l2_dev) {
                int ret = v4l2_device_register_subdev(&vi->v4l2_dev, sd);
                if (ret) {
                    pr_err("Failed to register %s: %d\n", sd->name, ret);
                    put_device(&client->dev);
                    continue;
                }
            }

            if (vi->channels[channel].sensor_sd) {
                pr_warn("Channel %d already has sensor %s, skipping %s\n",
                    channel, vi->channels[channel].sensor_sd->name, sd->name);
                put_device(&client->dev);
                continue;
            }

            if (channel < vi->num_channels) {
                u32 num_lanes_val = 4; /* default */
                of_property_read_u32(np, "num-lanes", &num_lanes_val);
                vi->channels[channel].sensor_sd = sd;
                vi->channels[channel].csi_port = csi_port_val;
                vi->channels[channel].num_lanes = num_lanes_val;
                pr_info("Channel %d: sensor=%s, csi_port=%d, lanes=%d\n",
                        channel, sd->name, csi_port_val, num_lanes_val);
            } else {
                pr_warn("Sensor %s: channel %d (csi-port %d) out of range, skipping\n",
                        sd->name, channel, csi_port_val);
            }

            put_device(&client->dev);
            found++;
        }
    }

    pr_info("Found %d sensors\n", found);

    if (found > 0)
        tegra_vi_async_complete(vi);

    return found > 0 ? 0 : -ENODEV;
}

/* Initialize Media Controller */
static int tegra_vi_media_init(struct tegra_vi *vi)
{
    int ret;
    int i;

    /* Initialize media device */
    strlcpy(vi->mdev.model, "NVIDIA Tegra VI",
            sizeof(vi->mdev.model));
    vi->mdev.dev = vi->dev;
    media_device_init(&vi->mdev);

    ret = media_device_register(&vi->mdev);
    if (ret) {
        pr_err("Failed to register media device: %d\n", ret);
        return ret;
    }
    vi->media_registered = true;

    /* Link v4l2_device to media_device */
    vi->v4l2_dev.mdev = &vi->mdev;

    /* Create VI entity with pads */
    vi->entity.name = "tegra-vi";
    vi->entity.type = MEDIA_ENT_TYPE_DEV_ATOM;
    vi->entity.flags = MEDIA_ENT_FL_SINK;

    for (i = 0; i < vi->num_channels; i++) {
        vi->pads[i].index = i;
        vi->pads[i].flags = MEDIA_PAD_FL_SINK;
    }

    ret = media_entity_init(&vi->entity, vi->num_channels, vi->pads, 0);
    if (ret) {
        pr_err("Failed to init VI entity: %d\n", ret);
        goto err_media;
    }
    vi->entity_registered = true;

    ret = media_device_register_entity(&vi->mdev, &vi->entity);
    if (ret) {
        pr_err("Failed to register VI entity: %d\n", ret);
        goto err_entity;
    }

    pr_info("Media device initialized: %s\n", vi->mdev.model);
    return 0;

err_entity:
    if (vi->entity_registered)
        media_entity_cleanup(&vi->entity);
err_media:
    if (vi->media_registered)
        media_device_unregister(&vi->mdev);
    return ret;
}

/* Find CSI port struct for a given VI channel */
static struct media_entity *tegra_vi_get_csi_entity(struct tegra_vi *vi,
                                                      int channel)
{
    if (!vi->csi || channel >= vi->csi->num_ports)
        return NULL;

    /* CSI ports are indexed by initialization order in tegra_csi_init():
     *   ports[0].port = 0 (CSI-A)  -> VI channel 0
     *   ports[1].port = 4 (CSI-E)  -> VI channel 1
     * Return the entity for the matching CSI port index.
     */
    return &vi->csi->ports[channel].entity;
}

/* Create Media Controller links: sensor -> CSI -> VI */
static int tegra_vi_create_links(struct tegra_vi *vi)
{
    int i;
    int ret;

    pr_info("Creating Media Controller links\n");

    for (i = 0; i < vi->num_channels; i++) {
        struct v4l2_subdev *sensor = vi->channels[i].sensor_sd;
        struct media_entity *csi_entity;

        if (!sensor || !sensor->entity.pads)
            continue;

        pr_info("Creating links for channel %d: %s\n", i, sensor->name);

        /* Link 1: sensor output (pad 0) -> CSI sink (pad 0) */
        csi_entity = tegra_vi_get_csi_entity(vi, i);
        if (csi_entity) {
            ret = media_create_pad_link(&sensor->entity, 0,
                                         csi_entity, 0,
                                         MEDIA_LNK_FL_ENABLED);
            if (ret)
                pr_warn("Failed sensor->CSI link ch%d: %d\n", i, ret);
            else
                pr_info("Link: %s -> %s (sensor->CSI)\n",
                        sensor->name, csi_entity->name);

            /* Link 2: CSI source (pad 1) -> VI sink (pad i) */
            ret = media_create_pad_link(csi_entity, 1,
                                         &vi->entity, i,
                                         MEDIA_LNK_FL_ENABLED);
            if (ret)
                pr_warn("Failed CSI->VI link ch%d: %d\n", i, ret);
            else
                pr_info("Link: %s -> VI channel %d (CSI->VI)\n",
                        csi_entity->name, i);
        } else {
            /* Fallback: direct sensor->VI link (no CSI entity) */
            ret = media_create_pad_link(&sensor->entity, 0,
                                         &vi->entity, i,
                                         MEDIA_LNK_FL_ENABLED);
            if (ret)
                pr_warn("Failed direct sensor->VI link ch%d: %d\n", i, ret);
            else
                pr_info("Link: %s -> VI channel %d (direct)\n",
                        sensor->name, i);
        }
    }

    pr_info("Media Controller links created\n");
    return 0;
}

/*
 * Async sensor discovery system
 * Kernel 3.10 no tiene v4l2_async_notifier, implementación custom.
 * Sensores llaman tegra_vi_register_sensor() desde su probe.
 * Un workqueue de respaldo scannea DT cada 200ms por si algún sensor
 * no llama la función de registro.
 */

#define SENSOR_SCAN_INTERVAL_MS 200
#define SENSOR_SCAN_MAX_RETRIES 50  /* 10 segundos máximo */

static void tegra_vi_sensor_entry_free(struct tegra_vi_sensor_entry *entry)
{
    if (entry) {
        of_node_put(entry->node);
        list_del(&entry->list);
        kfree(entry);
    }
}

static struct tegra_vi_sensor_entry *
tegra_vi_find_pending_entry(struct tegra_vi *vi, const char *compatible)
{
    struct tegra_vi_sensor_entry *entry;

    list_for_each_entry(entry, &vi->sensor_pending_list, list) {
        if (entry->found)
            continue;
        if (strcmp(entry->compatible, compatible) == 0)
            return entry;
    }
    return NULL;
}

static struct tegra_vi_sensor_entry *
tegra_vi_find_pending_by_node(struct tegra_vi *vi, struct device_node *node)
{
    struct tegra_vi_sensor_entry *entry;

    list_for_each_entry(entry, &vi->sensor_pending_list, list) {
        if (entry->found)
            continue;
        if (entry->node == node)
            return entry;
    }
    return NULL;
}

/* Construye lista de sensores esperados desde DT */
static void tegra_vi_build_sensor_list(struct tegra_vi *vi)
{
    struct tegra_vi_sensor_entry *entry;
    const char **compat;
    u32 csi_port_val;
    int channel;

    INIT_LIST_HEAD(&vi->sensor_pending_list);
    vi->num_expected_sensors = 0;
    vi->num_found_sensors = 0;

    /* Buscar sensores por compatible conocido en lugar de seguir endpoints */
    for (compat = tegra_vi_known_sensors; *compat; compat++) {
        struct device_node *np;

        for_each_compatible_node(np, NULL, *compat) {
            if (!of_device_is_available(np))
                continue;

            /* Leer csi-port del sensor para determinar el canal VI */
            channel = vi->num_expected_sensors;
            if (of_property_read_u32(np, "csi-port", &csi_port_val) == 0)
                channel = csi_port_val;

            /* Validar csi-port contra num_channels */
            if (channel >= vi->num_channels) {
                pr_warn("Sensor %s: csi-port %d >= num_channels %d, clamping\n",
                        np->name, channel, vi->num_channels);
                channel = vi->num_expected_sensors;
            }

            /* Crear entry pending */
            entry = kzalloc(sizeof(*entry), GFP_KERNEL);
            if (!entry) {
                continue;
            }

            strlcpy(entry->compatible, *compat, sizeof(entry->compatible));
            entry->node = of_node_get(np);
            entry->channel = channel;
            entry->found = false;
            entry->is_focuser = (strstr(*compat, "ad5823") != NULL) ||
                                (strstr(*compat, "ad5820") != NULL) ||
                                (strstr(*compat, "focus") != NULL);

            list_add_tail(&entry->list, &vi->sensor_pending_list);
            vi->num_expected_sensors++;

            pr_info("Expected sensor: %s (%s) channel=%d%s\n",
                    np->full_name, *compat, channel,
                    entry->is_focuser ? " [focuser]" : "");
        }
    }

    pr_info("Built sensor list: %d expected\n", vi->num_expected_sensors);
}

/* Escanea DT periódicamente para encontrar sensores que no llamaron register */
static void tegra_vi_sensor_scan_work(struct work_struct *work)
{
    struct tegra_vi *vi = container_of(work, struct tegra_vi,
                                        sensor_scan_work.work);
    struct tegra_vi_sensor_entry *entry, *tmp;
    struct i2c_client *client;
    struct v4l2_subdev *sd;
    bool all_found = true;
    int ret;

    list_for_each_entry_safe(entry, tmp, &vi->sensor_pending_list, list) {
        if (entry->found || entry->is_focuser)
            continue;

        client = of_find_i2c_device_by_node(entry->node);
        if (!client) {
            all_found = false;
            continue;
        }

        sd = i2c_get_clientdata(client);
        if (!sd) {
            put_device(&client->dev);
            all_found = false;
            continue;
        }

        /* Found this sensor via DT scan */
        if (!sd->v4l2_dev) {
            ret = v4l2_device_register_subdev(&vi->v4l2_dev, sd);
            if (ret) {
                pr_warn("Failed to register sensor %s via scan: %d\n",
                        sd->name, ret);
                put_device(&client->dev);
                all_found = false;
                continue;
            }
        }

        {
            u32 csi_port_val = entry->channel;
            u32 num_lanes_val = 4;
            of_property_read_u32(entry->node, "csi-port", &csi_port_val);
            of_property_read_u32(entry->node, "num-lanes", &num_lanes_val);
            vi->channels[entry->channel].sensor_sd = sd;
            vi->channels[entry->channel].csi_port = csi_port_val;
            vi->channels[entry->channel].num_lanes = num_lanes_val;
        }

        pr_info("Sensor %s found via DT scan (channel %d, port=%d, lanes=%d)\n",
                sd->name, entry->channel,
                vi->channels[entry->channel].csi_port,
                vi->channels[entry->channel].num_lanes);

        put_device(&client->dev);
    }

    /* Verificar si todos encontrados */
    list_for_each_entry(entry, &vi->sensor_pending_list, list) {
        if (!entry->found && !entry->is_focuser) {
            all_found = false;
            break;
        }
    }

    if (all_found && vi->num_found_sensors > 0) {
        pr_info("All sensors found via DT scan, completing async setup\n");
        tegra_vi_async_complete(vi);
        vi->sensor_scan_done = true;
        return;
    }

    /* Re-schedule si no hemos excedido retries */
    if (vi->num_found_sensors < vi->num_expected_sensors &&
        !vi->sensor_scan_done) {
        vi->sensor_scan_retries++;
        if (vi->sensor_scan_retries < SENSOR_SCAN_MAX_RETRIES) {
            schedule_delayed_work(&vi->sensor_scan_work,
                                  msecs_to_jiffies(SENSOR_SCAN_INTERVAL_MS));
        } else {
            pr_warn("Sensor scan timed out after %d retries\n",
                    vi->sensor_scan_retries);
            vi->sensor_scan_done = true;
        }
    }
}

/* Llamada por sensores I2C desde su probe */
int tegra_vi_register_sensor(struct v4l2_subdev *sd, const char *compatible)
{
    struct tegra_vi *vi = g_vi;
    struct tegra_vi_sensor_entry *entry;
    int ret;

    if (!vi) {
        pr_debug("VI not ready yet, deferring sensor %s\n",
                 compatible ? compatible : "unknown");
        return -EPROBE_DEFER;
    }

    entry = tegra_vi_find_pending_entry(vi, compatible);
    if (!entry) {
        pr_warn("No pending entry for sensor %s, checking by node\n",
                compatible ? compatible : "unknown");
        /* Fallback: buscar por node via i2c_client (kernel 3.10) */
        {
            struct i2c_client *client = v4l2_get_subdevdata(sd);
            if (client && client->dev.of_node)
                entry = tegra_vi_find_pending_by_node(vi, client->dev.of_node);
        }
        if (!entry) {
            pr_warn("Sensor %s not in pending list, ignoring\n",
                    compatible ? compatible : "unknown");
            return -ENOENT;
        }
    }

    if (entry->found) {
        pr_debug("Sensor %s already registered\n", compatible);
        return 0;
    }

    /* Registrar subdev con VI */
    if (!sd->v4l2_dev) {
        ret = v4l2_device_register_subdev(&vi->v4l2_dev, sd);
        if (ret) {
            pr_err("Failed to register subdev %s: %d\n", sd->name, ret);
            return ret;
        }
    }

    {
        u32 csi_port_val = entry->channel;
        u32 num_lanes_val = 4;
        of_property_read_u32(entry->node, "csi-port", &csi_port_val);
        of_property_read_u32(entry->node, "num-lanes", &num_lanes_val);

        entry->sd = sd;
        entry->found = true;
        vi->num_found_sensors++;

        if (entry->channel < vi->num_channels) {
            vi->channels[entry->channel].sensor_sd = sd;
            vi->channels[entry->channel].csi_port = csi_port_val;
            vi->channels[entry->channel].num_lanes = num_lanes_val;
        }
    }

    pr_info("Sensor registered async: %s (compatible=%s, channel=%d, port=%d, lanes=%d)\n",
            sd->name, compatible, entry->channel,
            vi->channels[entry->channel].csi_port,
            vi->channels[entry->channel].num_lanes);

    /* Verificar si todos los sensores están listos */
    {
        struct tegra_vi_sensor_entry *e;
        bool all_found = true;

        list_for_each_entry(e, &vi->sensor_pending_list, list) {
            if (!e->found && !e->is_focuser) {
                all_found = false;
                break;
            }
        }

        if (all_found && vi->num_found_sensors > 0 &&
            !vi->sensor_scan_done) {
            pr_info("All sensors registered async, completing setup\n");
            tegra_vi_async_complete(vi);
            vi->sensor_scan_done = true;
        }
    }

    return 0;
}
EXPORT_SYMBOL_GPL(tegra_vi_register_sensor);

/* Llamada por focuser I2C desde su probe */
int tegra_vi_register_focuser(struct v4l2_subdev *sd, const char *compatible)
{
    struct tegra_vi *vi = g_vi;
    struct tegra_vi_sensor_entry *entry;

    if (!vi)
        return -EPROBE_DEFER;

    entry = tegra_vi_find_pending_entry(vi, compatible);
    if (!entry) {
        pr_warn("No pending entry for focuser %s\n",
                compatible ? compatible : "unknown");
        return -ENOENT;
    }

    if (entry->found) {
        pr_debug("Focuser %s already registered\n", compatible);
        return 0;
    }

    entry->sd = sd;
    entry->found = true;

    /* Assign focuser to channel 0 (IMX179 rear camera has AF) */
    vi->channels[0].focuser_sd = sd;
    pr_info("Focuser registered on channel 0: %s\n", sd->name);

    return 0;
}
EXPORT_SYMBOL_GPL(tegra_vi_register_focuser);

/* Cuando todos los sensores están listos */
static int tegra_vi_async_complete(struct tegra_vi *vi)
{
    pr_info("Async sensor registration complete (%d/%d found)\n",
            vi->num_found_sensors, vi->num_expected_sensors);

    /* Crear Media Controller links usando tegra_vi_create_links */
    tegra_vi_create_links(vi);

    pr_info("Async setup complete, sensors ready for streaming\n");
    return 0;
}

/* Helper to get channel from video device */
static int tegra_vi_get_channel(struct file *file)
{
    struct tegra_vi_vdev_data *data = video_drvdata(file);
    return data->channel;
}

/* Query capabilities */
static int tegra_vi_querycap(struct file *file, void *priv,
                                 struct v4l2_capability *cap)
{
    struct tegra_vi_vdev_data *data = video_drvdata(file);
    struct tegra_vi *vi = data->vi;

    strlcpy(cap->driver, TEGRA_VI_DRIVER_NAME, sizeof(cap->driver));
    strlcpy(cap->card, "Tegra Video Input", sizeof(cap->card));
    snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
             dev_name(vi->dev));

    cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

    return 0;
}

/* Enumerate formats */
static int tegra_vi_enum_fmt_vid_cap(struct file *file, void *priv,
                                         struct v4l2_fmtdesc *f)
{
    if (f->index >= NUM_FORMATS)
        return -EINVAL;

    f->pixelformat = tegra_vi_formats[f->index].fourcc;
    strlcpy(f->description, tegra_vi_formats[f->index].description,
            sizeof(f->description));
    return 0;
}

/* Get current format */
static int tegra_vi_g_fmt_vid_cap(struct file *file, void *priv,
                                      struct v4l2_format *f)
{
    int ch = tegra_vi_get_channel(file);
    struct tegra_vi_vdev_data *data = video_drvdata(file);
    struct tegra_vi *vi = data->vi;

    f->fmt.pix.width = vi->channels[ch].width;
    f->fmt.pix.height = vi->channels[ch].height;
    f->fmt.pix.pixelformat = vi->channels[ch].format;
    f->fmt.pix.field = V4L2_FIELD_NONE;
    f->fmt.pix.bytesperline = vi->channels[ch].width *
        tegra_vi_format_depth(vi->channels[ch].format);
    f->fmt.pix.sizeimage = f->fmt.pix.bytesperline * vi->channels[ch].height;
    f->fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;

    return 0;
}

/* Try format */
static int tegra_vi_try_fmt_vid_cap(struct file *file, void *priv,
                                        struct v4l2_format *f)
{
    struct v4l2_pix_format *pix = &f->fmt.pix;
    int i;

    for (i = 0; i < NUM_FORMATS; i++) {
        if (tegra_vi_formats[i].fourcc == pix->pixelformat)
            break;
    }

    if (i == NUM_FORMATS)
        pix->pixelformat = V4L2_PIX_FMT_SRGGB10;

    pix->width = ALIGN(pix->width, 2);
    pix->height = ALIGN(pix->height, 2);
    pix->bytesperline = pix->width * tegra_vi_format_depth(pix->pixelformat);
    pix->sizeimage = pix->bytesperline * pix->height;
    pix->field = V4L2_FIELD_NONE;
    pix->colorspace = V4L2_COLORSPACE_SRGB;

    return 0;
}

/* Set format */
static int tegra_vi_s_fmt_vid_cap(struct file *file, void *priv,
                                      struct v4l2_format *f)
{
    int ch = tegra_vi_get_channel(file);
    struct tegra_vi_vdev_data *data = video_drvdata(file);
    struct tegra_vi *vi = data->vi;
    int ret;

    if (vi->streaming)
        return -EBUSY;

    ret = tegra_vi_try_fmt_vid_cap(file, priv, f);
    if (ret)
        return ret;

    vi->channels[ch].width = f->fmt.pix.width;
    vi->channels[ch].height = f->fmt.pix.height;
    vi->channels[ch].format = f->fmt.pix.pixelformat;

    return 0;
}

/* Custom ioctl handlers that set active_channel */
static int tegra_vi_reqbufs(struct file *file, void *priv,
                            struct v4l2_requestbuffers *p)
{
    struct tegra_vi_vdev_data *data = video_drvdata(file);
    int ret;

    pr_info("REQBUFS entry: ch=%d count=%d type=%s\n",
            data->channel, p->count,
            p->type == V4L2_BUF_TYPE_VIDEO_CAPTURE ? "VIDEO_CAPTURE" : "OTHER");
    data->vi->active_channel = data->channel;
    ret = vb2_ioctl_reqbufs(file, priv, p);
    pr_info("REQBUFS exit: ch=%d ret=%d\n", data->channel, ret);
    return ret;
}

static int tegra_vi_streamon(struct file *file, void *priv,
                             enum v4l2_buf_type type)
{
    struct tegra_vi_vdev_data *data = video_drvdata(file);
    int ret;

    pr_info("STREAMON entry: ch=%d\n", data->channel);
    data->vi->active_channel = data->channel;
    ret = vb2_ioctl_streamon(file, priv, type);
    pr_info("STREAMON exit: ch=%d ret=%d\n", data->channel, ret);
    return ret;
}

static int tegra_vi_streamoff(struct file *file, void *priv,
                              enum v4l2_buf_type type)
{
    struct tegra_vi_vdev_data *data = video_drvdata(file);
    int ret;

    pr_info("STREAMOFF entry: ch=%d\n", data->channel);
    data->vi->active_channel = data->channel;
    ret = vb2_ioctl_streamoff(file, priv, type);
    pr_info("STREAMOFF exit: ch=%d ret=%d\n", data->channel, ret);
    return ret;
}

/* Forward V4L2 controls to the sensor or focuser subdev */
static int tegra_vi_s_ctrl(struct file *file, void *fh, struct v4l2_control *ctrl)
{
    struct tegra_vi_vdev_data *data = video_drvdata(file);
    struct tegra_vi_channel *chan;
    int ret;

    if (!data) {
        pr_err("s_ctrl: no data\n");
        return -EINVAL;
    }
    chan = &data->vi->channels[data->channel];

    /* Try focuser first (V4L2_CID_FOCUS_ABSOLUTE) */
    if (chan->focuser_sd && chan->focuser_sd->ctrl_handler) {
        ret = v4l2_s_ctrl(NULL, chan->focuser_sd->ctrl_handler, ctrl);
        if (ret != -EINVAL) {
            pr_info("s_ctrl ch%d id=%d val=%d (focuser) -> ret=%d\n",
                    data->channel, ctrl->id, ctrl->value, ret);
            return ret;
        }
    }

    /* Fall back to sensor subdev (exposure, gain, etc.) */
    if (!chan->sensor_sd) {
        pr_err("s_ctrl ch%d: sensor_sd is NULL\n", data->channel);
        return -EINVAL;
    }
    if (!chan->sensor_sd->ctrl_handler) {
        pr_err("s_ctrl ch%d: ctrl_handler is NULL\n", data->channel);
        return -EINVAL;
    }
    ret = v4l2_s_ctrl(NULL, chan->sensor_sd->ctrl_handler, ctrl);
    pr_info("s_ctrl ch%d id=%d val=%d (sensor) -> ret=%d\n",
            data->channel, ctrl->id, ctrl->value, ret);
    return ret;
}

static int tegra_vi_g_ctrl(struct file *file, void *fh, struct v4l2_control *ctrl)
{
    struct tegra_vi_vdev_data *data = video_drvdata(file);
    struct tegra_vi_channel *chan;
    int ret;

    if (!data) {
        pr_err("g_ctrl: no data\n");
        return -EINVAL;
    }
    chan = &data->vi->channels[data->channel];

    /* Try focuser first */
    if (chan->focuser_sd && chan->focuser_sd->ctrl_handler) {
        ret = v4l2_g_ctrl(chan->focuser_sd->ctrl_handler, ctrl);
        if (ret != -EINVAL) {
            pr_info("g_ctrl ch%d id=%d val=%d (focuser) -> ret=%d\n",
                    data->channel, ctrl->id, ctrl->value, ret);
            return ret;
        }
    }

    /* Fall back to sensor subdev */
    if (!chan->sensor_sd) {
        pr_err("g_ctrl ch%d: sensor_sd is NULL\n", data->channel);
        return -EINVAL;
    }
    if (!chan->sensor_sd->ctrl_handler) {
        pr_err("g_ctrl ch%d: ctrl_handler is NULL\n", data->channel);
        return -EINVAL;
    }
    ret = v4l2_g_ctrl(chan->sensor_sd->ctrl_handler, ctrl);
    pr_info("g_ctrl ch%d id=%d val=%d (sensor) -> ret=%d\n",
            data->channel, ctrl->id, ctrl->value, ret);
    return ret;
}

/* IOCTL operations */
static const struct v4l2_ioctl_ops tegra_vi_ioctl_ops = {
    .vidioc_querycap         = tegra_vi_querycap,
    .vidioc_enum_fmt_vid_cap = tegra_vi_enum_fmt_vid_cap,
    .vidioc_g_fmt_vid_cap    = tegra_vi_g_fmt_vid_cap,
    .vidioc_try_fmt_vid_cap  = tegra_vi_try_fmt_vid_cap,
    .vidioc_s_fmt_vid_cap    = tegra_vi_s_fmt_vid_cap,
    .vidioc_reqbufs          = tegra_vi_reqbufs,
    .vidioc_querybuf         = vb2_ioctl_querybuf,
    .vidioc_qbuf             = vb2_ioctl_qbuf,
    .vidioc_dqbuf            = vb2_ioctl_dqbuf,
    .vidioc_streamon         = tegra_vi_streamon,
    .vidioc_streamoff        = tegra_vi_streamoff,
    .vidioc_s_ctrl           = tegra_vi_s_ctrl,
    .vidioc_g_ctrl           = tegra_vi_g_ctrl,
};

/* Queue setup - kernel 3.10 signature */
static int tegra_vi_queue_setup(struct vb2_queue *vq,
                                    const struct v4l2_format *fmt,
                                    unsigned int *num_buffers,
                                    unsigned int *num_planes,
                                    unsigned int sizes[],
                                    void *alloc_ctxs[])
{
    struct tegra_vi *vi = vb2_get_drv_priv(vq);
    int ch = vi->active_channel;
    unsigned int size;

    if (ch < 0 || ch >= TEGRA_VI_MAX_CHANNELS)
        return -EINVAL;

    if (*num_planes)
        return sizes[0] < vi->channels[ch].width * vi->channels[ch].height *
                   tegra_vi_format_depth(vi->channels[ch].format) ? -EINVAL : 0;

    size = vi->channels[ch].width * vi->channels[ch].height *
        tegra_vi_format_depth(vi->channels[ch].format);
    *num_planes = 1;
    sizes[0] = size;

    return 0;
}

/* Buffer queue - add buffer to processing list */
static void tegra_vi_buf_queue(struct vb2_buffer *vb)
{
    struct tegra_vi *vi = vb2_get_drv_priv(vb->vb2_queue);
    struct tegra_vi_buffer *buf = container_of(vb, struct tegra_vi_buffer, vb);
    unsigned long flags;

    buf->state = TEGRA_VI_BUF_STATE_QUEUED;

    spin_lock_irqsave(&vi->slock, flags);
    list_add_tail(&buf->list, &vi->buf_queue);
    if (vi->streaming)
        wake_up_interruptible(&vi->capture_wait);
    spin_unlock_irqrestore(&vi->slock, flags);
}

/* ========== CSI + VI HW Capture Functions ========== */

/* Map V4L2 pixel format to TEGRA_IMAGE_FORMAT_T_* for VI_CSI_CH_IMAGE_DEF */
static int tegra_vi_v4l2_to_image_format(u32 v4l2_fmt)
{
    switch (v4l2_fmt) {
    case V4L2_PIX_FMT_SBGGR8:
    case V4L2_PIX_FMT_SGBRG8:
    case V4L2_PIX_FMT_SGRBG8:
    case V4L2_PIX_FMT_SRGGB8:
        return TEGRA_IMAGE_FORMAT_T_L8;
    case V4L2_PIX_FMT_SBGGR10:
    case V4L2_PIX_FMT_SGBRG10:
    case V4L2_PIX_FMT_SGRBG10:
    case V4L2_PIX_FMT_SRGGB10:
        return TEGRA_IMAGE_FORMAT_T_R16_I;
    default:
        return TEGRA_IMAGE_FORMAT_T_R16_I;
    }
}

/* Map V4L2 pixel format to MIPI CSI data type */
static int tegra_vi_v4l2_to_image_dt(u32 v4l2_fmt)
{
    switch (v4l2_fmt) {
    case V4L2_PIX_FMT_SBGGR8:
    case V4L2_PIX_FMT_SGBRG8:
    case V4L2_PIX_FMT_SGRBG8:
    case V4L2_PIX_FMT_SRGGB8:
        return TEGRA_IMAGE_DT_RAW8;
    case V4L2_PIX_FMT_SBGGR10:
    case V4L2_PIX_FMT_SGBRG10:
    case V4L2_PIX_FMT_SGRBG10:
    case V4L2_PIX_FMT_SRGGB10:
        return TEGRA_IMAGE_DT_RAW10;
    default:
        return TEGRA_IMAGE_DT_RAW10;
    }
}

/* Map V4L2 pixel format to mbus code for sensor format negotiation */
static u32 tegra_vi_v4l2_to_mbus(u32 v4l2_fmt)
{
    switch (v4l2_fmt) {
    case V4L2_PIX_FMT_SBGGR8:
        return V4L2_MBUS_FMT_SBGGR8_1X8;
    case V4L2_PIX_FMT_SGBRG8:
        return V4L2_MBUS_FMT_SGBRG8_1X8;
    case V4L2_PIX_FMT_SGRBG8:
        return V4L2_MBUS_FMT_SGRBG8_1X8;
    case V4L2_PIX_FMT_SRGGB8:
        return V4L2_MBUS_FMT_SRGGB8_1X8;
    case V4L2_PIX_FMT_SBGGR10:
        return V4L2_MBUS_FMT_SBGGR10_1X10;
    case V4L2_PIX_FMT_SGBRG10:
        return V4L2_MBUS_FMT_SGBRG10_1X10;
    case V4L2_PIX_FMT_SGRBG10:
        return V4L2_MBUS_FMT_SGRBG10_1X10;
    case V4L2_PIX_FMT_SRGGB10:
        return V4L2_MBUS_FMT_SRGGB10_1X10;
    default:
        return V4L2_MBUS_FMT_SRGGB10_1X10;
    }
}

/* Compute bytes-per-line aligned to 64 bytes (required by VI HW) */
static u32 tegra_vi_bytes_per_line(u32 width, u32 v4l2_fmt)
{
    u32 bpl = width * tegra_vi_format_depth(v4l2_fmt);
    if (bpl % 64)
        bpl = bpl + (64 - (bpl % 64));
    return bpl;
}

/*
 * MIPI Calibration para pads CSI.
 * Copiado de vi2_mipi_calibration() en vi2.c del kernel stock.
 * Sin esta calibración, el pixel parser VI no puede interpretar las
 * señales MIPI del sensor (syncpt timeout, error_status=0x00000000).
 *
 * Mapeo de canales a pads CIL:
 *   Channel 0 (IMX179, CSI-A, 4-lane) → CIL A + CIL B
 *   Channel 1 (OV5693, CSI-E, 1-lane) → CIL E
 */
static int tegra_vi_mipi_calibration(struct tegra_vi *vi, int ch)
{
    void __iomem *mipi_cal;
    struct clk *clk_mipi_cal = NULL, *clk_72mhz = NULL;
    u32 val;
    int retry = 500;
    int ret = 0;
    int lanes = vi->channels[ch].num_lanes;

    if (ch != 0 && ch != 1) {
        pr_err("MIPI cal: unsupported channel %d\n", ch);
        return -EINVAL;
    }

    if (vi->channels[ch].cal_done) {
        pr_debug("MIPI cal already done for channel %d, skipping\n", ch);
        return 0;
    }

    /* Get clocks */
    clk_mipi_cal = clk_get_sys("mipi-cal", NULL);
    if (IS_ERR_OR_NULL(clk_mipi_cal)) {
        pr_warn("MIPI cal: cannot get mipi-cal clock, skipping\n");
        /* Non-fatal: we proceed anyway */
        clk_mipi_cal = NULL;
    }

    clk_72mhz = clk_get_sys("clk72mhz", NULL);
    if (IS_ERR_OR_NULL(clk_72mhz)) {
        pr_warn("MIPI cal: cannot get clk72mhz, skipping\n");
        clk_72mhz = NULL;
    }

    /* Map MIPI CAL registers */
    mipi_cal = ioremap(MIPI_CAL_BASE, 0x100);
    if (!mipi_cal) {
        pr_err("MIPI cal: ioremap failed\n");
        ret = -ENOMEM;
        goto out_clocks;
    }

    /* Enable clocks */
    if (clk_mipi_cal)
        clk_prepare_enable(clk_mipi_cal);
    if (clk_72mhz)
        clk_prepare_enable(clk_72mhz);

    pr_info("MIPI calibration starting for channel %d (lanes=%d)\n", ch, lanes);

    /* Step 1: CLKEN_OVR */
    iowrite32(ioread32(mipi_cal + MIPI_CAL_CTRL) | CLKEN_OVR,
              mipi_cal + MIPI_CAL_CTRL);

    /* Step 2: Clear status */
    iowrite32(0xF1F10000, mipi_cal + CIL_MIPI_CAL_STATUS);

    /* Step 3: Deselect all pads */
    iowrite32(ioread32(mipi_cal + DSIA_MIPI_CAL_CONFIG) & ~SELDSIA,
              mipi_cal + DSIA_MIPI_CAL_CONFIG);
    iowrite32(ioread32(mipi_cal + DSIB_MIPI_CAL_CONFIG) & ~SELDSIB,
              mipi_cal + DSIB_MIPI_CAL_CONFIG);
    iowrite32(ioread32(mipi_cal + MIPI_BIAS_PAD_CFG0) | E_VCLAMP_REF,
              mipi_cal + MIPI_BIAS_PAD_CFG0);
    iowrite32(ioread32(mipi_cal + MIPI_BIAS_PAD_CFG2) & ~PDVREG,
              mipi_cal + MIPI_BIAS_PAD_CFG2);
    iowrite32(ioread32(mipi_cal + CILA_MIPI_CAL_CONFIG) & ~SELA,
              mipi_cal + CILA_MIPI_CAL_CONFIG);
    iowrite32(ioread32(mipi_cal + DSIA_MIPI_CAL_CONFIG_2) & ~CLKSELDSIA,
              mipi_cal + DSIA_MIPI_CAL_CONFIG_2);
    iowrite32(ioread32(mipi_cal + CILB_MIPI_CAL_CONFIG) & ~SELB,
              mipi_cal + CILB_MIPI_CAL_CONFIG);
    iowrite32(ioread32(mipi_cal + DSIB_MIPI_CAL_CONFIG_2) & ~CLKSELDSIB,
              mipi_cal + DSIB_MIPI_CAL_CONFIG_2);
    iowrite32(ioread32(mipi_cal + CILC_MIPI_CAL_CONFIG) & ~SELC,
              mipi_cal + CILC_MIPI_CAL_CONFIG);
    iowrite32(ioread32(mipi_cal + CILC_MIPI_CAL_CONFIG_2) & ~CLKSELC,
              mipi_cal + CILC_MIPI_CAL_CONFIG_2);
    iowrite32(ioread32(mipi_cal + CILD_MIPI_CAL_CONFIG) & ~SELD,
              mipi_cal + CILD_MIPI_CAL_CONFIG);
    iowrite32(ioread32(mipi_cal + CILD_MIPI_CAL_CONFIG_2) & ~CLKSELD,
              mipi_cal + CILD_MIPI_CAL_CONFIG_2);
    iowrite32(ioread32(mipi_cal + CILE_MIPI_CAL_CONFIG) & ~SELE,
              mipi_cal + CILE_MIPI_CAL_CONFIG);
    iowrite32(ioread32(mipi_cal + CSIE_MIPI_CAL_CONFIG_2) & ~CLKSELE,
              mipi_cal + CSIE_MIPI_CAL_CONFIG_2);

    /* Step 4: Select the CIL pad(s) for this channel */
    if (ch == 0) {
        /* Channel 0: CSI-A → CIL A + CIL B (4-lane) */
        iowrite32(ioread32(mipi_cal + CILA_MIPI_CAL_CONFIG) | SELA,
                  mipi_cal + CILA_MIPI_CAL_CONFIG);
        iowrite32(ioread32(mipi_cal + DSIA_MIPI_CAL_CONFIG_2) & ~CLKSELDSIA,
                  mipi_cal + DSIA_MIPI_CAL_CONFIG_2);
        if (lanes > 2) {
            iowrite32(ioread32(mipi_cal + CILB_MIPI_CAL_CONFIG) | SELB,
                      mipi_cal + CILB_MIPI_CAL_CONFIG);
            iowrite32(ioread32(mipi_cal + DSIB_MIPI_CAL_CONFIG_2) & ~CLKSELDSIB,
                      mipi_cal + DSIB_MIPI_CAL_CONFIG_2);
        }
    } else if (ch == 1) {
        /* Channel 1: CSI-E → CIL E (1-lane) */
        iowrite32(ioread32(mipi_cal + CILE_MIPI_CAL_CONFIG) | SELE,
                  mipi_cal + CILE_MIPI_CAL_CONFIG);
        iowrite32(ioread32(mipi_cal + CSIE_MIPI_CAL_CONFIG_2) | CLKSELE,
                  mipi_cal + CSIE_MIPI_CAL_CONFIG_2);
    }

    /* Step 5: Trigger calibration */
    iowrite32(ioread32(mipi_cal + MIPI_CAL_CTRL) | STARTCAL,
              mipi_cal + MIPI_CAL_CTRL);

    /* Step 6: Wait for CAL_DONE */
    while (--retry) {
        val = ioread32(mipi_cal + CIL_MIPI_CAL_STATUS);
        if (val & CAL_DONE)
            break;
        usleep_range(200, 300);
    }

    if (!retry) {
        pr_err("MIPI calibration timeout for channel %d!\n", ch);
        ret = -EBUSY;
    } else {
        pr_info("MIPI calibration done for channel %d (status=0x%08x)\n", ch, val);
    }

    /* Step 7: Cleanup - deselect all pads to avoid interfering with DSI */
    iowrite32(ioread32(mipi_cal + CILA_MIPI_CAL_CONFIG) & ~SELA,
              mipi_cal + CILA_MIPI_CAL_CONFIG);
    iowrite32(ioread32(mipi_cal + DSIA_MIPI_CAL_CONFIG_2) | CLKSELDSIA,
              mipi_cal + DSIA_MIPI_CAL_CONFIG_2);
    iowrite32(ioread32(mipi_cal + CILB_MIPI_CAL_CONFIG) & ~SELB,
              mipi_cal + CILB_MIPI_CAL_CONFIG);
    iowrite32(ioread32(mipi_cal + DSIB_MIPI_CAL_CONFIG_2) | CLKSELDSIB,
              mipi_cal + DSIB_MIPI_CAL_CONFIG_2);
    iowrite32(ioread32(mipi_cal + CILC_MIPI_CAL_CONFIG) & ~SELC,
              mipi_cal + CILC_MIPI_CAL_CONFIG);
    iowrite32(ioread32(mipi_cal + CILC_MIPI_CAL_CONFIG_2) | CLKSELC,
              mipi_cal + CILC_MIPI_CAL_CONFIG_2);
    iowrite32(ioread32(mipi_cal + CILD_MIPI_CAL_CONFIG) & ~SELD,
              mipi_cal + CILD_MIPI_CAL_CONFIG);
    iowrite32(ioread32(mipi_cal + CILD_MIPI_CAL_CONFIG_2) | CLKSELD,
              mipi_cal + CILD_MIPI_CAL_CONFIG_2);
    iowrite32(ioread32(mipi_cal + CILE_MIPI_CAL_CONFIG) & ~SELE,
              mipi_cal + CILE_MIPI_CAL_CONFIG);
    iowrite32(ioread32(mipi_cal + CSIE_MIPI_CAL_CONFIG_2) & ~CLKSELE,
              mipi_cal + CSIE_MIPI_CAL_CONFIG_2);

    /* Disable clocks */
    if (clk_mipi_cal)
        clk_disable_unprepare(clk_mipi_cal);
    if (clk_72mhz)
        clk_disable_unprepare(clk_72mhz);

    iounmap(mipi_cal);

    if (!ret)
        vi->channels[ch].cal_done = true;

    return ret;

out_clocks:
    if (clk_mipi_cal)
        clk_put(clk_mipi_cal);
    if (clk_72mhz)
        clk_put(clk_72mhz);
    return ret;
}

/* Forward declaration */
static void tegra_vi_channel_free_bounce(struct tegra_vi *vi, int ch);

/* Allocate DMA bounce buffer for a channel */
static int tegra_vi_channel_alloc_bounce(struct tegra_vi *vi, int ch)
{
    struct tegra_vi_channel *chan = &vi->channels[ch];
    size_t size;

    if (!chan->width || !chan->height)
        return 0;

    size = chan->width * chan->height * tegra_vi_format_depth(chan->format);

    if (chan->bounce_buf_cpu && chan->bounce_size >= size)
        return 0; /* Already allocated with sufficient size */

    /* Free old if exists */
    if (chan->bounce_buf_cpu)
        tegra_vi_channel_free_bounce(vi, ch);

    chan->bounce_buf_cpu = dma_alloc_coherent(vi->dev, size,
                                               &chan->bounce_buf_dma,
                                               GFP_KERNEL);
    if (!chan->bounce_buf_cpu) {
        pr_err("Failed to allocate DMA bounce buffer for channel %d (%zu bytes)\n",
               ch, size);
        return -ENOMEM;
    }

    chan->bounce_size = size;
    pr_info("Channel %d: DMA bounce buffer allocated (%zu bytes, dma=0x%llx)\n",
            ch, size, (unsigned long long)chan->bounce_buf_dma);
    return 0;
}

static void tegra_vi_channel_free_bounce(struct tegra_vi *vi, int ch)
{
    struct tegra_vi_channel *chan = &vi->channels[ch];

    if (chan->bounce_buf_cpu) {
        dma_free_coherent(vi->dev, chan->bounce_size,
                          chan->bounce_buf_cpu, chan->bounce_buf_dma);
        chan->bounce_buf_cpu = NULL;
        chan->bounce_buf_dma = 0;
        chan->bounce_size = 0;
    }
}

/*
 * tegra_vi_setup_csi_channel() - Configure CSI PHY + pixel parser + VI capture
 * registers for a given channel. Based on vi2_capture_setup_csi_0/1 from vi2.c
 *
 * Channel 0: CSI-A (port 0, pixel parser A, CIL A+B) -> IMX179, 4-lane
 * Channel 1: CSI-E (port 4, pixel parser B, CIL E) -> OV5693, 1-lane
 */
static int tegra_vi_setup_csi_channel(struct tegra_vi *vi, int ch)
{
    u32 width = vi->channels[ch].width;
    u32 height = vi->channels[ch].height;
    u32 v4l2_fmt = vi->channels[ch].format;
    int fmt = tegra_vi_v4l2_to_image_format(v4l2_fmt);
    int dt = tegra_vi_v4l2_to_image_dt(v4l2_fmt);
    int lanes = vi->channels[ch].num_lanes;
    u32 image_size_wc;
    u32 val;

    if (ch == 0) {
        /* ==== Channel 0: CSI-A (port 0), pixel parser A, CIL A+B ==== */

        /* Power up CIL pads */
        tegra_vi_write(vi, TEGRA_CSI_CILA_PAD_CONFIG0, 0x10000);
        tegra_vi_write(vi, TEGRA_CSI_CILB_PAD_CONFIG0, 0x0);

        /* Mask CIL interrupts */
        tegra_vi_write(vi, TEGRA_CSI_CSI_CIL_A_INTERRUPT_MASK, 0x0);
        tegra_vi_write(vi, TEGRA_CSI_CSI_CIL_B_INTERRUPT_MASK, 0x0);

        /* Configure PHY */
        tegra_vi_write(vi, TEGRA_CSI_PHY_CILA_CONTROL0, 0x9);
        tegra_vi_write(vi, TEGRA_CSI_PHY_CILB_CONTROL0, 0x9);

        /* Configure pixel parser A */
        tegra_vi_write(vi, TEGRA_CSI_PIXEL_STREAM_PPA_COMMAND, 0xf007);
        tegra_vi_write(vi, TEGRA_CSI_CSI_PIXEL_PARSER_A_INTERRUPT_MASK, 0x0);
        tegra_vi_write(vi, TEGRA_CSI_PIXEL_STREAM_A_CONTROL0, 0x280301f0);
        tegra_vi_write(vi, TEGRA_CSI_PIXEL_STREAM_PPA_COMMAND, 0xf007);
        tegra_vi_write(vi, TEGRA_CSI_PIXEL_STREAM_A_CONTROL1, 0x11);
        tegra_vi_write(vi, TEGRA_CSI_PIXEL_STREAM_A_GAP, 0x140000);
        tegra_vi_write(vi, TEGRA_CSI_PIXEL_STREAM_A_EXPECTED_FRAME, 0x0);

        /* Configure input stream (lane count) */
        tegra_vi_write(vi, TEGRA_CSI_INPUT_STREAM_A_CONTROL,
                       0x3f0000 | (lanes - 1));

        /* Set CIL PHY command for 4-lane */
        val = tegra_vi_read(vi, TEGRA_CSI_PHY_CIL_COMMAND);
        if (lanes == 4)
            tegra_vi_write(vi, TEGRA_CSI_PHY_CIL_COMMAND,
                          (val & 0xFFFF0000) | 0x0101);
        else if (lanes == 1)
            tegra_vi_write(vi, TEGRA_CSI_PHY_CIL_COMMAND,
                          (val & 0xFFFF0000) | 0x0201);
        else
            tegra_vi_write(vi, TEGRA_CSI_PHY_CIL_COMMAND,
                          (val & 0xFFFF0000) | 0x0101);

        /* Disable TPG pattern generator (we want real sensor data) */
        tegra_vi_write(vi, TEGRA_CSI_PATTERN_GENERATOR_CTRL_A, 0x0);

        /* Configure VI capture registers for channel 0 */
        /* IMAGE_DEF: bit24=use sensor (0=TPG), format in bits 23:16, bit0=enable */
        tegra_vi_write(vi, TEGRA_VI_CSI_0_IMAGE_DEF,
                      (1 << 24) | (fmt << 16) | 0x1);

        tegra_vi_write(vi, TEGRA_VI_CSI_0_CSI_IMAGE_DT, dt);

        /* IMAGE_SIZE_WC = packed bytes per line (RAW10: width * 10 / 8) */
        image_size_wc = (width * 10) >> 3;

        tegra_vi_write(vi, TEGRA_VI_CSI_0_CSI_IMAGE_SIZE_WC, image_size_wc);

        tegra_vi_write(vi, TEGRA_VI_CSI_0_CSI_IMAGE_SIZE,
                      (height << 16) | width);

        /* Start pixel parser in single shot mode */
        tegra_vi_write(vi, TEGRA_CSI_PIXEL_STREAM_PPA_COMMAND, 0xf005);

    } else if (ch == 1) {
        /* ==== Channel 1: CSI-E (port 4), pixel parser B, CIL E ==== */

        /* Power up CIL pads (CIL C+D+E for CSI-B/C group, E for our case) */
        tegra_vi_write(vi, TEGRA_CSI_CILC_PAD_CONFIG0, 0x10000);
        tegra_vi_write(vi, TEGRA_CSI_CILD_PAD_CONFIG0, 0x0);
        tegra_vi_write(vi, TEGRA_CSI_CILE_PAD_CONFIG0, 0x0);

        /* Mask CIL interrupts */
        tegra_vi_write(vi, TEGRA_CSI_CSI_CIL_C_INTERRUPT_MASK, 0x0);
        tegra_vi_write(vi, TEGRA_CSI_CSI_CIL_D_INTERRUPT_MASK, 0x0);
        tegra_vi_write(vi, TEGRA_CSI_CSI_CIL_E_INTERRUPT_MASK, 0x0);

        /* Configure PHY (only CIL E for 1-lane on CSI-E) */
        tegra_vi_write(vi, TEGRA_CSI_PHY_CILE_CONTROL0, 0x9);

        /* Configure pixel parser B */
        tegra_vi_write(vi, TEGRA_CSI_PIXEL_STREAM_PPB_COMMAND, 0xf007);
        tegra_vi_write(vi, TEGRA_CSI_CSI_PIXEL_PARSER_B_INTERRUPT_MASK, 0x0);
        tegra_vi_write(vi, TEGRA_CSI_PIXEL_STREAM_B_CONTROL0, 0x280301f1);
        tegra_vi_write(vi, TEGRA_CSI_PIXEL_STREAM_PPB_COMMAND, 0xf007);
        tegra_vi_write(vi, TEGRA_CSI_PIXEL_STREAM_B_CONTROL1, 0x11);
        tegra_vi_write(vi, TEGRA_CSI_PIXEL_STREAM_B_GAP, 0x140000);
        tegra_vi_write(vi, TEGRA_CSI_PIXEL_STREAM_B_EXPECTED_FRAME, 0x0);

        /* Configure input stream (lane count) */
        tegra_vi_write(vi, TEGRA_CSI_INPUT_STREAM_B_CONTROL,
                       0x3f0000 | (lanes - 1));

        /* Set CIL PHY command for 1-lane on CSI-E */
        val = tegra_vi_read(vi, TEGRA_CSI_PHY_CIL_COMMAND);
        tegra_vi_write(vi, TEGRA_CSI_PHY_CIL_COMMAND,
                      (val & 0x0000FFFF) | 0x12020000);

        /* Disable TPG pattern generator */
        tegra_vi_write(vi, TEGRA_CSI_PATTERN_GENERATOR_CTRL_B, 0x0);

        /* Configure VI capture registers for channel 1 */
        tegra_vi_write(vi, TEGRA_VI_CSI_1_IMAGE_DEF,
                      (1 << 24) | (fmt << 16) | 0x1);

        tegra_vi_write(vi, TEGRA_VI_CSI_1_CSI_IMAGE_DT, dt);

        image_size_wc = (width * 10) >> 3;

        tegra_vi_write(vi, TEGRA_VI_CSI_1_CSI_IMAGE_SIZE_WC, image_size_wc);

        tegra_vi_write(vi, TEGRA_VI_CSI_1_CSI_IMAGE_SIZE,
                      (height << 16) | width);

        /* Start pixel parser in single shot mode */
        tegra_vi_write(vi, TEGRA_CSI_PIXEL_STREAM_PPB_COMMAND, 0xf005);

    } else {
        pr_err("CSI setup: unsupported channel %d\n", ch);
        return -EINVAL;
    }

    pr_info("CSI channel %d configured: %dx%d, fmt=%d, dt=%d, lanes=%d\n",
            ch, width, height, fmt, dt, lanes);
    return 0;
}

/* Set up sensor format via v4l2_subdev set_fmt */
static int tegra_vi_sensor_set_format(struct tegra_vi *vi, int ch)
{
    struct v4l2_subdev *sd = vi->channels[ch].sensor_sd;
    struct v4l2_subdev_format fmt;
    u32 mbus_code;
    int ret;

    if (!sd)
        return -ENODEV;

    mbus_code = tegra_vi_v4l2_to_mbus(vi->channels[ch].format);

    memset(&fmt, 0, sizeof(fmt));
    fmt.pad = 0;
    fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
    fmt.format.width = vi->channels[ch].width;
    fmt.format.height = vi->channels[ch].height;
    fmt.format.code = mbus_code;
    fmt.format.field = V4L2_FIELD_NONE;
    fmt.format.colorspace = V4L2_COLORSPACE_SRGB;

    /* Try to set format on sensor */
    ret = v4l2_subdev_call(sd, pad, set_fmt, NULL, &fmt);
    if (ret) {
        pr_warn("Sensor set_fmt failed ch%d: %d\n", ch, ret);
        return ret;
    }

    pr_info("Sensor format set: %dx%d, code=0x%x\n",
            fmt.format.width, fmt.format.height, fmt.format.code);
    return 0;
}

/* Power on and start streaming on sensor */
static int tegra_vi_sensor_start(struct tegra_vi *vi, int ch)
{
    struct v4l2_subdev *sd = vi->channels[ch].sensor_sd;
    int ret;

    if (!sd)
        return -ENODEV;

    /* Power on if needed */
    if (!vi->channels[ch].sensor_powered) {
        ret = v4l2_subdev_call(sd, core, s_power, 1);
        if (ret && ret != -ENOIOCTLCMD) {
            pr_warn("Sensor s_power(1) failed ch%d: %d\n", ch, ret);
            return ret;
        }
        msleep(20);
    }

    /* Set format on sensor */
    ret = tegra_vi_sensor_set_format(vi, ch);
    if (ret) {
        pr_warn("Sensor set_format failed ch%d: %d\n", ch, ret);
        goto err_power;
    }

    /* Start streaming */
    ret = v4l2_subdev_call(sd, video, s_stream, 1);
    if (ret) {
        pr_warn("Sensor s_stream(1) failed ch%d: %d\n", ch, ret);
        goto err_power;
    }

    msleep(100); /* Wait for sensor to stabilize and send first frames */

    vi->channels[ch].sensor_powered = true;
    vi->channels[ch].sensor_streaming = true;
    pr_info("Sensor streaming started on channel %d\n", ch);
    return 0;

err_power:
    /* Power off so next retry will re-initialize fully */
    v4l2_subdev_call(sd, core, s_power, 0);
    return ret;
}

/* Stop streaming on sensor */
static void tegra_vi_sensor_stop(struct tegra_vi *vi, int ch)
{
    struct v4l2_subdev *sd = vi->channels[ch].sensor_sd;

    if (!sd)
        return;

    if (vi->channels[ch].sensor_streaming) {
        v4l2_subdev_call(sd, video, s_stream, 0);
        vi->channels[ch].sensor_streaming = false;
        pr_info("Sensor streaming stopped on channel %d\n", ch);
    }

    if (vi->channels[ch].sensor_powered) {
        v4l2_subdev_call(sd, core, s_power, 0);
        vi->channels[ch].sensor_powered = false;
        pr_info("Sensor power off on channel %d\n", ch);
    }

    /* Power off focuser (VCM) */
    if (vi->channels[ch].focuser_sd) {
        v4l2_subdev_call(vi->channels[ch].focuser_sd, core, s_power, 0);
        pr_info("Focuser power off on channel %d\n", ch);
    }
}

/*
 * Trigger a single-shot VI HW capture and poll for completion.
 * Instead of nvhost syncpts (cuyos eventos DMA ACK no son fiables),
 * hacemos polling del bit SINGLE_SHOT que el HW auto-limpiá
 * cuando la captura y el DMA han terminado.
 */
static int tegra_vi_single_shot_capture(struct tegra_vi *vi, int ch)
{
    u32 single_shot_reg, error_reg;
    int timeout;
    u32 val;

    if (ch == 0) {
        single_shot_reg = TEGRA_VI_CSI_0_SINGLE_SHOT;
        error_reg = TEGRA_VI_CSI_0_ERROR_STATUS;
    } else if (ch == 1) {
        single_shot_reg = TEGRA_VI_CSI_1_SINGLE_SHOT;
        error_reg = TEGRA_VI_CSI_1_ERROR_STATUS;
    } else {
        return -EINVAL;
    }

    /* Set DMA address to bounce buffer */
    if (ch == 0) {
        tegra_vi_write(vi, TEGRA_VI_CSI_0_SURFACE0_OFFSET_MSB, 0);
        tegra_vi_write(vi, TEGRA_VI_CSI_0_SURFACE0_OFFSET_LSB,
                       (u32)vi->channels[ch].bounce_buf_dma);
        tegra_vi_write(vi, TEGRA_VI_CSI_0_SURFACE0_STRIDE,
                       tegra_vi_bytes_per_line(vi->channels[ch].width,
                                               vi->channels[ch].format));
    } else {
        tegra_vi_write(vi, TEGRA_VI_CSI_1_SURFACE0_OFFSET_MSB, 0);
        tegra_vi_write(vi, TEGRA_VI_CSI_1_SURFACE0_OFFSET_LSB,
                       (u32)vi->channels[ch].bounce_buf_dma);
        tegra_vi_write(vi, TEGRA_VI_CSI_1_SURFACE0_STRIDE,
                       tegra_vi_bytes_per_line(vi->channels[ch].width,
                                               vi->channels[ch].format));
    }

    /* Limpiar estado de error antes de capturar */
    tegra_vi_write(vi, error_reg, 0xFFFFFFFF);

    /* Disparar SINGLE_SHOT */
    tegra_vi_write(vi, single_shot_reg, 1);

    /* Hacer polling de SINGLE_SHOT hasta que el HW lo auto-limpié
     * (indica que la captura y DMA han terminado).
     * Timeout de 500ms (suficiente para 640x480@15fps ~66ms por frame) */
    timeout = 500;
    while (timeout--) {
        val = tegra_vi_read(vi, single_shot_reg);
        if (!(val & 1))
            break;
        usleep_range(1000, 2000);
    }

    if (!(val & 1)) {
        val = tegra_vi_read(vi, error_reg);
        if (val)
            pr_warn_ratelimited("VI capture complete ch%d (error=0x%08x)\n", ch, val);
        return 0;
    }

    /* Timeout: verificar errores */
    val = tegra_vi_read(vi, error_reg);
    pr_warn_ratelimited("VI capture timeout ch%d (single_shot=0x%08x, error=0x%08x)\n",
                        ch, tegra_vi_read(vi, single_shot_reg), val);
    return -ETIMEDOUT;
}

/* Capture kthread - handles both sensor HW path and TPG fallback */
static int tegra_vi_capture_thread(void *data)
{
    struct tegra_vi *vi = data;
    int ch = vi->active_channel;
    u32 width = vi->channels[ch].width;
    u32 height = vi->channels[ch].height;
    u32 v4l2_fmt = vi->channels[ch].format;
    struct tegra_vi_buffer *buf;
    unsigned long flags;
    struct timeval tv;
    int ret;
    u32 bytesused;
    u8 *vaddr;
    bool use_sensor = (vi->channels[ch].sensor_sd != NULL);
    bool first_frame = true;
    int sensor_failures = 0;
    int max_sensor_failures = 5;

    pr_info("VI capture thread started (channel %d) - %s mode, buf_queue=%s, streaming=%d\n",
            ch, use_sensor ? "SENSOR HW" : "TPG fallback",
            list_empty(&vi->buf_queue) ? "EMPTY" : "HAS_BUFFERS",
            vi->streaming);

    while (!kthread_should_stop()) {
        ret = wait_event_interruptible(vi->capture_wait,
            (!list_empty(&vi->buf_queue) && vi->streaming) ||
            vi->thread_should_stop);
        if (ret == -ERESTARTSYS)
            continue;

        if (vi->thread_should_stop)
            break;

        if (!vi->streaming)
            continue;

        spin_lock_irqsave(&vi->slock, flags);
        if (list_empty(&vi->buf_queue)) {
            spin_unlock_irqrestore(&vi->slock, flags);
            continue;
        }
        buf = list_first_entry(&vi->buf_queue, struct tegra_vi_buffer, list);
        list_del(&buf->list);
        spin_unlock_irqrestore(&vi->slock, flags);

        buf->state = TEGRA_VI_BUF_STATE_ACTIVE;

        vaddr = vb2_plane_vaddr(&buf->vb, 0);
        if (!vaddr) {
            pr_warn_ratelimited("No vaddr for buffer on channel %d\n", ch);
            vi->frame_errors++;
            vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
            usleep_range(10000, 20000);
            continue;
        }

        bytesused = width * height * tegra_vi_format_depth(v4l2_fmt);

        if (use_sensor && !vi->channels[ch].sensor_dead &&
            sensor_failures < max_sensor_failures) {
            /* ===== SENSOR (HW) PATH ===== */
		if (first_frame) {
			/* Power on focuser (VCM) FIRST so it settles during sensor init */
			if (vi->channels[ch].focuser_sd) {
				struct v4l2_subdev *focuser = vi->channels[ch].focuser_sd;
				struct v4l2_ctrl *ctrl;
				v4l2_subdev_call(focuser, core, s_power, 1);
				ctrl = v4l2_ctrl_find(focuser->ctrl_handler, V4L2_CID_FOCUS_ABSOLUTE);
				if (ctrl) {
					v4l2_ctrl_s_ctrl(ctrl, 400);
					pr_info("Focuser set to position 400 on channel %d\n", ch);
				}
				msleep(30); /* Let VCM start moving - sensor_start's 100ms delay finishes settling */
			}

			ret = tegra_vi_sensor_start(vi, ch);
			if (ret) {
				/* Power off focuser since sensor failed */
				if (vi->channels[ch].focuser_sd)
					v4l2_subdev_call(vi->channels[ch].focuser_sd, core, s_power, 0);
				pr_warn("Sensor start failed, falling back to TPG: %d\n", ret);
				sensor_failures = max_sensor_failures;
				vi->channels[ch].sensor_dead = true;
				goto do_tpg;
			}

                /* Configurar CSI PHY + pixel parser + VI capture registers */
                ret = tegra_vi_setup_csi_channel(vi, ch);
                if (ret) {
                    pr_warn("CSI setup failed ch%d: %d, falling back to TPG\n",
                            ch, ret);
                    sensor_failures = max_sensor_failures;
                    vi->channels[ch].sensor_dead = true;
                    goto do_tpg;
                }
                vi->channels[ch].csi_configured = true;

                /* Allocar DMA bounce buffer para VI HW capture */
                ret = tegra_vi_channel_alloc_bounce(vi, ch);
                if (ret) {
                    pr_warn("Bounce buffer alloc failed ch%d: %d, falling back to TPG\n",
                            ch, ret);
                    sensor_failures = max_sensor_failures;
                    vi->channels[ch].sensor_dead = true;
                    goto do_tpg;
                }

                /* MIPI calibration (solo una vez) — necesaria para pixel parser */
                ret = tegra_vi_mipi_calibration(vi, ch);
                if (ret)
                    pr_warn("MIPI cal failed ch%d: %d, continuing anyway\n", ch, ret);

                first_frame = false;
                pr_info("Sensor HW streaming started on channel %d\n", ch);
            }

            /* Captura real via VI HW con nvhost syncpts */
            ret = tegra_vi_single_shot_capture(vi, ch);
            if (ret) {
                pr_warn_ratelimited("VI capture failed ch%d: %d, falling back to TPG\n",
                                    ch, ret);
                sensor_failures = max_sensor_failures;
                vi->channels[ch].sensor_dead = true;
                goto do_tpg;
            }

            /* Copiar datos del bounce buffer al buffer USERPTR */
            memcpy(vaddr, vi->channels[ch].bounce_buf_cpu, bytesused);
        } else {
do_tpg:
            /* ===== TPG FALLBACK PATH ===== */
            ret = tegra_tpg_fill_buffer(vi->tpg, vaddr, width, height);
            if (ret) {
                pr_warn_ratelimited("TPG fill failed on channel %d: %d\n", ch, ret);
                vi->frame_errors++;
                vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
                usleep_range(10000, 20000);
                continue;
            }
        }

        do_gettimeofday(&tv);
        buf->vb.v4l2_buf.timestamp.tv_sec = tv.tv_sec;
        buf->vb.v4l2_buf.timestamp.tv_usec = tv.tv_usec;
        vb2_set_plane_payload(&buf->vb, 0, bytesused);
        buf->state = TEGRA_VI_BUF_STATE_DONE;
        buf->sequence = vi->sequence++;
        buf->vb.v4l2_buf.field = V4L2_FIELD_NONE;
        buf->vb.v4l2_buf.sequence = buf->sequence;
        vb2_buffer_done(&buf->vb, VB2_BUF_STATE_DONE);
        vi->frame_count++;
    }

    /* Stop sensor if streaming */
    if (use_sensor && vi->channels[vi->active_channel].sensor_streaming)
        tegra_vi_sensor_stop(vi, vi->active_channel);

    /* Disable CSI pixel parser */
    {
        int c;
        for (c = 0; c < vi->num_channels; c++) {
            if (vi->channels[c].csi_configured) {
                if (c == 0)
                    tegra_vi_write(vi, TEGRA_CSI_PIXEL_STREAM_PPA_COMMAND, 0xf002);
                else if (c == 1)
                    tegra_vi_write(vi, TEGRA_CSI_PIXEL_STREAM_PPB_COMMAND, 0xf002);
                vi->channels[c].csi_configured = false;
            }
        }
    }

    pr_info("VI capture thread stopped (frames=%d, errors=%d)\n",
            vi->frame_count, vi->frame_errors);
    return 0;
}

/* Start streaming */
static int tegra_vi_start_streaming(struct vb2_queue *vq, unsigned int count)
{
    struct tegra_vi *vi = vb2_get_drv_priv(vq);
    int ch = vi->active_channel;
    bool use_sensor = (vi->channels[ch].sensor_sd != NULL);

    pr_info("Starting streaming on channel %d (%dx%d, %d buffers, %s)\n",
            ch, vi->channels[ch].width, vi->channels[ch].height, count,
            use_sensor ? "SENSOR" : "TPG");

    vi->streaming = true;
    vi->sequence = 0;
    vi->frame_count = 0;
    vi->frame_errors = 0;
    init_waitqueue_head(&vi->capture_wait);
    vi->thread_should_stop = false;

    vi->capture_thread = kthread_run(tegra_vi_capture_thread, vi,
                                      "vi-capture-%d", ch);
    if (IS_ERR(vi->capture_thread)) {
        int err = PTR_ERR(vi->capture_thread);
        pr_err("Failed to create capture thread: %d\n", err);
        vi->capture_thread = NULL;
        vi->streaming = false;
        return err;
    }

    pr_info("Streaming started on channel %d (%s)\n",
            ch, use_sensor ? "SENSOR" : "TPG");
    return 0;
}

/* Stop streaming */
static int tegra_vi_stop_streaming(struct vb2_queue *vq)
{
    struct tegra_vi *vi = vb2_get_drv_priv(vq);
    int ch = vi->active_channel;
    struct tegra_vi_buffer *buf, *tmp;
    unsigned long flags;
    LIST_HEAD(local_list);

    vi->streaming = false;

    /* Stop capture kthread */
    if (vi->capture_thread) {
        vi->thread_should_stop = true;
        wake_up_interruptible(&vi->capture_wait);
        kthread_stop(vi->capture_thread);
        vi->capture_thread = NULL;
    }

    /* Return all queued buffers */
    spin_lock_irqsave(&vi->slock, flags);
    list_splice_init(&vi->buf_queue, &local_list);
    spin_unlock_irqrestore(&vi->slock, flags);

    list_for_each_entry_safe(buf, tmp, &local_list, list) {

        vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
    }

    pr_info("Streaming stopped (channel %d)\n", ch);
    return 0;
}

/* File operations */
static const struct v4l2_file_operations tegra_vi_fops = {
    .owner          = THIS_MODULE,
    .open           = v4l2_fh_open,
    .release        = vb2_fop_release,
    .unlocked_ioctl = video_ioctl2,
    .mmap           = vb2_fop_mmap,
    .poll           = vb2_fop_poll,
};

/* Video device template - no device_caps in kernel 3.10 */
static struct video_device tegra_vi_video_device_template = {
    .name          = "tegra-vi",
    .fops          = &tegra_vi_fops,
    .ioctl_ops     = &tegra_vi_ioctl_ops,
    .release       = video_device_release,
    .vfl_dir       = VFL_DIR_RX,
};

/* Parse device tree */
static int tegra_vi_parse_dt(struct tegra_vi *vi)
{
    struct device_node *np = vi->dev->of_node;
    int ret;

    if (!np)
        return -ENODEV;

    ret = of_property_read_u32(np, "num-channels", &vi->num_channels);
    if (ret)
        vi->num_channels = 2;

    return 0;
}

/* Platform driver probe */
static int tegra_vi_probe(struct platform_device *pwd)
{
    struct tegra_vi *vi;
    struct resource *res;
    bool venc_powered = false;
    int ret;

    vi = devm_kzalloc(&pwd->dev, sizeof(*vi), GFP_KERNEL);
    if (!vi)
        return -ENOMEM;

    vi->dev = &pwd->dev;
    mutex_init(&vi->lock);
    spin_lock_init(&vi->slock);
    INIT_LIST_HEAD(&vi->buf_queue);
    INIT_LIST_HEAD(&vi->sensor_pending_list);
    init_waitqueue_head(&vi->capture_wait);
    vi->capture_thread = NULL;
    vi->thread_should_stop = false;
    vi->frame_count = 0;
    INIT_DELAYED_WORK(&vi->sensor_scan_work, tegra_vi_sensor_scan_work);

    res = platform_get_resource(pwd, IORESOURCE_MEM, 0);
    if (!res) {
        dev_err(&pwd->dev, "Failed to get memory resource\n");
        return -ENODEV;
    }

    vi->base = devm_ioremap(&pwd->dev, res->start, resource_size(res));
    if (!vi->base) {
        dev_err(&pwd->dev, "Failed to map VI registers\n");
        return -ENOMEM;
    }

    /* Set DMA mask for coherent allocations (needed if switching back to dma-contig) */
    if (!dma_set_mask(&pwd->dev, DMA_BIT_MASK(32)))
        dma_set_coherent_mask(&pwd->dev, DMA_BIT_MASK(32));

    ret = tegra_vi_parse_dt(vi);
    if (ret) {
        dev_err(&pwd->dev, "Failed to parse device tree: %d\n", ret);
        return ret;
    }

    /* Set g_vi early so sensor probes can call tegra_vi_register_sensor() */
    g_vi = vi;

    /* Register V4L2 device first */
    ret = v4l2_device_register(&pwd->dev, &vi->v4l2_dev);
    if (ret) {
        dev_err(&pwd->dev, "Failed to register V4L2 device: %d\n", ret);
        g_vi = NULL;
        return ret;
    }

    /* Habilitar clocks del pipeline VI+CSI */
    vi->clk_vi = devm_clk_get(&pwd->dev, "vi");
    vi->clk_csi = devm_clk_get(&pwd->dev, "csi");
    vi->clk_csus = devm_clk_get(&pwd->dev, "csus");
    vi->clk_cilab = devm_clk_get(&pwd->dev, "cilab");
    vi->clk_cile = devm_clk_get(&pwd->dev, "cile");

    if (!IS_ERR_OR_NULL(vi->clk_vi)) {
        if (clk_prepare_enable(vi->clk_vi))
            dev_err(&pwd->dev, "Failed to enable vi clock\n");
    } else {
        dev_dbg(&pwd->dev, "Clock vi not available\n");
    }

    if (!IS_ERR_OR_NULL(vi->clk_csi)) {
        if (clk_prepare_enable(vi->clk_csi))
            dev_err(&pwd->dev, "Failed to enable csi clock\n");
    } else {
        dev_dbg(&pwd->dev, "Clock csi not available\n");
    }

    if (!IS_ERR_OR_NULL(vi->clk_csus)) {
        if (clk_prepare_enable(vi->clk_csus))
            dev_err(&pwd->dev, "Failed to enable csus clock\n");
    } else {
        dev_dbg(&pwd->dev, "Clock csus not available\n");
    }

    if (!IS_ERR_OR_NULL(vi->clk_cilab)) {
        if (clk_prepare_enable(vi->clk_cilab))
            dev_err(&pwd->dev, "Failed to enable cilab clock\n");
    } else {
        dev_dbg(&pwd->dev, "Clock cilab not available\n");
    }

    if (!IS_ERR_OR_NULL(vi->clk_cile)) {
        if (clk_prepare_enable(vi->clk_cile))
            dev_err(&pwd->dev, "Failed to enable cile clock\n");
    } else {
        dev_dbg(&pwd->dev, "Clock cile not available\n");
    }

    /* Power up VENC partition (contains VI + CSI + ISP). VI and CSI
     * are in the same power domain; without this, register access
     * returns "Host read timeout" because the hardware is powered down.
     * ISP manages this automatically via nvhost, but our driver does not. */
    ret = tegra_unpowergate_partition(TEGRA_POWERGATE_VENC);
    if (ret)
        dev_warn(&pwd->dev, "VENC unpowergate failed: %d, CSI/VI may timeout\n", ret);
    else {
        venc_powered = true;
        dev_info(&pwd->dev, "VENC partition powered on\n");
    }

    msleep(20);  /* Allow power / PLL to stabilize */

    /* Initialize Media Controller - NUEVO */
    ret = tegra_vi_media_init(vi);
    if (ret) {
        dev_warn(&pwd->dev, "Media init failed: %d, continuing without MC\n", ret);
    }

    /* Construir lista de sensores esperados desde DT (async discovery) */
    tegra_vi_build_sensor_list(vi);

    /* Intentar encontrar sensores sincrónicamente (si I2C ya probearon) */
    ret = tegra_vi_find_sensors(vi);
    if (ret)
        dev_warn(&pwd->dev, "No sensors found at probe time\n");

    /* Crear Media Controller links para sensores encontrados sincrónicamente */
    if (vi->channels[0].sensor_sd || vi->channels[1].sensor_sd) {
        ret = tegra_vi_create_links(vi);
        if (ret)
            dev_warn(&pwd->dev, "Links creation failed: %d\n", ret);
    }

    /* Registrar TPG como fallback siempre (útil para debug) */
    dev_info(&pwd->dev, "Registering TPG fallback\n");
    ret = tegra_tpg_register(&vi->v4l2_dev, &vi->tpg);
    if (ret) {
        dev_err(&pwd->dev, "Failed to register TPG: %d\n", ret);
    } else {
        dev_info(&pwd->dev, "TPG registered as fallback\n");
    }

    /* Si no hay sensores ahora, iniciar scan async para cuando aparezcan */
    if (!vi->channels[0].sensor_sd && !vi->channels[1].sensor_sd &&
        vi->num_expected_sensors > 0) {
        vi->sensor_scan_retries = 0;
        vi->sensor_scan_done = false;
        schedule_delayed_work(&vi->sensor_scan_work,
                              msecs_to_jiffies(SENSOR_SCAN_INTERVAL_MS));
        dev_info(&pwd->dev, "Async sensor scan started (%d expected)\n",
                 vi->num_expected_sensors);
    } else {
        vi->sensor_scan_done = true;
    }

    vi->queue.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    vi->queue.io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
    vi->queue.drv_priv = vi;
    vi->queue.buf_struct_size = sizeof(struct tegra_vi_buffer);
    vi->queue.ops = &tegra_vi_vb2_ops;
    vi->queue.mem_ops = &vb2_vmalloc_memops;
    vi->queue.timestamp_type = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    vi->queue.lock = &vi->lock;

    ret = vb2_queue_init(&vi->queue);
    if (ret) {
        dev_err(&pwd->dev, "Failed to init VB2 queue: %d\n", ret);
        goto err_media;
    }

    vi->csi = devm_kzalloc(&pwd->dev, sizeof(*vi->csi), GFP_KERNEL);
    if (!vi->csi) {
        ret = -ENOMEM;
        goto err_media;
    }

    vi->csi->base = devm_ioremap(&pwd->dev, TEGRA_CSI_BASE, TEGRA_CSI_SIZE);
    if (!vi->csi->base) {
        dev_err(&pwd->dev, "Failed to map CSI registers\n");
        ret = -ENOMEM;
        goto err_media;
    }

    /* Pequeño delay para estabilizar clocks/power antes de acceder a CSI */
    msleep(100);

    ret = tegra_csi_init(vi->csi);
    if (ret) {
        dev_err(&pwd->dev, "Failed to initialize CSI: %d\n", ret);
        goto err_csi;
    }

    /* Connect CSI to Media Controller - NUEVO */
    ret = tegra_csi_set_media_device(vi->csi, &vi->mdev);
    if (ret)
        dev_warn(&pwd->dev, "CSI media device setup failed: %d\n", ret);

    vi->active_channel = 0;

    /* Initialize default formats and create video device per channel */
    {
        int num_registered = 0;
        int ch;

        for (ch = 0; ch < vi->num_channels; ch++) {
            struct tegra_vi_vdev_data *vdata;

            vi->channels[ch].width = 640;
            vi->channels[ch].height = 480;
            vi->channels[ch].format = V4L2_PIX_FMT_SRGGB10;

            /* Allocate nvhost syncpt for VI HW capture */
            vi->channels[ch].syncpt_id = nvhost_get_syncpt_client_managed("vi_ch");
            if (vi->channels[ch].syncpt_id) {
                vi->channels[ch].syncpt_initialized = true;
                vi->channels[ch].syncpt_thresh = 0;
                dev_info(&pwd->dev, "Channel %d: allocated syncpt id=%d\n",
                         ch, vi->channels[ch].syncpt_id);
            } else {
                dev_warn(&pwd->dev, "Channel %d: no syncpt available\n", ch);
                vi->channels[ch].syncpt_initialized = false;
            }

            vi->channels[ch].vdev = video_device_alloc();
            if (!vi->channels[ch].vdev) {
                ret = -ENOMEM;
                goto err_vdev;
            }

            *vi->channels[ch].vdev = tegra_vi_video_device_template;
            snprintf(vi->channels[ch].vdev->name, sizeof(vi->channels[ch].vdev->name),
                     "tegra-vi-%d", ch);
            vi->channels[ch].vdev->lock = &vi->lock;
            vi->channels[ch].vdev->v4l2_dev = &vi->v4l2_dev;
            vi->channels[ch].vdev->queue = &vi->queue;

            vdata = kzalloc(sizeof(*vdata), GFP_KERNEL);
            if (!vdata) {
                video_device_release(vi->channels[ch].vdev);
                vi->channels[ch].vdev = NULL;
                ret = -ENOMEM;
                goto err_vdev;
            }
            vdata->vi = vi;
            vdata->channel = ch;
            video_set_drvdata(vi->channels[ch].vdev, vdata);

            ret = video_register_device(vi->channels[ch].vdev, VFL_TYPE_GRABBER, ch);
            if (ret) {
                dev_err(&pwd->dev, "Failed to register video device %d: %d\n", ch, ret);
                kfree(vdata);
                video_device_release(vi->channels[ch].vdev);
                vi->channels[ch].vdev = NULL;
                goto err_vdev;
            }
            num_registered++;

            dev_info(&pwd->dev, "Registered /dev/video%d (channel %d)\n",
                     vi->channels[ch].vdev->num, ch);
        }

        /* Register IRQ handler */
        ret = devm_request_irq(&pwd->dev, 69, tegra_vi_irq_handler,
                                 0, dev_name(&pwd->dev), vi);
        if (ret) {
            dev_err(&pwd->dev, "Failed to request IRQ: %d\n", ret);
            goto err_vdev;
        }

        platform_set_drvdata(pwd, vi);

        dev_info(&pwd->dev, "Tegra VI probed (%d channels, IRQ 69 registered)\n", vi->num_channels);
        return 0;

err_vdev:
        /* Clean up registered video devices */
        for (ch = 0; ch < num_registered; ch++) {
            struct tegra_vi_vdev_data *vdata = video_get_drvdata(vi->channels[ch].vdev);
            kfree(vdata);
            video_unregister_device(vi->channels[ch].vdev);
        }
        /* Clean up the current (failed) vdev if it was allocated but not registered */
        if (ch < vi->num_channels && vi->channels[ch].vdev) {
            struct tegra_vi_vdev_data *vdata = video_get_drvdata(vi->channels[ch].vdev);
            kfree(vdata);
            video_device_release(vi->channels[ch].vdev);
            vi->channels[ch].vdev = NULL;
        }
    }
    /* Free syncpts for all channels on error */
    {
        int i;
        for (i = 0; i < vi->num_channels; i++) {
        if (vi->channels[i].syncpt_initialized) {
            nvhost_free_syncpt(vi->channels[i].syncpt_id);
            vi->channels[i].syncpt_initialized = false;
        }
    }
}
err_csi:
err_media:
    cancel_delayed_work_sync(&vi->sensor_scan_work);
    if (vi->tpg) {
        tegra_tpg_unregister(vi->tpg);
        vi->tpg = NULL;
    }
    if (vi->csi)
        tegra_csi_media_cleanup(vi->csi);
    if (!IS_ERR_OR_NULL(vi->clk_vi))
        clk_disable_unprepare(vi->clk_vi);
    if (!IS_ERR_OR_NULL(vi->clk_csi))
        clk_disable_unprepare(vi->clk_csi);
    if (!IS_ERR_OR_NULL(vi->clk_csus))
        clk_disable_unprepare(vi->clk_csus);
    if (!IS_ERR_OR_NULL(vi->clk_cilab))
        clk_disable_unprepare(vi->clk_cilab);
    if (!IS_ERR_OR_NULL(vi->clk_cile))
        clk_disable_unprepare(vi->clk_cile);
    if (venc_powered)
        tegra_powergate_partition(TEGRA_POWERGATE_VENC);
    if (vi->entity_registered)
        media_entity_cleanup(&vi->entity);
    if (vi->media_registered)
        media_device_unregister(&vi->mdev);
    v4l2_device_unregister(&vi->v4l2_dev);
    g_vi = NULL;
    return ret;
}


/* Platform driver remove */
static int tegra_vi_remove(struct platform_device *pwd)
{
    struct tegra_vi *vi = platform_get_drvdata(pwd);
    int i;

    if (!vi)
        return 0;

    for (i = 0; i < vi->num_channels; i++) {
        if (vi->channels[i].vdev) {
            struct tegra_vi_vdev_data *vdata = video_get_drvdata(vi->channels[i].vdev);
            kfree(vdata);
            video_unregister_device(vi->channels[i].vdev);
            vi->channels[i].vdev = NULL;
        }
        /* Free nvhost syncpt */
        if (vi->channels[i].syncpt_initialized) {
            nvhost_free_syncpt(vi->channels[i].syncpt_id);
            vi->channels[i].syncpt_initialized = false;
        }
        /* Free DMA bounce buffer */
        tegra_vi_channel_free_bounce(vi, i);
    }

    /* Disable pipeline clocks */
    if (!IS_ERR_OR_NULL(vi->clk_vi))
        clk_disable_unprepare(vi->clk_vi);
    if (!IS_ERR_OR_NULL(vi->clk_csi))
        clk_disable_unprepare(vi->clk_csi);
    if (!IS_ERR_OR_NULL(vi->clk_csus))
        clk_disable_unprepare(vi->clk_csus);
    if (!IS_ERR_OR_NULL(vi->clk_cilab))
        clk_disable_unprepare(vi->clk_cilab);
    if (!IS_ERR_OR_NULL(vi->clk_cile))
        clk_disable_unprepare(vi->clk_cile);

    /* Power down VENC partition (balanced with unpowergate in probe) */
    tegra_powergate_partition(TEGRA_POWERGATE_VENC);

    /* Cleanup CSI media entities */
    if (vi->csi)
        tegra_csi_media_cleanup(vi->csi);

    /* Cancelar async sensor scan */
    cancel_delayed_work_sync(&vi->sensor_scan_work);

    /* Limpiar lista de sensores pendientes */
    {
        struct tegra_vi_sensor_entry *entry, *tmp;
        list_for_each_entry_safe(entry, tmp, &vi->sensor_pending_list, list) {
            tegra_vi_sensor_entry_free(entry);
        }
    }

    /* Cleanup Media Controller - NUEVO */
    if (vi->entity_registered)
        media_entity_cleanup(&vi->entity);
    if (vi->media_registered)
        media_device_unregister(&vi->mdev);

    /* Cleanup TPG if registered */
    if (vi->tpg) {
        tegra_tpg_unregister(vi->tpg);
        vi->tpg = NULL;
    }

    /* Unregister V4L2 device */
    v4l2_device_unregister(&vi->v4l2_dev);
    g_vi = NULL;

    return 0;
}

static const struct of_device_id tegra_vi_of_match[] = {
    { .compatible = "nvidia,mocha-vi" },
    { .compatible = "nvidia,tegra124-vi" },  /* fallback for compatibility */
    { }
};
MODULE_DEVICE_TABLE(of, tegra_vi_of_match);

static struct platform_driver tegra_vi_driver = {
    .probe  = tegra_vi_probe,
    .remove = tegra_vi_remove,
    .driver = {
        .name           = TEGRA_VI_DRIVER_NAME,
        .of_match_table = tegra_vi_of_match,
    },
};
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Dargons10 <dargons10@users.noreply.github.com>");
MODULE_DESCRIPTION("Tegra Video Input (VI) V4L2 capture driver");
module_platform_driver(tegra_vi_driver);
