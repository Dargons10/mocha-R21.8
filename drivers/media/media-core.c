/*
 * media-core.c - Media Controller functions for kernel 3.10
 *
 * Copyright (c) 2026, Dargons10
 *
 * Implements missing Media Controller functions
 */

#include <media/media-device.h>
#include <media/media-entity.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>

void media_device_init(struct media_device *mdev)
{
	if (!mdev)
		return;

	INIT_LIST_HEAD(&mdev->entities);
	mdev->entity_id = 0;
	mutex_init(&mdev->graph_mutex);
}
EXPORT_SYMBOL_GPL(media_device_init);

void media_device_cleanup(struct media_device *mdev)
{
	if (!mdev)
		return;

	media_device_unregister(mdev);
}
EXPORT_SYMBOL_GPL(media_device_cleanup);

int media_create_pad_link(struct media_entity *source, u16 source_pad,
			  struct media_entity *sink, u16 sink_pad, u32 flags)
{
	return media_entity_create_link(source, source_pad, sink, sink_pad, flags);
}
EXPORT_SYMBOL_GPL(media_create_pad_link);