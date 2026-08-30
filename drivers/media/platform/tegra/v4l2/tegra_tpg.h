#ifndef __TEGRA_TPG_H__
#define __TEGRA_TPG_H__

#include <linux/types.h>

/* Forward declarations */
struct tegra_tpg;
struct v4l2_device;

int tegra_tpg_register(struct v4l2_device *v4l2_dev, struct tegra_tpg **tpg_out);
void tegra_tpg_unregister(struct tegra_tpg *tpg);
int tegra_tpg_fill_buffer(struct tegra_tpg *tpg, u8 *buf, u32 width, u32 height);

#endif /* __TEGRA_TPG_H__ */
