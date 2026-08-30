#
# Makefile for NVIDIA Tegra V4L2 Camera Drivers
#

obj-$(CONFIG_VIDEO_TEGRA_VI_V4L2) += tegra_vi.o tegra_csi.o tegra_tpg.o
obj-$(CONFIG_VIDEO_TEGRA_VI_V4L2) += sensors/

ccflags-y += -Idrivers/media/platform/tegra/v4l2
ccflags-y += -Idrivers/video/tegra/host
ccflags-y += -Idrivers/video/tegra/host/host1x
