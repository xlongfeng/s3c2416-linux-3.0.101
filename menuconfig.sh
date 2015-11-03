#! /bin/sh

make ARCH=arm menuconfig
cp .config ../config/platform/imx/imx6_defconfig.dev
