#! /bin/sh

export PATH="/home/development/freescale/usr/local/gcc-4.6.2-glibc-2.13-linaro-multilib-2011.12/fsl-linaro-toolchain/bin:$PATH"

make -j4 ARCH=arm CROSS_COMPILE=arm-fsl-linux-gnueabi- uImage
