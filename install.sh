#! /bin/sh

export PATH="/home/development/freescale/usr/local/gcc-4.6.2-glibc-2.13-linaro-multilib-2011.12/fsl-linaro-toolchain/bin:$PATH"

RPM_BUILD_ROOT=/home/development/samsung/kernel.install

make ARCH=arm CROSS_COMPILE=arm-fsl-linux-gnueabi- DEPMOD=/bin/true INSTALL_MOD_PATH=$RPM_BUILD_ROOT modules_install

KERNEL_VER=`ls $RPM_BUILD_ROOT/lib/modules`

for i in build source
do
	rm -f $RPM_BUILD_ROOT/lib/modules/$KERNEL_VER/$i
	ln -s /usr/src/linux $RPM_BUILD_ROOT/lib/modules/$KERNEL_VER/$i
done
