#!/bin/bash

LOCAL_PATH=$(pwd)

KERNEL_PACKAGE=linux-4.19.125.tar.gz
KERNEL_PATH=

if [ $# -ge 1 ]; then
    KERNEL_PATH=$1
    if [ ! -f $KERNEL_PATH ]; then
        echo "[ERROR] $KERNEL_PATH does not exist, please input $KERNEL_PACKAGE absolute path"
        exit 1
    fi
    result=$(echo $KERNEL_PATH | grep "${KERNEL_PACKAGE}")
    if [ "$result" == "" ]; then
        echo "[ERROR] please input $KERNEL_PACKAGE absolute path"
        exit 1
    fi
fi

echo "kernel path $KERNEL_PATH ..."


echo "unpacking build"
mkdir -p build/
tar -xvzf package/build.tgz

echo "unpacking tools"
mkdir -p tools/
tar -xvzf package/tools.tgz

echo "unpacking app"
mkdir -p app/
tar -xvzf package/app.tgz

echo "unpacking rootfs"
mkdir -p rootfs
tar -xvzf package/rootfs.tgz
tar -xvzf rootfs/rootfs.tar -C rootfs/
rm -f rootfs/rootfs.tar

echo "unpacking boot"
mkdir -p boot
tar -xvzf package/boot.tgz
cd boot
mkdir -p bl1
tar -xvzf bl1.tgz
rm -f bl1.tgz

mkdir -p uboot
tar -xvzf uboot.tgz
rm -f uboot.tgz
cd $LOCAL_PATH

echo "unpacking kernel"
mkdir -p kernel/
tar -xvzf package/kernel.tgz
mkdir -p kernel/osdrv
cd kernel
tar -xvzf osdrv.tgz
rm -f osdrv.tgz
cd linux

if [ ! -n "$KERNEL_PATH" ]; then
    echo "download kernel linux-4.19.125 ..."
    #curl -O https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/snapshot/linux-4.19.125.tar.gz
    wget -t 5 https://mirror.tuna.tsinghua.edu.cn/kernel/v4.x/linux-4.19.125.tar.gz --no-check-certificate
    if [[ $? -ne 0 ]]; then
        echo "download linux kernel failed, please retry..."
        exit 1;
    fi
else
    cp -f $KERNEL_PATH ./
fi

tar -xzf linux-4.19.125.tar.gz
echo "patch linux-4.19.125 ..."
patch -Np1 < linux-4.19.125.patch
cd $LOCAL_PATH

echo "unpacking msp"
mkdir -p msp/
tar -xvzf package/msp.tgz

echo "unpacking third-party"
mkdir -p third-party/
tar -xvzf package/third-party.tgz

echo "unpacking osal"
mkdir -p osal/
tar -xvzf package/osal.tgz

echo "unpack finished"
