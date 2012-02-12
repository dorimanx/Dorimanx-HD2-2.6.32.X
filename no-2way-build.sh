#!/bin/sh
rm -f arch/arm/boot/zImage
rm -f /home/root/kernel/dorimanx-tytyng/output_gb/boot/zImage
rm -f  /home/root/kernel/dorimanx-tytyng/output_gb/boot/boot.img
rm -rf /home/root/kernel/dorimanx-tytyng/output_gb/system/lib/modules/*
rm -rf /home/root/kernel/dorimanx-tytyng/output_gb/devs/*
make ARCH=arm CROSS_COMPILE=/root/CodeSourcery-2010.09/arm-2010.09/bin/arm-none-eabi- zImage -j4
#make ARCH=arm CROSS_COMPILE=/root/CodeSourcery/Sourcery_G++_Lite/bin/arm-none-linux-gnueabi- zImage -j4
if [ -f arch/arm/boot/zImage ]; then
if [ ! -d /home/root/kernel/dorimanx-tytyng/output_gb ]; then
mkdir /home/root/kernel/
mkdir /home/root/kernel/dorimanx-tytyng/
mkdir /home/root/kernel/dorimanx-tytyng/output_gb/
mkdir /home/root/kernel/dorimanx-tytyng/output_gb/boot/
mkdir /home/root/kernel/dorimanx-tytyng/output_gb/devs
mkdir /home/root/kernel/dorimanx-tytyng/backup
mkdir /home/root/kernel/dorimanx-tytyng/system
mkdir /home/root/kernel/dorimanx-tytyng/system/lib
mkdir /home/root/kernel/dorimanx-tytyng/system/lib/modules
fi
cp /home/root/kernel/dorimanx-tytyng/output_gb/Kernel_Dorimanx-V*-2way-rec-OFF.zip /home/root/kernel/dorimanx-tytyng/backup
rm -f /home/root/kernel/dorimanx-tytyng/output_gb/Kernel_Dorimanx-V*-2way-rec-OFF.zip
cp arch/arm/boot/zImage /home/root/kernel/dorimanx-tytyng/output_gb/boot/zImage
#mkbootimg --kernel /home/root/kernel/dorimanx-tytyng/output_gb/boot/zImage --ramdisk /home/root/kernel/dorimanx-tytyng/output_gb/boot/initrd.gz --cmdline "console=null" --base 0x11800000 -o /home/root/kernel/dorimanx-tytyng/output_gb/boot.img
cp .config /home/root/kernel/dorimanx-tytyng/output_gb/devs/config
cp .config arch/arm/configs/htcleo_defconfig
cp arch/arm/configs/htcleo_defconfig /home/root/kernel/dorimanx-tytyng/output_gb/devs/htcleo_defconfig
rm -rf /home/root/kernel/dorimanx-tytyng/output_gb/system/lib/modules/*
make ARCH=arm CROSS_COMPILE=/root/CodeSourcery-2010.09/arm-2010.09/bin/arm-none-eabi- modules -j4
make ARCH=arm CROSS_COMPILE=/root/CodeSourcery-2010.09/arm-2010.09/bin/arm-none-eabi- INSTALL_MOD_PATH=/home/root/kernel/dorimanx-tytyng/output_gb/system/lib/modules modules_install -j4
#make ARCH=arm CROSS_COMPILE=/root/CodeSourcery/Sourcery_G++_Lite/bin/arm-none-linux-gnueabi- modules
#make ARCH=arm CROSS_COMPILE=/root/CodeSourcery/Sourcery_G++_Lite/bin/arm-none-linux-gnueabi- INSTALL_MOD_PATH=/home/root/kernel/dorimanx-tytyng/output_gb/system/lib/modules modules_install
cd /home/root/kernel/dorimanx-tytyng/output_gb/system/lib/modules
find -iname *.ko | xargs -i -t cp {} .
rm -rf /home/root/kernel/dorimanx-tytyng/output_gb/system/lib/modules/lib
stat /home/root/kernel/dorimanx-tytyng/output_gb/boot/zImage
cd /home/root/kernel/dorimanx-tytyng/output_gb
mv /home/root/kernel/dorimanx-tytyng/output_gb/Kernel_Dorimanx-V1.*-2way-rec-ON.zip /home/root/kernel/dorimanx-tytyng/
zip -r Kernel_Dorimanx-V1.`date +"%H-%M-%d%m%y"`-2way-rec-OFF.zip .
mv /home/root/kernel/dorimanx-tytyng/Kernel_Dorimanx-V1.*-2way-rec-ON.zip /home/root/kernel/dorimanx-tytyng/output_gb/
else
echo "Kernel STUCK in BUILD! no zImage exist"
fi

