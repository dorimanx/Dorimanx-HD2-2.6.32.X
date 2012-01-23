cp .config .config.bkp
make ARCH=arm CROSS_COMPILE=/root/CodeSourcery/Sourcery_G++_Lite/bin/arm-none-linux-gnueabi- mrproper
cp .config.bkp .config

