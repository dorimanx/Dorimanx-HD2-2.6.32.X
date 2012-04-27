KERNEL_NAME=v66CHWA
KERNEL_BOOT=$KERNEL_NAME/boot
KERNEL_MODULES=$KERNEL_NAME/system/lib/modules

echo $KERNEL_NAME $KERNEL_BOOT $KERNEL_MODULES

rm -rf ../$KERNEL_NAME
mkdir -p ../$KERNEL_BOOT
mkdir -p ../$KERNEL_MODULES


cp  drivers/net/kineto_gan.ko                       ../$KERNEL_MODULES/
cp  drivers/net/msm_rmnet.ko                        ../$KERNEL_MODULES/
cp  drivers/net/tun.ko                              ../$KERNEL_MODULES/
cp  drivers/net/wireless/bcm4329/bcm4329.ko         ../$KERNEL_MODULES/
cp  fs/reiserfs/reiserfs.ko                         ../$KERNEL_MODULES/
cp  drivers/staging/zram/zram.ko                    ../$KERNEL_MODULES/
cp  sound/htcleo/alsa-mix-htc-leo.ko                ../$KERNEL_MODULES/
cp  fs/cifs/cifs.ko                                 ../$KERNEL_MODULES/
cp  fs/fuse/fuse.ko                                 ../$KERNEL_MODULES/
cp  sound/htcleo/alsa-pcm-htc-leo.ko                ../$KERNEL_MODULES/
cp  crypto/zlib.ko                                  ../$KERNEL_MODULES/

cp  arch/arm/boot/zImage  ../$KERNEL_BOOT/


cd ..

tar -czf $KERNEL_NAME.tgz $KERNEL_NAME
                          
