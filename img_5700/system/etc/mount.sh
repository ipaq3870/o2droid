#!/system/bin/sh
mkdir /efs 
chmod 777 /efs

chmod 777 /dev/*

chown system.graphics /dev/pmem_gpu
chown system.graphics /dev/pmem_gpu1
chown system.graphics /dev/pmem_render
chown system.graphics /dev/pmem_stream
chown system.graphics /dev/pmem_preview
chown system.graphics /dev/pmem_picture
chown system.graphics /dev/pmem_jpeg
chown system.graphics /dev/pmem_skia
chown system.graphics /dev/s3c-cmm
chown system.graphics /dev/pmem_stream2


chown system.graphics /dev/s3c-mfc
chown system.graphics /dev/s3c-rotator
chown system.graphics /dev/s3c-g3d
chown system.graphics /dev/s3c-pp
chown system.graphics /dev/s3c-g2d
chown system.graphics /dev/s3c-mem

chown root.root /dev/mem

chown radio.radio  /dev/ttySMD0
chown radio.radio  /dev/multipdp
chown radio.radio  /dev/dpramerr
chown radio.radio  /dev/dpram1
chown radio.radio  /dev/dpram0
chown radio.radio  /dev/s3c_serial0
chown radio.radio  /dev/ttyCSD0

chown system.system /dev/ttyGPS0
chown system.system /dev/ttyXTRA0






