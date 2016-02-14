#!/bin/bash

echo "TBS drivers set for x64 Linux 3.x"

./v4l/tbs-x86_64.sh
#./v4l/tbs-dvbc-x86_64.sh

# Enable some staging drivers
make stagingconfig

echo "TBS drivers building..."
make -j4

echo "TBS drivers installing..."
sudo rm -r -f /lib/modules/$(test $VER && echo $VER || uname -r)/kernel/drivers/media
sudo rm -r -f /lib/modules/$(test $VER && echo $VER || uname -r)/kernel/drivers/staging/media
sudo rm -r -f /lib/modules/$(test $VER && echo $VER || uname -r)/kernel/drivers/misc/altera-stapl
sudo make install

echo "TBS drivers installation done"
echo "You need to reboot..."
