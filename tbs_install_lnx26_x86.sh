#!/bin/bash

echo "TBS drivers set for x86 Linux 2.6.x"

./v4l/tbs-x86.sh
#./v4l/tbs-dvbc-x86.sh

# Enable some staging drivers
make stagingconfig

echo "TBS drivers building..."
make -j2

echo "TBS drivers installing..."
sudo make install

echo "TBS drivers installation done"
echo "You need to reboot..."
