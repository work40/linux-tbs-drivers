#!/bin/bash

echo "TBS drivers set for x64 Linux 2.6.x"

./v4l/tbs-x86_64.sh
#./v4l/tbs-dvbc-x86_64.sh

# Enable some staging drivers
make stagingconfig

echo "TBS drivers building..."
make -j4

echo "TBS drivers installing..."
sudo make install

echo "TBS drivers installation done"
echo "You need to reboot..."
