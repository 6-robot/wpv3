#!/bin/bash

echo "***************"
echo "remap the device serial ports(ttyUSBX) to wpv3 device"
echo "start copy wpv3.rules to  /etc/udev/rules.d/"
sudo cp `rospack find wpv3_bringup`/scripts/wpv3.rules  /etc/udev/rules.d
echo "Restarting udev"
sudo service udev reload
sudo service udev restart
echo "finish"
echo "***************"
