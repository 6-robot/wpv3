#!/bin/bash

echo "***************"
echo "delete the remap device serial ports of wpv3"
sudo rm   /etc/udev/rules.d/wpv3.rules
echo "Restarting udev"
sudo service udev reload
sudo service udev restart
echo "finish"
echo "***************"
