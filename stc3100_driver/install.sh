#!/bin/bash

if make && make dtbo ; then
    sudo sh -c "echo dtoverlay=stc3100 >> /boot/config.txt"
    sudo cp stc3100.dtbo /boot/overlays/
    
    sudo sh -c "echo stc3100 >> /etc/modules"
    sudo cp stc3100.ko /lib/modules/$(uname -r)/kernel/drivers/i2c/stc3100.ko

    sudo depmod

    echo -e "\nmake succeeded, please reboot"
else
    echo "MAKE DID NOT SUCCEED!!!\n"
fi
