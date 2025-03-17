#!/bin/bash

if make && make dtbo ; then
    sudo sh -c "echo dtoverlay=i2c-nvkbd,irq_pin=27 >> /boot/config.txt"
    sudo cp i2c-nvkbd.dtbo /boot/overlays/
    
    sudo sh -c "echo nvkbd >> /etc/modules"
    sudo cp nvkbd.ko /lib/modules/$(uname -r)/kernel/drivers/i2c/nvkbd.ko

    sudo depmod

    echo -e "\nmake succeeded, please reboot to get the keyboard working."
else
    echo "MAKE DID NOT SUCCEED!!!\n"
fi
