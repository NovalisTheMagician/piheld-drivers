#!/bin/bash

sudo cp nvkbd.ko /lib/modules/$(uname -r)/kernel/drivers/i2c/nvkbd.ko
sudo modprobe -r nvkbd
sudo modprobe nvkbd
sleep 2
echo 255 > /sys/class/backlight/nvbacklight/brightness
