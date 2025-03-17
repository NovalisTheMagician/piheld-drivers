#!/bin/bash

sudo cp stc3100.ko /lib/modules/$(uname -r)/kernel/drivers/i2c/stc3100.ko
sudo modprobe -r stc3100
sudo modprobe stc3100
