# SPDX-License-Identifier: GPL-2.0
# Keyboard Driver for Blackberry Keyboards BBQ10 from arturo182. Software written by wallComputer.

include Kbuild
CFLAGS_$(MODULE_NAME).o := -DDEBUG

modules:
	make -C $(KDIR) M=$(PWD) modules

dtbo:
	dtc -I dts -O dtb -o $(MODULE_NAME).dtbo src/dts_src/$(MODULE_NAME).dts

clean:
	make -C $(KDIR) M=$(PWD) clean
	rm -rf $(MODULE_NAME).dtbo
