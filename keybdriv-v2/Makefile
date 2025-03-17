# SPDX-License-Identifier: GPL-2.0
# Keyboard Driver for Blackberry Keyboards BBQ10 from arturo182. Software written by wallComputer.

include Kbuild
CFLAGS_$(MODULE_NAME)_main.o := -DDEBUG

modules:
	make -C $(KDIR) M=$(PWD) modules

dtbo:
	dtc -I dts -O dtb -o i2c-nvkbd.dtbo src/dts_src/i2c-nvkbd.dts

clean:
	make -C $(KDIR) M=$(PWD) clean
	rm -rf i2c-nvkbd.dtbo


