APPLICATION := keybfirm

SRC_DIR := src

DEFINES := 
INC_DIRS := $(SRC_DIR)

CC := sdcc
CCFLAGS := -mstm8 --std=c23

LD := $(CC)
LDFLAGS := -mstm8 --out-fmt-ihx

FLASH := stm8flash
FLASHFLAGS := -c stlinkv2 -p stm8s005c6

CPPFLAGS := $(addprefix -I,$(INC_DIRS)) $(addprefix -D,$(DEFINES))

OBJS := main.rel

APP_TARGET := $(APPLICATION).ihx

all: $(APP_TARGET)

$(APP_TARGET): $(OBJS)
	$(LD) $(LDFLAGS) $(OBJS) -o $@

%.rel: $(SRC_DIR)/%.c Makefile
	$(CC) $(CPPFLAGS) $(CCFLAGS) -c $<

.PHONY: clean flash verify
flash: $(APP_TARGET)
	$(FLASH) $(FLASHFLAGS) -w $<

verify: $(APP_TARGET)
	$(FLASH) $(FLASHFLAGS) -v $<

clean:
	rm -f *.d
	rm -f *.rel
	rm -f *.asm
	rm -f *.rst
	rm -f *.lst
	rm -f *.sym
	rm -f $(APPLICATION).*
