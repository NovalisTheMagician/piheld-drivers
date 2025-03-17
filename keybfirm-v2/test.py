import smbus
import time

bus = smbus.SMBus(1)

DEVICE_ADDRESS = 0x1f

REG_VER = 0x01
REG_KEY = 0x04
REG_BKL = 0x05
REG_FIF = 0x09

def write_reg(reg, val):
    bus.write_byte_data(DEVICE_ADDRESS, reg | 0x80, val)

def read_reg(reg):
    return bus.read_word_data(DEVICE_ADDRESS, reg)

keys_in_fifo = read_reg(REG_KEY)
print("keys in fifo: " + str(keys_in_fifo))

fifo_item = read_reg(REG_FIF)

key = (fifo_item >> 8) & 0xff
state = fifo_item & 0xff

print("state: " + str(state) + " key: " + str(hex(key)))
