/* SPDX-License-Identifier: GPL-2.0 */
#ifndef NVKBD_MAIN_H_
#define NVKBD_MAIN_H_

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/ktime.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/timekeeping.h>

#include "nvkbd_i2cHelper.h"
#include "nvkbd_registers.h"
#include "debug_levels.h"
#include "nvkbd_codes.h"

#define COMPATIBLE_NAME			"novalis,nvkbd"
#define DEVICE_NAME				"nvkbd"
#define BACKLIGHT_NAME			"nvbacklight"
#define NV_CHAR_NAME			"nvstatus"
#define NV_CLASS_NAME			"nvclass"
#define NVKBD_BUS_TYPE			BUS_I2C
#define NVKBD_VENDOR_ID			0x0001
#define NVKBD_PRODUCT_ID		0x0001
#define NVKBD_VERSION_ID		0x0001

struct nvkbd_data {
	struct work_struct work_struct;
	uint8_t fifoCount;
	uint8_t fifoData[NVKBD_FIFO_SIZE][2];

	uint8_t version_number;

	uint8_t layer_switch, layer_lock, last_released_scancode;
	ktime_t last_release_time;

	struct backlight_properties backlight_props;
	struct backlight_ops backlight_ops;

	unsigned int keycode[NUM_KEYCODES];
	struct i2c_client *i2c_client;
	struct input_dev *input_dev;
	struct backlight_device *backlight_dev;

	struct cdev status_dev;
	struct device *status_device;
	int status_major;
	struct class *status_class;
};

MODULE_LICENSE("GPL");
MODULE_AUTHOR("novalis");
MODULE_DESCRIPTION("Keyboard driver for the mini Keyboard by novalis.");
MODULE_VERSION("0.1");

static const struct i2c_device_id nvkbd_i2c_device_id[] = {
	{ DEVICE_NAME, 0, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nvkbd_i2c_device_id);

static const struct of_device_id nvkbd_of_device_id[] = {
	{ .compatible = COMPATIBLE_NAME, },
	{ }
};
MODULE_DEVICE_TABLE(of, nvkbd_of_device_id);

#endif
