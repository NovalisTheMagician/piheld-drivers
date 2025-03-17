/* SPDX-License-Identifier: GPL-2.0 */
#ifndef NVKBD_I2CHELPER_H_
#define NVKBD_I2CHELPER_H_

#include <linux/i2c.h>

#include "nvkbd_registers.h"
#include "debug_levels.h"

extern int nvkbd_write(struct i2c_client *i2c_client, uint8_t registerAddress, const uint8_t *buffer, uint8_t bufferSize);
extern int nvkbd_read(struct i2c_client *i2c_client, uint8_t registerAddress, uint8_t *buffer, uint8_t bufferSize);
#endif
