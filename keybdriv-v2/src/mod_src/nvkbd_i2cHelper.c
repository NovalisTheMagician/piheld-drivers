// SPDX-License-Identifier: GPL-2.0-only

#include "nvkbd_i2cHelper.h"

extern int nvkbd_write(struct i2c_client *i2c_client, uint8_t registerAddress, const uint8_t *buffer, uint8_t bufferSize)
{
	int returnValue;

	returnValue = i2c_smbus_write_byte_data(i2c_client, registerAddress | NVKBD_WRITE_MASK, *buffer);
	if (returnValue != 0) {
		dev_err(&i2c_client->dev, "%s Could not write to register 0x%02X, Error: %d\n", __func__, registerAddress, returnValue);
		return returnValue;
	}
#if (DEBUG_LEVEL & DEBUG_LEVEL_RW)
	if (bufferSize == sizeof(uint8_t))
		dev_err(&i2c_client->dev, "%s Wrote data: [0x%02X] to register 0x%02X\n", __func__, *buffer, registerAddress);
	else
		dev_err(&i2c_client->dev, "%s Wrote data: [0x%02X%02X] to register 0x%02X\n", __func__, *buffer, *(buffer+1), registerAddress);
#endif
	return 0;
}

extern int nvkbd_read(struct i2c_client *i2c_client, uint8_t registerAddress, uint8_t *buffer, uint8_t bufferSize)
{
	int returnValue;

	if (bufferSize == 2*sizeof(uint8_t)) {
		returnValue = i2c_smbus_read_word_data(i2c_client, registerAddress);
		if (returnValue < 0) {
			dev_err(&i2c_client->dev, "%s Could not read from register 0x%02X, error: %d\n", __func__, registerAddress, returnValue);
			return returnValue;
		}
		*buffer = (uint8_t)(returnValue & 0xFF);
		*(buffer+1) = (uint8_t)((returnValue & 0xFF00) >> 8);
	} else {
		returnValue = i2c_smbus_read_byte_data(i2c_client, registerAddress);
		if (returnValue < 0) {
			dev_err(&i2c_client->dev, "%s Could not read from register 0x%02X, error: %d\n", __func__, registerAddress, returnValue);
			return returnValue;
		}
		*buffer = returnValue & 0xFF;
	}
#if (DEBUG_LEVEL & DEBUG_LEVEL_RW)
	if (bufferSize == sizeof(uint8_t))
		dev_err(&i2c_client->dev, "%s Read data: [0x%02X] from register 0x%02X\n", __func__, *buffer, registerAddress);
	else
		dev_err(&i2c_client->dev, "%s Read data: [0x%02X%02X]/%c  from register 0x%02X\n", __func__, *buffer, *(buffer+1), *(buffer+1), registerAddress);
#endif
	return 0;
}
