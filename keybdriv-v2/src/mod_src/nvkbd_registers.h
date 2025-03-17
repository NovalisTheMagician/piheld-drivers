/* SPDX-License-Identifier: GPL-2.0 */

#ifndef NVKBD_REGISTERS_H_
#define NVKBD_REGISTERS_H_

#define NVKBD_I2C_ADDRESS			    0x1F
#define NVKBD_INT_PIN                   27

#define NVKBD_WRITE_MASK				0x80
#define NVKBD_FIFO_SIZE				    31

#define REG_VER                         0x01
#define NVKBD_I2C_SW_VERSION			0x02
#define REG_CFG                         0x02
#define REG_CFG_PANIC_INT               BIT(2)
#define REG_CFG_OVERFLOW_INT            BIT(1)
#define REG_CFG_OVERFLOW_ON             BIT(0)
#define REG_CFG_DEFAULT_SETTING         (REG_CFG_OVERFLOW_ON | REG_CFG_OVERFLOW_INT)

#define REG_INT                         0x03
#define REG_INT_PANIC                   BIT(2)
#define REG_INT_KEY                     BIT(1)
#define REG_INT_OVERFLOW                BIT(0)
#define REG_INT_RESET_VALUE             0x00

#define REG_KEY                         0x04
#define REG_KEY_KEYCOUNT_MASK           0x1F

#define REG_BKL                         0x05

#define REG_DEB                         0x06

#define REG_FRQ                         0x07

#define REG_RST                         0x08
#define REG_FIF                         0x09
#define KEY_PRESSED_STATE               1
#define KEY_RELEASED_STATE              0

#define BRIGHTNESS_DELTA			    16

#endif
