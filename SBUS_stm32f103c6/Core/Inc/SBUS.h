/*
 * SBUS.h
 *
 *  Created on: 17 июн. 2022 г.
 *      Author: arosl
 */

#ifndef INC_SBUS_H_
#define INC_SBUS_H_

#include "main.h"

#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03

uint16_t CalculateSBUS(uint8_t buf[25], uint16_t CH[18]);

#endif /* INC_SBUS_H_ */
