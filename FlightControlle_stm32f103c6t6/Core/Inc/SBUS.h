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

#define SBUS_TARGET_MIN 1000
#define SBUS_RANGE_MIN 300
#define SBUS_SCALE_FACTOR 1000/1400

uint16_t CalculateSBUS(uint8_t buf[25], uint16_t CH[18]);
uint16_t sbus_to_pwm(uint16_t sbus_value);

#endif /* INC_SBUS_H_ */
