/*
 * SBUS.cpp
 *
 *  Created on: 17 июн. 2022 г.
 *      Author: arosl
 */

#include "SBUS.h"

uint16_t CalculateSBUS(uint8_t buf[25], uint16_t CH[18]){
if (buf[0] == 0x0F) {
		CH[0] = (buf[1] >> 0 | (buf[2] << 8)) & 0x07FF;
		CH[1] = (buf[2] >> 3 | (buf[3] << 5)) & 0x07FF;
		CH[2] = (buf[3] >> 6 | (buf[4] << 2) | buf[5] << 10) & 0x07FF;
		CH[3] = (buf[5] >> 1 | (buf[6] << 7)) & 0x07FF;
		CH[4] = (buf[6] >> 4 | (buf[7] << 4)) & 0x07FF;
		CH[5] = (buf[7] >> 7 | (buf[8] << 1) | buf[9] << 9) & 0x07FF;
		CH[6] = (buf[9] >> 2 | (buf[10] << 6)) & 0x07FF;
		CH[7] = (buf[10] >> 5 | (buf[11] << 3)) & 0x07FF;
		CH[8] = (buf[12] << 0 | (buf[13] << 8)) & 0x07FF;
		CH[9] = (buf[13] >> 3 | (buf[14] << 5)) & 0x07FF;
		CH[10] = (buf[14] >> 6 | (buf[15] << 2) | buf[16] << 10) & 0x07FF;
		CH[11] = (buf[16] >> 1 | (buf[17] << 7)) & 0x07FF;
		CH[12] = (buf[17] >> 4 | (buf[18] << 4)) & 0x07FF;
		CH[13] = (buf[18] >> 7 | (buf[19] << 1) | buf[20] << 9) & 0x07FF;
		CH[14] = (buf[20] >> 2 | (buf[21] << 6)) & 0x07FF;
		CH[15] = (buf[21] >> 5 | (buf[22] << 3)) & 0x07FF;

		if (buf[23] & (1 << 0)) {
			CH[16] = 1;
		} else {
			CH[16] = 0;
		}

		if (buf[23] & (1 << 1)) {
			CH[17] = 1;
		} else {
			CH[17] = 0;
		}

		// Failsafe
		uint16_t failsafe_status = SBUS_SIGNAL_OK;
		if (buf[23] & (1 << 2)) {
			failsafe_status = SBUS_SIGNAL_LOST;
		}

		if (buf[23] & (1 << 3)) {
			failsafe_status = SBUS_SIGNAL_FAILSAFE;
		}

		//	SBUS_footer=buf[24];

		return(failsafe_status);

	}
else{
	return(0);
}
}
