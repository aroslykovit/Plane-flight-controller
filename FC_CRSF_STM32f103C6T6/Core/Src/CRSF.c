/*
 * CRSF.c
 *
 *  Created on: 24 июл. 2022 г.
 *      Author: arosl
 */

#include "CRSF.h"
#include "main.h"
#include "string.h"
#include "crc.h"

crsfFrame_t crsfFrame;
crsfFrame_t crsfChannelDataFrame;
uint32_t crsfChannelData[CRSF_MAX_CHANNEL];
uint8_t buffer_cursor = 0;

void ProcessCRSF(uint8_t data) {

	if (DWT_CYCCNT > MAX_CRSF_INTERVAL) {

		buffer_cursor = 0;
		DWT_CYCCNT = 0;

	}

	const int fullFrameLength = buffer_cursor < 3 ? 5 : crsfFrame.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH;
	if (buffer_cursor < fullFrameLength) {
		crsfFrame.bytes[buffer_cursor++] = data;

		if (buffer_cursor >= fullFrameLength) {
			buffer_cursor = 0;
			DWT_CYCCNT = 0;

			const uint8_t crc = crsfFrameCRC();
			if (crc == crsfFrame.bytes[fullFrameLength - 1]) {

				switch (crsfFrame.frame.type) {
				case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
					if (crsfFrame.frame.deviceAddress == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
						memcpy(&crsfChannelDataFrame, &crsfFrame,
								sizeof(crsfFrame));
						ProcessFrame();
					}
					break;
				}
			}
		}

	}

}

uint8_t crsfFrameCRC(void)
{
    // CRC includes type and payload
    uint8_t crc = crc8_dvb_s2(0, crsfFrame.frame.type);
    for (int ii = 0; ii < crsfFrame.frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC; ++ii) {
        crc = crc8_dvb_s2(crc, crsfFrame.frame.payload[ii]);
    }
    return crc;
}

void DWT_Init(void) {
	SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // разрешаем использовать счётчик
	DWT_CONTROL |= DWT_CTRL_CYCCNTENA_Msk;   // запускаем счётчик
}

void ProcessFrame(void) {

	// unpack the RC channels
	const crsfPayloadRcChannelsPacked_t *const rcChannels =
			(crsfPayloadRcChannelsPacked_t*) &crsfChannelDataFrame.frame.payload;
	crsfChannelData[0] = rcChannels->chan0;
	crsfChannelData[1] = rcChannels->chan1;
	crsfChannelData[2] = rcChannels->chan2;
	crsfChannelData[3] = rcChannels->chan3;
	crsfChannelData[4] = rcChannels->chan4;
	crsfChannelData[5] = rcChannels->chan5;
	crsfChannelData[6] = rcChannels->chan6;
	crsfChannelData[7] = rcChannels->chan7;
	crsfChannelData[8] = rcChannels->chan8;
	crsfChannelData[9] = rcChannels->chan9;
	crsfChannelData[10] = rcChannels->chan10;
	crsfChannelData[11] = rcChannels->chan11;
	crsfChannelData[12] = rcChannels->chan12;
	crsfChannelData[13] = rcChannels->chan13;
	crsfChannelData[14] = rcChannels->chan14;
	crsfChannelData[15] = rcChannels->chan15;

}

uint16_t crsfReadRawRC(uint8_t chan) {
	/* conversion from RC value to PWM
	 *       RC     PWM
	 * min  172 ->  988us
	 * mid  992 -> 1500us
	 * max 1811 -> 2012us
	 * scale factor = (2012-988) / (1811-172) = 0.62477120195241
	 * offset = 988 - 172 * 0.62477120195241 = 880.53935326418548
	 */
	//return (0.62477120195241f * crsfChannelData[chan]) + 881;
	return (0.610738255f * crsfChannelData[chan]) + 895; //just for my radio
}
