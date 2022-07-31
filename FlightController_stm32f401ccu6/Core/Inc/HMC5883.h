/*
 * HMC5883.h
 *
 *  Created on: 26 июл. 2022 г.
 *      Author: arosl
 */

#ifndef INC_HMC5883_H_
#define INC_HMC5883_H_

#include "main.h"
#define HMC5883_I2C &hi2c2
#define COMPASS_TIME_TO_UPDATE 67

#define HMC5883L_ADDRESS  (0x1E << 1)

#define COMPASS_CONFIG_REGISTER_A 0x00
#define COMPASS_CONFIG_REGISTER_B 0x01
#define COMPASS_MODE_REGISTER     0x02
#define COMPASS_DATA_REGISTER     0x03
#define COMPASS_DATA_READY_REGISER 0x09

#define Data_Output_X_MSB 0x03
#define Data_Output_X_LSB 0x04
#define Data_Output_Z_MSB 0x05
#define Data_Output_Z_LSB 0x06
#define Data_Output_Y_MSB 0x07
#define Data_Output_Y_LSB 0x08

#define COMPASS_SAMPLE1 0x00 // 1-average(Default)
#define COMPASS_SAMPLE2 0x20 // 2-average
#define COMPASS_SAMPLE4 0x40 // 4-average
#define COMPASS_SAMPLE8 0x60 // 8-average

#define COMPASS_RATE0_75 0x00 // 0.75Hz
#define COMPASS_RATE1_5  0x04 // 1.5Hz
#define COMPASS_RATE3    0x08 // 3Hz
#define COMPASS_RATE7_5  0x0C // 7.5Hz
#define COMPASS_RATE15   0x10 // 15Hz (Default)
#define COMPASS_RATE30   0x14 // 30Hz
#define COMPASS_RATE75   0x18 // 75Hz

#define COMPASS_MEASURE_NORMAL   0x00 // Normal measurement configuration (Default)
#define COMPASS_MEASURE_POSITIVE 0x01 // Positive bias configuration for X, Y, and Z axes.
#define COMPASS_MEASURE_NEGATIVE 0x02 // Negative bias configuration for X, Y and Z axes.

#define COMPASS_SCALE_088  0x0  //0.88 Ga
#define COMPASS_SCALE_130  0x20 //1.3 Ga (Default)
#define COMPASS_SCALE_190  0x40 //1.9 Ga
#define COMPASS_SCALE_250  0x60 //2.5 Ga
#define COMPASS_SCALE_400  0x80 //4.0 Ga
#define COMPASS_SCALE_470  0xA0 //4.7 Ga
#define COMPASS_SCALE_560  0xC0 //5.6 Ga
#define COMPASS_SCALE_810  0xE0 //8.1 Ga

#define COMPASS_CONTINUOUS 0x00  //Continuous-Measurement Mode.
#define COMPASS_SINGLE     0x01  //Single-Measurement Mode (Default).
#define COMPASS_IDLE       0x02  //Idle Mode. Device is placed in idle mode


uint8_t HMC_Init();
uint8_t HMC_Update(float *azimuth);
void SetDeclination( int declination_degs , int declination_mins, char declination_dir );
void SetSamplingMode( uint16_t sampling_mode , uint16_t rate, uint16_t measure);
void SetScaleMode(uint8_t ScaleMode);
void SetMeasureMode( uint8_t Measure);
float CompasReadAndCompensate(float pitch, float roll);



#endif /* INC_HMC5883_H_ */
