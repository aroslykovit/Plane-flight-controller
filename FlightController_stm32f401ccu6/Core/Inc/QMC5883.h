/*
 * QMC5883.h
 *
 *  Created on: 26 июл. 2022 г.
 *      Author: arosl
 */

#ifdef INC_QMC5883_H_
#define INC_QMC5883_H_

#include "main.h"
#define QMC5883_I2C &hi2c2
#define COMPASS_TIME_TO_UPDATE 10

#define QMC5883L_ADDRESS  0x3C

#define COMPASS_CONFIG_REGISTER_A 0x09
#define COMPASS_CONFIG_REGISTER_B 0x0A
#define COMPASS_DATA_REGISTER     0x00
#define COMPASS_DATA_READY_REGISER 0x06
#define COMPASS_CONFIG_FBR        0x0B

#define COMPASS_FBR_DEFAULT       0x01

#define Data_Output_X_MSB 0x00
#define Data_Output_X_LSB 0x01
#define Data_Output_Y_MSB 0x02
#define Data_Output_Y_LSB 0x03
#define Data_Output_Z_MSB 0x04
#define Data_Output_Z_LSB 0x05

#define COMPASS_DATA_RATE_10HZ 0x00
#define COMPASS_DATA_RATE_50HZ 0x04
#define COMPASS_DATA_RATE_100HZ 0x08
#define COMPASS_DATA_RATE_200HZ 0x0C

#define COMPASS_MODE_STANDBY    0x00
#define COMPASS_MODE_Continuous 0x01

#define COMPASS_RANGE_2G        0x00
#define COMPASS_RANGE_8G        0x10

#define COMPASS_OVERSAMPLING_512 0x00
#define COMPASS_OVERSAMPLING_256 0x40
#define COMPASS_OVERSAMPLING_128 0x80
#define COMPASS_OVERSAMPLING_64  0xC0



uint8_t QMC_Init();
uint8_t QMC_Update(float *azimuth);
void SetDeclination( int declination_degs , int declination_mins, char declination_dir );
void SetQMCMode( uint16_t oversampling , uint16_t range, uint16_t frequncy, uint16_t mode);
void SetQMCSetReset(uint16_t command);
float CompasReadAndCompensate(float pitch, float roll);




#endif /* INC_QMC5883_H_ */
