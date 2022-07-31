/*
 * QMC5883.c
 *
 *  Created on: 26 июл. 2022 г.
 *      Author: arosl
 */

#ifdef INC_QMC5883_H_
#include "QMC5883.h"
#include "math.h"
#include "main.h"
#include "mpu6050.h"


float declination_offset_radians;
uint32_t last_update_time = 0;
uint8_t readyData = 0;

extern MPU6050_t MPU6050;
extern I2C_HandleTypeDef hi2c2;

uint8_t QMC_Init(){
	SetQMCMode(COMPASS_OVERSAMPLING_512 , COMPASS_RANGE_2G,  COMPASS_DATA_RATE_100HZ, COMPASS_MODE_Continuous);
	SetQMCSetReset(COMPASS_FBR_DEFAULT);
	SetDeclination(13, 17, 'E'); // Rybinsk

	uint8_t addr = 0;
	while( HAL_I2C_IsDeviceReady(QMC5883_I2C, addr, 3, 100)!=HAL_OK){
		addr++;
		if(addr == 0b11111111){
			break;
		}
	}
	return addr;
}


uint8_t QMC_Update(float *azimuth){
	if(HAL_GetTick() - last_update_time > COMPASS_TIME_TO_UPDATE){
		last_update_time = HAL_GetTick();

		HAL_I2C_Mem_Read(QMC5883_I2C, QMC5883L_ADDRESS, 0xD, I2C_MEMADD_SIZE_8BIT, &readyData, 1, 1000);
		*azimuth = CompasReadAndCompensate(-MPU6050.AngleX, -MPU6050.AngleY);
		if(readyData & (1 << 0)){
		//*azimuth = CompasReadAndCompensate(-MPU6050.AngleX, -MPU6050.AngleY);
		return 1;
		}
		else{
			return 0;
		}

	}
	else{
		return 0;
	}
}


// Magnetic Declination is the correction applied according to your present location
// in order to get True North from Magnetic North, it varies from place to place.
//
// The declination for your area can be obtained from http://www.magnetic-declination.com/
// Take the "Magnetic Declination" line that it gives you in the information,
//
// Examples:
//   Christchurch, 23° 35' EAST
//   Wellington  , 22° 14' EAST
//   Dunedin     , 25° 8'  EAST
//   Auckland    , 19° 30' EAS
//   SANFINA     , 4°  33' WEST

void SetDeclination( int declination_degs , int declination_mins, char declination_dir )
{
  // Convert declination to decimal degrees
  switch (declination_dir)
  {
    // North and East are positive
    case 'E':
      declination_offset_radians = 0 - ( declination_degs + (1 / 60 * declination_mins)) * (M_PI / 180);
      break;

    // South and West are negative
    case 'W':
      declination_offset_radians =  (( declination_degs + (1 / 60 * declination_mins) ) * (M_PI / 180));
      break;
  }
}

void SetQMCMode( uint16_t oversampling , uint16_t range, uint16_t frequncy, uint16_t mode)
{
  uint8_t data = (oversampling | range | frequncy | mode);
  HAL_I2C_Mem_Write(QMC5883_I2C, QMC5883L_ADDRESS, COMPASS_CONFIG_REGISTER_A, 1, &data, 1, HAL_MAX_DELAY);
}

void SetQMCSetReset(uint16_t command){
	uint8_t data = command;
	HAL_I2C_Mem_Write(QMC5883_I2C, QMC5883L_ADDRESS, COMPASS_CONFIG_FBR, 1, &data, 1, HAL_MAX_DELAY);
}

float CompasReadAndCompensate(float pitch, float roll)
{
  uint8_t buffer[6];
  HAL_I2C_Mem_Read(QMC5883_I2C, QMC5883L_ADDRESS, COMPASS_DATA_REGISTER, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&buffer, 6, HAL_MAX_DELAY);
  int16_t compass_X = ((buffer[0] << 8) | buffer[1]);
  int16_t compass_Y = ((buffer[2] << 8) | buffer[3]);
  int16_t compass_Z = ((buffer[4] << 8) | buffer[5]);

  float IMU_roll = pitch * (M_PI / 180);
    float IMU_pitch = roll * (M_PI / 180);

    float XH = compass_X * cos(IMU_pitch) + compass_Y * sin(IMU_roll) * sin(IMU_pitch) - compass_Z * cos(IMU_roll) * sin(IMU_pitch);
    float YH = compass_Y * cos(IMU_roll) + compass_Z * sin(IMU_roll);
         // Azimuth = atan2(YH / XH)
    float Azimuth = atan2(YH, XH) * 180 / M_PI;
    Azimuth += declination_offset_radians; // see https://www.magnetic-declination.com/

    if (Azimuth < 0) {
      Azimuth += 360;
    }
    else if (Azimuth >= 360) {
      Azimuth -= 360;
    }

    return Azimuth;
}
#endif
