/*
 * HMC5883.c
 *
 *  Created on: 26 июл. 2022 г.
 *      Author: arosl
 */


#include "HMC5883.h"
#include "math.h"
#include "main.h"
#include "mpu6050.h"

float scale;
float declination_offset_radians;
uint32_t last_update_time = 0;
uint8_t readyData = 0;
uint8_t settings = 0;

extern MPU6050_t MPU6050;
extern I2C_HandleTypeDef hi2c2;
extern int16_t compassX;
extern int16_t compassY;
extern int16_t compassZ;


uint8_t HMC_Init(){
	SetSamplingMode(COMPASS_SAMPLE8 , COMPASS_RATE15, COMPASS_MEASURE_NORMAL);
	SetScaleMode( COMPASS_SCALE_400);
	SetMeasureMode(COMPASS_CONTINUOUS);
	SetDeclination(13, 17, 'E'); // Rybinsk

	HAL_I2C_Mem_Read(HMC5883_I2C, HMC5883L_ADDRESS, COMPASS_CONFIG_REGISTER_A, I2C_MEMADD_SIZE_8BIT, &settings, 1, 1000);
	return settings;
}

uint8_t HMC_Update(float *azimuth){
	if(HAL_GetTick() - last_update_time > COMPASS_TIME_TO_UPDATE){
		last_update_time = HAL_GetTick();

		HAL_I2C_Mem_Read(HMC5883_I2C, HMC5883L_ADDRESS, COMPASS_DATA_READY_REGISER, I2C_MEMADD_SIZE_8BIT, &readyData, 1, 1000);
		*azimuth = CompasReadAndCompensate(-MPU6050.AngleX, -MPU6050.AngleY);
		/*if(readyData & (1<<0)){
		*azimuth = CompasReadAndCompensate(-MPU6050.AngleX, -MPU6050.AngleY);
		return 1;
		}
		else{
			return 0;
		}*/


	}
	return readyData;
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




void SetSamplingMode( uint16_t sampling_mode , uint16_t rate, uint16_t measure)
{
  uint8_t data = (sampling_mode | rate | measure);
  HAL_I2C_Mem_Write(HMC5883_I2C, HMC5883L_ADDRESS, COMPASS_CONFIG_REGISTER_A, 1, &data, 1, HAL_MAX_DELAY);
}



void SetScaleMode(uint8_t ScaleMode)
{
  switch (ScaleMode) {
    case COMPASS_SCALE_088:
      scale = 0.73;
      break;
    case COMPASS_SCALE_130:
      scale = 0.92;
      break;
    case COMPASS_SCALE_190:
      scale = 1.22;
      break;
    case COMPASS_SCALE_250:
      scale = 1.52;
      break;
    case COMPASS_SCALE_400:
      scale = 2.27;
      break;
    case COMPASS_SCALE_470:
      scale = 2.56;
      break;
    case COMPASS_SCALE_560:
      scale = 3.03;
      break;
    case COMPASS_SCALE_810:
      scale = 4.35;
      break;
    default:
      scale = 0.92;
      ScaleMode = COMPASS_SCALE_130;
  }

  HAL_I2C_Mem_Write(HMC5883_I2C, HMC5883L_ADDRESS, COMPASS_CONFIG_REGISTER_B, 1, &ScaleMode, 1, HAL_MAX_DELAY);
}



void SetMeasureMode( uint8_t Measure)
{
  HAL_I2C_Mem_Write(HMC5883_I2C, HMC5883L_ADDRESS, COMPASS_MODE_REGISTER, 1, &Measure, 1, HAL_MAX_DELAY);
}

float CompasReadAndCompensate(float pitch, float roll)
{
  uint8_t buffer[6];
  HAL_I2C_Mem_Read(HMC5883_I2C, HMC5883L_ADDRESS, COMPASS_DATA_REGISTER, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&buffer, 6, HAL_MAX_DELAY);
  int16_t compass_X = ((buffer[0] << 8) | buffer[1]) * scale;
  int16_t compass_Y = ((buffer[2] << 8) | buffer[3]) * scale;
  int16_t compass_Z = ((buffer[4] << 8) | buffer[5]) * scale;

  compassX = compass_X;
  compassY = compass_Y;
  compassZ = compass_Z;

  float IMU_roll = pitch * (M_PI / 180);
    float IMU_pitch = roll * (M_PI / 180);

    float XH = compass_X * cos(IMU_pitch) + compass_Y * sin(IMU_roll) * sin(IMU_pitch) - compass_Z * cos(IMU_roll) * sin(IMU_pitch);
    float YH = compass_Y * cos(IMU_roll) + compass_Z * sin(IMU_roll);
    //float      Azimuth = atan2(compass_Y, compass_X) * 180 / M_PI;
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



