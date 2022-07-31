/*
 * mpu6050.c
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 *
 *  Contact information
 *  -------------------
 *
 * e-mail   :  leech001@gmail.com
 */

/*
 * |---------------------------------------------------------------------------------
 * | Copyright (C) Bulanov Konstantin,2021
 * |
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |
 * | Kalman filter algorithm used from https://github.com/TKJElectronics/KalmanFilter
 * |---------------------------------------------------------------------------------
 */

#include <math.h>
#include "mpu6050.h"
#include "BiQuadFilter.h"
#include "filter.h"
#include "main.h"
#include "IMU.h"

extern MPU6050_t MPU6050;

#define MPU_CORRECTION_GX  3.0f
#define MPU_CORRECTION_GY  0.7f
#define MPU_CORRECTION_GZ  1.20f

extern I2C_HandleTypeDef hi2c1;

#define RAD_TO_DEG 57.295779513082320876798154814105



// Setup MPU6050
#define MPU6050_ADDR 0xD0
#define MPU6050_ADDRESS 0xD0
const uint16_t i2c_timeout = 100;
const float Accel_Z_corrector = 14418.0;
float qm[4]  = {1.0f, 0.0f, 0.0f, 0.0f};
double dt = 0;
float GyroMeasError = M_PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta = sqrt(3.0f / 4.0f) * M_PI * (40.0f / 180.0f);  // compute beta
float GyroMeasDrift = M_PI * (2.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta = sqrt(3.0f / 4.0f) * M_PI * (2.0f / 180.0f);  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}; // Bias corrections for gyro and accelerometer


uint32_t timer;

extern uint8_t Read_Data[14];
struct BiQuadFilter XFiter;
struct BiQuadFilter YFiter;
struct BiQuadFilter ZFiter;



uint32_t microes(){
	return  TIM5->CNT;
}

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check;
    uint8_t Data;

   // calibrateMPU6050(gyroBias, accelBias);

    // check device ID WHO_AM_I

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);

    if (check == 104) // 0x68 will be returned by the sensor if everything goes well
    {

    	//calibrateMPU6050(gyroBias, accelBias);
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 16g
        Data = 0b00011000;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 1000 �/s
        Data = 0b00010000;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);


        //Set interrupt on int pin for data ready
        Data = 0x01;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, INT_PIN_INTERUPT_REG, 1, &Data, 1, i2c_timeout);

        Data = 0x01;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, LPF_FILTER_REG, 1, &Data, 1, i2c_timeout);

        //return 1;
         return 0;
    }
    return 1;
}

void BQfilterInit(){
	/*BiQuadFilterInit(&XFiter, 15, 1000, 0.32);
	BiQuadFilterInit(&YFiter, 15, 1000, 0.32);
	BiQuadFilterInit(&ZFiter, 15, 1000, 0.32);*/
	for(int i = 0; i < 3; i++){
		biquadFilterInitLPF(&MPU6050.Alowpasfilter2[i], 40, 1000);
		biquadFilterInitLPF(&MPU6050.Glowpasfilter2[i], 40, 1000);
	}
}



void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{

    // Read 14 BYTES of data starting from ACCEL_XOUT_H register

    if(HAL_I2C_Mem_Read_IT(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Read_Data, 14) != HAL_OK){
    	int fer = 1;
    }



}

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gyrox, float gyroy, float gyroz)
        {
            float q1 = qm[0], q2 = qm[1], q3 = qm[2], q4 = qm[3];         // short name local variable for readability
            float norm;                                               // vector norm
            float f1, f2, f3;                                         // objetive funcyion elements
            float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
            float qDot1, qDot2, qDot3, qDot4;
            float hatDot1, hatDot2, hatDot3, hatDot4;
            float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // gyro bias error

            // Auxiliary variables to avoid repeated arithmetic
            float _halfq1 = 0.5f * q1;
            float _halfq2 = 0.5f * q2;
            float _halfq3 = 0.5f * q3;
            float _halfq4 = 0.5f * q4;
            float _2q1 = 2.0f * q1;
            float _2q2 = 2.0f * q2;
            float _2q3 = 2.0f * q3;
            float _2q4 = 2.0f * q4;
            float _2q1q3 = 2.0f * q1 * q3;
            float _2q3q4 = 2.0f * q3 * q4;

            // Normalise accelerometer measurement
            norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Compute the objective function and Jacobian
            f1 = _2q2 * q4 - _2q1 * q3 - ax;
            f2 = _2q1 * q2 + _2q3 * q4 - ay;
            f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
            J_11or24 = _2q3;
            J_12or23 = _2q4;
            J_13or22 = _2q1;
            J_14or21 = _2q2;
            J_32 = 2.0f * J_14or21;
            J_33 = 2.0f * J_11or24;

            // Compute the gradient (matrix multiplication)
            hatDot1 = J_14or21 * f2 - J_11or24 * f1;
            hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
            hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
            hatDot4 = J_14or21 * f1 + J_11or24 * f2;

            // Normalize the gradient
            norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
            hatDot1 /= norm;
            hatDot2 /= norm;
            hatDot3 /= norm;
            hatDot4 /= norm;

            // Compute estimated gyroscope biases
            gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
            gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
            gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

            // Compute and remove gyroscope biases
            gbiasx = gerrx * dt * zeta;
            gbiasy = gerry * dt * zeta;
            gbiasz = gerrz * dt * zeta;
            gyrox -= gbiasx;
            gyroy -= gbiasy;
            gyroz -= gbiasz;

            // Compute the quaternion derivative
            qDot1 = -_halfq2 * gyrox - _halfq3 * gyroy - _halfq4 * gyroz;
            qDot2 =  _halfq1 * gyrox + _halfq3 * gyroz - _halfq4 * gyroy;
            qDot3 =  _halfq1 * gyroy - _halfq2 * gyroz + _halfq4 * gyrox;
            qDot4 =  _halfq1 * gyroz + _halfq2 * gyroy - _halfq3 * gyrox;

            // Compute then integrate estimated quaternion derivative
            q1 += (qDot1 -(beta * hatDot1)) * dt;
            q2 += (qDot2 -(beta * hatDot2)) * dt;
            q3 += (qDot3 -(beta * hatDot3)) * dt;
            q4 += (qDot4 -(beta * hatDot4)) * dt;

            // Normalize the quaternion
            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            norm = 1.0f/norm;
            qm[0] = q1 * norm;
            qm[1] = q2 * norm;
            qm[2] = q3 * norm;
            qm[3] = q4 * norm;
        }


void Calculate_all(uint8_t Rec_Data[14], MPU6050_t *DataStruct){
	int16_t temp;

		/*DataStruct->Accel_X_RAW = CalcBiQuad(&YFiter, (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]));
	    DataStruct->Accel_Y_RAW = CalcBiQuad(&YFiter, (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]));
	    DataStruct->Accel_Z_RAW = CalcBiQuad(&ZFiter, (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]));*/
		DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
		DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
		DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
	    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
	    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
	    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
	    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

	    DataStruct->Ax = biquadFilterApplyDF1(&MPU6050.Alowpasfilter2[0],  DataStruct->Accel_X_RAW / 2048.0);
	    DataStruct->Ay = biquadFilterApplyDF1(&MPU6050.Alowpasfilter2[1], DataStruct->Accel_Y_RAW / 2048.0);
	    DataStruct->Az = biquadFilterApplyDF1(&MPU6050.Alowpasfilter2[2], DataStruct->Accel_Z_RAW / 2048.0);
	    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
	    DataStruct->Gx = biquadFilterApplyDF1(&MPU6050.Glowpasfilter2[0], DataStruct->Gyro_X_RAW / 32.8 + MPU_CORRECTION_GX);
	    DataStruct->Gy = biquadFilterApplyDF1(&MPU6050.Glowpasfilter2[1], DataStruct->Gyro_Y_RAW / 32.8 + MPU_CORRECTION_GY);
	    DataStruct->Gz = biquadFilterApplyDF1(&MPU6050.Glowpasfilter2[2], DataStruct->Gyro_Z_RAW / 32.8 + MPU_CORRECTION_GZ);

	    for(int i = 0; i < 3; i++){
	    	biquadFilterUpdateLPF(&MPU6050.Alowpasfilter2[i], 40, 1000);
	    	biquadFilterUpdateLPF(&MPU6050.Glowpasfilter2[i], 40, 1000);
	    }

	    dt = (double)(microes() - timer) / 1000000.0f;
	    timer = microes();

	    if(microes() > 6000000){
	    //imuUpdateAttitude(microes());
	    MadgwickQuaternionUpdate( DataStruct->Ax, DataStruct->Ay, DataStruct->Az, DataStruct->Gx  * M_PI / 180.0f, DataStruct->Gy  * M_PI / 180.0f, DataStruct->Gz  * M_PI / 180.0f);
	    DataStruct->YAW   = atan2(2.0f * (qm[1] * qm[2] + qm[0] * qm[3]), qm[0] * qm[0] + qm[1] * qm[1] - qm[2] * qm[2] - qm[3] * qm[3]) * 180.0f / M_PI;
	    DataStruct->PITCH= -asin(2.0f * (qm[1] * qm[3] - qm[0] * qm[2])) * 180.0f / M_PI;
	    DataStruct->ROLL  = atan2(2.0f * (qm[0] * qm[1] + qm[2] * qm[3]), qm[0] * qm[0] - qm[1] * qm[1] - qm[2] * qm[2] + qm[3] * qm[3]) * 180.0f / M_PI;

	    }
	   // DataStruct->AngleX = DataStruct->Ax;
	  //  DataStruct->AngleY = DataStruct->Ay;
	   // DataStruct->YAW = DataStruct->Az;

	   /*Legacy code*/

	    /* float roll;
	    float roll_sqrt = sqrt(
	        DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
	    if (roll_sqrt != 0.0)
	    {
	        roll = atan2(DataStruct->Accel_Y_RAW, roll_sqrt) * RAD_TO_DEG;
	    }
	    else
	    {
	        roll = 0.0;
	    }
	    DataStruct->ROLL = roll;

	    float pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
	    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90))
	    {
	        //KalmanY.angle = pitch;
	        //DataStruct->KalmanAngleY = pitch;
	    	 DataStruct->PITCH = pitch;
	    }
	    else
	    {
	        //DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
	    	DataStruct->PITCH = pitch;
	    }
	    if (fabs(DataStruct->KalmanAngleY) > 90)
	        DataStruct->Gx = -DataStruct->Gx;
	   // DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);


	    /*float roll = atan(DataStruct->Accel_Y_RAW / DataStruct->Accel_Z_RAW)  * RAD_TO_DEG;
	    DataStruct->AngleX = roll;

	    float pich_sqrt = sqrt(DataStruct->Accel_Y_RAW * DataStruct->Accel_Y_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
	    float pitch = atan(-DataStruct->Accel_X_RAW / pich_sqrt)  * RAD_TO_DEG;
	    DataStruct->AngleY = pitch;*/

	    /*float delta_angle = (DataStruct->Gz - DataStruct->Gz_offset) *dt  / RAD_TO_DEG;

	    DataStruct->YAW += (delta_angle - (sin(-DataStruct->AngleX / RAD_TO_DEG) * delta_angle) - (sin(-DataStruct->AngleX / RAD_TO_DEG) * delta_angle)) * RAD_TO_DEG;*/

}

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Doesn't work!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
void writeByte(uint16_t adr, uint16_t reg, uint16_t data){
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg, 1, data, 1, i2c_timeout);
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU6050(float * dest1, float * dest2)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

// reset device, reset all registers, clear gyro and accelerometer bias registers
 // HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1, 1, 0x80, 1, i2c_timeout);
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  //wait(0.1);
  HAL_Delay(100);

// get stable time source
// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  //HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1, 1, 0x01, 1, i2c_timeout);
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);
 // HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_2, 1, 0x00, 1, i2c_timeout);
  writeByte(MPU6050_ADDRESS, PWR_MGMT_2, 0x00);
 // wait(0.2);
  HAL_Delay(200);

// Configure device for bias calculation
 // HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, INT_ENABLE, 1, 0x00, 1, i2c_timeout);
  writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
 // HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, FIFO_EN, 1, 0x00, 1, i2c_timeout);
  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
 // HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1, 1, 0x00, 1, i2c_timeout);
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
 // HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, I2C_MST_CTRL, 1, 0x00, 1, i2c_timeout);
  writeByte(MPU6050_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
 // HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, USER_CTRL, 1, 0x00, 1, i2c_timeout);
  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  //HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, USER_CTRL, 1, 0x0C, 1, i2c_timeout);
  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  //wait(0.015);
  HAL_Delay(200);

// Configure MPU6050 gyro and accelerometer for bias calculation
  //HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, CONFIG, 1, 0x01, 1, i2c_timeout);
  writeByte(MPU6050_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
 // HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, USER_CTRL, 1, 0x00, 1, i2c_timeout);
  writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
 // HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, USER_CTRL, 1, 0x00, 1, i2c_timeout);
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
//  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, USER_CTRL, 1, 0x00, 1, i2c_timeout);
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
 // HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, USER_CTRL, 1, 0x00, 1, i2c_timeout);
  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
 // HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, USER_CTRL, 1, 0x00, 1, i2c_timeout);
  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
  HAL_Delay(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

// At end of sample accumulation, turn off FIFO sensor read
 // HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, USER_CTRL, 1, 0x00, 1, i2c_timeout);
  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO

  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, FIFO_COUNTH, 1, &data[0], 2, i2c_timeout);
  //readBytes(MPU6050_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, FIFO_R_W, 1, &data[0], 12, i2c_timeout);
    //readBytes(MPU6050_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];

}
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}

// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

// Push gyro biases to hardware registers
  writeByte(MPU6050_ADDRESS, XG_OFFS_USRH, data[0]);
  writeByte(MPU6050_ADDRESS, XG_OFFS_USRL, data[1]);
  writeByte(MPU6050_ADDRESS, YG_OFFS_USRH, data[2]);
  writeByte(MPU6050_ADDRESS, YG_OFFS_USRL, data[3]);
  writeByte(MPU6050_ADDRESS, ZG_OFFS_USRH, data[4]);
  writeByte(MPU6050_ADDRESS, ZG_OFFS_USRL, data[5]);

  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, XA_OFFSET_H, 1, &data[0], 2, i2c_timeout);
  //readBytes(MPU6050_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, YA_OFFSET_H, 1, &data[0], 2, i2c_timeout);
  //readBytes(MPU6050_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ZA_OFFSET_H, 1, &data[0], 2, i2c_timeout);
  //readBytes(MPU6050_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for(ii = 0; ii < 3; ii++) {
    if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

  // Push accelerometer biases to hardware registers
//  writeByte(MPU6050_ADDRESS, XA_OFFSET_H, data[0]);
//  writeByte(MPU6050_ADDRESS, XA_OFFSET_L_TC, data[1]);
//  writeByte(MPU6050_ADDRESS, YA_OFFSET_H, data[2]);
//  writeByte(MPU6050_ADDRESS, YA_OFFSET_L_TC, data[3]);
//  writeByte(MPU6050_ADDRESS, ZA_OFFSET_H, data[4]);
//  writeByte(MPU6050_ADDRESS, ZA_OFFSET_L_TC, data[5]);

// Output scaled accelerometer biases for manual subtraction in the main program
   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}





/*float Kalman_getAngle(Kalman_t *Kalman, float newAngle, float newRate, float dt)
{
	float rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    float S = Kalman->P[0][0] + Kalman->R_measure;
    float K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    float y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    float P00_temp = Kalman->P[0][0];
    float P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};*/
