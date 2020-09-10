/*
 * MPU925.h
 *
 *  Created on: 23 ��� 2018 �.
 *      Author: Max
 */

#ifndef MPU925_H_
#define MPU925_H_

#include "main.h"
#include "MPU9250_Config.h"
#include <stdbool.h>

typedef enum GyroRange_ {
	GYRO_RANGE_250DPS = 0,
	GYRO_RANGE_500DPS,
	GYRO_RANGE_1000DPS,
	GYRO_RANGE_2000DPS,
} GyroRange;

typedef enum AccelRange_ {
	ACCEL_RANGE_2G = 0,
	ACCEL_RANGE_4G,
	ACCEL_RANGE_8G,
	ACCEL_RANGE_16G,
} AccelRange;

typedef enum DLPFBandwidth_ {
	DLPF_BANDWIDTH_184HZ = 0,
	DLPF_BANDWIDTH_92HZ,
	DLPF_BANDWIDTH_41HZ,
	DLPF_BANDWIDTH_20HZ,
	DLPF_BANDWIDTH_10HZ,
	DLPF_BANDWIDTH_5HZ,
} DLPFBandwidth;

typedef enum SampleRateDivider_ {
	LP_ACCEL_ODR_0_24HZ = 0,
	LP_ACCEL_ODR_0_49HZ,
	LP_ACCEL_ODR_0_98HZ,
	LP_ACCEL_ODR_1_95HZ,
	LP_ACCEL_ODR_3_91HZ,
	LP_ACCEL_ODR_7_81HZ,
	LP_ACCEL_ODR_15_63HZ,
	LP_ACCEL_ODR_31_25HZ,
	LP_ACCEL_ODR_62_50HZ,
	LP_ACCEL_ODR_125HZ,
	LP_ACCEL_ODR_250HZ,
	LP_ACCEL_ODR_500HZ,
} SampleRateDivider;

uint8_t MPU9250_Init(void);
/* read the data, each argument should point to an array for x, y, and z */
HAL_StatusTypeDef MPU9250_GetData_Phys(float* AccData, float* MagData, float* GyroData);
HAL_StatusTypeDef MPU9250_GetData_Dig(int16_t* AccData, int16_t* MagData, int16_t* GyroData);

/* sets the sample rate divider to values other than default */
HAL_StatusTypeDef MPU9250_SetSampleRateDivider(SampleRateDivider srd);
/* sets the DLPF bandwidth to values other than default */
HAL_StatusTypeDef MPU9250_SetDLPFBandwidth(DLPFBandwidth bandwidth);
/* sets the gyro full scale range to values other than default */
HAL_StatusTypeDef MPU9250_SetGyroRange(uint8_t range);
/* sets the accelerometer full scale range to values other than default */
HAL_StatusTypeDef MPU9250_SetAccelRange(uint8_t range);

// Sets the IMU to low power accelerometer mode, turning everything off except for the accelerometer
// Takes an argument to specify the sample frequency of the accelerometer
/*	odr | frequency in Hz
	---------------------
	 0	|      	0.24	|
	 1	|		0.49	|
	 2	|		0.98	|
	 3	|		1.95	|
	 4	|		3.91	|
	 5	|		7.81	|
	 6	|	   15.63	|
	 7	|	   31.25	|
	 8	|	   62.50	|	
	 9	|	  125.00	|
	 10	|	  250.00	|
	 11	|	  500.00	|
	---------------------  */
HAL_StatusTypeDef MPU9250_AccelLP(uint8_t odr);

#endif /* MPU925_H_ */





