/*
 * MPU6050.h
 *
 *  Created on: 08-Sept-2026
 *      Author: manoj_kumar
 */


#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdint.h>
#include "main.h"

#define MPU_ADDR    (0x68<<1)
#define MPU_7BITADDR  0x68

#define WHO_AM_I 		0x75U
#define PWR_MGMT 		0x6BU
#define USR_CTRL 		0x6AU
#define SAMP_RATE       0x19U
#define CONFIG          0x1AU
#define GYRO_CONFIG     0x1BU
#define ACC_CONFIG      0x1CU
#define ACC_OUT         0X3BU
#define INT_PIN_CFG     0x37U


typedef struct {

	int16_t accX;
	int16_t accY;
	int16_t accZ;

	int16_t gyroX;
	int16_t gyroY;
	int16_t gyroZ;

}RAW_DATA;


typedef struct {

	float accX;
	float accY;
	float accZ;

	float gyroX;
	float gyroY;
	float gyroZ;

}ACT_DATA;

void MPU_Init(void);
void MPU_RegWrite(uint8_t reg, uint8_t value);
uint8_t MPU_RegRead(uint8_t reg);
void MPU_Raw(RAW_DATA *raw);
void MPU_Convert(RAW_DATA *raw, ACT_DATA *data);



#endif /* INC_MPU6050_H_ */
