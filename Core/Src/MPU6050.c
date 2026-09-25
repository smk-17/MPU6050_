/*
 * MPU6050.c
 *
 *  Created on: 08-Sept-2026
 *      Author: manoj_kumar
 */


#include "MPU6050.h"
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;

void MPU_Init(void){

 uint8_t onbus =0;
 onbus = MPU_RegRead(WHO_AM_I);
 printf("%d\n",onbus);
 if (onbus != MPU_7BITADDR){

	 printf("DeviceNotFound\n");
	 return ;
 }
 else {

	 printf("WHO = 0x%02X\r\n", MPU_RegRead(WHO_AM_I));
	 MPU_RegWrite(PWR_MGMT,0x01);
	 HAL_Delay(100);
	 MPU_RegWrite(SAMP_RATE,0x07);
	 MPU_RegWrite(CONFIG,0x03);
	 MPU_RegWrite(GYRO_CONFIG,0x00);
	 MPU_RegWrite(ACC_CONFIG,0x00);
 }

}

void MPU_RegWrite(uint8_t reg, uint8_t value){

	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR ,reg, I2C_MEMADD_SIZE_8BIT, &value, 1 , 100);

}

uint8_t MPU_RegRead(uint8_t reg){

	uint8_t readvalue =0;
	HAL_I2C_Mem_Read(&hi2c1,MPU_ADDR, reg ,I2C_MEMADD_SIZE_8BIT, &readvalue, 1,100);
	return readvalue;
}

void MPU_Raw(RAW_DATA *raw){

	uint8_t buffer[14];

	//Reading Raw values from the MPU sensor

 HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1,MPU_ADDR, ACC_OUT,I2C_MEMADD_SIZE_8BIT, buffer, 14 , 100);

 if (status != HAL_OK)
    {
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        printf("I2C READ FAILED: %d\r\n", status);
        return;
    }


 //Each values of ACC and GYro in 2 regs H and L

 	raw->accX = (int16_t)((buffer[0] << 8) | buffer[1]);
    raw->accY = (int16_t)((buffer[2] << 8) | buffer[3]);
    raw->accZ = (int16_t)((buffer[4] << 8) | buffer[5]);

    raw->gyroX = (int16_t)((buffer[8] << 8) | buffer[9]);
    raw->gyroY = (int16_t)((buffer[10] << 8) | buffer[11]);
    raw->gyroZ = (int16_t)((buffer[12] << 8) | buffer[13]);
}

void MPU_Convert(RAW_DATA *raw, ACT_DATA *data ){

	data->accX = ((float)raw->accX)/ 16384.0f;
	data->accY = ((float)raw->accY)/  16384.0f;
	data->accZ = ((float)raw->accZ)/ 16384.0f;

	data->gyroX = ((float)raw->gyroX)/ 131.0f;
	data->gyroY =((float)raw->gyroY)/ 131.0f;
	data->gyroZ = ((float)raw->gyroZ)/ 131.0f;

}
