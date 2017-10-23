/*
 * mpu9250.c
 *
 *  Created on: 13 Αυγ 2017
 *      Author: Lefteris
 */

/* Includes ------------------------------------------------------------------*/
#include "mpu9250.h"

static MPU9250_InitTypeDef MPU9250InitDef;

uint8_t getMPU9250WhoAmI(){
	uint8_t* RxData = (uint8_t*) malloc(sizeof(uint8_t));
	*RxData = 0x0;
	HAL_I2C_Mem_Read(MPU9250InitDef.hi2c, MPU9250InitDef.hi2c->Init.OwnAddress1, MPU9250_REG_WHO_AM_I, 1, RxData, 1, MPU9250_TIMEOUT);
	return *RxData;
}

void writeRegByteMPU9250(uint8_t reg,  uint8_t value) {
	uint8_t* TxData = (uint8_t*) malloc(sizeof(uint8_t));
	*TxData = value;
	HAL_I2C_Mem_Write(MPU9250InitDef.hi2c, MPU9250InitDef.hi2c->Init.OwnAddress1, reg, 1, TxData, 1, MPU9250_TIMEOUT);
}

uint8_t readRegByteMPU9250(uint8_t reg){
	uint8_t* RxData = (uint8_t*) malloc(sizeof(uint8_t));
	RxData[0] = 0x0;
	HAL_I2C_Mem_Read(MPU9250InitDef.hi2c, MPU9250InitDef.hi2c->Init.OwnAddress1, reg, 1, RxData, 1, MPU9250_TIMEOUT);
	return *RxData;
}

void initMPU9250(I2C_HandleTypeDef *hi2c){

	MPU9250InitDef.hi2c = hi2c;

	writeRegByteMPU9250(MPU9250_REG_PWR_MGMT_1, MPU9250_RESET_MASK & 0xFF);//reset the whole module first

	HAL_Delay(50);    //wait for 50ms for the gyro to stable

	writeRegByteMPU9250(MPU9250_REG_PWR_MGMT_1, 0x1);//PLL with Z axis gyroscope reference

	writeRegByteMPU9250(MPU9250_REG_CONFIG, 0x01);        //DLPF_CFG = 1: Fs=1khz; bandwidth=42hz

	writeRegByteMPU9250(MPU9250_REG_SMPLRT_DIV, 0x01);    //1kHz / (1+SMPLRT_DIV) sample rate ~ 2.9ms

	writeRegByteMPU9250(MPU9250_REG_GYRO_CONFIG, MPU9250_GYRO_FS_2000);    //Gyro full scale setting

	writeRegByteMPU9250(MPU9250_REG_ACCEL_CONFIG, MPU9250_ACCEL_FS_2);    //Accel full scale setting

	//writeRegByteMPU9250(MPU9250_REG_INT_PIN_CFG, 0x30);        //interrupt status bits are cleared on any read operation

	//writeRegByteMPU9250(MPU9250_REG_INT_ENABLE, 0x01);        //Interrupt occurs when data is ready. The interrupt routine is in the stm32l1xx_it.c file.

	writeRegByteMPU9250(MPU9250_REG_SIGNAL_PATH_RESET, 0x07);//reset gyro and accel sensor
}

int16_t getMPU9250RawAccelX(){
	uint8_t* accel = (uint8_t*) malloc(sizeof(uint8_t)*2);
	*accel = 0x0;
	accel[1] = readRegByteMPU9250(MPU9250_REG_ACCEL_XOUT_H);
	accel[0] = readRegByteMPU9250(MPU9250_REG_ACCEL_XOUT_L);
	return (int16_t)accel[1]*256+accel[0];
}

int16_t getMPU9250RawAccelY(){
	uint8_t* accel = (uint8_t*) malloc(sizeof(uint8_t)*2);
	*accel = 0x0;
	accel[1] = readRegByteMPU9250(MPU9250_REG_ACCEL_YOUT_H);
	accel[0] = readRegByteMPU9250(MPU9250_REG_ACCEL_YOUT_L);
	return (int16_t)accel[1]*256+accel[0];
}

int16_t getMPU9250RawAccelZ(){
	uint8_t* accel = (uint8_t*) malloc(sizeof(uint8_t)*2);
	*accel = 0x0;
	accel[1] = readRegByteMPU9250(MPU9250_REG_ACCEL_ZOUT_H);
	accel[0] = readRegByteMPU9250(MPU9250_REG_ACCEL_ZOUT_L);
	return (int16_t)accel[1]*256+accel[0];
}

double getMPU9250AccelX(){
	return 16384.0;
}
double getMPU9250AccelY(){
	return ((double)getMPU9250RawAccelY() / 16384.0f);
}
double getMPU9250AccelZ(){
	return ((double)getMPU9250RawAccelZ() / 16384.0f);
}
