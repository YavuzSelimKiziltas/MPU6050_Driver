/*
 * MPU6050.c
 *
 *  Created on: Sep 5, 2023
 *      Author: Yavuz
 */

#include "MPU6050.h"


/* @brief MPU6050 Sensor initialization function
 * @param Typedef structure that holds i2c handle, gyro and acclerometer data
 * @param Related I2C Handle
 * @retval Returns the number of errors during initialization
 * */
uint8_t MPU6050_Init(MPU6050 *dev, I2C_HandleTypeDef *hi2c)
{
	dev->i2cHandle = hi2c;

	dev->accel_x = 0;
	dev->accel_y = 0;
	dev->accel_z = 0;
	dev->gyro_x  = 0;
	dev->gyro_y  = 0;
	dev->gyro_z  = 0;

	uint8_t errorNumber = 0;
	uint8_t regData = 0x00;
	HAL_StatusTypeDef status;

	/* Perform Reset Sequence (Page 41) */
	status = MPU6050_ReadRegister(dev, MPU6050_PWR_MGMT_1, &regData);
	regData &= ~(1 << MPU6050_DEVICE_RESET);
	status = MPU6050_WriteRegister(dev, MPU6050_PWR_MGMT_1, &regData);
	HAL_Delay(100);
	errorNumber += (status != HAL_OK);


	status = MPU6050_ReadRegister(dev, MPU6050_SIGNAL_PATH_RESET, &regData);
	regData &= ~(7 << 0);	// GYRO_RESET = ACCEL_RESET = TEMP_RESET = Bit 2:0
	status = MPU6050_WriteRegister(dev, MPU6050_SIGNAL_PATH_RESET, &regData);
	HAL_Delay(100);
	errorNumber += (status != HAL_OK);

	regData |= (1 << MPU6050_CLK_PLL_XGYRO);	// Recommended Clock Source
	regData &= ~(1 << MPU6050_SLEEP_ENABLE);	// Disable Sleep Enabled
	regData &= ~(1 << MPU6050_TEMP_DISABLE);	// Temperature Reading Enabled

	status = MPU6050_WriteRegister(dev, MPU6050_PWR_MGMT_1, &regData);
	errorNumber += (status != HAL_OK);

	/* Gyroscope Settings (Page 14) */
	status = MPU6050_ReadRegister(dev, MPU6050_GYRO_CONFIG, &regData);
	errorNumber += (status != HAL_OK);

	regData |= (MPU6050_GYRO_FS_250 << 3);
	status = MPU6050_WriteRegister(dev, MPU6050_GYRO_CONFIG, &regData);
	errorNumber += (status != HAL_OK);

	/* Accelerometer Settings (Page 15) */
	status = MPU6050_ReadRegister(dev, MPU6050_ACCEL_CONFIG, &regData);
	errorNumber += (status != HAL_OK);

	regData |= (MPU6050_ACCEL_FS_2 << 3);
	status = MPU6050_WriteRegister(dev, MPU6050_ACCEL_CONFIG, &regData);
	errorNumber += (status != HAL_OK);


	/* Check if the address is correct */
	MPU6050_ReadRegister(dev, MPU6050_WHO_AM_I, &regData);
	if(regData != 0x68)
		errorNumber = 255;

	return errorNumber;
}


void MPU6050_Read_All(MPU6050 *dev)
{
	uint8_t regData[14];
	int16_t tempRaw;

	MPU6050_ReadRegisters(dev, MPU6050_ACCEL_XOUT_H, regData, 14);

	dev->accel_x = (((int16_t)regData[0]) << 8) | regData[1];

	dev->accel_y = (((int16_t)regData[2]) << 8) | regData[3];

	dev->accel_z = (((int16_t)regData[4]) << 8) | regData[5];

	tempRaw 		 = (((int16_t)regData[6]) << 8) | regData[7];
	dev->temp	 = (tempRaw / 340) + 36.53;

	dev->gyro_x = (((int16_t)regData[8]) << 8) | regData[9];

	dev->gyro_y = (((int16_t)regData[10]) << 8) | regData[11];

	dev->gyro_z = (((int16_t)regData[12]) << 8) | regData[13];
}

void MPU6050_Read_Gyro(MPU6050 *dev, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z)
{
	uint8_t regData[6];

	MPU6050_ReadRegisters(dev, MPU6050_GYRO_XOUT_H, regData, 6);

	*gyro_x = (((int16_t)regData[0]) << 8) | regData[1];

	*gyro_y = (((int16_t)regData[2]) << 8) | regData[3];

	*gyro_z = (((int16_t)regData[4]) << 8) | regData[5];

	dev->gyro_x = *gyro_x;
	dev->gyro_y = *gyro_y;
	dev->gyro_z = *gyro_z;
}

void MPU6050_Read_Accel(MPU6050 *dev, int16_t *accel_x, int16_t *accel_y, int16_t *accel_z)
{
	uint8_t regData[6];

	MPU6050_ReadRegisters(dev, MPU6050_ACCEL_XOUT_H, regData, 6);

	*accel_x = (((int16_t)regData[0]) << 8) | regData[1];

	*accel_y = (((int16_t)regData[2]) << 8) | regData[3];

	*accel_z = (((int16_t)regData[4]) << 8) | regData[5];

	dev->accel_x = *accel_x;
	dev->accel_y = *accel_y;
	dev->accel_z = *accel_z;
}

void MPU6050_Read_Temp(MPU6050 *dev, int16_t *temp)
{
	uint8_t regData[2];

	MPU6050_ReadRegisters(dev, MPU6050_TEMP_OUT_H, regData, 2);

	*temp= (((int16_t)regData[0]) << 8) | regData[1];

	// Conversion to Celcius (Page 30)
	*temp = ((*temp) / 340) + 36.53;

	dev->temp = *temp;
}

HAL_StatusTypeDef MPU6050_ReadRegister(MPU6050 *dev, uint8_t reg, uint8_t *data)
{
	return HAL_I2C_Mem_Read(dev->i2cHandle, MPU6050_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU6050_ReadRegisters(MPU6050 *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
	return HAL_I2C_Mem_Read(dev->i2cHandle, MPU6050_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU6050_WriteRegister(MPU6050 *dev, uint8_t reg, uint8_t *data)
{
	return HAL_I2C_Mem_Write(dev->i2cHandle, MPU6050_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU6050_WriteRegisters(MPU6050 *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
	return HAL_I2C_Mem_Write(dev->i2cHandle, MPU6050_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}
