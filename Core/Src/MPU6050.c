/*
 * MPU6050.c
 *
 *  Created on: Sep 5, 2023
 *      Author: Yavuz
 */

#include "MPU6050.h"
#include <math.h>

/* @brief MPU6050 Sensor initialization function
 * @param Typedef structure that holds i2c handle, gyro and accelerometer and temperature data
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


/* @brief  Reads Gyro, Acceleroemter and Temperature data and stores it inside MPU6050 Typedef Struct
 * @param  Typedef structure that holds i2c handle, gyro and accelerometer data
 * @retval None
 * */
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

/* @brief  Reads only Gyro data and stores it inside function parameters
 * @param  Typedef structure that holds i2c handle, gyro and accelerometer data
 * @param  int16_t type parameters to hold 3 axis gyro data
 * @retval None
 * */
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

/* @brief  Reads only Accelerometer data and stores it inside function parameters
 * @param  Typedef structure that holds i2c handle, gyro and accelerometer data
 * @param  int16_t type parameters to hold 3 axis acceloremeter data
 * @retval None
 * */
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

/* @brief  Reads only Temperature data and stores it inside function parameters
 * @param  Typedef structure that holds i2c handle, gyro and accelerometer data
 * @param  int16_t type to store temperature data (In Celcius)
 * @retval None
 * */
void MPU6050_Read_Temp(MPU6050 *dev, int16_t *temp)
{
	uint8_t regData[2];

	MPU6050_ReadRegisters(dev, MPU6050_TEMP_OUT_H, regData, 2);

	*temp= (((int16_t)regData[0]) << 8) | regData[1];

	// Conversion to Celcius (Page 30)
	*temp = ((*temp) / 340) + 36.53;

	dev->temp = *temp;
}

/* @brief  Performs Self-Test on Gyroscope
 * @param  Typedef structure that holds i2c handle, gyro and accelerometer data
 * @param  Double type parameters to hold 3 axis Change from Factory Trim of the Self-Test Response(%)
 * @retval returns 1 if self-test is succesfull, else returns -1
 * */
uint8_t MPU6050_Gyro_SelfTest(MPU6050 *dev, double *xG_change, double *yG_change, double *zG_change)
{
	uint8_t regData = 0x00;
	uint8_t XG_TEST, YG_TEST, ZG_TEST;
	double X_FT, Y_FT, Z_FT;

	int16_t X_OUT_ST_EN, Y_OUT_ST_EN, Z_OUT_ST_EN;
	int16_t X_OUT_ST_DIS, Y_OUT_ST_DIS, Z_OUT_ST_DIS;
	int16_t X_STR, Y_STR, Z_STR;

	// Make sure Full Scale Range is +-250 dps & Self Test is enabled
	MPU6050_ReadRegister(dev, MPU6050_GYRO_CONFIG, &regData);
	regData |= (MPU6050_GYRO_FS_250 << 3);
	regData |= (1 << MPU6050_X_SELFTEST_EN) | (1 << MPU6050_Y_SELFTEST_EN) | (1 << MPU6050_Z_SELFTEST_EN);
	MPU6050_WriteRegister(dev, MPU6050_GYRO_CONFIG, &regData);

	// Gyroscope Output with Self-Test Enabled
	MPU6050_Read_Gyro(dev, &X_OUT_ST_EN, &Y_OUT_ST_EN, &Z_OUT_ST_EN);

	/* Read Self-Test Registers */
	MPU6050_ReadRegister(dev, MPU6050_SELF_TEST_X, &regData);
	XG_TEST = regData & (0x1F);

	MPU6050_ReadRegister(dev, MPU6050_SELF_TEST_Y, &regData);
	YG_TEST = regData & (0x1F);

	MPU6050_ReadRegister(dev, MPU6050_SELF_TEST_Z, &regData);
	ZG_TEST = regData & (0x1F);

	// Calculate Gyro_X Factory Trim Value
	if(XG_TEST == 0)
		X_FT = 0;
	else
		X_FT = 25 * 131 * pow(1.046, XG_TEST - 1);

	// Calculate Gyro_Y Factory Trim Value
	if(YG_TEST == 0)
		Y_FT = 0;
	else
		Y_FT = -25 * 131 * pow(1.046, YG_TEST - 1);

	// Calculate Gyro_Z Factory Trim Value
	if(ZG_TEST == 0)
		Z_FT = 0;
	else
		Z_FT = 25 * 131 * pow(1.046, ZG_TEST - 1);


	// Disable Self Test
	MPU6050_ReadRegister(dev, MPU6050_GYRO_CONFIG, &regData);
	regData &= ~((1 << MPU6050_X_SELFTEST_EN) | (1 << MPU6050_Y_SELFTEST_EN) | (1 << MPU6050_Z_SELFTEST_EN));
	MPU6050_WriteRegister(dev, MPU6050_GYRO_CONFIG, &regData);

	// Gyroscope Output with Self-Test Disabled
	MPU6050_Read_Gyro(dev, &X_OUT_ST_DIS, &Y_OUT_ST_DIS, &Z_OUT_ST_DIS);

	/* Calculate Self Test Response (STR) */
	X_STR = X_OUT_ST_EN - X_OUT_ST_DIS;
	Y_STR = Y_OUT_ST_EN - Y_OUT_ST_DIS;
	Z_STR = Z_OUT_ST_EN - Z_OUT_ST_DIS;

	/* Change from Factory Trim of the Self-Test Response(%) */
	*xG_change = (X_STR - X_FT) / X_FT;

	*yG_change = (Y_STR - Y_FT) / Y_FT;

	*zG_change = (Z_STR - Z_FT) / Z_FT;

	if((*xG_change > 14) || (*xG_change < -14) || (*yG_change > 14) || (*yG_change < -14) || (*zG_change > 14) || (*zG_change < -14))
		return -1;
	else
		return 1;
}


/* @brief  Performs Self-Test on Accelerometer
 * @param  Typedef structure that holds i2c handle, gyro and accelerometer data
 * @param  Double type parameters to hold 3 axis Change from Factory Trim of the Self-Test Response(%)
 * @retval returns 1 if self-test is succesfull, else returns -1
 * */
uint8_t MPU6050_Accel_SelfTest(MPU6050 *dev, double *xA_change, double *yA_change, double *zA_change)
{
	uint8_t regData = 0x00;

	uint8_t XA_TEST, YA_TEST, ZA_TEST;
	uint8_t XA_TEST_HIGH, YA_TEST_HIGH, ZA_TEST_HIGH;
	uint8_t XA_TEST_LOW, YA_TEST_LOW, ZA_TEST_LOW;

	double X_FT, Y_FT, Z_FT;

	int16_t X_OUT_ST_EN, Y_OUT_ST_EN, Z_OUT_ST_EN;
	int16_t X_OUT_ST_DIS, Y_OUT_ST_DIS, Z_OUT_ST_DIS;
	int16_t X_STR, Y_STR, Z_STR;


	// Make sure Full Scale Range is +-8 g & Self Test is enabled
	MPU6050_ReadRegister(dev, MPU6050_ACCEL_CONFIG, &regData);
	regData |= (MPU6050_ACCEL_FS_8 << 3);
	regData |= (1 << MPU6050_X_SELFTEST_EN) | (1 << MPU6050_Y_SELFTEST_EN) | (1 << MPU6050_Z_SELFTEST_EN);
	MPU6050_WriteRegister(dev, MPU6050_ACCEL_CONFIG, &regData);

	// Accelerometer Output with Self-Test Enabled
	MPU6050_Read_Accel(dev, &X_OUT_ST_EN, &Y_OUT_ST_EN, &Z_OUT_ST_EN);

	/* Read Self-Test Registers */
	MPU6050_ReadRegister(dev, MPU6050_SELF_TEST_X, &regData);
	XA_TEST_HIGH = (regData & (0xE0)) >> 5;
	MPU6050_ReadRegister(dev, MPU6050_SELF_TEST_A, &regData);
	XA_TEST_LOW  = (regData & (0x30)) >> 4;

	MPU6050_ReadRegister(dev, MPU6050_SELF_TEST_Y, &regData);
	YA_TEST_HIGH = (regData & (0xE0)) >> 5;
	MPU6050_ReadRegister(dev, MPU6050_SELF_TEST_A, &regData);
	YA_TEST_LOW  = (regData & (0x0C)) >> 2;

	MPU6050_ReadRegister(dev, MPU6050_SELF_TEST_Z, &regData);
	ZA_TEST_HIGH = (regData & (0xE0)) >> 5;
	MPU6050_ReadRegister(dev, MPU6050_SELF_TEST_A, &regData);
	ZA_TEST_LOW  = regData & (0x03);

	XA_TEST = (XA_TEST_HIGH << 2) + XA_TEST_LOW;
	YA_TEST = (YA_TEST_HIGH << 2) + YA_TEST_LOW;
	ZA_TEST = (ZA_TEST_HIGH << 2) + ZA_TEST_LOW;

	// Calculate Accel_X Factory Trim Value
	if(XA_TEST == 0)
		X_FT = 0;
	else
		X_FT = 4096 * 0.34 * pow(0.92 / 0.34, (XA_TEST - 1) / 30);

	// Calculate Accel_Y Factory Trim Value
	if(YA_TEST == 0)
		Y_FT = 0;
	else
		Y_FT = 4096 * 0.34 * pow(0.92 / 0.34, (YA_TEST - 1) / 30);

	// Calculate Accel_Z Factory Trim Value
	if(ZA_TEST == 0)
		Z_FT = 0;
	else
		Z_FT = 4096 * 0.34 * pow(0.92 / 0.34, (ZA_TEST - 1) / 30);


	// Disable Self Test
	MPU6050_ReadRegister(dev, MPU6050_ACCEL_CONFIG, &regData);
	regData &= ~((1 << MPU6050_X_SELFTEST_EN) | (1 << MPU6050_Y_SELFTEST_EN) | (1 << MPU6050_Z_SELFTEST_EN));
	MPU6050_WriteRegister(dev, MPU6050_ACCEL_CONFIG, &regData);

	// Accelerometer Output with Self-Test Disabled
	MPU6050_Read_Accel(dev, &X_OUT_ST_DIS, &Y_OUT_ST_DIS, &Z_OUT_ST_DIS);

	/* Calculate Self Test Response (STR) */
	X_STR = X_OUT_ST_EN - X_OUT_ST_DIS;
	Y_STR = Y_OUT_ST_EN - Y_OUT_ST_DIS;
	Z_STR = Z_OUT_ST_EN - Z_OUT_ST_DIS;

	/* Change from Factory Trim of the Self-Test Response(%) */
	*xA_change = (X_STR - X_FT) / X_FT;

	*yA_change = (Y_STR - Y_FT) / Y_FT;

	*zA_change = (Z_STR - Z_FT) / Z_FT;

	if((*xA_change > 14) || (*xA_change < -14) || (*yA_change > 14) || (*yA_change < -14) || (*zA_change > 14) || (*zA_change < -14))
		return -1;
	else
		return 1;
}


/* @brief  Low-Level I2C Read Function
 * @param  Typedef structure that holds i2c handle, gyro and accelerometer data
 * @param  reg parameter is the register address from MPU6050
 * @param  data is the parameter to hold register data
 * @retval returns HAL_StatusTypeDef
 * */
HAL_StatusTypeDef MPU6050_ReadRegister(MPU6050 *dev, uint8_t reg, uint8_t *data)
{
	return HAL_I2C_Mem_Read(dev->i2cHandle, MPU6050_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

/* @brief  Low-Level I2C Read Function to consecutive Addresses
 * @param  Typedef structure that holds i2c handle, gyro and accelerometer data
 * @param  reg parameter is the register address from MPU6050
 * @param  data is the parameter to hold register data
 * @param  length is the number of consecutive register address
 * @retval returns HAL_StatusTypeDef
 * */
HAL_StatusTypeDef MPU6050_ReadRegisters(MPU6050 *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
	return HAL_I2C_Mem_Read(dev->i2cHandle, MPU6050_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}


/* @brief  Low-Level I2C Write Function
 * @param  Typedef structure that holds i2c handle, gyro and accelerometer data
 * @param  reg parameter is the register address from MPU6050
 * @param  data is the parameter that will be written to register address
 * @retval returns HAL_StatusTypeDef
 * */
HAL_StatusTypeDef MPU6050_WriteRegister(MPU6050 *dev, uint8_t reg, uint8_t *data)
{
	return HAL_I2C_Mem_Write(dev->i2cHandle, MPU6050_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

/* @brief  Low-Level I2C Write Function to Consecutive Addresses
 * @param  Typedef structure that holds i2c handle, gyro and accelerometer data
 * @param  reg parameter is the register address from MPU6050
 * @param  data is the parameter that will be written to register address
 * @param  length is the number of consecutive register address
 * @retval returns HAL_StatusTypeDef
 * */
HAL_StatusTypeDef MPU6050_WriteRegisters(MPU6050 *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
	return HAL_I2C_Mem_Write(dev->i2cHandle, MPU6050_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}
