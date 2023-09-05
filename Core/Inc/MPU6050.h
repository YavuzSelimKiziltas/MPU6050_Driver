/*
 * MPU6050.h
 *
 * MPU6050 Gyro Sensor Driver
 *
 *  Created on: Sep 5, 2023
 *      Author: Yavuz
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

// Needed for I2C
#include "stm32f4xx_hal.h"

/* Self Test Registers */
#define MPU6050_SELF_TEST_X				0X0D
#define MPU6050_SELF_TEST_Y				0x0E
#define MPU6050_SELF_TEST_Z				0x0F
#define MPU6050_SELF_TEST_A				0x10

#define MPU6050_SMPLRT_DIV					0x19		// Sample Rate Divider
#define MPU6050_CONFIG						0x1A		// Configuration
#define MPU6050_GYRO_CONFIG				0x1B		// Gyroscope Configuration
#define MPU6050_ACCEL_CONFIG				0x1C		// Accelerometer Configuration
#define MPU6050_FIFOEN						0x23		// FIFO Enable
#define MPU6050_I2C_MST_CTRL				0x24		// I2C Master Control

/* I2C Slave 0 Control Registers */
#define MPU6050_I2C_SLV0_ADDR				0x25
#define MPU6050_I2C_SLV0_REG				0x26
#define MPU6050_I2C_SLV0_CTRL				0x27
#define MPU6050_I2C_SLV0_DO				0x63

/* I2C Slave 1 Control Registers */
#define MPU6050_I2C_SLV1_ADDR				0x28
#define MPU6050_I2C_SLV1_REG				0x29
#define MPU6050_I2C_SLV1_CTRL				0x2A
#define MPU6050_I2C_SLV1_DO				0x64

/* I2C Slave 2 Control Registers */
#define MPU6050_I2C_SLV2_ADDR				0x2B
#define MPU6050_I2C_SLV2_REG				0x2C
#define MPU6050_I2C_SLV2_CTRL				0x2D
#define MPU6050_I2C_SLV2_DO				0x65

/* I2C Slave 3 Control Registers */
#define MPU6050_I2C_SLV3_ADDR				0x2E
#define MPU6050_I2C_SLV3_REG				0x2F
#define MPU6050_I2C_SLV3_CTRL				0x30
#define MPU6050_I2C_SLV3_DO				0x66

/* I2C Slave 4 Control Registers */
#define MPU6050_I2C_SLV4_ADDR				0x31
#define MPU6050_I2C_SLV4_REG				0x32
#define MPU6050_I2C_SLV4_DO				0x33
#define MPU6050_I2C_SLV4_CTRL				0x34
#define MPU6050_I2C_SLV4_DI				0x35

#define MPU6050_I2C_MST_STATUS			0x36		// I2C Master Status	(Read Only)

/* Interrupt Pin Registers */
#define MPU6050_INT_PIN_CFG				0x37		// Interrupt Pin Configuration
#define MPU6050_INT_ENABLE					0x38		// Interrupt Enable
#define MPU6050_INT_STATUS					0x3A		// Interrupt Status

/* Accelerometer Measurements */
#define MPU6050_ACCEL_XOUT_H				0x3B
#define MPU6050_ACCEL_XOUT_L				0x3C
#define MPU6050_ACCEL_YOUT_H				0x3D
#define MPU6050_ACCEL_YOUT_L				0x3E
#define MPU6050_ACCEL_ZOUT_H				0x3F
#define MPU6050_ACCEL_ZOUT_L				0x40

/* Temperature Measurements */
#define MPU6050_TEMP_OUT_H					0x41
#define MPU6050_TEMP_OUT_L					0x42

/* Gyroscope Measurements */
#define MPU6050_GYRO_XOUT_H				0x43
#define MPU6050_GYRO_XOUT_L				0x44
#define MPU6050_GYRO_YOUT_H				0x45
#define MPU6050_GYRO_YOUT_L				0x46
#define MPU6050_GYRO_ZOUT_H				0x47
#define MPU6050_GYRO_ZOUT_L				0x48

#define MPU6050_I2C_MST_DELAY_CTRL		0x67	// I2C Master Delay Control
#define MPU6050_SIGNAL_PATH_RESET		0x68	// Signal Path Reset
#define MPU6050_USER_CTRL					0x6A	// User Control

/* Power Management */
#define MPU6050_PWR_MGMT_1					0x6B
#define MPU6050_PWR_MGMT_2					0x6C

/* FIFO Registers */
#define MPU6050_FIFO_COUNT_H				0x72
#define MPU6050_FIFO_COUNT_L				0x73
#define MPU6050_FIFO_R_W					0x74

#define MPU6050_WHO_AM_I					0x75


// MPU6050 structure to hold accelorometer and gyro data
typedef struct
{
	 I2C_HandleTypeDef *i2cHandle;

    int accel_x;

    int accel_y;

    int accel_z;

    int gyro_x;

    int gyro_y;

    int gyro_z;

    int temp;

} MPU6050;







#endif /* INC_MPU6050_H_ */
