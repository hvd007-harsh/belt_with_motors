/*
 * icm20948.h
 *
 *  Created on: Jul 25, 2023
 *      Author: harsh
 */

#ifndef INC_ICM20948_H_
#define INC_ICM20948_H_

#include <stdint.h>
#include <i2c.h>


// BANK ADDRESS
#define REG_BANK_SEL   0x7F

//Bank - 0

#define WHO_AM_I_REG   0x00
#define USER_CTRL      0x30
#define LP_CONFIG      0x05
#define PWR_MGMT_1     0x06
#define PWR_MGMT_2     0x07
#define INT_PIN_CFG    0x0F
#define INT_ENABLE     0x10
#define INT_ENABLE_1   0x17
#define INT_ENABLE_2   0x12

//Acceleration registers
#define ACCEL_XOUT_H   0x2D
#define ACCEL_XOUT_L   0x2E
#define ACCEL_YOUT_H   0x2F
#define ACCEL_YOUT_L   0x30
#define ACCEL_ZOUT_H   0x31
#define ACCEL_ZOUT_L   0x32

//Gyroscope registers
#define GYRO_XOUT_H    0x33
#define GYRO_XOUT_L    0x34
#define GYRO_YOUT_H    0x35
#define GYRO_YOUT_L    0x36
#define GYRO_ZOUT_H    0x37
#define GYRO_ZOUT_L    0x36

//Temperature Registers

#define TEMP_OUT_H     0x39
#define TEMP_OUT_L     0x3A

//CONFIG OF SENSOR
#define GYRO_CONFIG    0x01
#define GYRO_CONFIG2   0x02
#define ACCEL_CONFIG   0x14
#define ACCEL_CONFIG2   0x15
#define TEMP_CONFIG    0x53
#define MOD_CTRL_USR   0x54
#define DELAY_TIMEL    0x28
#define DELAY_TIMEH    0x29

#define ICM20948_ADDR  0x69
#define ICM20948_ADDR_LOW 0x68

typedef struct {
	int16_t ACCEL_X_RAW;
	int16_t ACCEL_Y_RAW;
	int16_t ACCEL_Z_RAW;
	double Ax;
	double Ay;
	double Az;

	int16_t Gyro_X_RAW;
	int16_t Gyro_Y_RAW;
	int16_t Gyro_Z_RAW;
	double Gx;
	double Gy;
	double Gz;
	double NormGx;
	double NormGy;
	double NormGz;

	float Temperature;

	double KalmanAngleX;
	double KalmanAngleY;
} ICM20948_t;

// Kalman structure
typedef struct {
	double Q_angle;
	double Q_bias;
	double R_measure;
	double angle;
	double bias;
	double P[2][2];
} Kalman_t;

uint8_t ICM20948_Init(I2C_HandleTypeDef *I2Cx);

void ICM20948_Read_Accel(I2C_HandleTypeDef *I2Cx, ICM20948_t *DataStruct);

void ICM20948_Read_Gyro(I2C_HandleTypeDef *I2Cx, ICM20948_t *DataStruct);

void ICM20948_Read_Temp(I2C_HandleTypeDef *I2Cx, ICM20948_t *DataStruct);

void ICM20948_Read_All(I2C_HandleTypeDef *I2Cx, ICM20948_t *DataStruct);

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate,
		double dt);

#endif /* INC_ICM20948_H_ */

