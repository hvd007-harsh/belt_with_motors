/*
 * icm20948.c
 *
 *  Created on: Aug 3, 2023
 *      Author: harsh
 */


#include <math.h>
#include "icm20948.h"

const uint16_t i2c_timeout = 100;
const double Accel_Z_corrector = 14418.00;

uint32_t timer;

HAL_StatusTypeDef ICM20948_WriteRegister(uint8_t regAddr, uint8_t data,
		I2C_HandleTypeDef *I2Cx) {
	return HAL_I2C_Mem_Write(I2Cx, (ICM20948_ADDR << 1), regAddr,
			I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef SelectBank(uint8_t bank, I2C_HandleTypeDef *I2Cx) {
	return ICM20948_WriteRegister(REG_BANK_SEL, (bank << 4), I2Cx);
}

uint8_t ICM20948_Init(I2C_HandleTypeDef *I2Cx) {
	uint8_t check;
//	uint8_t Data;

	HAL_Delay(100);

	while (HAL_I2C_Mem_Read(I2Cx, (ICM20948_ADDR << 1), WHO_AM_I_REG, 1, &check,
			1, HAL_MAX_DELAY) != HAL_OK)
		;

	SelectBank(0, I2Cx);

	if (check == 234) {
		if (ICM20948_WriteRegister(PWR_MGMT_1, 0x89, I2Cx) != HAL_OK) {
			while (1)
				;
		}
		HAL_Delay(10);
		if (ICM20948_WriteRegister(PWR_MGMT_1, 0x00, I2Cx) != HAL_OK) {
			while (1)
				;
		}
		HAL_Delay(10);

		if (ICM20948_WriteRegister(PWR_MGMT_2, 0x00, I2Cx) != HAL_OK) {
			while (1)
				;
		}
		HAL_Delay(10);
		if (ICM20948_WriteRegister(USER_CTRL, 0x00, I2Cx) != HAL_OK) {
			while (1)
				;
		}
		HAL_Delay(10);
		SelectBank(2, I2Cx);
		HAL_Delay(10);

		if (ICM20948_WriteRegister(ACCEL_CONFIG, 0x1D, I2Cx) != HAL_OK) {
			while (1);
		}
		HAL_Delay(10);
		SelectBank(2, I2Cx);
		HAL_Delay(10);
		if (ICM20948_WriteRegister(GYRO_CONFIG, 0x01, I2Cx) != HAL_OK) {
			while (1);
		}
		HAL_Delay(10);
		return 0;
	}

	return 1;

}

void ICM20948_Read_Accel(I2C_HandleTypeDef *I2Cx, ICM20948_t *DataStruct) {
    SelectBank(0, I2Cx);
	uint8_t Rec_Data[6];

	HAL_I2C_Mem_Read(I2Cx, (ICM20948_ADDR << 1), ACCEL_XOUT_H, 1, Rec_Data, 6,
			i2c_timeout);
//    HAL_Delay(100);

	DataStruct->ACCEL_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	DataStruct->ACCEL_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
	DataStruct->ACCEL_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

	DataStruct->Ax = DataStruct->ACCEL_X_RAW / 2048.0;
	DataStruct->Ay = DataStruct->ACCEL_Y_RAW / 2048.0;
	DataStruct->Az = DataStruct->ACCEL_Z_RAW / 2048.0;
}

void ICM20948_Read_Gyro(I2C_HandleTypeDef *I2Cx, ICM20948_t *DataStruct) {
	uint8_t Rec_Data[6];
	SelectBank(0,I2Cx);

	HAL_I2C_Mem_Read(I2Cx, (ICM20948_ADDR << 1), GYRO_XOUT_H, 1, Rec_Data, 6, i2c_timeout);

	DataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	DataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
	DataStruct->Gyro_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

	DataStruct->Gx = DataStruct->Gyro_X_RAW / 131;
	DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131;
	DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131;

    DataStruct->NormGx = DataStruct->Gx * .060975f;
    DataStruct->NormGy = DataStruct->Gy * .060975f;
    DataStruct->NormGz = DataStruct->Gz * .060975f;
}


