/*
 * BMP180.c
 *
 *  Created on: Jan 21, 2024
 *      Author: Admin
 */


#include "BMP180.h"


BMP180_Calibration_t BMP180_Calibration;

uint8_t BMP180_Init() {
	MX_GPIO_Init();
	MX_I2C1_Init();
	if (HAL_I2C_IsDeviceReady(&hi2c1, BMP180_ADDR, 1, HAL_MAX_DELAY))
		return 1;
	BMP180_reset();
	return 0;
}

void BMP180_reset(){
	uint8_t reset = (uint8_t)0xb6;
	HAL_I2C_Mem_Write(&hi2c1, BMP180_ADDR, BMP180_SOFT_RESET_REG, I2C_MEMADD_SIZE_8BIT, &reset, 1, HAL_MAX_DELAY);
	HAL_Delay(10);
}

void BMP180_ReadCalibration(){
	uint8_t buffer[BMP180_PROM_DATA_LEN] = {0};
	for (int i = 0; i < BMP180_PROM_DATA_LEN; i++){
		HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDR, BMP180_PROM_START_ADDR + i, I2C_MEMADD_SIZE_8BIT, &buffer[i], 1, HAL_MAX_DELAY);
	}


	BMP180_Calibration.AC1 = (buffer[0]  << 8) | buffer[1];
	BMP180_Calibration.AC2 = (buffer[2]  << 8) | buffer[3];
	BMP180_Calibration.AC3 = (buffer[4]  << 8) | buffer[5];
	BMP180_Calibration.AC4 = (buffer[6]  << 8) | buffer[7];
	BMP180_Calibration.AC5 = (buffer[8]  << 8) | buffer[9];
	BMP180_Calibration.AC6 = (buffer[10] << 8) | buffer[11];
	BMP180_Calibration.B1  = (buffer[12] << 8) | buffer[13];
	BMP180_Calibration.B2  = (buffer[14] << 8) | buffer[15];
	BMP180_Calibration.MB  = (buffer[16] << 8) | buffer[17];
	BMP180_Calibration.MC  = (buffer[18] << 8) | buffer[19];
	BMP180_Calibration.MD  = (buffer[20] << 8) | buffer[21];
}

uint32_t BMP180_Read_UT(){
	uint32_t UT = 0;
	uint8_t T_measure = BMP180_T_MEASURE;
	uint8_t MSB = 0;
	uint8_t LSB = 0;

	HAL_I2C_Mem_Write(&hi2c1, BMP180_ADDR, BMP180_CTRL_MEAS_REG, I2C_MEMADD_SIZE_8BIT, &T_measure, 1, HAL_MAX_DELAY);
	HAL_Delay(6);
	HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDR, BMP180_ADC_OUT_MSB_REG, I2C_MEMADD_SIZE_8BIT, &MSB, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDR, BMP180_ADC_OUT_LSB_REG, I2C_MEMADD_SIZE_8BIT, &LSB, 1, HAL_MAX_DELAY);
	UT = (MSB << 8) + LSB;

	return UT;
}

uint32_t BMP180_Read_UP(uint8_t oss){
	uint32_t UP;
	uint8_t cmd,delay;
	uint8_t MSB = 0;
	uint8_t LSB = 0;
	uint8_t XLSB = 0;

	switch(oss) {
	case 0:
		cmd = BMP180_P0_MEASURE;
		delay   = 6;
		break;
	case 1:
		cmd = BMP180_P1_MEASURE;
		delay   = 9;
		break;
	case 2:
		cmd = BMP180_P2_MEASURE;
		delay   = 15;
		break;
	case 3:
		cmd = BMP180_P3_MEASURE;
		delay   = 27;
		break;
	}

	HAL_I2C_Mem_Write(&hi2c1, BMP180_ADDR, BMP180_CTRL_MEAS_REG, I2C_MEMADD_SIZE_8BIT, &cmd, 1, HAL_MAX_DELAY);
	HAL_Delay(delay);
	HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDR, BMP180_ADC_OUT_MSB_REG, I2C_MEMADD_SIZE_8BIT, &MSB, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDR, BMP180_ADC_OUT_LSB_REG, I2C_MEMADD_SIZE_8BIT, &LSB, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDR, BMP180_ADC_OUT_XLSB_REG, I2C_MEMADD_SIZE_8BIT, &XLSB, 1, HAL_MAX_DELAY);
	UP = ((MSB << 16) + (LSB << 8) + XLSB) >> (8 - oss);

	return UP;
}

uint32_t BMP180_Calculate_RT(){
	uint32_t UT = BMP180_Read_UT();
	BMP180_Calibration.B5  = (((int32_t)UT - (int32_t)BMP180_Calibration.AC6) * (int32_t)BMP180_Calibration.AC5) >> 15;
	BMP180_Calibration.B5 += ((int32_t)BMP180_Calibration.MC << 11) / (BMP180_Calibration.B5 + BMP180_Calibration.MD);

	return (BMP180_Calibration.B5 + 8) >> 4;
}

uint32_t BMP180_Calculate_RP(uint8_t oss){
	uint32_t UP = BMP180_Read_UP(oss);
	int32_t B3,B6,X3,p;
	uint32_t B4,B7;

	B6 = BMP180_Calibration.B5 - 4000;
	X3 = ((BMP180_Calibration.B2 * ((B6 * B6) >> 12)) >> 11) + ((BMP180_Calibration.AC2 * B6) >> 11);
	B3 = (((((int32_t)BMP180_Calibration.AC1) * 4 + X3) << oss) + 2) >> 2;
	X3 = (((BMP180_Calibration.AC3 * B6) >> 13) + ((BMP180_Calibration.B1 * ((B6 * B6) >> 12)) >> 16) + 2) >> 2;
	B4 = (BMP180_Calibration.AC4 * (uint32_t)(X3 + 32768)) >> 15;
	B7 = ((uint32_t)UP - B3) * (50000 >> oss);
	if (B7 < 0x80000000) p = (B7 << 1) / B4; else p = (B7 / B4) << 1;
	p += ((((p >> 8) * (p >> 8) * BMP180_PARAM_MG) >> 16) + ((BMP180_PARAM_MH * p) >> 16) + BMP180_PARAM_MI) >> 4;

	return p;
}

uint32_t BMP180_Calculate_Altitude(uint8_t oss){
	uint32_t hPa = BMP180_Calculate_RP(oss) * 0.01;
	return (((745 * (11390 - (hPa / 10))) / 256 + 46597) * (11390 - (hPa / 10))) / 65536 - 966;
}
