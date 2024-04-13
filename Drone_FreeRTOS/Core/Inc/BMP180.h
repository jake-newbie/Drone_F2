/*
 * BMP180.h
 *
 *  Created on: Jan 21, 2024
 *      Author: Admin
 */

#ifndef INC_BMP180_H_
#define INC_BMP180_H_

#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include "stm32f4xx_hal_i2c.h"
#include "math.h"

#define BMP180_ADDR                  0xEE
// BMP180 registers
#define BMP180_PROM_START_ADDR          0xAA // E2PROM calibration data start register
#define BMP180_PROM_DATA_LEN            22   // E2PROM length
#define BMP180_CHIP_ID_REG              0xD0 // Chip ID
#define BMP180_VERSION_REG              0xD1 // Version
#define BMP180_CTRL_MEAS_REG            0xF4 // Measurements control (OSS[7.6], SCO[5], CTL[4.0]
#define BMP180_ADC_OUT_MSB_REG          0xF6 // ADC out MSB  [7:0]
#define BMP180_ADC_OUT_LSB_REG          0xF7 // ADC out LSB  [7:0]
#define BMP180_ADC_OUT_XLSB_REG         0xF8 // ADC out XLSB [7:3]
#define BMP180_SOFT_RESET_REG           0xE0 // Soft reset control

// BMP180 control values
#define BMP180_T_MEASURE                0x2E // temperature measurement
#define BMP180_P0_MEASURE               0x34 // pressure measurement (OSS=0, 4.5ms)
#define BMP180_P1_MEASURE               0x74 // pressure measurement (OSS=1, 7.5ms)
#define BMP180_P2_MEASURE               0xB4 // pressure measurement (OSS=2, 13.5ms)
#define BMP180_P3_MEASURE               0xF4 // pressure measurement (OSS=3, 25.5ms)
#define BMP180_PARAM_MG                 3038
#define BMP180_PARAM_MH                -7357
#define BMP180_PARAM_MI                 3791

typedef struct {
	int16_t AC1;
	int16_t AC2;
	int16_t AC3;
	uint16_t AC4;
	uint16_t AC5;
	uint16_t AC6;
	int16_t B1;
	int16_t B2;
	int16_t MB;
	int16_t MC;
	int16_t MD;
	int32_t B5;
} BMP180_Calibration_t;


uint8_t BMP180_Init();
void BMP180_reset();
void BMP180_ReadCalibration();
uint32_t BMP180_Read_UT();
uint32_t BMP180_Read_UP(uint8_t oss);
uint32_t BMP180_Calculate_RT();
uint32_t BMP180_Calculate_RP(uint8_t oss);
uint32_t BMP180_Calculate_Altitude(uint8_t oss);

#endif /* INC_BMP180_H_ */
