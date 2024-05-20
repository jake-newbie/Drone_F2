/*
 * PWM.c
 *
 *  Created on: Apr 5, 2024
 *      Author: Admin
 */


#include "PWM.h"

extern uint32_t measured_time;
extern TIM_OC_InitTypeDef PWM_config;
extern uint8_t chanel_seclect_counter;
extern uint32_t Chanel_1;
extern uint32_t Chanel_2;
extern uint32_t Chanel_3;
extern uint32_t Chanel_4;
extern uint32_t Chanel_5;
extern uint32_t Chanel_6;
extern uint16_t esc_1;
extern uint16_t esc_2;
extern uint16_t esc_3;
extern uint16_t esc_4;
extern uint16_t throttle;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
		chanel_seclect_counter++;
		if(chanel_seclect_counter > 10)chanel_seclect_counter = 0;
		measured_time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		if (measured_time > 3000)chanel_seclect_counter = 0;
		if(chanel_seclect_counter == 1)Chanel_1 = measured_time + 424;
		if(chanel_seclect_counter == 2)Chanel_2 = measured_time + 424;
		if(chanel_seclect_counter == 3)Chanel_3 = measured_time + 424;
		if(chanel_seclect_counter == 4)Chanel_4 = measured_time + 424;
		if(chanel_seclect_counter == 5)Chanel_5 = measured_time;
		if(chanel_seclect_counter == 6)Chanel_6 = measured_time;
	}
}

void PWM_Init(){
	MX_TIM4_Init();
	MX_TIM2_Init();
	PWM_config.OCMode = TIM_OCMODE_PWM1;
	HAL_TIM_PWM_ConfigChannel(&htim4, &PWM_config, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&htim4, &PWM_config, TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(&htim4, &PWM_config, TIM_CHANNEL_3);
	HAL_TIM_PWM_ConfigChannel(&htim4, &PWM_config, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
}


void Throttle_esc(){
	if (throttle >= 4000) throttle = 3999;
	PWM_config.Pulse = esc_1;
	HAL_TIM_PWM_ConfigChannel(&htim4, &PWM_config, TIM_CHANNEL_1);
	PWM_config.Pulse = esc_2;
	HAL_TIM_PWM_ConfigChannel(&htim4, &PWM_config, TIM_CHANNEL_2);
	PWM_config.Pulse = esc_3;
	HAL_TIM_PWM_ConfigChannel(&htim4, &PWM_config, TIM_CHANNEL_3);
	PWM_config.Pulse = esc_4;
	HAL_TIM_PWM_ConfigChannel(&htim4, &PWM_config, TIM_CHANNEL_4);
}
