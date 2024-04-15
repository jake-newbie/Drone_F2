/*
 * PWM.h
 *
 *  Created on: Apr 5, 2024
 *      Author: Admin
 */

#ifndef INC_PWM_H_
#define INC_PWM_H_

#include "tim.h"
#include "stm32f4xx_hal_pwr.h"
#include "stm32f4xx_hal_tim.h"

void PWM_Init();
void Throttle_esc();

#endif /* INC_PWM_H_ */
