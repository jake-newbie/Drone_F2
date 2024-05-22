/*
 * pid.h
 *
 *  Created on: 25 Apr 2024
 *      Author: Admin
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"


float calculate_pid(float pid_p_gain,float pid_i_gain,float pid_d_gain, float gyro_input,uint32_t channel_value,float* last_d_error);


#endif /* INC_PID_H_ */
