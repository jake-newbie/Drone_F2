

#include "pid.h"



float calculate_pid(float pid_p_gain,float pid_i_gain,float pid_d_gain, float gyro_input,uint32_t channel_value){
	float pid_setpoint = 0;
	float pid_i_mem = 0;
	int pid_max = 0;
	float pid_last_d_error = 0;
	float pid_output;

	if (channel_value > 1508){
		pid_setpoint = channel_value - 1508;
	}
  	else if (channel_value < 1492){
		pid_setpoint = channel_value - 1492;
	}

	pid_setpoint /= 3.0;
	
	float pid_error_temp = gyro_input - pid_setpoint;
	pid_i_mem += pid_i_gain*pid_error_temp;
	if(pid_i_mem>pid_max){
		pid_i_mem = pid_max;
	}
	else if(pid_i_mem < -1*pid_max){
		pid_i_mem = -1*pid_max;
	}
	pid_output = pid_p_gain*pid_error_temp + pid_i_mem + pid_d_gain*(pid_error_temp-pid_last_d_error);
	if(pid_output > pid_max){
		pid_output = pid_max;
	}
	else if(pid_output < -1 * pid_max){
		pid_output = -1 * pid_max;
	}
	pid_last_d_error = pid_error_temp;

	return pid_output;
}
