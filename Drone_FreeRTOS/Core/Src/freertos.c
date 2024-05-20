/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BMP180.h"
#include "MPU6050.h"
#include "GPS.h"
#include "usart.h"
#include "tim.h"
#include "PWM.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define calibrate 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
MPU6050_t data;
double roll;
double pitch;
uint8_t status_mpu;

uint32_t temp;
uint32_t preasure;
uint32_t altitude;
uint8_t status;
uint8_t flag_gps;
mavlink_gps_raw_int_t data_GPS = {0};
mavlink_raw_imu_t raw_imu;
TIM_OC_InitTypeDef PWM_config;
uint16_t throttle;
uint16_t esc_1;
uint16_t esc_2;
uint16_t esc_3;
uint16_t esc_4;
uint32_t Chanel_1 = 1000;
uint32_t Chanel_2 = 1000;
uint32_t Chanel_3 = 1000;
uint32_t Chanel_4 = 1000;
uint32_t Chanel_5 = 1000;
uint32_t Chanel_6 = 1000;

uint32_t measured_time;
uint8_t chanel_seclect_counter;

float pid_p_gain_roll = 1.3;               //Gain setting for the pitch and roll P-controller (default = 1.3).
float pid_i_gain_roll = 0.04;              //Gain setting for the pitch and roll I-controller (default = 0.04).
float pid_d_gain_roll = 18.0;              //Gain setting for the pitch and roll D-controller (default = 18.0).
uint16_t pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-).

float pid_p_gain_pitch = 1.3;             //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = 0.04;            //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = 18.0;            //Gain setting for the pitch D-controller.
uint16_t pid_max_pitch = 400;                    //Maximum output of the PID-controller (+/-).

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller (default = 4.0).
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller (default = 0.02).
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller (default = 0.0).
uint16_t pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-).
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for _BMP180 */
osThreadId_t _BMP180Handle;
const osThreadAttr_t _BMP180_attributes = {
  .name = "_BMP180",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for _IMU */
osThreadId_t _IMUHandle;
const osThreadAttr_t _IMU_attributes = {
  .name = "_IMU",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for _GPS */
osThreadId_t _GPSHandle;
const osThreadAttr_t _GPS_attributes = {
  .name = "_GPS",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for _main */
osThreadId_t _mainHandle;
const osThreadAttr_t _main_attributes = {
  .name = "_main",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for UART2_Transmit_Lock */
osMutexId_t UART2_Transmit_LockHandle;
const osMutexAttr_t UART2_Transmit_Lock_attributes = {
  .name = "UART2_Transmit_Lock"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
long map(long x, long in_min, long in_max, long out_min, long out_max){
	return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void BMP180_Func(void *argument);
void IMU_Func(void *argument);
void GPS_Func(void *argument);
void main_Func(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of UART2_Transmit_Lock */
  UART2_Transmit_LockHandle = osMutexNew(&UART2_Transmit_Lock_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of _BMP180 */
  _BMP180Handle = osThreadNew(BMP180_Func, NULL, &_BMP180_attributes);

  /* creation of _IMU */
  _IMUHandle = osThreadNew(IMU_Func, NULL, &_IMU_attributes);

  /* creation of _GPS */
  _GPSHandle = osThreadNew(GPS_Func, NULL, &_GPS_attributes);

  /* creation of _main */
  _mainHandle = osThreadNew(main_Func, NULL, &_main_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	  if(osMutexAcquire(UART2_Transmit_LockHandle, 100) == osOK){
	  		  Mavlink_TX_Heartbeat();
	  	  }
	  	 osMutexRelease(UART2_Transmit_LockHandle);
	  	 osDelay(10000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_BMP180_Func */
/**
* @brief Function implementing the _BMP180 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BMP180_Func */
void BMP180_Func(void *argument)
{
  /* USER CODE BEGIN BMP180_Func */
	status = BMP180_Init();
	uint8_t oss = 2;
	BMP180_ReadCalibration();
  /* Infinite loop */
  for(;;)
  {
	  temp = BMP180_Calculate_RT();
	  preasure = BMP180_Calculate_RP(oss);
	  altitude = BMP180_Calculate_Altitude(oss);
	  osDelay(500);
  }

  /* USER CODE END BMP180_Func */
}

/* USER CODE BEGIN Header_IMU_Func */
/**
* @brief Function implementing the _IMU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_Func */
void IMU_Func(void *argument)
{
  /* USER CODE BEGIN IMU_Func */
	MPU6050_Init(&hi2c2);
  /* Infinite loop */
  for(;;)
  {
	  MPU6050_Read_All(&hi2c2, &data);
	  if(osMutexAcquire(UART2_Transmit_LockHandle, 1000) == osOK){
		  Transmit_raw_data_IMU(&data, raw_imu);
	  }
	osMutexRelease(UART2_Transmit_LockHandle);

    osDelay(600);
  }
  /* USER CODE END IMU_Func */
}

/* USER CODE BEGIN Header_GPS_Func */
/**
* @brief Function implementing the _GPS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GPS_Func */
void GPS_Func(void *argument)
{
  /* USER CODE BEGIN GPS_Func */
	  MX_USART2_UART_Init();
	  MX_USART1_UART_Init();
	  //MX_USB_DEVICE_Init();
	  GPS_Init();

  /* Infinite loop */
  for(;;)
  {
	  if(flag_gps == 1){
		  if(osMutexAcquire(UART2_Transmit_LockHandle, 1000) == osOK){
			  Transmit_mavlink_data_GPS(data_GPS);
		  }
		osMutexRelease(UART2_Transmit_LockHandle);
		flag_gps = 0;
	  }
    osDelay(500);
  }
  /* USER CODE END GPS_Func */
}

/* USER CODE BEGIN Header_main_Func */
/**
* @brief Function implementing the _main thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_main_Func */
void main_Func(void *argument)
{
  /* USER CODE BEGIN main_Func */
	PWM_Init();
	uint16_t pid_output_pitch = 0;
	uint16_t pid_output_roll = 0;
	uint16_t pid_output_yaw = 0;
#if calibrate
	htim4.Instance->CCR1 = 400;
	htim4.Instance->CCR2 = 400;
	osDelay(2000);
	htim4.Instance->CCR1 = 200;
	htim4.Instance->CCR2 = 200;
	osDelay(1000);
	htim4.Instance->CCR1 = 0;
	htim4.Instance->CCR2 = 0;
#endif
  /* Infinite loop */
  for(;;)
  {
	pid_output_pitch = calculate_pid(pid_p_gain_pitch, pid_i_gain_pitch, pid_d_gain_pitch, data.KalmanAngleY, Chanel_2);
	pid_output_roll  = calculate_pid(pid_p_gain_roll, pid_i_gain_roll, pid_d_gain_roll, data.KalmanAngleX, Chanel_1);
	//pid_output_yaw   = calculate_pid(pid_p_gain, pid_i_gain, pid_d_gain, gyro_input)
	throttle = Chanel_3;
	if(throttle < 1090) throttle = 1000;
	if(throttle > 1977) throttle = 2000;

	esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 1 (front-right - CCW).
	esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 2 (rear-right - CW).
	esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 3 (rear-left - CCW).
	esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 4 (front-left - CW).


	esc_1 = map(esc_1, 1000, 2000, 200, 400);
	esc_2 = map(esc_2, 1000, 2000, 200, 400);
	esc_3 = map(esc_3, 1000, 2000, 200, 400);
	esc_4 = map(esc_4, 1000, 2000, 200, 400);
	htim4.Instance->CCR1 = esc_1;
	htim4.Instance->CCR2 = esc_2;
	htim4.Instance->CCR3 = esc_3;
	htim4.Instance->CCR4 = esc_4;
    osDelay(150);
  }
  /* USER CODE END main_Func */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

