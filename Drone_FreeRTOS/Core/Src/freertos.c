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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for _BMP180 */
osThreadId_t _BMP180Handle;
const osThreadAttr_t _BMP180_attributes = {
  .name = "_BMP180",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for _MPU6050 */
osThreadId_t _MPU6050Handle;
const osThreadAttr_t _MPU6050_attributes = {
  .name = "_MPU6050",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for _GPS */
osThreadId_t _GPSHandle;
const osThreadAttr_t _GPS_attributes = {
  .name = "_GPS",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UART2_Transmit_Lock */
osMutexId_t UART2_Transmit_LockHandle;
const osMutexAttr_t UART2_Transmit_Lock_attributes = {
  .name = "UART2_Transmit_Lock"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void BMP180_Func(void *argument);
void MPU6050_Func(void *argument);
void GPS_Func(void *argument);

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

  /* creation of _MPU6050 */
  _MPU6050Handle = osThreadNew(MPU6050_Func, NULL, &_MPU6050_attributes);

  /* creation of _GPS */
  _GPSHandle = osThreadNew(GPS_Func, NULL, &_GPS_attributes);

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
	  osDelay(50);
  }

  /* USER CODE END BMP180_Func */
}

/* USER CODE BEGIN Header_MPU6050_Func */
/**
* @brief Function implementing the _MPU6050 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MPU6050_Func */
void MPU6050_Func(void *argument)
{
  /* USER CODE BEGIN MPU6050_Func */
	MPU6050_Init(&hi2c2);
  /* Infinite loop */
  for(;;)
  {
	  mavlink_raw_imu_t raw_imu;
	  status_mpu = Data_Ready();
	  if(Data_Ready()){
	  	  MPU6050_Read_All(&hi2c2, &data);
	  	  roll  = data.KalmanAngleX;
	  	  pitch = data.KalmanAngleY;
	  	  if(osMutexAcquire(UART2_Transmit_LockHandle, 1000) == osOK){
	  		  Transmit_raw_data_IMU(&data, raw_imu);
	  	  }
	  	 osMutexRelease(UART2_Transmit_LockHandle);
	  }

    osDelay(50);
  }
  /* USER CODE END MPU6050_Func */
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
    osDelay(200);
  }
  /* USER CODE END GPS_Func */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

