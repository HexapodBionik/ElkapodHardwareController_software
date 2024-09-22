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

#include "usbd_cdc_if.h"
#include "string.h"
#include "stdio.h"
#include "icm20948.h"
#include "math.h"

#include "imu_kalman_filter_task.hpp"
#include "timers.h"

#include "can.h"


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

CAN_RxHeaderTypeDef RxHeader;
uint8_t flag = 0;
uint8_t RxData[8];

ICM20948_DATA data;
volatile float euler_angles[4] = {0, 0, 0, 0};


TimerHandle_t my_timer;
IMU_HANDLE handle ={
  .data = &data,
  .euler_angles = euler_angles,
};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for IMU */
osThreadId_t IMUHandle;
const osThreadAttr_t IMU_attributes = {
  .name = "IMU",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */


int _write(int file, char *ptr, int len) {
  static uint8_t rc = USBD_OK;

  do {
    rc = CDC_Transmit_FS(ptr, len);
  } while (USBD_BUSY == rc);

  if (USBD_FAIL == rc) {
    /// NOTE: Should never reach here.
    /// TODO: Handle this error.
    return 0;
  }
  return len;
}

void imu_update() {
  ICM20948_ReadAccelData_DMA(&hi2c1, &data);
  ICM20948_ReadAccelData_DMA_complete(&data);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData);
  if(RxHeader.DLC == 2) {
    flag = 1;
  }
  flag = 1;
}


/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void IMU_TASK_DEFAULT(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  volatile HAL_StatusTypeDef status = HAL_CAN_Start(&hcan1);
  volatile HAL_StatusTypeDef status2 = HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);

  ICM20948_Init(&hi2c1);
  my_timer = xTimerCreate("imu_update", IMU_MEASURE_RATE_MS / portTICK_PERIOD_MS, pdTRUE, (void*)0, imu_update);
  xTimerStart(my_timer, portMAX_DELAY);

  /* USER CODE END Init */

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

  /* creation of IMU */
  IMUHandle = osThreadNew(IMU_TASK_DEFAULT, (void*) &handle, &IMU_attributes);

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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */

  for(;;)
  {
    printf("Roll: %.02f\tPitch: %.02f\tRollKF: %.02f\tPitchKF: %.02f\r\n",  euler_angles[0],  euler_angles[1],  euler_angles[2],  euler_angles[3]);
    osDelay(50);
    if(flag == 1) {
      printf("Received a message\r\n");
      flag = 0;
    }
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_IMU_TASK_DEFAULT */
/**
* @brief Function implementing the IMU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_TASK_DEFAULT */
void IMU_TASK_DEFAULT(void *argument)
{
  /* USER CODE BEGIN IMU_TASK_DEFAULT */
  /* Infinite loop */
  imu_task(argument);
  for(;;)
  {
    //HAL_GPIO_TogglePin(TASK_CHECK_1_GPIO_Port, TASK_CHECK_1_Pin);
    osDelay(10);
  }
  /* USER CODE END IMU_TASK_DEFAULT */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

