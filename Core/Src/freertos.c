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

ICM20948_DATA data;
uint8_t flag = 0;
volatile int counter = 0;
int counter2 = 0;
uint8_t counter3 = 0;
uint8_t ready = 0;

volatile float accelX = 0;
volatile float accelY = 0;
volatile float accelZ = 0;

volatile float gyroX = 0;
volatile float gyroY = 0;
volatile float gyroZ = 0;

volatile float euler_angles[4] = {0, 0, 0, 0};
volatile float global_pitch = 0;
volatile float global_pitchKF = 0;


const osMutexAttr_t data_mutex_attr = {
  .name="data_mutex",
  .attr_bits = osMutexPrioInherit,
  .cb_mem = &data,
  .cb_size = sizeof(data),

  };



osMutexId_t data_mutex;



TimerHandle_t my_timer;



IMU_HANDLE handle ={
  .data = &data,
  .data_mutex = &data_mutex,
  .euler_angles = euler_angles,
};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for IMU */
osThreadId_t IMUHandle;
const osThreadAttr_t IMU_attributes = {
  .name = "IMU",
  .stack_size = 256 * 4,
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
  for(;;) {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    osDelay(100);
  }
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  data_mutex = osMutexNew(&data_mutex_attr);
  MX_USB_DEVICE_Init();

  ICM20948_Init(&hi2c1);

  my_timer = xTimerCreate("imu_update", 10 / portTICK_PERIOD_MS, pdTRUE, (void*)0, imu_update);
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
  float pitch = 21.1;
  float roll = 17.5;
  float pitch_kf = 0;
  float roll_kf = 0;

  for(;;)
  {

    roll = euler_angles[0];
    pitch = euler_angles[1];

    roll_kf = euler_angles[2];
    pitch_kf = euler_angles[3];




    printf("Roll: %.04f\tPitch: %.04f\tRollKF: %.04f\tPitchKF: %.04f\r\n", roll, pitch, roll_kf, pitch_kf);
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    osDelay(100);
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

