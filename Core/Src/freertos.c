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

#include "icm20948.h"
#include "math.h"
#include "stdio.h"
#include "string.h"
#include "usbd_cdc_if.h"

#include "imu_kalman_filter_task.hpp"
#include "timers.h"

#include "can.h"
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

uint8_t UART_RX_buffer[20];

uint8_t can_test_data[2] = {21, 87};
ICM20948_DATA data;
volatile float euler_angles[4] = {0, 0, 0, 0};

TimerHandle_t my_timer;
IMU_HANDLE handle = {
    .data = &data,
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
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static uint8_t message[14] = "Hello world!\r\n";


/* Callback when data is fully received (DMA mode) */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)  // Ensure this is the right UART instance
    {
        // Echo the received byte back via UART
        HAL_UART_Transmit(&huart2, UART_RX_buffer, 1, 100);

        // Indicate data reception by toggling the onboard LED
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

        // Restart the DMA reception to continue receiving data
        HAL_UART_Receive_DMA(&huart2, UART_RX_buffer, 1);
    }
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {

    }
}


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
    // Read the data from the IMU sensor and save in into the data variable
    ICM20948_ReadAccelData_DMA(&hi2c1, &data);
    ICM20948_ReadAccelData_DMA_complete(&data);

    // Notify the IMU-Kalman Filter thread that new data have arrived
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(IMUHandle, &xHigherPriorityTaskWoken);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == USER_PUSHBUTTON_Pin) {
        CAN_send_data(0x100, can_test_data);
    }
}

osThreadId_t *getDefaultTaskHandle() {
    return &defaultTaskHandle;
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
extern void imu_task(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName) {
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
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
    HAL_CAN_Start(&hcan1);

    ICM20948_Init(&hi2c1);
    my_timer = xTimerCreate("imu_update", IMU_MEASURE_RATE_MS / portTICK_PERIOD_MS, pdTRUE,
                            (void *)0, imu_update);
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
  IMUHandle = osThreadNew(imu_task, (void*) &handle, &IMU_attributes);

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

     uint8_t message[14] = "Hello world!\r\n";
     uint32_t notification_value = 0;

     HAL_UART_Receive_DMA(&huart2, UART_RX_buffer, 1);
     for (;;) {
         printf("Roll: %.02f\tPitch: %.02f\tRollKF: %.02f\tPitchKF: %.02f\r\n", euler_angles[0],
                euler_angles[1], euler_angles[2], euler_angles[3]);
         //HAL_UART_Transmit(&huart2, message, sizeof(message), 100);
         //HAL_UART_Receive(&huart2, UART_RX_buffer, 1, 100);
         osDelay(100);
         xTaskNotifyWait(0x00, 0xffffffff, &notification_value, 10);
         if (notification_value > 0) {
             //HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
             notification_value = 0;
         }
         // printf("Notification value: %d\r\n", notification_value);
     }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

