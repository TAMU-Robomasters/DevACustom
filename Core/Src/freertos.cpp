/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "init.hpp"

#include "subsystems/chassis.hpp"
#include "subsystems/feeder.hpp"
#include "subsystems/flywheel.hpp"
#include "subsystems/gimbal.hpp"

#include "information/can_protocol.hpp"
#include "information/uart_protocol.hpp"
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

/* USER CODE END Variables */
osThreadId indicatorTaskHandle;
osThreadId chassisTaskHandle;
osThreadId gimbalTaskHandle;
osThreadId flywheelTaskHandle;
osThreadId feederTaskHandle;
osThreadId rcTaskHandle;
osThreadId sensorTaskHandle;
osThreadId canTaskHandle;
osThreadId uartTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void indicatorTaskFunc(void const* argument);
void chassisTaskFunc(void const* argument);
void gimbalTaskFunc(void const* argument);
void flywheelTaskFunc(void const* argument);
void feederTaskFunc(void const* argument);
void rcTaskFunc(void const* argument);
void sensorTaskFunc(void const* argument);
void canTaskFunc(void const* argument);
void uartTaskFunc(void const* argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer, StackType_t** ppxIdleTaskStackBuffer, uint32_t* pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer, StackType_t** ppxIdleTaskStackBuffer, uint32_t* pulIdleTaskStackSize) {
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
    /* USER CODE BEGIN Init */

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
    /* definition and creation of indicatorTask */
    osThreadDef(indicatorTask, indicatorTaskFunc, osPriorityNormal, 0, 128);
    indicatorTaskHandle = osThreadCreate(osThread(indicatorTask), NULL);

    /* definition and creation of chassisTask */
    osThreadDef(chassisTask, chassisTaskFunc, osPriorityNormal, 0, 128);
    chassisTaskHandle = osThreadCreate(osThread(chassisTask), NULL);

    /* definition and creation of gimbalTask */
    osThreadDef(gimbalTask, gimbalTaskFunc, osPriorityNormal, 0, 128);
    gimbalTaskHandle = osThreadCreate(osThread(gimbalTask), NULL);

    /* definition and creation of flywheelTask */
    osThreadDef(flywheelTask, flywheelTaskFunc, osPriorityNormal, 0, 128);
    flywheelTaskHandle = osThreadCreate(osThread(flywheelTask), NULL);

    /* definition and creation of feederTask */
    osThreadDef(feederTask, feederTaskFunc, osPriorityNormal, 0, 128);
    feederTaskHandle = osThreadCreate(osThread(feederTask), NULL);

    /* definition and creation of rcTask */
    osThreadDef(rcTask, rcTaskFunc, osPriorityNormal, 0, 128);
    rcTaskHandle = osThreadCreate(osThread(rcTask), NULL);

    /* definition and creation of sensorTask */
    osThreadDef(sensorTask, sensorTaskFunc, osPriorityNormal, 0, 128);
    sensorTaskHandle = osThreadCreate(osThread(sensorTask), NULL);

    /* definition and creation of canTask */
    osThreadDef(canTask, canTaskFunc, osPriorityNormal, 0, 128);
    canTaskHandle = osThreadCreate(osThread(canTask), NULL);

    /* definition and creation of uartTask */
    osThreadDef(uartTask, uartTaskFunc, osPriorityNormal, 0, 128);
    uartTaskHandle = osThreadCreate(osThread(uartTask), NULL);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_indicatorTaskFunc */
/**
  * @brief  Function implementing the indicatorTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_indicatorTaskFunc */
void indicatorTaskFunc(void const* argument) {
    /* USER CODE BEGIN indicatorTaskFunc */
    /* Infinite loop */
    userInit::initialize();
    for (;;) {
        //friendly reminder that "GPIOG" refers to the GPIO port G, and the "LED_A_Pin" directs it to the specific pin under that port
        HAL_GPIO_TogglePin(GPIOG, LED_A_Pin);
        osDelay(125);
        HAL_GPIO_TogglePin(GPIOG, LED_A_Pin);
        HAL_GPIO_TogglePin(GPIOG, LED_B_Pin);
        osDelay(125);
        HAL_GPIO_TogglePin(GPIOG, LED_B_Pin);
        HAL_GPIO_TogglePin(GPIOG, LED_C_Pin);
        osDelay(125);
        HAL_GPIO_TogglePin(GPIOG, LED_C_Pin);
        HAL_GPIO_TogglePin(GPIOG, LED_D_Pin);
        osDelay(125);
        HAL_GPIO_TogglePin(GPIOG, LED_D_Pin);
        HAL_GPIO_TogglePin(GPIOG, LED_E_Pin);
        osDelay(125);
        HAL_GPIO_TogglePin(GPIOG, LED_E_Pin);
        HAL_GPIO_TogglePin(GPIOG, LED_F_Pin);
        osDelay(125);
        HAL_GPIO_TogglePin(GPIOG, LED_F_Pin);
        HAL_GPIO_TogglePin(GPIOG, LED_G_Pin);
        osDelay(125);
        HAL_GPIO_TogglePin(GPIOG, LED_G_Pin);
        HAL_GPIO_TogglePin(GPIOG, LED_H_Pin);
        osDelay(125);
        HAL_GPIO_TogglePin(GPIOG, LED_H_Pin);
        osDelay(2);
    }
    /* USER CODE END indicatorTaskFunc */
}

/* USER CODE BEGIN Header_chassisTaskFunc */
/**
* @brief Function implementing the chassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassisTaskFunc */
void chassisTaskFunc(void const* argument) {
    /* USER CODE BEGIN chassisTaskFunc */
    /* Infinite loop */
    chassis::task();
    /* USER CODE END chassisTaskFunc */
}

/* USER CODE BEGIN Header_gimbalTaskFunc */
/**
* @brief Function implementing the gimbalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gimbalTaskFunc */
void gimbalTaskFunc(void const* argument) {
    /* USER CODE BEGIN gimbalTaskFunc */
    /* Infinite loop */
    gimbal::task();
    /* USER CODE END gimbalTaskFunc */
}

/* USER CODE BEGIN Header_flywheelTaskFunc */
/**
* @brief Function implementing the flywheelTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_flywheelTaskFunc */
void flywheelTaskFunc(void const* argument) {
    /* USER CODE BEGIN flywheelTaskFunc */
    /* Infinite loop */
    flywheel::task();
    /* USER CODE END flywheelTaskFunc */
}

/* USER CODE BEGIN Header_feederTaskFunc */
/**
* @brief Function implementing the feederTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_feederTaskFunc */
void feederTaskFunc(void const* argument) {
    /* USER CODE BEGIN feederTaskFunc */
    /* Infinite loop */
    feeder::task();
    /* USER CODE END feederTaskFunc */
}

/* USER CODE BEGIN Header_rcTaskFunc */
/**
* @brief Function implementing the rcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_rcTaskFunc */
void rcTaskFunc(void const* argument) {
    /* USER CODE BEGIN rcTaskFunc */
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
    /* USER CODE END rcTaskFunc */
}

/* USER CODE BEGIN Header_sensorTaskFunc */
/**
* @brief Function implementing the sensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sensorTaskFunc */
void sensorTaskFunc(void const* argument) {
    /* USER CODE BEGIN sensorTaskFunc */
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
    /* USER CODE END sensorTaskFunc */
}

/* USER CODE BEGIN Header_canTaskFunc */
/**
* @brief Function implementing the canTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_canTaskFunc */
void canTaskFunc(void const* argument) {
    /* USER CODE BEGIN canTaskFunc */
    /* Infinite loop */
    userCAN::task();
    /* USER CODE END canTaskFunc */
}

/* USER CODE BEGIN Header_uartTaskFunc */
/**
* @brief Function implementing the uartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_uartTaskFunc */
void uartTaskFunc(void const* argument) {
    /* USER CODE BEGIN uartTaskFunc */
    /* Infinite loop */
    userUART::task();
    /* USER CODE END uartTaskFunc */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
