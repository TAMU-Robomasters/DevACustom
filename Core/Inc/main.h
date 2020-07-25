/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define APB2_TIMER_CLOCKS 168000000
#define APB1_TIMER_CLOCKS 84000000
#define PWM_RESOLUTION 10000
#define PWM_FREQUENCE 50
#define PWM_DEFAULT_DUTY 5000
#define TIM_PSC_APB1 ((APB1_TIMER_CLOCKS/PWM_FREQUENCE)/PWM_RESOLUTION -1)
#define TIM_PSC_APB2 ((APB2_TIMER_CLOCKS/PWM_FREQUENCE)/PWM_RESOLUTION -1)
#define PORT_Y_Pin GPIO_PIN_7
#define PORT_Y_GPIO_Port GPIOI
#define PORT_X_Pin GPIO_PIN_6
#define PORT_X_GPIO_Port GPIOI
#define PORT_W_Pin GPIO_PIN_5
#define PORT_W_GPIO_Port GPIOI
#define PORT_Z_Pin GPIO_PIN_2
#define PORT_Z_GPIO_Port GPIOI
#define PORT_A_Pin GPIO_PIN_0
#define PORT_A_GPIO_Port GPIOI
#define POWER1_CTRL_Pin GPIO_PIN_2
#define POWER1_CTRL_GPIO_Port GPIOH
#define POWER2_CTRL_Pin GPIO_PIN_3
#define POWER2_CTRL_GPIO_Port GPIOH
#define POWER3_CTRL_Pin GPIO_PIN_4
#define POWER3_CTRL_GPIO_Port GPIOH
#define LED_A_Pin GPIO_PIN_8
#define LED_A_GPIO_Port GPIOG
#define POWER4_CTRL_Pin GPIO_PIN_5
#define POWER4_CTRL_GPIO_Port GPIOH
#define LED_B_Pin GPIO_PIN_7
#define LED_B_GPIO_Port GPIOG
#define LED_C_Pin GPIO_PIN_6
#define LED_C_GPIO_Port GPIOG
#define PORT_B_Pin GPIO_PIN_12
#define PORT_B_GPIO_Port GPIOH
#define LED_D_Pin GPIO_PIN_5
#define LED_D_GPIO_Port GPIOG
#define LED_E_Pin GPIO_PIN_4
#define LED_E_GPIO_Port GPIOG
#define LED_F_Pin GPIO_PIN_3
#define LED_F_GPIO_Port GPIOG
#define PORT_C_Pin GPIO_PIN_11
#define PORT_C_GPIO_Port GPIOH
#define PORT_D_Pin GPIO_PIN_10
#define PORT_D_GPIO_Port GPIOH
#define PORT_E_Pin GPIO_PIN_15
#define PORT_E_GPIO_Port GPIOD
#define LED_G_Pin GPIO_PIN_2
#define LED_G_GPIO_Port GPIOG
#define LED_H_Pin GPIO_PIN_1
#define LED_H_GPIO_Port GPIOG
#define PORT_F_Pin GPIO_PIN_14
#define PORT_F_GPIO_Port GPIOD
#define PORT_G_Pin GPIO_PIN_13
#define PORT_G_GPIO_Port GPIOD
#define PORT_T_Pin GPIO_PIN_1
#define PORT_T_GPIO_Port GPIOA
#define PORT_S_Pin GPIO_PIN_0
#define PORT_S_GPIO_Port GPIOA
#define PORT_H_Pin GPIO_PIN_12
#define PORT_H_GPIO_Port GPIOD
#define PORT_U_Pin GPIO_PIN_2
#define PORT_U_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_11
#define LED_RED_GPIO_Port GPIOE
#define PORT_V_Pin GPIO_PIN_3
#define PORT_V_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_14
#define LED_GREEN_GPIO_Port GPIOF
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
