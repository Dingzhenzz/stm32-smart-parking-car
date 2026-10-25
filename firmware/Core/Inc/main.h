/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#define KEY2_Pin GPIO_PIN_2
#define KEY2_GPIO_Port GPIOE
#define KEY1_Pin GPIO_PIN_3
#define KEY1_GPIO_Port GPIOE
#define KEY0_Pin GPIO_PIN_4
#define KEY0_GPIO_Port GPIOE
#define KEY_UP_Pin GPIO_PIN_0
#define KEY_UP_GPIO_Port GPIOA
#define BEEP_Pin GPIO_PIN_8
#define BEEP_GPIO_Port GPIOF
#define MOTOR_RF_IN1_Pin GPIO_PIN_8
#define MOTOR_RF_IN1_GPIO_Port GPIOD
#define MOTOR_RF_IN2_Pin GPIO_PIN_9
#define MOTOR_RF_IN2_GPIO_Port GPIOD
#define MOTOR_RR_IN1_Pin GPIO_PIN_10
#define MOTOR_RR_IN1_GPIO_Port GPIOD
#define MOTOR_RR_IN2_Pin GPIO_PIN_11
#define MOTOR_RR_IN2_GPIO_Port GPIOD
#define MOTOR_RIGHT_STBY_Pin GPIO_PIN_12
#define MOTOR_RIGHT_STBY_GPIO_Port GPIOD
#define MOTOR_LF_IN1_Pin GPIO_PIN_0
#define MOTOR_LF_IN1_GPIO_Port GPIOD
#define MOTOR_LF_IN2_Pin GPIO_PIN_1
#define MOTOR_LF_IN2_GPIO_Port GPIOD
#define MOTOR_LR_IN1_Pin GPIO_PIN_2
#define MOTOR_LR_IN1_GPIO_Port GPIOD
#define MOTOR_LR_IN2_Pin GPIO_PIN_3
#define MOTOR_LR_IN2_GPIO_Port GPIOD
#define MOTOR_LEFT_STBY_Pin GPIO_PIN_4
#define MOTOR_LEFT_STBY_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
