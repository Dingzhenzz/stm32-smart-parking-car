/**
  ******************************************************************************
  * @file           : servo.h
  * @brief          : SG90舵机控制头文件
  ******************************************************************************
  */
#ifndef __SERVO_H
#define __SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"

/* Exported types ------------------------------------------------------------*/
typedef struct {
    TIM_HandleTypeDef* htim;
    uint32_t channel;
    uint16_t current_angle;  // 当前角度(0-180)
} Servo_HandleTypeDef;

/* Exported constants --------------------------------------------------------*/
#define SERVO_MIN_ANGLE  0
#define SERVO_MAX_ANGLE  180

/* Exported functions prototypes ---------------------------------------------*/
void Servo_Init(Servo_HandleTypeDef* hservo, TIM_HandleTypeDef* htim, uint32_t channel);
void Servo_SetAngle(Servo_HandleTypeDef* hservo, uint16_t angle);
uint16_t Servo_GetAngle(Servo_HandleTypeDef* hservo);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_H */