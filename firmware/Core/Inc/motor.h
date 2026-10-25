/**
  ******************************************************************************
  * @file           : motor.h
  * @brief          : TT130 + TB6612 motor driver interface
  ******************************************************************************
  */
#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    MOTOR_LF = 0,  // Left front
    MOTOR_LR,      // Left rear
    MOTOR_RF,      // Right front
    MOTOR_RR,      // Right rear
    MOTOR_COUNT
} MotorId_t;

typedef enum
{
    MOTOR_STOP_COAST = 0,
    MOTOR_STOP_BRAKE = 1
} MotorStopMode_t;

/* Exported functions --------------------------------------------------------*/
void Motor_Init(void);
void Motor_SetEnable(uint8_t enable);
HAL_StatusTypeDef Motor_SetSpeedPercent(MotorId_t id, int16_t speed_percent);
HAL_StatusTypeDef Motor_Stop(MotorId_t id, MotorStopMode_t mode);
void Motor_StopAll(MotorStopMode_t mode);
int16_t Motor_GetSpeedPercent(MotorId_t id);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
