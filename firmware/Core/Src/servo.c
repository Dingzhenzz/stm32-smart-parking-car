/**
  ******************************************************************************
  * @file           : servo.c
  * @brief          : SG90舵机控制实现
  ******************************************************************************
  */
#include "servo.h"

/**
  * @brief  初始化舵机
  * @param  hservo: 舵机句柄
  * @param  htim: 定时器句柄
  * @param  channel: PWM通道
  * @retval None
  */
void Servo_Init(Servo_HandleTypeDef* hservo, TIM_HandleTypeDef* htim, uint32_t channel)
{
    hservo->htim = htim;
    hservo->channel = channel;
    hservo->current_angle = 90;  // 初始化到90度
    
    // 启动PWM输出
    HAL_TIM_PWM_Start(htim, channel);
    
    // 设置初始角度
    Servo_SetAngle(hservo, 90);
}

/**
  * @brief  设置舵机角度
  * @param  hservo: 舵机句柄
  * @param  angle: 目标角度(0-180)
  * @retval None
  */
void Servo_SetAngle(Servo_HandleTypeDef* hservo, uint16_t angle)
{
    // 限制角度范围
    if(angle > SERVO_MAX_ANGLE)
        angle = SERVO_MAX_ANGLE;
    
    // 计算CCR值: 0.5ms(500) ~ 2.5ms(2500)
    // 0° -> 500, 90° -> 1500, 180° -> 2500
    uint16_t ccr = 500 + (angle * 2000) / 180;
    
    // 设置PWM占空比
    __HAL_TIM_SET_COMPARE(hservo->htim, hservo->channel, ccr);
    
    hservo->current_angle = angle;
}

/**
  * @brief  获取当前舵机角度
  * @param  hservo: 舵机句柄
  * @retval 当前角度
  */
uint16_t Servo_GetAngle(Servo_HandleTypeDef* hservo)
{
    return hservo->current_angle;
}