/**
  ******************************************************************************
  * @file           : sr04.h
  * @brief          : SR04超声波测距模块头文件
  ******************************************************************************
  */

#ifndef __SR04_H
#define __SR04_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"

/* Exported types ------------------------------------------------------------*/
typedef struct {
    GPIO_TypeDef* trig_port;      // TRIG引脚端口
    uint16_t trig_pin;             // TRIG引脚编号
    TIM_HandleTypeDef* htim;       // 定时器句柄
    uint32_t tim_channel;          // 定时器通道
    
    uint32_t ic_val1;              // 第一次捕获值(上升沿)
    uint32_t ic_val2;              // 第二次捕获值(下降沿)
    uint8_t is_first_captured;     // 是否已捕获上升沿
    uint8_t distance_valid;        // 距离数据是否有效
    float distance;                // 距离值(mm)
} SR04_HandleTypeDef;

/* Exported constants --------------------------------------------------------*/
#define SR04_MIN_DISTANCE  20.0f   // 最小测量距离(mm)
#define SR04_MAX_DISTANCE  4000.0f // 最大测量距离(mm)

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void SR04_Init(SR04_HandleTypeDef* hsr04, 
               GPIO_TypeDef* trig_port, 
               uint16_t trig_pin,
               TIM_HandleTypeDef* htim,
               uint32_t tim_channel);

void SR04_Trigger(SR04_HandleTypeDef* hsr04);
void SR04_IRQHandler(SR04_HandleTypeDef* hsr04, TIM_HandleTypeDef *htim);
float SR04_GetDistance(SR04_HandleTypeDef* hsr04);
uint8_t SR04_IsDataValid(SR04_HandleTypeDef* hsr04);
void SR04_ClearData(SR04_HandleTypeDef* hsr04);

#ifdef __cplusplus
}
#endif

#endif /* __SR04_H */