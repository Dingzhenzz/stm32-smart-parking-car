/**
  ******************************************************************************
  * @file           : key.h
  * @brief          : 按键扫描头文件
  ******************************************************************************
  */
#ifndef __KEY_H
#define __KEY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/
typedef enum {
    KEY_NONE = 0,
    KEY0_PRESS,    // PE4 - 右
    KEY1_PRESS,    // PE3 - 下
    KEY2_PRESS,    // PE2 - 左
    KEY_UP_PRESS   // PA0 - 上
} KEY_TypeDef;

/* Exported constants --------------------------------------------------------*/
// 按键引脚定义(根据你的实际连接修改标签名)
#define KEY0_GPIO_Port   GPIOE
#define KEY0_Pin         GPIO_PIN_4
#define KEY1_GPIO_Port   GPIOE
#define KEY1_Pin         GPIO_PIN_3
#define KEY2_GPIO_Port   GPIOE
#define KEY2_Pin         GPIO_PIN_2
#define KEY_UP_GPIO_Port GPIOA
#define KEY_UP_Pin       GPIO_PIN_0

/* Exported functions prototypes ---------------------------------------------*/
KEY_TypeDef KEY_Scan(void);

#ifdef __cplusplus
}
#endif

#endif /* __KEY_H */