/**
  ******************************************************************************
  * @file           : key.c
  * @brief          : 按键扫描实现
  ******************************************************************************
  */

#include "key.h"

#define KEY_DEBOUNCE_MS 5U

/**
  * @brief  按键扫描函数
  * @param  None
  * @retval 按键值
  */
KEY_TypeDef KEY_Scan(void)
{
    static uint8_t key_state = 0;  // 0:未按下, 1:已按下
    
    // KEY0, KEY1, KEY2是低电平有效(内部上拉)
    // KEY_UP是高电平有效(内部下拉)
    
    if(key_state == 0)  // 未按下状态
    {
        if(HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET)
        {
            HAL_Delay(KEY_DEBOUNCE_MS);  // 消抖
            if(HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET)
            {
                key_state = 1;
                return KEY0_PRESS;
            }
        }
        else if(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
        {
            HAL_Delay(KEY_DEBOUNCE_MS);
            if(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
            {
                key_state = 1;
                return KEY1_PRESS;
            }
        }
        else if(HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)
        {
            HAL_Delay(KEY_DEBOUNCE_MS);
            if(HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)
            {
                key_state = 1;
                return KEY2_PRESS;
            }
        }
        else if(HAL_GPIO_ReadPin(KEY_UP_GPIO_Port, KEY_UP_Pin) == GPIO_PIN_SET)
        {
            HAL_Delay(KEY_DEBOUNCE_MS);
            if(HAL_GPIO_ReadPin(KEY_UP_GPIO_Port, KEY_UP_Pin) == GPIO_PIN_SET)
            {
                key_state = 1;
                return KEY_UP_PRESS;
            }
        }
    }
    else  // 已按下状态,等待释放
    {
        if(HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_SET &&
           HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_SET &&
           HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_SET &&
           HAL_GPIO_ReadPin(KEY_UP_GPIO_Port, KEY_UP_Pin) == GPIO_PIN_RESET)
        {
            key_state = 0;
        }
    }
    
    return KEY_NONE;
}
