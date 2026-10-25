/**
  ******************************************************************************
  * @file           : sr04.c
  * @brief          : SR04超声波测距模块实现
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sr04.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void delay_us(TIM_HandleTypeDef* htim, uint16_t us);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  微秒级延迟
  * @param  htim: 定时器句柄
  * @param  us: 延迟时间(微秒)
  * @retval None
  */
static void delay_us(TIM_HandleTypeDef* htim, uint16_t us)
{
    __HAL_TIM_SET_COUNTER(htim, 0);
    while(__HAL_TIM_GET_COUNTER(htim) < us);
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化SR04模块
  * @param  hsr04: SR04句柄
  * @param  trig_port: TRIG引脚端口
  * @param  trig_pin: TRIG引脚编号
  * @param  htim: 定时器句柄
  * @param  tim_channel: 定时器通道
  * @retval None
  */
void SR04_Init(SR04_HandleTypeDef* hsr04, 
               GPIO_TypeDef* trig_port, 
               uint16_t trig_pin,
               TIM_HandleTypeDef* htim,
               uint32_t tim_channel)
{
    hsr04->trig_port = trig_port;
    hsr04->trig_pin = trig_pin;
    hsr04->htim = htim;
    hsr04->tim_channel = tim_channel;
    
    hsr04->ic_val1 = 0;
    hsr04->ic_val2 = 0;
    hsr04->is_first_captured = 0;
    hsr04->distance_valid = 0;
    hsr04->distance = 0;
    
    // 启动定时器
    HAL_TIM_Base_Start(htim);
    
    // 启动输入捕获中断
    HAL_TIM_IC_Start_IT(htim, tim_channel);
    
    // 初始化TRIG引脚为低电平
    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);
}

/**
  * @brief  触发SR04发送超声波
  * @param  hsr04: SR04句柄
  * @retval None
  */
void SR04_Trigger(SR04_HandleTypeDef* hsr04)
{
    // 发送10us高电平触发信号
    HAL_GPIO_WritePin(hsr04->trig_port, hsr04->trig_pin, GPIO_PIN_SET);
    delay_us(hsr04->htim, 10);
    HAL_GPIO_WritePin(hsr04->trig_port, hsr04->trig_pin, GPIO_PIN_RESET);
}

/**
  * @brief  SR04输入捕获中断处理函数
  * @param  hsr04: SR04句柄
  * @param  htim: 定时器句柄
  * @retval None
  */
void SR04_IRQHandler(SR04_HandleTypeDef* hsr04, TIM_HandleTypeDef *htim)
{
    if(htim->Instance != hsr04->htim->Instance)
        return;
    
    if(hsr04->is_first_captured == 0)  // 捕获到上升沿
    {
        hsr04->ic_val1 = HAL_TIM_ReadCapturedValue(htim, hsr04->tim_channel);
        hsr04->is_first_captured = 1;
        
        // 切换到下降沿捕获
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, hsr04->tim_channel, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else  // 捕获到下降沿
    {
        hsr04->ic_val2 = HAL_TIM_ReadCapturedValue(htim, hsr04->tim_channel);
        
        uint32_t difference;
        
        // 计算时间差
        if(hsr04->ic_val2 > hsr04->ic_val1)
        {
            difference = hsr04->ic_val2 - hsr04->ic_val1;
        }
        else  // 计数器溢出
        {
            difference = (0xFFFF - hsr04->ic_val1) + hsr04->ic_val2;
        }
        
        // 计算距离 (mm) = (高电平时间 * 声速) / 2
        // 声速 = 340m/s = 0.034cm/us = 0.34mm/us
        hsr04->distance = (difference * 0.34f) / 2.0f;
        
        hsr04->distance_valid = 1;
        hsr04->is_first_captured = 0;
        
        // 切换回上升沿捕获
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, hsr04->tim_channel, TIM_INPUTCHANNELPOLARITY_RISING);
    }
}

/**
  * @brief  获取测量距离
  * @param  hsr04: SR04句柄
  * @retval 距离值(mm)
  */
float SR04_GetDistance(SR04_HandleTypeDef* hsr04)
{
    return hsr04->distance;
}

/**
  * @brief  检查数据是否有效
  * @param  hsr04: SR04句柄
  * @retval 1-有效, 0-无效
  */
uint8_t SR04_IsDataValid(SR04_HandleTypeDef* hsr04)
{
    return hsr04->distance_valid;
}

/**
  * @brief  清除数据有效标志
  * @param  hsr04: SR04句柄
  * @retval None
  */
void SR04_ClearData(SR04_HandleTypeDef* hsr04)
{
    hsr04->distance_valid = 0;
}