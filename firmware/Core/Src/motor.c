/**
  ******************************************************************************
  * @file           : motor.c
  * @brief          : TT130 + TB6612 motor driver implementation
  ******************************************************************************
  */

#include "motor.h"

typedef struct
{
    uint32_t pwm_channel;
    GPIO_TypeDef *in1_port;
    uint16_t in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t in2_pin;
    GPIO_TypeDef *stby_port;
    uint16_t stby_pin;
    uint8_t inverted;
    int16_t speed_percent;
} MotorConfig_t;

static MotorConfig_t motors[MOTOR_COUNT] =
{
    // LF: TB6612 #1 PWMA/AIN1/AIN2/STBY
    {TIM_CHANNEL_3, MOTOR_LF_IN1_GPIO_Port, MOTOR_LF_IN1_Pin, MOTOR_LF_IN2_GPIO_Port, MOTOR_LF_IN2_Pin, MOTOR_LEFT_STBY_GPIO_Port, MOTOR_LEFT_STBY_Pin, 0U, 0},
    // LR: TB6612 #1 PWMB/BIN1/BIN2/STBY
    {TIM_CHANNEL_4, MOTOR_LR_IN1_GPIO_Port, MOTOR_LR_IN1_Pin, MOTOR_LR_IN2_GPIO_Port, MOTOR_LR_IN2_Pin, MOTOR_LEFT_STBY_GPIO_Port, MOTOR_LEFT_STBY_Pin, 1U, 0},
    // RF: TB6612 #2 PWMA/AIN1/AIN2/STBY
    {TIM_CHANNEL_1, MOTOR_RF_IN1_GPIO_Port, MOTOR_RF_IN1_Pin, MOTOR_RF_IN2_GPIO_Port, MOTOR_RF_IN2_Pin, MOTOR_RIGHT_STBY_GPIO_Port, MOTOR_RIGHT_STBY_Pin, 1U, 0},
    // RR: TB6612 #2 PWMB/BIN1/BIN2/STBY
    {TIM_CHANNEL_2, MOTOR_RR_IN1_GPIO_Port, MOTOR_RR_IN1_Pin, MOTOR_RR_IN2_GPIO_Port, MOTOR_RR_IN2_Pin, MOTOR_RIGHT_STBY_GPIO_Port, MOTOR_RIGHT_STBY_Pin, 0U, 0}
};

static uint8_t motor_enabled = 0U;

static int16_t clamp_speed_percent(int16_t speed_percent)
{
    if (speed_percent > 100)
    {
        return 100;
    }
    if (speed_percent < -100)
    {
        return -100;
    }
    return speed_percent;
}

static HAL_StatusTypeDef is_valid_motor(MotorId_t id)
{
    if ((id < MOTOR_LF) || (id >= MOTOR_COUNT))
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

static uint32_t speed_to_ccr(int16_t speed_percent_abs)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim8);
    uint32_t ccr = ((uint32_t)speed_percent_abs * (arr + 1U)) / 100U;

    if (ccr > arr)
    {
        ccr = arr;
    }
    return ccr;
}

static void set_direction_forward(MotorConfig_t *motor)
{
    HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
}

static void set_direction_reverse(MotorConfig_t *motor)
{
    HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_SET);
}

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

    Motor_SetEnable(1U);
    Motor_StopAll(MOTOR_STOP_BRAKE);
}

void Motor_SetEnable(uint8_t enable)
{
    GPIO_PinState stby_state = (enable == 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET;

    HAL_GPIO_WritePin(MOTOR_LEFT_STBY_GPIO_Port, MOTOR_LEFT_STBY_Pin, stby_state);
    HAL_GPIO_WritePin(MOTOR_RIGHT_STBY_GPIO_Port, MOTOR_RIGHT_STBY_Pin, stby_state);
    motor_enabled = (enable == 0U) ? 0U : 1U;
}

HAL_StatusTypeDef Motor_SetSpeedPercent(MotorId_t id, int16_t speed_percent)
{
    MotorConfig_t *motor;
    uint8_t forward;
    int16_t speed_abs;

    if (is_valid_motor(id) != HAL_OK)
    {
        return HAL_ERROR;
    }

    speed_percent = clamp_speed_percent(speed_percent);
    motor = &motors[id];
    motor->speed_percent = speed_percent;

    if (speed_percent == 0)
    {
        return Motor_Stop(id, MOTOR_STOP_BRAKE);
    }

    if (motor_enabled == 0U)
    {
        return HAL_OK;
    }

    forward = (speed_percent > 0) ? 1U : 0U;
    if (motor->inverted != 0U)
    {
        forward = (forward == 0U) ? 1U : 0U;
    }

    if (forward != 0U)
    {
        set_direction_forward(motor);
    }
    else
    {
        set_direction_reverse(motor);
    }

    speed_abs = (speed_percent < 0) ? (int16_t)(-speed_percent) : speed_percent;
    __HAL_TIM_SET_COMPARE(&htim8, motor->pwm_channel, speed_to_ccr(speed_abs));
    return HAL_OK;
}

HAL_StatusTypeDef Motor_Stop(MotorId_t id, MotorStopMode_t mode)
{
    MotorConfig_t *motor;

    if (is_valid_motor(id) != HAL_OK)
    {
        return HAL_ERROR;
    }

    motor = &motors[id];
    motor->speed_percent = 0;
    __HAL_TIM_SET_COMPARE(&htim8, motor->pwm_channel, 0U);

    if (mode == MOTOR_STOP_BRAKE)
    {
        HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
    }

    return HAL_OK;
}

void Motor_StopAll(MotorStopMode_t mode)
{
    Motor_Stop(MOTOR_LF, mode);
    Motor_Stop(MOTOR_LR, mode);
    Motor_Stop(MOTOR_RF, mode);
    Motor_Stop(MOTOR_RR, mode);
}

int16_t Motor_GetSpeedPercent(MotorId_t id)
{
    if (is_valid_motor(id) != HAL_OK)
    {
        return 0;
    }
    return motors[id].speed_percent;
}
