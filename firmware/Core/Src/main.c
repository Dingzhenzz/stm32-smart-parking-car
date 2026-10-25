/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "esp8266.h"
#include "sr04.h"
#include "servo.h"
#include "key.h"
#include "motor.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    CAR_MODE_IDLE = 0,
    CAR_MODE_WAIT_DIRECTION,
    CAR_MODE_SEARCHING_SLOT,
    CAR_MODE_WAIT_KEY1_CONFIRM,
    CAR_MODE_PARKING
} CarMode_t;

typedef enum
{
    SEARCH_DIR_LEFT = 0,
    SEARCH_DIR_RIGHT
} SearchDirection_t;

typedef enum
{
    SEARCH_MOTION_FORWARD = 0,
    SEARCH_MOTION_BACKWARD
} SearchMotion_t;

typedef enum
{
    SLOT_TYPE_NONE = 0,
    SLOT_TYPE_PARALLEL,
    SLOT_TYPE_VERTICAL
} SlotType_t;

typedef enum
{
    PARK_STAGE_INIT = 0,
    PARK_STAGE_FIRST_STRAIGHT,
    PARK_STAGE_FIRST_TURN,
    PARK_STAGE_SECOND_STRAIGHT,
    PARK_STAGE_SECOND_TURN,
    PARK_STAGE_VERTICAL_ALIGN,
    PARK_STAGE_VERTICAL_TURN,
    PARK_STAGE_VERTICAL_REVERSE_IN,
    PARK_STAGE_DONE,
    PARK_STAGE_ABORT
} ParkingStage_t;

/* Private define ------------------------------------------------------------*/
#define SERVO_CENTER_ANGLE          108U
#define SERVO_LEFT_SCAN_ANGLE       26U
#define SERVO_RIGHT_SCAN_CCR        2611U

/* 2026-03-20:
   1) Parallel slot:
      - R2F search + reverse park is the current baseline set.
      - F2R search + forward park uses its own first-straight and first-turn tuning.
   2) Vertical slot:
      - R2F search + reverse park parameters below are the current validated baseline.
      - F2R search + forward park now has an independent parameter set for the next calibration round. */
#define MOTOR_SEARCH_SPEED_PERCENT  10
#define MOTOR_REVERSE_SPEED_PERCENT 10
#define MOTOR_SPIN_SPEED_PERCENT    15

#define CAR_LENGTH_CM               31.0f
#define CAR_WIDTH_CM                17.0f
#define SLOT_EXTRA_MARGIN_CM        6.0f
#define SLOT_MIN_LENGTH_CM          (CAR_LENGTH_CM + SLOT_EXTRA_MARGIN_CM)

#define SLOT_SEARCH_SPEED_CM_S      18.46f
#define SLOT_SENSOR_OFFSET_CM       9.0f
#define SLOT_WIDTH_THRESHOLD_CM     22.0f
#define SLOT_TIME_THRESHOLD_S       (SLOT_MIN_LENGTH_CM / SLOT_SEARCH_SPEED_CM_S)
#define VERTICAL_SLOT_EXTRA_MARGIN_CM 5.0f
#define VERTICAL_SLOT_MIN_OPENING_CM (CAR_WIDTH_CM + VERTICAL_SLOT_EXTRA_MARGIN_CM)
#define VERTICAL_SLOT_MIN_DEPTH_CM  40.0f

#define V_MOVE_CM_S                 18.46f
#define PARK_TARGET1_RATIO_R2F      (4.0f / 5.0f)
#define PARK_TARGET1_RATIO_F2R      (1.0f / 2.0f)
#define T_FIRST_TURN_90_R2F_MS      1301U
#define T_FIRST_TURN_90_F2R_MS      1464U
#define T_SECOND_TURN_90_R2F_MS     1544U
#define T_SECOND_TURN_90_F2R_MS     1563U
#define VERTICAL_ALIGN_RATIO_R2F    (9.0f / 10.0f)   /* validated on 2026-03-20 for vertical R2F search + reverse park */
#define VERTICAL_ALIGN_RATIO_F2R    (3.0f / 4.0f)    /* updated on 2026-03-20 for vertical F2R search + forward park */
#define VERTICAL_ALIGN_MIN_CM       8.0f
#define VERTICAL_REVERSE_MARGIN_CM  6.0f
#define VERTICAL_FORWARD_MARGIN_CM  18.0f            /* updated on 2026-03-20 for vertical F2R search + forward park */
#define VERTICAL_FORWARD_D2_MAX_CM  40.0f            /* relaxed on 2026-03-20 to keep only extreme-value protection */
#define T_VERTICAL_TURN_90_R2F_MS   1303U            /* corrected from ~80 deg to ~90 deg on 2026-03-21 */
#define T_VERTICAL_TURN_90_F2R_MS   1765U            /* corrected from ~80 deg to ~90 deg on 2026-03-21 */
#define D_SAFE_CM                   5.0f
#define REVERSE_OBSTACLE_AVOID_ENABLE 0
#define REVERSE_BEEP_ENABLE         0
#define REVERSE_BEEP_NEAR_CM        12.0f
#define REVERSE_BEEP_SLOW_ON_MS     120U
#define REVERSE_BEEP_SLOW_OFF_MS    280U
#define REVERSE_BEEP_FAST_ON_MS     80U
#define REVERSE_BEEP_FAST_OFF_MS    120U
#define ESP8266_BRIDGE_TEST         0

/* Private variables ---------------------------------------------------------*/
SR04_HandleTypeDef hsr04;
Servo_HandleTypeDef hservo;

static CarMode_t g_car_mode = CAR_MODE_IDLE;
static SearchDirection_t g_search_dir = SEARCH_DIR_LEFT;
static SearchMotion_t g_search_motion = SEARCH_MOTION_FORWARD;
static uint8_t g_in_open_region = 0U;
static uint32_t g_open_start_tick_ms = 0U;
static float g_open_region_max_side_cm = 0.0f;
static float g_last_open_time_s = 0.0f;
static float g_last_slot_length_cm = 0.0f;
static float g_last_slot_depth_cm = 0.0f;
static float g_latest_raw_cm = 0.0f;
static float g_latest_side_cm = 0.0f;
static float g_slot_stop_raw_cm = 0.0f;
static float g_slot_stop_side_cm = 0.0f;
static SlotType_t g_last_slot_type = SLOT_TYPE_NONE;
static ParkingStage_t g_park_stage = PARK_STAGE_INIT;
static float g_park_target1_cm = 0.0f;
static float g_park_target2_cm = 0.0f;
static float g_park_moved1_cm = 0.0f;
static float g_park_moved2_cm = 0.0f;
static uint8_t g_parking_paused_by_obstacle = 0U;
static uint32_t g_park_last_tick_ms = 0U;
static uint32_t g_park_turn_start_tick_ms = 0U;
static uint8_t g_buzzer_enabled = 0U;
static uint8_t g_buzzer_on = 0U;
static uint32_t g_buzzer_last_toggle_tick_ms = 0U;
#if ESP8266_BRIDGE_TEST
static const uint32_t g_bridge_baud_table[] = {115200U, 9600U, 57600U, 38400U, 74880U};
static uint8_t g_bridge_baud_index = 0U;
#endif

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Servo_SetAngleRaw(Servo_HandleTypeDef* hservo, uint16_t ccr_value);
static void UartSendLocal(const char *msg);
static void UartSend(const char *msg);
static const char *GetSearchSideLabel(void);
static const char *GetSearchMotionLabel(void);
static const char *GetSlotTypeLabel(SlotType_t slot_type);
static uint8_t IsParkingBackward(void);
static void Motion_SearchForward(void);
static void Motion_SearchBackward(void);
static void Motion_ParkForward(void);
static void Motion_ParkBackward(void);
static void Motion_SpinLeft20(void);
static void Motion_SpinRight20(void);
static void Motion_StopBrake(void);
static void Buzzer_Set(uint8_t on);
static void Buzzer_Stop(void);
static void ReverseBuzzer_Update(uint8_t enable, uint8_t obstacle_stop, uint8_t rear_ok, float rear_distance_cm);
static void SetSearchDirection(SearchDirection_t direction);
static uint8_t MeasureDistanceCm(float *distance_cm);
static uint8_t MeasureRearDistanceCm(float *distance_cm);
#if ESP8266_BRIDGE_TEST
static void BridgeUartTraffic(void);
static void Bridge_SetUart3Baud(uint32_t baud_rate);
static void Bridge_ReportBaud(void);
#endif
static void StartSlotSearch(SearchDirection_t direction);
static void UpdateSlotSearch(void);
static void Parking_Start(void);
static void Parking_Update(void);
static void Parking_Abort(const char *reason);
static void HandleKeyEvent(KEY_TypeDef key);

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        SR04_IRQHandler(&hsr04, htim);
    }
}

static void Servo_SetAngleRaw(Servo_HandleTypeDef* hservo, uint16_t ccr_value)
{
    __HAL_TIM_SET_COMPARE(hservo->htim, hservo->channel, ccr_value);
    hservo->current_angle = 180;
}

static void UartSendLocal(const char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 1000);
}

static void UartSend(const char *msg)
{
    UartSendLocal(msg);
    (void)ESP8266_SendLog(msg);
}

#if ESP8266_BRIDGE_TEST
static void BridgeUartTraffic(void)
{
    uint8_t byte;

    if(HAL_UART_Receive(&huart1, &byte, 1U, 0U) == HAL_OK)
    {
        HAL_UART_Transmit(&huart3, &byte, 1U, 1000U);
    }

    if(HAL_UART_Receive(&huart3, &byte, 1U, 0U) == HAL_OK)
    {
        HAL_UART_Transmit(&huart1, &byte, 1U, 1000U);
    }
}

static void Bridge_SetUart3Baud(uint32_t baud_rate)
{
    huart3.Init.BaudRate = baud_rate;
    HAL_UART_Init(&huart3);
}

static void Bridge_ReportBaud(void)
{
    char uart_buf[80];

    sprintf(uart_buf, "[Bridge] USART3 baud = %lu\r\n", (unsigned long)g_bridge_baud_table[g_bridge_baud_index]);
    HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 1000U);
}
#endif

static const char *GetSearchSideLabel(void)
{
    return (g_search_dir == SEARCH_DIR_LEFT) ? "左侧" : "右侧";
}

static const char *GetSearchMotionLabel(void)
{
    return (g_search_motion == SEARCH_MOTION_FORWARD) ? "后往前搜位 + 倒车入位"
                                                      : "前往后搜位 + 前进入位";
}

static const char *GetSlotTypeLabel(SlotType_t slot_type)
{
    switch(slot_type)
    {
        case SLOT_TYPE_PARALLEL:
            return "侧方位车位";

        case SLOT_TYPE_VERTICAL:
            return "垂直倒车车位";

        case SLOT_TYPE_NONE:
        default:
            return "未知车位";
    }
}

static uint8_t IsParkingBackward(void)
{
    return (g_search_motion == SEARCH_MOTION_FORWARD) ? 1U : 0U;
}

static void Motion_SearchForward(void)
{
    Motor_SetSpeedPercent(MOTOR_LF, MOTOR_SEARCH_SPEED_PERCENT);
    Motor_SetSpeedPercent(MOTOR_LR, MOTOR_SEARCH_SPEED_PERCENT);
    Motor_SetSpeedPercent(MOTOR_RF, MOTOR_SEARCH_SPEED_PERCENT);
    Motor_SetSpeedPercent(MOTOR_RR, MOTOR_SEARCH_SPEED_PERCENT);
}

static void Motion_SearchBackward(void)
{
    Motor_SetSpeedPercent(MOTOR_LF, -MOTOR_SEARCH_SPEED_PERCENT);
    Motor_SetSpeedPercent(MOTOR_LR, -MOTOR_SEARCH_SPEED_PERCENT);
    Motor_SetSpeedPercent(MOTOR_RF, -MOTOR_SEARCH_SPEED_PERCENT);
    Motor_SetSpeedPercent(MOTOR_RR, -MOTOR_SEARCH_SPEED_PERCENT);
}

static void Motion_ParkForward(void)
{
    Motor_SetSpeedPercent(MOTOR_LF, MOTOR_REVERSE_SPEED_PERCENT);
    Motor_SetSpeedPercent(MOTOR_LR, MOTOR_REVERSE_SPEED_PERCENT);
    Motor_SetSpeedPercent(MOTOR_RF, MOTOR_REVERSE_SPEED_PERCENT);
    Motor_SetSpeedPercent(MOTOR_RR, MOTOR_REVERSE_SPEED_PERCENT);
}

static void Motion_ParkBackward(void)
{
    Motor_SetSpeedPercent(MOTOR_LF, -MOTOR_REVERSE_SPEED_PERCENT);
    Motor_SetSpeedPercent(MOTOR_LR, -MOTOR_REVERSE_SPEED_PERCENT);
    Motor_SetSpeedPercent(MOTOR_RF, -MOTOR_REVERSE_SPEED_PERCENT);
    Motor_SetSpeedPercent(MOTOR_RR, -MOTOR_REVERSE_SPEED_PERCENT);
}

static void Motion_SpinLeft20(void)
{
    Motor_SetSpeedPercent(MOTOR_LF, -MOTOR_SPIN_SPEED_PERCENT);
    Motor_SetSpeedPercent(MOTOR_LR, -MOTOR_SPIN_SPEED_PERCENT);
    Motor_SetSpeedPercent(MOTOR_RF, MOTOR_SPIN_SPEED_PERCENT);
    Motor_SetSpeedPercent(MOTOR_RR, MOTOR_SPIN_SPEED_PERCENT);
}

static void Motion_SpinRight20(void)
{
    Motor_SetSpeedPercent(MOTOR_LF, MOTOR_SPIN_SPEED_PERCENT);
    Motor_SetSpeedPercent(MOTOR_LR, MOTOR_SPIN_SPEED_PERCENT);
    Motor_SetSpeedPercent(MOTOR_RF, -MOTOR_SPIN_SPEED_PERCENT);
    Motor_SetSpeedPercent(MOTOR_RR, -MOTOR_SPIN_SPEED_PERCENT);
}

static void Motion_StopBrake(void)
{
    Motor_StopAll(MOTOR_STOP_BRAKE);
}

static void Buzzer_Set(uint8_t on)
{
#if REVERSE_BEEP_ENABLE
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    g_buzzer_on = (on != 0U) ? 1U : 0U;
#else
    (void)on;
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    g_buzzer_on = 0U;
#endif
}

static void Buzzer_Stop(void)
{
    g_buzzer_enabled = 0U;
    g_buzzer_last_toggle_tick_ms = HAL_GetTick();
    Buzzer_Set(0U);
}

static void ReverseBuzzer_Update(uint8_t enable, uint8_t obstacle_stop, uint8_t rear_ok, float rear_distance_cm)
{
#if !REVERSE_BEEP_ENABLE
    (void)enable;
    (void)obstacle_stop;
    (void)rear_ok;
    (void)rear_distance_cm;
    Buzzer_Stop();
    return;
#else
    uint32_t now_tick_ms;
    uint32_t on_time_ms;
    uint32_t off_time_ms;

    if(enable == 0U)
    {
        Buzzer_Stop();
        return;
    }

    if(obstacle_stop != 0U)
    {
        g_buzzer_enabled = 0U;
        Buzzer_Set(1U);
        return;
    }

    if((rear_ok != 0U) && (rear_distance_cm <= REVERSE_BEEP_NEAR_CM))
    {
        on_time_ms = REVERSE_BEEP_FAST_ON_MS;
        off_time_ms = REVERSE_BEEP_FAST_OFF_MS;
    }
    else
    {
        on_time_ms = REVERSE_BEEP_SLOW_ON_MS;
        off_time_ms = REVERSE_BEEP_SLOW_OFF_MS;
    }

    now_tick_ms = HAL_GetTick();
    if(g_buzzer_enabled == 0U)
    {
        g_buzzer_enabled = 1U;
        g_buzzer_last_toggle_tick_ms = now_tick_ms;
        Buzzer_Set(1U);
        return;
    }

    if(g_buzzer_on != 0U)
    {
        if((now_tick_ms - g_buzzer_last_toggle_tick_ms) >= on_time_ms)
        {
            g_buzzer_last_toggle_tick_ms = now_tick_ms;
            Buzzer_Set(0U);
        }
    }
    else if((now_tick_ms - g_buzzer_last_toggle_tick_ms) >= off_time_ms)
    {
        g_buzzer_last_toggle_tick_ms = now_tick_ms;
        Buzzer_Set(1U);
    }
#endif
}

static void SetSearchDirection(SearchDirection_t direction)
{
    if(direction == SEARCH_DIR_LEFT)
    {
        Servo_SetAngle(&hservo, SERVO_LEFT_SCAN_ANGLE);
    }
    else
    {
        Servo_SetAngleRaw(&hservo, SERVO_RIGHT_SCAN_CCR);
    }
    HAL_Delay(60);
}

static uint8_t MeasureDistanceCm(float *distance_cm)
{
    SR04_Trigger(&hsr04);
    HAL_Delay(60);

    if(SR04_IsDataValid(&hsr04))
    {
        *distance_cm = SR04_GetDistance(&hsr04) / 10.0f;
        SR04_ClearData(&hsr04);
        if(*distance_cm >= 2.0f && *distance_cm <= 400.0f)
        {
            return 1U;
        }
    }
    return 0U;
}

static uint8_t MeasureRearDistanceCm(float *distance_cm)
{
    return MeasureDistanceCm(distance_cm);
}

static void StartSlotSearch(SearchDirection_t direction)
{
    char uart_buf[120];

    g_search_dir = direction;
    g_in_open_region = 0U;
    g_open_start_tick_ms = 0U;
    g_open_region_max_side_cm = 0.0f;
    g_last_open_time_s = 0.0f;
    g_last_slot_length_cm = 0.0f;
    g_last_slot_depth_cm = 0.0f;
    g_latest_raw_cm = 0.0f;
    g_latest_side_cm = 0.0f;
    g_slot_stop_raw_cm = 0.0f;
    g_slot_stop_side_cm = 0.0f;
    g_last_slot_type = SLOT_TYPE_NONE;
    g_park_stage = PARK_STAGE_INIT;
    g_park_target1_cm = 0.0f;
    g_park_target2_cm = 0.0f;
    g_park_moved1_cm = 0.0f;
    g_park_moved2_cm = 0.0f;
    g_parking_paused_by_obstacle = 0U;
    g_park_last_tick_ms = 0U;
    g_park_turn_start_tick_ms = 0U;
    Buzzer_Stop();

    SetSearchDirection(direction);
    if(g_search_motion == SEARCH_MOTION_FORWARD)
    {
        Motion_SearchForward();
    }
    else
    {
        Motion_SearchBackward();
    }
    g_car_mode = CAR_MODE_SEARCHING_SLOT;

    sprintf(uart_buf, "\r\n开始%s，%s。\r\n", GetSearchMotionLabel(), GetSearchSideLabel());
    UartSend(uart_buf);
}

static void UpdateSlotSearch(void)
{
    float distance_cm;
    float side_distance_cm;
    uint32_t now_tick_ms;
    char uart_buf[180];

    if(MeasureDistanceCm(&distance_cm) == 0U)
    {
        return;
    }

    side_distance_cm = distance_cm - SLOT_SENSOR_OFFSET_CM;
    if(side_distance_cm < 0.0f)
    {
        side_distance_cm = 0.0f;
    }
    g_latest_raw_cm = distance_cm;
    g_latest_side_cm = side_distance_cm;

    sprintf(uart_buf,
            "Scan[%s]: raw=%5.1f cm, side=%5.1f cm\r\n",
            (g_search_dir == SEARCH_DIR_LEFT) ? ((g_search_motion == SEARCH_MOTION_FORWARD) ? "L-F" : "L-B")
                                             : ((g_search_motion == SEARCH_MOTION_FORWARD) ? "R-F" : "R-B"),
            distance_cm,
            side_distance_cm);

    now_tick_ms = HAL_GetTick();
    if(side_distance_cm > SLOT_WIDTH_THRESHOLD_CM)
    {
        if(g_in_open_region == 0U)
        {
            g_open_start_tick_ms = now_tick_ms;
            g_in_open_region = 1U;
            g_open_region_max_side_cm = side_distance_cm;
        }
        else if(side_distance_cm > g_open_region_max_side_cm)
        {
            g_open_region_max_side_cm = side_distance_cm;
        }
    }
    else if(g_in_open_region != 0U)
    {
        float open_time_s;
        float slot_length_cm;
        uint8_t parallel_candidate;
        uint8_t vertical_candidate;
        SlotType_t slot_type = SLOT_TYPE_NONE;

        open_time_s = (float)(now_tick_ms - g_open_start_tick_ms) / 1000.0f;
        slot_length_cm = SLOT_SEARCH_SPEED_CM_S * open_time_s;
        g_in_open_region = 0U;

        g_last_open_time_s = open_time_s;
        g_last_slot_length_cm = slot_length_cm;
        g_last_slot_depth_cm = g_open_region_max_side_cm;
        g_last_slot_type = SLOT_TYPE_NONE;

        parallel_candidate = (open_time_s >= SLOT_TIME_THRESHOLD_S) ? 1U : 0U;
        vertical_candidate = ((slot_length_cm >= VERTICAL_SLOT_MIN_OPENING_CM) &&
                              (g_last_slot_depth_cm >= VERTICAL_SLOT_MIN_DEPTH_CM)) ? 1U : 0U;

        if(parallel_candidate != 0U)
        {
            slot_type = SLOT_TYPE_PARALLEL;
        }
        else if(vertical_candidate != 0U)
        {
            slot_type = SLOT_TYPE_VERTICAL;
        }

        if(slot_type != SLOT_TYPE_NONE)
        {
            g_last_slot_type = slot_type;
            g_slot_stop_raw_cm = g_latest_raw_cm;
            g_slot_stop_side_cm = g_latest_side_cm;
            Motion_StopBrake();
            g_car_mode = CAR_MODE_WAIT_KEY1_CONFIRM;
            sprintf(uart_buf,
                    "\r\n*** 发现%s并已停车（%s，%s）***\r\n"
                    "开口时长 = %.2f s，车位长度 = %.1f cm，车位深度 = %.1f cm\r\n"
                    "停车时原始距离 = %.1f cm，侧向距离 = %.1f cm\r\n"
                    "按下 KEY1 开始自动泊车。\r\n",
                    GetSlotTypeLabel(slot_type),
                    GetSearchSideLabel(),
                    GetSearchMotionLabel(),
                    g_last_open_time_s,
                    g_last_slot_length_cm,
                    g_last_slot_depth_cm,
                    g_slot_stop_raw_cm,
                    g_slot_stop_side_cm);
            UartSend(uart_buf);
        }
    }
}

static void Parking_Start(void)
{
    char uart_buf[160];

    if((g_last_slot_length_cm <= 0.0f) || (g_slot_stop_raw_cm <= 0.0f) || (g_last_slot_type == SLOT_TYPE_NONE))
    {
        Motion_StopBrake();
        g_park_stage = PARK_STAGE_ABORT;
        g_car_mode = CAR_MODE_WAIT_DIRECTION;
        return;
    }

    Servo_SetAngle(&hservo, SERVO_CENTER_ANGLE);
    HAL_Delay(60);

    g_park_moved1_cm = 0.0f;
    g_park_moved2_cm = 0.0f;
    g_parking_paused_by_obstacle = 0U;
    g_park_last_tick_ms = HAL_GetTick();
    g_park_turn_start_tick_ms = 0U;
    Buzzer_Stop();
    g_car_mode = CAR_MODE_PARKING;

    Motion_StopBrake();

    if(g_last_slot_type == SLOT_TYPE_VERTICAL)
    {
        g_park_target1_cm = g_last_slot_length_cm * ((IsParkingBackward() != 0U) ? VERTICAL_ALIGN_RATIO_R2F
                                                                                  : VERTICAL_ALIGN_RATIO_F2R);
        if(g_park_target1_cm < VERTICAL_ALIGN_MIN_CM)
        {
            g_park_target1_cm = VERTICAL_ALIGN_MIN_CM;
        }
        if(g_park_target1_cm > g_last_slot_length_cm)
        {
            g_park_target1_cm = g_last_slot_length_cm;
        }

        g_park_target2_cm = g_last_slot_depth_cm - ((IsParkingBackward() != 0U) ? VERTICAL_REVERSE_MARGIN_CM
                                                                                : VERTICAL_FORWARD_MARGIN_CM);
        if((IsParkingBackward() == 0U) && (g_park_target2_cm > VERTICAL_FORWARD_D2_MAX_CM))
        {
            g_park_target2_cm = VERTICAL_FORWARD_D2_MAX_CM;
        }
        if(g_park_target2_cm < (CAR_LENGTH_CM * 0.5f))
        {
            Motion_StopBrake();
            g_park_stage = PARK_STAGE_ABORT;
            g_car_mode = CAR_MODE_WAIT_DIRECTION;
            return;
        }

        g_park_stage = PARK_STAGE_VERTICAL_ALIGN;
        sprintf(uart_buf,
                "\r\n开始垂直%s：对中距离 = %.1f cm，入位距离 = %.1f cm。\r\n",
                IsParkingBackward() ? "倒车入位" : "前进入位",
                g_park_target1_cm,
                g_park_target2_cm);
    }
    else
    {
        g_park_target1_cm = g_last_slot_length_cm * ((IsParkingBackward() != 0U) ? PARK_TARGET1_RATIO_R2F
                                                                                  : PARK_TARGET1_RATIO_F2R);
        g_park_target2_cm = g_slot_stop_raw_cm * 1.1f;
        g_park_stage = PARK_STAGE_FIRST_STRAIGHT;

        sprintf(uart_buf,
                "\r\n开始%s：d1 = %.1f cm，d2 = %.1f cm。\r\n",
                IsParkingBackward() ? "倒车入位" : "前进入位",
                g_park_target1_cm,
                g_park_target2_cm);
    }
    UartSend(uart_buf);
}

static void Parking_Abort(const char *reason)
{
    Motion_StopBrake();
    g_parking_paused_by_obstacle = 0U;
    Buzzer_Stop();
    g_park_stage = PARK_STAGE_ABORT;
    g_car_mode = CAR_MODE_WAIT_DIRECTION;
    (void)reason;
}

static void Parking_Update(void)
{
    float rear_distance_cm = 0.0f;
    uint8_t rear_ok = 0U;
    uint32_t now_tick_ms = HAL_GetTick();
    float dt_s = (float)(now_tick_ms - g_park_last_tick_ms) / 1000.0f;

    g_park_last_tick_ms = now_tick_ms;

    switch(g_park_stage)
    {
        case PARK_STAGE_FIRST_STRAIGHT:
            if(IsParkingBackward() != 0U)
            {
                rear_ok = MeasureRearDistanceCm(&rear_distance_cm);

                if(REVERSE_OBSTACLE_AVOID_ENABLE && rear_ok && (rear_distance_cm < D_SAFE_CM))
                {
                    Motion_StopBrake();
                    g_parking_paused_by_obstacle = 1U;
                }
                else if((REVERSE_OBSTACLE_AVOID_ENABLE == 0) || (rear_ok && (rear_distance_cm > D_SAFE_CM)))
                {
                    Motion_ParkBackward();
                    g_parking_paused_by_obstacle = 0U;
                    g_park_moved1_cm += V_MOVE_CM_S * dt_s;
                }

                ReverseBuzzer_Update(1U, g_parking_paused_by_obstacle, rear_ok, rear_distance_cm);
            }
            else
            {
                Motion_ParkForward();
                g_parking_paused_by_obstacle = 0U;
                g_park_moved1_cm += V_MOVE_CM_S * dt_s;
                Buzzer_Stop();
            }

            if(g_park_moved1_cm >= g_park_target1_cm)
            {
                Motion_StopBrake();
                Buzzer_Stop();
                g_park_turn_start_tick_ms = HAL_GetTick();
                g_park_stage = PARK_STAGE_FIRST_TURN;
            }
            break;

        case PARK_STAGE_FIRST_TURN:
            Buzzer_Stop();
            if(IsParkingBackward() != 0U)
            {
                if(g_search_dir == SEARCH_DIR_LEFT)
                {
                    Motion_SpinRight20();
                }
                else
                {
                    Motion_SpinLeft20();
                }
            }
            else
            {
                if(g_search_dir == SEARCH_DIR_LEFT)
                {
                    Motion_SpinLeft20();
                }
                else
                {
                    Motion_SpinRight20();
                }
            }

            if((HAL_GetTick() - g_park_turn_start_tick_ms) >= ((IsParkingBackward() != 0U) ? T_FIRST_TURN_90_R2F_MS
                                                                                             : T_FIRST_TURN_90_F2R_MS))
            {
                Motion_StopBrake();
                g_park_stage = PARK_STAGE_SECOND_STRAIGHT;
            }
            break;

        case PARK_STAGE_SECOND_STRAIGHT:
            if(IsParkingBackward() != 0U)
            {
                rear_ok = MeasureRearDistanceCm(&rear_distance_cm);

                if(REVERSE_OBSTACLE_AVOID_ENABLE && rear_ok && (rear_distance_cm < D_SAFE_CM))
                {
                    Motion_StopBrake();
                    g_parking_paused_by_obstacle = 1U;
                }
                else if((REVERSE_OBSTACLE_AVOID_ENABLE == 0) || (rear_ok && (rear_distance_cm > D_SAFE_CM)))
                {
                    Motion_ParkBackward();
                    g_parking_paused_by_obstacle = 0U;
                    g_park_moved2_cm += V_MOVE_CM_S * dt_s;
                }

                ReverseBuzzer_Update(1U, g_parking_paused_by_obstacle, rear_ok, rear_distance_cm);
            }
            else
            {
                Motion_ParkForward();
                g_parking_paused_by_obstacle = 0U;
                g_park_moved2_cm += V_MOVE_CM_S * dt_s;
                Buzzer_Stop();
            }

            if(g_park_moved2_cm >= g_park_target2_cm)
            {
                Motion_StopBrake();
                Buzzer_Stop();
                g_park_turn_start_tick_ms = HAL_GetTick();
                g_park_stage = PARK_STAGE_SECOND_TURN;
            }
            break;

        case PARK_STAGE_SECOND_TURN:
            Buzzer_Stop();
            if(IsParkingBackward() != 0U)
            {
                if(g_search_dir == SEARCH_DIR_LEFT)
                {
                    Motion_SpinLeft20();
                }
                else
                {
                    Motion_SpinRight20();
                }
            }
            else
            {
                if(g_search_dir == SEARCH_DIR_LEFT)
                {
                    Motion_SpinRight20();
                }
                else
                {
                    Motion_SpinLeft20();
                }
            }

            if((HAL_GetTick() - g_park_turn_start_tick_ms) >= ((IsParkingBackward() != 0U) ? T_SECOND_TURN_90_R2F_MS
                                                                                             : T_SECOND_TURN_90_F2R_MS))
            {
                Motion_StopBrake();
                Buzzer_Stop();
                g_park_stage = PARK_STAGE_DONE;
                g_car_mode = CAR_MODE_WAIT_DIRECTION;
                UartSend(IsParkingBackward() ? "\r\n倒车入位完成。\r\n"
                                            : "\r\n前进入位完成。\r\n");
            }
            break;

        case PARK_STAGE_VERTICAL_ALIGN:
            if(IsParkingBackward() != 0U)
            {
                rear_ok = MeasureRearDistanceCm(&rear_distance_cm);

                if(REVERSE_OBSTACLE_AVOID_ENABLE && rear_ok && (rear_distance_cm < D_SAFE_CM))
                {
                    Motion_StopBrake();
                    g_parking_paused_by_obstacle = 1U;
                }
                else if((REVERSE_OBSTACLE_AVOID_ENABLE == 0) || (rear_ok && (rear_distance_cm > D_SAFE_CM)))
                {
                    Motion_ParkBackward();
                    g_parking_paused_by_obstacle = 0U;
                    g_park_moved1_cm += V_MOVE_CM_S * dt_s;
                }

                ReverseBuzzer_Update(1U, g_parking_paused_by_obstacle, rear_ok, rear_distance_cm);
            }
            else
            {
                Motion_ParkForward();
                g_parking_paused_by_obstacle = 0U;
                g_park_moved1_cm += V_MOVE_CM_S * dt_s;
                Buzzer_Stop();
            }
            if(g_park_moved1_cm >= g_park_target1_cm)
            {
                Motion_StopBrake();
                Buzzer_Stop();
                g_park_turn_start_tick_ms = HAL_GetTick();
                g_park_stage = PARK_STAGE_VERTICAL_TURN;
            }
            break;

        case PARK_STAGE_VERTICAL_TURN:
            Buzzer_Stop();
            if(IsParkingBackward() != 0U)
            {
                if(g_search_dir == SEARCH_DIR_LEFT)
                {
                    Motion_SpinRight20();
                }
                else
                {
                    Motion_SpinLeft20();
                }
            }
            else
            {
                if(g_search_dir == SEARCH_DIR_LEFT)
                {
                    Motion_SpinLeft20();
                }
                else
                {
                    Motion_SpinRight20();
                }
            }

            if((HAL_GetTick() - g_park_turn_start_tick_ms) >= ((IsParkingBackward() != 0U) ? T_VERTICAL_TURN_90_R2F_MS
                                                                                            : T_VERTICAL_TURN_90_F2R_MS))
            {
                Motion_StopBrake();
                g_park_stage = PARK_STAGE_VERTICAL_REVERSE_IN;
            }
            break;

        case PARK_STAGE_VERTICAL_REVERSE_IN:
            if(IsParkingBackward() != 0U)
            {
                rear_ok = MeasureRearDistanceCm(&rear_distance_cm);

                if(REVERSE_OBSTACLE_AVOID_ENABLE && rear_ok && (rear_distance_cm < D_SAFE_CM))
                {
                    Motion_StopBrake();
                    g_parking_paused_by_obstacle = 1U;
                }
                else if((REVERSE_OBSTACLE_AVOID_ENABLE == 0) || (rear_ok && (rear_distance_cm > D_SAFE_CM)))
                {
                    Motion_ParkBackward();
                    g_parking_paused_by_obstacle = 0U;
                    g_park_moved2_cm += V_MOVE_CM_S * dt_s;
                }

                ReverseBuzzer_Update(1U, g_parking_paused_by_obstacle, rear_ok, rear_distance_cm);
            }
            else
            {
                Motion_ParkForward();
                g_parking_paused_by_obstacle = 0U;
                g_park_moved2_cm += V_MOVE_CM_S * dt_s;
                Buzzer_Stop();
            }
            if(g_park_moved2_cm >= g_park_target2_cm)
            {
                Motion_StopBrake();
                Buzzer_Stop();
                g_park_stage = PARK_STAGE_DONE;
                g_car_mode = CAR_MODE_WAIT_DIRECTION;
                UartSend(IsParkingBackward() ? "\r\n垂直倒车入位完成。\r\n"
                                            : "\r\n垂直前进入位完成。\r\n");
            }
            break;

        case PARK_STAGE_DONE:
        case PARK_STAGE_ABORT:
        case PARK_STAGE_INIT:
        default:
            Buzzer_Stop();
            Motion_StopBrake();
            break;
    }
}

static void HandleKeyEvent(KEY_TypeDef key)
{
    if(key == KEY_NONE)
    {
        return;
    }

    switch(g_car_mode)
    {
        case CAR_MODE_IDLE:
            if(key == KEY_UP_PRESS)
            {
                g_search_motion = SEARCH_MOTION_FORWARD;
                Motion_StopBrake();
                g_car_mode = CAR_MODE_WAIT_DIRECTION;
            }
            else if(key == KEY1_PRESS)
            {
                g_search_motion = SEARCH_MOTION_BACKWARD;
                Motion_StopBrake();
                g_car_mode = CAR_MODE_WAIT_DIRECTION;
            }
            break;

        case CAR_MODE_WAIT_DIRECTION:
        {
            char uart_buf[120];

            if(key == KEY_UP_PRESS)
            {
                g_search_motion = SEARCH_MOTION_FORWARD;
                sprintf(uart_buf, "\r\nCurrent mode: %s\r\n", GetSearchMotionLabel());
                UartSend(uart_buf);
            }
            else if(key == KEY1_PRESS)
            {
                g_search_motion = SEARCH_MOTION_BACKWARD;
                sprintf(uart_buf, "\r\nCurrent mode: %s\r\n", GetSearchMotionLabel());
                UartSend(uart_buf);
            }
            else if(key == KEY2_PRESS)
            {
                StartSlotSearch(SEARCH_DIR_LEFT);
            }
            else if(key == KEY0_PRESS)
            {
                StartSlotSearch(SEARCH_DIR_RIGHT);
            }
            break;
        }

        case CAR_MODE_SEARCHING_SLOT:
            if(key == KEY_UP_PRESS)
            {
                Motion_StopBrake();
                g_in_open_region = 0U;
                g_car_mode = CAR_MODE_WAIT_DIRECTION;
            }
            break;

        case CAR_MODE_WAIT_KEY1_CONFIRM:
            if(key == KEY1_PRESS)
            {
                Parking_Start();
            }
            break;

        case CAR_MODE_PARKING:
            if(key == KEY_UP_PRESS)
            {
                Parking_Abort("User stop");
            }
            break;

        default:
            break;
    }
}

int main(void)
{
  char uart_buf[100];
  KEY_TypeDef key;

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();

#if ESP8266_BRIDGE_TEST
  Bridge_SetUart3Baud(g_bridge_baud_table[g_bridge_baud_index]);
  sprintf(uart_buf, "\r\n================================\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 1000);
  sprintf(uart_buf, " ESP8266 UART Bridge Test Mode\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 1000);
  sprintf(uart_buf, "================================\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 1000);
  sprintf(uart_buf, "PC serial  <-> USART1 <-> STM32 <-> USART3 <-> ATK-ESP8266\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 1000);
  sprintf(uart_buf, "Use serial tool on USART1 at 115200 8N1.\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 1000);
  sprintf(uart_buf, "KEY0 next baud, KEY1 previous baud, KEY_UP report baud.\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 1000);
  sprintf(uart_buf, "Send: AT\\r\\n and watch for module replies.\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 1000);
  sprintf(uart_buf, "Make sure P2 is set to ATK-MODULE.\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 1000);
  Bridge_ReportBaud();
#else
  ESP8266_Init();

  sprintf(uart_buf,
          "[ESP8266] init=%s, stage=%s, failed_stage=%s, baud=%lu\r\n",
          (ESP8266_IsInitialized() != 0U) ? "ok" : "fail",
          ESP8266_GetInitStageName(),
          ESP8266_GetFailedStageName(),
          (unsigned long)ESP8266_GetActiveBaud());
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 1000U);
  if(ESP8266_IsInitialized() != 0U)
  {
      sprintf(uart_buf, "[ESP8266] AP=ATK-ESP8266, TCP=192.168.4.1:8086\r\n");
  }
  else
  {
      sprintf(uart_buf, "[ESP8266] init failed, check P2=ATK-MODULE and USART3.\r\n");
  }
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 1000U);

  SR04_Init(&hsr04, GPIOA, GPIO_PIN_6, &htim3, TIM_CHANNEL_2);

  Servo_Init(&hservo, &htim4, TIM_CHANNEL_1);
  Servo_SetAngle(&hservo, SERVO_CENTER_ANGLE);

  Motor_Init();
  Buzzer_Stop();

  sprintf(uart_buf, "\r\n================================\r\n");
  UartSend(uart_buf);
  sprintf(uart_buf, " Smart parking system ready\r\n");
  UartSend(uart_buf);
  sprintf(uart_buf, "================================\r\n");
  UartSend(uart_buf);
  sprintf(uart_buf, "KEY_UP: select R2F search + reverse park\r\n");
  UartSend(uart_buf);
  sprintf(uart_buf, "KEY1: select F2R search + forward park\r\n");
  UartSend(uart_buf);
  sprintf(uart_buf, "KEY2: start left-side search\r\n");
  UartSend(uart_buf);
  sprintf(uart_buf, "KEY0: start right-side search\r\n");
  UartSend(uart_buf);
  sprintf(uart_buf, "Default mode: %s\r\n", GetSearchMotionLabel());
  UartSend(uart_buf);
  sprintf(uart_buf, "After selecting mode, press KEY2 or KEY0 to start\r\n");
  UartSend(uart_buf);
  sprintf(uart_buf, "Press KEY1 after slot detection to auto park\r\n");
  UartSend(uart_buf);
  sprintf(uart_buf, "Search speed=%d%%, park speed=%d%%, spin speed=%d%%\r\n",
          MOTOR_SEARCH_SPEED_PERCENT,
          MOTOR_REVERSE_SPEED_PERCENT,
          MOTOR_SPIN_SPEED_PERCENT);
  UartSend(uart_buf);
  sprintf(uart_buf, "Mode A: R2F search, auto-detect parallel/vertical slot, then reverse park\r\n");
  UartSend(uart_buf);
  sprintf(uart_buf, "Mode B: F2R search, auto-detect parallel/vertical slot, then forward park\r\n");
  UartSend(uart_buf);
  sprintf(uart_buf, "================================\r\n\r\n");
  UartSend(uart_buf);

  HAL_Delay(500);

  sprintf(uart_buf, "Press KEY_UP or KEY1 to choose search mode.\r\n\r\n");
  UartSend(uart_buf);
#endif

  while (1)
  {
#if ESP8266_BRIDGE_TEST
    key = KEY_Scan();
    if(key == KEY0_PRESS)
    {
        g_bridge_baud_index = (uint8_t)((g_bridge_baud_index + 1U) % (sizeof(g_bridge_baud_table) / sizeof(g_bridge_baud_table[0])));
        Bridge_SetUart3Baud(g_bridge_baud_table[g_bridge_baud_index]);
        Bridge_ReportBaud();
        HAL_Delay(150);
    }
    else if(key == KEY1_PRESS)
    {
        if(g_bridge_baud_index == 0U)
        {
            g_bridge_baud_index = (uint8_t)((sizeof(g_bridge_baud_table) / sizeof(g_bridge_baud_table[0])) - 1U);
        }
        else
        {
            g_bridge_baud_index--;
        }
        Bridge_SetUart3Baud(g_bridge_baud_table[g_bridge_baud_index]);
        Bridge_ReportBaud();
        HAL_Delay(150);
    }
    else if(key == KEY_UP_PRESS)
    {
        Bridge_ReportBaud();
        HAL_Delay(150);
    }

    BridgeUartTraffic();
    HAL_Delay(1);
#else
    ESP8266_Poll();

    key = KEY_Scan();
    HandleKeyEvent(key);

    if(g_car_mode == CAR_MODE_SEARCHING_SLOT)
    {
        UpdateSlotSearch();
        HAL_Delay(30);
    }
    else if(g_car_mode == CAR_MODE_PARKING)
    {
        Parking_Update();
        HAL_Delay(20);
    }
    else
    {
        HAL_Delay(20);
    }
#endif
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
