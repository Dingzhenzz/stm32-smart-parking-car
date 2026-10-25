#include "esp8266.h"

#include <stdio.h>
#include <string.h>

#include "usart.h"

#define ESP8266_RX_WINDOW_SIZE         192U
#define ESP8266_LINE_BUFFER_SIZE        96U
#define ESP8266_MATCH_BUFFER_SIZE      192U

#define ESP8266_FLUSH_MS                30U
#define ESP8266_AT_TIMEOUT_MS          800U
#define ESP8266_RESET_TIMEOUT_MS      5000U
#define ESP8266_AP_TIMEOUT_MS         3000U
#define ESP8266_SERVER_TIMEOUT_MS     5000U
#define ESP8266_PROMPT_TIMEOUT_MS      500U
#define ESP8266_SEND_TIMEOUT_MS        600U

typedef enum
{
    ESP8266_STAGE_NONE = 0,
    ESP8266_STAGE_AT,
    ESP8266_STAGE_RESET,
    ESP8266_STAGE_MODE,
    ESP8266_STAGE_AP,
    ESP8266_STAGE_MUX,
    ESP8266_STAGE_SERVER,
    ESP8266_STAGE_READY
} ESP8266_InitStage_t;

static uint8_t s_initialized = 0U;
static uint8_t s_client_connected = 0U;
static uint8_t s_link_id = 0U;
static uint32_t s_active_baud = 115200U;
static ESP8266_InitStage_t s_init_stage = ESP8266_STAGE_NONE;
static ESP8266_InitStage_t s_failed_stage = ESP8266_STAGE_NONE;
static char s_rx_window[ESP8266_RX_WINDOW_SIZE];
static char s_line_buffer[ESP8266_LINE_BUFFER_SIZE];
static uint16_t s_line_len = 0U;

static const char *ESP8266_GetStageName(ESP8266_InitStage_t stage);
static void ESP8266_ResetRuntime(void);
static void ESP8266_AppendWindowChar(uint8_t ch);
static void ESP8266_ParseEventLine(const char *line);
static void ESP8266_ProcessRxByte(uint8_t ch);
static void ESP8266_FlushRx(uint32_t timeout_ms);
static int32_t ESP8266_WaitForAny(const char * const *patterns, uint32_t count, uint32_t timeout_ms);
static HAL_StatusTypeDef ESP8266_SendRaw(const char *data, uint16_t len);
static HAL_StatusTypeDef ESP8266_SendCommandAndWait(const char *cmd,
                                                    const char *expect,
                                                    uint32_t timeout_ms,
                                                    ESP8266_InitStage_t stage);
static HAL_StatusTypeDef ESP8266_SendCommandAndWaitAny(const char *cmd,
                                                       const char * const *expects,
                                                       uint32_t expect_count,
                                                       uint32_t timeout_ms,
                                                       ESP8266_InitStage_t stage);
static HAL_StatusTypeDef ESP8266_ResetAndSync(void);
static HAL_StatusTypeDef ESP8266_EnableServer(void);
static void ESP8266_TryDisableEcho(void);
static void ESP8266_UpdateLinkFromStatusText(const char *text);
static void ESP8266_QueryClientState(void);
static HAL_StatusTypeDef ESP8266_BeginSendOnLink(uint8_t link_id, uint32_t len);

static const char *ESP8266_GetStageName(ESP8266_InitStage_t stage)
{
    switch(stage)
    {
        case ESP8266_STAGE_AT:
            return "AT";
        case ESP8266_STAGE_RESET:
            return "RESET";
        case ESP8266_STAGE_MODE:
            return "MODE";
        case ESP8266_STAGE_AP:
            return "AP";
        case ESP8266_STAGE_MUX:
            return "MUX";
        case ESP8266_STAGE_SERVER:
            return "SERVER";
        case ESP8266_STAGE_READY:
            return "READY";
        case ESP8266_STAGE_NONE:
        default:
            return "NONE";
    }
}

static void ESP8266_ResetRuntime(void)
{
    s_initialized = 0U;
    s_client_connected = 0U;
    s_link_id = 0U;
    s_init_stage = ESP8266_STAGE_NONE;
    s_failed_stage = ESP8266_STAGE_NONE;
    s_line_len = 0U;
    s_rx_window[0] = '\0';
    s_line_buffer[0] = '\0';
    s_active_baud = huart3.Init.BaudRate;
}

static void ESP8266_AppendWindowChar(uint8_t ch)
{
    uint32_t len;
    char normalized;

    normalized = (char)ch;
    if((normalized < 32) && (normalized != '\r') && (normalized != '\n') && (normalized != '\t'))
    {
        normalized = ' ';
    }

    len = (uint32_t)strlen(s_rx_window);
    if(len >= (ESP8266_RX_WINDOW_SIZE - 1U))
    {
        memmove(s_rx_window, s_rx_window + 1, ESP8266_RX_WINDOW_SIZE - 2U);
        len = ESP8266_RX_WINDOW_SIZE - 2U;
    }

    s_rx_window[len] = normalized;
    s_rx_window[len + 1U] = '\0';
}

static void ESP8266_ParseEventLine(const char *line)
{
    uint32_t line_len;

    if(line == NULL)
    {
        return;
    }

    line_len = (uint32_t)strlen(line);
    if((line_len >= 9U) && (strstr(line, ",CONNECT") != NULL))
    {
        if((line[0] >= '0') && (line[0] <= '4'))
        {
            s_link_id = (uint8_t)(line[0] - '0');
            s_client_connected = 1U;
        }
    }

    if((line_len >= 8U) && (strstr(line, ",CLOSED") != NULL))
    {
        if((line[0] >= '0') && (line[0] <= '4'))
        {
            if((uint8_t)(line[0] - '0') == s_link_id)
            {
                s_client_connected = 0U;
            }
        }
        else
        {
            s_client_connected = 0U;
        }
    }

    if((strstr(line, "link is not valid") != NULL) ||
       (strstr(line, "SEND FAIL") != NULL) ||
       (strstr(line, "ERROR") != NULL))
    {
        s_client_connected = 0U;
    }
}

static void ESP8266_UpdateLinkFromStatusText(const char *text)
{
    const char *p;

    if(text == NULL)
    {
        return;
    }

    p = strstr(text, "+CIPSTATUS:");
    while(p != NULL)
    {
        p += 11;
        if((*p >= '0') && (*p <= '4') && (*(p + 1) == ','))
        {
            s_link_id = (uint8_t)(*p - '0');
            s_client_connected = 1U;
            return;
        }

        p = strstr(p, "+CIPSTATUS:");
    }
}

static void ESP8266_ProcessRxByte(uint8_t ch)
{
    ESP8266_AppendWindowChar(ch);

    if((ch == '\r') || (ch == '\n'))
    {
        if(s_line_len > 0U)
        {
            s_line_buffer[s_line_len] = '\0';
            ESP8266_ParseEventLine(s_line_buffer);
            s_line_len = 0U;
            s_line_buffer[0] = '\0';
        }
        return;
    }

    if(s_line_len >= (ESP8266_LINE_BUFFER_SIZE - 1U))
    {
        memmove(s_line_buffer, s_line_buffer + 1, ESP8266_LINE_BUFFER_SIZE - 2U);
        s_line_len = ESP8266_LINE_BUFFER_SIZE - 2U;
    }

    s_line_buffer[s_line_len++] = (char)ch;
    s_line_buffer[s_line_len] = '\0';
}

static void ESP8266_FlushRx(uint32_t timeout_ms)
{
    uint32_t start_tick;
    uint8_t ch;

    start_tick = HAL_GetTick();
    while((HAL_GetTick() - start_tick) < timeout_ms)
    {
        if(HAL_UART_Receive(&huart3, &ch, 1U, 0U) == HAL_OK)
        {
            ESP8266_ProcessRxByte(ch);
            start_tick = HAL_GetTick();
        }
        else
        {
            HAL_Delay(1U);
        }
    }
}

static int32_t ESP8266_WaitForAny(const char * const *patterns, uint32_t count, uint32_t timeout_ms)
{
    char match_buffer[ESP8266_MATCH_BUFFER_SIZE];
    uint32_t match_len;
    uint32_t start_tick;
    uint32_t i;
    uint8_t ch;

    match_buffer[0] = '\0';
    match_len = 0U;
    start_tick = HAL_GetTick();

    while((HAL_GetTick() - start_tick) < timeout_ms)
    {
        if(HAL_UART_Receive(&huart3, &ch, 1U, 2U) == HAL_OK)
        {
            ESP8266_ProcessRxByte(ch);

            if(match_len >= (ESP8266_MATCH_BUFFER_SIZE - 1U))
            {
                memmove(match_buffer, match_buffer + 1, ESP8266_MATCH_BUFFER_SIZE - 2U);
                match_len = ESP8266_MATCH_BUFFER_SIZE - 2U;
            }

            match_buffer[match_len++] = (char)ch;
            match_buffer[match_len] = '\0';

            for(i = 0U; i < count; i++)
            {
                if((patterns[i] != NULL) && (strstr(match_buffer, patterns[i]) != NULL))
                {
                    return (int32_t)i;
                }
            }
        }
        else
        {
            HAL_Delay(1U);
        }
    }

    return -1;
}

static HAL_StatusTypeDef ESP8266_SendRaw(const char *data, uint16_t len)
{
    if((data == NULL) || (len == 0U))
    {
        return HAL_OK;
    }

    return HAL_UART_Transmit(&huart3, (uint8_t *)data, len, 1000U);
}

static HAL_StatusTypeDef ESP8266_SendCommandAndWait(const char *cmd,
                                                    const char *expect,
                                                    uint32_t timeout_ms,
                                                    ESP8266_InitStage_t stage)
{
    const char *expects[1];

    expects[0] = expect;
    return ESP8266_SendCommandAndWaitAny(cmd, expects, 1U, timeout_ms, stage);
}

static HAL_StatusTypeDef ESP8266_SendCommandAndWaitAny(const char *cmd,
                                                       const char * const *expects,
                                                       uint32_t expect_count,
                                                       uint32_t timeout_ms,
                                                       ESP8266_InitStage_t stage)
{
    static const char *fail_patterns[] =
    {
        "ERROR",
        "FAIL",
        "busy",
        "link is not valid"
    };
    const char *patterns[8];
    uint32_t i;
    int32_t result;

    if((cmd == NULL) || (expects == NULL) || (expect_count == 0U) || (expect_count > 4U))
    {
        s_failed_stage = stage;
        return HAL_ERROR;
    }

    ESP8266_FlushRx(ESP8266_FLUSH_MS);

    if(ESP8266_SendRaw(cmd, (uint16_t)strlen(cmd)) != HAL_OK)
    {
        s_failed_stage = stage;
        return HAL_ERROR;
    }

    for(i = 0U; i < expect_count; i++)
    {
        patterns[i] = expects[i];
    }
    patterns[expect_count + 0U] = fail_patterns[0];
    patterns[expect_count + 1U] = fail_patterns[1];
    patterns[expect_count + 2U] = fail_patterns[2];
    patterns[expect_count + 3U] = fail_patterns[3];
    result = ESP8266_WaitForAny(patterns, expect_count + 4U, timeout_ms);

    if((result >= 0) && ((uint32_t)result < expect_count))
    {
        s_init_stage = stage;
        return HAL_OK;
    }

    s_failed_stage = stage;
    return (result < 0) ? HAL_TIMEOUT : HAL_ERROR;
}

static HAL_StatusTypeDef ESP8266_ResetAndSync(void)
{
    static const char *reset_patterns[] =
    {
        "OK",
        "ready",
        "reay",
        "ray",
        "ALIENTEK",
        "Cloud Integrated"
    };
    uint32_t attempt;

    ESP8266_FlushRx(ESP8266_FLUSH_MS);

    if(ESP8266_SendRaw("AT+RST\r\n", 8U) != HAL_OK)
    {
        s_failed_stage = ESP8266_STAGE_RESET;
        return HAL_ERROR;
    }

    (void)ESP8266_WaitForAny(reset_patterns, 6U, 1500U);

    HAL_Delay(1800U);
    ESP8266_FlushRx(200U);

    for(attempt = 0U; attempt < 5U; attempt++)
    {
        if(ESP8266_SendCommandAndWait("AT\r\n", "OK", ESP8266_AT_TIMEOUT_MS, ESP8266_STAGE_RESET) == HAL_OK)
        {
            return HAL_OK;
        }

        HAL_Delay(250U);
        ESP8266_FlushRx(50U);
    }

    s_failed_stage = ESP8266_STAGE_RESET;
    return HAL_TIMEOUT;
}

static HAL_StatusTypeDef ESP8266_EnableServer(void)
{
    static const char *server_ok_patterns[] =
    {
        "OK",
        "no change",
        "ALREADY",
        "Linked"
    };
    uint32_t attempt;

    HAL_Delay(1200U);
    ESP8266_FlushRx(100U);

    for(attempt = 0U; attempt < 3U; attempt++)
    {
        if(ESP8266_SendCommandAndWaitAny("AT+CIPSERVER=1,8086\r\n",
                                         server_ok_patterns,
                                         4U,
                                         ESP8266_SERVER_TIMEOUT_MS,
                                         ESP8266_STAGE_SERVER) == HAL_OK)
        {
            return HAL_OK;
        }

        HAL_Delay(500U);
        ESP8266_FlushRx(100U);
    }

    s_failed_stage = ESP8266_STAGE_SERVER;
    return HAL_TIMEOUT;
}

static void ESP8266_TryDisableEcho(void)
{
    static const char *patterns[] =
    {
        "OK",
        "ERROR"
    };

    ESP8266_FlushRx(ESP8266_FLUSH_MS);
    if(ESP8266_SendRaw("ATE0\r\n", 6U) == HAL_OK)
    {
        (void)ESP8266_WaitForAny(patterns, 2U, 500U);
    }
    ESP8266_FlushRx(50U);
}

static void ESP8266_QueryClientState(void)
{
    static const char *patterns[] =
    {
        "+CIPSTATUS:",
        "STATUS:",
        "OK",
        "ERROR",
        "FAIL"
    };

    ESP8266_FlushRx(ESP8266_FLUSH_MS);
    s_client_connected = 0U;

    if(ESP8266_SendRaw("AT+CIPSTATUS\r\n", 14U) != HAL_OK)
    {
        return;
    }

    (void)ESP8266_WaitForAny(patterns, 5U, 1200U);
    HAL_Delay(50U);
    ESP8266_FlushRx(80U);
    ESP8266_UpdateLinkFromStatusText(s_rx_window);
}

static HAL_StatusTypeDef ESP8266_BeginSendOnLink(uint8_t link_id, uint32_t len)
{
    char cmd[32];
    static const char *prompt_patterns[] = {" >", ">", "ERROR", "FAIL", "busy", "link is not valid"};
    int32_t result;
    int written;

    written = snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u,%lu\r\n", (unsigned int)link_id, (unsigned long)len);
    if((written <= 0) || ((uint32_t)written >= sizeof(cmd)))
    {
        return HAL_ERROR;
    }

    ESP8266_FlushRx(ESP8266_FLUSH_MS);
    if(ESP8266_SendRaw(cmd, (uint16_t)written) != HAL_OK)
    {
        return HAL_ERROR;
    }

    result = ESP8266_WaitForAny(prompt_patterns, 6U, ESP8266_PROMPT_TIMEOUT_MS);
    if((result == 0) || (result == 1))
    {
        s_link_id = link_id;
        s_client_connected = 1U;
        return HAL_OK;
    }

    if(result >= 2)
    {
        s_client_connected = 0U;
    }

    return (result < 0) ? HAL_TIMEOUT : HAL_ERROR;
}

void ESP8266_Init(void)
{
    ESP8266_ResetRuntime();

    if(ESP8266_SendCommandAndWait("AT\r\n", "OK", ESP8266_AT_TIMEOUT_MS, ESP8266_STAGE_AT) != HAL_OK)
    {
        return;
    }

    if(ESP8266_ResetAndSync() != HAL_OK)
    {
        return;
    }

    ESP8266_TryDisableEcho();

    HAL_Delay(200U);

    if(ESP8266_SendCommandAndWait("AT+CWMODE=2\r\n", "OK", ESP8266_AT_TIMEOUT_MS, ESP8266_STAGE_MODE) != HAL_OK)
    {
        return;
    }

    if(ESP8266_SendCommandAndWait("AT+CWSAP=\"ATK-ESP8266\",\"12345678\",1,4\r\n",
                                  "OK",
                                  ESP8266_AP_TIMEOUT_MS,
                                  ESP8266_STAGE_AP) != HAL_OK)
    {
        return;
    }

    if(ESP8266_SendCommandAndWait("AT+CIPMUX=1\r\n", "OK", ESP8266_AT_TIMEOUT_MS, ESP8266_STAGE_MUX) != HAL_OK)
    {
        return;
    }

    if(ESP8266_EnableServer() != HAL_OK)
    {
        return;
    }

    s_initialized = 1U;
    s_failed_stage = ESP8266_STAGE_NONE;
    s_init_stage = ESP8266_STAGE_READY;
    ESP8266_FlushRx(50U);
}

void ESP8266_Poll(void)
{
    uint8_t ch;

    while(HAL_UART_Receive(&huart3, &ch, 1U, 0U) == HAL_OK)
    {
        ESP8266_ProcessRxByte(ch);
    }
}

HAL_StatusTypeDef ESP8266_SendLog(const char *msg)
{
    static const char *send_patterns[] = {"SEND OK", "ERROR", "FAIL", "busy", "link is not valid"};
    int32_t result;
    uint32_t len;
    uint8_t try_link;
    uint8_t started;

    if((msg == NULL) || (msg[0] == '\0'))
    {
        return HAL_OK;
    }

    if(s_initialized == 0U)
    {
        return HAL_BUSY;
    }

    ESP8266_Poll();

    len = (uint32_t)strlen(msg);
    started = 0U;

    if(s_client_connected != 0U)
    {
        if(ESP8266_BeginSendOnLink(s_link_id, len) == HAL_OK)
        {
            started = 1U;
        }
    }

    if(started == 0U)
    {
        ESP8266_QueryClientState();
        if(s_client_connected != 0U)
        {
            if(ESP8266_BeginSendOnLink(s_link_id, len) == HAL_OK)
            {
                started = 1U;
            }
        }
    }

    if(started == 0U)
    {
        for(try_link = 0U; try_link < 5U; try_link++)
        {
            if(ESP8266_BeginSendOnLink(try_link, len) == HAL_OK)
            {
                started = 1U;
                break;
            }
        }
    }

    if(started == 0U)
    {
        return HAL_BUSY;
    }

    if(ESP8266_SendRaw(msg, (uint16_t)len) != HAL_OK)
    {
        return HAL_ERROR;
    }

    result = ESP8266_WaitForAny(send_patterns, 5U, ESP8266_SEND_TIMEOUT_MS);
    if(result == 0)
    {
        return HAL_OK;
    }

    if(result >= 1)
    {
        s_client_connected = 0U;
    }

    return (result < 0) ? HAL_TIMEOUT : HAL_ERROR;
}

uint8_t ESP8266_IsInitialized(void)
{
    return s_initialized;
}

uint8_t ESP8266_IsClientConnected(void)
{
    return s_client_connected;
}

const char *ESP8266_GetInitStageName(void)
{
    return ESP8266_GetStageName(s_init_stage);
}

const char *ESP8266_GetFailedStageName(void)
{
    return ESP8266_GetStageName(s_failed_stage);
}

uint32_t ESP8266_GetActiveBaud(void)
{
    return s_active_baud;
}
