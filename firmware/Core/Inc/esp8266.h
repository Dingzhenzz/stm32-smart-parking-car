#ifndef __ESP8266_H__
#define __ESP8266_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void ESP8266_Init(void);
void ESP8266_Poll(void);
HAL_StatusTypeDef ESP8266_SendLog(const char *msg);

uint8_t ESP8266_IsInitialized(void);
uint8_t ESP8266_IsClientConnected(void);
const char *ESP8266_GetInitStageName(void);
const char *ESP8266_GetFailedStageName(void);
uint32_t ESP8266_GetActiveBaud(void);

#ifdef __cplusplus
}
#endif

#endif /* __ESP8266_H__ */
