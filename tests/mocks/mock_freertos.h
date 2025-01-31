#ifndef MOCKS_FREERTOS_H
#define MOCKS_FREERTOS_H

#include "FreeRTOS.h"
#include "fff.h"
#include "semphr.h"

// FFF function declarations
FAKE_VALUE_FUNC(SemaphoreHandle_t, xSemaphoreCreateMutex);
FAKE_VALUE_FUNC(SemaphoreHandle_t, xSemaphoreCreateBinary);
FAKE_VOID_FUNC(vSemaphoreDelete, SemaphoreHandle_t);
FAKE_VALUE_FUNC(BaseType_t, xSemaphoreTake, SemaphoreHandle_t, TickType_t);
FAKE_VALUE_FUNC(BaseType_t, xSemaphoreGive, SemaphoreHandle_t);

void freertos_mocks_init(void);
void freertos_mocks_reset(void);

#endif // MOCKS_FREERTOS_H