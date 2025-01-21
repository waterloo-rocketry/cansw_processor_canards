#ifndef MOCK_FREERTOS_H
#define MOCK_FREERTOS_H

#include "FreeRTOS.h" /* Our minimal FreeRTOS.h */
#include "fff.h"
#include "semphr.h" /* Uses our minimal semphr.h */

/* Fake declarations for the FreeRTOS primitives */
DECLARE_FAKE_VALUE_FUNC(BaseType_t, xSemaphoreTake, SemaphoreHandle_t, TickType_t);
DECLARE_FAKE_VALUE_FUNC(BaseType_t, xSemaphoreGive, SemaphoreHandle_t);
DECLARE_FAKE_VALUE_FUNC(SemaphoreHandle_t, xSemaphoreCreateMutex);
DECLARE_FAKE_VALUE_FUNC(SemaphoreHandle_t, xSemaphoreCreateBinary);
DECLARE_FAKE_VOID_FUNC(vSemaphoreDelete, SemaphoreHandle_t);

#endif // MOCK_FREERTOS_H
