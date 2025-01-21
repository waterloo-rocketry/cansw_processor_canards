/**
 * @file mock_freertos.h
 * @brief Mock definitions for FreeRTOS functions
 */
#ifndef MOCK_FREERTOS_H
#define MOCK_FREERTOS_H

#include "fff.h"
#include <stdint.h>

// Type definitions needed from FreeRTOS
typedef void *SemaphoreHandle_t;
typedef uint32_t TickType_t;
typedef int32_t BaseType_t;
typedef BaseType_t portBASE_TYPE;

#define pdTRUE 1
#define pdFALSE 0
#define portMAX_DELAY 0xFFFFFFFF

// Declare FreeRTOS semaphore mock functions
DECLARE_FAKE_VALUE_FUNC(SemaphoreHandle_t, xSemaphoreCreateMutex);
DECLARE_FAKE_VALUE_FUNC(SemaphoreHandle_t, xSemaphoreCreateBinary);
DECLARE_FAKE_VALUE_FUNC(BaseType_t, xSemaphoreTake, SemaphoreHandle_t, TickType_t);
DECLARE_FAKE_VOID_FUNC(xSemaphoreGive, SemaphoreHandle_t);
DECLARE_FAKE_VOID_FUNC(vSemaphoreDelete, SemaphoreHandle_t);
DECLARE_FAKE_VALUE_FUNC(BaseType_t, xSemaphoreGiveFromISR, SemaphoreHandle_t, BaseType_t *);
DECLARE_FAKE_VOID_FUNC(portYIELD_FROM_ISR, BaseType_t);

// Convert milliseconds to ticks (simplified mock version)
#define pdMS_TO_TICKS(xTimeInMs) (xTimeInMs)

// Function to reset all FreeRTOS mocks
void mock_freertos_Reset(void);

#endif // MOCK_FREERTOS_H