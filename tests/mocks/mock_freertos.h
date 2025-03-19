#ifndef MOCK_FREERTOS_H
#define MOCK_FREERTOS_H

// Include FreeRTOS types first to ensure proper type definitions
#include "FreeRTOS.h"
#include "fff.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------
// Declare (but dont define) FFF mocks for general freertos functions.
// Actual definitions are in the .c file to avoid multiple-definitions errors
// The comments indicate the actual function signatures
// ---------------------

// void vTaskDelay(uint32_t xTicksToDelay );
DECLARE_FAKE_VOID_FUNC(vTaskDelay, TickType_t);

// TickType_t xTaskGetTickCount(void);
DECLARE_FAKE_VALUE_FUNC(TickType_t, xTaskGetTickCount);

// BaseType_t xTaskDelayUntil(TickType_t *pxPreviousWakeTime, TickType_t xTimeIncrement);
DECLARE_FAKE_VALUE_FUNC(BaseType_t, xTaskDelayUntil, TickType_t *, TickType_t);

// Function to initialize the mocks with default values
void mock_freertos_init(void);

#ifdef __cplusplus
}
#endif

#endif // MOCK_FREERTOS_H
