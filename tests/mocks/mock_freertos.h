#ifndef MOCK_FREERTOS_H
#define MOCK_FREERTOS_H

#include "FreeRTOS.h"
#include "fff.h"
#include <stdint.h>

// ---------------------
// Declare (but dont define) FFF mocks for general freertos functions.
// Actual definitions are in the .c file to avoid multiple-definitions errors
// The comments indicate the actual function signatures
// ---------------------

// void vTaskDelay(uint32_t xTicksToDelay );
DECLARE_FAKE_VOID_FUNC(vTaskDelay, TickType_t);

#endif // MOCK_FREERTOS_H
