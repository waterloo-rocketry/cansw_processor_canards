#ifndef MOCKS_FREERTOS_H
#define MOCKS_FREERTOS_H

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

void freertos_mocks_init(void);
void freertos_mocks_reset(void);

#endif // MOCKS_FREERTOS_H