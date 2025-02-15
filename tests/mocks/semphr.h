#ifndef SEMPHR_H
#define SEMPHR_H

#include "fff.h"
#include "FreeRTOS.h"

// Typedef SemaphoreHandle_t as a void pointer for mocking
typedef void* SemaphoreHandle_t;

// ---------------------
// Declare (but dont define) FFF mocks for semphr functions.
// Actual definitions are in the .c file to avoid multiple-definitions errors
// The comments indicate the actual function signatures
// ---------------------

// SemaphoreHandle_t xSemaphoreCreateBinary(void);
DECLARE_FAKE_VALUE_FUNC(SemaphoreHandle_t, xSemaphoreCreateBinary)

// SemaphoreHandle_t xSemaphoreCreateMutex(void);
DECLARE_FAKE_VALUE_FUNC(SemaphoreHandle_t, xSemaphoreCreateMutex)

// BaseType_t xSemaphoreTake(SemaphoreHandle_t xSemaphore, TickType_t xTicksToWait);
DECLARE_FAKE_VALUE_FUNC(BaseType_t, xSemaphoreTake, SemaphoreHandle_t, TickType_t)

// BaseType_t xSemaphoreGive(SemaphoreHandle_t xSemaphore);
DECLARE_FAKE_VALUE_FUNC(BaseType_t, xSemaphoreGive, SemaphoreHandle_t)

#endif // SEMPHR_H
