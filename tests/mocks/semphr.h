#ifndef MOCK_SEMPHR_H
#define MOCK_SEMPHR_H

#include "fff.h"
#include "FreeRTOS.h"

// Semaphore types from FreeRTOS semphr.h
typedef void *SemaphoreHandle_t;
typedef void *QueueHandle_t;

// Mock function declarations for semaphore operations
DECLARE_FAKE_VALUE_FUNC(SemaphoreHandle_t, xSemaphoreCreateMutex);
DECLARE_FAKE_VALUE_FUNC(SemaphoreHandle_t, xSemaphoreCreateBinary);
DECLARE_FAKE_VALUE_FUNC(BaseType_t, xSemaphoreTake, SemaphoreHandle_t, TickType_t);
DECLARE_FAKE_VALUE_FUNC(BaseType_t, xSemaphoreGive, SemaphoreHandle_t);
DECLARE_FAKE_VALUE_FUNC(BaseType_t, xSemaphoreGiveFromISR, SemaphoreHandle_t, BaseType_t *);
DECLARE_FAKE_VOID_FUNC(vSemaphoreDelete, SemaphoreHandle_t);

// Helper functions
void mock_semphr_init(void);
void mock_semphr_verify(void);
void mock_semphr_reset(void);

#endif // MOCK_SEMPHR_H