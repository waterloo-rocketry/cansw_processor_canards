/**
 * @file mock_freertos.c
 * @brief Implementation of FreeRTOS mocks
 */
#include "mock_freertos.h"

// Define FreeRTOS mock functions
DEFINE_FAKE_VALUE_FUNC(SemaphoreHandle_t, xSemaphoreCreateMutex);
DEFINE_FAKE_VALUE_FUNC(SemaphoreHandle_t, xSemaphoreCreateBinary);
DEFINE_FAKE_VALUE_FUNC(BaseType_t, xSemaphoreTake, SemaphoreHandle_t, TickType_t);
DEFINE_FAKE_VOID_FUNC(xSemaphoreGive, SemaphoreHandle_t);
DEFINE_FAKE_VOID_FUNC(vSemaphoreDelete, SemaphoreHandle_t);
DEFINE_FAKE_VALUE_FUNC(BaseType_t, xSemaphoreGiveFromISR, SemaphoreHandle_t, BaseType_t *);
DEFINE_FAKE_VOID_FUNC(portYIELD_FROM_ISR, BaseType_t);

void mock_freertos_Reset(void)
{
    RESET_FAKE(xSemaphoreCreateMutex);
    RESET_FAKE(xSemaphoreCreateBinary);
    RESET_FAKE(xSemaphoreTake);
    RESET_FAKE(xSemaphoreGive);
    RESET_FAKE(vSemaphoreDelete);
    RESET_FAKE(xSemaphoreGiveFromISR);
    RESET_FAKE(portYIELD_FROM_ISR);

    // Set default return values
    xSemaphoreCreateMutex_fake.return_val = (SemaphoreHandle_t)1;
    xSemaphoreCreateBinary_fake.return_val = (SemaphoreHandle_t)2;
    xSemaphoreTake_fake.return_val = pdTRUE;
    xSemaphoreGiveFromISR_fake.return_val = pdTRUE;
}