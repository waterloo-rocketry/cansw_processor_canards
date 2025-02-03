#include "semphr.h"

// Define the mock functions
DEFINE_FAKE_VALUE_FUNC(SemaphoreHandle_t, xSemaphoreCreateMutex);
DEFINE_FAKE_VALUE_FUNC(SemaphoreHandle_t, xSemaphoreCreateBinary);
DEFINE_FAKE_VALUE_FUNC(BaseType_t, xSemaphoreTake, SemaphoreHandle_t, TickType_t);
DEFINE_FAKE_VALUE_FUNC(BaseType_t, xSemaphoreGive, SemaphoreHandle_t);
DEFINE_FAKE_VALUE_FUNC(BaseType_t, xSemaphoreGiveFromISR, SemaphoreHandle_t, BaseType_t *);
DEFINE_FAKE_VOID_FUNC(vSemaphoreDelete, SemaphoreHandle_t);

void mock_semphr_init(void)
{
    RESET_FAKE(xSemaphoreCreateMutex);
    RESET_FAKE(xSemaphoreCreateBinary);
    RESET_FAKE(xSemaphoreTake);
    RESET_FAKE(xSemaphoreGive);
    RESET_FAKE(xSemaphoreGiveFromISR);
    RESET_FAKE(vSemaphoreDelete);
}

void mock_semphr_verify(void)
{
    // Add verification logic if needed
}

void mock_semphr_reset(void)
{
    FFF_RESET_HISTORY();
    mock_semphr_init();
}