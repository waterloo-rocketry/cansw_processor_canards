#include "semphr.h"

// Actually define the mocks here
DEFINE_FAKE_VALUE_FUNC(SemaphoreHandle_t, xSemaphoreCreateMutex);
DEFINE_FAKE_VALUE_FUNC(BaseType_t, xSemaphoreTake, SemaphoreHandle_t, TickType_t);
DEFINE_FAKE_VALUE_FUNC(BaseType_t, xSemaphoreGive, SemaphoreHandle_t);
