#include "mock_freertos.h"

// Define mock here
DEFINE_FAKE_VOID_FUNC(vTaskDelay, uint32_t);

// void portMAX_DELAY(void);
#define portMAX_DELAY 100;