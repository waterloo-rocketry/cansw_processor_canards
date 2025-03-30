#include "mock_freertos.h"
#include <stdlib.h>

// After one iteration, exit the task
static void vTaskDelay_custom_fake(TickType_t xTicksToDelay) {
    (void)xTicksToDelay;
    // Don't exit in tests, just do nothing
}

// DEFINE_FAKE_VOID_FUNC(vTaskDelay, TickType_t);
// DEFINE_FAKE_VALUE_FUNC(TickType_t, xTaskGetTickCount);
// DEFINE_FAKE_VALUE_FUNC(BaseType_t, xTaskDelayUntil, TickType_t *, TickType_t);
// DEFINE_FAKE_VOID_FUNC(vTaskDelayUntil, TickType_t *, TickType_t);

// Initialize the custom fake
void mock_freertos_init(void) {
    vTaskDelay_fake.custom_fake = vTaskDelay_custom_fake;
    xTaskGetTickCount_fake.return_val = 0;
    xTaskDelayUntil_fake.return_val = pdTRUE;
    // No need to set anything for vTaskDelayUntil_fake, it's a void function
}


