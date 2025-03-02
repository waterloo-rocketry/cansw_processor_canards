#include "mock_freertos.h"
#include <stdlib.h>

// After one iteration, exit the task
static void vTaskDelay_custom_fake(TickType_t xTicksToDelay) {
    (void)xTicksToDelay;
    exit(0); // Cleanly exit after first iteration
}

DEFINE_FAKE_VOID_FUNC(vTaskDelay, TickType_t);

// Initialize the custom fake
void mock_freertos_init(void) {
    vTaskDelay_fake.custom_fake = vTaskDelay_custom_fake;
}
