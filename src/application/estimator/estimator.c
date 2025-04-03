#include "application/estimator/estimator.h"
#include "FreeRTOS.h"
#include "task.h"

// Stub implementation
w_status_t estimator_init() {
    return W_SUCCESS;
}

w_status_t estimator_update_inputs_imu(estimator_all_imus_input_t *data) {
    (void)data; // Explicitly mark parameter as unused
    // Just a stub that accepts data but does nothing with it
    return W_SUCCESS;
}

/**
 * @brief FreeRTOS task for the estimator.
 *
 * @param pvParameters Unused.
 */
void estimator_task(void *pvParameters) {
    (void)pvParameters; // Mark unused

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // Example: Run every 10 ms

    for (;;) {
        // Task logic would go here in the future

        // Delay until the next execution time
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

