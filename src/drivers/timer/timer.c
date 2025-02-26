#include "drivers/timer/timer.h"
#include "stm32h7xx_hal.h"

// Static counter to simulate timer
static uint32_t timer_counter = 0;

w_status_t timer_get_ms(float *ms) {
    if (ms == NULL) {
        return W_INVALID_PARAM;
    }

    // For stub implementation, just return an incrementing value
    *ms = (float)(timer_counter++);

    return W_SUCCESS;
}