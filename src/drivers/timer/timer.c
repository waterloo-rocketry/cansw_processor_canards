/**
 * Timer module
 * Driver for tracking system time
 * (currently configured to tick at 10,000 Hz or 0.1ms/tick)
 */

#include "drivers/timer/timer.h"
#include "application/logger/log.h"
#include "stm32h7xx_hal.h"

// external timer handle declaration
extern TIM_HandleTypeDef htim2;

// Error tracking
static timer_health_t timer_health = {0};
w_status_t timer_get_ms(float *ms) {
    // check if there exists a valid locatiton to store the time
    if (ms == NULL) {
        timer_health.invalid_param++;
        return W_INVALID_PARAM;
    }
    // check the timer handle pointer to ensure it is valid
    if (htim2.Instance == NULL) {
        timer_health.timer_invalid++;
        return W_FAILURE;
    }

    // check and validate whether the timer is actually running or not
    HAL_TIM_StateTypeDef state = HAL_TIM_IC_GetState(&htim2);
    if (state != HAL_TIM_STATE_BUSY) {
        timer_health.timer_stopped++;
        return W_FAILURE;
    }

    // retrieve the current timer count (in clock ticks)
    uint32_t timer_count = __HAL_TIM_GET_COUNTER(&htim2);

    // convert the timer count to milliseconds
    // each tick is 0.1 ms, so we multiply by 0.1
    *ms = (float)timer_count * 0.1f;

    timer_health.valid_calls++;
    return W_SUCCESS;
}

/**
 * @brief Report timer module health status
 *
 * Retrieves and reports timer error statistics through log messages.
 *
 * @return W_SUCCESS if reporting was successful
 */
w_status_t timer_get_status(void) {
    // Calculate total calls
    uint32_t total_calls = timer_health.valid_calls + timer_health.invalid_param +
                           timer_health.timer_stopped + timer_health.timer_invalid;

    // Log call statistics
    log_text(
        0,
        "timer",
        "Call statistics: total=%lu, successful=%lu (%.1f%%)",
        total_calls,
        timer_health.valid_calls,
        total_calls > 0 ? ((float)timer_health.valid_calls / total_calls * 100.0f) : 0.0f
    );

    // Log error statistics if any errors occurred
    uint32_t total_errors =
        timer_health.invalid_param + timer_health.timer_stopped + timer_health.timer_invalid;

    if (total_errors > 0) {
        log_text(
            0,
            "timer",
            "Error statistics: invalid_param=%lu, timer_stopped=%lu, timer_invalid=%lu",
            timer_health.invalid_param,
            timer_health.timer_stopped,
            timer_health.timer_invalid
        );

        // Log critical error if error rate is too high (more than 5%) and there are significant
        // calls
        if (total_calls > 100 && ((float)total_errors / total_calls > 0.05f)) {
            log_text(
                0,
                "timer",
                "CRITICAL ERROR: High error rate (%.1f%%), total errors=%lu",
                (float)total_errors / total_calls * 100.0f,
                total_errors
            );
        }
    }

    return W_SUCCESS;
}
