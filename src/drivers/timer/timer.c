/**
 * Timer module
 * Driver for tracking system time
 * (currently configured to tick at 10,000 Hz or 0.1ms/tick)
 */

#include "timer.h"
#ifdef GTEST
#include "hal_timer_mock.h"
#else
#include "stm32h7xx_hal.h"
#endif

// external timer handle declaration
extern TIM_HandleTypeDef htim2;

w_status_t timer_get_ms(float *ms) {
    // check if there exists a valid locatiton to store the time
    if (ms == NULL) {
        return W_INVALID_PARAM;
    }

    // check the timer handle pointer to ensure it is valid
    if (htim2.Instance == NULL) {
        return W_FAILURE;
    }

    // check and validate whether the timer is actually running or not
    HAL_TIM_StateTypeDef state = HAL_TIM_IC_GetState(&htim2);
    if (state != HAL_TIM_STATE_BUSY) {
        return W_FAILURE;
    }

    // retrieve the current timer count (in clock ticks)
    uint32_t timer_count = __HAL_TIM_GET_COUNTER(&htim2);

    // convert the timer count to milliseconds
    // each tick is 0.1 ms, so we multiply by 0.1
    *ms = (float)timer_count * 0.1f;

    return W_SUCCESS;
}
