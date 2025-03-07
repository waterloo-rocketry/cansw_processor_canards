#include "hal_timer_mock.h"

// handle to be used in testing
TIM_HandleTypeDef htim2;

// define mocks for timer functions
DEFINE_FAKE_VALUE_FUNC(HAL_TIM_StateTypeDef, HAL_TIM_IC_GetState, TIM_HandleTypeDef *);
DEFINE_FAKE_VALUE_FUNC(uint32_t, __HAL_TIM_GET_COUNTER, TIM_HandleTypeDef *);