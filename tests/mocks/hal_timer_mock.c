/**
 * @file hal_timer_mock.c
 * @brief Implementation of Timer HAL mock functions for unit testing
 */

#include "hal_timer_mock.h"

TIM_HandleTypeDef htim2;

// Define mocks for Timer functions
DEFINE_FAKE_VALUE_FUNC(HAL_TIM_StateTypeDef, HAL_TIM_IC_GetState, TIM_HandleTypeDef *);
DEFINE_FAKE_VALUE_FUNC(uint32_t, __HAL_TIM_GET_COUNTER, TIM_HandleTypeDef *);