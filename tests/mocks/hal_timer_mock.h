#ifndef HAL_TIMER_MOCK_H
#define HAL_TIMER_MOCK_H

#include "fff.h"
#include <stdint.h>

/** @brief Mock Timer state enumeration */
typedef enum {
    HAL_TIM_STATE_RESET = 0x00, /**< Timer not initialized */
    HAL_TIM_STATE_READY = 0x01, /**< Timer initialized and ready */
    HAL_TIM_STATE_BUSY = 0x02, /**< Timer internal process ongoing */
    HAL_TIM_STATE_ERROR = 0x03 /**< Timer error state */
} HAL_TIM_StateTypeDef;

/** @brief Mock Timer registers structure */
typedef struct {
    uint32_t CNT; /**< Timer counter register */
    uint32_t CR1; /**< Timer control register 1 */
} TIM_TypeDef;

/** @brief Mock Timer handle structure */
typedef struct {
    TIM_TypeDef *Instance; /**< Timer registers base address */
    HAL_TIM_StateTypeDef State; /**< Timer operating state */
} TIM_HandleTypeDef;

/* Mock function declarations */
DECLARE_FAKE_VALUE_FUNC(HAL_TIM_StateTypeDef, HAL_TIM_IC_GetState, TIM_HandleTypeDef *);
DECLARE_FAKE_VALUE_FUNC(uint32_t, __HAL_TIM_GET_COUNTER, TIM_HandleTypeDef *);

extern TIM_HandleTypeDef htim2;

#endif // HAL_TIMER_MOCK_H
