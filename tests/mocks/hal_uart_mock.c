/**
 * @file hal_uart_mock.c
 * @brief Implementation of UART HAL mock functions for unit testing
 */

#include "hal_uart_mock.h"

/** @name Mock Function Definitions
 * @{ */

/** @brief Mock for HAL_UART_Receive_IT */
DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_UART_Receive_IT, UART_HandleTypeDef *, uint8_t *, uint16_t);

/** @brief Mock for __HAL_UART_GET_FLAG */
DEFINE_FAKE_VALUE_FUNC(FlagStatus, __HAL_UART_GET_FLAG, UART_HandleTypeDef *, uint32_t);

/** @brief Mock for HAL_UART_ClearIdleFlag */
DEFINE_FAKE_VOID_FUNC(HAL_UART_ClearIdleFlag, UART_HandleTypeDef *);

/** @brief Mock for HAL_UARTEx_ReceiveToIdle_IT */
DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_UARTEx_ReceiveToIdle_IT, UART_HandleTypeDef *, uint8_t *, uint16_t);

/** @brief Mock for HAL_UART_GetError */
DEFINE_FAKE_VALUE_FUNC(uint32_t, HAL_UART_GetError, UART_HandleTypeDef *);

/** @brief Mock for HAL_UART_RegisterCallback */
DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_UART_RegisterCallback, UART_HandleTypeDef *, HAL_UART_CallbackIDTypeDef, void (*)(UART_HandleTypeDef *));

/** @} */