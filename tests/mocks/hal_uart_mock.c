/**
 * @file hal_uart_mock.c
 * @brief Implementation of UART HAL mock functions for unit testing
 */

#include "hal_uart_mock.h"
#include <stdint.h>

// Define mocks for UART functions
DEFINE_FAKE_VALUE_FUNC(
    HAL_StatusTypeDef, HAL_UART_Transmit_IT, UART_HandleTypeDef *, uint8_t *, uint16_t
);
DEFINE_FAKE_VALUE_FUNC(
    HAL_StatusTypeDef, HAL_UART_Transmit_DMA, UART_HandleTypeDef *, uint8_t *, uint16_t
);
DEFINE_FAKE_VALUE_FUNC(
    HAL_StatusTypeDef, HAL_UART_Receive_IT, UART_HandleTypeDef *, uint8_t *, uint16_t
);
DEFINE_FAKE_VALUE_FUNC(FlagStatus, __HAL_UART_GET_FLAG, UART_HandleTypeDef *, uint32_t);
DEFINE_FAKE_VOID_FUNC(HAL_UART_ClearIdleFlag, UART_HandleTypeDef *);
DEFINE_FAKE_VALUE_FUNC(
    HAL_StatusTypeDef, HAL_UARTEx_ReceiveToIdle_IT, UART_HandleTypeDef *, uint8_t *, uint16_t
);
DEFINE_FAKE_VALUE_FUNC(
    HAL_StatusTypeDef, HAL_UARTEx_ReceiveToIdle_DMA, UART_HandleTypeDef *, uint8_t *, uint16_t
);
DEFINE_FAKE_VALUE_FUNC(uint32_t, HAL_UART_GetError, UART_HandleTypeDef *);
DEFINE_FAKE_VALUE_FUNC(
    HAL_StatusTypeDef, HAL_UART_RegisterCallback, UART_HandleTypeDef *, HAL_UART_CallbackIDTypeDef,
    pUART_CallbackTypeDef
);
DEFINE_FAKE_VALUE_FUNC(
    HAL_StatusTypeDef, HAL_UART_RegisterRxEventCallback, UART_HandleTypeDef *,
    pUART_RxEventCallbackTypeDef
);