#include "hal_uart_mock.h"

// Define storage for the fake functions
DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_UART_Receive_IT, UART_HandleTypeDef *, uint8_t *, uint16_t);
DEFINE_FAKE_VALUE_FUNC(FlagStatus, __HAL_UART_GET_FLAG, UART_HandleTypeDef *, uint32_t);
DEFINE_FAKE_VOID_FUNC(HAL_UART_ClearIdleFlag, UART_HandleTypeDef *);