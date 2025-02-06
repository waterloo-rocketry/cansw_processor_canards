#ifndef HAL_UART_MOCK_H
#define HAL_UART_MOCK_H

#include "fff.h"
#include <stdint.h>

// Mock the basic HAL types we need
typedef enum
{
    HAL_OK = 0x00,
    HAL_ERROR = 0x01,
    HAL_BUSY = 0x02,
    HAL_TIMEOUT = 0x03
} HAL_StatusTypeDef;

typedef enum
{
    RESET = 0,
    SET = !RESET
} FlagStatus;

// Mock minimal UART structures needed
typedef struct
{
    uint32_t ISR; // UART status register
    uint32_t CR1; // Control register 1
    // Add other registers if needed
} USART_TypeDef;

typedef struct
{
    USART_TypeDef *Instance; // UART registers base address
    uint32_t ErrorCode;      // Error code
    // Add other fields if needed
} UART_HandleTypeDef;

// Mock UART flag definitions
#define UART_FLAG_IDLE 0x00000010U // Made up value for testing
#define UART_FLAG_RXNE 0x00000020U // Made up value for testing

// Define the main UART receive function mock
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_UART_Receive_IT, UART_HandleTypeDef *, uint8_t *, uint16_t);

// Define mock for checking IDLE flag
DECLARE_FAKE_VALUE_FUNC(FlagStatus, __HAL_UART_GET_FLAG, UART_HandleTypeDef *, uint32_t);

// Define mock for clearing IDLE flag
DECLARE_FAKE_VOID_FUNC(HAL_UART_ClearIdleFlag, UART_HandleTypeDef *);

// Reset all mocks - call this in test setup
#define UART_MOCK_RESET()                   \
    do                                      \
    {                                       \
        RESET_FAKE(HAL_UART_Receive_IT);    \
        RESET_FAKE(__HAL_UART_GET_FLAG);    \
        RESET_FAKE(HAL_UART_ClearIdleFlag); \
    } while (0)

#endif