/**
 * @file hal_uart_mock.h
 * @brief Mock implementation of STM32 HAL UART functions for testing
 */

#ifndef HAL_UART_MOCK_H
#define HAL_UART_MOCK_H

#include "fff.h"
#include "stm32h7xx_hal.h"
#include <stdint.h>

// Forward declaration of handle type
struct __UART_HandleTypeDef;
typedef struct __UART_HandleTypeDef UART_HandleTypeDef;

// Mock callback function prototypes (these are implemented by the driver)
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

// Mock callback function pointer types
typedef void (*pUART_CallbackTypeDef)(UART_HandleTypeDef *huart);
typedef void (*pUART_RxEventCallbackTypeDef)(UART_HandleTypeDef *huart, uint16_t Size);

/**
 * @brief Mock UART registers structure
 */
typedef struct {
    uint32_t ISR; /**< Status register */
    uint32_t CR1; /**< Control register 1 */
} USART_TypeDef;

/**
 * @brief Mock UART handle structure
 */
struct __UART_HandleTypeDef {
    USART_TypeDef *Instance; /**< UART registers base address */
    uint32_t ErrorCode; /**< Error code */
};

/** @name UART Flag Definitions
 * @{ */
#define UART_FLAG_IDLE 0x00000010U /**< IDLE line detected flag */
#define UART_FLAG_RXNE 0x00000020U /**< Read data register not empty */
/** @} */

/** @name UART Error Codes
 * @{ */
#define HAL_UART_ERROR_NONE 0x00000000U /**< No error */
#define HAL_UART_ERROR_PE 0x00000001U /**< Parity error */
#define HAL_UART_ERROR_NE 0x00000002U /**< Noise error */
#define HAL_UART_ERROR_FE 0x00000004U /**< Frame error */
#define HAL_UART_ERROR_ORE 0x00000008U /**< Overrun error */
#define HAL_UART_ERROR_DMA 0x00000010U /**< DMA transfer error */
/** @} */

/**
 * @brief Mock UART callback IDs
 */
typedef enum {
    HAL_UART_TX_COMPLETE_CB_ID = 0x00U, /**< TX Complete callback ID */
    HAL_UART_RX_COMPLETE_CB_ID = 0x01U, /**< RX Complete callback ID */
    HAL_UART_ERROR_CB_ID = 0x02U, /**< Error callback ID */
    HAL_UART_ABORT_COMPLETE_CB_ID = 0x03U, /**< Abort Complete callback ID */
    HAL_UART_ABORT_TX_COMPLETE_CB_ID = 0x04U, /**< Abort TX Complete callback ID */
    HAL_UART_ABORT_RX_COMPLETE_CB_ID = 0x05U, /**< Abort RX Complete callback ID */
    HAL_UART_WAKEUP_CB_ID = 0x06U, /**< Wakeup callback ID */
    HAL_UART_RX_FIFO_FULL_CB_ID = 0x07U, /**< RX FIFO Full callback ID */
    HAL_UART_TX_FIFO_EMPTY_CB_ID = 0x08U, /**< TX FIFO Empty callback ID */
    HAL_UART_MSPINIT_CB_ID = 0x09U, /**< MSP Init callback ID */
    HAL_UART_MSPDEINIT_CB_ID = 0x0AU /**< MSP DeInit callback ID */
} HAL_UART_CallbackIDTypeDef;

/** @brief Mock for HAL_UART_Transmit_IT function */
DECLARE_FAKE_VALUE_FUNC(
    HAL_StatusTypeDef, HAL_UART_Transmit_IT, UART_HandleTypeDef *, uint8_t *, uint16_t
);

/** @brief Mock for HAL_UART_Transmit_DMA function */
DECLARE_FAKE_VALUE_FUNC(
    HAL_StatusTypeDef, HAL_UART_Transmit_DMA, UART_HandleTypeDef *, uint8_t *, uint16_t
);

/** @brief Mock for HAL_UART_Receive_IT function */
DECLARE_FAKE_VALUE_FUNC(
    HAL_StatusTypeDef, HAL_UART_Receive_IT, UART_HandleTypeDef *, uint8_t *, uint16_t
);

/** @brief Mock for __HAL_UART_GET_FLAG function */
DECLARE_FAKE_VALUE_FUNC(FlagStatus, __HAL_UART_GET_FLAG, UART_HandleTypeDef *, uint32_t);

/** @brief Mock for HAL_UART_ClearIdleFlag function */
DECLARE_FAKE_VOID_FUNC(HAL_UART_ClearIdleFlag, UART_HandleTypeDef *);

/** @brief Mock for HAL_UARTEx_ReceiveToIdle_IT function */
DECLARE_FAKE_VALUE_FUNC(
    HAL_StatusTypeDef, HAL_UARTEx_ReceiveToIdle_IT, UART_HandleTypeDef *, uint8_t *, uint16_t
);

/** @brief Mock for HAL_UARTEx_ReceiveToIdle_DMA function */
DECLARE_FAKE_VALUE_FUNC(
    HAL_StatusTypeDef, HAL_UARTEx_ReceiveToIdle_DMA, UART_HandleTypeDef *, uint8_t *, uint16_t
);

/** @brief Mock for HAL_UART_GetError function */
DECLARE_FAKE_VALUE_FUNC(uint32_t, HAL_UART_GetError, UART_HandleTypeDef *);

/** @brief Mock for HAL_UART_RegisterCallback function */
DECLARE_FAKE_VALUE_FUNC(
    HAL_StatusTypeDef, HAL_UART_RegisterCallback, UART_HandleTypeDef *, HAL_UART_CallbackIDTypeDef,
    pUART_CallbackTypeDef
);

/** @brief Mock for HAL_UART_RegisterRxEventCallback function */
DECLARE_FAKE_VALUE_FUNC(
    HAL_StatusTypeDef, HAL_UART_RegisterRxEventCallback, UART_HandleTypeDef *,
    pUART_RxEventCallbackTypeDef
);

/**
 * @brief Reset all UART mocks to initial state
 * @note Call this in test setup
 */
#define UART_MOCK_RESET()                                                                          \
    do {                                                                                           \
        RESET_FAKE(HAL_UART_Transmit_IT);                                                          \
        RESET_FAKE(HAL_UART_Transmit_DMA);                                                         \
        RESET_FAKE(HAL_UART_Receive_IT);                                                           \
        RESET_FAKE(__HAL_UART_GET_FLAG);                                                           \
        RESET_FAKE(HAL_UART_ClearIdleFlag);                                                        \
        RESET_FAKE(HAL_UARTEx_ReceiveToIdle_IT);                                                   \
        RESET_FAKE(HAL_UARTEx_ReceiveToIdle_DMA);                                                  \
        RESET_FAKE(HAL_UART_GetError);                                                             \
    } while (0)

#endif // HAL_UART_MOCK_H