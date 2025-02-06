#include "drivers/uart/uart.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32h7xx_hal.h"

// Maximum length of a single UART message (256 Bytes)
#define MAX_MSG_LENGTH 256

// Handle structure for UART peripherals as specified in design doc
typedef struct {
    UART_HandleTypeDef *hal_handle; // STM32 HAL UART handle
    SemaphoreHandle_t tx_complete_sem; // Signals completion of transmission
    SemaphoreHandle_t rx_complete_sem; // Signals completion of reception
    uint8_t rx_buffer[MAX_MSG_LENGTH]; // Buffer for receiving data
    uint16_t rx_len; // Current length of received data
} uart_handle_t;

// Array of UART handles indexed by uart_channel_t enum
static uart_handle_t uart_handles[UART_CHANNEL_COUNT];

