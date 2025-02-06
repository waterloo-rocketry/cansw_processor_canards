#include "drivers/uart/uart.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32h7xx_hal.h"

// Maximum length of a single UART message (256 Bytes)
#define MAX_MSG_LENGTH 256

// Handle structure for UART peripherals as specified in design doc
typedef struct
{
    UART_HandleTypeDef *hal_handle;    // STM32 HAL UART handle
    SemaphoreHandle_t tx_complete_sem; // Signals completion of transmission
    SemaphoreHandle_t rx_complete_sem; // Signals completion of reception
    uint8_t rx_buffer[MAX_MSG_LENGTH]; // Buffer for receiving data
    uint16_t rx_len;                   // Current length of received data
} uart_handle_t;

// Array of UART handles indexed by uart_channel_t enum
static uart_handle_t uart_handles[UART_CHANNEL_COUNT];

// UART receive complete callback, called from ISR context
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    // Find which channel this callback is for
    uart_channel_t channel;
    for (channel = 0; channel < UART_CHANNEL_COUNT; channel++)
    {
        if (uart_handles[channel].hal_handle == huart)
        {
            break;
        }
    }

    if (channel == UART_CHANNEL_COUNT)
    {
        return; // Unknown UART handle
    }
    uart_handle_t *handle = &uart_handles[channel];

    // Check if we've received a complete message (IDLE frame or max length)
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) ||
        handle->rx_len >= MAX_MSG_LENGTH)
    {

        // Signal message completion
        xSemaphoreGiveFromISR(handle->rx_complete_sem, &higher_priority_task_woken);
    }
    // Start next receive if not at max length
    else if (handle->rx_len < MAX_MSG_LENGTH)
    {
        HAL_UART_Receive_IT(huart,
                            &handle->rx_buffer[handle->rx_len],
                            1);
        handle->rx_len++;
    }

    portYIELD_FROM_ISR(higher_priority_task_woken);
}
