#include "drivers/uart/uart.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32h7xx_hal.h"

// Define maximum allowed length for a single UART message (256 Bytes)
#define MAX_MSG_LENGTH 256 // Maximum buffer size for UART messages

// Structure defining the UART handle, which includes:
// - Pointer to the STM32 HAL UART handle
// - Semaphores to signal completion of transmission and reception
// - A buffer to store incoming data and a counter for received bytes
typedef struct
{
    UART_HandleTypeDef *hal_handle;    // Pointer to the hardware-specific UART handle
    SemaphoreHandle_t tx_complete_sem; // Semaphore to signal transmit completion
    SemaphoreHandle_t rx_complete_sem; // Semaphore to signal reception completion
    uint8_t rx_buffer[MAX_MSG_LENGTH]; // Buffer used to store received UART data
    uint16_t rx_len;                   // Counter for the number of bytes received so far
} uart_handle_t;

// Array of UART handles for each channel defined by uart_channel_t enumeration
static uart_handle_t uart_handles[UART_CHANNEL_COUNT];

// Callback function triggered by the HAL library when a UART receive operation completes.
// This function is executed in interrupt context.
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // Variable to indicate if a higher priority task needs to be woken after the ISR exits.
    BaseType_t higher_priority_task_woken = pdFALSE;

    // Identify which UART channel's handle matches the provided HAL handle.
    uart_channel_t channel;
    for (channel = 0; channel < UART_CHANNEL_COUNT; channel++)
    {
        if (uart_handles[channel].hal_handle == huart)
        {
            break; // Found the corresponding channel.
        }
    }

    // If no matching channel was found, exit the callback.
    if (channel == UART_CHANNEL_COUNT)
    {
        return; // Unknown UART handle
    }
    uart_handle_t *handle = &uart_handles[channel];

    // Check if a complete message was received:
    // The condition becomes true if an IDLE frame is detected or if the receive buffer is full.
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) || handle->rx_len >= MAX_MSG_LENGTH)
    {
        // Signal that the complete message is available by releasing the semaphore.
        xSemaphoreGiveFromISR(handle->rx_complete_sem, &higher_priority_task_woken);
    }
    // Otherwise, if the buffer is not yet full, initiate reception of the next byte.
    else if (handle->rx_len < MAX_MSG_LENGTH)
    {
        // Start an interrupt-based reception for the next byte.
        HAL_UART_Receive_IT(huart, &handle->rx_buffer[handle->rx_len], 1);
        // Increment the received byte counter.
        handle->rx_len++;
    }

    // Yield to a higher priority task if necessary when exiting the ISR.
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

w_status_t uart_read(uart_channel_t channel, uint8_t *data, uint8_t *len, uint32_t timeout)
{
    // Validate input parameters (channel index and buffer pointers).
    if (channel >= UART_CHANNEL_COUNT || data == NULL || len == NULL)
    {
        return W_INVALID_PARAM;
    }

    uart_handle_t *handle = &uart_handles[channel];

    // Wait (block) until a complete UART message is signaled via the semaphore.
    if (xSemaphoreTake(handle->rx_complete_sem, timeout) != pdTRUE)
    {
        *len = 0; // Set length to 0 on timeout
        return W_IO_TIMEOUT;
    }

    // Copy the received message from the driver buffer into the user's buffer.
    memcpy(data, handle->rx_buffer, handle->rx_len);
    *len = handle->rx_len; // Provide the received data length to the caller

    // Reset the reception state for the next incoming message.
    handle->rx_len = 0;

    // Restart the interrupt-based reception mechanism for subsequent messages.
    HAL_UART_RECEIVE_IT(handle->hal_handle, handle->rx_buffer, 1);

    return W_SUCCESS;
}
