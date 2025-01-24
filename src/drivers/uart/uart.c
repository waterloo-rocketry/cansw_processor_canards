#include "FreeRTOS.h"
#include "semphr.h"

#include "drivers/uart/uart.h"

// Datatypes ----------------------------

/**
 * The data associated with a UART channel
 */
typedef struct {
    UART_HandleTypeDef *handle; // HAL uart handle
    SemaphoreHandle_t
        transfer_complete; // Communicate transfer complete between uart_write and the ISR
    SemaphoreHandle_t write_mutex; // Allows tasks to line up to use uart_write
} uart_handle_t;

/**
 * Maps each `uart_channel_t` to its `uart_handle_t` data
 */
uart_handle_t uart_channel_map[UART_CHANNEL_COUNT];

// Private ----------------------------

/**
 * The ISR triggered when the `huart` channel has completed a transmission
 */
void uart_transmit_complete_isr(UART_HandleTypeDef *huart) {
    // TODO: do stuff...
}

// Public ----------------------------

// Must be called before RTOS scheduler starts
// Initialize the specified UART channel with given timeout value.
w_status_t uart_init(uart_channel_t device, UART_HandleTypeDef *handle) {
    w_status_t status = W_SUCCESS;

    // Register the transmit-complete ISR for this UART channel
    HAL_UART_RegisterCallback(handle, HAL_UART_TX_COMPLETE_CB_ID, uart_transmit_complete_isr);
    // TODO: check the return status of registerCallback and handle errors

    // TODO: init other stuff ...
    uart_channel_map[device].handle = handle; // init this device's handle

    return status;
}

// Write to the specified UART channel
// One task can write to a channel at once, and concurrent calls will block for `timeout`
// Returns the status of data transfer
w_status_t uart_write(uart_channel_t channel, const uint8_t *data, uint8_t len, uint32_t timeout) {
    w_status_t status = W_SUCCESS;

    // TODO: call hal uart_transmit_IT ...

    return status;
}

// Get latest full msg received by `channel`
// Will drop msgs if msgs are received faster than they are read
// Blocks for up to `timeout` ms if no msg is available
// Returns the status, received msg into `data`, msg length into `len`
w_status_t uart_read(uart_channel_t channel, uint8_t *data, uint8_t *len, uint32_t timeout) {
    w_status_t status = W_SUCCESS;
    // aaaaaaaaaaaa
    return status;
}
