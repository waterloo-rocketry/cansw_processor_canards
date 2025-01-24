#ifndef UART_H
#define UART_H

#include "rocketlib/include/common.h"
#include <stdint.h>

typedef enum {
    UART_MOVELLA, // Movella IMU
    UART_DEBUG_SERIAL, // debugger serial
    UART_CHANNEL_COUNT // Number of UART channels
} uart_channel_t;

// Must be called before RTOS scheduler starts
// Initialize the specified UART channel with given timeout value.
w_status_t uart_init(uart_channel_t device, uint32_t timeout_ms);

// Write to the specified UART channel
// One task can write to a channel at once, and concurrent calls will block for `timeout`
// Returns the status of data transfer
w_status_t uart_write(uart_channel_t channel, const uint8_t *data, uint8_t len, uint32_t timeout);

// Get latest full msg received by `channel`
// Will drop msgs if msgs are received faster than they are read
// Blocks for up to `timeout` ms if no msg is available
// Returns the status, received msg into `data`, msg length into `len`
w_status_t uart_read(uart_channel_t channel, uint8_t *data, uint8_t *len, uint32_t timeout);

#endif // UART_H