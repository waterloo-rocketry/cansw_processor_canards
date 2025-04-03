/**
 * @file hil.h
 * @brief Hardware-in-the-Loop interface
 *
 * This module provides functionality to support Hardware-in-the-Loop (HIL)
 * testing by allowing external control over RTOS ticks.
 */

#ifndef HIL_H
#define HIL_H

#include "rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

// HIL UART protocol constants
#define HIL_UART_HEADER_CHAR '?'
#define HIL_UART_FOOTER_CHAR '\n'


/**
 * @brief Initializes the simulator logic (e.g., Canard setup if needed here).
 * 
 * Implementation moved to hil.c
 * 
 * @return w_status_t W_SUCCESS if initialization is successful, error code otherwise.
 */
w_status_t simulator_init(void);

/**
 * @brief Process HIL UART data and trigger tick updates
 *
 * This function should be called from the UART receive interrupt
 * handler. It validates the frame, processes simulator data,
 * and triggers the tick update.
 *
 * @param data Pointer to the received UART data (including header/footer)
 * @param size Size of the received data (including header/footer)
 * @return Status of the processing (W_SUCCESS, W_INVALID_PARAM, W_ERROR)
 */
w_status_t hil_process_uart_data(const uint8_t *data, uint16_t size);

/**
 * @brief Manually advance the RTOS tick counter by one
 *
 * This function manually increments the FreeRTOS tick counter by one,
 * bypassing the normal SysTick interrupt-based tick mechanism.
 *
 * @return true if a context switch is requested, false otherwise
 */
bool hil_increment_tick(void);

#endif /* HIL_H */