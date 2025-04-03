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
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Manually advance the RTOS tick counter by one
 * 
 * This function manually increments the FreeRTOS tick counter by one,
 * bypassing the normal SysTick interrupt-based tick mechanism.
 * 
 * @return true if a context switch is requested, false otherwise
 */
bool hil_increment_tick(void);

/**
 * @brief Process HIL UART data and trigger tick updates
 * 
 * This function should be called from the UART receive interrupt
 * handler. It validates the frame and triggers the tick update.
 * 
 * @param data Pointer to the received UART data
 * @param size Size of the received data
 * @return Status of the processing (W_SUCCESS, W_INVALID_PARAM)
 */
w_status_t hil_process_uart_data(const uint8_t *data, uint16_t size);

#endif /* HIL_H */ 