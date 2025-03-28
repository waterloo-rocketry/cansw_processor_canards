/**
 * @file hil.h
 * @brief Hardware-in-the-Loop interface for tick manipulation
 * 
 * This module provides functionality to manipulate FreeRTOS ticks
 * for Hardware-in-the-Loop (HIL) testing.
 */

#ifndef HIL_H
#define HIL_H

#include <stdbool.h>

/**
 * @brief Manually advance the RTOS tick counter
 * 
 * This function manually increments the FreeRTOS tick counter by one,
 * bypassing the normal SysTick interrupt-based tick mechanism.
 * 
 * @return true if a context switch is requested, false otherwise
 */
bool hil_increment_tick(void);

#endif /* HIL_H */ 