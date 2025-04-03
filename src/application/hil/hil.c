/**
 * @file hil.c
 * @brief Implementation of Hardware-in-the-Loop interface
 */

#include "application/hil/hil.h"
#include "FreeRTOS.h"
#include "application/hil/simulator.h"
#include "core_cm7.h"
#include "task.h"

// External declaration for the function in the FreeRTOS kernel
extern BaseType_t xTaskIncrementTick(void);

// HIL UART protocol constants
#define HIL_UART_HEADER_CHAR '?'
#define HIL_UART_FOOTER_CHAR '\n'
// Calculate minimum size: Header (1) + Minimum payload (e.g., 1 float = 4) + Footer (1) = 6
#define HIL_UART_MIN_FRAME_SIZE 6

/**
 * @brief Manually advance the RTOS tick counter by one
 *
 * This function manually increments the FreeRTOS tick counter by one,
 * bypassing the normal SysTick interrupt-based tick mechanism.
 *
 * @return true if a context switch is requested, false otherwise
 */
bool hil_increment_tick(void) {
    BaseType_t xSwitchRequired = pdFALSE;

    // Disable interrupts before manipulating the tick counter
    portDISABLE_INTERRUPTS();

    // Increment the RTOS tick counter
    xSwitchRequired = xTaskIncrementTick();

    // If a context switch is required, pend the PendSV interrupt
    if (xSwitchRequired != pdFALSE) {
        // This is what would normally happen in the SysTick handler
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    }

    // Re-enable interrupts
    portENABLE_INTERRUPTS();

    return (xSwitchRequired != pdFALSE);
}

/**
 * @brief Process HIL UART data and trigger tick updates
 *
 * This function should be called from the UART receive interrupt
 * handler. It validates the frame, processes simulator data,
 * and triggers the tick update.
 *
 * @param data Pointer to the received UART data (including header/footer)
 * @param size Size of the received data (including header/footer)
 * @return Status of the processing (W_SUCCESS, W_INVALID_PARAM)
 */
w_status_t hil_process_uart_data(const uint8_t *data, uint16_t size) {
    if (data == NULL) {
        return W_INVALID_PARAM;
    }

    // Validate frame size
    if (size < HIL_UART_MIN_FRAME_SIZE) {
        return W_INVALID_PARAM;
    }

    // Check header and footer
    if (data[0] != HIL_UART_HEADER_CHAR || data[size - 1] != HIL_UART_FOOTER_CHAR) {
        return W_INVALID_PARAM;
    }

    // At this point, we have a valid HIL frame

    // Process the simulator data (payload is between header and footer)
    w_status_t sim_status = simulator_process_data(data + 1, size - 2);

    // Increment the RTOS tick counter regardless of simulator processing result
    // Each valid UART frame represents one tick
    hil_increment_tick();

    return W_SUCCESS; // Return overall success if frame was valid
}