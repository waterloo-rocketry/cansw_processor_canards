/**
 * @file hil.c
 * @brief Implementation of Hardware-in-the-Loop tick manipulation
 */

#include "application/hil/hil.h"
#include "FreeRTOS.h"
#include "task.h"
#include "core_cm7.h"

// External declaration for the function in the FreeRTOS kernel
extern BaseType_t xTaskIncrementTick(void);

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