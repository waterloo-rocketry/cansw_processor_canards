#include "application/init/init.h"
#include "application/can_handler/can_handler.h"
#include "application/flight_phase/flight_phase.h"
#include "application/health_checks/health_checks.h"
#include "application/imu_handler/imu_handler.h"
#include "drivers/gpio/gpio.h"
#include "drivers/i2c/i2c.h"
#include "drivers/uart/uart.h"
#include "main.h"

// Add these includes for hardware handles
#include "fdcan.h" // For hfdcan1
#include "i2c.h" // For hi2c2, hi2c4
#include "usart.h" // For huart4, huart8

// Initialize a function with retry logic
w_status_t init_with_retry(w_status_t (*init_fn)(void)) {
    w_status_t status;
    uint32_t retry_count = 0;

    while (retry_count < MAX_INIT_RETRIES) {
        status = init_fn();

        if (status == W_SUCCESS) {
            return W_SUCCESS;
        }

        retry_count++;
        if (retry_count < MAX_INIT_RETRIES) {
            HAL_Delay(INIT_RETRY_DELAY_MS);
        }
    }

    return W_FAILURE;
}

// Initialize a function with retry logic and parameter
w_status_t init_with_retry_param(w_status_t (*init_fn)(void *), void *param) {
    w_status_t status;
    uint32_t retry_count = 0;

    while (retry_count < MAX_INIT_RETRIES) {
        status = init_fn(param);

        if (status == W_SUCCESS) {
            return W_SUCCESS;
        }

        retry_count++;
        if (retry_count < MAX_INIT_RETRIES) {
            HAL_Delay(INIT_RETRY_DELAY_MS);
        }
    }

    return W_FAILURE;
}

// Main initialization function
w_status_t system_init(void) {
    w_status_t status = W_SUCCESS;

    // Initialize hardware peripherals
    status |= gpio_init();
    status |= i2c_init(I2C_BUS_2, &hi2c2, 0);
    status |= i2c_init(I2C_BUS_4, &hi2c4, 0);
    status |= uart_init(UART_DEBUG_SERIAL, &huart4, 0);
    status |= uart_init(UART_DEBUG_SERIAL, &huart8, 0);

    // Initialize application modules with retry logic
    status |= init_with_retry(flight_phase_init);
    status |= init_with_retry(imu_handler_init);
    status |= init_with_retry_param((w_status_t(*)(void *))can_handler_init, &hfdcan1);

    if (status != W_SUCCESS) {
        return status;
    }

    // Create FreeRTOS tasks
    BaseType_t task_status = pdTRUE;

    task_status &=
        xTaskCreate(flight_phase_task, "flightphase", 512, NULL, 1, &flight_phase_task_handle);
    task_status &=
        xTaskCreate(imu_handler_task, "imuHandler", 128 * 4, NULL, 3, &imu_handler_task_handle);

    if (task_status != pdTRUE) {
        return W_FAILURE;
    }

    return W_SUCCESS;
}