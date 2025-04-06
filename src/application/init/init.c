#include "application/init/init.h"
#include "application/can_handler/can_handler.h"
#include "application/controller/controller.h"
#include "application/estimator/estimator.h"
#include "application/flight_phase/flight_phase.h"
#include "application/health_checks/health_checks.h"
#include "application/imu_handler/imu_handler.h"
#include "application/logger/log.h"
#include "drivers/adc/adc.h"
#include "drivers/altimu-10/altimu-10.h"
#include "drivers/gpio/gpio.h"
#include "drivers/i2c/i2c.h"
#include "drivers/movella/movella.h"
#include "drivers/sd_card/sd_card.h"
#include "drivers/uart/uart.h"
#include "stm32h7xx_hal.h"
// Add these includes for hardware handles
#include "adc.h" // For hadc1
#include "fdcan.h" // For hfdcan1
#include "i2c.h" // For hi2c2, hi2c4
#include "usart.h" // For huart4, huart8

// Initialize task handles to NULL
TaskHandle_t log_task_handle = NULL;
TaskHandle_t estimator_task_handle = NULL;
TaskHandle_t can_handler_handle_tx = NULL;
TaskHandle_t can_handler_handle_rx = NULL;
TaskHandle_t health_checks_task_handle = NULL;
TaskHandle_t controller_task_handle = NULL;
TaskHandle_t flight_phase_task_handle = NULL;
TaskHandle_t imu_handler_task_handle = NULL;
TaskHandle_t movella_task_handle = NULL;

// Task priorities
// flight phase must have highest priority to preempt everything else
const uint32_t flight_phase_task_priority = configMAX_PRIORITIES - 1;
// TODO: replace with actual priorities once determined. for now just make all
// same priority
const uint32_t log_task_priority = configMAX_PRIORITIES - 5;
const uint32_t estimator_task_priority = configMAX_PRIORITIES - 5;
const uint32_t controller_task_priority = configMAX_PRIORITIES - 5;
const uint32_t can_handler_rx_priority = configMAX_PRIORITIES - 5;
const uint32_t can_handler_tx_priority = configMAX_PRIORITIES - 5;
const uint32_t health_checks_task_priority = configMAX_PRIORITIES - 5;
const uint32_t imu_handler_task_priority = configMAX_PRIORITIES - 5;
const uint32_t movella_task_priority = configMAX_PRIORITIES - 5;

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
    status |= uart_init(UART_DEBUG_SERIAL, &huart4, 100);
    status |= uart_init(UART_MOVELLA, &huart8, 100);
    status |= adc_init(&hadc1);
    status |= sd_card_init();

    // Initialize application modules with retry logic
    status |= log_init();
    status |= init_with_retry(altimu_init);
    status |= init_with_retry(movella_init);
    status |= init_with_retry(flight_phase_init);
    status |= init_with_retry(imu_handler_init);
    status |= init_with_retry_param((w_status_t(*)(void *))can_handler_init, &hfdcan1);
    status |= init_with_retry(controller_init);

    if (status != W_SUCCESS) {
        return status;
    }

    // Create FreeRTOS tasks
    BaseType_t task_status = pdTRUE;

    task_status &= xTaskCreate(
        flight_phase_task,
        "flight phase",
        512,
        NULL,
        flight_phase_task_priority,
        &flight_phase_task_handle
    );

    task_status &= xTaskCreate(
        imu_handler_task,
        "imu handler",
        512,
        NULL,
        imu_handler_task_priority,
        &imu_handler_task_handle
    );

    task_status &= xTaskCreate(
        can_handler_task_rx,
        "can handler rx",
        512,
        NULL,
        can_handler_rx_priority,
        &can_handler_handle_rx
    );

    task_status &= xTaskCreate(
        can_handler_task_tx,
        "can handler tx",
        512,
        NULL,
        can_handler_tx_priority,
        &can_handler_handle_tx
    );

    task_status &= xTaskCreate(
        movella_task, "movella", 2560, NULL, movella_task_priority, &movella_task_handle
    );

    task_status &= xTaskCreate(log_task, "logger", 2048, NULL, log_task_priority, &log_task_handle);

    task_status &= xTaskCreate(
        controller_task, "controller", 1024, NULL, controller_task_priority, &controller_task_handle
    );

    if (task_status != pdTRUE) {
        return W_FAILURE;
    }

    return W_SUCCESS;
}