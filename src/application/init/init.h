#ifndef INIT_H
#define INIT_H

#include "FreeRTOS.h"
#include "main.h" // For HAL handle declarations
#include "rocketlib/include/common.h"
#include "task.h"

// Maximum number of initialization retries before giving up
#define MAX_INIT_RETRIES 1

// Delay between initialization retries in milliseconds
#define INIT_RETRY_DELAY_MS 1000

// Task handles - defined in init.c
extern TaskHandle_t log_task_handle;
extern TaskHandle_t estimator_task_handle;
extern TaskHandle_t can_handler_handle_tx;
extern TaskHandle_t can_handler_handle_rx;
extern TaskHandle_t health_checks_task_handle;
extern TaskHandle_t controller_task_handle;
extern TaskHandle_t flight_phase_task_handle;
extern TaskHandle_t imu_handler_task_handle;
extern TaskHandle_t movella_task_handle;

// Task priorities
extern const uint32_t flight_phase_task_priority;
extern const uint32_t log_task_priority;
extern const uint32_t estimator_task_priority;
extern const uint32_t controller_task_priority;
extern const uint32_t can_handler_rx_priority;
extern const uint32_t can_handler_tx_priority;
extern const uint32_t health_checks_task_priority;
extern const uint32_t imu_handler_task_priority;
extern const uint32_t movella_task_priority;

// Helper function to retry an initialization function
w_status_t init_with_retry(w_status_t (*init_fn)(void));
w_status_t init_with_retry_param(w_status_t (*init_fn)(void *), void *param);

// Main initialization function that sets up everything
w_status_t system_init(void);

#endif // INIT_H