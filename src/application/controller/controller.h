#ifndef CONTROLLER_H_
#define CONTROLLER_H_
#include "FreeRTOS.h"
#include "application/controller/controller_algorithm.h"
#include "third_party/rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

/* Enums/Types */

typedef union {
    float roll_state_arr[NEW_ROLL_STATE_NUM];
    struct {
        float roll_angle;
        float roll_rate;
        // float canard_angle;
    };
} roll_state_t;

// input from state estimation module
typedef struct {
    // Timestamp in ms
    uint32_t timestamp;
    // Roll state
    roll_state_t roll_state;
    // Scheduling variables (flight condition)
    float canard_coeff;
    float pressure_dynamic;
} controller_input_t;

// Output of controller: latest commanded canard angle
typedef struct {
    float commanded_angle; // radians
    uint32_t timestamp; // ms
} controller_output_t;

// main controller state using in task
typedef struct {
    controller_input_t current_state;
    bool controller_active;
    uint32_t last_ms;
    uint32_t can_send_errors;
    uint32_t data_miss_counter;
} controller_t;

/**
 * Initialize controller module
 * Must be called before RTOS scheduler starts
 * @return W_SUCCESS if initialization successful
 */
w_status_t controller_init(void);

/**
 * Update controller with new state data - called by state estimation module
 * @param new_state Latest state estimate from state estimation
 * @return W_FAILURE if validation/queueing fails
 */
w_status_t controller_update_inputs(controller_input_t *new_state);

/**
 * Get most recent control output - called by state estimation module
 * @param output Pointer to store output -> type defined in state_estimation.h
 * @return W_FAILURE if no output available
 */
w_status_t controller_get_latest_output(controller_output_t *output);

/**
 * Controller task function for RTOS
 */
void controller_task(void *argument);

#endif // CONTROLLER_H_