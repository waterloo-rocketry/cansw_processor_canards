#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "FreeRTOS.h"
#include "application/controller/gain_table.h"
#include "third_party/rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

#define FEEDBACK_GAIN_NUM (GAIN_NUM - 1) // subtract 1 for the pre-gain
#define ROLL_STATE_NUM (FEEDBACK_GAIN_NUM)
#define MIN_COOR_BOUND 0

/* Enums/Types */
typedef union {
    double roll_state_arr[ROLL_STATE_NUM];
    struct {
        double roll_angle;
        double roll_rate;
        double canard_angle;
    };
} roll_state_t;

// input from state estimation module
typedef struct {
    // Roll state
    roll_state_t roll_state;
    // Scheduling variables (flight condition)
    double pressure_dynamic;
    double canard_coeff;
} controller_input_t;

// Output of controller: latest commanded canard angle
typedef struct {
    double commanded_angle; // radians
    uint32_t timestamp; // ms
} controller_output_t;

// main controller state using in task
typedef struct {
    uint32_t last_ms; // currently unused...
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