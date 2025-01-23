#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "FreeRTOS.h"
#include "state_estimation.h"
#include <stdbool.h>
#include "flight_phase.h"
#include "queue.h"

#include "math.h"



/* Enums/Types */


// input from state estimation module
typedef struct {
    quaternion_t attitude; // Current attitude vector
    vector3d_t rates; // Current angular rates
    vector3d_t velocity; // Current velocity vector
    float altitude; // Current altitude
    float timestamp; // Timestamp in ms
    float canard_coeff_CL; // Canard coefficient
    float canard_angle_delta; // Canard angle
} controller_state_t;


// main controller state using in task
typedef struct {
    controller_state_t current_state;
    flight_phase_t last_error; 
    bool controller_active;
    uint32_t last_ms;
    uint32_t can_send_errors;
    uint32_t data_miss_counter;
} controller_t;

// flight_conditions_t


/* defined in state est.
typedef struct{
    float command_angle;
    uint32_t timestamp;
} control_output_SE_t; */



extern QueueHandle_t internalStateQueue;


/**
 * Initialize controller module
 * Must be called before RTOS scheduler starts
 * @return true if initialization successful
 */
bool controller_init(void);

/**
 * Update controller with new state data - called by state estimation module
 * @param new_state Latest state estimate from state estimation
 * @return false if validation/queueing fails
 */
bool controller_update_inputs(controller_state_t *new_state);

/**
 * Get most recent control output - called by state estimation module
 * @param output Pointer to store output -> type defined in state_estimation.h
 * @return false if no output available
 */
bool controller_get_latest_output(control_output_SE_t *output);

/**
 * Controller task function for RTOS
 */
void controller_task(void *argument);

#endif