#ifndef CONTROLLER_H_
#define CONTROLLER_H_


#include <stdbool.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "state_estimation.h"
#include "flight_phase.h"
#include "queue.h"
#include "math.h"
#include "common.h"



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
} controller_input_t;


// main controller state using in task
typedef struct {
    controller_input_t current_state;
    bool controller_active;
    uint32_t last_ms;
    uint32_t can_send_errors;
    uint32_t data_miss_counter;
} controller_t;

//flight_condition_t

//gain_table_entry_t





extern QueueHandle_t internalStateQueue; 


/**
 * Initialize controller module
 * Must be called before RTOS scheduler starts
 * @return true if initialization successful
 */
w_status_t controller_init(void);

/**
 * Update controller with new state data - called by state estimation module
 * @param new_state Latest state estimate from state estimation
 * @return false if validation/queueing fails
 */
w_status_t controller_update_inputs(controller_input_t *new_state);

/**
 * Get most recent control output - called by state estimation module
 * @param output Pointer to store output -> type defined in state_estimation.h
 * @return false if no output available
 */
w_status_t controller_get_latest_output(control_output_SE_t *output);

/**
 * Controller task function for RTOS
 */
void controller_task(void *argument);

/*
Send `canard_angle`, the desired canard angle (radians) to CAN
*/
w_status_t Controller_Send_Can(float canard_angle) 

#endif // CONTROLLER_H_