#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "FreeRTOS.h"
#include "state_estimation.h"
#include <stdbool.h>
#include "flight_phase.h"
#include "queue.h"
#include <math.h>

#include "state_estimation.h"

/* Enums/Types */
typedef struct {
    float x;
    float y;
    float z;
} vector3_t;

// quaterunions 
typedef struct {
    float w;
    float x;
    float y;
    float z;
} quaternion_t;

// input from state estimation module
typedef struct {
    quaternion_t attitude; // Current attitude vector
    vector3_t rates; // Current angular rates
    vector3_t velocity; // Current velocity vector
    float altitude; // Current altitude
    float timestamp; // Timestamp in ms
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
bool Controller_Init(void);

/**
 * Update controller with new state data - called by state estimation module
 * @param new_state Latest state estimate from state estimation
 * @return false if validation/queueing fails
 */
bool Controller_Update_Inputs(controller_state_t *new_state);

/**
 * Get most recent control output - called by state estimation module
 * @param output Pointer to store output -> type defined in state_estimation.h
 * @return false if no output available
 */
bool Controller_Get_Latest_Output(control_output_SE_t *output);

/**
 * Controller task function for RTOS
 */
void Controller_Task(void *argument);

#endif