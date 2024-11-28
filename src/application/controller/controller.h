#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "FreeRTOS.h"
#include "state_estimation.h"
#include <stdbool.h>

/* Enums/Types */
typedef struct {
    float x;
    float y;
    float z;
} vector3_t;

// main controller state
typedef struct {
    vector3_t attitude; // Current attitude vector
    vector3_t rates; // Current angular rates
    vector3_t velocity; // Current velocity vector
    float altitude; // Current altitude
    float timestamp; // Timestamp in ms
} controller_state_t;

// Output of controller to be sent to servo via CAN
typedef struct {
    float commanded_angle; // Desired angle
    float actual_angle; // Current angle
    uint32_t timestamp; // Timestamp
    uint32_t servo_id; // Servo identifier
} controller_can_output_t;

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