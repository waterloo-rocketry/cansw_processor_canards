#include "controller.h"

/**
 * Initialize controller module
 * Must be called before RTOS scheduler starts
 * @return true if initialization successful
 */
bool Controller_Init(void){

}

/**
 * Update controller with new state data - called by state estimation module
 * @param new_state Latest state estimate from state estimation
 * @return false if validation/queueing fails
 */
bool Controller_Update_Inputs(controller_state_t *new_state){

}

/**
 * Get most recent control output - called by state estimation module
 * @param output Pointer to store output -> type defined in state_estimation.h
 * @return false if no output available
 */
bool Controller_Get_Latest_Output(control_output_SE_t *output){

}

/**
 * Controller task function for RTOS
 */
void Controller_Task(void *argument){
    //1. initialize controller

    //2. active control phase
    
}