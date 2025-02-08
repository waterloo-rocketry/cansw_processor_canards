#include "application/controller/controller.h"

QueueHandle_t internalStateQueue;
QueueHandle_t outputQueue;

// internal: flight_condition_t
typedef struct {
    float dynamic_pressure;
    float canard_coeff_lift;
} flight_condition_t;

flight_condition_t flight_condition;
controller_t controller;
controller_output_t estimator_controller_input;

/*
Send `canard_angle`, the desired canard angle (radians) to CAN
*/
w_status_t controller_send_san(float canard_angle) {
    // Build the CAN msg using [canard-specific canlib function to be defined
    // later]. Send this to can handler module’s tx
}

/**
 * Initialize controller module
 * Must be called before RTOS scheduler starts
 * @return W_SUCCESS if initialization successful
 */
w_status_t controller_init(void) {
    bool init_status = true;

    // Create internal state queue (length = 1)
    internalStateQueue = xQueueCreate(1, sizeof(controller_input_t));

    // check queue creation
    if (internalStateQueue == NULL) {
        init_status = false;
    }

    // gain table initialization

    // Log initialization status
    if (init_status) {
        logInfo("controller", "initialization successful");
    } else {
        logInfo("controller", "initialization failed");
        return W_FAILURE;
    }

    // return w_status_t state
    return W_SUCCESS;
}

/**
 * Update controller with new state data - called by state estimation module
 * @param new_state Latest state estimate from state estimation
 * @return W_FAILURE if validation/queueing fails
 */
w_status_t controller_update_inputs(controller_input_t *new_state) {
    controller_input_t new_state_msg;

    if (xQueueReceive(internalStateQueue, &new_state_msg, 0) == pdPASS) {
        controller.current_state = new_state_msg;
        logInfo(
            "controller",
            "controller inputs updated, current altitude %f",
            controller.current_state.altitude
        );
        return W_SUCCESS;
    }
    return W_FAILURE;
}

/**
 * Get most recent control output (cammanded angle and timestamp) - called by
 * state estimation module
 * @param output Pointer to store output -> type defined in controller
 * @return W_FAILURE if no output available
 */
w_status_t controller_get_latest_output(controller_output_t *output) {
    // calculate cammand angle + send timestamp

    // send outputs to estimator
    xQueueCreate(outputQueue, sizeof(controller_output_t));
    if (xQueueSend(outputQueue, output, 0) == pdPASS) {
        logInfo("controller", "controller output sent, command angle %f", output->commanded_angle);
        return W_SUCCESS;
    }
    logInfo(
        "controller", "controller output didn't send, command angle %f", output->commanded_angle
    );
    return W_FAILURE;
}

/**
 * Controller task function for RTOS
 */
void controller_task(void *argument) {}
