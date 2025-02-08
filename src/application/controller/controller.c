#include "application/controller/controller.h"

QueueHandle_t inputQueue;
QueueHandle_t internalStateQueue;
QueueHandle_t outputQueue;

TickType_t timeout = pdMS_TO_TICKS(5);

// internal: flight_condition_t
typedef struct {
    float dynamic_pressure;
    float canard_coeff_lift;
} flight_condition_t;

flight_condition_t flight_condition;
controller_t controller_state;
controller_input_t controller_input;
controller_output_t estimator_controller_input;

/*
Send `canard_angle`, the desired canard angle (radians) to CAN
*/
w_status_t controller_send_san(float canard_angle) {
    // Build the CAN msg using [canard-specific canlib function to be defined
    // later]. Send this to can handler module’s tx
    return W_SUCCESS;
}

/**
 * Initialize controller module
 * Must be called before RTOS scheduler starts
 * @return W_SUCCESS if initialization successful
 */
w_status_t controller_init(void) {
    bool init_status = true;

    // Create internal state queue and input queue (length = 1)
    inputQueue = xQueueCreate(1, sizeof(controller_input_t)); // updated by estimator
    internalStateQueue =
        xQueueCreate(1, sizeof(controller_input_t)); // estimator function to controller task

    // check queue creation
    if (internalStateQueue == NULL) {
        init_status = false;
    }

    // initialize structs: placeholder values
    flight_condition.canard_coeff_lift = 0;
    flight_condition.dynamic_pressure = 0;

    controller_state.current_state.altitude = 0;
    controller_state.current_state.attitude.element.w = 0;
    controller_state.current_state.attitude.element.x = 0;
    controller_state.current_state.attitude.element.y = 0;
    controller_state.current_state.attitude.element.z = 0;
    controller_state.current_state.rates.component.x = 0;
    controller_state.current_state.rates.component.y = 0;
    controller_state.current_state.rates.component.z = 0;
    controller_state.current_state.velocity.component.x = 0;
    controller_state.current_state.velocity.component.y = 0;
    controller_state.current_state.velocity.component.z = 0;
    controller_state.current_state.altitude = 0;
    controller_state.current_state.timestamp = 0;
    controller_state.current_state.canard_coeff_CL = 0;
    controller_state.current_state.canard_angle_delta = 0;

    controller_state.controller_active = false;
    controller_state.data_miss_counter = 0;
    controller_state.last_ms = 0;
    controller_state.can_send_errors = 0;

    controller_input.attitude.element.w = 0;
    controller_input.attitude.element.x = 0;
    controller_input.attitude.element.y = 0;
    controller_input.attitude.element.z = 0;
    controller_input.rates.component.x = 0;
    controller_input.rates.component.y = 0;
    controller_input.rates.component.z = 0;
    controller_input.velocity.component.x = 0;
    controller_input.velocity.component.y = 0;
    controller_input.velocity.component.z = 0;
    controller_input.altitude = 0;
    controller_input.timestamp = 0;
    controller_input.canard_coeff_CL = 0;
    controller_input.canard_angle_delta = 0;

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
    if (xQueueReceive(inputQueue, new_state, 0) == pdPASS) {
        controller_state.current_state = *new_state;
        logInfo(
            "controller",
            "controller inputs updated by estimator, current altitude %f",
            controller_state.current_state.altitude
        );
        if (xQueueSend(internalStateQueue, new_state, 0) == pdPASS) {
            logInfo("controller", "controller inputs sent to internal state queue");
            return W_SUCCESS;
        }
        // internal queue error caught and logged in task
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
void controller_task(void *argument) {
    // get current flight phase
    flight_phase_state_t current_phase = STATE_INIT;

    // log phase transitions, specifics logged in flight phase
    if (current_phase != flight_phase_get_state()) {
        logInfo("controller", "flight phase changed");
    }

    if (current_phase != STATE_ACT_ALLOWED || current_phase != STATE_COAST) { // if not proper state
        vTaskDelay(pdMS_TO_TICKS(1)); // delay 1 ms
    }

    // wait for new state data (5ms timeout)
    controller_input_t new_state_msg;
    if (xQueueReceive(internalStateQueue, &new_state_msg, timeout) == pdPASS) {
        // validate data

        // log data received
        logInfo("controller", "new state data received for controller computations");

    } else {
        logInfo("controller", "no new state data received for controller computations");
        controller_state.data_miss_counter++;

        // if number of data misses exceed threshold, transition to safe mode
    }

    // calculate gains based on current conditions

    // compute control output

    // send command visa CAN

    // log status/errors

    // provide feedback to estimator
}
