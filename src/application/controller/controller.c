#include "application/controller/controller.h"

QueueHandle_t internal_state_queue;
QueueHandle_t output_queue;

TickType_t timeout = pdMS_TO_TICKS(5);

// internal: flight_condition_t
typedef struct {
    float dynamic_pressure;
    float canard_coeff_lift;
} flight_condition_t;

// initialize structs: placeholder values
static flight_condition_t flight_condition = {0};
static controller_t controller_state = {0};
static controller_input_t controller_input = {0};
static controller_output_t controller_output = {0};

/*
Send `canard_angle`, the desired canard angle (radians) to CAN
*/
static w_status_t controller_send_can(float canard_angle) {
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
    w_status_t init_status = W_SUCCESS;

    // Create internal state queue (length = 1)
    internal_state_queue =
        xQueueCreate(1, sizeof(controller_input_t)); // estimator function to controller task
    output_queue = xQueueCreate(1, sizeof(controller_output_t));

    // check queue creation
    if (internal_state_queue == NULL || output_queue == NULL) {
        init_status = W_FAILURE;
        log_text("controller", "queue creation failed");
    }

    // initialize structs: non-placeholder values

    // gain table initialization

    // Log initialization status
    if (init_status == W_SUCCESS) {
        log_text("controller", "initialization successful");

    } else {
        log_text("controller", "initialization failed");
    }

    // return w_status_t state
    return init_status;
}

/**
 * Update controller with new state data - called by state estimation module
 * @param new_state Latest state estimate from state estimation
 * @return W_FAILURE if validation/queueing fails
 */
w_status_t controller_update_inputs(controller_input_t *new_state) {
    if (xQueueOverwrite(internal_state_queue, new_state) == pdPASS) { // overwrite internal queue
        log_text("controller", "latest output received, altitude %f", new_state->altitude);
        return W_SUCCESS;
    }
    log_text("controller", "state overwritting fails");
    return W_FAILURE;
}

/**
 * Get most recent control output (cammanded angle and timestamp) - called by
 * state estimation module
 * @param output Pointer to store output -> type defined in controller
 * @return W_FAILURE if no output available
 */
w_status_t controller_get_latest_output(controller_output_t *output) {
    // return cammand angle + send timestamp

    if (xQueuePeek(output_queue, output, 0) == pdPASS) {
        log_text("controller", "latest output returned");
        return W_SUCCESS;
    }
    log_text("controller", "no output available");
    return W_FAILURE;
}

/**
 * Controller task function for RTOS
 */
void controller_task(void *argument) {
    // get current flight phase
    flight_phase_state_t current_phase = STATE_INIT;

    while (true) {
        // log phase transitions, specifics logged in flight phase
        if (current_phase != flight_phase_get_state()) {
            log_text("controller", "flight phase changed");
        }

#if current_phase == STATE_ACT_ALLOWED || current_phase == STATE_COAST // if is proper state

        // wait for new state data (5ms timeout)
        controller_input_t new_state_msg;
        if (xQueueReceive(internal_state_queue, &new_state_msg, timeout) == pdPASS) {
            controller_state.current_state = new_state_msg;
            // validate data

            // log data received
            log_text("controller", "new state data received for controller computations");

        } else {
            log_text("controller", "no new state data received for controller computations");
            controller_state.data_miss_counter++;

            // if number of data misses exceed threshold, transition to safe mode
        }

        // calculate gains based on current conditions

        // compute control output and overwrites output queue

        // send command visa CAN

        // log status/errors

        // provide feedback to estimator

#else
        vTaskDelay(pdMS_TO_TICKS(1)); // delay 1 ms
#endif
    }
}