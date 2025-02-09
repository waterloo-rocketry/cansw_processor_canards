#include "application/controller/controller.h"

QueueHandle_t input_queue;
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

    // Create internal state queue and input queue (length = 1)
    input_queue = xQueueCreate(1, sizeof(controller_input_t)); // updated by estimator
    internal_state_queue =
        xQueueCreate(1, sizeof(controller_input_t)); // estimator function to controller task

    // check queue creation
    if (internal_state_queue == NULL || input_queue == NULL) {
        init_status = W_FAILURE;
        logInfo("controller", "queue creation failed");
    }

    // initialize structs: non-placeholder values

    // gain table initialization

    // Log initialization status
    if (init_status == W_SUCCESS) {
        logInfo("controller", "initialization successful");

    } else {
        logInfo("controller", "initialization failed");
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
    if (xQueueReceive(input_queue, new_state, 0) == pdPASS) {
        controller_state.current_state = *new_state;
        logInfo(
            "controller",
            "controller inputs updated by estimator, current altitude %f",
            controller_state.current_state.altitude
        );
        if (xQueueSend(internal_state_queue, new_state, 0) == pdPASS) {
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
    output_queue = xQueueCreate(1, sizeof(controller_output_t));
    if (xQueueSend(output_queue, output, 0) == pdPASS) {
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

    while (true) {
        // log phase transitions, specifics logged in flight phase
        if (current_phase != flight_phase_get_state()) {
            logInfo("controller", "flight phase changed");
        }

#if current_phase == STATE_ACT_ALLOWED || current_phase == STATE_COAST // if is proper state

        // wait for new state data (5ms timeout)
        controller_input_t new_state_msg;
        if (xQueueReceive(internal_state_queue, &new_state_msg, timeout) == pdPASS) {
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

#else
        vTaskDelay(pdMS_TO_TICKS(1)); // delay 1 ms
#endif
    }
}