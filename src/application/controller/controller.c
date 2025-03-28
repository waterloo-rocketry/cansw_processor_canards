#include "application/controller/controller.h"

#include "application/flight_phase/flight_phase.h"
#include "application/logger/log.h"
#include "queue.h"

static QueueHandle_t internal_state_queue;
static QueueHandle_t output_queue;

#define CONTROLLER_CYCLE_TIMEOUT_MS 5

static controller_t controller_state = {0};
static controller_input_t controller_input __attribute__((unused)) = {0};
static controller_output_t controller_output = {0};
static controller_gain_t controller_gain __attribute__((unused)) = {0};

/*
    TODO Send `canard_angle`, the desired canard angle (radians) to CAN
*/
static w_status_t controller_send_can(float canard_angle) {
    // Build the CAN msg using [canard-specific canlib function to be defined later].

    // Send this to can handler module’s tx
    (void)canard_angle;
    return W_SUCCESS;
}

/**
 * Initialize controller module
 * Must be called before RTOS scheduler starts
 * @return W_SUCCESS if initialization successful
 */
w_status_t controller_init(void) {
    // Create internal state queue (length = 1)
    internal_state_queue =
        xQueueCreate(1, sizeof(controller_input_t)); // estimator function to controller task
    output_queue = xQueueCreate(1, sizeof(controller_output_t));

    // check queue creation
    if (NULL == internal_state_queue || NULL == output_queue) {
        log_text("controller", "queue creation failed");
        return W_FAILURE;
    }

    // avoid controller/estimator deadlock
    xQueueOverwrite(output_queue, &controller_output);

    // TODO gain instance init

    // return w_status_t state
    log_text("controller", "initialization successful");
    return W_SUCCESS;
}

/**
 * Update controller with new state data - called by state estimation module
 * @param new_state Latest state estimate from state estimation
 * @return W_FAILURE if validation/queueing fails
 */
w_status_t controller_update_inputs(controller_input_t *new_state) {
    xQueueOverwrite(internal_state_queue, new_state); // overwrite internal queue

    return W_SUCCESS;
}

/**
 * Get most recent control output (cammanded angle and timestamp) - called by
 * state estimation module
 * @param output Pointer to store output -> type defined in controller
 * @return W_FAILURE if no output available
 */
w_status_t controller_get_latest_output(controller_output_t *output) {
    if (pdPASS == xQueuePeek(output_queue, output, 0)) {
        return W_SUCCESS;
    }

    return W_FAILURE;
}

/**
 * Controller task function for RTOS
 */
void controller_task(void *argument) {
    (void)argument;

    while (true) {
        // no phase change track
        flight_phase_state_t current_phase = flight_phase_get_state();
        if (current_phase != STATE_ACT_ALLOWED) { // if not in proper state
            vTaskDelay(pdMS_TO_TICKS(1));
        } else {
            // wait for new state data (5ms timeout)
            controller_input_t new_state_msg;
            if (pdPASS ==
                xQueueReceive(
                    internal_state_queue, &new_state_msg, pdMS_TO_TICKS(CONTROLLER_CYCLE_TIMEOUT_MS)
                )) {
                controller_state.current_state = new_state_msg;
                // TODO validate data

                // no roll program for test flight

            } else {
                controller_state.data_miss_counter++;

                // TODO if number of data misses exceed threshold, notify health check module
            }

            // TODO controller calc: interpolate

            // update output queue
            xQueueOverwrite(output_queue, &controller_output);

            // send command visa CAN + log status/errors
            if (W_SUCCESS != controller_send_can(controller_output.commanded_angle)) {
                log_text("controller", "commanded angle failed to send via CAN");
            }
        }
    }
}

