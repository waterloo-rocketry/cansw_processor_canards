#include "application/controller/controller.h"
#include "application/can_handler/can_handler.h"
#include "application/flight_phase/flight_phase.h"
#include "application/logger/log.h"
#include "drivers/timer/timer.h"
#include "queue.h"
#include "third_party/canlib/message/msg_actuator.h"
#include <math.h>

static QueueHandle_t internal_state_queue;
static QueueHandle_t output_queue;

#define CONTROLLER_CYCLE_TIMEOUT_MS 5

static controller_t controller_state = {0};
static controller_output_t controller_output = {0};
static controller_gain_t controller_gain = {0};

// output related const
static const float max_commanded_angle = 20 / 180.0 * M_PI; // 20 degrees in radians
static const float reference_signal = 0.0f; // no roll program for test flight

// Send `canard_angle`, the desired canard angle (radians) to CAN
static w_status_t controller_send_can(float canard_angle) {
    // convert canard angle from radians to millidegrees
    int16_t canard_cmd_signed = (int16_t)(canard_angle / M_PI * 180.0 * 1000.0);
    uint16_t canard_cmd = (uint16_t)canard_cmd_signed;

    // get timestamp for can msg
    float time_ms;
    if (W_SUCCESS != timer_get_ms(&time_ms)) {
        time_ms = 0.0f;
    }
    uint32_t can_timestamp = (uint32_t)time_ms;

    // Build the CAN msg
    can_msg_t msg;
    build_actuator_analog_cmd_msg(
        PRIO_HIGHEST, can_timestamp, ACTUATOR_CANARD_ANGLE, canard_cmd, &msg
    );

    // Send this to can handler module’s tx
    return can_handler_transmit(&msg);
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
    xQueueOverwrite(output_queue, 0);

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
        if (STATE_ACT_ALLOWED != current_phase) { // if not in proper state
            vTaskDelay(pdMS_TO_TICKS(1));
        } else {
            // wait for new state data (5ms timeout)
            controller_input_t new_state_msg;
            if (pdPASS ==
                xQueueReceive(
                    internal_state_queue, &new_state_msg, pdMS_TO_TICKS(CONTROLLER_CYCLE_TIMEOUT_MS)
                )) {
                controller_state.current_state = new_state_msg;

                // no roll program for test flight

            } else {
                controller_state.data_miss_counter++;

                // TODO if number of data misses exceed threshold, notify health check module
            }

            // controller calc: interpolate
            if (W_SUCCESS != interpolate_gain(
                                 controller_state.current_state.pressure_dynamic,
                                 controller_state.current_state.canard_coeff,
                                 &controller_gain
                             )) {
                log_text("controller", "failed to interpolate controller gain");
            }

            // compute commanded angle
            float dot_prod = 0.0f;
            arm_dot_prod_f32(
                controller_gain.gain_k,
                controller_state.current_state.roll_state.roll_state_arr,
                FEEDBACK_GAIN_NUM,
                &dot_prod
            );
            controller_output.commanded_angle =
                dot_prod + controller_gain.gain_k_pre * reference_signal;

            controller_output.commanded_angle = fmin(
                fmax(controller_output.commanded_angle, -max_commanded_angle), max_commanded_angle
            );

            // update timestamp for controller output
            float current_timestamp_ms;
            if (W_SUCCESS != timer_get_ms(&current_timestamp_ms)) {
                current_timestamp_ms = 0.0f;
                log_text("controller", "failed to get timestamp for controller output");
            }
            controller_output.timestamp = (uint32_t)current_timestamp_ms;

            // update output queue
            xQueueOverwrite(output_queue, &controller_output);

            // send command visa CAN + log status/errors
            if (W_SUCCESS != controller_send_can(controller_output.commanded_angle)) {
                log_text("controller", "commanded angle failed to send via CAN");
            }
        }
    }
}

