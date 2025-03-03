#include "application/controller/controller.h"
#include "arm_const_structs_f16.h"
#include "arm_math.h"
#include "arm_math_f16.h"
#include <math.h>
#include <stdio.h>

#define GAIN_NUM 4
#define TIME_START 5
#define FEEDBACK_GAIN_NUM 3 // size of feedback gain vector

QueueHandle_t internal_state_queue;
QueueHandle_t output_queue;

TickType_t timeout = pdMS_TO_TICKS(5);

typedef struct {
    float gain_arr[GAIN_NUM];
    arm_bilinear_interp_instance_f32 gain_instance;

    float gain_k[3];
    float gain_k_pre;
    float reference_signal;
} gain_table_entry_t;

// placeholder: suppose this is imported daa
// for normalizing inputs
float canard_coeff_scale;
float canard_coeff_offset;
float pressure_dynamic_scale;
float pressure_dynamic_offset;
// for building each interpolation instance
const uint16_t numRow;
const uint16_t numCol;
static float gain_table[GAIN_NUM][10000];

static controller_t controller_state = {0};
static controller_input_t controller_input = {0};
static controller_output_t controller_output = {0};
static gain_table_entry_t gain_entry = {0};

static const float max_commanded_angle = 20 * 180.0 / M_PI;

/*
    TODO Send `canard_angle`, the desired canard angle (radians) to CAN
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
    xQueueOverwrite(internal_state_queue, new_state); // overwrite internal queue
    log_text("controller", "latest output received, timestamp %d", new_state->timestamp);
    return W_SUCCESS;
}

/**
 * Get most recent control output (cammanded angle and timestamp) - called by
 * state estimation module
 * @param output Pointer to store output -> type defined in controller
 * @return W_FAILURE if no output available
 */
w_status_t controller_get_latest_output(controller_output_t *output) {
    if (xQueuePeek(output_queue, output, timeout) == pdPASS) {
        log_text("controller", "latest output returned");
        return W_SUCCESS;
    }
    log_text("controller", "timeout: no output available");
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
            current_phase = flight_phase_get_state();
            log_text("controller", "flight phase changed");
        }

        if (current_phase != STATE_ACT_ALLOWED &&
            current_phase != STATE_COAST) { // if not in proper state
            vTaskDelay(pdMS_TO_TICKS(1));
        } else {
            // wait for new state data (5ms timeout)
            controller_input_t new_state_msg;
            if (xQueueReceive(internal_state_queue, &new_state_msg, timeout) == pdPASS) {
                controller_state.current_state = new_state_msg;
                // TODO validate data

                // log data received
                log_text("controller", "new state data received for controller computations");

            } else {
                log_text("controller", "no new state data received for controller computations");
                controller_state.data_miss_counter++;

                // TODO if number of data misses exceed threshold, notify health check module
            }

            // calculate gains based on current conditions
            uint32_t time = controller_state.current_state.timestamp - TIME_START;
            if (time > 5) {
                if (time < 12) {
                    gain_entry.reference_signal = 1;
                } else if (time < 19) {
                    gain_entry.reference_signal = -1;
                } else if (time > 26) {
                    gain_entry.reference_signal = 0;
                }
            }

            // TODO Ks: load table, interpolate
            gain_entry.gain_instance.numCols = numCol;
            gain_entry.gain_instance.numRows = numRow;

            for (int i = 0; i < GAIN_NUM; i++) {
                // set instance
                gain_entry.gain_instance.pData = gain_table[i];

                // normalize inputs based on scaling and offset floats
                controller_state.current_state.canard_coeff =
                    controller_state.current_state.canard_coeff * canard_coeff_scale +
                    canard_coeff_offset;
                controller_state.current_state.pressure_dynamic =
                    controller_state.current_state.pressure_dynamic * pressure_dynamic_scale +
                    pressure_dynamic_offset;

                gain_entry.gain_arr[i] = arm_bilinear_interp_f32(
                    &gain_entry.gain_instance,
                    controller_state.current_state.canard_coeff,
                    controller_state.current_state.pressure_dynamic

                );

                // deconstruct into K and K_pre
                if (i < FEEDBACK_GAIN_NUM) {
                    gain_entry.gain_k[i] = gain_entry.gain_arr[i];
                } else {
                    gain_entry.gain_k_pre = gain_entry.gain_arr[i];
                }
            }

            // TODO compute control output and overwrites output queue
            float roll_state[3] = {
                controller_state.current_state.roll_angle,
                controller_state.current_state.roll_rate,
                controller_state.current_state.canard_angle

            };

            arm_dot_prod_f32(
                gain_entry.gain_k, roll_state, FEEDBACK_GAIN_NUM, &controller_output.commanded_angle
            );
            controller_output.commanded_angle +=
                gain_entry.reference_signal * gain_entry.gain_k_pre;

            // limit output to allowable angle
            controller_output.commanded_angle = fmin(
                fmax(controller_output.commanded_angle, -max_commanded_angle), max_commanded_angle
            );
            xQueueOverwrite(output_queue, &controller_output);

            // send command visa CAN + log status/errors
            if (controller_send_can(controller_output.commanded_angle) == W_SUCCESS) {
                log_text("controller", "commanded angle sent via CAN");

            } else {
                log_text("controller", "commanded angle failed to send via CAN");
            }
        }
    }
}

