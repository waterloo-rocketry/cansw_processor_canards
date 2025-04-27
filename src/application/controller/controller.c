#include "application/controller/controller.h"
#include "application/can_handler/can_handler.h"
#include "application/controller/controller_algorithm.h"
#include "application/flight_phase/flight_phase.h"
#include "application/logger/log.h"
#include "drivers/timer/timer.h"
#include "queue.h"
#include "third_party/canlib/message/msg_actuator.h"

static QueueHandle_t internal_state_queue;
static QueueHandle_t output_queue;

#define CONTROLLER_CYCLE_TIMEOUT_MS 5
#define ERROR_TIMEOUT_MS 10
#define RECOVERY_TIMEOUT_MS 1000
#define STATE_ELSE_TIMEOUT 1

static controller_t controller_state = {0};
static controller_output_t controller_output = {0};
static controller_gain_t controller_gain = {0};

// Send `canard_angle`, the desired canard angle (radians) to CAN
static w_status_t controller_send_can(float canard_angle) {
    // convert canard angle from radians to millidegrees
    int16_t canard_cmd_signed = (int16_t)(canard_angle / M_PI * 180.0 * 1000.0);
    uint16_t canard_cmd_shifted = canard_cmd_signed + 32768;

    // get timestamp for can msg
    float time_ms;
    if (W_SUCCESS != timer_get_ms(&time_ms)) {
        time_ms = 0.0f;
    }
    uint32_t can_timestamp = (uint32_t)time_ms;

    // Build the CAN msg
    can_msg_t msg;
    if (!build_actuator_analog_cmd_msg(
            PRIO_HIGHEST, can_timestamp, ACTUATOR_CANARD_ANGLE, canard_cmd_shifted, &msg
        )) {
        log_text(ERROR_TIMEOUT_MS, "controller", "actuator message build failure");
    }

    // Send this to can handler module's tx
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
        log_text(ERROR_TIMEOUT_MS, "controller", "queue creation failed");
        return W_FAILURE;
    }

    // avoid controller/estimator deadlock
    xQueueOverwrite(output_queue, &commanded_angle_zero);

    // return w_status_t state
    log_text(ERROR_TIMEOUT_MS, "controller", "initialization successful");
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
    float current_timestamp_ms = 0.0f;
    log_data_container_t data_container = {0};
    TickType_t last_wake_time;
    last_wake_time = xTaskGetTickCount();

    while (true) {
        // no phase change track
        flight_phase_state_t current_phase = flight_phase_get_state();
        switch (current_phase) {
            case STATE_RECOVERY:

                // update timestamp for controller output
                if (W_SUCCESS != timer_get_ms(&current_timestamp_ms)) {
                    current_timestamp_ms = 0.0f;
                    log_text(
                        ERROR_TIMEOUT_MS,
                        "controller",
                        "failed to get timestamp for controller output"
                    );
                }
                controller_output.timestamp = (uint32_t)current_timestamp_ms;

                // actively cmd 0 deg at 1hz
                controller_output.commanded_angle = commanded_angle_zero;

                // update output queue
                xQueueOverwrite(output_queue, &controller_output);

                // send command visa CAN + log status/errors at 1hz
                if (W_SUCCESS != controller_send_can(controller_output.commanded_angle)) {
                    log_text(
                        ERROR_TIMEOUT_MS, "controller", "commanded angle failed to send via CAN"
                    );
                }

                // log cmd angle

                data_container.controller.cmd_angle = controller_output.commanded_angle;
                if (W_SUCCESS !=
                    log_data(CONTROLLER_CYCLE_TIMEOUT_MS, LOG_TYPE_CANARD_CMD, &data_container)) {
                    log_text(ERROR_TIMEOUT_MS, "controller", "timeout for logging commanded angle");
                }

                vTaskDelayUntil(
                    &last_wake_time, pdMS_TO_TICKS(RECOVERY_TIMEOUT_MS)
                ); // 1s per iteration
                break;
            case STATE_ACT_ALLOWED:
                // wait for new state data (5ms timeout)
                controller_input_t new_state_msg;
                if (pdPASS == xQueueReceive(
                                  internal_state_queue,
                                  &new_state_msg,
                                  pdMS_TO_TICKS(CONTROLLER_CYCLE_TIMEOUT_MS)
                              )) {
                    controller_state.current_state = new_state_msg;

                    // no roll program for test flight

                } else {
                    controller_state.data_miss_counter++;

                    // TODO if number of data misses exceed threshold, transition to safe mode
                }

                // controller calc: interpolate
                if (W_SUCCESS != interpolate_gain(
                                     controller_state.current_state.pressure_dynamic,
                                     controller_state.current_state.canard_coeff,
                                     &controller_gain
                                 )) {
                    controller_output.commanded_angle =
                        commanded_angle_zero; // command zero when out of bound
                    log_text(ERROR_TIMEOUT_MS, "controller", "flight conditions out of bound");
                } else {
                    if (W_SUCCESS != get_commanded_angle(
                                         controller_gain,
                                         controller_state.current_state.roll_state.roll_state_arr,
                                         &controller_output.commanded_angle
                                     )) {
                        controller_output.commanded_angle = commanded_angle_zero;
                        log_text(ERROR_TIMEOUT_MS, "controller", "failed to get commanded angle");
                    }
                }

                // update timestamp for controller output
                if (W_SUCCESS != timer_get_ms(&current_timestamp_ms)) {
                    current_timestamp_ms = 0.0f;
                    log_text(
                        ERROR_TIMEOUT_MS,
                        "controller",
                        "failed to get timestamp for controller output"
                    );
                }
                controller_output.timestamp = (uint32_t)current_timestamp_ms;

                // update output queue
                xQueueOverwrite(output_queue, &controller_output);

                // log cmd angle

                data_container.controller.cmd_angle = controller_output.commanded_angle;
                if (W_SUCCESS !=
                    log_data(CONTROLLER_CYCLE_TIMEOUT_MS, LOG_TYPE_CANARD_CMD, &data_container)) {
                    log_text(ERROR_TIMEOUT_MS, "controller", "timeout for logging commanded angle");
                }

                // send command visa CAN + log status/errors
                if (W_SUCCESS != controller_send_can(controller_output.commanded_angle)) {
                    log_text(
                        ERROR_TIMEOUT_MS, "controller", "commanded angle failed to send via CAN"
                    );
                }
                break;
            default: // if not in proper state
                vTaskDelay(pdMS_TO_TICKS(STATE_ELSE_TIMEOUT));
                break;
        }
    }
}

