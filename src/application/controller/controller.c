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
static controller_error_data_t controller_error_stats = {0};

// Send `canard_angle`, the desired canard angle (radians) to CAN
static w_status_t controller_send_can(float canard_angle) {
    // convert canard angle from radians to millidegrees
    int16_t canard_cmd_signed = (int16_t)(canard_angle / M_PI * 180.0 * 1000.0);
    uint16_t canard_cmd_shifted = canard_cmd_signed + 32768;

    // get timestamp for can msg
    float time_ms;
    if (W_SUCCESS != timer_get_ms(&time_ms)) {
        time_ms = 0.0f;
        controller_error_stats.timestamp_errors++;
    }
    uint32_t can_timestamp = (uint32_t)time_ms;

    // Build the CAN msg
    can_msg_t msg;
    if (!build_actuator_analog_cmd_msg(
            PRIO_HIGHEST, can_timestamp, ACTUATOR_CANARD_ANGLE, canard_cmd_shifted, &msg
        )) {
        log_text(ERROR_TIMEOUT_MS, "controller", "actuator message build failure");
        controller_error_stats.can_send_errors++;
    }

    // Send this to can handler module's tx
    w_status_t result = can_handler_transmit(&msg);
    if (result != W_SUCCESS) {
        controller_state.can_send_errors++;
    }
    return result;
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

    // Initialize error tracking
    controller_error_stats.is_init = true;
    controller_error_stats.can_send_errors = 0;
    controller_error_stats.data_miss_counter = 0;
    controller_error_stats.timestamp_errors = 0;
    controller_error_stats.gain_interpolation_errors = 0;
    controller_error_stats.angle_calculation_errors = 0;
    controller_error_stats.log_errors = 0;
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

    while (true) {
        // no phase change track
        flight_phase_state_t current_phase = flight_phase_get_state();
        switch (current_phase) {
            case STATE_RECOVERY:

                // update timestamp for controller output
                if (W_SUCCESS != timer_get_ms(&current_timestamp_ms)) {
                    current_timestamp_ms = 0.0f;
                    controller_error_stats.timestamp_errors++;
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

                if (W_SUCCESS != log_data(
                                     CONTROLLER_CYCLE_TIMEOUT_MS,
                                     LOG_TYPE_CANARD_CMD,
                                     (log_data_container_t *)&controller_output.commanded_angle
                                 )) {
                    log_text(ERROR_TIMEOUT_MS, "controller", "timeout for logging commanded angle");
                    controller_error_stats.log_errors++;
                }

                // delay 1s per iteration. not as precise as taskdelayuntil but doesnt matter here.
                // AVOID vtaskdelayuntil: it breaks cuz we dont use it in ACT_ALLOWED phase, so
                // last_wake_time doesnt get updated consistently and causes this to break as it
                // tries to catch up in time. TDOO: update freertos versions to where delayuntil has
                // a return value...
                vTaskDelay(pdMS_TO_TICKS(RECOVERY_TIMEOUT_MS));
                // vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(RECOVERY_TIMEOUT_MS));
                break;

            // for testflight: allowed to actuate from boost phase immediately
            case STATE_BOOST:
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
                    controller_error_stats.data_miss_counter++;

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
                    controller_error_stats.gain_interpolation_errors++;
                } else {
                    if (W_SUCCESS != get_commanded_angle(
                                         controller_gain,
                                         controller_state.current_state.roll_state.roll_state_arr,
                                         &controller_output.commanded_angle
                                     )) {
                        controller_output.commanded_angle = commanded_angle_zero;
                        log_text(ERROR_TIMEOUT_MS, "controller", "failed to get commanded angle");
                        controller_error_stats.angle_calculation_errors++;
                    }
                }

                // update timestamp for controller output
                if (W_SUCCESS != timer_get_ms(&current_timestamp_ms)) {
                    current_timestamp_ms = 0.0f;
                    controller_error_stats.timestamp_errors++;
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
                if (W_SUCCESS != log_data(
                                     CONTROLLER_CYCLE_TIMEOUT_MS,
                                     LOG_TYPE_CANARD_CMD,
                                     (log_data_container_t *)&controller_output.commanded_angle
                                 )) {
                    log_text(ERROR_TIMEOUT_MS, "controller", "timeout for logging commanded angle");
                    controller_error_stats.log_errors++;
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

/**
 * @brief Report controller module health status
 *
 * Retrieves and reports controller error statistics and initialization status
 * through log messages.
 *
 * @return W_SUCCESS if reporting was successful
 */
w_status_t controller_get_status(void) {
    // Log initialization status
    log_text(
        0, "controller", "Module initialized: %s", controller_error_stats.is_init ? "true" : "false"
    );

    // Log all error statistics
    log_text(
        0,
        "controller",
        "Error statistics: can_send=%lu, data_misses=%lu, timestamp=%lu, gain_interp=%lu, "
        "angle_calc=%lu, log=%lu",
        controller_error_stats.can_send_errors,
        controller_error_stats.data_miss_counter,
        controller_error_stats.timestamp_errors,
        controller_error_stats.gain_interpolation_errors,
        controller_error_stats.angle_calculation_errors,
        controller_error_stats.log_errors
    );

    // Also log the internal controller state error counters for comparison
    log_text(
        0,
        "controller",
        "Internal state counters: can_send_errors=%lu, data_miss_counter=%lu",
        controller_state.can_send_errors,
        controller_state.data_miss_counter
    );

    // Calculate total errors
    uint32_t total_errors =
        controller_error_stats.can_send_errors + controller_error_stats.data_miss_counter +
        controller_error_stats.timestamp_errors + controller_error_stats.gain_interpolation_errors +
        controller_error_stats.angle_calculation_errors + controller_error_stats.log_errors;

    // Log critical errors if significant issues are detected
    if (total_errors > 20) {
        log_text(0, "controller", "CRITICAL ERROR: Total errors: %lu", total_errors);
    }

    return W_SUCCESS;
}

