#include "application/controller/controller.h"
#include "application/can_handler/can_handler.h"
#include "application/controller/controller_algorithm.h"
#include "application/controller/controller_module.h"
#include "application/flight_phase/flight_phase.h"
#include "application/logger/log.h"
#include "drivers/timer/timer.h"
#include "queue.h"
#include "third_party/canlib/message/msg_actuator.h"

static QueueHandle_t internal_state_queue;
static QueueHandle_t output_queue;

#define CONTROLLER_CYCLE_MS 5
#define ERROR_TIMEOUT_MS 10
#define RECOVERY_TIMEOUT_MS 1000
#define STATE_ELSE_TIMEOUT 1

static controller_t controller_state = {0};
static const double cmd_angle_zero = 0.0; // safe mode, init overwrite, p and c out of bound

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
 * helper to handle a new canard cmd:
 * send canard cmd to CAN, update output queue, and log to sd card.
 * Return W_FAILURE if any of the steps fail, but still try to do everything regardless.
 */
static w_status_t process_new_cmd(double cmd) {
    w_status_t status = W_SUCCESS; // track status but still try to do everything regardless
    float timestamp_ms = 0.0f;
    log_data_container_t log_container = {0};
    // object to copy into the outputs queue (queue is pass by copy, not by reference)
    controller_output_t controller_output = {0};

    // get current timestamp
    if (W_SUCCESS != timer_get_ms(&timestamp_ms)) {
        timestamp_ms = 0.0f;
        log_text(ERROR_TIMEOUT_MS, "controller", "get_ms fail");
        status |= W_FAILURE;
    }

    // set controller output
    controller_output.commanded_angle = cmd;
    controller_output.timestamp = (uint32_t)timestamp_ms;

    // send command via CAN
    if (W_SUCCESS != controller_send_can(controller_output.commanded_angle)) {
        controller_state.can_send_errors++;
        log_text(ERROR_TIMEOUT_MS, "controller", "CAN send failure");
        status |= W_FAILURE;
    }

    // update output queue
    xQueueOverwrite(output_queue, &controller_output);

    // log to sd card
    log_container.controller.cmd_angle = controller_output.commanded_angle;
    if (W_SUCCESS != log_data(5, LOG_TYPE_CANARD_CMD, &log_container)) {
        log_text(ERROR_TIMEOUT_MS, "controller", "log cmd fail");
        status |= W_FAILURE;
    }

    return status;
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
    xQueueOverwrite(output_queue, &cmd_angle_zero);

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
 * Controller freertos task
 */
void controller_task(void *argument) {
    (void)argument;

    while (true) {
        w_status_t status = W_SUCCESS;
        flight_phase_state_t current_phase = flight_phase_get_state();

        // do the following steps which vary depending on curr flight phase:
        // 1. determine new canard angle cmd
        // 2. process new cmd (send to CAN, update output queue, log)
        // 3. do task delay
        switch (current_phase) {
            // recovery: actively cmd 0 deg at 1hz
            case STATE_RECOVERY:
                status |= process_new_cmd(cmd_angle_zero);

                // delay 1s per iteration. not as precise as taskdelayuntil but doesnt matter here.
                // AVOID vtaskdelayuntil: it breaks cuz we dont use it in ACT_ALLOWED phase, so
                // last_wake_time doesnt get updated consistently and causes this to break as it
                // tries to catch up in time. TDOO: update freertos versions to where delayuntil has
                // a return value...
                vTaskDelay(pdMS_TO_TICKS(RECOVERY_TIMEOUT_MS));
                // vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(RECOVERY_TIMEOUT_MS));
                break;

            // actuation allowed: run controller module
            case STATE_ACT_ALLOWED:
                controller_input_t new_state_msg = {0};
                float ref_signal = 0.0f; // track latest reference signal for logging only
                uint32_t flight_time_ms = 0; // elapsed flight time
                double new_cmd = 0.0;

                // wait for new state data (5ms timeout)
                if (xQueueReceive(
                        internal_state_queue, &new_state_msg, pdMS_TO_TICKS(CONTROLLER_CYCLE_MS)
                    ) != pdPASS) {
                    controller_state.data_miss_counter++;
                    // TODO if number of data misses exceed threshold, transition to safe mode
                }

                // get elapsed flight time
                status |= flight_phase_get_flight_ms(&flight_time_ms);

                // run controller module
                status |= controller_module(new_state_msg, flight_time_ms, &new_cmd, &ref_signal);

                // send cmd
                if (status != W_SUCCESS) {
                    // if something failed, fail-safe to 0 deg cmd
                    new_cmd = cmd_angle_zero;
                    log_text(ERROR_TIMEOUT_MS, "controller", "controller_module fail");
                }

                status |= process_new_cmd(new_cmd);
                break;

            default: // do nothing / wait for new state
                vTaskDelay(pdMS_TO_TICKS(STATE_ELSE_TIMEOUT));
                break;
        }
    }
}

