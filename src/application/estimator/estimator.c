#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "stm32h7xx_hal.h"
#include "task.h"

#include "application/can_handler/can_handler.h"
#include "application/controller/controller.h"
#include "application/estimator/estimator.h"
#include "application/estimator/estimator_types.h"
#include "application/flight_phase/flight_phase.h"
#include "application/imu_handler/imu_handler.h"
#include "application/logger/log.h"
#include "canlib.h"
#include "drivers/timer/timer.h"
#include "message_types.h"

extern TaskHandle_t estimator_task_handle;
// part of initiation

// ---------- private variables ----------
static const uint32_t ESTIMATOR_TASK_PERIOD_MS = 5;
// rate limit CAN tx: only send data every 50 times estimator runs (4hz)
static const uint32_t ESTIMATOR_CAN_TX_RATE = 500;

// latest imu readings from imu handler
static QueueHandle_t imu_data_queue = NULL;
// latest encoder reading (millidegrees) from CAN
static QueueHandle_t encoder_data_queue = NULL;
// latest control command (radians) from controller
static QueueHandle_t controller_cmd_queue = NULL;

// ---------- private static functions ----------
/**
 * callback to be registered with CAN handler for encoder msgs.
 * msg_type = MSG_SENSOR_ANALOG
 */
static w_status_t can_encoder_msg_callback(const can_msg_t *msg) {
    can_analog_sensor_id_t sensor_id;
    uint16_t raw_data;

    if (get_analog_data(msg, &sensor_id, &raw_data) == false) {
        log_text(1, "Estimator", "WARN: Failed to parse analog sensor CAN msg.");
        return W_FAILURE;
    }

    int16_t shifted_data = (int16_t)raw_data - 32768; // shift back to signed

    if (SENSOR_CANARD_ENCODER_1 == sensor_id) {
        xQueueOverwrite(encoder_data_queue, &shifted_data);
        // log raw CAN data; is just a uint16_t
        log_data(1, LOG_TYPE_ENCODER, (log_data_container_t *)&raw_data);
    }
    return W_SUCCESS;
}

/**
 * run the pad filter. this is called repeatedly in the estimator task
 * during the "on the pad" state
 */
static w_status_t estimator_run_pad_filter(void) {
    // TODO: run pad filter ...

    return W_SUCCESS;
}

// ---------- public functions ----------

w_status_t estimator_init(void) {
    // register the callback for the encoder can msgs
    if (W_SUCCESS != can_handler_register_callback(MSG_SENSOR_ANALOG, can_encoder_msg_callback)) {
        return W_FAILURE;
    }

    // create queues for imu data, encoder data, and controller cmd
    imu_data_queue = xQueueCreate(1, sizeof(estimator_all_imus_input_t));
    encoder_data_queue = xQueueCreate(1, sizeof(int16_t));
    controller_cmd_queue = xQueueCreate(1, sizeof(controller_output_t));

    if ((NULL == imu_data_queue) || (NULL == encoder_data_queue) ||
        (NULL == controller_cmd_queue)) {
        return W_FAILURE;
    }
    return W_SUCCESS;
}

w_status_t estimator_update_imu_data(estimator_all_imus_input_t *data) {
    if (NULL == data) {
        return W_FAILURE;
    }
    xQueueOverwrite(imu_data_queue, data);
    return W_SUCCESS;
}

/**
 * run 1 cycle of the estimator loop. this is called from the task
 */
w_status_t estimator_run_loop(uint32_t loop_count) {
    flight_phase_state_t curr_flight_phase = flight_phase_get_state();

    switch (curr_flight_phase) {
        // ------- if idle state: do nothing -------
        case STATE_PAD:
            // do nothing.
            break;

        // ------- on the pad: run the pad filter -------
        case STATE_SE_INIT:
            if (estimator_run_pad_filter() != W_SUCCESS) {
                // estimator_run_pad_filter should log its own errors
                log_text(10, "Estimator", "ERROR: Pad filter run failed.");
                return W_FAILURE;
            }
            break;

        // ------- flight!! perform a state estimation cycle -------
        case STATE_BOOST:
        case STATE_ACT_ALLOWED:
        case STATE_RECOVERY:
            estimator_all_imus_input_t latest_imu_data;
            controller_output_t latest_controller_cmd;
            controller_input_t output_to_controller;
            int16_t latest_encoder_data = 0;

            // get the latest imu readings
            // TODO: max timeout should be <5ms cuz the rest of the task takes time too...
            // Timeout set to 5ms just for development stage
            if (xQueueReceive(
                    imu_data_queue, &latest_imu_data, pdMS_TO_TICKS(ESTIMATOR_TASK_PERIOD_MS)
                ) != pdTRUE) {
                log_text(
                    5,
                    "Estimator",
                    "ERROR: Failed to receive IMU data within %dms!",
                    ESTIMATOR_TASK_PERIOD_MS
                );
                return W_FAILURE;
            }

            // get the latest encoder reading. should always be populated, hence 0 wait
            if (xQueuePeek(encoder_data_queue, &latest_encoder_data, 0) != pdTRUE) {
                // RIP ENCODER FOR TEST FLIGHT
                log_text(3, "Estimator", "failed to receive encoder data");
                // return W_FAILURE;
            }

            // get the latest controller cmd
            if (controller_get_latest_output(&latest_controller_cmd) != W_SUCCESS) {
                log_text(10, "Estimator", "failed to get latest controller output");
                return W_FAILURE;
            }

            // TODO: run the state estimation algorithm...
            // double x_new[13];
            // double P_new[13 * 13];

            // ekf_algorithm(x_new, P_new, latest_imu_data, latest_controller_cmd);

            // write information from x_new and P_new into output_to_controller

            // TODO: Remove this dummy state once EKF is implemented
            x_state_t dummy_state = {0};
            // Populate with some dummy values for testing
            dummy_state.attitude.w = 1.0f;
            dummy_state.rates.x = 0.1f;
            dummy_state.velocity.z = -9.8f;
            dummy_state.altitude = 100.0f;
            dummy_state.CL = 0.5f;
            dummy_state.delta = 0.05f;
            // END DUMMY STATE

            if (controller_update_inputs(&output_to_controller) != W_SUCCESS) {
                log_text(10, "Estimator", "failed to update controller inputs.");
                return W_FAILURE;
            }

            // ------- do sdcard data logging at 200hz (every loop) -------
            log_data_container_t log_data_payload = {0};
            // log data sent to controller
            log_data_payload.controller_input = output_to_controller; // Copy struct
            log_data(1, LOG_TYPE_CONTROLLER_INPUT, &log_data_payload);
            // log current state est state
            log_data_payload.estimator_state = dummy_state; // Copy struct
            log_data(1, LOG_TYPE_ESTIMATOR_STATE, &log_data_payload);

            // do CAN logging as backup less frequently to avoid flooding can bus
            if (loop_count % ESTIMATOR_CAN_TX_RATE == 0) {
                loop_count = 0;

                // do CAN logging
                if (estimator_log_state_to_can(&dummy_state) != W_SUCCESS) {
                    log_text(0, "Estimator", "Failed to log state data to CAN");
                    // Decide if this should be a hard failure idk
                }
            }
            break;
        default:
            log_text(10, "Estimator", "invalid flight phase: %d", curr_flight_phase);
            return W_FAILURE;
            break;
    }

    return W_SUCCESS;
}

w_status_t estimator_log_state_to_can(const x_state_t *current_state) {
    can_msg_t msg;
    float current_time_ms;
    w_status_t status = W_SUCCESS;

    if (W_SUCCESS != timer_get_ms(&current_time_ms)) {
        current_time_ms = 0.0f; // Default to 0 if timer fails
    }
    uint16_t timestamp_16bit = (uint16_t)current_time_ms;

    // Iterate through all defined state IDs
    for (can_state_est_id_t state_id = 0; state_id < STATE_ID_ENUM_MAX; ++state_id) {
        // The x_state_t union maps directly to the enum order if accessed as an array
        // Convert the doubles in x_state_t to floats for CAN message
        float state_value = (float)current_state->array[state_id];

        if (!build_state_est_data_msg(PRIO_LOW, timestamp_16bit, state_id, &state_value, &msg)) {
            log_text(0, "Estimator", "Failed to build CAN message for state ID %d", state_id);
            status = W_FAILURE; // Mark as failure but continue trying other states
            continue;
        }

        if (W_SUCCESS != can_handler_transmit(&msg)) {
            log_text(0, "Estimator", "Failed to transmit CAN message for state ID %d", state_id);
            status = W_FAILURE; // Mark as failure but continue trying other states
        }
    }

    return status;
}

void estimator_task(void *argument) {
    (void)argument;
    TickType_t last_wake_time;
    last_wake_time = xTaskGetTickCount();
    // track how many times we ran the loop, so we can rate limit the CAN tx per N loops
    uint32_t state_est_loop_counter = 0;

    log_text(10, "EstimatorTask", "Estimator task started.");

    while (true) {
        w_status_t run_status = estimator_run_loop(state_est_loop_counter);
        if (run_status != W_SUCCESS) {
            log_text(
                1, "EstimatorTask", "ERROR: Estimator run loop failed (status: %d).", run_status
            );
        }

        state_est_loop_counter++;

        // do delay here instead of inside the run to unify the timing
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(ESTIMATOR_TASK_PERIOD_MS));
    }
}
