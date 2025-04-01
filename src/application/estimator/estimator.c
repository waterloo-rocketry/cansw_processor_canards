#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "stm32h7xx_hal.h"
#include "task.h"

#include "application/can_handler/can_handler.h"
#include "application/controller/controller.h"
#include "application/estimator/estimator.h"
#include "application/flight_phase/flight_phase.h"
#include "canlib.h"

extern TaskHandle_t estimator_task_handle;

// ---------- private variables ----------
static const uint32_t ESTIMATOR_TASK_PERIOD_MS = 5;
// rate limit CAN tx: only send data every 3 times estimator runs
static const uint32_t ESTIMATOR_CAN_TX_RATE = 3;

// latest imu readings from imu handler
static QueueHandle_t imu_data_queue = NULL;
// latest encoder reading (radians) from CAN
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
    uint16_t data;

    if (get_analog_data(msg, &sensor_id, &data) == false) {
        // log_text("Estimator", "failed to get can sensor data");
        return W_FAILURE;
    }

    if (SENSOR_CANARD_ENCODER_1 == sensor_id) {
        xQueueOverwrite(encoder_data_queue, &data);
        // log_text("Estimator", "encoder data: %d", data);
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
    encoder_data_queue = xQueueCreate(1, sizeof(float));
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

    if (xQueueOverwrite(imu_data_queue, data) != pdPASS) {
        return W_FAILURE;
    }

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
                return W_FAILURE;
                // log_text("Estimator", "failed to run pad filter");
            }
            break;

        // ------- flight!! perform a state estimation cycle -------
        case STATE_BOOST:
        case STATE_ACT_ALLOWED:
        case STATE_RECOVERY:
            estimator_all_imus_input_t latest_imu_data;
            controller_output_t latest_controller_cmd;
            controller_input_t output_to_controller;

            // get the latest imu readings
            // TODO: max timeout should be <5ms cuz the rest of the task takes time too...
            if (xQueueReceive(
                    imu_data_queue, &latest_imu_data, pdMS_TO_TICKS(ESTIMATOR_TASK_PERIOD_MS)
                ) != pdTRUE) {
                // log_text("State estimation", "failed to receive imu data!");
                return W_FAILURE;
            }

            // get the latest encoder reading. should always be populated, hence 0 wait
            if (xQueuePeek(encoder_data_queue, &latest_imu_data, 0) != pdTRUE) {
                // log_text("State estimation", "failed to receive imu data!");
                return W_FAILURE;
            }

            // get the latest controller cmd
            if (controller_get_latest_output(&latest_controller_cmd) != W_SUCCESS) {
                // log_text("State estimation", "failed to receive controller data");
                return W_FAILURE;
            }

            // TODO: run the state estimation algorithm...
            // double x_new[13];
            // double P_new[13 * 13];

            // ekf_algorithm(x_new, P_new, latest_imu_data, latest_controller_cmd);

            // write information from x_new and P_new into output_to_controller

            if (controller_update_inputs(&output_to_controller) != W_SUCCESS) {
                // log_text("State estimation", "failed to give controller the output from state
                // estimation");
                return W_FAILURE;
            }

            // ------- do data logging -------
            if (loop_count % ESTIMATOR_CAN_TX_RATE == 0) {
                loop_count = 0;
                // TODO: send to CAN for logging
            }
            loop_count++;
            break;
        default:
            // log_text("Estimator", "invalid flight phase: %d", curr_flight_phase);
            return W_FAILURE;
            break;
    }

    return W_SUCCESS;
}

void estimator_task(void *argument) {
    (void)argument;
    TickType_t last_wake_time;
    last_wake_time = xTaskGetTickCount();
    // track how many times we ran the loop, so we can rate limit the CAN tx per N loops
    uint32_t state_est_loop_counter = 0;

    while (true) {
        estimator_run_loop(state_est_loop_counter);

        // do delay here instead of inside the run to unify the timing
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(ESTIMATOR_TASK_PERIOD_MS));
    }
}
