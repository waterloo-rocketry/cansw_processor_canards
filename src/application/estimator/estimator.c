#include "application/estimator/estimator.h"
#include "FreeRTOS.h"
#include "application/can_handler/can_handler.h"
#include "application/controller/controller.h"
#include "canlib.h"
#include "queue.h"
#include "stm32h7xx_hal.h"
#include <semphr.h>

QueueHandle_t imu_data_handle = NULL;
bool filter_is_initialized = false;
QueueHandle_t encoder_data_handle = NULL;
estimator_all_imus_input_t imu_data_buffer;
int STATE_EST_TASK_DELAY_MS = 5;
const int STATE_EST_CAN_RATE = 3; // we will send a message to CAN every 3 times estimator runs
uint8_t state_est_can_counter = 0;
controller_output_t input_from_controller;
controller_input_t output_to_controller;

w_status_t can_handler_encoder_msg(const can_msg_t *msg) {
    /*if (get_analog_data(msg, &sensor_id, &output_data)) {
        if (sensor_id == SENSOR_CANARD_ENCODER_1) {
            // return W_SUCCESS;
        }
    }*/
    return W_FAILURE;
}

w_status_t estimator_init() {
    imu_data_handle = xQueueCreate(1, sizeof(imu_data_buffer));

    if (NULL == imu_data_handle) {
        return W_FAILURE;
    }
    return W_SUCCESS;
}

w_status_t estimator_initialize_filter(void) {
    // check for message from can_handler
    if (W_SUCCESS != can_handler_register_callback(MSG_ACTUATOR_CMD, can_handler_encoder_msg)) {
        return W_FAILURE;
    }
    // initialize filter

    return W_SUCCESS;
}

w_status_t estimator_update_inputs_imu(estimator_all_imus_input_t *data) {
    if (NULL == data) {
        return W_FAILURE;
    }
    if (xQueueOverwrite(imu_data_handle, data) != pdPASS) {
        return W_FAILURE;
    }
    return W_SUCCESS;
}

void estimator_task(void *argument) {
    (void)argument;

    while (true) {
        if (filter_is_initialized == false) {
            continue;
        }
        if (xQueueReceive(imu_data_handle, &imu_data_buffer, pdMS_TO_TICKS(5)) != pdTRUE) {
            // log_text("State estimation", "failed to receive imu data within 5ms");
            return;
        }

        if (controller_get_latest_output(&input_from_controller) != W_SUCCESS) {
            // log_text("State estimation", "failed to receive controller data");
            return;
        }

        // double x_new[13];
        // double P_new[13 * 13];

        // ekf_algorithm(x_new, P_new, imu_data_buffer, input_from_controller);

        // write information from x_new and P_new into output_to_controller

        if (controller_update_inputs(&output_to_controller) != W_SUCCESS) {
            // log_text("State estimation", "failed to give controller the output from state
            // estimation");
        }

        if (state_est_can_counter % STATE_EST_CAN_RATE == 0) {
            // send to CAN
        }
        ++state_est_can_counter;
        vTaskDelay(pdMS_TO_TICKS(STATE_EST_TASK_DELAY_MS));
    }
}
