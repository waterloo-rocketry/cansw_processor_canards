#include "application/estimator/estimator.h"
#include "FreeRTOS.h"
#include "application/controller/controller.h"
#include "stm32h7xx_hal.h"
#include <queue.h>
#include <semphr.h>

QueueHandle_t imu_data_handle = NULL;
estimator_imu_input_data imu_data_buffer;
const int STATE_EST_CAN_RATE = 3;
uint8_t state_est_can_counter = 0;
estimator_controller_input_t controller_input_buffer;
controller_input_t output_to_controller;

w_status_t estimator_init(estimator_controller_input_t *data)
{
    imu_data_handle = xQueueCreate(1, sizeof(estimator_imu_input_data));

    if (imu_data_handle == NULL)
    {
        return W_FAILURE;
    }
    return W_SUCCESS;
}

w_status_t estimator_initialize_filter(void)
{
    // check for message from can_handler

    // initialize filter

    return W_SUCCESS;
}

w_status_t estimator_update_inputs_imu(estimator_imu_input_data *data)
{
    if (data == NULL)
    {
        return W_FAILURE;
    }
    if (xQueueOverwrite(imu_data_handle, data) != pdPASS)
    {
        return W_FAILURE;
    }
    return W_SUCCESS;
}

void estimator_task()
{
    // check for estimator initialization by CAN
    if (xQueueReceive(imu_data_handle, &imu_data_buffer, pdMS_TO_TICKS(5)) != pdTRUE)
    {
        log_text("State estimation", "failed to receive imu data within 5ms");
        return;
    }

    if (controller_get_latest_output(&controller_input_buffer) != W_SUCCESS)
    {
        log_text("State estimation", "failed to receive controller data");
        return;
    }

    double x_new[13];
    double P_new[13 * 13];

    // ekf_algorithm(x_new, P_new, imu_data_buffer, controller_input_buffer);

    // write information from x_new and P_new into output_to_controller

    if (controller_update_inputs(output_to_controller) != W_SUCCESS)
    {
        log_text("State estimation", "failed to give controller the output from state estimation");
    }

    if (state_est_can_counter % STATE_EST_CAN_RATE == 0)
    {
        // send to CAN
    }
    ++state_est_can_counter;
}
