#include "math.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "canlib.h"

#include "application/can_handler/can_handler.h"
#include "application/controller/controller.h"
#include "application/estimator/estimator.h"
#include "application/estimator/estimator_module.h"
#include "application/estimator/estimator_types.h"
#include "application/estimator/pad_filter.h"
#include "application/flight_phase/flight_phase.h"
#include "application/imu_handler/imu_handler.h"
#include "application/logger/log.h"
#include "drivers/timer/timer.h"

// ---------- private variables ----------
static const uint32_t ESTIMATOR_TASK_PERIOD_MS = 5;
// Rate limit CAN tx: only send data at 10Hz, every 100ms
#define ESTIMATOR_CAN_TX_PERIOD_MS 100
#define ESTIMATOR_CAN_TX_RATE (ESTIMATOR_CAN_TX_PERIOD_MS / ESTIMATOR_TASK_PERIOD_MS)

// latest imu readings from imu handler
static QueueHandle_t imu_data_queue = NULL;
// latest encoder reading (millidegrees) from CAN
static QueueHandle_t encoder_data_queue_rad = NULL;
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
        log_text(1, "Estimator", "get_analog_data fail");
        return W_FAILURE;
    }

    if (SENSOR_CANARD_ENCODER_1 == sensor_id) {
        // shift back to signed and convert from mdeg to radians
        float encoder_val_rad = ((int16_t)raw_data - 32768) * (RAD_PER_DEG) / 1000.0f;

        // send to internal data queue
        xQueueOverwrite(encoder_data_queue_rad, &encoder_val_rad);

        // log converted encoder val
        log_data_container_t log_payload = {0};
        log_payload.encoder = encoder_val_rad;
        log_data(1, LOG_TYPE_ENCODER, &log_payload);
    }
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
    encoder_data_queue_rad = xQueueCreate(1, sizeof(float));
    controller_cmd_queue = xQueueCreate(1, sizeof(controller_output_t));

    if ((NULL == imu_data_queue) || (NULL == encoder_data_queue_rad) ||
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

w_status_t estimator_run_loop(estimator_module_ctx_t *ctx, uint32_t loop_count) {
    w_status_t status = W_SUCCESS;
    flight_phase_state_t curr_flight_phase = flight_phase_get_state();
    estimator_all_imus_input_t latest_imu_data = {0};
    controller_output_t latest_controller_cmd = {0};
    controller_input_t output_to_controller = {0};
    float latest_encoder_rad = 0;
    float curr_time_sec = 0.0f;
    bool encoder_is_dead = false;

    // get latest imu data, transform into estimator data structs.
    if (xQueueReceive(imu_data_queue, &latest_imu_data, 5) != pdTRUE) {
        log_text(5, "estimator", "imu data q empty");
        return W_FAILURE;
    }
    y_imu_t movella = {
        .accelerometer = latest_imu_data.movella.accelerometer,
        .gyroscope = latest_imu_data.movella.gyroscope,
        .magnetometer = latest_imu_data.movella.magnetometer,
        .barometer = latest_imu_data.movella.barometer
    };
    y_imu_t pololu = {
        .accelerometer = latest_imu_data.pololu.accelerometer,
        .gyroscope = latest_imu_data.pololu.gyroscope,
        .magnetometer = latest_imu_data.pololu.magnetometer,
        .barometer = latest_imu_data.pololu.barometer
    };

    // get the latest encoder reading. should be populating at 200Hz so 0ms wait
    if (xQueueReceive(encoder_data_queue_rad, &latest_encoder_rad, 0) != pdTRUE) {
        log_text(3, "Estimator", "encoder queue empty");
        encoder_is_dead = true; // mark encoder as dead if no data received
    }

    // get the latest controller cmd, only during flight
    // for testflight, boost state is also allowed
    if ((STATE_BOOST == curr_flight_phase) || (STATE_ACT_ALLOWED == curr_flight_phase)) {
        if (controller_get_latest_output(&latest_controller_cmd) != W_SUCCESS) {
            log_text(10, "Estimator", "controller_get_latest_output fail");
            status = W_FAILURE;
        }
    }

    // get current time. as failsafe: default to 5ms period
    float curr_time_ms = 0.0;
    if (timer_get_ms(&curr_time_ms) == W_SUCCESS) {
        curr_time_sec = curr_time_ms / 1000.0; // convert ms to seconds
    } else {
        log_text(10, "Estimator", "timer_get_ms fail");
        curr_time_sec = ctx->t + 0.005;
    }

    // run estimator module with all the inputs and ctx
    estimator_module_input_t estimator_input = {
        .timestamp = curr_time_sec,
        .movella = movella,
        .pololu = pololu,
        .movella_is_dead = latest_imu_data.movella.is_dead,
        .pololu_is_dead = latest_imu_data.pololu.is_dead,
        .cmd = latest_controller_cmd,
        .encoder = latest_encoder_rad,
        .encoder_is_dead = encoder_is_dead
    };

    // only run estimator with minimum 1 imu alive to avoid div by 0
    if (latest_imu_data.movella.is_dead && latest_imu_data.pololu.is_dead) {
        log_text(5, "Estimator", "both imus dead");
        status = W_FAILURE;
    } else {
        if (estimator_module(&estimator_input, curr_flight_phase, ctx, &output_to_controller) !=
            W_SUCCESS) {
            log_text(10, "Estimator", "estimator_module fail");
            status = W_FAILURE;
        }
    }

    // send controller cmd, only during flight, and if all data collected successfully
    if (W_SUCCESS == status) {
        if ((STATE_BOOST == curr_flight_phase) || (STATE_ACT_ALLOWED == curr_flight_phase)) {
            if (controller_update_inputs(&output_to_controller) != W_SUCCESS) {
                log_text(10, "Estimator", "failed to update controller inputs.");
                status = W_FAILURE;
            }
        }
    }

    // ------- do sdcard data logging at 200hz (only after pad filter starts) -------
    if (curr_flight_phase >= STATE_SE_INIT) {
        // log data sent to controller
        log_data_container_t log_payload = {0};

        log_payload.controller_input_t.roll_angle =
            (float)output_to_controller.roll_state.roll_angle;
        log_payload.controller_input_t.roll_rate = (float)output_to_controller.roll_state.roll_rate;
        log_payload.controller_input_t.canard_coeff = (float)output_to_controller.canard_coeff;
        log_payload.controller_input_t.pressure_dynamic =
            (float)output_to_controller.pressure_dynamic;

        log_data(1, LOG_TYPE_CONTROLLER_INPUT, &log_payload);

        // log current state est ctx

        log_payload.estimator_ctx_pt1.w = (float)ctx->x.attitude.w;
        log_payload.estimator_ctx_pt1.x = (float)ctx->x.attitude.x;
        log_payload.estimator_ctx_pt1.y = (float)ctx->x.attitude.y;
        log_payload.estimator_ctx_pt1.z = (float)ctx->x.attitude.z;

        log_payload.estimator_ctx_pt1.altitude = (float)ctx->x.altitude;

        log_data(1, LOG_TYPE_ESTIMATOR_CTX_PT1, &log_payload);

        log_payload.estimator_ctx_pt2.rates.x = (float)ctx->x.rates.x;
        log_payload.estimator_ctx_pt2.rates.y = (float)ctx->x.rates.y;
        log_payload.estimator_ctx_pt2.rates.z = (float)ctx->x.rates.z;

        log_payload.estimator_ctx_pt2.CL = (float)ctx->x.CL;

        log_payload.estimator_ctx_pt2.delta = (float)ctx->x.delta;

        log_data(1, LOG_TYPE_ESTIMATOR_CTX_PT2, &log_payload);

        log_payload.estimator_ctx_pt3.velocity.x = (float)ctx->x.velocity.x;
        log_payload.estimator_ctx_pt3.velocity.y = (float)ctx->x.velocity.y;
        log_payload.estimator_ctx_pt3.velocity.z = (float)ctx->x.velocity.z;

        log_payload.estimator_ctx_pt3.t = (float)ctx->t;

        log_data(1, LOG_TYPE_ESTIMATOR_CTX_PT3, &log_payload);

        // do CAN logging as backup less frequently to avoid flooding can bus
        if ((loop_count % ESTIMATOR_CAN_TX_RATE) == 0) {
            // do CAN logging
            if (estimator_log_state_to_can(&ctx->x) != W_SUCCESS) {
                log_text(0, "Estimator", "Failed to log state data to CAN");
                status = W_FAILURE; // mark failure but keep try to log other states
            }
        }
    }

    return status;
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
    // TickType_t last_wake_time;
    // last_wake_time = xTaskGetTickCount();

    // track how many times we ran estimator to ratelimit the CAN tx per N loops
    uint32_t estimator_loop_counter = 0;

    // estimator_module persistent ctx for the whole program
    estimator_module_ctx_t g_estimator_ctx = {0};

    // initialize ctx timestamp to current time
    float init_time_ms = 0.0f;
    if (timer_get_ms(&init_time_ms) != W_SUCCESS) {
        proc_handle_fatal_error("estini");
    }
    g_estimator_ctx.t = init_time_ms / 1000.0f; // convert ms to seconds

    log_text(10, "EstimatorTask", "Estimator task started.");

    while (true) {
        w_status_t run_status = estimator_run_loop(&g_estimator_ctx, estimator_loop_counter);

        if (run_status != W_SUCCESS) {
            log_text(
                1, "EstimatorTask", "ERROR: Estimator run loop failed (status: %d).", run_status
            );
        }

        estimator_loop_counter++;

        // // do delay here instead of inside the run to unify the timing
        // vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(ESTIMATOR_TASK_PERIOD_MS));
    }
}
