#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "application/can_handler/can_handler.h"
#include "application/estimator/estimator.h"
#include "application/imu_handler/imu_handler.h"
#include "application/logger/log.h"
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"
#include "drivers/movella/movella.h"
#include "drivers/timer/timer.h"

#include "canlib.h"

// Period of IMU sampling in milliseconds
#define IMU_SAMPLING_PERIOD_MS 5

// Timeout values for freshness check (in milliseconds)
#define GYRO_FRESHNESS_TIMEOUT_MS 5
#define MAG_FRESHNESS_TIMEOUT_MS 10
#define ACCEL_FRESHNESS_TIMEOUT_MS 5
#define BARO_FRESHNESS_TIMEOUT_MS 25

// Rate limit CAN tx: only send data at 10Hz, every 100ms
#define IMU_HANDLER_CAN_TX_PERIOD_MS 100
#define IMU_HANDLER_CAN_TX_RATE (IMU_HANDLER_CAN_TX_PERIOD_MS / IMU_SAMPLING_PERIOD_MS)

// correct orientation from simulink-canards model_params.m, commit e20e5d1
// S1 (movella)
static const matrix3d_t g_movella_upd_mat = {.array = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
// S2 (pololu)
static const matrix3d_t g_pololu_upd_mat = {.array = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};

// Module state tracking
typedef struct {
    bool initialized;
    uint32_t sample_count;
    uint32_t error_count;

    // Per-IMU stats
    struct {
        uint32_t success_count;
        uint32_t failure_count;
    } pololu_stats, movella_stats;
} imu_handler_state_t;

static imu_handler_state_t imu_handler_state = {0};

static w_status_t log_raw_to_can(raw_pololu_data_t *raw_data) {
    // Log raw data to CAN
    can_msg_t msg;
    float timestamp = 0.0f;
    timer_get_ms(&timestamp);
    bool build_sts = true;
    w_status_t can_tx_sts = W_SUCCESS;

    // Build CAN message with raw data
    build_sts &= build_imu_data_msg(
        PRIO_LOW,
        (uint16_t)timestamp,
        'X',
        IMU_PROC_ALTIMU10,
        raw_data->raw_acc.x,
        raw_data->raw_gyro.x,
        &msg
    );
    can_tx_sts |= can_handler_transmit(&msg);

    build_sts &= build_imu_data_msg(
        PRIO_LOW,
        (uint16_t)timestamp,
        'Y',
        IMU_PROC_ALTIMU10,
        raw_data->raw_acc.y,
        raw_data->raw_gyro.y,
        &msg
    );
    can_tx_sts |= can_handler_transmit(&msg);

    build_sts &= build_imu_data_msg(
        PRIO_LOW,
        (uint16_t)timestamp,
        'Z',
        IMU_PROC_ALTIMU10,
        raw_data->raw_acc.z,
        raw_data->raw_gyro.z,
        &msg
    );
    can_tx_sts |= can_handler_transmit(&msg);

    build_sts &= build_mag_data_msg(
        PRIO_LOW, (uint16_t)timestamp, 'X', IMU_PROC_ALTIMU10, raw_data->raw_mag.x, &msg
    );
    can_tx_sts |= can_handler_transmit(&msg);

    build_sts &= build_mag_data_msg(
        PRIO_LOW, (uint16_t)timestamp, 'Y', IMU_PROC_ALTIMU10, raw_data->raw_mag.y, &msg
    );
    can_tx_sts |= can_handler_transmit(&msg);

    build_sts &= build_mag_data_msg(
        PRIO_LOW, (uint16_t)timestamp, 'Z', IMU_PROC_ALTIMU10, raw_data->raw_mag.z, &msg
    );
    can_tx_sts |= can_handler_transmit(&msg);

    build_sts &= build_baro_data_msg(
        PRIO_LOW,
        (uint16_t)timestamp,
        IMU_PROC_ALTIMU10,
        raw_data->raw_baro.pressure,
        raw_data->raw_baro.temperature,
        &msg
    );
    can_tx_sts |= can_handler_transmit(&msg);

    // Transmit CAN message
    if (can_tx_sts != W_SUCCESS) {
        log_text(0, "IMUHandler", "CAN tx failed");
    }
    if (!build_sts) {
        log_text(0, "IMUHandler", "build raw CAN msg failed");
    }

    if ((can_tx_sts != W_SUCCESS) || !build_sts) {
        imu_handler_state.error_count++;
        return W_FAILURE;
    }
    return W_SUCCESS;
}

/**
 * @brief Read data from the pololu AltIMU-10 sensor
 * @param imu_data Pointer to store the converted data
 * @param raw_data Pointer to store the raw data
 * @return Status of the read operation
 */
static w_status_t
read_pololu_imu(estimator_imu_measurement_t *imu_data, raw_pololu_data_t *raw_data) {
    w_status_t status = W_SUCCESS;

    // Read accelerometer, gyro, and magnetometer data
    status |= altimu_get_acc_data(&imu_data->accelerometer, &raw_data->raw_acc);
    status |= altimu_get_gyro_data(&imu_data->gyroscope, &raw_data->raw_gyro);
    status |= altimu_get_mag_data(&imu_data->magnetometer, &raw_data->raw_mag);

    // Read barometer data
    altimu_barometer_data_t baro_data;
    status |= altimu_get_baro_data(&baro_data, &raw_data->raw_baro);

    if (W_SUCCESS == status) {
        // convert gyro to rad/sec
        imu_data->gyroscope.x = imu_data->gyroscope.x * RAD_PER_DEG;
        imu_data->gyroscope.y = imu_data->gyroscope.y * RAD_PER_DEG;
        imu_data->gyroscope.z = imu_data->gyroscope.z * RAD_PER_DEG;

        // Apply orientation correction
        imu_data->accelerometer =
            math_vector3d_rotate(&g_pololu_upd_mat, &(imu_data->accelerometer));
        imu_data->gyroscope = math_vector3d_rotate(&g_pololu_upd_mat, &(imu_data->gyroscope));
        imu_data->magnetometer = math_vector3d_rotate(&g_pololu_upd_mat, &(imu_data->magnetometer));

        imu_data->barometer = baro_data.pressure;
        imu_data->is_dead = false;
        imu_handler_state.pololu_stats.success_count++;
    } else {
        // Set is_dead flag to indicate IMU failure
        imu_data->is_dead = true;
        imu_handler_state.pololu_stats.failure_count++;
    }

    return status;
}

/**
 * @brief Read data from the Movella MTi-630 sensor
 * @param imu_data Pointer to store the IMU data
 * @return Status of the read operation
 */
static w_status_t read_movella_imu(estimator_imu_measurement_t *imu_data) {
    w_status_t status;

    // Read all data from Movella in one call
    movella_data_t movella_data = {0}; // Initialize to zero
    status = movella_get_data(&movella_data, 100); // Add 100ms timeout

    if (W_SUCCESS == status) {
        // convert m/s^2 to gravities
        movella_data.acc.x = movella_data.acc.x / 9.81f;
        movella_data.acc.y = movella_data.acc.y / 9.81f;
        movella_data.acc.z = movella_data.acc.z / 9.81f;

        // Copy data from Movella
        // Apply orientation correction
        imu_data->accelerometer = math_vector3d_rotate(&g_movella_upd_mat, &movella_data.acc);
        imu_data->gyroscope = math_vector3d_rotate(&g_movella_upd_mat, &movella_data.gyr);
        imu_data->magnetometer = math_vector3d_rotate(&g_movella_upd_mat, &movella_data.mag);

        imu_data->barometer = movella_data.pres;
        imu_data->is_dead = false;
        imu_handler_state.movella_stats.success_count++;
    } else {
        // Set is_dead flag to indicate IMU failure
        imu_data->is_dead = true;
        imu_handler_state.movella_stats.failure_count++;
    }

    return status;
}

/**
 * @brief Initialize the IMU handler module
 * @note This function is called before the scheduler starts
 * @return Status of initialization
 */
w_status_t imu_handler_init(void) {
    // TODO: poll all imus to make sure theyre initialized alr or smth

    // Set initialized flag directly here instead of calling initialize_all_imus()
    imu_handler_state.initialized = true;

    log_text(10, "IMUHandler", "IMU Handler Initialized.");
    return W_SUCCESS;
}

/**
 * @brief Execute one iteration of the IMU handler processing
 * Reads data from all IMUs and updates the estimator
 * @param loop_count Number of loops run, for CAN send rate limiting
 * @note This function is non-static to allow exposed to unit tests
 * @return Status of the execution
 */
w_status_t imu_handler_run(uint32_t loop_count) {
    estimator_all_imus_input_t imu_data = {
        .pololu = {.is_dead = false}, .movella = {.is_dead = false}
    };
    raw_pololu_data_t raw_pololu_data = {0};
    float current_time_ms;
    w_status_t status = W_SUCCESS;

    // Get current timestamp
    if (W_SUCCESS != timer_get_ms(&current_time_ms)) {
        current_time_ms = 0.0f;
    }
    uint32_t now_ms = (uint32_t)current_time_ms;

    // Set timestamps for all IMUs
    // Note: All IMUs get the same timestamp intentionally for synchronization
    imu_data.pololu.timestamp_imu = now_ms;
    imu_data.movella.timestamp_imu = now_ms;

    // Read from all IMUs, including orientation correction
    w_status_t pololu_status = read_pololu_imu(&imu_data.pololu, &raw_pololu_data);
    w_status_t movella_status = read_movella_imu(&imu_data.movella);

    // If both IMUs fail, consider it a system-level failure
    if ((W_FAILURE == pololu_status) && (W_FAILURE == movella_status)) {
        log_text(1, "IMUHandler", "ERROR: Both pololu and Movella IMU reads failed.");
        status = W_FAILURE;
    } else if (W_FAILURE == pololu_status) {
        log_text(1, "IMUHandler", "WARN: pololu IMU read failed.");
    } else if (W_FAILURE == movella_status) {
        log_text(1, "IMUHandler", "WARN: Movella IMU read failed.");
    }

    // Log one imu data at a time + raw pololu
    log_data_container_t log_data_container = {0}; //{.imu_reading = imu_data.movella};
    log_data_container.imu_reading.accelerometer = imu_data.movella.accelerometer;
    log_data_container.imu_reading.gyroscope = imu_data.movella.gyroscope;
    log_data_container.imu_reading.magnetometer = imu_data.movella.magnetometer;
    log_data_container.imu_reading.barometer = imu_data.movella.barometer;
    log_data_container.imu_reading.timestamp_imu = imu_data.movella.timestamp_imu;
    log_data_container.imu_reading.is_dead = imu_data.movella.is_dead;
    log_data(1, LOG_TYPE_MOVELLA_READING, &log_data_container);

    log_data_container.imu_reading.accelerometer = imu_data.pololu.accelerometer;
    log_data_container.imu_reading.gyroscope = imu_data.pololu.gyroscope;
    log_data_container.imu_reading.magnetometer = imu_data.pololu.magnetometer;
    log_data_container.imu_reading.barometer = imu_data.pololu.barometer;
    log_data_container.imu_reading.timestamp_imu = imu_data.pololu.timestamp_imu;
    log_data_container.imu_reading.is_dead = imu_data.pololu.is_dead;
    log_data(1, LOG_TYPE_POLOLU_READING, &log_data_container);

    log_data_container.raw_pololu_data = raw_pololu_data;
    log_data(1, LOG_TYPE_POLOLU_RAW, &log_data_container);

    // do CAN logging as backup less frequently to avoid flooding can bus
    if ((loop_count % IMU_HANDLER_CAN_TX_RATE) == 0) {
        if (log_raw_to_can(&raw_pololu_data) != W_SUCCESS) {
            log_text(0, "imuhandler", "raw log to CAN fail");
        }
    }

    // Send data to estimator with status flags
    w_status_t estimator_status = estimator_update_imu_data(&imu_data);
    if (W_SUCCESS != estimator_status) {
        status = estimator_status;
        imu_handler_state.error_count++;
        log_text(1, "IMUHandler", "estimator update fail (status: %d).", estimator_status);
    }

    imu_handler_state.sample_count++;

    // Return overall status
    return status;
}

/**
 * @brief IMU handler task function for FreeRTOS
 * @note This task will be created during system initialization
 * @param argument Task argument (unused)
 */
void imu_handler_task(void *argument) {
    (void)argument; // Unused parameter

    // Variables for precise timing control
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(IMU_SAMPLING_PERIOD_MS);

    // track loop count for CAN tx rate limiting
    uint32_t loop_count = 0;

    // Main task loop
    log_text(10, "IMUHandlerTask", "IMU Handler task started.");
    while (1) {
        w_status_t run_status = imu_handler_run(loop_count++);
        if (W_SUCCESS != run_status) {
            // Log or handle run failures if needed
            imu_handler_state.error_count++;
            log_text(1, "IMUHandlerTask", "run failed (status: %d).", run_status);
        }

        // Wait for next sampling period with precise timing
        vTaskDelayUntil(&last_wake_time, frequency);
    }
}
