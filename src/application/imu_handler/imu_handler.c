#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "application/estimator/estimator.h"
#include "application/imu_handler/imu_handler.h"
#include "common/math/math-algebra3d.h"
#include "drivers/altimu-10/altimu-10.h"
#include "drivers/movella/movella.h"
#include "drivers/timer/timer.h"

// Period of IMU sampling in milliseconds
#define IMU_SAMPLING_PERIOD_MS 5

// Timeout values for freshness check (in milliseconds)
#define GYRO_FRESHNESS_TIMEOUT_MS 5
#define MAG_FRESHNESS_TIMEOUT_MS 10
#define ACCEL_FRESHNESS_TIMEOUT_MS 5
#define BARO_FRESHNESS_TIMEOUT_MS 25

// correct orientation from simulink-canards model_params.m, commit e20e5d1
// S1 (movella)
static const matrix3d_t g_movella_upd_mat = {.array = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
// S2 (polulu)
static const matrix3d_t g_polulu_upd_mat = {.array = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};

// Module state tracking
typedef struct {
    bool initialized;
    uint32_t sample_count;
    uint32_t error_count;

    // Per-IMU stats
    struct {
        uint32_t success_count;
        uint32_t failure_count;
    } polulu_stats, movella_stats;
} imu_handler_state_t;

static imu_handler_state_t imu_handler_state = {0};

/**
 * @brief Read data from the Polulu AltIMU-10 sensor
 * @param imu_data Pointer to store the IMU data
 * @return Status of the read operation
 */
static w_status_t read_pololu_imu(estimator_imu_measurement_t *imu_data) {
    w_status_t status = W_SUCCESS;

    // Read accelerometer, gyro, and magnetometer data
    status |= altimu_get_acc_data(&imu_data->accelerometer);
    status |= altimu_get_gyro_data(&imu_data->gyroscope);
    status |= altimu_get_mag_data(&imu_data->magnetometer);

    // Read barometer data
    altimu_barometer_data_t baro_data;
    status |= altimu_get_baro_data(&baro_data);

    if (W_SUCCESS == status) {
        // Apply orientation correction
        imu_data->accelerometer =
            math_vector3d_rotate(&g_polulu_upd_mat, &(imu_data->accelerometer));
        imu_data->gyroscope = math_vector3d_rotate(&g_polulu_upd_mat, &(imu_data->gyroscope));
        imu_data->magnetometer = math_vector3d_rotate(&g_polulu_upd_mat, &(imu_data->magnetometer));

        imu_data->barometer = baro_data.pressure;
        imu_data->is_dead = false;
        imu_handler_state.polulu_stats.success_count++;
    } else {
        // Set is_dead flag to indicate IMU failure
        imu_data->is_dead = true;
        imu_handler_state.polulu_stats.failure_count++;
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

    return W_SUCCESS;
}

/**
 * @brief Execute one iteration of the IMU handler processing
 * Reads data from all IMUs and updates the estimator
 * @return Status of the execution
 */
w_status_t imu_handler_run(void) {
    estimator_all_imus_input_t imu_data = {
        .polulu = {.is_dead = false}, .movella = {.is_dead = false}
    };
    float current_time_ms;
    w_status_t status = W_SUCCESS;

    // Get current timestamp
    if (W_SUCCESS != timer_get_ms(&current_time_ms)) {
        current_time_ms = 0.0f;
    }
    uint32_t now_ms = (uint32_t)current_time_ms;

    // Set timestamps for all IMUs
    // Note: All IMUs get the same timestamp intentionally for synchronization
    imu_data.polulu.timestamp_imu = now_ms;
    imu_data.movella.timestamp_imu = now_ms;

    // Read from all IMUs, including orientation correction
    w_status_t polulu_status = read_pololu_imu(&imu_data.polulu);
    w_status_t movella_status = read_movella_imu(&imu_data.movella);

    // If both IMUs fail, consider it a system-level failure
    if (W_FAILURE == polulu_status && W_FAILURE == movella_status) {
        status = W_FAILURE;
    }

    // Send data to estimator with status flags
    w_status_t estimator_status = estimator_update_imu_data(&imu_data);
    if (W_SUCCESS != estimator_status) {
        status = estimator_status;
        imu_handler_state.error_count++;
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
    TickType_t last_wake_time;
    const TickType_t frequency = pdMS_TO_TICKS(IMU_SAMPLING_PERIOD_MS);

    // Initialize last_wake_time to current time
    last_wake_time = xTaskGetTickCount();

    // Main task loop
    while (1) {
        w_status_t run_status = imu_handler_run();
        if (W_SUCCESS != run_status) {
            // Log or handle run failures if needed
            imu_handler_state.error_count++;
        }

        // Wait for next sampling period with precise timing
        vTaskDelayUntil(&last_wake_time, frequency);
    }
}
