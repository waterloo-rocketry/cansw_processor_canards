#include "application/imu_handler/imu_handler.h"
#include "application/estimator/estimator.h"
#include "drivers/altimu-10/altimu-10.h"
#include "drivers/lsm6dsv32x/lsm6dsv32x.h"
#include "drivers/movella/movella.h"
#include "drivers/timer/timer.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

// Period of IMU sampling in milliseconds
#define IMU_SAMPLING_PERIOD_MS 5

// Timeout values for freshness check (in milliseconds)
#define GYRO_FRESHNESS_TIMEOUT_MS 5
#define MAG_FRESHNESS_TIMEOUT_MS 10
#define ACCEL_FRESHNESS_TIMEOUT_MS 5
#define BARO_FRESHNESS_TIMEOUT_MS 25

// Module state tracking
typedef struct {
    bool initialized;
    uint32_t sample_count;
    uint32_t error_count;

    // Per-IMU stats
    struct {
        uint32_t success_count;
        uint32_t failure_count;
    } polulu_stats, st_stats, movella_stats;
} imu_handler_state_t;

static imu_handler_state_t imu_handler_state = {0};

/**
 * @brief Initialize all IMU hardware
 * @return Status of the initialization operation (success only if all IMUs initialize)
 */
static w_status_t initialize_all_imus(void) {
    w_status_t status = W_SUCCESS;

    if (altimu_init() != W_SUCCESS) {
        status = W_FAILURE;
    }

    if (lsm6dsv32_init() != W_SUCCESS) {
        status = W_FAILURE;
    }

    if (movella_init() != W_SUCCESS) {
        status = W_FAILURE;
    }

    imu_handler_state.initialized = (status == W_SUCCESS);
    return status;
}

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
    status |= altimu_get_mag_data(&imu_data->magnometer);

    // Read barometer data
    altimu_barometer_data_t baro_data;
    status |= altimu_get_baro_data(&baro_data);

    if (status == W_SUCCESS) {
        imu_data->barometer = baro_data.pressure;
        imu_handler_state.polulu_stats.success_count++;
    } else {
        // Zero all data if any reading fails to meet requirements
        memset(&imu_data->accelerometer, 0, sizeof(vector3d_t));
        memset(&imu_data->gyroscope, 0, sizeof(vector3d_t));
        memset(&imu_data->magnometer, 0, sizeof(vector3d_t));
        imu_data->barometer = 0.0f;

        imu_handler_state.polulu_stats.failure_count++;
    }

    return status;
}

/**
 * @brief Read data from the ST LSM6DSV32X sensor
 * @param imu_data Pointer to store the IMU data
 * @return Status of the read operation
 */
static w_status_t read_st_imu(estimator_imu_measurement_t *imu_data) {
    w_status_t status = W_SUCCESS;

    // Read accelerometer and gyroscope
    status |= lsm6dsv32_get_acc_data(&imu_data->accelerometer);
    status |= lsm6dsv32_get_gyro_data(&imu_data->gyroscope);

    if (status == W_SUCCESS) {
        imu_handler_state.st_stats.success_count++;
    } else {
        // Zero all data if any reading fails
        memset(&imu_data->accelerometer, 0, sizeof(vector3d_t));
        memset(&imu_data->gyroscope, 0, sizeof(vector3d_t));

        imu_handler_state.st_stats.failure_count++;
    }

    // ST IMU doesn't have magnetometer or barometer
    memset(&imu_data->magnometer, 0, sizeof(vector3d_t));
    imu_data->barometer = 0.0f;

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
    status = movella_get_data(&movella_data);

    if (status == W_SUCCESS) {
        // Copy data from Movella
        imu_data->accelerometer = movella_data.acc;
        imu_data->gyroscope = movella_data.gyr;
        imu_data->magnometer = movella_data.mag;
        imu_data->barometer = movella_data.pres;

        imu_handler_state.movella_stats.success_count++;
    } else {
        // Zero all data if reading fails
        memset(&imu_data->accelerometer, 0, sizeof(vector3d_t));
        memset(&imu_data->gyroscope, 0, sizeof(vector3d_t));
        memset(&imu_data->magnometer, 0, sizeof(vector3d_t));
        imu_data->barometer = 0.0f;

        imu_handler_state.movella_stats.failure_count++;
    }

    return status;
}

/**
 * @brief Initialize the IMU handler module
 * Must be called before scheduler starts
 * @return Status of initialization
 */
w_status_t imu_handler_init(void) {
    // Initialize module state
    memset(&imu_handler_state, 0, sizeof(imu_handler_state));

    // This function only initializes the module state
    // The actual IMU hardware initialization happens in the task
    return W_SUCCESS;
}

/**
 * @brief Execute one iteration of the IMU handler processing
 * Reads data from all IMUs and updates the estimator
 * @return Status of the execution
 */
w_status_t imu_handler_run(void) {
    estimator_all_imus_input_t imu_data = {0};
    float current_time_ms;

    // Get current timestamp
    if (timer_get_ms(&current_time_ms) != W_SUCCESS) {
        current_time_ms = 0.0f;
    }
    uint32_t now_ms = (uint32_t)current_time_ms;

    // Set timestamps for all IMUs
    // Note: All IMUs get the same timestamp intentionally for synchronization
    imu_data.polulu.timestamp_imu = now_ms;
    imu_data.st.timestamp_imu = now_ms;
    imu_data.movella.timestamp_imu = now_ms;

    // Read from all IMUs
    read_pololu_imu(&imu_data.polulu);
    read_st_imu(&imu_data.st);
    read_movella_imu(&imu_data.movella);

    // The estimator will determine if IMUs are "dead" based on the data provided

    // Send data to estimator with status flags
    w_status_t status = estimator_update_inputs_imu(&imu_data);
    if (status != W_SUCCESS) {
        imu_handler_state.error_count++;
    }

    imu_handler_state.sample_count++;

    // Return the status from the estimator to propagate errors upward
    return status;
}

/**
 * @brief IMU handler task function for FreeRTOS
 * Should be created during system startup
 * @param argument Task argument (unused)
 */
void imu_handler_task(void *argument) {
    (void)argument; // Unused parameter

    // Initialize all IMUs
    initialize_all_imus();

    // Main task loop
    while (1) {
        imu_handler_run();

        // Wait for next sampling period
        vTaskDelay(pdMS_TO_TICKS(IMU_SAMPLING_PERIOD_MS));
    }
}