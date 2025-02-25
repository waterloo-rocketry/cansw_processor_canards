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
static struct {
    bool initialized;
    uint32_t sample_count;
    uint32_t error_count;

    // Per-IMU stats
    struct {
        uint32_t success_count;
        uint32_t failure_count;
    } polulu_stats, st_stats, movella_stats;
} state = {0};

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
        state.polulu_stats.success_count++;
    } else {
        // Zero all data if any reading fails to meet requirements in 7e
        memset(&imu_data->accelerometer, 0, sizeof(vector3d_t));
        memset(&imu_data->gyroscope, 0, sizeof(vector3d_t));
        memset(&imu_data->magnometer, 0, sizeof(vector3d_t));
        imu_data->barometer = 0.0f;
        state.polulu_stats.failure_count++;
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
        state.st_stats.success_count++;
    } else {
        // Zero all data if any reading fails
        memset(&imu_data->accelerometer, 0, sizeof(vector3d_t));
        memset(&imu_data->gyroscope, 0, sizeof(vector3d_t));
        state.st_stats.failure_count++;
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
    movella_data_t movella_data;
    status = movella_get_data(&movella_data);

    if (status == W_SUCCESS) {
        // Copy data from Movella
        imu_data->accelerometer = movella_data.acc;
        imu_data->gyroscope = movella_data.gyr;
        imu_data->magnometer = movella_data.mag;
        imu_data->barometer = movella_data.pres;
        state.movella_stats.success_count++;
    } else {
        // Zero all data if reading fails
        memset(&imu_data->accelerometer, 0, sizeof(vector3d_t));
        memset(&imu_data->gyroscope, 0, sizeof(vector3d_t));
        memset(&imu_data->magnometer, 0, sizeof(vector3d_t));
        imu_data->barometer = 0.0f;
        state.movella_stats.failure_count++;
    }

    return status;
}

w_status_t imu_handler_init(void) {
    // Initialize module state
    memset(&state, 0, sizeof(state));

    // This function only initializes the module state
    // The actual IMU hardware initialization happens in the task
    // because some IMU drivers require RTOS to be running

    return W_SUCCESS;
}

void imu_handler_task(void *argument) {
    (void)argument; // Unused parameter

    // Initialize all IMUs
    w_status_t status = W_SUCCESS;

    status |= altimu_init();
    status |= lsm6dsv32_init();
    status |= movella_init();

    state.initialized = (status == W_SUCCESS);

    // Even if initialization fails, continue attempting operation
    // IMUs might connect later or be reset

    estimator_all_imus_input_t imu_data;
    float current_time_ms;

    // Main task loop
    while (1) {
        // Wait for the next sampling period
        vTaskDelay(pdMS_TO_TICKS(IMU_SAMPLING_PERIOD_MS));

        // Get current timestamp
        timer_get_ms(&current_time_ms);
        uint32_t now_ms = (uint32_t)current_time_ms;

        // Set timestamps for all IMUs
        imu_data.polulu.timestamp_imu = now_ms;
        imu_data.st.timestamp_imu = now_ms;
        imu_data.movella.timestamp_imu = now_ms;

        // Read from all IMUS
        // Since all readings happen in quick succession, they satisfy
        // the "same timeframe" requirement in 7f
        read_pololu_imu(&imu_data.polulu);
        read_st_imu(&imu_data.st);
        read_movella_imu(&imu_data.movella);

        // Send data to estimator
        status = estimator_update_inputs_imu(&imu_data);
        if (status != W_SUCCESS) {
            state.error_count++;
        }
        state.sample_count++;
    }
}
