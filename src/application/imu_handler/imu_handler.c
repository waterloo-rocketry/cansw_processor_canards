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

