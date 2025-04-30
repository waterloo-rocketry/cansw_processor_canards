#ifndef ALTIMU_10_H
#define ALTIMU_10_H

#include <stdint.h>

#include "common/math/math.h"
#include "rocketlib/include/common.h"

typedef struct {
    float pressure; // pascal
    float temperature; // celsius
} altimu_barometer_data_t;

/**
 * raw data from barometer registers
 */
typedef struct {
    uint32_t pressure;
    uint16_t temperature;
} altimu_raw_baro_data_t;

/**
 * raw data from imu/mag registers
 */
typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z;
} altimu_raw_imu_data_t;

// Function prototypes

/**
 * @brief Initializes the Pololu AltIMU incl all register configs
 * @note Must be called after scheduler start
 * @return Status of the operation
 */
w_status_t altimu_init();

/**
 * @brief Retrieves accelerometer data.
 * @param data Pointer to store the converted accelerometer data (gravities)
 * @param raw_data Pointer to store the raw register readings
 */
w_status_t altimu_get_acc_data(vector3d_t *data, altimu_raw_imu_data_t *raw_data);

/**
 * @brief Retrieves gyroscope data.
 * @param data Pointer to store the converted gyroscope data (deg/s)
 * @param raw_data Pointer to store the raw register readings
 */
w_status_t altimu_get_gyro_data(vector3d_t *data, altimu_raw_imu_data_t *raw_data);

/**
 * @brief Retrieves magnetometer data.
 * @param data Pointer to store the converted magnetometer data (gauss)
 * @param raw_data Pointer to store the raw register readings
 */
w_status_t altimu_get_mag_data(vector3d_t *data, altimu_raw_imu_data_t *raw_data);

/**
 * @brief Retrieves barometer data.
 * @param data Pointer to store the converted barometer data (pascal, celsius)
 * @param raw_data Pointer to store the raw register readings
 */
w_status_t altimu_get_baro_data(altimu_barometer_data_t *data, altimu_raw_baro_data_t *raw_data);

/**
 * @brief Performs a basic sanity check using the WHO_AM_I register.
 * @return W_SUCCESS if imu,mag,baro are all detected correctly
 */
w_status_t altimu_check_sanity(void);

#endif // ALTIMU_10_H