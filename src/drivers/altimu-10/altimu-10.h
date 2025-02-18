#ifndef ALTIMU_10_H
#define ALTIMU_10_H

#include <stdint.h>

#include "common/math/math.h"
#include "rocketlib/include/common.h"

typedef struct {
    float pressure; // pascal
    float temperature; // celsius
} altimu_barometer_data_t;

// Function prototypes

/**
 * @brief Initializes the Pololu AltIMU incl all register configs
 * @note Must be called after scheduler start
 * @return Status of the operation
 */
w_status_t altimu_init();

/**
 * @brief Retrieves accelerometer data.
 * @return Accelerometer data (gravities)
 */
w_status_t altimu_get_acc_data(vector3d_t *data);

/**
 * @brief Retrieves gyroscope data.
 * @return Gyroscope data (deg/s)
 */
w_status_t altimu_get_gyro_data(vector3d_t *data);

/**
 * @brief Retrieves magnetometer data.
 * @return Magnetometer data (gauss)
 */
w_status_t altimu_get_mag_data(vector3d_t *data);

/**
 * @brief Retrieves barometer data.
 * @return Barometer data (pascal, celsius)
 */
w_status_t altimu_get_baro_data(altimu_barometer_data_t *data);

/**
 * @brief Performs a basic sanity check using the WHO_AM_I register.
 * @return Status of the operation.
 */
w_status_t altimu_check_sanity(void);

#endif // ALTIMU_10_H