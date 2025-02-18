#ifndef LSM6DSV32X_H
#define LSM6DSV32X_H

#include <stdint.h>

#include "common/math/math.h"
#include "rocketlib/include/common.h"

/**
 * @brief Initializes the imu incl all register configs
 * @note Must be called after scheduler start
 * @return Status of the operation
 */
w_status_t lsm6dsv32_init();

/**
 * @brief Retrieves accelerometer data.
 * @return Accelerometer data (gravities)
 */
w_status_t lsm6dsv32_get_acc_data(vector3d_t *data);

/**
 * @brief Retrieves gyroscope data.
 * @return Gyroscope data (deg/s)
 */
w_status_t lsm6dsv32_get_gyro_data(vector3d_t *data);

/**
 * @brief Performs a basic sanity check using the WHO_AM_I register.
 * @return Status of the operation.
 */
w_status_t lsm6dsv32_check_sanity(void);

#endif // LSM6DSV32X_H
