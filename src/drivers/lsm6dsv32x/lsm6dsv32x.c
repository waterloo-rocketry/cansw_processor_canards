#include "drivers/lsm6dsv32x/lsm6dsv32x.h"
#include "drivers/lsm6dsv32x/lsm6dsv32x_regmap.h"
#include "drivers/i2c/i2c.h"

w_status_t lsm6dsv32_init(void) {
    // First verify device is present
    if (W_SUCCESS != lsm6dsv32_check_sanity()) {
        return W_FAILURE;
    }

    // TODO: Add proper initialization code
    return W_SUCCESS;
}

w_status_t lsm6dsv32_get_acc_data(vector3d_t *data) {
    (void)data;
    // TODO: Add proper accelerometer reading code
    return W_SUCCESS;
}

w_status_t lsm6dsv32_get_gyro_data(vector3d_t *data) {
    (void)data;
    // TODO: Add proper gyroscope reading code
    return W_SUCCESS;
}

w_status_t lsm6dsv32_check_sanity(void) {
    uint8_t who_am_i;
    w_status_t status;

    // Read WHO_AM_I register
    status = i2c_read_reg(I2C_BUS_4, LSM6DSV32X_I2C_ADDR, LSM6DSV32X_WHO_AM_I, &who_am_i, 1);
    if (W_SUCCESS != status) {
        return W_FAILURE;
    }

    // Verify WHO_AM_I value matches expected value
    if (LSM6DSV32X_WHO_AM_I_VALUE != who_am_i) {
        return W_FAILURE;
    }

    return W_SUCCESS;
}