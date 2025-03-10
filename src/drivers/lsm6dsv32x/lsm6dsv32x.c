#include "drivers/lsm6dsv32x/lsm6dsv32x.h"

w_status_t lsm6dsv32_init(void) {
    return W_SUCCESS;
}

w_status_t lsm6dsv32_get_acc_data(vector3d_t *data) {
    (void)data;
    return W_SUCCESS;
}

w_status_t lsm6dsv32_get_gyro_data(vector3d_t *data) {
    (void)data;
    return W_SUCCESS;
}

w_status_t lsm6dsv32_check_sanity(void) {
    return W_SUCCESS;
}