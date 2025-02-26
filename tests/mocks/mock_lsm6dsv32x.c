#include "mock_lsm6dsv32x.h"

DEFINE_FAKE_VALUE_FUNC(w_status_t, lsm6dsv32_init);
DEFINE_FAKE_VALUE_FUNC(w_status_t, lsm6dsv32_get_acc_data, vector3d_t*);
DEFINE_FAKE_VALUE_FUNC(w_status_t, lsm6dsv32_get_gyro_data, vector3d_t*);
DEFINE_FAKE_VALUE_FUNC(w_status_t, lsm6dsv32_check_sanity);