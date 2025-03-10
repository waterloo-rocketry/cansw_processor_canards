#ifndef MOCK_LSM6DSV32X_H
#define MOCK_LSM6DSV32X_H

#include "drivers/lsm6dsv32x/lsm6dsv32x.h"
#include "fff.h"

// Declare FFF fakes for all lsm6dsv32x functions
DECLARE_FAKE_VALUE_FUNC(w_status_t, lsm6dsv32_init);
DECLARE_FAKE_VALUE_FUNC(w_status_t, lsm6dsv32_get_acc_data, vector3d_t *);
DECLARE_FAKE_VALUE_FUNC(w_status_t, lsm6dsv32_get_gyro_data, vector3d_t *);
DECLARE_FAKE_VALUE_FUNC(w_status_t, lsm6dsv32_check_sanity);

#endif // MOCK_LSM6DSV32X_H