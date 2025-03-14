#ifndef MOCK_ALTIMU_10_H
#define MOCK_ALTIMU_10_H

#include "drivers/altimu-10/altimu-10.h"
#include "fff.h"

FAKE_VALUE_FUNC(w_status_t, altimu_init);
FAKE_VALUE_FUNC(w_status_t, altimu_get_acc_data, vector3d_t *);
FAKE_VALUE_FUNC(w_status_t, altimu_get_gyro_data, vector3d_t *);
FAKE_VALUE_FUNC(w_status_t, altimu_get_mag_data, vector3d_t *);
FAKE_VALUE_FUNC(w_status_t, altimu_get_baro_data, altimu_barometer_data_t *);
FAKE_VALUE_FUNC(w_status_t, altimu_check_sanity);

#endif // MOCK_ALTIMU_10_H