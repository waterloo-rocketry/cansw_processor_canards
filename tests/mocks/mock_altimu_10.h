#ifndef MOCK_ALTIMU_10_H
#define MOCK_ALTIMU_10_H

#include "common/math/math.h"
#include "fff.h"
#include "third_party/rocketlib/include/common.h"

// Include the struct definition from the real header
typedef struct {
    float pressure; // pascal
    float temperature; // celsius
} altimu_barometer_data_t;

// Declare FFF fakes for all altimu-10 functions
DECLARE_FAKE_VALUE_FUNC(w_status_t, altimu_init);
DECLARE_FAKE_VALUE_FUNC(w_status_t, altimu_get_acc_data, vector3d_t *);
DECLARE_FAKE_VALUE_FUNC(w_status_t, altimu_get_gyro_data, vector3d_t *);
DECLARE_FAKE_VALUE_FUNC(w_status_t, altimu_get_mag_data, vector3d_t *);
DECLARE_FAKE_VALUE_FUNC(w_status_t, altimu_get_baro_data, altimu_barometer_data_t *);
DECLARE_FAKE_VALUE_FUNC(w_status_t, altimu_check_sanity);

#endif // MOCK_ALTIMU_10_H