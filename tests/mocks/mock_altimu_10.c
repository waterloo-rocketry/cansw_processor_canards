#include "mock_altimu_10.h"

// Define the FFF fakes for all altimu-10 functions
DEFINE_FAKE_VALUE_FUNC(w_status_t, altimu_init);
DEFINE_FAKE_VALUE_FUNC(w_status_t, altimu_get_acc_data, vector3d_t*);
DEFINE_FAKE_VALUE_FUNC(w_status_t, altimu_get_gyro_data, vector3d_t*);
DEFINE_FAKE_VALUE_FUNC(w_status_t, altimu_get_mag_data, vector3d_t*);
DEFINE_FAKE_VALUE_FUNC(w_status_t, altimu_get_baro_data, altimu_barometer_data_t*);
DEFINE_FAKE_VALUE_FUNC(w_status_t, altimu_check_sanity);