#include "mock_estimator.h"

// Define the FFF fakes for all estimator functions
DEFINE_FAKE_VALUE_FUNC(w_status_t, estimator_init);
DEFINE_FAKE_VALUE_FUNC(w_status_t, estimator_update_inputs_imu, estimator_all_imus_input_t*);