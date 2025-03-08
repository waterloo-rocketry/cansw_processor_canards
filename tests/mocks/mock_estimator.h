#ifndef MOCK_ESTIMATOR_H
#define MOCK_ESTIMATOR_H

#include "common/math/math.h"
#include "fff.h"
#include "third_party/rocketlib/include/common.h"
#include <stdint.h>

// Define the structs directly in the mock
typedef struct {
    uint32_t timestamp_imu;
    vector3d_t accelerometer;
    vector3d_t gyroscope;
    vector3d_t magnometer;
    float barometer;
} estimator_imu_measurement_t;

// measurements from all 3 imus together
typedef struct {
    estimator_imu_measurement_t polulu;
    estimator_imu_measurement_t st;
    estimator_imu_measurement_t movella;
} estimator_all_imus_input_t;

// Declare FFF fakes for all estimator functions
DECLARE_FAKE_VALUE_FUNC(w_status_t, estimator_init);
DECLARE_FAKE_VALUE_FUNC(w_status_t, estimator_update_inputs_imu, estimator_all_imus_input_t *);

#endif // MOCK_ESTIMATOR_H