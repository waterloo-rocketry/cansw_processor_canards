#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
#include "application/estimator/estimator_types.h"
#include "application/estimator/model/model_imu.h"
}

TEST(ModelImuTest, model_measurement_imu_test) {
    // ARRANGE:
    x_state_t input_estimator_state;
    y_imu_t input_estimator_imu_data;
    y_imu_t expectedResult;

    // Initialize test input data
    input_estimator_state.attitude = (quaternion_t){1.0, 0.0, 3.0, 8.0};
    input_estimator_state.rates = (vector3d_t){1.0, 0.0, 3.0};
    input_estimator_state.altitude = 1000.0;

    input_estimator_imu_data.accelerometer = (vector3d_t){1.0, 2.0, 3.0};
    input_estimator_imu_data.gyroscope = (vector3d_t){4.0, 5.0, 6.0};
    input_estimator_imu_data.magnetometer = (vector3d_t){7.0, 8.0, 9.0};
    input_estimator_imu_data.barometer = 101325.0;

    // Initialize expected result
    expectedResult.accelerometer = (vector3d_t){5.0, 5.0, 9.0}; // Expected accelerometer value
    expectedResult.gyroscope = (vector3d_t){4.0, 5.0, 6.0};
    expectedResult.magnetometer = (vector3d_t
    ){-5.81081081081081, -1.51351351351351, 12.5675675675676}; // Expected magnetometer value
    expectedResult.barometer = 89869.3545312582; // Expected barometer value

    // ACT:
    y_imu_t actualResult =
        model_measurement_imu(&input_estimator_state, &input_estimator_imu_data);

    // ASSERT:
    double tolerance = 1e-5;

    for(int i = 0; i < Y_IMU_SIZE_ITEMS; i++) {
        EXPECT_NEAR(actualResult.array[i], expectedResult.array[i], tolerance);
    }

}
