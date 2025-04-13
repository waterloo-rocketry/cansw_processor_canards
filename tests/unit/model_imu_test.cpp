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
    input_estimator_state.attitude = (quaternion_t){1.0f, 0.0f, 3.0f, 8.0f};
    input_estimator_state.rates = (vector3d_t){1.0f, 0.0f, 3.0f};
    input_estimator_state.altitude = 1000.0f;

    input_estimator_imu_data.accelerometer = (vector3d_t){1.0f, 2.0f, 3.0f};
    input_estimator_imu_data.gyroscope = (vector3d_t){4.0f, 5.0f, 6.0f};
    input_estimator_imu_data.magnetometer = (vector3d_t){7.0f, 8.0f, 9.0f};
    input_estimator_imu_data.barometer = 101325.0f;

    // Initialize expected result
    expectedResult.accelerometer = (vector3d_t){5.0f, 5.0f, 9.0f}; // Expected accelerometer value
    expectedResult.magnetometer = (vector3d_t
    ){-5.81081081081081f, -1.51351351351351f, 12.5675675675676f}; // Expected magnetometer value
    expectedResult.barometer = 89869.3545312582f; // Expected barometer value

    // ACT:
    y_imu_t actualResult =
        model_measurement_imu_(&input_estimator_state, &input_estimator_imu_data);

    // ASSERT:
    float tolerance = 0.05f; // 5% tolerance

    EXPECT_NEAR(
        actualResult.accelerometer.x,
        expectedResult.accelerometer.x,
        abs(expectedResult.accelerometer.x * tolerance)
    );
    EXPECT_NEAR(
        actualResult.accelerometer.y,
        expectedResult.accelerometer.y,
        abs(expectedResult.accelerometer.y * tolerance)
    );
    EXPECT_NEAR(
        actualResult.accelerometer.z,
        expectedResult.accelerometer.z,
        abs(expectedResult.accelerometer.z * tolerance)
    );

    EXPECT_NEAR(
        actualResult.magnetometer.x,
        expectedResult.magnetometer.x,
        abs(expectedResult.magnetometer.x * tolerance)
    );
    EXPECT_NEAR(
        actualResult.magnetometer.y,
        expectedResult.magnetometer.y,
        abs(expectedResult.magnetometer.y * tolerance)
    );
    EXPECT_NEAR(
        actualResult.magnetometer.z,
        expectedResult.magnetometer.z,
        abs(expectedResult.magnetometer.z * tolerance)
    );

    EXPECT_NEAR(
        actualResult.barometer, expectedResult.barometer, abs(expectedResult.barometer * tolerance)
    );
}