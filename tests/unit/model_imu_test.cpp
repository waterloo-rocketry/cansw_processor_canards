#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
#include "application/estimator/estimator_types.h"
#include "application/estimator/model/jacobians.h"
#include "application/estimator/model/model_imu.h"
}

TEST(ModelImuTest, ModelMeasurementIMUCheck) {
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
    expectedResult.gyroscope = {0};
    expectedResult.magnetometer = (vector3d_t
    ){-5.81081081081081, -1.51351351351351, 12.5675675675676}; // Expected magnetometer value
    expectedResult.barometer = 89869.3545312582; // Expected barometer value

    // ACT:
    y_imu_t actualResult = model_measurement_imu(&input_estimator_state, &input_estimator_imu_data);

    // ASSERT:
    double tolerance = 1e-5;

    for (int i = 0; i < Y_IMU_SIZE_ITEMS; i++) {
        EXPECT_NEAR(actualResult.array[i], expectedResult.array[i], tolerance);
    }
}

TEST(ModelImuTest, ModelMeasurementJacobianCheck) {
    // ARRANGE:
    // Initialize test input data
    x_state_t input_estimator_state = {
        .array = {
            0.814723686393179,
            0.905791937075619,
            0.126986816293506,
            0.913375856139019,
            0.632359246225410,
            0.097540404999410,
            0.278498218867048,
            0.546881519204984,
            0.957506835434298,
            0.964888535199277,
            0.157613081677548,
            0.970592781760616,
            0.957166948242946
        }
    };
    y_imu_t input_estimator_imu_data = {
        .array = {
            4.245646529343886,
            4.669966238787753,
            3.393675774288867,
            3.788700652891667,
            3.715662340624581,
            1.961135097670841,
            3.277389450887783,
            0.855933439057809,
            3.530230440098044
        }
    };

    // Initialize expected result
    double expectedResultFlat[MEASUREMENT_MODEL_SIZE * X_STATE_SIZE_ITEMS] = {
        0,
        0,
        0,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
        3.931638554,
        8.248665933,
        -3.294687432,
        1.180036365,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        1.180036365,
        3.294687432,
        8.248665933,
        -3.931638554,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        3.294687432,
        -1.180036365,
        3.931638554,
        8.248665933,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        -12.00460892,
        0,
        0
    };

    // ACT:
    arm_matrix_instance_f64 actualResult = {0};
    model_measurement_imu_jacobian(
        &actualResult, &input_estimator_state, &input_estimator_imu_data
    );

    // ASSERT:
    double tolerance = 1e-6;

    for (int i = 0; i < MEASUREMENT_MODEL_SIZE * X_STATE_SIZE_ITEMS; i++) {
        EXPECT_NEAR(actualResult.pData[i], expectedResultFlat[i], tolerance);
    }
}
