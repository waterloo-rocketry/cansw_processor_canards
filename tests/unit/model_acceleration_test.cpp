#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
#include "application/estimator/estimator_types.h"
#include "application/estimator/model/model_acceleration.h"
}

/**
 * all test expected vals generated from matlab 27d11db
 */

TEST(ModelAccelerationTest, BasicTestTwoAlive) {
    // Arrange
    x_state_t input_state = {0};
    y_imu_t input_imu_1 = {0};
    y_imu_t input_imu_2 = {0};

    input_state.rates = (vector3d_t){0.1, -0.2, 0.3};

    input_imu_1.accelerometer = (vector3d_t){1.0, 2.0, 3.0};
    input_imu_2.accelerometer = (vector3d_t){4.0, 5.0, 6.0};

    bool is_dead_1 = false;
    bool is_dead_2 = false;

    // Act
    vector3d_t actual_result =
        model_acceleration(&input_state, &input_imu_1, is_dead_1, &input_imu_2, is_dead_2);

    // Assert
    double tolerance = 1e-6;

    double expected_x = 2.482400000000000;
    double expected_y = 3.489910000000000;
    double expected_z = 4.499140000000000;
    
    EXPECT_NEAR(actual_result.x, expected_x, tolerance);
    EXPECT_NEAR(actual_result.y, expected_y, tolerance);
    EXPECT_NEAR(actual_result.z, expected_z, tolerance);
}

TEST(ModelAccelerationTest, OnlyOneAlive_IMU1Alive) {
    // Arrange
    x_state_t input_state = {0};
    y_imu_t input_imu_1 = {0};
    y_imu_t input_imu_2 = {0};

    input_state.rates = (vector3d_t){0.1, 0.2, -0.1};

    input_imu_1.accelerometer = (vector3d_t){-1.0, 0.0, 1.0};
    input_imu_2.accelerometer = (vector3d_t){100.0, 200.0, 300.0}; // Should be ignored

    bool is_dead_1 = false;
    bool is_dead_2 = true;

    // Act
    vector3d_t actual_result =
        model_acceleration(&input_state, &input_imu_1, is_dead_1, &input_imu_2, is_dead_2);

    // Assert
    double tolerance = 1e-5;

    double expected_x = -1.004600000000000;
    double expected_y = 0.001600000000000;
    double expected_z = 0.998600000000000;
    

    EXPECT_NEAR(actual_result.x, expected_x, tolerance);
    EXPECT_NEAR(actual_result.y, expected_y, tolerance);
    EXPECT_NEAR(actual_result.z, expected_z, tolerance);
}

TEST(ModelAccelerationTest, OnlyOneAlive_IMU2Alive) {
    // Arrange
    x_state_t input_state = {0};
    y_imu_t input_imu_1 = {0};
    y_imu_t input_imu_2 = {0};

    input_state.rates = (vector3d_t){-0.2, 0.1, 0.4};

    input_imu_1.accelerometer = (vector3d_t){50.0, -50.0, 0.0}; // Should be ignored
    input_imu_2.accelerometer = (vector3d_t){0.5, -0.5, 1.5};

    bool is_dead_1 = true;
    bool is_dead_2 = false;

    // Act
    vector3d_t actual_result =
        model_acceleration(&input_state, &input_imu_1, is_dead_1, &input_imu_2, is_dead_2);

    // Assert
    double tolerance = 1e-6;

    double expected_x = 0.473350000000000;
    double expected_y = -0.513660000000000;
    double expected_z = 1.490090000000000;


    EXPECT_NEAR(actual_result.x, expected_x, tolerance);
    EXPECT_NEAR(actual_result.y, expected_y, tolerance);
    EXPECT_NEAR(actual_result.z, expected_z, tolerance);
}

TEST(ModelAccelerationTest, AllDead) {
    // Arrange
    x_state_t input_state = {0};
    y_imu_t input_imu_1 = {0};
    y_imu_t input_imu_2 = {0};

    input_state.rates = (vector3d_t){0.0, 0.0, 0.0};

    input_imu_1.accelerometer = (vector3d_t){1.0, 2.0, 3.0};
    input_imu_2.accelerometer = (vector3d_t){4.0, 5.0, 6.0};

    bool is_dead_1 = true;
    bool is_dead_2 = true;

    // Act
    vector3d_t actual_result =
        model_acceleration(&input_state, &input_imu_1, is_dead_1, &input_imu_2, is_dead_2);

    // Assert
    double tolerance = 1e-5;

    EXPECT_NEAR(actual_result.x, 0.0, tolerance);
    EXPECT_NEAR(actual_result.y, 0.0, tolerance);
    EXPECT_NEAR(actual_result.z, 0.0, tolerance);
}
