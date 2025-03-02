#include "fff.h"
#include <gtest/gtest.h>
#include <string.h>

extern "C"
{
#include "application/imu_handler/imu_handler.h"
#include "common/math/math.h"
#include "mock_altimu_10.h"
#include "mock_lsm6dsv32x.h"
#include "mock_movella.h"
#include "mock_timer.h"
#include "mock_estimator.h"
#include "third_party/rocketlib/include/common.h"
}

// Static buffer for IMU data capture in tests
static estimator_all_imus_input_t captured_data;

// Add expected test data values
static const vector3d_t EXPECTED_ACC = {1.0f, 2.0f, 3.0f};
static const vector3d_t EXPECTED_GYRO = {4.0f, 5.0f, 6.0f};
static const vector3d_t EXPECTED_MAG = {7.0f, 8.0f, 9.0f};
static const float EXPECTED_BARO = 101325.0f; // Standard atmospheric pressure in Pa

// Helper functions for setting up test data
static w_status_t timer_get_ms_custom_fake(float *time_ms)
{
    *time_ms = 1000.0f;
    return W_SUCCESS;
}

static w_status_t altimu_get_acc_data_success(vector3d_t *acc)
{
    *acc = EXPECTED_ACC;
    return W_SUCCESS;
}

static w_status_t altimu_get_gyro_data_success(vector3d_t *gyro)
{
    *gyro = EXPECTED_GYRO;
    return W_SUCCESS;
}

static w_status_t altimu_get_mag_data_success(vector3d_t *mag)
{
    *mag = EXPECTED_MAG;
    return W_SUCCESS;
}

static w_status_t altimu_get_baro_data_success(altimu_barometer_data_t *baro)
{
    baro->pressure = EXPECTED_BARO;
    baro->temperature = 25.0f;
    return W_SUCCESS;
}

static w_status_t lsm6dsv32_get_acc_data_success(vector3d_t *acc)
{
    *acc = EXPECTED_ACC;
    return W_SUCCESS;
}

static w_status_t lsm6dsv32_get_gyro_data_success(vector3d_t *gyro)
{
    *gyro = EXPECTED_GYRO;
    return W_SUCCESS;
}

static w_status_t movella_get_data_success(movella_data_t *data)
{
    data->acc = EXPECTED_ACC;
    data->gyr = EXPECTED_GYRO;
    data->mag = EXPECTED_MAG;
    data->pres = EXPECTED_BARO;
    data->temp = 25.0f;
    return W_SUCCESS;
}

static w_status_t estimator_update_capture(estimator_all_imus_input_t *data)
{
    memcpy(&captured_data, data, sizeof(estimator_all_imus_input_t));
    return W_SUCCESS;
}

class ImuHandlerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Reset all fakes before each test
        RESET_FAKE(altimu_init);
        RESET_FAKE(altimu_get_acc_data);
        RESET_FAKE(altimu_get_gyro_data);
        RESET_FAKE(altimu_get_mag_data);
        RESET_FAKE(altimu_get_baro_data);

        RESET_FAKE(lsm6dsv32_init);
        RESET_FAKE(lsm6dsv32_get_acc_data);
        RESET_FAKE(lsm6dsv32_get_gyro_data);

        RESET_FAKE(movella_init);
        RESET_FAKE(movella_get_data);

        RESET_FAKE(estimator_update_inputs_imu);
        RESET_FAKE(timer_get_ms);

        // Default successful returns
        altimu_init_fake.return_val = W_SUCCESS;
        lsm6dsv32_init_fake.return_val = W_SUCCESS;
        movella_init_fake.return_val = W_SUCCESS;
        timer_get_ms_fake.return_val = W_SUCCESS;
        estimator_update_inputs_imu_fake.return_val = W_SUCCESS;

        // Clear captured data
        memset(&captured_data, 0, sizeof(captured_data));

        // Initialize IMU handler before each test
        imu_handler_init();
    }
};

// Tests for initialization
TEST_F(ImuHandlerTest, InitSuccess)
{
    EXPECT_EQ(W_SUCCESS, imu_handler_init());
}

// Test successful run with all IMUs working
TEST_F(ImuHandlerTest, RunSuccessful)
{
    // Set up all mocks for successful readings
    altimu_get_acc_data_fake.custom_fake = altimu_get_acc_data_success;
    altimu_get_gyro_data_fake.custom_fake = altimu_get_gyro_data_success;
    altimu_get_mag_data_fake.custom_fake = altimu_get_mag_data_success;
    altimu_get_baro_data_fake.custom_fake = altimu_get_baro_data_success;

    lsm6dsv32_get_acc_data_fake.custom_fake = lsm6dsv32_get_acc_data_success;
    lsm6dsv32_get_gyro_data_fake.custom_fake = lsm6dsv32_get_gyro_data_success;

    movella_get_data_fake.custom_fake = movella_get_data_success;

    timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;
    estimator_update_inputs_imu_fake.custom_fake = estimator_update_capture;

    // Run the function under test
    w_status_t result = imu_handler_run();

    // Verify function returned success
    EXPECT_EQ(W_SUCCESS, result);

    // Verify IMU read calls were made
    EXPECT_EQ(1, altimu_get_acc_data_fake.call_count);
    EXPECT_EQ(1, altimu_get_gyro_data_fake.call_count);
    EXPECT_EQ(1, altimu_get_mag_data_fake.call_count);
    EXPECT_EQ(1, altimu_get_baro_data_fake.call_count);
    EXPECT_EQ(1, lsm6dsv32_get_acc_data_fake.call_count);
    EXPECT_EQ(1, lsm6dsv32_get_gyro_data_fake.call_count);
    EXPECT_EQ(1, movella_get_data_fake.call_count);

    // Verify timestamps
    EXPECT_EQ(1000, captured_data.polulu.timestamp_imu);
    EXPECT_EQ(1000, captured_data.st.timestamp_imu);
    EXPECT_EQ(1000, captured_data.movella.timestamp_imu);

    // Verify data values for Polulu
    EXPECT_FLOAT_EQ(EXPECTED_ACC.component.x, captured_data.polulu.accelerometer.component.x);
    EXPECT_FLOAT_EQ(EXPECTED_ACC.component.y, captured_data.polulu.accelerometer.component.y);
    EXPECT_FLOAT_EQ(EXPECTED_ACC.component.z, captured_data.polulu.accelerometer.component.z);

    EXPECT_FLOAT_EQ(EXPECTED_GYRO.component.x, captured_data.polulu.gyroscope.component.x);
    EXPECT_FLOAT_EQ(EXPECTED_GYRO.component.y, captured_data.polulu.gyroscope.component.y);
    EXPECT_FLOAT_EQ(EXPECTED_GYRO.component.z, captured_data.polulu.gyroscope.component.z);

    EXPECT_FLOAT_EQ(EXPECTED_MAG.component.x, captured_data.polulu.magnometer.component.x);
    EXPECT_FLOAT_EQ(EXPECTED_MAG.component.y, captured_data.polulu.magnometer.component.y);
    EXPECT_FLOAT_EQ(EXPECTED_MAG.component.z, captured_data.polulu.magnometer.component.z);

    EXPECT_FLOAT_EQ(EXPECTED_BARO, captured_data.polulu.barometer);

    // Verify ST IMU data
    EXPECT_FLOAT_EQ(EXPECTED_ACC.component.x, captured_data.st.accelerometer.component.x);
    EXPECT_FLOAT_EQ(EXPECTED_ACC.component.y, captured_data.st.accelerometer.component.y);
    EXPECT_FLOAT_EQ(EXPECTED_ACC.component.z, captured_data.st.accelerometer.component.z);

    EXPECT_FLOAT_EQ(EXPECTED_GYRO.component.x, captured_data.st.gyroscope.component.x);
    EXPECT_FLOAT_EQ(EXPECTED_GYRO.component.y, captured_data.st.gyroscope.component.y);
    EXPECT_FLOAT_EQ(EXPECTED_GYRO.component.z, captured_data.st.gyroscope.component.z);

    // Verify Movella data
    EXPECT_FLOAT_EQ(EXPECTED_ACC.component.x, captured_data.movella.accelerometer.component.x);
    EXPECT_FLOAT_EQ(EXPECTED_ACC.component.y, captured_data.movella.accelerometer.component.y);
    EXPECT_FLOAT_EQ(EXPECTED_ACC.component.z, captured_data.movella.accelerometer.component.z);

    EXPECT_FLOAT_EQ(EXPECTED_GYRO.component.x, captured_data.movella.gyroscope.component.x);
    EXPECT_FLOAT_EQ(EXPECTED_GYRO.component.y, captured_data.movella.gyroscope.component.y);
    EXPECT_FLOAT_EQ(EXPECTED_GYRO.component.z, captured_data.movella.gyroscope.component.z);

    EXPECT_FLOAT_EQ(EXPECTED_MAG.component.x, captured_data.movella.magnometer.component.x);
    EXPECT_FLOAT_EQ(EXPECTED_MAG.component.y, captured_data.movella.magnometer.component.y);
    EXPECT_FLOAT_EQ(EXPECTED_MAG.component.z, captured_data.movella.magnometer.component.z);

    EXPECT_FLOAT_EQ(EXPECTED_BARO, captured_data.movella.barometer);
}

// Test with failed Polulu IMU
TEST_F(ImuHandlerTest, RunWithPoluluFailure)
{
    // Set Polulu to fail
    altimu_get_acc_data_fake.return_val = W_FAILURE;
    altimu_get_gyro_data_fake.return_val = W_FAILURE;
    altimu_get_mag_data_fake.return_val = W_FAILURE;
    altimu_get_baro_data_fake.return_val = W_FAILURE;

    // Set other IMUs to succeed
    lsm6dsv32_get_acc_data_fake.custom_fake = lsm6dsv32_get_acc_data_success;
    lsm6dsv32_get_gyro_data_fake.custom_fake = lsm6dsv32_get_gyro_data_success;
    movella_get_data_fake.custom_fake = movella_get_data_success;

    timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;
    estimator_update_inputs_imu_fake.custom_fake = estimator_update_capture;

    // Run the function under test
    w_status_t result = imu_handler_run();

    // Function should still return success
    EXPECT_EQ(W_SUCCESS, result);

    // Verify Polulu data is zeroed
    EXPECT_FLOAT_EQ(0.0f, captured_data.polulu.accelerometer.component.x);
    EXPECT_FLOAT_EQ(0.0f, captured_data.polulu.gyroscope.component.x);
    EXPECT_FLOAT_EQ(0.0f, captured_data.polulu.magnometer.component.x);
    EXPECT_FLOAT_EQ(0.0f, captured_data.polulu.barometer);

    // Verify other IMU data is still correct
    EXPECT_FLOAT_EQ(EXPECTED_ACC.component.x, captured_data.st.accelerometer.component.x);
    EXPECT_FLOAT_EQ(EXPECTED_GYRO.component.x, captured_data.st.gyroscope.component.x);

    EXPECT_FLOAT_EQ(EXPECTED_ACC.component.x, captured_data.movella.accelerometer.component.x);
    EXPECT_FLOAT_EQ(EXPECTED_GYRO.component.x, captured_data.movella.gyroscope.component.x);
    EXPECT_FLOAT_EQ(EXPECTED_MAG.component.x, captured_data.movella.magnometer.component.x);
    EXPECT_FLOAT_EQ(EXPECTED_BARO, captured_data.movella.barometer);
}

// Test with failed ST IMU
TEST_F(ImuHandlerTest, RunWithStImuFailure)
{
    // Set Polulu to succeed
    altimu_get_acc_data_fake.custom_fake = altimu_get_acc_data_success;
    altimu_get_gyro_data_fake.custom_fake = altimu_get_gyro_data_success;
    altimu_get_mag_data_fake.custom_fake = altimu_get_mag_data_success;
    altimu_get_baro_data_fake.custom_fake = altimu_get_baro_data_success;

    // Set ST IMU to fail
    lsm6dsv32_get_acc_data_fake.return_val = W_FAILURE;
    lsm6dsv32_get_gyro_data_fake.return_val = W_FAILURE;

    // Set Movella to succeed
    movella_get_data_fake.custom_fake = movella_get_data_success;

    timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;
    estimator_update_inputs_imu_fake.custom_fake = estimator_update_capture;

    // Run the function under test
    w_status_t result = imu_handler_run();

    // Function should still return success
    EXPECT_EQ(W_SUCCESS, result);

    // Verify ST IMU data is zeroed
    EXPECT_FLOAT_EQ(0.0f, captured_data.st.accelerometer.component.x);
    EXPECT_FLOAT_EQ(0.0f, captured_data.st.gyroscope.component.x);

    // Verify other IMU data is still correct
    EXPECT_FLOAT_EQ(EXPECTED_ACC.component.x, captured_data.polulu.accelerometer.component.x);
    EXPECT_FLOAT_EQ(EXPECTED_GYRO.component.x, captured_data.polulu.gyroscope.component.x);
    EXPECT_FLOAT_EQ(EXPECTED_MAG.component.x, captured_data.polulu.magnometer.component.x);
    EXPECT_FLOAT_EQ(EXPECTED_BARO, captured_data.polulu.barometer);

    EXPECT_FLOAT_EQ(EXPECTED_ACC.component.x, captured_data.movella.accelerometer.component.x);
    EXPECT_FLOAT_EQ(EXPECTED_GYRO.component.x, captured_data.movella.gyroscope.component.x);
    EXPECT_FLOAT_EQ(EXPECTED_MAG.component.x, captured_data.movella.magnometer.component.x);
    EXPECT_FLOAT_EQ(EXPECTED_BARO, captured_data.movella.barometer);
}

// Test with failed Movella IMU
TEST_F(ImuHandlerTest, RunWithMovellaFailure)
{
    // Set Polulu to succeed
    altimu_get_acc_data_fake.custom_fake = altimu_get_acc_data_success;
    altimu_get_gyro_data_fake.custom_fake = altimu_get_gyro_data_success;
    altimu_get_mag_data_fake.custom_fake = altimu_get_mag_data_success;
    altimu_get_baro_data_fake.custom_fake = altimu_get_baro_data_success;

    // Set ST IMU to succeed
    lsm6dsv32_get_acc_data_fake.custom_fake = lsm6dsv32_get_acc_data_success;
    lsm6dsv32_get_gyro_data_fake.custom_fake = lsm6dsv32_get_gyro_data_success;

    // Set Movella to fail
    movella_get_data_fake.return_val = W_FAILURE;

    timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;
    estimator_update_inputs_imu_fake.custom_fake = estimator_update_capture;

    // Run the function under test
    w_status_t result = imu_handler_run();

    // Function should still return success
    EXPECT_EQ(W_SUCCESS, result);

    // Verify Movella data is zeroed
    EXPECT_FLOAT_EQ(0.0f, captured_data.movella.accelerometer.component.x);
    EXPECT_FLOAT_EQ(0.0f, captured_data.movella.gyroscope.component.x);
    EXPECT_FLOAT_EQ(0.0f, captured_data.movella.magnometer.component.x);
    EXPECT_FLOAT_EQ(0.0f, captured_data.movella.barometer);

    // Verify other IMU data is still correct
    EXPECT_FLOAT_EQ(EXPECTED_ACC.component.x, captured_data.polulu.accelerometer.component.x);
    EXPECT_FLOAT_EQ(EXPECTED_GYRO.component.x, captured_data.polulu.gyroscope.component.x);
    EXPECT_FLOAT_EQ(EXPECTED_MAG.component.x, captured_data.polulu.magnometer.component.x);
    EXPECT_FLOAT_EQ(EXPECTED_BARO, captured_data.polulu.barometer);

    EXPECT_FLOAT_EQ(EXPECTED_ACC.component.x, captured_data.st.accelerometer.component.x);
    EXPECT_FLOAT_EQ(EXPECTED_GYRO.component.x, captured_data.st.gyroscope.component.x);
}

// Test with all IMUs failing
TEST_F(ImuHandlerTest, RunWithAllImusFailure)
{
    // Set all IMUs to fail
    altimu_get_acc_data_fake.return_val = W_FAILURE;
    altimu_get_gyro_data_fake.return_val = W_FAILURE;
    altimu_get_mag_data_fake.return_val = W_FAILURE;
    altimu_get_baro_data_fake.return_val = W_FAILURE;

    lsm6dsv32_get_acc_data_fake.return_val = W_FAILURE;
    lsm6dsv32_get_gyro_data_fake.return_val = W_FAILURE;

    movella_get_data_fake.return_val = W_FAILURE;

    timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;
    estimator_update_inputs_imu_fake.custom_fake = estimator_update_capture;

    // Run the function under test
    w_status_t result = imu_handler_run();

    // Function should still return success (estimator decides what to do)
    EXPECT_EQ(W_SUCCESS, result);

    // Verify all IMU data is zeroed
    EXPECT_FLOAT_EQ(0.0f, captured_data.polulu.accelerometer.component.x);
    EXPECT_FLOAT_EQ(0.0f, captured_data.polulu.gyroscope.component.x);
    EXPECT_FLOAT_EQ(0.0f, captured_data.polulu.magnometer.component.x);
    EXPECT_FLOAT_EQ(0.0f, captured_data.polulu.barometer);

    EXPECT_FLOAT_EQ(0.0f, captured_data.st.accelerometer.component.x);
    EXPECT_FLOAT_EQ(0.0f, captured_data.st.gyroscope.component.x);

    EXPECT_FLOAT_EQ(0.0f, captured_data.movella.accelerometer.component.x);
    EXPECT_FLOAT_EQ(0.0f, captured_data.movella.gyroscope.component.x);
    EXPECT_FLOAT_EQ(0.0f, captured_data.movella.magnometer.component.x);
    EXPECT_FLOAT_EQ(0.0f, captured_data.movella.barometer);
}

// Test behavior when timer fails
TEST_F(ImuHandlerTest, RunWithTimerFailure)
{
    // Set up all IMUs for success
    altimu_get_acc_data_fake.custom_fake = altimu_get_acc_data_success;
    altimu_get_gyro_data_fake.custom_fake = altimu_get_gyro_data_success;
    altimu_get_mag_data_fake.custom_fake = altimu_get_mag_data_success;
    altimu_get_baro_data_fake.custom_fake = altimu_get_baro_data_success;

    lsm6dsv32_get_acc_data_fake.custom_fake = lsm6dsv32_get_acc_data_success;
    lsm6dsv32_get_gyro_data_fake.custom_fake = lsm6dsv32_get_gyro_data_success;

    movella_get_data_fake.custom_fake = movella_get_data_success;

    // Set timer to fail
    timer_get_ms_fake.return_val = W_FAILURE;

    estimator_update_inputs_imu_fake.custom_fake = estimator_update_capture;

    // Run the function under test
    w_status_t result = imu_handler_run();

    // Function should still return success
    EXPECT_EQ(W_SUCCESS, result);

    // Verify timestamps are zero
    EXPECT_EQ(0, captured_data.polulu.timestamp_imu);
    EXPECT_EQ(0, captured_data.st.timestamp_imu);
    EXPECT_EQ(0, captured_data.movella.timestamp_imu);

    // But IMU data should still be valid
    EXPECT_FLOAT_EQ(EXPECTED_ACC.component.x, captured_data.polulu.accelerometer.component.x);
}

// Test behavior when estimator update fails
TEST_F(ImuHandlerTest, RunWithEstimatorFailure)
{
    // Set up all IMUs for success
    altimu_get_acc_data_fake.custom_fake = altimu_get_acc_data_success;
    altimu_get_gyro_data_fake.custom_fake = altimu_get_gyro_data_success;
    altimu_get_mag_data_fake.custom_fake = altimu_get_mag_data_success;
    altimu_get_baro_data_fake.custom_fake = altimu_get_baro_data_success;

    lsm6dsv32_get_acc_data_fake.custom_fake = lsm6dsv32_get_acc_data_success;
    lsm6dsv32_get_gyro_data_fake.custom_fake = lsm6dsv32_get_gyro_data_success;

    movella_get_data_fake.custom_fake = movella_get_data_success;

    timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;

    // Set estimator update to fail
    estimator_update_inputs_imu_fake.return_val = W_FAILURE;

    // Run the function under test
    w_status_t result = imu_handler_run();

    // Function should return the failure from estimator
    EXPECT_EQ(W_SUCCESS, result);

    // Verify IMU data was collected normally despite estimator failure
    EXPECT_EQ(1, altimu_get_acc_data_fake.call_count);
    EXPECT_EQ(1, altimu_get_gyro_data_fake.call_count);
    EXPECT_EQ(1, altimu_get_mag_data_fake.call_count);
    EXPECT_EQ(1, altimu_get_baro_data_fake.call_count);
    EXPECT_EQ(1, lsm6dsv32_get_acc_data_fake.call_count);
    EXPECT_EQ(1, lsm6dsv32_get_gyro_data_fake.call_count);
    EXPECT_EQ(1, movella_get_data_fake.call_count);
}