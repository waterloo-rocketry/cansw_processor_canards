#include "fff.h"
#include <gtest/gtest.h>
#include <string.h>

extern "C" {
#include "application/imu_handler/imu_handler.h"
#include "common/math/math.h"
#include "drivers/movella/movella.h"
#include "mock_altimu_10.h"
#include "mock_estimator.h"
#include "mock_freertos.h"
#include "mock_lsm6dsv32x.h"
#include "mock_movella.h"
#include "mock_timer.h"
#include "third_party/rocketlib/include/common.h"
}

// Static buffer for IMU data capture in tests
static estimator_all_imus_input_t *g_captured_imu_data = NULL;

// Add expected test data values
static const vector3d_t EXPECTED_ACC = {1.0f, 2.0f, 3.0f};
static const vector3d_t EXPECTED_GYRO = {4.0f, 5.0f, 6.0f};
static const vector3d_t EXPECTED_MAG = {7.0f, 8.0f, 9.0f};
static const float EXPECTED_BARO = 101325.0f; // Standard atmospheric pressure in Pa

// Helper functions for setting up test data
static w_status_t timer_get_ms_custom_fake(float *time_ms) {
    *time_ms = 1000.0f;
    return W_SUCCESS;
}

static w_status_t altimu_get_acc_data_success(vector3d_t *acc) {
    *acc = EXPECTED_ACC;
    return W_SUCCESS;
}

static w_status_t altimu_get_gyro_data_success(vector3d_t *gyro) {
    *gyro = EXPECTED_GYRO;
    return W_SUCCESS;
}

static w_status_t altimu_get_mag_data_success(vector3d_t *mag) {
    *mag = EXPECTED_MAG;
    return W_SUCCESS;
}

static w_status_t altimu_get_baro_data_success(altimu_barometer_data_t *baro) {
    baro->pressure = EXPECTED_BARO;
    baro->temperature = 25.0f;
    return W_SUCCESS;
}

static w_status_t lsm6dsv32_get_acc_data_success(vector3d_t *acc) {
    *acc = EXPECTED_ACC;
    return W_SUCCESS;
}

static w_status_t lsm6dsv32_get_gyro_data_success(vector3d_t *gyro) {
    *gyro = EXPECTED_GYRO;
    return W_SUCCESS;
}

static w_status_t movella_get_data_success(movella_data_t *data) {
    data->acc = EXPECTED_ACC;
    data->gyr = EXPECTED_GYRO;
    data->mag = EXPECTED_MAG;
    data->pres = EXPECTED_BARO;
    data->temp = 25.0f;
    return W_SUCCESS;
}

static w_status_t estimator_update_capture(estimator_all_imus_input_t *data) {
    if (g_captured_imu_data) {
        memcpy(g_captured_imu_data, data, sizeof(estimator_all_imus_input_t));
    }
    return W_FAILURE; // Return failure to exit the task loop
}

class ImuHandlerTest : public ::testing::Test {
protected:
    estimator_all_imus_input_t captured_data;

    void SetUp() override {
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
        RESET_FAKE(estimator_init);
        RESET_FAKE(vTaskDelay);
        RESET_FAKE(timer_get_ms);

        // Default successful returns
        altimu_init_fake.return_val = W_SUCCESS;
        lsm6dsv32_init_fake.return_val = W_SUCCESS;
        movella_init_fake.return_val = W_SUCCESS;
        timer_get_ms_fake.return_val = W_SUCCESS;
        estimator_update_inputs_imu_fake.return_val = W_SUCCESS;
        estimator_init_fake.return_val = W_SUCCESS;

        // Clear captured data
        memset(&captured_data, 0, sizeof(captured_data));
        g_captured_imu_data = &captured_data;
    }

    void TearDown() override {
        g_captured_imu_data = NULL;
    }
};

// Tests
TEST_F(ImuHandlerTest, InitSuccess) {
    w_status_t status = imu_handler_init();
    EXPECT_EQ(W_SUCCESS, status);
}

TEST_F(ImuHandlerTest, TaskInitPhase) {
    // Set up successful IMU initialization
    altimu_init_fake.return_val = W_SUCCESS;
    lsm6dsv32_init_fake.return_val = W_SUCCESS;
    movella_init_fake.return_val = W_SUCCESS;

    // Set up estimator callback to capture data and exit loop
    estimator_update_inputs_imu_fake.custom_fake = estimator_update_capture;

    imu_handler_task(nullptr);

    EXPECT_EQ(1, altimu_init_fake.call_count);
    EXPECT_EQ(1, lsm6dsv32_init_fake.call_count);
    EXPECT_EQ(1, movella_init_fake.call_count);
}

TEST_F(ImuHandlerTest, TaskSuccessfulReads) {
    altimu_get_acc_data_fake.custom_fake = altimu_get_acc_data_success;
    altimu_get_gyro_data_fake.custom_fake = altimu_get_gyro_data_success;
    altimu_get_mag_data_fake.custom_fake = altimu_get_mag_data_success;
    altimu_get_baro_data_fake.custom_fake = altimu_get_baro_data_success;

    lsm6dsv32_get_acc_data_fake.custom_fake = lsm6dsv32_get_acc_data_success;
    lsm6dsv32_get_gyro_data_fake.custom_fake = lsm6dsv32_get_gyro_data_success;

    movella_get_data_fake.custom_fake = movella_get_data_success;

    timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;

    estimator_update_inputs_imu_fake.custom_fake = estimator_update_capture;

    imu_handler_task(nullptr);

    // Verify IMU read calls
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

    // Verify Polulu data
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

    EXPECT_EQ(1, vTaskDelay_fake.call_count);
}

TEST_F(ImuHandlerTest, TaskFailedReads) {
    altimu_get_acc_data_fake.return_val = W_FAILURE;
    altimu_get_gyro_data_fake.return_val = W_FAILURE;
    altimu_get_mag_data_fake.return_val = W_FAILURE;
    altimu_get_baro_data_fake.return_val = W_FAILURE;

    lsm6dsv32_get_acc_data_fake.return_val = W_FAILURE;
    lsm6dsv32_get_gyro_data_fake.return_val = W_FAILURE;

    movella_get_data_fake.return_val = W_FAILURE;

    timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;

    estimator_update_inputs_imu_fake.custom_fake = estimator_update_capture;

    imu_handler_task(nullptr);

    // Verify all IMU data is zeroed due to failures
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

TEST_F(ImuHandlerTest, TaskInitFailures) {
    altimu_init_fake.return_val = W_FAILURE;
    lsm6dsv32_init_fake.return_val = W_FAILURE;
    movella_init_fake.return_val = W_FAILURE;

    estimator_update_inputs_imu_fake.custom_fake = estimator_update_capture;

    imu_handler_task(nullptr);

    EXPECT_EQ(1, altimu_init_fake.call_count);
    EXPECT_EQ(1, lsm6dsv32_init_fake.call_count);
    EXPECT_EQ(1, movella_init_fake.call_count);
    EXPECT_EQ(1, vTaskDelay_fake.call_count);
}

// Test IMU failure handling
TEST_F(ImuHandlerTest, TaskHandlesImuFailures) {
    // Make all IMUs fail
    altimu_get_acc_data_fake.return_val = W_FAILURE;
    lsm6dsv32_get_acc_data_fake.return_val = W_FAILURE;
    movella_get_data_fake.return_val = W_FAILURE;

    estimator_update_inputs_imu_fake.custom_fake = estimator_update_capture;

    // Run one iteration
    imu_handler_task(nullptr);

    // Verify all data is zeroed on failure
    vector3d_t zero_vec = {0};
    float zero_float = 0.0f;

    // Check Polulu
    EXPECT_EQ(0, memcmp(&captured_data.polulu.accelerometer, &zero_vec, sizeof(vector3d_t)));
    EXPECT_EQ(0, memcmp(&captured_data.polulu.gyroscope, &zero_vec, sizeof(vector3d_t)));
    EXPECT_EQ(0, memcmp(&captured_data.polulu.magnometer, &zero_vec, sizeof(vector3d_t)));
    EXPECT_FLOAT_EQ(captured_data.polulu.barometer, zero_float);

    // Check ST
    EXPECT_EQ(0, memcmp(&captured_data.st.accelerometer, &zero_vec, sizeof(vector3d_t)));
    EXPECT_EQ(0, memcmp(&captured_data.st.gyroscope, &zero_vec, sizeof(vector3d_t)));
    EXPECT_EQ(0, memcmp(&captured_data.st.magnometer, &zero_vec, sizeof(vector3d_t)));
    EXPECT_FLOAT_EQ(captured_data.st.barometer, zero_float);

    // Check Movella
    EXPECT_EQ(0, memcmp(&captured_data.movella.accelerometer, &zero_vec, sizeof(vector3d_t)));
    EXPECT_EQ(0, memcmp(&captured_data.movella.gyroscope, &zero_vec, sizeof(vector3d_t)));
    EXPECT_EQ(0, memcmp(&captured_data.movella.magnometer, &zero_vec, sizeof(vector3d_t)));
    EXPECT_FLOAT_EQ(captured_data.movella.barometer, zero_float);
}

// Test sampling period
TEST_F(ImuHandlerTest, TaskSamplingPeriod) {
    estimator_update_inputs_imu_fake.custom_fake = estimator_update_capture;

    imu_handler_task(nullptr);

    // Verify that task delays for correct sampling period
    EXPECT_EQ(1, vTaskDelay_fake.call_count);
    EXPECT_EQ(pdMS_TO_TICKS(5), vTaskDelay_fake.arg0_val);
}

TEST_F(ImuHandlerTest, TaskHandlesPartialFailures) {
    // Make only Polulu IMU fail
    altimu_get_acc_data_fake.return_val = W_FAILURE;
    altimu_get_gyro_data_fake.return_val = W_FAILURE;
    altimu_get_mag_data_fake.return_val = W_FAILURE;
    altimu_get_baro_data_fake.return_val = W_FAILURE;

    // ST and Movella succeed with expected data
    lsm6dsv32_get_acc_data_fake.custom_fake = lsm6dsv32_get_acc_data_success;
    lsm6dsv32_get_gyro_data_fake.custom_fake = lsm6dsv32_get_gyro_data_success;
    movella_get_data_fake.custom_fake = movella_get_data_success;

    timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;
    estimator_update_inputs_imu_fake.custom_fake = estimator_update_capture;

    imu_handler_task(nullptr);

    // Verify Polulu data is zeroed
    vector3d_t zero_vec = {0};
    EXPECT_EQ(0, memcmp(&captured_data.polulu.accelerometer, &zero_vec, sizeof(vector3d_t)));
    EXPECT_EQ(0, memcmp(&captured_data.polulu.gyroscope, &zero_vec, sizeof(vector3d_t)));
    EXPECT_EQ(0, memcmp(&captured_data.polulu.magnometer, &zero_vec, sizeof(vector3d_t)));
    EXPECT_FLOAT_EQ(captured_data.polulu.barometer, 0.0f);

    // Verify ST data is valid
    EXPECT_FLOAT_EQ(EXPECTED_ACC.component.x, captured_data.st.accelerometer.component.x);
    EXPECT_FLOAT_EQ(EXPECTED_ACC.component.y, captured_data.st.accelerometer.component.y);
    EXPECT_FLOAT_EQ(EXPECTED_ACC.component.z, captured_data.st.accelerometer.component.z);

    EXPECT_FLOAT_EQ(EXPECTED_GYRO.component.x, captured_data.st.gyroscope.component.x);
    EXPECT_FLOAT_EQ(EXPECTED_GYRO.component.y, captured_data.st.gyroscope.component.y);
    EXPECT_FLOAT_EQ(EXPECTED_GYRO.component.z, captured_data.st.gyroscope.component.z);

    // Verify Movella data is valid
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

// Test specific failure scenarios for completeness
TEST_F(ImuHandlerTest, TaskHandlesSpecificFailures) {
    // Only fail specific calls to ensure all error paths are exercised
    altimu_get_acc_data_fake.custom_fake = altimu_get_acc_data_success;
    altimu_get_gyro_data_fake.return_val = W_FAILURE;  // Only gyro fails
    altimu_get_mag_data_fake.custom_fake = altimu_get_mag_data_success;
    altimu_get_baro_data_fake.custom_fake = altimu_get_baro_data_success;

    lsm6dsv32_get_acc_data_fake.custom_fake = lsm6dsv32_get_acc_data_success;
    lsm6dsv32_get_gyro_data_fake.return_val = W_FAILURE;  // Only gyro fails

    // Set up Movella to succeed then fail on next call
    static int movella_call_count = 0;
    movella_get_data_fake.custom_fake = [](movella_data_t *data) -> w_status_t {
        if (movella_call_count++ == 0) {
            return movella_get_data_success(data);
        }
        return W_FAILURE;
    };

    timer_get_ms_fake.custom_fake = timer_get_ms_custom_fake;
    estimator_update_inputs_imu_fake.custom_fake = estimator_update_capture;

    // First run
    imu_handler_task(nullptr);

    // Verify that partial failures are handled correctly
    vector3d_t zero_vec = {0};

    // Polulu should have zeroed data due to gyro failure
    EXPECT_EQ(0, memcmp(&captured_data.polulu.accelerometer, &zero_vec, sizeof(vector3d_t)));
    EXPECT_EQ(0, memcmp(&captured_data.polulu.gyroscope, &zero_vec, sizeof(vector3d_t)));
    EXPECT_EQ(0, memcmp(&captured_data.polulu.magnometer, &zero_vec, sizeof(vector3d_t)));
    EXPECT_FLOAT_EQ(0.0f, captured_data.polulu.barometer);

    // ST should have zeroed data due to gyro failure
    EXPECT_EQ(0, memcmp(&captured_data.st.accelerometer, &zero_vec, sizeof(vector3d_t)));
    EXPECT_EQ(0, memcmp(&captured_data.st.gyroscope, &zero_vec, sizeof(vector3d_t)));

    // Movella should have valid data (first call)
    EXPECT_FLOAT_EQ(EXPECTED_ACC.component.x, captured_data.movella.accelerometer.component.x);
    EXPECT_FLOAT_EQ(EXPECTED_GYRO.component.x, captured_data.movella.gyroscope.component.x);
    EXPECT_FLOAT_EQ(EXPECTED_MAG.component.x, captured_data.movella.magnometer.component.x);
    EXPECT_FLOAT_EQ(EXPECTED_BARO, captured_data.movella.barometer);
}