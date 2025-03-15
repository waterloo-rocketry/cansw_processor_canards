#include "gtest/gtest.h"

extern "C" {
#include "application/estimator/estimator.h"
#include "common/math/math.h"
#include "third_party/rocketlib/include/common.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
}

using ::testing::_;
using ::testing::Return;

// Mocking global queue handle
QueueHandle_t imu_data_handle;

// Function to test
w_status_t estimator_update_inputs_imu(estimator_imu_input_data *data) {
    if (data == nullptr) {
        return W_FAILURE;
    }
    if (xQueueOverwrite(imu_data_handle, data) != pdPASS) {
        return W_FAILURE;
    }
    return W_SUCCESS;
}

class EstimatorIMUTest : public ::testing::Test {
protected:
    void SetUp() override {
        imu_data_handle = xQueueCreate(1, sizeof(estimator_imu_input_data));
    }

    void TearDown() override {
        vQueueDelete(imu_data_handle);
    }
};

TEST_F(EstimatorIMUTest, NullDataReturnsFailure) {
    EXPECT_EQ(estimator_update_inputs_imu(nullptr), W_FAILURE);
}

TEST_F(EstimatorIMUTest, QueueOverwriteSuccess) {
    estimator_imu_input_data test_data = {};
    xQueueOverwrite_ExpectAndReturn(imu_data_handle, &test_data, pdPASS);
    EXPECT_EQ(estimator_update_inputs_imu(&test_data), W_SUCCESS);
}

TEST_F(EstimatorIMUTest, QueueOverwriteFailure) {
    estimator_imu_input_data test_data = {};
    xQueueOverwrite_ExpectAndReturn(imu_data_handle, &test_data, pdFAIL);
    EXPECT_EQ(estimator_update_inputs_imu(&test_data), W_FAILURE);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}