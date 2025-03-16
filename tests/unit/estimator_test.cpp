#include <gtest/gtest.h>

extern "C" {
#include "common/math/math.h"
#include "src/application/controller/controller.h"
#include "src/application/estimator/estimator.h"
#include "src/application/estimator/model/model_airdata.h"
#include "src/application/logger/log.h"
#include "third_party/rocketlib/include/common.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
}

using ::testing::_;
using ::testing::Return;

// Global variables used by estimator_task
QueueHandle_t imu_data_handle;
int state_est_can_counter = 0;
const int STATE_EST_CAN_RATE = 10;

// init

class EstimatorLibTest : public ::testing::Test {
protected:
    void SetUp() override {}

    void TearDown() override {}
};

TEST_F(EstimatorLibTest, Init_Outputs) {
    QueueHandle_t x_actual = xQueueCreate(1, sizeof(estimator_imu_input_data));

    QueueHandle_t x_expected = xQueueCreate(1, sizeof(5));
    // push sample values into queue

    EXPECT_EQ(x_actual, x_expected);
}

// task
typedef struct {
    double data[13];
} estimator_imu_input_data;

typedef struct {
    double data[13];
} controller_input_data;

controller_input_data controller_input_buffer;
estimator_imu_input_data imu_data_buffer;

typedef struct {
    double data[13];
} controller_output_data;
controller_output_data output_to_controller;

class EstimatorTaskTest : public ::testing::Test {
protected:
    void SetUp() override {
        imu_data_handle = xQueueCreate(1, sizeof(estimator_imu_input_data));
    }

    void TearDown() override {
        vQueueDelete(imu_data_handle);
    }
};

TEST_F(EstimatorTaskTest, FailsToReceiveImuData) {
    xQueueReceive_ExpectAndReturn(imu_data_handle, &imu_data_buffer, _, pdFALSE);
    EXPECT_CALL(
        MockLogger::GetInstance(),
        log_text("State estimation", "failed to receive imu data within 5ms")
    );
    estimator_task();
}

TEST_F(EstimatorTaskTest, FailsToReceiveControllerData) {
    xQueueReceive_ExpectAndReturn(imu_data_handle, &imu_data_buffer, _, pdTRUE);
    EXPECT_CALL(MockController::GetInstance(), controller_get_latest_output(_))
        .WillOnce(Return(W_FAILURE));
    EXPECT_CALL(
        MockLogger::GetInstance(), log_text("State estimation", "failed to receive controller data")
    );
    estimator_task();
}

TEST_F(EstimatorTaskTest, SuccessfulExecution) {
    xQueueReceive_ExpectAndReturn(imu_data_handle, &imu_data_buffer, _, pdTRUE);
    EXPECT_CALL(MockController::GetInstance(), controller_get_latest_output(_))
        .WillOnce(Return(W_SUCCESS));
    EXPECT_CALL(MockController::GetInstance(), controller_update_inputs(_))
        .WillOnce(Return(W_SUCCESS));
    estimator_task();
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

// update values

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