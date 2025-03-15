#include <gtest/gtest.h>

// Include the mock header
extern "C" {
#include "common/math/math.h"
#include "src/application/estimator/estimator.h"
#include "third_party/rocketlib/include/common.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
}

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