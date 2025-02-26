#include <gtest/gtest.h>

// Include the mock header
extern "C" {
    #include "application/estimator/model/model_airdata.h"
    #include "application/estimator/estimator.h"
    #include "common/math/math.h"
    #include "third_party/rocketlib/include/common.h"
    #include <stdbool.h>
    #include <stdint.h>
    #include <math.h>
}

// Test fixture for estimator lib test
class EstimatorLibTest : public ::testing::Test {
protected:
    void SetUp() override {}

    void TearDown() override {}
};

// example of testing addition
TEST_F(EstimatorLibTest, AirdataOutputs) {
    // Arrange: set up the inputs and expected result
    float altitude = 0;
    estimator_airdata_t expected_result = {101325.0, 288.15, 1.225, 340.297};

    // Act: Call the addition function
    estimator_airdata_t actual_result = model_airdata(altitude);

    // Assert: Verify that the expected result matches the actual result
    EXPECT_EQ(actual_result, expected_result);
}