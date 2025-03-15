#include <gtest/gtest.h>

// Include the mock header
extern "C" {
#include "application/estimator/estimator.h"
#include "common/math/math.h"
#include "src/application/estimator/model/model_airdata.h"
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

TEST_F(EstimatorLibTest, Init_Filter) {}