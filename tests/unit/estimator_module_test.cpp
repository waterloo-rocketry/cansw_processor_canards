#include "fff.h"
#include <gtest/gtest.h>

#include "utils/mock_helpers.hpp"

extern "C" {

#include "application/estimator/estimator_types.h"
#include "application/estimator/model/model_dynamics.h"
#include "application/estimator/model/quaternion.h"
#include "arm_math.h"
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"
#include "third_party/rocketlib/include/common.h"
#include <math.h>
#include <stdlib.h>

#define TOLERANCE 0.000001

FAKE_VALUE_FUNC(w_status_t, log_text, uint32_t, const char *, const char *);
}

DEFINE_FFF_GLOBALS;

class EstimatorModuleTest : public ::testing::Test {
protected:
    void SetUp() override {
        FFF_RESET_HISTORY();
    }

    void TearDown() override {}
};

TEST_F(EstimatorModuleTest, OneIteration) {}

