#include "fff.h"
#include <gtest/gtest.h>

extern "C" {

#include "application/estimator/model/model_dynamics.h"
#include "application/estimator/model/quaternion.h"
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"
#include "third_party/rocketlib/include/common.h"
#include <math.h>
#include <stdlib.h>

#define TOLERANCE 0.0001f

FAKE_VALUE_FUNC(w_status_t, log_text, uint32_t, const char *, const char *);
FAKE_VALUE_FUNC(w_status_t, timer_get_ms, float *);
}

DEFINE_FFF_GLOBALS;

class ModelDynamicTest : public ::testing::Test {
protected:
    void SetUp() override {
        RESET_FAKE(log_text);
        RESET_FAKE(timer_get_ms);
        FFF_RESET_HISTORY();
    }

    void TearDown() override {}
};

// Test example
TEST_F(ModelDynamicTest, NominalCheck) {
    // Arrange
    // Set up any necessary variables, mocks, etc
    x_state_t expected_state = {
        .array = {
            -0.110999723707431,
            0.359328744821869,
            0.584887448198861,
            0.718657489643738,
            77.075955796064676,
            9.969992917198999,
            3.899588192702720,
            12.652799999999999,
            31.301439999999999,
            11.537920000000000,
            12.023999999999999,
            11.464166318621979,
            -19.000000000000000
        }
    }; // CL

    double dt = 0.32;
    x_state_t estimator_state = {.array = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13}};
    u_dynamics_t estimator_input = {.cmd = 8.0f, .acceleration = {5.0f, 77.0f, 9.0f}};

    // Act
    // Call the function to be tested
    x_state_t actual_state = model_dynamics_update(&estimator_state, &estimator_input, dt);

    // Assert
    // Verify the expected behavior of the above Act

    for (int i = 0; i < 13; i++) {
        EXPECT_NEAR(
            expected_state.array[i], actual_state.array[i], abs(expected_state.array[i] * TOLERANCE)
        ); // Example assertion
    }
}
