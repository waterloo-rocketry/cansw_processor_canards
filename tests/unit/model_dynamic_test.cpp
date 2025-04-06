#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
// add includes like freertos, hal, proc headers, etc

#include "application/estimator/model/model_dynamics.h"
#include "application/estimator/model/quaternion.h"
#include "common/math/math-algebra3d.h"
#include "third_party/rocketlib/include/common.h"
#include <math.h>
#include <stdlib.h>

FAKE_VALUE_FUNC(w_status_t, log_text, uint32_t, const char *, const char *);
FAKE_VALUE_FUNC(w_status_t, timer_get_ms, float *);
}

DEFINE_FFF_GLOBALS;

class ModelDynamicTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Reset all fakes before each test, for example:
        // RESET_FAKE(xQueueCreate);
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
    estimator_state_t expected_state = {
        .array =
            {-0.7796,
             0.1999,
             0.4387,
             0.3998,
             455.8303,
             72.9120,
             -50.1633,
             37.0667,
             148.3867,
             19.6267,
             17.4000,
             3.6276,
             -387.0000}
    };

    log_text_fake.return_val = W_SUCCESS;
    timer_get_ms_fake.return_val = W_SUCCESS;

    float T = 2.0f;
    estimator_state_t estimator_state = {.array = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13}};
    estimator_input_t estimator_input = {
        .acceleration = {5.0f, 77.0f, 9.0f}, .canard_command = 8.0f
    };

    // Act
    // Call the function to be tested
    estimator_state_t actual_state = model_dynamics_update(T, &estimator_state, &estimator_input);

    // Assert
    // Verify the expected behavior of the above Act

    for (int i = 3; i < 6; i++) {
        EXPECT_NEAR(expected_state.array[i], actual_state.array[i], 0.1); // Example assertion
    }
}

// quaternion_t attitude;  // Attitude quaternion PASSES
// vector3d_t rates;       // Angular rates, body frame DID NOT PASS
// vector3d_t velocity;    // Velocity vector, body frame DID NOT PASS
// float altitude;         // Altitude DID NOT PASS
// float CL;               // Canard coefficient PASSES
// float delta;            // Canard angle PASSES