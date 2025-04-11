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

#define TOLERANCE 0.0001f

FAKE_VALUE_FUNC(w_status_t, log_text, uint32_t, const char *, const char *);
FAKE_VALUE_FUNC(w_status_t, timer_get_ms, float *);
}

w_status_t timer_get_ms_fake_override(float* ptr){
    *ptr = 0.32f * 1000.0; //0.32s in ms
    return W_SUCCESS;
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
    x_state_t expected_state = {
        .array =
            {-0.1110,
        0.3593,
        0.5849,
        0.7187, 
        77.0760, 
        16.7514, 
        -2.1879,
        12.6528,
        31.3014,
        11.5379,
        12.0240,
        3.6276, // 10.88
        -19.0000}
    }; //attitude and rates
    
    timer_get_ms_fake.custom_fake = timer_get_ms_fake_override;
    
    
    x_state_t estimator_state = {.array = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13}};
    u_dynamics_t estimator_input = {
        .cmd = 8.0f, .acceleration = {5.0f, 77.0f, 9.0f}
    };

    // Act
    // Call the function to be tested
    x_state_t actual_state = model_dynamics_update(&estimator_state, &estimator_input);

    // Assert
    // Verify the expected behavior of the above Act

    for (int i = 0; i < 13; i++) {
        EXPECT_NEAR(expected_state.array[i], actual_state.array[i], abs(expected_state.array[i] * TOLERANCE)); // Example assertion
    }
}

// quaternion_t attitude;  // Attitude quaternion PASSES
// vector3d_t rates;       // Angular rates, body frame DID NOT PASS
// vector3d_t velocity;    // Velocity vector, body frame DID NOT PASS
// float altitude;         // Altitude DID NOT PASS
// float CL;               // Canard coefficient PASSES
// float delta;            // Canard angle PASSES