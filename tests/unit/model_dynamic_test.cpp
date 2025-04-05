#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
// add includes like freertos, hal, proc headers, etc
#include "application/estimator/model/quaternion.h"
#include "application/estimator/model/model_dynamics.h"
#include <stdlib.h>
#include <math.h>


}

DEFINE_FFF_GLOBALS;

class ModelDynamicTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Reset all fakes before each test, for example:
        // RESET_FAKE(xQueueCreate);
        FFF_RESET_HISTORY();
    }

    void TearDown() override {}
};

// Test example
TEST_F(ModelDynamicTest, NominalCheck) {
    // Arrange
    // Set up any necessary variables, mocks, etc
    estimator_state_t expected_state;

    float T;
    estimator_state_t estimator_state;
    estimator_input_t estimator_input;

    // Act
    // Call the function to be tested
    estimator_state_t actual_state = model_dynamics_update(float dt, estimator_state_t *est_state, estimator_input_t *est_input);

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(1, 1); // Example assertion
}