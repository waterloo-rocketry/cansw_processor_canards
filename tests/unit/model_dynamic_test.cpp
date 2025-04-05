#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
// add includes like freertos, hal, proc headers, etc
#include "application/estimator/model/quaternion.h"
#include "application/estimator/model/model_dynamics.h"



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

    // Act
    // Call the function to be tested

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(1, 1); // Example assertion
}