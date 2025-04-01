#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
// add includes like freertos, hal, proc headers, etc
#include "third_party/canlib/message/msg_actuator.h"
#include "rocketlib/include/common.h"
}

// needed unit test: interpolate_gain(), get_commanded_angle()

//fake defines
FAKE_VALUE_FUNC(w_status_t, timer_get_ms, float*);



DEFINE_FFF_GLOBALS;

class ControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Reset all fakes before each test, for example:
        // RESET_FAKE(xQueueCreate);
        FFF_RESET_HISTORY();
    }

    void TearDown() override {}
};

// Test example
TEST_F(ControllerTest, InterpolationCheck) {
    // Arrange
    // Set up any necessary variables, mocks, etc

    // Act
    // Call the function to be tested

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(1, 1); // Example assertion
}

TEST_F(ControllerTest, InterpolationFailureCheck) {
    // Arrange
    // Set up any necessary variables, mocks, etc

    // Act
    // Call the function to be tested

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(1, 1); // Example assertion
}

TEST_F(ControllerTest, CommandedAngleCheck) {
    // Arrange
    // Set up any necessary variables, mocks, etc

    // Act
    // Call the function to be tested

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(1, 1); // Example assertion
}

TEST_F(ControllerTest, CommandedAngleFailureheck) {
    // Arrange
    // Set up any necessary variables, mocks, etc

    // Act
    // Call the function to be tested

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(1, 1); // Example assertion
}