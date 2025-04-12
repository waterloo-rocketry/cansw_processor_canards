#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
// add includes like freertos, hal, proc headers, etc
#include "application/estimator/model/model_airdata.h"
#include <math.h>

#define TOLERANCE 0.001

DEFINE_FFF_GLOBALS; // this must be called within the extern c block
}


class AirdataJacobianTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Reset all fakes before each test, for example:
        // RESET_FAKE(xQueueCreate);
        FFF_RESET_HISTORY();
    }

    void TearDown() override {}
};

// Test example
TEST_F(AirdataJacobianTest, NominalCheck) {
    // Arrange
    // Set up any necessary variables, mocks, etc
    double expected_output = -11.4522; // -11.452243086187998
    
    double altitude = 500;

    // Act
    // Call the function to be tested
    double actual_output = model_airdata_jacobian(altitude);

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_NEAR(expected_output, actual_output, expected_output * TOLERANCE); // Example assertion
}