#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
// add includes like freertos, hal, proc headers, etc
DEFINE_FFF_GLOBALS; // this must be called within the extern c block
}


class ExampleTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Reset all fakes before each test, for example:
        // RESET_FAKE(xQueueCreate);
        FFF_RESET_HISTORY();
    }

    void TearDown() override {}
};

// Test example
TEST_F(ExampleTest, DescriptiveTestNameYup) {
    // Arrange
    // Set up any necessary variables, mocks, etc

    // Act
    // Call the function to be tested

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(1, 1); // Example assertion
}