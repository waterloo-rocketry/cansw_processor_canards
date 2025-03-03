#include <gtest/gtest.h>

// Include the mock header
extern "C"
{
#include "stm32h7xx_hal.h"
}

// This macro must be called exactly 1 time in the entire test suite so that fff is defined for all tests.
DEFINE_FFF_GLOBALS;

// Test fixture for the dummy test
class DummyTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        RESET_FAKE(HAL_Init); // Reset the fake before each test
    }

    void TearDown() override {}
};

// Dummy test to verify FFF integration
TEST_F(DummyTest, HALInitIsCalled)
{
    // Arrange: (nothing needed for this test)

    // Act: Call the fake HAL_Init
    HAL_Init();

    // Assert: Verify that HAL_Init was called
    EXPECT_EQ(HAL_Init_fake.call_count, 1);
}
