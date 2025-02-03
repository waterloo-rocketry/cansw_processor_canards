#include <gtest/gtest.h>
#include "fff.h"

extern "C"
{
#include "application/flight_phase/flight_phase.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"
}

class FPTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        RESET_FAKE(xQueueCreate);
        FFF_RESET_HISTORY();
    }

    void TearDown() override {}
};

// Test gpio_init
TEST_F(FPTest, InitCreatesMutexes)
{
    // Arrange

    // Act

    // Assert
}