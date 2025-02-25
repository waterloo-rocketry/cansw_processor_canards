// #include "fff.h"
// #include <gtest/gtest.h>

// extern "C" {
// #include "FreeRTOS.h"
// #include "application/flight_phase/flight_phase.h"
// #include "queue.h"
// #include "timers.h"
// }

// class FPTest : public ::testing::Test {
// protected:
//     void SetUp() override {
//         RESET_FAKE(xQueueCreate);
//         RESET_FAKE(xTimerCreate);
//         RESET_FAKE(xQueueOverwrite);
//         FFF_RESET_HISTORY();
//     }

//     void TearDown() override {}
// };

// // Test gpio_init
// TEST_F(FPTest, InitCreatesMutexes) {
//     // Arrange
//     xQueueCreate_fake.return_val = (QueueHandle_t)1;

//     // Act
//     w_status_t status = flight_phase_init();
//     // Assert
//     EXPECT_EQ(status, W_SUCCESS);
// }