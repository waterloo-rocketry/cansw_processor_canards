#include "../mocks/fff/fff.h"
#include <gtest/gtest.h>

extern "C" {
// add includes like freertos, hal, proc headers, etc
#include "FreeRTOS.h"
#include "rocketlib/include/common.h"
#include "drivers/adc/adc.h"
#include "canlib.h"
#include "message_types.h"
extern w_status_t health_check_exec();

FAKE_VALUE_FUNC(w_status_t, adc_get_value, adc_channel_t, uint32_t*, uint32_t);
FAKE_VALUE_FUNC(w_status_t, timer_get_ms, float*);
FAKE_VALUE_FUNC(w_status_t, can_handler_transmit, can_msg_t*);
FAKE_VALUE_FUNC5(bool, build_general_board_status_msg, can_msg_prio_t, uint16_t, uint32_t, uint16_t, can_msg_t*);
}

DEFINE_FFF_GLOBALS;

class HealthChecksTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Reset all fakes before each test, for example:
        // RESET_FAKE(xQueueCreate);
        FFF_RESET_HISTORY();

        // default return values for fakes
    }

    void TearDown() override {}
};
// Constants used in the tests
static const float ADC_VREF = 3.3;
static const float INA180A3_GAIN = 100.0;
static const float R_SENSE = 0.033;
static const uint16_t MAX_CURRENT_mA = 400;

TEST_F(HealthChecksTest, NominalHealthCheck) {
    // Arrange
    w_status_t expect_return = W_SUCCESS;
    adc_get_value_fake.return_val = W_SUCCESS;
    build_general_board_status_msg_fake.return_val = true;
    // todo: set up more fakes here


    // Act
    w_status_t actual_return = health_check_exec();

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(expect_return, actual_return);
    EXPECT_EQ(build_general_board_status_msg_fake.call_count, 1);
    EXPECT_EQ(build_general_board_status_msg_fake.arg0_val, PRIO_LOW);
}

TEST_F(HealthChecksTest, FailureHealthCheck) {
    // Arrange
    w_status_t expect_return = W_FAILURE;
    adc_get_value_fake.return_val = W_SUCCESS;
    // mock a failure
    build_general_board_status_msg_fake.return_val = false;

    // Act
    w_status_t actual_return = health_check_exec();

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(expect_return, actual_return);
}
