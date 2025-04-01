#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
// add includes like freertos, hal, proc headers, etc
#include "FreeRTOS.h"
#include "application/controller/controller.h"
#include "application/controller/gain_table.h"
#include "application/flight_phase/flight_phase.h"
#include "queue.h"
#include "rocketlib/include/common.h"
#include "third_party/canlib/message/msg_actuator.h"

extern w_status_t interpolate_gain(float p_dyn, float coeff, controller_gain_t *gain_output);
extern w_status_t get_commanded_angle(
    controller_gain_t control_gain, float control_roll_state[FEEDBACK_GAIN_NUM], float *cmd_angle
);

// fake defines
FAKE_VALUE_FUNC(w_status_t, timer_get_ms, float *);
FAKE_VALUE_FUNC(w_status_t, can_handler_transmit, can_msg_t *);
FAKE_VALUE_FUNC(w_status_t, log_text, const char *, const char *);
FAKE_VALUE_FUNC(flight_phase_state_t, flight_phase_get_state);

}

// needed unit test: interpolate_gain(), get_commanded_angle()

;

DEFINE_FFF_GLOBALS;

class ControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Reset all fakes before each test, for example:
        // RESET_FAKE(xQueueCreate);
        RESET_FAKE(timer_get_ms);
        RESET_FAKE(can_handler_transmit);
        RESET_FAKE(log_text);
        RESET_FAKE(flight_phase_get_state);
        FFF_RESET_HISTORY();
    }

    void TearDown() override {}
};

// Test example
TEST_F(ControllerTest, NominalCheck) {
    // Arrange
    // Set up any necessary variables, mocks, etc
    w_status_t expected_status = W_SUCCESS;
    float expected_angle = -0.1745;
    flight_phase_get_state_fake.return_val = STATE_ACT_ALLOWED;

    float p_dyn = 1e4;
    float coeff = 1.0f;
    controller_gain_t controller_gain = {0};
    float roll_state_arr[FEEDBACK_GAIN_NUM] = {1, 1, 1};

    // Act
    // Call the function to be tested

    w_status_t actual_status = interpolate_gain(p_dyn, coeff, &controller_gain);

    float actual_angle;
    get_commanded_angle(controller_gain, roll_state_arr, &actual_angle);

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(expected_status, actual_status); // Example assertion
    EXPECT_EQ(expected_angle, actual_angle);
}

TEST_F(ControllerTest, InterpolationOutOfBoundCheck) {
    // Arrange
    // Set up any necessary variables, mocks, etc
    w_status_t expected_status = W_FAILURE;
    float expected_angle = 0.0f;
    flight_phase_get_state_fake.return_val = STATE_ACT_ALLOWED;

    float p_dyn = 10.0f;
    float coeff = 1.0f;
    controller_gain_t controller_gain = {0};
    float roll_state_arr[FEEDBACK_GAIN_NUM] = {1, 1, 1};

    // Act
    // Call the function to be tested
    w_status_t actual_status = interpolate_gain(p_dyn, coeff, &controller_gain);

    float actual_angle;
    get_commanded_angle(controller_gain, roll_state_arr, &actual_angle);

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(expected_status, actual_status); // Example assertion
    EXPECT_EQ(expected_angle, actual_angle);
}

