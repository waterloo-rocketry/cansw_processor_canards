#include "fff.h"
#include <gtest/gtest.h>
using namespace std;
#include <iostream>

extern "C" {
// add includes like freertos, hal, proc headers, etc
#include "FreeRTOS.h"
#include "application/controller/controller.h"
#include "application/controller/gain_table.h"
#include "application/flight_phase/flight_phase.h"
#include "queue.h"
#include "rocketlib/include/common.h"
#include "third_party/canlib/message/msg_actuator.h"


#define CMD_ANGLE_PRECISION 0.003
#define GAIN_PRECISION  0.3

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
TEST_F(ControllerTest, NominalCheck1) {
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
    EXPECT_NEAR(expected_angle, actual_angle, CMD_ANGLE_PRECISION); // 0.56 millidegree precision
}
TEST_F(ControllerTest, NominalCheck2) {
    // Arrange
    // Set up any necessary variables, mocks, etc
    w_status_t expected_status = W_SUCCESS;
    float expected_angle = -0.0028;
    flight_phase_get_state_fake.return_val = STATE_ACT_ALLOWED;

    float p_dyn = 13420.0f;
    float coeff = 1.0f;
    controller_gain_t controller_gain = {0};
    float roll_state_arr[FEEDBACK_GAIN_NUM] = {0, 0, 0.001};

    // Act
    // Call the function to be tested

    w_status_t actual_status = interpolate_gain(p_dyn, coeff, &controller_gain);

    float actual_angle;
    get_commanded_angle(controller_gain, roll_state_arr, &actual_angle);

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(expected_status, actual_status); // Example assertion
    EXPECT_NEAR(expected_angle, actual_angle, CMD_ANGLE_PRECISION); // 0.56 millidegree precision
}
TEST_F(ControllerTest, NominalCheck3) {
    // Arrange
    // Set up any necessary variables, mocks, etc
    w_status_t expected_status = W_SUCCESS;
    float expected_angle = -0.0284;
    flight_phase_get_state_fake.return_val = STATE_ACT_ALLOWED;

    float p_dyn = 13420.0f;
    float coeff = 1.0f;
    controller_gain_t controller_gain = {0};
    float roll_state_arr[FEEDBACK_GAIN_NUM] = {0.001, 0, 0.01};

    // Act
    // Call the function to be tested

    w_status_t actual_status = interpolate_gain(p_dyn, coeff, &controller_gain);

    float actual_angle;
    get_commanded_angle(controller_gain, roll_state_arr, &actual_angle);

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(expected_status, actual_status); // Example assertion

    EXPECT_NEAR(expected_angle, actual_angle, CMD_ANGLE_PRECISION); // 0.56 millidegree precision
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
TEST_F(ControllerTest, GainInterpolationCheck) {
    // Arrange
    // Set up any necessary variables, mocks, etc

    float p_dyn = 12345.0f;
    float coeff = 1.0f;
    controller_gain_t controller_gain = {0};
    float expected_output[4] = {-0.6321, -0.5047, -2.7207, 1.3507};
    w_status_t expected_status = W_SUCCESS;
    float roll_state_arr[3] = {0.02, 0, 0.001};
    float expected_angle = -0.0154;
    // Act
    // Call the function to be tested
    w_status_t actual_status = interpolate_gain(
        p_dyn, coeff, &controller_gain
    ); // FAILED WITH {-2.84129, -2.1746, -4.18964, 5.57639}
    float actual_angle;
    get_commanded_angle(
        controller_gain, roll_state_arr, &actual_angle
    );
    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(expected_status, actual_status); // Example assertion
    EXPECT_NEAR(
        expected_angle, actual_angle, CMD_ANGLE_PRECISION
    ); 
    for (int i = 0; i < 4; i++) {
        EXPECT_NEAR(
            expected_output[i], controller_gain.gain_arr[i], GAIN_PRECISION
        ); // 0.56 millidegree precision in radians
    }
}

