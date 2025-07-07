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

// these are abs ignore the naming
#define CMD_TOLERANCE 1e-5
#define GAIN_TOLERANCE 3.2e-5

// fake defines
FAKE_VALUE_FUNC(w_status_t, timer_get_ms, double *);
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
    double expected_angle = -0.174532925199433;
    flight_phase_get_state_fake.return_val = STATE_ACT_ALLOWED;

    double p_dyn = 1e3;
    double coeff = 0.5f;
    controller_gain_t controller_gain = {0};
    double roll_state_arr[ROLL_STATE_NUM] = {1, 1};

    double expected_output[3] = {-1.236960284820258, -0.447879197399534, 1.236960284820258};

    // Act
    // Call the function to be tested

    w_status_t actual_status = interpolate_gain(p_dyn, coeff, &controller_gain);

    for (int i = 0; i < GAIN_NUM; i++) {
        EXPECT_NEAR(expected_output[i], controller_gain.gain_arr[i], GAIN_TOLERANCE);
    }

    double actual_angle;
    float ref_signal = 0.0;
    get_commanded_angle(controller_gain, roll_state_arr, ref_signal, &actual_angle);

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(expected_status, actual_status); // Example assertion
    EXPECT_NEAR(expected_angle, actual_angle, CMD_TOLERANCE); // 0.56 millidegree precision
}
TEST_F(ControllerTest, NominalCheck2) {
    // Arrange
    // Set up any necessary variables, mocks, etc
    w_status_t expected_status = W_SUCCESS;
    double expected_angle = -6.548558257302171e-04;
    flight_phase_get_state_fake.return_val = STATE_ACT_ALLOWED;

    double p_dyn = 13420.0f;
    double coeff = -1.23f;
    controller_gain_t controller_gain = {0};
    double roll_state_arr[ROLL_STATE_NUM] = {0, 0.01};

    // Act
    // Call the function to be tested

    w_status_t actual_status = interpolate_gain(p_dyn, coeff, &controller_gain);

    double actual_angle;
    float ref_signal = 0.0;
    get_commanded_angle(controller_gain, roll_state_arr, ref_signal, &actual_angle);

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(expected_status, actual_status); // Example assertion
    EXPECT_NEAR(expected_angle, actual_angle, CMD_TOLERANCE); // 0.56 millidegree precision
}
TEST_F(ControllerTest, NominalCheck3) {
    // Arrange
    // Set up any necessary variables, mocks, etc
    w_status_t expected_status = W_SUCCESS;
    double expected_angle = -1.830257035155605e-04;
    flight_phase_get_state_fake.return_val = STATE_ACT_ALLOWED;

    double p_dyn = 13420.0f;
    double coeff = -1.46f;
    controller_gain_t controller_gain = {0};
    double roll_state_arr[ROLL_STATE_NUM] = {0.001, 0};

    // Act
    // Call the function to be tested

    w_status_t actual_status = interpolate_gain(p_dyn, coeff, &controller_gain);

    double actual_angle;
    float ref_signal = 0.0;
    get_commanded_angle(controller_gain, roll_state_arr, ref_signal, &actual_angle);

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(expected_status, actual_status); // Example assertion

    EXPECT_NEAR(expected_angle, actual_angle, CMD_TOLERANCE); // 0.56 millidegree precision
}

TEST_F(ControllerTest, InterpolationOutOfBoundCheck) {
    // Arrange
    // Set up any necessary variables, mocks, etc
    w_status_t expected_status = W_FAILURE;
    double expected_angle = 0.0f;
    flight_phase_get_state_fake.return_val = STATE_ACT_ALLOWED;

    double p_dyn = 10.0f;
    double coeff = 1.0f;
    controller_gain_t controller_gain = {0};
    double roll_state_arr[ROLL_STATE_NUM] = {1, 1};

    // Act
    // Call the function to be tested
    w_status_t actual_status = interpolate_gain(p_dyn, coeff, &controller_gain);

    double actual_angle;
    float ref_signal = 0.0;
    get_commanded_angle(controller_gain, roll_state_arr, ref_signal, &actual_angle);

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(expected_status, actual_status); // Example assertion
    EXPECT_EQ(expected_angle, actual_angle);
}
TEST_F(ControllerTest, GainInterpolationCheck) {
    // Arrange
    // Set up any necessary variables, mocks, etc

    double p_dyn = 12345.0f;
    double coeff = 1.0f;
    controller_gain_t controller_gain = {0};
    double expected_output[3] = {-0.197389375124674, -0.070631480005056, 0.197389375124674};

    w_status_t expected_status = W_SUCCESS;
    double roll_state_arr[2] = {0.02, 0.001};
    double expected_angle = -0.004018418982499;
    // Act
    // Call the function to be tested
    w_status_t actual_status = interpolate_gain(p_dyn, coeff, &controller_gain);

    double actual_angle;
    float ref_signal = 0.0;
    get_commanded_angle(controller_gain, roll_state_arr, ref_signal, &actual_angle);

    // Assert
    // Verify the expected behavior of the above Act
    EXPECT_EQ(expected_status, actual_status); // Example assertion
    EXPECT_NEAR(expected_angle, actual_angle, CMD_TOLERANCE);
    for (int i = 0; i < 3; i++) {
        EXPECT_NEAR(expected_output[i], controller_gain.gain_arr[i], GAIN_TOLERANCE);
    }
}

