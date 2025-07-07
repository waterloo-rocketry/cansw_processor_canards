#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
// add includes like freertos, hal, proc headers, etc
#include "FreeRTOS.h"
#include "application/controller/controller.h"
#include "application/controller/controller_module.h"
#include "application/controller/gain_table.h"
#include "application/flight_phase/flight_phase.h"
#include "queue.h"
#include "rocketlib/include/common.h"
#include "third_party/canlib/message/msg_actuator.h"

FAKE_VALUE_FUNC(w_status_t, timer_get_ms, double *);
FAKE_VALUE_FUNC(w_status_t, can_handler_transmit, can_msg_t *);
FAKE_VALUE_FUNC(w_status_t, log_text, const char *, const char *);
FAKE_VALUE_FUNC(flight_phase_state_t, flight_phase_get_state);
}

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

// NOTE: to get the test outputs, edit matlab controller_module.m:
// time_start = 0; % pad delay time
// (Set it to 0 to make it avoid simulating pad filter time)

// Test step 0 (initial reference signal)
/**
clear all; clc;

roll_angle = 0.16345;
roll_rate = 0.154534;
canard_angle = 0.000134;
pressure_dynamic = 12093;
canard_coeff = 0.55234;

input = [roll_angle; roll_rate; canard_angle; pressure_dynamic; canard_coeff];

timestamp = 11; % sec

[u, r] = controller_module(timestamp, input);

fprintf('Control Output (u): %.8f rad (%.4f deg)\n', u, rad2deg(u));
fprintf('Reference Signal (r): %.8f rad (%.4f deg)\n', r, rad2deg(r));
 */
TEST_F(ControllerTest, Step0) {
    // Arrange
    controller_input_t input = {0};
    input.roll_state.roll_angle = 0.16345;
    input.roll_state.roll_rate = 0.054534;
    input.roll_state.canard_angle = 0.000134;
    input.canard_coeff = 0.55234;
    input.pressure_dynamic = 12093;

    uint32_t flight_ms = 11000; // step 1
    double output_angle = 0.0;
    float ref_signal = 0.0;

    // Act
    w_status_t status = controller_module(input, flight_ms, &output_angle, &ref_signal);

    // Assert
    double expect_output_angle = 0.01381245;
    float expect_ref_signal = 0.5f;

    EXPECT_EQ(status, W_SUCCESS);
    EXPECT_FLOAT_EQ(ref_signal, expect_ref_signal);
    EXPECT_FLOAT_EQ(output_angle, expect_output_angle);
}