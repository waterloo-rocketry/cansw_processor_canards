#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
#include "FreeRTOS.h"
#include "application/can_handler/can_handler.h"
#include "application/controller/controller.h"
#include "application/estimator/estimator.h"
#include "application/flight_phase/flight_phase.h"
#include "canlib.h"
#include "common/math/math.h"
#include "queue.h"
#include "task.h"
#include "third_party/rocketlib/include/common.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

extern w_status_t estimator_run_loop(uint32_t loop_count);

// fakes
DEFINE_FFF_GLOBALS;
FAKE_VALUE_FUNC(w_status_t, controller_get_latest_output, controller_output_t *);
FAKE_VALUE_FUNC(flight_phase_state_t, flight_phase_get_state);
FAKE_VALUE_FUNC(w_status_t, can_handler_register_callback, can_msg_type_t, can_callback_t);
FAKE_VALUE_FUNC(w_status_t, controller_update_inputs, controller_input_t *);
FAKE_VALUE_FUNC(bool, get_analog_data, const can_msg_t *, can_analog_sensor_id_t *, uint16_t *);
FAKE_VALUE_FUNC(w_status_t, timer_get_ms, float *);
FAKE_VALUE_FUNC(bool, build_state_est_data_msg, can_msg_prio_t, uint16_t, can_state_est_id_t, const float *, can_msg_t *);
FAKE_VALUE_FUNC(w_status_t, can_handler_transmit, const can_msg_t *);
FAKE_VOID_FUNC(log_text, const char *, const char *, ...);
}

class EstimatorTest : public ::testing::Test {
protected:
    void SetUp() override {}

    void TearDown() override {}
};

// -------- test init function --------
TEST_F(EstimatorTest, EstimatorInitSuccess) {
    // Arrange
    RESET_FAKE(can_handler_register_callback);
    xQueueCreate_fake.return_val = (QueueHandle_t)1; // Simulate successful queue creation
    can_handler_register_callback_fake.return_val = W_SUCCESS;

    // Act
    w_status_t actual_ret = estimator_init();

    // Assert
    EXPECT_EQ(actual_ret, W_SUCCESS);
    EXPECT_EQ(can_handler_register_callback_fake.call_count, 1);
    EXPECT_EQ(can_handler_register_callback_fake.arg0_val, MSG_SENSOR_ANALOG);
}

TEST_F(EstimatorTest, EstimatorInitQueueFail) {
    // Arrange
    RESET_FAKE(can_handler_register_callback);
    xQueueCreate_fake.return_val = NULL; // Simulate failed queue creation

    // Act
    w_status_t actual_ret = estimator_init();

    // Assert
    EXPECT_EQ(actual_ret, W_FAILURE);
}

TEST_F(EstimatorTest, EstimatorInitCanFail) {
    // Arrange
    can_handler_register_callback_fake.return_val = W_FAILURE; // Simulate failed CAN registration
    xQueueCreate_fake.return_val = (QueueHandle_t)1; // Simulate successful queue creation

    // Act
    w_status_t actual_ret = estimator_init();

    // Assert
    EXPECT_EQ(actual_ret, W_FAILURE);
}

// -------- test imu data update ------

TEST_F(EstimatorTest, NominalUpdateImuData) {
    // Arrange
    estimator_all_imus_input_t test_imu_data = {};
    RESET_FAKE(xQueueOverwrite);
    xQueueOverwrite_fake.return_val =
        pdPASS; // xqueueoverwrite literally can't fail hence this is the only test case

    // Act
    w_status_t actual_ret = estimator_update_imu_data(&test_imu_data);

    // Assert
    EXPECT_EQ(actual_ret, W_SUCCESS);
    EXPECT_EQ(xQueueOverwrite_fake.call_count, 1);
    EXPECT_EQ(xQueueOverwrite_fake.arg1_val, &test_imu_data);
}

// -------- test estimator task main loop -----

// --- pad state ---
TEST_F(EstimatorTest, EstimatorRunLoopPadStateNominal) {
    // Arrange
    RESET_FAKE(flight_phase_get_state);
    RESET_FAKE(xQueueReceive);
    RESET_FAKE(xQueuePeek);
    RESET_FAKE(controller_get_latest_output);
    RESET_FAKE(controller_update_inputs);

    flight_phase_get_state_fake.return_val = STATE_PAD; // Simulate flight phase state

    // Act
    w_status_t actual_ret = estimator_run_loop(0);

    // Assert
    // expect do absolutely nothing!!!
    EXPECT_EQ(actual_ret, W_SUCCESS);
    EXPECT_EQ(xQueueReceive_fake.call_count, 0);
    EXPECT_EQ(controller_get_latest_output_fake.call_count, 0);
    EXPECT_EQ(controller_update_inputs_fake.call_count, 0);
    // TODO: expect pad filter not run (cant check cuz its empty rn skull)
}

// --- init pad filter state ---
TEST_F(EstimatorTest, EstimatorRunLoopInitStateNominal) {
    // Arrange
    RESET_FAKE(flight_phase_get_state);
    RESET_FAKE(xQueueReceive);
    RESET_FAKE(xQueuePeek);
    RESET_FAKE(controller_get_latest_output);
    RESET_FAKE(controller_update_inputs);

    flight_phase_get_state_fake.return_val = STATE_SE_INIT; // Simulate flight phase state

    // Act
    w_status_t actual_ret = estimator_run_loop(0);

    // Assert
    // TODO: expect pad filter to run. should NOT be doing calculations
    EXPECT_EQ(actual_ret, W_SUCCESS);
    EXPECT_EQ(xQueueReceive_fake.call_count, 0);
    EXPECT_EQ(controller_get_latest_output_fake.call_count, 0);
    // should NOT be updating controller in this state
    EXPECT_EQ(controller_update_inputs_fake.call_count, 0);
}

// --- flight state ---
TEST_F(EstimatorTest, EstimatorRunLoopBoostStateNominal) {
    // Arrange
    RESET_FAKE(flight_phase_get_state);
    RESET_FAKE(xQueueReceive);
    RESET_FAKE(xQueuePeek);
    RESET_FAKE(controller_get_latest_output);
    RESET_FAKE(controller_update_inputs);
    RESET_FAKE(timer_get_ms);
    RESET_FAKE(build_state_est_data_msg);
    RESET_FAKE(can_handler_transmit);
    RESET_FAKE(log_text);

    float expect_estimator_output; // TODO: fill in with real numbers

    flight_phase_get_state_fake.return_val = STATE_BOOST; // Simulate flight phase state
    xQueueReceive_fake.return_val = pdTRUE; // Simulate successful queue receive
    xQueuePeek_fake.return_val = pdTRUE; // Simulate successful queue peek
    controller_get_latest_output_fake.return_val =
        W_SUCCESS; // Simulate successful controller output
    controller_update_inputs_fake.return_val = W_SUCCESS; // Simulate successful controller update
    timer_get_ms_fake.return_val = W_SUCCESS;
    build_state_est_data_msg_fake.return_val = true;
    can_handler_transmit_fake.return_val = W_SUCCESS;

    // Act
    w_status_t actual_ret = estimator_run_loop(0);

    // Assert
    // TODO: expect pad filter to NOT be running
    EXPECT_EQ(actual_ret, W_SUCCESS);
    EXPECT_EQ(xQueueReceive_fake.call_count, 1);
    EXPECT_EQ(xQueuePeek_fake.call_count, 1);
    EXPECT_EQ(controller_get_latest_output_fake.call_count, 1);
    // expect controller updates in this state
    EXPECT_EQ(controller_update_inputs_fake.call_count, 1);
}

TEST_F(EstimatorTest, EstimatorRunLoopActallowedStateNominal) {
    // Arrange
    RESET_FAKE(flight_phase_get_state);
    RESET_FAKE(xQueueReceive);
    RESET_FAKE(xQueuePeek);
    RESET_FAKE(controller_get_latest_output);
    RESET_FAKE(controller_update_inputs);
    RESET_FAKE(timer_get_ms);
    RESET_FAKE(build_state_est_data_msg);
    RESET_FAKE(can_handler_transmit);
    RESET_FAKE(log_text); 

    float expect_estimator_output; // TODO: fill in with real numbers

    flight_phase_get_state_fake.return_val = STATE_ACT_ALLOWED; // Simulate flight phase state
    xQueueReceive_fake.return_val = pdTRUE; // Simulate successful queue receive
    xQueuePeek_fake.return_val = pdTRUE; // Simulate successful queue peek
    controller_get_latest_output_fake.return_val =
        W_SUCCESS; // Simulate successful controller output
    controller_update_inputs_fake.return_val = W_SUCCESS; // Simulate successful controller update
    timer_get_ms_fake.return_val = W_SUCCESS;
    build_state_est_data_msg_fake.return_val = true;
    can_handler_transmit_fake.return_val = W_SUCCESS;

    // Act
    w_status_t actual_ret = estimator_run_loop(0);

    // Assert
    // TODO: expect pad filter to NOT be running
    EXPECT_EQ(actual_ret, W_SUCCESS);
    EXPECT_EQ(xQueueReceive_fake.call_count, 1);
    EXPECT_EQ(xQueuePeek_fake.call_count, 1);
    EXPECT_EQ(controller_get_latest_output_fake.call_count, 1);
    // expect controller updates in this state
    EXPECT_EQ(controller_update_inputs_fake.call_count, 1);
}

TEST_F(EstimatorTest, EstimatorRunLoopRecoveryStateNominal) {
    // Arrange
    RESET_FAKE(flight_phase_get_state);
    RESET_FAKE(xQueueReceive);
    RESET_FAKE(xQueuePeek);
    RESET_FAKE(controller_get_latest_output);
    RESET_FAKE(controller_update_inputs);
    RESET_FAKE(timer_get_ms);
    RESET_FAKE(build_state_est_data_msg);
    RESET_FAKE(can_handler_transmit);
    RESET_FAKE(log_text); 

    float expect_estimator_output; // TODO: fill in with real numbers

    flight_phase_get_state_fake.return_val = STATE_RECOVERY; // Simulate flight phase state
    xQueueReceive_fake.return_val = pdTRUE; // Simulate successful queue receive
    xQueuePeek_fake.return_val = pdTRUE; // Simulate successful queue peek
    controller_get_latest_output_fake.return_val =
        W_SUCCESS; // Simulate successful controller output
    controller_update_inputs_fake.return_val = W_SUCCESS; // Simulate successful controller update
    timer_get_ms_fake.return_val = W_SUCCESS;
    build_state_est_data_msg_fake.return_val = true;
    can_handler_transmit_fake.return_val = W_SUCCESS;

    // Act
    w_status_t actual_ret = estimator_run_loop(0);

    // Assert
    // TODO: expect pad filter to NOT be running
    EXPECT_EQ(actual_ret, W_SUCCESS);
    EXPECT_EQ(xQueueReceive_fake.call_count, 1);
    EXPECT_EQ(xQueuePeek_fake.call_count, 1);
    EXPECT_EQ(controller_get_latest_output_fake.call_count, 1);
    // expect controller updates in this state
    EXPECT_EQ(controller_update_inputs_fake.call_count, 1);
}

TEST_F(EstimatorTest, EstimatorRunLoopFlightStateQueueFail) {
    // Arrange
    RESET_FAKE(flight_phase_get_state);
    RESET_FAKE(xQueueReceive);
    RESET_FAKE(xQueuePeek);
    RESET_FAKE(controller_get_latest_output);
    RESET_FAKE(controller_update_inputs);

    float expect_estimator_output; // TODO: fill in with real numbers

    flight_phase_get_state_fake.return_val = STATE_BOOST; // Simulate flight phase state
    xQueueReceive_fake.return_val = pdFALSE; // Simulate failed queue receive
    xQueuePeek_fake.return_val = pdTRUE; // Simulate successful queue peek
    controller_get_latest_output_fake.return_val =
        W_SUCCESS; // Simulate successful controller output
    controller_update_inputs_fake.return_val = W_SUCCESS; // Simulate successful controller update

    // Act
    w_status_t actual_ret = estimator_run_loop(0);

    // Assert
    // TODO: expect pad filter to NOT be running
    EXPECT_EQ(actual_ret, W_FAILURE);
    EXPECT_EQ(xQueueReceive_fake.call_count, 1);
    // expect no controller updates since something failed
    EXPECT_EQ(controller_update_inputs_fake.call_count, 0);
}

TEST_F(EstimatorTest, EstimatorRunLoopFlightStateQueueFail2) {
    // Arrange
    RESET_FAKE(flight_phase_get_state);
    RESET_FAKE(xQueueReceive);
    RESET_FAKE(xQueuePeek);
    RESET_FAKE(controller_get_latest_output);
    RESET_FAKE(controller_update_inputs);

    float expect_estimator_output; // TODO: fill in with real numbers

    flight_phase_get_state_fake.return_val = STATE_BOOST; // Simulate flight phase state
    xQueueReceive_fake.return_val = pdTRUE; // Simulate successful queue receive
    xQueuePeek_fake.return_val = pdFALSE; // Simulate failed queue peek
    controller_get_latest_output_fake.return_val =
        W_SUCCESS; // Simulate successful controller output
    controller_update_inputs_fake.return_val = W_SUCCESS; // Simulate successful controller update

    // Act
    w_status_t actual_ret = estimator_run_loop(0);

    // Assert
    // TODO: expect pad filter to NOT be running
    EXPECT_EQ(actual_ret, W_FAILURE);
    EXPECT_EQ(xQueuePeek_fake.call_count, 1);
    // expect no controller updates since something failed
    EXPECT_EQ(controller_update_inputs_fake.call_count, 0);
}

TEST_F(EstimatorTest, EstimatorRunLoopFlightStateControllerFail) {
    // Arrange
    RESET_FAKE(flight_phase_get_state);
    RESET_FAKE(xQueueReceive);
    RESET_FAKE(xQueuePeek);
    RESET_FAKE(controller_get_latest_output);
    RESET_FAKE(controller_update_inputs);

    float expect_estimator_output; // TODO: fill in with real numbers

    flight_phase_get_state_fake.return_val = STATE_BOOST; // Simulate flight phase state
    xQueueReceive_fake.return_val = pdTRUE; // Simulate successful queue receive
    xQueuePeek_fake.return_val = pdTRUE; // Simulate successful queue peek
    controller_get_latest_output_fake.return_val = W_FAILURE; // Simulate FAIL controller output
    controller_update_inputs_fake.return_val = W_SUCCESS; // Simulate successful controller update

    // Act
    w_status_t actual_ret = estimator_run_loop(0);

    // Assert
    // TODO: expect pad filter to NOT be running
    EXPECT_EQ(actual_ret, W_FAILURE);
    EXPECT_EQ(controller_get_latest_output_fake.call_count, 1);
    // expect no controller updates since something failed
    EXPECT_EQ(controller_update_inputs_fake.call_count, 0);
}

TEST_F(EstimatorTest, EstimatorRunLoopFlightStateControllerUpdateFail) {
    // Arrange
    RESET_FAKE(flight_phase_get_state);
    RESET_FAKE(xQueueReceive);
    RESET_FAKE(xQueuePeek);
    RESET_FAKE(controller_get_latest_output);
    RESET_FAKE(controller_update_inputs);

    float expect_estimator_output; // TODO: fill in with real numbers

    flight_phase_get_state_fake.return_val = STATE_BOOST; // Simulate flight phase state
    xQueueReceive_fake.return_val = pdTRUE; // Simulate successful queue receive
    xQueuePeek_fake.return_val = pdTRUE; // Simulate successful queue peek
    controller_get_latest_output_fake.return_val = W_SUCCESS; // Simulate controller output
    controller_update_inputs_fake.return_val = W_FAILURE; // Simulate successful controller update

    // Act
    w_status_t actual_ret = estimator_run_loop(0);

    // Assert
    // TODO: expect pad filter to NOT be running
    EXPECT_EQ(actual_ret, W_FAILURE);
    EXPECT_EQ(xQueueReceive_fake.call_count, 1);
    EXPECT_EQ(xQueuePeek_fake.call_count, 1);
    EXPECT_EQ(controller_get_latest_output_fake.call_count, 1);
    EXPECT_EQ(controller_update_inputs_fake.call_count, 1);
}

// TODO: add actual full integration calculation tests
// TODO: add loop counter test make sure CAN logging works at correct rate