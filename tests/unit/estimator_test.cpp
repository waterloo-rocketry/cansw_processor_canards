#include "fff.h"
#include <gtest/gtest.h>

// Includes needed for FFF types
extern "C" {
#include "FreeRTOS.h"
#include "application/can_handler/can_handler.h" // For can_callback_t, can_msg_t, etc.
#include "application/controller/controller.h" // For controller types
#include "application/flight_phase/flight_phase.h" // For flight_phase_state_t
#include "application/logger/log.h"
#include "canlib.h" // For can types
#include "drivers/timer/timer.h" // For timer_get_ms signature
#include "message_types.h" // For can types
#include "queue.h"
#include "task.h"
#include "third_party/rocketlib/include/common.h" // For w_status_t

#include "application/estimator/estimator.h"

extern w_status_t estimator_run_loop(uint32_t loop_count);
extern w_status_t estimator_log_state_to_can(const x_state_t *current_state);
}

DEFINE_FFF_GLOBALS;

// Define Fakes
// w_status_t controller_get_latest_output(controller_output_t *output);
FAKE_VALUE_FUNC(w_status_t, controller_get_latest_output, controller_output_t *);
// flight_phase_state_t flight_phase_get_state(void);
FAKE_VALUE_FUNC(flight_phase_state_t, flight_phase_get_state);
// w_status_t can_handler_register_callback(can_msg_type_t msg_type, can_callback_t callback);
FAKE_VALUE_FUNC(w_status_t, can_handler_register_callback, can_msg_type_t, can_callback_t);
// w_status_t controller_update_inputs(controller_input_t *inputs);
FAKE_VALUE_FUNC(w_status_t, controller_update_inputs, controller_input_t *);
// bool get_analog_data(const can_msg_t *msg, can_analog_sensor_id_t *sensor_id, uint16_t *value);
FAKE_VALUE_FUNC(bool, get_analog_data, const can_msg_t *, can_analog_sensor_id_t *, uint16_t *);
// w_status_t timer_get_ms(float *time_ms);
FAKE_VALUE_FUNC(w_status_t, timer_get_ms, float *);
// bool build_state_est_data_msg(can_msg_prio_t prio, uint16_t timestamp, can_state_est_id_t id,
// const float *data, can_msg_t *msg);
FAKE_VALUE_FUNC(bool, build_state_est_data_msg, can_msg_prio_t, uint16_t, can_state_est_id_t, const float *, can_msg_t *);
// w_status_t can_handler_transmit(const can_msg_t *msg);
FAKE_VALUE_FUNC(w_status_t, can_handler_transmit, const can_msg_t *);
// w_status_t log_text(const char *source, const char *format, ...);
FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, const char *, const char *, ...)

// Helper function to create a simple x_state_t for testing
x_state_t create_test_state() {
    x_state_t test_state = {0};
    test_state.attitude.w = 1.0f;
    test_state.attitude.x = 0.1f;
    test_state.attitude.y = 0.2f;
    test_state.attitude.z = 0.3f;
    test_state.rates.x = 1.1f;
    test_state.rates.y = 1.2f;
    test_state.rates.z = 1.3f;
    test_state.velocity.x = 2.1f;
    test_state.velocity.y = 2.2f;
    test_state.velocity.z = 2.3f;
    test_state.altitude = 100.0f;
    test_state.CL = 0.5f;
    test_state.delta = 0.05f;
    return test_state;
}

class EstimatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Reset all FFF fakes
        RESET_FAKE(controller_get_latest_output);
        RESET_FAKE(flight_phase_get_state);
        RESET_FAKE(can_handler_register_callback);
        RESET_FAKE(controller_update_inputs);
        RESET_FAKE(get_analog_data);
        RESET_FAKE(timer_get_ms);
        RESET_FAKE(build_state_est_data_msg);
        RESET_FAKE(can_handler_transmit);
        RESET_FAKE(log_text);
    }

    void TearDown() override {}
};

// -------- test init function --------
TEST_F(EstimatorTest, EstimatorInitSuccess) {
    // Arrange
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
    flight_phase_get_state_fake.return_val = STATE_BOOST;
    xQueueReceive_fake.return_val = pdTRUE;
    xQueuePeek_fake.return_val = pdTRUE;
    controller_get_latest_output_fake.return_val = W_SUCCESS;
    controller_update_inputs_fake.return_val = W_SUCCESS;
    timer_get_ms_fake.return_val = W_SUCCESS;
    build_state_est_data_msg_fake.return_val = true;
    can_handler_transmit_fake.return_val = W_SUCCESS;
    log_text_fake.return_val = W_SUCCESS;

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
    EXPECT_EQ(log_text_fake.call_count, 0);
}

TEST_F(EstimatorTest, EstimatorRunLoopActallowedStateNominal) {
    // Arrange
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
    log_text_fake.return_val = W_SUCCESS; // Set return for FFF fake

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
    EXPECT_EQ(log_text_fake.call_count, 0); // Use FFF fake count
}

TEST_F(EstimatorTest, EstimatorRunLoopRecoveryStateNominal) {
    // Arrange
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
    log_text_fake.return_val = W_SUCCESS; // Set return for FFF fake

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
    EXPECT_EQ(log_text_fake.call_count, 0); // Use FFF fake count
}

TEST_F(EstimatorTest, EstimatorRunLoopFlightStateQueueFail) {
    // Arrange
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
    float expect_estimator_output; // TODO: fill in with real numbers

    flight_phase_get_state_fake.return_val = STATE_BOOST; // Simulate flight phase state
    xQueueReceive_fake.return_val = pdTRUE; // Simulate successful queue receive
    xQueuePeek_fake.return_val = pdTRUE; // Simulate successful queue peek
    controller_get_latest_output_fake.return_val = W_SUCCESS; // Simulate controller output
    controller_update_inputs_fake.return_val = W_FAILURE; // Simulate successful controller update
    log_text_fake.return_val = W_SUCCESS; // Set return for FFF fake

    // Act
    w_status_t actual_ret = estimator_run_loop(0);

    // Assert
    // TODO: expect pad filter to NOT be running
    EXPECT_EQ(actual_ret, W_FAILURE);
    EXPECT_EQ(xQueueReceive_fake.call_count, 1);
    EXPECT_EQ(xQueuePeek_fake.call_count, 1);
    EXPECT_EQ(controller_get_latest_output_fake.call_count, 1);
    EXPECT_EQ(controller_update_inputs_fake.call_count, 1);
    EXPECT_EQ(log_text_fake.call_count, 0);
}

// -------- Test estimator_log_state_to_can directly --------

TEST_F(EstimatorTest, EstimatorLogStateToCan_Nominal) {
    // Arrange
    timer_get_ms_fake.return_val = W_SUCCESS; // Simulate timer success
    build_state_est_data_msg_fake.return_val = true; // Simulate build success
    can_handler_transmit_fake.return_val = W_SUCCESS; // Simulate transmit success
    log_text_fake.return_val = W_SUCCESS; // Set return for FFF fake
    x_state_t test_state = create_test_state();
    const uint16_t expected_timestamp = 0; // Assuming timer_get_ms returns 0 for simplicity here

    // Act
    w_status_t status = estimator_log_state_to_can(&test_state);

    // Assert
    EXPECT_EQ(status, W_SUCCESS); // Expect overall success
    EXPECT_EQ(timer_get_ms_fake.call_count, 1); // Timer called once
    EXPECT_EQ(
        build_state_est_data_msg_fake.call_count, STATE_ID_ENUM_MAX
    ); // Build called for each state
    EXPECT_EQ(
        can_handler_transmit_fake.call_count, STATE_ID_ENUM_MAX
    ); // Transmit called for each state
    EXPECT_EQ(log_text_fake.call_count, 0); // No error logs expected

    // Verify arguments for the first and last call (as examples)
    EXPECT_EQ(build_state_est_data_msg_fake.arg0_history[0], PRIO_LOW);
    EXPECT_EQ(build_state_est_data_msg_fake.arg1_history[0], expected_timestamp);
    EXPECT_EQ(build_state_est_data_msg_fake.arg2_history[0], (can_state_est_id_t)0); // First ID
    EXPECT_FLOAT_EQ(
        ((float *)build_state_est_data_msg_fake.arg3_history[0])[0], test_state.array[0]
    );

    EXPECT_EQ(
        build_state_est_data_msg_fake.arg2_history[STATE_ID_ENUM_MAX - 1],
        (can_state_est_id_t)(STATE_ID_ENUM_MAX - 1)
    ); // Last ID
    EXPECT_FLOAT_EQ(
        ((float *)build_state_est_data_msg_fake.arg3_history[STATE_ID_ENUM_MAX - 1])[0],
        test_state.array[STATE_ID_ENUM_MAX - 1]
    );
}

TEST_F(EstimatorTest, EstimatorLogStateToCan_BuildFail) {
    // Arrange
    timer_get_ms_fake.return_val = W_SUCCESS;
    build_state_est_data_msg_fake.return_val = false; // Simulate build failure
    can_handler_transmit_fake.return_val = W_SUCCESS;
    log_text_fake.return_val = W_SUCCESS; // Set return for FFF fake

    x_state_t test_state = create_test_state();

    // Act
    w_status_t status = estimator_log_state_to_can(&test_state);

    // Assert
    EXPECT_EQ(status, W_FAILURE); // Expect overall failure
    EXPECT_EQ(timer_get_ms_fake.call_count, 1);
    EXPECT_EQ(
        build_state_est_data_msg_fake.call_count, STATE_ID_ENUM_MAX
    ); // Build still attempted for all
    EXPECT_EQ(can_handler_transmit_fake.call_count, 0); // Transmit never called due to build fail
    EXPECT_EQ(log_text_fake.call_count, STATE_ID_ENUM_MAX); // Error logged for each build fail
}

TEST_F(EstimatorTest, EstimatorLogStateToCan_TransmitFail) {
    // Arrange
    timer_get_ms_fake.return_val = W_SUCCESS;
    build_state_est_data_msg_fake.return_val = true; // Simulate build success
    can_handler_transmit_fake.return_val = W_FAILURE; // Simulate transmit failure
    log_text_fake.return_val = W_SUCCESS; // Set return for FFF fake

    x_state_t test_state = create_test_state();

    // Act
    w_status_t status = estimator_log_state_to_can(&test_state);

    // Assert
    EXPECT_EQ(status, W_FAILURE); // Expect overall failure
    EXPECT_EQ(timer_get_ms_fake.call_count, 1);
    EXPECT_EQ(build_state_est_data_msg_fake.call_count, STATE_ID_ENUM_MAX); // Build called for each
    EXPECT_EQ(
        can_handler_transmit_fake.call_count, STATE_ID_ENUM_MAX
    ); // Transmit attempted for each
    EXPECT_EQ(log_text_fake.call_count, STATE_ID_ENUM_MAX); // Error logged for each transmit fail
}

// TODO: add actual full integration calculation tests
// TODO: add loop counter test make sure CAN logging works at correct rate
