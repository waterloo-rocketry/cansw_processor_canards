#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
#include "FreeRTOS.h"
#include "application/can_handler/can_handler.h"
#include "application/flight_phase/flight_phase.h"
#include "canlib.h"
#include "queue.h"
#include "timers.h"

// FAKES
//  w_status_t log_text(const char *source, const char *format, ...)
FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, const char *, const char *, ...)

// w_status_t can_register_callback(can_msg_type_t msg_type, can_callback_t callback)
FAKE_VALUE_FUNC(w_status_t, can_register_callback, can_msg_type_t, can_callback_t)

// int get_actuator_id(const can_msg_t *msg);

FAKE_VALUE_FUNC(int, get_actuator_id, const can_msg_t *)

// int get_cmd_actuator_state(const can_msg_t *msg);
FAKE_VALUE_FUNC(int, get_cmd_actuator_state, const can_msg_t *)

BaseType_t
xQueuePeek_state_pad(QueueHandle_t xQueue, void *const pvBuffer, TickType_t xTicksToWait) {
    flight_phase_state_t *p = (flight_phase_state_t *)pvBuffer;
    *p = STATE_PAD;
    return pdPASS;
}
}

class FlightPhaseTest : public ::testing::Test {
protected:
    void SetUp() override {
        RESET_FAKE(xQueueCreate);
        RESET_FAKE(xTimerCreate);
        RESET_FAKE(xQueueOverwrite);
        RESET_FAKE(log_text)
        RESET_FAKE(can_register_callback)
        RESET_FAKE(get_actuator_id)
        RESET_FAKE(get_cmd_actuator_state)
        FFF_RESET_HISTORY();
    }

    void TearDown() override {}
};

// flight_phase_init should pass when valid handles are created/callback is registered
TEST_F(FlightPhaseTest, InitCreatesMutexes) {
    // Arrange
    xQueueCreate_fake.return_val = (QueueHandle_t)1;
    xTimerCreate_fake.return_val = (TimerHandle_t)1;
    can_register_callback_fake.return_val = W_SUCCESS;

    // Act
    w_status_t status = flight_phase_init();

    // Assert
    EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(FlightPhaseTest, InitFailsIfMutexCreationFails1) {
    // Arrange
    xQueueCreate_fake.return_val = (QueueHandle_t)NULL;
    xTimerCreate_fake.return_val = (TimerHandle_t)1;
    can_register_callback_fake.return_val = W_SUCCESS;

    // Act
    w_status_t status = flight_phase_init();

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

TEST_F(FlightPhaseTest, InitFailsIfMutexCreationFails2) {
    // Arrange
    xQueueCreate_fake.return_val = (QueueHandle_t)1;
    xTimerCreate_fake.return_val = (TimerHandle_t)NULL;
    can_register_callback_fake.return_val = W_SUCCESS;

    // Act
    w_status_t status = flight_phase_init();

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

TEST_F(FlightPhaseTest, InitFailsIfRegistrationFails) {
    // // Arrange
    xQueueCreate_fake.return_val = (QueueHandle_t)1;
    xTimerCreate_fake.return_val = (TimerHandle_t)1;
    can_register_callback_fake.return_val = W_FAILURE;

    // Act
    w_status_t status = flight_phase_init();

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

TEST_F(FlightPhaseTest, GetStateReturnsCorrectDefaultState) {
    // Arrange
    xQueuePeek_fake.custom_fake = xQueuePeek_state_pad;

    // Act
    flight_phase_state_t state = flight_phase_get_state();

    // Assert
    EXPECT_EQ(state, STATE_PAD);
}

TEST_F(FlightPhaseTest, GetStateReturnsCorrectErrorState) {
    // Arrange
    // Do nothing when xQueuePeek is called

    // Act
    flight_phase_state_t state = flight_phase_get_state();

    // Assert
    EXPECT_EQ(state, STATE_ERROR);
}

TEST_F(FlightPhaseTest, SendEventSucceeds) {
    // Arrange
    xQueueSend_fake.return_val = pdPASS;

    // Act
    w_status_t status = flight_phase_send_event(EVENT_ESTIMATOR_INIT);

    // Assert
    EXPECT_EQ(status, W_SUCCESS);
}

TEST_F(FlightPhaseTest, SendEventFails) {
    // Arrange
    xQueueSend_fake.return_val = pdFAIL;

    // Act
    w_status_t status = flight_phase_send_event(EVENT_ESTIMATOR_INIT);

    // Assert
    EXPECT_EQ(status, W_FAILURE);
}

TEST_F(FlightPhaseTest, ResetSendsResetEvent) {
    // Arrange
    xQueueSend_fake.return_val = pdPASS;

    // Act
    w_status_t status = flight_phase_reset();

    // Assert
    EXPECT_EQ(xQueueSend_fake.call_count, 1);
    EXPECT_EQ(status, W_SUCCESS);
}