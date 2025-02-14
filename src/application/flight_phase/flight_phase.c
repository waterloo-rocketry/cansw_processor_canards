#include "application/flight_phase/flight_phase.h"
#include "application/can_handler/can_handler.h"
#include "application/logger/log.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"

// TODO: these are made up values, up to FIDO what these actually are
// See the flowchart in the design doc for more context on these
#define ACT_DELAY_MS 9000 // Q - the minimum time after launch before allowing canards to actuate
#define FLIGHT_TIMEOUT_MS 50000 // K - the approximate time between launch and apogee

#define TASK_TIMEOUT_MS 1000

// static members
static flight_phase_state_t curr_state = STATE_PAD;

static QueueHandle_t state_mailbox = NULL;
static QueueHandle_t event_queue = NULL;
static TimerHandle_t act_delay_timer = NULL;
static TimerHandle_t flight_timer = NULL;

static void act_delay_timer_callback(TimerHandle_t xTimer);
static void flight_timer_callback(TimerHandle_t xTimer);
static w_status_t act_cmd_callback(const can_msg_t *msg);

/**
 * Intialize flight phase module.
 * Creates and allocates state/event queues and timers
 * Sets and populates the default state.
 */
w_status_t flight_phase_init(void) {
    state_mailbox = xQueueCreate(1, sizeof(flight_phase_state_t));
    event_queue = xQueueCreate(1, sizeof(flight_phase_event_t));
    act_delay_timer = xTimerCreate(
        "act delay", pdMS_TO_TICKS(ACT_DELAY_MS), pdFALSE, NULL, act_delay_timer_callback
    );
    flight_timer = xTimerCreate(
        "flight", pdMS_TO_TICKS(FLIGHT_TIMEOUT_MS), pdFALSE, NULL, flight_timer_callback
    );

    if (NULL == state_mailbox || NULL == event_queue || NULL == act_delay_timer ||
        NULL == flight_timer ||
        (W_SUCCESS != can_register_callback(MSG_ACTUATOR_CMD, act_cmd_callback))) {
        return W_FAILURE;
    }

    xQueueOverwrite(state_mailbox, &curr_state); // initialize state queue
    return W_SUCCESS;
}

/**
 * Task to execute the state machine itself. Consumes events and transitions the state
 */
void flight_phase_task(void *args) {
    (void)args;
    flight_phase_event_t event;
    while (1) {
        if (xQueueReceive(event_queue, &event, pdMS_TO_TICKS(TASK_TIMEOUT_MS)) == pdPASS) {
            log_text("flight_phase", "transition-entry:%d-state:%d\n", curr_state, event);

            switch (curr_state) {
                case STATE_PAD:
                    if (EVENT_ESTIMATOR_INIT == event) {
                        curr_state = STATE_SE_INIT;
                    } else if (EVENT_RESET == event) {
                        curr_state = STATE_PAD;
                    } else {
                        curr_state = STATE_ERROR;
                    }
                    break;

                case STATE_SE_INIT:
                    if (EVENT_INJ_OPEN == event) {
                        curr_state = STATE_BOOST;
                        xTimerReset(act_delay_timer, 0);
                        xTimerReset(flight_timer, 0);
                    } else if (EVENT_RESET == event) {
                        curr_state = STATE_PAD;
                    } else {
                        curr_state = STATE_ERROR;
                    }
                    break;

                case STATE_BOOST:
                    if (EVENT_ACT_DELAY_ELAPSED == event) {
                        curr_state = STATE_ACT_ALLOWED;
                    } else if (EVENT_FLIGHT_ELAPSED == event) {
                        xTimerStop(act_delay_timer, 0);
                        curr_state = STATE_RECOVERY;
                    } else if (EVENT_RESET == event) {
                        curr_state = STATE_PAD;
                    } else {
                        curr_state = STATE_ERROR;
                    }
                    break;

                case STATE_ACT_ALLOWED:
                    if (EVENT_FLIGHT_ELAPSED == event) {
                        curr_state = STATE_RECOVERY;
                    } else if (EVENT_RESET == event) {
                        curr_state = STATE_PAD;
                    } else {
                        curr_state = STATE_ERROR;
                    }
                    break;

                case STATE_RECOVERY:
                    if (EVENT_RESET == event) {
                        curr_state = STATE_PAD;
                    } else {
                        curr_state = STATE_ERROR;
                    }
                    break;
                default:
                    log_text("flight_phase", "error-unhandled-state:%d\n", curr_state);
                    break;
            }

            log_text("flight_phase", "transition-exit:%d\n", curr_state);
            (void)xQueueOverwrite(
                state_mailbox, &curr_state
            ); // pdPASS is the only value that can be returned
        } else {
            log_text("flight_phase", "timeout:%d\n", curr_state);
        }
    }
}

/**
 * Returns the current state of the state machine
 * Not ISR safe
 * @return STATE_ERROR if getting the current state failed/timed out, otherwise the current flight
 * phase
 */
flight_phase_state_t flight_phase_get_state() {
    flight_phase_state_t state = STATE_ERROR;
    xQueuePeek(state_mailbox, &state, 5);
    return state;
}

/**
 * Send a flight phase event to the state machine
 * Not ISR safe
 */
w_status_t flight_phase_send_event(flight_phase_event_t event) {
    return xQueueSend(event_queue, &event, 0) == pdPASS
               ? W_SUCCESS
               : W_FAILURE; // This cannot block because it is called in the timer daemon task
}

/**
 * Timer callback for actuation delay timer
 */
static void act_delay_timer_callback(TimerHandle_t xTimer) {
    (void)xTimer;
    flight_phase_send_event(EVENT_ACT_DELAY_ELAPSED);
}

/**
 * Timer callback for flight elapsed timer
 */
static void flight_timer_callback(TimerHandle_t xTimer) {
    (void)xTimer;
    flight_phase_send_event(EVENT_FLIGHT_ELAPSED);
}

/**
 * Global CAN callback for messages of type MSG_ACTUATOR_CMD
 * Handles OX_INJECTOR_VALVE->OPEN and PROC_ESTIMATOR_INIT->OPEN
 */
static w_status_t act_cmd_callback(const can_msg_t *msg) {
    if (get_actuator_id(msg) == ACTUATOR_INJECTOR_VALVE && // TODO: use OX_INJECTOR_VALVE
        get_req_actuator_state(msg) == ACTUATOR_ON) {
        return flight_phase_send_event(EVENT_INJ_OPEN);
    } else if (get_actuator_id(msg) == ACTUATOR_INJECTOR_VALVE && // TODO: use PROC_ESTIMATOR_INIT
               get_req_actuator_state(msg) == ACTUATOR_ON) {
        return flight_phase_send_event(EVENT_ESTIMATOR_INIT);
    } else {
        return W_SUCCESS; // TODO: anything else to do here? Log in case of skill issue in above
                          // conditions?
    }
}

/**
 * Resets the flight phase state machine to initial state
 */
void flight_phase_reset(void) {
    flight_phase_send_event(EVENT_RESET);
}