#include "application/flight_phase/flight_phase.h"
#include "application/logger/log.h"
#include "application/can_handler/can_handler.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"

// TODO: these are made up values, up to FIDO what these actually are
#define ACT_DELAY_MS 2000 // this needs to be shorter than our boost duration
#define BOOST_TIMEOUT_MS 9000
#define COAST_TIMEOUT_MS 35000

// static members
static flight_phase_state_t curr_state = STATE_PAD;

static QueueHandle_t state_mailbox = NULL;
static QueueHandle_t event_queue = NULL;
static TimerHandle_t act_delay_timer = NULL;
static TimerHandle_t boost_timer = NULL;
static TimerHandle_t coast_timer = NULL;

void act_delay_timer_callback(TimerHandle_t xTimer);
void boost_timer_callback(TimerHandle_t xTimer);
void coast_timer_callback(TimerHandle_t xTimer);
w_status_t inj_open_callback(const can_msg_t *msg);

w_status_t flight_phase_init(void)
{
    state_mailbox = xQueueCreate(1, sizeof(flight_phase_state_t));
    event_queue = xQueueCreate(1, sizeof(flight_phase_event_t));
    act_delay_timer = xTimerCreate("act delay", pdMS_TO_TICKS(ACT_DELAY_MS), pdFALSE, NULL, act_delay_timer_callback);
    boost_timer = xTimerCreate("boost", pdMS_TO_TICKS(BOOST_TIMEOUT_MS), pdFALSE, NULL, boost_timer_callback);
    coast_timer = xTimerCreate("coast", pdMS_TO_TICKS(COAST_TIMEOUT_MS), pdFALSE, NULL, coast_timer_callback);
    can_register_callback(MSG_ACTUATOR_CMD, inj_open_callback);

    if (state_mailbox == NULL || event_queue == NULL || act_delay_timer == NULL || boost_timer == NULL || coast_timer == NULL)
    {
        return W_FAILURE;
    }

    xQueueOverwrite(state_mailbox, &curr_state); // initialize state queue

    return W_SUCCESS;
}

void flight_phase_task(void *args)
{
    flight_phase_event_t event;
    while (1)
    {
        if (xQueueReceive(event_queue, &event, pdMS_TO_TICKS(1000)) == pdPASS)
        {
            // TODO log entry state

            switch (curr_state)
            {
            case STATE_PAD:
                if (event == EVENT_INJ_OPEN)
                {
                    curr_state = STATE_BOOST;
                    xTimerReset(boost_timer, 0);
                    xTimerReset(act_delay_timer, 0);
                }
                else
                {
                    curr_state = STATE_ERROR;
                }
                break;
            case STATE_BOOST:
                if (event == EVENT_ACT_DELAY_ELAPSED)
                {
                    curr_state = STATE_ACT_ALLOWED;
                }
                else if (event == EVENT_BOOST_ELAPSED)
                {
                    curr_state = STATE_COAST;
                    xTimerStop(act_delay_timer, 0); // If we exit boost before the delay time, stop the act delay timer
                }
                else
                {
                    curr_state = STATE_ERROR;
                }
                break;
            case STATE_ACT_ALLOWED:
                if (event == EVENT_BOOST_ELAPSED)
                {
                    curr_state = STATE_COAST;
                }
                else
                {
                    curr_state = STATE_ERROR;
                }
                break;
            case STATE_COAST:
                if (event == EVENT_COAST_ELAPSED)
                {
                    curr_state = STATE_RECOVERY;
                }
                else
                {
                    curr_state = STATE_ERROR;
                }
                break;
            case STATE_RECOVERY:
                // TODO log an error there should be no more events
                break;

            default:
                // TODO log an error we should never get here
                break;
            }

            // TODO log exit state
            (void)xQueueOverwrite(state_mailbox, &curr_state); // pdPASS is the only value that can be returned
        }
        else
        {
            // we didn't see an event
            //  TODO log current state
        }
    }
}

flight_phase_state_t flight_phase_get_state()
{
    flight_phase_state_t state;
    if (xQueuePeek(state_mailbox, &state, 5) == pdPASS)
    {
        return state;
    }

    return NONE;
}

w_status_t flight_phase_send_event(flight_phase_event_t event)
{
    return xQueueSend(event_queue, event, 0) == pdPASS ? W_SUCCESS : W_FAILURE; // This cannot block because it is called in the timer daemon task
}

void act_delay_timer_callback(TimerHandle_t xTimer)
{
    flight_phase_send_event(EVENT_ACT_DELAY_ELAPSED);
}

void boost_timer_callback(TimerHandle_t xTimer)
{
    flight_phase_send_event(EVENT_BOOST_ELAPSED);
}

void coast_timer_callback(TimerHandle_t xTimer)
{
    flight_phase_send_event(EVENT_COAST_ELAPSED);
}

w_status_t inj_open_callback(const can_msg_t *msg)
{
    if (get_actuator_id(msg) == ACTUATOR_INJECTOR_VALVE && get_req_actuator_state(msg) == ACTUATOR_ON)
    {
        return flight_phase_send_event(EVENT_INJ_OPEN);
    }
    return W_SUCCESS;
}