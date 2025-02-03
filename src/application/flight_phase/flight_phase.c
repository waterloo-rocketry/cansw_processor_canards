#include "application/flight_phase/flight_phase.h"
#include "application/logger/log.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"

#define INIT_TIMEOUT_MS 10000 // TODO: these are made up values, up to FIDO/Finn what these actually are
#define BOOST_TIMEOUT_MS 9000
#define COAST_TIMEOUT_MS 35000

// static members
static flight_phase_state_t curr_state = STATE_INIT;

static QueueHandle_t state_mailbox = NULL;
static QueueHandle_t event_queue = NULL;
static TimerHandle_t init_timer = NULL;
static TimerHandle_t boost_timer = NULL;
static TimerHandle_t coast_timer = NULL;

void timer_callback(TimerHandle_t xTimer);

w_status_t flight_phase_init(void)
{
    state_mailbox = xQueueCreate(1, sizeof(flight_phase_state_t));
    event_queue = xQueueCreate(1, sizeof(flight_phase_event_t));
    init_timer = xTimerCreate("init", pdMS_TO_TICKS(INIT_TIMEOUT_MS), pdFALSE, 0, timer_callback); // TODO make timer ID an enum too? do something hacky passing the event as the ID?
    boost_timer = xTimerCreate("boost", pdMS_TO_TICKS(INIT_TIMEOUT_MS), pdFALSE, 1, timer_callback);
    coast_timer = xTimerCreate("coast", pdMS_TO_TICKS(INIT_TIMEOUT_MS), pdFALSE, 2, timer_callback);

    if (state_mailbox == NULL || event_queue == NULL || init_timer == NULL || boost_timer == NULL || coast_timer == NULL)
    {
        return W_FAILURE;
    }

    if (xQueueOverwrite(state_mailbox, &curr_state) != pdPASS)
    {
        return W_FAILURE;
    }

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
            case STATE_INIT:
                if (event == EVENT_INIT_ELAPSED)
                {
                    curr_state = STATE_PAD;
                }
                else
                {
                }
                break;

            default:
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
    xQueuePeek(state_mailbox, &state, 5);
}

w_status_t flight_phase_send_event(flight_phase_event_t event)
{
    return xQueueSend(event_queue, event, 0) == pdPASS ? W_SUCCESS : W_FAILURE; // This cannot block because it is called in the timer daemon task
}

/**
 * This function is called whenever one of the state machine timers expires
 * Behaviour is implemented based on the triggering timer's ID
 */
void timer_callback(TimerHandle_t xTimer)
{
    uint32_t id;
    id = *(uint32_t *)pvTimerGetTimerID(xTimer);

    switch (id)
    {
    case 0:
        flight_phase_send_event(STATE_INIT);
        break;
    case 1:
        flight_phase_send_event(STATE_BOOST);
        break;
    case 2:
        flight_phase_send_event(STATE_COAST);
        break;
    default:
        // TODO: log an error this should not get here
        break;
    }
}