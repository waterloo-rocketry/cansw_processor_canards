#ifndef FLIGHT_PHASE_H
#define FLIGHT_PHASE_H

#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"
#include "rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * Enum representing phase of flight (state machine state)
 */
typedef enum
{
    STATE_PAD,
    STATE_BOOST,
    STATE_COAST,
    STATE_RECOVERY,
    STATE_ERROR
} flight_phase_state_t;

/**
 * Enum representing a state transition event
 */
typedef enum
{
    EVENT_INJ_OPEN,
    EVENT_BOOST_ELAPSED,
    EVENT_COAST_ELAPSED,
    EVENT_RESET
} flight_phase_event_t;

/**
 * Intialize flight phase module.
 * Creates and allocates state/event queues and timers
 * Sets and populates the default state.
 */
w_status_t flight_phase_init(void);

/**
 * Task to execute the state machine itself. Consumes events and transitions the state
 */
void flight_phase_task(void *args);

/**
 * Returns the current state of the state machine
 * Not ISR safe
 */
flight_phase_state_t flight_phase_get_state();

/**
 * Send a flight phase event to the state machine
 * Not ISR safe
 */
w_status_t flight_phase_send_event(flight_phase_event_t event);

#endif