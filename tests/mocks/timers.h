#ifndef TIMERS_H
#define TIMERS_H

#include "FreeRTOS.h"
#include "fff.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Common Queue types from FreeRTOS */
typedef void *TimerHandle_t;
typedef void *TimerCallbackFunction_t;

/* Declare fake functions for common queue operations */

// TimerHandle_t xTimerCreate(	const char * const pcTimerName, const TickType_t
// xTimerPeriodInTicks, const UBaseType_t uxAutoReload, void * const pvTimerID,
// TimerCallbackFunction_t pxCallbackFunction )
DECLARE_FAKE_VALUE_FUNC(
    TimerHandle_t, xTimerCreate, const char *const, TickType_t, UBaseType_t, void *,
    TimerCallbackFunction_t
);
// BaseType_t xTimerReset( TimerHandle_t xTimer, TickType_t xTicksToWait );
DECLARE_FAKE_VALUE_FUNC(BaseType_t, xTimerReset, TimerHandle_t, TickType_t);
// BaseType_t xTimerStop(TimerHandle_t xTimer, TickType_t xTicksToWait);
DECLARE_FAKE_VALUE_FUNC(BaseType_t, xTimerStop, TimerHandle_t, TickType_t);

#ifdef __cplusplus
}
#endif

#endif /* TIMERS_H */