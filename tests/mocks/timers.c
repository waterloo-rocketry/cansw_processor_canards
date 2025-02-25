#include "timers.h"

DEFINE_FAKE_VALUE_FUNC(
    TimerHandle_t, xTimerCreate, const char *const, TickType_t, UBaseType_t, void *,
    TimerCallbackFunction_t
);
DEFINE_FAKE_VALUE_FUNC(BaseType_t, xTimerReset, TimerHandle_t, TickType_t);
DEFINE_FAKE_VALUE_FUNC(BaseType_t, xTimerStop, TimerHandle_t, TickType_t);