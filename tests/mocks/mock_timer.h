#ifndef MOCK_TIMER_H
#define MOCK_TIMER_H

#include "fff.h"
#include "third_party/rocketlib/include/common.h"

FAKE_VALUE_FUNC(w_status_t, timer_get_ms, float *);

#endif // MOCK_TIMER_H