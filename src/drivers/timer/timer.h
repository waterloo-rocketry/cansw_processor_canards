#ifndef TIMER_H
#define TIMER_H

#include "rocketlib/include/common.h"

// tracks system time since program startup
// retrieves time passed in the form of clock ticks
// timer resolution set to 0.1ms (10000Hz frequency)
w_status_t timer_get_ms(float *ms);

#endif
