#ifndef TIMER_H
#define TIMER_H

//tracks system time since program startup
//returns time passed in the form of clock ticks
//each clock tick is equivalent to 0.1ms
w_status_t timer_get_ms();

#endif
