// General freertos mocks
#ifndef FREERTOS_H
#define FREERTOS_H

#include <stdint.h>

// Copy typedefs from freertos for mocks

// portmacro.h -----------------------------------
typedef long BaseType_t;
typedef uint32_t TickType_t;
#define portYIELD_FROM_ISR(x) \
    do                        \
    {                         \
        (void)(x);            \
    } while (0)

// projdefs.h -----------------------------------
#ifndef pdMS_TO_TICKS
	#define pdMS_TO_TICKS( xTimeInMs ) xTimeInMs // skip conversion cuz we use 1-to-1 ticks to ms
#endif

#define pdFALSE			0
#define pdTRUE			1

#endif // FREERTOS_H
