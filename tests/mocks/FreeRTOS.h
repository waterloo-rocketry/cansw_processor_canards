// General freertos mocks
#ifndef FREERTOS_H
#define FREERTOS_H

#include <stdint.h>

// Copy typedefs from freertos for mocks

// portmacro.h -----------------------------------
typedef long BaseType_t;
typedef uint32_t TickType_t;

typedef uint32_t UBaseType_t;

// projdefs.h -----------------------------------
#ifndef pdMS_TO_TICKS
#define pdMS_TO_TICKS(xTimeInMs) xTimeInMs // skip conversion cuz we use 1-to-1 ticks to ms
#endif

#define pdFALSE 0
#define pdTRUE 1
#define portMAX_DELAY (TickType_t)0xffffffffUL
#define pdPASS (pdTRUE)
#define pdFAIL (pdFALSE)

// Add portYIELD_FROM_ISR macro
#define portYIELD_FROM_ISR(x) (void)(x)

#endif // FREERTOS_H
