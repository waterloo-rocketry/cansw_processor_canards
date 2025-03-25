// General freertos mocks
#ifndef FREERTOS_H
#define FREERTOS_H

#include <stdint.h>

/* FreeRTOS basic types */
typedef uint32_t TickType_t;
typedef int32_t BaseType_t;
typedef void * TaskHandle_t;
typedef void * QueueHandle_t;
typedef void * SemaphoreHandle_t;
typedef void * TimerHandle_t;
typedef long UBaseType_t;

/* Constants */
#define pdMS_TO_TICKS(xTimeInMs)  ((TickType_t)((xTimeInMs)))
#define pdTRUE                     (1)
#define pdFALSE                    (0)
#define pdPASS                     pdTRUE
#define pdFAIL                     pdFALSE
#define portMAX_DELAY              (TickType_t)0xFFFFFFFFUL

/* Priority definitions */
#define configMAX_PRIORITIES       (7)
#define tskIDLE_PRIORITY           (0)

/* Task utilities */
#define vTaskDelayMs(ms)           vTaskDelay(pdMS_TO_TICKS(ms))

/* Task function macros */
#define taskENTER_CRITICAL()
#define taskEXIT_CRITICAL()


/* Add portYIELD_FROM_ISR macro */
#define portYIELD_FROM_ISR(x) (void)(x)

#endif // FREERTOS_H
