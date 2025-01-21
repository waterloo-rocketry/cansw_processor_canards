#ifndef FREERTOS_H
#define FREERTOS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stddef.h>

    /* Minimal type definitions */
    typedef uint32_t TickType_t;
    typedef int BaseType_t;
    typedef unsigned int UBaseType_t;

/* Boolean definitions */
#define pdFALSE 0
#define pdTRUE 1

    /* Function pointer types */
    typedef void (*TaskFunction_t)(void *);
    typedef void (*TimerCallbackFunction_t)(void *);

/* Minimal configuration definitions */
#define configSTACK_DEPTH_TYPE uint16_t
#define portNUM_CONFIGURABLE_REGIONS 1

    /* Task handle and priority */
    typedef void *TaskHandle_t;
#define tskIDLE_PRIORITY 0

/* Minimal macros */
#define pdMS_TO_TICKS(x) ((TickType_t)(x))
#define xSemaphoreGiveFromISR(sem, pxHigherPriorityTaskWoken) ((BaseType_t)1)
#define portYIELD_FROM_ISR(x) \
    do                        \
    {                         \
        (void)(x);            \
    } while (0)

/* Include minimal stubs */
#include "semphr.h"
#include "queue.h"
#include "task.h"

    /* Forward declarations for functions (their definitions come from our mocks) */
    BaseType_t xSemaphoreTake(SemaphoreHandle_t xSemaphore, TickType_t xTicksToWait);
    BaseType_t xSemaphoreGive(SemaphoreHandle_t xSemaphore);
    SemaphoreHandle_t xSemaphoreCreateMutex(void);
    SemaphoreHandle_t xSemaphoreCreateBinary(void);
    void vSemaphoreDelete(SemaphoreHandle_t xSemaphore);

#ifdef __cplusplus
}
#endif

#endif /* FREERTOS_H */
