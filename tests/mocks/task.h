#ifndef TASK_H
#define TASK_H

#include "FreeRTOS.h"

/* Task creation functions */
#define xTaskCreate(pvTaskCode, pcName, usStackDepth, pvParameters, uxPriority, pxCreatedTask)
#define xTaskCreateStatic(pvTaskCode, pcName, ulStackDepth, pvParameters, uxPriority, pxStackBuffer, pxTaskBuffer)

/* Task-specific function declarations not already in FreeRTOS.h */
TaskHandle_t xTaskGetCurrentTaskHandle(void);
BaseType_t xTaskGetSchedulerState(void);

void vTaskDelay(TickType_t xTicksToDelay);
TickType_t xTaskGetTickCount(void);
BaseType_t xTaskDelayUntil(TickType_t* pxPreviousWakeTime, TickType_t xTimeIncrement);
void vTaskDelayUntil(TickType_t* pxPreviousWakeTime, TickType_t xTimeIncrement);

/* Additional task definitions if needed by the application */
#define configMAX_TASK_NAME_LEN 16

#endif /* TASK_H */
