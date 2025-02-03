#ifndef QUEUE_H
#define QUEUE_H

#include "fff.h"
#include "FreeRTOS.h"

// Typedef QueueHandle_t as a void pointer for mocking
typedef void *QueueHandle_t;

DECLARE_FAKE_VALUE_FUNC(QueueHandle_t, xQueueCreate)

#endif // QUEUE_H