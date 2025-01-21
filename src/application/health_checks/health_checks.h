#ifndef HEALTH_CHECKS_H
#define HEALTH_CHECKS_H

#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"

w_status_t health_check_init(void);
void health_check_task(void *argument);
void watchdog_kick(void);
void watchdog_register_task(TaskHandle_t task_handle, uint32_t timeout_ticks);

#endif
