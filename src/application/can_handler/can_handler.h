#ifndef _CAN_HANDLER_H
#define _CAN_HANDLER_H

#include "common.h"
#include "canlib.h"
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern xQueueHandle busQueue_tx;
extern xQueueHandle busQueue_rx;

//Called by other tasks to send to can
w_status_t can_handle_tx(const can_msg_t *message);

//Used by main to init
w_status_t can_handle_rx(const can_msg_t *message, uint32_t timestamp);
void canHandlerTask_rx(void *argument);
void canHandlerTask_tx(void *argument);
w_status_t canHandlerInit(void);

#endif
