#ifndef __CAN_HANDLER
#define __CAN_HANDLER

#include "canlib.h"
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern xQueueHandle busQueue_tx;
extern xQueueHandle busQueue_rx;

void can_handle_rx(consst can_msg_t *message, uint32_t timestamp);
void can_handle_tx(consst can_msg_t *message);

void canHandlerTask_rx(void *argument);
void canHandlerTask_tx(void *argument);
bool canHandlerInit(void); 

#endif
