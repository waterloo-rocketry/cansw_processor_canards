#ifndef _CAN_HANDLER_H
#define _CAN_HANDLER_H

#include "rocketlib/include/common.h"
#include "canlib.h"
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"

extern FDCAN_HandleTypeDef hfdcan1;

//Used to store the callbacks for each message type
typedef w_status_t (*can_callback_t)(const can_msg_t*);


//      Called By Tasks

/**
 * @brief Used to write a message to the can queue.
 * @param message The message to write to the queue
 * @return Status of the operation.
*/
w_status_t can_handle_tx(const can_msg_t *message);

/**
 * @brief Used by other tasks to register a rx message handler, if there is already a handler for the message type, it will be overwritten 
 * @param msg_type The message type to register the callback for
 * @param callback The callback function to call when the message is recieved
 * @return Status of the operation.
*/
w_status_t can_register_callback(can_msg_type_t msg_type, can_callback_t callback);


//      Used by main to init

/**
 * @brief This function recieves the messages, it is setup as in interrupt in main
 * @param message The message recieved
 * @param timestamp The time the message was recieved
 * @return Status of the operation.
*/
w_status_t can_handle_rx(const can_msg_t *message, uint32_t timestamp);

/**
 * @brief When busqueue_rx recieves a message, this task calls the corresponding callback
 * @return Status of the operation.
*/
void can_handler_task_rx(void *argument);

/**
 * @brief When busqueue_tx recieves a message, this task sends it to the can bus
 * @return Status of the operation.
*/
void can_handler_task_tx(void *argument);

/**
 * @brief Initializer to setup queues
 * @return Status of the operation.
*/
w_status_t can_handler_init(void); 

#endif
