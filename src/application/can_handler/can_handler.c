#include "can_handler.h"

QueueHandle_t busQueue_rx;
QueueHandle_t busQueue_tx;

can_callback_t callback_map[30];

void can_handle_rx_isr(const can_msg_t *message, uint32_t timestamp)
{
    // The timestamp parameter passed to the handler is some internal FDCAN thing that I don't know how to convert to a sensible value
    // Just use millis_() for now
    xQueueSend(busQueue_rx, &message, 10);
    return;
}

w_status_t can_register_callback(can_msg_type_t msg_type, can_callback_t callback)
{
    callback_map[msg_type] = callback;
    return W_SUCCESS;
}

void can_handler_tx(const can_msg_t *message)
{
    if (xQueueSend(busQueue_tx, message, 10))
    {
        return W_SUCCESS;
    }
    else
    {
        return W_FAILURE;
    }
}

void can_handler_task_rx(void *argument)
{
    for (;;)
    {
        can_msg_t *rx_msg;
        if (xQueueReceive(busQueue_rx, &rx_msg, 10) == pdTRUE)
        {
            uint16_t msgType = get_message_type(rx_msg);
            if (callback_map[msgType] != NULL)
            {
                callback_map[msgType](rx_msg);
            }
        }
    }
}

void can_handler_task_tx(void *argument)
{
    for (;;)
    {
        can_msg_t tx_msg;
        // Block the thread until we see data in the bus queue or 1 sec elapses
        if (xQueueReceive(busQueue_tx, &tx_msg, 1000) == pdTRUE)
        { // Returns pdTRUE if we got a message, pdFALSE if timed out
            if (can_send(&tx_msg) == false)
            {
                logError("CAN", "CAN send failed!");
            }
            vTaskDelay(1);
            HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10); // write and toggle D3 when we send a CAN message
        }
    }
}

w_status_t can_handler_init(void)
{
    busQueue_rx = xQueueCreate(16, sizeof(can_msg_t));
    busQueue_tx = xQueueCreate(16, sizeof(can_msg_t));

    if (busQueue_tx == NULL)
        return W_FAILURE;
    if (busQueue_rx == NULL)
        return W_FAILURE;
    if (!can_init_stm(&hfdcan1, can_handle_rx_isr))
        return W_FAILURE;
    return W_SUCCESS;
}

void test_thread()
{
    can_register_callback(0x1, test_callback);
    for (;;)
    {
        can_msg_t msg;
        msg.sid = 0x2;
        msg.data[0] = 0x01;
        msg.data[1] = 0x02;
        msg.data[2] = 0x03;
        msg.data[3] = 0x04;
        msg.data[4] = 0x05;
        msg.data[5] = 0x06;
        msg.data[6] = 0x07;
        msg.data[7] = 0x08;
        msg.data_len = 8;
        can_handler_tx(&msg);
        vTaskDelay(1000);
    }
}

w_status_t test_callback(const can_msg_t *msg)
{
    logInfo("CAN", "Received message with SID: %x", msg->sid);
    return W_SUCCESS;
}