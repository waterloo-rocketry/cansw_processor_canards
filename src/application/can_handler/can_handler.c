#include "application/can_handler/can_handler.h"
#include "FreeRTOS.h"
#include "queue.h"

static QueueHandle_t bus_queue_rx = NULL;
static QueueHandle_t bus_queue_tx = NULL;

static can_callback_t callback_map[30] = {NULL};

static w_status_t can_reset_callback(const can_msg_t *msg) {
    can_board_type_id_t target = get_board_type_unique_id(msg);
    if (BOARD_TYPE_ID_ANY == target || BOARD_TYPE_UNIQUE_ID == target) {
        NVIC_SystemReset();
    }
    return W_SUCCESS;
}

static void can_handle_rx_isr(const can_msg_t *message, uint32_t timestamp) {
    // The timestamp parameter passed to the handler is some internal FDCAN thing that I don't know
    // how to convert to a sensible value Just use millis_() for now
    xQueueSend(bus_queue_rx, &message, 10);
    return;
}

w_status_t can_handler_init(FDCAN_HandleTypeDef *hfdcan) {
    if (hfdcan == NULL) {
        return W_INVALID_PARAM;
    }

    bus_queue_rx = xQueueCreate(16, sizeof(can_msg_t));
    bus_queue_tx = xQueueCreate(16, sizeof(can_msg_t));

    if (bus_queue_tx == NULL) {
        return W_FAILURE;
    }
    if (bus_queue_rx == NULL) {
        return W_FAILURE;
    }
    if (!can_init_stm(hfdcan, can_handle_rx_isr)) {
        return W_FAILURE;
    }

    if (can_register_callback(MSG_RESET_CMD, can_reset_callback) != W_SUCCESS) {
        return W_FAILURE;
    }

    return W_SUCCESS;
}

w_status_t can_register_callback(can_msg_type_t msg_type, can_callback_t callback) {
    callback_map[msg_type] = callback;
    return W_SUCCESS;
}

w_status_t can_handle_tx(const can_msg_t *message) {
    if (xQueueSend(bus_queue_tx, message, 10) == pdPASS) {
        return W_SUCCESS;
    }
    return W_FAILURE;
}

void can_handler_task_rx(void *argument) {
    for (;;) {
        can_msg_t rx_msg;
        if (xQueueReceive(bus_queue_rx, &rx_msg, 100) == pdTRUE) {
            can_msg_type_t msg_type = get_message_type(&rx_msg);
            if (callback_map[msg_type] != NULL) {
                callback_map[msg_type](&rx_msg);
            }
        } else {
            // TODO log timeout w no messages?
        }
    }
}

void can_handler_task_tx(void *argument) {
    for (;;) {
        can_msg_t tx_msg;
        // Block the thread until we see data in the bus queue or 1 sec elapses
        if (xQueueReceive(bus_queue_tx, &tx_msg, 100) == pdTRUE) {
            if (!can_send(&tx_msg)) {
                // logError("CAN", "CAN send failed!");
            }
            vTaskDelay(1); // hardware limitation - cannot enqueue more than 2 messages back to back
        } else {
            // TODO log timeout w no messages?
        }
    }
}

// w_status_t test_callback(const can_msg_t *msg) {
//     // logInfo("CAN", "Received message with SID: %x", msg->sid);
//     return W_SUCCESS;
// }

// void test_thread(void *argument) {
//     can_register_callback(0x1, test_callback);
//     for (;;) {
//         can_msg_t msg;
//         msg.sid = 0x2;
//         msg.data[0] = 0x01;
//         msg.data[1] = 0x02;
//         msg.data[2] = 0x03;
//         msg.data[3] = 0x04;
//         msg.data[4] = 0x05;
//         msg.data[5] = 0x06;
//         msg.data[6] = 0x07;
//         msg.data[7] = 0x08;
//         msg.data_len = 8;
//         can_handle_tx(&msg);
//         vTaskDelay(1000);
//     }
// }
