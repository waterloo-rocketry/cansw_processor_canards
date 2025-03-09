#include "application/can_handler/can_handler.h"
#include "FreeRTOS.h"
#include "drivers/gpio/gpio.h"
#include "queue.h"

static QueueHandle_t bus_queue_rx = NULL;
static QueueHandle_t bus_queue_tx = NULL;
static uint32_t dropped_rx_counter = 0;

static can_callback_t callback_map[30] = {NULL};

static w_status_t can_reset_callback(const can_msg_t *msg) {
    if (check_board_need_reset(msg)) {
        NVIC_SystemReset();
    }
    return W_SUCCESS;
}

static w_status_t can_led_on_callback(const can_msg_t *msg) {
    (void)msg;
    gpio_write(GPIO_PIN_RED_LED, 0, 5);
    gpio_write(GPIO_PIN_GREEN_LED, 0, 5);
    gpio_write(GPIO_PIN_BLUE_LED, 0, 5);
    return W_SUCCESS;
}

static w_status_t can_led_off_callback(const can_msg_t *msg) {
    (void)msg;
    gpio_write(GPIO_PIN_RED_LED, 1, 5);
    gpio_write(GPIO_PIN_GREEN_LED, 1, 5);
    gpio_write(GPIO_PIN_BLUE_LED, 1, 5);
    return W_SUCCESS;
}

static void can_handle_rx_isr(const can_msg_t *message, uint32_t timestamp) {
    // The timestamp parameter passed to the handler is some internal FDCAN thing that I don't know
    // how to convert to a sensible value Just use millis_() for now
    (void)timestamp;

    BaseType_t higher_priority_task_woken = pdFALSE;
    if (pdPASS != xQueueSendFromISR(bus_queue_rx, message, &higher_priority_task_woken)) {
        dropped_rx_counter++; // We can't return an error code or log from isr handler, so this is
                              // the best I could come up with
    }
    portYIELD_FROM_ISR(higher_priority_task_woken);
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

    if (can_handler_register_callback(MSG_RESET_CMD, can_reset_callback) != W_SUCCESS) {
        return W_FAILURE;
    }
    if (can_handler_register_callback(MSG_LEDS_ON, can_led_on_callback) != W_SUCCESS) {
        return W_FAILURE;
    }
    if (can_handler_register_callback(MSG_LEDS_OFF, can_led_off_callback) != W_SUCCESS) {
        return W_FAILURE;
    }

    return W_SUCCESS;
}

w_status_t can_handler_register_callback(can_msg_type_t msg_type, can_callback_t callback) {
    callback_map[msg_type] = callback;
    return W_SUCCESS;
}

w_status_t can_handler_transmit(const can_msg_t *message) {
    if (xQueueSend(bus_queue_tx, message, 10) == pdPASS) {
        return W_SUCCESS;
    }
    return W_FAILURE;
}

void can_handler_task_rx(void *argument) {
    (void)argument;
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
    (void)argument;
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
