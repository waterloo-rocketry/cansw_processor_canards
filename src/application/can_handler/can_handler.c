#include "application/can_handler/can_handler.h"
#include "FreeRTOS.h"
#include "application/logger/log.h"
#include "drivers/gpio/gpio.h"
#include "queue.h"

#define BUS_QUEUE_LENGTH 16

static QueueHandle_t bus_queue_rx = NULL;
static QueueHandle_t bus_queue_tx = NULL;
static uint32_t dropped_rx_counter = 0;

static can_callback_t callback_map[30] = {NULL};

static w_status_t can_reset_callback(const can_msg_t *msg) {
    if (check_board_need_reset(msg)) {
        NVIC_SystemReset();
        return W_FAILURE; // Should never reach here
    }
    return W_SUCCESS;
}

static w_status_t can_led_on_callback(const can_msg_t *msg) {
    (void)msg;
    w_status_t status = W_SUCCESS;
    status |= gpio_write(GPIO_PIN_RED_LED, GPIO_LEVEL_LOW, 5);
    status |= gpio_write(GPIO_PIN_GREEN_LED, GPIO_LEVEL_LOW, 5);
    status |= gpio_write(GPIO_PIN_BLUE_LED, GPIO_LEVEL_LOW, 5);

    if (status != W_SUCCESS) {
        log_text(1, "CANCallback", "ERROR: LED ON callback failed to set GPIO.");
    }
    return status;
}

static w_status_t can_led_off_callback(const can_msg_t *msg) {
    (void)msg;
    w_status_t status = W_SUCCESS;
    status |= gpio_write(GPIO_PIN_RED_LED, GPIO_LEVEL_HIGH, 5);
    status |= gpio_write(GPIO_PIN_GREEN_LED, GPIO_LEVEL_HIGH, 5);
    status |= gpio_write(GPIO_PIN_BLUE_LED, GPIO_LEVEL_HIGH, 5);

    if (status != W_SUCCESS) {
        log_text(1, "CANCallback", "ERROR: LED OFF callback failed to set GPIO.");
    }
    return status;
}

static void can_handle_rx_isr(const can_msg_t *message, uint32_t timestamp) {
    (void)timestamp;

    BaseType_t higher_priority_task_woken = pdFALSE;
    if (pdPASS != xQueueSendFromISR(bus_queue_rx, message, &higher_priority_task_woken)) {
        dropped_rx_counter++; // We can't return an error code or log from isr handler, so this is
                              // the best I could come up with
    }
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

w_status_t can_handler_init(FDCAN_HandleTypeDef *hfdcan) {
    if (NULL == hfdcan) {
        return W_INVALID_PARAM;
    }

    bus_queue_rx = xQueueCreate(BUS_QUEUE_LENGTH, sizeof(can_msg_t));
    bus_queue_tx = xQueueCreate(BUS_QUEUE_LENGTH, sizeof(can_msg_t));

    if ((NULL == bus_queue_tx) || (NULL == bus_queue_rx)) {
        return W_FAILURE;
    }

    if (!can_init_stm(hfdcan, can_handle_rx_isr)) {
        log_text(1, "CANHandler", "ERROR: can_init_stm failed.");
        return W_FAILURE;
    }

    if ((W_SUCCESS != can_handler_register_callback(MSG_RESET_CMD, can_reset_callback)) ||
        (W_SUCCESS != can_handler_register_callback(MSG_LEDS_ON, can_led_on_callback)) ||
        (W_SUCCESS != can_handler_register_callback(MSG_LEDS_OFF, can_led_off_callback))) {
        log_text(1, "CANHandler", "ERROR: Failed to register mandatory CAN callbacks.");
        return W_FAILURE;
    }

    return W_SUCCESS;
}

w_status_t can_handler_register_callback(can_msg_type_t msg_type, can_callback_t callback) {
    callback_map[msg_type] = callback;
    return W_SUCCESS;
}

w_status_t can_handler_transmit(const can_msg_t *message) {
    if (pdPASS == xQueueSend(bus_queue_tx, message, 0)) {
        return W_SUCCESS;
    }
    log_text(1, "CANHandler", "ERROR: Failed to queue message for TX. Queue full?");
    return W_FAILURE;
}

void can_handler_task_rx(void *argument) {
    (void)argument;
    for (;;) {
        can_msg_t rx_msg;
        if (pdPASS == xQueueReceive(bus_queue_rx, &rx_msg, 100)) {
            can_msg_type_t msg_type = get_message_type(&rx_msg);
            if (NULL != callback_map[msg_type]) {
                if (callback_map[msg_type](&rx_msg) != W_SUCCESS) {
                    log_text(1, "CANHandlerRX", "WARN: Callback failed for msg type %d.", msg_type);
                }
            } else {
                log_text(
                    1, "CANHandlerRX", "WARN: No callback registered for msg type %d.", msg_type
                );
            }
        } else {
            log_text(1, "CANHandlerRX", "WARN: Timed out waiting for RX message.");
        }
    }
}

void can_handler_task_tx(void *argument) {
    (void)argument;
    for (;;) {
        can_msg_t tx_msg;
        if (pdPASS == xQueueReceive(bus_queue_tx, &tx_msg, 100)) {
            can_send(&tx_msg);
            vTaskDelay(1); // hardware limitation - cannot enqueue more than 2 messages back to back
        } else {
            log_text(1, "CANHandlerTX", "WARN: Timed out waiting for TX message.");
        }
    }
}
