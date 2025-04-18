#include "application/can_handler/can_handler.h"
#include "FreeRTOS.h"
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
    return status;
}

static w_status_t can_led_off_callback(const can_msg_t *msg) {
    (void)msg;
    w_status_t status = W_SUCCESS;
    status |= gpio_write(GPIO_PIN_RED_LED, GPIO_LEVEL_HIGH, 5);
    status |= gpio_write(GPIO_PIN_GREEN_LED, GPIO_LEVEL_HIGH, 5);
    status |= gpio_write(GPIO_PIN_BLUE_LED, GPIO_LEVEL_HIGH, 5);
    return status;
}

static void can_handle_rx_isr(const can_msg_t *message, uint32_t timestamp) {
    (void)timestamp;
    BaseType_t higher_priority_task_woken = pdFALSE;

    can_msg_type_t msg_type = (can_msg_type_t)get_message_type(message);

    // filter only messages we care about for efficiency
    switch (msg_type) {
        // do extra filtering for sensor_analog because theres a shit ton of them
        case MSG_SENSOR_ANALOG:
            can_analog_sensor_id_t sensor_id;
            uint16_t data;
            if ((get_analog_data(message, &sensor_id, &data) == true) &&
                (SENSOR_CANARD_ENCODER_1 == sensor_id)) {
                if (pdPASS !=
                    xQueueSendFromISR(bus_queue_rx, message, &higher_priority_task_woken)) {
                    dropped_rx_counter++; // We can't return err or log from isr so use tracker var
                }
            }
            break;
        case MSG_RESET_CMD:
        case MSG_ACTUATOR_CMD:
        case MSG_LEDS_ON:
        case MSG_LEDS_OFF:
            if (pdPASS != xQueueSendFromISR(bus_queue_rx, message, &higher_priority_task_woken)) {
                dropped_rx_counter++; // We can't return err or log from isr so use tracker var
            }
            break;

        default:
            break;
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
        return W_FAILURE;
    }

    if ((W_SUCCESS != can_handler_register_callback(MSG_RESET_CMD, can_reset_callback)) ||
        (W_SUCCESS != can_handler_register_callback(MSG_LEDS_ON, can_led_on_callback)) ||
        (W_SUCCESS != can_handler_register_callback(MSG_LEDS_OFF, can_led_off_callback))) {
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
    return W_FAILURE;
}

void can_handler_task_rx(void *argument) {
    (void)argument;
    for (;;) {
        can_msg_t rx_msg;
        if (pdPASS == xQueueReceive(bus_queue_rx, &rx_msg, 100)) {
            can_msg_type_t msg_type = get_message_type(&rx_msg);
            if (NULL != callback_map[msg_type]) {
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
        if (pdPASS == xQueueReceive(bus_queue_tx, &tx_msg, 100)) {
            if (!can_send(&tx_msg)) {
                // logError("CAN", "CAN send failed!");
            }
            vTaskDelay(1); // hardware limitation - cannot enqueue more than 2 messages back to back
        } else {
            // TODO log timeout w no messages?
        }
    }
}
