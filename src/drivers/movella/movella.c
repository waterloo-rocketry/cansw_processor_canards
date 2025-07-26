#include <stdbool.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "stm32h7xx_hal.h"
#include "task.h"

#include "third_party/xsens-mti/src/xsens_mti.h"

#include "application/logger/log.h"
#include "common/math/math.h"
#include "drivers/movella/movella.h"
#include "drivers/uart/uart.h"

#define MOVELLA_TX_TIMEOUT_MS 100
// should be within 5 ms, so assume err after 10 ms
#define MOVELLA_RX_TIMEOUT_MS 10
#define XSENS_ARR_ELEM 7
// movella docs: max length for non-extended data msg is 256 bytes
#define MOVELLA_RX_MAX_BYTES 256

typedef struct {
    UART_HandleTypeDef *movella_huart;
    xsens_interface_t xsens_interface;
    SemaphoreHandle_t data_mutex;
    TaskHandle_t task_handle;
    movella_data_t latest_data;
    bool initialized;
    bool configured;
} movella_state_t;

// a received raw packet from movella
typedef struct {
    uint8_t *buffer;
    uint32_t bytes_received;
} rx_packet_t;

// use ping pong buffering to continue receiving data while parsing
static uint8_t rx_buffer_ping[MOVELLA_RX_MAX_BYTES] = {0};
static uint8_t rx_buffer_pong[MOVELLA_RX_MAX_BYTES] = {0};

// the buffer currently being written to from movella
static volatile uint8_t *active_rx_buffer = rx_buffer_ping;
// the buffer with complete data ready to be parsed
static volatile uint8_t *ready_rx_buffer = NULL;

// true if ready_rx_buffer is currently being parsed and should not be touched
static volatile bool is_ready_buffer_busy = false;
// semaphore to signal when a new ready_rx_buffer is ready for parsing
static SemaphoreHandle_t ready_buffer_semaphore = NULL;

// internal queue for received rx packets. contains rx_packet_t
static QueueHandle_t ready_packet_queue = NULL;

// state of this module
static movella_state_t s_movella = {0};

/**
 * This must be called in the UART RX ISR when movella uart receives a full msg.
 *
 * this signals to the movella task that new data is ready for parsing.
 * the new data is in `ready_rx_buffer`.
 * this restarts the UART reception in `active_rx_buffer`.
 *
 * If this cannot rotate buffers because the next buffer is busy, it will drop the data
 * and restart UART reception anyway.
 */
void movella_uart_rx_cb(uint32_t len) {
    if (s_movella.initialized) {
        if (is_ready_buffer_busy) {
            // can't rotate buffers (the ready buffer is currently being parsed)
        } else {
            // rotate buffers from ping to pong or pong to ping
            if (active_rx_buffer == rx_buffer_ping) {
                active_rx_buffer = rx_buffer_pong;
                ready_rx_buffer = rx_buffer_ping;
            } else {
                active_rx_buffer = rx_buffer_ping;
                ready_rx_buffer = rx_buffer_pong;
            }
        }

        // restart uart reception into active buffer
        HAL_UARTEx_ReceiveToIdle_DMA(
            s_movella.movella_huart, (uint8_t *)active_rx_buffer, MOVELLA_RX_MAX_BYTES
        );

        // ready buffer is now busy since it is going to wait in the queue
        is_ready_buffer_busy = true;

        // send new buffer to task via queue
        rx_packet_t ready_rx_packet = {.buffer = (uint8_t *)ready_rx_buffer, .bytes_received = len};
        xQueueOverwriteFromISR(ready_packet_queue, &ready_rx_packet, NULL);
    }
}

static void movella_event_callback(XsensEventFlag_t event, XsensEventData_t *mtdata) {
    if (xSemaphoreTake(s_movella.data_mutex, 0) == pdTRUE) {
        switch (event) {
            case XSENS_EVT_ACCELERATION:
                if (mtdata->type == XSENS_EVT_TYPE_FLOAT3) {
                    s_movella.latest_data.acc.x = mtdata->data.f4x3[0];
                    s_movella.latest_data.acc.y = mtdata->data.f4x3[1];
                    s_movella.latest_data.acc.z = mtdata->data.f4x3[2];
                }
                break;

            case XSENS_EVT_RATE_OF_TURN:
                if (mtdata->type == XSENS_EVT_TYPE_FLOAT3) {
                    s_movella.latest_data.gyr.x = mtdata->data.f4x3[0];
                    s_movella.latest_data.gyr.y = mtdata->data.f4x3[1];
                    s_movella.latest_data.gyr.z = mtdata->data.f4x3[2];
                }
                break;

            case XSENS_EVT_MAGNETIC:
                if (mtdata->type == XSENS_EVT_TYPE_FLOAT3) {
                    s_movella.latest_data.mag.x = mtdata->data.f4x3[0];
                    s_movella.latest_data.mag.y = mtdata->data.f4x3[1];
                    s_movella.latest_data.mag.z = mtdata->data.f4x3[2];
                }
                break;

            case XSENS_EVT_QUATERNION:
                if (mtdata->type == XSENS_EVT_TYPE_FLOAT4) {
                    float euler_rad[3] = {0};
                    // xsens_quaternion_to_euler(mtdata->data.f4x4, euler_rad);
                    // TODO: add quaternion function once implemented

                    s_movella.latest_data.euler.x = euler_rad[0] * DEG_PER_RAD;
                    s_movella.latest_data.euler.y = euler_rad[1] * DEG_PER_RAD;
                    s_movella.latest_data.euler.z = euler_rad[2] * DEG_PER_RAD;
                }
                break;

            case XSENS_EVT_PRESSURE:
                if (mtdata->type == XSENS_EVT_TYPE_U32) {
                    s_movella.latest_data.pres = (float)mtdata->data.u4;
                }
                break;

            case XSENS_EVT_TEMPERATURE:
                if (mtdata->type == XSENS_EVT_TYPE_FLOAT) {
                    s_movella.latest_data.temp = mtdata->data.f4;
                }
                break;
            default:
                break;
        }

        xSemaphoreGive(s_movella.data_mutex);
    }
}

static void movella_uart_send(uint8_t *data, uint16_t length) {
    (void)uart_write(UART_MOVELLA, data, length, MOVELLA_TX_TIMEOUT_MS);
}

static void movella_configure(void) {
    XsensFrequencyConfig_t settings[XSENS_ARR_ELEM] = {
        {.id = XDI_QUATERNION, .frequency = 200}, // 5ms
        {.id = XDI_ACCELERATION, .frequency = 200}, // 5ms
        {.id = XDI_RATE_OF_TURN, .frequency = 200}, // 5ms
        {.id = XDI_MAGNETIC_FIELD, .frequency = 100}, // 10ms
        {.id = XDI_TEMPERATURE, .frequency = 5}, // 200ms
        {.id = XDI_BARO_PRESSURE, .frequency = 40}, // 25ms
        {.id = XDI_STATUS_WORD, .frequency = 0xFFFF}
    };

    xsens_mti_request(&s_movella.xsens_interface, MT_GOTOCONFIG);
    vTaskDelay(pdMS_TO_TICKS(100));

    xsens_mti_set_configuration(&s_movella.xsens_interface, settings, XSENS_ARR_ELEM);
    vTaskDelay(pdMS_TO_TICKS(100));

    xsens_mti_request(&s_movella.xsens_interface, MT_GOTOMEASUREMENT);
    vTaskDelay(pdMS_TO_TICKS(100));

    s_movella.configured = true;
}

w_status_t movella_init(UART_HandleTypeDef *huart) {
    if (s_movella.initialized) {
        return W_SUCCESS;
    }

    s_movella.data_mutex = xSemaphoreCreateMutex();
    ready_buffer_semaphore = xSemaphoreCreateBinary();
    ready_packet_queue = xQueueCreate(1, sizeof(rx_packet_t));

    if ((s_movella.data_mutex == NULL) || (ready_buffer_semaphore == NULL) ||
        (ready_packet_queue == NULL)) {
        return W_FAILURE;
    }

    s_movella.xsens_interface.event_cb = movella_event_callback;
    s_movella.xsens_interface.output_cb = movella_uart_send;

    s_movella.movella_huart = huart;

    s_movella.initialized = true;
    return W_SUCCESS;
}

w_status_t movella_get_data(movella_data_t *out_data, uint32_t timeout_ms) {
    if (NULL == out_data) {
        return W_INVALID_PARAM;
    }

    if (!s_movella.initialized || !s_movella.configured) {
        return W_FAILURE;
    }

    if (pdTRUE == xSemaphoreTake(s_movella.data_mutex, pdMS_TO_TICKS(timeout_ms))) {
        *out_data = s_movella.latest_data;
        xSemaphoreGive(s_movella.data_mutex);
        return W_SUCCESS;
    }

    return W_FAILURE;
}

void movella_task(void *parameters) {
    (void)parameters;

    while (1) {
        rx_packet_t ready_rx_packet = {0};

        // wait for a new ready buffer to be available
        if (xQueueReceive(ready_packet_queue, &ready_rx_packet, MOVELLA_RX_TIMEOUT_MS) == pdTRUE) {
            is_ready_buffer_busy = true;

            if (ready_rx_packet.bytes_received > 0) {
                xsens_mti_parse_buffer(
                    &s_movella.xsens_interface,
                    ready_rx_packet.buffer,
                    ready_rx_packet.bytes_received
                );
            }

            is_ready_buffer_busy = false;
        } else {
            // timeout, no new data received
            log_text(10, "movella", "no rx");
        }
    }
}
